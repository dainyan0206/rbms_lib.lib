#include "rbms.h"
#include <cmath>
#include <cstdint>

namespace {
constexpr uint32_t kDataEvent = 0x01U;
constexpr float kPositionSettlingThresholdDeg = 25.0f;
constexpr float kPositionDeadBandDeg = 2.5f;
}

rbms::rbms(CAN &can, bool motor_type, int motor_num, int id_offset)
    : _can(can),
      _motor_type(motor_type),
      _motor_num(motor_num),
      _id_offset(id_offset),
      _motor_max(motor_type ? 16384 : 10000),
      _kp(motor_type ? 35.0f : 15.0f),
      _ki(motor_type ? 50.0f : 12.0f),
      _kd(0.0f),
      _kp_p(motor_type ? 5.0f : 4.5f),
      _ki_p(0.0f),
      _kd_p(motor_type ? 0.15f : 0.25f),
      _new_data_mask(0U),
      _thread_started(false) {

    // 不正なID範囲を指定された場合は、モーターを有効化しない。
    if (_motor_num < 1 || _motor_num > MAX_MOTORS ||
        _id_offset < 0 || _id_offset >= MAX_MOTORS ||
        _id_offset + _motor_num > MAX_MOTORS) {
        _motor_num = 0;
    }

    for (int i = 0; i < MAX_MOTORS; ++i) {
        _control_modes[i] = SPD_MODE;
        _target_speeds[i] = 0;
        _target_torques[i] = 0;
        _target_angles[i] = 0.0f;
        _output_torques[i] = 0;
        _torque_limits[i] = _motor_max;
        _gear_ratios[i] = _motor_type ? 19.2f : 36.0f;

        _pid_states[i].prev_err = 0.0f;
        _pid_states[i].integral = 0.0f;
        _pid_states[i].pos_prev_err = 0.0f;
        _pid_states[i].pos_integral = 0.0f;
        _pid_states[i].last_raw_angle = 0U;
        _pid_states[i].accumulated_angle = 0.0f;
        _pid_states[i].is_initialized = false;
        _pid_states[i].speed_limit_rpm = 0.0f;
        _pid_states[i].accel_limit_rpm_s = 0.0f;
        _pid_states[i].current_target_rpm = 0.0f;
        _pid_states[i].timer.start();
    }
}

bool rbms::valid_local_id(int id) const {
    return id >= 0 && id < _motor_num && id < MAX_MOTORS;
}

void rbms::set_control_mode(int id, ControlMode mode) {
    if (!valid_local_id(id)) return;
    if (mode != SPD_MODE && mode != TRQ_MODE && mode != POS_MODE) return;

    std::lock_guard<std::mutex> lock(_data_mutex);
    _control_modes[id] = mode;
    _pid_states[id].integral = 0.0f;
    _pid_states[id].prev_err = 0.0f;
    _pid_states[id].pos_integral = 0.0f;
    _pid_states[id].pos_prev_err = 0.0f;
    _pid_states[id].current_target_rpm = 0.0f;
}

void rbms::set_target_speed(int id, int speed_rpm) {
    if (!valid_local_id(id)) return;
    std::lock_guard<std::mutex> lock(_data_mutex);
    _target_speeds[id] = speed_rpm;
}

void rbms::set_target_torque(int id, int current_command) {
    if (!valid_local_id(id)) return;
    std::lock_guard<std::mutex> lock(_data_mutex);
    _target_torques[id] = current_command;
}

void rbms::set_gear_ratio(int id, float gear_ratio) {
    if (!valid_local_id(id) || !std::isfinite(gear_ratio) || gear_ratio <= 0.0f) return;
    std::lock_guard<std::mutex> lock(_data_mutex);
    _gear_ratios[id] = gear_ratio;
}

void rbms::set_target_angle(int id, float angle_deg) {
    if (!valid_local_id(id) || !std::isfinite(angle_deg)) return;
    std::lock_guard<std::mutex> lock(_data_mutex);
    _target_angles[id] = angle_deg;
}

void rbms::reset_angle(int id) {
    if (!valid_local_id(id)) return;
    std::lock_guard<std::mutex> lock(_data_mutex);
    _pid_states[id].accumulated_angle = 0.0f;
    _pid_states[id].last_raw_angle = 0U;
    _pid_states[id].is_initialized = false;
    _pid_states[id].pos_integral = 0.0f;
    _pid_states[id].pos_prev_err = 0.0f;
    _target_angles[id] = 0.0f;
}

void rbms::set_pid_gains(float kp, float ki, float kd) {
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd)) return;
    std::lock_guard<std::mutex> lock(_data_mutex);
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void rbms::set_pos_pid_gains(float kp, float ki, float kd) {
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd)) return;
    std::lock_guard<std::mutex> lock(_data_mutex);
    _kp_p = kp;
    _ki_p = ki;
    _kd_p = kd;
}

void rbms::set_speed_limit(int id, float max_speed_rpm) {
    if (!valid_local_id(id) || !std::isfinite(max_speed_rpm)) return;
    std::lock_guard<std::mutex> lock(_data_mutex);
    _pid_states[id].speed_limit_rpm = std::fabs(max_speed_rpm);
}

void rbms::set_accel_limit(int id, float max_accel_rpm_s) {
    if (!valid_local_id(id) || !std::isfinite(max_accel_rpm_s)) return;
    std::lock_guard<std::mutex> lock(_data_mutex);
    _pid_states[id].accel_limit_rpm_s = std::fabs(max_accel_rpm_s);
}

void rbms::set_torque_limit(int id, int max_command) {
    if (!valid_local_id(id)) return;
    if (max_command < 0) max_command = -max_command;
    if (max_command > _motor_max) max_command = _motor_max;
    std::lock_guard<std::mutex> lock(_data_mutex);
    _torque_limits[id] = max_command;
}

float rbms::pid_calculate(int id, float target, float current, float dt) {
    PIDState &p = _pid_states[id];
    if (!std::isfinite(dt) || dt <= 0.0f) {
        p.integral = 0.0f;
        p.prev_err = 0.0f;
        return 0.0f;
    }

    const float error = target - current;
    p.integral += (error + p.prev_err) * dt * 0.5f;
    const float ki_abs = (_ki > 0.0f) ? _ki : 1.0f;
    const float integral_limit = static_cast<float>(_motor_max) / ki_abs;
    if (p.integral > integral_limit) p.integral = integral_limit;
    if (p.integral < -integral_limit) p.integral = -integral_limit;

    const float derivative = (error - p.prev_err) / dt;
    const float output = _kp * error + _ki * p.integral + _kd * derivative;
    p.prev_err = error;
    return output;
}

float rbms::pos_pid_calculate(int id, float target, float current, float dt, float limit) {
    PIDState &p = _pid_states[id];
    if (!std::isfinite(dt) || dt <= 0.0f) {
        p.pos_integral = 0.0f;
        p.pos_prev_err = 0.0f;
        return 0.0f;
    }

    const float error = target - current;
    p.pos_integral += (error + p.pos_prev_err) * dt * 0.5f;
    const float ki_abs = (_ki_p > 0.0f) ? _ki_p : 1.0f;
    const float integral_limit = limit / ki_abs;
    if (p.pos_integral > integral_limit) p.pos_integral = integral_limit;
    if (p.pos_integral < -integral_limit) p.pos_integral = -integral_limit;

    const float derivative = (error - p.pos_prev_err) / dt;
    const float output_speed = _kp_p * error + _ki_p * p.pos_integral + _kd_p * derivative;
    p.pos_prev_err = error;
    return output_speed;
}

void rbms::spd_control() {
    if (_thread_started || _motor_num <= 0) return;
    const auto status = _thread.start(callback(this, &rbms::control_thread_entry));
    if (status == osOK) _thread_started = true;
}

void rbms::control_thread_entry() {
    while (true) {
        _event_flags.wait_any(kDataEvent);

        for (int id = 0; id < _motor_num; ++id) {
            CANMessage local_msg;
            bool has_new = false;

            {
                std::lock_guard<std::mutex> lock(_data_mutex);
                const uint32_t bit = (1UL << id);
                if ((_new_data_mask & bit) != 0U) {
                    local_msg = _msg_buffer[id];
                    _new_data_mask &= ~bit;
                    has_new = true;
                }
            }
            if (!has_new) continue;

            float dt = _pid_states[id].timer.read();
            _pid_states[id].timer.reset();
            if (!std::isfinite(dt) || dt <= 0.0f || dt > 0.05f) {
                dt = 0.001f;
                std::lock_guard<std::mutex> lock(_data_mutex);
                _pid_states[id].integral = 0.0f;
                _pid_states[id].prev_err = 0.0f;
                _pid_states[id].pos_integral = 0.0f;
                _pid_states[id].pos_prev_err = 0.0f;
                _pid_states[id].current_target_rpm = 0.0f;
            }

            short rotation = 0;
            short raw_speed = 0;
            parse_can_data(id, local_msg, &rotation, &raw_speed);

            std::lock_guard<std::mutex> lock(_data_mutex);
            const float ratio = _gear_ratios[id];
            if (!std::isfinite(ratio) || ratio <= 0.0f) {
                _output_torques[id] = 0;
                continue;
            }
            const float current_rpm = static_cast<float>(raw_speed) / ratio;
            const ControlMode mode = _control_modes[id];
            int final_output = 0;

            if (mode == TRQ_MODE) {
                final_output = _target_torques[id];
                _pid_states[id].current_target_rpm = current_rpm;
            } else {
                const float limit = (_pid_states[id].speed_limit_rpm > 0.0f)
                    ? _pid_states[id].speed_limit_rpm
                    : ((mode == POS_MODE) ? 500.0f : 10000.0f);
                float raw_target_rpm = 0.0f;
                bool bypass_accel_limit = false;

                if (mode == POS_MODE) {
                    const float current_angle = _pid_states[id].accumulated_angle;
                    const float error_angle = _target_angles[id] - current_angle;
                    const float abs_error = std::fabs(error_angle);

                    if (abs_error > kPositionSettlingThresholdDeg &&
                        _pid_states[id].accel_limit_rpm_s > 0.0f) {
                        _pid_states[id].pos_prev_err = error_angle;
                        _pid_states[id].pos_integral = 0.0f;
                        float profile_rpm = limit;
                        const float safe_accel = _pid_states[id].accel_limit_rpm_s * 0.85f;
                        const float decel_limit = std::sqrt((safe_accel * abs_error) / 3.0f);
                        if (profile_rpm > decel_limit) profile_rpm = decel_limit;
                        if (error_angle < 0.0f) profile_rpm = -profile_rpm;
                        raw_target_rpm = profile_rpm;
                    } else if (abs_error > kPositionDeadBandDeg) {
                        raw_target_rpm = pos_pid_calculate(
                            id, _target_angles[id], current_angle, dt, limit);
                        bypass_accel_limit = true;
                    } else {
                        _pid_states[id].pos_prev_err = 0.0f;
                        _pid_states[id].pos_integral = 0.0f;
                        raw_target_rpm = 0.0f;
                        bypass_accel_limit = true;
                    }
                } else {
                    raw_target_rpm = static_cast<float>(_target_speeds[id]);
                }

                if (raw_target_rpm > limit) raw_target_rpm = limit;
                if (raw_target_rpm < -limit) raw_target_rpm = -limit;

                if (_pid_states[id].accel_limit_rpm_s > 0.0f && !bypass_accel_limit) {
                    const float max_delta = _pid_states[id].accel_limit_rpm_s * dt;
                    const float old_target = _pid_states[id].current_target_rpm;
                    if (raw_target_rpm > old_target + max_delta) {
                        _pid_states[id].current_target_rpm = old_target + max_delta;
                    } else if (raw_target_rpm < old_target - max_delta) {
                        _pid_states[id].current_target_rpm = old_target - max_delta;
                    } else {
                        _pid_states[id].current_target_rpm = raw_target_rpm;
                    }
                } else {
                    _pid_states[id].current_target_rpm = raw_target_rpm;
                }

                final_output = static_cast<int>(pid_calculate(
                    id, _pid_states[id].current_target_rpm, current_rpm, dt));
            }

            if (final_output > _motor_max) final_output = _motor_max;
            if (final_output < -_motor_max) final_output = -_motor_max;
            if (final_output > _torque_limits[id]) final_output = _torque_limits[id];
            if (final_output < -_torque_limits[id]) final_output = -_torque_limits[id];
            _output_torques[id] = final_output;
        }
    }
}

int rbms::rbms_send() {
    if (_motor_num <= 0) return -1;

    CANMessage low;
    CANMessage high;
    low.id = 0x200;
    low.len = 8;
    high.id = 0x1FF;
    high.len = 8;
    for (int i = 0; i < 8; ++i) {
        low.data[i] = 0;
        high.data[i] = 0;
    }

    bool send_low = false;
    bool send_high = false;
    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        for (int i = 0; i < _motor_num; ++i) {
            const int global_id = i + _id_offset;
            const int value = _output_torques[i];
            if (global_id < 4) {
                low.data[global_id * 2] = static_cast<char>((value >> 8) & 0xFF);
                low.data[global_id * 2 + 1] = static_cast<char>(value & 0xFF);
                send_low = true;
            } else {
                const int slot = global_id - 4;
                high.data[slot * 2] = static_cast<char>((value >> 8) & 0xFF);
                high.data[slot * 2 + 1] = static_cast<char>(value & 0xFF);
                send_high = true;
            }
        }
    }

    bool success = true;
    if (send_low && !_can.write(low)) success = false;
    if (send_high && !_can.write(high)) success = false;
    return success ? 1 : -1;
}

void rbms::parse_can_data(int id, const CANMessage &msg, short *rotation, short *speed) {
    if (!valid_local_id(id) || rotation == nullptr || speed == nullptr) return;

    const uint8_t b0 = static_cast<uint8_t>(msg.data[0]);
    const uint8_t b1 = static_cast<uint8_t>(msg.data[1]);
    const uint8_t b2 = static_cast<uint8_t>(msg.data[2]);
    const uint16_t raw_angle = static_cast<uint16_t>(((uint16_t)b0 << 8) | b1) & 0x1FFFU;
    *speed = static_cast<int16_t>(((uint16_t)b2 << 8) |
                                  static_cast<uint8_t>(msg.data[3]));
    *rotation = static_cast<short>((static_cast<float>(raw_angle) / 8192.0f) * 360.0f);

    std::lock_guard<std::mutex> lock(_data_mutex);
    PIDState &p = _pid_states[id];
    if (!p.is_initialized) {
        p.last_raw_angle = raw_angle;
        p.accumulated_angle = 0.0f;
        p.is_initialized = true;
        return;
    }

    int diff = static_cast<int>(raw_angle) - static_cast<int>(p.last_raw_angle);
    if (diff > 4096) diff -= 8192;
    else if (diff < -4096) diff += 8192;
    p.last_raw_angle = raw_angle;

    const float ratio = _gear_ratios[id];
    if (std::isfinite(ratio) && ratio > 0.0f) {
        p.accumulated_angle += (static_cast<float>(diff) / 8192.0f) * 360.0f / ratio;
    }
}

bool rbms::handle_message(const CANMessage &msg) {
    if (msg.len < 4) return false;
    const int global_index = static_cast<int>(msg.id) - 0x201;
    const int local_index = global_index - _id_offset;
    if (!valid_local_id(local_index)) return false;

    {
        std::lock_guard<std::mutex> lock(_data_mutex);
        _msg_buffer[local_index] = msg;
        _new_data_mask |= (1UL << local_index);
    }
    _event_flags.set(kDataEvent);
    return true;
}

bool rbms::claims_id(uint32_t id) const {
    const uint32_t first_id = 0x201U + static_cast<uint32_t>(_id_offset);
    const uint32_t end_id = first_id + static_cast<uint32_t>(_motor_num);
    return _motor_num > 0 && id >= first_id && id < end_id;
}

float rbms::get_angle(int id) {
    if (!valid_local_id(id)) return 0.0f;
    std::lock_guard<std::mutex> lock(_data_mutex);
    return _pid_states[id].accumulated_angle;
}

void rbms::force_set_angle(int id, float new_angle) {
    if (!valid_local_id(id) || !std::isfinite(new_angle)) return;
    std::lock_guard<std::mutex> lock(_data_mutex);
    _pid_states[id].accumulated_angle = new_angle;
    _pid_states[id].pos_integral = 0.0f;
    _pid_states[id].pos_prev_err = 0.0f;
    _target_angles[id] = new_angle;
}
