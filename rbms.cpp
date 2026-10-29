#include "rbms.h"
#include "mbed.h"
#include <cmath>
#include <mutex>
// 修正:コンストラクタで _id_offset を初期化リストに追加
rbms::rbms(CAN &can, bool motor_type, int motor_num, int id_offset)
    : _can(can), _motor_type(motor_type), _motor_num(motor_num), _id_offset(id_offset) {
    if (_motor_type) { // M3508
        _kp = 35.0f; _ki = 50.0f; _kd = 0.0f;
        _kp_p = 5.0f; _ki_p = 0.0f; _kd_p = 0.15f;
        _motor_max = 16384;
    } else { // M2006
        _kp = 15.0f; _ki = 12.0f; _kd = 0.0f;
        _kp_p = 4.5f; _ki_p = 0.0f; _kd_p = 0.25f;
        _motor_max = 10000;
    }
    for(int i = 0; i < 8; i++) {
        _control_modes[i] = SPD_MODE;
        _target_speeds[i] = 0;
        _target_torques[i] = 0;
        _target_angles[i] = 0.0f;
        _output_torques[i] = 0;
        _gear_ratios[i] = _motor_type ? 19.2f : 36.0f;
        
        _pid_states[i].prev_err = 0.0f;
        _pid_states[i].integral = 0.0f;
        _pid_states[i].pos_prev_err = 0.0f;
        _pid_states[i].pos_integral = 0.0f;
        _pid_states[i].last_raw_angle = 0;
        _pid_states[i].accumulated_angle = 0.0f;
        _pid_states[i].is_initialized = false;

        _pid_states[i].speed_limit_rpm = 0.0f;
        _pid_states[i].accel_limit_rpm_s = 0.0f;
        _pid_states[i].current_target_rpm = 0.0f;
        
        _pid_states[i].timer.start();
    }
    if(_motor_num <= 8){
        _can.frequency(1000000);
        _can.mode(CAN::Normal);
    }
    for(int i = 0; i < 8; i++){
        _torque_limits[i] = 10000;
    }
}

void rbms::set_control_mode(int id, ControlMode mode) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _control_modes[id] = mode;
    _pid_states[id].integral = 0;
    _pid_states[id].prev_err = 0;
    _pid_states[id].pos_integral = 0;
    _pid_states[id].pos_prev_err = 0;
    _data_mutex.unlock();
}

void rbms::set_target_speed(int id, int speed) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _target_speeds[id] = speed;
    _data_mutex.unlock();
}

void rbms::set_target_torque(int id, int torque) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _target_torques[id] = torque;
    _data_mutex.unlock();
}

void rbms::set_gear_ratio(int id, float gear_ratio) {
    if (id < 0 || id >= _motor_num) return;
    if (gear_ratio <= 0.0f) return;

    _data_mutex.lock();
    _gear_ratios[id] = gear_ratio;
    _data_mutex.unlock();
}

void rbms::set_target_angle(int id, float angle) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _target_angles[id] = angle;
    _data_mutex.unlock();
}

void rbms::reset_angle(int id) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _pid_states[id].accumulated_angle = 0.0f;
    _target_angles[id] = 0.0f;
    _data_mutex.unlock();
}

void rbms::set_pid_gains(float kp, float ki, float kd) {
    _data_mutex.lock();
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _data_mutex.unlock();
}

void rbms::set_pos_pid_gains(float kp, float ki, float kd) {
    _data_mutex.lock();
    _kp_p = kp;
    _ki_p = ki;
    _kd_p = kd;
    _data_mutex.unlock();
}

void rbms::set_speed_limit(int id, float max_speed) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _pid_states[id].speed_limit_rpm = max_speed;
    _data_mutex.unlock();
}

void rbms::set_accel_limit(int id, float max_accel) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _pid_states[id].accel_limit_rpm_s = max_accel;
    _data_mutex.unlock();
}

void rbms::set_torque_limit(int id, int max_torque){
    if(id < 0 || id >= _motor_num) return;
    if(max_torque < 0) max_torque = -max_torque;
    if(max_torque > 10000) max_torque = 10000;

    _data_mutex.lock();
    _torque_limits[id] = max_torque;
    _data_mutex.unlock();
}

float rbms::pid_calculate(int id, float target, float current, float dt) {
    float error = target - current;
    _pid_states[id].integral += (error + _pid_states[id].prev_err) * dt / 2.0f;

    float integral_limit = _motor_max / (_ki > 0.0f ? _ki : 1.0f); 
    if (_pid_states[id].integral > integral_limit) {
        _pid_states[id].integral = integral_limit;
    } else if (_pid_states[id].integral < -integral_limit) {
        _pid_states[id].integral = -integral_limit;
    }

    float derivative = (error - _pid_states[id].prev_err) / dt;
    float out = (_kp * error) + (_ki * _pid_states[id].integral) + (_kd * derivative);
    _pid_states[id].prev_err = error;
    return out;
}

float rbms::pos_pid_calculate(int id, float target, float current, float dt, float limit) {
    float error = target - current;
    _pid_states[id].pos_integral += (error + _pid_states[id].pos_prev_err) * dt / 2.0f;

    float integral_limit = limit / (_ki_p > 0.0f ? _ki_p : 1.0f); 
    if (_pid_states[id].pos_integral > integral_limit) {
        _pid_states[id].pos_integral = integral_limit;
    } else if (_pid_states[id].pos_integral < -integral_limit) {
        _pid_states[id].pos_integral = -integral_limit;
    }

    float derivative = (error - _pid_states[id].pos_prev_err) / dt;
    float out_target_speed = (_kp_p * error) + (_ki_p * _pid_states[id].pos_integral) + (_kd_p * derivative);
    _pid_states[id].pos_prev_err = error;
    
    return out_target_speed;
}

void rbms::spd_control() {
    _thread.start(callback(this, &rbms::control_thread_entry));
}

void rbms::control_thread_entry() {
    while (true) {
        _event_flags.wait_any(0x01); 

        for (int id = 0; id < _motor_num; id++) {
            CANMessage local_msg;
            bool has_new = false;
            ControlMode mode;
            int target_s, target_t;
            float target_a;

            _data_mutex.lock();
            if (_new_data_mask & (1 << id)) {
                local_msg = _msg_buffer[id];
                _new_data_mask &= ~(1 << id);
                has_new = true;
            }
            mode = _control_modes[id];
            target_s = _target_speeds[id];
            target_t = _target_torques[id];
            target_a = _target_angles[id];
            _data_mutex.unlock();

            if (has_new) {
                float dt = _pid_states[id].timer.read();
                _pid_states[id].timer.reset();                    
                
                if (dt <= 0.0f || dt > 0.05f) {
                    dt = 0.001f;
                    _pid_states[id].integral = 0.0f;
                    _pid_states[id].prev_err = 0.0f;
                    _pid_states[id].pos_integral = 0.0f;
                    _pid_states[id].pos_prev_err = 0.0f;
                    _pid_states[id].current_target_rpm = 0.0f; 
                }

                short rot, raw_spd;
                parse_can_data(id, local_msg, &rot, &raw_spd); 
                
                int final_out = 0;

                float gear_ratio;

                _data_mutex.lock();
                gear_ratio = _gear_ratios[id];
                _data_mutex.unlock();

                float current_rpm = raw_spd / gear_ratio;
                
                if (mode == POS_MODE || mode == SPD_MODE) {
                    float limit = (_pid_states[id].speed_limit_rpm > 0.0f) ? 
                                  _pid_states[id].speed_limit_rpm : 
                                  ((mode == POS_MODE) ? 500.0f : 10000.0f);

                    float raw_target_rpm = 0.0f;
                    bool bypass_accel_limit = false;

                    if (mode == POS_MODE) {
                        float current_angle = _pid_states[id].accumulated_angle;
                        float error_angle = target_a - current_angle;
                        float abs_error = std::fabs(error_angle);
                        const float SETTLING_THRESHOLD = 25.0f; 
                        const float DEAD_BAND = 2.5f;

                        if (abs_error > SETTLING_THRESHOLD && _pid_states[id].accel_limit_rpm_s > 0.0f) {
                            _pid_states[id].pos_prev_err = error_angle; 
                            _pid_states[id].pos_integral = 0.0f;
                            float profile_rpm = limit;

                            if (_pid_states[id].accel_limit_rpm_s > 0.0f) {
                                float safe_accel = _pid_states[id].accel_limit_rpm_s * 0.85f; 
                                float decel_limit = std::sqrt((safe_accel * abs_error) / 3.0f);
                                if (profile_rpm > decel_limit) {
                                    profile_rpm = decel_limit;
                                }
                            }

                            if (error_angle < 0.0f) profile_rpm = -profile_rpm;
                            raw_target_rpm = profile_rpm;

                        } else if(abs_error > DEAD_BAND){
                            raw_target_rpm = pos_pid_calculate(id, target_a, current_angle, dt, limit);
                            bypass_accel_limit = true; 
                        } else {
                            _pid_states[id].pos_prev_err = 0.0f; 
                            _pid_states[id].pos_integral = 0.0f; 
                            raw_target_rpm = 0.0f; 
                            bypass_accel_limit = true;
                        }
                    } else {
                        raw_target_rpm = (float)target_s;
                    }

                    if (raw_target_rpm > limit) raw_target_rpm = limit;
                    if (raw_target_rpm < -limit) raw_target_rpm = -limit;

                    if (_pid_states[id].accel_limit_rpm_s > 0.0f && !bypass_accel_limit) {
                        float max_delta = _pid_states[id].accel_limit_rpm_s * dt;
                        
                        if (raw_target_rpm > _pid_states[id].current_target_rpm + max_delta) {
                            _pid_states[id].current_target_rpm += max_delta;
                        } else if (raw_target_rpm < _pid_states[id].current_target_rpm - max_delta) {
                            _pid_states[id].current_target_rpm -= max_delta;
                        } else {
                            _pid_states[id].current_target_rpm = raw_target_rpm;
                        }
                    } else {
                        _pid_states[id].current_target_rpm = raw_target_rpm;
                    }

                    final_out = (int)pid_calculate(id, _pid_states[id].current_target_rpm, current_rpm, dt);

                } else {
                    final_out = target_t;
                    _pid_states[id].current_target_rpm = current_rpm;
                }

                if (final_out > _motor_max) final_out = _motor_max;
                else if (final_out < -_motor_max) final_out = -_motor_max;

                _data_mutex.lock();
                _output_torques[id] = final_out;

                if(_output_torques[id] > _torque_limits[id])  _output_torques[id] = _torque_limits[id];
                if(_output_torques[id] < -_torque_limits[id]) _output_torques[id] = -_torque_limits[id];
                _data_mutex.unlock();
            }
        }
    }
}

// 修正:_id_offset を考慮し、必要な CAN ID (0x200 または 0x1FF) のみを送信するよう変更
int rbms::rbms_send() {
    _tx_msg_low.id = 0x200; _tx_msg_low.len = 8;
    _tx_msg_high.id = 0x1ff; _tx_msg_high.len = 8;

    // 追加:送信フラグ（他方インスタンスのデータへのゼロ上書き干渉を防止）
    bool send_low = false;
    bool send_high = false;

    _data_mutex.lock();
    for(int i = 0; i < _motor_num; i++) {
        int global_id = i + _id_offset; //追加:実機の物理IDインデックス(0~7)を算出
        int val = _output_torques[i];

        if (global_id < 4) {
            _tx_msg_low.data[global_id * 2]     = (char)(val >> 8);
            _tx_msg_low.data[global_id * 2 + 1] = (char)(val & 0xFF);
            send_low = true; // 0x200 に該当データあり
        } else if (global_id < 8) {
            _tx_msg_high.data[(global_id - 4) * 2]     = (char)(val >> 8);
            _tx_msg_high.data[(global_id - 4) * 2 + 1] = (char)(val & 0xFF);
            send_high = true; // 0x1FF に該当データあり
        }
    }
    _data_mutex.unlock();

    // 修正:データが存在する CAN メッセージのみを送信する
    bool success = true;
    if (send_low)  success = success && _can.write(_tx_msg_low);
    if (send_high) success = success && _can.write(_tx_msg_high);

    return success ? 1 : -1;
}

void rbms::parse_can_data(int id, const CANMessage &msg, short *rotation, short *speed) {
    uint16_t raw_angle = (msg.data[0] << 8) | (msg.data[1] & 0xff);
    *speed = (int16_t)((msg.data[2] << 8) | (msg.data[3] & 0xff));

    _data_mutex.lock();

    if (!_pid_states[id].is_initialized) {
        _pid_states[id].last_raw_angle = raw_angle;
        _pid_states[id].accumulated_angle = 0.0f;
        _pid_states[id].is_initialized = true;
    }

    int16_t diff = raw_angle - _pid_states[id].last_raw_angle;
    if (diff > 4096) diff -= 8192;
    else if (diff < -4096) diff += 8192;

    _pid_states[id].last_raw_angle = raw_angle;

    float gear_ratio = _gear_ratios[id];
    _pid_states[id].accumulated_angle +=　((float)diff / 8192.0f) * 360.0f / gear_ratio;

    _data_mutex.unlock();

    *rotation = (short)(raw_angle / 8192.0f * 360.0f);
}

//修正:_id_offset を考慮して受信メッセージの該当ローカルインデックスを計算
bool rbms::handle_message(const CANMessage &msg) {
    int global_idx = msg.id - 0x201; // 例: 0x205 -> global_idx = 4
    int local_idx = global_idx - _id_offset; // 例: 4 - 4 = local_idx 0

    if (local_idx >= 0 && local_idx < _motor_num) {
        _data_mutex.lock();
        _msg_buffer[local_idx] = msg;
        _new_data_mask |= (1 << local_idx);
        _data_mutex.unlock();
        _event_flags.set(0x01); 
        return true;
    }
    return false;
}

//モーターを動かさずにプログラム内の角度のみ変更
void rbms::force_set_angle(int id, float new_angle) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _pid_states[id].accumulated_angle = new_angle;
    _pid_states[id].pos_integral = 0.0f;
    _pid_states[id].pos_prev_err = 0.0f;
    _target_angles[id] = new_angle;
    _data_mutex.unlock();
}
