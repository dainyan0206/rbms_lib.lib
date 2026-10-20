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
        _measured_currents[i] = 0;

        RLSState &rs = _rls_states[i];
        rs.a = 0.98f;
        rs.b = 0.0f;//内部では速度・電流を正規化したモデルの定数
        rs.p00 = 1000.0f;
        rs.p10 = 0.0f;
        rs.p11 = 1000.0f;
        rs.prev_speed_scaled = 0.0f;
        rs.prev_current_scaled = 0.0f;
        rs.initialized = false;
        rs.enabled = false;
        rs.numerical_valid = true;
        rs.samples = 0;
        rs.rmse_initialized = true;
        rs.samples = 0;
        rs.rmse_ema_sq = 0.0f;
        rs.rmse_initialized = false;
        rs.current_mean_scaled = 0.0f;
        rs.current_var_ema_scaled = 0.0f;
        rs.excitation_samples = 0;
        rs.stability_anchor_a = rs.a;
        rs.stability_anchor_b = rs.b;
        rs.last_delta_a = 0.0f;
        rs.last_delta_b = 0.0f;
        rs.stability_window_samples = 0;
        rs.stable_windows = 0;
        rs.coefficients_stable = false;
        rs.last_error_rpm = 0.0f;
        rs.min_valid_samples = 200;
        rs.max_rmse_rpm = 50.0f;
        rs.min_current_std_raw = 50.0f;
        rs.max_data_age_ms = 50;
        rs.last_update_timer.start();

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
    _new_data_mask = 0;
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
                float current_rpm = _motor_type ? (raw_spd / 19.2f) : (raw_spd / 36.0f);
                // ESCフィードバックの電流フィールド（data[4..5]）
                const int16_t raw_current = (int16_t)(((uint16_t)(uint8_t)local_msg.data[4] << 8) |(uint8_t)local_msg.data[5]);
                rls_update(id, current_rpm, (float)raw_current);

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

int rbms::get_current(int id) {
    if (id < 0 || id >= _motor_num) return 0;
    _data_mutex.lock();
    int value = _measured_currents[id];
    _data_mutex.unlock();
    return value;
}

void rbms::enable_rls(int id, bool enabled) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    RLSState &s = _rls_states[id];
    s.enabled = enabled;
    s.initialized = false;
    s.numerical_valid = true;
    s.samples = 0;
    s.a = 0.98f;
    s.b = 0.0f;
    s.p00 = 1000.0f;
    s.p01 = 0.0f;
    s.p10 = 0.0f;
    s.p11 = 1000.0f;

    s.prev_speed_scaled = 0.0f;
    s.prev_current_scaled = 0.0f;
    s.rmse_ema_sq = 0.0f;
    s.rmse_initialized = false;
    s.current_mean_scaled = 0.0f;
    s.current_var_ema_scaled = 0.0f;
    s.excitation_samples = 0;

    s.stability_anchor_a = s.a;
    s.stability_anchor_b = s.b;
    s.last_delta_b = 0.0f;
    s.last_delta_a = 0.0f;
    s.stability_window_samples = 0;
    s.stable_windows = 0;
    s.coefficients_stable = false;
    s.last_error_rpm = 0.0f;
    s.last_update_timer.reset();
    
    _data_mutex.unlock();
}

void rbms::set_rls_validation_limits(int id, uint32_t min_samples, float max_rmse_rpm, float min_current_std_raw, uint32_t max_data_age_ms){
    if(id < 0 || id>= _motor_num) return;
    if(min_samples < 1 || !std::isfinite(max_rmse_rpm) || max_rmse_rpm <= 0.0f || !std::isfinite(min_current_std_raw) || min_current_std_raw < 0.0f || max_data_age_ms == 0){
        return;
    }
    _data_mutex.lock();
    RLSState &s = _rls_states[id];
        s.min_valid_samples = min_samples;
    s.max_rmse_rpm = max_rmse_rpm;
    s.min_current_std_raw = min_current_std_raw;
    s.max_data_age_ms = max_data_age_ms;
    _data_mutex.unlock();
}

bool rbms::get_rls_diagnostics(int id, RLSDiagnostics *out) {
    if (id < 0 || id >= _motor_num || out == nullptr) return false;

    _data_mutex.lock();
    RLSState &s = _rls_states[id];

    out->enabled = s.enabled;
    out->numerical_ok = s.numerical_valid &&
        std::isfinite(s.a) && std::isfinite(s.b) &&
        std::isfinite(s.p00) && std::isfinite(s.p01) &&
        std::isfinite(s.p10) && std::isfinite(s.p11) &&
        std::isfinite(s.rmse_ema_sq) && s.rmse_ema_sq >= 0.0f &&
        std::isfinite(s.current_mean_scaled) &&
        std::isfinite(s.current_var_ema_scaled) && s.current_var_ema_scaled >= 0.0f &&
        std::isfinite(s.last_error_rpm) &&
        s.p00 > 0.0f && s.p11 > 0.0f &&
        (s.p00 * s.p11 - s.p01 * s.p10) > 0.0f;
    out->samples = s.samples;
    out->a = s.a;
    // 内部速度をRPM/1000に正規化しているため、APIでは元のRPMモデルのbに戻す。
    out->b = s.b * 1000.0f;
    out->rmse_rpm = s.rmse_initialized
        ? std::sqrt((s.rmse_ema_sq > 0.0f) ? s.rmse_ema_sq : 0.0f) * 1000.0f
        : std::numeric_limits<float>::infinity();
    out->last_error_rpm = s.last_error_rpm;
    out->coefficient_delta_a = s.last_delta_a;
    out->coefficient_delta_b = s.last_delta_b * 1000.0f; // APIのb単位に換算

    // 直近の入力変動を反映する指数移動分散から標準偏差を算出。
    const float current_var = (s.current_var_ema_scaled > 0.0f)
        ? s.current_var_ema_scaled : 0.0f;
    out->current_std_raw = std::sqrt(current_var) * 1000.0f;

    const int age_ms = s.last_update_timer.read_ms();
    out->last_update_age_ms = (age_ms >= 0)
        ? static_cast<uint32_t>(age_ms)
        : UINT32_MAX;

    out->sample_count_ok = s.samples >= s.min_valid_samples;
    out->rmse_ok = s.rmse_initialized &&
        std::isfinite(out->rmse_rpm) && out->rmse_rpm <= s.max_rmse_rpm;
    out->coefficients_stable = s.coefficients_stable;
    out->input_excitation_ok = s.excitation_samples >= s.min_valid_samples &&
        std::isfinite(out->current_std_raw) &&
        out->current_std_raw >= s.min_current_std_raw;
    out->data_fresh = age_ms >= 0 &&
        static_cast<uint32_t>(age_ms) <= s.max_data_age_ms;

    out->valid = out->enabled && out->numerical_ok &&
        out->sample_count_ok && out->rmse_ok &&
        out->coefficients_stable && out->input_excitation_ok &&
        out->data_fresh;

    _data_mutex.unlock();
    return true; // 診断情報を取得できた。モデルの有効性は out->valid を確認する。
}

bool rbms::get_rls_model(int id, float *a, float *b) {
    if (id < 0 || id >= _motor_num || a == nullptr || b == nullptr) return false;
    _data_mutex.lock();
    const RLSState &s = _rls_states[id];

        bool numerical_ok = s.numerical_valid &&
        std::isfinite(s.a) && std::isfinite(s.b) &&
        std::isfinite(s.p00) && std::isfinite(s.p01) &&
        std::isfinite(s.p10) && std::isfinite(s.p11) &&
        std::isfinite(s.rmse_ema_sq) && s.rmse_ema_sq >= 0.0f &&
        std::isfinite(s.current_mean_scaled) &&
        std::isfinite(s.current_var_ema_scaled) && s.current_var_ema_scaled >= 0.0f &&
        std::isfinite(s.last_error_rpm) &&
        s.p00 > 0.0f && s.p11 > 0.0f &&
        (s.p00 * s.p11 - s.p01 * s.p10) > 0.0f;

    float rmse_rpm = s.rmse_initialized
        ? std::sqrt((s.rmse_ema_sq > 0.0f) ? s.rmse_ema_sq : 0.0f) * 1000.0f
        : std::numeric_limits<float>::infinity();

    const float current_var = (s.current_var_ema_scaled > 0.0f)
        ? s.current_var_ema_scaled : 0.0f;
    const float current_std_raw = std::sqrt(current_var) * 1000.0f;

    const int age_ms = s.last_update_timer.read_ms();
    bool valid = s.enabled && numerical_ok &&
        s.samples >= s.min_valid_samples &&
        s.rmse_initialized && std::isfinite(rmse_rpm) &&
        rmse_rpm <= s.max_rmse_rpm &&
        s.coefficients_stable &&
        s.excitation_samples >= s.min_valid_samples &&
        std::isfinite(current_std_raw) &&
        current_std_raw >= s.min_current_std_raw &&
        age_ms >= 0 && static_cast<uint32_t>(age_ms) <= s.max_data_age_ms;

    if (valid) {
        *a = s.a;
        // 内部モデル: speed_rpm/1000 = a*(prev_speed_rpm/1000)
        //                              + b_internal*(current_raw/1000)
        // 従来のAPIモデル: speed_rpm = a*prev_speed_rpm + b_api*(current_raw/1000)
        *b = s.b * 1000.0f;
    }

    _data_mutex.unlock();
    return valid;
}

void rbms::rls_update(int id, float speed_rpm, float current_raw) {
    if (id < 0 || id >= _motor_num ||
        !std::isfinite(speed_rpm) || !std::isfinite(current_raw)) {
        return;
    }

    _data_mutex.lock();
    _measured_currents[id] = static_cast<int>(current_raw);
    RLSState &s = _rls_states[id];
    s.last_update_timer.reset();

    if (!s.enabled || !s.numerical_valid) {
        _data_mutex.unlock();
        return;
    }

    const float speed_scaled = speed_rpm / 1000.0f;
    const float current_scaled = current_raw / 1000.0f;

    if (!s.initialized) {
        s.prev_speed_scaled = speed_scaled;
        s.prev_current_scaled = current_scaled;
        s.initialized = true;
        _data_mutex.unlock();
        return;
    }

    // phi = [前回の正規化速度, 前回の正規化電流]
    const float phi0 = s.prev_speed_scaled;
    const float phi1 = s.prev_current_scaled;
    const float lambda = 0.995f;

    // q = P * phi
    const float q0 = s.p00 * phi0 + s.p01 * phi1;
    const float q1 = s.p10 * phi0 + s.p11 * phi1;
    const float denominator = lambda + phi0 * q0 + phi1 * q1;

    if (!std::isfinite(denominator) || denominator <= 1.0e-9f) {
        s.numerical_valid = false;
        s.initialized = false;
        _data_mutex.unlock();
        return;
    }

    const float k0 = q0 / denominator;
    const float k1 = q1 / denominator;
    const float prediction_scaled = s.a * phi0 + s.b * phi1;
    const float error_scaled = speed_scaled - prediction_scaled;

    // 係数の候補を作る。検査に通るまで現在の係数は変更しない。
    const float a_candidate = s.a + k0 * error_scaled;
    const float b_candidate = s.b + k1 * error_scaled;

    const float old00 = s.p00;
    const float old01 = s.p01;
    const float old10 = s.p10;
    const float old11 = s.p11;

    const float p00_candidate =
        (old00 - k0 * (phi0 * old00 + phi1 * old10)) / lambda;
    const float p01_candidate =
        (old01 - k0 * (phi0 * old01 + phi1 * old11)) / lambda;
    const float p10_candidate =
        (old10 - k1 * (phi0 * old00 + phi1 * old10)) / lambda;
    const float p11_candidate =
        (old11 - k1 * (phi0 * old01 + phi1 * old11)) / lambda;

    const bool candidates_finite =
        std::isfinite(a_candidate) && std::isfinite(b_candidate) &&
        std::isfinite(p00_candidate) && std::isfinite(p01_candidate) &&
        std::isfinite(p10_candidate) && std::isfinite(p11_candidate) &&
        p00_candidate > 0.0f && p11_candidate > 0.0f &&
        (p00_candidate * p11_candidate - p01_candidate * p10_candidate) > 0.0f;

    if (!candidates_finite) {
        s.numerical_valid = false;
        s.initialized = false;
        _data_mutex.unlock();
        return;
    }

    // 係数・共分散行列を確定
    s.a = a_candidate;
    s.b = b_candidate;
    s.p00 = p00_candidate;
    s.p11 = p11_candidate;
    const float off_diagonal = 0.5f * (p01_candidate + p10_candidate);
    s.p01 = off_diagonal;
    s.p10 = off_diagonal;

    // 予測RMSEを指数移動平均で記録（speed_scaled単位なのでgetterでRPMへ変換）。
    // alpha=0.05は初期値。応答性とノイズをログで確認して調整する。
    const float error_sq = error_scaled * error_scaled;
    const float rmse_alpha = 0.05f;
    if (!s.rmse_initialized) {
        s.rmse_ema_sq = error_sq;
        s.rmse_initialized = true;
    } else {
        s.rmse_ema_sq = (1.0f - rmse_alpha) * s.rmse_ema_sq
                      + rmse_alpha * error_sq;
    }
    s.last_error_rpm = error_scaled * 1000.0f;

    // 入力電流の直近の変動を指数移動分散で追跡する。
    // 古い変動だけで後々まで「励起あり」と判定し続けないようにする。
    const float variance_alpha = 0.05f;
    if (s.excitation_samples == 0) {
        s.current_mean_scaled = phi1;
        s.current_var_ema_scaled = 0.0f;
        s.excitation_samples = 1;
    } else {
        const float delta = phi1 - s.current_mean_scaled;
        s.current_mean_scaled += variance_alpha * delta;
        const float delta_after = phi1 - s.current_mean_scaled;
        s.current_var_ema_scaled =
            (1.0f - variance_alpha) * s.current_var_ema_scaled +
            variance_alpha * delta * delta_after;
        if (s.current_var_ema_scaled < 0.0f) {
            // 浮動小数点誤差による微小な負値を防ぐ
            s.current_var_ema_scaled = 0.0f;
        }
        if (s.excitation_samples < UINT32_MAX) {
            ++s.excitation_samples;
        }
    }

    if (s.samples < UINT32_MAX) {
        ++s.samples;
    }

    // 係数安定性を25回ごとに判定し、3区間連続で安定している場合にtrue。
    ++s.stability_window_samples;
    if (s.stability_window_samples >= 25) {
        const float delta_a = std::fabs(s.a - s.stability_anchor_a);
        const float delta_b = std::fabs(s.b - s.stability_anchor_b);
        s.last_delta_a = delta_a;
        s.last_delta_b = delta_b;
        float tolerance_b = 0.1f * std::fabs(s.stability_anchor_b);
        if (tolerance_b < 0.0001f) tolerance_b = 0.0001f;

        if (delta_a <= 0.02f && delta_b <= tolerance_b) {
            if (s.stable_windows < 255) ++s.stable_windows;
        } else {
            s.stable_windows = 0;
        }
        s.coefficients_stable = (s.stable_windows >= 3);
        s.stability_anchor_a = s.a;
        s.stability_anchor_b = s.b;
        s.stability_window_samples = 0;
    }

    s.prev_speed_scaled = speed_scaled;
    s.prev_current_scaled = current_scaled;
    _data_mutex.unlock();
}


void rbms::parse_can_data(int id, const CANMessage &msg, short *rotation, short *speed) {
    const uint8_t b0 = static_cast<uint8_t>(msg.data[0]);
    const uint8_t b1 = static_cast<uint8_t>(msg.data[1]);
    const uint8_t b2 = static_cast<uint8_t>(msg.data[2]);
    const uint8_t b3 = static_cast<uint8_t>(msg.data[3]);
    const uint16_t raw_angle =
        (static_cast<uint16_t>(b0) << 8) | static_cast<uint16_t>(b1);
    *speed = static_cast<int16_t>(
        (static_cast<uint16_t>(b2) << 8) | static_cast<uint16_t>(b3));

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

    float gear_ratio = _motor_type ? 19.2f : 36.0f;
    _pid_states[id].accumulated_angle += ((float)diff / 8192.0f) * 360.0f / gear_ratio;

    _data_mutex.unlock();

    *rotation = (short)(raw_angle / 8192.0f * 360.0f);
}

//修正:_id_offset を考慮して受信メッセージの該当ローカルインデックスを計算
bool rbms::handle_message(const CANMessage &msg) {
    // DJI ESCのフィードバックは8バイト想定。短いフレームをRLS/PIDに渡さない。
    if (msg.len != 8) return false;

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
