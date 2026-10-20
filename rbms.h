#ifndef INCLUDED_rbms_H
#define INCLUDED_rbms_H

#include "mbed.h"
#include "CANManager.h"
#include <cstdint>

class rbms : public CANReceiver {
public:
    enum ControlMode {
        SPD_MODE,
        TRQ_MODE,
        POS_MODE
    };

    // RLSの信頼性診断。get_rls_diagnostics() は診断値を取得できればtrueを返し、
    // 実際にモデルを利用できるかは valid と各判定フラグで確認する。
    struct RLSDiagnostics {
        bool enabled;
        bool numerical_ok;
        bool sample_count_ok;
        bool rmse_ok;
        bool coefficients_stable;
        bool input_excitation_ok;
        bool data_fresh;
        bool valid;
        uint32_t samples;
        float a;
        float b;                 // 速度[RPM]モデルに対応する係数
        float rmse_rpm;          // 予測誤差の指数移動RMSE [RPM]
        float last_error_rpm;    // 直近の1ステップ予測誤差 [RPM]
        float current_std_raw;   // 直近の電流カウント値の指数移動標準偏差
        float coefficient_delta_a; // 直近の25サンプル窓でのa変化量
        float coefficient_delta_b; // 直近の25サンプル窓での内部b変化量
        uint32_t last_update_age_ms;
    };

    rbms(CAN &can, bool motor_type, int motor_num, int id_offset = 0);

    void set_control_mode(int id, ControlMode mode);
    void set_target_speed(int id, int speed);
    void set_target_torque(int id, int torque);
    void set_target_angle(int id, float angle);
    void reset_angle(int id);
    void set_pid_gains(float kp, float ki, float kd);
    void set_pos_pid_gains(float kp, float ki, float kd);
    void set_speed_limit(int id, float max_speed);
    void set_accel_limit(int id, float max_accel);
    void set_torque_limit(int id, int max_torque);

    bool handle_message(const CANMessage &msg) override;
    void spd_control();
    int rbms_send();
    void rbms_read(CANMessage &msg, short *rotation, short *speed);

    float get_angle(int id) {
        if (id < 0 || id >= _motor_num) return 0.0f;
        _data_mutex.lock();
        float value = _pid_states[id].accumulated_angle;
        _data_mutex.unlock();
        return value;
    }

    int get_tqr(int id) {
        if (id < 0 || id >= _motor_num) return 0;
        _data_mutex.lock();
        int value = _output_torques[id];
        _data_mutex.unlock();
        return value;
    }

    // ESCフィードバック電流（生のCAN値。アンペア換算ではない）
    int get_current(int id);

    // RLSモデルを取得。品質基準をすべて満たすときだけtrueを返す。
    // APIのモデル形式: speed[k] = a*speed[k-1] + b*(current_raw[k-1]/1000)
    bool get_rls_model(int id, float *a, float *b);
    void enable_rls(int id, bool enabled = true);

    // 診断値を取得。戻り値trueは「診断情報を取得できた」を意味する。
    // モデルの利用可否は out->valid を確認する。
    bool get_rls_diagnostics(int id, RLSDiagnostics *out);

    // 信頼性判定のしきい値を設定。設定値は実測ログに基づいて調整する。
    // max_rmse_rpmは速度単位[RPM]、min_current_std_rawは生電流カウント単位。
    void set_rls_validation_limits(int id,uint32_t min_samples,float max_rmse_rpm,float min_current_std_raw,uint32_t max_data_age_ms);

    // モーターを動かさずにプログラム内の角度のみ変更
    void force_set_angle(int id, float new_angle = 0.0f);

private:
    void control_thread_entry();
    float pid_calculate(int id, float target, float current, float dt);
    float pos_pid_calculate(int id, float target, float current, float dt, float limit);
    void parse_can_data(int id, const CANMessage &msg, short *rotation, short *speed);
    void rls_update(int id, float speed_rpm, float current_raw);

    CAN &_can;
    bool _motor_type;
    int _motor_num, _motor_max;
    int _id_offset;

    ControlMode _control_modes[8];
    int _target_speeds[8];
    int _target_torques[8];
    float _target_angles[8];
    int _output_torques[8];
    int _torque_limits[8];
    int _measured_currents[8];

    struct RLSState {
        // 内部モデルは正規化速度と正規化電流を使う。
        // speed_norm[k] = a*speed_norm[k-1] + b*current_norm[k-1]
        float a;
        float b;
        float p00, p01, p10, p11;
        float prev_speed_scaled;
        float prev_current_scaled;
        bool initialized;
        bool enabled;
        bool numerical_valid;
        uint32_t samples;

        // RMSE（指数移動平均の二乗誤差）
        float rmse_ema_sq;
        bool rmse_initialized;
        float last_error_rpm;

        // 入力電流の直近の指数移動分散
        float current_mean_scaled;
        float current_var_ema_scaled;
        uint32_t excitation_samples;

        // 係数安定性を25サンプルごとに確認するための状態
        float stability_anchor_a;
        float stability_anchor_b;
        float last_delta_a;
        float last_delta_b;
        uint32_t stability_window_samples;
        uint8_t stable_windows;
        bool coefficients_stable;
        

        // モデルの品質判定しきい値（初期値は必ず実測で調整）
        uint32_t min_valid_samples;
        float max_rmse_rpm;
        float min_current_std_raw;
        uint32_t max_data_age_ms;
        Timer last_update_timer;
    } _rls_states[8];

    float _kp, _ki, _kd;
    float _kp_p, _ki_p, _kd_p;

    Thread _thread;
    Mutex _data_mutex;
    EventFlags _event_flags;

    struct PIDState {
        float prev_err;
        float integral;
        float pos_prev_err;
        float pos_integral;
        uint16_t last_raw_angle;
        float accumulated_angle;
        bool is_initialized;
        float speed_limit_rpm;
        float accel_limit_rpm_s;
        float current_target_rpm;
        Timer timer;
    } _pid_states[8];

    CANMessage _tx_msg_low, _tx_msg_high;
    CANMessage _msg_buffer[8];
    uint8_t _new_data_mask;
};

#endif
