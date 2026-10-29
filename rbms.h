#ifndef INCLUDED_rbms_H
#define INCLUDED_rbms_H

#include "mbed.h"
#include "CANManager.h"

class rbms : public CANReceiver {
public:
    enum ControlMode {
        SPD_MODE,
        TRQ_MODE,
        POS_MODE
    };

    // 【修正】第4引数に id_offset（ID開始位置オフセット）を追加（デフォルト値: 0）
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
    void set_gear_ratio(int id, float gear_ratio);

    bool handle_message(const CANMessage &msg) override;
    void spd_control();
    int rbms_send();
    void rbms_read(CANMessage &msg, short *rotation, short *speed);

    float get_angle(int id) {
        if (id < 0 || id >= _motor_num) return 0.0f;
        return _pid_states[id].accumulated_angle;
    }

    int get_tqr(int id) {
        if (id < 0 || id >= _motor_num) return 0.0f;
        return _output_torques[id];
    }
    //モーター動かさずにプログラム内の角度のみ変更
    void force_set_angle(int id, float new_angle = 0.0f);

private:
    void control_thread_entry();
    float pid_calculate(int id, float target, float current, float dt);
    float pos_pid_calculate(int id, float target, float current, float dt, float limit);
    void parse_can_data(int id, const CANMessage &msg, short *rotation, short *speed);

    CAN &_can;
    bool _motor_type;
    int _motor_num, _motor_max;
    int _id_offset; // 追加:モーターIDオフセット（0x201からのズレ）を保持する変数
    
    ControlMode _control_modes[8];
    int _target_speeds[8];
    int _target_torques[8];
    float _target_angles[8];
    int _output_torques[8];
    int _torque_limits[8];
    float _gear_ratios[8];

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
