#ifndef INCLUDED_rbms_H
#define INCLUDED_rbms_H

#include "mbed.h"
#include "CANManager.h"
#include <cstdint>
#include <mutex>

class rbms : public CANReceiver {
public:
    enum ControlMode {
        SPD_MODE,  // 速度制御: 指令値は出力軸RPM
        TRQ_MODE,  // トルク/電流指令: ESCの生指令値
        POS_MODE   // 位置制御: 目標値は出力軸角度[deg]
    };

    // motor_type: true=M3508、false=M2006
    // motor_num : このインスタンスが扱うモーター数 (1～8)
    // id_offset : 最初のモーターのグローバル番号 (0～7)。
    //              global番号0はCAN受信ID 0x201に対応。
    rbms(CAN &can, bool motor_type, int motor_num, int id_offset = 0);

    rbms(const rbms &) = delete;
    rbms &operator=(const rbms &) = delete;

    void set_control_mode(int id, ControlMode mode);
    void set_target_speed(int id, int speed_rpm);       // 出力軸RPM
    void set_target_torque(int id, int current_command); // ESC生指令値
    void set_gear_ratio(int id, float gear_ratio);      // 出力軸RPM = モーターRPM / gear_ratio
    void set_target_angle(int id, float angle_deg);     // 出力軸角度[deg]
    void reset_angle(int id);

    void set_pid_gains(float kp, float ki, float kd);       // 全モーターの速度PID
    void set_pos_pid_gains(float kp, float ki, float kd);  // 全モーターの位置PID
    void set_speed_limit(int id, float max_speed_rpm);
    void set_accel_limit(int id, float max_accel_rpm_s);
    void set_torque_limit(int id, int max_command);

    // 制御スレッドを開始する。main()から1回だけ呼ぶ。
    void spd_control();

    // 現在計算されている出力指令をCAN送信する。
    // この版では、同じCANバス上の全対象モーターを1つのrbmsインスタンスで扱う構成
    int rbms_send();

    bool handle_message(const CANMessage &msg) override;
    bool claims_id(uint32_t id) const override;

    float get_angle(int id); // 出力軸角度[deg]

    // ソフトウェア上の角度だけを変更する。実モーターは動かない。
    void force_set_angle(int id, float new_angle = 0.0f);

private:
    static constexpr int MAX_MOTORS = 8;

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
    };

    void control_thread_entry();
    void parse_can_data(int id, const CANMessage &msg, short *rotation, short *speed);
    float pid_calculate(int id, float target, float current, float dt);
    float pos_pid_calculate(int id, float target, float current, float dt, float limit);
    bool valid_local_id(int id) const;

    CAN &_can;
    bool _motor_type;
    int _motor_num;
    int _id_offset;
    int _motor_max;

    float _kp;
    float _ki;
    float _kd;
    float _kp_p;
    float _ki_p;
    float _kd_p;

    ControlMode _control_modes[MAX_MOTORS];
    int _target_speeds[MAX_MOTORS];
    int _target_torques[MAX_MOTORS];
    float _target_angles[MAX_MOTORS];
    int _output_torques[MAX_MOTORS];
    int _torque_limits[MAX_MOTORS];
    float _gear_ratios[MAX_MOTORS];
    PIDState _pid_states[MAX_MOTORS];

    CANMessage _msg_buffer[MAX_MOTORS];
    uint32_t _new_data_mask;

    Thread _thread;
    EventFlags _event_flags;
    bool _thread_started;
    std::mutex _data_mutex;
};

#endif // INCLUDED_rbms_H
