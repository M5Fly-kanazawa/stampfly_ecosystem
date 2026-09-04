#include "workshop_api.hpp"

// =========================================================================
// Lesson 5: Rate P-Control + First Flight
// レッスン 5: レートP制御 + 初フライト
// =========================================================================
//
// Goal: Implement proportional (P) feedback control on angular rate.
// 目標: 角速度に対する比例(P)フィードバック制御を実装する
//
// Block diagram:
//   rc_stick --> [scale] --> target_rate -->(+)--> error --> [Kp] --> output --> motor_mixer
//                                           ^(-)
//                                           |
//                                      gyro (actual rate)

void setup()
{
    ws::print("Lesson 5: Rate P-Control");

    // TODO: Set your WiFi channel (1, 6, or 11)
    // TODO: 自分のWiFiチャンネルを設定する（1, 6, 11のいずれか）
    // ws::set_channel(1);
}

void loop_400Hz(float dt)
{
    // Safety: only run control when armed
    // 安全: ARM状態のときだけ制御を実行
    if (!ws::is_armed()) {
        ws::motor_stop_all();
        return;
    }

    float throttle = ws::rc_throttle();

    // --- P gain ---
    // P ゲイン
    float Kp_roll  = 0.5f;  // TODO: tune this value / この値を調整
    float Kp_pitch = 0.5f;  // TODO: tune this value / この値を調整
    float Kp_yaw   = 2.0f;  // TODO: tune this value / この値を調整

    // --- Maximum angular rate [rad/s] ---
    // 最大角速度 [rad/s]
    float rate_max_rp  = 1.0f;  // roll/pitch max rate
    float rate_max_yaw = 5.0f;  // yaw max rate

    // --- Roll axis ---
    // ロール軸
    // TODO: Compute target rate from stick input
    // ヒント: float roll_target = ws::rc_roll() * rate_max_rp;

    // TODO: Read actual rate from gyro
    // ヒント: float roll_actual = ws::gyro_x();

    // TODO: Compute error (target - actual)
    // TODO: Apply P gain to get control output

    float roll_output = 0.0f;  // Replace with your calculation / 計算結果に置き換える

    // --- Pitch axis ---
    // ピッチ軸
    // TODO: Same as roll but for pitch axis
    // ヒント: ws::rc_pitch(), ws::gyro_y()

    float pitch_output = 0.0f;  // Replace with your calculation / 計算結果に置き換える

    // --- Yaw axis ---
    // ヨー軸
    // TODO: Same as roll but for yaw axis
    // ヒント: ws::rc_yaw(), ws::gyro_z()

    float yaw_output = 0.0f;  // Replace with your calculation / 計算結果に置き換える

    // TODO: Record your rate targets for the Data Stream (Lesson 7 sysid uses this)
    // TODO: Data Stream に角速度目標を記録する（レッスン7のシステム同定で使用）
    // ヒント: ws::set_rate_target(roll_target, pitch_target, yaw_target);

    // --- Apply to motors ---
    // モーターに適用
    ws::motor_mixer(throttle, roll_output, pitch_output, yaw_output);
}
