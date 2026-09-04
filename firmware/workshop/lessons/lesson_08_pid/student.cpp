#include "workshop_api.hpp"

// =========================================================================
// Lesson 8: PID Control
// レッスン 8: PID制御
// =========================================================================
//
// Goal: Add Integral (I) and Derivative (D) terms to eliminate
//       steady-state error and reduce overshoot.
// 目標: 積分(I)項と微分(D)項を追加し、定常偏差を除去し
//       オーバーシュートを低減する
//
// PID block diagram:
//                             +-->[Kp * e]----------+
//                             |                      |
//   target -->(+)--> error ---+-->[Ki * integral(e)]-+--> output --> mixer
//               ^(-)          |                      |
//               |             +-->[Kd * de/dt]-------+
//          gyro (actual)

// --- PID gains ---
// PID ゲイン
static float Kp_roll  = 0.25f;  // From L6 model (zeta=0.7, K=102)
static float Ki_roll  = 0.0f;   // TODO: add integral gain (try 0.3)
static float Kd_roll  = 0.0f;   // TODO: add derivative gain (try 0.005)

static float Kp_pitch = 0.36f;  // From L6 model (zeta=0.7, K=70)
static float Ki_pitch = 0.0f;   // TODO: add integral gain
static float Kd_pitch = 0.0f;   // TODO: add derivative gain

static float Kp_yaw   = 2.0f;   // TODO: tune
static float Ki_yaw   = 0.0f;   // TODO: add integral gain (try 0.5)
static float Kd_yaw   = 0.0f;   // TODO: add derivative gain (try 0.01)

// --- State variables ---
// 状態変数
// TODO: Declare variables for integral accumulator and previous error
//       per axis (roll, pitch, yaw)
// ヒント:
// static float roll_integral  = 0.0f;
// static float roll_prev_error = 0.0f;

// Maximum rate [rad/s]
// 最大角速度 [rad/s]
static float rate_max_rp  = 1.0f;
static float rate_max_yaw = 5.0f;

void setup()
{
    ws::print("Lesson 8: PID Control");

    // TODO: Set your WiFi channel (1, 6, or 11)
    // TODO: 自分のWiFiチャンネルを設定する（1, 6, 11のいずれか）
    // ws::set_channel(1);
}

void loop_400Hz(float dt)
{
    if (!ws::is_armed()) {
        ws::motor_stop_all();
        // TODO: Reset integral and prev_error when disarmed
        // ディスアーム時に積分値と前回誤差をリセット
        return;
    }

    float throttle = ws::rc_throttle();

    // --- Roll axis PID ---
    // ロール軸 PID
    float roll_target = ws::rc_roll() * rate_max_rp;
    float roll_actual = ws::gyro_x();
    float roll_error  = roll_target - roll_actual;

    // P term
    // P項
    float roll_P = Kp_roll * roll_error;

    // TODO: I term - accumulate error over time
    // I項 - 誤差を時間で積分
    // ヒント: roll_integral += roll_error * dt;
    // ヒント: Anti-windup: clamp integral to [-0.5, 0.5]
    float roll_I = 0.0f;  // Replace with Ki_roll * roll_integral

    // TODO: D term - rate of change of error
    // D項 - 誤差の変化率
    // ヒント: float roll_D = Kd_roll * (roll_error - roll_prev_error) / dt;
    // ヒント: roll_prev_error = roll_error;
    float roll_D = 0.0f;  // Replace with your calculation

    float roll_output = roll_P + roll_I + roll_D;

    // --- Pitch axis PID ---
    // ピッチ軸 PID
    // TODO: Same structure as roll
    float pitch_output = 0.0f;  // Replace with your PID calculation

    // --- Yaw axis PID ---
    // ヨー軸 PID
    // TODO: Same structure as roll
    float yaw_output = 0.0f;  // Replace with your PID calculation

    // --- Output limit ---
    // 出力リミット
    // TODO: Clamp outputs to [-1.0, 1.0]
    // ヒント: if (roll_output > 1.0f) roll_output = 1.0f;

    // TODO: Record your rate targets for the Data Stream (Lesson 7 sysid uses this)
    // TODO: Data Stream に角速度目標を記録する（レッスン7のシステム同定で使用）
    // ヒント: ws::set_rate_target(roll_target, pitch_target, yaw_target);

    // --- Apply to motors ---
    // モーターに適用
    ws::motor_mixer(throttle, roll_output, pitch_output, yaw_output);
}
