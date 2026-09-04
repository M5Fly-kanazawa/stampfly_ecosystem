#include "workshop_api.hpp"

// =========================================================================
// Lesson 8: PID Control - Solution
// レッスン 8: PID制御 - 解答
// =========================================================================

static uint32_t tick = 0;

// --- PID gains ---
// PID ゲイン
static const float Kp_roll  = 0.25f;  // Model-based (zeta=0.7, K=102)
static const float Ki_roll  = 0.3f;
static const float Kd_roll  = 0.005f;

static const float Kp_pitch = 0.36f;  // Model-based (zeta=0.7, K=70)
static const float Ki_pitch = 0.3f;
static const float Kd_pitch = 0.005f;

static const float Kp_yaw   = 2.0f;
static const float Ki_yaw   = 0.5f;
static const float Kd_yaw   = 0.01f;

// --- Anti-windup limit ---
// アンチワインドアップ制限
static const float integral_limit = 0.5f;

// --- Output limit ---
// 出力リミット
static const float output_limit = 1.0f;

// --- Maximum rate [rad/s] ---
// 最大角速度 [rad/s]
static const float rate_max_rp  = 1.0f;
static const float rate_max_yaw = 5.0f;

// --- State variables ---
// 状態変数
static float roll_integral   = 0.0f;
static float roll_prev_error = 0.0f;

static float pitch_integral   = 0.0f;
static float pitch_prev_error = 0.0f;

static float yaw_integral   = 0.0f;
static float yaw_prev_error = 0.0f;

// Helper: clamp a value to [-limit, +limit]
// ヘルパー: 値を [-limit, +limit] にクランプ
static float clamp(float value, float limit)
{
    if (value >  limit) return  limit;
    if (value < -limit) return -limit;
    return value;
}

// @@snippet: setup
void setup()
{
    ws::print("Lesson 8: PID Control - Solution");

    // Set WiFi channel (use 1, 6, or 11 to avoid interference)
    // WiFiチャンネルを設定（混信を避けるため1, 6, 11のいずれかを使用）
    ws::set_channel(1);
}
// @@end-snippet: setup

// @@snippet: loop
void loop_400Hz(float dt)
{
    tick++;

    if (!ws::is_armed()) {
        ws::motor_stop_all();
        ws::led_color(0, 0, 50);

        // Reset all state when disarmed
        // ディスアーム時に全状態をリセット
        roll_integral   = 0.0f;
        roll_prev_error = 0.0f;
        pitch_integral   = 0.0f;
        pitch_prev_error = 0.0f;
        yaw_integral   = 0.0f;
        yaw_prev_error = 0.0f;
        return;
    }

    ws::led_color(0, 50, 0);

    float throttle = ws::rc_throttle();

    // =====================================================================
    // Roll axis PID
    // ロール軸 PID
    // =====================================================================
    float roll_target = ws::rc_roll() * rate_max_rp;
    float roll_actual = ws::gyro_x();
    float roll_error  = roll_target - roll_actual;

    // P term
    float roll_P = Kp_roll * roll_error;

    // I term with anti-windup
    // I項（アンチワインドアップ付き）
    roll_integral += roll_error * dt;
    roll_integral = clamp(roll_integral, integral_limit);
    float roll_I = Ki_roll * roll_integral;

    // D term (derivative of error)
    // D項（誤差の微分）
    float roll_D = Kd_roll * (roll_error - roll_prev_error) / dt;
    roll_prev_error = roll_error;

    float roll_output = clamp(roll_P + roll_I + roll_D, output_limit);

    // =====================================================================
    // Pitch axis PID
    // ピッチ軸 PID
    // =====================================================================
    float pitch_target = ws::rc_pitch() * rate_max_rp;
    float pitch_actual = ws::gyro_y();
    float pitch_error  = pitch_target - pitch_actual;

    float pitch_P = Kp_pitch * pitch_error;

    pitch_integral += pitch_error * dt;
    pitch_integral = clamp(pitch_integral, integral_limit);
    float pitch_I = Ki_pitch * pitch_integral;

    float pitch_D = Kd_pitch * (pitch_error - pitch_prev_error) / dt;
    pitch_prev_error = pitch_error;

    float pitch_output = clamp(pitch_P + pitch_I + pitch_D, output_limit);

    // =====================================================================
    // Yaw axis PID
    // ヨー軸 PID
    // =====================================================================
    float yaw_target = ws::rc_yaw() * rate_max_yaw;
    float yaw_actual = ws::gyro_z();
    float yaw_error  = yaw_target - yaw_actual;

    float yaw_P = Kp_yaw * yaw_error;

    yaw_integral += yaw_error * dt;
    yaw_integral = clamp(yaw_integral, integral_limit);
    float yaw_I = Ki_yaw * yaw_integral;

    float yaw_D = Kd_yaw * (yaw_error - yaw_prev_error) / dt;
    yaw_prev_error = yaw_error;

    float yaw_output = clamp(yaw_P + yaw_I + yaw_D, output_limit);

    // =====================================================================
    // Record targets for the Data Stream (Lesson 7 sysid uses this)
    // Data Stream に目標値を記録（レッスン7のシステム同定で使用）
    // =====================================================================
    ws::set_rate_target(roll_target, pitch_target, yaw_target);

    // =====================================================================
    // Apply to motors
    // モーターに適用
    // =====================================================================
    ws::motor_mixer(throttle, roll_output, pitch_output, yaw_output);

    // --- Debug print (2Hz) ---
    if (tick % 200 == 0) {
        ws::print("R:P%.2f I%.2f D%.3f  P:P%.2f I%.2f  Y:P%.2f I%.2f",
                  roll_P, roll_I, roll_D,
                  pitch_P, pitch_I,
                  yaw_P, yaw_I);
    }
}
// @@end-snippet: loop
