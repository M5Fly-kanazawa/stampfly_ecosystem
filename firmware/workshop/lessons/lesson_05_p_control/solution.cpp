#include "workshop_api.hpp"

// =========================================================================
// Lesson 5: Rate P-Control + First Flight - Solution
// レッスン 5: レートP制御 + 初フライト - 解答
// =========================================================================

static uint32_t tick = 0;

// @@snippet: setup
void setup()
{
    ws::print("Lesson 5: Rate P-Control - Solution");

    // Set WiFi channel (use 1, 6, or 11 to avoid interference)
    // WiFiチャンネルを設定（混信を避けるため1, 6, 11のいずれかを使用）
    ws::set_channel(1);
}
// @@end-snippet: setup

// @@snippet: loop
void loop_400Hz(float dt)
{
    tick++;

    // Safety: only run control when armed
    // 安全: ARM状態のときだけ制御を実行
    if (!ws::is_armed()) {
        ws::motor_stop_all();
        ws::led_color(0, 0, 50);  // Blue = disarmed / 青 = 非ARM
        return;
    }

    ws::led_color(0, 50, 0);  // Green = armed / 緑 = ARM

    float throttle = ws::rc_throttle();

    // --- P gains ---
    // P ゲイン
    float Kp_roll  = 0.5f;
    float Kp_pitch = 0.5f;
    float Kp_yaw   = 2.0f;

    // --- Maximum angular rate [rad/s] ---
    // 最大角速度 [rad/s]
    float rate_max_rp  = 1.0f;   // roll/pitch
    float rate_max_yaw = 5.0f;   // yaw

    // --- Roll axis ---
    // ロール軸
    float roll_target = ws::rc_roll() * rate_max_rp;
    float roll_actual = ws::gyro_x();
    float roll_error  = roll_target - roll_actual;
    float roll_output = Kp_roll * roll_error;

    // --- Pitch axis ---
    // ピッチ軸
    float pitch_target = ws::rc_pitch() * rate_max_rp;
    float pitch_actual = ws::gyro_y();
    float pitch_error  = pitch_target - pitch_actual;
    float pitch_output = Kp_pitch * pitch_error;

    // --- Yaw axis ---
    // ヨー軸
    float yaw_target = ws::rc_yaw() * rate_max_yaw;
    float yaw_actual = ws::gyro_z();
    float yaw_error  = yaw_target - yaw_actual;
    float yaw_output = Kp_yaw * yaw_error;

    // --- Record targets for the Data Stream (Lesson 7 sysid uses this) ---
    // Data Stream に目標値を記録（レッスン7のシステム同定で使用）
    ws::set_rate_target(roll_target, pitch_target, yaw_target);

    // --- Apply to motors ---
    // モーターに適用
    ws::motor_mixer(throttle, roll_output, pitch_output, yaw_output);

    // --- Debug print (2Hz) ---
    // デバッグ出力 (2Hz): 実測値/目標値
    if (tick % 200 == 0) {
        ws::print("R:%.2f/%.2f P:%.2f/%.2f Y:%.2f/%.2f T:%.2f",
                  roll_actual, roll_target,
                  pitch_actual, pitch_target,
                  yaw_actual, yaw_target,
                  throttle);
    }
}
// @@end-snippet: loop
