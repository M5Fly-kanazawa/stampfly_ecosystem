#include "workshop_api.hpp"

// =========================================================================
// Lesson 7: System Identification - Solution
// レッスン 7: システム同定 - 解答
// =========================================================================
//
// Fly with P control (Kp=0.5), capture telemetry via WiFi,
// then run: sf sysid fit flight.csv --kp 0.5 --plot
//
// P 制御（Kp=0.5）で飛行し、WiFi テレメトリを取得後:
//   sf sysid fit flight.csv --kp 0.5 --plot
//
// ws::set_rate_target() below records the rate target into the Data Stream
// (rate_ref_*) so `sf sysid fit` can reconstruct the plant input without
// re-deriving it from the raw stick value.
// 下の ws::set_rate_target() が角速度目標を Data Stream（rate_ref_*）に記録
// する。`sf sysid fit` はこれを読むだけでプラント入力を復元でき、生の
// スティック値から再導出する必要がない。

static uint32_t tick = 0;

// P gain (remember for sf sysid fit --kp 0.5)
// P ゲイン（sf sysid fit --kp 0.5 に渡す値）
static float Kp = 0.5f;
static float Kp_yaw = 2.0f;

// Rate limits (sf sysid fit --rate-max uses default 1.0 for roll/pitch)
// レート制限（sf sysid fit --rate-max のデフォルト 1.0 が roll/pitch 用）
static const float rate_max_rp  = 1.0f;    // [rad/s]
static const float rate_max_yaw = 5.0f;    // [rad/s]

static float clamp(float val, float lim)
{
    if (val >  lim) return  lim;
    if (val < -lim) return -lim;
    return val;
}

// @@snippet: setup
void setup()
{
    ws::print("Lesson 7: System Identification - Solution");

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
        return;
    }

    ws::led_color(0, 50, 0);  // Green = flying
    float throttle = ws::rc_throttle();

    // =====================================================================
    // P control (same as L5)
    // P 制御（L5 と同じ）
    // =====================================================================
    float roll_target  = ws::rc_roll()  * rate_max_rp;
    float pitch_target = ws::rc_pitch() * rate_max_rp;
    float yaw_target   = ws::rc_yaw()   * rate_max_yaw;

    // Record targets for the Data Stream: `sf sysid fit` reads rate_ref_*
    // (this call) + gyro_* to reconstruct u = Kp * (rate_ref - gyro).
    // Data Stream に目標値を記録: `sf sysid fit` は rate_ref_*（この呼び出し）
    // + gyro_* を読み、u = Kp * (rate_ref - gyro) でプラント入力を復元する。
    ws::set_rate_target(roll_target, pitch_target, yaw_target);

    float roll_cmd  = Kp     * (roll_target  - ws::gyro_x());
    float pitch_cmd = Kp     * (pitch_target - ws::gyro_y());
    float yaw_cmd   = Kp_yaw * (yaw_target   - ws::gyro_z());

    ws::motor_mixer(throttle,
                    clamp(roll_cmd,  1.0f),
                    clamp(pitch_cmd, 1.0f),
                    clamp(yaw_cmd,   1.0f));

    // Debug print (2Hz)
    // デバッグ出力 (2Hz)
    if (tick % 200 == 0) {
        ws::print("Kp=%.2f rate_max=%.1f gyro_x=%.3f",
                  Kp, rate_max_rp, ws::gyro_x());
    }
}
// @@end-snippet: loop
