#include "workshop_api.hpp"

// =========================================================================
// Lesson 7: System Identification
// レッスン 7: システム同定
// =========================================================================
//
// Goal: Fly with P control and capture telemetry data.
//       Use sf sysid fit to identify plant parameters K, tau_m.
// 目標: P 制御で飛行しテレメトリデータを取得する。
//       sf sysid fit でプラントパラメータ K, τm を同定する。
//
// Plant model: G_p(s) = K / (s * (tau_m * s + 1))
//   K    = plant gain [rad/s^2 per duty]
//   tau_m = motor time constant [s]
//
// The sf sysid fit tool reconstructs plant I/O from this data:
//   u_plant = Kp * (rate_ref - gyro)   <- plant input
//   y_plant = gyro                     <- plant output
// rate_ref comes from the Data Stream, recorded by your ws::set_rate_target()
// call below -- so you need to remember only the Kp value used here (not
// rate_max: the target is logged as an absolute rad/s value already).
// rate_ref は Data Stream から得る（下の ws::set_rate_target() 呼び出しが
// 記録する）— そのため覚えておくのは Kp の値だけでよい（rate_max は不要:
// 目標値は既に絶対値 [rad/s] としてログされる）。

static uint32_t tick = 0;

// --- P gain ---
// TODO: Set the Kp you want to use (same as L5, or your own value)
// TODO: 使用する Kp を設定する（L5 と同じ値、または自分の値）
// IMPORTANT: Remember this value for sf sysid fit --kp <value>
// 重要: この値を sf sysid fit --kp に渡すので覚えておくこと
static float Kp = 0.5f;        // TODO: Set your P gain
static float Kp_yaw = 2.0f;

// Rate limits (also needed for sf sysid fit --rate-max)
// レート制限（sf sysid fit --rate-max にも必要）
static const float rate_max_rp  = 1.0f;    // [rad/s] roll/pitch
static const float rate_max_yaw = 5.0f;    // [rad/s] yaw

static float clamp(float val, float lim)
{
    if (val >  lim) return  lim;
    if (val < -lim) return -lim;
    return val;
}

void setup()
{
    ws::print("Lesson 7: System Identification");

    // TODO: Set your WiFi channel (1, 6, or 11)
    // TODO: 自分のWiFiチャンネルを設定する（1, 6, 11のいずれか）
    // ws::set_channel(1);
}

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
    // P control (same structure as L5)
    // P 制御（L5 と同じ構造）
    // =====================================================================
    float roll_target  = ws::rc_roll()  * rate_max_rp;
    float pitch_target = ws::rc_pitch() * rate_max_rp;
    float yaw_target   = ws::rc_yaw()   * rate_max_yaw;

    // TODO: Record your rate targets for the Data Stream (sf sysid fit reads this)
    // TODO: Data Stream に角速度目標を記録する（sf sysid fit がこれを読む）
    // ヒント: ws::set_rate_target(roll_target, pitch_target, yaw_target);

    float roll_cmd  = Kp     * (roll_target  - ws::gyro_x());
    float pitch_cmd = Kp     * (pitch_target - ws::gyro_y());
    float yaw_cmd   = Kp_yaw * (yaw_target   - ws::gyro_z());

    ws::motor_mixer(throttle,
                    clamp(roll_cmd,  1.0f),
                    clamp(pitch_cmd, 1.0f),
                    clamp(yaw_cmd,   1.0f));

    // =====================================================================
    // Debug print (2Hz)
    // デバッグ出力 (2Hz)
    // =====================================================================
    if (tick % 200 == 0) {
        ws::print("Kp=%.2f rate_max=%.1f gyro_x=%.3f",
                  Kp, rate_max_rp, ws::gyro_x());
    }
}
