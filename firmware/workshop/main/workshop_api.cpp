/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (workshop firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file workshop_api.cpp
 * @brief Workshop API implementation — bridges ws:: calls to sf:: Pub-Sub
 *        topics / L1 API on the vehicle component base
 *        ワークショップAPI実装 — ws:: 呼び出しを vehicle コンポーネント基盤の
 *        sf:: Pub-Subトピック / L1 API へブリッジする
 *
 * Every function here is a thin wrapper: read/write exactly one topic (or the
 * L1 sf::api convenience accessor), no logic of its own beyond unit/clamp
 * conversion. Motor functions are the one exception — they only RECORD a
 * request into ws_internal::motor_request(); WorkshopControlTask resolves and
 * applies it once per 400Hz cycle (see ws_internal.hpp for why no lock is
 * needed).
 *
 * 本ファイルの各関数は薄いラッパー: 1つのトピック（または L1 の sf::api 便利
 * 関数）を読み書きするだけで、単位変換/クランプ以外のロジックを持たない。
 * モータ関数だけは例外 — ws_internal::motor_request() へ要求を「記録」する
 * だけで、WorkshopControlTask が 400Hz 周期に一度解決・適用する（ロック不要な
 * 理由は ws_internal.hpp 参照）。
 *
 * @design W1_SPEC.md §3 — ws:: API 実装対応表                         [OK]
 */

#include "workshop_api.hpp"
#include "ws_internal.hpp"

#include "topics.hpp"
#include "data_types.hpp"
#include "sf_api.hpp"
#include "params.hpp"
#include "sf_math.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"   // vTaskDelay (set_channel reboot grace)
#include "esp_timer.h"
#include "esp_system.h"      // esp_restart (set_channel reboot)

#include <algorithm>   // std::clamp
#include <cstdarg>
#include <cstdio>
#include <cstdint>

namespace {
/// Current time in microseconds, as the uint32_t every topic timestamp uses.
/// 現在時刻[us]。全トピックの timestamp が使う uint32_t で返す。
inline uint32_t nowUs()
{
    return static_cast<uint32_t>(esp_timer_get_time());
}
}  // namespace

// =============================================================================
// Communication Setup
// 通信設定
// =============================================================================

void ws::set_channel(int channel)
{
    if (channel != 1 && channel != 6 && channel != 11) {
        ws::print("ERROR: channel must be 1, 6, or 11");
        return;
    }

    int32_t current = 0;
    sf::params::get_int("wifi.channel", current);
    if (current == channel) {
        ws::print("WiFi channel: %d", channel);
        return;
    }

    // Same "set + save_one + reboot" shape as the CLI `param set` + `param
    // save` + reboot flow: the radio only reads wifi.channel once at boot
    // (sf_comm::comm.cpp), so a live channel change needs a restart to apply.
    // CLI の `param set` + `param save` + 再起動と同じ形: 無線は wifi.channel を
    // 起動時に一度だけ読む（sf_comm::comm.cpp）ため、変更を反映するには再起動が要る。
    sf::params::set_int("wifi.channel", channel);
    sf::params::save_one("wifi.channel");

    ws::print("WiFi channel -> %d, rebooting...", channel);
    std::fflush(stdout);
    // Give USB-CDC time to actually push the message out before the reset
    // (same 500ms grace the legacy implementation used).
    // リセット前に USB-CDC がメッセージを送出し切る猶予を与える
    // （旧実装と同じ 500ms）。
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

// =============================================================================
// Motor Control
// =============================================================================

void ws::motor_set_duty(int id, float duty)
{
    duty = std::clamp(duty, 0.0f, 1.0f);
    ws_internal::MotorRequest& req = ws_internal::motor_request();
    req.mode = ws_internal::MotorMode::Direct;
    // Motor ID 1..4 = FR, RR, RL, FL, matching sf::Actuator::applyTestDuties'
    // duties[0..3] order — see actuator.hpp motor layout diagram.
    // モータ ID 1..4 = FR, RR, RL, FL。sf::Actuator::applyTestDuties の
    // duties[0..3] 順と一致（配置図は actuator.hpp 参照）。
    if (id >= 1 && id <= 4) {
        req.duties[id - 1] = duty;
    }
}

void ws::motor_set_all(float duty)
{
    duty = std::clamp(duty, 0.0f, 1.0f);
    ws_internal::MotorRequest& req = ws_internal::motor_request();
    req.mode = ws_internal::MotorMode::Direct;
    for (float& d : req.duties) d = duty;
}

void ws::motor_stop_all()
{
    ws_internal::MotorRequest& req = ws_internal::motor_request();
    req.mode = ws_internal::MotorMode::Direct;
    for (float& d : req.duties) d = 0.0f;
}

void ws::motor_mixer(float thrust, float roll, float pitch, float yaw)
{
    ws_internal::MotorRequest& req = ws_internal::motor_request();
    req.mode   = ws_internal::MotorMode::Mixer;
    req.thrust = thrust;
    req.roll   = roll;
    req.pitch  = pitch;
    req.yaw    = yaw;
}

// =============================================================================
// Controller Input
// =============================================================================

float ws::rc_throttle() { return sf::api::command_latest().throttle; }
float ws::rc_roll()     { return sf::api::command_latest().roll; }
float ws::rc_pitch()    { return sf::api::command_latest().pitch; }
float ws::rc_yaw()      { return sf::api::command_latest().yaw; }

void ws::arm()
{
    sf::api_command.publish(sf::ApiCommand{
        static_cast<uint8_t>(sf::ApiCmd::Arm), 0, nowUs()});
}

void ws::disarm()
{
    sf::api_command.publish(sf::ApiCommand{
        static_cast<uint8_t>(sf::ApiCmd::Disarm), 0, nowUs()});
}

bool ws::is_armed() { return sf::api::is_armed(); }

// =============================================================================
// Controller Buttons / Modes
// =============================================================================

bool ws::rc_throttle_yaw_button() { return sf::pilot_request.latest().arm; }

bool ws::rc_roll_pitch_button()
{
    // The current vehicle protocol carries no FLIP flag (only ARM/ACRO/
    // ALT/POS switches — see sf::PilotRequest). No lesson reads this
    // function's return value, so a fixed false is a safe, honest stand-in
    // rather than guessing at a bit that no longer exists.
    // 現行 vehicle プロトコルには FLIP フラグが無い（ARM/ACRO/ALT/POS のみ —
    // sf::PilotRequest 参照）。この戻り値を使うレッスンは無いため、もう存在しない
    // ビットを推測するより固定 false のほうが安全で正直。
    return false;
}

bool ws::rc_stabilize_acro_mode() { return sf::pilot_request.latest().acro; }
bool ws::rc_alt_mode()            { return sf::pilot_request.latest().alt_hold; }
bool ws::rc_pos_mode()            { return sf::pilot_request.latest().pos_hold; }

// =============================================================================
// LED Control
// =============================================================================

static bool s_led_task_disabled = false;

void ws::disable_led_task()
{
    sf::ui_command.publish(sf::UiCommand{
        static_cast<uint8_t>(sf::UiCmd::LedUserOverride), 1, 0, 0, 0, nowUs()});
    s_led_task_disabled = true;
}

void ws::enable_led_task()
{
    sf::ui_command.publish(sf::UiCommand{
        static_cast<uint8_t>(sf::UiCmd::LedUserOverride), 0, 0, 0, 0, nowUs()});
    s_led_task_disabled = false;
}

bool ws::is_led_task_disabled() { return s_led_task_disabled; }

void ws::led_color(uint8_t r, uint8_t g, uint8_t b)
{
    // Publish only on change: ui_command is a Queue(4) drained at NotifyTask's
    // 30Hz, but loop_400Hz() can call this every cycle (400Hz) — without this
    // guard a student holding a fixed color would flood/overflow the queue.
    // 変化時のみ publish: ui_command は NotifyTask の 30Hz で排出される
    // Queue(4) だが、loop_400Hz() は毎周期（400Hz）呼びうる — このガードが無いと
    // 固定色を保持するだけの学生コードでもキューを溢れさせる。
    static uint8_t s_last_r = 0, s_last_g = 0, s_last_b = 0;
    static bool s_first_call = true;

    if (!s_first_call && r == s_last_r && g == s_last_g && b == s_last_b) {
        return;
    }
    s_first_call = false;
    s_last_r = r; s_last_g = g; s_last_b = b;

    sf::ui_command.publish(sf::UiCommand{
        static_cast<uint8_t>(sf::UiCmd::LedUserColor), 0, r, g, b, nowUs()});
}

// =============================================================================
// IMU Sensor
// =============================================================================

float ws::gyro_x() { return sf::api::estimate_latest().angular_rate[0]; }
float ws::gyro_y() { return sf::api::estimate_latest().angular_rate[1]; }
float ws::gyro_z() { return sf::api::estimate_latest().angular_rate[2]; }

float ws::accel_x() { return sf::api::estimate_latest().specific_force[0]; }
float ws::accel_y() { return sf::api::estimate_latest().specific_force[1]; }
float ws::accel_z() { return sf::api::estimate_latest().specific_force[2]; }

// =============================================================================
// Environmental / Distance Sensors
// =============================================================================

float ws::baro_altitude() { return sf::sensor_snapshot.latest().baro_altitude; }
float ws::baro_pressure() { return sf::sensor_snapshot.latest().baro_pressure; }

float ws::mag_x() { return sf::sensor_snapshot.latest().mag[0]; }
float ws::mag_y() { return sf::sensor_snapshot.latest().mag[1]; }
float ws::mag_z() { return sf::sensor_snapshot.latest().mag[2]; }

float ws::tof_bottom() { return sf::sensor_snapshot.latest().tof_distance; }

float ws::tof_front()
{
    // The current vehicle sensor pipeline has no front ToF (SensorSnapshot
    // carries a single bottom-facing tof_distance only). -1 matches the
    // "unavailable" convention documented in workshop_api.hpp.
    // 現行 vehicle のセンサパイプラインには前方 ToF が無い（SensorSnapshot は
    // 下向き tof_distance のみを運ぶ）。-1 は workshop_api.hpp に文書化された
    // 「利用不可」の慣例と一致する。
    return -1.0f;
}

float ws::flow_vx()
{
    // Unit change vs. the legacy workshop API: this now returns the RAW
    // optical-flow displacement count (SensorSnapshot.flow_dx), not an
    // estimated velocity in m/s. Only Lesson 10's print() reads this value,
    // so the change is documented here rather than reconstructing a velocity
    // estimate the current pipeline does not compute at this layer.
    // 旧 workshop API との単位変更: 推定速度 [m/s] ではなく、生のオプティカル
    // フロー変位カウント（SensorSnapshot.flow_dx）を返すようになった。この値を
    // 読むのは Lesson 10 の print() のみのため、この層では計算しない速度推定を
    // 再構成するのではなく、ここに変更を明記する。
    return static_cast<float>(sf::sensor_snapshot.latest().flow_dx);
}

float ws::flow_vy()
{
    return static_cast<float>(sf::sensor_snapshot.latest().flow_dy);
}

uint8_t ws::flow_quality() { return sf::sensor_snapshot.latest().flow_squal; }

// =============================================================================
// Estimation
// =============================================================================

float ws::estimated_roll()
{
    const sf::StateEstimate est = sf::api::estimate_latest();
    const sf::math::Quat q{est.attitude[0], est.attitude[1], est.attitude[2], est.attitude[3]};
    return q.to_euler().x;
}

float ws::estimated_pitch()
{
    const sf::StateEstimate est = sf::api::estimate_latest();
    const sf::math::Quat q{est.attitude[0], est.attitude[1], est.attitude[2], est.attitude[3]};
    return q.to_euler().y;
}

float ws::estimated_yaw()
{
    const sf::StateEstimate est = sf::api::estimate_latest();
    const sf::math::Quat q{est.attitude[0], est.attitude[1], est.attitude[2], est.attitude[3]};
    return q.to_euler().z;
}

float ws::estimated_altitude()
{
    // NED -> altitude (positive up)
    // NED -> 高度（上向き正）
    return -sf::api::estimate_latest().position[2];
}

// =============================================================================
// Control Target Logging
// =============================================================================

void ws::set_rate_target(float roll, float pitch, float yaw)
{
    // Latch only — WorkshopControlTask::publishLogStream() reads this once
    // per 400Hz cycle into the Data Stream's rate_ref[]. No effect on motor
    // output (see ws_internal.hpp ControlTargets doc).
    // ラッチのみ — WorkshopControlTask::publishLogStream() が 400Hz 周期に
    // 一度これを読んで Data Stream の rate_ref[] へ反映する。モータ出力には
    // 影響しない（ws_internal.hpp ControlTargets のドキュメント参照）。
    ws_internal::ControlTargets& targets = ws_internal::control_targets();
    targets.rate[0] = roll;
    targets.rate[1] = pitch;
    targets.rate[2] = yaw;
}

void ws::set_angle_target(float roll, float pitch)
{
    ws_internal::ControlTargets& targets = ws_internal::control_targets();
    targets.angle[0] = roll;
    targets.angle[1] = pitch;
}

// =============================================================================
// Utility
// =============================================================================

uint32_t ws::millis()
{
    // Divide in 64-bit FIRST, then truncate: wraps at ~49.7 days. Routing
    // through the uint32 microsecond clock (nowUs()/1000) would wrap every
    // ~71.6 minutes — inside a single workshop session.
    // 64bit のまま除算してから丸める: ラップは約49.7日。uint32 のマイクロ秒時計
    // 経由（nowUs()/1000）だと約71.6分で巻き戻り、講座1回の中で起きてしまう。
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

float ws::battery_voltage() { return sf::api::power_latest().voltage; }

void ws::print(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
}
