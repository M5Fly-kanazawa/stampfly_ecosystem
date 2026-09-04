/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (workshop firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file ws_internal.hpp
 * @brief Task-local motor-request and control-target state shared by
 *        workshop_api.cpp and workshop_control_task.cpp
 *        workshop_api.cpp と workshop_control_task.cpp が共有するタスク内
 *        モータ要求・制御目標状態
 *
 * The learner's ws::motor_* functions (workshop_api.cpp) only RECORD a
 * request; WorkshopControlTask (workshop_control_task.cpp) RESOLVES it into
 * duties and applies them via the actuator, once per 400Hz cycle.
 * ws::set_rate_target()/set_angle_target() follow the same latch pattern but
 * feed the Data Stream logger only (ControlTargets below), not the motors.
 *
 * IMPORTANT — no locking: every ws:: motor/target function is called from
 * inside loop_400Hz(), which WorkshopControlTask calls directly from ITS OWN
 * task context. motor_request()/control_targets() therefore are only ever
 * touched from the WorkshopControlTask task — there is no cross-task access
 * and no lock is needed. Do not call these ws:: functions from any other task.
 *
 * 学習者の ws::motor_* 関数（workshop_api.cpp）は要求を「記録」するだけ。
 * WorkshopControlTask（workshop_control_task.cpp）が 400Hz 周期に一度、それを
 * duty へ「解決」してアクチュエータに適用する。ws::set_rate_target()/
 * set_angle_target() も同じラッチ方式だが、モータではなく Data Stream ロガー
 * （下記 ControlTargets）だけに供給する。
 *
 * 重要 — ロック不要: 全ての ws:: モータ/目標関数は loop_400Hz() 内から呼ばれ、
 * loop_400Hz() は WorkshopControlTask が「自分自身のタスク文脈」から直接
 * 呼ぶ。よって motor_request()/control_targets() は WorkshopControlTask
 * タスクからしか触られず、タスク跨ぎのアクセスは発生しないためロックは不要。
 * これらの ws:: 関数を他のタスクから呼ばないこと。
 *
 * @design W1_SPEC.md §2-6 — ws_internal.hpp                           [OK]
 * @design workshop_migration.md §8 — rate_ref/angle_ref 解消策         [OK]
 */

#pragma once

#include <algorithm>  // std::clamp
#include <cstdint>

namespace ws_internal {

/// Motor request source: Direct (ws::motor_set_duty/all/stop_all — raw duty
/// per motor) or Mixer (ws::motor_mixer — thrust/roll/pitch/yaw allocation).
/// モータ要求の種別: Direct（ws::motor_set_duty/all/stop_all — モータ毎の生 duty）
/// または Mixer（ws::motor_mixer — 推力/ロール/ピッチ/ヨー配分）。
enum class MotorMode : uint8_t { Direct, Mixer };

/// Latched motor request, written by workshop_api.cpp, read once per cycle
/// by workshop_control_task.cpp.
/// ラッチされたモータ要求。workshop_api.cpp が書き、workshop_control_task.cpp が
/// 周期に一度読む。
struct MotorRequest {
    MotorMode mode = MotorMode::Direct;
    float duties[4] = {0, 0, 0, 0};                  // FR, RR, RL, FL (Direct)
    float thrust = 0, roll = 0, pitch = 0, yaw = 0;   // Mixer inputs / Mixer 入力
};

/// Reference to the single MotorRequest instance. `inline` + a function-local
/// static gives ONE instance across every translation unit that includes this
/// header (C++17 inline-function linkage merging) — no separate .cpp needed
/// and no risk of workshop_api.cpp / workshop_control_task.cpp each getting
/// their own copy.
/// 単一の MotorRequest インスタンスへの参照。`inline` + 関数内 static により、
/// 本ヘッダを include する全翻訳単位で「1個」のインスタンスになる（C++17 の
/// inline 関数リンケージ統合）— 別途 .cpp は不要で、workshop_api.cpp /
/// workshop_control_task.cpp がそれぞれ別コピーを持つ心配もない。
inline MotorRequest& motor_request()
{
    static MotorRequest request;
    return request;
}

/// Latched control targets, written by ws::set_rate_target()/set_angle_target()
/// (workshop_api.cpp) and read once per cycle by WorkshopControlTask's
/// publishLogStream() to fill the Data Stream's rate_ref[]/angle_ref[] fields.
/// LOGGING ONLY — unlike MotorRequest, nothing here feeds back into motor
/// output; it exists purely so `sf sysid fit` (Lesson 7) can reconstruct the
/// plant input u = Kp * (rate_ref - gyro) from a recorded target instead of
/// re-deriving it from the raw stick value.
/// ラッチされた制御目標。ws::set_rate_target()/set_angle_target()
/// （workshop_api.cpp）が書き、WorkshopControlTask の publishLogStream() が
/// 周期に一度読んで Data Stream の rate_ref[]/angle_ref[] へ反映する。
/// ロギング専用 — MotorRequest と異なり、ここの値はモータ出力に一切フィード
/// バックしない。`sf sysid fit`（レッスン7）が記録済み目標から
/// u = Kp * (rate_ref - gyro) でプラント入力を復元できるようにするためだけに
/// 存在する。
struct ControlTargets {
    float rate[3]  = {0, 0, 0};  // [rad/s] Body FRD R,P,Y (set_rate_target)
    float angle[2] = {0, 0};     // [rad] Body FRD R,P     (set_angle_target)
};

/// Reference to the single ControlTargets instance — same inline-function
/// linkage-merging trick as motor_request() above, and the same "no lock
/// needed" contract (written only from loop_400Hz(), read only from
/// WorkshopControlTask's own cycle).
/// ControlTargets の単一インスタンスへの参照 — motor_request() と同じ
/// inline 関数リンケージ統合のトリック、同じ「ロック不要」契約（loop_400Hz()
/// からのみ書かれ、WorkshopControlTask 自身の周期からのみ読まれる）。
inline ControlTargets& control_targets()
{
    static ControlTargets targets;
    return targets;
}

/// Resolve the current motor request into 4 clamped [0,1] duties (FR, RR, RL,
/// FL). Direct mode passes the recorded duties through unchanged (already
/// clamped at write time in ws::motor_set_duty/all). Mixer mode applies the
/// LEGACY voltage-scale X-quad mixer — reproduced digit-for-digit from
/// vehicle_old motor_driver.cpp:144-165 (setMixerOutput) so learner gains
/// tuned against the old firmware still behave the same way here.
/// 現在のモータ要求を [0,1] にクランプ済みの 4 duty（FR, RR, RL, FL）へ解決する。
/// Direct モードは記録済み duty をそのまま通す（ws::motor_set_duty/all の書き込み
/// 時に既にクランプ済み）。Mixer モードは「旧」電圧スケール X-quad ミキサーを
/// 適用する — vehicle_old motor_driver.cpp:144-165（setMixerOutput）と
/// 数値まで完全一致するよう再現し、旧ファーム向けに調整した学習者ゲインが
/// ここでも同じ挙動になるようにする。
inline void resolve_motor_request(float out[4])
{
    const MotorRequest& req = motor_request();

    if (req.mode == MotorMode::Direct) {
        for (int i = 0; i < 4; ++i) out[i] = req.duties[i];
        return;
    }

    // Legacy voltage-scale X-quad mixer (identical to vehicle_old
    // setMixerOutput). 3.7f and 0.25f are the legacy tuning constants, kept
    // as literals here to match the original bit-for-bit (they are NOT
    // physical quantities — see vehicle_old for the historical derivation).
    // 旧電圧スケール X-quad ミキサー（vehicle_old setMixerOutput と同一）。
    // 3.7f と 0.25f は旧チューニング定数で、原典と数値まで一致させるため
    // リテラルのまま残す（物理量ではない — 由来は vehicle_old 参照）。
    const float T = req.thrust;
    const float R = req.roll;
    const float P = req.pitch;
    const float Y = req.yaw;
    out[0] = T + 0.25f * (-R + P + Y) / 3.7f;   // M1 FR (CCW)
    out[1] = T + 0.25f * (-R - P - Y) / 3.7f;   // M2 RR (CW)
    out[2] = T + 0.25f * ( R - P + Y) / 3.7f;   // M3 RL (CCW)
    out[3] = T + 0.25f * ( R + P - Y) / 3.7f;   // M4 FL (CW)

    for (int i = 0; i < 4; ++i) out[i] = std::clamp(out[i], 0.0f, 1.0f);
}

}  // namespace ws_internal
