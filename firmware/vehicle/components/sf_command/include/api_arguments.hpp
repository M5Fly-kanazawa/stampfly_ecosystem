/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file api_arguments.hpp
 * @brief Range checks for the numbers in a Tello-style API command line (pure functions)
 *        Tello 風 API のコマンド行に含まれる数値の範囲検査（純粋関数）
 *
 * The text after a verb is parsed with sscanf("%f"), which accepts "nan", "inf" and values
 * such as 1e39 (= inf in float). A non-finite number that reaches the guidance target or a
 * wait time corrupts it for good (inf * 0 = NaN), so ApiTask checks every parsed number with
 * these functions at the parsing boundary, before any command uses it. Pure functions of the
 * numbers only (no ESP-IDF, no FreeRTOS), checked on the host in test/test_api_arguments.cpp.
 * verb の後ろの文字列は sscanf("%f") で解析され、"nan"・"inf"・1e39（float では inf）も
 * 受け付ける。有限でない数が誘導目標や待ち時間に届くと、それは元に戻らない（inf * 0 = NaN）。
 * そこで ApiTask は、解析した全ての数を、コマンドが使う前の解析の境界で、これらの関数で検査する。
 * 数だけの純関数（ESP-IDF・FreeRTOS 不要）で、test/test_api_arguments.cpp でホスト検査する。
 *
 * @design requirements.md §7 — コマンド受信 (RC入力、API) / TelloAPI   [OK]
 * @design detailed_design.md §3 注13 — API の引数の範囲                  [OK]
 */

#pragma once

#include <cmath>
#include <initializer_list>

namespace sf {
namespace api_arguments {

/// Tello `cw` / `ccw` angle range [deg] (Tello SDK: 1-360).
/// Tello の `cw` / `ccw` の角度の範囲 [deg]（Tello SDK: 1〜360）。
constexpr float kRotateMinDegrees = 1.0f;
constexpr float kRotateMaxDegrees = 360.0f;

/// Largest `sysid` excitation amplitude [deg/s] and duration [s] that the controller takes
/// unchanged. They equal PidController::kExciteAmpMax (1.5 rad/s = 85.9 deg/s) and
/// PidController::kExciteDurMax (10 s), beyond which the controller clamps silently; the API
/// refuses what the controller would cut, so the wait ApiTask makes matches the excitation.
/// 制御器が変更せずに受け取る `sysid` 励振の振幅 [deg/s] と継続時間 [s] の最大値。
/// PidController::kExciteAmpMax（1.5 rad/s = 85.9 deg/s）・PidController::kExciteDurMax（10 s）
/// と同じ値で、これを超えると制御器は黙って切り詰める。API は制御器が切り詰める値を拒否し、
/// ApiTask の待ち時間が励振の長さと一致するようにする。
constexpr float kSysidAmplitudeMaxDegPerSec = 85.9f;
constexpr float kSysidDurationMaxSeconds    = 10.0f;

/// True when every number is finite (neither NaN nor +-inf).
/// 全ての数が有限（NaN でも +-inf でもない）なら true。
inline bool allFinite(std::initializer_list<float> values)
{
    for (const float value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

/// True when a `cw` / `ccw` angle [deg] is finite and in 1-360.
/// `cw` / `ccw` の角度 [deg] が有限で 1〜360 なら true。
inline bool isRotateAngleInRange(float angle_degrees)
{
    return std::isfinite(angle_degrees) &&
           angle_degrees >= kRotateMinDegrees && angle_degrees <= kRotateMaxDegrees;
}

/// True when a `sysid` amplitude [deg/s] and duration [s] are finite, positive and within the
/// controller's limits.
/// `sysid` の振幅 [deg/s] と継続時間 [s] が有限・正で、制御器の上限以内なら true。
inline bool isSysidRangeValid(float amplitude_deg_per_sec, float duration_seconds)
{
    return std::isfinite(amplitude_deg_per_sec) && std::isfinite(duration_seconds) &&
           amplitude_deg_per_sec > 0.0f && amplitude_deg_per_sec <= kSysidAmplitudeMaxDegPerSec &&
           duration_seconds > 0.0f && duration_seconds <= kSysidDurationMaxSeconds;
}

}  // namespace api_arguments
}  // namespace sf
