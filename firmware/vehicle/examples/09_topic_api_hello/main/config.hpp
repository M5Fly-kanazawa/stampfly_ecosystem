/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file config.hpp
 * @brief Named constants for Example 09 (no magic numbers).
 *        サンプル09の名前付き定数（マジックナンバー禁止）。
 *
 * @design coding_and_education.md §2 — マジックナンバー禁止              [OK]
 */

#pragma once

#include <cstdint>

namespace config {

// =============================================================================
// Sensor feed rate
// センサ供給レート
//
// The real vehicle firmware's ImuTask (tasks/imu_task.cpp) predicts at 400 Hz
// (main/config.hpp::IMU_DT). This example intentionally runs slower: a
// hand-held teaching demo does not need flight-grade bandwidth, and 100 Hz is
// comfortably fast for a person tilting the board to see the printed angles
// respond.
// 実ファームの ImuTask（tasks/imu_task.cpp）は400Hzで予測する
// （main/config.hpp::IMU_DT）。本サンプルは意図的にそれより遅く動かす —
// 手持ちの教材デモにフライト級の帯域は不要で、100Hzなら人が機体を傾けたときの
// 応答を見るのに十分速い。
// =============================================================================

inline constexpr uint32_t kSensorFeedPeriodMilliseconds = 10;      // 100 Hz
inline constexpr float kSensorFeedPeriodSeconds = 0.010f;          // same, as seconds

// =============================================================================
// Console print rate (task spec: 10 Hz roll/pitch/yaw readout)
// コンソール表示レート（仕様: 10Hz の roll/pitch/yaw 表示）
// =============================================================================

inline constexpr uint32_t kAttitudePrintPeriodMilliseconds = 100;  // 10 Hz

/// How many sensor-feed cycles make up one print period.
/// 表示1回あたりのセンサ供給サイクル数。
inline constexpr uint32_t kSensorFeedCyclesPerPrint =
    kAttitudePrintPeriodMilliseconds / kSensorFeedPeriodMilliseconds;

// =============================================================================
// Unit conversion (console display only — every Topic value stays in SI
// radians, matching the rest of vehicle)
// 単位変換（コンソール表示専用 — Topic の値は常に SI のラジアン、vehicle 本体と同じ）
// =============================================================================

/// Radians to degrees, same value as components/sf_failsafe/failsafe.cpp's
/// kRadToDeg (180/pi) — kept as a literal, not computed from M_PI, so both
/// call sites are visibly the same constant.
/// ラジアン→度。components/sf_failsafe/failsafe.cpp の kRadToDeg (180/pi) と同じ値 —
/// M_PI から計算せずリテラルで持ち、両呼び出し箇所が同じ定数だと一目で分かるようにする。
inline constexpr float kRadToDeg = 57.2957795f;

}  // namespace config
