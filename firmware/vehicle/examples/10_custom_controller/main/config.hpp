/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file config.hpp
 * @brief Named constants for Example 10 (no magic numbers).
 *        サンプル10の名前付き定数（マジックナンバー禁止）。
 *
 * @design coding_and_education.md §2 — マジックナンバー禁止              [OK]
 */

#pragma once

#include <cstdint>

namespace config {

// =============================================================================
// The exercise hook (see learner_controller.cpp compute())
// 演習フック（learner_controller.cpp の compute() 参照）
//
// 1.0 = no change from plain PidController. Change this, rebuild, and watch
// the printed torque[2] (yaw) scale — this is the one number the exercise
// wraps around.
// 1.0 = 素の PidController と無変化。これを変えて再ビルドすると、表示される
// torque[2]（ヨー）が伸縮する — 本演習が包む唯一の数値。
// =============================================================================

inline constexpr float kYawTorqueScale = 1.0f;

// =============================================================================
// Timing
// タイミング
//
// Every compute() call in this bench uses the SAME per-call dt as the real
// ControlTask (main/config.hpp::IMU_DT, 400 Hz) — but we only call it once
// per console line instead of 400 times per second, so the response you see
// unfolds in "slow motion": kPrintPeriodMilliseconds of wall-clock time
// advances the controller's internal state by only
// kControlComputePeriodSeconds of simulated flight time. There is no plant
// model here (this bench doesn't simulate the drone's physics), so this
// slow-motion mismatch is a deliberate simplification, not a bug.
// 本ベンチの全 compute() 呼び出しは、実 ControlTask と同じ1回あたりの dt
// （main/config.hpp::IMU_DT、400Hz）を使う — しかし毎秒400回でなく画面1行につき
// 1回しか呼ばないため、見える応答は「スローモーション」で進む:
// kPrintPeriodMilliseconds の実時間が進んでも、制御器の内部状態は
// kControlComputePeriodSeconds 分の模擬飛行時間しか進まない。本ベンチには
// プラントモデルが無い（ドローンの物理をシミュレートしない）ため、このスロー
// モーションのずれは意図的な単純化であり、バグではない。
// =============================================================================

inline constexpr float kControlComputePeriodSeconds = 0.0025f;    // matches IMU_DT (400 Hz)
inline constexpr uint32_t kPrintPeriodMilliseconds = 100;         // 10 Hz console line

// =============================================================================
// Synthetic disturbance (this bench's stand-in for "the drone got tilted")
// 合成外乱（本ベンチにおける「機体が傾いた」の代替）
// =============================================================================

inline constexpr float kSyntheticPitchAmplitudeRadians = 0.15f;   // ~8.6 deg
inline constexpr uint32_t kSyntheticPitchPeriodSteps = 40;        // 4 s of print cadence

// =============================================================================
// Unit conversion (console display only)
// 単位変換（コンソール表示専用）
// =============================================================================

/// Same value as components/sf_failsafe/failsafe.cpp's kRadToDeg (180/pi).
/// components/sf_failsafe/failsafe.cpp の kRadToDeg (180/pi) と同じ値。
inline constexpr float kRadToDeg = 57.2957795f;

}  // namespace config
