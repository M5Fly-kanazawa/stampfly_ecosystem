/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file takeoff_landing.hpp
 * @brief Takeoff/landing manager — ground/airborne detection, sequences
 *        離着陸マネージャー — 地上/空中判定、シーケンス管理
 *
 * Uses ToF sensor data and state estimation to detect ground contact,
 * manage takeoff/landing sequences, and trigger state transitions.
 *
 * ToFセンサデータと状態推定を使用して地面接地を検出し、
 * 離陸/着陸シーケンスを管理し、状態遷移をトリガーする。
 *
 * @design architecture.md §4 — State machine transitions               [--]
 * @design detailed_design.md §11 — Takeoff/landing logic               [--]
 * @design coding_and_education.md §2 — Bilingual comments               [--]
 */

#pragma once

#include "flight_state.hpp"
#include <cstdint>

namespace sf {

/// Takeoff/landing configuration
/// 離着陸設定
struct TakeoffLandingConfig {
    float ground_tof_m       = 0.05f;   // Ground threshold [m]    / 地上閾値
    float airborne_tof_m     = 0.15f;   // Airborne threshold [m]  / 空中閾値
    float landing_vel_mps    = 0.05f;   // Landing velocity [m/s]  / 着陸速度
    uint32_t takeoff_hold_ms = 500;     // Takeoff confirm [ms]    / 離陸確認時間
    uint32_t landing_hold_ms = 1000;    // Landing confirm [ms]    / 着陸確認時間
};

/// Takeoff/landing manager
/// 離着陸マネージャー
class TakeoffLandingMgr {
public:
    /// Initialize with default configuration
    /// デフォルト設定で初期化する
    void init();

    /// Initialize with custom configuration
    /// カスタム設定で初期化する
    void init(const TakeoffLandingConfig& config);

    /// Update detection logic (call at control rate)
    /// 検出ロジックを更新する（制御レートで呼ぶ）
    void update();

    /// Check if vehicle is on the ground
    /// 機体が地上にあるか確認する
    bool isOnGround() const { return on_ground_; }

    /// Check if takeoff is detected
    /// 離陸を検出したか確認する
    bool isTakeoffDetected() const { return takeoff_detected_; }

    /// Check if landing is detected
    /// 着陸を検出したか確認する
    bool isLandingDetected() const { return landing_detected_; }

private:
    /// Evaluate ToF distance for ground contact
    /// ToF距離で地面接地を評価する
    void evaluateToF();

    /// Detect takeoff: sustained altitude above threshold
    /// 離陸検出: 閾値以上の高度を持続
    void detectTakeoff();

    /// Detect landing: low altitude + low velocity sustained
    /// 着陸検出: 低高度＋低速度の持続
    void detectLanding();

    TakeoffLandingConfig config_;
    bool     on_ground_        = true;   // Ground contact flag  / 地面接地フラグ
    bool     takeoff_detected_ = false;  // Takeoff event flag   / 離陸イベントフラグ
    bool     landing_detected_ = false;  // Landing event flag   / 着陸イベントフラグ
    uint32_t takeoff_start_ms_ = 0;      // Takeoff timer start  / 離陸タイマー開始
    uint32_t landing_start_ms_ = 0;      // Landing timer start  / 着陸タイマー開始
};

}  // namespace sf
