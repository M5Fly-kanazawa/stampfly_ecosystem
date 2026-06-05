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
#include "data_types.hpp"   // sf::TofData
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

    /// Update detection logic with a fresh ToF sample (call when ToF data arrives).
    /// The owning task reads the sensor_tof topic ONCE and injects the sample here,
    /// so this manager does not compete with the estimator for the same queue.
    ///
    /// `armed` gates ground detection: while disarmed the craft is definitively on
    /// the ground (on the ground the ToF sits below its minimum range and returns
    /// invalid, so it cannot by itself confirm ground contact). This mirrors the
    /// proven firmware/vehicle landing handler ("landed whenever disarmed") and lets
    /// the manager re-anchor after landing and arm the next flight cleanly.
    /// 新しい ToF サンプルで検出ロジックを更新する（ToF データ到着時に呼ぶ）。
    /// 所有タスクが sensor_tof を1回だけ読んで注入するため推定器とキューを奪い合わない。
    /// `armed` で接地判定を制御: disarmed 中は確実に接地（接地中 ToF は最小レンジ未満で
    /// 無効を返し単独で接地を確認できない）。実証済みの firmware/vehicle（disarmed なら
    /// landed）と同じで、着地後の再錨付けと次飛行のクリーンな開始を可能にする。
    void update(const TofData& tof, bool armed);

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
    void evaluateToF(const TofData& tof);

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
