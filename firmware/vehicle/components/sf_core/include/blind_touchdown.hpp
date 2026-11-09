/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file blind_touchdown.hpp
 * @brief Touchdown detector for the descent while the downward ToF is silent: impact, then stillness
 *        下向き ToF の出力停止中の降下の接地検出: 衝撃のあとの静止
 *
 * Pure logic (no ESP-IDF / FreeRTOS), host unit-testable (test/test_blind_landing.cpp).
 * TakeoffLandingMgr owns one instance and feeds it one IMU sample per call (400 Hz).
 *
 * An impact alone is not evidence of a touchdown: real flight logs show |accel| of 1.56 g in hover
 * (99.9th percentile 1.50 g) and 2.28 g in aggressive flight. A craft that the floor holds up has
 * three properties at once, and the detector latches only after an impact AND all three for
 * kBlindTouchdownStillS:
 *   1. it is not accelerating: the low-passed |accel| is within kBlindTouchdownStillToleranceG of 1 g;
 *   2. the commanded thrust could not hold it up: thrust / support thrust < kBlindTouchdownThrustFrac;
 *   3. it is not moving: |vertical velocity estimate| < kBlindTouchdownVelocityMps.
 * Any one alone is also true in flight (a steady descent reads 1 g; a high-thrust-error descent has
 * low thrust; a hover has zero velocity), the three together with the impact are not.
 * A second path needs no impact: the FLOOR HOLDING THE CRAFT UP. The commanded thrust is below kBlindFloorThrustFrac
 * of the support thrust AND the low-passed |accel| is within kBlindFloorToleranceG of 1 g, for kBlindFloorS. An
 * accelerometer reads the thrust per mass: in the air with a thrust below 0.7 of the weight it reads below 0.7 g, so
 * 1 g with that thrust can only be the floor carrying the rest of the weight. The two paths are ORed.
 * The latch holds until the descent without a ToF ends (reset(), called when it does).
 *
 * 衝撃だけでは接地の証拠にならない: 実機ログでは |加速度| がホバーで 1.56 g（99.9 パーセンタイル 1.50 g）、
 * 激しい飛行で 2.28 g に達する。床に支えられた機体は 3 つの性質を同時に持つので、検出器は衝撃の後、
 * kBlindTouchdownStillS の間その 3 つが全部成り立ったときだけラッチする:
 *   1. 加速していない: ろ波した |加速度| が 1 g の ±kBlindTouchdownStillToleranceG 以内。
 *   2. 指令推力が機体を支えられない: 推力 ÷ 支持推力 < kBlindTouchdownThrustFrac。
 *   3. 動いていない: |鉛直速度の推定| < kBlindTouchdownVelocityMps。
 * どれか 1 つだけなら飛行中にも成り立つ（定常の降下は 1 g を読む、推力の誤差が大きい降下は推力が低い、
 * ホバーは速度ゼロ）が、衝撃と 3 つが揃うことは無い。
 * 2 つ目の経路は衝撃を要らない: 「床が機体を支えている」。指令推力が支持推力の kBlindFloorThrustFrac 未満、かつ
 * ろ波した |加速度| が 1 g の ±kBlindFloorToleranceG 以内、が kBlindFloorS 続く。加速度計は「推力 ÷ 質量」を読む:
 * 空中で推力が重さの 0.7 倍未満なら読みは 0.7 g 未満なので、その推力で 1 g を読むのは、残りの重さを床が
 * 支えているときだけ。2 つの経路は OR。
 * ラッチは ToF なしの着陸が終わるまで保持する（終わったら reset()）。
 *
 * @design detailed_design.md §3.1 — ToF silent: touchdown = impact then stillness [OK]
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "flight_limits.hpp"
#include "sf_math.hpp"

namespace sf {

/// Detector settings (defaults: sf::limits). 検出器の設定（既定は sf::limits）。
struct BlindTouchdownConfig {
    float    impact_accel_mps2  = limits::kBlindTouchdownAccelMps2;    ///< Impact threshold [m/s^2]  衝撃の閾値
    uint16_t impact_samples     = limits::kBlindTouchdownSamples;      ///< Consecutive samples        連続サンプル数
    float    still_tolerance_g  = limits::kBlindTouchdownStillToleranceG;
    float    still_lpf_tau_s    = limits::kBlindTouchdownStillLpfTauS;
    float    still_s            = limits::kBlindTouchdownStillS;
    float    thrust_frac_max    = limits::kBlindTouchdownThrustFrac;
    float    velocity_max_mps   = limits::kBlindTouchdownVelocityMps;
    float    floor_thrust_frac_max = limits::kBlindFloorThrustFrac;   ///< Floor path: thrust ratio below this / 床の経路: 推力比がこれ未満
    float    floor_tolerance_g  = limits::kBlindFloorToleranceG;      ///< Floor path: | |accel| - 1 g | below this / 床の経路: |加速度| の 1 g からのずれ
    float    floor_s            = limits::kBlindFloorS;               ///< Floor path: both for this long [s] / 床の経路: 両方がこの時間続く [s]
};

/// One IMU cycle of input. IMU 1 周期分の入力。
struct BlindTouchdownInput {
    float dt_s;                 ///< Cycle time [s]                                   周期 [s]
    float accel_magnitude;      ///< |accelerometer| [m/s^2]                          |加速度計| [m/s^2]
    float thrust_ratio;         ///< Commanded / support thrust [-]                   指令推力 ÷ 支持推力 [-]
    float vertical_velocity_up; ///< Vertical velocity estimate, up positive [m/s]    鉛直速度の推定（上が正）[m/s]
};

class BlindTouchdownDetector {
public:
    void init(const BlindTouchdownConfig& config)
    {
        config_ = config;
        reset();
    }

    /// Change the thresholds WITHOUT touching the detector state (a live parameter reload: a latch that is
    /// already set stays set, a count in progress goes on). init() resets the state instead.
    /// 閾値だけを変える。検出器の状態には触れない（パラメータのライブ再読込: すでにラッチしていればそのまま、
    /// 数えている途中のものは続く）。init() は状態もリセットする。
    void setConfig(const BlindTouchdownConfig& config) { config_ = config; }

    /// Back to "no impact, not still" (the blind landing ended or has not begun).
    /// 「衝撃なし、静止でない」に戻す（ToF なしの着陸が終わった、または始まっていない）。
    void reset()
    {
        impact_streak_ = 0;
        impact_seen_   = false;
        still_s_       = 0.0f;
        floor_s_       = 0.0f;
        accel_lpf_g_   = 1.0f;
        lpf_primed_    = false;
        touchdown_     = false;
        by_floor_      = false;
    }

    /// Feed one sample. 1 サンプルを与える。
    void update(const BlindTouchdownInput& in)
    {
        updateImpact(in.accel_magnitude);
        updateStillness(in);
        if (touchdown_) return;
        if (impact_seen_ && still_s_ >= config_.still_s) {
            touchdown_ = true;
        } else if (floor_s_ >= config_.floor_s) {
            touchdown_ = true;
            by_floor_  = true;
        }
    }

    bool touchdown() const { return touchdown_; }        ///< Latched touchdown / ラッチした接地
    bool impactSeen() const { return impact_seen_; }     ///< An impact has been seen / 衝撃を見た
    float stillSeconds() const { return still_s_; }      ///< Current stillness run [s] / 現在の静止の継続 [s]
    float floorSeconds() const { return floor_s_; }      ///< Current floor-holds-the-craft run [s] / 現在の「床が支えている」の継続 [s]
    bool  byFloorSupport() const { return by_floor_; }   ///< The latch came from the floor path (no impact) / ラッチは床の経路（衝撃なし）から

private:
    /// Impact: |accel| above the threshold for impact_samples consecutive samples (latched).
    /// 衝撃: |加速度| が閾値を impact_samples 回連続で超える（ラッチ）。
    void updateImpact(float accel_magnitude)
    {
        if (accel_magnitude > config_.impact_accel_mps2) {
            if (impact_streak_ < UINT16_MAX) ++impact_streak_;
        } else {
            impact_streak_ = 0;
        }
        if (impact_streak_ >= config_.impact_samples) impact_seen_ = true;
    }

    /// Stillness: the three conditions together; the run restarts when any one breaks.
    /// 静止: 3 条件が同時に成り立つ。どれか 1 つが崩れたら継続を数え直す。
    void updateStillness(const BlindTouchdownInput& in)
    {
        const float accel_g = in.accel_magnitude / math::kGravity;
        if (!lpf_primed_) {
            accel_lpf_g_ = accel_g;
            lpf_primed_  = true;
        } else {
            accel_lpf_g_ += (accel_g - accel_lpf_g_) * (in.dt_s / config_.still_lpf_tau_s);
        }
        const bool not_accelerating = std::fabs(accel_lpf_g_ - 1.0f) < config_.still_tolerance_g;
        const bool thrust_too_low   = in.thrust_ratio < config_.thrust_frac_max;
        const bool not_moving       = std::fabs(in.vertical_velocity_up) < config_.velocity_max_mps;
        still_s_ = (not_accelerating && thrust_too_low && not_moving) ? (still_s_ + in.dt_s) : 0.0f;
        const bool floor_holds = std::fabs(accel_lpf_g_ - 1.0f) < config_.floor_tolerance_g &&
                                 in.thrust_ratio < config_.floor_thrust_frac_max;
        floor_s_ = floor_holds ? (floor_s_ + in.dt_s) : 0.0f;
    }

    BlindTouchdownConfig config_;
    uint16_t impact_streak_ = 0;
    bool     impact_seen_   = false;
    float    still_s_       = 0.0f;
    float    floor_s_       = 0.0f;
    float    accel_lpf_g_   = 1.0f;
    bool     lpf_primed_    = false;
    bool     touchdown_     = false;
    bool     by_floor_      = false;
};

}  // namespace sf
