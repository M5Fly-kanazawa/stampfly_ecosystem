/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file tilt_consistency.hpp
 * @brief Tilt consistency monitor — does the attitude estimate agree with gravity on the ground?
 *        傾き整合モニタ — 地上で姿勢推定が重力と一致しているかの検出
 *
 * DETECTION only (INV-3). While the craft is on the ground its accelerometer measures
 * gravity (the craft is not accelerating), so the low-pass filtered accel direction must
 * point where the estimated attitude says "up" is. If they disagree by more than a
 * threshold for a sustained time, the estimate has been shaken off (e.g. by propeller
 * vibration during a ground spin) and the accel-attitude chi2 gate may have latched
 * (Issue #4), rejecting the very correction that would fix it. The monitor raises
 * `mismatch`; the DECISION (disarm / re-level) belongs to StateManager / StateTask.
 *
 * Pure logic: no ESP-IDF, FreeRTOS or parameter-system dependency, so it is host
 * unit-testable (test/test_tilt_consistency.cpp). It lives in sf_estimator because the
 * fact it detects is a property of the estimator output.
 *
 * 検出のみ（INV-3）。機体が地上にある間、加速度計は重力を測る（加速していない）ので、
 * ローパスした加速度方向は推定姿勢が示す「上」と一致するはず。しきい値以上の不一致が
 * 持続したら、推定が（例: 地上でのプロペラ振動で）外れており、accel-attitude の χ² 判定が
 * latch して（Issue #4）それを直す補正自体を棄却している可能性がある。本モニタは
 * `mismatch` を立てるだけで、判断（DISARM・再水平化）は StateManager / StateTask が行う。
 *
 * 純粋ロジック: ESP-IDF・FreeRTOS・パラメータ系に依存せず、ホスト単体テスト可能
 * （test/test_tilt_consistency.cpp）。検出対象は推定器出力の性質なので sf_estimator に置く。
 *
 * @design architecture.md INV-3 — detection here, decision in StateManager/StateTask [OK]
 * @design detailed_design.md §3 注10 — attitude/gravity mismatch on the ground       [OK]
 * @design coding_and_education.md §2 — Bilingual comments                            [OK]
 */

#pragma once

namespace sf {

/// Monitor configuration. The three tilt_check values are PROVISIONAL (see
/// detailed_design.md §3 注10); the norm band is a plausibility gate, not a tuning knob.
/// モニタ設定。tilt_check の3値は暫定（detailed_design.md §3 注10）。ノルム帯は
/// もっともらしさの判定であり調整対象ではない。
struct TiltConsistencyConfig {
    float max_deg       = 10.0f;   // Mismatch threshold [deg]           / 不一致しきい値
    float persist_s     = 0.5f;    // Continuous time above it [s]       / 超過の連続時間
    float lpf_s         = 1.0f;    // Accel low-pass time constant [s]   / 加速度LPF時定数
    float norm_min_mps2 = 4.9f;    // Plausible |accel| lower bound [m/s2] (0.5 g) / ノルム下限
    float norm_max_mps2 = 14.7f;   // Plausible |accel| upper bound [m/s2] (1.5 g) / ノルム上限
};

/// Tilt consistency monitor
/// 傾き整合モニタ
class TiltConsistencyMonitor {
public:
    /// Set the configuration and clear all state.
    /// 設定を適用し、状態を全て消去する。
    void init(const TiltConsistencyConfig& config);

    /// Clear the filter and timers (call when the estimator is reset, so the low-pass
    /// filter re-seeds from the new attitude instead of carrying the old one).
    /// フィルタとタイマを消去する（推定器 reset 時に呼ぶ。LPF が古い姿勢を引きずらず
    /// 新しい姿勢で再シードされる）。
    void reset();

    /// Feed one IMU cycle.
    /// IMU 1周期分を与える。
    /// @param dt_s        Cycle time [s] / 周期
    /// @param accel       Raw accel, body FRD [m/s2] (at rest ≈ [0,0,-9.8]) / 生加速度（FRD）
    /// @param quaternion  Estimated attitude [w,x,y,z], body→NED / 推定姿勢（body→NED）
    /// @param on_ground   Craft is on the ground (ToF) / 地上にいる（ToF 判定）
    void update(float dt_s, const float accel[3], const float quaternion[4], bool on_ground);

    /// True while the estimate disagrees with gravity (sustained, on the ground).
    /// 推定が重力と不一致（地上で持続）の間 true。
    bool mismatch() const { return mismatch_; }

    /// POSITIVE verdict: the monitor has judged (on the ground, filter settled) and the last
    /// angle is within the threshold. False right after reset()/boot (unjudged window of about
    /// lpf_s), off the ground, and while the estimate disagrees (even before persist_s).
    /// 肯定の判定: モニタが判定済み（地上でフィルタ整定済み）かつ直近の角度がしきい値以内。
    /// reset()/起動直後（約 lpf_s の未判定窓）、地上でない間、推定が不一致の間（persist_s
    /// 到達前でも）は false。
    bool verified() const { return verified_; }

    /// Last evaluated angle between filtered accel and estimated up [deg] (diagnostics).
    /// 直近に評価した、LPF 加速度と推定「上」の成す角 [deg]（診断用）。
    float angleDeg() const { return angle_deg_; }

private:
    TiltConsistencyConfig config_{};
    float filtered_accel_[3] = {0.0f, 0.0f, 0.0f};  // LPF state / LPF 状態
    float filter_age_s_      = 0.0f;   // Time since (re)seed [s] / 再シードからの時間
    float over_time_s_       = 0.0f;   // Continuous time above threshold [s] / 超過継続時間
    float angle_deg_         = 0.0f;
    bool  mismatch_          = false;
    bool  verified_          = false;
};

}  // namespace sf
