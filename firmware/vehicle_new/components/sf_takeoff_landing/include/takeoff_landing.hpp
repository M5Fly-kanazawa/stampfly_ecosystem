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
 * @design architecture.md §4 — State machine transitions               [OK]
 * @design architecture.md INV-3 — detection here, decision in StateTask [OK]
 * @design detailed_design.md §3 — State transition table (TAKEOFF/LANDING) [OK]
 * @design detailed_design.md §3 注7 — stalled-descent touchdown detector [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
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
    uint32_t landing_hold_ms = 1000;    // Landing confirm [ms]    / 着陸確認時間
    // Stalled-descent touchdown ("ground-effect float"). During a commanded landing
    // descent, near the ground the rotors make MORE lift (ground effect), so the craft
    // holds altitude on reduced thrust and the constant-velocity descent STALLS above the
    // 5cm ground threshold — a ToF-only detector never triggers and the craft floats
    // (pilot report 2026-06-14). Touchdown is then declared from KINEMATICS: while a
    // landing descent is commanded AND the ToF reads within near_ground_tof_m AND the
    // vertical velocity has stalled (< landing_vel_mps) — we are trying to descend but the
    // ground has stopped us. No thrust threshold (a fragile, hover-sensitive signal) is
    // needed; the stalled descent IS the weight-on-ground evidence.
    // 降下停滞による接地（「地面効果フロート」）。着陸降下の指令中、接地近傍ではローター揚力が
    // 増す（地面効果）ため機体は低推力で高度を保ち、一定速度降下が 5cm 地上閾値の上で停滞する
    // — ToF のみの検出器は発火せず機体は浮く（パイロット報告 2026-06-14）。そこで接地を
    // キネマティクスから宣言: 着陸降下が指令されており ∧ ToF が near_ground_tof_m 以内 ∧
    // 鉛直速度が停滞（< landing_vel_mps）＝降下しようとしているのに地面に止められている。脆い
    // （ホバー感度の高い）推力閾値は不要で、降下停滞そのものが地面支持の証拠。
    float near_ground_tof_m  = 0.12f;   // Near-ground band for stalled-descent touchdown [m] / 停滞接地の近傍帯
    uint32_t stall_hold_ms   = 600;     // Stalled-descent confirm [ms] (< firm-ground confirm) / 停滞確認時間
    // Consecutive ground-side ToF samples required to flip on_ground_ false→true
    // (air→ground debounce). A single near-reflection ghost mid-flight can no
    // longer assert "on ground"; landing already needs landing_hold_ms downstream
    // so the 2-sample (~66ms at 30Hz) delay is negligible (M-6).
    // NOTE: the ground→air flip (takeoff) is INTENTIONALLY immediate — the ESKF
    // vertical handoff keys off that edge and is a sample-precision class-B reset
    // (architecture.md §4); debouncing it delays the handoff and degrades the
    // take-off transient (measured: duty_max 0.80 → 1.0 with a 2-sample delay).
    // on_ground_ を false→true に反転させるのに必要な地上側連続 ToF サンプル数
    // （air→ground デバウンス）。飛行中の単発近距離ゴーストで「接地」を主張できなくする。
    // 着陸は下流に landing_hold_ms があるので2サンプル(≈66ms)の遅延は無視できる (M-6)。
    // 注: ground→air（離陸）の反転は意図的に即時 — ESKF 鉛直ハンドオフがこのエッジを
    // 起点とするサンプル精度のクラスB reset（architecture.md §4）であり、デバウンスすると
    // ハンドオフが遅れ離陸過渡が劣化する（実測: 2サンプル遅延で duty_max 0.80→1.0）。
    uint8_t flip_confirm_samples = 2;
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
    /// `vertical_velocity` [m/s, NED down] is injected (not read from a topic) so the
    /// landing detector stays a pure function of its inputs — detection here, the
    /// transition decision in StateTask (architecture §2: separate detection from decision).
    /// `in_landing_descent` is true while the autonomous landing descent is commanded
    /// (FlightState::LANDING). It is a STATE-derived INPUT (like `armed`), not a decision —
    /// it enables the stalled-descent touchdown branch (a stall near the ground only means
    /// "landed" when we are actually trying to descend; a deliberate low hover is not).
    /// `vertical_velocity`[m/s, NED down] を注入する（トピックから読まない）。着陸検出器を入力の
    /// 純粋な関数に保ち、検出はここ・遷移判断は StateTask（architecture §2: 検出と判断の分離）。
    /// `in_landing_descent` は自動着陸降下の指令中（FlightState::LANDING）に true。`armed` と同じ
    /// 状態由来の「入力」（判断ではない）で、降下停滞による接地判定を有効にする（接地近傍の停滞が
    /// 「着陸」を意味するのは実際に降下しようとしている時のみ。意図的な低ホバーは着陸でない）。
    void update(const TofData& tof, bool armed, float vertical_velocity,
                bool in_landing_descent);

    /// Check if vehicle is on the ground
    /// 機体が地上にあるか確認する
    bool isOnGround() const { return on_ground_; }

    /// Check if landing is detected (sustained low altitude + low vertical velocity).
    /// A LEVEL flag (true while the landing condition holds), not a one-shot pulse, so a
    /// slower consumer (StateTask at 50 Hz) cannot miss it through a Latest topic.
    /// 着陸を検出したか確認する（低高度＋低鉛直速度の持続）。ワンショットでなく条件成立中 true の
    /// レベルフラグ。低速の消費者（50Hz の StateTask）が Latest トピック越しに取りこぼさない。
    bool isLandingDetected() const { return landing_detected_; }

    /// Check if the vehicle is held in hand (disarmed + ToF reads a valid distance above
    /// the airborne threshold). Drives the IDLE_GROUND ↔ IDLE_HELD transition.
    /// 機体が手で持たれているか確認する（disarmed＋ToF が空中閾値以上の有効距離）。
    /// IDLE_GROUND ↔ IDLE_HELD 遷移を駆動する。
    bool isHeld() const { return held_; }

private:
    /// Evaluate ToF distance for ground contact
    /// ToF距離で地面接地を評価する
    void evaluateToF(const TofData& tof);

    /// Detect landing: firm ground (ToF<5cm) OR a stalled landing descent near the ground,
    /// each with low vertical velocity sustained (level flag, see getter).
    /// 着陸検出: 確実な接地（ToF<5cm）or 接地近傍で着陸降下が停滞、いずれも低鉛直速度の持続。
    void detectLanding(float vertical_velocity, bool in_landing_descent);

    /// Evaluate held-in-hand from the ToF distance (disarmed only, hysteresis)
    /// ToF 距離から手持ちを評価する（disarmed 時のみ、ヒステリシス）
    void evaluateHeld(const TofData& tof);

    TakeoffLandingConfig config_;
    bool     on_ground_        = true;   // Ground contact flag (ToF<5cm) / 地面接地フラグ
    bool     near_ground_      = false;  // ToF within near_ground_tof_m  / 接地近傍フラグ
    bool     held_             = false;  // Held-in-hand flag    / 手持ちフラグ
    bool     landing_detected_ = false;  // Landing detected flag (level) / 着陸検出フラグ
    uint32_t landing_start_ms_ = 0;      // Firm-ground landing timer start / 接地着陸タイマー開始
    uint32_t stall_start_ms_   = 0;      // Stalled-descent timer start  / 降下停滞タイマー開始
    uint8_t  ground_streak_    = 0;      // Consecutive ground-side ToF samples (air→ground debounce) / 地上側連続数
};

}  // namespace sf
