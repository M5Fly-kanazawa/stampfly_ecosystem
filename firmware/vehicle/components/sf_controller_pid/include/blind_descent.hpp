/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file blind_descent.hpp
 * @brief Thrust of the descent while the downward ToF is silent: a PI on the estimated vertical velocity under a
 *        thrust ceiling that is below the weight
 *        下向き ToF の出力停止中の降下の推力: 重さより小さい推力の上限の下での、鉛直速度の推定に対する PI
 *
 * Pure logic (no ESP-IDF / FreeRTOS), host unit-testable (test/test_blind_landing.cpp).
 * PidController owns one instance and calls update() every cycle of the Landing phase while
 * StateEstimate.tof_silent is set.
 *
 *   ceiling = kNoTofThrustCapRatio * support                      (< weight: the craft cannot climb)
 *   floor   = kBlindDescentThrustMinFrac * support
 *   velocity trusted (age <= kVerticalVelocityTrustS):  thrust = clamp(support + PI(-descent_rate - velocity_up),
 *                                                                      floor, ceiling)
 *   velocity not trusted:                                thrust = ceiling (a constant small downward acceleration)
 *   thrust *= blindLandingLateFactor(elapsed)
 *
 * The velocity is the accelerometer-based estimate (vertical_velocity_estimator.hpp), not the ESKF's: without the ToF
 * the ESKF has no altitude observation, and below the ToF handoff height it pins the velocity at zero. Its error grows
 * with the time since its last reset, so the PI uses it only for kVerticalVelocityTrustS; and the ceiling is a
 * fraction of the SUPPORT thrust (the commanded thrust that held the craft, calibrated by the accelerometer,
 * blind_landing.hpp), never of the hover estimate: however wrong the velocity estimate is, the thrust stays below the
 * weight and the craft cannot rise (a PI that raised the thrust to brake a "too fast" descent climbed when the estimate
 * was too low: SILS, offset 0.1 m/s^2 -> 1.43 m/s at touchdown after a climb). The cost is that the loop can only
 * lower the thrust: the descent speed is set by the ceiling (0.05 g) once the velocity target is reached. The PI gains are
 * fractions of the hover-thrust estimate (kBlindDescentKp per m/s), so a wrong estimate scales the gains, nothing more.
 * The PI is the common PID (pid.hpp, conditional-integration anti-windup).
 * The loop reports thrustRatio() = commanded thrust / support thrust (the touchdown detector, blind_touchdown.hpp).
 *
 * 速度は ESKF のものでなく加速度計に基づく推定（vertical_velocity_estimator.hpp）: ToF が無いと ESKF は高度の観測を
 * 持たず、ToF の引き継ぎ高さより下では速度を 0 に固定する。その誤差は最後のリセットからの時間とともに増えるので、
 * PI が使うのは kVerticalVelocityTrustS の間だけ。そして上限は「支持推力」（機体を支えていた指令推力を加速度計で
 * 較正した値、blind_landing.hpp）の割合で、hover 推力の見積もりの割合ではない: 速度推定がどれだけ誤っていても推力は
 * 重さを下回り、機体は上昇できない（「速すぎる降下」を止めるために推力を上げる PI は、推定が低すぎると上昇した:
 * SILS、オフセット 0.1 m/s^2 で再上昇して 1.43 m/s で接地）。代償として、ループは推力を下げることしかできない:
 * 速度の目標に達した後の降下速度は上限（0.05 g）で決まる。PI のゲインは hover 推力の見積もりに対する割合（速度
 * 1 m/s あたり kBlindDescentKp）で、見積もりが誤っていてもゲインが比例して変わるだけ。
 * PI は共通の PID（pid.hpp、条件付き積分のアンチワインドアップ）。
 * ループは thrustRatio() = 指令推力 ÷ 支持推力 を報告する（接地検出器、blind_touchdown.hpp）。
 *
 * @design detailed_design.md §3.1 — ToF silent: descent by a thrust that cannot climb [OK]
 */

#pragma once

#include "blind_landing.hpp"
#include "flight_limits.hpp"
#include "pid.hpp"

namespace sf {

/// Tunable gains of the descent without a ToF (defaults: sf::limits; SILS sweeps override them by parameter).
/// ToF なしの降下の調整値（既定は sf::limits。SILS の掃引はパラメータで上書きする）。
struct BlindDescentGains {
    float descent_rate_mps = limits::kBlindDescentRateMps;   ///< Target descent speed, down [m/s]    目標の降下速度（下向き）
    float kp_frac          = limits::kBlindDescentKp;        ///< P gain [1/(m/s)], x hover thrust      P ゲイン（hover 推力に対する割合）
    float ti_s             = limits::kBlindDescentTiS;       ///< Integral time [s]                    積分時間
    float cap_ratio        = limits::kNoTofThrustCapRatio;   ///< Thrust ceiling / support thrust [-] (< 1: cannot climb)   推力の上限 ÷ 支持推力（1 未満: 上昇できない）
    float trust_s          = limits::kVerticalVelocityTrustS;///< The PI uses the velocity this long after its last reset [s]   PI が速度を使う、最後のリセットからの時間 [s]
};

class BlindDescent {
public:
    /// Begin a descent without a ToF. ToF なしの降下を始める。
    /// The time of the late fall is NOT restarted: it is counted over the descents of one LANDING (beginLanding()).
    /// 後半の低下の時間は数え直さない: 1 回の LANDING の中の降下を通して数える（beginLanding()）。
    /// @param support_thrust_n Feed-forward thrust and reference of the ceiling [N] (blindLandingStartThrust)
    ///                         フィードフォワードの推力と上限の基準 [N]
    /// @param hover_thrust_n   The (possibly wrong) hover-thrust estimate [N], scales the PI gains
    ///                         ホバー推力の見積もり [N]（PI のゲインの基準）
    /// @param gains            Gains                                              ゲイン
    void start(float support_thrust_n, float hover_thrust_n, const BlindDescentGains& gains)
    {
        support_thrust_n_ = support_thrust_n;
        descent_rate_mps_ = gains.descent_rate_mps;
        cap_ratio_        = gains.cap_ratio;
        trust_s_          = gains.trust_s;
        pid_.reset();
        pid_.kp           = gains.kp_frac * hover_thrust_n;
        pid_.ti           = gains.ti_s;
        pid_.td           = 0.0f;
        pid_.output_limit = limits::kBlindDescentCorrectionLimitFrac * hover_thrust_n;
        thrust_ratio_     = 1.0f;
        active_           = true;
    }

    /// A new LANDING begins: the late-fall time restarts. 新しい LANDING が始まる: 後半の低下の時間を数え直す。
    void beginLanding() { elapsed_s_ = 0.0f; }

    /// Back to idle (the descent ended or the ToF recovered). 待機に戻る（降下が終わった、または ToF が復帰した）。
    void stop()
    {
        active_       = false;
        thrust_ratio_ = 0.0f;
    }

    /// One control cycle: thrust command [N]. 1 制御周期: 推力指令 [N]。
    /// @param vertical_velocity_up Vertical velocity estimate, up positive [m/s]  鉛直速度の推定（上が正）[m/s]
    /// @param velocity_age_s       Time since the estimate's last reset [s]; above kVerticalVelocityTrustS the PI is not
    ///                             used and the thrust is the constant ceiling
    ///                             推定の最後のリセットからの時間 [s]。kVerticalVelocityTrustS を超えたら PI を使わず、
    ///                             推力は上限の一定値
    /// @param dt_s                 Cycle time [s]                                  周期 [s]
    float update(float vertical_velocity_up, float velocity_age_s, float dt_s)
    {
        elapsed_s_ += dt_s;
        const float ceiling = blindThrustCeiling(support_thrust_n_, cap_ratio_);   // below the weight: no thrust above it, whatever the velocity says / 重さ未満: 速度が何であれこれ以上の推力は出さない
        const float floor   = limits::kBlindDescentThrustMinFrac * support_thrust_n_;
        float thrust = ceiling;
        pid_.integration_enabled = isVelocityTrusted(velocity_age_s);   // no PI, no integration, once the estimate is not trusted / 推定を信用できないなら PI も積分も止める
        if (pid_.integration_enabled) {
            thrust = piThrust(vertical_velocity_up, dt_s, floor, ceiling);
        }
        thrust *= blindLandingLateFactor(elapsed_s_);
        thrust_ratio_ = (support_thrust_n_ > 0.0f) ? (thrust / support_thrust_n_) : 0.0f;
        return thrust;
    }

    bool  active() const          { return active_; }              ///< A descent without a ToF is running / 降下中
    float elapsedS() const        { return elapsed_s_; }           ///< Descent time of this LANDING [s] / この LANDING の降下の時間
    float supportThrust() const   { return support_thrust_n_; }    ///< Feed-forward [N] / フィードフォワード
    float ceilingThrust() const   { return blindThrustCeiling(support_thrust_n_, cap_ratio_); }   ///< Thrust ceiling [N] / 推力の上限
    float thrustRatio() const     { return thrust_ratio_; }        ///< Commanded / support thrust, 0 when idle / 指令 ÷ 支持推力、待機中は 0
    float integral() const        { return pid_.integral; }        ///< PI integrator [N] / PI の積分値

private:
    bool isVelocityTrusted(float velocity_age_s) const { return velocity_age_s <= trust_s_; }

    /// Thrust of the PI around the support thrust, bounded to [floor, ceiling], with a ONE-SIDED anti-windup
    /// that acts in the SAME cycle: when the bounded thrust is clamped, the integral step of THIS cycle is
    /// undone only if it pushed further into the clamp (above the ceiling and the integral rose, or below the
    /// floor and it fell). A step back toward the allowed range is kept, so the integrator can unwind while the
    /// thrust is still clamped. (The earlier rule switched integration off for the NEXT cycle from the previous
    /// thrust, in both directions: one cycle late, and unable to unwind while clamped.)
    /// support 推力のまわりの PI の推力を [下限, 上限] に制限する。同じ周期で効く「片側」のアンチワインドアップ:
    /// 制限にかかったとき、「この周期」の積分の増分を、制限の奥へ押す向きの場合だけ取り消す（上限の上で積分が
    /// 増えた、または下限の下で減った）。許す範囲へ戻る向きの増分は残すので、推力が制限にかかったままでも積分器は
    /// 戻れる。（以前の規則は、前の周期の推力で「次の周期」の積分を両方向とも止めた: 1 周期遅れで、制限にかかった間は
    /// 戻れなかった。）
    float piThrust(float vertical_velocity_up, float dt_s, float floor, float ceiling)
    {
        const float integral_before = pid_.integral;
        float correction = pid_.compute(-descent_rate_mps_, vertical_velocity_up, dt_s);
        float thrust     = support_thrust_n_ + correction;
        const float step = pid_.integral - integral_before;
        const bool windup_into_ceiling = (thrust > ceiling) && (step > 0.0f);
        const bool windup_into_floor   = (thrust < floor) && (step < 0.0f);
        if (windup_into_ceiling || windup_into_floor) {
            pid_.integral = integral_before;
            thrust       -= step;
        }
        if (thrust > ceiling) thrust = ceiling;
        if (thrust < floor)   thrust = floor;
        return thrust;
    }

    PID   pid_;
    bool  active_            = false;
    float support_thrust_n_  = 0.0f;   // [N]
    float descent_rate_mps_  = 0.0f;   // [m/s]
    float cap_ratio_         = limits::kNoTofThrustCapRatio;   // [-]
    float trust_s_           = limits::kVerticalVelocityTrustS;   // [s]
    float elapsed_s_         = 0.0f;   // [s]
    float thrust_ratio_      = 0.0f;   // [-]
};

}  // namespace sf
