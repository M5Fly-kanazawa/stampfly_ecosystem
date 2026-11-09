/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file blind_landing.hpp
 * @brief Pure pieces of the descent while the downward ToF is silent: support thrust, its calibration, thrust ceiling,
 *        late fall
 *        下向き ToF の出力停止中の降下の純関数: 支持推力とその較正、推力の上限、後半の低下
 *
 * Pure functions and one small filter class (no ESP-IDF / FreeRTOS), host unit-testable
 * (test/test_blind_landing.cpp). The velocity loop itself is sf::BlindDescent
 * (sf_controller_pid/blind_descent.hpp); the velocity is sf::VerticalVelocityEstimator
 * (vertical_velocity_estimator.hpp). All the numbers are in sf::limits (flight_limits.hpp, kBlind*, kNoTof*,
 * kSupport*), with their basis.
 *
 * Support thrust (SupportThrustFilter, blindLandingStartThrust): the commanded thrust that held the craft,
 * calibrated by the accelerometer (thrust / mass is what it measures), so it does not depend on the hover-thrust
 * estimate. The window of samples opens when the craft has LEFT THE FLOOR: before the take-off the floor carries
 * the weight and the reading is g whatever the thrust, afterwards thrust x g / reading is the hover thrust even
 * while accelerating. Before the window (take-off not yet made) the estimate is used, lowered further
 * (kBlindUncalibratedFrac), and the thrust ceiling follows the support thrust.
 * Thrust ceiling (blindThrustCeiling): kNoTofThrustCapRatio x support thrust, below the weight: the descent
 * without a ToF cannot climb, whatever the velocity estimate says.
 * Late fall (blindLandingLateFactor): the thrust factor that starts falling after kBlindLandingLateStartS and reaches
 * zero at kBlindLandingLateStartS + 1 / kBlindLandingLateRampPerS, whatever the velocity estimate says: the last
 * safety when the integrated velocity has drifted.
 *
 * 支持推力（SupportThrustFilter、blindLandingStartThrust）: 機体を支えていた指令推力を加速度計（測るのは
 * 「推力 ÷ 質量」）で較正した値。ホバー推力の見積もりに依らない。サンプルの窓は機体が「床を離れた」ときに開く:
 * 離陸前は床が重さを支え、読みは推力に関係なく g。離陸後は 推力 × g ÷ 読み が、加速中でもホバー推力になる。
 * 窓の前（離陸していない）は見積もりを使い、さらに低くして（kBlindUncalibratedFrac）、推力の上限は支持推力に従う。
 * 推力の上限（blindThrustCeiling）: kNoTofThrustCapRatio × 支持推力で、重さより小さい: ToF なしの降下は、速度推定が
 * 何であれ上昇できない。
 * 後半の低下（blindLandingLateFactor）: kBlindLandingLateStartS 後に下がり始め、
 * kBlindLandingLateStartS + 1 / kBlindLandingLateRampPerS でゼロになる推力の係数。速度推定が何であれ働く。
 * 積分した速度がドリフトしたときの最後の安全策。
 *
 * @design detailed_design.md §3.1 — ToF silent: descent by a thrust that cannot climb [OK]
 */

#pragma once

#include <cmath>

#include "flight_limits.hpp"
#include "sf_math.hpp"   // sf::math::kGravity

namespace sf {

/**
 * @brief Upward specific force [m/s^2] from the attitude and the body-frame accelerometer reading.
 *        姿勢と機体座標の加速度計測定値から、上向きの加速度計の測定値 [m/s^2] を求める
 *
 * Third row of the body-to-NED rotation (same convention as math::Quat::to_dcm; NED z is down, so
 * the sign is negated). A craft in free flight reads about +g at hover; the excess over g is its
 * vertical acceleration. The thrust divided by the mass is what this measures, so it does not depend
 * on the hover-thrust estimate.
 * 機体→NED 回転の第 3 行（math::Quat::to_dcm と同じ規約。NED の z は下向きなので符号を反転）。
 * ホバー中の機体はおよそ +g を読み、g を超えた分が鉛直加速度。測っているのは「推力 ÷ 質量」なので、
 * ホバー推力の見積もりに依らない。
 *
 * @param attitude Quaternion [w, x, y, z], body to NED   クォータニオン [w, x, y, z]（機体→NED）
 * @param specific_force Bias-corrected specific force, body FRD [m/s^2]
 *                       バイアス補正済みの加速度計の測定値（機体 FRD）[m/s^2]
 */
inline float upwardSpecificForce(const float attitude[4], const float specific_force[3])
{
    const float qw = attitude[0], qx = attitude[1], qy = attitude[2], qz = attitude[3];
    const float r31 = 2.0f * (qx * qz - qw * qy);
    const float r32 = 2.0f * (qy * qz + qw * qx);
    const float r33 = 1.0f - 2.0f * (qx * qx + qy * qy);
    return -(r31 * specific_force[0] + r32 * specific_force[1] + r33 * specific_force[2]);
}

/**
 * @brief Is the filtered upward specific force a real measurement?
 *        ろ波した上向きの加速度計の測定値は、本物の測定値か
 * An estimator that does not fill specific_force reads 0; in flight the value sits near g.
 * specific_force を埋めない推定器は 0 を出す。飛行中の値は g の近く。
 */
inline bool upwardSpecificForceValid(float upward_specific_force)
{
    return upward_specific_force >= limits::kBlindMinUpwardSpecificForce;
}

/**
 * @brief Means of the commanded thrust and of the upward specific force over the NEAR-HOVER samples after the craft has
 *        left the floor (the support thrust is their quotient x g)
 *        機体が床を離れた後の「ホバー近傍」のサンプルでの、指令推力と上向きの加速度計の測定値の平均（支持推力はその商 × g）
 *
 * Nothing is averaged before the craft leaves the floor (the floor carries part of the weight: the reading is about g while the
 * thrust rises). It has left when the healthy ToF says it is airborne, or the estimated rise since rest reaches kSupportLeaveRiseM,
 * or the low-passed excess of the reading over g reaches kSupportLeaveExcessMps2 (the last two are accelerometer integrals / readings:
 * an offset error can hide a gentle take-off from them, SILS: rise -1.9 m after +0.2 m/s^2 from the take-off). After that only the samples whose low-passed reading is within
 * kSupportHoverBandFrac x g of g enter the means: there the thrust is the weight, so the quotient is the hover thrust
 * and the nonlinearity of the thrust against the command (SILS: the quotient is 12 % high in a climb of +0.2 g) does not
 * matter. The means are plain means over the first kBlindThrustFilterTauS of such samples (equal weights: no seed noise), then a
 * low-pass of that time constant (it follows the battery sag over a long flight). The commanded thrust is first passed through the
 * actuation lag (kSupportThrustLagS). calibrated() holds once kSupportWindowS of near-hover samples have been averaged:
 * real hovers show a 2 s mean within +-1 % of the long mean (0.3 s: +-5 %, 1 s: +-2.5 %).
 * 機体が床を離れるまでは何も平均しない（床が重さの一部を支え、推力が上がっても読みは約 g）。離れたとは、健全な ToF が空中と
 * 言っている、静止からの上昇量の推定が kSupportLeaveRiseM に達する、または読みの g に対するろ波した超過が kSupportLeaveExcessMps2 に達すること。その後は、
 * ろ波した読みが g の ±kSupportHoverBandFrac × g 以内のサンプルだけを平均に入れる: そこでは推力は重さに等しく、商はホバー
 * 推力になり、推力と指令の非線形性（SILS: +0.2 g の上昇では商が 12 % 高い）は影響しない。平均は、そのようなサンプルの最初の
 * kBlindThrustFilterTauS の間は単純平均（重みが等しい: 初期値の雑音が残らない）、その後は同じ時定数のローパス（長い飛行の
 * 電圧低下に追従する）。指令推力は先に作動の遅れ（kSupportThrustLagS）を通す。calibrated() は、ホバー近傍のサンプルを
 * kSupportWindowS 分平均したら真になる: 実機のホバーでは 2 s の平均が長い平均の ±1 % に収まる（0.3 s: ±5 %、1 s: ±2.5 %）。
 */
class SupportThrustFilter {
public:
    /// Back to "on the floor, nothing averaged". 「床の上、何も平均していない」に戻す。
    void reset()
    {
        left_floor_ = false;
        primed_     = false;
        excess_lpf_ = 0.0f;
        excess_primed_ = false;
        band_s_     = 0.0f;
        lagged_n_   = 0.0f;
        lag_primed_ = false;
        age_s_      = 0.0f;
        thrust_     = 0.0f;
        force_      = 0.0f;
        air_age_s_  = 0.0f;
        air_thrust_ = 0.0f;
        air_force_  = 0.0f;
    }

    /// One control cycle. 1 制御周期。
    /// @param dt_s              Cycle time [s]                                        周期 [s]
    /// @param thrust_n          Commanded thrust this cycle [N]                       今周期の指令推力 [N]
    /// @param upward_force      Upward specific force (bias-corrected) [m/s^2]        上向きの加速度計の測定値（バイアス補正済み）[m/s^2]
    /// @param rise_m            Estimated rise since the craft last rested [m]        機体が最後に静止してからの上昇量の推定 [m]
    /// @param tof_airborne      The healthy ToF says the craft is above the ground (ground-contact verdict) - a fact the accelerometer
    ///                          integral cannot spoil with an offset error     健全な ToF が「地面から離れた」と言っている（接地の判定）。
    ///                          加速度計の積分がオフセット誤差で狂っても影響されない事実
    void update(float dt_s, float thrust_n, float upward_force, float rise_m, bool tof_airborne = false)
    {
        const bool force_valid = (upward_force >= limits::kBlindMinUpwardSpecificForce);
        // The thrust that acts on the craft trails the command (actuation delay and lag): compare the accelerometer with that.
        // 機体に働く推力は指令に遅れる（作動の遅れ）: 加速度計はそれと比べる。
        lagged_n_   = lag_primed_ ? (lagged_n_ + (thrust_n - lagged_n_) * (dt_s / limits::kSupportThrustLagS)) : thrust_n;
        lag_primed_ = true;
        if (force_valid) {
            const float excess = upward_force - math::kGravity;
            excess_lpf_    = excess_primed_ ? (excess_lpf_ + (excess - excess_lpf_) * (dt_s / limits::kSupportLeaveLpfTauS)) : excess;
            excess_primed_ = true;
        }
        if (!left_floor_) {
            left_floor_ = tof_airborne || (rise_m >= limits::kSupportLeaveRiseM) || (excess_lpf_ >= limits::kSupportLeaveExcessMps2);
            if (!left_floor_) return;
        }
        if (force_valid) {   // every airborne sample, for the early estimate before a hover / ホバーの前の早い推定のための、空中の全サンプル
            air_age_s_ += dt_s;
            const float air_window_s = (air_age_s_ < limits::kBlindThrustFilterTauS) ? air_age_s_ : limits::kBlindThrustFilterTauS;
            const float air_gain = (air_age_s_ > dt_s) ? (dt_s / air_window_s) : 1.0f;
            air_thrust_ += (lagged_n_ - air_thrust_) * air_gain;
            air_force_  += (upward_force - air_force_) * air_gain;
        }
        const bool near_hover = force_valid && (std::fabs(excess_lpf_) <= limits::kSupportHoverBandFrac * math::kGravity);
        band_s_ = near_hover ? (band_s_ + dt_s) : 0.0f;
        if (band_s_ < limits::kSupportBandSettleS) return;   // only samples where the thrust is the weight, once the low-pass has settled in the band / 推力が重さに等しいサンプルだけ。ローパスが帯の中で落ち着いてから
        age_s_ += dt_s;
        const float window_s = (age_s_ < limits::kBlindThrustFilterTauS) ? age_s_ : limits::kBlindThrustFilterTauS;
        const float gain = primed_ ? (dt_s / window_s) : 1.0f;   // plain mean first, then the low-pass / 最初は単純平均、その後ローパス
        thrust_ += (lagged_n_ - thrust_) * gain;
        force_  += (upward_force - force_) * gain;
        primed_ = true;
    }

    bool  leftFloor() const  { return left_floor_; }                         ///< The craft has left the floor / 機体は床を離れた
    bool  calibrated() const { return left_floor_ && primed_ && age_s_ >= limits::kSupportWindowS; }   ///< Window long enough / 窓が十分長い
    bool  hasThrust() const  { return primed_; }                             ///< thrust() holds a value / thrust() が値を持つ
    float nearHoverSeconds() const { return age_s_; }
    float airborneSeconds() const { return air_age_s_; }                     ///< Airborne time averaged so far [s] / ここまでに平均した空中の時間 [s]
    /// Early estimate: the quotient over ALL airborne samples, x kSupportClimbDerate; 0 when there are fewer than kSupportClimbMinS of them
    /// 早い推定: 空中の「全」サンプルでの商 × kSupportClimbDerate。kSupportClimbMinS 未満なら 0
    float climbEstimate() const
    {
        if (air_age_s_ < limits::kSupportClimbMinS || air_force_ < limits::kBlindMinUpwardSpecificForce) return 0.0f;
        return limits::kSupportClimbDerate * air_thrust_ * math::kGravity / air_force_;
    }                        ///< Near-hover time averaged so far [s] / ここまでに平均したホバー近傍の時間 [s]
    float thrust() const     { return thrust_; }                             ///< Mean commanded thrust [N] / 指令推力の平均 [N]
    float upwardForce() const { return force_; }                             ///< Mean upward specific force [m/s^2], 0 = none / 上向きの加速度計の測定値の平均。0 = 無し

private:
    bool  left_floor_ = false;
    bool  primed_     = false;
    float excess_lpf_ = 0.0f;   // [m/s^2] low-passed (reading - g) / （読み − g）のローパス
    bool  excess_primed_ = false;
    float band_s_     = 0.0f;   // [s] time the low-passed reading has stayed in the near-hover band / ろ波した読みがホバー近傍の帯にいた連続時間
    float lagged_n_   = 0.0f;   // [N] commanded thrust through the actuation lag / 作動の遅れを通した指令推力
    bool  lag_primed_ = false;
    float age_s_      = 0.0f;   // [s] near-hover time averaged / 平均したホバー近傍の時間
    float thrust_     = 0.0f;   // [N]
    float force_      = 0.0f;   // [m/s^2]
    float air_age_s_  = 0.0f;   // [s] airborne time averaged / 平均した空中の時間
    float air_thrust_ = 0.0f;   // [N] mean commanded thrust over all airborne samples / 空中の全サンプルでの指令推力の平均
    float air_force_  = 0.0f;   // [m/s^2]
};

/**
 * @brief Support thrust: the thrust that held the craft, the feed-forward of the descent without a ToF.
 *        支持推力: 機体を支えていた推力。ToF なしの降下のフィードフォワード
 *
 * Calibrated (the window is long enough and the mean reading is usable):
 *   support = mean_thrust * g / mean_upward_specific_force
 * (a steady hover reads g, so support = mean_thrust; during a climb the thrust is above the hover thrust and the
 * reading is above g by the same ratio, which the quotient removes). It needs neither the hover-thrust estimate nor a
 * hover to have been flown, and is bounded to [kBlindCalibratedMinFrac, kBlindCalibratedMaxFrac] x hover_thrust
 * against a bad reading.
 * Not calibrated (take-off not yet made, window still short, or no usable reading): the hover-thrust estimate,
 * lowered: min(mean_thrust if any, kBlindUncalibratedFrac * hover_thrust). The thrust ceiling follows it, so an estimate
 * that is too high cannot make the craft climb unless it is more than 1 / kBlindUncalibratedFrac / kNoTofThrustCapRatio
 * (= 40 %) above the truth.
 * 較正済み（窓が十分長く、読みの平均が使える）:
 *   support = 推力の平均 × g ÷ 上向きの加速度計の測定値の平均
 * （定常ホバーは g を読むので support = 推力の平均。上昇中は推力がホバー推力より高く、読みも同じ比率で g より
 * 高いので、商でその分が消える）。ホバー推力の見積もりも、ホバーを飛んだ履歴も要らず、誤った読みに備え
 * [kBlindCalibratedMinFrac, kBlindCalibratedMaxFrac] × hover_thrust に制限する。
 * 未較正（離陸していない、窓がまだ短い、または使える読みが無い）: ホバー推力の見積もりを低くしたもの:
 * min(推力の平均（あれば）, kBlindUncalibratedFrac × hover_thrust)。推力の上限はそれに従うので、見積もりが高すぎても、
 * 真値を 1 / kBlindUncalibratedFrac / kNoTofThrustCapRatio（= 40 %）超えて高くない限り、機体は上昇しない。
 *
 * @param filter       The window means (SupportThrustFilter)                  窓の平均
 * @param hover_thrust The (possibly wrong) hover thrust estimate [N]          ホバー推力の見積もり（誤っていることがある）[N]
 */
inline float blindLandingStartThrust(const SupportThrustFilter& filter, float hover_thrust)
{
    const float uncalibrated_cap = limits::kBlindUncalibratedFrac * hover_thrust;
    if (filter.calibrated() && upwardSpecificForceValid(filter.upwardForce())) {
        float support = filter.thrust() * math::kGravity / filter.upwardForce();
        const float lower = limits::kBlindCalibratedMinFrac * hover_thrust;
        const float upper = limits::kBlindCalibratedMaxFrac * hover_thrust;
        if (support < lower) support = lower;
        if (support > upper) support = upper;
        return support;
    }
    const float early = filter.climbEstimate();   // 0 = none yet / 0 = まだ無い
    return (early > 0.0f && early < uncalibrated_cap) ? early : uncalibrated_cap;
}

/**
 * @brief Thrust ceiling of the descent without a ToF: below the weight, so the craft cannot climb.
 *        ToF なしの降下の推力の上限: 重さより小さいので、機体は上昇できない
 * @param support_thrust Support thrust [N] (blindLandingStartThrust)   支持推力 [N]
 * @param cap_ratio      Ceiling / support thrust (default kNoTofThrustCapRatio)   上限 ÷ 支持推力（既定は kNoTofThrustCapRatio）
 */
inline float blindThrustCeiling(float support_thrust, float cap_ratio = limits::kNoTofThrustCapRatio)
{
    return cap_ratio * support_thrust;
}

/**
 * @brief Factor on the thrust t seconds into the descent without a ToF: the late fall.
 *        ToF なしの降下の t 秒後の、推力に掛ける係数（後半の低下）
 *
 * 1 until kBlindLandingLateStartS, then it falls at kBlindLandingLateRampPerS and stays at 0.
 * kBlindLandingLateStartS までは 1、以後 kBlindLandingLateRampPerS で下がり、0 で止まる。
 * @param elapsed_s Time of the descent without a ToF in this LANDING [s]   この LANDING の ToF なしの降下の時間 [s]
 */
inline float blindLandingLateFactor(float elapsed_s)
{
    if (elapsed_s <= limits::kBlindLandingLateStartS) return 1.0f;
    const float factor =
        1.0f - limits::kBlindLandingLateRampPerS * (elapsed_s - limits::kBlindLandingLateStartS);
    return (factor > 0.0f) ? factor : 0.0f;
}

}  // namespace sf
