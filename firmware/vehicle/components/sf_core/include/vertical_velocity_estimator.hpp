/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file vertical_velocity_estimator.hpp
 * @brief Vertical velocity from an accelerometer integral, for the descent without a ToF
 *        ToF なしの降下のための、加速度積分による鉛直速度
 *
 * Pure logic (no ESP-IDF / FreeRTOS), host unit-testable (test/test_blind_landing.cpp).
 * PidController owns one instance and calls update() every control cycle.
 *
 * Why a separate estimator: while the downward ToF is silent the ESKF has no altitude observation,
 * and while the ground-contact flag says "on the ground" it pins its vertical velocity to ZERO (the
 * take-off climb below the ToF handoff height). In both cases state.velocity says nothing about a
 * climb that is under way, yet the blind descent has to remove that climb. This estimator keeps its
 * own number, by three rules (up positive, g = 9.80665):
 *   1. At rest on the ground (DISARM, ARMED_GROUND below the take-off thrust): velocity := 0 and the
 *      accelerometer offset B := low-passed (f_raw - g): at rest the reading must be exactly g, so what is
 *      left is the offset (a zero-velocity update).
 *   2. The ToF is healthy, consumed by the estimator and the ESKF vertical state is free (not pinned by
 *      the on-ground hold) for at least kVerticalVelocityHandoffSettleS: a second-order complementary
 *      observer follows the ESKF vertical velocity (time constant kVerticalVelocityEskfTauS):
 *        v' = (f_raw - g - B) + (2/tau) e,   B' = -(1/tau^2) e,   e = v_eskf - v.
 *      The accelerometer gives the fast part, the ESKF the long-term mean, and B learns what the
 *      accelerometer reads off 1 g in flight (offset, vibration rectification, the ESKF bias error).
 *   3. Otherwise (ToF silent / invalid for the episode, or the ESKF vertical state pinned or just
 *      handed off): v += (f_raw - g - B) dt with B FROZEN: the accelerometer integrated since the last
 *      time rule 1 or 2 held. B is the offset of the RAW accelerometer, so the ESKF changing its own
 *      bias estimate during the episode (it does: SILS, +0.19 m/s^2 in the first 0.6 s of a descent)
 *      does not move the integral.
 *   Contact (part of rule 3): when the upward reading exceeds the contact threshold (the same as the touchdown
 *      impact: kBlindTouchdownAccelG) for kBlindTouchdownSamples consecutive samples, the floor has stopped the craft;
 *      once the reading falls back below the release level the velocity is set to ZERO (a zero-velocity update).
 *      Without it the integral of the impact (clipped by the accelerometer range, noisy) leaves an error of
 *      +0.2 m/s on the ground (SILS noise n2: 22 of 80 runs) which keeps the touchdown detector from seeing "not moving".
 * Why not copy the ESKF velocity (reset to it every cycle): right after the
 * ground-to-air handoff the ESKF velocity is wrong for ~1 s (SILS: 0.56 m/s while the craft climbs at
 * 1.27 m/s), and its hover noise is +-0.1-0.2 m/s; the observer keeps the accelerometer's fast part.
 * The error of rule 3 is (offset error) x (time since the last reset): 0.1 m/s^2 for 3 s is 0.3 m/s.
 * That is why the descent without a ToF trusts it for only kVerticalVelocityTrustS after its last reset, never lets the thrust exceed a ceiling below the weight, and keeps the late thrust fall as the last safety
 * (blind_landing.hpp), and why fuseExternalVelocity() exists: the barometer (next stage) feeds its
 * velocity into rule 3 through it, with no change to the callers.
 *
 * ESKF の鉛直速度は、ToF が出力停止している間は高度の観測を失い、接地フラグが「接地」の間
 * （ToF の引き継ぎ高さより下の離陸の上昇）は 0 に固定される。どちらの場合も state.velocity は進行中の
 * 上昇を表さないのに、ToF なしの降下はその上昇を止めなければならない。そこで専用の値を持つ。規則は 3 つ
 * （上が正、g = 9.80665）:
 *   1. 地上で静止（DISARM、離陸推力未満の ARMED_GROUND）: 速度 := 0、加速度計のオフセット B := (f_raw − g) の
 *      ローパス値: 静止していれば読みはちょうど g のはずなので、残りがオフセット（ゼロ速度更新）。
 *   2. ToF が健全で推定器が取り込み済み、かつ ESKF の鉛直状態が自由（接地ホールドで固定されていない）が
 *      kVerticalVelocityHandoffSettleS 以上続いた: 2 次の相補オブザーバが ESKF の鉛直速度に従う
 *      （時定数 kVerticalVelocityEskfTauS）:
 *        v' = (f_raw − g − B) + (2/tau) e,   B' = −(1/tau^2) e,   e = v_eskf − v。
 *      速い成分は加速度計、長期の平均は ESKF が与え、B は飛行中に加速度計が 1 g からずれて読む分
 *      （オフセット・振動による読みの偏り・ESKF のバイアス誤差）を学習する。
 *   3. それ以外（エピソード中の ToF 出力停止・無効、ESKF の鉛直状態が固定中、または引き継ぎ直後）:
 *      v += (f_raw − g − B) dt。B は固定されたまま: 規則 1・2 が最後に成り立ってからの加速度計の積分。B は生の
 *      加速度計のオフセットなので、エピソード中に ESKF が自分のバイアス推定を動かしても（動く: SILS で降下の
 *      最初の 0.6 s に +0.19 m/s^2）積分は動かない。
 *   接触（規則 3 の一部）: 上向きの読みが接触の閾値（接地の衝撃と同じ kBlindTouchdownAccelG）を kBlindTouchdownSamples
 *      サンプル連続で超えたら、床が機体を止めた。読みが解除の水準を下回った時点で速度を「0」にする（ゼロ速度更新）。
 *      無いと、衝撃の積分（加速度計のレンジで頭打ちになり、ノイズも乗る）が地上で +0.2 m/s の誤差を残し、接地検出器が
 *      「動いていない」を見られない（SILS のノイズ n2 で 80 本中 22 本）。
 * ESKF の速度をそのまま写さない理由（毎周期そのままリセットする方式）: 接地→空中のハンドオフの直後、ESKF の
 * 速度は約 1 s 間違っており（SILS: 機体が 1.27 m/s で上昇中に 0.56 m/s）、ホバーでのノイズも ±0.1〜0.2 m/s
 * ある。オブザーバは加速度計の速い成分を残す。
 * 規則 3 の誤差は（オフセット誤差）×（最後のリセットからの時間）: 0.1 m/s^2 で 3 s なら 0.3 m/s。だから
 * ToF なしの降下はこの値を最後のリセットから kVerticalVelocityTrustS の間だけ信用し、推力を重さ未満の上限で抑え、最後の安全策として推力の後半の低下を残し（blind_landing.hpp）、気圧計（次の段階）が
 * fuseExternalVelocity() 経由で規則 3 に速度を融合できるようにしてある（呼び出し側は変わらない）。
 *
 * @design detailed_design.md §3.1 — ToF silent: blind descent by vertical-velocity loop [OK]
 */

#pragma once

#include <cstdint>

#include "flight_limits.hpp"
#include "sf_math.hpp"   // sf::math::kGravity

namespace sf {

/// Which rule produced the current value (telemetry, host tests).
/// 現在の値を作った規則（テレメトリ、ホスト試験）。
enum class VerticalVelocitySource : uint8_t {
    GroundZero = 0,   ///< rule 1: at rest on the ground / 規則 1: 地上で静止
    Eskf       = 1,   ///< rule 2: follows the ESKF (observer) / 規則 2: ESKF に追従（オブザーバ）
    Integrated = 2,   ///< rule 3: accelerometer integral / 規則 3: 加速度計の積分
};

/// Inputs of one update cycle. 1 周期分の入力。
struct VerticalVelocityInput {
    float dt_s;                    ///< Cycle time [s]                            周期 [s]
    bool  at_rest_on_ground;       ///< Rule 1 condition                          規則 1 の条件
    bool  eskf_trusted;            ///< ToF healthy and consumed AND the ESKF vertical state is free
                                   ///< (not pinned at zero by the on-ground hold)
                                   ///< ToF が健全で取り込み済み、かつ ESKF の鉛直状態が自由（接地ホールドで 0 固定でない）
    float eskf_velocity_up;        ///< ESKF vertical velocity, up positive [m/s]  ESKF の鉛直速度（上が正）[m/s]
    float upward_force_raw;        ///< Upward component of the RAW accelerometer reading [m/s^2]
                                   ///< (upwardSpecificForce of specific_force + accel_bias); below
                                   ///< kBlindMinUpwardSpecificForce it is not used
                                   ///< 生の加速度計の読みの上向き成分 [m/s^2]（specific_force + accel_bias の
                                   ///< upwardSpecificForce）。kBlindMinUpwardSpecificForce 未満は使わない
};

/// Tunable settings (defaults: sf::limits; SILS sweeps override them by parameter).
/// 調整値（既定は sf::limits。SILS の掃引はパラメータで上書きする）。
struct VerticalVelocityConfig {
    float handoff_settle_s = limits::kVerticalVelocityHandoffSettleS;   ///< [s] ESKF free this long before it is followed / ESKF が自由になってからこの時間後に従う
    float eskf_tau_s       = limits::kVerticalVelocityEskfTauS;         ///< [s] observer time constant                   / オブザーバの時定数
    float contact_force_mps2 = limits::kBlindTouchdownAccelMps2;        ///< [m/s^2] contact threshold (the touchdown impact) / 接触の閾値（接地の衝撃）
    uint16_t contact_samples = limits::kBlindTouchdownSamples;          ///< consecutive samples above it                     / 閾値を超える連続サンプル数
};

/// Vertical velocity estimator, up positive. 鉛直速度推定器（上が正）。
class VerticalVelocityEstimator {
public:
    void setConfig(const VerticalVelocityConfig& config) { config_ = config; }   ///< Apply settings / 設定を反映


    /// Back to rest (velocity 0, offset 0, rule 1). 静止に戻す（速度 0、オフセット 0、規則 1）。
    void reset()
    {
        velocity_up_ = 0.0f;
        rise_m_      = 0.0f;
        offset_up_   = 0.0f;
        trusted_s_   = 0.0f;
        since_reset_s_ = 0.0f;
        rest_primed_ = false;
        contact_streak_ = 0;
        contact_     = false;
        source_      = VerticalVelocitySource::GroundZero;
    }

    /// Advance by one cycle and return the vertical velocity [m/s], up positive.
    /// 1 周期進めて鉛直速度 [m/s]（上が正）を返す。
    float update(const VerticalVelocityInput& in)
    {
        const bool force_valid = in.upward_force_raw >= limits::kBlindMinUpwardSpecificForce;
        if (in.at_rest_on_ground) {
            velocity_up_ = 0.0f;
            rise_m_      = 0.0f;
            trusted_s_   = 0.0f;
            since_reset_s_ = 0.0f;   // rule 1 is a reset / 規則 1 はリセット
            source_      = VerticalVelocitySource::GroundZero;
            learnOffsetAtRest(in, force_valid);
            return velocity_up_;
        }
        rest_primed_ = false;   // the next rest seeds the offset afresh / 次の静止でオフセットを取り直す
        if (in.eskf_trusted) {   // the contact rule belongs to the integral / 接触の規則は積分に属する
            contact_streak_ = 0;
            contact_        = false;
        }
        trusted_s_ = in.eskf_trusted ? (trusted_s_ + in.dt_s) : 0.0f;
        const float acceleration = force_valid ? (in.upward_force_raw - math::kGravity - offset_up_) : 0.0f;
        if (trusted_s_ >= config_.handoff_settle_s) {
            source_ = VerticalVelocitySource::Eskf;
            since_reset_s_ = 0.0f;   // rule 2 keeps following the ESKF every cycle, so no integral error builds up: the age stays 0 / 規則 2 は毎周期 ESKF に追従し続けるので、積分の誤差が溜まらない: 経過時間は 0 のまま
            followEskf(in, acceleration, force_valid);
        } else {
            source_       = VerticalVelocitySource::Integrated;
            since_reset_s_ += in.dt_s;
            velocity_up_  = clampVelocity(velocity_up_ + acceleration * in.dt_s);
            applyContact(in.upward_force_raw);
        }
        rise_m_ += velocity_up_ * in.dt_s;
        return velocity_up_;
    }

    /// Set the velocity to the ESKF's at once, when the ESKF has just taken a ToF sample again after a silence episode in flight (the
    /// estimator re-acquired the altitude, its vertical state is free and it knows the climb rate). The integral of a long episode carries the
    /// whole accelerometer offset error (SILS scenario ceiling_brake_stale_velocity: offset -0.4 m/s^2, 3.7 m/s after 15 s on a craft that was not climbing), and rule 2
    /// would wait kVerticalVelocityHandoffSettleS before it follows the ESKF; for that time every reader of the integral (the ceiling brake, the
    /// blind descent) would see a stale value. The integral error restarts at zero. The rise is not touched.
    /// ESKF が飛行中の ToF 出力停止のエピソードの後で ToF サンプルを再び取り込んだとき（推定器は高度を取り込み直し、鉛直状態は自由で、上昇率を持っている）、
    /// 速度を ESKF の値へ一度に合わせる。長いエピソードの積分は加速度オフセットの誤差を丸ごと持つ（SILS のシナリオ ceiling_brake_stale_velocity: オフセット −0.4 m/s^2、
    /// 上昇していない機体で 15 s 後に 3.7 m/s）。規則 2 は ESKF に従うまで kVerticalVelocityHandoffSettleS 待つので、その間、積分を読む側
    /// （天井ブレーキ、ToF なしの降下）は古い値を見る。積分の誤差は 0 から数え直す。上昇量には触れない。
    /// @param eskf_velocity_up ESKF vertical velocity, up positive [m/s]  ESKF の鉛直速度（上が正）[m/s]
    void alignToEskf(float eskf_velocity_up)
    {
        velocity_up_    = clampVelocity(eskf_velocity_up);
        since_reset_s_  = 0.0f;
        contact_streak_ = 0;
        contact_        = false;
    }

    /// Pull the integral toward an external velocity measurement (the barometer, next stage):
    /// velocity += gain * (measured - velocity), only while the integral runs (rule 3). The ESKF and
    /// the ground rule own the value otherwise.
    /// 積分を外部の速度測定（気圧計、次の段階）へ寄せる: 速度 += gain × (測定 − 速度)。積分が動いている間
    /// （規則 3）だけ効く。それ以外は ESKF と地上の規則が値を持つ。
    /// @param measured_up External vertical velocity, up positive [m/s]  外部の鉛直速度（上が正）[m/s]
    /// @param gain        Correction fraction per call, clamped to [0, 1]  1 回の補正の割合、[0, 1] に制限
    void fuseExternalVelocity(float measured_up, float gain)
    {
        if (source_ != VerticalVelocitySource::Integrated) return;
        if (gain < 0.0f) gain = 0.0f;
        if (gain > 1.0f) gain = 1.0f;
        velocity_up_ += gain * (measured_up - velocity_up_);
    }

    float velocityUp() const { return velocity_up_; }                  ///< Current value [m/s], up positive / 現在値
    /// Estimated rise since the craft last rested on the ground [m]: the integral of the velocity
    /// (0 at rest, so it starts at the take-off). Used by the ToF health tracker to tell "the craft has left
    /// the floor" from "the craft is still in the sensor's dead zone" (tof_health_tracker.hpp), and to open the
    /// support-thrust window (blind_landing.hpp). It is an accelerometer integral, so an offset error makes it drift on the floor: both
    /// readers use it only in the direction in which an error is harmless (the health tracker counts invalid samples EARLIER, the window
    /// opens EARLIER and its support thrust can only come out LOWER), never as a gate of a safety decision.
    /// 機体が地上に最後に静止してからの上昇量の推定 [m]: 速度の積分（静止中は 0 なので離陸から始まる）。
    /// ToF 健全性トラッカーが「機体は床を離れた」と「まだセンサの不感帯にいる」を区別するのと、支持推力の窓を開くのに使う。加速度計の
    /// 積分なので、オフセット誤差で床の上でもずれる: どちらの使い手も、誤っても害の無い向きにだけ使う（健全性トラッカーは無効サンプルを「早く」
    /// 数え、窓は「早く」開き、支持推力は「低く」しか出ない）。安全判断の判定条件には使わない。
    float riseM() const { return rise_m_; }
    /// Time the value has been an integral since its last reset (rule 1 at rest, or rule 2 following the ESKF) [s]. The
    /// error of the integral grows with it (offset error x time), so the PI of the descent without a ToF trusts the
    /// value only while this is below kVerticalVelocityTrustS.
    /// 値が最後のリセット（規則 1 の静止、または規則 2 の ESKF への追従）から積分であり続けた時間 [s]。積分の誤差は
    /// これとともに増える（オフセット誤差 × 時間）ので、ToF なしの降下の PI は、これが kVerticalVelocityTrustS 未満の間
    /// だけ値を信用する。
    float sinceResetS() const { return since_reset_s_; }
    float offsetUp() const { return offset_up_; }                      ///< Accelerometer offset B [m/s^2] / 加速度計のオフセット B
    VerticalVelocitySource source() const { return source_; }          ///< Rule that made it / 値を作った規則
    float trustedSeconds() const { return trusted_s_; }                ///< Time the ESKF has been trusted [s] / ESKF を信用した時間

private:
    /// Velocity gain of the critically damped 2nd-order observer is 2 / tau (offset gain 1 / tau^2).
    /// 臨界減衰の 2 次オブザーバの速度ゲインは 2 / tau（オフセットのゲインは 1 / tau^2）。
    static constexpr float kObserverDampingFactor = 2.0f;

    /// Contact: a run of readings above the contact threshold = the floor stopped the craft; when the
    /// reading falls back below the release level (0.75 x the threshold) the velocity is set to zero.
    /// 接触: 閾値を超える読みが続いた = 床が機体を止めた。読みが解除の水準（閾値の 0.75 倍）を下回ったら速度を 0 にする。
    void applyContact(float upward_force)
    {
        if (upward_force > config_.contact_force_mps2) {
            if (contact_streak_ < UINT16_MAX) ++contact_streak_;
            if (contact_streak_ >= config_.contact_samples) contact_ = true;
            return;
        }
        contact_streak_ = 0;
        if (contact_ && upward_force < limits::kVerticalVelocityContactReleaseFrac * config_.contact_force_mps2) {
            velocity_up_ = 0.0f;
            contact_     = false;
        }
    }

    /// Rule 1: the offset is the low-passed (reading - g); the first sample at rest seeds it.
    /// 規則 1: オフセットは（読み − g）のローパス値。静止での最初のサンプルが初期値になる。
    void learnOffsetAtRest(const VerticalVelocityInput& in, bool force_valid)
    {
        if (!force_valid) return;
        const float measured = in.upward_force_raw - math::kGravity;
        if (!rest_primed_) {
            offset_up_   = measured;
            rest_primed_ = true;
        } else {
            offset_up_ += (measured - offset_up_) * (in.dt_s / limits::kVerticalVelocityRestTauS);
        }
        offset_up_ = clampOffset(offset_up_);
    }

    /// Rule 2: the complementary observer (v' = a + (2/tau) e, B' = -(1/tau^2) e).
    /// 規則 2: 相補オブザーバ（v' = a + (2/tau) e、B' = −(1/tau^2) e）。
    void followEskf(const VerticalVelocityInput& in, float acceleration, bool force_valid)
    {
        if (!force_valid) {   // no accelerometer: nothing to complement / 加速度計が無い: 補う相手が無い
            velocity_up_ = clampVelocity(in.eskf_velocity_up);
            return;
        }
        const float tau   = config_.eskf_tau_s;
        const float error = in.eskf_velocity_up - velocity_up_;
        velocity_up_ = clampVelocity(velocity_up_ + (acceleration + (kObserverDampingFactor / tau) * error) * in.dt_s);
        offset_up_   = clampOffset(offset_up_ - (error / (tau * tau)) * in.dt_s);
    }

    static float clampVelocity(float v)
    {
        if (v >  limits::kVerticalVelocityLimitMps) return  limits::kVerticalVelocityLimitMps;
        if (v < -limits::kVerticalVelocityLimitMps) return -limits::kVerticalVelocityLimitMps;
        return v;
    }
    static float clampOffset(float b)
    {
        if (b >  limits::kVerticalVelocityOffsetLimit) return  limits::kVerticalVelocityOffsetLimit;
        if (b < -limits::kVerticalVelocityOffsetLimit) return -limits::kVerticalVelocityOffsetLimit;
        return b;
    }

    VerticalVelocityConfig config_;
    float velocity_up_ = 0.0f;                                  // [m/s] up positive
    float rise_m_      = 0.0f;                                  // [m] integral of velocity_up_ since the last rest / 最後の静止からの velocity_up_ の積分
    float offset_up_   = 0.0f;                                  // [m/s^2] accelerometer offset B (raw reading minus truth, up) / 加速度計のオフセット B（読み − 真値、上向き）
    float trusted_s_   = 0.0f;                                  // [s]
    float since_reset_s_ = 0.0f;                                // [s] time as an integral since the last reset / 最後のリセットから積分であり続けた時間
    bool  rest_primed_ = false;                                 // the offset holds a rest measurement / オフセットが静止の測定値を持つ
    uint16_t contact_streak_ = 0;                               // consecutive samples above the contact threshold / 接触の閾値を超えた連続サンプル数
    bool  contact_ = false;                                     // the floor has stopped the craft, release pending / 床が機体を止めた。解除待ち
    VerticalVelocitySource source_ = VerticalVelocitySource::GroundZero;
};

}  // namespace sf
