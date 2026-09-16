/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file flip_sequencer.cpp
 * @brief Angle-scheduled flip (宙返り) setpoint sequencer — implementation
 *        角度スケジュール型フリップの設定点列生成器 — 実装
 *
 * @design docs/plans/flip-maneuver-plan.md §3.2/§3.5/§7 — see flip_sequencer.hpp [OK]
 */

#include "flip_sequencer.hpp"
#include <cmath>

namespace sf {

// -----------------------------------------------------------------------------
// start — begin a flip: pick the rotation axis/sign from the direction table
// (flip-maneuver-plan.md §3.2), capture the yaw/height to return to, and
// enter Boost.
// start — フリップ開始: 方向表（plan §3.2）から回転軸/符号を選び、復帰先の
// ヨー/高度を取り込み、Boost へ入る。
// -----------------------------------------------------------------------------
void FlipSequencer::start(FlipDirection dir, float yaw_now, float height_now)
{
    direction_ = dir;
    switch (dir) {
    case FlipDirection::Right:   axis_ = 0; sign_ =  1.0f; break;  // roll,  +p
    case FlipDirection::Left:    axis_ = 0; sign_ = -1.0f; break;  // roll,  -p
    case FlipDirection::Back:    axis_ = 1; sign_ =  1.0f; break;  // pitch, +q
    case FlipDirection::Forward: axis_ = 1; sign_ = -1.0f; break;  // pitch, -q
    }
    start_yaw_      = yaw_now;
    start_height_m_ = height_now;
    phi_rad_        = 0.0f;
    rate_cmd_       = 0.0f;
    result_         = FlipResult::Ok;   // stays Ok unless an abort path overwrites it / 打ち切り経路が上書きしない限りOkのまま
    enterPhase(Phase::Boost);
}

// -----------------------------------------------------------------------------
// update — advance one control cycle. Dispatches to the current phase's
// handler; Spin/Brake also accumulate the rotation angle here so every phase
// handler can just read phi_rad_.
// update — 1制御周期進める。現在フェーズのハンドラへ振り分け、Spin/Brake は
// ここで回転角を積算する（各フェーズハンドラは phi_rad_ を読むだけでよい）。
// -----------------------------------------------------------------------------
FlipSequencer::Output FlipSequencer::update(const Input& input)
{
    phase_elapsed_s_ += input.dt;

    switch (phase_) {
    case Phase::Boost:
        return updateBoost(input);
    case Phase::Spin:
        accumulatePhi(input);
        return updateSpin(input);
    case Phase::Brake:
        accumulatePhi(input);
        return updateBrake(input);
    case Phase::Recover:
        return updateRecover(input);
    case Phase::Done:
    case Phase::Idle:
    default:
        return updateDone();
    }
}

bool FlipSequencer::done(FlipResult& result) const
{
    result = result_;
    return phase_ == Phase::Done;
}

float FlipSequencer::rotationAngleDeg() const
{
    return phi_rad_ * kRadToDeg;
}

void FlipSequencer::reset()
{
    // See flip_sequencer.hpp's doc on reset(): result_ is deliberately left
    // untouched (the API/telemetry consumer still needs it after FlipComplete).
    // flip_sequencer.hpp の reset() ドキュメント参照: result_ は意図的に
    // そのまま（FlipComplete 後も API/テレメトリ側が読む必要がある）。
    if (phase_ != Phase::Idle) {
        cooldown_remaining_s_ = config.cooldown_ms * 0.001f;
    }
    phase_ = Phase::Idle;
    phase_elapsed_s_ = 0.0f;
    phi_rad_ = 0.0f;
    rate_cmd_ = 0.0f;
}

// -----------------------------------------------------------------------------
// ready — execution conditions C2-C7 (flip-maneuver-plan.md §3.1). Checked in
// the same order as the plan's table so the first violated condition is the
// one reported.
// ready — 実行条件 C2-C7（plan §3.1）。plan の表と同じ順で判定し、最初に
// 不成立になった条件を報告する。
// -----------------------------------------------------------------------------
bool FlipSequencer::ready(const Input& input, FlipBlockReason& reason) const
{
    if (cooldown_remaining_s_ > 0.0f) {
        cooldown_remaining_s_ -= input.dt;
        if (cooldown_remaining_s_ < 0.0f) cooldown_remaining_s_ = 0.0f;
    }

    if (input.height_m < config.min_height_m) {              // C2
        reason = FlipBlockReason::TooLow;
        return false;
    }
    if (!withinSteadyBounds(input)) {                         // C3/C4
        reason = FlipBlockReason::NotSteady;
        return false;
    }
    if (input.battery_v < config.min_voltage_v) {             // C5
        reason = FlipBlockReason::BatteryLow;
        return false;
    }
    if (!input.estimator_ok || !input.tof_valid) {            // C6
        reason = FlipBlockReason::EstimatorUnhealthy;
        return false;
    }
    if (cooldown_remaining_s_ > 0.0f) {                        // C7
        reason = FlipBlockReason::Cooldown;
        return false;
    }
    reason = FlipBlockReason::None;
    return true;
}

// -----------------------------------------------------------------------------
// withinSteadyBounds — C3 (attitude/rate) and C4 (velocity) of §3.1: the
// craft must be steady enough that the rotation-angle integral can start
// from a clean, level, near-hover baseline.
// withinSteadyBounds — §3.1 の C3（姿勢/角速度）と C4（速度）:
// 回転角の積算をきれいな水平・準ホバー基準から始められるだけ安定している
// 必要がある。
// -----------------------------------------------------------------------------
bool FlipSequencer::withinSteadyBounds(const Input& input) const
{
    const math::Quat q(input.quat[0], input.quat[1], input.quat[2], input.quat[3]);
    const math::Vec3 euler = q.to_euler();

    const float tilt_limit_rad = config.max_tilt_deg * kDegToRad;
    const float rate_limit_rad = config.max_rate_dps * kDegToRad;

    const bool tilted =
        fabsf(euler.x) > tilt_limit_rad || fabsf(euler.y) > tilt_limit_rad;
    const bool spinning =
        fabsf(input.gyro[0]) > rate_limit_rad ||
        fabsf(input.gyro[1]) > rate_limit_rad ||
        fabsf(input.gyro[2]) > rate_limit_rad;
    const bool fast =
        input.horizontal_speed_mps > config.max_hvel_mps ||
        fabsf(input.vertical_velocity_up_mps) > config.max_vvel_mps;

    return !tilted && !spinning && !fast;
}

// -----------------------------------------------------------------------------
// enterPhase — the single writer of phase_, resetting the per-phase clock so
// every phase's timeout/duration check starts from its own entry.
// enterPhase — phase_ の唯一の書き手。フェーズ別クロックをリセットし、
// 各フェーズのタイムアウト/継続時間判定が自身の突入時点から始まるようにする。
// -----------------------------------------------------------------------------
void FlipSequencer::enterPhase(Phase next)
{
    phase_ = next;
    phase_elapsed_s_ = 0.0f;
}

// -----------------------------------------------------------------------------
// accumulatePhi — phi = integral of sign_ * gyro[axis_] dt (plan §3.2), the
// SAME sign convention as the direction table so phi always increases.
// Clamped at 0 to absorb sensor noise before the spin has actually begun.
// accumulatePhi — phi = sign_ * gyro[axis_] の dt 積分（plan §3.2）。
// 方向表と同じ符号規約により phi は常に増加する。スピン開始前のセンサ雑音を
// 吸収するため 0 でクランプする。
// -----------------------------------------------------------------------------
void FlipSequencer::accumulatePhi(const Input& input)
{
    phi_rad_ += sign_ * input.gyro[axis_] * input.dt;
    if (phi_rad_ < 0.0f) phi_rad_ = 0.0f;
}

// -----------------------------------------------------------------------------
// rampRate — move `current` toward `target` at rate_ramp_rps2, never
// overshooting in one step (used for both the Spin ramp-up and the Brake
// ramp-down — plan §7 item 1).
// rampRate — `current` を rate_ramp_rps2 で `target` へ近づける。1ステップで
// 行き過ぎない（Spin の立上げと Brake の立下げ両方に使う — plan §7-1）。
// -----------------------------------------------------------------------------
float FlipSequencer::rampRate(float current, float target, float dt, float ramp_rps2) const
{
    const float max_step = ramp_rps2 * dt;
    const float diff = target - current;
    if (diff >  max_step) return current + max_step;
    if (diff < -max_step) return current - max_step;
    return target;
}

float FlipSequencer::axisRateDps() const
{
    return (axis_ == 0) ? config.rate_roll_dps : config.rate_pitch_dps;
}

// -----------------------------------------------------------------------------
// brakeAngleDeg — phi_brake = 360 - (omega_f^2/(2*alpha_cmd) + omega_f*t_lag)
// (flip-maneuver-plan.md §7 item 1), evaluated from the CURRENT MEASURED rate
// every cycle so a battery-sagged (slower) spin needs less stopping angle and
// automatically starts braking later — closer to 360 deg (plan §3.2).
// brakeAngleDeg — phi_brake = 360 - (omega_f^2/(2*alpha_cmd) + omega_f*t_lag)
// （plan §7-1）を「計測」レートから毎周期評価する。電池電圧低下で回転が遅い
// ときは必要な制動角が小さくなり、自動的により360°に近い角度まで
// 減速開始を遅らせる（plan §3.2）。
// -----------------------------------------------------------------------------
float FlipSequencer::brakeAngleDeg(float measured_rate_dps) const
{
    const float omega_f_rad = measured_rate_dps * kDegToRad;   // [rad/s]
    // Achievable deceleration = brake_margin * brake ramp (Config doc).
    // 達成できる減速度 = brake_margin × 減速ランプ（Config の説明参照）。
    const float alpha_brake = config.brake_margin * config.brake_ramp_rps2;   // [rad/s^2]
    const float t_lag       = config.motor_lag_ms * 0.001f;    // [s]
    const float delta_rad = (omega_f_rad * omega_f_rad) / (2.0f * alpha_brake) +
                             omega_f_rad * t_lag;
    return 360.0f - delta_rad * kRadToDeg;
}

// -----------------------------------------------------------------------------
// spinThrustN — the 3-window thrust schedule of plan §3.2/§5.3: high torque
// headroom (T_spin_hi) during the accel window and again just before the
// brake, low collective (T_lo) while coasting inverted in between.
// spinThrustN — plan §3.2/§5.3 の3区間推力スケジュール: 加速窓とブレーキ
// 直前は高トルク余裕（T_spin_hi）、その間の反転惰性中は低集合推力（T_lo）。
// -----------------------------------------------------------------------------
float FlipSequencer::spinThrustN(float phi_deg, float phi_brake_deg) const
{
    const float phi_b_deg = phi_brake_deg - kBrakePrepMarginDeg;
    const float thrust_hi = config.thrust_spin_hi_ratio * config.max_thrust_n;
    if (phi_deg < config.angle_a_deg) return thrust_hi;
    if (phi_deg < phi_b_deg)          return config.thrust_lo_n;
    return thrust_hi;
}

// -----------------------------------------------------------------------------
// outputAttitudeLevel — shared Boost/Recover/Done output shape: attitude loop
// engaged, level (roll_sp=pitch_sp=0), yaw held, caller-supplied thrust.
// outputAttitudeLevel — Boost/Recover/Done 共通の出力形: 姿勢ループ係合、
// 水平（roll_sp=pitch_sp=0）、ヨー保持、呼び出し側指定の推力。
// -----------------------------------------------------------------------------
FlipSequencer::Output FlipSequencer::outputAttitudeLevel(float thrust_n) const
{
    Output out{};
    out.attitude_loop = true;
    out.roll_sp = 0.0f;
    out.pitch_sp = 0.0f;
    out.hold_yaw = true;
    out.thrust_n = thrust_n;
    return out;
}

// -----------------------------------------------------------------------------
// outputRotating — shared Spin/Brake output shape: rate loop driven directly
// on the rotation axis, the other two axes commanded to zero.
// outputRotating — Spin/Brake 共通の出力形: レートループを回転軸に直接指令し、
// 他の2軸はゼロを指令する。
// -----------------------------------------------------------------------------
FlipSequencer::Output FlipSequencer::outputRotating(float rate_cmd, float thrust_n) const
{
    Output out{};
    out.attitude_loop = false;
    out.rate_sp[0] = (axis_ == 0) ? rate_cmd : 0.0f;
    out.rate_sp[1] = (axis_ == 1) ? rate_cmd : 0.0f;
    out.rate_sp[2] = 0.0f;
    out.hold_yaw = false;
    out.thrust_n = thrust_n;
    return out;
}

// -----------------------------------------------------------------------------
// updateBoost — P1 (plan §3.2): level attitude, boosted thrust, for
// boost_ms. The Boost->Spin transition takes effect on the NEXT update()
// call (same one-cycle-deferred-transition idiom pid_controller.cpp already
// uses for its capture_alt_/capture_pos_ flags) — this cycle still returns
// a pure Boost-shaped output.
// updateBoost — P1（plan §3.2）: 水平姿勢・増強推力を boost_ms の間。
// Boost→Spin の遷移は「次の」update() 呼び出しで有効になる
// （pid_controller.cpp の capture_alt_/capture_pos_ フラグと同じ
// 1周期遅延遷移の作法）— この周期は純粋な Boost 形の出力を返す。
// -----------------------------------------------------------------------------
FlipSequencer::Output FlipSequencer::updateBoost(const Input&)
{
    const float boost_s = config.boost_ms * 0.001f;
    if (phase_elapsed_s_ >= boost_s) {
        enterPhase(Phase::Spin);
    }
    return outputAttitudeLevel(config.thrust_boost_ratio * config.max_thrust_n);
}

// -----------------------------------------------------------------------------
// updateSpin — P2 (plan §3.2/§3.5): ramp the rate command toward the flip
// peak, schedule thrust by rotation angle, and watch for the three ways out:
// the gyro-abort limit (straight to Brake), reaching phi_brake (normal entry
// to Brake), or the spin timeout (straight to Recover — no point braking a
// rotation that never picked up).
// updateSpin — P2（plan §3.2/§3.5）: レート指令をフリップのピークへランプし、
// 回転角で推力をスケジュールし、3通りの脱出を監視する: ジャイロ異常上限
// （即 Brake へ）、phi_brake 到達（通常の Brake 突入）、spin タイムアウト
// （即 Recover へ — 進まなかった回転を減速する意味はない）。
// -----------------------------------------------------------------------------
FlipSequencer::Output FlipSequencer::updateSpin(const Input& input)
{
    const float target_rate = sign_ * axisRateDps() * kDegToRad;
    rate_cmd_ = rampRate(rate_cmd_, target_rate, input.dt, config.rate_ramp_rps2);

    const float measured_dps = fabsf(input.gyro[axis_]) * kRadToDeg;
    const float thrust_hi = config.thrust_spin_hi_ratio * config.max_thrust_n;

    if (measured_dps > config.gyro_abort_dps) {
        result_ = FlipResult::AbortedGyroLimit;
        enterPhase(Phase::Brake);
        return outputRotating(rate_cmd_, thrust_hi);
    }

    const float phi_deg = phi_rad_ * kRadToDeg;
    const float phi_brake_deg = brakeAngleDeg(measured_dps);
    const float thrust_n = spinThrustN(phi_deg, phi_brake_deg);
    const float spin_timeout_s = config.spin_timeout_ms * 0.001f;

    if (phi_deg >= phi_brake_deg) {
        enterPhase(Phase::Brake);
    } else if (phase_elapsed_s_ >= spin_timeout_s) {
        result_ = FlipResult::AbortedSpinTimeout;
        enterPhase(Phase::Recover);   // rotation never picked up — nothing to brake / 回転が進んでおらず減速の意味がない
    }
    return outputRotating(rate_cmd_, thrust_n);
}

// -----------------------------------------------------------------------------
// updateBrake — P3 (plan §3.2/§3.3): ramp the rate command toward zero at
// constant thrust_spin_hi. Hands off to Recover once the rotation has turned
// far enough AND slowed down enough, or unconditionally past handoff_force_deg.
// updateBrake — P3（plan §3.2/§3.3）: レート指令を一定推力 thrust_spin_hi の
// もとゼロへランプ。十分回りかつ十分減速したら、または無条件で
// handoff_force_deg を過ぎたら Recover へ引き渡す。
// -----------------------------------------------------------------------------
FlipSequencer::Output FlipSequencer::updateBrake(const Input& input)
{
    rate_cmd_ = rampRate(rate_cmd_, 0.0f, input.dt, config.brake_ramp_rps2);

    const float phi_deg = phi_rad_ * kRadToDeg;
    const float measured_dps = fabsf(input.gyro[axis_]) * kRadToDeg;
    const bool settled = phi_deg >= config.handoff_min_deg &&
                         measured_dps <= config.handoff_rate_dps;
    const bool forced  = phi_deg >= config.handoff_force_deg;
    if (settled || forced) {
        enterPhase(Phase::Recover);
    }
    return outputRotating(rate_cmd_, config.thrust_spin_hi_ratio * config.max_thrust_n);
}

// -----------------------------------------------------------------------------
// updateRecover — P4 (plan §3.2/§3.5): level attitude, boosted thrust, until
// the craft is climbing again (vz>=0) or recover_timeout_ms elapses (still
// reaches Done either way — plan §3.5: "Doneにはする").
// updateRecover — P4（plan §3.2/§3.5）: 水平姿勢・増強推力を、上昇に転じる
// （vz>=0）か recover_timeout_ms 経過まで（いずれにせよ Done にはする —
// plan §3.5「Doneにはする」）。
// -----------------------------------------------------------------------------
FlipSequencer::Output FlipSequencer::updateRecover(const Input& input)
{
    const float recover_s = config.recover_timeout_ms * 0.001f;
    if (input.vertical_velocity_up_mps >= 0.0f) {
        enterPhase(Phase::Done);
    } else if (phase_elapsed_s_ >= recover_s) {
        result_ = FlipResult::AbortedRecoverTimeout;
        enterPhase(Phase::Done);
    }
    return outputAttitudeLevel(recoverThrustN(input));
}

// -----------------------------------------------------------------------------
// recoverThrustN — boost only once (nearly) level; keep the spin-window
// collective while the attitude loop is still bringing the craft back
// (Config::recover_boost_tilt_deg). Tilt from the estimator quaternion:
// cos(tilt) = R33 = 1 - 2(x^2 + y^2).
// recoverThrustN — ほぼ水平に戻ってから増強し、姿勢ループが戻している間は
// 回転窓の集合推力に留める（Config::recover_boost_tilt_deg）。傾きは推定
// クォータニオンから cos(tilt) = R33 = 1 − 2(x² + y²) で求める。
// -----------------------------------------------------------------------------
float FlipSequencer::recoverThrustN(const Input& input) const
{
    const float qx = input.quat[1];
    const float qy = input.quat[2];
    const float cos_tilt = 1.0f - 2.0f * (qx * qx + qy * qy);
    const float cos_limit = std::cos(config.recover_boost_tilt_deg * kDegToRad);
    const float ratio = (cos_tilt >= cos_limit) ? config.thrust_boost_ratio
                                                : config.thrust_spin_hi_ratio;
    return ratio * config.max_thrust_n;
}

// -----------------------------------------------------------------------------
// updateDone — terminal steady output (identical shape to Recover's), held
// for however many cycles pass before the caller sees active()==false via
// reset() — see flip_sequencer.hpp's doc on active()/reset().
// updateDone — 終端の定常出力（Recoverと同じ形）。呼び出し側が reset() で
// active()==false を見るまで何周期でも保持する — flip_sequencer.hpp の
// active()/reset() ドキュメント参照。
// -----------------------------------------------------------------------------
FlipSequencer::Output FlipSequencer::updateDone() const
{
    return outputAttitudeLevel(config.thrust_boost_ratio * config.max_thrust_n);
}

}  // namespace sf
