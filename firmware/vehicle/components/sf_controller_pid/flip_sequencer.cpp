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
    cmd_phi_rad_ = 0.0f;
    gyro_over_count_ = 0;
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
    cmd_phi_rad_ = 0.0f;
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
// brakeDue / brakeStartDeg — the PLANNED rate profile. It is decided from the
// COMMAND alone, no measurement: the angle the command has swept so far
// (cmd_phi_rad_) plus the angle the ramp-down from the current command rate would
// still sweep, rate^2/(2*a_down), reaches 2*pi exactly when the ramp-down must start.
// While the ramp-up is still running this makes the profile a triangle (peak
// sqrt(2*pi/(1/(2*a_up) + 1/(2*a_down)))); once the rate limit is hit it adds the
// plateau that fills the rest of the area. The swept angle is integrated per cycle, so
// the area is exact for any control period. A lagging actuator only shifts the measured
// waveform in time (flip-maneuver-plan.md section 5.7).
// brakeDue / brakeStartDeg — 「計画」レートプロファイル。計測を使わず「指令」だけで決める:
// 指令がここまでに掃いた角（cmd_phi_rad_）と、現在の指令レートからの減速ランプが今後
// 掃く角 rate²/(2·a_down) の和がちょうど 2π になる時点が、減速ランプの開始点。加速ランプの
// 途中でこれに達すれば三角形（ピーク sqrt(2π/(1/(2 a_up) + 1/(2 a_down)))）、レート上限に
// 先に達すれば残りの面積を埋める平坦部が付く。掃いた角は周期ごとに積分するので、どの制御周期
// でも面積は正確。アクチュエータの遅れは計測波形を時間方向にずらすだけ
// （flip-maneuver-plan.md 5.7 節）。
// -----------------------------------------------------------------------------
bool FlipSequencer::brakeDue() const
{
    const float stop_angle_rad = (rate_cmd_ * rate_cmd_) / (2.0f * config.brake_ramp_rps2);
    return cmd_phi_rad_ + stop_angle_rad >= 2.0f * kPi;
}

float FlipSequencer::brakeStartDeg() const
{
    const float stop_angle_rad = (rate_cmd_ * rate_cmd_) / (2.0f * config.brake_ramp_rps2);
    return (2.0f * kPi - stop_angle_rad) * kRadToDeg;
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
// gyro saturation (kGyroAbortConsecutiveSamples cycles above gyro_abort_dps,
// straight to Brake), reaching phi_brake (normal entry to Brake), or the spin
// timeout (straight to Recover — no point braking a rotation that never
// picked up).
// updateSpin — P2（plan §3.2/§3.5）: レート指令をフリップのピークへランプし、
// 回転角で推力をスケジュールし、3通りの脱出を監視する: ジャイロ飽和
// （gyro_abort_dps 超えが kGyroAbortConsecutiveSamples 周期連続、即 Brake へ）、
// phi_brake 到達（通常の Brake 突入）、spin タイムアウト（即 Recover へ —
// 進まなかった回転を減速する意味はない）。
// -----------------------------------------------------------------------------
FlipSequencer::Output FlipSequencer::updateSpin(const Input& input)
{
    const float target_rate = sign_ * axisRateDps() * kDegToRad;
    rate_cmd_ = rampRate(rate_cmd_, target_rate, input.dt, config.rate_ramp_rps2);
    cmd_phi_rad_ += sign_ * rate_cmd_ * input.dt;

    const float measured_dps = fabsf(input.gyro[axis_]) * kRadToDeg;
    const float thrust_hi = config.thrust_spin_hi_ratio * config.max_thrust_n;

    if (gyroSaturated(measured_dps)) {
        markAbort(FlipResult::AbortedGyroLimit);
        enterPhase(Phase::Brake);
        return outputRotating(rate_cmd_, thrust_hi);
    }

    const float phi_deg = phi_rad_ * kRadToDeg;
    const float thrust_n = spinThrustN(phi_deg, brakeStartDeg());
    const float spin_timeout_s = config.spin_timeout_ms * 0.001f;

    if (brakeDue()) {
        if (phi_rad_ >= kMinMeasuredToCommandedAngle * cmd_phi_rad_) {
            enterPhase(Phase::Brake);
        } else {
            markAbort(FlipResult::AbortedSpinTimeout);
            enterPhase(Phase::Recover);   // the command swept 360 deg but the craft did not turn / 指令は 360° 掃いたが機体は回っていない
        }
    } else if (phase_elapsed_s_ >= spin_timeout_s) {
        markAbort(FlipResult::AbortedSpinTimeout);
        enterPhase(Phase::Recover);   // rotation never picked up — nothing to brake / 回転が進んでおらず減速の意味がない
    }
    return outputRotating(rate_cmd_, thrust_n);
}

// -----------------------------------------------------------------------------
// markAbort — record WHY the flip was aborted. First cause wins: a later
// consequence (e.g. the Recover timeout after a gyro abort) must not hide the
// event that started the abort in the reported result.
// markAbort — フリップが「なぜ」打ち切られたかを記録する。最初の原因を優先:
// 後続の帰結（例: ジャイロ打ち切り後の Recover タイムアウト）が、打ち切りを
// 始めた事象を報告結果から隠してはならない。
// -----------------------------------------------------------------------------
void FlipSequencer::markAbort(FlipResult cause)
{
    if (result_ == FlipResult::Ok) {
        result_ = cause;
    }
}

// -----------------------------------------------------------------------------
// gyroSaturated — true once the measured rate has exceeded the abort limit
// (clamped to the sensor range) for kGyroAbortConsecutiveSamples consecutive
// cycles. A cycle below the limit restarts the count.
// gyroSaturated — 計測レートが打ち切り上限（センサレンジでクランプ）を
// kGyroAbortConsecutiveSamples 周期連続で超えたら true。上限以下の周期が
// 1回あればカウントは最初からやり直し。
// -----------------------------------------------------------------------------
bool FlipSequencer::gyroSaturated(float measured_dps)
{
    const float limit_dps = fminf(config.gyro_abort_dps, kGyroRangeDps);
    gyro_over_count_ = (measured_dps > limit_dps) ? gyro_over_count_ + 1 : 0;
    return gyro_over_count_ >= kGyroAbortConsecutiveSamples;
}

// -----------------------------------------------------------------------------
// updateBrake — P3 (plan §3.2/§3.3/§3.5): ramp the rate command toward zero at
// constant thrust_spin_hi. ALWAYS terminates: hands off to Recover as soon as
// the measured rate is below handoff_rate_dps (at any rotation angle, so an
// aborted flip that stopped part-way is recovered too), or past
// handoff_force_deg, or after brake_timeout_ms (the latter two are backstops;
// only the timeout marks the result AbortedBrakeTimeout).
// updateBrake — P3（plan §3.2/§3.3/§3.5）: レート指令を一定推力 thrust_spin_hi
// のもとゼロへランプ。「必ず終了する」: 計測レートが handoff_rate_dps 未満に
// なったら（回転角に関係なく。途中で止まった打ち切りも回復させる）、または
// handoff_force_deg を過ぎたら、または brake_timeout_ms 経過で Recover へ
// 引き渡す（後ろ2つは安全弁で、結果を AbortedBrakeTimeout にするのは
// タイムアウトのみ）。
// -----------------------------------------------------------------------------
FlipSequencer::Output FlipSequencer::updateBrake(const Input& input)
{
    rate_cmd_ = rampRate(rate_cmd_, 0.0f, input.dt, config.brake_ramp_rps2);
    cmd_phi_rad_ += sign_ * rate_cmd_ * input.dt;

    const float phi_deg = phi_rad_ * kRadToDeg;
    const float measured_dps = fabsf(input.gyro[axis_]) * kRadToDeg;
    const bool stopped = measured_dps <= config.handoff_rate_dps;
    const bool forced  = phi_deg >= config.handoff_force_deg;
    const bool timed_out = phase_elapsed_s_ >= config.brake_timeout_ms * 0.001f;
    if (stopped || forced) {
        enterPhase(Phase::Recover);
    } else if (timed_out) {
        markAbort(FlipResult::AbortedBrakeTimeout);
        enterPhase(Phase::Recover);
    }
    return outputRotating(rate_cmd_, config.thrust_spin_hi_ratio * config.max_thrust_n);
}

// -----------------------------------------------------------------------------
// updateRecover — P4 (plan §3.2/§3.5): level attitude, thrust managed by
// recoverThrustN(), until the craft is back within recover_boost_tilt_deg of
// level AND climbing again (vz>=0), or recover_timeout_ms elapses (still
// reaches Done either way — plan §3.5: "Doneにはする"). Requiring "levelled"
// matters for an aborted flip: it enters Recover tilted/inverted while vz may
// still be >=0 from the Boost climb, and must not be released to the normal
// law before the attitude loop has actually righted it.
// updateRecover — P4（plan §3.2/§3.5）: 水平姿勢、推力は recoverThrustN() で
// 管理し、水平から recover_boost_tilt_deg 以内に戻り「かつ」上昇に転じる
// （vz>=0）か recover_timeout_ms 経過まで（いずれにせよ Done にはする —
// plan §3.5「Doneにはする」）。「水平に戻った」を要求するのは打ち切り後の
// ため: 傾いた/反転した状態で Recover に入り、Boost の上昇で vz が
// まだ >=0 のことがあり、姿勢ループが実際に立て直す前に通常則へ放して
// はならない。
// -----------------------------------------------------------------------------
FlipSequencer::Output FlipSequencer::updateRecover(const Input& input)
{
    const float recover_s = config.recover_timeout_ms * 0.001f;
    const bool levelled = tiltRad(input.quat) <= config.recover_boost_tilt_deg * kDegToRad;
    if (levelled && input.vertical_velocity_up_mps >= 0.0f) {
        enterPhase(Phase::Done);
    } else if (phase_elapsed_s_ >= recover_s) {
        markAbort(FlipResult::AbortedRecoverTimeout);
        enterPhase(Phase::Done);
    }
    return outputAttitudeLevel(recoverThrustN(input));
}

// -----------------------------------------------------------------------------
// recoverThrustN — collective thrust while the attitude loop brings the craft
// back, by tilt: near level (<= recover_boost_tilt_deg) boost; up to angle_a_deg
// (the same angle where the spin's accel window ends: the thrust vector still
// points mostly up) the spin-window collective thrust_spin_hi, which keeps
// differential-torque headroom for the attitude loop; beyond angle_a_deg it is
// scaled down in proportion to the upward component cos(tilt) (= R33) to
// thrust_lo_n at 90 deg and stays there while inverted, because inverted thrust
// points DOWN. The attitude P law (kp = 5 /s) is slow and needs almost no
// torque, so the low collective does not slow the righting: SILS
// flip_abort_recover (2026-10-02, stopped at 168 deg tilt) reached the same
// tilt with 0.03 N as with 0.2 N while falling less fast, and a constant
// thrust_spin_hi accelerated the 2026-10-02 hardware craft at ~1.9 g.
// recoverThrustN — 姿勢ループが機体を戻している間の集合推力を傾きで決める:
// ほぼ水平（<= recover_boost_tilt_deg）は増強。angle_a_deg（スピンの加速窓が
// 終わるのと同じ角度: 推力ベクトルがまだ概ね上向き）までは回転窓の集合推力
// thrust_spin_hi で、姿勢ループの差動トルク余裕を残す。angle_a_deg を超えたら
// 上向き成分 cos(tilt)（= R33）に比例して縮め、90° で thrust_lo_n、反転中は
// それを保つ（反転中の推力は「下向き」のため）。姿勢の P 則（kp = 5 /s）は遅く、
// ほとんどトルクを要しないので、集合推力を低くしても立て直しは遅くならない:
// SILS flip_abort_recover（2026-10-02, 傾き 168° で停止）は 0.2 N でも 0.03 N でも
// 同じ傾きに戻り、0.03 N の方が落下は遅かった。一定の thrust_spin_hi だと
// 2026-10-02 の実機は約 1.9 g で加速した。
// -----------------------------------------------------------------------------
float FlipSequencer::recoverThrustN(const Input& input) const
{
    const float tilt = tiltRad(input.quat);
    if (tilt <= config.recover_boost_tilt_deg * kDegToRad) {
        return config.thrust_boost_ratio * config.max_thrust_n;
    }
    const float thrust_hi_n = config.thrust_spin_hi_ratio * config.max_thrust_n;
    const float full_cos = std::cos(config.angle_a_deg * kDegToRad);
    const float upward = fminf(fmaxf(std::cos(tilt) / full_cos, 0.0f), 1.0f);   // 1 up to angle_a, 0 once inverted / angle_a まで 1、反転で 0
    return config.thrust_lo_n + (thrust_hi_n - config.thrust_lo_n) * upward;
}

// -----------------------------------------------------------------------------
// tiltRad / levelError — attitude measures that are valid at ANY attitude, from
// the gravity direction in the body frame: g_b = third row of R(q) =
// (2(xz - wy), 2(yz + wx), 1 - 2(x^2 + y^2)). Level: g_b = (0, 0, 1).
// The shortest rotation taking g_b to +z has axis a = g_b x z = (gy, -gx, 0)
// and angle atan2(|a|, gz), giving roll_err = angle * ax/|a| (= Euler roll for
// small tilts) and pitch_err = angle * ay/|a| (= Euler pitch).
// tiltRad / levelError — 機体座標の重力方向から、「どの姿勢でも」有効な姿勢
// 尺度: g_b = R(q) の第3行 = (2(xz − wy), 2(yz + wx), 1 − 2(x² + y²))。水平なら
// g_b = (0, 0, 1)。g_b を +z へ運ぶ最短回転は軸 a = g_b × z = (gy, −gx, 0)、
// 角 atan2(|a|, gz) で、roll_err = 角 × ax/|a|（小さな傾きでオイラー roll と
// 一致）、pitch_err = 角 × ay/|a|（同 pitch）。
// -----------------------------------------------------------------------------
float FlipSequencer::tiltRad(const float quat[4])
{
    const float cos_tilt = 1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]);
    return std::acos(fminf(fmaxf(cos_tilt, -1.0f), 1.0f));
}

void FlipSequencer::levelError(const float quat[4], float& roll_rad, float& pitch_rad)
{
    const float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    const float gx = 2.0f * (x * z - w * y);
    const float gy = 2.0f * (y * z + w * x);
    const float gz = 1.0f - 2.0f * (x * x + y * y);
    const float axis_norm = std::sqrt(gx * gx + gy * gy);   // = sin(tilt)
    if (axis_norm < kAxisUndefinedEpsilon) {
        // Level (no error) or exactly inverted (axis undefined: roll over).
        // 水平（誤差なし）またはちょうど反転（軸不定: ロールで戻す）。
        roll_rad  = (gz < 0.0f) ? kPi : 0.0f;
        pitch_rad = 0.0f;
        return;
    }
    const float angle = std::atan2(axis_norm, gz);
    roll_rad  = angle * gy / axis_norm;
    pitch_rad = -angle * gx / axis_norm;
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
