/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file snn_controller.cpp
 * @brief SnnController implementation. See the header for network/design notes.
 *        SnnController の実装。ネットワーク・設計メモはヘッダ参照。
 */

#include "snn_controller.hpp"

#include "sf_math.hpp"
#include "snn_weight_init.hpp"

namespace sf {

namespace {

// Fixed seeds (task spec design point 5): reproducible, UNTRAINED placeholder
// weights, distinct from the estimator's seeds (snn_estimator.cpp) so the two
// networks' random weights are not identical.
// 固定シード（タスク仕様 設計方針5）: 再現可能な未学習プレースホルダー重み。
// 推定器のシード（snn_estimator.cpp）と別値にし、2つのネットワークの
// ランダム重みが一致しないようにする。
constexpr uint32_t kSeedHiddenIn = 142;
constexpr uint32_t kSeedReadout  = 144;

// -----------------------------------------------------------------------
// Readout -> physical-units placeholder mapping.
//
// An UNTRAINED network's raw readout sum is tiny and centered near zero (16
// spikes x ~0.1-magnitude weights), so using it directly as thrust would
// almost always command ~0 N and the motors would never spin — defeating
// this task's "confirm actuation reaches the motor" goal. So the thrust
// readout is added to a fixed hover-thrust PRIOR (mg, no empirical
// correction — PidController's hover_thrust_ correction factor is a learned
// tuning this network does not have) instead of being the sole source of
// thrust. Torque has no such prior (zero-torque = level, a sane default).
// Both are scaled to roughly the same order of magnitude as PidController's
// own limits (max_thrust_, max_roll_pitch_torque_) so a saturated readout
// cannot command something physically absurd. ALL FOUR CONSTANTS BELOW ARE
// HAND-PICKED PLACEHOLDERS with no learning behind them — Stage 3 replaces
// this whole mapping once the network's own output scale is learned
// end-to-end (see the task report's hand-off notes).
//
// 読み出し→物理単位のプレースホルダー写像。
//
// 未学習ネットワークの生の読み出し和は0近傍の微小値（16スパイク×振幅~0.1の
// 重み）で、そのまま推力に使うとほぼ常に約0Nとなりモータが回らない —
// 本タスクの「アクチュエータまで出力が届くことを確認する」ゴールに反する。
// そこで推力の読み出しは「唯一の推力源」ではなく、固定のホバー推力事前値
// （mg、経験補正なし — PidController の hover_thrust_ 補正係数は本ネット
// ワークが持たない学習済みチューニング）に加算する。トルクにはそのような
// 事前値は無い（トルク0=水平、妥当な既定）。どちらも PidController 自身の
// 上限（max_thrust_, max_roll_pitch_torque_）と同程度の桁にスケールし、
// 読み出しが飽和しても物理的にあり得ない値にならないようにする。以下の
// 4定数は全て学習の裏付けが無い手作りプレースホルダー — ネットワーク自身の
// 出力スケールをend-to-endで学習したらStage3でこの写像全体を置き換える
// （タスク報告の申し送り参照）。
// -----------------------------------------------------------------------
constexpr float kHoverThrustN      = 0.037f * math::kGravity;  // mg [N], no hover correction
constexpr float kThrustReadoutGain = 0.05f;    // [N per readout unit]
constexpr float kTorqueReadoutGain = 1.0e-3f;  // [Nm per readout unit]
constexpr float kMaxThrustN        = 0.672f;   // matches PidController::max_thrust_
constexpr float kMaxTorqueNm       = 5.0e-3f;  // matches PidController::max_roll_pitch_torque_ order

float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

void SnnController::init()
{
    float w_in[kHiddenDim * kInputDim];
    snn::fillUniformRandom(w_in, kHiddenDim * kInputDim, kSeedHiddenIn);
    hidden_.setWeights(w_in);  // non-recurrent: no w_rec to set

    float w_out[kOutputDim * kHiddenDim];
    snn::fillUniformRandom(w_out, kOutputDim * kHiddenDim, kSeedReadout);
    readout_.setWeights(w_out);

    reset();
}

void SnnController::reset()
{
    hidden_.reset();
}

ControlOutput SnnController::compute(const StateEstimate& state,
                                     const CommandSetpoint& setpoint,
                                     float dt)
{
    // dt is unused: same placeholder-decay-constant rationale as
    // SnnEstimator::predict() (fixed 400Hz control rate).
    // dt は未使用: SnnEstimator::predict() と同じ理由（固定400Hz制御レート
    // 向けのプレースホルダー減衰定数）。
    (void)dt;

    const math::Quat q{state.attitude[0], state.attitude[1],
                       state.attitude[2], state.attitude[3]};
    const math::Vec3 euler = q.to_euler();

    const float input[kInputDim] = {
        state.angular_rate[0], state.angular_rate[1], state.angular_rate[2],
        euler.x, euler.y, euler.z,
        setpoint.roll, setpoint.pitch, setpoint.yaw, setpoint.throttle,
    };
    const float* spikes = hidden_.step(input);
    const float* y = readout_.compute(spikes);

    ControlOutput out{};
    out.thrust    = clampf(kHoverThrustN + y[0] * kThrustReadoutGain, 0.0f, kMaxThrustN);
    out.torque[0] = clampf(y[1] * kTorqueReadoutGain, -kMaxTorqueNm, kMaxTorqueNm);
    out.torque[1] = clampf(y[2] * kTorqueReadoutGain, -kMaxTorqueNm, kMaxTorqueNm);
    out.torque[2] = clampf(y[3] * kTorqueReadoutGain, -kMaxTorqueNm, kMaxTorqueNm);
    // Stage 1: the network has no explicit cascade setpoints to export (see
    // class comment) — zero placeholders rather than a value that would
    // imply a meaning the network does not actually have.
    // Stage1: ネットワークは輸出すべき明示的なカスケード設定点を持たない
    // （クラスコメント参照）— 存在しない意味を暗示する値を出すより
    // ゼロプレースホルダーとする。
    out.rate_ref[0] = out.rate_ref[1] = out.rate_ref[2] = 0.0f;
    out.angle_ref[0] = out.angle_ref[1] = 0.0f;
    out.timestamp = state.timestamp;
    return out;
}

}  // namespace sf
