/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file tilt_consistency.cpp
 * @brief Tilt consistency monitor implementation
 *        傾き整合モニタの実装
 *
 * @design architecture.md INV-3 — detection here, decision in StateManager/StateTask [OK]
 * @design detailed_design.md §3 注10 — attitude/gravity mismatch on the ground       [OK]
 */

#include "tilt_consistency.hpp"

#include <cmath>

namespace sf {

namespace {

constexpr float kRadToDeg   = 57.29577951f;
constexpr float kMinNorm    = 1.0e-6f;   // Guard against dividing by zero / ゼロ除算ガード

/// Euclidean norm of a 3-vector. / 3次元ベクトルのノルム。
float norm3(const float v[3])
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/// "Up" direction in the body frame: R^T * [0,0,-1] for the body→NED quaternion q
/// (the same convention as the ESKF's expected gravity R^T * [0,0,-g], eskf_core.cpp).
/// At rest the accelerometer reads +up (specific force), i.e. ≈ [0,0,-g] when level.
/// body 座標の「上」方向: body→NED クォータニオン q に対する R^T * [0,0,-1]
/// （ESKF の期待重力 R^T * [0,0,-g] と同じ規約、eskf_core.cpp）。静止時の加速度計は
/// +上（比力）を測るので、水平なら ≈ [0,0,-g]。
void estimatedUpInBody(const float q[4], float up[3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    up[0] = -2.0f * (x * z - w * y);
    up[1] = -2.0f * (y * z + w * x);
    up[2] = -(1.0f - 2.0f * (x * x + y * y));
}

}  // namespace

void TiltConsistencyMonitor::init(const TiltConsistencyConfig& config)
{
    config_ = config;
    reset();
}

void TiltConsistencyMonitor::reset()
{
    for (int axis = 0; axis < 3; ++axis) {
        filtered_accel_[axis] = 0.0f;
    }
    filter_age_s_ = 0.0f;
    over_time_s_  = 0.0f;
    angle_deg_    = 0.0f;
    mismatch_     = false;
    verified_     = false;
}

void TiltConsistencyMonitor::update(float dt_s, const float accel[3],
                                    const float quaternion[4], bool on_ground)
{
    // Off the ground the accelerometer measures thrust, not gravity: no judgement, and
    // the filter re-seeds on the next landing.
    // 地上でなければ加速度計は重力でなく推力を測る: 判定せず、次の接地でフィルタを再シード。
    if (!on_ground) {
        reset();
        return;
    }

    // A raw sample outside the plausibility band is a glitch: hold everything (state and
    // timers) and do not let it into the filter.
    // もっともらしさの帯を外れた生サンプルはグリッチ: 状態もタイマも保持し、フィルタにも入れない。
    const float raw_norm = norm3(accel);
    if (raw_norm < config_.norm_min_mps2 || raw_norm > config_.norm_max_mps2) {
        return;
    }

    // Cumulative mean for the first lpf_s seconds, then a first-order low-pass: the filter
    // seeds from the first sample (alpha = 1) without a single noisy sample dominating it.
    // 最初の lpf_s 秒は累積平均、以後は1次ローパス: 初回サンプルで再シード（alpha=1）しつつ、
    // 1サンプルのノイズがフィルタを支配しない。
    filter_age_s_ += dt_s;
    const float window_s = (filter_age_s_ < config_.lpf_s) ? filter_age_s_ : config_.lpf_s;
    const float alpha    = dt_s / window_s;
    for (int axis = 0; axis < 3; ++axis) {
        filtered_accel_[axis] += alpha * (accel[axis] - filtered_accel_[axis]);
    }

    // Judge only once the filter has had a full time constant to settle, and only when the
    // filtered norm is plausible (otherwise hold).
    // フィルタが時定数1つ分整定してから、かつ LPF 後のノルムがもっともらしい時のみ判定する
    // （それ以外は保持）。
    const float filtered_norm = norm3(filtered_accel_);
    const bool  settled       = filter_age_s_ >= config_.lpf_s;
    if (!settled || filtered_norm < config_.norm_min_mps2 || filtered_norm > config_.norm_max_mps2) {
        return;
    }

    float up[3];
    estimatedUpInBody(quaternion, up);
    float cosine = (filtered_accel_[0] * up[0] + filtered_accel_[1] * up[1] +
                    filtered_accel_[2] * up[2]) / (filtered_norm + kMinNorm);
    cosine = std::fmax(-1.0f, std::fmin(1.0f, cosine));
    angle_deg_ = std::acos(cosine) * kRadToDeg;
    verified_  = angle_deg_ <= config_.max_deg;   // positive verdict / 肯定の判定

    // Raise only after the angle has stayed above the threshold for persist_s; clear at once
    // when it drops below.
    // 角度が persist_s の間しきい値を超え続けて初めて立てる。下回ったら即座に下げる。
    if (angle_deg_ > config_.max_deg) {
        over_time_s_ += dt_s;
        mismatch_ = over_time_s_ >= config_.persist_s;
    } else {
        over_time_s_ = 0.0f;
        mismatch_    = false;
    }
}

}  // namespace sf
