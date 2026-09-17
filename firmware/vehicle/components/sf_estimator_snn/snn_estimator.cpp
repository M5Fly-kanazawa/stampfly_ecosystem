/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file snn_estimator.cpp
 * @brief SnnEstimator implementation. See the header for scope/design notes.
 *        SnnEstimator の実装。スコープ・設計メモはヘッダ参照。
 */

#include "snn_estimator.hpp"

#include "sf_math.hpp"
#include "snn_weight_init.hpp"

#include <cmath>

namespace sf {

namespace {

// Fixed seeds (task spec design point 5): reproducible, UNTRAINED placeholder
// weights. Different seeds per weight group only so the three matrices are
// not identical to each other.
// 固定シード（タスク仕様 設計方針5）: 再現可能な未学習プレースホルダー重み。
// 重みグループごとに異なるシードを使うのは3つの行列が互いに一致しないため。
constexpr uint32_t kSeedHiddenIn  = 42;
constexpr uint32_t kSeedHiddenRec = 43;
constexpr uint32_t kSeedReadout   = 44;

// Input scaling: normalize accel to ~1g so gyro [rad/s] and accel [g] enter
// the network on comparable scales (placeholder choice, not a learned
// normalization).
// 入力スケーリング: 加速度を~1gに正規化し、ジャイロ[rad/s]と加速度[g]を
// 同程度のスケールでネットワークに入れる（プレースホルダーの選択、学習済み
// 正規化ではない）。
constexpr float kAccelInputScale = 1.0f / math::kGravity;

// Readout scale: maps the linear-readout's raw spike-weighted sum to radians.
// With ~0.1-magnitude random weights over 32 spikes this keeps the attitude
// estimate in a small-angle range instead of immediately saturating asinf()
// in eulerToQuat() — still an arbitrary placeholder (task spec: "動けばよい").
// 読み出しスケール: 線形読み出しの生のスパイク加重和をラジアンへ写す。
// 32スパイク×振幅~0.1のランダム重みなら姿勢推定を小角範囲に収め、
// eulerToQuat() 内で飽和させない。それでも任意のプレースホルダー
// （タスク仕様:「動けばよい」）。
constexpr float kAttitudeReadoutScale = 0.2f;  // [rad] per readout unit

/// Build a quaternion from roll/pitch/yaw [rad] using the aerospace ZYX
/// convention — the inverse of sf::math::Quat::to_euler()'s formulas, kept
/// local here rather than added to sf_math.hpp (not on this task's list of
/// files to change).
/// roll/pitch/yaw[rad] から航空宇宙ZYX規約でクォータニオンを構築する —
/// sf::math::Quat::to_euler() の逆変換。sf_math.hpp
/// （本タスクの変更対象ファイル一覧に無い）へは追加せずここに局所実装する。
math::Quat eulerToQuat(float roll, float pitch, float yaw)
{
    const float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f),   sy = sinf(yaw * 0.5f);
    return math::Quat{
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    };
}

}  // namespace

void SnnEstimator::init()
{
    float w_in[kHiddenDim * kInputDim];
    snn::fillUniformRandom(w_in, kHiddenDim * kInputDim, kSeedHiddenIn);
    float w_rec[kHiddenDim * kHiddenDim];
    snn::fillUniformRandom(w_rec, kHiddenDim * kHiddenDim, kSeedHiddenRec);
    hidden_.setWeights(w_in, w_rec);

    float w_out[kOutputDim * kHiddenDim];
    snn::fillUniformRandom(w_out, kOutputDim * kHiddenDim, kSeedReadout);
    readout_.setWeights(w_out);

    reset();
}

void SnnEstimator::reset()
{
    hidden_.reset();
    state_ = StateEstimate{};
    state_.attitude[0] = 1.0f;  // identity quaternion (w,x,y,z) = (1,0,0,0)
}

void SnnEstimator::predict(const ImuData& imu, float dt)
{
    // dt is unused: the CUBA-LIF decay constants are placeholder values tuned
    // for the fixed 400Hz IMU rate (task spec), not a function of dt.
    // dt は未使用: CUBA-LIF の減衰定数は固定400Hz IMUレート向けのプレース
    // ホルダー値であり（タスク仕様）、dt の関数ではない。
    (void)dt;

    const float input[kInputDim] = {
        imu.gyro[0], imu.gyro[1], imu.gyro[2],
        imu.accel[0] * kAccelInputScale,
        imu.accel[1] * kAccelInputScale,
        imu.accel[2] * kAccelInputScale,
    };
    const float* spikes = hidden_.step(input);
    const float* y = readout_.compute(spikes);

    const float roll  = y[0] * kAttitudeReadoutScale;
    const float pitch = y[1] * kAttitudeReadoutScale;
    const float yaw   = y[2] * kAttitudeReadoutScale;
    const math::Quat q = eulerToQuat(roll, pitch, yaw);
    state_.attitude[0] = q.w; state_.attitude[1] = q.x;
    state_.attitude[2] = q.y; state_.attitude[3] = q.z;

    // ACRO-scope limitation (design point 4): no bias estimate exists, so the
    // rate/specific-force outputs are the raw sensor values, unfiltered.
    // Position/velocity/biases stay at StateEstimate{}'s zero (set in reset()
    // and never written here).
    // ACROスコープの限界（設計方針4）: バイアス推定を持たないため、レート/比力
    // 出力は生センサ値そのまま（未フィルタ）。位置・速度・バイアスは
    // StateEstimate{} のゼロのまま（reset() で設定、ここでは書き換えない）。
    state_.angular_rate[0] = imu.gyro[0];
    state_.angular_rate[1] = imu.gyro[1];
    state_.angular_rate[2] = imu.gyro[2];
    state_.specific_force[0] = imu.accel[0];
    state_.specific_force[1] = imu.accel[1];
    state_.specific_force[2] = imu.accel[2];
    state_.sensor_mask = 0;  // no observation fused (ToF/Flow/Mag/Baro are no-ops)
    state_.timestamp = imu.timestamp;
}

}  // namespace sf
