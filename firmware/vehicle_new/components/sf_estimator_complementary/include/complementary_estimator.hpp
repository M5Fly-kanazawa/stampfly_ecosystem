/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file complementary_estimator.hpp
 * @brief Mahony-style complementary filter — a second IEstimator implementation.
 *        Mahony 型相補フィルタ — 2つ目の IEstimator 実装。
 *
 * Proves the SIL is algorithm-independent (RESET_PLAN P2): swapping ESKF →
 * complementary filter requires ZERO change to the SIL bench or the firmware
 * tasks — only the `estimator.type` parameter. This estimator computes what
 * stabilized hover and altitude-hold need: attitude (gyro integration corrected
 * toward the accelerometer's gravity direction), the body angular rate (= gyro),
 * and the vertical channel (altitude + vertical velocity from the accelerometer
 * integral, anchored by the barometer). Horizontal position/velocity are NOT
 * estimated; those StateEstimate fields stay zero. ~90 lines of actual math.
 *
 * SIL がアルゴリズム非依存であることを実証する（RESET_PLAN P2）: ESKF → 相補フィルタ
 * の差し替えは SIL ベンチもファームタスクも一切変更せず、`estimator.type` パラメータ
 * だけで行える。本推定器は安定化ホバーと高度保持に必要な分を計算する: 姿勢（ジャイロ積分を
 * 加速度計の重力方向へ補正）、機体角速度（= ジャイロ）、鉛直チャネル（加速度積分による
 * 高度＋鉛直速度を気圧で固定）。水平位置・速度は推定しない（該当フィールドは 0）。
 *
 * @design requirements.md §10 — replaceable estimation                  [--]
 * @design coding_and_education.md §… — 22_custom_estimator exercise      [--]
 */

#pragma once

#include "estimator.hpp"
#include "sf_math.hpp"

namespace sf {

/// Mahony complementary-filter attitude estimator (a swappable IEstimator).
/// Mahony 相補フィルタ姿勢推定器（差し替え可能な IEstimator）。
class ComplementaryEstimator : public IEstimator {
public:
    /// Initialize (level attitude, zero rate). / 初期化（水平姿勢・角速度ゼロ）。
    void init();

    void predict(const ImuData& imu, float dt) override;
    void updateTof(const TofData& /*tof*/) override {}    // not used / 未使用
    void updateFlow(const FlowData& /*flow*/) override {}  // not used / 未使用
    void updateMag(const MagData& /*mag*/) override {}     // not used / 未使用
    void updateBaro(const BaroData& baro) override;        // altitude only / 高度のみ
    StateEstimate getState() const override;
    void reset() override;
    void resetPositionVelocity() override;

private:
    math::Quat q_{1.0f, 0.0f, 0.0f, 0.0f};  ///< attitude q_nb (body→NED)
    math::Vec3 rate_{0.0f, 0.0f, 0.0f};     ///< body angular rate FRD [rad/s] (= gyro)
    float altitude_ = 0.0f;                 ///< altitude [m] up (pos_z = −altitude)
    float vz_up_ = 0.0f;                    ///< vertical velocity [m/s] up (vel_z = −vz_up)
    float mahony_kp_ = 1.0f;                ///< accel attitude-correction gain
    uint32_t timestamp_ = 0;
};

}  // namespace sf
