/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file complementary_estimator.cpp
 * @brief Mahony complementary-filter attitude estimator implementation.
 *        Mahony 相補フィルタ姿勢推定器の実装。
 */

#include "complementary_estimator.hpp"

namespace sf {

using math::Vec3;
using math::Quat;

void ComplementaryEstimator::init()
{
    reset();
}

void ComplementaryEstimator::reset()
{
    q_ = Quat{1.0f, 0.0f, 0.0f, 0.0f};
    rate_ = Vec3{0.0f, 0.0f, 0.0f};
    altitude_ = 0.0f;
    timestamp_ = 0;
}

void ComplementaryEstimator::resetPositionVelocity()
{
    altitude_ = 0.0f;
}

// -----------------------------------------------------------------------------
// predict — gyro integration with a Mahony accelerometer correction (roll/pitch).
// predict — ジャイロ積分＋Mahony 加速度補正（ロール/ピッチ）。
//
// The body rate for the controller is the gyro itself. The attitude integrates
// the gyro, then is nudged so its estimated gravity-down direction matches the
// accelerometer's (gravity carries no yaw, so yaw rides on the gyro only).
//
// 制御器用の機体角速度はジャイロそのもの。姿勢はジャイロを積分し、推定した重力下方向が
// 加速度計のそれに合うよう補正する（重力はヨー情報を持たないのでヨーはジャイロのみ）。
// -----------------------------------------------------------------------------
void ComplementaryEstimator::predict(const ImuData& imu, float dt)
{
    const Vec3 gyro{imu.gyro[0], imu.gyro[1], imu.gyro[2]};
    const Vec3 accel{imu.accel[0], imu.accel[1], imu.accel[2]};

    rate_ = gyro;  // body angular rate for the rate controller / レート制御器用

    // Accelerometer (driver-normalized) ≈ −gravity_body at low acceleration, so
    // the measured gravity-down direction in body is −accel/|accel|. The estimate
    // is q⁻¹·[0,0,1] (NED down rotated into body). The cross product is the small
    // rotation that aligns the estimate with the measurement.
    // 加速度計（ドライバ正規化）は低加速度で ≈ −重力(機体) なので、測定の重力下方向は
    // −accel/|accel|。推定は q⁻¹·[0,0,1]。外積が両者を合わせる微小回転。
    Vec3 gyro_corrected = gyro;
    const float a_norm = accel.norm();
    if (a_norm > 1e-3f) {
        const Vec3 meas_down = accel * (-1.0f / a_norm);     // measured gravity-down
        const Vec3 est_down = q_.inv_rotate(Vec3{0.0f, 0.0f, 1.0f});
        const Vec3 error = est_down.cross(meas_down);        // drives estimate → measurement
        gyro_corrected = gyro + error * mahony_kp_;
    }

    // Integrate the corrected body rate (right-multiply for a body-frame rate).
    // 補正した機体角速度を積分（機体系レートなので右から掛ける）。
    q_ = q_ * Quat::from_rotvec(gyro_corrected * dt);
    q_.normalize();
    timestamp_ = imu.timestamp;
}

void ComplementaryEstimator::updateBaro(const BaroData& baro)
{
    // Simple altitude: trust the barometer directly (optional; not used by a
    // stabilize-mode hover, but provided so altitude-hold could use this estimator).
    // 単純な高度: 気圧高度をそのまま信頼（任意。STABILIZE ホバーは未使用だが、高度
    // 保持でも本推定器を使えるよう提供）。
    altitude_ = baro.altitude;
}

StateEstimate ComplementaryEstimator::getState() const
{
    StateEstimate s = {};
    s.attitude[0] = q_.w; s.attitude[1] = q_.x;
    s.attitude[2] = q_.y; s.attitude[3] = q_.z;
    s.angular_rate[0] = rate_.x; s.angular_rate[1] = rate_.y; s.angular_rate[2] = rate_.z;
    s.position[2] = -altitude_;   // NED z = −altitude; position x/y + velocity not estimated
    s.sensor_mask = 0;            // attitude + rate only (no position/velocity)
    s.timestamp = timestamp_;
    return s;
}

}  // namespace sf
