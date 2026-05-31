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

// Vertical-channel constants. Gravity closes the accelerometer→acceleration loop;
// the baro gains are the complementary blend applied once per barometer update
// (strong on altitude, gentle on velocity — the accel integral carries the fast
// motion, the baro anchors the slow drift).
// 鉛直チャネル定数。重力で加速度計→加速度を閉じ、気圧ゲインは気圧更新ごとの相補ブレンド
// （高度に強く・速度に弱く — 速い動きは加速度積分、遅いドリフトは気圧で固定）。
namespace {
constexpr float kGravity     = 9.81f;   ///< [m/s²] NED down-positive
constexpr float kBaroAltGain = 0.30f;   ///< altitude correction per baro update
constexpr float kBaroVelGain = 0.05f;   ///< velocity correction per baro update
}  // namespace

void ComplementaryEstimator::init()
{
    reset();
}

void ComplementaryEstimator::reset()
{
    q_ = Quat{1.0f, 0.0f, 0.0f, 0.0f};
    rate_ = Vec3{0.0f, 0.0f, 0.0f};
    altitude_ = 0.0f;
    vz_up_ = 0.0f;
    timestamp_ = 0;
}

void ComplementaryEstimator::resetPositionVelocity()
{
    altitude_ = 0.0f;
    vz_up_ = 0.0f;
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

    // Vertical channel: rotate the body specific force into NED, add gravity to get
    // inertial acceleration, integrate up-positive into velocity then altitude. The
    // baro corrects the inevitable drift in updateBaro (complementary blend).
    // At rest accel ≈ [0,0,−g] (FRD), so a_world ≈ 0 and the integrator stays put.
    // 鉛直チャネル: 機体比力を NED に回し重力を足して慣性加速度を得て、上正で速度→高度へ
    // 積分する。ドリフトは updateBaro が気圧で補正（相補ブレンド）。静止時 accel≈[0,0,−g]
    // なので a_world≈0 で積分器は動かない。
    Vec3 a_world = q_.rotate(accel);
    a_world.z += kGravity;                 // inertial accel (NED down-positive)
    const float a_up = -a_world.z;         // up-positive
    vz_up_ += a_up * dt;
    altitude_ += vz_up_ * dt;

    timestamp_ = imu.timestamp;
}

void ComplementaryEstimator::updateBaro(const BaroData& baro)
{
    // Complementary correction toward the barometer: nudge altitude (strongly) and
    // velocity (gently) by the same measured error. This anchors the accelerometer
    // integral so altitude-hold gets a non-drifting altitude AND vertical velocity.
    // 気圧への相補補正: 同じ観測誤差で高度を強く・速度を弱く引き寄せる。加速度積分を固定し、
    // 高度保持にドリフトのない高度と鉛直速度を供給する。
    const float err = baro.altitude - altitude_;
    altitude_ += kBaroAltGain * err;
    vz_up_    += kBaroVelGain * err;
}

StateEstimate ComplementaryEstimator::getState() const
{
    StateEstimate s = {};
    s.attitude[0] = q_.w; s.attitude[1] = q_.x;
    s.attitude[2] = q_.y; s.attitude[3] = q_.z;
    s.angular_rate[0] = rate_.x; s.angular_rate[1] = rate_.y; s.angular_rate[2] = rate_.z;
    s.position[2] = -altitude_;   // NED z = −altitude (down-positive)
    s.velocity[2] = -vz_up_;      // NED vz = −vz_up; position/velocity x/y not estimated
    s.sensor_mask = 0;            // attitude + rate + vertical (no horizontal pos/vel)
    s.timestamp = timestamp_;
    return s;
}

}  // namespace sf
