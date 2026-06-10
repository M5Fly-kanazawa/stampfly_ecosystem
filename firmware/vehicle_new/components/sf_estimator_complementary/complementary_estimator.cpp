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

#include <cmath>   // std::sqrt / std::cos (ToF tilt compensation)

namespace sf {

using math::Vec3;
using math::Quat;

// Vertical-channel constants. Gravity closes the accelerometer→acceleration loop;
// the per-update gains are the complementary blend (strong on altitude, gentle on
// velocity — the accel integral carries the fast motion, the slow drift is
// anchored by the absolute sensor). The ToF is the primary vertical anchor (same
// sensor set as the ESKF, project policy); the baro blends in only when fitted.
// 鉛直チャネル定数。重力で加速度計→加速度を閉じ、更新ごとのゲインが相補ブレンド
// （高度に強く・速度に弱く — 速い動きは加速度積分、遅いドリフトは絶対センサで固定）。
// 鉛直アンカーの主役は ToF（ESKF と同一センサ構成、プロジェクト方針）。気圧計は搭載
// 構成でのみブレンド。
namespace {
constexpr float kGravity     = math::kGravity;  ///< [m/s²] NED down-positive (SSOT: sf::math)
constexpr float kTofAltGain  = 0.30f;   ///< altitude correction per ToF update
constexpr float kTofVelGain  = 0.05f;   ///< velocity correction per ToF update
constexpr float kBaroAltGain = 0.30f;   ///< altitude correction per baro update
constexpr float kBaroVelGain = 0.05f;   ///< velocity correction per baro update
constexpr float kTofMaxTiltRad = 0.5f;  ///< skip ToF beyond ~29° tilt (beam off-ground)
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
    // Same contract as the ESKF: reset zeroes the biases; ImuTask re-applies the
    // boot calibration right after (reseedCalibration).
    // ESKF と同じ契約: reset はバイアスをゼロ化し、直後に ImuTask が起動校正を
    // 再適用する（reseedCalibration）。
    gyro_bias_  = Vec3{0.0f, 0.0f, 0.0f};
    accel_bias_ = Vec3{0.0f, 0.0f, 0.0f};
    timestamp_ = 0;
}

void ComplementaryEstimator::resetPositionVelocity()
{
    altitude_ = 0.0f;
    vz_up_ = 0.0f;
}

void ComplementaryEstimator::holdPositionVelocity()
{
    // On-ground anchor (called every cycle while grounded, same contract as the
    // ESKF): pin the vertical channel so predict-only drift cannot accumulate
    // before takeoff.
    // 接地アンカー（接地中毎周期呼ばれる。ESKF と同じ契約）: 鉛直チャネルを固定し、
    // 離陸前に予測のみのドリフトが溜まらないようにする。
    altitude_ = 0.0f;
    vz_up_ = 0.0f;
}

void ComplementaryEstimator::applyCalibration(const float gyro_bias[3],
                                              const float accel_bias[3])
{
    gyro_bias_  = Vec3{gyro_bias[0], gyro_bias[1], gyro_bias[2]};
    accel_bias_ = Vec3{accel_bias[0], accel_bias[1], accel_bias[2]};
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
    // Subtract the boot-calibration biases first: the rate controller closes its
    // loop on rate_, and the vertical integral below amplifies any accel bias
    // into a velocity ramp.
    // まず起動校正バイアスを減算する: レート制御器は rate_ で閉ループし、下の鉛直積分は
    // 加速度バイアスを速度ランプに増幅してしまう。
    const Vec3 gyro{imu.gyro[0] - gyro_bias_.x,
                    imu.gyro[1] - gyro_bias_.y,
                    imu.gyro[2] - gyro_bias_.z};
    const Vec3 accel{imu.accel[0] - accel_bias_.x,
                     imu.accel[1] - accel_bias_.y,
                     imu.accel[2] - accel_bias_.z};

    rate_ = gyro;  // body angular rate for the rate controller / レート制御器用

    // Accelerometer (driver-normalized) ≈ −gravity_body at low acceleration, so
    // the measured gravity-down direction in body is −accel/|accel|. The estimate
    // is q⁻¹·[0,0,1] (NED down rotated into body). The Mahony error is
    // measured × estimated: with the right-multiply integration below,
    // est_down' ≈ est_down + (est_down × ω)·dt, and ω = meas×est gives
    // est_down × (meas×est) ≈ meas − est, i.e. the estimate moves TOWARD the
    // measurement (negative feedback). The reversed order est×meas would push
    // it away (positive feedback → divergence to the inverted equilibrium).
    // 加速度計（ドライバ正規化）は低加速度で ≈ −重力(機体) なので、測定の重力下方向は
    // −accel/|accel|。推定は q⁻¹·[0,0,1]。Mahony 誤差は「測定 × 推定」: 下の右乗算積分では
    // est_down' ≈ est_down + (est_down × ω)·dt となり、ω = meas×est なら
    // est_down × (meas×est) ≈ meas − est、つまり推定が測定へ向かう（負帰還）。
    // 逆順 est×meas は離れていく（正帰還 → 上下反転平衡点へ発散）。
    Vec3 gyro_corrected = gyro;
    const float a_norm = accel.norm();
    if (a_norm > 1e-3f) {
        const Vec3 meas_down = accel * (-1.0f / a_norm);     // measured gravity-down
        const Vec3 est_down = q_.inv_rotate(Vec3{0.0f, 0.0f, 1.0f});
        const Vec3 error = meas_down.cross(est_down);        // drives estimate → measurement
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

void ComplementaryEstimator::updateTof(const TofData& tof)
{
    // ToF is the primary vertical anchor (project policy: ToF-only vertical, the
    // same sensor set the ESKF uses). Tilt-compensate the slant range to height,
    // then blend like the baro: strong on altitude, gentle on velocity. Skip
    // invalid readings and large tilts (the beam leaves the ground footprint).
    // ToF が鉛直アンカーの主役（方針: 鉛直は ToF のみ — ESKF と同一センサ構成）。
    // 斜距離を傾き補正で高度に直し、気圧と同じく高度に強く・速度に弱くブレンドする。
    // 無効読みと大傾斜（ビームが接地footprintを外れる）はスキップ。
    if (!tof.valid) {
        return;
    }
    const Vec3 euler = q_.to_euler();
    const float tilt = std::sqrt(euler.x * euler.x + euler.y * euler.y);
    if (tilt > kTofMaxTiltRad) {
        return;
    }
    const float height = tof.distance * std::cos(euler.x) * std::cos(euler.y);

    const float err = height - altitude_;
    altitude_ += kTofAltGain * err;
    vz_up_    += kTofVelGain * err;
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
