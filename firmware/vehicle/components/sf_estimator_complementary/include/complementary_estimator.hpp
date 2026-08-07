/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file complementary_estimator.hpp
 * @brief Mahony-style complementary filter — a second IEstimator implementation.
 *        Mahony 型相補フィルタ — 2つ目の IEstimator 実装。
 *
 * Proves the SILS is algorithm-independent (RESET_PLAN P2): swapping ESKF →
 * complementary filter requires ZERO change to the SILS bench or the firmware
 * tasks — only the `estimator.type` parameter. This estimator computes what
 * stabilized hover and altitude-hold need: attitude (gyro integration corrected
 * toward the accelerometer's gravity direction), the body angular rate
 * (bias-corrected gyro), and the vertical channel (altitude + vertical velocity
 * from the accelerometer integral, anchored by the ToF — same sensor set as the
 * ESKF for a fair comparison; the barometer blends in only when present).
 * Horizontal position/velocity are NOT estimated; those StateEstimate fields
 * stay zero. ~120 lines of actual math.
 *
 * SILS がアルゴリズム非依存であることを実証する（RESET_PLAN P2）: ESKF → 相補フィルタ
 * の差し替えは SILS ベンチもファームタスクも一切変更せず、`estimator.type` パラメータ
 * だけで行える。本推定器は安定化ホバーと高度保持に必要な分を計算する: 姿勢（ジャイロ積分を
 * 加速度計の重力方向へ補正）、機体角速度（バイアス補正済みジャイロ）、鉛直チャネル
 * （加速度積分による高度＋鉛直速度を ToF で固定 — ESKF と同一センサ構成で公平比較。
 * 気圧計は搭載構成でのみブレンド）。水平位置・速度は推定しない（該当フィールドは 0）。
 *
 * @design requirements.md §10 — replaceable estimation                  [OK]
 * @design coding_and_education.md §… — 22_custom_estimator exercise      [OK]
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
    void updateTof(const TofData& tof) override;            // vertical anchor / 鉛直アンカー
    void updateFlow(const FlowData& /*flow*/) override {}   // not used / 未使用
    void updateMag(const MagData& /*mag*/) override {}      // not used / 未使用
    void updateBaro(const BaroData& baro) override;         // altitude (if fitted) / 高度（搭載時のみ）
    StateEstimate getState() const override;
    void reset() override;
    void resetPositionVelocity() override;
    void holdPositionVelocity() override;
    void applyCalibration(const float gyro_bias[3], const float accel_bias[3]) override;

private:
    math::Quat q_{1.0f, 0.0f, 0.0f, 0.0f};  ///< attitude q_nb (body→NED)
    math::Vec3 rate_{0.0f, 0.0f, 0.0f};     ///< body angular rate FRD [rad/s] (bias-corrected gyro)
    float altitude_ = 0.0f;                 ///< altitude [m] up (pos_z = −altitude)
    float vz_up_ = 0.0f;                    ///< vertical velocity [m/s] up (vel_z = −vz_up)
    float mahony_kp_ = 1.0f;                ///< accel attitude-correction gain

    // Boot-calibration biases, subtracted from every raw sample in predict().
    // Without this the rate controller closed its loop on the RAW gyro and the
    // vertical integral drifted with the accel bias.
    // 起動校正バイアス。predict() で毎サンプルの生値から減算する。これが無いと
    // レート制御は「生ジャイロ」で閉ループし、鉛直積分は加速度バイアスでドリフトした。
    math::Vec3 gyro_bias_{0.0f, 0.0f, 0.0f};   ///< [rad/s]
    math::Vec3 accel_bias_{0.0f, 0.0f, 0.0f};  ///< [m/s²]
    uint32_t timestamp_ = 0;
};

}  // namespace sf
