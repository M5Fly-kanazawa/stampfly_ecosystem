/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file eskf_core.hpp
 * @brief Error-State Kalman Filter (15-state) — core implementation
 *        誤差状態カルマンフィルタ（15状態）— コア実装
 *
 * Implemented from mathematical foundations. State vector:
 *   [pos(3), vel(3), att_err(3), gyro_bias(3), accel_bias(3)] = 15 states
 *
 * 数学的基礎から実装。状態ベクトル:
 *   [位置(3), 速度(3), 姿勢誤差(3), ジャイロバイアス(3), 加速度バイアス(3)] = 15状態
 *
 * @design requirements.md §4 — Component #2: state estimation         [--]
 * @design architecture.md §3 — Sensor observation switch              [--]
 */

#pragma once

#include "sf_math.hpp"
#include <cstdint>

namespace sf {

using namespace math;

// State indices / 状態インデックス
// 各状態変数の開始インデックス
enum StateIdx {
    POS_X = 0, POS_Y = 1, POS_Z = 2,
    VEL_X = 3, VEL_Y = 4, VEL_Z = 5,
    ATT_X = 6, ATT_Y = 7, ATT_Z = 8,
    BG_X = 9,  BG_Y = 10, BG_Z = 11,
    BA_X = 12, BA_Y = 13, BA_Z = 14,
};

/// ESKF configuration / ESKF設定
struct EskfConfig {
    // Process noise (Q) / プロセスノイズ
    float gyro_noise       = 0.009655f;   // [rad/s/√Hz]
    float accel_noise      = 0.3f;        // [m/s²/√Hz]
    float gyro_bias_noise  = 0.000013f;   // [rad/s/√Hz]
    float accel_bias_noise = 0.0001f;     // [m/s²/√Hz]

    // Observation noise (R) / 観測ノイズ
    float tof_noise        = 0.03f;       // [m]
    float flow_noise       = 0.30f;       // [m/s]
    float baro_noise       = 0.1f;        // [m]
    float mag_noise        = 1.0f;        // [uT]
    float accel_att_noise  = 2.0f;        // [m/s²] — matches vibration noise at hover

    // Initial covariance / 初期共分散
    float init_pos_std     = 0.1f;        // [m]
    float init_vel_std     = 0.1f;        // [m/s]
    float init_att_std     = 0.1f;        // [rad]
    float init_bg_std      = 0.01f;       // [rad/s]
    float init_ba_std      = 0.1f;        // [m/s²]

    // Constants / 定数
    float gravity          = 9.81f;       // [m/s²]
    Vec3 mag_ref           = {20.0f, 0.0f, 40.0f};  // NED [uT]

    // Gates / ゲート閾値
    float tof_innov_gate   = 0.5f;        // [m] absolute
    float baro_innov_gate  = 0.5f;        // [m] absolute
    float mag_chi2_gate    = 7.81f;       // χ²(3, 0.95)
    float accel_chi2_gate  = 7.81f;       // χ²(3, 0.95)
    float tof_tilt_threshold = 0.70f;     // [rad]

    // Flow / フロー
    float flow_rad_per_pixel = 0.00222f;
    float flow_gyro_scale    = 1.0f;
    float flow_min_height    = 0.02f;
    float flow_innov_clamp   = 0.3f;

    // Attitude correction / 姿勢補正 (proven firmware/vehicle values; no norm gate —
    // the accel-attitude update downweights via k_adaptive + χ², never hard-gates).
    // 実証済み firmware/vehicle 値。norm gate は持たない（k_adaptive＋χ² で弱める）。
    float att_correction_clamp = 0.05f;   // [rad] per-update roll/pitch correction clamp
    float k_adaptive       = 10.0f;       // Adaptive R: R *= (1 + k * |a-g|²)

    // Acceleration-compensated accel-attitude (POS_HOLD). The accelerometer measures the
    // specific force f = a_kin − g; the plain accel-attitude update assumes a_kin = 0, so
    // during a horizontal maneuver it mistakes the kinematic term a_kin for a tilt and the
    // estimate sticks at the "apparent gravity" angle atan(a/g) → POS_HOLD flies away. We
    // estimate a_kin from the OPTICAL-FLOW velocity (independent of attitude-from-accel) with
    // an α-β tracker, and updateAccelAttitude predicts f = g_expected + R^T·a_kin so the
    // residual is the TRUE attitude error. SIL Layer-4: all of roll/pitch/diagonal/yaw hold
    // (clean + N0). 運動加速度補償の accel-attitude（POS_HOLD）。加速度計は比力 f=a_kin−g を
    // 測り、素の更新は a_kin=0 を仮定するので水平マニューバ中に a_kin を傾きと誤認し推定が
    // 「見かけの重力」角 atan(a/g) に張付き POS_HOLD が飛び去る。a_kin を（姿勢-加速度と独立な）
    // オプティカルフロー速度から α-β トラッカで推定し、updateAccelAttitude が
    // f = g_expected + R^T·a_kin と予測 → 残差が真の姿勢誤差に。SIL Layer-4 で roll/pitch/斜め/
    // yaw すべて保持（clean + N0）。
    bool  accel_comp_enable = true;     // master enable / マスタ有効
    float accel_comp_alpha  = 0.2f;     // α-β velocity gain / α-β 速度ゲイン
    float accel_comp_beta   = 0.02f;    // α-β acceleration gain (small = capture DC drift)
    float accel_comp_max    = 5.0f;     // [m/s²] physical clamp on a_kin / a_kin の物理クランプ

    // Sensor enable / センサ有効
    bool use_tof           = true;
    bool use_flow          = true;
    bool use_baro          = false;
    bool use_mag           = false;
};

/// ESKF core class / ESKFコアクラス
class EskfCore {
public:
    /// Initialize with config / 設定で初期化
    void init(const EskfConfig& cfg);

    /// Reset all state / 全状態リセット
    void reset();

    /// Reset position and velocity only / 位置・速度のみリセット
    void resetPositionVelocity();

    /// Inflate covariance for the states selected by `state_mask` (bit i = StateIdx i):
    /// set each selected diagonal back to its init value and zero its cross-covariance,
    /// WITHOUT changing the state estimate. Declares "I am no longer confident about these
    /// states" while keeping the best estimate — used at the ground→flight boundary where
    /// the on-ground convergence is not flight-representative, but a full reset of the
    /// state destabilizes the takeoff transient.
    /// state_mask（ビット i = StateIdx i）で選んだ状態の共分散を膨張する: 各対角を init 値に
    /// 戻しクロス共分散をゼロ化するが、状態推定値は変えない。「これらの状態の自信を捨てる」
    /// 宣言で最良推定は保持 — 地上収束が飛行を代表しないが状態の全リセットは離陸過渡を
    /// 不安定化する、地上→飛行境界で使う。
    void inflateCovariance(uint16_t state_mask);

    // =========================================================================
    // Prediction / 予測
    // =========================================================================

    /// IMU prediction step / IMU予測ステップ
    /// @param accel Raw accelerometer [m/s²] / 加速度計生値
    /// @param gyro  Raw gyroscope [rad/s] / ジャイロ生値
    /// @param dt    Time step [s] / タイムステップ
    void predict(const Vec3& accel, const Vec3& gyro, float dt);

    // =========================================================================
    // Observation updates / 観測更新
    // =========================================================================

    void updateToF(float distance);
    void updateToFVelocity(float distance, float dt);  // ToF velocity observation
    void updateBaro(float altitude);
    void updateMag(const Vec3& mag);
    void updateAccelAttitude(const Vec3& accel);
    void updateFlowRaw(int16_t dx, int16_t dy, float height,
                       float dt, float gyro_x, float gyro_y);

    // =========================================================================
    // State access / 状態アクセス
    // =========================================================================

    Quat getAttitude() const { return q_; }
    Vec3 getPosition() const { return pos_; }
    Vec3 getVelocity() const { return vel_; }
    Vec3 getGyroBias() const { return bg_; }
    Vec3 getAccelBias() const { return ba_; }

    /// Get bias-corrected body angular rate FRD [rad/s] / バイアス補正済み機体角速度を取得
    Vec3 getAngularRate() const { return ang_rate_; }

    /// Get P-matrix diagonal element / P行列対角要素を取得
    float getPDiag(int idx) const { return P_(idx, idx); }

    /// Set gyro bias (from calibration) / ジャイロバイアスを設定（キャリブレーションから）
    void setGyroBias(const Vec3& bias) { bg_ = bias; }

    /// Set accel bias (from calibration) / 加速度バイアスを設定（キャリブレーションから）
    void setAccelBias(const Vec3& bias) { ba_ = bias; }

    /// Set initial attitude from gravity vector (roll/pitch from accel)
    /// 重力ベクトルから初期姿勢を設定（加速度からroll/pitch）
    void setAttitudeFromGravity(const Vec3& accel_avg);

    /// Shrink bias covariance (trust calibration result)
    /// バイアス共分散を縮小（キャリブレーション結果を信頼）
    void shrinkBiasCovariance(float factor);

    /// Get active sensor mask / 有効センサマスクを取得
    uint16_t getActiveMask() const { return active_mask_; }

    /// Set sensor enable / センサ有効設定
    void setSensorEnabled(int group, bool enabled);

    /// Hold position/velocity at zero (for landed state)
    /// 位置/速度をゼロにホールド（着陸状態用）
    void holdPositionVelocity();

    /// Freeze accel bias estimation / 加速度バイアス推定をフリーズ
    void setFreezeAccelBias(bool freeze);

private:
    // Nominal state / 名目状態
    Vec3 pos_;         // Position NED [m] / 位置
    Vec3 vel_;         // Velocity NED [m/s] / 速度
    Quat q_;           // Attitude quaternion / 姿勢クォータニオン
    Vec3 bg_;          // Gyro bias [rad/s] / ジャイロバイアス
    Vec3 ba_;          // Accel bias [m/s²] / 加速度バイアス
    Vec3 ang_rate_;    // Bias-corrected body angular rate FRD [rad/s] / バイアス補正済み機体角速度

    // Error-state covariance P (15x15) / 誤差状態共分散
    SymMat15 P_;

    // Config / 設定
    EskfConfig cfg_;

    // Active mask (bit i = state i is active) / 有効マスク
    uint16_t active_mask_ = 0x7FFF;  // All 15 states active
    bool freeze_accel_bias_ = false;

    // Acceleration-compensated accel-attitude state (α-β tracker on the flow velocity).
    // flow_vel_lpf_ = filtered velocity state, a_kin_ned_ = the horizontal NED kinematic
    // acceleration that updateAccelAttitude subtracts from the specific force.
    // 運動加速度補償の状態（フロー速度の α-β トラッカ）。flow_vel_lpf_=濾波速度状態、
    // a_kin_ned_=updateAccelAttitude が比力から差し引く水平 NED 運動加速度。
    Vec3  flow_vel_lpf_  = {0, 0, 0};   // α-β velocity state / α-β 速度状態
    Vec3  a_kin_ned_     = {0, 0, 0};   // α-β acceleration state (a_kin, NED) / α-β 加速度状態
    bool  have_flow_vel_ = false;

    // Internal / 内部
    void recomputeActiveMask();
    void enforceCovarianceConstraints();
    void applyMaskedErrorState(float dx[N]);

    /// Scalar Kalman update (1D observation) / スカラーカルマン更新
    void scalarUpdate(const float H[N], float innovation, float R);

    /// 3D Kalman update / 3次元カルマン更新
    void vectorUpdate3(const float H[3][N], const float innov[3], float R);
};

}  // namespace sf
