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
 * @design requirements.md §4 — Component #2: state estimation         [OK]
 * @design detailed_design.md §5 — Sensor observation switch           [OK]
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
    // Low-pass (1-pole, Hz) on the accel used for the ATTITUDE update only (NOT predict's
    // velocity integration). The gravity reference is corrupted by airframe vibration
    // (broadband prop + a structural mode); a ~30 Hz LPF cleans it so the accel-bias is
    // pulled less and fewer updates are χ²-rejected — the flight-log offline sweep cut the
    // accel-bias drift 0.28→0.18 m/s² at 30 Hz (a 12 Hz notch did NOT help: broadband).
    // 0 = off. 姿勢更新「のみ」に使う accel の1次ローパス[Hz]（predict の速度積分には掛けない）。
    // 重力基準が機体振動（広帯域プロペラ＋構造モード）で汚れるのを ~30Hz で清浄化 → accel
    // バイアスが引きずられにくく χ² 棄却も減る（実機ログ掃引で 30Hz がドリフト 0.28→0.18。
    // 12Hz ノッチは広帯域ゆえ無効）。0=無効。
    float accel_att_lpf_hz = 0.0f;        // [Hz] 0 = off

    // Initial covariance / 初期共分散
    float init_pos_std     = 0.1f;        // [m]
    float init_vel_std     = 0.1f;        // [m/s]
    float init_att_std     = 0.1f;        // [rad]
    float init_bg_std      = 0.01f;       // [rad/s]
    float init_ba_std      = 0.1f;        // [m/s²]

    // Constants / 定数
    float gravity          = math::kGravity;  // [m/s²] (SSOT: sf::math)
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
    // Minimum PMW3901 surface quality (SQUAL) to fuse a flow sample. Low-quality
    // surfaces (poor texture, dark, high altitude) give noisy displacement that
    // would corrupt POS_HOLD; reject below this. Default 10 only drops near-no-lock
    // flow (real usable flow has SQUAL well above this; the SIL plant reports 100).
    // Tune up from real POS_HOLD SQUAL logs during bring-up (code_review L-1).
    // フローを融合する PMW3901 表面品質(SQUAL)の下限。低品質面（特徴乏しい/暗い/高高度）は
    // ノイズだらけの変位を出し POS_HOLD を汚すため、これ未満は棄却。既定 10 はほぼ no-lock の
    // フローのみ落とす（実用フローの SQUAL はこれより十分高い。SIL プラントは 100 を報告）。
    // 実機 POS_HOLD の SQUAL ログから引き上げて調整する (L-1)。
    uint8_t flow_min_squal   = 10;

    // Gyro-bias deviation limit around the boot-calibration nominal [rad/s].
    // The Kalman gain routes EVERY observation into the bias states through the
    // cross-covariances — that coupling is what makes the bias observable, but it
    // also lets a misbehaving "irrelevant" sensor (mag disturbance, bad-surface
    // flow) drag the bias that feeds the rate loop. The clamp bounds the damage:
    // the in-flight estimate may refine the bias only within ± this value of the
    // verified-still boot measurement. Budget: BMI270 temperature drift is
    // ~0.02 dps/K → a 30 K warm-up shifts ~0.6 dps ≈ 0.01 rad/s; 0.03 rad/s
    // (~1.7 dps) leaves 3x headroom while capping a contaminated rate feedback
    // offset at 0.03 rad/s (PX4-style bias limiting).
    // 起動校正ノミナルまわりのジャイロバイアス偏差上限 [rad/s]。カルマンゲインは
    // 全観測をクロス共分散経由でバイアス状態へ流す — この結合がバイアスを可観測に
    // する一方、レート制御に無関係なセンサの異常（磁気外乱・質の悪い床のフロー）が
    // レートループに使うバイアスを引きずる経路にもなる。クランプは被害を有界化する:
    // 飛行中の推定は静止確認済み起動測定値の ± この値の範囲でのみバイアスを更新
    // できる。予算: BMI270 の温度ドリフトは ~0.02 dps/K → 30 K の昇温で ~0.6 dps ≈
    // 0.01 rad/s。0.03 rad/s（~1.7 dps）は 3 倍の余裕を残しつつ、汚染時のレート
    // フィードバックのオフセットを 0.03 rad/s に制限する（PX4 流バイアス制限）。
    float bg_deviation_max   = 0.03f;

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

    /// Apply a new config WITHOUT touching the state/covariance — live tuning
    /// (param set → ReloadParams). Recomputes the active mask in case use_*
    /// sensor switches changed.
    /// 状態・共分散に触れずに新しい設定を適用する — ライブチューニング
    /// （param set → ReloadParams）。use_* センサスイッチの変化に備えて
    /// active mask を再計算する。
    void setConfig(const EskfConfig& cfg);

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
                       float dt, float gyro_x, float gyro_y, uint8_t squal);

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

    /// Set gyro bias from calibration. Also anchors the NOMINAL for the
    /// deviation clamp (cfg_.bg_deviation_max): in-flight Kalman corrections may
    /// move the bias only within ± the limit of this verified-still measurement.
    /// キャリブレーションからジャイロバイアスを設定。偏差クランプ
    /// （cfg_.bg_deviation_max）の「ノミナル」もここで固定する: 飛行中のカルマン
    /// 補正は、この静止確認済み測定値の ± 上限の範囲でのみバイアスを動かせる。
    void setGyroBias(const Vec3& bias) { bg_ = bias; bg_nominal_ = bias; }

    /// Set accel bias (from calibration) / 加速度バイアスを設定（キャリブレーションから）
    void setAccelBias(const Vec3& bias) { ba_ = bias; }

    /// Set the magnetometer NED reference [uT] (boot capture; see EskfEstimator)
    /// 地磁気 NED 参照 [uT] を設定（起動時捕捉。EskfEstimator 参照）
    void setMagReference(const Vec3& ned) { cfg_.mag_ref = ned; }

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

    // Gyro-bias nominal for the deviation clamp (= boot calibration measurement;
    // {0,0,0} until seeded). See EskfConfig::bg_deviation_max.
    // 偏差クランプ用ジャイロバイアスノミナル（= 起動校正測定値。種付けまで {0,0,0}）。
    // EskfConfig::bg_deviation_max 参照。
    Vec3 bg_nominal_{};

    // Active mask (bit i = state i is active) / 有効マスク
    uint16_t active_mask_ = 0x7FFF;  // All 15 states active
    bool freeze_accel_bias_ = false;

    // Mag calibration gate, INDEPENDENT of the param-derived cfg_.use_mag. The
    // boot-mag policy (ImuTask) clears this when the magnetometer is uncalibrated
    // so hard-iron offsets cannot contaminate yaw, regardless of eskf.use_mag.
    // It is NOT touched by setConfig()/reloadParams(), so live tuning or
    // re-calibration cannot silently re-admit an uncalibrated mag (code_review
    // L-5). updateMag fuses only when (cfg_.use_mag && mag_calib_gate_).
    // 磁気校正ゲート。param 由来の cfg_.use_mag とは独立。起動磁気ポリシー(ImuTask)が
    // 未校正時にこれを下ろし、eskf.use_mag に関わらずハードアイアンがヨーを汚染しない
    // ようにする。setConfig()/reloadParams() では触らないので、ライブチューニングや再校正で
    // 未校正磁気が黙って復帰しない (L-5)。updateMag は (cfg_.use_mag && mag_calib_gate_)
    // のときのみ融合する。
    bool mag_calib_gate_ = true;

    // Acceleration-compensated accel-attitude state (α-β tracker on the flow velocity).
    // flow_vel_lpf_ = filtered velocity state, a_kin_ned_ = the horizontal NED kinematic
    // acceleration that updateAccelAttitude subtracts from the specific force.
    // 運動加速度補償の状態（フロー速度の α-β トラッカ）。flow_vel_lpf_=濾波速度状態、
    // a_kin_ned_=updateAccelAttitude が比力から差し引く水平 NED 運動加速度。
    Vec3  flow_vel_lpf_  = {0, 0, 0};   // α-β velocity state / α-β 速度状態
    Vec3  a_kin_ned_     = {0, 0, 0};   // α-β acceleration state (a_kin, NED) / α-β 加速度状態
    bool  have_flow_vel_ = false;

    // Accel-attitude LPF state (cfg_.accel_att_lpf_hz). dt comes from predict() (same
    // IMU cycle as the attitude update). NED-frame-agnostic: it filters the body accel.
    // accel 姿勢 LPF の状態（cfg_.accel_att_lpf_hz）。dt は predict() から（姿勢更新と同一
    // IMU サイクル）。body accel を濾波する。
    Vec3  accel_att_lpf_ = {0, 0, 0};   // filtered body accel for the attitude update
    float accel_lpf_dt_  = 0.0f;        // [s] last predict dt (for the LPF coefficient)
    bool  accel_lpf_init_ = false;      // seed the filter to the first sample (no ramp)

    // ToF-velocity differentiation history (updateToFVelocity). Member, not a
    // function-local static, so reset() discards it (no spike on re-takeoff).
    // ToF 速度微分の履歴（updateToFVelocity）。関数ローカル static でなくメンバとし、
    // reset() で破棄する（再離陸時のスパイク防止）。
    float tof_prev_height_      = 0;
    bool  tof_have_prev_height_ = false;

    // Internal / 内部
    void recomputeActiveMask();
    void enforceCovarianceConstraints();
    void applyMaskedErrorState(float dx[N]);

    /// Scalar Kalman update (1D observation) / スカラーカルマン更新
    void scalarUpdate(const float H[N], float innovation, float R);

    /// 3D Kalman update / 3次元カルマン更新
    void vectorUpdate3(const float H[3][N], const float innov[3], float R,
                       float chi2_gate);
};

}  // namespace sf
