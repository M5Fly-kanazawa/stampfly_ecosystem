/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file estimator.hpp
 * @brief State estimation interface definition
 *        状態推定インターフェース定義
 *
 * Defines the contract that all state estimators must satisfy.
 * ESKF, EKF, Complementary Filter, or any custom estimator can
 * be used by implementing this interface.
 *
 * 全ての状態推定器が満たすべき規約を定義する。
 * ESKF、EKF、相補フィルタ、その他のカスタム推定器は
 * このインターフェースを実装すれば使用可能。
 *
 * Sensor observation ON/OFF is controlled via parameters inside
 * the estimator implementation. Raw data always reaches the
 * estimator; it decides whether to use each observation.
 *
 * センサ観測のON/OFFは推定器実装内部でパラメータ制御する。
 * 生データは常に推定器に届き、各観測を使うかは推定器が判断する。
 *
 * @design requirements.md §4 — Component #2: replaceable estimation   [OK]
 * @design requirements.md §10 — Replaceable estimation                [OK]
 * @design architecture.md §2 — Estimation: unified interface          [OK]
 * @design detailed_design.md §5 — IEstimator definition               [OK]
 * @design coding_and_education.md §2 — Bilingual comments             [OK]
 */

#pragma once

#include "data_types.hpp"

namespace sf {

/// State estimation interface
/// 状態推定インターフェース
///
/// All estimator implementations (ESKF, EKF, etc.) inherit from this
/// class and implement the virtual methods. The estimation task calls
/// these methods; the estimator does not know about FreeRTOS tasks.
///
/// 全推定器実装（ESKF、EKF等）はこのクラスを継承し、
/// 仮想メソッドを実装する。推定タスクがこれらのメソッドを呼ぶ。
/// 推定器はFreeRTOSタスクを知らない。
///
/// @design detailed_design.md §5 — predict/update*/getState/reset     [OK]
class IEstimator {
public:
    virtual ~IEstimator() = default;

    // =========================================================================
    // Prediction
    // 予測ステップ
    // =========================================================================

    /// IMU prediction step — called at IMU rate (400Hz)
    /// IMU予測ステップ — IMUレート（400Hz）で呼ばれる
    ///
    /// Propagates the state forward using accelerometer and gyroscope.
    /// 加速度計とジャイロスコープを使って状態を前方に伝搬する。
    ///
    /// @param imu  Raw IMU data / IMU生データ
    /// @param dt   Time step [s] / タイムステップ
    virtual void predict(const ImuData& imu, float dt) = 0;

    // =========================================================================
    // Observation Updates (async — called when sensor data arrives)
    // 観測更新（非同期 — センサデータ到着時に呼ばれる）
    //
    // Each method receives raw sensor data. The estimator internally
    // decides whether to use it based on parameter settings
    // (e.g., eskf.use_tof = false → skip ToF update).
    //
    // 各メソッドは生センサデータを受け取る。推定器内部で
    // パラメータ設定に基づいて使用するか判断する
    // （例: eskf.use_tof = false → ToF更新をスキップ）。
    //
    // @design detailed_design.md §5 — Sensor observation switch       [OK]
    // =========================================================================

    /// ToF distance observation update
    /// ToF距離観測更新
    virtual void updateTof(const TofData& tof) = 0;

    /// Optical flow observation update
    /// オプティカルフロー観測更新
    virtual void updateFlow(const FlowData& flow) = 0;

    /// Magnetometer observation update
    /// 地磁気観測更新
    virtual void updateMag(const MagData& mag) = 0;

    /// Barometer observation update
    /// 気圧観測更新
    virtual void updateBaro(const BaroData& baro) = 0;

    // =========================================================================
    // State Output
    // 状態出力
    // =========================================================================

    /// Get the current state estimate
    /// 現在の推定値を取得する
    ///
    /// Returns a snapshot of the estimated state. Thread-safe when
    /// called from a different task than predict/update.
    ///
    /// 推定状態のスナップショットを返す。predict/updateとは
    /// 別のタスクから呼んでもスレッドセーフ。
    virtual StateEstimate getState() const = 0;

    // =========================================================================
    // Reset
    // リセット
    // =========================================================================

    /// Reset full state (called on ARM, mode transitions)
    /// 全状態リセット（ARM時、モード遷移時に呼ばれる）
    virtual void reset() = 0;

    /// Reset position and velocity only (keep attitude)
    /// 位置・速度のみリセット（姿勢は保持）
    ///
    /// Called at the ground→airborne edge to start position tracking from origin
    /// with a fresh covariance (clean ToF lock at takeoff).
    /// 接地→空中エッジで呼ばれ、新しい共分散で原点から位置追跡を開始する
    /// （離陸時のクリーンな ToF ロック）。
    virtual void resetPositionVelocity() = 0;

    /// Hold position and velocity at zero (called every cycle while on the ground)
    /// 位置・速度をゼロに保持（接地中は毎サイクル呼ばれる）
    ///
    /// While on the ground the only vertical observation (ToF) is invalid below its
    /// minimum range, so the predict-only vertical state would drift (vel_z
    /// integrates into a pos_z ramp). Clamping pos/vel to zero anchors the estimate
    /// at the known ground state. Default no-op: estimators that do not track
    /// position (e.g. attitude-only filters) need not implement it.
    /// 接地中は唯一の鉛直観測 ToF が最小レンジ未満で無効なため、予測のみの鉛直状態が
    /// ドリフトする（vel_z が積分され pos_z がランプ）。pos/vel をゼロに固定して既知の
    /// 地上状態に錨を打つ。既定 no-op: 位置を推定しない推定器（姿勢のみ等）は未実装でよい。
    virtual void holdPositionVelocity() {}

    // =========================================================================
    // Calibration
    // キャリブレーション
    // =========================================================================

    /// Seed the estimator with boot-calibration biases measured at rest (gyro and
    /// accel offsets, body frame, gravity already removed). Called ONCE on the ground
    /// before flight so the filter starts at the true bias instead of converging from
    /// zero during the critical takeoff transient — an uncalibrated accel bias is the
    /// documented ESKF attitude-loop destabilizer.
    ///
    /// Default no-op so an estimator that does not seed bias (e.g. an attitude-only
    /// complementary filter) need not implement it. NOTE: a full reset() zeros the
    /// bias, so if a future onEnter(ARMED_GROUND) wires an ESKF reset, the calibration
    /// must be re-applied AFTER that reset.
    ///
    /// 静止で測った起動校正バイアス（機体系のジャイロ/加速度オフセット、重力は除去済み）を
    /// 推定器に種付けする。飛行前に地上で1回呼び、フィルタがゼロから収束する代わりに真の
    /// バイアスから始められるようにする（離陸過渡で未校正の加速度バイアスは ESKF 姿勢ループを
    /// 不安定化させる既知の要因）。既定 no-op: バイアスを種付けしない推定器（姿勢のみの相補
    /// フィルタ等）は未実装でよい。注意: full reset() はバイアスをゼロ化するため、将来
    /// onEnter(ARMED_GROUND) で ESKF reset を配線したら、その後に校正を再適用すること。
    ///
    /// @param gyro_bias   Measured gyro bias [rad/s], body frame / 測定ジャイロバイアス
    /// @param accel_bias  Measured accel bias [m/s²], body frame (gravity removed) / 加速度バイアス
    virtual void applyCalibration(const float gyro_bias[3], const float accel_bias[3]) {}

    /// Set the magnetometer NED reference vector [uT] (captured at boot from the
    /// calibrated mag + converged attitude — defines "yaw 0 = boot heading").
    /// Default no-op for estimators without a mag observation.
    /// 地磁気の NED 参照ベクトル [uT] を設定（起動時に校正済み磁気と収束済み姿勢から
    /// 捕捉 — 「ヨー0 = 起動方位」を定義する）。磁気観測を持たない推定器では no-op。
    virtual void setMagReference(const float ned[3]) {}

    /// Enable/disable one observation group at runtime (0=ToF,1=Baro,2=Mag,3=Flow).
    /// Used to keep an UNCALIBRATED mag out of the fusion. Default no-op.
    /// 観測グループの実行時有効/無効（0=ToF,1=Baro,2=Mag,3=Flow）。未校正の磁気を
    /// 融合に入れないために使う。既定 no-op。
    virtual void setSensorEnabled(int group, bool enabled) {}

    // =========================================================================
    // Bias freeze (ground / flight regimes)
    // バイアス凍結（地上 / 飛行レジーム）
    //
    // CAUTION (NOT currently wired): the ESKF freeze mechanism (active_mask +
    // enforceCovarianceConstraints) is for PERMANENT sensor-absence isolation — it resets
    // the frozen state's covariance to its init value each cycle. Toggling it across the
    // ground↔flight boundary therefore restores a huge covariance at unfreeze and
    // destabilizes the takeoff transient (SIL-validated). detailed_design §3 note 3 defers
    // it; bias estimation stays active in flight. Do NOT re-wire into a transition without a
    // covariance-preserving "soft freeze". Kept as a capability for that future redesign.
    // 注意（現在未配線）: ESKF の凍結機構（active_mask＋enforceCovarianceConstraints）は
    // 「センサ恒久不在」の隔離用で、凍結状態の共分散を毎周期 init 値へ戻す。地上↔飛行で
    // トグルすると解除時に巨大な共分散が復活し離陸過渡を不安定化する（SIL 実証）。
    // detailed_design §3 注3 で見送り。バイアス推定は飛行中もアクティブのまま。共分散保持の
    // 「ソフト凍結」なしに遷移へ再配線しないこと。将来の再設計用に capability として残置。
    // =========================================================================

    /// Freeze bias estimation (on the ground / at landing). The at-rest bias is held
    /// constant instead of being re-estimated, so ground vibration/settling does not
    /// perturb the calibrated bias. Default no-op for estimators that do not estimate
    /// bias (e.g. the attitude-only complementary filter).
    /// バイアス推定を凍結（地上・着陸時）。静止バイアスを再推定せず一定に保ち、地上の振動・
    /// 整定が校正済みバイアスを乱さないようにする。バイアスを推定しない推定器（姿勢のみの相補
    /// フィルタ等）は既定 no-op。
    ///
    /// @design detailed_design.md §3 注3 — bias freeze: capability retained, NOT wired (dropped) [OK]
    virtual void freezeBias() {}

    /// Unfreeze bias estimation (at takeoff). Resumes online bias estimation for the
    /// dynamic flight regime. Default no-op.
    /// バイアス推定の凍結を解除（離陸時）。動的な飛行域でのオンライン推定を再開。既定 no-op。
    ///
    /// @design detailed_design.md §3 注3 — bias unfreeze: capability retained, NOT wired (dropped) [OK]
    virtual void unfreezeBias() {}

    /// Inflate the covariance of the states selected by `state_mask` (bit i = state i in
    /// the estimator's own ordering) back to "uncertain", WITHOUT changing the state
    /// estimate. Lets the state machine declare, at the ground→flight boundary, that the
    /// on-ground convergence is not flight-representative — without the destabilizing full
    /// reset of the state. Default no-op for estimators without an explicit covariance
    /// (e.g. the complementary filter).
    /// state_mask（ビット i = 推定器固有順の状態 i）で選んだ状態の共分散を「不確か」に膨張する。
    /// 状態推定値は変えない。地上→飛行境界で「地上収束は飛行を代表しない」ことを、状態の全
    /// リセット(不安定化)なしに宣言できる。明示的な共分散を持たない推定器(相補フィルタ等)は既定 no-op。
    ///
    /// @design architecture.md §4 — ground→flight covariance handoff (sweep)  [OK]
    virtual void inflateCovariance(uint16_t state_mask) { (void)state_mask; }

    /// Re-read the estimator's parameters from the parameter system and apply
    /// them WITHOUT resetting the filter state — live tuning. Issued via
    /// EstimatorCmd::ReloadParams when a parameter changes (param set callback).
    /// Default no-op for estimators without parameters (complementary filter).
    /// 推定器のパラメータをパラメータシステムから読み直し、フィルタ状態をリセット
    /// せずに適用する — ライブチューニング。パラメータ変更時（param set コールバック）
    /// に EstimatorCmd::ReloadParams 経由で実行。パラメータを持たない推定器
    /// （相補フィルタ）は既定 no-op。
    virtual void reloadParams() {}
};

}  // namespace sf
