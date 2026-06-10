/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file calibration.hpp
 * @brief Calibration manager — gyro bias, level correction, NVS storage
 *        キャリブレーション管理 — ジャイロバイアス、レベル補正、NVS保存
 *
 * Manages sensor calibration routines:
 * - Gyro bias: average N samples at rest, subtract offset
 * - Accelerometer level: measure gravity vector, compute tilt offset
 * - NVS persistence: save/load calibration data across reboots
 *
 * センサキャリブレーションルーチンを管理する:
 * - ジャイロバイアス: 静止時にNサンプルの平均を取り、オフセットを減算
 * - 加速度計レベル: 重力ベクトルを測定し、傾斜オフセットを計算
 * - NVS永続化: リブート間でキャリブレーションデータを保存/読み込み
 *
 * @design architecture.md §2 — Calibration (responsibility #11)        [OK]
 * @design detailed_design.md §3 — onEnter(IDLE_GROUND): boot calibration [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#pragma once

#include <cstdint>

namespace sf {

/// Calibration data stored in NVS
/// NVSに保存されるキャリブレーションデータ
struct CalibrationData {
    float gyro_bias[3];      // Gyro bias [rad/s]      / ジャイロバイアス
    float accel_bias[3];     // Accel bias [m/s²]      / 加速度バイアス
    float level_offset[2];   // Level offset [rad] R,P  / レベルオフセット
    bool  valid;             // Data validity flag      / データ有効フラグ
};

/// Stillness gate for sample accumulation. Calibration is only valid at rest:
/// averaging through motion (carrying the craft to the bench, picking it up
/// after a crash) yields a garbage bias and a wrong level reference. Any motion
/// DISCARDS the partial accumulation and restarts it, so the result is
/// deterministic — "the average of N verified-still samples" — no matter when
/// the human actually sets the craft down. Defaults are the flight-proven
/// legacy vehicle/ StationaryDetector thresholds; the accel-norm window is
/// widened because the boot path sees RAW (bias-uncorrected) accelerometer
/// readings (BMI270 offset spec is up to ~±0.5 m/s²).
/// サンプル蓄積の静止ゲート。校正は静止時のみ有効: 運搬中・墜落後の拾い上げ中の
/// 動きを平均するとバイアスも水平基準もゴミになる。動きを検出したら蓄積を破棄して
/// やり直すため、人間がいつ機体を置いても結果は「静止確認済み N サンプルの平均」で
/// 決定的になる。既定値は旧 vehicle/ StationaryDetector の実績閾値。加速度ノルム窓は
/// 起動時が生（バイアス未補正）の読みのため広げてある（BMI270 のオフセット仕様は
/// 最大 ±0.5 m/s² 程度）。
struct StillnessConfig {
    // The EMA |gyro| gate sees RAW gyro (bias not yet known at boot — that is what
    // is being calibrated), so it must tolerate the sensor offset spec (BMI270:
    // up to ~±1 dps/axis ≈ 0.03 rad/s magnitude). 0.05 still catches any real
    // handling motion (≳0.1 rad/s); the bias-INSENSITIVE gyro window variance
    // below is the precise judge.
    // EMA |gyro| ゲートは「生」のジャイロを見る（起動時はバイアス未知 — それを校正
    // する最中）ため、センサのオフセット仕様（BMI270: 軸あたり最大約±1 dps ≈ 合計
    // 0.03 rad/s）を許容する必要がある。0.05 でも実際の取り扱い動作（≳0.1 rad/s）は
    // 捉える。精密判定はバイアス不感な下のジャイロ窓内分散が担う。
    float gyro_max        = 0.05f;  // [rad/s] max EMA-filtered RAW |gyro| / EMA後の生|gyro|上限
    float gyro_var_max    = 1.0e-3f;// [(rad/s)²] max gyro variance over the window
                                    // (bias-insensitive; rest noise ≈3e-4, hand tremor ≫)
                                    // 窓内ジャイロ分散上限（バイアス不感。静止ノイズ≈3e-4、手持ちは≫）
    float accel_norm_min  = 9.3f;   // [m/s²] min EMA-filtered |accel| / EMA後の|accel|下限
    float accel_norm_max  = 10.3f;  // [m/s²] max EMA-filtered |accel| / EMA後の|accel|上限
    float accel_var_max   = 0.05f;  // [(m/s²)²] max accel variance over the window / 窓内分散上限
    float ema_alpha       = 0.05f;  // EMA factor (~3 Hz cutoff at 400 Hz) / EMA係数
};

/// Calibration manager
/// キャリブレーション管理
class CalibrationMgr {
public:
    /// Initialize calibration system (load from NVS)
    /// キャリブレーションシステムを初期化する（NVSから読み込み）
    void init();

    /// Start gyro bias calibration (collect N verified-still samples)
    /// ジャイロバイアスキャリブレーションを開始する（静止確認済みNサンプル収集）
    void startGyroCal(uint32_t num_samples = 1000,
                      const StillnessConfig& stillness = {});

    /// Feed one IMU sample during calibration. Samples taken while the craft
    /// is moving are rejected and restart the accumulation (see StillnessConfig).
    /// キャリブレーション中にIMUサンプルを1つ入力する。動いている間のサンプルは
    /// 拒否され、蓄積をやり直す（StillnessConfig 参照）。
    ///
    /// @return true when calibration is complete / 完了時にtrue
    bool feedSample(const float gyro[3], const float accel[3]);

    /// Get current calibration data
    /// 現在のキャリブレーションデータを取得する
    const CalibrationData& data() const { return data_; }

    /// Check if calibration is in progress
    /// キャリブレーション中か確認する
    bool isCalibrating() const { return calibrating_; }

    /// Save calibration data to NVS
    /// キャリブレーションデータをNVSに保存する
    void saveToNvs();

    /// Load calibration data from NVS
    /// NVSからキャリブレーションデータを読み込む
    bool loadFromNvs();

private:
    /// Compute averages from accumulated samples
    /// 蓄積サンプルから平均値を計算する
    void computeAverages();

    /// Compute level offset from gravity vector
    /// 重力ベクトルからレベルオフセットを計算する
    void computeLevelOffset();

    /// Update the stillness EMAs and judge whether the craft is at rest now
    /// 静止EMAを更新し、いま静止しているかを判定する
    bool updateStillness(const float gyro[3], const float accel[3]);

    /// Discard the partial accumulation (motion detected) and start over
    /// 部分蓄積を破棄して（動き検出）やり直す
    void restartAccumulation();

    /// Total variance of an accumulated 3-axis window (Σ per-axis variance)
    /// 蓄積した3軸窓の分散合計（軸ごとの分散の和）
    static float windowVariance(const double sum[3], const double sq_sum[3],
                                uint32_t count);

    CalibrationData data_ = {};
    bool     nvs_loaded_once_ = false; // NVS read happens once per boot (see init) / NVS読みは起動1回（init参照）
    bool     calibrating_    = false;
    uint32_t target_samples_ = 0;     // Target sample count  / 目標サンプル数
    uint32_t sample_count_   = 0;     // Current sample count / 現在のサンプル数
    double   gyro_sum_[3]    = {};    // Accumulator for gyro / ジャイロ累積器
    double   accel_sum_[3]   = {};    // Accumulator for accel / 加速度累積器
    double   gyro_sq_sum_[3]  = {};   // Gyro sum-of-squares (variance) / ジャイロ二乗和（分散用）
    double   accel_sq_sum_[3] = {};   // Accel sum-of-squares (variance) / 加速度二乗和（分散用）

    // Stillness gate state / 静止ゲートの状態
    StillnessConfig still_ = {};
    bool     ema_primed_     = false; // First sample primes the EMAs / 初サンプルでEMA初期化
    float    gyro_mag_ema_   = 0;     // EMA of |gyro| [rad/s]  / |gyro| のEMA
    float    accel_norm_ema_ = 0;     // EMA of |accel| [m/s²]  / |accel| のEMA
    uint32_t restart_count_  = 0;     // Motion restarts (diagnostics) / 動きによるやり直し回数
};

}  // namespace sf
