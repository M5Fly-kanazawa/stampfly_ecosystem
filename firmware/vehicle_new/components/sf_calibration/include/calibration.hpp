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
 * @design architecture.md §3 — Calibration subsystem                   [--]
 * @design detailed_design.md §12 — Calibration procedures              [--]
 * @design coding_and_education.md §2 — Bilingual comments               [--]
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

/// Calibration manager
/// キャリブレーション管理
class CalibrationMgr {
public:
    /// Initialize calibration system (load from NVS)
    /// キャリブレーションシステムを初期化する（NVSから読み込み）
    void init();

    /// Start gyro bias calibration (collect N samples)
    /// ジャイロバイアスキャリブレーションを開始する（Nサンプル収集）
    void startGyroCal(uint32_t num_samples = 1000);

    /// Feed one IMU sample during calibration
    /// キャリブレーション中にIMUサンプルを1つ入力する
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

    CalibrationData data_ = {};
    bool     calibrating_    = false;
    uint32_t target_samples_ = 0;     // Target sample count  / 目標サンプル数
    uint32_t sample_count_   = 0;     // Current sample count / 現在のサンプル数
    double   gyro_sum_[3]    = {};    // Accumulator for gyro / ジャイロ累積器
    double   accel_sum_[3]   = {};    // Accumulator for accel / 加速度累積器
};

}  // namespace sf
