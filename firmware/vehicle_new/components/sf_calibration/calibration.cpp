/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file calibration.cpp
 * @brief Calibration manager implementation
 *        キャリブレーション管理の実装
 *
 * @design architecture.md §3 — Calibration subsystem                   [--]
 * @design detailed_design.md §12 — Calibration procedures              [--]
 */

#include "calibration.hpp"
#include "esp_log.h"

// TODO: Include NVS headers
// TODO: NVSヘッダをインクルード
// #include "nvs_flash.h"
// #include "nvs.h"

#include <cmath>
#include <cstring>

static const char* TAG = "calibration";

// NVS namespace and keys
// NVS名前空間とキー
static const char* NVS_NAMESPACE = "sf_cal";
static const char* NVS_KEY       = "cal_data";

// Gravity constant
// 重力定数
static constexpr float G = 9.80665f;

namespace sf {

// -----------------------------------------------------------------------------
// init — load calibration from NVS
// 初期化 — NVSからキャリブレーションを読み込む
// -----------------------------------------------------------------------------
void CalibrationMgr::init()
{
    if (loadFromNvs()) {
        ESP_LOGI(TAG, "Calibration loaded from NVS");
        ESP_LOGI(TAG, "  gyro_bias: [%.6f, %.6f, %.6f]",
                 data_.gyro_bias[0], data_.gyro_bias[1], data_.gyro_bias[2]);
    } else {
        ESP_LOGW(TAG, "No calibration in NVS, using zeros");
        memset(&data_, 0, sizeof(data_));
    }
}

// -----------------------------------------------------------------------------
// startGyroCal — begin gyro bias calibration
// ジャイロキャリブレーション開始 — ジャイロバイアスキャリブレーションを開始
// -----------------------------------------------------------------------------
void CalibrationMgr::startGyroCal(uint32_t num_samples)
{
    calibrating_    = true;
    target_samples_ = num_samples;
    sample_count_   = 0;
    memset(gyro_sum_,  0, sizeof(gyro_sum_));
    memset(accel_sum_, 0, sizeof(accel_sum_));

    ESP_LOGI(TAG, "Gyro calibration started (%lu samples)", num_samples);
}

// -----------------------------------------------------------------------------
// feedSample — accumulate one IMU sample
// サンプル入力 — IMUサンプルを1つ蓄積
// -----------------------------------------------------------------------------
bool CalibrationMgr::feedSample(const float gyro[3], const float accel[3])
{
    if (!calibrating_) {
        return false;
    }

    // Accumulate gyro and accel values
    // ジャイロと加速度の値を蓄積
    for (int i = 0; i < 3; ++i) {
        gyro_sum_[i]  += static_cast<double>(gyro[i]);
        accel_sum_[i] += static_cast<double>(accel[i]);
    }

    sample_count_++;

    // Check if target reached
    // 目標に達したか確認
    if (sample_count_ >= target_samples_) {
        computeAverages();
        computeLevelOffset();
        data_.valid  = true;
        calibrating_ = false;

        ESP_LOGI(TAG, "Calibration complete");
        ESP_LOGI(TAG, "  gyro_bias: [%.6f, %.6f, %.6f]",
                 data_.gyro_bias[0], data_.gyro_bias[1], data_.gyro_bias[2]);
        return true;
    }

    return false;
}

// -----------------------------------------------------------------------------
// computeAverages — calculate mean from accumulated samples
// 平均計算 — 蓄積サンプルから平均を計算
// -----------------------------------------------------------------------------
void CalibrationMgr::computeAverages()
{
    double n = static_cast<double>(sample_count_);
    for (int i = 0; i < 3; ++i) {
        data_.gyro_bias[i]  = static_cast<float>(gyro_sum_[i]  / n);
        data_.accel_bias[i] = static_cast<float>(accel_sum_[i] / n);
    }

    // Remove gravity from Z-axis accel bias (assuming Z-down)
    // Z軸加速度バイアスから重力を除去（Z-down仮定）
    data_.accel_bias[2] -= G;
}

// -----------------------------------------------------------------------------
// computeLevelOffset — derive roll/pitch offset from gravity vector
// レベルオフセット計算 — 重力ベクトルからロール/ピッチオフセットを導出
// -----------------------------------------------------------------------------
void CalibrationMgr::computeLevelOffset()
{
    // Gravity vector at rest: [ax, ay, az]
    // 静止時の重力ベクトル: [ax, ay, az]
    float ax = data_.accel_bias[0];
    float ay = data_.accel_bias[1];
    float az = data_.accel_bias[2] + G;  // Add back gravity / 重力を戻す

    // Roll offset = atan2(ay, az)
    // ロールオフセット = atan2(ay, az)
    data_.level_offset[0] = std::atan2(ay, az);

    // Pitch offset = atan2(-ax, sqrt(ay² + az²))
    // ピッチオフセット = atan2(-ax, sqrt(ay² + az²))
    data_.level_offset[1] = std::atan2(-ax, std::sqrt(ay * ay + az * az));

    ESP_LOGI(TAG, "  level_offset: [%.4f, %.4f] rad",
             data_.level_offset[0], data_.level_offset[1]);
}

// -----------------------------------------------------------------------------
// saveToNvs — persist calibration data to NVS
// NVS保存 — キャリブレーションデータをNVSに永続化
// -----------------------------------------------------------------------------
void CalibrationMgr::saveToNvs()
{
    // TODO: nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle)
    // TODO: nvs_set_blob(handle, NVS_KEY, &data_, sizeof(data_))
    // TODO: nvs_commit(handle)
    // TODO: nvs_close(handle)

    ESP_LOGI(TAG, "Calibration saved to NVS (stub)");
}

// -----------------------------------------------------------------------------
// loadFromNvs — load calibration data from NVS
// NVS読み込み — NVSからキャリブレーションデータを読み込む
// -----------------------------------------------------------------------------
bool CalibrationMgr::loadFromNvs()
{
    // TODO: nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle)
    // TODO: nvs_get_blob(handle, NVS_KEY, &data_, &len)
    // TODO: nvs_close(handle)

    ESP_LOGI(TAG, "NVS load (stub)");
    return false;  // No data yet / まだデータなし
}

}  // namespace sf
