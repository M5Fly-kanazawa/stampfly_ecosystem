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
#include "nvs_flash.h"
#include "nvs.h"

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

    // Remove gravity from the Z-axis accel bias. At level rest the accelerometer
    // reads −G on body Z (gravity −9.8 on body-down, NED-consistent), so the true
    // bias is the mean PLUS G. (The old "-= G" yielded −2G — a latent sign bug:
    // the accelerometer is in the −g convention, not +g.)
    // Z軸加速度バイアスから重力を除去。水平静止で加速度計は機体 Z に −G を返す
    // （機体下方に重力 −9.8、NED 整合）ので、真のバイアスは平均＋G。
    // （旧 "-= G" は −2G を生む潜在符号バグ＝加速度計は −g 規約で +g ではない。）
    data_.accel_bias[2] += G;
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
//
// Pre-condition: nvs_flash_init() has been called from app_main() (Phase 0).
// We open the dedicated namespace "sf_cal", store data_ as a blob under
// the key "cal_data", commit, and close.
//
// 前提条件: app_main() (Phase 0) で nvs_flash_init() 済み。
// 専用 namespace "sf_cal" を開いて、キー "cal_data" 下に data_ を blob
// として保存、commit、close する流れ。
// -----------------------------------------------------------------------------
void CalibrationMgr::saveToNvs()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(handle, NVS_KEY, &data_, sizeof(data_));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob failed: %s", esp_err_to_name(err));
        nvs_close(handle);
        return;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Calibration saved to NVS (%u bytes)",
                 static_cast<unsigned>(sizeof(data_)));
    }
    nvs_close(handle);
}

// -----------------------------------------------------------------------------
// loadFromNvs — load calibration data from NVS
// NVS読み込み — NVSからキャリブレーションデータを読み込む
// -----------------------------------------------------------------------------
//
// Returns true only when:
//   1. nvs_open succeeds (namespace exists)
//   2. nvs_get_blob succeeds with the exact stored size
//   3. data_.valid flag is true (sentinel for genuine calibration)
//
// 以下の全てを満たすときのみ true を返す:
//   1. nvs_open 成功 (namespace 存在)
//   2. nvs_get_blob 成功、保存時と同サイズ
//   3. data_.valid フラグが true (キャリブ完了の sentinel)
// -----------------------------------------------------------------------------
bool CalibrationMgr::loadFromNvs()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        // First boot or never-saved is the common case — log at INFO not ERROR.
        // 初回起動 or 未保存は通常状況、INFO レベルでログ
        ESP_LOGI(TAG, "NVS namespace '%s' not found (%s)",
                 NVS_NAMESPACE, esp_err_to_name(err));
        return false;
    }

    size_t required = sizeof(data_);
    err = nvs_get_blob(handle, NVS_KEY, &data_, &required);
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGI(TAG, "NVS key '%s' not found (%s)",
                 NVS_KEY, esp_err_to_name(err));
        return false;
    }

    if (required != sizeof(data_)) {
        // Size mismatch means the struct layout changed since save. Treat
        // as invalid; user must recalibrate.
        // サイズ不一致は struct レイアウト変更を意味する。再キャリブ要求。
        ESP_LOGW(TAG, "NVS blob size mismatch (got %u, expected %u) — discarding",
                 static_cast<unsigned>(required),
                 static_cast<unsigned>(sizeof(data_)));
        std::memset(&data_, 0, sizeof(data_));
        return false;
    }

    if (!data_.valid) {
        ESP_LOGI(TAG, "NVS calibration blob present but valid=false");
        return false;
    }

    return true;
}

}  // namespace sf
