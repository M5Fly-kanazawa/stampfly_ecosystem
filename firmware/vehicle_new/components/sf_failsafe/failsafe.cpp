/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file failsafe.cpp
 * @brief Failsafe monitor implementation
 *        フェイルセーフモニター実装
 *
 * @design architecture.md §4 — Failsafe subsystem                     [--]
 * @design requirements.md §9 — Safety requirements                     [--]
 */

#include "failsafe.hpp"
#include "topics.hpp"
#include "esp_log.h"
#include "esp_timer.h"

#include <cmath>

static const char* TAG = "failsafe";

// Gravity constant for impact detection
// 衝撃検出用の重力定数
static constexpr float G = 9.80665f;

namespace sf {

// -----------------------------------------------------------------------------
// init — initialize with default thresholds
// 初期化 — デフォルト閾値で初期化
// -----------------------------------------------------------------------------
void Failsafe::init()
{
    config_ = FailsafeConfig{};
    ESP_LOGI(TAG, "Failsafe initialized (impact=%.1fG, batt=%.1fV, comm=%lums)",
             config_.impact_accel_g, config_.low_battery_v,
             config_.comm_timeout_ms);
}

// -----------------------------------------------------------------------------
// init — initialize with custom thresholds
// 初期化 — カスタム閾値で初期化
// -----------------------------------------------------------------------------
void Failsafe::init(const FailsafeConfig& config)
{
    config_ = config;
    ESP_LOGI(TAG, "Failsafe initialized (custom config)");
}

// -----------------------------------------------------------------------------
// update — run all safety checks
// 更新 — 全安全チェックを実行
// -----------------------------------------------------------------------------
void Failsafe::update()
{
    checkImpact();
    checkCommTimeout();
    checkBattery();
    checkGyroAnomaly();
}

// -----------------------------------------------------------------------------
// checkImpact — detect high-G acceleration event
// 衝撃チェック — 高G加速度イベントを検出
// -----------------------------------------------------------------------------
void Failsafe::checkImpact()
{
    // Read latest IMU data from topic
    // トピックから最新のIMUデータを読み取る
    ImuData imu = sensor_imu.latest();

    // Calculate total acceleration magnitude
    // 合成加速度の大きさを計算
    float mag = std::sqrt(imu.accel[0] * imu.accel[0] +
                          imu.accel[1] * imu.accel[1] +
                          imu.accel[2] * imu.accel[2]);

    float mag_g = mag / G;

    if (mag_g > config_.impact_accel_g && !impact_detected_) {
        impact_detected_ = true;
        raiseAlert(AlertType::IMPACT, AlertSeverity::EMERGENCY);
        ESP_LOGW(TAG, "Impact detected: %.1fG", mag_g);
    }
}

// -----------------------------------------------------------------------------
// checkCommTimeout — detect communication loss
// 通信タイムアウトチェック — 通信途絶を検出
// -----------------------------------------------------------------------------
void Failsafe::checkCommTimeout()
{
    // TODO: Check time since last ESP-NOW packet via Comm component
    // TODO: Commコンポーネント経由で最後のESP-NOWパケットからの経過時間を確認

    // TODO: If timeout exceeded, raise alert
    // TODO: タイムアウト超過時はアラートを発報
}

// -----------------------------------------------------------------------------
// checkBattery — detect low battery voltage
// バッテリーチェック — 低電圧を検出
// -----------------------------------------------------------------------------
void Failsafe::checkBattery()
{
    // Read power data from topic
    // トピックから電源データを読み取る
    PowerData power = sensor_power.latest();

    if (power.voltage > 0.1f && power.voltage < config_.critical_battery_v) {
        // Critical voltage — emergency landing
        // 危険電圧 — 緊急着陸
        if (!low_battery_) {
            raiseAlert(AlertType::LOW_BATTERY, AlertSeverity::EMERGENCY);
            ESP_LOGE(TAG, "Critical battery: %.2fV", power.voltage);
        }
        low_battery_ = true;
    } else if (power.voltage > 0.1f && power.voltage < config_.low_battery_v) {
        // Low voltage — warning
        // 低電圧 — 警告
        if (!low_battery_) {
            raiseAlert(AlertType::LOW_BATTERY, AlertSeverity::WARNING);
            ESP_LOGW(TAG, "Low battery: %.2fV", power.voltage);
        }
        low_battery_ = true;
    }
}

// -----------------------------------------------------------------------------
// checkGyroAnomaly — detect abnormal angular rates
// ジャイロ異常チェック — 異常角速度を検出
// -----------------------------------------------------------------------------
void Failsafe::checkGyroAnomaly()
{
    ImuData imu = sensor_imu.latest();

    // Convert rad/s to deg/s for threshold comparison
    // 閾値比較のためrad/sをdeg/sに変換
    static constexpr float RAD2DEG = 57.2957795f;

    for (int i = 0; i < 3; ++i) {
        float dps = std::fabs(imu.gyro[i]) * RAD2DEG;
        if (dps > config_.gyro_anomaly_dps) {
            raiseAlert(AlertType::GYRO_ANOMALY, AlertSeverity::CRITICAL);
            ESP_LOGW(TAG, "Gyro anomaly: axis=%d, %.0f deg/s", i, dps);
            return;
        }
    }
}

// -----------------------------------------------------------------------------
// raiseAlert — publish alert to system_alert topic
// アラート発報 — system_alertトピックにアラートを発行
// -----------------------------------------------------------------------------
void Failsafe::raiseAlert(AlertType type, AlertSeverity severity)
{
    SystemAlert alert = {};
    alert.type      = static_cast<uint8_t>(type);
    alert.severity  = static_cast<uint8_t>(severity);
    alert.timestamp = static_cast<uint32_t>(esp_timer_get_time());

    system_alert.publish(alert);
}

}  // namespace sf
