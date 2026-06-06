/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file failsafe.hpp
 * @brief Failsafe monitor — impact, comm timeout, voltage checks
 *        フェイルセーフモニター — 衝撃検出、通信タイムアウト、電圧監視
 *
 * Monitors safety-critical conditions and publishes SystemAlert
 * to the system_alert topic when thresholds are exceeded.
 *
 * 安全に関わる重要な状態を監視し、閾値を超えた場合に
 * system_alertトピックへSystemAlertを発行する。
 *
 * @design architecture.md §4 — Failsafe subsystem (wired in PowerTask) [OK]
 * @design requirements.md §9 — Safety requirements                     [--]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#pragma once

#include "flight_state.hpp"
#include <cstdint>

namespace sf {

/// Failsafe configuration thresholds
/// フェイルセーフ設定閾値
struct FailsafeConfig {
    float impact_accel_g     = 4.0f;     // Impact threshold [G]     / 衝撃閾値
    float low_battery_v      = 3.3f;     // Low battery [V]          / 低電圧閾値
    float critical_battery_v = 3.0f;     // Critical battery [V]     / 危険電圧閾値
    uint32_t comm_timeout_ms = 500;      // Comm timeout [ms]        / 通信タイムアウト
    float gyro_anomaly_dps   = 1000.0f;  // Gyro anomaly [deg/s]     / ジャイロ異常閾値
};

/// Failsafe monitor: detect unsafe conditions and raise alerts
/// フェイルセーフモニター: 危険な状態を検出しアラートを発報
class Failsafe {
public:
    /// Initialize failsafe with default thresholds
    /// デフォルト閾値でフェイルセーフを初期化する
    void init();

    /// Initialize with custom thresholds
    /// カスタム閾値で初期化する
    void init(const FailsafeConfig& config);

    /// Run all safety checks (call at ~50Hz)
    /// 全安全チェックを実行する（約50Hzで呼ぶ）
    void update();

private:
    /// Check for impact (high-G event)
    /// 衝撃をチェックする（高G検出）
    void checkImpact();

    /// Check communication timeout
    /// 通信タイムアウトをチェックする
    void checkCommTimeout();

    /// Check battery voltage
    /// バッテリー電圧をチェックする
    void checkBattery();

    /// Check gyro angular rate anomaly
    /// ジャイロ角速度の異常をチェックする
    void checkGyroAnomaly();

    /// Publish alert to system_alert topic
    /// system_alertトピックにアラートを発行する
    void raiseAlert(AlertType type, AlertSeverity severity);

    FailsafeConfig config_;
    bool comm_lost_        = false;   // Comm lost flag / 通信途絶フラグ
    bool low_battery_      = false;   // Low battery flag / 低電圧フラグ
    bool impact_detected_  = false;   // Impact flag / 衝撃検出フラグ
};

}  // namespace sf
