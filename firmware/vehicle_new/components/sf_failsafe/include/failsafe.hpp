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
 * @design requirements.md §9 — Safety requirements                     [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#pragma once

#include "flight_state.hpp"
#include <cstdint>

namespace sf {

/// Failsafe configuration thresholds (requirements §9)
/// フェイルセーフ設定閾値（要件§9）
struct FailsafeConfig {
    float impact_accel_g     = 3.0f;     // Impact threshold [G] (§9)     / 衝撃閾値
    float low_battery_v      = 3.4f;     // LiPo low-voltage warning [V] (§9) / 低電圧警告
    float critical_battery_v = 3.0f;     // Critical battery [V] — emergency land (extra safety net below §9 warning) / 危険電圧（§9警告の下の追加安全網）
    uint32_t comm_timeout_ms = 500;      // Comm timeout [ms] (§9)        / 通信タイムアウト
    float gyro_anomaly_dps   = 800.0f;   // Gyro anomaly [deg/s] (§9)     / ジャイロ異常閾値
    uint8_t consecutive_count = 2;       // §9 "× 連続2回" debounce for impact/gyro / 連続検出回数
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

    /// Run all safety checks. Called from PowerTask at 10 Hz.
    /// 全安全チェックを実行する。PowerTask から 10Hz で呼ばれる。
    void update();

private:
    /// Check for impact (high-G event); armed flights only
    /// 衝撃をチェックする（高G検出）; armed 飛行中のみ
    void checkImpact(bool armed);

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
    // Separate warning/emergency battery latches: a shared latch meant that once
    // the 3.4V warning fired, the 3.0V emergency could never escalate. Both clear
    // on voltage recovery (with hysteresis) so a later sag re-raises.
    // 電池の警告/危険ラッチを分離: 共有ラッチだと 3.4V 警告発報後に 3.0V 危険へ
    // エスカレーションできなかった。電圧回復（ヒステリシス付き）で両方クリアし、
    // 後のサグで再発報できるようにする。
    bool batt_warning_     = false;   // 3.4V warning latched / 低電圧警告ラッチ
    bool batt_emergency_   = false;   // 3.0V emergency latched / 危険電圧ラッチ
    bool impact_detected_  = false;   // Impact flag (cleared while disarmed) / 衝撃検出フラグ（disarm中クリア）
    uint8_t impact_count_  = 0;       // consecutive over-threshold impact cycles / 連続衝撃回数
    uint8_t gyro_count_    = 0;       // consecutive over-threshold gyro cycles / 連続ジャイロ異常回数
    uint8_t comm_reraise_count_ = 0;  // re-raise divider while comm stays lost / 喪失継続中の再発報分周
};

}  // namespace sf
