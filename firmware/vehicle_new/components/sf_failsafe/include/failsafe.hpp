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

/// Build a FailsafeConfig from the safety.* parameters (params SSOT). Used by
/// PowerTask (Failsafe) and ImuTask (ImuAnomalyDetector) at init so the
/// thresholds actually come from the parameter system instead of silently
/// using the struct defaults.
/// safety.* パラメータ（params SSOT）から FailsafeConfig を構築する。PowerTask
/// （Failsafe）と ImuTask（ImuAnomalyDetector）が init 時に使い、閾値が struct
/// 既定値の暗黙使用でなく実際にパラメータシステム由来になるようにする。
FailsafeConfig loadFailsafeConfigFromParams();

/// Failsafe monitor: comm-timeout and battery checks (10 Hz, PowerTask).
/// The IMU-rate checks (impact / gyro anomaly) live in ImuAnomalyDetector
/// below, fed per-sample by ImuTask at 400 Hz — a 10 Hz latest() peek missed
/// millisecond-scale crash spikes almost certainly.
/// フェイルセーフモニター: 通信タイムアウトと電池の監視（10Hz, PowerTask）。
/// IMU レートの監視（衝撃/ジャイロ異常）は下の ImuAnomalyDetector が担い、
/// ImuTask が 400Hz でサンプル毎に与える — 10Hz の latest() 覗き見では
/// ミリ秒オーダーの墜落スパイクをほぼ確実に取りこぼしていた。
class Failsafe {
public:
    /// Initialize failsafe with default thresholds
    /// デフォルト閾値でフェイルセーフを初期化する
    void init();

    /// Initialize with custom thresholds
    /// カスタム閾値で初期化する
    void init(const FailsafeConfig& config);

    /// Run the 10 Hz checks (comm timeout, battery). Called from PowerTask.
    /// 10Hz の監視（通信タイムアウト・電池）を実行する。PowerTask から呼ぶ。
    void update();

private:
    /// Check communication timeout
    /// 通信タイムアウトをチェックする
    void checkCommTimeout();

    /// Check battery voltage
    /// バッテリー電圧をチェックする
    void checkBattery();

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
    uint8_t comm_reraise_count_ = 0;  // re-raise divider while comm stays lost / 喪失継続中の再発報分周
    uint8_t batt_reraise_count_ = 0;  // re-raise divider for the in-flight low-battery warble / 低電圧警告の再発報分周
};

/// IMU-rate anomaly detector: impact (high-G) and gyro-anomaly (tumble) checks,
/// fed per-sample by ImuTask at 400 Hz. requirements §9 "threshold × 連続2回"
/// is interpreted at the sample rate: 2 consecutive samples = 5 ms — fast enough
/// for a real crash spike, debounced against single-sample noise. Owned by
/// ImuTask (its own instance; thresholds shared via FailsafeConfig).
/// IMU レート異常検出器: 衝撃（高G）とジャイロ異常（タンブル）の監視。ImuTask が
/// 400Hz でサンプル毎に与える。要件§9「閾値 × 連続2回」をサンプルレートで解釈し、
/// 連続2サンプル = 5ms — 実際の墜落スパイクに十分速く、単発ノイズは除く。
/// ImuTask が所有（独自インスタンス。閾値は FailsafeConfig を共用）。
class ImuAnomalyDetector {
public:
    /// Initialize with default thresholds / デフォルト閾値で初期化する
    void init();

    /// Initialize with custom thresholds / カスタム閾値で初期化する
    void init(const FailsafeConfig& config);

    /// Feed one IMU sample (400 Hz). accel [m/s²], gyro [rad/s], body frame.
    /// While disarmed, detection is skipped and the latches clear (ground
    /// handling produces benign high-G; re-fly regains protection).
    /// IMU サンプルを1つ与える（400Hz）。accel [m/s²]・gyro [rad/s]・機体座標。
    /// disarm 中は検出をスキップしラッチをクリア（地上ハンドリングは無害な高Gを
    /// 生む。再飛行で保護が復活する）。
    void feedSample(const float accel[3], const float gyro[3], bool armed);

private:
    FailsafeConfig config_;
    bool     impact_detected_ = false;  // latched while armed / armed 中ラッチ
    uint16_t impact_count_    = 0;      // consecutive over-threshold samples / 連続閾値超サンプル
    uint16_t gyro_count_      = 0;      // consecutive over-threshold samples / 連続閾値超サンプル
    uint16_t gyro_suppress_   = 0;      // re-raise suppression countdown / 再発報抑制カウントダウン
};

}  // namespace sf
