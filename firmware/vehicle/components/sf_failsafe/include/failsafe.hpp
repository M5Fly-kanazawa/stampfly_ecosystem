/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file failsafe.hpp
 * @brief Failsafe monitor — impact, comm timeout, downward-ToF silence, voltage checks
 *        フェイルセーフモニター — 衝撃検出、通信タイムアウト、下向き ToF 出力停止、電圧監視
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
#include "sensor_freshness.hpp"
#include "battery_alert_policy.hpp"
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
/// ミリ秒オーダーの墜落スパイクをほぼ確実に取りこぼす。
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

    /// Raise COMM_LOST on the rising edge and re-raise it while the link stays lost
    /// 通信途絶の立ち上がりで COMM_LOST を発報し、途絶が続く間は再発報する
    void raiseCommLost(uint32_t age_ms);

    /// Judge the downward-ToF silence EPISODE (no NEW sample for sf::limits::kTofSilenceUs, only
    /// invalid samples while flying, or the estimator has not re-acquired the ToF: facts from
    /// tof_health), publish the level on failsafe_status every update and raise
    /// AlertType::TOF_SILENT while armed.
    /// 下向き ToF の出力停止「エピソード」（sf::limits::kTofSilenceUs の間新サンプル無し、飛行中に
    /// 無効サンプルだけ、または推定器が ToF を取り込み直していない: tof_health の事実）を判定し、
    /// 毎回 failsafe_status にレベルを発行し、ARM 中は AlertType::TOF_SILENT を発報する。
    void checkTofSilence();

    /// Check battery voltage
    /// バッテリー電圧をチェックする
    void checkBattery();

    /// Re-raise period of a standing alert: the checks run at 10 Hz, so 10 updates = about once a
    /// second (level semantics: an alert consumed in a state that ignores it is not lost).
    /// 継続中のアラートの再発報周期: 監視は 10 Hz なので 10 回 = 約 1 秒ごと（レベル意味論:
    /// 無視する状態で消費されたアラートが失われない）。
    static constexpr uint8_t kReraiseEveryNUpdates = 10;

    FailsafeConfig config_;
    bool comm_lost_        = false;   // Comm lost flag / 通信途絶フラグ
    sf::SilenceWatch comm_watch_;     // Time since the last new setpoint / 最後の新 setpoint からの経過
    sf::SilenceWatch tof_watch_;      // Time since the downward ToF last produced a NEW sample / 下向き ToF の最後の新サンプルからの経過
    uint8_t tof_alert_cooldown_ = 0;  // updates until the next TOF_SILENT (re-)raise; 0 = raise now / 次の TOF_SILENT 発報までの更新回数（0=今発報）
    // Battery WARNING / EMERGENCY timing (re-raised while the voltage stays low, see battery_alert_policy.hpp).
    // 電池の WARNING / EMERGENCY のタイミング（電圧が低い間は再発報する。battery_alert_policy.hpp 参照）。
    BatteryAlertPolicy battery_policy_;
    uint8_t comm_reraise_count_ = 0;  // re-raise divider while comm stays lost / 喪失継続中の再発報分周
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
