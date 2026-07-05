/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file failsafe.cpp
 * @brief Failsafe monitor implementation
 *        フェイルセーフモニター実装
 *
 * @design architecture.md §4 — Failsafe subsystem (wired in PowerTask) [OK]
 * @design requirements.md §9 — Safety requirements                     [OK]
 */

#include "failsafe.hpp"
#include "topics.hpp"
#include "params.hpp"    // safety.* → FailsafeConfig
#include "esp_log.h"
#include "esp_timer.h"

#include <cmath>

static const char* TAG = "failsafe";

// Gravity constant for impact detection (matches the SSOT sf::math::kGravity; kept as a
// local literal to avoid an sf_math dependency in this leaf component)
// 衝撃検出用の重力定数（SSOT sf::math::kGravity と同値。leaf コンポーネントの sf_math 依存を
// 避けるためローカルリテラルで保持）
static constexpr float G = 9.80665f;

namespace sf {

// -----------------------------------------------------------------------------
// raiseAlert — publish an alert to the system_alert topic. File-scope helper
// shared by Failsafe (10 Hz checks) and ImuAnomalyDetector (400 Hz checks).
// raiseAlert — system_alert トピックにアラートを発行する。Failsafe（10Hz 監視）と
// ImuAnomalyDetector（400Hz 監視）が共用するファイルスコープのヘルパ。
// -----------------------------------------------------------------------------
static void raiseAlert(AlertType type, AlertSeverity severity)
{
    SystemAlert alert = {};
    alert.type      = static_cast<uint8_t>(type);
    alert.severity  = static_cast<uint8_t>(severity);
    alert.timestamp = static_cast<uint32_t>(esp_timer_get_time());

    system_alert.publish(alert);
}

// -----------------------------------------------------------------------------
// loadFailsafeConfigFromParams — safety.* parameters → FailsafeConfig.
// Missing parameters keep the struct defaults (get_* leaves the value untouched).
// loadFailsafeConfigFromParams — safety.* パラメータ → FailsafeConfig。
// 存在しないパラメータは struct 既定値のまま（get_* は値を変更しない）。
// -----------------------------------------------------------------------------
FailsafeConfig loadFailsafeConfigFromParams()
{
    FailsafeConfig config;
    params::get_float("safety.impact.accel_g",  config.impact_accel_g);
    params::get_float("safety.impact.gyro_dps", config.gyro_anomaly_dps);
    params::get_float("safety.battery.low_v",   config.low_battery_v);

    float comm_timeout_ms = static_cast<float>(config.comm_timeout_ms);
    if (params::get_float("safety.comm.timeout_ms", comm_timeout_ms)) {
        config.comm_timeout_ms = static_cast<uint32_t>(comm_timeout_ms);
    }
    return config;
}

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
    checkCommTimeout();
    checkBattery();
}

// -----------------------------------------------------------------------------
// checkCommTimeout — detect communication loss
// 通信タイムアウトチェック — 通信途絶を検出
// -----------------------------------------------------------------------------
void Failsafe::checkCommTimeout()
{
    // Judge link loss from the AGE of the latest CommandSetpoint (R16), not by
    // reaching across tasks into the Comm object: comm.cpp stamps every setpoint
    // with esp_timer_get_time() on receive, so a stale stamp means packets stopped.
    // リンク喪失は最新 CommandSetpoint の「経過時間」で判定する（R16）。タスクを
    // またいで Comm オブジェクトに触れるのではなく、comm.cpp が受信時に刻む
    // esp_timer_get_time() の古さで「パケットが途絶えた」と判断する。
    CommandSetpoint cmd = command_setpoint.latest();

    // No packet ever received (timestamp 0) → nothing to time out yet (boot/bench).
    // パケット未受信（timestamp 0）→ まだタイムアウト対象なし（起動時/ベンチ）。
    if (cmd.timestamp == 0) {
        return;
    }

    // uint32_t microsecond stamps wrap every ~71 min; the subtraction is still
    // correct modulo 2^32 for any age far below that, which the 500 ms timeout is.
    // uint32_t マイクロ秒は約71分で巻き戻るが、差分は 2^32 を法として正しく、
    // 500ms タイムアウトはそれより遥かに小さいので問題ない。
    uint32_t now    = static_cast<uint32_t>(esp_timer_get_time());
    uint32_t age_ms = (now - cmd.timestamp) / 1000;

    if (age_ms > config_.comm_timeout_ms) {
        // Raise on the rising edge, then RE-RAISE periodically while the link stays
        // lost (level semantics). A pure edge could be consumed by the StateManager
        // in a state that ignores it (e.g. during TAKEOFF) and never re-evaluated;
        // the periodic re-raise guarantees the failsafe eventually lands the craft.
        // At the 10 Hz update rate, 10 cycles = re-raise every ~1 s.
        // 立ち上がりエッジで発報し、リンク喪失が続く間は周期的に「再発報」する（レベル
        // 意味論）。純粋なエッジだと StateManager が無視する状態（例: TAKEOFF 中）で
        // 消費されたきり再評価されない。周期再発報でフェイルセーフが最終的に必ず着陸
        // させる。10Hz 更新で 10 サイクル = 約1秒毎。
        constexpr uint8_t kReraiseEveryNUpdates = 10;
        if (!comm_lost_) {
            raiseAlert(AlertType::COMM_LOST, AlertSeverity::CRITICAL);
            ESP_LOGW(TAG, "Comm lost: %lu ms since last packet", (unsigned long)age_ms);
            comm_reraise_count_ = 0;
        } else if (++comm_reraise_count_ >= kReraiseEveryNUpdates) {
            raiseAlert(AlertType::COMM_LOST, AlertSeverity::CRITICAL);
            comm_reraise_count_ = 0;
        }
        comm_lost_ = true;
    } else {
        comm_lost_ = false;   // link recovered / リンク復帰
        comm_reraise_count_ = 0;
    }
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

    // No reading yet (boot) — nothing to judge.
    // まだ読み値なし（起動直後）— 判定しない。
    if (power.voltage <= 0.1f) {
        return;
    }

    // Recovery hysteresis: clear both latches only when the voltage rises clearly
    // above the warning threshold, so load-transient sag does not chatter alerts.
    // 回復ヒステリシス: 警告閾値を明確に上回ったときだけ両ラッチをクリアし、負荷過渡の
    // サグでアラートがばたつかないようにする。
    constexpr float kRecoveryHysteresisV = 0.1f;

    if (power.voltage < config_.critical_battery_v) {
        // Critical voltage — emergency. Independent latch: escalates even after the
        // warning already fired (the old shared latch swallowed the escalation).
        // 危険電圧 — 緊急。独立ラッチ: 警告発報済みでもエスカレーションする
        // （旧共有ラッチはエスカレーションを握り潰していた）。
        if (!batt_emergency_) {
            batt_emergency_ = true;
            batt_warning_   = true;
            raiseAlert(AlertType::LOW_BATTERY, AlertSeverity::EMERGENCY);
            ESP_LOGE(TAG, "Critical battery: %.2fV", power.voltage);
        }
    } else if (power.voltage < config_.low_battery_v) {
        // Low voltage — warning. RE-RAISE periodically while latched, mirroring the
        // COMM_LOST divider above and the OLD firmware's PowerTask (which re-warbled
        // lowBatteryWarning() every ~10 s in flight). vehicle funnels every tone
        // through one notify_command consumer, so a SINGLE in-flight beep is missed
        // over motor noise — the warble must REPEAT to be heard while armed.
        // 低電圧 — 警告。ラッチ中は周期的に再発報する（上の COMM_LOST 分周・旧 PowerTask の
        // 10秒ごと再生に倣う）。新ファームは全音を単一の notify_command 消費に集約するため、
        // 飛行中の単発はモータ騒音で埋もれる — armed 中に聞こえるよう警告を反復する。
        constexpr uint8_t kBattReraiseEveryNUpdates = 50;  // ~5 s at the 10 Hz failsafe rate
        if (!batt_warning_) {
            batt_warning_ = true;
            raiseAlert(AlertType::LOW_BATTERY, AlertSeverity::WARNING);
            batt_reraise_count_ = 0;
            ESP_LOGW(TAG, "Low battery: %.2fV", power.voltage);
        } else if (++batt_reraise_count_ >= kBattReraiseEveryNUpdates) {
            raiseAlert(AlertType::LOW_BATTERY, AlertSeverity::WARNING);
            batt_reraise_count_ = 0;
        }
    } else if (power.voltage > config_.low_battery_v + kRecoveryHysteresisV) {
        batt_warning_       = false;
        batt_emergency_     = false;
        batt_reraise_count_ = 0;
    }
}

// =============================================================================
// ImuAnomalyDetector — IMU-rate impact / gyro-anomaly checks (400 Hz, ImuTask)
// ImuAnomalyDetector — IMU レートの衝撃/ジャイロ異常監視（400Hz, ImuTask）
//
// @design requirements.md §9 — threshold × 連続2回 → auto DISARM            [OK]
// =============================================================================

void ImuAnomalyDetector::init()
{
    config_ = FailsafeConfig{};
    ESP_LOGI(TAG, "IMU anomaly detector initialized (impact=%.1fG, gyro=%.0fdps, x%u @400Hz)",
             config_.impact_accel_g, config_.gyro_anomaly_dps,
             config_.consecutive_count);
}

void ImuAnomalyDetector::init(const FailsafeConfig& config)
{
    config_ = config;
    ESP_LOGI(TAG, "IMU anomaly detector initialized (custom config)");
}

void ImuAnomalyDetector::feedSample(const float accel[3], const float gyro[3],
                                    bool armed)
{
    // Disarmed → motors are already off; skip detection and clear the latches so
    // the next armed flight is protected again. Ground handling (pickup/transport
    // after a crash) produces benign high-G that must not alert.
    // disarm 中 → モータは既に停止。検出をスキップしラッチをクリアして、次の armed
    // 飛行で再び保護が効くようにする。地上ハンドリング（墜落後の拾い上げ・運搬）の
    // 無害な高Gで発報してはならない。
    if (!armed) {
        impact_count_    = 0;
        impact_detected_ = false;
        gyro_count_      = 0;
        gyro_suppress_   = 0;
        return;
    }

    // --- Impact: |a| over threshold for consecutive_count SAMPLES (= 5 ms at
    // 400 Hz — fast enough for a real crash spike, debounced against noise).
    // Latched until disarm so one crash raises exactly one alert.
    // --- 衝撃: |a| が連続 consecutive_count「サンプル」閾値超（400Hz で 5ms —
    // 実墜落スパイクに十分速く、ノイズはデバウンス）。disarm までラッチし、
    // 1回の墜落でアラートは1回だけ。
    const float mag_g = std::sqrt(accel[0] * accel[0] +
                                  accel[1] * accel[1] +
                                  accel[2] * accel[2]) / G;
    if (mag_g > config_.impact_accel_g) {
        if (++impact_count_ >= config_.consecutive_count && !impact_detected_) {
            impact_detected_ = true;
            raiseAlert(AlertType::IMPACT, AlertSeverity::EMERGENCY);
            ESP_LOGW(TAG, "Impact detected: %.1fG (x%u @400Hz)",
                     mag_g, impact_count_);
        }
    } else {
        impact_count_ = 0;
    }

    // --- Gyro anomaly: any axis over threshold for consecutive_count samples.
    // Re-raise while the anomaly persists, but at most once per second (the old
    // 10 Hz version re-raised every 200 ms; at 400 Hz an unsuppressed re-raise
    // would flood the alert queue and the log).
    // --- ジャイロ異常: いずれかの軸が連続 consecutive_count サンプル閾値超。
    // 異常継続中は再発報するが最大毎秒1回（旧 10Hz 版は 200ms 毎。400Hz で
    // 無抑制に再発報するとアラートキューとログが氾濫する）。
    static constexpr float    kRadToDeg          = 57.2957795f;
    static constexpr uint16_t kReraiseSuppressSamples = 400;   // 1 s at 400 Hz
    float max_dps = 0.0f;
    int   max_axis = 0;
    for (int i = 0; i < 3; ++i) {
        const float dps = std::fabs(gyro[i]) * kRadToDeg;
        if (dps > max_dps) { max_dps = dps; max_axis = i; }
    }

    if (gyro_suppress_ > 0) {
        --gyro_suppress_;
    }
    if (max_dps > config_.gyro_anomaly_dps) {
        if (++gyro_count_ >= config_.consecutive_count && gyro_suppress_ == 0) {
            raiseAlert(AlertType::GYRO_ANOMALY, AlertSeverity::CRITICAL);
            ESP_LOGW(TAG, "Gyro anomaly: axis=%d, %.0f deg/s (x%u @400Hz)",
                     max_axis, max_dps, gyro_count_);
            gyro_count_    = 0;
            gyro_suppress_ = kReraiseSuppressSamples;
        }
    } else {
        gyro_count_ = 0;
    }
}

}  // namespace sf
