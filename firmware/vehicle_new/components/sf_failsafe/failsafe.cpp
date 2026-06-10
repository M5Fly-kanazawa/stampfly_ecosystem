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
 * @design architecture.md §4 — Failsafe subsystem (wired in PowerTask) [OK]
 * @design requirements.md §9 — Safety requirements                     [OK]
 */

#include "failsafe.hpp"
#include "topics.hpp"
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
    // Impact detection only matters while armed: disarmed ground handling (pickup,
    // transport after a crash) produces benign high-G that must not alert, and the
    // latch must clear before the next flight so re-fly keeps impact protection.
    // 衝撃検出は armed 中のみ意味を持つ: disarm 中の地上ハンドリング（拾い上げ・運搬）は
    // 無害な高Gを生むので発報してはならず、ラッチも次の飛行前にクリアして再飛行でも
    // 衝撃保護が効くようにする。
    const SystemMode mode = system_mode.latest();
    const bool armed = isArmed(static_cast<FlightState>(mode.state));

    checkImpact(armed);
    checkCommTimeout();
    checkBattery();
    checkGyroAnomaly();
}

// -----------------------------------------------------------------------------
// checkImpact — detect high-G acceleration event
// 衝撃チェック — 高G加速度イベントを検出
// -----------------------------------------------------------------------------
void Failsafe::checkImpact(bool armed)
{
    // Disarmed → motors are already off; skip detection and clear the latch so the
    // next armed flight is protected again (the old permanent latch disabled impact
    // protection for every flight after the first alert — crash_refly regression).
    // disarm 中 → モータは既に停止。検出をスキップしラッチをクリアして、次の armed 飛行
    // で再び保護が効くようにする（旧実装の永久ラッチは初回発報後の全飛行で衝撃保護を
    // 無効化していた — crash_refly の退行）。
    if (!armed) {
        impact_count_ = 0;
        impact_detected_ = false;
        return;
    }

    // Read latest IMU data from topic
    // トピックから最新のIMUデータを読み取る
    ImuData imu = sensor_imu.latest();

    // Calculate total acceleration magnitude
    // 合成加速度の大きさを計算
    float mag = std::sqrt(imu.accel[0] * imu.accel[0] +
                          imu.accel[1] * imu.accel[1] +
                          imu.accel[2] * imu.accel[2]);

    float mag_g = mag / G;

    // requirements §9: impact = threshold G "× 連続2回" → auto DISARM. Require
    // consecutive_count over-threshold cycles before raising, so a single noisy
    // sample does not trip the failsafe; latch once raised. NOTE: update() runs at
    // the PowerTask rate (10 Hz), so "2 consecutive" is ~200 ms sustained — this
    // debounces noise but a very brief impact spike may be missed; revisit the
    // failsafe rate if sharper impact sensitivity is needed.
    // 要件§9: 衝撃 = 閾値G「× 連続2回」→ 自動DISARM。発報前に連続 consecutive_count 回の
    // 閾値超を要求し、単発ノイズで誤発報しない。発報後は latch。注: update() は PowerTask
    // レート(10Hz)で走るため「連続2回」は約200ms継続 — ノイズは除けるが極短の衝撃スパイクは
    // 取りこぼし得る。鋭敏化が必要なら failsafe レートの見直しを。
    if (mag_g > config_.impact_accel_g) {
        if (++impact_count_ >= config_.consecutive_count && !impact_detected_) {
            impact_detected_ = true;
            raiseAlert(AlertType::IMPACT, AlertSeverity::EMERGENCY);
            ESP_LOGW(TAG, "Impact detected: %.1fG (x%u)", mag_g, impact_count_);
        }
    } else {
        impact_count_ = 0;  // below threshold → reset the consecutive counter
    }
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
        // Low voltage — warning
        // 低電圧 — 警告
        if (!batt_warning_) {
            batt_warning_ = true;
            raiseAlert(AlertType::LOW_BATTERY, AlertSeverity::WARNING);
            ESP_LOGW(TAG, "Low battery: %.2fV", power.voltage);
        }
    } else if (power.voltage > config_.low_battery_v + kRecoveryHysteresisV) {
        batt_warning_   = false;
        batt_emergency_ = false;
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

    // requirements §9: abnormal angular rate = threshold deg/s "× 連続2回" → auto
    // DISARM. An uncontrolled tumble sustains a high rate, so the consecutive-count
    // debounce rejects single-sample spikes while still catching a real anomaly.
    // 要件§9: 異常角速度 = 閾値deg/s「× 連続2回」→ 自動DISARM。制御不能のタンブルは高い
    // レートが継続するため、連続回数デバウンスで単発スパイクを除きつつ実異常を検出する。
    float max_dps = 0.0f;
    int max_axis = 0;
    for (int i = 0; i < 3; ++i) {
        float dps = std::fabs(imu.gyro[i]) * RAD2DEG;
        if (dps > max_dps) { max_dps = dps; max_axis = i; }
    }

    if (max_dps > config_.gyro_anomaly_dps) {
        if (++gyro_count_ >= config_.consecutive_count) {
            raiseAlert(AlertType::GYRO_ANOMALY, AlertSeverity::CRITICAL);
            ESP_LOGW(TAG, "Gyro anomaly: axis=%d, %.0f deg/s (x%u)",
                     max_axis, max_dps, gyro_count_);
            gyro_count_ = 0;  // re-arm so a sustained anomaly re-raises periodically
        }
    } else {
        gyro_count_ = 0;  // below threshold → reset the consecutive counter
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
