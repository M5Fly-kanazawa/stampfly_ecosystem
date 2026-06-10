/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file power_task.cpp
 * @brief Power monitor + failsafe task (10Hz)
 *        電源モニタ + フェイルセーフタスク（10Hz）
 *
 * Reads INA3221 power data, publishes to sensor.power topic, and runs the
 * Failsafe monitor (battery / comm-loss / impact / gyro anomaly). The Failsafe
 * only DETECTS and publishes system_alert; the StateManager (sole transition
 * authority) decides the response — architecture.md §4 (failsafe as event).
 *
 * INA3221電源データを読み取り、sensor.powerトピックに発行し、Failsafe モニタ
 * （電池/通信断/衝撃/ジャイロ異常）を実行する。Failsafe は検出して system_alert
 * に発報するだけで、応答は StateManager（唯一の遷移実行者）が決める
 * — architecture.md §4（フェイルセーフはイベント）。
 *
 * @design architecture.md §6 — PowerTask: Sensing(Power) + Failsafe  [OK]
 * @design detailed_design.md §8 — PowerTask: 10Hz, priority 12       [OK]
 * @design requirements.md §9 — LiPo ≤3.4V warning, USB ≤3.3V ARM ban [OK]
 * @design hardware_init.md §5 — Power monitor = Optional             [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "topics.hpp"
#include "config.hpp"
#include "sf_board.hpp"
#include "power_monitor.hpp"
#include "failsafe.hpp"

static const char* TAG = "PowerTask";

/// INA3221 power monitor — task-local file-scope static (no extern global, R2).
/// INA3221 電源モニタ — タスクローカルな file-scope static（extern グローバル禁止, R2）。
static stampfly::PowerMonitor g_power;

/// Failsafe monitor — task-local file-scope static. Subscribes to topics and
/// publishes system_alert; owns no hardware.
/// Failsafe モニタ — タスクローカルな file-scope static。トピックを購読し
/// system_alert を発行する。ハードウェアは持たない。
static sf::Failsafe g_failsafe;

namespace {

/// Build the SensorHealth snapshot (R15): presence comes from the BSP (sensor_present,
/// its own data), freshness from each sensor topic's latest timestamp. We aggregate and
/// publish HERE rather than inside sf_board so the BSP layer does not read application-
/// level data topics (a layering inversion) — detailed_design §2 lists the publisher as
/// "sf_board / 各 task", so the task path is design-sanctioned. The IMU is Critical: if it
/// had failed, init() would have halted, so reaching this code means it is present.
/// SensorHealth スナップショットを構築する（R15）: presence は BSP（sensor_present、BSP 自身の
/// データ）、鮮度は各センサトピックの直近タイムスタンプから得る。集約と publish を sf_board 内
/// でなくここで行うのは、BSP 層がアプリ層のデータトピックを読む層逆転を避けるため
/// — detailed_design §2 は publisher を「sf_board / 各 task」と記載しており、タスク経由は
/// 設計上許容される。IMU は Critical: 失敗していれば init() で停止しているため、ここに来た
/// 時点で存在する。
///
/// @publisher sensor_health
/// @design architecture.md §3 — R15 sensor_health 1 Hz                 [OK]
/// @design detailed_design.md §2 — sensor_health publisher: sf_board / 各 task [OK]
sf::SensorHealth buildSensorHealth(uint32_t now_us)
{
    namespace board = sf::internal::board;
    using sf::SensorId;
    using BoardId = board::SensorId;

    // (published bit, last-publish [us], present). The IMU is Critical and always
    // present once running; its timestamp comes from its RingBuffer topic, whose
    // latest() does NOT consume (safe to read here). The 5 Optional sensors use the
    // BSP last-update registry (set_sensor_update) because their Queue-policy topics
    // are consumed by the fusion task — a second reader would steal data (R5).
    // （発行ビット, 最終 publish 時刻 [us], present）。IMU は Critical で起動中は常在。
    // タイムスタンプは RingBuffer トピックから取得し、その latest() は消費しない
    // (ここで読んで安全)。Optional な5センサは BSP の last-update レジストリ
    // (set_sensor_update) を使う — Queue 方式トピックは融合タスクが消費するため、
    // 2番目の読者がデータを奪う (R5)。
    const struct { SensorId id; uint32_t ts; bool present; } entries[] = {
        {SensorId::Imu,   sf::sensor_imu.latest().timestamp,        true},
        {SensorId::Mag,   board::sensor_last_update_us(BoardId::Mag),      board::sensor_present(BoardId::Mag)},
        {SensorId::Baro,  board::sensor_last_update_us(BoardId::Baro),     board::sensor_present(BoardId::Baro)},
        {SensorId::Tof,   board::sensor_last_update_us(BoardId::BottomToF), board::sensor_present(BoardId::BottomToF)},
        {SensorId::Flow,  board::sensor_last_update_us(BoardId::Flow),     board::sensor_present(BoardId::Flow)},
        {SensorId::Power, board::sensor_last_update_us(BoardId::Power),    board::sensor_present(BoardId::Power)},
    };

    sf::SensorHealth health{};
    health.timestamp = now_us;
    for (const auto& e : entries) {
        const int bit = static_cast<int>(e.id);
        health.last_update_us[bit] = e.ts;
        if (!e.present) {
            continue;
        }
        health.present_mask |= static_cast<uint8_t>(1u << bit);
        // Healthy = present AND a sample arrived within the staleness window. ts==0 means
        // "never published" → not fresh. The uint32 subtraction handles a single wrap.
        // 健全 = 存在 かつ staleness 窓内にサンプル到着。ts==0 は未発行→非鮮度。uint32 の
        // 引き算は1回の桁あふれを正しく扱う。
        const bool fresh = (e.ts != 0) && ((now_us - e.ts) < config::SENSOR_HEALTH_STALE_US);
        if (fresh) {
            health.healthy_mask |= static_cast<uint8_t>(1u << bit);
        }
    }
    return health;
}

}  // namespace

void PowerTask(void* pvParameters)
{
    ESP_LOGI(TAG, "PowerTask started");

    // -------------------------------------------------------------------
    // Setup: borrow the shared I2C bus from the BSP and init the INA3221.
    // Power monitoring is Optional (hardware_init.md §5): on init failure we
    // keep the task alive but stop publishing voltage (the 0 sentinel means
    // "unknown"), so flight stays possible — only the battery warning is lost.
    //
    // セットアップ: BSP から共有 I2C バスを借用し INA3221 を初期化する。
    // 電源モニタは Optional（hardware_init.md §5）。init 失敗時はタスクを生かした
    // まま電圧の発行を止める（0 のセンチネルは「不明」を意味する）。電圧監視を
    // 失うだけで飛行は可能。
    // -------------------------------------------------------------------
    stampfly::PowerMonitor::Config cfg{};
    cfg.i2c_bus = sf::internal::board::i2c_bus();
    const bool present = (g_power.init(cfg) == ESP_OK);
    sf::internal::board::set_sensor_present(
        sf::internal::board::SensorId::Power, present);
    if (present) {
        ESP_LOGI(TAG, "INA3221 ready (10Hz)");
    } else {
        ESP_LOGW(TAG, "INA3221 init failed — power monitoring disabled (Optional)");
    }

    // Failsafe needs no hardware — it reads topics and publishes alerts. The
    // thresholds come from the safety.* parameters (params SSOT) — the old bare
    // init() silently used the struct defaults.
    // Failsafe はハードウェア不要 — トピックを読みアラートを発行する。閾値は
    // safety.* パラメータ（params SSOT）から取得 — 旧来の素の init() は struct
    // 既定値を暗黙使用していた。
    g_failsafe.init(sf::loadFailsafeConfigFromParams());

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  // 10Hz
    bool logged_first = false;  // one-shot boot battery log / 起動時電池電圧の単発ログ
    bool logged_health = false; // one-shot boot sensor_health log / health 単発ログ
    int health_tick = 0;        // 10Hz → 1Hz divider for sensor_health / health 用分周

    while (true) {
        // Read battery voltage / current / power from the INA3221 bus channel.
        // INA3221 のバスチャンネルから電圧・電流・電力を読む。
        sf::PowerData data = {};
        data.timestamp = static_cast<uint32_t>(esp_timer_get_time());

        if (present) {
            stampfly::PowerData reading{};
            if (g_power.read(reading) == ESP_OK) {
                data.voltage = reading.voltage_v;
                data.current = reading.current_ma;
                data.power   = reading.power_mw;

                // One-shot boot battery log — useful for hardware bring-up
                // (don't fly on a low pack) and confirms the read path in SIL.
                // 起動時電池電圧の単発ログ — 実機ブリングアップに有用（低電圧では
                // 飛ばさない）。SIL では読み取り経路の確認も兼ねる。
                if (!logged_first) {
                    logged_first = true;
                    ESP_LOGI(TAG, "Battery: %.2fV (boot reading)", data.voltage);
                }
            }
            // On read failure voltage stays 0 → consumers treat it as "unknown".
            // 読み取り失敗時は voltage=0 のまま → 購読側は「不明」として扱う。
        }

        sf::sensor_power.publish(data);
        // Report freshness to the BSP for the 1 Hz sensor_health snapshot (R15).
        // 1Hz の sensor_health 用に鮮度を BSP へ報告する (R15)。
        sf::internal::board::set_sensor_update(
            sf::internal::board::SensorId::Power, data.timestamp);

        // Run all failsafe checks AFTER publishing power, so checkBattery() sees
        // this cycle's reading. Battery thresholds and the 0 = unknown guard now
        // live in the Failsafe component (failsafe.cpp checkBattery).
        // 電源 publish の後に全 failsafe チェックを走らせる（checkBattery が今周期の
        // 読み値を見るため）。電池閾値と 0=不明ガードは Failsafe コンポーネント
        // （failsafe.cpp checkBattery）に集約済み。
        g_failsafe.update();

        // Publish the system sensor-health snapshot at 1 Hz (R15), downsampled from this
        // task's 10 Hz tick: presence (BSP) + per-sensor freshness (topics), for Failsafe
        // and Telemetry liveness monitoring. The first tick publishes immediately so the
        // topic holds a valid snapshot early instead of a zero-initialized default.
        // システムのセンサ健全性スナップショットを 1Hz で publish（R15）。本タスクの 10Hz
        // から分周: presence（BSP）＋センサ毎の鮮度（トピック）を Failsafe/Telemetry の
        // 死活監視用に。初回ティックで即 publish し、トピックがゼロ初期化の既定値でなく
        // 早期に有効スナップショットを保持するようにする。
        if (health_tick == 0) {
            const sf::SensorHealth health = buildSensorHealth(data.timestamp);
            sf::sensor_health.publish(health);
            // One-shot bring-up log: the first snapshot where every present sensor is
            // fresh (healthy_mask == present_mask, non-empty). This is the "all sensors
            // up" indicator — early boot snapshots are sparse because the sensor tasks
            // are still initialising. bit order = SensorId: Imu,Mag,Baro,Tof,Flow,Power.
            // ブリングアップ単発ログ: present な全センサが鮮度OKになった最初のスナップショット
            // （healthy_mask == present_mask かつ非ゼロ）。「全センサ起動」の指標 — 起動直後の
            // スナップショットはセンサタスク初期化中で疎。ビット順 = SensorId。
            if (!logged_health && health.present_mask != 0 &&
                health.healthy_mask == health.present_mask) {
                logged_health = true;
                ESP_LOGI(TAG, "sensor_health: all present sensors fresh "
                              "(present=0x%02X)", health.present_mask);
            }
        }
        health_tick = (health_tick + 1) % 10;

        vTaskDelayUntil(&last_wake, period);
    }
}
