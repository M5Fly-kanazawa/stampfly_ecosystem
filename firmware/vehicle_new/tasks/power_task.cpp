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
 * @design requirements.md §9 — LiPo ≤3.4V warning, USB ≤3.3V ARM ban [--]
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

    // Failsafe needs no hardware — it reads topics and publishes alerts.
    // Failsafe はハードウェア不要 — トピックを読みアラートを発行する。
    g_failsafe.init();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  // 10Hz
    bool logged_first = false;  // one-shot boot battery log / 起動時電池電圧の単発ログ

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

        // Run all failsafe checks AFTER publishing power, so checkBattery() sees
        // this cycle's reading. Battery thresholds and the 0 = unknown guard now
        // live in the Failsafe component (failsafe.cpp checkBattery).
        // 電源 publish の後に全 failsafe チェックを走らせる（checkBattery が今周期の
        // 読み値を見るため）。電池閾値と 0=不明ガードは Failsafe コンポーネント
        // （failsafe.cpp checkBattery）に集約済み。
        g_failsafe.update();

        vTaskDelayUntil(&last_wake, period);
    }
}
