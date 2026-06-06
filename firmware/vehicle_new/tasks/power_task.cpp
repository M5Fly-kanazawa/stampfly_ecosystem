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
 * Reads INA3221 power data, publishes to sensor.power topic,
 * and performs failsafe checks (low battery, USB power).
 *
 * INA3221電源データを読み取り、sensor.powerトピックに発行し、
 * フェイルセーフチェック（低電圧、USB給電）を実行する。
 *
 * @design architecture.md §6 — PowerTask: Sensing(Power) + Failsafe  [--]
 * @design detailed_design.md §8 — PowerTask: 10Hz, priority 12       [OK]
 * @design requirements.md §9 — LiPo ≤3.4V warning, USB ≤3.3V ARM ban [--]
 * @design hardware_init.md §5 — Power monitor = Optional             [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "topics.hpp"
#include "flight_state.hpp"
#include "config.hpp"
#include "sf_board.hpp"
#include "power_monitor.hpp"

static const char* TAG = "PowerTask";

/// Notify state task of an alert
/// 状態タスクにアラートを通知する
extern TaskHandle_t g_state_task_handle;

/// INA3221 power monitor — task-local file-scope static (no extern global, R2).
/// INA3221 電源モニタ — タスクローカルな file-scope static（extern グローバル禁止, R2）。
static stampfly::PowerMonitor g_power;

static void publishAlert(sf::AlertType type, sf::AlertSeverity severity)
{
    sf::SystemAlert alert = {};
    alert.type = static_cast<uint8_t>(type);
    alert.severity = static_cast<uint8_t>(severity);
    alert.timestamp = static_cast<uint32_t>(esp_timer_get_time());

    sf::system_alert.publish(alert);

    // Notify state task to process alert
    // 状態タスクにアラート処理を通知
    if (g_state_task_handle != nullptr) {
        xTaskNotifyGive(g_state_task_handle);
    }
}

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
    if (present) {
        ESP_LOGI(TAG, "INA3221 ready (10Hz)");
    } else {
        ESP_LOGW(TAG, "INA3221 init failed — power monitoring disabled (Optional)");
    }

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  // 10Hz
    bool logged_first = false;  // one-shot boot battery log / 起動時電池電圧の単発ログ

    while (true) {
        // Read battery voltage / current / power from the INA3221 bus channel.
        // INA3221 のバスチャンネルから電圧・電流・電力を読む。
        sf::PowerData data = {};
        data.timestamp = static_cast<uint32_t>(esp_timer_get_time());

        bool valid = false;
        if (present) {
            stampfly::PowerData reading{};
            if (g_power.read(reading) == ESP_OK) {
                data.voltage = reading.voltage_v;
                data.current = reading.current_ma;
                data.power   = reading.power_mw;
                valid = true;

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

        // Failsafe checks — only on a valid reading, so a missing or failed
        // sensor (voltage == 0) never raises a false low-voltage alert.
        // (Alert thresholds will move into the Failsafe component in P2-2.)
        // フェイルセーフチェック — 有効な読み取り時のみ実行。センサ欠落/失敗
        // (voltage==0) で誤った低電圧アラートを出さない。（閾値は P2-2 で
        // Failsafe コンポーネントへ移管予定。）
        if (valid) {
            if (data.voltage <= 3.3f) {
                publishAlert(sf::AlertType::USB_POWER, sf::AlertSeverity::WARNING);
            } else if (data.voltage <= 3.4f) {
                publishAlert(sf::AlertType::LOW_BATTERY, sf::AlertSeverity::WARNING);
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
