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
 * @design detailed_design.md §8 — PowerTask: 10Hz, priority 12       [--]
 * @design requirements.md §9 — LiPo ≤3.4V warning, USB ≤3.3V ARM ban [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "topics.hpp"
#include "flight_state.hpp"
#include "config.hpp"

static const char* TAG = "PowerTask";

/// Notify state task of an alert
/// 状態タスクにアラートを通知する
extern TaskHandle_t g_state_task_handle;

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

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  // 10Hz

    while (true) {
        // Read from INA3221 power monitor
        // INA3221電源モニタから読み取る
        sf::PowerData data = {};
        data.timestamp = static_cast<uint32_t>(esp_timer_get_time());
        // TODO: Wire power_monitor driver
        // TODO: power_monitorドライバを結合
        data.voltage = 4.2f;  // Stub until driver is wired / ドライバ結合まで仮値

        sf::sensor_power.publish(data);

        // Failsafe checks
        // フェイルセーフチェック
        if (data.voltage <= 3.3f) {
            publishAlert(sf::AlertType::USB_POWER, sf::AlertSeverity::WARNING);
        } else if (data.voltage <= 3.4f) {
            publishAlert(sf::AlertType::LOW_BATTERY, sf::AlertSeverity::WARNING);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
