/**
 * @file telemetry_task.cpp
 * @brief Telemetry task (50Hz)
 *        テレメトリタスク（50Hz）
 *
 * Reads latest state estimate and mode from topics,
 * sends telemetry via UDP.
 *
 * トピックから最新の状態推定値とモードを読み取り、
 * UDP経由でテレメトリを送信する。
 *
 * @design architecture.md §6 — TelemetryTask                         [--]
 * @design detailed_design.md §8 — TelemetryTask: 50Hz, priority 13   [--]
 * @design requirements.md §7 — Telemetry: UDP only                    [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "topics.hpp"
#include "config.hpp"

static const char* TAG = "TelemetryTask";

void TelemetryTask(void* pvParameters)
{
    ESP_LOGI(TAG, "TelemetryTask started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    while (true) {
        // Read latest values from topics (non-blocking)
        // トピックから最新値を読む（ノンブロッキング）
        sf::StateEstimate state = sf::estimate_state.latest();
        sf::SystemMode mode = sf::system_mode.latest();

        // TODO: Format and send UDP telemetry packet
        // TODO: UDPテレメトリパケットをフォーマットして送信

        vTaskDelayUntil(&last_wake, period);
    }
}
