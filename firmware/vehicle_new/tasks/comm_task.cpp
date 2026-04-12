/**
 * @file comm_task.cpp
 * @brief Communication + command processing task (50Hz)
 *        通信 + コマンド処理タスク（50Hz）
 *
 * Handles ESP-NOW/UDP reception, normalizes input,
 * and publishes command setpoints.
 *
 * ESP-NOW/UDP受信を処理し、入力を正規化し、
 * コマンドセットポイントを発行する。
 *
 * @design architecture.md §6 — CommTask: Communication + Command      [--]
 * @design detailed_design.md §8 — CommTask: 50Hz, priority 15        [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "topics.hpp"
#include "config.hpp"

static const char* TAG = "CommTask";

void CommTask(void* pvParameters)
{
    ESP_LOGI(TAG, "CommTask started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    while (true) {
        // TODO: Process ESP-NOW received data
        // TODO: Process UDP received data
        // TODO: Normalize and arbitrate inputs
        // TODO: Publish command.setpoint
        // TODO: Check communication timeout → publish system.alert

        vTaskDelayUntil(&last_wake, period);
    }
}
