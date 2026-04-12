/**
 * @file button_task.cpp
 * @brief Button input task (50Hz)
 *        ボタン入力タスク（50Hz）
 *
 * Polls button state, detects click/long-press events,
 * and requests ARM/DISARM via StateManager.
 *
 * ボタン状態をポーリングし、クリック/長押しイベントを検出し、
 * StateManager経由でARM/DISARMをリクエストする。
 *
 * @design architecture.md §6 — ButtonTask                            [--]
 * @design detailed_design.md §8 — ButtonTask: 50Hz, priority 10      [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.hpp"

static const char* TAG = "ButtonTask";

void ButtonTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ButtonTask started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz

    while (true) {
        // TODO: Read button GPIO
        // TODO: Debounce processing
        // TODO: Detect click/double-click/long-press
        // TODO: On click → request ARM/DISARM via StateManager

        vTaskDelayUntil(&last_wake, period);
    }
}
