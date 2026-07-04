/**
 * @file button_task.cpp
 * @brief ボタンタスク (100Hz) - ボタン状態監視
 */

#include "tasks_common.hpp"

static const char* TAG = "ButtonTask";

using namespace config;
using namespace globals;

void ButtonTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ButtonTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz

    while (true) {
        if (g_button.isInitialized()) {
            g_button.tick();
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}
