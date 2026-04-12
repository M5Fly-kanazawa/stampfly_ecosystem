/**
 * @file flow_task.cpp
 * @brief Optical flow sensor task (100Hz)
 *        オプティカルフローセンサタスク（100Hz）
 *
 * Reads PMW3901 optical flow data and publishes to sensor.flow topic.
 * PMW3901オプティカルフローデータを読み取り、sensor.flowトピックに発行する。
 *
 * @design architecture.md §6 — FlowTask: Sensing(OptFlow)            [--]
 * @design detailed_design.md §8 — FlowTask: 100Hz, priority 20       [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "topics.hpp"
// TODO: #include "pmw3901_wrapper.hpp"
#include "config.hpp"

static const char* TAG = "FlowTask";

// TODO: PMW3901 driver instance

void FlowTask(void* pvParameters)
{
    ESP_LOGI(TAG, "FlowTask started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);  // 100Hz

    while (true) {
        sf::FlowData data = {};
        data.timestamp = static_cast<uint32_t>(esp_timer_get_time());
        // TODO: Wire PMW3901 API
        // TODO: PMW3901 APIを結合

        sf::sensor_flow.publish(data);
        vTaskDelayUntil(&last_wake, period);
    }
}
