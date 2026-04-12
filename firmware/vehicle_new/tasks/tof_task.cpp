/**
 * @file tof_task.cpp
 * @brief ToF sensor + takeoff/landing manager task (30Hz)
 *        ToFセンサ + 離着陸マネージャータスク（30Hz）
 *
 * Reads VL53L3CX ToF data, publishes to sensor.tof topic,
 * and runs takeoff/landing detection logic.
 *
 * VL53L3CX ToFデータを読み取り、sensor.tofトピックに発行し、
 * 離着陸検出ロジックを実行する。
 *
 * @design architecture.md §6 — TofTask: Sensing(ToF) + TL Manager    [--]
 * @design detailed_design.md §8 — TofTask: 30Hz, priority 14         [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "topics.hpp"
#include "config.hpp"

static const char* TAG = "TofTask";

void TofTask(void* pvParameters)
{
    ESP_LOGI(TAG, "TofTask started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(33);  // ~30Hz

    while (true) {
        // TODO: Read from VL53L3CX hardware (bottom + front)
        sf::TofData data = {};
        data.timestamp = static_cast<uint32_t>(esp_timer_get_time());

        sf::sensor_tof.publish(data);

        // TODO: Takeoff/landing detection logic
        // TODO: 離着陸検出ロジック

        vTaskDelayUntil(&last_wake, period);
    }
}
