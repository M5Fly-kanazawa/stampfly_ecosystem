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
// TODO: #include "vl53l3cx_wrapper.hpp"
#include "config.hpp"

static const char* TAG = "TofTask";

// TODO: ToF driver instances — wire when API confirmed
// TODO: ToFドライバインスタンス — API確認後に結合

void TofTask(void* pvParameters)
{
    ESP_LOGI(TAG, "TofTask started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(33);  // ~30Hz

    while (true) {
        uint32_t now = static_cast<uint32_t>(esp_timer_get_time());

        // Read bottom ToF for altitude
        // 底面ToFを高度用に読み取る
        sf::TofData data = {};
        data.timestamp = now;
        // TODO: Wire VL53L3CX API
        // TODO: VL53L3CX APIを結合

        sf::sensor_tof.publish(data);

        // TODO: Takeoff/landing detection using TakeoffLandingMgr
        // TODO: TakeoffLandingMgrを使った離着陸検出

        vTaskDelayUntil(&last_wake, period);
    }
}
