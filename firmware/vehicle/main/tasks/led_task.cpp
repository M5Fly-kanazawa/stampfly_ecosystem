/**
 * @file led_task.cpp
 * @brief LEDタスク (30Hz) - LEDManager経由でLED状態更新
 *
 * LEDManagerが優先度ベースで3つのLEDを管理：
 * - MCU LED (GPIO21): システム状態
 * - BODY_TOP (GPIO39-0): 飛行状態
 * - BODY_BOTTOM (GPIO39-1): センサー/バッテリー
 */

#include "tasks_common.hpp"
#include "led_manager.hpp"

static const char* TAG = "LEDTask";

using namespace config;
using namespace globals;

void LEDTask(void* pvParameters)
{
    ESP_LOGI(TAG, "LEDTask started");

    auto& led_mgr = stampfly::LEDManager::getInstance();
    auto& state = stampfly::StampFlyState::getInstance();

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(32);  // ~30Hz

    // Low battery threshold for battery replace warning
    constexpr float LOW_BATTERY_THRESHOLD = 3.4f;

    stampfly::FlightState prev_flight_state = stampfly::FlightState::INIT;
    bool prev_low_battery = false;

    while (true) {
        // バッテリー状態のチェックと通知
        float voltage = state.getVoltage();
        bool low_battery = (voltage > 0.5f) && (voltage < LOW_BATTERY_THRESHOLD);

        if (low_battery != prev_low_battery) {
            led_mgr.onBatteryStateChanged(voltage, low_battery);
            prev_low_battery = low_battery;
        }

        // 飛行状態のチェックと通知
        stampfly::FlightState flight_state = state.getFlightState();
        if (flight_state != prev_flight_state) {
            led_mgr.onFlightStateChanged(flight_state);
            prev_flight_state = flight_state;
        }

        // LEDアニメーション更新（タイムアウト処理含む）
        led_mgr.update();

        vTaskDelayUntil(&last_wake_time, period);
    }
}
