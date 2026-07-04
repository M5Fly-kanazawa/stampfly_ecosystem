/**
 * @file led_task.cpp
 * @brief LEDタスク (30Hz) - LEDManager経由でLED状態更新
 *
 * LED役割分担:
 *   SYSTEM (StampS3 GPIO21): モード + キャリブレーション（地上確認用）
 *   BODY (上面+下面 GPIO39):  フライト状態 + 警告（飛行中に見える情報）
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

    constexpr float LOW_BATTERY_THRESHOLD = 3.4f;

    stampfly::FlightState prev_flight_state = stampfly::FlightState::INIT;
    stampfly::FlightMode prev_flight_mode = stampfly::FlightMode::ACRO;
    bool prev_low_battery = false;

    while (true) {
        // Battery state → BODY channel
        // バッテリー状態 → BODYチャンネル
        float voltage = state.getVoltage();
        bool low_battery = (voltage > 0.5f) && (voltage < LOW_BATTERY_THRESHOLD);
        if (low_battery != prev_low_battery) {
            led_mgr.onBatteryStateChanged(voltage, low_battery);
            prev_low_battery = low_battery;
        }

        // Flight state → BODY channel
        // フライト状態 → BODYチャンネル
        stampfly::FlightState flight_state = state.getFlightState();
        if (flight_state != prev_flight_state) {
            led_mgr.onFlightStateChanged(flight_state);
            prev_flight_state = flight_state;
        }

        // Flight mode → SYSTEM channel
        // フライトモード → SYSTEMチャンネル
        stampfly::FlightMode flight_mode = state.getFlightMode();
        if (flight_mode != prev_flight_mode) {
            led_mgr.onFlightModeChanged(flight_mode);
            prev_flight_mode = flight_mode;
        }

        // Update LED animations and timeouts
        // LEDアニメーション更新（タイムアウト処理含む）
        led_mgr.update();

        vTaskDelayUntil(&last_wake_time, period);
    }
}
