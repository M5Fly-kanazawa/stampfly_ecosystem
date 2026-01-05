/**
 * @file power_task.cpp
 * @brief 電源監視タスク (10Hz) - INA3221読み取り、バッテリー状態監視
 */

#include "tasks_common.hpp"

static const char* TAG = "PowerTask";

using namespace config;
using namespace globals;

void PowerTask(void* pvParameters)
{
    ESP_LOGI(TAG, "PowerTask started");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);  // 10Hz

    auto& state = stampfly::StampFlyState::getInstance();
    static uint32_t log_counter = 0;
    static bool first_read = true;
    static bool low_battery_active = false;
    static uint32_t warning_interval_counter = 0;
    constexpr uint32_t WARNING_INTERVAL = 100;  // 10秒間隔 (10Hz × 100)

    while (true) {
        if (g_power.isInitialized()) {
            stampfly::PowerData power;
            if (g_power.read(power) == ESP_OK) {
                state.updatePower(power.voltage_v, power.current_ma / 1000.0f);

                // Log first reading immediately, then every 5 seconds
                if (first_read || ++log_counter >= 50) {
                    ESP_LOGI(TAG, "Battery: %.2fV, %.1fmA, LowBat=%d",
                             power.voltage_v, power.current_ma, g_power.isLowBattery());
                    log_counter = 0;
                    first_read = false;
                }

                // Low battery warning (警告のみ、エラーにはしない)
                if (g_power.isLowBattery()) {
                    if (!low_battery_active) {
                        // 最初の検出時
                        ESP_LOGW(TAG, "LOW BATTERY WARNING: %.2fV - Land soon!", power.voltage_v);
                        stampfly::LEDManager::getInstance().onBatteryStateChanged(
                            power.voltage_v, true);
                        g_buzzer.lowBatteryWarning();
                        low_battery_active = true;
                        warning_interval_counter = 0;
                    } else {
                        // 継続中: 定期的にブザーで警告
                        if (++warning_interval_counter >= WARNING_INTERVAL) {
                            ESP_LOGW(TAG, "LOW BATTERY: %.2fV - Land immediately!", power.voltage_v);
                            g_buzzer.lowBatteryWarning();
                            warning_interval_counter = 0;
                        }
                    }
                } else {
                    // バッテリー回復時（充電された場合など）
                    if (low_battery_active) {
                        ESP_LOGI(TAG, "Battery recovered: %.2fV", power.voltage_v);
                        stampfly::LEDManager::getInstance().onBatteryStateChanged(
                            power.voltage_v, false);
                        low_battery_active = false;
                    }
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}
