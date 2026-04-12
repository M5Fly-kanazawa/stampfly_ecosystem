/**
 * @file notify_task.cpp
 * @brief Notification task — LED/buzzer (30Hz)
 *        通知タスク — LED/ブザー（30Hz）
 *
 * Reads system.mode and system.alert topics, drives LED patterns
 * and buzzer tones to indicate vehicle state.
 * Directly controls HAL — no Pub-Sub output.
 *
 * system.modeとsystem.alertトピックを読み取り、
 * 機体状態を示すLEDパターンとブザー音を制御する。
 * HALを直接操作 — Pub-Sub出力なし。
 *
 * @design architecture.md §2 — Notification: direct HAL access        [--]
 * @design detailed_design.md §8 — NotifyTask: 30Hz, priority 8       [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "topics.hpp"
#include "flight_state.hpp"
#include "config.hpp"

static const char* TAG = "NotifyTask";

void NotifyTask(void* pvParameters)
{
    ESP_LOGI(TAG, "NotifyTask started");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(33);  // ~30Hz

    while (true) {
        sf::SystemMode mode = sf::system_mode.latest();

        // TODO: Set LED pattern based on flight state
        // TODO: フライト状態に基づいてLEDパターンを設定
        //
        // INIT:         white solid
        // IDLE_GROUND:  green breathe
        // IDLE_HELD:    blue breathe
        // ARMED_GROUND: yellow blink
        // FLYING:       green solid
        // LANDING:      orange blink
        //
        // LOW_BATTERY:  red blink (overrides)

        // TODO: Process buzzer alerts
        // TODO: ブザーアラートを処理

        vTaskDelayUntil(&last_wake, period);
    }
}
