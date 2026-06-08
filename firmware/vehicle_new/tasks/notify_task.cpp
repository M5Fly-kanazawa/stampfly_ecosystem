/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

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
 * @subscriber system_mode, notify_command (HAL egress: LED/buzzer — no topic publish)
 * @design architecture.md §2 — Notification: direct HAL access        [OK]
 * @design detailed_design.md §8 — NotifyTask: 30Hz, priority 8       [OK]
 * @design architecture.md §3 — R13 @publisher/@subscriber annotation  [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "notify.hpp"
#include "config.hpp"

static const char* TAG = "NotifyTask";

void NotifyTask(void* pvParameters)
{
    ESP_LOGI(TAG, "NotifyTask started");

    // sf::Notify owns the LED + buzzer HALs and maps flight state → LED pattern
    // and notify_command events → buzzer tones. The HW config is supplied here
    // (this task sees config.hpp; the sf_notify component does not depend on it).
    // sf::Notify が LED+ブザー HAL を所有し、フライト状態→LEDパターン、
    // notify_command イベント→ブザー音に対応づける。HW 構成はここで渡す
    // (本タスクは config.hpp を見るが、sf_notify コンポーネントは依存しない)。
    sf::NotifyConfig cfg{};
    cfg.led_gpio            = config::GPIO_LED_BODY;
    cfg.led_count           = config::LED_NUM_BODY;
    cfg.buzzer_gpio         = config::GPIO_BUZZER;
    cfg.buzzer_ledc_channel = config::BUZZER_LEDC_CHANNEL;
    cfg.buzzer_ledc_timer   = config::BUZZER_LEDC_TIMER;

    sf::Notify notify;
    notify.init(cfg);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(33);  // ~30Hz

    while (true) {
        notify.update();
        vTaskDelayUntil(&last_wake, period);
    }
}
