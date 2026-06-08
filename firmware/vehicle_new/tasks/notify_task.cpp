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
 * @subscriber system_mode, pairing_state, notify_command (HAL egress: LED/buzzer — no topic publish)
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

// TEMPORARY DIAGNOSTIC (real-hardware INIT-stuck investigation). Incremented each
// NotifyTask update so ImuTask can report whether NotifyTask (core 0) is being
// starved (LED frozen) while the state machine (core 1) may have progressed.
// 一時診断: NotifyTask が更新の度に増やす。NotifyTask(core0)が飢餓して LED が固まって
// いるか（状態は core1 で進んでいるか）を ImuTask が報告できるように。
extern "C" volatile uint32_t g_diag_notify_updates = 0;

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
        g_diag_notify_updates = g_diag_notify_updates + 1;  // DIAG: NotifyTask running
        vTaskDelayUntil(&last_wake, period);
    }
}
