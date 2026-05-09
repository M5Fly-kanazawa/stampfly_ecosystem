/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file telemetry_task.cpp
 * @brief Telemetry task (50 Hz)
 *        テレメトリタスク（50Hz）
 *
 * Owns a single sf::Telemetry instance, runs init() once (which blocks
 * until WiFi STA has an IP), then calls update() at 50 Hz forever.
 *
 * sf::Telemetry インスタンスを1つ所有し、init()（WiFi STA の IP 取得まで
 * ブロック）を1回実行した後、50Hz で永続的に update() を呼ぶ。
 *
 * @design architecture.md §6 — TelemetryTask                           [OK]
 * @design detailed_design.md §8 — TelemetryTask: 50Hz, priority 13     [OK]
 * @design requirements.md §7 — Telemetry: UDP only                     [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "telemetry.hpp"
#include "topics.hpp"
#include "config.hpp"

static const char* TAG = "TelemetryTask";

void TelemetryTask(void* pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "TelemetryTask started");

    // 50 Hz period — matches Phase 2a packet rate.
    // 50Hz 周期 — Phase 2a パケットレートに一致。
    const TickType_t period = pdMS_TO_TICKS(20);

    // Construct on the task stack (modest size; safe with STACK_TELEMETRY=4KB).
    // タスクスタックに構築（小サイズなので STACK_TELEMETRY=4KB で安全）。
    sf::Telemetry telemetry;
    telemetry.init();   // blocks until WiFi ready / WiFi 準備完了までブロック

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        // Pull latest topic data, build packet, sendto() — all inside update().
        // 最新トピックを取得→パケット構築→sendto() を update() 内で実行。
        telemetry.update();
        vTaskDelayUntil(&last_wake, period);
    }
}
