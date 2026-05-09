/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file log_task.cpp
 * @brief Data logger + Blackbox task (async)
 *        データロガー + Blackboxタスク（非同期）
 *
 * Subscribes to all topics via ring buffers, writes data to
 * Telemetry (UDP), Data Stream (UDP/USB), and Blackbox (SPIFFS).
 *
 * リングバッファ経由で全トピックを購読し、
 * Telemetry（UDP）、Data Stream（UDP/USB）、Blackbox（SPIFFS）に書き込む。
 *
 * @design architecture.md §5 — Log flow: all topics → logger          [--]
 * @design architecture.md §6 — LogTask: Data Logger + Blackbox        [--]
 * @design detailed_design.md §8 — LogTask: async, priority 5         [--]
 * @design requirements.md §7 — Telemetry/Data Stream/Blackbox        [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "topics.hpp"
#include "config.hpp"

static const char* TAG = "LogTask";

void LogTask(void* pvParameters)
{
    ESP_LOGI(TAG, "LogTask started");

    while (true) {
        // TODO: Read all topic ring buffers
        // TODO: Write to Blackbox (SPIFFS) — ring buffer, overwrite oldest
        // TODO: Send Data Stream via UDP/USB if enabled
        // TODO: Manage Blackbox file lifecycle

        // Wait for data availability notification or periodic check
        // データ有効通知を待つか定期チェック
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz max check rate
    }
}
