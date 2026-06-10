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
 * @subscriber estimate_state, control_output, actuator_motor, sensor_imu, system_mode (UDP egress)
 * @design architecture.md §6 — TelemetryTask                           [OK]
 * @design detailed_design.md §8 — TelemetryTask: 50Hz, priority 13     [OK]
 * @design requirements.md §7 — Telemetry: UDP only                     [OK]
 * @design architecture.md §3 — R13 @publisher/@subscriber annotation   [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "telemetry.hpp"
#include "data_stream.hpp"
#include "topics.hpp"
#include "config.hpp"

static const char* TAG = "TelemetryTask";

/// Data Stream server — control-rate (400Hz) analysis log over UDP, started
/// and stopped by the PC (`sf log wifi`). File-scope static (task-owned).
/// Data Stream サーバ — 制御周期（400Hz）の解析用 UDP ログ。PC 側
/// （`sf log wifi`）が開始/停止する。ファイルスコープ static（タスク所有）。
static sf::DataStream g_data_stream;

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

    // The Data Stream shares the WiFi the Telemetry waited for.
    // Data Stream は Telemetry が待った WiFi を共用する。
    g_data_stream.init();

    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        // Data Stream first (commands + ring drain), then the monitoring packet.
        // Exclusive-log policy: while a capture is active the 50Hz monitoring
        // packet is suppressed so the analysis stream owns the WiFi bandwidth
        // (loss-prevention design proven on firmware/vehicle).
        // 先に Data Stream（コマンド＋リング排出）、次にモニタ用パケット。
        // 排他ログ方針: キャプチャ中は 50Hz モニタ用パケットを抑止し、解析
        // ストリームに WiFi 帯域を専有させる（firmware/vehicle 実証済みの
        // 欠損防止設計）。
        g_data_stream.update();
        if (!g_data_stream.isActive()) {
            telemetry.update();
        }
        vTaskDelayUntil(&last_wake, period);
    }
}
