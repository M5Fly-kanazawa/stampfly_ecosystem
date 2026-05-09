/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file state_task.cpp
 * @brief State management task — event-driven (priority 22)
 *        状態管理タスク — イベント駆動（優先度22）
 *
 * Waits for events (alerts, commands) and delegates to StateManager.
 * This is the sole task that modifies flight state.
 *
 * イベント（アラート、コマンド）を待ち、StateManagerに委譲する。
 * フライト状態を変更する唯一のタスク。
 *
 * @design architecture.md §6 — StateTask: event-driven, priority 22   [--]
 * @design architecture.md §2 — State Management: sole transition owner [--]
 * @design detailed_design.md §8 — StateTask                          [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "topics.hpp"
#include "state_manager.hpp"

static const char* TAG = "StateTask";

/// Global state manager instance
/// グローバル状態管理インスタンス
sf::StateManager g_state_manager;

void StateTask(void* pvParameters)
{
    ESP_LOGI(TAG, "StateTask started");

    // Transition from INIT → IDLE_GROUND
    // INIT → IDLE_GROUNDへ遷移
    //
    // @design requirements.md §2 — INIT → IDLE_GROUND on init complete [--]
    g_state_manager.init();

    while (true) {
        // =====================================================================
        // Wait for event notification (pure event-driven)
        // イベント通知を待つ（純粋イベント駆動）
        //
        // @design detailed_design.md §8 — StateTask: event-driven     [--]
        // =====================================================================

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // =====================================================================
        // Process system alerts from failsafe
        // フェイルセーフからのシステムアラートを処理
        //
        // @design architecture.md §4 — FAILSAFE as event              [--]
        // =====================================================================

        sf::SystemAlert alert;
        while (sf::system_alert.read(alert)) {
            g_state_manager.handleAlert(alert);
        }
    }
}
