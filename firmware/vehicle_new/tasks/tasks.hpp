/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file tasks.hpp
 * @brief FreeRTOS task function declarations
 *        FreeRTOSタスク関数宣言
 *
 * All 14 task functions declared here. Each task file implements
 * one function. Tasks are started in main.cpp app_main().
 *
 * 全14タスク関数をここで宣言。各タスクファイルが1関数を実装する。
 * タスクはmain.cppのapp_main()で起動される。
 *
 * @design architecture.md §6 — Task design (14 tasks)                 [OK]
 * @design detailed_design.md §8 — Task list                           [OK]
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// === Task lifecycle (sf::tasks) ===
// === タスクのライフサイクル（sf::tasks） ===

namespace sf {
namespace tasks {

/// Create all 14 FreeRTOS tasks with their configured priorities/stacks/cores.
/// Called once from app_main() Phase 4 — keeps the entry point declarative.
/// 14 個の FreeRTOS タスクを所定の優先度/スタック/コアで生成する。app_main() の
/// Phase 4 から 1 回呼ぶ — エントリポイントを宣言的に保つ。
///
/// @design hardware_init.md §4 — Phase 4: tasks::start_all()           [OK]
void start_all();

/// Handle of the ControlTask, registered by the task itself at startup
/// (xTaskGetCurrentTaskHandle). ImuTask reads this to wake ControlTask after each
/// estimate. Returns nullptr until ControlTask has run its setup. Replaces the old
/// extern global g_control_task_handle (R3: no extern task handles).
/// ControlTask のハンドル。タスク自身が起動時に登録する（xTaskGetCurrentTaskHandle）。
/// ImuTask が推定後に ControlTask を起こすために読む。ControlTask の setup 実行前は
/// nullptr。旧 extern グローバル g_control_task_handle を置き換える（R3: extern 排除）。
TaskHandle_t control_handle();

}  // namespace tasks
}  // namespace sf

// === Core pipeline tasks (highest priority)
// === コアパイプラインタスク（最高優先度）

/// IMU reading + state estimation (400Hz, priority 24)
/// IMU読み取り + 状態推定（400Hz、優先度24）
void ImuTask(void* pvParameters);

/// Control + actuation (400Hz IMU-synced, priority 23)
/// 制御 + アクチュエーション（400Hz IMU同期、優先度23）
void ControlTask(void* pvParameters);

/// State management — event-driven (priority 22)
/// 状態管理 — イベント駆動（優先度22）
void StateTask(void* pvParameters);

// === Sensor tasks
// === センサタスク

/// Optical flow (100Hz, priority 20)
/// オプティカルフロー（100Hz、優先度20）
void FlowTask(void* pvParameters);

/// Magnetometer (25Hz, priority 18)
/// 地磁気（25Hz、優先度18）
void MagTask(void* pvParameters);

/// Barometer (50Hz, priority 16)
/// 気圧（50Hz、優先度16）
void BaroTask(void* pvParameters);

/// ToF + takeoff/landing manager (30Hz, priority 14)
/// ToF + 離着陸マネージャー（30Hz、優先度14）
void TofTask(void* pvParameters);

// === Communication and service tasks
// === 通信・サービスタスク

/// Communication + command processing (50Hz, priority 15)
/// 通信 + コマンド処理（50Hz、優先度15）
void CommTask(void* pvParameters);

/// Telemetry — UDP (50Hz, priority 13)
/// テレメトリ — UDP（50Hz、優先度13）
void TelemetryTask(void* pvParameters);

/// Power monitor + failsafe (10Hz, priority 12)
/// 電源モニタ + フェイルセーフ（10Hz、優先度12）
void PowerTask(void* pvParameters);

/// Button input (50Hz, priority 10)
/// ボタン入力（50Hz、優先度10）
void ButtonTask(void* pvParameters);

/// Notification — LED/buzzer (30Hz, priority 8)
/// 通知 — LED/ブザー（30Hz、優先度8）
void NotifyTask(void* pvParameters);

/// CLI + parameter access (20Hz, priority 5)
/// CLI + パラメータアクセス（20Hz、優先度5）
void CLITask(void* pvParameters);

/// Data logger + Blackbox (async, priority 5)
/// データロガー + Blackbox（非同期、優先度5）
void LogTask(void* pvParameters);
