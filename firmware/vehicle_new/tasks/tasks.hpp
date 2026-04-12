/**
 * @file tasks.hpp
 * @brief FreeRTOS task function declarations
 *        FreeRTOSタスク関数宣言
 *
 * @design architecture.md §6 — Task design (14 tasks)                 [--]
 * @design detailed_design.md §8 — Task list                           [--]
 */

#pragma once

/// IMU reading + state estimation (400Hz, priority 24)
/// IMU読み取り + 状態推定（400Hz、優先度24）
void ImuTask(void* pvParameters);

/// Control computation + actuation (400Hz IMU-synced, priority 23)
/// 制御演算 + アクチュエーション（400Hz IMU同期、優先度23）
void ControlTask(void* pvParameters);

/// State management — event-driven (priority 22)
/// 状態管理 — イベント駆動（優先度22）
void StateTask(void* pvParameters);
