/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file task.h
 * @brief Host stub for FreeRTOS tasks — the ACTIVE surface of the RTOS shim.
 *        FreeRTOS タスクのホスト用スタブ — RTOS シムの能動面。
 *
 * Declares the task/notification/delay functions the firmware tasks call.
 * Their definitions live in the deterministic cooperative scheduler
 * (simulator/sils/rtos/). The firmware runs UNMODIFIED against these.
 *
 * 本体タスクが呼ぶタスク/通知/遅延の関数を宣言する。実装は決定論的協調
 * スケジューラ（simulator/sils/rtos/）にある。本体ファームは無改変でこれらに
 * 対して動く。
 *
 * Single virtual clock; one task runs at a time (cooperative). See
 * rtos/scheduler.hpp for the model.
 *
 * 単一仮想時計・一度に1タスク（協調）。モデルは rtos/scheduler.hpp 参照。
 */

#pragma once

#include "freertos/FreeRTOS.h"

// Opaque task handle (points to a sils::rtos::Task).
// 不透明タスクハンドル（sils::rtos::Task を指す）。
typedef void* TaskHandle_t;

// Task entry point signature.
// タスクのエントリ関数シグネチャ。
typedef void (*TaskFunction_t)(void*);

#ifdef __cplusplus
extern "C" {
#endif

// Create a task. `core` is accepted but ignored (cooperative single core).
// タスクを生成する。`core` は受け取るが無視（協調・単一コア）。
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char* name,
                                   uint32_t stack_depth, void* param,
                                   UBaseType_t priority, TaskHandle_t* out_handle,
                                   BaseType_t core);

// Delete a task (NULL = the calling task). The calling task does not return.
// タスクを削除する（NULL = 呼び出しタスク自身）。呼び出しタスクは戻らない。
void vTaskDelete(TaskHandle_t handle);

// Block the calling task for `ticks` (relative).
// 呼び出しタスクを `ticks`（相対）ブロックする。
void vTaskDelay(TickType_t ticks);

// Block until *last_wake + period (absolute periodic). Updates *last_wake.
// *last_wake + period まで（絶対周期）ブロックする。*last_wake を更新する。
void vTaskDelayUntil(TickType_t* last_wake, TickType_t period);

// Current tick count from the virtual clock (1 tick = 1 ms).
// 仮想時計からの現在 tick（1 tick = 1 ms）。
TickType_t xTaskGetTickCount(void);

// Handle of the task currently running (used to register esp_timer notifications).
// 現在走っているタスクのハンドル（esp_timer 通知の登録に使う）。
TaskHandle_t xTaskGetCurrentTaskHandle(void);

// Give a notification to `handle` (increments its notification value).
// `handle` に通知を1つ与える（通知値をインクリメント）。
BaseType_t xTaskNotifyGive(TaskHandle_t handle);

// Take a notification: block until notified, then return the count.
// `clear` (pdTRUE) zeroes the count, else decrements by one.
// 通知を取る: 通知が来るまでブロックし、カウントを返す。
// `clear`(pdTRUE) でカウントを 0 に、そうでなければ 1 減らす。
uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t timeout);

#ifdef __cplusplus
}  // extern "C"
#endif

// SILS additions: non-pinned create (→ core 0) + stack high-water (stub).
static inline BaseType_t xTaskCreate(TaskFunction_t fn, const char* name, uint32_t stack,
                                     void* param, UBaseType_t prio, TaskHandle_t* handle)
{ return xTaskCreatePinnedToCore(fn, name, stack, param, prio, handle, 0); }
static inline UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t h) { (void)h; return 4096; }
