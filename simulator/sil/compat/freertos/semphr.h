/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file semphr.h
 * @brief Host stub for FreeRTOS semaphores — counter model, cooperative-safe.
 *        FreeRTOS セマフォのホスト用スタブ — カウンタモデル・協調安全。
 *
 * A semaphore is a bounded counter (mutex = {1,1}, binary = {0,1}, counting =
 * {init,max}). Take decrements (blocks if 0); Give increments. On the SINGLE-token
 * cooperative scheduler there is no real contention during a critical section
 * (the holder runs to completion before any other task), so a counter is exact
 * for mutual exclusion. For cross-task signalling (binary/counting), a Take that
 * finds the counter at 0 YIELDS ~1 tick and re-polls — real FreeRTOS would block
 * until a Give; here the blocked task must yield or the scheduler sees an infinite
 * loop. Deterministic (virtual-clock ticks).
 *
 * セマフォは有界カウンタ（mutex={1,1}/binary={0,1}/counting={init,max}）。Take は減算
 * （0 ならブロック）、Give は加算。単一トークンの協調スケジューラでは臨界区間中に実競合は
 * 起きない。クロスタスク信号（binary/counting）では Take が 0 を見たら ~1tick yield して
 * 再ポーリング。以前の std::mutex 実装はバイナリセマフォの意味論を満たさず、同一スレッドの
 * 二重 Take で自己ロックしていた（旧ファームの log_writer がこれでハング）。
 */

#pragma once

#include "freertos/FreeRTOS.h"

// Cooperative yield (defined in rtos/scheduler.cpp, declared in freertos/task.h).
// 協調 yield（rtos/scheduler.cpp に定義、freertos/task.h に宣言）。
extern "C" void vTaskDelay(TickType_t ticks);

// Internal semaphore object: a bounded counter.
// 内部セマフォオブジェクト: 有界カウンタ。
struct SilSem {
    int count;
    int max_count;
};

typedef void* SemaphoreHandle_t;

inline SemaphoreHandle_t xSemaphoreCreateMutex(void)  { return new SilSem{1, 1}; }
inline SemaphoreHandle_t xSemaphoreCreateBinary(void) { return new SilSem{0, 1}; }
inline SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t max, UBaseType_t initial)
{ return new SilSem{(int)initial, (int)max}; }

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t timeout)
{
    auto* s = static_cast<SilSem*>(handle);
    if (s->count > 0) { s->count--; return pdTRUE; }
    if (timeout == 0) return pdFALSE;
    // Block cooperatively: yield ~1 tick, then re-check once. The task re-polls
    // on its own loop until a Give raises the count.
    // 協調的にブロック: ~1tick yield して1回再確認。タスクは自身のループで再ポーリング。
    vTaskDelay(timeout < (TickType_t)1 ? timeout : (TickType_t)1);
    if (s->count > 0) { s->count--; return pdTRUE; }
    return pdFALSE;
}

inline BaseType_t xSemaphoreGive(SemaphoreHandle_t handle)
{
    auto* s = static_cast<SilSem*>(handle);
    if (s->count < s->max_count) s->count++;
    return pdTRUE;
}

inline BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t handle, BaseType_t* hp_woken)
{
    if (hp_woken) *hp_woken = pdFALSE;
    return xSemaphoreGive(handle);
}

inline void vSemaphoreDelete(SemaphoreHandle_t handle)
{
    delete static_cast<SilSem*>(handle);
}
