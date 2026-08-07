/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file queue.h
 * @brief Host stub for FreeRTOS queues (passive, byte-copy ring)
 *        FreeRTOS キューのホスト用スタブ（受動・バイトコピーのリング）
 *
 * A fixed-capacity FIFO of fixed-size items. Send drops when full (the same
 * lossy behaviour the firmware relies on for low-rate sensor topics). The
 * blocking receive timeout is ignored here; P1.1's scheduler integrates true
 * blocking for tasks that wait on a queue.
 *
 * 固定サイズ要素の固定容量 FIFO。満杯時の送信は捨てる（低レートのセンサ
 * トピックでファームが依存する欠落挙動と同じ）。ブロッキング受信の
 * タイムアウトはここでは無視。キューで待つタスクの真のブロッキングは
 * P1.1 のスケジューラが統合する。
 */

#pragma once

#include "freertos/FreeRTOS.h"

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <deque>
#include <vector>

// Internal queue object (one heap allocation per xQueueCreate).
// 内部キューオブジェクト（xQueueCreate ごとに1個ヒープ確保）。
struct SilQueue {
    size_t item_size = 0;
    size_t capacity = 0;
    std::deque<std::vector<uint8_t>> items;
};

// Opaque handle.
// 不透明ハンドル。
typedef void* QueueHandle_t;

inline QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size)
{
    auto* queue = new SilQueue();
    queue->item_size = item_size;
    queue->capacity = length;
    return queue;
}

inline BaseType_t xQueueSend(QueueHandle_t handle, const void* item, TickType_t /*timeout*/)
{
    auto* queue = static_cast<SilQueue*>(handle);
    if (queue->items.size() >= queue->capacity) {
        return pdFALSE;  // full → drop / 満杯 → 捨てる
    }
    const uint8_t* bytes = static_cast<const uint8_t*>(item);
    queue->items.emplace_back(bytes, bytes + queue->item_size);
    return pdTRUE;
}

// Forward decl of the cooperative-scheduler delay (defined in rtos/scheduler.cpp,
// declared in freertos/task.h). A task blocking on an empty queue must YIELD so
// the cooperative scheduler does not see an infinite loop — real FreeRTOS blocks
// until xQueueSend; here we yield ~1 tick and let the task re-poll. Deterministic.
// 空キューでブロックするタスクは協調スケジューラで yield せねばならない（無限ループ
// 扱いを避ける）。実 FreeRTOS は xQueueSend までブロック、ここは ~1tick yield。
extern "C" void vTaskDelay(TickType_t ticks);

inline BaseType_t xQueueReceive(QueueHandle_t handle, void* out, TickType_t timeout)
{
    auto* queue = static_cast<SilQueue*>(handle);
    if (queue->items.empty()) {
        if (timeout != 0) {
            vTaskDelay(timeout < (TickType_t)1 ? timeout : (TickType_t)1);
        }
        if (queue->items.empty()) return pdFALSE;
    }
    std::memcpy(out, queue->items.front().data(), queue->item_size);
    queue->items.pop_front();
    return pdTRUE;
}

inline UBaseType_t uxQueueMessagesWaiting(QueueHandle_t handle)
{
    return static_cast<UBaseType_t>(static_cast<SilQueue*>(handle)->items.size());
}

inline void vQueueDelete(QueueHandle_t handle)
{
    delete static_cast<SilQueue*>(handle);
}

// SILS addition: drop all queued items.
inline BaseType_t xQueueReset(QueueHandle_t handle)
{ static_cast<SilQueue*>(handle)->items.clear(); return pdTRUE; }

// SILS addition: free slots in the queue.
inline UBaseType_t uxQueueSpacesAvailable(QueueHandle_t handle)
{ auto* q = static_cast<SilQueue*>(handle); return (UBaseType_t)(q->capacity - q->items.size()); }
