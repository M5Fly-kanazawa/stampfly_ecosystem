/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL/test compatibility shim).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file queue.h (host shim)
 * @brief FreeRTOS queue mapped to a byte-copy std::deque for host builds
 *        ホストビルド用にバイトコピーstd::dequeへマップしたFreeRTOSキュー
 *
 * Used by TopicQueue in topic.hpp. Items are fixed-size byte blobs (sizeof(T)),
 * copied in/out exactly like the FreeRTOS queue. Non-blocking: a full queue
 * drops on send, an empty queue returns pdFALSE on receive.
 *
 * topic.hpp の TopicQueue が使用。要素は固定長バイト列(sizeof(T))で、
 * FreeRTOS キューと同じく値コピーで出し入れする。ノンブロッキング:
 * 満杯なら送信ドロップ、空なら受信は pdFALSE。
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include <deque>
#include <vector>
#include <mutex>
#include <cstring>

// Host queue: fixed-capacity FIFO of fixed-size byte items
// ホストキュー: 固定長要素の固定容量FIFO
struct HostQueue {
    size_t   item_size = 0;   // Bytes per item / 要素あたりバイト数
    size_t   capacity  = 0;   // Max items      / 最大要素数
    std::deque<std::vector<uint8_t>> items;
    std::mutex m;
};

typedef HostQueue* QueueHandle_t;

/// Create a queue holding `length` items of `item_size` bytes each
/// 各 item_size バイトの要素を length 個保持するキューを生成する
inline QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size)
{
    HostQueue* q = new HostQueue();
    q->item_size = item_size;
    q->capacity  = length;
    return q;
}

/// Send one item (copy). Drops and returns pdFALSE if full. Timeout ignored.
/// 1要素を送信（コピー）。満杯なら pdFALSE。タイムアウト無視。
inline BaseType_t xQueueSend(QueueHandle_t q, const void* item, TickType_t /*timeout*/)
{
    if (!q) return pdFALSE;
    std::lock_guard<std::mutex> lk(q->m);
    if (q->items.size() >= q->capacity) return pdFALSE;  // errQUEUE_FULL
    const uint8_t* p = static_cast<const uint8_t*>(item);
    q->items.emplace_back(p, p + q->item_size);
    return pdTRUE;
}

/// Receive one item (copy) into `out`. Returns pdFALSE if empty. Timeout ignored.
/// 1要素を out へ受信（コピー）。空なら pdFALSE。タイムアウト無視。
inline BaseType_t xQueueReceive(QueueHandle_t q, void* out, TickType_t /*timeout*/)
{
    if (!q) return pdFALSE;
    std::lock_guard<std::mutex> lk(q->m);
    if (q->items.empty()) return pdFALSE;
    std::memcpy(out, q->items.front().data(), q->item_size);
    q->items.pop_front();
    return pdTRUE;
}

/// Number of items currently waiting in the queue
/// 現在キューに待機中の要素数
inline UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q)
{
    if (!q) return 0;
    std::lock_guard<std::mutex> lk(q->m);
    return static_cast<UBaseType_t>(q->items.size());
}
