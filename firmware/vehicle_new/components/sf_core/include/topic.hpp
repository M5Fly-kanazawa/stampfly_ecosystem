/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file topic.hpp
 * @brief Lightweight Pub-Sub topic template
 *        軽量Pub-Subトピックテンプレート
 *
 * Provides a type-safe, compile-time topic system for inter-component
 * communication. Publishers and subscribers are decoupled — neither
 * knows the other. Data is exchanged via typed shared buffers.
 *
 * コンポーネント間通信のための型安全なコンパイル時トピックシステム。
 * 発行者と購読者は疎結合 — 互いを知らない。
 * 型付き共有バッファ経由でデータを交換する。
 *
 * @design architecture.md §3 — Lightweight Pub-Sub                    [--]
 * @design detailed_design.md §2 — Topic<T, BufferPolicy, Size>        [--]
 * @design coding_and_education.md §2 — Pub-Sub separation rule        [--]
 */

#pragma once

#include <cstring>
#include <cstdint>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace sf {

// =============================================================================
// Buffer Policies
// バッファ方式
//
// @design detailed_design.md §2 — Three buffer policies               [--]
// =============================================================================

/// Latest-value buffer: stores only the most recent value
/// 最新値バッファ: 最新の値のみを保持
///
/// Used for: estimate → control, estimate → telemetry
/// 用途: 推定値→制御、推定値→テレメトリ
struct Latest {};

/// Ring buffer: lock-free SPSC, retains all samples
/// リングバッファ: ロックフリーSPSC、全サンプル保持
///
/// Used for: IMU → estimation, all data → logger
/// 用途: IMU→推定、全データ→ログ
struct RingBuffer {};

/// Queue: FreeRTOS Queue with buffering
/// キュー: FreeRTOS Queueによるバッファリング
///
/// Used for: low-rate sensors (ToF, Flow, Mag, Baro)
/// 用途: 低レートセンサ（ToF、Flow、Mag、Baro）
struct Queue {};

// =============================================================================
// Topic Template — Latest Policy
// トピックテンプレート — 最新値方式
// =============================================================================

/// Topic with Latest buffer policy
/// 最新値バッファ方式のトピック
template<typename T>
class TopicLatest {
public:
    /// Publish a new value (thread-safe)
    /// 新しい値を発行する（スレッドセーフ）
    void publish(const T& data)
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        data_ = data;
        updated_ = true;
        xSemaphoreGive(mutex_);
    }

    /// Get the latest value
    /// 最新の値を取得する
    T latest() const
    {
        T copy;
        xSemaphoreTake(mutex_, portMAX_DELAY);
        copy = data_;
        xSemaphoreGive(mutex_);
        return copy;
    }

    /// Check if new data is available (and clear the flag)
    /// 新しいデータがあるか確認する（フラグをクリアする）
    bool updated()
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        bool was_updated = updated_;
        updated_ = false;
        xSemaphoreGive(mutex_);
        return was_updated;
    }

    /// Initialize the topic (must be called before use)
    /// トピックを初期化する（使用前に呼ぶこと）
    void init()
    {
        mutex_ = xSemaphoreCreateMutex();
        data_ = {};
        updated_ = false;
    }

    /// Overflow count — Latest overwrites by design (no loss concept), always 0.
    /// Provided for R14 uniformity so monitors can query every topic the same way.
    /// オーバーフロー数 — Latest は上書きが仕様（喪失概念なし）で常に0。R14 の統一性のため
    /// 監視側が全トピックを同じ方法で問い合わせられるよう提供する。
    uint32_t overflowCount() const { return 0; }

private:
    mutable SemaphoreHandle_t mutex_ = nullptr;
    T data_ = {};
    bool updated_ = false;
};

// =============================================================================
// Topic Template — Ring Buffer Policy (SPSC Lock-free)
// トピックテンプレート — リングバッファ方式（SPSCロックフリー）
// =============================================================================

/// Topic with lock-free SPSC ring buffer
/// ロックフリーSPSCリングバッファのトピック
///
/// Single producer, single consumer. ISR-safe for writing.
/// シングルプロデューサー、シングルコンシューマー。書き込みはISR安全。
template<typename T, int Size>
class TopicRing {
    static_assert(Size > 0 && (Size & (Size - 1)) == 0,
                  "Ring buffer size must be a power of 2");
public:
    /// Publish a new value (lock-free, ISR-safe)
    /// 新しい値を発行する（ロックフリー、ISR安全）
    void publish(const T& data)
    {
        uint32_t head = head_.load(std::memory_order_relaxed);
        uint32_t next = (head + 1) & mask_;
        // Full when the next slot is the consumer's tail: writing here overwrites an
        // unread sample, so count the loss (R14). The write still proceeds (drop-oldest).
        // 次スロットが consumer の tail なら満杯: ここに書くと未読サンプルを上書きするので
        // 喪失をカウント（R14）。書き込み自体は続行（最古をドロップ）。
        if (next == tail_.load(std::memory_order_acquire)) {
            overflow_count_.fetch_add(1, std::memory_order_relaxed);
        }
        buf_[head] = data;
        head_.store(next, std::memory_order_release);
    }

    /// Read next available value, returns false if empty
    /// 次の利用可能な値を読む、空ならfalseを返す
    bool read(T& out)
    {
        uint32_t tail = tail_.load(std::memory_order_relaxed);
        uint32_t head = head_.load(std::memory_order_acquire);
        if (tail == head) {
            return false;  // Empty / 空
        }
        out = buf_[tail];
        tail_.store((tail + 1) & mask_, std::memory_order_release);
        return true;
    }

    /// Get the latest value without consuming
    /// 消費せずに最新の値を取得する
    T latest() const
    {
        uint32_t head = head_.load(std::memory_order_acquire);
        uint32_t idx = (head == 0) ? (Size - 1) : (head - 1);
        return buf_[idx];
    }

    /// Check if data is available
    /// データがあるか確認する
    bool available() const
    {
        return tail_.load(std::memory_order_relaxed) !=
               head_.load(std::memory_order_acquire);
    }

    /// Initialize the topic
    /// トピックを初期化する
    void init()
    {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
        overflow_count_.store(0, std::memory_order_relaxed);
        memset(buf_, 0, sizeof(buf_));
    }

    /// Overflow count — samples lost to drop-oldest while the ring was full (R14)
    /// オーバーフロー数 — 満杯時に最古ドロップで失われたサンプル数（R14）
    uint32_t overflowCount() const { return overflow_count_.load(std::memory_order_relaxed); }

private:
    static constexpr uint32_t mask_ = Size - 1;
    T buf_[Size] = {};
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
    std::atomic<uint32_t> overflow_count_{0};
};

// =============================================================================
// Topic Template — Queue Policy (FreeRTOS Queue)
// トピックテンプレート — キュー方式（FreeRTOS Queue）
// =============================================================================

/// Topic with FreeRTOS Queue
/// FreeRTOS Queueを使ったトピック
template<typename T, int Size>
class TopicQueue {
public:
    /// Publish a new value (non-blocking, drops if full)
    /// 新しい値を発行する（ノンブロッキング、満杯時はドロップ）
    void publish(const T& data)
    {
        // Non-blocking send; if the queue is full the sample is dropped — count it (R14).
        // ノンブロッキング送信。満杯ならサンプルは破棄される — カウントする（R14）。
        if (xQueueSend(queue_, &data, 0) != pdTRUE) {
            overflow_count_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    /// Read next value, returns false if empty
    /// 次の値を読む、空ならfalseを返す
    bool read(T& out)
    {
        return xQueueReceive(queue_, &out, 0) == pdTRUE;
    }

    /// Read next value with timeout
    /// タイムアウト付きで次の値を読む
    bool read(T& out, TickType_t timeout)
    {
        return xQueueReceive(queue_, &out, timeout) == pdTRUE;
    }

    /// Check if data is available
    /// データがあるか確認する
    bool available() const
    {
        return uxQueueMessagesWaiting(queue_) > 0;
    }

    /// Initialize the topic
    /// トピックを初期化する
    void init()
    {
        queue_ = xQueueCreate(Size, sizeof(T));
        overflow_count_.store(0, std::memory_order_relaxed);
    }

    /// Overflow count — samples dropped because the queue was full (R14)
    /// オーバーフロー数 — キュー満杯で破棄されたサンプル数（R14）
    uint32_t overflowCount() const { return overflow_count_.load(std::memory_order_relaxed); }

private:
    QueueHandle_t queue_ = nullptr;
    std::atomic<uint32_t> overflow_count_{0};
};

// =============================================================================
// Topic type alias — selects implementation based on policy
// トピック型エイリアス — 方式に基づいて実装を選択
// =============================================================================

/// Primary topic template: select buffer policy via second parameter
/// メイントピックテンプレート: 第2引数でバッファ方式を選択
///
/// Usage / 使い方:
///   Topic<ImuData, RingBuffer, 8>  — lock-free ring, 8 slots
///   Topic<TofData, Queue, 2>       — FreeRTOS queue, 2 slots
///   Topic<StateEstimate, Latest, 1> — latest value only
///
template<typename T, typename Policy, int Size = 1>
struct Topic;

template<typename T, int Size>
struct Topic<T, Latest, Size> : TopicLatest<T> {};

template<typename T, int Size>
struct Topic<T, RingBuffer, Size> : TopicRing<T, Size> {};

template<typename T, int Size>
struct Topic<T, Queue, Size> : TopicQueue<T, Size> {};

}  // namespace sf
