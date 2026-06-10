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
 * @design architecture.md §3 — Lightweight Pub-Sub                    [OK]
 * @design detailed_design.md §2 — Topic<T, BufferPolicy, Size>        [OK]
 * @design coding_and_education.md §2 — Pub-Sub separation rule        [OK]
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
// @design detailed_design.md §2 — Three buffer policies               [OK]
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
///
/// Index design: head_/tail_ are FREE-RUNNING counters (only masked when
/// indexing the array). This distinguishes "empty" (head==tail) from "full"
/// (head−tail==Size) without sacrificing a slot, and removes the ABA hazard
/// in the retry loops below (a wrapped-around masked index would compare
/// equal; a free-running counter cannot).
/// インデックス設計: head_/tail_ は「フリーランニング」カウンタ（配列参照時のみ
/// マスク）。これにより「空」(head==tail) と「満杯」(head−tail==Size) をスロットを
/// 犠牲にせず区別でき、下のリトライループの ABA 問題（マスク済みインデックスは一周
/// すると同値に見える）も排除できる。
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
        uint32_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail >= static_cast<uint32_t>(Size)) {
            // Full: drop the OLDEST unread sample by advancing tail, and count the
            // loss (R14). CAS because the consumer may be claiming the same slot
            // concurrently — exactly one of us advances tail past it; if the
            // consumer wins, the slot was consumed (not lost) and space exists now.
            // (The old code instead set head==tail, which made the ring look EMPTY
            // — losing all Size−1 unread samples — and then overwrote a slot the
            // consumer might be copying.)
            // 満杯: tail を進めて「最古の未読」を1件ドロップし、喪失をカウント（R14）。
            // consumer が同じスロットを取りに来ている可能性があるため CAS — tail を
            // 進めるのはどちらか一方だけ。consumer が勝てばそのサンプルは消費済み
            // （喪失ではない）で空きもできている。（旧実装は head==tail にしてリングが
            // 「空」に見え、未読 Size−1 件を一括喪失した上、consumer がコピー中かも
            // しれないスロットを上書きしていた。）
            tail_.compare_exchange_strong(tail, tail + 1,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed);
            overflow_count_.fetch_add(1, std::memory_order_relaxed);
        }
        buf_[head & mask_] = data;
        head_.store(head + 1, std::memory_order_release);
    }

    /// Read next available value, returns false if empty
    /// 次の利用可能な値を読む、空ならfalseを返す
    bool read(T& out)
    {
        for (;;) {
            uint32_t tail = tail_.load(std::memory_order_relaxed);
            uint32_t head = head_.load(std::memory_order_acquire);
            if (tail == head) {
                return false;  // Empty / 空
            }
            out = buf_[tail & mask_];
            // Claim the slot AFTER copying: if the producer dropped it as the
            // oldest meanwhile (CAS fails), the possibly-torn copy is discarded
            // and we retry with the new tail — the returned value is never torn.
            // コピー後にスロットを確定（CAS）: その間に producer が最古として
            // ドロップしていたら（CAS 失敗）、破断の可能性があるコピーは捨てて
            // 新しい tail でリトライ — 返す値は決して破断しない。
            if (tail_.compare_exchange_strong(tail, tail + 1,
                                              std::memory_order_acq_rel,
                                              std::memory_order_relaxed)) {
                return true;
            }
        }
    }

    /// Get the latest value without consuming. Retries if a publish lands during
    /// the copy, so the returned value is always a consistent snapshot (the
    /// free-running head makes the before/after comparison ABA-safe).
    /// 消費せずに最新の値を取得する。コピー中に publish が重なったらリトライするため、
    /// 返り値は常に一貫したスナップショット（フリーランニング head により前後比較は
    /// ABA 安全）。
    T latest() const
    {
        for (;;) {
            uint32_t before = head_.load(std::memory_order_acquire);
            T copy = buf_[(before - 1) & mask_];
            uint32_t after = head_.load(std::memory_order_acquire);
            if (before == after) {
                return copy;
            }
        }
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

    /// Overflow count — samples lost to drop-oldest while the ring was full (R14).
    /// NOTE: a topic nobody consumes via read() (latest()-only use) accumulates
    /// this counter at the publish rate once the ring fills — interpret it only
    /// for topics that have a real read() consumer.
    /// オーバーフロー数 — 満杯時に最古ドロップで失われたサンプル数（R14）。
    /// 注: read() で消費する者がいないトピック（latest() 専用）はリングが埋まった後
    /// publish レートでカウントが増える — read() の実消費者がいるトピックでのみ解釈すること。
    uint32_t overflowCount() const { return overflow_count_.load(std::memory_order_relaxed); }

private:
    static constexpr uint32_t mask_ = Size - 1;
    T buf_[Size] = {};
    std::atomic<uint32_t> head_{0};   // free-running write counter / フリーラン書込カウンタ
    std::atomic<uint32_t> tail_{0};   // free-running read counter  / フリーラン読出カウンタ
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
