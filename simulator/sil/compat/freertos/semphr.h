/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL/test compatibility shim).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file semphr.h (host shim)
 * @brief FreeRTOS mutex semaphore mapped to std::mutex for host builds
 *        ホストビルド用にstd::mutexへマップしたFreeRTOSミューテックス
 *
 * Used by TopicLatest in topic.hpp. The host is single-process so the mutex
 * is functionally a no-op for determinism, but real locking keeps the
 * semantics identical to the target.
 *
 * topic.hpp の TopicLatest が使用。ホストは単一プロセスのため実質no-opだが、
 * 実際にロックを取ることでターゲットと意味を一致させる。
 */

#pragma once

#include "freertos/FreeRTOS.h"
// Real FreeRTOS semphr.h pulls in queue.h (semaphores are queue-backed);
// topic.hpp relies on that transitive include for TopicQueue. Mirror it.
// 本家 semphr.h は queue.h を取り込む（セマフォはキュー実装）。topic.hpp は
// その推移的インクルードに依存して TopicQueue を使う。これを模倣する。
#include "freertos/queue.h"
#include <mutex>

// Opaque mutex handle backed by std::mutex
// std::mutex を裏に持つ不透明ミューテックスハンドル
typedef std::mutex* SemaphoreHandle_t;

/// Create a mutex semaphore
/// ミューテックスセマフォを生成する
inline SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    return new std::mutex();
}

/// Take (lock) the mutex — timeout ignored on host
/// ミューテックスを取得（ロック）する — ホストではタイムアウト無視
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t /*timeout*/)
{
    if (handle) handle->lock();
    return pdTRUE;
}

/// Give (unlock) the mutex
/// ミューテックスを解放（アンロック）する
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t handle)
{
    if (handle) handle->unlock();
    return pdTRUE;
}
