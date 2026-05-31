/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file semphr.h
 * @brief Host stub for FreeRTOS mutex semaphores (passive)
 *        FreeRTOS ミューテックスセマフォのホスト用スタブ（受動）
 *
 * Backed by std::mutex. In the SIL's cooperative single-token execution a
 * topic mutex is only ever held across a short critical section (no yield in
 * between), so a plain mutex never deadlocks the scheduler.
 *
 * std::mutex で実装。SIL の協調シングルトークン実行では、トピックの
 * ミューテックスは短いクリティカルセクションの間だけ保持される（途中で
 * ゆずらない）ので、素のミューテックスでもスケジューラをデッドロックさせない。
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include <mutex>

// Opaque handle to a host mutex.
// ホストミューテックスへの不透明ハンドル。
typedef void* SemaphoreHandle_t;

inline SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    return new std::mutex();
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t /*timeout*/)
{
    static_cast<std::mutex*>(handle)->lock();
    return pdTRUE;
}

inline BaseType_t xSemaphoreGive(SemaphoreHandle_t handle)
{
    static_cast<std::mutex*>(handle)->unlock();
    return pdTRUE;
}

inline void vSemaphoreDelete(SemaphoreHandle_t handle)
{
    delete static_cast<std::mutex*>(handle);
}
