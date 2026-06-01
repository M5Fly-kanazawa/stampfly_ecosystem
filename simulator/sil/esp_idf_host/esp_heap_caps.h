/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file esp_heap_caps.h
 * @brief Host shim for capability-based heap allocation (heap_caps_*).
 *        能力指定ヒープ確保（heap_caps_*）のホスト用シム。
 *
 * ESP-IDF allocates DMA-capable / internal / SPIRAM memory via capability
 * flags. On the host all memory is equivalent, so the caps are ignored and
 * everything routes to malloc/calloc/realloc/free.
 * ESP-IDF は DMA 可能 / 内部 / SPIRAM メモリを能力フラグで確保する。ホストでは
 * メモリは等価なので caps を無視し malloc/calloc/realloc/free に流す。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

// Capability flags (values mirror ESP-IDF; ignored on host).
// 能力フラグ（値は ESP-IDF に合わせる。ホストでは無視）。
#define MALLOC_CAP_EXEC       (1 << 0)
#define MALLOC_CAP_32BIT      (1 << 1)
#define MALLOC_CAP_8BIT       (1 << 2)
#define MALLOC_CAP_DMA        (1 << 3)
#define MALLOC_CAP_SPIRAM     (1 << 10)
#define MALLOC_CAP_INTERNAL   (1 << 11)
#define MALLOC_CAP_DEFAULT    (1 << 12)

#ifdef __cplusplus
extern "C" {
#endif

static inline void* heap_caps_malloc(size_t size, uint32_t caps)
{
    (void)caps;
    return malloc(size);
}

static inline void* heap_caps_calloc(size_t n, size_t size, uint32_t caps)
{
    (void)caps;
    return calloc(n, size);
}

static inline void* heap_caps_realloc(void* ptr, size_t size, uint32_t caps)
{
    (void)caps;
    return realloc(ptr, size);
}

static inline void* heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps)
{
    (void)caps; (void)alignment;
    return malloc(size);
}

static inline void heap_caps_free(void* ptr)
{
    free(ptr);
}

static inline size_t heap_caps_get_free_size(uint32_t caps)
{
    (void)caps;
    return (size_t)1 << 20;   // pretend 1 MiB free
}

#ifdef __cplusplus
}
#endif
