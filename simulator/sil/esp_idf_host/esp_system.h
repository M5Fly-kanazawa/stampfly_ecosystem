/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file esp_system.h
 * @brief Host shim for ESP-IDF system helpers (reset, heap, chip info, random).
 *        ESP-IDF システムヘルパのホスト用シム（リセット・ヒープ・チップ情報・乱数）。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { CHIP_ESP32 = 1, CHIP_ESP32S2 = 2, CHIP_ESP32S3 = 9 } esp_chip_model_t;

typedef struct {
    esp_chip_model_t model;
    uint32_t features;
    uint16_t revision;
    uint8_t  cores;
} esp_chip_info_t;

// Reboot. On the host, just exit cleanly (no real reset).
// 再起動。ホストではクリーンに終了するだけ。
static inline void esp_restart(void) { exit(0); }

static inline void esp_chip_info(esp_chip_info_t* out)
{
    if (out) { out->model = CHIP_ESP32S3; out->features = 0; out->revision = 0; out->cores = 2; }
}

static inline uint32_t esp_get_free_heap_size(void)      { return (uint32_t)1 << 20; }
static inline uint32_t esp_get_minimum_free_heap_size(void) { return (uint32_t)1 << 20; }
static inline uint32_t esp_random(void)                  { return (uint32_t)rand(); }
static inline const char* esp_get_idf_version(void)      { return "sil-host"; }

#ifdef __cplusplus
}
#endif
