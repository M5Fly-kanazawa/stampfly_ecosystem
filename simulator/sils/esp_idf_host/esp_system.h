/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — ESP-IDF host platform).
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

// Reset reason (ESP-IDF enum subset). On the host a run is always a clean
// power-on, so esp_reset_reason() returns ESP_RST_POWERON.
// リセット理由（ESP-IDF enum の部分集合）。ホストでは常にクリーン起動を返す。
typedef enum {
    ESP_RST_UNKNOWN   = 0,
    ESP_RST_POWERON   = 1,
    ESP_RST_EXT       = 2,
    ESP_RST_SW        = 3,
    ESP_RST_PANIC     = 4,
    ESP_RST_INT_WDT   = 5,
    ESP_RST_TASK_WDT  = 6,
    ESP_RST_WDT       = 7,
    ESP_RST_DEEPSLEEP = 8,
    ESP_RST_BROWNOUT  = 9,
    ESP_RST_SDIO      = 10,
} esp_reset_reason_t;

static inline esp_reset_reason_t esp_reset_reason(void) { return ESP_RST_POWERON; }

static inline void esp_chip_info(esp_chip_info_t* out)
{
    if (out) { out->model = CHIP_ESP32S3; out->features = 0; out->revision = 0; out->cores = 2; }
}

static inline uint32_t esp_get_free_heap_size(void)      { return (uint32_t)1 << 20; }
static inline uint32_t esp_get_minimum_free_heap_size(void) { return (uint32_t)1 << 20; }
static inline uint32_t esp_random(void)                  { return (uint32_t)rand(); }
static inline const char* esp_get_idf_version(void)      { return "sils-host"; }

#ifdef __cplusplus
}
#endif
