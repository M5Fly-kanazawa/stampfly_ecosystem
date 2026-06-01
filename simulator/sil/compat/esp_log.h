/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file esp_log.h
 * @brief Host stub for ESP-IDF logging — redirects ESP_LOGx to stderr
 *        ESP-IDF ロギングのホスト用スタブ — ESP_LOGx を stderr に流す
 *
 * The SIL compiles the unmodified firmware on a PC. ESP-IDF logging is
 * redirected to stderr so it never pollutes stdout (which carries the
 * SIL's machine-readable results / CSV).
 *
 * SIL は本体ファームを無改変で PC 上にコンパイルする。ESP-IDF の
 * ロギングは stderr に流し、stdout（SIL の機械可読な結果・CSV）を
 * 汚さないようにする。
 */

#pragma once

#include <stdio.h>

// esp_heap_caps is broadly available on ESP-IDF (drivers call heap_caps_malloc
// with MALLOC_CAP_DMA without an explicit include). Re-export via esp_log.h,
// which the HAL .c drivers include, to mirror that transitive availability.
// ESP-IDF では heap_caps が広く使えるので、HAL ドライバが include する esp_log.h
// から再エクスポートして透過的な可用性を再現する。
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"    // esp_rom_delay_us / esp_rom_printf (broadly available)
#include "esp_err.h"

// Log levels map to stderr prints. ESP_LOGD/V are silenced by default.
// ログレベルは stderr 出力にマップ。ESP_LOGD/V は既定で無音。
#define ESP_LOGI(tag, fmt, ...) fprintf(stderr, "[INFO] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fprintf(stderr, "[WARN] %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) fprintf(stderr, "[ERR]  %s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)

// SIL addition: log levels (old firmware uses esp_log_level_t / esp_log_level_set).
typedef enum {
    ESP_LOG_NONE = 0, ESP_LOG_ERROR = 1, ESP_LOG_WARN = 2,
    ESP_LOG_INFO = 3, ESP_LOG_DEBUG = 4, ESP_LOG_VERBOSE = 5,
} esp_log_level_t;
static inline void esp_log_level_set(const char* tag, esp_log_level_t level) { (void)tag; (void)level; }
