/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file esp_log.h
 * @brief Host stub for ESP-IDF logging — redirects ESP_LOGx to stderr
 *        ESP-IDF ロギングのホスト用の代替実装 — ESP_LOGx を stderr に流す
 *
 * The SILS compiles the unmodified firmware on a PC. ESP-IDF logging is
 * redirected to stderr so it never pollutes stdout (which carries the
 * SILS's machine-readable results / CSV).
 *
 * SILS は本体ファームを無改変で PC 上にコンパイルする。ESP-IDF の
 * ロギングは stderr に流し、stdout（SILS の機械可読な結果・CSV）を
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

// ESP_LOGx go through esp_log_write(), like ESP-IDF: the line is formatted and handed to the
// current output function (esp_log_set_vprintf), which writes to stderr by default. The
// firmware's log sink (tasks/log_task.cpp) replaces that function, so the SILS runs the same
// copy-to-a-ring-then-write path as the target. Output text is unchanged: "[INFO] tag: message".
// ESP_LOGD/V are silenced by default.
// ESP_LOGx は ESP-IDF と同じく esp_log_write() を通る: 行を整形して現在の出力関数
// （esp_log_set_vprintf）へ渡し、既定では stderr へ書く。ファームのログシンク
// （tasks/log_task.cpp）がこの関数を置き換えるので、SILS も実機と同じ「リングへコピー→後で書く」
// 経路を通る。出力の文言は不変: "[INFO] tag: message"。ESP_LOGD/V は既定で無音。
#include <stdarg.h>

typedef int (*vprintf_like_t)(const char* format, va_list args);

typedef enum {
    ESP_LOG_NONE = 0, ESP_LOG_ERROR = 1, ESP_LOG_WARN = 2,
    ESP_LOG_INFO = 3, ESP_LOG_DEBUG = 4, ESP_LOG_VERBOSE = 5,
} esp_log_level_t;

static inline int esp_log_stderr_vprintf(const char* format, va_list args)
{
    return vfprintf(stderr, format, args);
}

// The installed output function (NULL = stderr). One variable for the whole program, defined in
// compat/log_shim.cpp, so the C sensor drivers and the C++ firmware log through the same path and
// their lines keep their relative order.
// 設定された出力関数（NULL = stderr）。プログラム全体で 1 つの変数で、compat/log_shim.cpp に定義する。
// C のセンサドライバも C++ のファームも同じ経路でログを出し、行の前後関係が保たれる。
#ifdef __cplusplus
extern "C" {
#endif
extern vprintf_like_t esp_log_output_function;
#ifdef __cplusplus
}
#endif

static inline vprintf_like_t esp_log_set_vprintf(vprintf_like_t function)
{
    vprintf_like_t previous = esp_log_output_function ? esp_log_output_function : esp_log_stderr_vprintf;
    esp_log_output_function = function;
    return previous;
}

static inline int esp_log_emit(vprintf_like_t function, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    const int written = function(format, args);
    va_end(args);
    return written;
}

#ifdef __cplusplus
inline void esp_log_write(esp_log_level_t level, const char* tag, const char* format, ...)
    __attribute__((format(printf, 3, 4)));
inline void esp_log_write(esp_log_level_t level, const char* tag, const char* format, ...)
#else
static inline void esp_log_write(esp_log_level_t level, const char* tag, const char* format, ...)
    __attribute__((format(printf, 3, 4)));
static inline void esp_log_write(esp_log_level_t level, const char* tag, const char* format, ...)
#endif
{
    const char* label = (level == ESP_LOG_ERROR) ? "[ERR] "
                      : (level == ESP_LOG_WARN)  ? "[WARN]" : "[INFO]";
    char message[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    vprintf_like_t function = esp_log_output_function;
    esp_log_emit(function ? function : esp_log_stderr_vprintf, "%s %s: %s\n", label, tag, message);
}

#define ESP_LOGI(tag, fmt, ...) esp_log_write(ESP_LOG_INFO,  tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) esp_log_write(ESP_LOG_WARN,  tag, fmt, ##__VA_ARGS__)
#define ESP_LOGE(tag, fmt, ...) esp_log_write(ESP_LOG_ERROR, tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)

// SILS addition: esp_log_level_set (old firmware uses esp_log_level_t / esp_log_level_set).
static inline void esp_log_level_set(const char* tag, esp_log_level_t level) { (void)tag; (void)level; }
