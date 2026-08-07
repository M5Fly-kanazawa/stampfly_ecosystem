/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file esp_err.h
 * @brief Host stub for ESP-IDF error codes
 *        ESP-IDF エラーコードのホスト用スタブ
 *
 * Provides the esp_err_t surface the firmware references so it compiles
 * unmodified on a PC. Values mirror ESP-IDF where it matters for logic.
 *
 * 本体ファームが参照する esp_err_t 一式を提供し、無改変で PC 上に
 * コンパイルできるようにする。ロジックに関わる値は ESP-IDF に合わせる。
 */

#pragma once

#include <stdint.h>

// Error code type (ESP-IDF uses a signed int)
// エラーコード型（ESP-IDF は符号付き int）
typedef int32_t esp_err_t;

#define ESP_OK                      0
#define ESP_FAIL                    (-1)

#define ESP_ERR_NO_MEM              0x101
#define ESP_ERR_INVALID_ARG         0x102
#define ESP_ERR_INVALID_STATE       0x103
#define ESP_ERR_INVALID_SIZE        0x104
#define ESP_ERR_NOT_FOUND           0x105
#define ESP_ERR_NOT_SUPPORTED       0x106
#define ESP_ERR_TIMEOUT             0x107
#define ESP_ERR_INVALID_RESPONSE    0x108
#define ESP_ERR_INVALID_CRC         0x109
#define ESP_ERR_INVALID_VERSION     0x10a
#define ESP_ERR_INVALID_MAC         0x10b
#define ESP_ERR_NOT_FINISHED        0x10c

// NVS-specific codes referenced by params / calibration
// params・calibration が参照する NVS 固有コード
#define ESP_ERR_NVS_BASE            0x1100
#define ESP_ERR_NVS_NOT_FOUND       (ESP_ERR_NVS_BASE + 0x07)
#define ESP_ERR_NVS_NO_FREE_PAGES   (ESP_ERR_NVS_BASE + 0x0d)
#define ESP_ERR_NVS_NEW_VERSION_FOUND (ESP_ERR_NVS_BASE + 0x10)

// Translate an error code to a name (debug only)
// エラーコードを名前に変換する（デバッグ用）
static inline const char* esp_err_to_name(esp_err_t err)
{
    return err == ESP_OK ? "ESP_OK" : "ESP_ERR";
}

// On host, a failed check aborts loudly (mirrors ESP_ERROR_CHECK intent)
// ホストでは失敗時に大きく abort する（ESP_ERROR_CHECK の意図を踏襲）
#include <stdio.h>
#include <stdlib.h>
#define ESP_ERROR_CHECK(x)                                                   \
    do {                                                                     \
        esp_err_t __err_rc = (x);                                           \
        if (__err_rc != ESP_OK) {                                            \
            fprintf(stderr, "ESP_ERROR_CHECK failed: %s (0x%x) at %s:%d\n",  \
                    #x, (int)__err_rc, __FILE__, __LINE__);                  \
            abort();                                                         \
        }                                                                    \
    } while (0)
