/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL/test compatibility shim).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file FreeRTOS.h (host shim)
 * @brief Minimal FreeRTOS base types/macros for host (PC) builds
 *        ホスト(PC)ビルド用の最小FreeRTOS基本型・マクロ
 *
 * This shim lets the unmodified firmware headers (topic.hpp etc.) compile
 * on a PC. It is injected ahead of the real ESP-IDF headers via `-I compat`.
 * Firmware source is never edited — Code Identity is preserved.
 *
 * このシムは無改変のファームヘッダ（topic.hpp等）をPCでコンパイルさせる。
 * `-I compat` で実ESP-IDFヘッダより先に注入される。
 * ファームのソースは一切編集しない — Code Identity を保つ。
 */

#pragma once

#include <cstdint>

// Tick type and boolean return types
// Tick型と真偽値の戻り型
typedef uint32_t      TickType_t;
typedef int           BaseType_t;
typedef unsigned int  UBaseType_t;

// FreeRTOS boolean / wait constants
// FreeRTOS の真偽値・待機定数
#define pdTRUE        ((BaseType_t)1)
#define pdFALSE       ((BaseType_t)0)
#define pdPASS        pdTRUE
#define pdFAIL        pdFALSE
#define portMAX_DELAY ((TickType_t)0xffffffffUL)
