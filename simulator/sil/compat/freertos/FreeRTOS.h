/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file FreeRTOS.h
 * @brief Host stub for FreeRTOS base types and macros (passive layer)
 *        FreeRTOS の基本型・マクロのホスト用スタブ（受動レイヤ）
 *
 * This is the PASSIVE part of the RTOS shim: base types, tick conversion,
 * and the data-structure primitives (semaphore/queue). The ACTIVE part —
 * the cooperative scheduler, tasks, and notifications — is added in P1.1
 * (simulator/sil/rtos/).
 *
 * これは RTOS シムの受動部分: 基本型・tick 変換・データ構造プリミティブ
 * （セマフォ/キュー）。能動部分（協調スケジューラ・タスク・通知）は
 * P1.1（simulator/sil/rtos/）で足す。
 *
 * SIL convention: 1 tick = 1 ms (configTICK_RATE_HZ = 1000).
 * SIL の取り決め: 1 tick = 1 ms（configTICK_RATE_HZ = 1000）。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

// BIT(n)/BIT0..BIT31 — widely available on ESP-IDF; re-export here so any file
// including FreeRTOS.h (event groups, register masks) gets them.
// BIT マクロを FreeRTOS.h から再エクスポート（ESP-IDF と同様に広く使えるように）。
#include "esp_bit_defs.h"
#include "esp_err.h"   // esp_err_t is ubiquitous on ESP-IDF; expose it broadly

typedef int32_t  BaseType_t;
typedef uint32_t UBaseType_t;
typedef uint32_t TickType_t;

#define pdTRUE   ((BaseType_t)1)
#define pdFALSE  ((BaseType_t)0)
#define pdPASS   pdTRUE
#define pdFAIL   pdFALSE

#define portMAX_DELAY        ((TickType_t)0xffffffffUL)
#define portTICK_PERIOD_MS   ((TickType_t)1)
#define configTICK_RATE_HZ   1000

// 1 tick = 1 ms on the SIL virtual clock.
// SIL の仮想時計では 1 tick = 1 ms。
#define pdMS_TO_TICKS(ms)    ((TickType_t)(ms))

// Pull in the passive data-structure primitives so any C++ file including
// FreeRTOS.h gets the semaphore/queue surface (as on ESP-IDF in practice).
// These are std::-backed (C++ only); the C sensor drivers that include
// FreeRTOS.h only need the base types + task API, so guard them out for C.
// FreeRTOS.h を include した C++ ファイルがセマフォ/キュー一式を得られるよう
// 取り込む（std:: 実装ゆえ C++ 専用）。C のセンサドライバは基本型＋タスク API
// だけ要るので C ではガードアウトする。
#ifdef __cplusplus
#include "freertos/semphr.h"
#include "freertos/queue.h"
#endif

// SIL: old-firmware files use xTaskGetTickCount etc. via FreeRTOS.h alone.
#include "freertos/task.h"
