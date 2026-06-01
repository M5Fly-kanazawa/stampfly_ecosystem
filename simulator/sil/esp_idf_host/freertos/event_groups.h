/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file freertos/event_groups.h
 * @brief Host shim for FreeRTOS event groups (bitmask-backed cooperative stub).
 *        FreeRTOS イベントグループのホスト用シム（ビットマスク方式の協調スタブ）.
 *
 * An event group is a set of bits that tasks can set, clear and wait on. On the
 * SIL host (single cooperative token, no preemption) a plain integer bitmask is
 * a faithful enough model: set/clear/get manipulate the mask, and wait returns
 * the current bits immediately (a blocking wait would deadlock the single-token
 * scheduler). This keeps the firmware's WiFi-got-IP / system-event handshakes
 * compiling and linking; SIL flows drive the bits explicitly when needed.
 *
 * イベントグループはタスクがセット/クリア/待機できるビット集合。SIL ホスト
 * （協調トークン1個・プリエンプションなし）では単純な整数ビットマスクで十分に
 * 忠実: set/clear/get はマスクを操作し、wait は現在のビットを即時返す
 * （ブロッキング待機は単一トークンのスケジューラをデッドロックさせるため）。
 * これによりファームの WiFi-got-IP / システムイベントのハンドシェイクが
 * コンパイル・リンク可能になる。必要なら SIL フローが明示的にビットを駆動する。
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Base FreeRTOS types (BaseType_t, TickType_t, portMAX_DELAY, pdTRUE/pdFALSE).
// The real ESP-IDF event_groups.h includes FreeRTOS.h for the same reason.
// FreeRTOS の基本型（BaseType_t, TickType_t, portMAX_DELAY, pdTRUE/pdFALSE）。
// 本物の ESP-IDF event_groups.h も同じ理由で FreeRTOS.h を include する。
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

// Bit field type. ESP-IDF maps EventBits_t to TickType_t (32-bit), of which the
// low 24 bits are usable as event bits.
// ビットフィールド型。ESP-IDF では EventBits_t は TickType_t（32bit）の別名で、
// 下位 24bit がイベントビットとして使える。
typedef TickType_t EventBits_t;

// Opaque handle: pointer to a forward-declared event-group struct.
// 不透明ハンドル: 前方宣言したイベントグループ構造体へのポインタ。
typedef struct EventGroupDef_t* EventGroupHandle_t;

// Internal storage for the host stub: just the current bit mask.
// ホストスタブの内部記憶: 現在のビットマスクだけ。
struct EventGroupDef_t {
    EventBits_t bits;
};

// Create a new event group. Returns NULL on allocation failure (as on ESP-IDF).
// 新しいイベントグループを作る。確保失敗時は NULL を返す（ESP-IDF と同じ）。
static inline EventGroupHandle_t xEventGroupCreate(void)
{
    EventGroupHandle_t group =
        (EventGroupHandle_t)calloc(1, sizeof(struct EventGroupDef_t));
    return group;  // calloc zero-fills, so bits start at 0
}

// Set the given bits and return the resulting value.
// 指定ビットをセットし、結果の値を返す。
static inline EventBits_t xEventGroupSetBits(EventGroupHandle_t group,
                                             EventBits_t bits_to_set)
{
    if (group == NULL) {
        return 0;
    }
    group->bits |= bits_to_set;
    return group->bits;
}

// Clear the given bits and return the value BEFORE clearing (FreeRTOS semantics).
// 指定ビットをクリアし、クリア "前" の値を返す（FreeRTOS の仕様）。
static inline EventBits_t xEventGroupClearBits(EventGroupHandle_t group,
                                               EventBits_t bits_to_clear)
{
    if (group == NULL) {
        return 0;
    }
    const EventBits_t before = group->bits;
    group->bits &= ~bits_to_clear;
    return before;
}

// Return the current bit value without modifying it.
// 現在のビット値を変更せずに返す。
static inline EventBits_t xEventGroupGetBits(EventGroupHandle_t group)
{
    return (group != NULL) ? group->bits : 0;
}

// Wait for one or all of the given bits.
//   clear_on_exit:  if pdTRUE, clear the waited bits before returning.
//   wait_for_all:   pdTRUE = wait for ALL bits, pdFALSE = wait for ANY.
//   ticks_to_wait:  timeout (ignored on the host — we never block).
// On the single-token SIL scheduler we return the current bits immediately
// rather than block, then honour clear_on_exit if the condition is met.
// 指定ビットの一部 / 全部を待つ。
//   clear_on_exit:  pdTRUE なら返す前に待機ビットをクリア。
//   wait_for_all:   pdTRUE = 全ビット待ち、pdFALSE = いずれか待ち。
//   ticks_to_wait:  タイムアウト（ホストでは無視 — ブロックしない）。
// 単一トークンの SIL スケジューラではブロックせず現在のビットを即時返し、
// 条件成立時に clear_on_exit を反映する。
static inline EventBits_t xEventGroupWaitBits(EventGroupHandle_t group,
                                              EventBits_t bits_to_wait_for,
                                              BaseType_t clear_on_exit,
                                              BaseType_t wait_for_all,
                                              TickType_t ticks_to_wait)
{
    (void)ticks_to_wait;
    if (group == NULL) {
        return 0;
    }
    const EventBits_t current = group->bits;
    const EventBits_t masked = current & bits_to_wait_for;

    const bool condition_met =
        (wait_for_all != pdFALSE) ? (masked == bits_to_wait_for)
                                  : (masked != 0);

    if (condition_met && clear_on_exit != pdFALSE) {
        group->bits &= ~bits_to_wait_for;
    }
    return current;
}

// Delete an event group and free its storage.
// イベントグループを削除し記憶領域を解放する。
static inline void vEventGroupDelete(EventGroupHandle_t group)
{
    free(group);
}

#ifdef __cplusplus
}
#endif
