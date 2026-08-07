/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — ESP-IDF host platform).
 */

/**
 * @file esp_event.h
 * @brief Host shim for ESP-IDF event loop library (esp_event)
 *        ESP-IDF イベントループライブラリ (esp_event) のホスト用シム
 *
 * Provides the esp_event API surface the firmware references so it compiles
 * and links unmodified on a PC. Registration is inert: handlers are accepted
 * but never dispatched (no WiFi/IP events occur on the host).
 *
 * 本体ファームが参照する esp_event API 一式を提供し、無改変で PC 上に
 * コンパイル・リンクできるようにする。登録は無効動作（ハンドラは受理する
 * が、ホスト上では WiFi/IP イベントが発生しないため呼び出されない）。
 */

#pragma once

#include "esp_err.h"

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// Event base identifier (real ESP-IDF: pointer to a const string token)
// イベントベース識別子（実 ESP-IDF: const 文字列トークンへのポインタ）
typedef const char* esp_event_base_t;

// Wildcard event id: register for every event id under a base
// ワイルドカードイベントID: あるベース配下の全イベントを登録対象にする
#define ESP_EVENT_ANY_ID (-1)

// Wildcard event base: register for every base
// ワイルドカードイベントベース: 全ベースを登録対象にする
#define ESP_EVENT_ANY_BASE NULL

// Bitmask hints for posting from interrupt context (no-op on host)
// 割り込みコンテキストからの post 用ビットマスクヒント（ホストでは無効）
#define ESP_EVENT_POST_FROM_ISR 1
#define ESP_EVENT_POST_FROM_IRAM_ISR 2

// Event handler callback signature
// イベントハンドラのコールバック型
typedef void (*esp_event_handler_t)(void* event_handler_arg,
                                    esp_event_base_t event_base,
                                    int32_t event_id,
                                    void* event_data);

// Opaque handle returned by instance-based registration
// インスタンス登録で返される不透明ハンドル
typedef struct esp_event_handler_instance_s* esp_event_handler_instance_t;

// Opaque user event loop handle (custom loops)
// ユーザー定義イベントループの不透明ハンドル
typedef struct esp_event_loop_s* esp_event_loop_handle_t;

// Create the default system event loop (inert on host)
// デフォルトのシステムイベントループを生成（ホストでは無効動作）
static inline esp_err_t esp_event_loop_create_default(void)
{
    return ESP_OK;
}

// Delete the default system event loop
// デフォルトのシステムイベントループを削除
static inline esp_err_t esp_event_loop_delete_default(void)
{
    return ESP_OK;
}

// Register an event handler on the default loop (legacy API)
// デフォルトループにイベントハンドラを登録（旧 API）
static inline esp_err_t esp_event_handler_register(esp_event_base_t event_base,
                                                   int32_t event_id,
                                                   esp_event_handler_t event_handler,
                                                   void* event_handler_arg)
{
    (void)event_base;
    (void)event_id;
    (void)event_handler;
    (void)event_handler_arg;
    return ESP_OK;
}

// Unregister an event handler from the default loop (legacy API)
// デフォルトループからイベントハンドラを登録解除（旧 API）
static inline esp_err_t esp_event_handler_unregister(esp_event_base_t event_base,
                                                     int32_t event_id,
                                                     esp_event_handler_t event_handler)
{
    (void)event_base;
    (void)event_id;
    (void)event_handler;
    return ESP_OK;
}

// Register an event handler instance on the default loop
// デフォルトループにイベントハンドラインスタンスを登録
static inline esp_err_t esp_event_handler_instance_register(
    esp_event_base_t event_base,
    int32_t event_id,
    esp_event_handler_t event_handler,
    void* event_handler_arg,
    esp_event_handler_instance_t* instance)
{
    (void)event_base;
    (void)event_id;
    (void)event_handler;
    (void)event_handler_arg;
    // Zero-fill the output handle (real ESP-IDF returns a valid token)
    // 出力ハンドルをゼロ埋め（実 ESP-IDF は有効トークンを返す）
    if (instance != NULL) {
        *instance = NULL;
    }
    return ESP_OK;
}

// Unregister an event handler instance from the default loop
// デフォルトループからイベントハンドラインスタンスを登録解除
static inline esp_err_t esp_event_handler_instance_unregister(
    esp_event_base_t event_base,
    int32_t event_id,
    esp_event_handler_instance_t instance)
{
    (void)event_base;
    (void)event_id;
    (void)instance;
    return ESP_OK;
}

// Post an event to the default loop (inert on host)
// デフォルトループへイベントを post（ホストでは無効動作）
static inline esp_err_t esp_event_post(esp_event_base_t event_base,
                                       int32_t event_id,
                                       const void* event_data,
                                       size_t event_data_size,
                                       uint32_t ticks_to_wait)
{
    (void)event_base;
    (void)event_id;
    (void)event_data;
    (void)event_data_size;
    (void)ticks_to_wait;
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif
