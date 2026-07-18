/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file nvs.h
 * @brief Host stub for ESP-IDF NVS — an in-memory key-value store
 *        ESP-IDF NVS のホスト用スタブ — インメモリのキー・バリュー保存
 *
 * The firmware persists parameters / calibration in NVS. On the SIL there
 * is no flash, so NVS is backed by an in-memory map that lives for the run.
 * A fresh run starts empty, so params load their defaults (the SIL passes
 * tuning via --params / topics, not flash).
 *
 * 本体ファームはパラメータ・キャリブを NVS に永続化する。SIL には
 * フラッシュが無いので、NVS は実行中だけ生きるインメモリの map で代用する。
 * 起動時は空なので各パラメータはデフォルトを読む（SIL は調整値を
 * フラッシュでなく --params / トピックで渡す）。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Opaque handle (an id into the in-memory store)
// 不透明ハンドル（インメモリ保存への id）
typedef uint32_t nvs_handle_t;

// Value type tag (mirrors ESP-IDF nvs_type_t); the SIL shim only needs the
// type to exist for nvs_find_key()'s signature, not its enumerators.
// 値型タグ（ESP-IDF nvs_type_t 相当）。SIL シムは nvs_find_key() のシグネチャに
// 型が存在すればよく、列挙子までは使わない。
typedef uint8_t nvs_type_t;

// Open modes (mirror ESP-IDF)
// オープンモード（ESP-IDF に合わせる）
typedef enum {
    NVS_READONLY = 0,
    NVS_READWRITE = 1,
} nvs_open_mode_t;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t nvs_open(const char* name, nvs_open_mode_t open_mode, nvs_handle_t* out_handle);
void      nvs_close(nvs_handle_t handle);
esp_err_t nvs_commit(nvs_handle_t handle);
esp_err_t nvs_erase_all(nvs_handle_t handle);
esp_err_t nvs_erase_key(nvs_handle_t handle, const char* key);

// Existence check (mirrors ESP-IDF nvs_find_key). Needed by sf::params::has_saved().
// 存在確認（ESP-IDF nvs_find_key 相当）。sf::params::has_saved() が使用する。
esp_err_t nvs_find_key(nvs_handle_t handle, const char* key, nvs_type_t* out_type);

esp_err_t nvs_set_u8(nvs_handle_t handle, const char* key, uint8_t value);
esp_err_t nvs_set_u32(nvs_handle_t handle, const char* key, uint32_t value);
esp_err_t nvs_set_i32(nvs_handle_t handle, const char* key, int32_t value);
esp_err_t nvs_set_blob(nvs_handle_t handle, const char* key, const void* value, size_t length);

esp_err_t nvs_get_u8(nvs_handle_t handle, const char* key, uint8_t* out_value);
esp_err_t nvs_get_u32(nvs_handle_t handle, const char* key, uint32_t* out_value);
esp_err_t nvs_get_i32(nvs_handle_t handle, const char* key, int32_t* out_value);
esp_err_t nvs_get_blob(nvs_handle_t handle, const char* key, void* out_value, size_t* length);

#ifdef __cplusplus
}  // extern "C"
#endif

// --- SIL additions: string get/set (old firmware CLI param store) ----------
static inline esp_err_t nvs_set_str(nvs_handle_t h, const char* key, const char* value)
{ (void)h; (void)key; (void)value; return ESP_OK; }
static inline esp_err_t nvs_get_str(nvs_handle_t h, const char* key, char* out, size_t* len)
{ (void)h; (void)key; if (out && len && *len) out[0] = 0; return ESP_ERR_NVS_NOT_FOUND; }
