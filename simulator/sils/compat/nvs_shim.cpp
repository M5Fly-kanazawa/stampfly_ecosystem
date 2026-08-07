/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file nvs_shim.cpp
 * @brief In-memory implementation of the NVS host stub
 *        NVS ホストスタブのインメモリ実装
 *
 * Keys are global for the run (param/calibration names are unique, so we
 * do not need per-namespace isolation for the SILS). A get on a missing key
 * returns ESP_ERR_NVS_NOT_FOUND, which makes the firmware fall back to the
 * compiled-in default — the desired SILS behaviour.
 *
 * キーは実行中グローバル（param/calibration 名は一意なので SILS では
 * namespace ごとの分離は不要）。存在しないキーの get は
 * ESP_ERR_NVS_NOT_FOUND を返し、本体ファームはコンパイル時デフォルトに
 * フォールバックする — これが SILS で望む挙動。
 */

#include "nvs.h"
#include "nvs_flash.h"

#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace {
// Single global store: key → raw bytes.
// 唯一のグローバル保存: キー → 生バイト列。
std::map<std::string, std::vector<uint8_t>> g_store;

// Store a typed value as raw bytes.
// 型付きの値を生バイト列として保存する。
template <typename T>
esp_err_t store_value(const char* key, T value)
{
    auto& bytes = g_store[key];
    bytes.resize(sizeof(T));
    std::memcpy(bytes.data(), &value, sizeof(T));
    return ESP_OK;
}

// Load a typed value; missing or size mismatch → NOT_FOUND.
// 型付きの値を読む; 不在やサイズ不一致は NOT_FOUND。
template <typename T>
esp_err_t load_value(const char* key, T* out)
{
    auto it = g_store.find(key);
    if (it == g_store.end() || it->second.size() != sizeof(T)) {
        return ESP_ERR_NVS_NOT_FOUND;
    }
    std::memcpy(out, it->second.data(), sizeof(T));
    return ESP_OK;
}
}  // namespace

extern "C" {

esp_err_t nvs_open(const char*, nvs_open_mode_t, nvs_handle_t* out_handle)
{
    if (out_handle) *out_handle = 1;  // a single shared store / 共有保存ひとつ
    return ESP_OK;
}

void      nvs_close(nvs_handle_t) {}
esp_err_t nvs_commit(nvs_handle_t) { return ESP_OK; }
esp_err_t nvs_erase_all(nvs_handle_t) { g_store.clear(); return ESP_OK; }
esp_err_t nvs_erase_key(nvs_handle_t, const char* key) { g_store.erase(key); return ESP_OK; }

// Existence check — used by sf::params::has_saved(). The shim has no per-value
// type tag, so out_type (if requested) is left untouched; callers pass nullptr.
// 存在確認 — sf::params::has_saved() が使用。シムは値ごとの型タグを持たないため
// out_type（要求時）はそのまま。呼び出し側は nullptr を渡す。
esp_err_t nvs_find_key(nvs_handle_t, const char* key, nvs_type_t*)
{
    return g_store.count(key) ? ESP_OK : ESP_ERR_NVS_NOT_FOUND;
}

esp_err_t nvs_set_u8(nvs_handle_t, const char* key, uint8_t value)   { return store_value(key, value); }
esp_err_t nvs_set_u32(nvs_handle_t, const char* key, uint32_t value) { return store_value(key, value); }
esp_err_t nvs_set_i32(nvs_handle_t, const char* key, int32_t value)  { return store_value(key, value); }

esp_err_t nvs_get_u8(nvs_handle_t, const char* key, uint8_t* out)   { return load_value(key, out); }
esp_err_t nvs_get_u32(nvs_handle_t, const char* key, uint32_t* out) { return load_value(key, out); }
esp_err_t nvs_get_i32(nvs_handle_t, const char* key, int32_t* out)  { return load_value(key, out); }

esp_err_t nvs_set_blob(nvs_handle_t, const char* key, const void* value, size_t length)
{
    auto& bytes = g_store[key];
    bytes.resize(length);
    std::memcpy(bytes.data(), value, length);
    return ESP_OK;
}

esp_err_t nvs_get_blob(nvs_handle_t, const char* key, void* out_value, size_t* length)
{
    auto it = g_store.find(key);
    if (it == g_store.end()) return ESP_ERR_NVS_NOT_FOUND;
    if (out_value && length && *length >= it->second.size()) {
        std::memcpy(out_value, it->second.data(), it->second.size());
    }
    if (length) *length = it->second.size();
    return ESP_OK;
}

esp_err_t nvs_flash_init(void)  { return ESP_OK; }
esp_err_t nvs_flash_erase(void) { g_store.clear(); return ESP_OK; }

}  // extern "C"
