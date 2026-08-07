/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — ESP-IDF host platform).
 */

/**
 * @file esp_spiffs.h
 * @brief Host stub for ESP-IDF's SPIFFS VFS integration
 *        ESP-IDF の SPIFFS VFS 連携のホスト用スタブ
 *
 * Inert host stubs so the logger/blackbox compile and "mount" harmlessly.
 * Registering returns ESP_OK; esp_spiffs_info() reports zero usage. The
 * firmware can still fopen/fwrite under the configured base_path because the
 * host filesystem is real, so the logger runs without touching real flash.
 *
 * ロガー/ブラックボックスが無害に「マウント」してコンパイルできるよう、
 * ホスト用の不活性スタブを提供する。register は ESP_OK を返し、
 * esp_spiffs_info() は使用量ゼロを報告する。base_path 配下の fopen/fwrite は
 * ホストの実ファイルシステムで動作するため、実フラッシュに触れずロガーが動く。
 */

#pragma once

#include "esp_err.h"

#include <cstddef>
#include <cstring>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Configuration passed to esp_vfs_spiffs_register().
 * Field set and types mirror ESP-IDF v5.
 *
 * esp_vfs_spiffs_register() に渡す設定。フィールド構成と型は ESP-IDF v5 準拠。
 */
typedef struct {
    const char* base_path;            // VFS mount point / VFS マウントポイント
    const char* partition_label;      // Partition label / パーティションラベル
    size_t      max_files;            // Max open files / 同時オープン最大数
    bool        format_if_mount_failed;  // Format on mount failure / 失敗時フォーマット
} esp_vfs_spiffs_conf_t;

/*
 * Register (mount) a SPIFFS partition. Host stub: always succeeds.
 * SPIFFS パーティションを登録（マウント）する。ホストスタブ：常に成功。
 */
static inline esp_err_t esp_vfs_spiffs_register(const esp_vfs_spiffs_conf_t* conf)
{
    (void)conf;
    return ESP_OK;
}

/*
 * Unregister (unmount) a SPIFFS partition. Host stub: always succeeds.
 * SPIFFS パーティションを登録解除（アンマウント）する。ホストスタブ：常に成功。
 */
static inline esp_err_t esp_vfs_spiffs_unregister(const char* partition_label)
{
    (void)partition_label;
    return ESP_OK;
}

/*
 * Report total/used bytes of a SPIFFS partition. Host stub: reports zeros.
 * SPIFFS パーティションの総容量/使用量を報告する。ホストスタブ：ゼロを報告。
 */
static inline esp_err_t esp_spiffs_info(const char* partition_label,
                                        size_t* total_bytes,
                                        size_t* used_bytes)
{
    (void)partition_label;
    if (total_bytes != nullptr) {
        *total_bytes = 0;
    }
    if (used_bytes != nullptr) {
        *used_bytes = 0;
    }
    return ESP_OK;
}

/*
 * Whether a SPIFFS partition is mounted. Host stub: reports not mounted.
 * SPIFFS パーティションがマウント済みか。ホストスタブ：未マウントを報告。
 */
static inline bool esp_spiffs_mounted(const char* partition_label)
{
    (void)partition_label;
    return false;
}

/*
 * Format a SPIFFS partition. Host stub: always succeeds (no-op).
 * SPIFFS パーティションをフォーマットする。ホストスタブ：常に成功（何もしない）。
 */
static inline esp_err_t esp_spiffs_format(const char* partition_label)
{
    (void)partition_label;
    return ESP_OK;
}

/*
 * GC pass over a SPIFFS partition. Host stub: always succeeds (no-op).
 * SPIFFS パーティションの GC を実行する。ホストスタブ：常に成功（何もしない）。
 */
static inline esp_err_t esp_spiffs_gc(const char* partition_label, size_t size_to_gc)
{
    (void)partition_label;
    (void)size_to_gc;
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif
