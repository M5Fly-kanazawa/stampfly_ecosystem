/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file driver/i2c_master.h
 * @brief Host stub for the ESP-IDF I2C master (new "bus + device") driver
 *        ESP-IDF I2C マスタ（新「バス＋デバイス」）ドライバのホスト用スタブ
 *
 * Provides the i2c_master_* API surface (handles, configs, transfer functions)
 * so the real StampFly firmware (board bring-up + BMP280/BMM150/VL53L3CX/power
 * HAL drivers) compiles and links on a PC. Function bodies are inert: they
 * return ESP_OK and zero-fill any output buffers/handles.
 *
 * 本体ファーム（ボード初期化 + BMP280/BMM150/VL53L3CX/電源 HAL ドライバ）が
 * 参照する i2c_master_* 一式を提供し、PC 上で無改変にコンパイル・リンク
 * できるようにする。関数本体は無動作で、ESP_OK を返し出力をゼロ埋めする。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Port number type and ports
 * ポート番号型とポート
 *
 * Real ESP-IDF v5 models the port as a signed integer (i2c_port_num_t).
 * I2C_NUM_0/I2C_NUM_1 select the hardware controller.
 * 実 ESP-IDF v5 ではポートは符号付き整数（i2c_port_num_t）。
 * I2C_NUM_0/I2C_NUM_1 はハードウェアコントローラを選択する。
 * ---------------------------------------------------------------------- */
typedef int i2c_port_num_t;

#ifndef I2C_NUM_0
#define I2C_NUM_0   0      // First  I2C controller / 1番目のI2Cコントローラ
#endif
#ifndef I2C_NUM_1
#define I2C_NUM_1   1      // Second I2C controller / 2番目のI2Cコントローラ
#endif
#ifndef I2C_NUM_MAX
#define I2C_NUM_MAX 2      // Number of controllers / コントローラ数
#endif

/* -------------------------------------------------------------------------
 * Clock source (subset of soc_periph_i2c_clk_src_t)
 * クロックソース（soc_periph_i2c_clk_src_t の部分集合）
 * ---------------------------------------------------------------------- */
typedef enum {
    I2C_CLK_SRC_DEFAULT = 0,   // Driver picks a sensible default / 既定クロック
} i2c_clock_source_t;

/* -------------------------------------------------------------------------
 * Address bit length
 * アドレスビット長
 * ---------------------------------------------------------------------- */
typedef enum {
    I2C_ADDR_BIT_LEN_7  = 0,   // 7-bit  device address / 7ビットアドレス
    I2C_ADDR_BIT_LEN_10 = 1,   // 10-bit device address / 10ビットアドレス
} i2c_addr_bit_len_t;

/* -------------------------------------------------------------------------
 * Opaque handles (typedef'd pointers to forward-declared structs)
 * 不透明ハンドル（前方宣言構造体へのポインタ typedef）
 * ---------------------------------------------------------------------- */
typedef struct i2c_master_bus_t* i2c_master_bus_handle_t;
typedef struct i2c_master_dev_t* i2c_master_dev_handle_t;

/* -------------------------------------------------------------------------
 * Master bus configuration
 * マスタバス設定
 *
 * Field layout mirrors real ESP-IDF; firmware sets fields by name
 * (cfg.i2c_port, cfg.sda_io_num, ...), so names must match exactly.
 * フィールド配置は実 ESP-IDF に一致。ファームは名前で代入するため
 * （cfg.i2c_port, cfg.sda_io_num 等）、名前は完全一致が必要。
 * ---------------------------------------------------------------------- */
typedef struct {
    i2c_port_num_t     i2c_port;           // Controller port (-1 = auto) / コントローラポート
    int                sda_io_num;         // SDA GPIO number             / SDA GPIO
    int                scl_io_num;         // SCL GPIO number             / SCL GPIO
    i2c_clock_source_t clk_source;         // Clock source                / クロックソース
    uint8_t            glitch_ignore_cnt;  // Glitch filter cycles        / グリッチ除去サイクル
    int                intr_priority;      // Interrupt priority (0=auto) / 割り込み優先度
    size_t             trans_queue_depth;  // Async transfer queue depth  / 非同期キュー段数
    struct {
        uint32_t enable_internal_pullup : 1;  // Enable internal pull-ups / 内蔵プルアップ有効
        uint32_t allow_pd               : 1;  // Allow power-down backup   / 省電力時バックアップ許可
    } flags;                               // Bus flags / バスフラグ
} i2c_master_bus_config_t;

/* -------------------------------------------------------------------------
 * Master device configuration
 * マスタデバイス設定
 *
 * Firmware uses designated initializers in this exact order
 * (.dev_addr_length, .device_address, .scl_speed_hz); keep field order.
 * ファームは指定初期化子をこの順で使う
 * （.dev_addr_length, .device_address, .scl_speed_hz）ため順序を保持。
 * ---------------------------------------------------------------------- */
typedef struct {
    i2c_addr_bit_len_t dev_addr_length;        // Address bit length      / アドレスビット長
    uint16_t           device_address;         // 7/10-bit device address / デバイスアドレス
    uint32_t           scl_speed_hz;           // SCL clock [Hz]          / SCLクロック
    uint32_t           scl_wait_us;            // SCL wait timeout [us]   / SCL待ちタイムアウト
    struct {
        uint32_t disable_ack_check : 1;        // Skip ACK check          / ACK確認を省く
    } flags;                                   // Device flags / デバイスフラグ
} i2c_device_config_t;

/* -------------------------------------------------------------------------
 * Stub function bodies (inert)
 * スタブ関数本体（無動作）
 * ---------------------------------------------------------------------- */

// Create a new master bus; hand back a non-null sentinel handle.
// 新しいマスタバスを生成し、非NULLの番兵ハンドルを返す。
static inline esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t* bus_config,
                                           i2c_master_bus_handle_t* ret_bus_handle)
{
    (void)bus_config;
    if (ret_bus_handle) {
        // Non-null opaque cookie so callers' null checks pass.
        // 呼び出し側のNULLチェックを通すための非NULLな疑似ハンドル。
        *ret_bus_handle = (i2c_master_bus_handle_t)(uintptr_t)0x1;
    }
    return ESP_OK;
}

// Delete a master bus (no-op on host).
// マスタバスを削除（ホストでは何もしない）。
static inline esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t bus_handle)
{
    (void)bus_handle;
    return ESP_OK;
}

// Add a device to a bus; hand back a non-null sentinel handle.
// バスにデバイスを追加し、非NULLの番兵ハンドルを返す。
static inline esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus_handle,
                                                  const i2c_device_config_t* dev_config,
                                                  i2c_master_dev_handle_t* ret_handle)
{
    (void)bus_handle;
    (void)dev_config;
    if (ret_handle) {
        *ret_handle = (i2c_master_dev_handle_t)(uintptr_t)0x1;
    }
    return ESP_OK;
}

// Remove a device from a bus (no-op on host).
// バスからデバイスを削除（ホストでは何もしない）。
static inline esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t dev_handle)
{
    (void)dev_handle;
    return ESP_OK;
}

// Transmit bytes to a device (discarded on host).
// デバイスへバイト列を送信（ホストでは破棄）。
static inline esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev_handle,
                                            const uint8_t* write_buffer,
                                            size_t write_size,
                                            int xfer_timeout_ms)
{
    (void)dev_handle;
    (void)write_buffer;
    (void)write_size;
    (void)xfer_timeout_ms;
    return ESP_OK;
}

// Receive bytes from a device (zero-filled on host).
// デバイスからバイト列を受信（ホストではゼロ埋め）。
static inline esp_err_t i2c_master_receive(i2c_master_dev_handle_t dev_handle,
                                           uint8_t* read_buffer,
                                           size_t read_size,
                                           int xfer_timeout_ms)
{
    (void)dev_handle;
    (void)xfer_timeout_ms;
    if (read_buffer && read_size) {
        memset(read_buffer, 0, read_size);
    }
    return ESP_OK;
}

// Write-then-read in one transaction (read side zero-filled on host).
// 1トランザクションで書込→読出（読出側はホストでゼロ埋め）。
static inline esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t dev_handle,
                                                    const uint8_t* write_buffer,
                                                    size_t write_size,
                                                    uint8_t* read_buffer,
                                                    size_t read_size,
                                                    int xfer_timeout_ms)
{
    (void)dev_handle;
    (void)write_buffer;
    (void)write_size;
    (void)xfer_timeout_ms;
    if (read_buffer && read_size) {
        memset(read_buffer, 0, read_size);
    }
    return ESP_OK;
}

// Probe whether a device ACKs at an address (host: always "present").
// アドレスでデバイスがACKするか確認（ホストでは常に「存在」）。
static inline esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus_handle,
                                         uint16_t address,
                                         int xfer_timeout_ms)
{
    (void)bus_handle;
    (void)address;
    (void)xfer_timeout_ms;
    return ESP_OK;
}

// Reset a master bus (no-op on host).
// マスタバスをリセット（ホストでは何もしない）。
static inline esp_err_t i2c_master_bus_reset(i2c_master_bus_handle_t bus_handle)
{
    (void)bus_handle;
    return ESP_OK;
}

// Wait for all pending async transactions (host: nothing pending).
// 保留中の非同期処理を待つ（ホストでは保留なし）。
static inline esp_err_t i2c_master_bus_wait_all_done(i2c_master_bus_handle_t bus_handle,
                                                     int timeout_ms)
{
    (void)bus_handle;
    (void)timeout_ms;
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif
