/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — ESP-IDF host platform).
 */

/**
 * @file driver/i2c.h
 * @brief Host stub for the legacy ESP-IDF I2C driver
 *        レガシー ESP-IDF I2C ドライバのホスト用スタブ
 *
 * The StampFly firmware uses the new bus/device driver (driver/i2c_master.h)
 * everywhere, but a few legacy/vendor files still pull in this header for the
 * port type and port constants. This stub provides the minimal legacy surface
 * (i2c_port_t, I2C_NUM_*, master read/write modes) and reuses the shared
 * I2C_NUM_* / i2c_master_* definitions from i2c_master.h to avoid redefinition.
 *
 * 本体ファームは新バス/デバイスドライバ（driver/i2c_master.h）を使うが、
 * 一部のレガシー/ベンダファイルがポート型・ポート定数のために本ヘッダを
 * 取り込む。本スタブは最小限のレガシー面（i2c_port_t, I2C_NUM_*,
 * マスタ読書きモード）を提供し、再定義を避けるため I2C_NUM_* /
 * i2c_master_* の共有定義を i2c_master.h から再利用する。
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

// Reuse shared port constants and the new master API.
// 共有ポート定数と新マスタ API を再利用する。
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Legacy port type
 * レガシーポート型
 *
 * In the legacy driver the port is an enum (i2c_port_t). I2C_NUM_0/1 are
 * already provided as macros by i2c_master.h, so they work as enum values too.
 * レガシードライバではポートは列挙型（i2c_port_t）。I2C_NUM_0/1 は
 * i2c_master.h でマクロ提供済みのため、そのまま値として利用できる。
 * ---------------------------------------------------------------------- */
typedef int i2c_port_t;

/* -------------------------------------------------------------------------
 * Legacy I2C mode
 * レガシー I2C 動作モード
 * ---------------------------------------------------------------------- */
typedef enum {
    I2C_MODE_SLAVE = 0,   // I2C slave mode  / スレーブモード
    I2C_MODE_MASTER,      // I2C master mode / マスタモード
    I2C_MODE_MAX,         // Sentinel        / 番兵
} i2c_mode_t;

/* -------------------------------------------------------------------------
 * Legacy read/write direction (used in command links)
 * レガシー読書き方向（コマンドリンクで使用）
 * ---------------------------------------------------------------------- */
typedef enum {
    I2C_MASTER_WRITE = 0,  // Master writes / マスタ書込み
    I2C_MASTER_READ  = 1,  // Master reads  / マスタ読出し
} i2c_rw_t;

/* -------------------------------------------------------------------------
 * Legacy ACK handling on master reads
 * マスタ読出し時の ACK 処理
 * ---------------------------------------------------------------------- */
typedef enum {
    I2C_MASTER_ACK      = 0x0,  // Send ACK after a byte   / バイト後にACK
    I2C_MASTER_NACK     = 0x1,  // Send NACK after a byte  / バイト後にNACK
    I2C_MASTER_LAST_NACK = 0x2, // NACK only on last byte  / 最終バイトのみNACK
    I2C_MASTER_ACK_MAX,         // Sentinel                / 番兵
} i2c_ack_type_t;

#ifdef __cplusplus
}
#endif
