/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file driver/gpio.h
 * @brief Host stub for the ESP-IDF GPIO driver
 *        ESP-IDF GPIO ドライバのホスト用スタブ
 *
 * Provides the GPIO API surface (pin enums, config struct, set/get level,
 * ISR service) the firmware references so it compiles and links on a PC.
 * Function bodies are inert: they return ESP_OK and zero-fill outputs.
 *
 * 本体ファームが参照する GPIO API 一式（ピン列挙・設定構造体・レベル
 * 入出力・ISR サービス）を提供し、PC 上でコンパイル・リンクできるように
 * する。関数本体は無動作で、ESP_OK を返し出力をゼロ埋めする。
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* GPIO pin numbers                                                          */
/* GPIO ピン番号                                                            */
/* ------------------------------------------------------------------------- */
typedef enum {
    GPIO_NUM_NC = -1,   /*!< Use to signal not connected to S3 GPIO / 未接続 */
    GPIO_NUM_0  = 0,
    GPIO_NUM_1  = 1,
    GPIO_NUM_2  = 2,
    GPIO_NUM_3  = 3,
    GPIO_NUM_4  = 4,
    GPIO_NUM_5  = 5,
    GPIO_NUM_6  = 6,
    GPIO_NUM_7  = 7,
    GPIO_NUM_8  = 8,
    GPIO_NUM_9  = 9,
    GPIO_NUM_10 = 10,
    GPIO_NUM_11 = 11,
    GPIO_NUM_12 = 12,
    GPIO_NUM_13 = 13,
    GPIO_NUM_14 = 14,
    GPIO_NUM_15 = 15,
    GPIO_NUM_16 = 16,
    GPIO_NUM_17 = 17,
    GPIO_NUM_18 = 18,
    GPIO_NUM_19 = 19,
    GPIO_NUM_20 = 20,
    GPIO_NUM_21 = 21,
    GPIO_NUM_22 = 22,
    GPIO_NUM_23 = 23,
    GPIO_NUM_24 = 24,
    GPIO_NUM_25 = 25,
    GPIO_NUM_26 = 26,
    GPIO_NUM_27 = 27,
    GPIO_NUM_28 = 28,
    GPIO_NUM_29 = 29,
    GPIO_NUM_30 = 30,
    GPIO_NUM_31 = 31,
    GPIO_NUM_32 = 32,
    GPIO_NUM_33 = 33,
    GPIO_NUM_34 = 34,
    GPIO_NUM_35 = 35,
    GPIO_NUM_36 = 36,
    GPIO_NUM_37 = 37,
    GPIO_NUM_38 = 38,
    GPIO_NUM_39 = 39,
    GPIO_NUM_40 = 40,
    GPIO_NUM_41 = 41,
    GPIO_NUM_42 = 42,
    GPIO_NUM_43 = 43,
    GPIO_NUM_44 = 44,
    GPIO_NUM_45 = 45,
    GPIO_NUM_46 = 46,
    GPIO_NUM_47 = 47,
    GPIO_NUM_48 = 48,
    GPIO_NUM_MAX = 49,
} gpio_num_t;

/* Validity check macro (ESP-IDF provides this on real hardware)             */
/* 有効ピン判定マクロ（実機 ESP-IDF が提供）                                */
#define GPIO_IS_VALID_GPIO(gpio_num) \
    ((gpio_num) >= 0 && (gpio_num) < GPIO_NUM_MAX)
#define GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) GPIO_IS_VALID_GPIO(gpio_num)

/* ------------------------------------------------------------------------- */
/* GPIO mode                                                                 */
/* GPIO モード                                                              */
/* ------------------------------------------------------------------------- */
typedef enum {
    GPIO_MODE_DISABLE       = 0,                     /*!< disable input/output / 入出力無効 */
    GPIO_MODE_INPUT         = (1 << 0),              /*!< input only / 入力のみ */
    GPIO_MODE_OUTPUT        = (1 << 1),              /*!< output only / 出力のみ */
    GPIO_MODE_OUTPUT_OD     = ((1 << 1) | (1 << 2)), /*!< output open-drain / オープンドレイン出力 */
    GPIO_MODE_INPUT_OUTPUT_OD = ((1 << 0) | (1 << 1) | (1 << 2)), /*!< in/out open-drain */
    GPIO_MODE_INPUT_OUTPUT  = ((1 << 0) | (1 << 1)), /*!< input and output / 入出力 */
} gpio_mode_t;

/* ------------------------------------------------------------------------- */
/* GPIO pull mode                                                            */
/* GPIO プルモード                                                          */
/* ------------------------------------------------------------------------- */
typedef enum {
    GPIO_PULLUP_ONLY,       /*!< pad pull up only / プルアップのみ */
    GPIO_PULLDOWN_ONLY,     /*!< pad pull down only / プルダウンのみ */
    GPIO_PULLUP_PULLDOWN,   /*!< pad pull up + down / 両方 */
    GPIO_FLOATING,          /*!< pad floating / フローティング */
} gpio_pull_mode_t;

/* Pull-up enable/disable for config struct                                  */
/* 設定構造体用のプルアップ有効/無効                                        */
typedef enum {
    GPIO_PULLUP_DISABLE = 0x0, /*!< disable pull-up / プルアップ無効 */
    GPIO_PULLUP_ENABLE  = 0x1, /*!< enable pull-up / プルアップ有効 */
} gpio_pullup_t;

/* Pull-down enable/disable for config struct                                */
/* 設定構造体用のプルダウン有効/無効                                        */
typedef enum {
    GPIO_PULLDOWN_DISABLE = 0x0, /*!< disable pull-down / プルダウン無効 */
    GPIO_PULLDOWN_ENABLE  = 0x1, /*!< enable pull-down / プルダウン有効 */
} gpio_pulldown_t;

/* ------------------------------------------------------------------------- */
/* GPIO interrupt type                                                       */
/* GPIO 割り込みタイプ                                                      */
/* ------------------------------------------------------------------------- */
typedef enum {
    GPIO_INTR_DISABLE    = 0, /*!< disable interrupt / 割り込み無効 */
    GPIO_INTR_POSEDGE    = 1, /*!< rising edge / 立ち上がりエッジ */
    GPIO_INTR_NEGEDGE    = 2, /*!< falling edge / 立ち下がりエッジ */
    GPIO_INTR_ANYEDGE    = 3, /*!< any edge / 両エッジ */
    GPIO_INTR_LOW_LEVEL  = 4, /*!< input low level / 入力 Low レベル */
    GPIO_INTR_HIGH_LEVEL = 5, /*!< input high level / 入力 High レベル */
    GPIO_INTR_MAX,
} gpio_int_type_t;

/* GPIO drive capability (referenced by some HALs)                           */
/* GPIO ドライブ能力（一部 HAL が参照）                                     */
typedef enum {
    GPIO_DRIVE_CAP_0       = 0, /*!< weakest / 最弱 */
    GPIO_DRIVE_CAP_1       = 1,
    GPIO_DRIVE_CAP_2       = 2, /*!< default / 既定 */
    GPIO_DRIVE_CAP_DEFAULT = 2,
    GPIO_DRIVE_CAP_3       = 3, /*!< strongest / 最強 */
    GPIO_DRIVE_CAP_MAX,
} gpio_drive_cap_t;

/* ------------------------------------------------------------------------- */
/* GPIO configuration struct                                                 */
/* GPIO 設定構造体                                                          */
/*                                                                           */
/* Field order mirrors ESP-IDF so C++ designated initializers compile        */
/* (.pin_bit_mask / .mode / .pull_up_en / .pull_down_en / .intr_type).        */
/* フィールド順は ESP-IDF に一致させ、C++ 指定初期化子が通るようにする。      */
/* ------------------------------------------------------------------------- */
typedef struct {
    uint64_t        pin_bit_mask; /*!< GPIO pin set: bit N = GPIO N / 対象ピンのビットマスク */
    gpio_mode_t     mode;         /*!< GPIO mode / モード */
    gpio_pullup_t   pull_up_en;   /*!< pull-up enable / プルアップ有効化 */
    gpio_pulldown_t pull_down_en; /*!< pull-down enable / プルダウン有効化 */
    gpio_int_type_t intr_type;    /*!< interrupt type / 割り込みタイプ */
} gpio_config_t;

/* ISR handler signature                                                     */
/* ISR ハンドラのシグネチャ                                                 */
typedef void (*gpio_isr_t)(void* arg);
typedef void* gpio_isr_handle_t;

/* ------------------------------------------------------------------------- */
/* API (inert host stubs)                                                    */
/* API（無動作のホストスタブ）                                              */
/* ------------------------------------------------------------------------- */

/* Configure GPIO pins from a config struct / 設定構造体から GPIO を構成 */
static inline esp_err_t gpio_config(const gpio_config_t* pGPIOConfig)
{
    (void)pGPIOConfig;
    return ESP_OK;
}

/* Reset a pin to its default (disconnected, input) state / ピンを既定状態へ */
static inline esp_err_t gpio_reset_pin(gpio_num_t gpio_num)
{
    (void)gpio_num;
    return ESP_OK;
}

/* Set the direction/mode of a pin / ピンの方向（モード）を設定 */
static inline esp_err_t gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode)
{
    (void)gpio_num;
    (void)mode;
    return ESP_OK;
}

/* Set output level (0 or 1) / 出力レベル（0 または 1）を設定 */
static inline esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level)
{
    (void)gpio_num;
    (void)level;
    return ESP_OK;
}

/* Read input level. Host stub returns 1 (idle high / button released).      */
/* 入力レベルを読む。ホストでは 1（アイドル High／ボタン非押下）を返す。     */
static inline int gpio_get_level(gpio_num_t gpio_num)
{
    (void)gpio_num;
    return 1;
}

/* Set the pull mode of a pin / ピンのプルモードを設定 */
static inline esp_err_t gpio_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull)
{
    (void)gpio_num;
    (void)pull;
    return ESP_OK;
}

/* Set the drive capability of a pin / ピンのドライブ能力を設定 */
static inline esp_err_t gpio_set_drive_capability(gpio_num_t gpio_num, gpio_drive_cap_t strength)
{
    (void)gpio_num;
    (void)strength;
    return ESP_OK;
}

/* Set/override the interrupt trigger type / 割り込みトリガタイプを設定 */
static inline esp_err_t gpio_set_intr_type(gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    (void)gpio_num;
    (void)intr_type;
    return ESP_OK;
}

/* Enable interrupts on a pin / ピンの割り込みを有効化 */
static inline esp_err_t gpio_intr_enable(gpio_num_t gpio_num)
{
    (void)gpio_num;
    return ESP_OK;
}

/* Disable interrupts on a pin / ピンの割り込みを無効化 */
static inline esp_err_t gpio_intr_disable(gpio_num_t gpio_num)
{
    (void)gpio_num;
    return ESP_OK;
}

/* Install the per-pin ISR dispatch service / ピン単位 ISR ディスパッチを導入 */
static inline esp_err_t gpio_install_isr_service(int intr_alloc_flags)
{
    (void)intr_alloc_flags;
    return ESP_OK;
}

/* Uninstall the ISR service / ISR サービスを解除 */
static inline void gpio_uninstall_isr_service(void)
{
}

/* Register a per-pin ISR handler / ピン単位の ISR ハンドラを登録 */
static inline esp_err_t gpio_isr_handler_add(gpio_num_t gpio_num, gpio_isr_t isr_handler, void* args)
{
    (void)gpio_num;
    (void)isr_handler;
    (void)args;
    return ESP_OK;
}

/* Remove a per-pin ISR handler / ピン単位の ISR ハンドラを解除 */
static inline esp_err_t gpio_isr_handler_remove(gpio_num_t gpio_num)
{
    (void)gpio_num;
    return ESP_OK;
}

/* Configure pull-up resistor / プルアップ抵抗を構成 */
static inline esp_err_t gpio_pullup_en(gpio_num_t gpio_num)
{
    (void)gpio_num;
    return ESP_OK;
}

static inline esp_err_t gpio_pullup_dis(gpio_num_t gpio_num)
{
    (void)gpio_num;
    return ESP_OK;
}

/* Configure pull-down resistor / プルダウン抵抗を構成 */
static inline esp_err_t gpio_pulldown_en(gpio_num_t gpio_num)
{
    (void)gpio_num;
    return ESP_OK;
}

static inline esp_err_t gpio_pulldown_dis(gpio_num_t gpio_num)
{
    (void)gpio_num;
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif
