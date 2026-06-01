/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file driver/ledc.h
 * @brief Host stub for the ESP-IDF LEDC (PWM) driver
 *        ESP-IDF LEDC（PWM）ドライバのホスト用スタブ
 *
 * Provides the ledc_* API surface (types, enums, configs, functions) so the
 * real StampFly firmware (motor + buzzer drivers) compiles and links on a PC.
 * Function bodies are inert: they return ESP_OK and zero-fill outputs.
 *
 * 本体ファーム（モータ・ブザードライバ）が参照する ledc_* 一式を提供し、
 * PC 上で無改変にコンパイル・リンクできるようにする。関数本体は無動作で、
 * ESP_OK を返し出力をゼロ埋めする。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Capability macros (referenced by firmware feature checks)
 * 機能ケイパビリティマクロ（ファームの機能チェックが参照）
 * ---------------------------------------------------------------------- */
#define LEDC_SUPPORTED            1
#define LEDC_SUPPORT_APB_CLOCK    1
#define LEDC_SUPPORT_XTAL_CLOCK   1
#define LEDC_SUPPORT_FADE_STOP    1
#define LEDC_CTRL_FUNC_IN_IRAM    0

/* Number of timers / channels per speed mode (ESP32 family) */
/* スピードモードあたりのタイマ数・チャンネル数（ESP32 系） */
#define LEDC_TIMER_NUM            4
#define LEDC_CHANNEL_NUM          8

/* Max timer bit width (ESP32 LEDC = 20 bit) */
/* タイマ最大ビット幅（ESP32 LEDC = 20 bit） */
#define LEDC_TIMER_BIT_WIDTH      20

/* -------------------------------------------------------------------------
 * Speed mode / タイマのスピードモード
 * ---------------------------------------------------------------------- */
typedef enum {
    LEDC_HIGH_SPEED_MODE = 0,  // High speed mode / 高速モード
    LEDC_LOW_SPEED_MODE,       // Low speed mode  / 低速モード
    LEDC_SPEED_MODE_MAX,       // Sentinel        / 番兵
} ledc_mode_t;

/* -------------------------------------------------------------------------
 * Timer selection / タイマ選択
 * ---------------------------------------------------------------------- */
typedef enum {
    LEDC_TIMER_0 = 0,  // Timer 0 / タイマ0
    LEDC_TIMER_1,      // Timer 1 / タイマ1
    LEDC_TIMER_2,      // Timer 2 / タイマ2
    LEDC_TIMER_3,      // Timer 3 / タイマ3
    LEDC_TIMER_MAX,    // Sentinel / 番兵
} ledc_timer_t;

/* -------------------------------------------------------------------------
 * Channel selection / チャンネル選択
 * ---------------------------------------------------------------------- */
typedef enum {
    LEDC_CHANNEL_0 = 0,  // Channel 0 / チャンネル0
    LEDC_CHANNEL_1,      // Channel 1 / チャンネル1
    LEDC_CHANNEL_2,      // Channel 2 / チャンネル2
    LEDC_CHANNEL_3,      // Channel 3 / チャンネル3
    LEDC_CHANNEL_4,      // Channel 4 / チャンネル4
    LEDC_CHANNEL_5,      // Channel 5 / チャンネル5
    LEDC_CHANNEL_6,      // Channel 6 / チャンネル6
    LEDC_CHANNEL_7,      // Channel 7 / チャンネル7
    LEDC_CHANNEL_MAX,    // Sentinel  / 番兵
} ledc_channel_t;

/* -------------------------------------------------------------------------
 * Duty resolution (timer bit width) / デューティ分解能（タイマビット幅）
 * ---------------------------------------------------------------------- */
typedef enum {
    LEDC_TIMER_1_BIT = 1,   //  1-bit resolution /  1ビット分解能
    LEDC_TIMER_2_BIT,       //  2-bit
    LEDC_TIMER_3_BIT,       //  3-bit
    LEDC_TIMER_4_BIT,       //  4-bit
    LEDC_TIMER_5_BIT,       //  5-bit
    LEDC_TIMER_6_BIT,       //  6-bit
    LEDC_TIMER_7_BIT,       //  7-bit
    LEDC_TIMER_8_BIT,       //  8-bit  (buzzer)
    LEDC_TIMER_9_BIT,       //  9-bit
    LEDC_TIMER_10_BIT,      // 10-bit  (buzzer)
    LEDC_TIMER_11_BIT,      // 11-bit
    LEDC_TIMER_12_BIT,      // 12-bit
    LEDC_TIMER_13_BIT,      // 13-bit
    LEDC_TIMER_14_BIT,      // 14-bit
    LEDC_TIMER_15_BIT,      // 15-bit
    LEDC_TIMER_16_BIT,      // 16-bit
    LEDC_TIMER_17_BIT,      // 17-bit
    LEDC_TIMER_18_BIT,      // 18-bit
    LEDC_TIMER_19_BIT,      // 19-bit
    LEDC_TIMER_20_BIT,      // 20-bit (max)
    LEDC_TIMER_BIT_MAX,     // Sentinel / 番兵
} ledc_timer_bit_t;

/* -------------------------------------------------------------------------
 * Interrupt type / 割り込み種別
 * ---------------------------------------------------------------------- */
typedef enum {
    LEDC_INTR_DISABLE = 0,    // Disable interrupt     / 割り込み無効
    LEDC_INTR_FADE_END,       // Fade-end interrupt    / フェード完了割り込み
    LEDC_INTR_MAX,            // Sentinel              / 番兵
} ledc_intr_type_t;

/* -------------------------------------------------------------------------
 * Clock source configuration / クロックソース設定
 * ---------------------------------------------------------------------- */
typedef enum {
    LEDC_AUTO_CLK = 0,        // Auto-select clock     / 自動選択
    LEDC_USE_APB_CLK,         // Use APB clock         / APBクロック
    LEDC_USE_RC_FAST_CLK,     // Use RC fast clock     / RC高速クロック
    LEDC_USE_REF_TICK,        // Use REF_TICK          / REF_TICK
    LEDC_USE_XTAL_CLK,        // Use external crystal  / 外部水晶
} ledc_clk_cfg_t;

/* -------------------------------------------------------------------------
 * Timer configuration struct / タイマ設定構造体
 * Field order mirrors real ESP-IDF (firmware uses designated initializers).
 * フィールド順は実 ESP-IDF に一致（ファームは指定初期化子を使用）。
 * ---------------------------------------------------------------------- */
typedef struct {
    ledc_mode_t speed_mode;            // Speed mode        / スピードモード
    ledc_timer_bit_t duty_resolution;  // Duty resolution   / デューティ分解能
    ledc_timer_t timer_num;            // Timer index       / タイマ番号
    uint32_t freq_hz;                  // PWM frequency [Hz]/ PWM周波数
    ledc_clk_cfg_t clk_cfg;            // Clock source      / クロックソース
    bool deconfigure;                  // Deconfigure timer / タイマ解除
} ledc_timer_config_t;

/* -------------------------------------------------------------------------
 * Channel configuration struct / チャンネル設定構造体
 * ---------------------------------------------------------------------- */
typedef struct {
    int gpio_num;                  // Output GPIO       / 出力GPIO
    ledc_mode_t speed_mode;        // Speed mode        / スピードモード
    ledc_channel_t channel;        // Channel index     / チャンネル番号
    ledc_intr_type_t intr_type;    // Interrupt type    / 割り込み種別
    ledc_timer_t timer_sel;        // Bound timer       / 紐付けタイマ
    uint32_t duty;                 // Initial duty      / 初期デューティ
    int hpoint;                    // High-point offset / ハイポイント
    struct {
        unsigned int output_invert : 1;  // Invert output / 出力反転
    } flags;                       // Channel flags     / チャンネルフラグ
} ledc_channel_config_t;

/* -------------------------------------------------------------------------
 * Stub function bodies (inert) / スタブ関数本体（無動作）
 * ---------------------------------------------------------------------- */

// Configure a LEDC timer (no-op on host)
// LEDC タイマを設定（ホストでは何もしない）
static inline esp_err_t ledc_timer_config(const ledc_timer_config_t* timer_conf)
{
    (void)timer_conf;
    return ESP_OK;
}

// Configure a LEDC channel (no-op on host)
// LEDC チャンネルを設定（ホストでは何もしない）
static inline esp_err_t ledc_channel_config(const ledc_channel_config_t* ledc_conf)
{
    (void)ledc_conf;
    return ESP_OK;
}

// Set channel duty (latched on next ledc_update_duty)
// チャンネルのデューティを設定（次の ledc_update_duty で反映）
// SIL: route motor channels (0..3) to the virtual board → Plant. 8-bit full scale.
// SIL: モータ channel(0..3) を仮想ボード→Plant へ。8bit フルスケール。
void sil_board_ledc_set_duty(int channel, uint32_t duty, uint32_t max_duty);
static inline esp_err_t ledc_set_duty(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t duty)
{
    (void)speed_mode;
    sil_board_ledc_set_duty((int)channel, duty, 255u);   // StampFly motors: 8-bit
    return ESP_OK;
}

// Apply the previously set duty
// 設定済みデューティを反映
static inline esp_err_t ledc_update_duty(ledc_mode_t speed_mode, ledc_channel_t channel)
{
    (void)speed_mode;
    (void)channel;
    return ESP_OK;
}

// Read back the current duty (host returns 0)
// 現在のデューティを取得（ホストは 0 を返す）
static inline uint32_t ledc_get_duty(ledc_mode_t speed_mode, ledc_channel_t channel)
{
    (void)speed_mode;
    (void)channel;
    return 0;
}

// Change PWM frequency of a timer (no-op on host)
// タイマの PWM 周波数を変更（ホストでは何もしない）
static inline esp_err_t ledc_set_freq(ledc_mode_t speed_mode, ledc_timer_t timer_num, uint32_t freq_hz)
{
    (void)speed_mode;
    (void)timer_num;
    (void)freq_hz;
    return ESP_OK;
}

// Stop a channel and drive its output to idle_level
// チャンネルを停止し、出力を idle_level に固定
static inline esp_err_t ledc_stop(ledc_mode_t speed_mode, ledc_channel_t channel, uint32_t idle_level)
{
    (void)speed_mode;
    (void)channel;
    (void)idle_level;
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif
