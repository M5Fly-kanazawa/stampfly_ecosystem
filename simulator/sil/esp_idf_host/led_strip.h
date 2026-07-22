/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — ESP-IDF host platform).
 */

/**
 * @file led_strip.h
 * @brief Host shim for the espressif "led_strip" managed component (RMT LEDs).
 *        espressif "led_strip" managed component（RMT LED）のホスト用シム。
 *
 * The StampFly LED HAL (sf_hal_led) drives WS2812-style LEDs via led_strip over
 * RMT. LEDs are not flight-critical, so on the host this is an inert stub (no RMT
 * dependency). A faithful capture of pixel state can be added in E4 if needed.
 * StampFly の LED HAL は RMT 経由で led_strip を使う。LED は飛行クリティカルでない
 * ので、ホストでは inert スタブ（RMT 非依存）。E4 で画素状態の記録を足してもよい。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct led_strip_t* led_strip_handle_t;

typedef enum { LED_PIXEL_FORMAT_GRB, LED_PIXEL_FORMAT_GRBW } led_pixel_format_t;
typedef enum { LED_MODEL_WS2812, LED_MODEL_SK6812 } led_model_t;

// Newer led_strip API: per-component color format (replaces led_pixel_format).
// 新しい led_strip API: 成分別カラーフォーマット（led_pixel_format を置換）。
typedef enum {
    LED_STRIP_COLOR_COMPONENT_FMT_GRB  = 0,
    LED_STRIP_COLOR_COMPONENT_FMT_GRBW = 1,
    LED_STRIP_COLOR_COMPONENT_FMT_RGB  = 2,
} led_color_component_format_t;

// RMT clock source (driver/rmt). Default suffices on the inert host stub.
// RMT クロック源。inert なホストスタブでは default で十分。
#ifndef RMT_CLK_SRC_DEFAULT
#define RMT_CLK_SRC_DEFAULT 0
#endif

typedef struct {
    int      strip_gpio_num;
    uint32_t max_leds;
    led_pixel_format_t led_pixel_format;        // legacy field (kept for compat)
    // Declaration order matches real ESP-IDF's led_strip_config_t (led_model
    // before color_component_format) — firmware's led.cpp initializes them in
    // that order, and C++ designated initializers require the designator
    // order to match declaration order (unlike C, which allows any order).
    // 実 ESP-IDF の led_strip_config_t と同じ宣言順（color_component_format より
    // 先に led_model）。firmware の led.cpp はこの順で初期化しており、C++の
    // 指示付き初期化子は宣言順と一致させる必要がある（任意順が許される C とは異なる）。
    led_model_t        led_model;
    led_color_component_format_t color_component_format;  // newer API
    struct { uint32_t invert_out : 1; } flags;
} led_strip_config_t;

typedef struct {
    uint32_t clk_src;
    uint32_t resolution_hz;
    size_t   mem_block_symbols;
    struct { uint32_t with_dma : 1; } flags;
} led_strip_rmt_config_t;

static inline esp_err_t led_strip_new_rmt_device(const led_strip_config_t* strip_cfg,
                                                 const led_strip_rmt_config_t* rmt_cfg,
                                                 led_strip_handle_t* out)
{
    (void)strip_cfg; (void)rmt_cfg;
    if (out) *out = (led_strip_handle_t)0x1;   // non-null sentinel
    return ESP_OK;
}

static inline esp_err_t led_strip_set_pixel(led_strip_handle_t h, uint32_t index,
                                            uint32_t r, uint32_t g, uint32_t b)
{
    (void)h; (void)index; (void)r; (void)g; (void)b;
    return ESP_OK;
}

static inline esp_err_t led_strip_refresh(led_strip_handle_t h) { (void)h; return ESP_OK; }
static inline esp_err_t led_strip_clear(led_strip_handle_t h)   { (void)h; return ESP_OK; }
static inline esp_err_t led_strip_del(led_strip_handle_t h)     { (void)h; return ESP_OK; }

#ifdef __cplusplus
}
#endif
