/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — ESP-IDF host platform).
 */

/**
 * @file esp_rom_sys.h
 * @brief Host shim for esp_rom_sys (ROM-resident busy-wait / printf helpers).
 *        esp_rom_sys のホスト用シム（ROM 常駐のビジーウェイト / printf）。
 *
 * On ESP32 these live in ROM and are used during early boot or inside ISRs
 * where the normal printf/vTaskDelay paths are unavailable. On the host we
 * forward esp_rom_printf to printf and make esp_rom_delay_us a tiny busy spin.
 *
 * ESP32 ではこれらは ROM 常駐で、通常の printf/vTaskDelay が使えない起動初期
 * や ISR 内で使われる。ホストでは esp_rom_printf を printf に転送し、
 * esp_rom_delay_us は小さなビジーウェイトにする。
 */

#pragma once

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

// Busy-wait for the given number of microseconds.
// On the host this is intentionally inert (no real-time guarantees in SILS),
// so we keep it as a no-op to avoid burning CPU during fast-forward sim.
// 指定マイクロ秒だけビジーウェイトする。
// ホストでは SILS にリアルタイム保証がないため意図的に inert（no-op）にし、
// 早送りシミュレーション中に CPU を浪費しないようにする。
static inline void esp_rom_delay_us(uint32_t us)
{
    (void)us;
}

// printf-equivalent usable from early boot / ISR context on ESP32.
// On the host we simply forward to the standard printf.
// ESP32 で起動初期 / ISR から使える printf 相当。
// ホストでは標準 printf に転送するだけ。
#define esp_rom_printf(...) printf(__VA_ARGS__)

#ifdef __cplusplus
}
#endif
