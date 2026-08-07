/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — ESP-IDF host platform).
 */

/**
 * @file rom/ets_sys.h
 * @brief Host shim for ESP32 ROM system helpers (ets_delay_us etc.).
 *        ESP32 ROM システムヘルパのホスト用シム。
 *
 * Some HAL drivers (e.g. sf_hal_bmi270/bmi270_spi.c) include rom/ets_sys.h for
 * busy-wait delays. On the host these are inert (no real-time in the SILS).
 * 一部の HAL ドライバ（bmi270_spi.c 等）が rom/ets_sys.h を include する。
 * ホストでは inert（SILS にリアルタイム保証なし）。
 */

#pragma once

#include <stdint.h>

#include "esp_rom_sys.h"   // esp_rom_delay_us (single definition)

#ifdef __cplusplus
extern "C" {
#endif

// Busy-wait microseconds (ROM symbol). Inert on the host.
// マイクロ秒ビジーウェイト（ROM シンボル）。ホストでは inert。
static inline void ets_delay_us(uint32_t us)
{
    (void)us;
}

#ifdef __cplusplus
}
#endif
