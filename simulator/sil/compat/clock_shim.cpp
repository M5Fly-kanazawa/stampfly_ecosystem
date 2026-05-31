/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file clock_shim.cpp
 * @brief Implementation of the SIL virtual clock behind esp_timer_get_time
 *        esp_timer_get_time の背後にある SIL 仮想時計の実装
 *
 * A single int64 microsecond counter. Deterministic: only the RTOS
 * emulator advances it, never the wall clock. Same inputs → same time.
 *
 * int64 のマイクロ秒カウンタ1個。決定論的：RTOS エミュレータだけが
 * 進め、壁時計は使わない。同じ入力 → 同じ時刻。
 */

#include "esp_timer.h"

namespace {
// The single source of virtual time. Defaults to 0.
// 仮想時間の唯一の出どころ。既定は 0。
int64_t g_virtual_time_us = 0;
}  // namespace

extern "C" int64_t esp_timer_get_time(void)
{
    return g_virtual_time_us;
}

namespace sil {
namespace compat {

void set_virtual_time_us(int64_t now_us)
{
    g_virtual_time_us = now_us;
}

void advance_virtual_time_us(int64_t delta_us)
{
    g_virtual_time_us += delta_us;
}

}  // namespace compat
}  // namespace sil
