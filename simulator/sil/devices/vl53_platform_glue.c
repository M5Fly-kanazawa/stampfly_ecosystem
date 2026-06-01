/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file vl53_platform_glue.c
 * @brief Host platform-port glue for the ST VL53L3CX BareDriver — the ONE
 *        platform function the firmware's vl53lx_platform.c is missing.
 *        ST VL53L3CX BareDriver の host プラットフォーム glue — ファームの
 *        vl53lx_platform.c に欠けている唯一のプラットフォーム関数。
 *
 * The ST driver delegates environment-specific functions (I2C, WaitMs, tick
 * count) to the integrator's platform layer. The firmware's vl53lx_platform.c
 * implements VL53LX_WaitUs/WaitMs/GetTimerFrequency/WaitValueMaskEx but NOT
 * VL53LX_GetTickCount (called from vl53lx_wait.c) — so the ToF driver does not
 * link as-is, on the host OR on hardware. **This is a real firmware gap the
 * emulator surfaced.** Until the firmware's platform.c adds it, the emulator
 * supplies the host port, sourced from the SIL virtual clock (deterministic).
 *
 * ST ドライバは環境固有関数（I2C・WaitMs・tick）を integrator のプラットフォーム層へ
 * 委譲する。ファームの vl53lx_platform.c は GetTickCount だけ未実装ゆえ ToF ドライバが
 * host でも実機でもリンクしない＝**エミュレータが炙り出した実ファームのギャップ**。
 * ファームが platform.c に足すまで、host port を SIL 仮想時計から供給する（決定論的）。
 *
 * @design simulator/sil/RESET_PLAN.md §3 — SIL surfaces firmware gaps  [--]
 */

#include "vl53lx_platform.h"
#include "esp_timer.h"

VL53LX_Error VL53LX_GetTickCount(VL53LX_DEV Dev, uint32_t *ptime_ms)
{
    (void)Dev;
    if (ptime_ms != 0) {
        // SIL virtual clock [us] → milliseconds (monotonic, reproducible).
        // SIL 仮想時計 [us] → ミリ秒（単調・再現可能）。
        *ptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    }
    return VL53LX_ERROR_NONE;
}
