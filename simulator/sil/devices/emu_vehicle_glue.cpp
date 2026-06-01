/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file emu_vehicle_glue.cpp
 * @brief Host glue for the OLD firmware (firmware/vehicle) emulator target —
 *        symbols the firmware leaves to its environment.
 *        旧ファーム(firmware/vehicle)エミュレータ用 host glue。
 *
 * g_setup_complete: the serial CLI (sf_svc_serial_cli) references a WEAK
 * `globals::g_setup_complete` that is only defined in the workshop/Arduino-style
 * sketch context, NOT in the vehicle firmware build. On the host the weak symbol
 * is left undefined by the firmware, so the emulator supplies it as "setup done".
 * serial CLI が weak で参照する `globals::g_setup_complete` は workshop 文脈のみで
 * 定義され vehicle ビルドには無い。host では「setup 完了」として供給する。
 */

namespace globals {
volatile bool g_setup_complete = true;
}
