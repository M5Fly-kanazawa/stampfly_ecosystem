/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 */

/**
 * @file emu_vehicle_old_glue.cpp
 * @brief Host glue for the OLD firmware (firmware/vehicle_old) emulator target —
 *        symbols the firmware leaves to its environment.
 *        旧ファーム(firmware/vehicle_old)エミュレータ用 host glue。
 *
 * g_setup_complete: the serial CLI (sf_svc_serial_cli) references a WEAK
 * `globals::g_setup_complete` that is only defined in the workshop/Arduino-style
 * sketch context, NOT in the vehicle firmware build. On the host the weak symbol
 * is left undefined by the firmware, so the emulator supplies it as "setup done".
 * serial CLI が weak で参照する `globals::g_setup_complete` は workshop 文脈のみで
 * 定義され vehicle ビルドには無い。host では「setup 完了」として供給する。
 *
 * NOTE: the review-video trajectory recorder (devices/emu_trajectory.cpp) exposes a
 * weak sils_emu_estimate hook a firmware-specific glue could implement to overlay the
 * firmware's own estimate on the truth. It is deliberately NOT implemented here yet:
 * the recorder samples from the scheduler's on_advance, which first fires during
 * early app_main (the USB-settle vTaskDelay) BEFORE StampFlyState's mutex exists —
 * reading firmware state there dereferences a null mutex and crashes. The estimate
 * overlay needs a boot-safe firmware sampling path first; until then the recorder
 * leaves estimate ≡ truth.
 *
 * 注: 動画軌跡レコーダ（emu_trajectory.cpp）はファーム固有 glue が実装できる弱フック
 * sils_emu_estimate を持つが、ここでは意図的に未実装。レコーダは on_advance（スケジューラ
 * 文脈）から採取し、それは app_main 初期（USB 待ちの vTaskDelay）に StampFlyState の
 * mutex 生成前に最初に発火するため、そこでファーム状態を読むと null mutex で落ちる。
 * 起動安全なファーム採取経路を先に用意するまで、レコーダは estimate ≡ truth とする。
 */

namespace globals {
volatile bool g_setup_complete = true;
}
