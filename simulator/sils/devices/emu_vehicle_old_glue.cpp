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
 * NOTE: the flight-log recorder (devices/emu_flightlog.cpp) exposes a weak
 * sils_emu_flightlog_firmware_sample hook that a firmware-specific glue implements to
 * write the firmware's own streams (imu/attitude/posvel/...) next to truth.csv. It is
 * deliberately NOT implemented for vehicle_old: the recorder samples from the
 * scheduler's on_advance, which first fires during early app_main (the USB-settle
 * vTaskDelay) BEFORE StampFlyState's mutex exists — reading firmware state there
 * dereferences a null mutex and crashes, and vehicle_old is frozen legacy anyway. Its
 * bundle therefore carries truth.csv only.
 *
 * 注: フライトログレコーダ（emu_flightlog.cpp）はファーム固有 glue が実装する弱フック
 * sils_emu_flightlog_firmware_sample を持ち、truth.csv の隣にファーム自身のストリーム
 * （imu/attitude/posvel/…）を書く。vehicle_old では意図的に未実装: レコーダは
 * on_advance（スケジューラ文脈）から採取し、それは app_main 初期（USB 待ちの
 * vTaskDelay）に StampFlyState の mutex 生成前に最初に発火するため、そこでファーム
 * 状態を読むと null mutex で落ちる。vehicle_old は凍結レガシーでもあるため、その一式は
 * truth.csv のみを持つ。
 */

namespace globals {
volatile bool g_setup_complete = true;
}
