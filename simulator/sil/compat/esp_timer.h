/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file esp_timer.h
 * @brief Host stub for esp_timer — exposes the SIL virtual clock
 *        esp_timer のホスト用スタブ — SIL の仮想時計を公開する
 *
 * The firmware uses esp_timer_get_time() as a microsecond monotonic clock.
 * On the SIL this MUST be the simulation's virtual time (driven by the
 * RTOS emulator), not the wall clock — otherwise runs are not reproducible.
 *
 * 本体ファームは esp_timer_get_time() をマイクロ秒の単調時計として使う。
 * SIL ではこれを壁時計でなく、シミュレーションの仮想時間（RTOS
 * エミュレータが進める）にしなければならない。さもないと再現性が失われる。
 *
 * In P1.0 the virtual time stays 0; the RTOS emulator (P1.1) drives it via
 * sil::compat::set_virtual_time_us().
 *
 * P1.0 では仮想時間は 0 のまま。RTOS エミュレータ（P1.1）が
 * sil::compat::set_virtual_time_us() で進める。
 */

#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/// Monotonic microsecond clock — returns the SIL virtual time
/// 単調マイクロ秒時計 — SIL の仮想時間を返す
int64_t esp_timer_get_time(void);

#ifdef __cplusplus
}  // extern "C"

namespace sil {
namespace compat {

/// Set the virtual time [us] (called by the RTOS emulator scheduler)
/// 仮想時間 [us] を設定する（RTOS エミュレータのスケジューラが呼ぶ）
void set_virtual_time_us(int64_t now_us);

/// Advance the virtual time by delta [us]
/// 仮想時間を delta [us] だけ進める
void advance_virtual_time_us(int64_t delta_us);

}  // namespace compat
}  // namespace sil
#endif
