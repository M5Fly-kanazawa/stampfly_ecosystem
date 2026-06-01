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

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Monotonic microsecond clock — returns the SIL virtual time
/// 単調マイクロ秒時計 — SIL の仮想時間を返す
int64_t esp_timer_get_time(void);

// -----------------------------------------------------------------------------
// Periodic timer API (subset the firmware uses to pace the 400Hz IMU loop).
// On the SIL these register a deterministic virtual-clock wake source with the
// RTOS emulator scheduler (see rtos/esp_timer_shim.cpp); the callback runs in the
// scheduler's advance phase and typically does xTaskNotifyGive.
// 周期タイマ API（400Hz IMU ループの駆動に本体が使う部分集合）。SIL では RTOS
// エミュレータのスケジューラに決定論的な仮想時計起床源を登録する（rtos/
// esp_timer_shim.cpp 参照）。コールバックは advance フェーズで走り、通常
// xTaskNotifyGive を行う。
// -----------------------------------------------------------------------------

/// Opaque timer handle. / 不透明なタイマハンドル。
typedef struct esp_timer* esp_timer_handle_t;

/// Timer callback. / タイマコールバック。
typedef void (*esp_timer_cb_t)(void* arg);

/// Dispatch method (only TASK is modeled on the SIL). / ディスパッチ方式。
typedef enum {
    ESP_TIMER_TASK,
    ESP_TIMER_ISR,
} esp_timer_dispatch_t;

/// Arguments for esp_timer_create (field order matches ESP-IDF).
/// esp_timer_create の引数（フィールド順は ESP-IDF と一致）。
typedef struct {
    esp_timer_cb_t callback;
    void* arg;
    esp_timer_dispatch_t dispatch_method;
    const char* name;
    bool skip_unhandled_events;
} esp_timer_create_args_t;

esp_err_t esp_timer_create(const esp_timer_create_args_t* args,
                           esp_timer_handle_t* out_handle);
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period_us);
esp_err_t esp_timer_stop(esp_timer_handle_t timer);
esp_err_t esp_timer_delete(esp_timer_handle_t timer);

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
