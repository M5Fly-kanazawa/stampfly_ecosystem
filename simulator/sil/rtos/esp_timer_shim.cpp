/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file esp_timer_shim.cpp
 * @brief Host implementation of the esp_timer periodic API on the RTOS emulator.
 *        RTOS エミュレータ上の esp_timer periodic API のホスト実装。
 *
 * The firmware paces its 400Hz IMU loop with an esp_timer periodic. On the SIL,
 * a periodic timer is a deterministic virtual-clock wake source: the scheduler
 * fires the callback when the virtual clock reaches each multiple of the period
 * (see Scheduler::fire_due_timers). This keeps the loop reproducible — same run,
 * same trace. esp_timer_get_time() lives in clock_shim.cpp (shared with
 * cores_smoke); these periodic functions need the scheduler, so they live here,
 * linked only into the RTOS-emulator target.
 *
 * 本体ファームは 400Hz IMU ループを esp_timer periodic で刻む。SIL では周期タイマは
 * 決定論的な仮想時計の起床源: 仮想時計が周期の各倍数に達したときスケジューラが
 * コールバックを発火する（Scheduler::fire_due_timers 参照）。これでループは再現
 * 可能になる（同じ実行・同じトレース）。esp_timer_get_time() は clock_shim.cpp
 * （cores_smoke と共有）にある。これら periodic 関数はスケジューラを必要とするため
 * ここに置き、RTOS エミュレータのターゲットにのみリンクする。
 */

#include "esp_timer.h"

#include "scheduler.hpp"

// esp_timer object: a callback + arg, plus the scheduler-side registration id.
// esp_timer オブジェクト: コールバック＋引数と、スケジューラ側の登録 id。
struct esp_timer {
    esp_timer_cb_t cb = nullptr;
    void* arg = nullptr;
    int sched_id = -1;
};

extern "C" {

esp_err_t esp_timer_create(const esp_timer_create_args_t* args,
                           esp_timer_handle_t* out_handle)
{
    if (args == nullptr || out_handle == nullptr || args->callback == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_timer* t = new esp_timer();
    t->cb = args->callback;
    t->arg = args->arg;
    t->sched_id = -1;
    *out_handle = t;
    return ESP_OK;
}

esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period_us)
{
    if (timer == nullptr || timer->cb == nullptr) return ESP_ERR_INVALID_ARG;
    timer->sched_id = sil::rtos::Scheduler::instance().add_periodic(
        timer->cb, timer->arg, static_cast<int64_t>(period_us));
    return ESP_OK;
}

esp_err_t esp_timer_stop(esp_timer_handle_t timer)
{
    if (timer == nullptr) return ESP_ERR_INVALID_ARG;
    if (timer->sched_id >= 0) {
        sil::rtos::Scheduler::instance().remove_periodic(timer->sched_id);
        timer->sched_id = -1;
    }
    return ESP_OK;
}

esp_err_t esp_timer_delete(esp_timer_handle_t timer)
{
    if (timer == nullptr) return ESP_ERR_INVALID_ARG;
    esp_timer_stop(timer);
    delete timer;
    return ESP_OK;
}

}  // extern "C"
