/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file scheduler.hpp
 * @brief Deterministic cooperative RTOS emulator for the SILS host bench.
 *        SILS ホストベンチ用の決定論的協調 RTOS エミュレータ。
 *
 * Runs the UNMODIFIED firmware task functions on a PC. Each task gets its own
 * std::thread, but a single run-token guarantees exactly one runs at a time
 * (cooperative). A single virtual microsecond clock advances only when every
 * task is blocked, so the schedule is a deterministic discrete-event sequence:
 * same inputs → same trace, every run (reproducibility).
 *
 * 本体タスク関数を無改変で PC 上で走らせる。各タスクは自分の std::thread を
 * 持つが、単一の実行トークンで常に1個だけ走る（協調）。単一の仮想マイクロ秒
 * 時計は、全タスクがブロックしたときだけ進む。よってスケジュールは決定論的な
 * 離散事象列になる: 同じ入力 → 毎回同じトレース（再現性）。
 *
 * What this DOES reproduce: task rates, ordering, IMU→Control notification,
 * sensor age, dt. What it does NOT (by construction): sub-step preemption and
 * the data races / jitter that come with it — those need real hardware
 * (RESET_PLAN §11). Each task step is atomic at its logical instant.
 *
 * 再現できるもの: タスクのレート・順序・IMU→Control 通知・センサ鮮度・dt。
 * 再現しないもの（構造上）: 計算途中の横取りと、それに伴う競合・ジッタ —
 * これらは実機が必要（RESET_PLAN §11）。各タスクのステップはその論理時刻で
 * 一瞬に完結する。
 *
 * @design simulator/sils/RESET_PLAN.md §5 — run the real loop unmodified
 * @design simulator/sils/RESET_PLAN.md §7 — host RTOS sim, virtual clock
 * @design simulator/sils/RESET_PLAN.md §11 — single deterministic loop
 */

#pragma once

#include <cstdint>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace sils {
namespace rtos {

// Thrown into a parked task at shutdown to unwind its (infinite-loop) body and
// let the thread exit cleanly, so the scheduler can join — no process-kill.
// The firmware tasks are unchanged: this is language-level stack unwinding
// through their frames, not a code modification (their locals are trivial).
//
// シャットダウン時に待機中のタスクへ投げ、(無限ループの) 本体を巻き戻して
// スレッドを綺麗に終了させ、スケジューラが join できるようにする — プロセス
// 強制終了をしない。本体タスクは無改変: これはそのフレームを通る言語レベルの
// スタック巻き戻しで、コード変更ではない (ローカルは自明な型)。
struct StopTask {};

// Run state of a task.
// タスクの実行状態。
enum class TaskState {
    Ready,         // can run now (newly created, or notification arrived)
    Running,       // currently holds the token
    BlockedDelay,  // sleeping until wake_us
    BlockedNotify, // waiting for a notification
    Finished,      // returned / deleted
};

// One emulated task.
// エミュレートされたタスク1個。
struct Task {
    std::string name;
    TaskFunction_t fn = nullptr;
    void* param = nullptr;
    UBaseType_t priority = 0;
    uint32_t id = 0;                 // creation order — deterministic tie-break

    TaskState state = TaskState::Ready;
    int64_t wake_us = 0;             // for BlockedDelay
    uint32_t notify_count = 0;       // notification value
    bool exited = false;             // thread has left its body (joinable, done)

    std::thread thread;
    std::condition_variable cv;      // signalled when this task is granted the token
};

// One scheduling event (for the trace / determinism check).
// スケジューリング事象1個（トレース・決定論チェック用）。
struct TraceEvent {
    int64_t time_us;
    uint32_t task_id;
};

// The cooperative scheduler. One instance per SILS run.
// 協調スケジューラ。SILS 実行ごとに1インスタンス。
class Scheduler {
public:
    static Scheduler& instance();

    // Register a task (does not run it yet). Returns its handle.
    // タスクを登録する（まだ走らせない）。ハンドルを返す。
    TaskHandle_t create(TaskFunction_t fn, void* param,
                        UBaseType_t priority, const char* name);

    // Optional per-step hook, called by the scheduler at the current virtual
    // time before picking the next task. Used to inject a flight scenario
    // (mode / setpoint) at scripted times. now_us is the virtual time.
    // 任意のステップフック。次タスク選択前に現在仮想時刻で呼ばれる。台本時刻で
    // 飛行シナリオ（モード/セットポイント）を注入するのに使う。
    void set_on_advance(std::function<void(int64_t now_us)> hook);

    // Run the cooperative loop until the virtual clock reaches max_sim_us
    // (or every task is permanently blocked).
    // 仮想時計が max_sim_us に達するまで（または全タスクが恒久ブロックになるまで）
    // 協調ループを回す。
    void run(int64_t max_sim_us);

    int64_t now_us() const { return now_us_; }
    const std::vector<TraceEvent>& trace() const { return trace_; }

    // --- Primitives called by the FreeRTOS shims (task.h) ---
    // --- FreeRTOS シム（task.h）から呼ばれるプリミティブ ---
    void delay_until_us(int64_t wake_us);
    // timeout_us < 0 = wait forever (portMAX_DELAY); else wake at now+timeout even
    // without a notification (returns 0 on timeout). FreeRTOS ulTaskNotifyTake semantics.
    // timeout_us<0 で無限待ち、それ以外は通知無しでも now+timeout で起床（timeout 時 0 を返す）。
    uint32_t notify_take(bool clear_on_exit, int64_t timeout_us);
    void notify_give(Task* target);
    void delete_self();

    // Handle of the task currently holding the token (xTaskGetCurrentTaskHandle).
    // 現在トークンを保持しているタスクのハンドル（xTaskGetCurrentTaskHandle）。
    TaskHandle_t current_handle();

    // --- Periodic timers (driven by the esp_timer shim) ---
    // A periodic timer is a deterministic virtual-clock wake source: when the
    // clock reaches next_fire_us, the scheduler runs the firmware callback (which
    // typically does xTaskNotifyGive). This is how a true 400Hz esp_timer loop is
    // reproduced on the host. Fired during the all-blocked advance phase, in
    // registration order, for a stable trace.
    // --- 周期タイマ（esp_timer シムが駆動）---
    // 周期タイマは決定論的な仮想時計の起床源: 時計が next_fire_us に達すると
    // スケジューラが本体コールバック（通常 xTaskNotifyGive）を実行する。これで真の
    // 400Hz esp_timer ループを host 上で再現する。全ブロックの advance 中に登録順で
    // 発火し、トレースを安定させる。
    using TimerCallback = void (*)(void*);
    int  add_periodic(TimerCallback cb, void* arg, int64_t period_us);  // returns id
    void remove_periodic(int id);

private:
    Scheduler() = default;

    Task* pick_ready();              // highest priority among Ready, tie-break by id
    int64_t earliest_wake() const;   // min wake_us among BlockedDelay; -1 if none
    int64_t earliest_timer_fire() const;  // min next_fire_us among active timers; -1 if none
    void fire_due_timers(std::unique_lock<std::mutex>& lk);  // run callbacks due at now_us_
    void run_task_thread(Task* self);          // per-task thread body
    void block_current(TaskState new_state);   // give up token, wait until re-granted
    void grant_and_wait(std::unique_lock<std::mutex>& lk, Task* task);  // hand off token, wait for yield
    void stop_all();                 // unwind + join every task thread (clean teardown)

    // A periodic virtual-clock timer. / 周期的な仮想時計タイマ。
    struct PeriodicTimer {
        TimerCallback cb = nullptr;
        void* arg = nullptr;
        int64_t period_us = 0;
        int64_t next_fire_us = 0;
        bool active = false;
    };

    std::mutex m_;
    std::condition_variable sched_cv_;   // signalled when the running task yields
    std::vector<Task*> tasks_;
    Task* running_ = nullptr;
    int64_t now_us_ = 0;
    bool shutdown_ = false;
    std::function<void(int64_t)> on_advance_;
    std::vector<TraceEvent> trace_;
    std::vector<PeriodicTimer> timers_;  // registered periodic (esp_timer) sources
};

}  // namespace rtos
}  // namespace sils
