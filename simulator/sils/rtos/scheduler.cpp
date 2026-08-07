/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file scheduler.cpp
 * @brief Deterministic cooperative scheduler + FreeRTOS active-surface shims.
 *        決定論的協調スケジューラ ＋ FreeRTOS 能動面のシム。
 *
 * See scheduler.hpp for the model. The FreeRTOS C functions the firmware calls
 * (xTaskCreatePinnedToCore / vTaskDelayUntil / ulTaskNotifyTake / ...) are
 * defined here and forward to the single Scheduler instance.
 *
 * Correctness rests on one invariant, NOT on the trace-hash test: every access
 * to shared task state happens under m_, and at most one task ever holds the
 * run-token (running_). Data races are therefore structurally impossible. The
 * trace hash only shows the LOGICAL schedule is reproducible.
 *
 * 正しさは1つの不変条件に依る（トレースハッシュ検査ではない）: 共有タスク
 * 状態への全アクセスは m_ の下で行われ、実行トークン(running_)を持つタスクは
 * 常に高々1個。よってデータ競合は構造的に起こり得ない。ハッシュは論理
 * スケジュールが再現可能であることを示すだけ。
 */

#include "scheduler.hpp"
#include "esp_timer.h"

#include <cstdio>
#include <cstdlib>

namespace sils {
namespace rtos {

namespace {
// The task running on the current OS thread (set in run_task_thread).
// 現在の OS スレッドで走っているタスク（run_task_thread で設定）。
thread_local Task* tls_self = nullptr;

// Wall-clock budget for one task step before we declare a hang. This is the
// ONLY use of the wall clock — purely a deadlock/infinite-loop detector, never
// for scheduling (the schedule uses the virtual clock only).
// 1タスクステップの壁時計上限。壁時計はここだけで使う — デッドロック/無限
// ループ検出専用で、スケジューリングには使わない（スケジュールは仮想時計のみ）。
constexpr std::chrono::seconds kHangBudget{10};
}  // namespace

Scheduler& Scheduler::instance()
{
    static Scheduler s;
    return s;
}

TaskHandle_t Scheduler::create(TaskFunction_t fn, void* param,
                               UBaseType_t priority, const char* name)
{
    Task* task = new Task();
    task->name = name ? name : "task";
    task->fn = fn;
    task->param = param;
    task->priority = priority;
    task->id = static_cast<uint32_t>(tasks_.size());
    task->state = TaskState::Ready;

    tasks_.push_back(task);

    // Start the thread; it parks until the scheduler grants it the token. Kept
    // joinable (not detached) so stop_all() can unwind and join it cleanly.
    // スレッド開始; スケジューラがトークンを与えるまで待機。join 可能に保つ
    // （detach しない）ので stop_all() が巻き戻して綺麗に join できる。
    task->thread = std::thread(&Scheduler::run_task_thread, this, task);

    return static_cast<TaskHandle_t>(task);
}

void Scheduler::set_on_advance(std::function<void(int64_t)> hook)
{
    on_advance_ = std::move(hook);
}

void Scheduler::run_task_thread(Task* self)
{
    tls_self = self;

    // Wait for the first grant of the token before entering the task function.
    // タスク関数に入る前に、最初のトークン付与を待つ。
    {
        std::unique_lock<std::mutex> lk(m_);
        self->cv.wait(lk, [&] { return running_ == self; });
    }

    // Run the unmodified firmware task function. Infinite-loop tasks never
    // return on their own; at shutdown a StopTask is thrown into the blocking
    // primitive to unwind the body so this thread can exit and be joined.
    // 無改変の本体タスク関数を走らせる。無限ループのタスクは自分では戻らない;
    // シャットダウン時にブロッキングプリミティブへ StopTask を投げて本体を
    // 巻き戻し、このスレッドが終了して join できるようにする。
    try {
        self->fn(self->param);
    } catch (const StopTask&) {
        // Normal teardown path. / 通常のteardown経路。
    }

    std::unique_lock<std::mutex> lk(m_);
    self->state = TaskState::Finished;
    self->exited = true;
    running_ = nullptr;
    sched_cv_.notify_all();
}

void Scheduler::block_current(TaskState new_state)
{
    // Caller holds m_. Give up the token and wait until re-granted. On shutdown
    // the re-grant carries shutdown_, so we throw to unwind the task body.
    // 呼び出し側は m_ を保持。トークンを返し、再付与まで待つ。シャットダウン時は
    // 再付与が shutdown_ を伴うので、本体を巻き戻すため throw する。
    std::unique_lock<std::mutex> lk(m_, std::adopt_lock);
    Task* self = tls_self;
    self->state = new_state;
    running_ = nullptr;
    sched_cv_.notify_all();
    self->cv.wait(lk, [&] { return running_ == self; });
    if (shutdown_) {
        throw StopTask{};  // lk's dtor unlocks m_ as the stack unwinds
    }
    self->state = TaskState::Running;
    lk.release();  // keep m_ LOCKED; the caller is contractually required to unlock
}

void Scheduler::delay_until_us(int64_t wake_us)
{
    m_.lock();
    tls_self->wake_us = wake_us;
    block_current(TaskState::BlockedDelay);
    m_.unlock();
}

uint32_t Scheduler::notify_take(bool clear_on_exit, int64_t timeout_us)
{
    m_.lock();
    Task* self = tls_self;
    if (self->notify_count == 0) {
        // Arm a timeout deadline so the scheduler can also wake us on time (not only
        // on a notification). INT64_MAX = wait forever (portMAX_DELAY). The scheduler
        // wakes BlockedNotify tasks at wake_us just like BlockedDelay.
        // タイムアウト期限を設定し、通知だけでなく時刻でも起床できるようにする。
        // INT64_MAX=無限待ち。スケジューラは BlockedDelay 同様 wake_us で起こす。
        self->wake_us = (timeout_us < 0) ? INT64_MAX : (now_us_ + timeout_us);
        block_current(TaskState::BlockedNotify);
    }
    uint32_t value = self->notify_count;
    self->notify_count = clear_on_exit ? 0
                          : (self->notify_count > 0 ? self->notify_count - 1 : 0);
    m_.unlock();
    return value;
}

void Scheduler::notify_give(Task* target)
{
    // The giver keeps running (cooperative, no preempt); the target becomes
    // Ready and runs at the next scheduling point. Firmware must use the
    // publish-then-notify convention (RESET_PLAN §11), never act-on-give.
    // 与える側は走り続ける（協調・横取りなし）; 対象は Ready になり次の
    // スケジューリング点で走る。本体は publish→notify 規約（RESET_PLAN §11）
    // を使い、give を即座に当てにしない。
    std::lock_guard<std::mutex> lk(m_);
    target->notify_count++;
    if (target->state == TaskState::BlockedNotify) {
        target->state = TaskState::Ready;
    }
}

void Scheduler::delete_self()
{
    // The task removed itself (vTaskDelete(NULL)). Yield the token and park
    // until shutdown, then unwind so the thread exits and can be joined.
    // タスクが自分を削除した（vTaskDelete(NULL)）。トークンを返し、シャット
    // ダウンまで待機し、巻き戻してスレッドを終了・join 可能にする。
    std::unique_lock<std::mutex> lk(m_);
    Task* self = tls_self;
    self->state = TaskState::Finished;
    running_ = nullptr;
    sched_cv_.notify_all();
    self->cv.wait(lk, [&] { return shutdown_; });
    throw StopTask{};
}

Task* Scheduler::pick_ready()
{
    // Highest priority among Ready; ties broken by lowest id (deterministic).
    // Ready の中で最高優先度; 同点は最小 id（決定論的）。
    Task* best = nullptr;
    for (Task* task : tasks_) {
        if (task->state != TaskState::Ready) continue;
        if (best == nullptr || task->priority > best->priority ||
            (task->priority == best->priority && task->id < best->id)) {
            best = task;
        }
    }
    return best;
}

int64_t Scheduler::earliest_wake() const
{
    int64_t earliest = -1;
    for (const Task* task : tasks_) {
        // BlockedDelay always has a finite deadline; BlockedNotify only when a finite
        // timeout was armed (INT64_MAX = wait-forever, excluded so it never advances time).
        // BlockedDelay は常に有限期限。BlockedNotify は有限 timeout を張ったときだけ
        // （INT64_MAX=無限待ちは除外、時計を進めない）。
        const bool timed = task->state == TaskState::BlockedDelay ||
                           (task->state == TaskState::BlockedNotify && task->wake_us < INT64_MAX);
        if (!timed) continue;
        if (earliest < 0 || task->wake_us < earliest) {
            earliest = task->wake_us;
        }
    }
    return earliest;
}

void Scheduler::grant_and_wait(std::unique_lock<std::mutex>& lk, Task* task)
{
    // Hand the token to `task` and wait until it yields it back. A wall-clock
    // budget turns a non-yielding (hung / infinite-loop) task into a clear
    // abort instead of a silent forever-block.
    // トークンを `task` に渡し、返すまで待つ。壁時計の上限で、ゆずらない
    // （ハング/無限ループ）タスクを、静かな永久ブロックでなく明確な abort にする。
    running_ = task;
    task->state = TaskState::Running;
    task->cv.notify_all();
    if (!sched_cv_.wait_for(lk, kHangBudget, [&] { return running_ == nullptr; })) {
        fprintf(stderr,
                "[scheduler] FATAL: task '%s' did not yield within %llds — "
                "infinite loop without a blocking primitive?\n",
                task->name.c_str(),
                static_cast<long long>(kHangBudget.count()));
        std::abort();
    }
}

// =============================================================================
// Current task handle + periodic timers (esp_timer backing).
// 現在タスクハンドル＋周期タイマ（esp_timer の裏付け）。
// =============================================================================

TaskHandle_t Scheduler::current_handle()
{
    // Called from inside a running task, so running_ is that task.
    // 走行中のタスクから呼ばれるので running_ がそのタスク。
    return static_cast<TaskHandle_t>(running_);
}

int Scheduler::add_periodic(TimerCallback cb, void* arg, int64_t period_us)
{
    // Called from a running task (it holds the token; no other thread runs), so
    // touching timers_ here needs no extra lock.
    // 走行中のタスクから呼ばれる（トークン保持・他スレッドは走らない）ので、ここで
    // timers_ を触るのに追加ロックは不要。
    PeriodicTimer t;
    t.cb = cb;
    t.arg = arg;
    t.period_us = period_us;
    t.next_fire_us = now_us_ + period_us;
    t.active = true;
    for (size_t i = 0; i < timers_.size(); ++i) {
        if (!timers_[i].active && timers_[i].cb == nullptr) {  // reuse a freed slot
            timers_[i] = t;
            return static_cast<int>(i);
        }
    }
    timers_.push_back(t);
    return static_cast<int>(timers_.size() - 1);
}

void Scheduler::remove_periodic(int id)
{
    if (id >= 0 && id < static_cast<int>(timers_.size())) {
        timers_[id].active = false;
        timers_[id].cb = nullptr;
    }
}

int64_t Scheduler::earliest_timer_fire() const
{
    int64_t earliest = -1;
    for (const PeriodicTimer& t : timers_) {
        if (!t.active) continue;
        if (earliest < 0 || t.next_fire_us < earliest) earliest = t.next_fire_us;
    }
    return earliest;
}

void Scheduler::fire_due_timers(std::unique_lock<std::mutex>& lk)
{
    // Fire every timer due at now_us_, in registration order, with the scheduler
    // mutex UNLOCKED: the firmware callback calls xTaskNotifyGive → notify_give,
    // which locks m_ (non-recursive). No task runs during the advance phase, so
    // unlocking here is safe and deterministic.
    // now_us_ に期限の来た全タイマを登録順で発火。スケジューラ mutex は解放して
    // 行う: 本体コールバックは xTaskNotifyGive → notify_give を呼び m_（非再帰）を
    // ロックするため。advance 中はどのタスクも走らないので安全かつ決定論的。
    for (size_t i = 0; i < timers_.size(); ++i) {
        if (!timers_[i].active) continue;
        if (timers_[i].next_fire_us <= now_us_) {
            TimerCallback cb = timers_[i].cb;
            void* arg = timers_[i].arg;
            timers_[i].next_fire_us += timers_[i].period_us;
            lk.unlock();
            cb(arg);
            lk.lock();
        }
    }
}

void Scheduler::run(int64_t max_sim_us)
{
    // Initial scenario injection at t = 0.
    // t = 0 でのシナリオ注入。
    if (on_advance_) on_advance_(now_us_);

    std::unique_lock<std::mutex> lk(m_);
    while (now_us_ < max_sim_us) {
        Task* next = pick_ready();
        if (next != nullptr) {
            trace_.push_back({now_us_, next->id});
            grant_and_wait(lk, next);
            continue;
        }

        // No ready task → advance the virtual clock to the next wake-up: the
        // earliest of a BlockedDelay wake or a periodic-timer fire.
        // Ready が無い → 仮想時計を次の起床へ進める。BlockedDelay 起床か周期タイマ
        // 発火のうち最も早いもの。
        int64_t next_wake  = earliest_wake();
        int64_t next_timer = earliest_timer_fire();
        if (next_wake < 0 && next_timer < 0) break;  // nothing pending → done
        if (next_wake < 0 || (next_timer >= 0 && next_timer < next_wake)) {
            next_wake = next_timer;
        }

        now_us_ = next_wake;
        sils::compat::set_virtual_time_us(now_us_);
        for (Task* task : tasks_) {
            // Wake both delay-blocked and notify-with-timeout tasks at their deadline.
            // A wait-forever notify (wake_us = INT64_MAX) is never time-woken (only by
            // a notification). 期限到達で BlockedDelay と timeout 付き BlockedNotify を起床。
            // 無限待ち(wake_us=INT64_MAX)は時刻では起きない（通知のみ）。
            if ((task->state == TaskState::BlockedDelay ||
                 task->state == TaskState::BlockedNotify) && task->wake_us <= now_us_) {
                task->state = TaskState::Ready;
            }
        }

        // Fire any periodic timers due now, before pick_ready, in registration
        // order. Their callbacks (xTaskNotifyGive) mark tasks Ready.
        // 期限の来た周期タイマを pick_ready 前に登録順で発火。コールバック
        // （xTaskNotifyGive）がタスクを Ready にする。
        fire_due_timers(lk);

        // Scenario injection at the new virtual time (unlock to avoid holding
        // the scheduler mutex while topics are touched).
        // 新しい仮想時刻でシナリオ注入（トピック操作中にスケジューラ
        // ミューテックスを保持しないよう一旦解放）。
        if (on_advance_) {
            lk.unlock();
            on_advance_(now_us_);
            lk.lock();
        }
    }

    // No task is executing now (the last grant returned with running_==nullptr).
    // Release the scheduler mutex before stop_all(), which re-locks it
    // (std::mutex is non-recursive — holding it here would self-deadlock).
    // 今どのタスクも実行していない（最後の付与は running_==nullptr で戻った）。
    // stop_all() は m_ を再ロックするので、その前に解放する（std::mutex は
    // 非再帰 — ここで保持したままだと自己デッドロックする）。
    lk.unlock();
    stop_all();
}

void Scheduler::stop_all()
{
    // Unwind every still-parked task (throw StopTask via its blocking primitive)
    // and join its thread, leaving a clean process state — no _Exit needed.
    // 待機中の各タスクを巻き戻し（ブロッキングプリミティブ経由で StopTask を
    // 投げ）スレッドを join する。クリーンなプロセス状態に — _Exit 不要。
    {
        std::unique_lock<std::mutex> lk(m_);
        shutdown_ = true;
        for (Task* task : tasks_) {
            if (!task->exited) {
                grant_and_wait(lk, task);  // wakes in its primitive → throws → exits
            }
        }
    }
    for (Task* task : tasks_) {
        if (task->thread.joinable()) {
            task->thread.join();
        }
    }
}

}  // namespace rtos
}  // namespace sils

// =============================================================================
// FreeRTOS active-surface shims — forward to the single Scheduler instance.
// FreeRTOS 能動面のシム — 唯一の Scheduler インスタンスへ転送。
// =============================================================================

using sils::rtos::Scheduler;
using sils::rtos::Task;

extern "C" {

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char* name,
                                   uint32_t /*stack_depth*/, void* param,
                                   UBaseType_t priority, TaskHandle_t* out_handle,
                                   BaseType_t /*core*/)
{
    TaskHandle_t handle = Scheduler::instance().create(fn, param, priority, name);
    if (out_handle) *out_handle = handle;
    return pdPASS;
}

void vTaskDelete(TaskHandle_t /*handle*/)
{
    // The firmware only ever deletes itself (vTaskDelete(NULL)).
    // 本体は自分自身しか削除しない（vTaskDelete(NULL)）。
    Scheduler::instance().delete_self();
}

void vTaskDelay(TickType_t ticks)
{
    Scheduler& s = Scheduler::instance();
    s.delay_until_us(s.now_us() + static_cast<int64_t>(ticks) * 1000);
}

void vTaskDelayUntil(TickType_t* last_wake, TickType_t period)
{
    // Absolute periodic wake (no drift), matching FreeRTOS: advance *last_wake
    // by period and sleep until that tick. 1 tick = 1 ms = 1000 us.
    // 絶対周期起床（ドリフトなし）、FreeRTOS と同じ: *last_wake を period 分
    // 進め、その tick まで眠る。1 tick = 1 ms = 1000 us。
    *last_wake += period;
    Scheduler::instance().delay_until_us(static_cast<int64_t>(*last_wake) * 1000);
}

TickType_t xTaskGetTickCount(void)
{
    return static_cast<TickType_t>(Scheduler::instance().now_us() / 1000);
}

TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    return Scheduler::instance().current_handle();
}

BaseType_t xTaskNotifyGive(TaskHandle_t handle)
{
    Scheduler::instance().notify_give(static_cast<Task*>(handle));
    return pdTRUE;
}

uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t timeout)
{
    // 1 SILS tick = 1 ms (xTaskGetTickCount = now_us/1000). portMAX_DELAY = wait forever
    // (timeout_us < 0). Honoring the timeout makes a periodic ulTaskNotifyTake poll work
    // in the SILS exactly as on real FreeRTOS (Code Identity).
    // 1 SILS tick=1ms。portMAX_DELAY は無限待ち(<0)。timeout を honor し、周期的な
    // ulTaskNotifyTake ポーリングが実 FreeRTOS と同じく SILS でも機能する（Code Identity）。
    const int64_t timeout_us = (timeout == portMAX_DELAY)
                                   ? -1 : (int64_t)timeout * 1000;
    return Scheduler::instance().notify_take(clear_on_exit != pdFALSE, timeout_us);
}

}  // extern "C"
