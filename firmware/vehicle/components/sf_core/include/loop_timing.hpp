/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file loop_timing.hpp
 * @brief Period and busy-time monitor for a task that wakes at a fixed rate (400 Hz)
 *        一定周期（400 Hz）で実行を開始するタスクの周期・処理時間の監視器
 *
 * Measures two things per task, with the clock the task already reads:
 *   period : time between two consecutive wakes. A value near 1.0 period is healthy; a value
 *            near 2.0 periods means one 400 Hz tick was missed (the task, or the timer that
 *            wakes it, was held up for more than one period).
 *   busy   : time from a wake to the next wait, as seen by the wall clock (so it includes the
 *            time a higher-priority task preempted this one).
 * A wake is "late" when its interval exceeds 1.5 periods: ordinary jitter (a few hundred
 * microseconds either way) is never counted, a missed tick always is.
 *
 * Pure logic (no FreeRTOS, no ESP-IDF): the caller passes the time in microseconds and
 * whether the craft is armed, so the class runs in the host test (test_loop_timing.cpp).
 * Cost per cycle: a few integer operations.
 *
 * タスクごとに 2 つを測る（タスクが既に読んでいる時計を使う）:
 *   周期   : 連続する 2 回の周期の実行開始の間隔。1.0 周期付近なら健全、2.0 周期付近は 400 Hz のティックを
 *            1 つ取りこぼした（タスク、またはタスクに通知するタイマが 1 周期以上止まった）。
 *   処理時間: 実行開始から次の待ちまでの経過時間（壁時計）。上位タスクに割り込まれた時間を含む。
 * 間隔が 1.5 周期を超えて始まった実行開始を「遅れ」と数える: 通常のジッタ（前後数百 µs）は数えず、ティックの
 * 取りこぼしは必ず数える。
 *
 * 純粋なロジック（FreeRTOS・ESP-IDF なし）: 呼び出し側が時刻 [µs] と ARM 中かを渡すので、ホスト試験
 * （test_loop_timing.cpp）で動く。1 周期あたりの費用は整数演算数回。
 *
 * @design detailed_design.md §8 — Loop timing monitor (imu_loop_timing / control_loop_timing)  [OK]
 */

#pragma once

#include <cstdint>

#include "data_types.hpp"

namespace sf {

/// Counts the period and busy time of one periodic task.
/// 周期タスク 1 本の周期と処理時間を数える。
class LoopTimingMonitor {
public:
    /// A wake whose interval exceeds this many periods (as a fraction 3/2) is late.
    /// 間隔がこの周期数（3/2 倍）を超えて始まった実行開始は「遅れ」。
    static constexpr uint32_t kLateNumerator   = 3;
    static constexpr uint32_t kLateDenominator = 2;

    explicit LoopTimingMonitor(uint32_t period_us) : period_us_(period_us) {}

    /// Call right after the task wakes. now_us is the time of the wake; armed is whether the
    /// craft is armed now. Wrap-around of the 32-bit microsecond clock is handled.
    /// タスクが実行を開始した直後に呼ぶ。now_us は実行開始の時刻、armed は今 ARM 中か。32 ビットのマイクロ秒
    /// 時計の桁あふれは扱う。
    void onWake(uint32_t now_us, bool armed)
    {
        armed_ = armed;
        if (has_wake_) {
            countInterval(now_us - last_wake_us_, now_us);
        }
        has_wake_     = true;
        last_wake_us_ = now_us;
        busy_pending_ = true;
        ++stats_.cycles;
        if (armed_) {
            ++stats_.armed_cycles;
        }
    }

    /// Call when the task is about to wait for the next wake (the end of the cycle).
    /// 次の実行開始を待つ直前（周期の終わり）に呼ぶ。
    void onDone(uint32_t now_us)
    {
        if (!busy_pending_) {
            return;
        }
        busy_pending_ = false;
        countBusy(now_us - last_wake_us_);
    }

    /// The counts so far. The caller stamps timestamp before publishing.
    /// ここまでの数え上げ。発行の前に呼び出し側が timestamp を入れる。
    const LoopTiming& result() const { return stats_; }

private:
    void countInterval(uint32_t interval_us, uint32_t wake_us)
    {
        const bool late = interval_us > period_us_ * kLateNumerator / kLateDenominator;
        if (interval_us > stats_.period_max_us) {
            stats_.period_max_us = interval_us;
        }
        if (late) {
            ++stats_.late_count;
            stats_.last_late_us = wake_us;
        }
        if (!armed_) {
            return;
        }
        if (interval_us > stats_.armed_period_max_us) {
            stats_.armed_period_max_us = interval_us;
        }
        if (late) {
            ++stats_.armed_late_count;
        }
    }

    void countBusy(uint32_t busy_us)
    {
        const bool over = busy_us > period_us_;
        if (busy_us > stats_.busy_max_us) {
            stats_.busy_max_us = busy_us;
        }
        if (over) {
            ++stats_.over_budget_count;
        }
        if (!armed_) {
            return;
        }
        if (busy_us > stats_.armed_busy_max_us) {
            stats_.armed_busy_max_us = busy_us;
        }
        if (over) {
            ++stats_.armed_over_budget_count;
        }
    }

    uint32_t   period_us_;
    LoopTiming stats_         = {};
    uint32_t   last_wake_us_  = 0;
    bool       has_wake_      = false;   // a previous wake exists / 前回の実行開始がある
    bool       busy_pending_  = false;   // a wake is waiting for its onDone / onDone を待つ実行開始がある
    bool       armed_         = false;   // armed at the latest wake / 直近の実行開始で ARM 中だったか
};

}  // namespace sf
