/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file step_timing.hpp
 * @brief Per-step time statistics of one 400 Hz cycle, and which observation updates ran in the
 *        cycles that overran the budget
 *        400 Hz の 1 周期の工程別の所要時間の統計と、周期超過（1 周期の処理時間が上限を超えた周期）でどの観測更新が走ったか
 *
 * LoopTimingMonitor says HOW LONG a cycle took; this class says WHERE the time went. The task
 * cuts its cycle into steps (read, predict, one step per observation kind, the rest) and hands
 * each step's duration to recordStep(). Per step it keeps the number of runs, the total and the
 * longest duration; the average is total / runs, computed by the reader (no division on the
 * 400 Hz path). For the observation kinds the task also calls noteUpdate() once per consumed
 * sample, so endCycle() can keep (a) the largest number of updates of one kind in a single cycle
 * and (b) for the cycles longer than the budget, a count per combination of the kinds that ran
 * (a bit set: bit k = kind k ran in that cycle). A window is read with the accessors and then
 * cleared with reset().
 *
 * Pure logic (no FreeRTOS, no ESP-IDF): the caller passes the durations in microseconds, so the
 * class runs in the host test (test_step_timing.cpp). Only one task touches an instance, so it
 * needs no lock. Cost per cycle: integer additions and comparisons; no allocation.
 *
 * LoopTimingMonitor は周期が「どれだけ」かかったかを言う。このクラスは時間が「どこで」使われたかを言う。
 * タスクは周期を工程（読み出し・予測・観測の種類ごとの更新・残り）に区切り、各工程の所要時間を
 * recordStep() に渡す。工程ごとに実行回数・合計・最大を持ち、平均（合計 / 回数）は読む側が出す
 * （400 Hz の経路に除算を入れない）。観測の種類については、タスクが標本 1 個ごとに noteUpdate() を呼ぶ
 * ので、endCycle() は (a) 1 周期に同じ種類を処理した個数の最大と、(b) 周期超過の周期について、走った
 * 種類の組み合わせ（ビットの組: ビット k = 種類 k がその周期に走った）ごとの回数を持てる。窓の終わりに
 * アクセサで読み、reset() で消す。
 *
 * 純粋なロジック（FreeRTOS・ESP-IDF なし）: 呼び出し側が所要時間 [µs] を渡すので、ホスト試験
 * （test_step_timing.cpp）で動く。1 つのタスクだけが触るのでロック不要。1 周期あたりの費用は整数の加算と
 * 比較。動的確保なし。
 *
 * @design detailed_design.md §8 — Loop timing monitor, step timing (imu_task.cpp, 60 s log)  [OK]
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace sf {

/// Run count, total and longest duration of one step within a window.
/// 窓の中の 1 つの工程の実行回数・合計・最大。
struct StepStat {
    uint32_t count    = 0;   ///< runs / 実行回数
    uint64_t total_us = 0;   ///< sum of durations [us] / 所要時間の合計 [µs]
    uint32_t max_us   = 0;   ///< longest duration [us] / 最長の所要時間 [µs]
};

/// One entry of the over-budget breakdown: a combination of observation kinds and its cycle count.
/// 周期超過の内訳の 1 項目: 観測の種類の組み合わせと、その周期の数。
struct KindCombination {
    uint32_t mask  = 0;   ///< bit k set = kind k ran / ビット k = 種類 k が走った
    uint32_t count = 0;   ///< over-budget cycles with exactly this combination / ちょうどこの組み合わせだった超過周期の数
};

/// Step statistics of one periodic task (kMaxSteps steps, kMaxKinds observation kinds).
/// 周期タスク 1 本の工程別統計（工程 kMaxSteps 個、観測の種類 kMaxKinds 個）。
class StepTimingStats {
public:
    static constexpr size_t kMaxSteps = 8;                    ///< steps per cycle / 1 周期の工程数の上限
    static constexpr size_t kMaxKinds = 5;                    ///< observation kinds / 観測の種類数の上限
    static constexpr size_t kCombinations = size_t{1} << kMaxKinds;   ///< possible kind sets / 種類の組の総数

    /// Add one run of a step. An out-of-range step is ignored.
    /// 工程 1 回分を加える。範囲外の工程は無視する。
    void recordStep(size_t step, uint32_t duration_us)
    {
        if (step >= kMaxSteps) {
            return;
        }
        StepStat& stat = steps_[step];
        ++stat.count;
        stat.total_us += duration_us;
        if (duration_us > stat.max_us) {
            stat.max_us = duration_us;
        }
        cycle_step_sum_us_ += duration_us;
    }

    /// Count one consumed sample of an observation kind in the current cycle.
    /// 現在の周期で、観測の種類の標本を 1 個処理したと数える。
    void noteUpdate(size_t kind)
    {
        if (kind < kMaxKinds) {
            ++cycle_updates_[kind];
        }
    }

    /// Close the cycle. cycle_us = the whole cycle; the part not covered by recordStep() goes to
    /// rest_step. A cycle longer than budget_us adds one to its kind combination.
    /// 周期を閉じる。cycle_us = 周期全体。recordStep() で数えなかった残りは rest_step へ入れる。
    /// 処理時間の上限 budget_us を超えた周期（周期超過）は、その種類の組み合わせに 1 を足す。
    void endCycle(uint32_t cycle_us, uint32_t budget_us, size_t rest_step)
    {
        const uint32_t covered = cycle_step_sum_us_ > UINT32_MAX
                                     ? UINT32_MAX : static_cast<uint32_t>(cycle_step_sum_us_);
        recordStep(rest_step, cycle_us > covered ? cycle_us - covered : 0u);

        uint32_t mask = 0;
        for (size_t kind = 0; kind < kMaxKinds; ++kind) {
            if (cycle_updates_[kind] > 0) {
                mask |= uint32_t{1} << kind;
            }
            if (cycle_updates_[kind] > max_updates_[kind]) {
                max_updates_[kind] = cycle_updates_[kind];
            }
            cycle_updates_[kind] = 0;
        }
        if (cycle_us > budget_us) {
            ++over_budget_cycles_;
            ++combinations_[mask];
        }
        cycle_step_sum_us_ = 0;
        ++cycles_;
    }

    /// Clear the window (the per-cycle counters of a cycle in progress are kept).
    /// 窓を消す（進行中の周期の周期内カウンタは残す）。
    void reset()
    {
        for (StepStat& stat : steps_) {
            stat = StepStat{};
        }
        for (size_t kind = 0; kind < kMaxKinds; ++kind) {
            max_updates_[kind] = 0;
        }
        for (uint32_t& count : combinations_) {
            count = 0;
        }
        over_budget_cycles_ = 0;
        cycles_             = 0;
    }

    const StepStat& step(size_t index) const { return steps_[index < kMaxSteps ? index : 0]; }
    uint32_t cycles() const { return cycles_; }
    uint32_t overBudgetCycles() const { return over_budget_cycles_; }
    uint32_t maxUpdates(size_t kind) const { return kind < kMaxKinds ? max_updates_[kind] : 0u; }
    uint32_t combinationCount(uint32_t mask) const
    {
        return mask < kCombinations ? combinations_[mask] : 0u;
    }

    /// Average duration of a step [us] (0 when it never ran). Division: call it off the 400 Hz path.
    /// 工程の平均所要時間 [µs]（一度も走らなければ 0）。除算を含むので 400 Hz の経路の外で呼ぶ。
    static uint32_t averageUs(const StepStat& stat)
    {
        return stat.count == 0 ? 0u : static_cast<uint32_t>(stat.total_us / stat.count);
    }

    /// Fill out[] with the over-budget combinations that occurred, most frequent first (ties:
    /// smaller mask first); at most capacity entries. Returns the number written.
    /// 起きた超過の組み合わせを多い順（同数なら mask の小さい方が先）に out[] へ入れる。最大 capacity 個。
    /// 書いた個数を返す。
    size_t largestCombinations(KindCombination* out, size_t capacity) const
    {
        bool taken[kCombinations] = {};
        size_t written = 0;
        while (written < capacity) {
            uint32_t best_mask  = 0;
            uint32_t best_count = 0;
            for (uint32_t mask = 0; mask < kCombinations; ++mask) {
                if (!taken[mask] && combinations_[mask] > best_count) {
                    best_mask  = mask;
                    best_count = combinations_[mask];
                }
            }
            if (best_count == 0) {
                break;
            }
            taken[best_mask] = true;
            out[written++]   = KindCombination{best_mask, best_count};
        }
        return written;
    }

private:
    StepStat steps_[kMaxSteps]               = {};
    uint32_t cycle_updates_[kMaxKinds]       = {};   // samples consumed in the current cycle / 現在の周期で処理した標本数
    uint32_t max_updates_[kMaxKinds]         = {};   // largest of the above in the window / 窓の中での最大
    uint32_t combinations_[kCombinations]    = {};   // over-budget cycles per kind set / 種類の組ごとの超過周期数
    uint64_t cycle_step_sum_us_              = 0;    // durations recorded in the current cycle / 現在の周期で記録した所要時間の和
    uint32_t over_budget_cycles_             = 0;
    uint32_t cycles_                         = 0;
};

}  // namespace sf
