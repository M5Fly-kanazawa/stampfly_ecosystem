/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_step_timing.cpp
 * @brief Deterministic unit tests for sf::StepTimingStats (per-step time of a 400 Hz cycle)
 *        sf::StepTimingStats（400 Hz 周期の工程別の所要時間）の決定論ユニットテスト
 *
 * Header-only logic (step_timing.hpp) — no ESP-IDF, no FreeRTOS. Own TEST/ASSERT framework and
 * main(), same pattern as test_loop_timing.cpp.
 * ヘッダのみのロジック（step_timing.hpp）— ESP-IDF・FreeRTOS 不要。test_loop_timing.cpp と同じ
 * 自前の TEST/ASSERT フレームワークと main()。
 *
 * @design detailed_design.md §8 — Loop timing monitor, step timing (imu_task.cpp, 60 s log)  [OK]
 */

#include <cstdint>
#include <cstdio>

#include "step_timing.hpp"

// Same value as config::IMU_PERIOD_US (main/config.hpp) — the cycle budget at 400 Hz.
// config::IMU_PERIOD_US（main/config.hpp）と同じ値 — 400 Hz の周期の予算。
static constexpr uint32_t kBudgetUs = 2500;

// Step and kind numbers used by the tests (the task defines its own).
// 試験で使う工程と種類の番号（タスクは自前の番号を持つ）。
static constexpr size_t kStepRead = 0;
static constexpr size_t kStepTof  = 1;
static constexpr size_t kStepRest = 7;
static constexpr size_t kKindTof  = 0;
static constexpr size_t kKindFlow = 2;

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-46s ", #name); \
        try { test_##name(); tests_passed++; printf("PASS\n"); } \
        catch (...) { tests_failed++; printf("FAIL\n"); } \
    } \
    static void test_##name()

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        printf("FAIL: %s:%d: condition false\n", __FILE__, __LINE__); \
        throw 1; \
    }

TEST(count_total_and_max_per_step)
{
    sf::StepTimingStats stats;
    stats.recordStep(kStepRead, 200);
    stats.recordStep(kStepRead, 400);
    stats.recordStep(kStepRead, 300);
    ASSERT_TRUE(stats.step(kStepRead).count == 3);
    ASSERT_TRUE(stats.step(kStepRead).total_us == 900);
    ASSERT_TRUE(stats.step(kStepRead).max_us == 400);
    ASSERT_TRUE(sf::StepTimingStats::averageUs(stats.step(kStepRead)) == 300);
    ASSERT_TRUE(stats.step(kStepTof).count == 0);
    ASSERT_TRUE(sf::StepTimingStats::averageUs(stats.step(kStepTof)) == 0);   // never ran / 一度も走らない
}

TEST(out_of_range_step_and_kind_are_ignored)
{
    sf::StepTimingStats stats;
    stats.recordStep(sf::StepTimingStats::kMaxSteps, 500);
    stats.noteUpdate(sf::StepTimingStats::kMaxKinds);
    stats.endCycle(100, kBudgetUs, kStepRest);
    ASSERT_TRUE(stats.step(kStepRead).count == 0);
    ASSERT_TRUE(stats.step(kStepRest).total_us == 100);   // nothing was covered / 何も数えていない
    ASSERT_TRUE(stats.combinationCount(0) == 0);
}

TEST(rest_is_the_cycle_minus_the_recorded_steps)
{
    sf::StepTimingStats stats;
    stats.recordStep(kStepRead, 200);
    stats.recordStep(kStepTof, 300);
    stats.endCycle(1000, kBudgetUs, kStepRest);
    ASSERT_TRUE(stats.step(kStepRest).total_us == 500);
    ASSERT_TRUE(stats.step(kStepRest).count == 1);

    // The next cycle starts from zero: the previous sum does not leak.
    // 次の周期はゼロから始まる: 前の和が持ち越されない。
    stats.recordStep(kStepRead, 100);
    stats.endCycle(400, kBudgetUs, kStepRest);
    ASSERT_TRUE(stats.step(kStepRest).total_us == 500 + 300);
    ASSERT_TRUE(stats.step(kStepRest).max_us == 500);
    ASSERT_TRUE(stats.cycles() == 2);
}

TEST(rest_never_goes_negative)
{
    sf::StepTimingStats stats;
    stats.recordStep(kStepRead, 900);
    stats.endCycle(800, kBudgetUs, kStepRest);          // steps longer than the cycle (clock skew) / 工程が周期より長い
    ASSERT_TRUE(stats.step(kStepRest).total_us == 0);
}

TEST(over_budget_cycles_are_counted_by_kind_combination)
{
    sf::StepTimingStats stats;
    // Slow cycle with flow + tof / フロー + ToF で遅い周期
    stats.noteUpdate(kKindFlow);
    stats.noteUpdate(kKindTof);
    stats.endCycle(3000, kBudgetUs, kStepRest);
    // Slow cycle with nothing observed / 何も観測しない遅い周期
    stats.endCycle(2600, kBudgetUs, kStepRest);
    // Fast cycle with flow: not an over-budget cycle / 速い周期（超過に数えない）
    stats.noteUpdate(kKindFlow);
    stats.endCycle(1200, kBudgetUs, kStepRest);
    // Exactly on the budget is not over / ちょうど予算は超過ではない
    stats.noteUpdate(kKindTof);
    stats.endCycle(kBudgetUs, kBudgetUs, kStepRest);
    // Slow cycle with flow + tof again / もう一度フロー + ToF
    stats.noteUpdate(kKindTof);
    stats.noteUpdate(kKindFlow);
    stats.endCycle(4000, kBudgetUs, kStepRest);

    const uint32_t flow_and_tof = (1u << kKindTof) | (1u << kKindFlow);
    ASSERT_TRUE(stats.overBudgetCycles() == 3);
    ASSERT_TRUE(stats.combinationCount(flow_and_tof) == 2);
    ASSERT_TRUE(stats.combinationCount(0) == 1);
    ASSERT_TRUE(stats.combinationCount(1u << kKindFlow) == 0);
}

TEST(several_samples_of_one_kind_count_once_in_the_set_and_raise_the_maximum)
{
    sf::StepTimingStats stats;
    stats.noteUpdate(kKindFlow);
    stats.noteUpdate(kKindFlow);
    stats.noteUpdate(kKindFlow);
    stats.endCycle(3000, kBudgetUs, kStepRest);
    stats.noteUpdate(kKindFlow);
    stats.endCycle(1000, kBudgetUs, kStepRest);
    ASSERT_TRUE(stats.maxUpdates(kKindFlow) == 3);
    ASSERT_TRUE(stats.maxUpdates(kKindTof) == 0);
    ASSERT_TRUE(stats.combinationCount(1u << kKindFlow) == 1);   // one cycle, not three / 3 周期ではなく 1 周期
}

TEST(reset_clears_the_window)
{
    sf::StepTimingStats stats;
    stats.recordStep(kStepRead, 200);
    stats.noteUpdate(kKindTof);
    stats.endCycle(3000, kBudgetUs, kStepRest);
    stats.reset();
    ASSERT_TRUE(stats.cycles() == 0);
    ASSERT_TRUE(stats.overBudgetCycles() == 0);
    ASSERT_TRUE(stats.step(kStepRead).count == 0);
    ASSERT_TRUE(stats.step(kStepRest).count == 0);
    ASSERT_TRUE(stats.maxUpdates(kKindTof) == 0);
    ASSERT_TRUE(stats.combinationCount(1u << kKindTof) == 0);
    sf::KindCombination out[4];
    ASSERT_TRUE(stats.largestCombinations(out, 4) == 0);
}

TEST(largest_combinations_are_sorted_and_limited)
{
    sf::StepTimingStats stats;
    const uint32_t tof_only  = 1u << kKindTof;
    const uint32_t flow_only = 1u << kKindFlow;
    const uint32_t both      = tof_only | flow_only;
    for (int i = 0; i < 5; ++i) { stats.noteUpdate(kKindFlow); stats.noteUpdate(kKindTof); stats.endCycle(3000, kBudgetUs, kStepRest); }
    for (int i = 0; i < 2; ++i) { stats.noteUpdate(kKindTof); stats.endCycle(3000, kBudgetUs, kStepRest); }
    for (int i = 0; i < 2; ++i) { stats.noteUpdate(kKindFlow); stats.endCycle(3000, kBudgetUs, kStepRest); }
    stats.endCycle(3000, kBudgetUs, kStepRest);

    sf::KindCombination out[8];
    const size_t all = stats.largestCombinations(out, 8);
    ASSERT_TRUE(all == 4);
    ASSERT_TRUE(out[0].mask == both && out[0].count == 5);
    ASSERT_TRUE(out[1].mask == tof_only && out[1].count == 2);     // tie: smaller mask first / 同数は mask の小さい方
    ASSERT_TRUE(out[2].mask == flow_only && out[2].count == 2);
    ASSERT_TRUE(out[3].mask == 0 && out[3].count == 1);

    const size_t two = stats.largestCombinations(out, 2);
    ASSERT_TRUE(two == 2);
    ASSERT_TRUE(out[1].mask == tof_only);
}

int main()
{
    printf("=== StepTimingStats unit tests ===\n");
    run_count_total_and_max_per_step();
    run_out_of_range_step_and_kind_are_ignored();
    run_rest_is_the_cycle_minus_the_recorded_steps();
    run_rest_never_goes_negative();
    run_over_budget_cycles_are_counted_by_kind_combination();
    run_several_samples_of_one_kind_count_once_in_the_set_and_raise_the_maximum();
    run_reset_clears_the_window();
    run_largest_combinations_are_sorted_and_limited();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
