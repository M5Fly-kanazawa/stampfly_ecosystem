/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_loop_timing.cpp
 * @brief Deterministic unit tests for sf::LoopTimingMonitor (400 Hz period / busy-time counts)
 *        sf::LoopTimingMonitor（400 Hz の周期・処理時間の数え上げ）の決定論ユニットテスト
 *
 * Header-only logic (loop_timing.hpp) — no ESP-IDF, no FreeRTOS. Own TEST/ASSERT framework and
 * main(), same pattern as test_sensor_freshness.cpp.
 * ヘッダのみのロジック（loop_timing.hpp）— ESP-IDF・FreeRTOS 不要。test_sensor_freshness.cpp と
 * 同じ自前の TEST/ASSERT フレームワークと main()。
 *
 * @design detailed_design.md §8 — Loop timing monitor (imu_loop_timing / control_loop_timing)  [OK]
 */

#include <cstdint>
#include <cstdio>

#include "loop_timing.hpp"

// Same value as config::IMU_PERIOD_US (main/config.hpp) — 400 Hz.
// config::IMU_PERIOD_US（main/config.hpp）と同じ値 — 400 Hz。
static constexpr uint32_t kPeriodUs = 2500;

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

// One cycle: wake at wake_us, finish busy_us later.
// 1 周期: wake_us に起き、busy_us 後に終える。
static void cycle(sf::LoopTimingMonitor& monitor, uint32_t wake_us, uint32_t busy_us, bool armed = false)
{
    monitor.onWake(wake_us, armed);
    monitor.onDone(wake_us + busy_us);
}

TEST(steady_400hz_counts_no_late_wake)
{
    sf::LoopTimingMonitor monitor(kPeriodUs);
    for (uint32_t i = 0; i < 1000; ++i) {
        cycle(monitor, 1000000u + i * kPeriodUs, 900);
    }
    const sf::LoopTiming& t = monitor.result();
    ASSERT_TRUE(t.cycles == 1000);
    ASSERT_TRUE(t.late_count == 0);
    ASSERT_TRUE(t.period_max_us == kPeriodUs);
    ASSERT_TRUE(t.busy_max_us == 900);
    ASSERT_TRUE(t.over_budget_count == 0);
    ASSERT_TRUE(t.last_late_us == 0);
}

TEST(jitter_below_one_and_a_half_periods_is_not_late)
{
    sf::LoopTimingMonitor monitor(kPeriodUs);
    cycle(monitor, 0, 500);
    cycle(monitor, 3749, 500);     // 3749 us after the previous wake: still not late / まだ遅れではない
    ASSERT_TRUE(monitor.result().late_count == 0);
    ASSERT_TRUE(monitor.result().period_max_us == 3749);
}

TEST(a_missed_tick_is_late_and_remembers_when)
{
    sf::LoopTimingMonitor monitor(kPeriodUs);
    cycle(monitor, 0, 500);
    cycle(monitor, 2500, 500);
    cycle(monitor, 7500, 500);     // the tick at 5000 was missed: interval 5000 / 5000 のティックを取りこぼし
    cycle(monitor, 10000, 500);
    const sf::LoopTiming& t = monitor.result();
    ASSERT_TRUE(t.late_count == 1);
    ASSERT_TRUE(t.period_max_us == 5000);
    ASSERT_TRUE(t.last_late_us == 7500);
}

TEST(interval_of_exactly_one_and_a_half_periods_is_not_late)
{
    sf::LoopTimingMonitor monitor(kPeriodUs);
    cycle(monitor, 0, 100);
    cycle(monitor, 3750, 100);     // 3750 = 1.5 periods: the boundary is "longer than" / 境界は「超える」
    ASSERT_TRUE(monitor.result().late_count == 0);
    cycle(monitor, 7501, 100);     // 3751 us
    ASSERT_TRUE(monitor.result().late_count == 1);
}

TEST(busy_longer_than_a_period_is_over_budget)
{
    sf::LoopTimingMonitor monitor(kPeriodUs);
    cycle(monitor, 0, 2500);       // exactly one period: within budget / ちょうど 1 周期: 上限の内側
    cycle(monitor, 2500, 2501);    // one microsecond over / 1 µs 超過
    cycle(monitor, 5000, 4800);
    const sf::LoopTiming& t = monitor.result();
    ASSERT_TRUE(t.over_budget_count == 2);
    ASSERT_TRUE(t.busy_max_us == 4800);
}

TEST(armed_counts_are_kept_apart_from_ground_idle)
{
    sf::LoopTimingMonitor monitor(kPeriodUs);
    cycle(monitor, 0, 500, false);
    cycle(monitor, 10000, 500, false);          // a 10 ms stall on the ground / 地上での 10 ms の停止
    cycle(monitor, 12500, 500, true);
    cycle(monitor, 17500, 3000, true);          // armed: a missed tick and an over-budget cycle
    const sf::LoopTiming& t = monitor.result();
    ASSERT_TRUE(t.cycles == 4);
    ASSERT_TRUE(t.armed_cycles == 2);
    ASSERT_TRUE(t.late_count == 2);
    ASSERT_TRUE(t.armed_late_count == 1);
    ASSERT_TRUE(t.period_max_us == 10000);
    ASSERT_TRUE(t.armed_period_max_us == 5000);
    ASSERT_TRUE(t.over_budget_count == 1);
    ASSERT_TRUE(t.armed_over_budget_count == 1);
    ASSERT_TRUE(t.armed_busy_max_us == 3000);
}

TEST(the_arm_edge_stall_is_counted_as_armed)
{
    sf::LoopTimingMonitor monitor(kPeriodUs);
    cycle(monitor, 0, 500, false);
    cycle(monitor, 2500, 500, false);
    cycle(monitor, 52500, 500, true);           // 50 ms after the previous wake, first armed wake / 前回から 50 ms 後、最初の ARM 中の起床
    ASSERT_TRUE(monitor.result().armed_period_max_us == 50000);
    ASSERT_TRUE(monitor.result().armed_late_count == 1);
}

TEST(uint32_clock_wrap_is_handled)
{
    sf::LoopTimingMonitor monitor(kPeriodUs);
    const uint32_t before_wrap = 0xFFFFFFFFu - 1000u;    // 1001 us before the wrap / 桁あふれの 1001 µs 前
    cycle(monitor, before_wrap, 500);
    cycle(monitor, before_wrap + kPeriodUs, 500);        // wraps to 1499 / 1499 に巻き戻る
    ASSERT_TRUE(monitor.result().period_max_us == kPeriodUs);
    ASSERT_TRUE(monitor.result().late_count == 0);
}

TEST(done_without_a_wake_is_ignored)
{
    sf::LoopTimingMonitor monitor(kPeriodUs);
    monitor.onDone(5000);                       // the first loop pass has no previous wake / 最初の周回には前の起床がない
    cycle(monitor, 10000, 700);
    monitor.onDone(99999);                      // a second onDone for the same wake changes nothing / 同じ起床への 2 回目は何も変えない
    ASSERT_TRUE(monitor.result().busy_max_us == 700);
}

int main()
{
    printf("=== LoopTimingMonitor unit tests ===\n");
    run_steady_400hz_counts_no_late_wake();
    run_jitter_below_one_and_a_half_periods_is_not_late();
    run_a_missed_tick_is_late_and_remembers_when();
    run_interval_of_exactly_one_and_a_half_periods_is_not_late();
    run_busy_longer_than_a_period_is_over_budget();
    run_armed_counts_are_kept_apart_from_ground_idle();
    run_the_arm_edge_stall_is_counted_as_armed();
    run_uint32_clock_wrap_is_handled();
    run_done_without_a_wake_is_ignored();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
