/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_heavy_kind_select.cpp
 * @brief Arrival-pattern test of the "one heavy observation kind per cycle" rule (SF_EXP_ONE_KIND_PER_CYCLE)
 *        「1 周期に重い観測は 1 種類まで」規則（SF_EXP_ONE_KIND_PER_CYCLE）の到着パターン試験
 *
 * The rule (selectHeavyKind / nextDeferredMask) is taken VERBATIM from tasks/imu_task.cpp: the Makefile
 * extracts the region between the SF_PURE_BEGIN / SF_PURE_END markers into imu_heavy_kind_select.gen.hpp
 * (imu_task.cpp itself needs ESP-IDF and cannot be compiled on the host). The queues are modelled with
 * depth 2 and drop-the-new-sample-when-full, as sf::TopicQueue does.
 * 規則（selectHeavyKind / nextDeferredMask）は tasks/imu_task.cpp から「そのまま」使う: Makefile が
 * SF_PURE_BEGIN / SF_PURE_END の間を imu_heavy_kind_select.gen.hpp に取り出す（imu_task.cpp 自体は
 * ESP-IDF が要りホストではコンパイルできない）。キューは深さ 2、満杯なら新しい標本を捨てる
 * （sf::TopicQueue と同じ）でモデル化する。
 *
 * Arrivals: flow 100 Hz, downward ToF 30 Hz, baro 50 Hz, mag 25 Hz, cycle 400 Hz, 10 s, every phase, with and
 * without one-cycle arrival jitter. A sample that arrives in cycle k is visible from cycle k.
 * 到着: フロー 100 Hz、下向き ToF 30 Hz、気圧 50 Hz、地磁気 25 Hz、周期 400 Hz、10 秒、すべての位相、
 * 到着に 1 周期のゆらぎがある場合とない場合。周期 k に着いた標本は周期 k から見える。
 */

#include <cstdint>
#include <cstdio>

#include "imu_heavy_kind_select.gen.hpp"

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

static constexpr int kCycles = 4000;     // 10 s at 400 Hz / 400 Hz で 10 秒
static constexpr int kQueueDepth = 2;    // sf core: Topic<..., Queue, 2>
static constexpr int kKinds = 4;         // 0 = ToF, 1 = flow, 2 = mag, 3 = baro (the bit of heavy kind k is 1 << (k - 1))

/// A sensor queue holding the arrival cycle of each waiting sample.
/// 待っている各標本の到着周期を持つセンサのキュー。
struct Queue {
    int arrival[kQueueDepth] = {};
    int count = 0;
};

struct Result {
    int dropped[kKinds] = {};
    int max_delay[kKinds] = {};    // cycles from arrival to processing / 到着から処理までの周期数
    int processed[kKinds] = {};
    int heavy_in_cycle_max = 0;    // most heavy kinds (ToF counted) processed in one cycle / 1 周期に処理した重い種類の最大数
};

/// Deterministic LCG (no <random>, so the result is the same on every host).
/// 決定論の線形合同法（<random> を使わない。どのホストでも同じ結果）。
static uint32_t nextRandom(uint32_t& state)
{
    state = state * 1664525u + 1013904223u;
    return state >> 16;
}

static void drain(Queue& queue, int cycle, int kind, Result& result)
{
    for (int i = 0; i < queue.count; ++i) {
        const int delay = cycle - queue.arrival[i];
        if (delay > result.max_delay[kind]) {
            result.max_delay[kind] = delay;
        }
        ++result.processed[kind];
    }
    queue.count = 0;
}

/// Run kCycles cycles. period_x100[k]: arrival period of kind k in cycles x100 (the ToF 30 Hz is 1333);
/// phase[k]: first arrival cycle; jitter: each arrival is delayed by 0 or 1 cycle, pseudo-randomly.
/// kCycles 周期を走らせる。period_x100[k]: 種類 k の到着周期（周期数 x100。ToF 30 Hz は 1333）。
/// phase[k]: 最初の到着周期。jitter: 各到着が 0 か 1 周期、疑似乱数で遅れる。
/// prefer_deferred: 実際の規則（true）か、単純な優先順位だけ（false）。
static Result simulate(const int period_x100[kKinds], const int phase[kKinds], bool jitter)
{
    Queue queues[kKinds];
    Result result;
    int next_x100[kKinds];
    for (int k = 0; k < kKinds; ++k) {
        next_x100[k] = phase[k] * 100;
    }
    uint32_t rng = 12345u;
    int pending_count[kKinds] = {};   // arrivals pushed to the next cycle by the jitter / ゆらぎで次の周期に送られた到着

    for (int cycle = 0; cycle < kCycles; ++cycle) {
        for (int k = 0; k < kKinds; ++k) {
            int arrivals_now = pending_count[k];
            pending_count[k] = 0;
            while (next_x100[k] <= cycle * 100) {
                next_x100[k] += period_x100[k];
                if (jitter && (nextRandom(rng) & 1u) != 0u) {
                    ++pending_count[k];
                } else {
                    ++arrivals_now;
                }
            }
            for (int i = 0; i < arrivals_now; ++i) {
                if (queues[k].count < kQueueDepth) {
                    queues[k].arrival[queues[k].count++] = cycle;
                } else {
                    ++result.dropped[k];
                }
            }
        }

        const bool tof_processed = queues[0].count > 0;
        if (tof_processed) {
            drain(queues[0], cycle, 0, result);
        }
        uint8_t ready = 0;
        for (int k = 1; k < kKinds; ++k) {
            if (queues[k].count > 0) {
                ready = static_cast<uint8_t>(ready | (1u << (k - 1)));
            }
        }
        const uint8_t picked = selectHeavyKind(tof_processed, ready);
        int heavy = tof_processed ? 1 : 0;
        for (int k = 1; k < kKinds; ++k) {
            if ((picked & (1u << (k - 1))) != 0u) {
                drain(queues[k], cycle, k, result);
                ++heavy;
            }
        }
        if (heavy > result.heavy_in_cycle_max) {
            result.heavy_in_cycle_max = heavy;
        }
    }
    return result;
}

/// Worst case over every phase combination. period 400 Hz / rate.
/// すべての位相の組み合わせでの最悪値。
struct Worst {
    int dropped_total = 0;
    int max_delay[kKinds] = {};
    int heavy_in_cycle_max = 0;
    int combinations = 0;
};

static Worst sweepPhases(bool jitter)
{
    const int period_x100[kKinds] = {1333, 400, 1600, 800};   // ToF 30 Hz, flow 100 Hz, mag 25 Hz, baro 50 Hz
    Worst worst;
    for (int tof = 0; tof < 14; ++tof) {
        for (int flow = 0; flow < 4; ++flow) {
            for (int mag = 0; mag < 16; ++mag) {
                for (int baro = 0; baro < 8; ++baro) {
                    const int phase[kKinds] = {tof, flow, mag, baro};
                    const Result r = simulate(period_x100, phase, jitter);
                    ++worst.combinations;
                    for (int k = 0; k < kKinds; ++k) {
                        worst.dropped_total += r.dropped[k];
                        if (r.max_delay[k] > worst.max_delay[k]) {
                            worst.max_delay[k] = r.max_delay[k];
                        }
                    }
                    if (r.heavy_in_cycle_max > worst.heavy_in_cycle_max) {
                        worst.heavy_in_cycle_max = r.heavy_in_cycle_max;
                    }
                }
            }
        }
    }
    return worst;
}

static void report(const char* label, const Worst& w)
{
    printf("\n    %-34s combos=%d drops=%d  max delay [cycles]: tof=%d flow=%d mag=%d baro=%d  max heavy/cycle=%d\n      ",
           label, w.combinations, w.dropped_total, w.max_delay[0], w.max_delay[1], w.max_delay[2], w.max_delay[3],
           w.heavy_in_cycle_max);
}

TEST(select_follows_the_priority_and_never_passes_the_tof)
{
    ASSERT_TRUE(selectHeavyKind(true, kHeavyFlow | kHeavyMag | kHeavyBaro) == 0);   // ToF processed: none / ToF 処理済み: なし
    ASSERT_TRUE(selectHeavyKind(false, 0) == 0);
    ASSERT_TRUE(selectHeavyKind(false, kHeavyFlow | kHeavyMag | kHeavyBaro) == kHeavyFlow);
    ASSERT_TRUE(selectHeavyKind(false, kHeavyMag | kHeavyBaro) == kHeavyMag);
    ASSERT_TRUE(selectHeavyKind(false, kHeavyBaro) == kHeavyBaro);
}

TEST(no_sample_is_dropped_and_the_tof_is_never_late_without_jitter)
{
    const Worst w = sweepPhases(false);
    report("no jitter", w);
    ASSERT_TRUE(w.dropped_total == 0);
    ASSERT_TRUE(w.max_delay[0] == 0);
    ASSERT_TRUE(w.heavy_in_cycle_max == 1);
}

TEST(no_sample_is_dropped_and_the_tof_is_never_late_with_jitter)
{
    const Worst w = sweepPhases(true);
    report("jitter 0 or 1 cycle", w);
    ASSERT_TRUE(w.dropped_total == 0);
    ASSERT_TRUE(w.max_delay[0] == 0);
    ASSERT_TRUE(w.heavy_in_cycle_max == 1);
}

TEST(the_delay_stays_far_below_the_queue_depth_margin)
{
    // A depth-2 queue holds a sample for 2 arrival periods before the next arrival would be dropped:
    // flow 8 cycles, mag 32, baro 16. Require at most a quarter of that.
    // 深さ 2 のキューは、次の到着が捨てられるまで到着周期 2 つ分標本を持てる: フロー 8 周期・地磁気 32・気圧 16。
    // その 4 分の 1 以下を求める。
    const Worst w = sweepPhases(true);
    ASSERT_TRUE(w.max_delay[1] <= 2 && w.max_delay[2] <= 8 && w.max_delay[3] <= 4);
}

int main()
{
    printf("=== One-heavy-kind-per-cycle rule: arrival-pattern tests ===\n");
    run_select_follows_the_priority_and_never_passes_the_tof();
    run_no_sample_is_dropped_and_the_tof_is_never_late_without_jitter();
    run_no_sample_is_dropped_and_the_tof_is_never_late_with_jitter();
    run_the_delay_stays_far_below_the_queue_depth_margin();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
