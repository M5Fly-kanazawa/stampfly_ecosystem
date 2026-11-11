/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_param_range.cpp
 * @brief Deterministic unit tests for the range check of a float parameter (sf::params::isFloatInRange)
 *        float パラメータの範囲検査（sf::params::isFloatInRange）の決定論ユニットテスト
 *
 * Header-only logic (param_range.hpp) — no ESP-IDF, no NVS. Own TEST/ASSERT framework and main(),
 * same pattern as test_api_arguments.cpp.
 * ヘッダのみのロジック（param_range.hpp）— ESP-IDF・NVS 不要。test_api_arguments.cpp と同じ
 * 自前の TEST/ASSERT フレームワークと main()。
 *
 * @design detailed_design.md §6 — Parameter system                    [OK]
 */

#include <cstdio>
#include <limits>

#include "param_range.hpp"

static constexpr float kNotANumber = std::numeric_limits<float>::quiet_NaN();
static constexpr float kInfinity   = std::numeric_limits<float>::infinity();
// A parameter range [-1, 5] (the sign and the width do not matter to the check).
// パラメータの範囲 [-1, 5]（検査には符号も幅も関係ない）。
static constexpr float kRangeMin = -1.0f;
static constexpr float kRangeMax = 5.0f;

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

using sf::params::isFloatInRange;

TEST(accepts_values_inside_the_range_and_on_its_edges)
{
    ASSERT_TRUE(isFloatInRange(0.0f, kRangeMin, kRangeMax));
    ASSERT_TRUE(isFloatInRange(kRangeMin, kRangeMin, kRangeMax));
    ASSERT_TRUE(isFloatInRange(kRangeMax, kRangeMin, kRangeMax));
}

TEST(rejects_values_outside_the_range)
{
    ASSERT_TRUE(!isFloatInRange(-1.0001f, kRangeMin, kRangeMax));
    ASSERT_TRUE(!isFloatInRange(5.0001f, kRangeMin, kRangeMax));
}

TEST(rejects_nan_which_both_comparisons_of_a_rejection_test_let_through)
{
    // The rejection test `value < min || value > max` is false for a NaN, so it accepts it.
    // `value < min || value > max` の拒否検査は NaN に対して偽になり、NaN を受理してしまう。
    ASSERT_TRUE(!(kNotANumber < kRangeMin || kNotANumber > kRangeMax));
    ASSERT_TRUE(!isFloatInRange(kNotANumber, kRangeMin, kRangeMax));
}

TEST(rejects_infinity_even_when_the_range_has_no_finite_edge)
{
    ASSERT_TRUE(!isFloatInRange(kInfinity, kRangeMin, kRangeMax));
    ASSERT_TRUE(!isFloatInRange(-kInfinity, kRangeMin, kRangeMax));
    // A range whose edges are infinite would contain them by the comparison alone.
    // 端が無限大の範囲は、比較だけなら無限大を含んでしまう。
    ASSERT_TRUE(!isFloatInRange(kInfinity, -kInfinity, kInfinity));
    ASSERT_TRUE(!isFloatInRange(kNotANumber, -kInfinity, kInfinity));
}

int main()
{
    printf("=== sf::params::isFloatInRange unit tests ===\n");

    run_accepts_values_inside_the_range_and_on_its_edges();
    run_rejects_values_outside_the_range();
    run_rejects_nan_which_both_comparisons_of_a_rejection_test_let_through();
    run_rejects_infinity_even_when_the_range_has_no_finite_edge();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
