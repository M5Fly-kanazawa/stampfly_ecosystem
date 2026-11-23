/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_api_arguments.cpp
 * @brief Deterministic unit tests for the API argument checks (sf::api_arguments) and sf::math::wrapAngleRad
 *        API の引数検査（sf::api_arguments）と sf::math::wrapAngleRad の決定論ユニットテスト
 *
 * Header-only logic (api_arguments.hpp, sf_math.hpp) — no ESP-IDF, no FreeRTOS. Own TEST/ASSERT
 * framework and main(), same pattern as test_sensor_freshness.cpp.
 * ヘッダのみのロジック（api_arguments.hpp、sf_math.hpp）— ESP-IDF・FreeRTOS 不要。
 * test_sensor_freshness.cpp と同じ自前の TEST/ASSERT フレームワークと main()。
 *
 * @design requirements.md §7 — コマンド受信 (RC入力、API) / TelloAPI   [OK]
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "api_arguments.hpp"
#include "sf_math.hpp"

static constexpr float kNotANumber = std::numeric_limits<float>::quiet_NaN();
static constexpr float kInfinity   = std::numeric_limits<float>::infinity();
// Tolerance of the angle comparisons [rad] (single precision, values up to 2*pi).
// 角度の比較の許容差 [rad]（単精度、2*pi までの値）。
static constexpr float kAngleToleranceRad = 1e-5f;

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

using sf::api_arguments::allFinite;
using sf::api_arguments::isRotateAngleInRange;
using sf::api_arguments::isSysidRangeValid;

TEST(all_finite_accepts_ordinary_numbers)
{
    ASSERT_TRUE(allFinite({0.0f}));
    ASSERT_TRUE(allFinite({-120.5f, 0.0f, 300.0f, 1e30f}));
}

TEST(all_finite_rejects_nan_and_infinity_in_any_position)
{
    ASSERT_TRUE(!allFinite({kNotANumber}));
    ASSERT_TRUE(!allFinite({kInfinity}));
    ASSERT_TRUE(!allFinite({-kInfinity}));
    ASSERT_TRUE(!allFinite({1.0f, 2.0f, kNotANumber}));
    ASSERT_TRUE(!allFinite({kInfinity, 2.0f, 3.0f}));
}

TEST(float_parsing_of_overflowing_text_is_rejected)
{
    // sscanf("%f") of "1e39" gives +inf in single precision: the case that made inf * 0 = NaN.
    // "1e39" の sscanf("%f") は単精度で +inf になる: inf * 0 = NaN の原因になった入力。
    float parsed = 0.0f;
    ASSERT_TRUE(std::sscanf("1e39", "%f", &parsed) == 1);
    ASSERT_TRUE(!allFinite({parsed}));
    ASSERT_TRUE(std::sscanf("nan", "%f", &parsed) == 1);
    ASSERT_TRUE(!allFinite({parsed}));
}

TEST(rotate_angle_range_is_one_to_three_hundred_sixty)
{
    ASSERT_TRUE(isRotateAngleInRange(1.0f));
    ASSERT_TRUE(isRotateAngleInRange(90.0f));
    ASSERT_TRUE(isRotateAngleInRange(360.0f));
    ASSERT_TRUE(!isRotateAngleInRange(0.0f));
    ASSERT_TRUE(!isRotateAngleInRange(0.99f));
    ASSERT_TRUE(!isRotateAngleInRange(360.01f));
    ASSERT_TRUE(!isRotateAngleInRange(-90.0f));
    ASSERT_TRUE(!isRotateAngleInRange(9999999999.0f));
    ASSERT_TRUE(!isRotateAngleInRange(kInfinity));
    ASSERT_TRUE(!isRotateAngleInRange(kNotANumber));
}

TEST(sysid_range_accepts_what_the_controller_takes)
{
    ASSERT_TRUE(isSysidRangeValid(25.0f, 4.0f));        // the SILS scenarios / SILS シナリオの値
    ASSERT_TRUE(isSysidRangeValid(85.9f, 10.0f));       // the controller's limits / 制御器の上限
    ASSERT_TRUE(isSysidRangeValid(0.5f, 0.1f));
}

TEST(sysid_range_rejects_long_negative_zero_and_non_finite)
{
    ASSERT_TRUE(!isSysidRangeValid(25.0f, 600.0f));     // would hold ApiTask for 10 minutes / 10 分止める
    ASSERT_TRUE(!isSysidRangeValid(25.0f, 10.01f));
    ASSERT_TRUE(!isSysidRangeValid(86.0f, 4.0f));
    ASSERT_TRUE(!isSysidRangeValid(25.0f, -4.0f));
    ASSERT_TRUE(!isSysidRangeValid(-25.0f, 4.0f));
    ASSERT_TRUE(!isSysidRangeValid(0.0f, 4.0f));
    ASSERT_TRUE(!isSysidRangeValid(25.0f, 0.0f));
    ASSERT_TRUE(!isSysidRangeValid(kNotANumber, 4.0f));
    ASSERT_TRUE(!isSysidRangeValid(25.0f, kNotANumber));
    ASSERT_TRUE(!isSysidRangeValid(kInfinity, 4.0f));
    ASSERT_TRUE(!isSysidRangeValid(25.0f, kInfinity));
}

// `sysid thrust ...`: the API hands on any finite positive number (the controller judges the limits and names the reason);
// the step count must be a whole number from 1 to 65535.
// `sysid thrust ...`: API は有限で正の数なら渡す（上限の判断と理由の名前は制御器）。階段の回数は 1〜65535 の整数。
TEST(thrust_excitation_numbers_are_finite_positive_and_the_count_is_whole)
{
    using sf::api_arguments::isThrustExcitationCountValid;
    using sf::api_arguments::isThrustExcitationNumberValid;
    ASSERT_TRUE(isThrustExcitationNumberValid(5.0f));
    ASSERT_TRUE(isThrustExcitationNumberValid(1000.0f));   // a limit is the controller's call, with a reason / 上限は制御器の判断（理由つき）
    ASSERT_TRUE(!isThrustExcitationNumberValid(0.0f));
    ASSERT_TRUE(!isThrustExcitationNumberValid(-1.0f));
    ASSERT_TRUE(!isThrustExcitationNumberValid(kNotANumber));
    ASSERT_TRUE(!isThrustExcitationNumberValid(kInfinity));
    ASSERT_TRUE(isThrustExcitationCountValid(1.0f));
    ASSERT_TRUE(isThrustExcitationCountValid(65535.0f));
    ASSERT_TRUE(!isThrustExcitationCountValid(0.0f));
    ASSERT_TRUE(!isThrustExcitationCountValid(2.5f));
    ASSERT_TRUE(!isThrustExcitationCountValid(65536.0f));
    ASSERT_TRUE(!isThrustExcitationCountValid(kNotANumber));
    ASSERT_TRUE(!isThrustExcitationCountValid(-3.0f));
}

TEST(wrap_angle_leaves_values_inside_the_range)
{
    ASSERT_TRUE(std::fabs(sf::math::wrapAngleRad(0.0f)) < kAngleToleranceRad);
    ASSERT_TRUE(std::fabs(sf::math::wrapAngleRad(1.0f) - 1.0f) < kAngleToleranceRad);
    ASSERT_TRUE(std::fabs(sf::math::wrapAngleRad(-3.0f) + 3.0f) < kAngleToleranceRad);
}

TEST(wrap_angle_folds_into_minus_pi_to_plus_pi)
{
    ASSERT_TRUE(std::fabs(sf::math::wrapAngleRad(sf::math::kTwoPi + 0.5f) - 0.5f) < kAngleToleranceRad);
    ASSERT_TRUE(std::fabs(sf::math::wrapAngleRad(-sf::math::kTwoPi - 0.5f) + 0.5f) < kAngleToleranceRad);
    ASSERT_TRUE(std::fabs(sf::math::wrapAngleRad(3.0f * sf::math::kPi) ) <= sf::math::kPi + kAngleToleranceRad);
    ASSERT_TRUE(std::fabs(sf::math::wrapAngleRad(1000.0f)) <= sf::math::kPi + kAngleToleranceRad);
}

TEST(wrap_angle_terminates_and_stays_in_range_for_huge_finite_input)
{
    // 1e10 rad: a loop that subtracts 2*pi never ends once the float spacing exceeds 2*pi.
    // 1e10 rad: 2*pi を引くループは、float の刻みが 2*pi を超えると終わらない。
    const float huge_values[] = {1e10f, -1e10f, 9999999999.0f, 3e38f, -3e38f};
    for (const float value : huge_values) {
        const float wrapped = sf::math::wrapAngleRad(value);
        ASSERT_TRUE(std::isfinite(wrapped));
        ASSERT_TRUE(std::fabs(wrapped) <= sf::math::kPi + kAngleToleranceRad);
    }
}

TEST(wrap_angle_of_non_finite_input_is_not_finite)
{
    // The caller checks the input first (allFinite); the wrap itself must not pretend to fix it.
    // 呼び出し側が先に入力を検査する（allFinite）。折り返し自体は直したふりをしない。
    ASSERT_TRUE(!std::isfinite(sf::math::wrapAngleRad(kNotANumber)));
    ASSERT_TRUE(!std::isfinite(sf::math::wrapAngleRad(kInfinity)));
}

int main()
{
    printf("=== sf::api_arguments / sf::math::wrapAngleRad unit tests ===\n");

    run_all_finite_accepts_ordinary_numbers();
    run_all_finite_rejects_nan_and_infinity_in_any_position();
    run_float_parsing_of_overflowing_text_is_rejected();
    run_rotate_angle_range_is_one_to_three_hundred_sixty();
    run_sysid_range_accepts_what_the_controller_takes();
    run_sysid_range_rejects_long_negative_zero_and_non_finite();
    run_thrust_excitation_numbers_are_finite_positive_and_the_count_is_whole();
    run_wrap_angle_leaves_values_inside_the_range();
    run_wrap_angle_folds_into_minus_pi_to_plus_pi();
    run_wrap_angle_terminates_and_stays_in_range_for_huge_finite_input();
    run_wrap_angle_of_non_finite_input_is_not_finite();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
