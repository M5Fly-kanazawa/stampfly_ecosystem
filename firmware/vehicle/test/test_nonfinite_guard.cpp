/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_nonfinite_guard.cpp
 * @brief Deterministic unit tests for the non-finite number detection (nonfinite_guard.hpp)
 *        有限でない数の検出（nonfinite_guard.hpp）の決定論ユニットテスト
 *
 * Header-only logic — no ESP-IDF, no FreeRTOS. Own TEST/ASSERT framework and main(), same pattern as
 * test_loop_timing.cpp. Covers every branch of the mask, the monitor (stage counts, first boundary and
 * time, checked / detected cycles) and the motor-output guard (finite duties unchanged bit for bit,
 * NaN and +/-infinity driven as 0, +infinity hidden by a limit still caught through the upstream mask).
 * ヘッダのみのロジック — ESP-IDF・FreeRTOS 不要。test_loop_timing.cpp と同じ自前の TEST/ASSERT
 * フレームワークと main()。マスク・監視器（境界ごとの数・最初の境界と時刻・検査した周期と検出した周期）・
 * モータ出力の保護（有限な duty がビット単位で不変、NaN と ±無限大は 0、上下限に隠れる +無限大は上流の
 * マスクで捕まる）の全分岐を検査する。
 *
 * @design detailed_design.md §8 — Non-finite number monitor (nonfinite_status)  [OK]
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

#include "nonfinite_guard.hpp"

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-52s ", #name); \
        try { test_##name(); tests_passed++; printf("PASS\n"); } \
        catch (...) { tests_failed++; printf("FAIL\n"); } \
    } \
    static void test_##name()

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        printf("FAIL: %s:%d: condition false\n", __FILE__, __LINE__); \
        throw 1; \
    }

static const float kNan = std::numeric_limits<float>::quiet_NaN();
static const float kInf = std::numeric_limits<float>::infinity();

/// The limit of the motor path (same comparison form as actuator.cpp clampDuty()).
/// モータ経路の上下限（actuator.cpp の clampDuty() と同じ比較の形）。
static float clampDuty(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

TEST(a_comparison_limit_passes_nan_through)
{
    ASSERT_TRUE(std::isnan(clampDuty(kNan)));    // the problem the guard exists for / 保護が必要な理由
    ASSERT_TRUE(clampDuty(kInf) == 1.0f);        // +infinity is hidden as a finite maximum / +無限大は有限の最大値に隠れる
}

TEST(mask_is_zero_for_finite_values_including_the_extremes)
{
    const float values[] = {0.0f, -0.0f, 1.0f, -1.0f, 3.4028234e38f, -3.4028234e38f, 1.0e-45f};
    ASSERT_TRUE(sf::nonFiniteMask(values, 7) == 0);
    ASSERT_TRUE(sf::countNonFinite(values, 7) == 0);
}

TEST(mask_marks_nan_and_both_infinities_by_position)
{
    const float values[] = {1.0f, kNan, 2.0f, kInf, -kInf};
    ASSERT_TRUE(sf::nonFiniteMask(values, 5) == ((1u << 1) | (1u << 3) | (1u << 4)));
    ASSERT_TRUE(sf::countNonFinite(values, 5) == 3);
}

TEST(count_of_zero_values_is_zero)
{
    ASSERT_TRUE(sf::nonFiniteMask(nullptr, 0) == 0);
    ASSERT_TRUE(sf::countMaskBits(0) == 0);
    ASSERT_TRUE(sf::countMaskBits(0xFu) == 4);
}

TEST(monitor_with_clean_cycles_detects_nothing)
{
    sf::NonFiniteMonitor monitor;
    for (uint32_t cycle = 0; cycle < 10; ++cycle) {
        monitor.beginCycle(1000 + cycle);
        ASSERT_TRUE(!monitor.record(sf::NonFiniteStage::EstimateInput, 0));
        ASSERT_TRUE(!monitor.record(sf::NonFiniteStage::MotorCommand, 0));
        ASSERT_TRUE(!monitor.detectedThisCycle());
    }
    const sf::NonFiniteStatus& status = monitor.result();
    ASSERT_TRUE(status.checked_cycles == 10);
    ASSERT_TRUE(status.detected_cycles == 0);
    ASSERT_TRUE(status.first_stage == sf::kNonFiniteNoStage);
    ASSERT_TRUE(status.first_us == 0);
    for (size_t stage = 0; stage < sf::kNonFiniteStageCount; ++stage) {
        ASSERT_TRUE(status.stage_count[stage] == 0);
    }
}

TEST(first_detection_remembers_the_most_upstream_boundary_and_the_time)
{
    sf::NonFiniteMonitor monitor;
    monitor.beginCycle(5000);
    monitor.record(sf::NonFiniteStage::EstimateInput, 0);
    ASSERT_TRUE(monitor.record(sf::NonFiniteStage::ControlOutput, 2));   // the first one in path order / 経路の順で最初
    monitor.record(sf::NonFiniteStage::MixerOutput, 4);
    monitor.record(sf::NonFiniteStage::MotorCommand, 4);
    ASSERT_TRUE(monitor.detectedThisCycle());

    const sf::NonFiniteStatus& status = monitor.result();
    ASSERT_TRUE(status.first_stage == static_cast<uint8_t>(sf::NonFiniteStage::ControlOutput));
    ASSERT_TRUE(status.first_us == 5000);
    ASSERT_TRUE(status.detected_cycles == 1);                  // one cycle, however many boundaries / 境界がいくつでも 1 周期
    ASSERT_TRUE(status.stage_count[static_cast<size_t>(sf::NonFiniteStage::EstimateInput)] == 0);
    ASSERT_TRUE(status.stage_count[static_cast<size_t>(sf::NonFiniteStage::ControlOutput)] == 1);
    ASSERT_TRUE(status.stage_count[static_cast<size_t>(sf::NonFiniteStage::MixerOutput)] == 1);
    ASSERT_TRUE(status.stage_count[static_cast<size_t>(sf::NonFiniteStage::MotorCommand)] == 1);
}

TEST(a_later_detection_keeps_the_first_boundary_and_time)
{
    sf::NonFiniteMonitor monitor;
    monitor.beginCycle(100);
    monitor.record(sf::NonFiniteStage::MixerOutput, 1);
    monitor.beginCycle(2600);
    monitor.record(sf::NonFiniteStage::EstimateInput, 3);      // more upstream, but later: not "first" / より上流でも後なら「最初」ではない
    monitor.record(sf::NonFiniteStage::MixerOutput, 1);

    const sf::NonFiniteStatus& status = monitor.result();
    ASSERT_TRUE(status.first_stage == static_cast<uint8_t>(sf::NonFiniteStage::MixerOutput));
    ASSERT_TRUE(status.first_us == 100);
    ASSERT_TRUE(status.detected_cycles == 2);
    ASSERT_TRUE(status.checked_cycles == 2);
    ASSERT_TRUE(status.stage_count[static_cast<size_t>(sf::NonFiniteStage::MixerOutput)] == 2);
    ASSERT_TRUE(status.stage_count[static_cast<size_t>(sf::NonFiniteStage::EstimateInput)] == 1);
}

TEST(a_clean_cycle_after_a_detection_does_not_count_as_detected)
{
    sf::NonFiniteMonitor monitor;
    monitor.beginCycle(0);
    monitor.record(sf::NonFiniteStage::SetpointInput, 1);
    monitor.beginCycle(2500);
    ASSERT_TRUE(!monitor.detectedThisCycle());                 // the flag is per cycle / フラグは周期ごと
    monitor.record(sf::NonFiniteStage::SetpointInput, 0);
    ASSERT_TRUE(monitor.result().detected_cycles == 1);
    ASSERT_TRUE(monitor.result().checked_cycles == 2);
}

TEST(guard_leaves_finite_duties_unchanged_bit_for_bit)
{
    const float original[sf::kMotorCount] = {0.0f, 0.37f, -0.0f, 1.0f};
    float duties[sf::kMotorCount];
    std::memcpy(duties, original, sizeof(duties));
    ASSERT_TRUE(sf::guardMotorDuties(duties, 0) == 0);
    ASSERT_TRUE(std::memcmp(duties, original, sizeof(duties)) == 0);
}

TEST(guard_drives_nan_and_infinities_as_zero_and_touches_only_those)
{
    float duties[sf::kMotorCount] = {kNan, 0.5f, kInf, -kInf};
    const uint32_t replaced = sf::guardMotorDuties(duties, 0);
    ASSERT_TRUE(replaced == ((1u << 0) | (1u << 2) | (1u << 3)));
    ASSERT_TRUE(duties[0] == 0.0f && !std::signbit(duties[0]));
    ASSERT_TRUE(duties[1] == 0.5f);
    ASSERT_TRUE(duties[2] == 0.0f);
    ASSERT_TRUE(duties[3] == 0.0f);
}

TEST(guard_catches_a_value_the_limit_hid_through_the_upstream_mask)
{
    float duties[sf::kMotorCount] = {0.2f, kInf, kNan, 0.9f};   // mixer output / ミキサーの出力
    const uint32_t upstream = sf::nonFiniteMask(duties, sf::kMotorCount);
    for (float& duty : duties) {
        duty = clampDuty(duty);                                  // +infinity became 1.0, NaN stayed / +無限大は 1.0 に、NaN はそのまま
    }
    ASSERT_TRUE(duties[1] == 1.0f);
    const uint32_t replaced = sf::guardMotorDuties(duties, upstream);
    ASSERT_TRUE(replaced == ((1u << 1) | (1u << 2)));
    ASSERT_TRUE(duties[0] == 0.2f);
    ASSERT_TRUE(duties[1] == 0.0f);                              // not left at full throttle / 全開のまま残さない
    ASSERT_TRUE(duties[2] == 0.0f);
    ASSERT_TRUE(duties[3] == 0.9f);
}

TEST(an_injected_nan_is_detected_and_every_motor_command_is_a_finite_integer_source)
{
    // A NaN thrust reaches the mixer as four NaN duties: counted at the mixer and at the motor
    // command, and all four motors end at 0 — a value that converts to an integer without undefined behaviour.
    // NaN の推力は 4 個の NaN の duty としてミキサーに届く: ミキサーとモータ指令で数え、4 モータとも 0 になる
    // — 未定義動作なしで整数に変換できる値。
    sf::NonFiniteMonitor monitor;
    monitor.beginCycle(7777);
    float duties[sf::kMotorCount] = {kNan, kNan, kNan, kNan};
    const uint32_t upstream = sf::nonFiniteMask(duties, sf::kMotorCount);
    monitor.record(sf::NonFiniteStage::MixerOutput, sf::countMaskBits(upstream));
    for (float& duty : duties) {
        duty = clampDuty(duty);
    }
    const uint32_t replaced = sf::guardMotorDuties(duties, upstream);
    monitor.record(sf::NonFiniteStage::MotorCommand, sf::countMaskBits(replaced));

    for (float duty : duties) {
        ASSERT_TRUE(std::isfinite(duty));
        ASSERT_TRUE(static_cast<uint32_t>(duty * 8191.0f) == 0);
    }
    ASSERT_TRUE(monitor.result().first_stage == static_cast<uint8_t>(sf::NonFiniteStage::MixerOutput));
    ASSERT_TRUE(monitor.result().first_us == 7777);
}

TEST(stage_names_cover_every_boundary_and_the_none_value)
{
    ASSERT_TRUE(std::strcmp(sf::nonFiniteStageName(static_cast<uint8_t>(sf::NonFiniteStage::EstimateInput)), "estimate input") == 0);
    ASSERT_TRUE(std::strcmp(sf::nonFiniteStageName(static_cast<uint8_t>(sf::NonFiniteStage::SetpointInput)), "setpoint input") == 0);
    ASSERT_TRUE(std::strcmp(sf::nonFiniteStageName(static_cast<uint8_t>(sf::NonFiniteStage::ControlReference)), "control reference") == 0);
    ASSERT_TRUE(std::strcmp(sf::nonFiniteStageName(static_cast<uint8_t>(sf::NonFiniteStage::ControlOutput)), "control output") == 0);
    ASSERT_TRUE(std::strcmp(sf::nonFiniteStageName(static_cast<uint8_t>(sf::NonFiniteStage::MixerOutput)), "mixer output") == 0);
    ASSERT_TRUE(std::strcmp(sf::nonFiniteStageName(static_cast<uint8_t>(sf::NonFiniteStage::MotorCommand)), "motor command") == 0);
    ASSERT_TRUE(std::strcmp(sf::nonFiniteStageName(sf::kNonFiniteNoStage), "none") == 0);
}

int main()
{
    printf("=== NonFinite guard unit tests ===\n");
    run_a_comparison_limit_passes_nan_through();
    run_mask_is_zero_for_finite_values_including_the_extremes();
    run_mask_marks_nan_and_both_infinities_by_position();
    run_count_of_zero_values_is_zero();
    run_monitor_with_clean_cycles_detects_nothing();
    run_first_detection_remembers_the_most_upstream_boundary_and_the_time();
    run_a_later_detection_keeps_the_first_boundary_and_time();
    run_a_clean_cycle_after_a_detection_does_not_count_as_detected();
    run_guard_leaves_finite_duties_unchanged_bit_for_bit();
    run_guard_drives_nan_and_infinities_as_zero_and_touches_only_those();
    run_guard_catches_a_value_the_limit_hid_through_the_upstream_mask();
    run_an_injected_nan_is_detected_and_every_motor_command_is_a_finite_integer_source();
    run_stage_names_cover_every_boundary_and_the_none_value();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
