/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_battery_alert_policy.cpp
 * @brief Deterministic unit tests for sf::BatteryAlertPolicy (low-battery WARNING / EMERGENCY timing)
 *        sf::BatteryAlertPolicy の決定論ユニットテスト（低電圧 WARNING / EMERGENCY のタイミング）
 *
 * Header-only logic (battery_alert_policy.hpp) — no ESP-IDF, no FreeRTOS. Own TEST/ASSERT
 * framework and main(), same pattern as test_sensor_freshness.cpp.
 * ヘッダのみのロジック（battery_alert_policy.hpp）— ESP-IDF・FreeRTOS 不要。
 * test_sensor_freshness.cpp と同じ自前の TEST/ASSERT フレームワークと main()。
 *
 * @design requirements.md §9 — low battery warning / emergency landing   [OK]
 */

#include <cstdint>
#include <cstdio>

#include "battery_alert_policy.hpp"

// Thresholds of FailsafeConfig (failsafe.hpp): low 3.4 V, critical 3.0 V.
// FailsafeConfig（failsafe.hpp）の閾値: 低電圧 3.4 V、危険 3.0 V。
static constexpr float kLowV      = 3.4f;
static constexpr float kCriticalV = 3.0f;
// Voltages used by the cases: below critical, in the warning band, recovered with margin.
// ケースで使う電圧: 危険閾値の下、警告帯、余裕をもって回復。
static constexpr float kCriticalVoltageV = 2.9f;
static constexpr float kWarningBandV     = 3.2f;
static constexpr float kRecoveredV       = 3.8f;
// Just above the recovery hysteresis edge (3.4 + 0.1 V) and just below it.
// 回復ヒステリシスの境界（3.4 + 0.1 V）の直上と直下。
static constexpr float kJustAboveHysteresisV = 3.51f;
static constexpr float kJustBelowHysteresisV = 3.49f;

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

#define ASSERT_EQ(actual, expected) \
    if ((actual) != (expected)) { \
        printf("FAIL: %s:%d: %d != %d\n", __FILE__, __LINE__, \
               static_cast<int>(actual), static_cast<int>(expected)); \
        throw 1; \
    }

using sf::BatteryAlertAction;
using sf::BatteryAlertPolicy;

// Feed `updates` samples of the same voltage and count the actions.
// 同じ電圧のサンプルを `updates` 回与え、動作を数える。
struct Counts {
    int warnings   = 0;
    int emergencies = 0;
    int first_emergency_index = -1;   // update index of the first EMERGENCY / 最初の EMERGENCY の更新番号
};

static Counts feed(BatteryAlertPolicy& policy, float voltage, int updates)
{
    Counts counts;
    for (int i = 0; i < updates; ++i) {
        const BatteryAlertAction action = policy.update(voltage, kLowV, kCriticalV);
        if (action == BatteryAlertAction::Warning) {
            ++counts.warnings;
        } else if (action == BatteryAlertAction::Emergency) {
            if (counts.first_emergency_index < 0) {
                counts.first_emergency_index = i;
            }
            ++counts.emergencies;
        }
    }
    return counts;
}

TEST(first_entry_raises_emergency_at_once)
{
    BatteryAlertPolicy policy;
    const Counts counts = feed(policy, kCriticalVoltageV, 1);
    ASSERT_EQ(counts.emergencies, 1);
    ASSERT_EQ(counts.first_emergency_index, 0);
    ASSERT_EQ(counts.warnings, 0);
}

TEST(healthy_voltage_raises_nothing)
{
    BatteryAlertPolicy policy;
    const Counts counts = feed(policy, kRecoveredV, 600);
    ASSERT_EQ(counts.emergencies, 0);
    ASSERT_EQ(counts.warnings, 0);
}

TEST(staying_critical_reraises_every_five_seconds)
{
    BatteryAlertPolicy policy;
    // 10 Hz: 300 updates = 30 s. Raises at update 0, 50, 100, ... 250 = 6 times.
    // 10 Hz: 300 回 = 30 s。更新 0・50・100 … 250 の 6 回。
    const Counts counts = feed(policy, kCriticalVoltageV, 300);
    ASSERT_EQ(counts.emergencies, 6);
    ASSERT_EQ(counts.warnings, 0);
}

TEST(no_emergency_between_the_reraises)
{
    BatteryAlertPolicy policy;
    ASSERT_TRUE(policy.update(kCriticalVoltageV, kLowV, kCriticalV) == BatteryAlertAction::Emergency);
    for (int i = 1; i < BatteryAlertPolicy::kEmergencyReraiseEveryNUpdates; ++i) {
        ASSERT_TRUE(policy.update(kCriticalVoltageV, kLowV, kCriticalV) == BatteryAlertAction::None);
    }
    ASSERT_TRUE(policy.update(kCriticalVoltageV, kLowV, kCriticalV) == BatteryAlertAction::Emergency);
}

TEST(recovery_to_warning_band_then_drop_raises_again)
{
    BatteryAlertPolicy policy;
    ASSERT_EQ(feed(policy, kCriticalVoltageV, 10).emergencies, 1);
    // Voltage recovers without load to the band between critical and low (latch NOT cleared by
    // the recovery hysteresis), stays there for a minute, then drops below critical in flight.
    // 無負荷で電圧が危険と低電圧の間の帯へ回復し（回復ヒステリシスではラッチは解けない）、
    // 1 分そこにいて、飛行中に危険閾値の下へ落ちる。
    const Counts in_band = feed(policy, kWarningBandV, 600);
    ASSERT_EQ(in_band.emergencies, 0);
    ASSERT_TRUE(policy.warningLatched());
    const Counts dropped = feed(policy, kCriticalVoltageV, 1);
    ASSERT_EQ(dropped.emergencies, 1);
    ASSERT_EQ(dropped.first_emergency_index, 0);
}

TEST(warning_band_reraises_the_warning_not_the_emergency)
{
    BatteryAlertPolicy policy;
    // Entry raises one WARNING at update 0, then every 50 updates: 0, 50, 100, 150 = 4 in 200.
    // 突入で更新 0 に WARNING を 1 回、以後 50 回ごと: 200 回で 0・50・100・150 の 4 回。
    const Counts counts = feed(policy, kWarningBandV, 200);
    ASSERT_EQ(counts.warnings, 4);
    ASSERT_EQ(counts.emergencies, 0);
}

TEST(chattering_around_the_threshold_is_bounded)
{
    BatteryAlertPolicy policy;
    // The voltage alternates below / above the critical threshold at every update for 60 s.
    // 電圧が 60 s の間、毎回の更新で危険閾値の下と上を交互に行き来する。
    int emergencies = 0;
    constexpr int kUpdates = 600;
    for (int i = 0; i < kUpdates; ++i) {
        const float voltage = (i % 2 == 0) ? kCriticalVoltageV : kWarningBandV;
        if (policy.update(voltage, kLowV, kCriticalV) == BatteryAlertAction::Emergency) {
            ++emergencies;
        }
    }
    // At most one per kEmergencyMinSpacingUpdates (20 updates = 2 s): 30 in 60 s, and at least
    // one so the craft is never left without an alert.
    // 最短間隔 kEmergencyMinSpacingUpdates（20 回 = 2 s）に 1 回まで: 60 s で最大 30 回。
    // 1 回以上は出す（アラートなしにしない）。
    ASSERT_TRUE(emergencies >= 1);
    ASSERT_TRUE(emergencies <= kUpdates / BatteryAlertPolicy::kEmergencyMinSpacingUpdates);
}

TEST(chattering_with_every_third_update_critical_is_bounded)
{
    BatteryAlertPolicy policy;
    int emergencies = 0;
    constexpr int kUpdates = 600;
    for (int i = 0; i < kUpdates; ++i) {
        const float voltage = (i % 3 == 0) ? kCriticalVoltageV : kWarningBandV;
        if (policy.update(voltage, kLowV, kCriticalV) == BatteryAlertAction::Emergency) {
            ++emergencies;
        }
    }
    ASSERT_TRUE(emergencies >= 1);
    ASSERT_TRUE(emergencies <= kUpdates / BatteryAlertPolicy::kEmergencyMinSpacingUpdates);
}

TEST(reentry_waits_for_the_minimum_spacing)
{
    BatteryAlertPolicy policy;
    ASSERT_EQ(feed(policy, kCriticalVoltageV, 1).emergencies, 1);        // update 0
    constexpr int kUpdatesOutside = 4;
    ASSERT_EQ(feed(policy, kWarningBandV, kUpdatesOutside).emergencies, 0);   // updates 1-4
    // Re-entry at update 5 (5 updates after the raise): pending until the minimum spacing (20
    // updates since the raise) has passed, then raised once.
    // 更新 5（発報の 5 回後）に再突入: 最短間隔（発報から 20 回）が過ぎるまで保留し、その後 1 回発報。
    const Counts reentry = feed(policy, kCriticalVoltageV, 30);
    ASSERT_EQ(reentry.emergencies, 1);
    ASSERT_EQ(reentry.first_emergency_index + kUpdatesOutside + 1,
              BatteryAlertPolicy::kEmergencyMinSpacingUpdates);
}

TEST(full_recovery_clears_both_latches)
{
    BatteryAlertPolicy policy;
    ASSERT_EQ(feed(policy, kCriticalVoltageV, 1).emergencies, 1);
    ASSERT_TRUE(policy.warningLatched());
    // Inside the hysteresis (3.4 .. 3.5 V) the warning latch stays set; above it, it clears.
    // ヒステリシスの内側（3.4〜3.5 V）では警告のラッチは残り、外側で解ける。
    feed(policy, kJustBelowHysteresisV, 5);
    ASSERT_TRUE(policy.warningLatched());
    feed(policy, kJustAboveHysteresisV, 1);
    ASSERT_TRUE(!policy.warningLatched());
    // After a full recovery both alerts come back on the next drop: the WARNING at once, the
    // EMERGENCY once the minimum spacing since the last one has passed.
    // 完全に回復した後は、次の低下で両方が戻る: WARNING は直ちに、EMERGENCY は前回からの最短間隔が
    // 過ぎていれば発報する。
    feed(policy, kRecoveredV, 100);
    ASSERT_EQ(feed(policy, kWarningBandV, 1).warnings, 1);
    ASSERT_EQ(feed(policy, kCriticalVoltageV, 1).emergencies, 1);
}

int main()
{
    printf("=== sf::BatteryAlertPolicy unit tests ===\n");

    run_first_entry_raises_emergency_at_once();
    run_healthy_voltage_raises_nothing();
    run_staying_critical_reraises_every_five_seconds();
    run_no_emergency_between_the_reraises();
    run_recovery_to_warning_band_then_drop_raises_again();
    run_warning_band_reraises_the_warning_not_the_emergency();
    run_chattering_around_the_threshold_is_bounded();
    run_chattering_with_every_third_update_critical_is_bounded();
    run_reentry_waits_for_the_minimum_spacing();
    run_full_recovery_clears_both_latches();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
