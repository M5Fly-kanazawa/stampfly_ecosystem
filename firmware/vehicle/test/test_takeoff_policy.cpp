/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_takeoff_policy.cpp
 * @brief Host tests for the TAKEOFF decisions (takeoff_policy.hpp): every cell of the normative table
 *        TAKEOFF の判断（takeoff_policy.hpp）のホスト試験: 規範表の全セル
 *
 * Pure logic, no ESP-IDF / FreeRTOS. The cells are the rows of detailed_design.md §3.1 (TAKEOFF -> FLYING manual / auto, TAKEOFF -> IDLE_GROUND, comm loss):
 *  1. decideTakeoff(): manual (ACRO / STABILIZE) x airborne; auto (ALT_HOLD / POS_HOLD) x controller_reached x floor_stays x elapsed (before / at the abort time).
 *  2. decideCommLossLanding(): every FlightState x grace elapsed or not.
 *  3. TakeoffFloorWatch: the positive floor evidence (in band and not rising for the hold time; an invalid sample is neutral; a silent ToF, a reading above the band
 *     or a rise ends or restarts it).
 * The expected values are written out cell by cell from the table, not recomputed with the function's own expression.
 * Own TEST/ASSERT framework and main(), same pattern as test_tof_silence.cpp.
 *
 * @design architecture.md INV-4 — every (state x input) cell of the table is checked  [OK]
 * @design detailed_design.md §3.1 — TAKEOFF rows, comm-loss row                         [OK]
 */

#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "flight_limits.hpp"
#include "flight_state.hpp"
#include "takeoff_policy.hpp"

using sf::FlightState;
using sf::TakeoffAction;
using sf::TakeoffFacts;
using sf::TakeoffFloorWatch;

static constexpr uint32_t kAbortUs = sf::limits::kAutoTakeoffNoLiftAbortUs;
static constexpr uint32_t kHoldUs  = sf::limits::kAutoTakeoffFloorHoldUs;
static constexpr float    kBandM   = sf::limits::kTakeoffGroundBandM;
static constexpr float    kRiseM   = sf::limits::kAutoTakeoffFloorRiseM;

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

static TakeoffAction decide(bool auto_takeoff, bool airborne, bool reached, bool floor_stays, uint32_t elapsed_us)
{
    return sf::decideTakeoff(TakeoffFacts{auto_takeoff, airborne, reached, floor_stays, elapsed_us});
}

// ----------------------------------------------------------------------------
// 1. decideTakeoff()
// ----------------------------------------------------------------------------

TEST(manual_takeoff_completes_when_the_tof_says_airborne_and_never_aborts)
{
    // ACRO / STABILIZE: the ToF verdict alone decides (a silent ToF makes it "armed = not on the ground"); the controller's flag, the floor evidence and the time do not matter.
    // ACRO / STABILIZE: ToF の判定だけで決まる（ToF が出力停止ならそれは「ARM 中 = 接地していない」）。制御器のフラグ、床の証拠、時間は関係しない。
    for (bool reached : {false, true}) {
        for (bool floor_stays : {false, true}) {
            for (uint32_t elapsed_us : {0u, kAbortUs - 1u, kAbortUs, kAbortUs * 4u}) {
                ASSERT_TRUE(decide(false, true,  reached, floor_stays, elapsed_us) == TakeoffAction::Complete);
                ASSERT_TRUE(decide(false, false, reached, floor_stays, elapsed_us) == TakeoffAction::Continue);
            }
        }
    }
}

TEST(auto_takeoff_completes_when_captured_and_the_floor_is_not_shown)
{
    // The airborne verdict is not read in the auto-takeoff (the controller's capture decides). 自動離陸は airborne を読まない（制御器の捕捉が決める）。
    for (bool airborne : {false, true}) {
        for (uint32_t elapsed_us : {0u, kAbortUs - 1u, kAbortUs, kAbortUs * 4u}) {
            ASSERT_TRUE(decide(true, airborne, true,  false, elapsed_us) == TakeoffAction::Complete);
            ASSERT_TRUE(decide(true, airborne, false, false, elapsed_us) == TakeoffAction::Continue);   // no floor evidence: never aborted, the ToF-invalid rule / the controller backstop end it / 床の証拠なし: 中止しない
        }
    }
}

TEST(auto_takeoff_with_the_floor_shown_stays_and_is_aborted_after_the_abort_time)
{
    for (bool airborne : {false, true}) {
        for (bool reached : {false, true}) {
            ASSERT_TRUE(decide(true, airborne, reached, true, 0u)           == TakeoffAction::Continue);   // held, even if the controller says reached / 保持（制御器が到達と言っても）
            ASSERT_TRUE(decide(true, airborne, reached, true, kAbortUs - 1) == TakeoffAction::Continue);
            ASSERT_TRUE(decide(true, airborne, reached, true, kAbortUs)     == TakeoffAction::AbortDisarm);
            ASSERT_TRUE(decide(true, airborne, reached, true, kAbortUs * 3) == TakeoffAction::AbortDisarm);
        }
    }
}

TEST(a_slow_lift_is_never_aborted_and_the_missing_evidence_is_not_the_floor)
{
    // The case the 7th-pass rule got wrong: a craft that lifts slowly (the ToF reads above the band, so the floor evidence is absent) is not aborted at any time, whatever the accelerometer
    // would say; and the controller's backstop completes it.
    // 7 回目の規則が誤った場面: ゆっくり浮く機体（ToF は帯より上を読むので、床の証拠は無い）は、加速度計が何と言おうと、どの時刻でも中止されず、制御器の備えが完了させる。
    for (uint32_t elapsed_us : {kAbortUs, kAbortUs + 1000000u, 60000000u}) {
        ASSERT_TRUE(decide(true, false, false, false, elapsed_us) == TakeoffAction::Continue);
        ASSERT_TRUE(decide(true, false, true,  false, elapsed_us) == TakeoffAction::Complete);
    }
}

TEST(abort_time_is_longer_than_the_controller_backstop_and_the_ordinary_takeoff)
{
    // The controller's own backstop is 3 s (kTakeoffMaxCycles 1200 at 400 Hz); an ordinary takeoff leaves the band within 1 s. / 制御器自身の備えは 3 s、普通の離陸は 1 s 以内に帯を出る。
    ASSERT_TRUE(kAbortUs > 3000000u);
    ASSERT_TRUE(kAbortUs >= 5u * 900000u);   // five times the slowest ordinary takeoff (0.9 s) / 最も遅い普通の離陸（0.9 s）の 5 倍
    ASSERT_TRUE(kAbortUs < 0x80000000u);
}

// ----------------------------------------------------------------------------
// 2. decideCommLossLanding()
// ----------------------------------------------------------------------------

TEST(comm_loss_lands_from_takeoff_and_flying_only_after_the_grace)
{
    struct Row { FlightState state; bool lands; };
    const Row rows[] = {
        {FlightState::INIT,         false},
        {FlightState::IDLE_GROUND,  false},
        {FlightState::IDLE_HELD,    false},
        {FlightState::ARMED_GROUND, false},
        {FlightState::TAKEOFF,      true},    // new (8th pass): a TAKEOFF that does not complete used to keep climbing / 新規: 完了しない TAKEOFF は上昇し続けた
        {FlightState::FLYING,       true},
        {FlightState::LANDING,      false},   // already landing / 既に着陸中
        {FlightState::FLIP,         false},   // waits for FLYING / FLYING まで待つ
    };
    for (const Row& row : rows) {
        ASSERT_TRUE(sf::decideCommLossLanding(row.state, true) == row.lands);
        ASSERT_TRUE(!sf::decideCommLossLanding(row.state, false));   // before the grace: never / 猶予の前: 決して
    }
}

// ----------------------------------------------------------------------------
// 3. TakeoffFloorWatch
// ----------------------------------------------------------------------------

static constexpr uint32_t kSampleUs = 33000;   // 30 Hz

// Feed `count` valid samples of a constant reading from `start_us`; returns the time of the last one. / 一定の読みの有効サンプルを count 個与える。
static uint32_t feed(TakeoffFloorWatch& watch, uint32_t start_us, int count, float distance_m, bool healthy = true, bool valid = true)
{
    uint32_t t = start_us;
    for (int i = 0; i < count; ++i) {
        watch.onSample(t, healthy, valid, distance_m);
        t += kSampleUs;
    }
    return t - kSampleUs;
}

TEST(floor_watch_needs_a_valid_reading_in_the_band_held_for_the_hold_time)
{
    TakeoffFloorWatch watch;
    watch.reset();
    ASSERT_TRUE(!watch.stays(10000000u, true));                          // nothing seen / 何も見ていない
    const uint32_t start = 1000000u;
    uint32_t last = feed(watch, start, 20, 0.04f);                        // 0.66 s: not yet / まだ
    ASSERT_TRUE(!watch.stays(last, true));
    last = feed(watch, last + kSampleUs, 20, 0.04f);                      // 1.3 s: held / 保持
    ASSERT_TRUE(watch.stays(last, true));
    ASSERT_TRUE(!watch.stays(last, false));                              // a ToF that is not healthy now: no evidence / ToF が今は健全でない: 証拠なし
    ASSERT_TRUE(watch.stays(start + kHoldUs, true));                     // boundary: exactly the hold time / 境界: ちょうど保持時間
    ASSERT_TRUE(!watch.stays(start + kHoldUs - 1u, true));
}

TEST(floor_watch_takes_no_evidence_from_invalid_samples_or_a_silent_tof)
{
    TakeoffFloorWatch invalid_only;
    invalid_only.reset();
    feed(invalid_only, 1000000u, 200, 0.0f, true, false);                // the real floor in the dead zone: invalid samples only / 不感帯の実際の床: 無効サンプルだけ
    ASSERT_TRUE(!invalid_only.stays(8000000u, true));
    TakeoffFloorWatch silent_tof;
    silent_tof.reset();
    feed(silent_tof, 1000000u, 100, 0.04f, false, true);                 // samples while the ToF is judged silent / ToF が出力停止と判定されている間のサンプル
    ASSERT_TRUE(!silent_tof.stays(5000000u, true));
}

TEST(floor_watch_invalid_samples_are_neutral_inside_a_run)
{
    TakeoffFloorWatch watch;
    watch.reset();
    uint32_t last = feed(watch, 1000000u, 10, 0.04f);
    last = feed(watch, last + kSampleUs, 10, 0.0f, true, false);         // a flicker of invalid samples / 無効サンプルのちらつき
    last = feed(watch, last + kSampleUs, 20, 0.04f);
    ASSERT_TRUE(watch.stays(last, true));                                // the run was not broken / 区間は切れていない
}

TEST(floor_watch_ends_when_the_reading_leaves_the_band_or_the_tof_is_not_healthy)
{
    TakeoffFloorWatch above;
    above.reset();
    uint32_t last = feed(above, 1000000u, 50, 0.04f);
    ASSERT_TRUE(above.stays(last, true));
    above.onSample(last + kSampleUs, true, true, kBandM + 0.01f);        // lifted above the band / 帯より上に浮いた
    ASSERT_TRUE(!above.stays(last + kSampleUs, true));
    last = feed(above, last + 2 * kSampleUs, 50, 0.04f);                 // a new run must hold again / 新しい区間はまた保持しなければならない
    ASSERT_TRUE(above.stays(last, true));

    TakeoffFloorWatch unhealthy;
    unhealthy.reset();
    last = feed(unhealthy, 1000000u, 50, 0.04f);
    unhealthy.onSample(last + kSampleUs, false, true, 0.04f);            // the ToF turned silent / ToF が出力停止になった
    ASSERT_TRUE(!unhealthy.stays(last + kSampleUs, true));
}

TEST(floor_watch_restarts_when_the_reading_rises_and_ignores_noise)
{
    // A craft that lifts slowly inside the band (0.031 -> 0.049 m) is rising, so it is not "on the floor": each rise by more than kRiseM restarts the run.
    // The ToF noise (+-0.5 cm) never restarts it.
    // 帯の中でゆっくり浮く機体（0.031 → 0.049 m）は上がっているので「床の上」ではない: kRiseM を超える上昇のたびに区間を数え直す。ToF ノイズ（±0.5 cm）では数え直さない。
    TakeoffFloorWatch rising;
    rising.reset();
    uint32_t t = 1000000u;
    for (float h = 0.031f; h < 0.049f; h += kRiseM * 1.2f) {
        t = feed(rising, t, 25, h) + kSampleUs;                          // 0.8 s per step: never reaches the hold time within a run / 1 段 0.8 s: 区間の中では保持時間に届かない
        ASSERT_TRUE(!rising.stays(t, true));
    }
    TakeoffFloorWatch noisy;
    noisy.reset();
    t = 1000000u;
    for (int i = 0; i < 60; ++i) {
        noisy.onSample(t, true, true, 0.040f + ((i % 2) ? 0.005f : -0.005f));
        t += kSampleUs;
    }
    ASSERT_TRUE(noisy.stays(t, true));
}

TEST(floor_watch_reset_forgets_the_run_and_the_wrap_of_the_clock_is_harmless)
{
    TakeoffFloorWatch watch;
    watch.reset();
    uint32_t last = feed(watch, 0xFFFFF000u, 60, 0.04f);                 // the clock wraps inside the run / 区間の途中で時計が巻き戻る
    ASSERT_TRUE(watch.stays(last, true));
    watch.reset();
    ASSERT_TRUE(!watch.stays(last, true));
}

int main()
{
    printf("=== takeoff_policy host tests ===\n");
    run_manual_takeoff_completes_when_the_tof_says_airborne_and_never_aborts();
    run_auto_takeoff_completes_when_captured_and_the_floor_is_not_shown();
    run_auto_takeoff_with_the_floor_shown_stays_and_is_aborted_after_the_abort_time();
    run_a_slow_lift_is_never_aborted_and_the_missing_evidence_is_not_the_floor();
    run_abort_time_is_longer_than_the_controller_backstop_and_the_ordinary_takeoff();
    run_comm_loss_lands_from_takeoff_and_flying_only_after_the_grace();
    run_floor_watch_needs_a_valid_reading_in_the_band_held_for_the_hold_time();
    run_floor_watch_takes_no_evidence_from_invalid_samples_or_a_silent_tof();
    run_floor_watch_invalid_samples_are_neutral_inside_a_run();
    run_floor_watch_ends_when_the_reading_leaves_the_band_or_the_tof_is_not_healthy();
    run_floor_watch_restarts_when_the_reading_rises_and_ignores_noise();
    run_floor_watch_reset_forgets_the_run_and_the_wrap_of_the_clock_is_harmless();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
