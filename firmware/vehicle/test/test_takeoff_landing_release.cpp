/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_takeoff_landing_release.cpp
 * @brief Host test: the ground flag's hysteresis and the estimator-release fact of TakeoffLandingMgr
 *        ホスト試験: 接地フラグのヒステリシスと、TakeoffLandingMgr の推定器の解除の事実
 *
 * 1. The hysteresis of the ground flag (isOnGround()), which had no direct test: the airborne side flips at once, the ground side needs
 *    flip_confirm_samples consecutive valid readings, the band between holds the flag, an invalid sample holds it and breaks a streak.
 * 2. The estimator-release fact (isEstimatorReleased()): a LEVEL set by a valid reading above estimator_release_tof_m (and by markAirborne()), cleared
 *    wherever on_ground_ becomes true; with a release height of its own (lower than airborne_tof_m) it differs from !isOnGround() only between the two heights.
 * 3. With estimator_release_tof_m == airborne_tof_m (set explicitly here, so the test survives a change of the default) the new fact equals
 *    !isOnGround() on EVERY sample of the real ToF sequences of the flights of 2026-11-23 (takeoff_tof_samples.hpp).
 * 4. The order of the constants.
 * Own TEST/ASSERT framework and main(), same pattern as test_takeoff_policy.cpp. Linked with takeoff_landing.cpp.
 *
 * @design architecture.md INV-3 — detection in sf_takeoff_landing, ImuTask the only reader of the release fact [OK]
 * @design detailed_design.md §3 注16 — estimator release vs ground flag                                       [OK]
 */

#include <cstdint>
#include <cstdio>

#include "flight_limits.hpp"
#include "takeoff_landing.hpp"
#include "takeoff_tof_samples.hpp"

int64_t g_mock_esp_time_us = 0;   // the mock clock of the esp_timer stub (esp_timer.h) / esp_timer の代替の仮想時計

namespace {

constexpr float    kGroundBandM  = sf::limits::kTakeoffGroundBandM;
constexpr float    kAirborneM    = sf::limits::kTakeoffAirborneBandM;
constexpr float    kLowRelease   = 0.08f;       // a release height of its own, between the two bands (the value of stage 2) / 専用の解除の高さ（第 2 段の値）
constexpr int      kSamplePeriodMs = 33;        // the ToF sample period (30 Hz) / ToF のサンプル周期（30 Hz）
constexpr float    kFloorReadingM  = 0.040f;    // a reading on the floor / 床の上の読み
constexpr float    kHopReadingM    = 0.10f;     // between the release height and the airborne side / 解除の高さと空中側の間
constexpr float    kClimbReadingM  = 0.30f;     // well airborne / 十分に空中

static_assert(kGroundBandM < sf::limits::kEstimatorReleaseHeightM && sf::limits::kEstimatorReleaseHeightM <= kAirborneM,
              "the default release height lies in (ground band, airborne band]");

int tests_run = 0;
int tests_passed = 0;
int tests_failed = 0;

}  // namespace

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-70s ", #name); \
        try { test_##name(); tests_passed++; printf("PASS\n"); } \
        catch (...) { tests_failed++; printf("FAIL\n"); } \
    } \
    static void test_##name()

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        printf("FAIL: %s:%d: condition false\n", __FILE__, __LINE__); \
        throw 1; \
    }

/// A manager (armed after init) with the given release height. 与えた解除の高さの（init の後 ARM した）マネージャ。
static sf::TakeoffLandingMgr makeManager(float release_height_m)
{
    sf::TakeoffLandingConfig config;
    config.estimator_release_tof_m = release_height_m;
    sf::TakeoffLandingMgr mgr;
    mgr.init(config);
    return mgr;
}

/// One armed ToF sample (a valid reading in metres, or an invalid one). 1 つの ARM 中の ToF サンプル（有効な読み [m]、または無効）。
static void sample(sf::TakeoffLandingMgr& mgr, float distance_m, bool valid = true)
{
    sf::TofData tof{};
    tof.distance = distance_m;
    tof.valid    = valid;
    g_mock_esp_time_us += static_cast<int64_t>(kSamplePeriodMs) * 1000;
    mgr.update(tof, true, 0.0f, false);
}

static void samples(sf::TakeoffLandingMgr& mgr, float distance_m, int count)
{
    for (int i = 0; i < count; ++i) sample(mgr, distance_m);
}

// ----------------------------------------------------------------------------
// 1. The hysteresis of the ground flag (no direct test existed)
// ----------------------------------------------------------------------------

TEST(ground_flag_starts_on_the_ground_and_a_floor_reading_does_not_release_it)
{
    sf::TakeoffLandingMgr mgr = makeManager(kAirborneM);
    ASSERT_TRUE(mgr.isOnGround());
    samples(mgr, kFloorReadingM, 10);
    ASSERT_TRUE(mgr.isOnGround());
    samples(mgr, kHopReadingM, 10);       // the band between the two sides holds the flag / 間の帯はフラグを保つ
    ASSERT_TRUE(mgr.isOnGround());
}

TEST(ground_flag_flips_to_air_at_once_on_a_reading_above_the_airborne_side_and_not_at_it)
{
    sf::TakeoffLandingMgr mgr = makeManager(kAirborneM);
    sample(mgr, kAirborneM);              // exactly at the threshold is not above it / ちょうど閾値は「超える」でない
    ASSERT_TRUE(mgr.isOnGround());
    sample(mgr, kAirborneM + 0.001f);     // one sample above: immediately airborne (the takeoff handoff is not debounced) / 1 サンプル超えで即時
    ASSERT_TRUE(!mgr.isOnGround());
}

TEST(ground_flag_holds_in_the_band_when_descending_from_the_air)
{
    sf::TakeoffLandingMgr mgr = makeManager(kAirborneM);
    samples(mgr, kClimbReadingM, 3);
    ASSERT_TRUE(!mgr.isOnGround());
    samples(mgr, kHopReadingM, 20);       // 0.05 < 0.10 < 0.15: hold / 帯の中は保つ
    ASSERT_TRUE(!mgr.isOnGround());
}

TEST(ground_flag_needs_consecutive_valid_floor_readings_to_come_back)
{
    sf::TakeoffLandingMgr mgr = makeManager(kAirborneM);
    samples(mgr, kClimbReadingM, 3);
    sample(mgr, kFloorReadingM);          // one ghost reading: no flip / 単発のゴースト: 反転しない
    ASSERT_TRUE(!mgr.isOnGround());
    sample(mgr, kHopReadingM);            // a band reading breaks the streak / 帯の読みは連続数を切る
    sample(mgr, kFloorReadingM);
    ASSERT_TRUE(!mgr.isOnGround());
    sample(mgr, 0.0f, false);             // an invalid sample breaks it as well and holds the flag / 無効サンプルも切り、フラグは保つ
    sample(mgr, kFloorReadingM);
    ASSERT_TRUE(!mgr.isOnGround());
    sample(mgr, kFloorReadingM);          // the second consecutive one (flip_confirm_samples = 2) / 2 回連続
    ASSERT_TRUE(mgr.isOnGround());
}

TEST(ground_flag_is_not_set_by_a_reading_exactly_at_the_ground_band)
{
    sf::TakeoffLandingMgr mgr = makeManager(kAirborneM);
    samples(mgr, kClimbReadingM, 3);
    samples(mgr, kGroundBandM, 5);        // "distance < ground_tof_m": exactly 0.05 m is not on the ground / ちょうど 0.05 m は接地でない
    ASSERT_TRUE(!mgr.isOnGround());
}

// ----------------------------------------------------------------------------
// 2. The estimator-release fact
// ----------------------------------------------------------------------------

TEST(release_starts_held_and_a_floor_reading_keeps_it_held)
{
    sf::TakeoffLandingMgr mgr = makeManager(kLowRelease);
    ASSERT_TRUE(!mgr.isEstimatorReleased());
    samples(mgr, kFloorReadingM, 10);
    ASSERT_TRUE(!mgr.isEstimatorReleased());
}

TEST(release_with_its_own_height_comes_before_the_ground_flag_and_only_between_the_two_heights)
{
    sf::TakeoffLandingMgr mgr = makeManager(kLowRelease);
    sample(mgr, kLowRelease);             // exactly at the release height is not above it / ちょうど解除の高さは「超える」でない
    ASSERT_TRUE(!mgr.isEstimatorReleased() && mgr.isOnGround());
    sample(mgr, kLowRelease + 0.001f);    // released, the ground flag is not / 解除、接地フラグはまだ
    ASSERT_TRUE(mgr.isEstimatorReleased() && mgr.isOnGround());
    sample(mgr, kHopReadingM);
    ASSERT_TRUE(mgr.isEstimatorReleased() && mgr.isOnGround());
    sample(mgr, kAirborneM + 0.001f);     // both now / 両方
    ASSERT_TRUE(mgr.isEstimatorReleased() && !mgr.isOnGround());
}

TEST(release_is_a_level_a_hop_that_never_reaches_the_airborne_side_re_holds_when_it_lands)
{
    sf::TakeoffLandingMgr mgr = makeManager(kLowRelease);
    samples(mgr, kHopReadingM, 5);        // up to 0.10 m, never above 0.15 m / 0.10 m まで上がり 0.15 m には届かない
    ASSERT_TRUE(mgr.isEstimatorReleased() && mgr.isOnGround());
    samples(mgr, kFloorReadingM, 1);      // one floor reading: not yet (debounced like the ground flag) / 1 回ではまだ
    ASSERT_TRUE(mgr.isEstimatorReleased());
    samples(mgr, kFloorReadingM, 1);      // the second one confirms the floor: the hold is back, with no airborne edge in between / 2 回目で床を確定: 固定が戻る
    ASSERT_TRUE(!mgr.isEstimatorReleased() && mgr.isOnGround());
}

TEST(release_is_not_dropped_by_a_reading_in_the_band_below_the_release_height_after_release)
{
    sf::TakeoffLandingMgr mgr = makeManager(kLowRelease);
    samples(mgr, kClimbReadingM, 3);
    samples(mgr, 0.06f, 20);              // above the ground band, below the release height: the level holds / 床の帯の上、解除の高さの下: レベルは保つ
    ASSERT_TRUE(mgr.isEstimatorReleased());
}

TEST(release_ignores_invalid_samples_both_ways)
{
    sf::TakeoffLandingMgr mgr = makeManager(kLowRelease);
    for (int i = 0; i < 10; ++i) sample(mgr, 1.0f, false);   // a huge distance that is invalid proves nothing / 無効な大きな距離は何も示さない
    ASSERT_TRUE(!mgr.isEstimatorReleased());
    samples(mgr, kClimbReadingM, 3);
    for (int i = 0; i < 10; ++i) sample(mgr, 0.0f, false);
    ASSERT_TRUE(mgr.isEstimatorReleased());
}

TEST(release_is_set_by_mark_airborne_and_cleared_by_disarm)
{
    sf::TakeoffLandingMgr mgr = makeManager(kLowRelease);
    mgr.markAirborne();                   // armed, ToF silent: nothing proves the floor / ARM 中の ToF 出力停止
    ASSERT_TRUE(mgr.isEstimatorReleased() && !mgr.isOnGround());
    sf::TofData tof{};
    tof.valid = false;
    g_mock_esp_time_us += 33000;
    mgr.update(tof, false, 0.0f, false);  // a disarmed cycle: on the ground, pinned / DISARM 中の周期: 接地、固定
    ASSERT_TRUE(!mgr.isEstimatorReleased() && mgr.isOnGround());
}

TEST(release_is_cleared_by_every_disarmed_cycle_after_a_flight)
{
    sf::TakeoffLandingMgr mgr = makeManager(kLowRelease);
    samples(mgr, kClimbReadingM, 3);
    ASSERT_TRUE(mgr.isEstimatorReleased());
    sf::TofData tof{};
    tof.distance = kClimbReadingM;
    tof.valid    = true;
    g_mock_esp_time_us += 33000;
    mgr.update(tof, false, 0.0f, false);  // disarmed in the air (held in the hand): the hold is back / 空中で DISARM（手で持つ）
    ASSERT_TRUE(!mgr.isEstimatorReleased());
}

// ----------------------------------------------------------------------------
// 3. Equal heights: the new fact equals !isOnGround() on every sample of the real flights
// ----------------------------------------------------------------------------

TEST(with_equal_heights_the_release_equals_not_on_the_ground_on_every_sample_of_the_real_flights)
{
    size_t compared = 0;
    size_t released_samples = 0;
    size_t edges = 0;
    for (const flight_samples::Segment& segment : flight_samples::kSegments) {
        sf::TakeoffLandingMgr mgr = makeManager(kAirborneM);   // set explicitly: the test must survive a change of the default / 明示: 既定値を変えても試験は残る
        bool previous = mgr.isEstimatorReleased();
        for (size_t i = 0; i < segment.count; ++i) {
            const bool valid = segment.mm[i] >= 0;
            sample(mgr, valid ? static_cast<float>(segment.mm[i]) / 1000.0f : 0.0f, valid);
            ASSERT_TRUE(mgr.isEstimatorReleased() == !mgr.isOnGround());
            if (mgr.isEstimatorReleased()) ++released_samples;
            if (mgr.isEstimatorReleased() != previous) ++edges;
            previous = mgr.isEstimatorReleased();
            ++compared;
        }
    }
    printf("(%zu samples, %zu released, %zu edges) ", compared, released_samples, edges);
    ASSERT_TRUE(compared > 1000);         // not a vacuous comparison / 空の比較でない
    ASSERT_TRUE(released_samples > 100);
    ASSERT_TRUE(edges >= 6);              // each takeoff segment releases at least once / 離陸の区間ごとに 1 回は解除
}

TEST(with_its_own_height_the_release_comes_earlier_on_every_real_takeoff_and_re_holds_with_the_ground_flag)
{
    size_t takeoffs_checked = 0;
    for (const flight_samples::Segment& segment : flight_samples::kSegments) {
        sf::TakeoffLandingMgr own = makeManager(kLowRelease);
        sf::TakeoffLandingMgr same = makeManager(kAirborneM);
        for (size_t i = 0; i < segment.count; ++i) {
            const bool valid = segment.mm[i] >= 0;
            const float distance = valid ? static_cast<float>(segment.mm[i]) / 1000.0f : 0.0f;
            sample(own, distance, valid);
            sample(same, distance, valid);
            ASSERT_TRUE(own.isOnGround() == same.isOnGround());               // the ground flag is untouched by the new height / 接地フラグは新しい高さの影響を受けない
            ASSERT_TRUE(!same.isEstimatorReleased() || own.isEstimatorReleased());   // earlier or together, never later / 同時か早い、遅くはならない
            ASSERT_TRUE(!(!own.isOnGround()) || own.isEstimatorReleased());   // airborne side reached -> already released / 空中側に届いたら解除済み
        }
        ++takeoffs_checked;
    }
    ASSERT_TRUE(takeoffs_checked == sizeof(flight_samples::kSegments) / sizeof(flight_samples::kSegments[0]));
}

// ----------------------------------------------------------------------------
// 4. The constants
// ----------------------------------------------------------------------------

TEST(constants_keep_the_order_ground_band_below_release_height_below_or_at_the_airborne_side)
{
    ASSERT_TRUE(sf::limits::kTakeoffGroundBandM < sf::limits::kEstimatorReleaseHeightM);
    ASSERT_TRUE(sf::limits::kEstimatorReleaseHeightM <= sf::limits::kTakeoffAirborneBandM);
    const sf::TakeoffLandingConfig defaults;
    ASSERT_TRUE(defaults.estimator_release_tof_m == sf::limits::kEstimatorReleaseHeightM);
    ASSERT_TRUE(defaults.ground_tof_m < defaults.estimator_release_tof_m && defaults.estimator_release_tof_m <= defaults.airborne_tof_m);
}

int main()
{
    printf("=== TakeoffLandingMgr: ground-flag hysteresis and estimator release ===\n");
    run_ground_flag_starts_on_the_ground_and_a_floor_reading_does_not_release_it();
    run_ground_flag_flips_to_air_at_once_on_a_reading_above_the_airborne_side_and_not_at_it();
    run_ground_flag_holds_in_the_band_when_descending_from_the_air();
    run_ground_flag_needs_consecutive_valid_floor_readings_to_come_back();
    run_ground_flag_is_not_set_by_a_reading_exactly_at_the_ground_band();
    run_release_starts_held_and_a_floor_reading_keeps_it_held();
    run_release_with_its_own_height_comes_before_the_ground_flag_and_only_between_the_two_heights();
    run_release_is_a_level_a_hop_that_never_reaches_the_airborne_side_re_holds_when_it_lands();
    run_release_is_not_dropped_by_a_reading_in_the_band_below_the_release_height_after_release();
    run_release_ignores_invalid_samples_both_ways();
    run_release_is_set_by_mark_airborne_and_cleared_by_disarm();
    run_release_is_cleared_by_every_disarmed_cycle_after_a_flight();
    run_with_equal_heights_the_release_equals_not_on_the_ground_on_every_sample_of_the_real_flights();
    run_with_its_own_height_the_release_comes_earlier_on_every_real_takeoff_and_re_holds_with_the_ground_flag();
    run_constants_keep_the_order_ground_band_below_release_height_below_or_at_the_airborne_side();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n", tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
