/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_barometric_height.cpp
 * @brief Host tests for sf::BarometricHeightEstimator (floor zero, learned offset, hold, repeated samples)
 *        sf::BarometricHeightEstimator（床のゼロ、学習するオフセット、保持、繰り返しサンプル）のホスト試験
 *
 * Pure logic, no ESP-IDF / FreeRTOS. Own TEST/ASSERT framework and main(), same pattern as test_tof_silence.cpp.
 * The estimator is fed synthetic barometer samples at the 13.3 Hz conversion rate, repeated three times as BaroTask publishes
 * them (50 Hz), with a known absolute altitude, a thrust term and a drift, and a ToF at 30 Hz.
 * 合成した気圧計のサンプルを、BaroTask の発行（50 Hz）のとおり変換レート 13.3 Hz の値を 3 回ずつ繰り返して与える。
 * 既知の絶対高度・推力の項・ドリフトと、30 Hz の ToF を使う。
 *
 * @design detailed_design.md §8 — barometric height, observation only  [OK]
 */

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "barometric_height_estimator.hpp"

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-60s ", #name); \
        try { test_##name(); tests_passed++; printf("PASS\n"); } \
        catch (...) { tests_failed++; printf("FAIL\n"); } \
    } \
    static void test_##name()

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        printf("FAIL: %s:%d: condition false\n", __FILE__, __LINE__); \
        throw 1; \
    }

#define ASSERT_NEAR(a, b, tol) \
    if (std::fabs((a) - (b)) > (tol)) { \
        printf("FAIL: %s:%d: %f vs %f (tol %f)\n", __FILE__, __LINE__, (double)(a), (double)(b), (double)(tol)); \
        throw 1; \
    }

namespace {

constexpr uint32_t kConversionPeriodUs = 75000;   // 13.3 Hz conversions / 変換 13.3 Hz
constexpr uint32_t kPublishPeriodUs    = 20000;   // 50 Hz BaroTask / BaroTask 50 Hz
constexpr uint32_t kTofPeriodUs        = 33000;   // 30 Hz ToF
constexpr float    kAbsoluteAltitudeM  = 780.0f;  // weather and place / 天候と場所
constexpr float    kPressureSlopePaPerM = -12.0f;

/// A fake sensor world: real height, thrust term (the wash raises the reading), drift. 擬似の世界: 実際の高さ、推力の項、ドリフト。
struct World {
    float    height_m     = 0.0f;
    float    wash_m       = 0.0f;
    float    drift_m_per_s = 0.0f;   // rate of the drift / ドリフトの速さ
    float    drift_m      = 0.0f;   // accumulated drift / 積算したドリフト
    float    tilt_rad     = 0.0f;
    bool     armed        = false;
    bool     on_ground    = true;
    bool     tof_alive    = true;
    uint32_t now_us       = 1000000;
    uint32_t next_conversion_us = 0;
    uint32_t next_tof_us  = 0;
    float    conversion_altitude_m = kAbsoluteAltitudeM;
    float    conversion_counter = 0.0f;
};

/// Advance 20 ms: a new conversion every 75 ms (distinct pressure), the ToF every 33 ms. 20 ms 進める。
void step(sf::BarometricHeightEstimator& estimator, World& w)
{
    w.now_us += kPublishPeriodUs;
    w.drift_m += w.drift_m_per_s * static_cast<float>(kPublishPeriodUs) / 1.0e6f;
    if (w.now_us >= w.next_conversion_us) {
        w.next_conversion_us = w.now_us + kConversionPeriodUs;
        w.conversion_counter += 1.0f;
        // A tiny sawtooth stands for the noise so that consecutive conversions differ. 変換ごとに値が変わるよう、小さなのこぎり波をノイズ代わりにする。
        const float noise = 0.002f * std::fmod(w.conversion_counter, 7.0f);
        w.conversion_altitude_m = kAbsoluteAltitudeM + w.height_m + w.wash_m + w.drift_m + noise;
    }
    if (w.tof_alive && w.now_us >= w.next_tof_us) {
        w.next_tof_us = w.now_us + kTofPeriodUs;
        sf::BarometricToFSample tof{};
        tof.height_m = w.height_m;
        tof.tilt_rad = w.tilt_rad;
        tof.timestamp_us = w.now_us;
        tof.usable = w.height_m > 0.0f;
        estimator.onToFSample(tof);
    }
    sf::BarometricSample baro{};
    baro.altitude_m = w.conversion_altitude_m;
    baro.pressure_pa = 101000.0f + kPressureSlopePaPerM * (w.conversion_altitude_m - kAbsoluteAltitudeM);
    baro.timestamp_us = w.now_us;
    estimator.onBaroSample(baro, {w.armed, w.on_ground});
}

void run_for(sf::BarometricHeightEstimator& estimator, World& w, float seconds)
{
    const int steps = static_cast<int>(seconds * 1.0e6f / static_cast<float>(kPublishPeriodUs));
    for (int i = 0; i < steps; ++i) step(estimator, w);
}

float height(const sf::BarometricHeightEstimator& estimator, const World& w) { return estimator.output(w.now_us).height_m; }

}  // namespace

TEST(nothing_is_valid_before_the_first_sample)
{
    sf::BarometricHeightEstimator estimator;
    const sf::BarometricHeightOutput out = estimator.output(0);
    ASSERT_TRUE(!out.valid && !out.learned);
    ASSERT_TRUE(out.offset_age_s == sf::BarometricHeightEstimator::kNeverLearnedAgeS);
}

TEST(floor_zero_is_the_average_of_the_absolute_altitude_while_disarmed_on_the_floor)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    run_for(estimator, w, 6.0f);
    ASSERT_TRUE(estimator.output(w.now_us).valid);
    ASSERT_NEAR(estimator.zeroM(), kAbsoluteAltitudeM, 0.05f);
    ASSERT_NEAR(height(estimator, w), 0.0f, 0.05f);   // an absolute 780 m reads as 0 m above the floor / 絶対 780 m が床から 0 m と読める
}

TEST(floor_zero_is_frozen_at_the_arm_and_the_propeller_wash_does_not_move_it)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    run_for(estimator, w, 6.0f);
    const float zero_before_arm = estimator.zeroM();
    w.armed = true;
    w.wash_m = 0.3f;   // the motors run on the floor / 床でモータが回る
    run_for(estimator, w, 3.0f);
    ASSERT_NEAR(estimator.zeroM(), zero_before_arm, 1.0e-6f);
    ASSERT_NEAR(height(estimator, w), 0.3f, 0.05f);   // floor zero only: the wash is in the height / 床のゼロだけ: 吹き下ろしは高さに入る
}

TEST(offset_is_learned_from_the_tof_in_flight_and_removes_the_wash_term)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    run_for(estimator, w, 6.0f);
    w.armed = true; w.on_ground = false; w.wash_m = 0.6f; w.height_m = 0.8f;
    run_for(estimator, w, 20.0f);
    const sf::BarometricHeightOutput out = estimator.output(w.now_us);
    ASSERT_TRUE(out.learned);
    ASSERT_NEAR(out.offset_m, 0.6f, 0.05f);
    ASSERT_NEAR(out.height_m, 0.8f, 0.05f);
    ASSERT_TRUE(out.offset_age_s < 0.1f);
}

TEST(offset_is_held_when_the_tof_is_silent_and_the_age_grows)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    run_for(estimator, w, 6.0f);
    w.armed = true; w.on_ground = false; w.wash_m = 0.6f; w.height_m = 0.8f;
    run_for(estimator, w, 20.0f);
    const float offset = estimator.output(w.now_us).offset_m;
    w.tof_alive = false;
    w.height_m = 0.4f;   // the craft sinks: the barometer must follow it with the held offset / 機体が沈む: 気圧計は保持したオフセットで追う
    run_for(estimator, w, 5.0f);
    const sf::BarometricHeightOutput out = estimator.output(w.now_us);
    ASSERT_NEAR(out.offset_m, offset, 0.02f);   // a ToF sample up to 0.1 s old may still be paired in the first 0.1 s / 最初の 0.1 s は最大 0.1 s 古い ToF サンプルが組になりうる
    ASSERT_NEAR(out.height_m, 0.4f, 0.05f);
    ASSERT_NEAR(out.offset_age_s, 5.0f, 0.2f);
}

TEST(a_drift_during_the_silence_is_the_error_and_is_not_learned)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    run_for(estimator, w, 6.0f);
    w.armed = true; w.on_ground = false; w.height_m = 0.8f;
    run_for(estimator, w, 20.0f);
    w.tof_alive = false;
    w.drift_m_per_s = 0.02f;
    run_for(estimator, w, 5.0f);
    ASSERT_NEAR(height(estimator, w) - w.height_m, 0.1f, 0.03f);   // 0.02 m/s x 5 s
}

TEST(learning_follows_a_slow_drift_while_the_tof_is_alive)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    run_for(estimator, w, 6.0f);
    w.armed = true; w.on_ground = false; w.height_m = 0.8f; w.drift_m_per_s = 0.02f;
    run_for(estimator, w, 30.0f);
    ASSERT_NEAR(height(estimator, w) - 0.8f, 0.02f * sf::limits::kBaroLearnTauS, 0.03f);   // the first-order filter lags a ramp by rate x tau (0.1 m) / 1 次フィルタはランプに「速さ × 時定数」（0.1 m）遅れる
}

TEST(repeated_samples_change_nothing_and_are_not_counted)
{
    sf::BarometricHeightEstimator estimator;
    sf::BarometricSample sample{kAbsoluteAltitudeM, 101000.0f, 1000000};
    ASSERT_TRUE(estimator.onBaroSample(sample, {false, true}));
    sample.timestamp_us += kPublishPeriodUs;
    ASSERT_TRUE(!estimator.onBaroSample(sample, {false, true}));   // the same conversion / 同じ変換
    sample.timestamp_us += kPublishPeriodUs;
    ASSERT_TRUE(!estimator.onBaroSample(sample, {false, true}));
    sample.pressure_pa += 0.2f;
    sample.timestamp_us += kPublishPeriodUs;
    ASSERT_TRUE(estimator.onBaroSample(sample, {false, true}));    // a new conversion / 新しい変換
}

TEST(a_gap_in_the_conversions_does_not_make_one_sample_weigh_more_than_a_regular_one)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    run_for(estimator, w, 6.0f);
    w.armed = true; w.on_ground = false; w.height_m = 0.8f;
    run_for(estimator, w, 20.0f);
    const float offset = estimator.output(w.now_us).offset_m;
    w.now_us += 4000000;                 // the barometer was silent for 4 s / 気圧計が 4 s 出力しなかった
    w.next_conversion_us = w.now_us;
    w.conversion_altitude_m += 1.0f;     // and one wild sample comes back / 外れた 1 サンプルが戻る
    step(estimator, w);
    ASSERT_TRUE(std::fabs(estimator.output(w.now_us).offset_m - offset) < 0.3f);   // 1.0 m x (0.25 s / 5 s) at most / 高々 1.0 m × (0.25 s / 5 s)
}

TEST(a_tof_that_jumps_beyond_the_gate_is_not_learned)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    run_for(estimator, w, 6.0f);
    w.armed = true; w.on_ground = false; w.height_m = 0.8f;
    run_for(estimator, w, 20.0f);
    const float offset = estimator.output(w.now_us).offset_m;
    w.height_m = 0.8f + 3.0f;   // the barometer is right, but then the "ToF" says 3 m higher: a reflection / 気圧計は正しいのに ToF が 3 m 高く言う（反射）
    sf::BarometricToFSample tof{3.8f, 0.0f, w.now_us, true};
    estimator.onToFSample(tof);
    sf::BarometricSample baro{kAbsoluteAltitudeM + 0.8f + 0.001f, 100123.0f, w.now_us + kPublishPeriodUs};
    estimator.onBaroSample(baro, {true, false});
    ASSERT_NEAR(estimator.output(baro.timestamp_us).offset_m, offset, 1.0e-6f);
}

TEST(no_learning_below_the_minimum_height_above_the_maximum_height_or_beyond_the_tilt_limit)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    run_for(estimator, w, 6.0f);
    w.armed = true; w.on_ground = false; w.wash_m = 0.5f;
    w.height_m = 0.05f;                                           // below kBaroLearnMinHeightM / 下限未満
    run_for(estimator, w, 1.0f);
    ASSERT_TRUE(!estimator.output(w.now_us).learned);
    w.height_m = 1.55f;                                           // above kBaroLearnMaxHeightM / 上限超え
    run_for(estimator, w, 1.0f);
    ASSERT_TRUE(!estimator.output(w.now_us).learned);
    w.height_m = 0.8f; w.tilt_rad = sf::limits::kBaroLearnMaxTiltRad + 0.05f;   // tilted / 傾きすぎ
    run_for(estimator, w, 1.0f);
    ASSERT_TRUE(!estimator.output(w.now_us).learned);
    w.tilt_rad = 0.0f;
    run_for(estimator, w, 1.0f);
    ASSERT_TRUE(estimator.output(w.now_us).learned);
}

TEST(a_stale_tof_sample_is_not_paired_with_a_barometer_sample)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    run_for(estimator, w, 6.0f);
    w.armed = true; w.on_ground = false; w.height_m = 0.8f;
    w.tof_alive = false;   // the ToF has never delivered a sample in flight / 飛行中に ToF が一度もサンプルを出していない
    run_for(estimator, w, 3.0f);
    ASSERT_TRUE(!estimator.output(w.now_us).learned);
}

TEST(a_disarm_forgets_the_offset_and_the_floor_zero_is_taken_again)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    run_for(estimator, w, 6.0f);
    w.armed = true; w.on_ground = false; w.wash_m = 0.6f; w.height_m = 0.8f;
    run_for(estimator, w, 10.0f);
    ASSERT_TRUE(estimator.output(w.now_us).learned);
    w.armed = false; w.on_ground = true; w.wash_m = 0.0f; w.height_m = 0.0f; w.drift_m_per_s = 0.0f;
    run_for(estimator, w, 15.0f);
    const sf::BarometricHeightOutput out = estimator.output(w.now_us);
    ASSERT_TRUE(!out.learned && out.offset_m == 0.0f && out.valid);
    ASSERT_NEAR(out.height_m, 0.0f, 0.05f);
}

TEST(armed_in_the_air_without_a_floor_zero_becomes_valid_only_when_the_offset_is_learned)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    w.armed = true; w.on_ground = false; w.height_m = 0.8f; w.tof_alive = false;
    run_for(estimator, w, 2.0f);
    ASSERT_TRUE(!estimator.output(w.now_us).valid);
    w.tof_alive = true;
    run_for(estimator, w, 3.0f);
    const sf::BarometricHeightOutput out = estimator.output(w.now_us);
    ASSERT_TRUE(out.valid && out.learned);
    ASSERT_NEAR(out.height_m, 0.8f, 0.05f);
}

TEST(learning_can_be_switched_off_for_the_floor_zero_only_baseline)
{
    sf::BarometricHeightEstimator estimator;
    sf::BarometricHeightConfig config;
    config.learning_enabled = false;
    estimator.setConfig(config);
    World w;
    run_for(estimator, w, 6.0f);
    w.armed = true; w.on_ground = false; w.wash_m = 0.6f; w.height_m = 0.8f;
    run_for(estimator, w, 10.0f);
    ASSERT_TRUE(!estimator.output(w.now_us).learned);
    ASSERT_NEAR(height(estimator, w), 1.4f, 0.05f);
}

TEST(timestamps_wrap_around_uint32_without_a_jump)
{
    sf::BarometricHeightEstimator estimator;
    World w;
    w.now_us = 0xFFFFFFFFu - 3000000u;   // 3 s before the 71.6 min wrap / 巻き戻りの 3 s 前
    w.next_conversion_us = w.now_us;
    run_for(estimator, w, 3.0f);
    w.armed = true; w.on_ground = false; w.wash_m = 0.6f; w.height_m = 0.8f;
    run_for(estimator, w, 20.0f);   // crosses the wrap / 巻き戻りを跨ぐ
    const sf::BarometricHeightOutput out = estimator.output(w.now_us);
    ASSERT_TRUE(out.learned);
    ASSERT_NEAR(out.height_m, 0.8f, 0.05f);
    ASSERT_TRUE(out.offset_age_s < 0.1f);
}

int main()
{
    printf("=== BarometricHeightEstimator host tests ===\n");
    run_nothing_is_valid_before_the_first_sample();
    run_floor_zero_is_the_average_of_the_absolute_altitude_while_disarmed_on_the_floor();
    run_floor_zero_is_frozen_at_the_arm_and_the_propeller_wash_does_not_move_it();
    run_offset_is_learned_from_the_tof_in_flight_and_removes_the_wash_term();
    run_offset_is_held_when_the_tof_is_silent_and_the_age_grows();
    run_a_drift_during_the_silence_is_the_error_and_is_not_learned();
    run_learning_follows_a_slow_drift_while_the_tof_is_alive();
    run_repeated_samples_change_nothing_and_are_not_counted();
    run_a_gap_in_the_conversions_does_not_make_one_sample_weigh_more_than_a_regular_one();
    run_a_tof_that_jumps_beyond_the_gate_is_not_learned();
    run_no_learning_below_the_minimum_height_above_the_maximum_height_or_beyond_the_tilt_limit();
    run_a_stale_tof_sample_is_not_paired_with_a_barometer_sample();
    run_a_disarm_forgets_the_offset_and_the_floor_zero_is_taken_again();
    run_armed_in_the_air_without_a_floor_zero_becomes_valid_only_when_the_offset_is_learned();
    run_learning_can_be_switched_off_for_the_floor_zero_only_baseline();
    run_timestamps_wrap_around_uint32_without_a_jump();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n", tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
