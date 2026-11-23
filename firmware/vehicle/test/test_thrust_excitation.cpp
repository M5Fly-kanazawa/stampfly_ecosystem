/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_thrust_excitation.cpp
 * @brief Host tests of the thrust excitation: waveform, start and abort judgement, state machine, and its place in the controller
 *        推力励振のホスト試験: 波形・開始と打ち切りの判断・状態機械・制御器の中での扱い
 *
 * Part 1 (header only, thrust_excitation.hpp): the multisine (peak, period, table against std::sin), the start conditions with their reason
 * codes, the abort conditions, the state machine (Standby, ramps, run, end, abort ramp without a step, cancel, reset), the length, the step
 * edges, the learning hold-off.
 * Part 2 (pid_controller.cpp compiled unmodified, params and the clock from simulator/sils/compat): the output of the controller is
 * the same bit for bit when no excitation runs (also after a refused request), the excitation adds exactly the waveform to
 * output.thrust, the hover-thrust learner does not move during and shortly after a run, the abort causes that the controller reads
 * itself (mode, stick, height window, ToF, battery), the exclusion with the rate-axis excitation, and reset().
 * 第 1 部（ヘッダのみ）: 多重正弦波（ピーク・周期・表と std::sin の一致）、理由の符号つきの開始条件、打ち切り条件、状態機械（待機・
 * 立ち上げ・実行・終了・段差のない打ち切りの立ち下げ・取り消し・reset）、長さ、階段の段、学習の停止時間。
 * 第 2 部（pid_controller.cpp を無改変でコンパイル。パラメータと時計は simulator/sils/compat）: 励振が動かないとき制御器の出力は
 * ビット単位で同じ（拒否された要求の後も）、励振は output.thrust に波形をちょうど足す、ホバー推力の学習は実行中と直後に動かない、
 * 制御器が自分で読む打ち切りの原因（モード・スティック・高さの窓・ToF・電池）、角速度の軸の励振との排他、reset()。
 *
 * @design detailed_design.md §3 — thrust excitation [OK]
 * @design coding_and_education.md §2 — bilingual comments, named constants [OK]
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

// The controller tests read a private member (hover_thrust_) to check that the learner stays still.
// 制御器の試験は、学習が止まっていることを確かめるために private のメンバー（hover_thrust_）を読む。
#define private public
#include "params.hpp"
#include "pid_controller.hpp"
#undef private

using namespace sf;

extern "C" void vTaskDelay(TickType_t) {}

// =============================================================================
// Test framework (self-registering TEST, own main)
// 試験の枠組み（自己登録の TEST、独自の main）
// =============================================================================

struct TestCase {
    const char* name;
    void (*body)();
};

static std::vector<TestCase>& registry()
{
    static std::vector<TestCase> cases;
    return cases;
}

struct Registrar {
    Registrar(const char* name, void (*body)()) { registry().push_back({name, body}); }
};

struct AssertionFailure {};

#define TEST(name) \
    static void test_##name(); \
    static Registrar registrar_##name(#name, test_##name); \
    static void test_##name()

#define ASSERT_MSG(cond, ...) \
    do { \
        if (!(cond)) { \
            printf("\n      FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
            printf(__VA_ARGS__); \
            printf("\n"); \
            throw AssertionFailure{}; \
        } \
    } while (0)

// =============================================================================
// Named constants
// 名前つき定数
// =============================================================================

namespace lim = sf::limits;
static constexpr float kControlPeriodS = 0.0025f;          ///< 400 Hz control cycle [s] / 400 Hz の制御周期
static constexpr uint32_t kControlPeriodUs = 2500;         ///< the same [us] / 同じ値 [us]
static constexpr uint32_t kStartTimeUs = 1000000;          ///< first virtual time (0 means "never") / 仮想時間の開始
static constexpr float kHoverThrustN = 0.407f;             ///< the controller's default hover thrust [N] / 制御器の既定のホバー推力
static constexpr float kHoverHeightM = 0.5f;               ///< hover height of the rig [m] / 試験台のホバー高さ
static constexpr float kGoodBatteryV = 4.0f;               ///< a healthy battery [V] / 健全な電池
static constexpr float kAmplitudePercent = 5.0f;           ///< the amplitude used by most tests [%] / 多くの試験が使う振幅
static constexpr float kMultisineLengthS = 20.0f;          ///< the length used by most tests [s] / 多くの試験が使う長さ
static constexpr float kPeakTolerance = 1.0e-4f;           ///< tolerance on a normalised peak / 正規化したピークの許容
static constexpr float kSineTolerance = 2.0e-7f;           ///< table against std::sin / 表と std::sin の差の許容
static constexpr int kSettleCycles = 600;                  ///< 1.5 s: past the 0.5 s of Standby and the 0.5 s ramp-up / 1.5 s: 待機の 0.5 s と立ち上げの 0.5 s を過ぎる
static constexpr int kStandbyCycles = 200;                 ///< 0.5 s of stillness that ends Standby / 待機を終える静止 0.5 s
static constexpr int kQuarterCycle = 100;                  ///< 0.25 s in cycles / 0.25 s の周期数

static bool near(float value, float expected, float tolerance)
{
    return std::fabs(value - expected) <= tolerance;
}

/// Facts of a craft that hovers still at the target inside every window. / 全ての窓の中でターゲットに静止してホバーする機体の事実。
static ThrustExcitationFacts stillFacts()
{
    ThrustExcitationFacts facts{};
    facts.airborne = true;
    facts.altitude_mode = true;
    facts.altitude_m = kHoverHeightM;
    facts.altitude_target_m = kHoverHeightM;
    facts.battery_voltage = kGoodBatteryV;
    facts.tof_usable = true;
    facts.guidance_still = true;
    return facts;
}

static ThrustExcitationCommand multisineCommand(float amplitude_percent, float length_s)
{
    ThrustExcitationCommand command{};
    command.kind = static_cast<uint8_t>(ThrustExcitationKind::Multisine);
    command.amplitude_percent = amplitude_percent;
    command.duration_s = length_s;
    return command;
}

static ThrustExcitationCommand stepsCommand(float amplitude_percent, float half_period_s, uint16_t count)
{
    ThrustExcitationCommand command{};
    command.kind = static_cast<uint8_t>(ThrustExcitationKind::Steps);
    command.amplitude_percent = amplitude_percent;
    command.half_period_s = half_period_s;
    command.count = count;
    return command;
}

/// Run an excitation to its end with still facts and collect every cycle's value [N]. / 静止した事実で励振を最後まで流し、各周期の値 [N] を集める。
static std::vector<float> runToEnd(ThrustExcitation& excitation, const ThrustExcitationFacts& facts, int limit_cycles)
{
    std::vector<float> values;
    for (int cycle = 0; cycle < limit_cycles && excitation.isActive(); ++cycle) {
        values.push_back(excitation.step(facts));
    }
    return values;
}

// =============================================================================
// Part 1: the header
// 第 1 部: ヘッダ
// =============================================================================

TEST(sine_table_matches_std_sin_over_the_whole_period)
{
    float worst = 0.0f;
    for (uint32_t index = 0; index < lim::kThrustExcitePeriodCycles; ++index) {
        const double exact = std::sin(2.0 * 3.14159265358979323846 * index / lim::kThrustExcitePeriodCycles);
        worst = std::fmax(worst, static_cast<float>(std::fabs(thrustExciteSine(index) - exact)));
    }
    ASSERT_MSG(worst <= kSineTolerance, "table error %g", static_cast<double>(worst));
}

TEST(multisine_peak_is_one_repeats_and_has_no_dc)
{
    float peak = 0.0f;
    double sum = 0.0;
    for (uint32_t cycle = 0; cycle < lim::kThrustExcitePeriodCycles; ++cycle) {
        const float value = thrustExciteMultisine(cycle);
        peak = std::fmax(peak, std::fabs(value));
        sum += value;
        ASSERT_MSG(value == thrustExciteMultisine(cycle + lim::kThrustExcitePeriodCycles), "not periodic at %u", cycle);
    }
    ASSERT_MSG(near(peak, 1.0f, kPeakTolerance), "peak %g, expected 1", static_cast<double>(peak));
    ASSERT_MSG(std::fabs(sum / lim::kThrustExcitePeriodCycles) < 1.0e-4, "mean %g", sum / lim::kThrustExcitePeriodCycles);
}

TEST(request_decision_reports_each_reason_in_its_order)
{
    const ThrustExcitationCommand command = multisineCommand(kAmplitudePercent, kMultisineLengthS);
    ThrustExcitationFacts facts = stillFacts();
    ASSERT_MSG(decideThrustExcitationRequest(command, facts, false) == ThrustExcitationReason::None, "a valid request was refused");

    ThrustExcitationFacts changed = facts;
    changed.airborne = false;
    ASSERT_MSG(decideThrustExcitationRequest(command, changed, false) == ThrustExcitationReason::NotAirborne, "not airborne");
    changed = facts; changed.altitude_mode = false;
    ASSERT_MSG(decideThrustExcitationRequest(command, changed, false) == ThrustExcitationReason::ModeNotAllowed, "mode");
    ASSERT_MSG(decideThrustExcitationRequest(command, facts, true) == ThrustExcitationReason::Busy, "already active");
    changed = facts; changed.flip_active = true;
    ASSERT_MSG(decideThrustExcitationRequest(command, changed, false) == ThrustExcitationReason::Busy, "flip");
    changed = facts; changed.rate_excitation_active = true;
    ASSERT_MSG(decideThrustExcitationRequest(command, changed, false) == ThrustExcitationReason::Busy, "rate excitation");
    changed = facts; changed.battery_voltage = lim::kThrustExciteBatteryStartMinV - 0.01f;
    ASSERT_MSG(decideThrustExcitationRequest(command, changed, false) == ThrustExcitationReason::BatteryLow, "battery");
    changed = facts; changed.altitude_m = lim::kThrustExciteStartHeightMinM - 0.01f;
    ASSERT_MSG(decideThrustExcitationRequest(command, changed, false) == ThrustExcitationReason::HeightOutOfRange, "height low");
    changed = facts; changed.altitude_m = lim::kThrustExciteStartHeightMaxM + 0.01f;
    ASSERT_MSG(decideThrustExcitationRequest(command, changed, false) == ThrustExcitationReason::HeightOutOfRange, "height high");
    changed = facts; changed.tof_usable = false;
    ASSERT_MSG(decideThrustExcitationRequest(command, changed, false) == ThrustExcitationReason::ToFInvalid, "tof");
    changed = facts; changed.tilt_rad = lim::kThrustExciteStartTiltMaxRad + 0.01f;
    ASSERT_MSG(decideThrustExcitationRequest(command, changed, false) == ThrustExcitationReason::TiltTooLarge, "tilt");
    // The edges are inside. / 境界は内側。
    changed = facts; changed.altitude_m = lim::kThrustExciteStartHeightMinM;
    ASSERT_MSG(decideThrustExcitationRequest(command, changed, false) == ThrustExcitationReason::None, "height at the lower edge");
    changed.altitude_m = lim::kThrustExciteStartHeightMaxM;
    ASSERT_MSG(decideThrustExcitationRequest(command, changed, false) == ThrustExcitationReason::None, "height at the upper edge");
}

TEST(ranges_are_refused_not_clipped)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    ASSERT_MSG(decideThrustExcitationRanges(multisineCommand(10.0f, 20.0f)) == ThrustExcitationReason::None, "10 percent is allowed");
    ASSERT_MSG(decideThrustExcitationRanges(multisineCommand(10.01f, 20.0f)) == ThrustExcitationReason::AmplitudeLimit, "10.01 percent");
    ASSERT_MSG(decideThrustExcitationRanges(multisineCommand(0.0f, 20.0f)) == ThrustExcitationReason::AmplitudeLimit, "0 percent");
    ASSERT_MSG(decideThrustExcitationRanges(multisineCommand(-1.0f, 20.0f)) == ThrustExcitationReason::AmplitudeLimit, "negative");
    ASSERT_MSG(decideThrustExcitationRanges(multisineCommand(nan, 20.0f)) == ThrustExcitationReason::AmplitudeLimit, "NaN amplitude");
    ASSERT_MSG(decideThrustExcitationRanges(multisineCommand(5.0f, nan)) == ThrustExcitationReason::DurationLimit, "NaN length");
    ASSERT_MSG(decideThrustExcitationRanges(multisineCommand(5.0f, inf)) == ThrustExcitationReason::DurationLimit, "infinite length");
    ASSERT_MSG(decideThrustExcitationRanges(multisineCommand(5.0f, 30.0f)) == ThrustExcitationReason::None, "30 s is allowed");
    ASSERT_MSG(decideThrustExcitationRanges(multisineCommand(5.0f, 30.1f)) == ThrustExcitationReason::DurationLimit, "30.1 s");
    ASSERT_MSG(decideThrustExcitationRanges(multisineCommand(5.0f, 2.0f)) == ThrustExcitationReason::None, "2 s is allowed");
    ASSERT_MSG(decideThrustExcitationRanges(multisineCommand(5.0f, 1.9f)) == ThrustExcitationReason::DurationLimit, "1.9 s");
    // Steps: amplitude x half period <= 4.0 %.s, half period >= 0.2 s, whole length <= 30 s.
    // 階段: 振幅 × 半周期 ≤ 4.0 %·s、半周期 ≥ 0.2 s、全体の長さ ≤ 30 s。
    ASSERT_MSG(decideThrustExcitationRanges(stepsCommand(5.0f, 0.8f, 3)) == ThrustExcitationReason::None, "5 percent for 0.8 s is allowed");
    ASSERT_MSG(decideThrustExcitationRanges(stepsCommand(5.0f, 0.81f, 3)) == ThrustExcitationReason::StepLimit, "5 percent for 0.81 s");
    ASSERT_MSG(decideThrustExcitationRanges(stepsCommand(10.0f, 0.4f, 3)) == ThrustExcitationReason::None, "10 percent for 0.4 s is allowed");
    ASSERT_MSG(decideThrustExcitationRanges(stepsCommand(10.0f, 0.5f, 3)) == ThrustExcitationReason::StepLimit, "10 percent for 0.5 s");
    ASSERT_MSG(decideThrustExcitationRanges(stepsCommand(5.0f, 0.1f, 3)) == ThrustExcitationReason::StepLimit, "half period below 0.2 s");
    ASSERT_MSG(decideThrustExcitationRanges(stepsCommand(5.0f, 0.5f, 0)) == ThrustExcitationReason::StepLimit, "zero pairs");
    ASSERT_MSG(decideThrustExcitationRanges(stepsCommand(5.0f, 0.5f, 30)) == ThrustExcitationReason::None, "30 s of steps is allowed");
    ASSERT_MSG(decideThrustExcitationRanges(stepsCommand(5.0f, 0.5f, 31)) == ThrustExcitationReason::DurationLimit, "31 s of steps");
}

TEST(abort_decision_covers_every_cell)
{
    const ThrustExcitationFacts facts = stillFacts();
    ASSERT_MSG(decideThrustExcitationAbort(facts, 0.0f, 0.0f) == ThrustExcitationReason::None, "still facts abort");
    ThrustExcitationFacts changed = facts;
    changed.airborne = false;
    ASSERT_MSG(decideThrustExcitationAbort(changed, 0.0f, 0.0f) == ThrustExcitationReason::FlightEnded, "phase");
    changed = facts; changed.flip_active = true;
    ASSERT_MSG(decideThrustExcitationAbort(changed, 0.0f, 0.0f) == ThrustExcitationReason::Preempted, "flip");
    changed = facts; changed.altitude_mode = false;
    ASSERT_MSG(decideThrustExcitationAbort(changed, 0.0f, 0.0f) == ThrustExcitationReason::ModeNotAllowed, "mode");
    changed = facts; changed.stick_moved = true;
    ASSERT_MSG(decideThrustExcitationAbort(changed, 0.0f, 0.0f) == ThrustExcitationReason::StickMoved, "stick");
    changed = facts; changed.altitude_m = lim::kThrustExciteRunHeightMinM - 0.01f; changed.altitude_target_m = changed.altitude_m;
    ASSERT_MSG(decideThrustExcitationAbort(changed, 0.0f, 0.0f) == ThrustExcitationReason::HeightWindow, "below the window");
    changed = facts; changed.altitude_m = lim::kThrustExciteRunHeightMaxM + 0.01f; changed.altitude_target_m = changed.altitude_m;
    ASSERT_MSG(decideThrustExcitationAbort(changed, 0.0f, 0.0f) == ThrustExcitationReason::HeightWindow, "above the window");
    changed = facts; changed.altitude_target_m = facts.altitude_m + lim::kThrustExciteAltitudeErrorAbortM + 0.01f;
    ASSERT_MSG(decideThrustExcitationAbort(changed, 0.0f, 0.0f) == ThrustExcitationReason::HeightError, "height error");
    changed = facts; changed.altitude_target_m = facts.altitude_m + lim::kThrustExciteAltitudeErrorAbortM - 0.01f;
    ASSERT_MSG(decideThrustExcitationAbort(changed, 0.0f, 0.0f) == ThrustExcitationReason::None, "just inside the height error");
    changed = facts; changed.tof_usable = false;
    ASSERT_MSG(decideThrustExcitationAbort(changed, 0.0f, 0.0f) == ThrustExcitationReason::None, "one lost ToF cycle does not abort");
    ASSERT_MSG(decideThrustExcitationAbort(changed, lim::kThrustExciteToFLostAbortS, 0.0f) == ThrustExcitationReason::ToFInvalid, "ToF lost");
    changed = facts; changed.tilt_rad = lim::kThrustExciteTiltAbortRad + 0.01f;
    ASSERT_MSG(decideThrustExcitationAbort(changed, 0.0f, 0.0f) == ThrustExcitationReason::TiltTooLarge, "tilt");
    ASSERT_MSG(decideThrustExcitationAbort(facts, 0.0f, lim::kThrustExciteBatteryAbortHoldS) == ThrustExcitationReason::BatteryLow, "battery");
}

TEST(multisine_runs_its_length_from_zero_to_zero_inside_the_amplitude)
{
    ThrustExcitation excitation;
    const ThrustExcitationFacts facts = stillFacts();
    ASSERT_MSG(excitation.request(multisineCommand(kAmplitudePercent, kMultisineLengthS), facts, kHoverThrustN) == ThrustExcitationReason::None,
               "request refused");
    ASSERT_MSG(excitation.state() == ThrustExcitationState::Standby && excitation.isStepping(), "no standby");
    std::vector<float> values = runToEnd(excitation, facts, 100000);
    const size_t standby = lim::kThrustExciteSampleRateHz / 2;   // 0.5 s of stillness / 静止 0.5 s
    const size_t run = static_cast<size_t>(kMultisineLengthS * lim::kThrustExciteSampleRateHz);
    ASSERT_MSG(values.size() == standby + run, "%zu cycles, expected %zu", values.size(), standby + run);
    ASSERT_MSG(excitation.result() == ThrustExcitationResult::Completed && excitation.reason() == ThrustExcitationReason::None, "result");
    for (size_t i = 0; i < standby; ++i) {
        ASSERT_MSG(values[i] == 0.0f, "signal during Standby");
    }
    const float limit = kAmplitudePercent * 0.01f * kHoverThrustN;
    float peak = 0.0f;
    for (size_t i = standby; i < values.size(); ++i) {
        peak = std::fmax(peak, std::fabs(values[i]));
    }
    ASSERT_MSG(values[standby] == 0.0f, "starts at %g, expected 0", static_cast<double>(values[standby]));
    ASSERT_MSG(values.back() == 0.0f, "ends at %g, expected 0", static_cast<double>(values.back()));
    ASSERT_MSG(peak <= limit * (1.0f + kPeakTolerance) && peak >= 0.9f * limit, "peak %g, limit %g", static_cast<double>(peak), static_cast<double>(limit));
}

TEST(multisine_signal_is_the_table_waveform_times_the_amplitude_in_the_middle)
{
    ThrustExcitation excitation;
    const ThrustExcitationFacts facts = stillFacts();
    excitation.request(multisineCommand(kAmplitudePercent, kMultisineLengthS), facts, kHoverThrustN);
    const std::vector<float> values = runToEnd(excitation, facts, 100000);
    const size_t standby = lim::kThrustExciteSampleRateHz / 2;
    const uint32_t cycle = 3000;   // well inside the run (the ramps are 200 cycles) / 実行の真ん中（立ち上げは 200 周期）
    const float expected = kAmplitudePercent * 0.01f * kHoverThrustN * thrustExciteMultisine(cycle);
    ASSERT_MSG(near(values[standby + cycle], expected, 1.0e-7f), "%g vs %g", static_cast<double>(values[standby + cycle]), static_cast<double>(expected));
}

TEST(steps_have_true_edges_a_soft_start_and_an_appended_fall)
{
    ThrustExcitation excitation;
    const ThrustExcitationFacts facts = stillFacts();
    constexpr float kHalfPeriodS = 0.5f;
    constexpr uint16_t kPairs = 2;
    ASSERT_MSG(excitation.request(stepsCommand(kAmplitudePercent, kHalfPeriodS, kPairs), facts, kHoverThrustN) == ThrustExcitationReason::None, "refused");
    const std::vector<float> values = runToEnd(excitation, facts, 100000);
    const size_t standby = lim::kThrustExciteSampleRateHz / 2;
    const size_t half = static_cast<size_t>(kHalfPeriodS * lim::kThrustExciteSampleRateHz);
    const size_t body = 2 * kPairs * half;
    ASSERT_MSG(values.size() == standby + body + lim::kThrustExciteStepsRampDownCycles, "%zu cycles", values.size());
    const float level = kAmplitudePercent * 0.01f * kHoverThrustN;
    ASSERT_MSG(values[standby] == 0.0f, "starts at %g", static_cast<double>(values[standby]));
    ASSERT_MSG(near(values[standby + lim::kThrustExciteStepsRampUpCycles + 5], level, 1.0e-6f), "first level");
    ASSERT_MSG(near(values[standby + half + 5], level * 1.0f, 1.0e-6f) == false && near(values[standby + half + 5], -level, 1.0e-6f), "second level is negative");
    ASSERT_MSG(near(values[standby + half - 1], level, 1.0e-6f) && near(values[standby + half], -level, 1.0e-6f), "the interior edge is a true step");
    ASSERT_MSG(near(values[standby + body - 1], -level, 1.0e-6f), "last level held to the nominal end");
    ASSERT_MSG(values.back() == 0.0f, "ends at %g", static_cast<double>(values.back()));
    ASSERT_MSG(excitation.result() == ThrustExcitationResult::Completed, "result");
}

TEST(standby_waits_for_stillness_and_gives_up_after_its_timeout)
{
    ThrustExcitation excitation;
    ThrustExcitationFacts facts = stillFacts();
    facts.altitude_target_m = kHoverHeightM + lim::kThrustExciteStartAltitudeErrorMaxM + 0.01f;   // never still / 静止しない
    excitation.request(multisineCommand(kAmplitudePercent, kMultisineLengthS), facts, kHoverThrustN);
    int cycles = 0;
    while (excitation.isActive() && cycles < 10000) {
        excitation.step(facts);
        ++cycles;
    }
    const int timeout_cycles = static_cast<int>(lim::kThrustExciteStartSettleTimeoutS * lim::kThrustExciteSampleRateHz);
    ASSERT_MSG(cycles == timeout_cycles, "gave up after %d cycles, expected %d", cycles, timeout_cycles);
    ASSERT_MSG(excitation.result() == ThrustExcitationResult::Rejected && excitation.reason() == ThrustExcitationReason::NotSettled, "result");
    ASSERT_MSG(!excitation.suppressesLearning(), "a refusal leaves no learning hold-off");
}

TEST(abort_ramps_out_over_a_quarter_second_without_a_step)
{
    ThrustExcitation excitation;
    ThrustExcitationFacts facts = stillFacts();
    excitation.request(multisineCommand(10.0f, kMultisineLengthS), facts, kHoverThrustN);
    std::vector<float> values;
    for (int cycle = 0; cycle < kSettleCycles + 2000; ++cycle) {
        values.push_back(excitation.step(facts));
    }
    ASSERT_MSG(excitation.state() == ThrustExcitationState::Running, "not running");
    facts.stick_moved = true;   // the cause of the abort / 打ち切りの原因
    int ramp_cycles = 0;
    float previous = values.back();
    float worst_jump = 0.0f;
    while (excitation.isActive() && ramp_cycles < 1000) {
        const float value = excitation.step(facts);
        worst_jump = std::fmax(worst_jump, std::fabs(value - previous));
        previous = value;
        ++ramp_cycles;
    }
    ASSERT_MSG(ramp_cycles >= kQuarterCycle && ramp_cycles <= kQuarterCycle + 2, "ramped for %d cycles, expected about %d", ramp_cycles, kQuarterCycle);
    ASSERT_MSG(excitation.result() == ThrustExcitationResult::Aborted && excitation.reason() == ThrustExcitationReason::StickMoved, "result");
    ASSERT_MSG(previous == 0.0f, "ended at %g", static_cast<double>(previous));
    // A step would be a jump of the order of the amplitude (0.04 N); the signal itself moves by far less per cycle.
    // 段差ならば振幅（0.04 N）の程度の跳びになる。信号そのものの 1 周期の動きはずっと小さい。
    ASSERT_MSG(worst_jump < 0.01f, "jump of %g N in one cycle", static_cast<double>(worst_jump));
}

TEST(cancel_ends_a_waiting_request_at_once_and_ramps_out_a_running_one)
{
    ThrustExcitation waiting;
    const ThrustExcitationFacts facts = stillFacts();
    waiting.request(multisineCommand(kAmplitudePercent, kMultisineLengthS), facts, kHoverThrustN);
    waiting.cancel();
    ASSERT_MSG(!waiting.isActive() && waiting.result() == ThrustExcitationResult::Aborted && waiting.reason() == ThrustExcitationReason::Cancelled,
               "a waiting request was not ended by the cancel");

    ThrustExcitation running;
    running.request(multisineCommand(kAmplitudePercent, kMultisineLengthS), facts, kHoverThrustN);
    for (int cycle = 0; cycle < kSettleCycles + 1000; ++cycle) running.step(facts);
    running.cancel();
    ASSERT_MSG(running.isActive() && running.state() == ThrustExcitationState::RampDown, "a running request did not start its ramp");
    runToEnd(running, facts, 1000);
    ASSERT_MSG(running.result() == ThrustExcitationResult::Aborted && running.reason() == ThrustExcitationReason::Cancelled, "result");
}

TEST(reset_stops_at_once_and_a_second_request_is_busy)
{
    ThrustExcitation excitation;
    const ThrustExcitationFacts facts = stillFacts();
    excitation.request(multisineCommand(kAmplitudePercent, kMultisineLengthS), facts, kHoverThrustN);
    for (int cycle = 0; cycle < kSettleCycles + 500; ++cycle) excitation.step(facts);
    ASSERT_MSG(excitation.request(multisineCommand(kAmplitudePercent, kMultisineLengthS), facts, kHoverThrustN) == ThrustExcitationReason::Busy,
               "a second request was not refused as busy");
    ASSERT_MSG(excitation.state() == ThrustExcitationState::RampUp || excitation.state() == ThrustExcitationState::Running, "the first run was disturbed");
    excitation.reset();
    ASSERT_MSG(!excitation.isActive() && !excitation.isStepping() && !excitation.suppressesLearning(), "reset did not stop");
    ASSERT_MSG(excitation.step(facts) == 0.0f, "reset left a signal");
}

TEST(learning_stays_suppressed_for_one_second_after_the_end)
{
    ThrustExcitation excitation;
    const ThrustExcitationFacts facts = stillFacts();
    excitation.request(multisineCommand(kAmplitudePercent, kMultisineLengthS), facts, kHoverThrustN);
    ASSERT_MSG(excitation.suppressesLearning(), "not suppressed while waiting");
    runToEnd(excitation, facts, 100000);
    const int holdoff_cycles = static_cast<int>(lim::kThrustExciteLearningHoldOffS * lim::kThrustExciteSampleRateHz);
    int cycles = 0;
    while (excitation.suppressesLearning() && cycles < 10 * holdoff_cycles) {
        ASSERT_MSG(excitation.step(facts) == 0.0f, "signal after the end");
        ++cycles;
    }
    ASSERT_MSG(cycles == holdoff_cycles, "hold-off lasted %d cycles, expected %d", cycles, holdoff_cycles);
}

TEST(a_run_that_loses_the_tof_or_the_battery_aborts_after_the_debounce_time)
{
    ThrustExcitation excitation;
    ThrustExcitationFacts facts = stillFacts();
    excitation.request(multisineCommand(kAmplitudePercent, kMultisineLengthS), facts, kHoverThrustN);
    for (int cycle = 0; cycle < kSettleCycles + 1000; ++cycle) excitation.step(facts);
    facts.tof_usable = false;
    const int lost_cycles = static_cast<int>(lim::kThrustExciteToFLostAbortS * lim::kThrustExciteSampleRateHz);
    for (int cycle = 0; cycle < lost_cycles - 1; ++cycle) excitation.step(facts);
    ASSERT_MSG(excitation.state() != ThrustExcitationState::RampDown, "aborted before the debounce time");
    excitation.step(facts);
    ASSERT_MSG(excitation.state() == ThrustExcitationState::RampDown && excitation.reason() == ThrustExcitationReason::ToFInvalid, "no ToF abort");

    ThrustExcitation battery;
    facts = stillFacts();
    battery.request(multisineCommand(kAmplitudePercent, kMultisineLengthS), facts, kHoverThrustN);
    for (int cycle = 0; cycle < kSettleCycles + 1000; ++cycle) battery.step(facts);
    facts.battery_voltage = lim::kThrustExciteBatteryAbortV - 0.01f;
    const int low_cycles = static_cast<int>(lim::kThrustExciteBatteryAbortHoldS * lim::kThrustExciteSampleRateHz);
    for (int cycle = 0; cycle < low_cycles; ++cycle) battery.step(facts);
    ASSERT_MSG(battery.state() == ThrustExcitationState::RampDown && battery.reason() == ThrustExcitationReason::BatteryLow, "no battery abort");
}

// =============================================================================
// Part 2: the controller
// 第 2 部: 制御器
// =============================================================================

/// One PidController in ALT_HOLD (or another mode) with the craft hovering still at the target. The state is held fixed (no plant), so
/// the controller output without an excitation is constant and the excitation shows as the difference.
/// ALT_HOLD（または別のモード）の PidController と、目標に静止してホバーする機体。状態は固定（プラントなし）なので、励振なしの
/// 制御器の出力は一定で、励振は差として見える。
struct Rig {
    PidController controller;
    StateEstimate state{};
    CommandSetpoint sticks{};
    ControlOutput output{};
    uint32_t sequence = 0;

    explicit Rig(FlightMode mode = FlightMode::ALT_HOLD)
    {
        params::init();
        controller.init();
        controller.reset();
        controller.onModeChange(mode);
        state.timestamp = kStartTimeUs;
        state.tof_valid = true;
        state.airborne = true;
        state.battery_voltage = kGoodBatteryV;
        state.attitude[0] = 1.0f;
        state.position[2] = -kHoverHeightM;
        sticks.timestamp = state.timestamp;
        controller.onTakeoff();
        run(20);
        controller.onTakeoffComplete();
        run(20);
    }

    void run(int cycles)
    {
        for (int n = 0; n < cycles; ++n) {
            state.timestamp += kControlPeriodUs;
            sticks.timestamp = state.timestamp;
            output = controller.compute(state, sticks, kControlPeriodS);
        }
    }

    /// Put the craft `distance_m` below the altitude target that the controller holds (set after the takeoff, which captured the target).
    /// 制御器が保持する高さの目標の `distance_m` 下に機体を置く（目標を取り込んだ離陸の後で設定する）。
    void moveBelowTarget(float distance_m) { state.position[2] = -(kHoverHeightM - distance_m); }

    void request(const ThrustExcitationCommand& command)
    {
        ThrustExcitationCommand numbered = command;
        numbered.sequence = ++sequence;
        controller.startThrustExcitation(numbered);
    }

    ControllerStatus diagnostics() const
    {
        ControllerStatus status{};
        controller.fillDiagnostics(status);
        return status;
    }

    ThrustExcitationState excitationState() const { return static_cast<ThrustExcitationState>(diagnostics().thrust_excitation_state); }
    ThrustExcitationReason excitationReason() const { return static_cast<ThrustExcitationReason>(diagnostics().thrust_excitation_reason); }
};

static uint32_t bitsOf(float value)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

TEST(controller_output_is_bit_identical_without_an_excitation_and_after_a_refused_request)
{
    Rig reference;
    Rig refused;
    refused.request(multisineCommand(15.0f, kMultisineLengthS));   // above the amplitude limit: refused / 振幅の上限超え: 拒否
    for (int cycle = 0; cycle < 3000; ++cycle) {
        reference.run(1);
        refused.run(1);
        ASSERT_MSG(bitsOf(reference.output.thrust) == bitsOf(refused.output.thrust), "thrust differs at cycle %d", cycle);
        for (int axis = 0; axis < 3; ++axis) {
            ASSERT_MSG(bitsOf(reference.output.torque[axis]) == bitsOf(refused.output.torque[axis]), "torque %d differs at cycle %d", axis, cycle);
        }
    }
    ASSERT_MSG(refused.excitationState() == ThrustExcitationState::None, "state after the refusal");
    ASSERT_MSG(static_cast<ThrustExcitationResult>(refused.diagnostics().thrust_excitation_result) == ThrustExcitationResult::Rejected &&
               refused.excitationReason() == ThrustExcitationReason::AmplitudeLimit, "the refusal is not shown in the status");
    ASSERT_MSG(refused.diagnostics().thrust_excitation_ack == 1u, "ack");
}

TEST(controller_adds_exactly_the_waveform_to_the_thrust)
{
    Rig reference;
    Rig excited;
    excited.request(multisineCommand(10.0f, kMultisineLengthS));
    const float hover = excited.controller.hover_thrust_;
    float worst = 0.0f;
    float peak = 0.0f;
    uint32_t observed = 0;   // cycles seen since Standby ended / 待機が終わってから見た周期数
    bool started = false;
    for (int cycle = 0; cycle < 12000; ++cycle) {
        reference.run(1);
        excited.run(1);
        const float added = excited.output.thrust - reference.output.thrust;
        if (excited.excitationState() == ThrustExcitationState::Standby) {
            ASSERT_MSG(added == 0.0f, "signal during Standby");
            continue;
        }
        if (!started) {   // the cycle in which Standby ended carries no signal; the waveform begins with the next one / 待機が終わった周期は信号なし。波形は次の周期から
            started = true;
            ASSERT_MSG(added == 0.0f, "signal in the cycle that ended Standby");
            continue;
        }
        const uint32_t waveform_cycle = observed++;
        const bool mid_run = waveform_cycle > 300 && waveform_cycle < 7700;   // away from the 0.5 s ramps (200 cycles each) / 0.5 s の立ち上げ・立ち下げ（各 200 周期）から離れる
        if (mid_run) {
            worst = std::fmax(worst, std::fabs(added - 0.1f * hover * thrustExciteMultisine(waveform_cycle)));
        }
        peak = std::fmax(peak, std::fabs(added));
    }
    ASSERT_MSG(started && worst < 1.0e-6f, "the added signal differs from the waveform by %g N", static_cast<double>(worst));
    ASSERT_MSG(peak <= 0.1f * hover * (1.0f + kPeakTolerance) && peak >= 0.9f * 0.1f * hover, "peak %g N", static_cast<double>(peak));
    ASSERT_MSG(excited.excitationState() == ThrustExcitationState::None &&
               static_cast<ThrustExcitationResult>(excited.diagnostics().thrust_excitation_result) == ThrustExcitationResult::Completed, "did not complete");
    // After the end (and its hold-off) the output is the same as the reference again. / 終わり（と停止の時間）の後は出力が基準と同じに戻る。
    ASSERT_MSG(excited.output.thrust == reference.output.thrust, "output did not return to the reference");
}

TEST(hover_thrust_learner_does_not_move_during_and_shortly_after_the_excitation)
{
    // The craft sits 0.03 m below the target: the velocity loop asks for more thrust and the learner folds it into hover_thrust_.
    // 機体は目標の 0.03 m 下: 速度ループが推力を増やし、学習がそれを hover_thrust_ に畳み込む。
    constexpr float kBelowM = 0.03f;
    Rig learning;
    learning.moveBelowTarget(kBelowM);
    const float before = learning.controller.hover_thrust_;
    learning.run(8000);
    ASSERT_MSG(learning.controller.hover_thrust_ > before + 1.0e-4f, "the learner is not moving in the reference run (%g -> %g)",
               static_cast<double>(before), static_cast<double>(learning.controller.hover_thrust_));

    Rig excited;
    excited.moveBelowTarget(kBelowM);
    excited.request(multisineCommand(kAmplitudePercent, 10.0f));
    const float start = excited.controller.hover_thrust_;
    // Standby (0.5 s) + 10 s + 1 s of hold-off, minus a few cycles so the learner is still frozen at the last check.
    // 待機（0.5 s）+ 10 s + 停止の 1 s から、最後の確認でまだ止まっているよう数周期引く。
    excited.run(kStandbyCycles + 4000 + 400 - 10);
    ASSERT_MSG(excited.controller.hover_thrust_ == start, "hover thrust moved %g -> %g during the excitation",
               static_cast<double>(start), static_cast<double>(excited.controller.hover_thrust_));
    excited.run(4000);   // the learner works again afterwards / その後は学習が再び動く
    ASSERT_MSG(excited.controller.hover_thrust_ > start, "the learner did not resume");
}

TEST(controller_aborts_on_mode_change_stick_height_window_tof_and_battery)
{
    // Mode change to STABILIZE. / STABILIZE へのモード変更。
    {
        Rig rig;
        rig.request(multisineCommand(kAmplitudePercent, kMultisineLengthS));
        rig.run(kSettleCycles + 1000);
        ASSERT_MSG(rig.excitationState() == ThrustExcitationState::Running, "not running before the mode change");
        rig.controller.onModeChange(FlightMode::STABILIZE);
        rig.run(2);
        ASSERT_MSG(rig.excitationState() == ThrustExcitationState::RampDown && rig.excitationReason() == ThrustExcitationReason::ModeNotAllowed, "mode abort");
    }
    // A stick moves by more than the guidance-cancel threshold. / スティックが誘導解除のしきい値を超えて動く。
    {
        Rig rig;
        rig.request(multisineCommand(kAmplitudePercent, kMultisineLengthS));
        rig.run(kSettleCycles + 1000);
        rig.sticks.roll = 0.5f;
        rig.run(2);
        ASSERT_MSG(rig.excitationReason() == ThrustExcitationReason::StickMoved, "stick abort");
    }
    // The height leaves the run window (the estimate is moved above it). / 高さが実行中の窓を出る。
    {
        Rig rig;
        rig.request(multisineCommand(kAmplitudePercent, kMultisineLengthS));
        rig.run(kSettleCycles + 1000);
        rig.state.position[2] = -(lim::kThrustExciteRunHeightMaxM + 0.05f);
        rig.run(2);
        ASSERT_MSG(rig.excitationReason() == ThrustExcitationReason::HeightWindow, "height window abort");
    }
    // The ToF becomes invalid. / ToF が無効になる。
    {
        Rig rig;
        rig.request(multisineCommand(kAmplitudePercent, kMultisineLengthS));
        rig.run(kSettleCycles + 1000);
        rig.state.tof_valid = false;
        rig.run(static_cast<int>(lim::kThrustExciteToFLostAbortS * lim::kThrustExciteSampleRateHz) + 2);
        ASSERT_MSG(rig.excitationReason() == ThrustExcitationReason::ToFInvalid, "ToF abort");
    }
    // The battery sags below the abort voltage. / 電池が打ち切りの電圧を下回る。
    {
        Rig rig;
        rig.request(multisineCommand(kAmplitudePercent, kMultisineLengthS));
        rig.run(kSettleCycles + 1000);
        rig.state.battery_voltage = lim::kThrustExciteBatteryAbortV - 0.1f;
        rig.run(static_cast<int>(lim::kThrustExciteBatteryAbortHoldS * lim::kThrustExciteSampleRateHz) + 2);
        ASSERT_MSG(rig.excitationReason() == ThrustExcitationReason::BatteryLow, "battery abort");
    }
}

TEST(controller_refuses_outside_altitude_modes_and_after_a_landing_starts)
{
    {
        Rig rig(FlightMode::STABILIZE);
        rig.request(multisineCommand(kAmplitudePercent, kMultisineLengthS));
        rig.run(2);
        ASSERT_MSG(rig.excitationState() == ThrustExcitationState::None, "STABILIZE started an excitation");
        ASSERT_MSG(static_cast<ThrustExcitationResult>(rig.diagnostics().thrust_excitation_result) == ThrustExcitationResult::Rejected, "no rejection");
    }
    {
        Rig rig;
        rig.request(multisineCommand(kAmplitudePercent, kMultisineLengthS));
        rig.run(kSettleCycles + 1000);
        rig.controller.onLanding();   // the vertical phase leaves Airborne / 鉛直フェーズが Airborne を出る
        rig.run(2);
        ASSERT_MSG(rig.excitationState() == ThrustExcitationState::RampDown && rig.excitationReason() == ThrustExcitationReason::FlightEnded,
                   "landing abort (state %u, reason %u)", static_cast<unsigned>(rig.excitationState()), static_cast<unsigned>(rig.excitationReason()));
    }
}

TEST(rate_axis_excitation_and_thrust_excitation_exclude_each_other)
{
    Rig rig;
    SysidCommand sysid{};
    sysid.axis = 0;
    sysid.waveform = 0;
    sysid.amplitude = 0.5f;
    sysid.duration = 2.0f;
    rig.controller.startExcitation(sysid);
    rig.run(2);
    ASSERT_MSG(rig.controller.excite_active_, "the rate excitation did not start");
    rig.request(multisineCommand(kAmplitudePercent, kMultisineLengthS));
    rig.run(2);
    ASSERT_MSG(rig.excitationState() == ThrustExcitationState::None, "a thrust excitation ran together with a rate excitation");

    Rig other;
    other.request(multisineCommand(kAmplitudePercent, kMultisineLengthS));
    other.run(2);
    other.controller.startExcitation(sysid);
    other.run(2);
    ASSERT_MSG(!other.controller.excite_active_, "a rate excitation started during a thrust excitation");
}

TEST(reset_stops_the_excitation_at_once_and_clears_a_latched_request)
{
    Rig rig;
    rig.request(multisineCommand(kAmplitudePercent, kMultisineLengthS));
    rig.run(kSettleCycles + 1000);
    rig.controller.reset();
    ASSERT_MSG(rig.excitationState() == ThrustExcitationState::None, "reset did not stop it");
    ASSERT_MSG(!rig.controller.thrust_excitation_.suppressesLearning(), "reset left the learning frozen");
    Rig latched;
    latched.request(multisineCommand(kAmplitudePercent, kMultisineLengthS));
    latched.controller.reset();
    ASSERT_MSG(!latched.controller.thrust_excitation_pending_, "reset left a latched request");
}

TEST(references_are_reported_in_the_diagnostics)
{
    Rig rig;
    rig.run(10);
    const ControllerStatus status = rig.diagnostics();
    ASSERT_MSG(near(status.reference_altitude_m, kHoverHeightM, 1.0e-6f), "altitude target %g", static_cast<double>(status.reference_altitude_m));
    Rig acro(FlightMode::ACRO);
    acro.run(10);
    ASSERT_MSG(acro.diagnostics().reference_altitude_m == 0.0f, "ACRO has no altitude target");
}

// =============================================================================
// Runner
// 実行部
// =============================================================================

static int silentLogOutput(const char*, va_list)
{
    return 0;
}

int main()
{
    esp_log_set_vprintf(silentLogOutput);
    printf("=== Thrust excitation host tests ===\n");
    int failed = 0;
    for (const TestCase& test_case : registry()) {
        printf("  [TEST] %-82s ", test_case.name);
        try {
            test_case.body();
            printf("PASS\n");
        } catch (const AssertionFailure&) {
            printf("  FAIL  %s\n", test_case.name);
            ++failed;
        }
    }
    printf("\n=== %zu run, %d failed ===\n", registry().size(), failed);
    return failed == 0 ? 0 : 1;
}
