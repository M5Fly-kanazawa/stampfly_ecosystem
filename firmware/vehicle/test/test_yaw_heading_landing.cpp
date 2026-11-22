/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_yaw_heading_landing.cpp
 * @brief Host tests: the heading hold keeps working when guidance has been set and LANDING begins
 *        誘導が立ったまま LANDING に入っても、ヘディングホールドが働くことの host 試験
 *
 * 2026-11-22 flight: after an API `takeoff` the controller's guidance flag stayed set into LANDING.
 * The guidance block (which drives the yaw rate) runs only in the Airborne phase, and the heading
 * hold was skipped whenever the flag was set, so nobody owned the yaw: a yaw-stick offset of
 * -0.0265 became a yaw-rate setpoint of -0.13 rad/s and the nose turned about 35 degrees in 3.3 s.
 * The yaw-rate setpoint is read from ControlOutput::rate_ref[2] (what the Data Stream logs).
 * 2026-11-22 の飛行: API の `takeoff` のあと、制御器の誘導の印が LANDING まで立ったままだった。
 * ヨーの角速度を駆動する誘導ブロックは Airborne でしか動かず、ヘディングホールドは印が立っている間
 * 止まっていたため、ヨーを誰も持たず、ヨースティックのずれ −0.0265 がそのまま −0.13 rad/s の
 * 角速度の目標になり、3.3 s で機首が約 35 度回った。角速度の目標は ControlOutput::rate_ref[2]
 * （Data Stream が記録する値）から読む。
 *
 * pid_controller.cpp is compiled unmodified; params and the clock come from simulator/sils/compat.
 * pid_controller.cpp は無改変でコンパイルし、パラメータと時計は simulator/sils/compat から取る。
 *
 * @design detailed_design.md §4 — heading hold through LANDING (INV-2: the pilot owns the attitude)   [OK]
 * @design coding_and_education.md §2 — Bilingual comments                                                 [OK]
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <vector>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

#include "params.hpp"
#include "pid_controller.hpp"

using namespace sf;

// The SILS shims call vTaskDelay() only to wait for a lock; the tests are single-threaded.
// SILS のシムは vTaskDelay() をロック待ちのときだけ呼ぶ。試験は 1 スレッド。
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

static constexpr float kControlPeriodS = 0.0025f;               ///< 400 Hz control cycle [s] / 400 Hz の制御周期
static constexpr uint32_t kControlPeriodUs = 2500;              ///< the same [us] / 同じ値 [us]
static constexpr uint32_t kStartTimeUs = 1000000;               ///< first virtual time (0 means "never") / 仮想時間の開始（0 は「一度も無い」）
static constexpr float kHoverHeightM = 0.5f;                    ///< height of the craft [m] / 機体の高さ
static constexpr float kYawStickOffset = -0.0265f;              ///< the offset seen in the flight log / 飛行記録のずれ
static constexpr float kYawStickIntended = -0.5f;               ///< a deliberate yaw command / 意図したヨー指令
static constexpr float kMaxYawRateRadS = 5.0f;                  ///< PidController::max_yaw_rate_ default / 既定値
static constexpr float kHeadingHoldGain = 3.0f;                 ///< attitude.yawhold.kp default [1/s] / 既定値
static constexpr float kStaleLinkGapUs = 800000.0f;             ///< longer than kLandingLinkStaleUs (500 ms) / 500 ms より長い
static constexpr float kRateTolerance = 0.01f;                  ///< tolerance on a yaw-rate setpoint [rad/s] / 角速度の目標の許容
static constexpr float kOldBugRate = kYawStickOffset * kMaxYawRateRadS;   ///< -0.13 rad/s, what the stick offset used to command / ずれが指令していた値
static constexpr int kSettleCycles = 20;                        ///< cycles to let the hold capture / 捕捉を待つ周期数

// =============================================================================
// Fixture: one PidController with a hovering state and live sticks
// 試験台: ホバー状態の PidController と、生きているスティック
// =============================================================================

struct Rig {
    PidController controller;
    StateEstimate state{};
    CommandSetpoint sticks{};
    ControlOutput output{};
    float yaw_rad = 0.3f;   ///< estimated heading [rad] / 推定方位

    explicit Rig(FlightMode mode)
    {
        params::init();
        controller.init();
        controller.reset();
        controller.onModeChange(mode);
        state.timestamp = kStartTimeUs;
        state.tof_valid = true;
        state.airborne = true;
        state.battery_voltage = 4.0f;
        state.position[2] = -kHoverHeightM;   // NED z
        sticks.timestamp = state.timestamp;
        sticks.yaw = kYawStickOffset;
        sticks.throttle = 0.6f;
        applyYaw();
    }

    void applyYaw()
    {
        state.attitude[0] = std::cos(yaw_rad / 2.0f);
        state.attitude[1] = 0.0f;
        state.attitude[2] = 0.0f;
        state.attitude[3] = std::sin(yaw_rad / 2.0f);
    }

    /// Run cycles with a LIVE link (the stick stamp follows the clock). / リンク生存のまま周期を進める。
    void run(int cycles)
    {
        for (int n = 0; n < cycles; ++n) {
            state.timestamp += kControlPeriodUs;
            sticks.timestamp = state.timestamp;
            applyYaw();
            output = controller.compute(state, sticks, kControlPeriodS);
        }
    }

    /// Run cycles with a DEAD link (the stick stamp stays where it was). / リンク途絶のまま進める。
    void runWithStaleLink(int cycles)
    {
        for (int n = 0; n < cycles; ++n) {
            state.timestamp += kControlPeriodUs;
            applyYaw();
            output = controller.compute(state, sticks, kControlPeriodS);
        }
    }

    float yawRateSetpoint() const { return output.rate_ref[2]; }

    /// Hand a velocity guidance (Tello `rc 0 0 0 0`) to the controller. / 速度誘導（rc 0 0 0 0）を渡す。
    void engageVelocityGuidance()
    {
        GuidanceTarget target{};
        target.mode = 2;
        target.timestamp = state.timestamp;
        controller.setGuidanceTarget(target, sticks);
    }

    /// Hand a position guidance that wants a different heading. / 別の方位を求める位置誘導を渡す。
    void engagePositionGuidance(float target_yaw_rad)
    {
        GuidanceTarget target{};
        target.mode = 1;
        target.position[2] = -kHoverHeightM;
        target.yaw = target_yaw_rad;
        target.speed = 0.5f;
        target.timestamp = state.timestamp;
        controller.setGuidanceTarget(target, sticks);
    }

    /// The API sequence: takeoff, guidance engaged, takeoff complete, flying. / API の手順。
    void takeoffAndFly()
    {
        controller.onTakeoff();
        run(kSettleCycles);
    }

    void finishTakeoff()
    {
        controller.onTakeoffComplete();
        run(kSettleCycles);
    }
};

static bool near(float value, float expected, float tolerance)
{
    return std::fabs(value - expected) <= tolerance;
}

// =============================================================================
// Tests
// 試験
// =============================================================================

TEST(landing_after_velocity_guidance_holds_heading_not_the_stick_offset)
{
    Rig rig(FlightMode::POS_HOLD);
    rig.takeoffAndFly();
    rig.engageVelocityGuidance();
    rig.finishTakeoff();
    ASSERT_MSG(rig.controller.isGuidanceActive(), "guidance should be engaged before the landing");

    rig.controller.onLanding();
    rig.run(kSettleCycles);
    ASSERT_MSG(near(rig.yawRateSetpoint(), 0.0f, kRateTolerance),
               "yaw-rate setpoint %g rad/s, expected 0 (the old behaviour gave %g)",
               static_cast<double>(rig.yawRateSetpoint()), static_cast<double>(kOldBugRate));
}

TEST(landing_holds_the_heading_captured_when_landing_starts)
{
    Rig rig(FlightMode::POS_HOLD);
    rig.takeoffAndFly();
    rig.engageVelocityGuidance();
    rig.finishTakeoff();
    rig.controller.onLanding();
    rig.run(kSettleCycles);

    const float disturbance_rad = 0.1f;   // the nose is pushed 0.1 rad / 機首が 0.1 rad 押される
    rig.yaw_rad += disturbance_rad;
    rig.run(1);
    ASSERT_MSG(near(rig.yawRateSetpoint(), -kHeadingHoldGain * disturbance_rad, kRateTolerance),
               "yaw-rate setpoint %g rad/s, expected %g (the hold pulls back to the captured heading)",
               static_cast<double>(rig.yawRateSetpoint()), static_cast<double>(-kHeadingHoldGain * disturbance_rad));
}

TEST(landing_does_not_chase_the_old_position_guidance_heading)
{
    Rig rig(FlightMode::POS_HOLD);
    rig.takeoffAndFly();
    rig.finishTakeoff();
    const float far_target_rad = 1.2f;   // the guidance wanted a turn that is not finished / 終わっていない回頭を求めていた
    rig.engagePositionGuidance(far_target_rad);
    rig.run(kSettleCycles);
    ASSERT_MSG(rig.yawRateSetpoint() > kRateTolerance,
               "while guidance runs, the yaw-rate setpoint %g should turn toward the target", static_cast<double>(rig.yawRateSetpoint()));

    rig.controller.onLanding();
    rig.run(kSettleCycles);
    ASSERT_MSG(near(rig.yawRateSetpoint(), 0.0f, kRateTolerance),
               "yaw-rate setpoint %g rad/s in LANDING, expected 0 (keep the present heading)",
               static_cast<double>(rig.yawRateSetpoint()));
}

TEST(a_deliberate_yaw_stick_in_landing_is_followed_and_the_hold_recaptures)
{
    Rig rig(FlightMode::POS_HOLD);
    rig.takeoffAndFly();
    rig.engageVelocityGuidance();
    rig.finishTakeoff();
    rig.controller.onLanding();
    rig.run(kSettleCycles);

    rig.sticks.yaw = kYawStickIntended;
    rig.run(1);
    ASSERT_MSG(near(rig.yawRateSetpoint(), kYawStickIntended * kMaxYawRateRadS, kRateTolerance),
               "yaw-rate setpoint %g rad/s, expected the pilot's %g",
               static_cast<double>(rig.yawRateSetpoint()), static_cast<double>(kYawStickIntended * kMaxYawRateRadS));

    rig.yaw_rad -= 0.5f;           // the craft turned while the pilot held the stick / スティックの間に機体が回った
    rig.sticks.yaw = kYawStickOffset;
    rig.run(kSettleCycles);
    ASSERT_MSG(near(rig.yawRateSetpoint(), 0.0f, kRateTolerance),
               "after the release the hold must capture the new heading, setpoint %g", static_cast<double>(rig.yawRateSetpoint()));
}

TEST(landing_on_a_stale_link_commands_zero_yaw_rate)
{
    Rig rig(FlightMode::POS_HOLD);
    rig.takeoffAndFly();
    rig.engageVelocityGuidance();
    rig.finishTakeoff();
    rig.controller.onLanding();
    rig.run(kSettleCycles);

    rig.state.timestamp += static_cast<uint32_t>(kStaleLinkGapUs);   // the stick stream stops / スティックの送信が止まる
    rig.runWithStaleLink(1);
    rig.sticks.yaw = kYawStickIntended;     // stale stick value must not steer / 古いスティック値で操縦しない
    rig.runWithStaleLink(kSettleCycles);
    ASSERT_MSG(rig.yawRateSetpoint() == 0.0f,
               "yaw-rate setpoint %g rad/s on a stale link, expected exactly 0", static_cast<double>(rig.yawRateSetpoint()));
}

TEST(manual_landing_with_a_low_throttle_stick_holds_heading)
{
    Rig rig(FlightMode::STABILIZE);
    rig.sticks.throttle = 0.1f;   // below the heading hold's throttle floor / ヘディングホールドのスロットル床値より低い
    rig.run(kSettleCycles);
    rig.controller.onLanding();
    rig.run(kSettleCycles);
    ASSERT_MSG(near(rig.yawRateSetpoint(), 0.0f, kRateTolerance),
               "yaw-rate setpoint %g rad/s, expected 0 (the old behaviour gave %g)",
               static_cast<double>(rig.yawRateSetpoint()), static_cast<double>(kOldBugRate));
}

TEST(takeoff_climb_with_guidance_holds_heading_too)
{
    Rig rig(FlightMode::POS_HOLD);
    rig.controller.onTakeoff();
    rig.run(kSettleCycles);
    rig.engageVelocityGuidance();
    rig.run(kSettleCycles);
    ASSERT_MSG(near(rig.yawRateSetpoint(), 0.0f, kRateTolerance),
               "yaw-rate setpoint %g rad/s in TakeoffClimb with guidance, expected 0",
               static_cast<double>(rig.yawRateSetpoint()));
}

TEST(velocity_guidance_still_owns_the_yaw_rate_while_airborne)
{
    Rig rig(FlightMode::POS_HOLD);
    rig.takeoffAndFly();
    rig.finishTakeoff();
    GuidanceTarget target{};
    target.mode = 2;
    target.vyaw = 0.4f;   // a deliberate `cw` turn / 意図した回頭
    target.timestamp = rig.state.timestamp;
    rig.controller.setGuidanceTarget(target, rig.sticks);
    rig.run(1);
    ASSERT_MSG(near(rig.yawRateSetpoint(), target.vyaw, kRateTolerance),
               "yaw-rate setpoint %g rad/s, expected the guidance's %g",
               static_cast<double>(rig.yawRateSetpoint()), static_cast<double>(target.vyaw));
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
    printf("=== Heading hold through LANDING host tests ===\n");
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
