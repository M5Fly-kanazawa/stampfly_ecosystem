/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_takeoff_hold_target.cpp
 * @brief Host tests: the POS_HOLD hold point of an auto-takeoff stays where it was captured
 *        自動離陸の POS_HOLD の保持点は、取り込んだ点のままであることの host 試験
 *
 * 2026-11-22 investigation: after an API `takeoff` the guidance target was seeded from the estimate
 * of the moment the takeoff completed, so the drift of the climb (0.09 m behind in the flight log)
 * became the hold target and was never corrected. The controller now exposes its hold point
 * (IController::positionHoldTarget, carried to ApiTask on controller_status) and ApiTask seeds the
 * guidance target with it. These tests check the controller side: the hold point is the captured
 * point through the climb and the takeoff completion, a guidance target equal to it leaves it
 * unchanged while the position loop pulls the craft back, and a guidance target at the drifted
 * estimate (the old seeding) does move it.
 * 2026-11-22 の調査: API の `takeoff` のあと、誘導目標は離陸の完了の瞬間の推定の位置から作られ、
 * 上昇中のずれ（飛行記録で後ろへ 0.09 m）がそのまま保持の目標になって戻されなかった。制御器は
 * 保持点を公開し（IController::positionHoldTarget、controller_status で ApiTask へ運ぶ）、ApiTask は
 * それで誘導目標を初期化する。この試験は制御器側を確かめる: 保持点は上昇と離陸の完了を通して取り込んだ
 * 点であること、それと同じ誘導目標は保持点を変えず位置ループが機体を引き戻すこと、ずれた推定の位置の
 * 誘導目標（従来の初期化）は保持点を動かすこと。
 *
 * pid_controller.cpp is compiled unmodified; params and the clock come from simulator/sils/compat.
 * pid_controller.cpp は無改変でコンパイルし、パラメータと時計は simulator/sils/compat から取る。
 *
 * @design detailed_design.md §3.1 note 4 — API takeoff seeds the hold point, not the estimate   [OK]
 * @design coding_and_education.md §2 — Bilingual comments                                       [OK]
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
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

static constexpr float kControlPeriodS = 0.0025f;        ///< 400 Hz control cycle [s] / 400 Hz の制御周期
static constexpr uint32_t kControlPeriodUs = 2500;       ///< the same [us] / 同じ値 [us]
static constexpr uint32_t kStartTimeUs = 1000000;        ///< first virtual time (0 means "never") / 仮想時間の開始
static constexpr float kClimbHeightM = 0.5f;             ///< height of the craft after the climb [m] / 上昇後の高さ
static constexpr float kPinnedNorthM = 0.0f;             ///< the estimator pins the position at 0 until its release / 解除まで推定器は位置を 0 に固定する
static constexpr float kClimbDriftNorthM = -0.09f;       ///< the drift of the 2026-11-22 flight (behind) [m] / 飛行のずれ（後ろ）
static constexpr float kPointTolerance = 1.0e-6f;        ///< tolerance on a hold point [m] / 保持点の許容
static constexpr int kCaptureCycles = 20;                ///< cycles to let the deferred capture run / 取り込みが走るまでの周期
static constexpr int kPullBackCycles = 200;              ///< 0.5 s of position control / 0.5 s の位置制御
static constexpr float kPullBackMinTiltRad = 0.001f;     ///< the pull-back must tilt at least this much [rad] / 引き戻しの最小の傾き

// =============================================================================
// Fixture: one PidController in POS_HOLD with a hovering state
// 試験台: POS_HOLD の PidController とホバー状態
// =============================================================================

struct Rig {
    PidController controller;
    StateEstimate state{};
    CommandSetpoint sticks{};
    ControlOutput output{};

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
        state.attitude[0] = 1.0f;
        state.position[0] = kPinnedNorthM;
        state.position[2] = -kClimbHeightM;
        sticks.timestamp = state.timestamp;
        sticks.throttle = 0.6f;
    }

    void run(int cycles)
    {
        for (int n = 0; n < cycles; ++n) {
            state.timestamp += kControlPeriodUs;
            sticks.timestamp = state.timestamp;
            output = controller.compute(state, sticks, kControlPeriodS);
        }
    }

    /// The takeoff chain of the flight: onTakeoff with the position pinned, the drift of the climb, the completion.
    /// 飛行の離陸の手順: 位置の固定中の onTakeoff、上昇中のずれ、離陸の完了。
    void takeoffWithClimbDrift()
    {
        controller.onTakeoff();
        run(kCaptureCycles);
        state.position[0] = kClimbDriftNorthM;
        run(kCaptureCycles);
        controller.onTakeoffComplete();
        run(kCaptureCycles);
    }

    /// Hand the controller a position guidance at north `north_m`, the altitude it holds and the heading.
    /// 北 `north_m`、保持している高度、いまの向きの位置誘導を渡す。
    void engageGuidanceAt(float north_m)
    {
        GuidanceTarget target{};
        target.mode = 1;
        target.position[0] = north_m;
        target.position[2] = -kClimbHeightM;
        target.speed = 0.5f;
        target.timestamp = state.timestamp;
        controller.setGuidanceTarget(target, sticks);
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

TEST(no_hold_point_before_the_capture_and_outside_pos_hold)
{
    Rig pos(FlightMode::POS_HOLD);
    pos.controller.onTakeoff();   // the capture is deferred to the next cycle / 取り込みは次の周期へ遅延
    float north = 1.0f, east = 1.0f;
    ASSERT_MSG(!pos.controller.positionHoldTarget(north, east), "a hold point was reported before the capture");

    Rig alt(FlightMode::ALT_HOLD);
    alt.controller.onTakeoff();
    alt.run(kCaptureCycles);
    ASSERT_MSG(!alt.controller.positionHoldTarget(north, east), "ALT_HOLD has no position hold point");
}

TEST(hold_point_is_the_captured_point_through_the_climb_and_the_completion)
{
    Rig rig(FlightMode::POS_HOLD);
    rig.takeoffWithClimbDrift();
    float north = 1.0f, east = 1.0f;
    ASSERT_MSG(rig.controller.positionHoldTarget(north, east), "no hold point after the takeoff");
    ASSERT_MSG(near(north, kPinnedNorthM, kPointTolerance) && near(east, 0.0f, kPointTolerance),
               "hold point N %g E %g, expected the captured point N %g E 0",
               static_cast<double>(north), static_cast<double>(east), static_cast<double>(kPinnedNorthM));
}

TEST(guidance_seeded_with_the_hold_point_keeps_it_and_the_craft_is_pulled_back)
{
    Rig rig(FlightMode::POS_HOLD);
    rig.takeoffWithClimbDrift();
    float hold_north = 0.0f, hold_east = 0.0f;
    ASSERT_MSG(rig.controller.positionHoldTarget(hold_north, hold_east), "no hold point");

    rig.engageGuidanceAt(hold_north);   // what ApiTask seeds at the end of `takeoff` / API の離陸の終わりの初期化
    rig.run(kPullBackCycles);

    float north = 1.0f, east = 1.0f;
    ASSERT_MSG(rig.controller.positionHoldTarget(north, east), "no hold point after the guidance");
    ASSERT_MSG(near(north, hold_north, kPointTolerance), "hold point moved to N %g, expected %g",
               static_cast<double>(north), static_cast<double>(hold_north));
    // The craft is 0.09 m behind the point (north negative): the position loop tilts it forward (pitch angle reference
    // negative = nose down in this convention) — the magnitude is what the hold needs.
    // 機体は保持点の 0.09 m 後ろ（北が負）: 位置ループは前へ傾ける。必要なのは大きさ。
    ASSERT_MSG(std::fabs(rig.output.angle_ref[1]) > kPullBackMinTiltRad, "no tilt toward the hold point (pitch ref %g rad)",
               static_cast<double>(rig.output.angle_ref[1]));
}

TEST(guidance_seeded_with_the_drifted_estimate_moves_the_hold_point_the_old_seeding)
{
    Rig rig(FlightMode::POS_HOLD);
    rig.takeoffWithClimbDrift();
    rig.engageGuidanceAt(kClimbDriftNorthM);   // the old seeding: the estimate of the completion / 従来: 完了の時点の推定
    rig.run(kPullBackCycles);
    float north = 1.0f, east = 1.0f;
    ASSERT_MSG(rig.controller.positionHoldTarget(north, east), "no hold point after the guidance");
    ASSERT_MSG(near(north, kClimbDriftNorthM, kPointTolerance),
               "hold point N %g, expected the drifted estimate %g (the controller follows its guidance target)",
               static_cast<double>(north), static_cast<double>(kClimbDriftNorthM));
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
    printf("=== Takeoff hold target host tests ===\n");
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
