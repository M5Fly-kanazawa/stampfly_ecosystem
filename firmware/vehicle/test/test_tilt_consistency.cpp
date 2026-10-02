/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_tilt_consistency.cpp
 * @brief Deterministic unit tests for TiltConsistencyMonitor
 *        TiltConsistencyMonitor の決定論ユニットテスト
 *
 * Runs on host PC against the UNMODIFIED tilt_consistency.cpp — no ESP-IDF, no FreeRTOS.
 * Own TEST/ASSERT framework and main(), same pattern as test_flip_sequencer.cpp.
 * ホストPCで無改変の tilt_consistency.cpp に対して実行する — ESP-IDF・FreeRTOS 不要。
 * test_flip_sequencer.cpp と同じ、自前の TEST/ASSERT フレームワークと main()。
 *
 * @design detailed_design.md §3 注10 — attitude/gravity mismatch on the ground [OK]
 */

#include <cmath>
#include <cstdio>

#include "sf_math.hpp"
#include "tilt_consistency.hpp"

using sf::TiltConsistencyConfig;
using sf::TiltConsistencyMonitor;

// =============================================================================
// Minimal test framework (mirrors test_flip_sequencer.cpp)
// 最小テストフレームワーク（test_flip_sequencer.cpp と同じ）
// =============================================================================

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

#define ASSERT_NEAR(a, b, tol) \
    if (fabsf((a) - (b)) > (tol)) { \
        printf("FAIL: %s:%d: %.6f != %.6f (tol=%.6f)\n", \
               __FILE__, __LINE__, (float)(a), (float)(b), (float)(tol)); \
        throw 1; \
    }

// =============================================================================
// Helpers
// ヘルパー
// =============================================================================

static constexpr float kDegToRad = 3.14159265358979f / 180.0f;
static constexpr float kSimDt    = 1.0f / 400.0f;   // IMU cycle / IMU 周期
static constexpr float kGravity  = 9.80665f;

/// Quaternion [w,x,y,z] for a roll of `deg` about body X (body→NED).
/// body X 軸まわりの roll `deg` のクォータニオン [w,x,y,z]（body→NED）。
static sf::math::Quat rollQuat(float deg)
{
    const float half = 0.5f * deg * kDegToRad;
    return {std::cos(half), std::sin(half), 0.0f, 0.0f};
}

/// What an ideal accelerometer reads at rest for attitude q: R^T * [0,0,-g] (specific force).
/// 姿勢 q で静止した理想加速度計の読み: R^T * [0,0,-g]（比力）。
static void restAccel(const sf::math::Quat& q, float accel[3])
{
    const sf::math::Vec3 g_body = q.inv_rotate({0.0f, 0.0f, -kGravity});
    accel[0] = g_body.x;
    accel[1] = g_body.y;
    accel[2] = g_body.z;
}

/// Run the monitor for `seconds` with the craft truly at `true_deg` roll while the
/// estimate says `estimated_deg`. Returns the mismatch flag at the end.
/// 機体の真の roll が `true_deg`、推定が `estimated_deg` の状態で `seconds` 秒回し、
/// 最後の mismatch を返す。
static bool run(TiltConsistencyMonitor& monitor, float seconds,
                float true_deg, float estimated_deg, bool on_ground = true)
{
    float accel[3];
    restAccel(rollQuat(true_deg), accel);
    const sf::math::Quat estimate = rollQuat(estimated_deg);
    const float quaternion[4] = {estimate.w, estimate.x, estimate.y, estimate.z};
    const int steps = static_cast<int>(seconds / kSimDt + 0.5f);
    for (int i = 0; i < steps; ++i) {
        monitor.update(kSimDt, accel, quaternion, on_ground);
    }
    return monitor.mismatch();
}

static TiltConsistencyMonitor makeMonitor()
{
    TiltConsistencyMonitor monitor;
    monitor.init(TiltConsistencyConfig{});   // 10 deg / 0.5 s / 1.0 s
    return monitor;
}

// =============================================================================
// Tests
// テスト
// =============================================================================

TEST(sign_convention_matches_sf_math_inv_rotate)
{
    // Level estimate and level accel: angle must be ~0 (would be 180 with a wrong sign).
    // 水平推定・水平加速度: 角度は ~0（符号が逆なら 180）。
    TiltConsistencyMonitor monitor = makeMonitor();
    run(monitor, 2.0f, 15.0f, 15.0f);
    ASSERT_NEAR(monitor.angleDeg(), 0.0f, 0.1f);
}

TEST(level_and_consistent_no_mismatch)
{
    TiltConsistencyMonitor monitor = makeMonitor();
    ASSERT_TRUE(!run(monitor, 3.0f, 0.0f, 0.0f));
    ASSERT_NEAR(monitor.angleDeg(), 0.0f, 0.1f);
}

TEST(disagreement_below_persist_no_mismatch)
{
    // Settle first (1 s filter), then the estimate is 20 deg off for 0.3 s < 0.5 s.
    // 先に整定（1 s）、その後 推定が 20 deg 外れるのを 0.3 s（< 0.5 s）。
    TiltConsistencyMonitor monitor = makeMonitor();
    ASSERT_TRUE(!run(monitor, 1.5f, 0.0f, 0.0f));
    ASSERT_TRUE(!run(monitor, 0.3f, 0.0f, 20.0f));
}

TEST(disagreement_for_persist_raises_mismatch)
{
    TiltConsistencyMonitor monitor = makeMonitor();
    run(monitor, 1.5f, 0.0f, 0.0f);
    ASSERT_TRUE(!run(monitor, 0.4f, 0.0f, 20.0f));   // 0.4 s: not yet
    ASSERT_TRUE(run(monitor, 0.2f, 0.0f, 20.0f));    // 0.6 s total: raised
    ASSERT_NEAR(monitor.angleDeg(), 20.0f, 0.5f);
}

TEST(mismatch_clears_when_back_below_threshold)
{
    TiltConsistencyMonitor monitor = makeMonitor();
    run(monitor, 1.5f, 0.0f, 0.0f);
    ASSERT_TRUE(run(monitor, 1.0f, 0.0f, 20.0f));
    ASSERT_TRUE(!run(monitor, 0.05f, 0.0f, 2.0f));
}

TEST(not_on_ground_is_false_and_resets_timer)
{
    TiltConsistencyMonitor monitor = makeMonitor();
    run(monitor, 1.5f, 0.0f, 0.0f);
    ASSERT_TRUE(run(monitor, 1.0f, 0.0f, 20.0f));
    ASSERT_TRUE(!run(monitor, 0.1f, 0.0f, 20.0f, /*on_ground=*/false));
    // Back on the ground: the filter re-seeds and the timer restarts, so a fresh
    // disagreement needs the full settle + persist time again.
    // 接地復帰: フィルタは再シードされタイマも再開するので、新たな不一致は整定＋持続時間を
    // 再び要する。
    ASSERT_TRUE(!run(monitor, 0.5f, 0.0f, 20.0f));
    ASSERT_TRUE(run(monitor, 1.2f, 0.0f, 20.0f));
}

TEST(glitch_sample_with_absurd_norm_is_ignored)
{
    TiltConsistencyMonitor monitor = makeMonitor();
    run(monitor, 1.5f, 0.0f, 0.0f);
    const float glitch[3] = {0.0f, 0.0f, -80.0f};   // ~8 g
    const float level[4]  = {1.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 10; ++i) {
        monitor.update(kSimDt, glitch, level, true);
    }
    ASSERT_TRUE(!monitor.mismatch());
    ASSERT_NEAR(monitor.angleDeg(), 0.0f, 0.1f);
    // A glitch during a real mismatch must not clear it either (state held).
    // 実際の不一致中のグリッチもそれを解除してはならない（状態保持）。
    ASSERT_TRUE(run(monitor, 1.0f, 0.0f, 20.0f));
    monitor.update(kSimDt, glitch, level, true);
    ASSERT_TRUE(monitor.mismatch());
}

TEST(reset_reseeds_filter)
{
    TiltConsistencyMonitor monitor = makeMonitor();
    run(monitor, 1.5f, 0.0f, 0.0f);
    ASSERT_TRUE(run(monitor, 1.0f, 0.0f, 20.0f));
    monitor.reset();
    ASSERT_TRUE(!monitor.mismatch());
    // After the estimator reset the estimate is consistent again.
    // 推定器 reset 後は推定が再び整合。
    ASSERT_TRUE(!run(monitor, 2.0f, 20.0f, 20.0f));
}

TEST(verified_is_a_positive_verdict)
{
    TiltConsistencyMonitor monitor = makeMonitor();
    ASSERT_TRUE(!monitor.verified());                 // fresh: unjudged
    run(monitor, 0.5f, 0.0f, 0.0f);
    ASSERT_TRUE(!monitor.verified());                 // still inside the settle window
    run(monitor, 1.0f, 0.0f, 0.0f);
    ASSERT_TRUE(monitor.verified());                  // judged and consistent
    ASSERT_TRUE(!run(monitor, 0.1f, 0.0f, 20.0f));    // disagreeing, before persist_s
    ASSERT_TRUE(!monitor.verified());                 // already not verified
    run(monitor, 1.0f, 0.0f, 20.0f);
    ASSERT_TRUE(monitor.mismatch() && !monitor.verified());
    monitor.reset();
    ASSERT_TRUE(!monitor.verified());                 // unjudged again after reset
    run(monitor, 1.5f, 0.0f, 0.0f);
    ASSERT_TRUE(monitor.verified());
    run(monitor, 0.1f, 0.0f, 0.0f, /*on_ground=*/false);
    ASSERT_TRUE(!monitor.verified());                 // off the ground
}

int main()
{
    printf("=== TiltConsistencyMonitor unit tests ===\n");

    run_sign_convention_matches_sf_math_inv_rotate();
    run_level_and_consistent_no_mismatch();
    run_disagreement_below_persist_no_mismatch();
    run_disagreement_for_persist_raises_mismatch();
    run_mismatch_clears_when_back_below_threshold();
    run_not_on_ground_is_false_and_resets_timer();
    run_glitch_sample_with_absurd_norm_is_ignored();
    run_reset_reseeds_filter();
    run_verified_is_a_positive_verdict();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
