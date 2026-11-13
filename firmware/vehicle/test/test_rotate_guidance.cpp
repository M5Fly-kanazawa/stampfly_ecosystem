/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_rotate_guidance.cpp
 * @brief Deterministic unit tests for the `cw` / `ccw` guidance rules (sf::rotate_guidance)
 *        `cw` / `ccw` の誘導規則（sf::rotate_guidance）の決定論ユニットテスト
 *
 * Header-only logic (rotate_guidance.hpp) — no ESP-IDF, no FreeRTOS. Own TEST/ASSERT framework
 * and main(), same pattern as test_api_arguments.cpp. The simulated craft turns at the yaw
 * rate limit toward the controller's shortest-path direction (wrapped error), the law the
 * firmware's controller applies, so the tests show the direction and total turn for every
 * angle 1..360 in both directions.
 * ヘッダのみのロジック（rotate_guidance.hpp）— ESP-IDF・FreeRTOS 不要。test_api_arguments.cpp と
 * 同じ自前の TEST/ASSERT フレームワークと main()。模擬の機体は、制御器と同じ法則（折り返した
 * 誤差の最短経路）に従いヨーレート上限で回るので、1〜360 度の全ての角度・両方向で、向きと
 * 回頭の総量を確かめられる。
 *
 * @design requirements.md §7 — コマンド受信 (RC入力、API) / TelloAPI   [OK]
 * @design detailed_design.md §3 注13 — API の引数の範囲と `cw` / `ccw` の回頭  [OK]
 */

#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <cstdlib>

#include "rotate_guidance.hpp"
#include "sf_math.hpp"

using sf::math::kPi;
using namespace sf::rotate_guidance;

// Simulation step [s] = the API's poll period (kWaitPollMs = 50 ms).
// 模擬の刻み [s] = API のポーリング周期（kWaitPollMs = 50 ms）。
static constexpr float kStepSeconds = 0.05f;
// Controller yaw P gain [1/s] (PidController::guide_yaw_kp_).
// 制御器のヨー P ゲイン [1/s]（PidController::guide_yaw_kp_）。
static constexpr float kControllerYawKp = sf::limits::kGuidanceYawKpPerSec;
static constexpr float kDegreesToRadians = kPi / 180.0f;
static constexpr float kToleranceRad = 1e-4f;

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

/// Result of one simulated turn / 模擬した 1 回の回頭の結果
struct TurnResult {
    float total_rad;      ///< unwrapped heading change / アンラップした向きの変化
    float seconds;        ///< time until the turn was complete / 完了までの時間
    float max_error_rad;  ///< largest |wrapped error| the controller saw / 制御器が見た折返し誤差の最大
    bool  completed;
    float published_rad;  ///< guidance target last handed to the controller (unwrapped) / 最後に制御器へ渡した目標
    float heading_rad;    ///< unwrapped measured heading at the end / 終了時のアンラップした測定の向き
};

/// Run a turn of `delta_rad` from `start_heading_rad` (wrapped, as the estimate is) with the
/// same loop as ApiTask::waitTurnComplete and a craft that follows the controller's P law.
/// `delta_rad` の回頭を、ApiTask::waitTurnComplete と同じ手順と、制御器の P 則に従う機体で模擬する。
static TurnResult simulateTurn(float start_heading_rad, float delta_rad, float gain = kControllerYawKp,
                               float time_limit_seconds = 0.0f)
{
    float truth = start_heading_rad;                // craft heading, unwrapped / 機体の向き（アンラップ）
    float unwrapped = sf::math::wrapAngleRad(truth);  // measured heading, unwrapped / 測定した向き（アンラップ）
    const float goal = unwrapped + delta_rad;
    float published = unwrapped;
    TurnResult result{0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f};
    const float timeout = time_limit_seconds > 0.0f ? time_limit_seconds : rotateTimeoutMs(delta_rad) * 0.001f;
    for (float t = 0.0f; t < timeout; t += kStepSeconds) {
        unwrapped = unwrapHeading(unwrapped, sf::math::wrapAngleRad(truth));
        published = nextGuidanceYaw(goal, unwrapped, published);
        if (isYawTurnComplete(goal, unwrapped)) {
            result.completed = true;
            result.seconds = t;
            break;
        }
        const float error = sf::math::wrapAngleRad(published - truth);
        result.max_error_rad = std::fmax(result.max_error_rad, std::fabs(error));
        float rate = gain * error;
        rate = std::fmax(-kYawRateLimitRadPerSec, std::fmin(kYawRateLimitRadPerSec, rate));
        truth += rate * kStepSeconds;
    }
    result.total_rad = unwrapped - start_heading_rad;
    result.published_rad = published;
    result.heading_rad = unwrapped;
    return result;
}

TEST(every_angle_turns_the_full_angle_in_the_commanded_direction)
{
    for (int degrees = 1; degrees <= 360; ++degrees) {
        for (float sign : {1.0f, -1.0f}) {
            const float delta = sign * degrees * kDegreesToRadians;
            const TurnResult result = simulateTurn(0.0f, delta);
            ASSERT_TRUE(result.completed);
            // Complete inside the tolerance of the commanded angle, in the commanded direction.
            // 指令角の許容差以内で、指令の向きに完了。
            ASSERT_TRUE(std::fabs(result.total_rad - delta) < kYawReachedToleranceRad + kToleranceRad);
        }
    }
}

TEST(turn_across_the_plus_minus_pi_seam_keeps_counting)
{
    for (float start_degrees : {170.0f, -170.0f, 179.0f, -179.0f, 90.0f}) {
        for (float delta_degrees : {90.0f, -90.0f, 270.0f, -360.0f, 180.0f}) {
            const float delta = delta_degrees * kDegreesToRadians;
            const TurnResult result = simulateTurn(start_degrees * kDegreesToRadians, delta);
            ASSERT_TRUE(result.completed);
            ASSERT_TRUE(std::fabs(result.total_rad - delta) < kYawReachedToleranceRad + kToleranceRad);
        }
    }
}

TEST(controller_error_never_reaches_180_degrees)
{
    for (int degrees = 1; degrees <= 360; ++degrees) {
        const TurnResult result = simulateTurn(0.0f, degrees * kDegreesToRadians);
        ASSERT_TRUE(result.max_error_rad <= kYawLeadMaxRad + kToleranceRad);
    }
}

TEST(turn_up_to_90_degrees_is_one_step_to_the_final_heading)
{
    // The same target the controller was handed before the lead existed: the final heading.
    // 先行量が無かった頃に制御器へ渡していた目標と同じ: 最終の向き。
    for (float degrees : {1.0f, 45.0f, 90.0f}) {
        const float goal = degrees * kDegreesToRadians;
        ASSERT_TRUE(nextGuidanceYaw(goal, 0.0f, 0.0f) == goal);
    }
}

TEST(a_residual_of_the_previous_turn_does_not_split_a_90_degree_turn)
{
    const float residual = kYawReachedToleranceRad;
    const float goal = 90.0f * kDegreesToRadians + residual;
    ASSERT_TRUE(nextGuidanceYaw(goal, 0.0f, 0.0f) == goal);
}

TEST(large_turn_places_the_target_the_lead_ahead_and_holds_it)
{
    const float goal = 2.0f * kPi;
    const float first = nextGuidanceYaw(goal, 0.0f, 0.0f);
    ASSERT_TRUE(std::fabs(first - kYawLeadMaxRad) < kToleranceRad);
    // Heading moved a little: the target stays until its lead shrinks to the refresh lead.
    // 向きが少し動いても、先行量が縮むまで目標は動かない。
    ASSERT_TRUE(nextGuidanceYaw(goal, 0.5f, first) == first);
    const float heading = first - kYawRefreshLeadRad - 0.01f;
    ASSERT_TRUE(nextGuidanceYaw(goal, heading, first) == first);
    const float shrunk_heading = first - kYawRefreshLeadRad + 0.01f;
    const float refreshed = nextGuidanceYaw(goal, shrunk_heading, first);
    ASSERT_TRUE(std::fabs(refreshed - (shrunk_heading + kYawLeadMaxRad)) < kToleranceRad);
}

TEST(negative_turn_places_the_target_behind_the_heading)
{
    const float first = nextGuidanceYaw(-2.0f * kPi, 0.0f, 0.0f);
    ASSERT_TRUE(std::fabs(first + kYawLeadMaxRad) < kToleranceRad);
}

TEST(lead_constants_keep_the_full_rate_and_the_shortest_path)
{
    // Error at which the P law reaches the rate limit.
    // P 則が速度上限に達する誤差。
    ASSERT_TRUE(kYawRefreshLeadRad > kYawRateLimitRadPerSec / kControllerYawKp);
    ASSERT_TRUE(kYawLeadMaxRad < kPi);
    ASSERT_TRUE(kYawLeadMaxRad > kPi / 2.0f + kYawReachedToleranceRad);
}

TEST(unwrap_heading_follows_a_continuous_turn_across_the_seam)
{
    float unwrapped = 3.0f;
    unwrapped = unwrapHeading(unwrapped, sf::math::wrapAngleRad(3.2f));
    ASSERT_TRUE(std::fabs(unwrapped - 3.2f) < kToleranceRad);
    unwrapped = unwrapHeading(unwrapped, sf::math::wrapAngleRad(4.5f));
    ASSERT_TRUE(std::fabs(unwrapped - 4.5f) < kToleranceRad);
    unwrapped = unwrapHeading(unwrapped, sf::math::wrapAngleRad(6.0f));
    ASSERT_TRUE(std::fabs(unwrapped - 6.0f) < kToleranceRad);
    unwrapped = unwrapHeading(unwrapped, sf::math::wrapAngleRad(-0.2f));
    ASSERT_TRUE(std::fabs(unwrapped - (2.0f * kPi - 0.2f)) < kToleranceRad);
}

TEST(a_full_turn_is_not_complete_where_it_started)
{
    // 360 degrees: the final heading is a full turn away although the wrapped error is zero.
    // 360 度: 折返した誤差は 0 でも、最終の向きは 1 周先。
    ASSERT_TRUE(!isYawTurnComplete(2.0f * kPi, 0.0f));
    ASSERT_TRUE(isYawTurnComplete(2.0f * kPi, 2.0f * kPi - 0.1f));
}

TEST(timeout_is_the_turn_time_at_the_rate_limit_plus_the_margin)
{
    ASSERT_TRUE(rotateTimeoutMs(0.0f) == kRotateMarginMs);
    ASSERT_TRUE(rotateTimeoutMs(1.0f) == 1000u + kRotateMarginMs);
    ASSERT_TRUE(rotateTimeoutMs(-1.0f) == 1000u + kRotateMarginMs);
    // A full turn needs 6.3 s at the rate limit; the margin is left over.
    // 1 周はヨーレート上限で 6.3 s かかる。余裕はその上に残る。
    ASSERT_TRUE(rotateTimeoutMs(2.0f * kPi) == 6283u + kRotateMarginMs);
}

TEST(a_full_turn_finishes_well_inside_its_timeout)
{
    const TurnResult result = simulateTurn(0.0f, 2.0f * kPi);
    ASSERT_TRUE(result.completed);
    ASSERT_TRUE(result.seconds < rotateTimeoutMs(2.0f * kPi) * 0.001f - 4.0f);
}

TEST(a_lower_gain_still_turns_the_right_way_only_slower)
{
    // Gain x 60 degrees below the rate limit: the 60-degree lead no longer saturates the P law.
    // The turn must still complete in the commanded direction, never with a 180-degree error.
    // ゲイン × 60 度 が上限を下回る: 60 度の先行角はもう P 則を飽和させない。それでも回頭は指令の向きに
    // 完了し、誤差は 180 度に届かない。
    float previous_seconds = 0.0f;
    for (float gain : {4.0f, 2.0f, 1.0f, 0.5f, 0.25f}) {
        for (float sign : {1.0f, -1.0f}) {
            const float delta = sign * 2.0f * kPi;
            const TurnResult result = simulateTurn(0.0f, delta, gain, 200.0f);
            ASSERT_TRUE(result.completed);
            ASSERT_TRUE(std::fabs(result.total_rad - delta) < kYawReachedToleranceRad + kToleranceRad);
            ASSERT_TRUE(result.max_error_rad <= kYawLeadMaxRad + kToleranceRad);
            if (sign > 0.0f) {
                printf("(gain %.2f: 360 deg in %.1f s) ", gain, result.seconds);
                ASSERT_TRUE(result.seconds >= previous_seconds);   // lower gain: not faster / 低ゲインは速くならない
                previous_seconds = result.seconds;
            }
        }
    }
}

TEST(the_timeout_covers_a_full_turn_down_to_the_gain_that_keeps_the_rate_limit)
{
    // At the shipped gain (and any gain with gain x refresh lead >= rate limit) a full turn is
    // inside the timeout; a gain far below that would not be, which is why the constants are shared.
    // 出荷時のゲイン（および ゲイン × 再設定の先行角 >= 上限 の任意のゲイン）では 1 周が待ち時間に収まる。
    // それを大きく下回るゲインでは収まらない。定数を共有しているのはそのため。
    for (float gain : {4.0f, 2.0f, kYawRateLimitRadPerSec / kYawRefreshLeadRad}) {
        ASSERT_TRUE(simulateTurn(0.0f, 2.0f * kPi, gain).completed);
    }
    ASSERT_TRUE(!simulateTurn(0.0f, 2.0f * kPi, 0.25f).completed);
}

TEST(after_a_timeout_the_target_is_the_last_point_given_and_the_next_turn_composes_on_it)
{
    // Gain 0.25 cannot finish a full turn inside the timeout (the API answers `error rotate timeout`).
    // The bookkeeping then holds the last point handed to the controller: at most the lead ahead of the
    // heading in the commanded direction (the final heading itself once it was within the lead), never
    // beyond it. A following `cw 90`
    // composes on that point and the craft ends 90 degrees beyond it.
    // ゲイン 0.25 では 1 周が待ち時間内に終わらない（API は `error rotate timeout` を返す）。帳簿は制御器に
    // 最後に渡した点を持つ: 向きの先行量以内で指令の向き、届かない最終の向きではない。続く `cw 90` は
    // その点に合成され、機体はその点の 90 度先で止まる。
    const float gain = 0.25f;
    const TurnResult timed_out = simulateTurn(0.0f, 2.0f * kPi, gain);
    ASSERT_TRUE(!timed_out.completed);
    const float lead = timed_out.published_rad - timed_out.heading_rad;
    ASSERT_TRUE(lead > 0.0f && lead <= kYawLeadMaxRad + kToleranceRad);
    ASSERT_TRUE(timed_out.published_rad <= 2.0f * kPi + kToleranceRad);   // never beyond the final heading / 最終の向きを越えない
    const float target_wrapped = sf::math::wrapAngleRad(timed_out.published_rad);
    const float measured_wrapped = sf::math::wrapAngleRad(timed_out.heading_rad);
    const float goal = composeFinalHeading(measured_wrapped, target_wrapped, kPi / 2.0f);
    ASSERT_TRUE(std::fabs(goal - (measured_wrapped + sf::math::wrapAngleRad(target_wrapped - measured_wrapped) +
                                  kPi / 2.0f)) < kToleranceRad);
    const TurnResult next = simulateTurn(measured_wrapped + sf::math::wrapAngleRad(target_wrapped - measured_wrapped),
                                         kPi / 2.0f, gain, 100.0f);
    ASSERT_TRUE(next.completed);
    ASSERT_TRUE(std::fabs(next.total_rad - kPi / 2.0f) < kYawReachedToleranceRad + kToleranceRad);
}

TEST(compose_final_heading_does_not_carry_the_estimate_residual)
{
    // Previous target 0 rad, heading 0.1 rad (a residual): the next 90-degree turn ends at the target
    // plus 90 degrees (heading + (-0.1) + 90 deg), not at heading + 90 degrees.
    // 直前の目標 0 rad、向き 0.1 rad（残差）: 次の 90 度は 目標 + 90 度 で終わり、向き + 90 度 ではない。
    const float goal = composeFinalHeading(0.1f, 0.0f, kPi / 2.0f);
    ASSERT_TRUE(std::fabs(goal - kPi / 2.0f) < kToleranceRad);
    // Across the seam: heading just past +pi, target just before it.
    // 継ぎ目をまたぐ: 向きは +pi を少し越え、目標は手前。
    const float across = composeFinalHeading(-kPi + 0.05f, kPi - 0.05f, kPi / 2.0f);
    ASSERT_TRUE(std::fabs(across - (-kPi + 0.05f - 0.1f + kPi / 2.0f)) < kToleranceRad);
}

int main()
{
    printf("=== sf::rotate_guidance unit tests ===\n");

    run_every_angle_turns_the_full_angle_in_the_commanded_direction();
    run_turn_across_the_plus_minus_pi_seam_keeps_counting();
    run_controller_error_never_reaches_180_degrees();
    run_turn_up_to_90_degrees_is_one_step_to_the_final_heading();
    run_a_residual_of_the_previous_turn_does_not_split_a_90_degree_turn();
    run_large_turn_places_the_target_the_lead_ahead_and_holds_it();
    run_negative_turn_places_the_target_behind_the_heading();
    run_lead_constants_keep_the_full_rate_and_the_shortest_path();
    run_unwrap_heading_follows_a_continuous_turn_across_the_seam();
    run_a_full_turn_is_not_complete_where_it_started();
    run_timeout_is_the_turn_time_at_the_rate_limit_plus_the_margin();
    run_a_full_turn_finishes_well_inside_its_timeout();
    run_a_lower_gain_still_turns_the_right_way_only_slower();
    run_the_timeout_covers_a_full_turn_down_to_the_gain_that_keeps_the_rate_limit();
    run_after_a_timeout_the_target_is_the_last_point_given_and_the_next_turn_composes_on_it();
    run_compose_final_heading_does_not_carry_the_estimate_residual();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
