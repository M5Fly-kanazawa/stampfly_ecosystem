/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_pid_derivative.cpp
 * @brief Host tests for the derivative term of sf::PID: td = 0 and the Nyquist gain
 *        sf::PID の微分項の host 試験: td = 0 とナイキストでのゲイン
 *
 * Background (2026-11-22 flights, simulation-policy.md backlog #12): for ANY td > 0 the
 * discretised derivative gives kp/eta at the Nyquist frequency, so the P + D loop gain there is
 * (1 + 1/eta) * kp = 9 * kp for eta = 0.125. The yaw rate loop therefore uses td = 0 (gain kp).
 * 背景（2026-11-22 の飛行、simulation-policy.md バックログ #12）: td > 0 なら、離散化した微分項の
 * ナイキスト周波数でのゲインは td によらず kp/eta で、P + D の一巡のゲインは eta = 0.125 で
 * (1 + 1/eta)·kp = 9·kp。ヨーの角速度ループは td = 0（ゲイン kp）を使う。
 *
 * Header-only: pid.hpp has no dependency, so this binary links nothing else.
 * ヘッダのみ: pid.hpp は依存を持たないので、他は何もリンクしない。
 *
 * @design detailed_design.md §4 — yaw rate PID without a derivative term   [OK]
 * @design coding_and_education.md §2 — Bilingual comments                   [OK]
 */

#include <cmath>
#include <cstdio>

#include "pid.hpp"

// Control period of the rate loops [s] (400 Hz). / レートループの制御周期 [s]（400 Hz）。
static constexpr float kControlPeriodS = 0.0025f;
// Default yaw rate gain [N*m/(rad/s)] (params.cpp rate.yaw.kp). / ヨーの角速度ゲイン既定値。
static constexpr float kYawKp = 8.029796e-4f;
// Large integral time [s]: makes the integral path negligible within a few samples.
// 大きな積分時間 [s]: 数標本の範囲では積分経路を無視できるようにする。
static constexpr float kNegligibleIntegralTimeS = 100.0f;
// Alternating measurement amplitude [rad/s] (sign flips every sample = Nyquist). / 交番する測定値の振幅。
static constexpr float kNyquistAmplitude = 1.0f;
// Samples to let the derivative filter settle (its pole is |a| < 1). / 微分フィルタが整定するまでの標本数。
static constexpr int kSettleSamples = 200;
// Tolerance on the gain ratio. / ゲイン比の許容。
static constexpr float kGainTolerance = 0.02f;

static int g_failures = 0;

#define CHECK(cond, ...) \
    do { \
        if (!(cond)) { \
            ++g_failures; \
            printf("      FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
            printf(__VA_ARGS__); \
            printf("\n"); \
        } \
    } while (0)

/// Output amplitude / kp for a measurement alternating +A, -A, +A, ... (setpoint 0).
/// 測定値が +A, -A, +A, ... と交番するときの出力振幅 / kp（目標 0）。
static float nyquistGainOverKp(sf::PID& pid)
{
    float last_output = 0.0f;
    for (int n = 0; n < kSettleSamples; ++n) {
        const float measurement = ((n % 2) == 0 ? 1.0f : -1.0f) * kNyquistAmplitude;
        last_output = pid.compute(0.0f, measurement, kControlPeriodS);
    }
    return std::fabs(last_output) / (pid.kp * kNyquistAmplitude);
}

static sf::PID makePid(float td)
{
    sf::PID pid;
    pid.kp = kYawKp;
    pid.ti = kNegligibleIntegralTimeS;
    pid.td = td;
    pid.output_limit = 1.0f;   // never reached / 到達しない
    return pid;
}

static void test_td_zero_has_no_derivative_output()
{
    printf("  [TEST] td_zero_output_is_proportional_plus_integral_only\n");
    sf::PID pid = makePid(0.0f);
    pid.compute(0.0f, 0.3f, kControlPeriodS);                       // first call primes / 初回は初期化のみ
    const float step_output = pid.compute(0.0f, 1.3f, kControlPeriodS);   // a 1.0 rad/s step in the measurement / 測定値の 1.0 rad/s の階段
    // P only: -kp * 1.3 (the integral over one sample of kp/ti is negligible).
    // P のみ: -kp × 1.3（1 標本での積分 kp/ti は無視できる）。
    CHECK(std::fabs(step_output + kYawKp * 1.3f) < 0.01f * kYawKp,
          "output %g, expected %g", static_cast<double>(step_output), static_cast<double>(-kYawKp * 1.3f));
}

static void test_td_zero_nyquist_gain_is_kp()
{
    printf("  [TEST] td_zero_nyquist_gain_is_kp\n");
    sf::PID pid = makePid(0.0f);
    const float gain = nyquistGainOverKp(pid);
    CHECK(std::fabs(gain - 1.0f) < kGainTolerance, "gain/kp = %g, expected 1", static_cast<double>(gain));
}

static void test_td_positive_nyquist_gain_is_one_plus_one_over_eta_for_any_td()
{
    printf("  [TEST] td_positive_nyquist_gain_is_(1 + 1/eta)*kp_for_any_td\n");
    const float tds[] = {0.002f, 0.01f, 0.025f};   // roll, old yaw, pitch defaults / ロール・旧ヨー・ピッチの既定値
    for (float td : tds) {
        sf::PID pid = makePid(td);
        const float expected = 1.0f + 1.0f / pid.eta;   // 9 for eta = 0.125 / eta = 0.125 で 9
        const float gain = nyquistGainOverKp(pid);
        CHECK(std::fabs(gain - expected) < expected * kGainTolerance,
              "td=%g: gain/kp = %g, expected %g", static_cast<double>(td),
              static_cast<double>(gain), static_cast<double>(expected));
    }
}

static void test_td_switched_to_zero_leaves_no_derivative_state()
{
    printf("  [TEST] td_switched_to_zero_leaves_no_derivative_state\n");
    sf::PID pid = makePid(0.01f);
    nyquistGainOverKp(pid);                       // fill the derivative filter state / 微分フィルタ状態を満たす
    CHECK(std::fabs(pid.deriv_filter) > 0.0f, "the filter state was not filled");
    pid.td = 0.0f;
    pid.compute(0.0f, 0.0f, kControlPeriodS);
    CHECK(pid.deriv_filter == 0.0f, "deriv_filter = %g, expected 0", static_cast<double>(pid.deriv_filter));
    // Back above 0: the output must start from a clean state (no stale decay).
    // 0 より大きく戻す: 古い値の減衰なしに、きれいな状態から始まる。
    pid.td = 0.01f;
    const float output = pid.compute(0.0f, 0.0f, kControlPeriodS);
    CHECK(std::fabs(output) < 0.01f * kYawKp, "output %g after re-enabling with a constant measurement",
          static_cast<double>(output));
}

static void test_td_zero_gives_finite_output_for_a_zero_dt_and_reset()
{
    printf("  [TEST] td_zero_is_finite_and_reset_clears_state\n");
    sf::PID pid = makePid(0.0f);
    CHECK(pid.compute(1.0f, 0.0f, 0.0f) == 0.0f, "dt = 0 must return 0");
    pid.compute(1.0f, 0.0f, kControlPeriodS);
    pid.reset();
    CHECK(pid.integral == 0.0f && pid.deriv_filter == 0.0f && pid.first_run, "reset left state behind");
    CHECK(std::isfinite(pid.compute(1.0f, 0.5f, kControlPeriodS)), "output not finite");
}

int main()
{
    printf("=== sf::PID derivative host tests ===\n");
    test_td_zero_has_no_derivative_output();
    test_td_zero_nyquist_gain_is_kp();
    test_td_positive_nyquist_gain_is_one_plus_one_over_eta_for_any_td();
    test_td_switched_to_zero_leaves_no_derivative_state();
    test_td_zero_gives_finite_output_for_a_zero_dt_and_reset();
    printf("\n=== %s (%d failed checks) ===\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
