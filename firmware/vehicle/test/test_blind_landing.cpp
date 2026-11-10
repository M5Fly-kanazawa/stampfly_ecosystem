/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_blind_landing.cpp
 * @brief Host tests for the descent with a silent downward ToF: velocity estimator, velocity loop,
 *        touchdown detector, late fall, support thrust
 *        下向き ToF の出力停止中の降下のホスト試験: 速度推定器・速度ループ・接地検出器・後半の低下・支持推力
 *
 * No ESP-IDF / FreeRTOS (header-only logic):
 *  1. Late fall: 1 until the start time, falls at the set rate, zero at the time the constants promise,
 *     and the time limit is longer than that.
 *  2. VerticalVelocityEstimator: rest (zero, offset learning), integration (hover reading = no motion,
 *     the offset B frozen), the ESKF hand-off (not followed until it has been free for the settle time,
 *     then followed, B learned), a ToF silence returning to the integral, an unusable reading, the
 *     barometer interface, the magnitude limit.
 *  3. BlindDescent (PI under a thrust ceiling below the weight) on a 1-D vertical plant with a motor lag: the thrust never
 *     exceeds the ceiling whatever the velocity estimate says, landing for a support-thrust error up to the margin,
 *     the constant ceiling after the trust time, an old estimate, a support thrust above the margin ending by the late fall,
 *     the thrust limits and the anti-windup, the late-fall time over one landing, the thrust ratio the touchdown detector reads.
 *  4. BlindTouchdownDetector: impact alone, stillness alone, steady descent, impact then stillness,
 *     each of the three stillness conditions, the bounce, reset, in-flight vibration, and the second path (the floor holding
 *     the craft up: no impact needed, never in the air).
 *  5. Support thrust (SupportThrustFilter): the window opens when the craft leaves the floor, near-hover samples only,
 *     accelerometer offset error, bounds, the lowered estimate and the early estimate without a window.
 *  6. Upward specific force from the attitude and the body reading.
 *  7. The pure pieces of the closed-loop landing (ToF alive): the base thrust bound and the rise brake.
 * Own TEST/ASSERT framework and main(), same pattern as test_tof_silence.cpp.
 *
 * ESP-IDF・FreeRTOS 不要（ヘッダのみのロジック）:
 *  1. 後半の低下: 開始時刻まで 1、設定した率で下がり、定数が約束する時刻でゼロ、時間切れはそれより長い。
 *  2. VerticalVelocityEstimator: 静止（ゼロ・オフセットの学習）、積分（ホバーの読み = 動かない、オフセット B は
 *     固定されたまま）、ESKF の引き継ぎ（整定時間だけ自由になるまで従わず、その後従い、B を学習）、ToF の出力停止で積分に
 *     戻ること、使えない読み、気圧計のインタフェース、大きさの制限。
 *  3. BlindDescent（重さ未満の推力の上限の下での PI）をモータの遅れ付きの 1 次元鉛直プラントで: 速度推定が何でも推力が
 *     上限を超えないこと、余裕までの支持推力の誤差での着地、信用時間を過ぎた後の一定の上限、古い推定、余裕を超えた
 *     支持推力が後半の低下で終わること、推力の制限とアンチワインドアップ、後半の低下の時間が 1 回の LANDING で通算
 *     されること、接地検出器が読む推力比。
 *  4. BlindTouchdownDetector: 衝撃だけ、静止だけ、定常の降下、衝撃のあとの静止、静止の 3 条件それぞれ、
 *     バウンド、リセット、飛行中の振動、2 つ目の経路（床が機体を支えている: 衝撃なしで、空中では決してラッチしない）。
 *  5. 支持推力（SupportThrustFilter）: 機体が床を離れたときに窓が開くこと、ホバー近傍のサンプルだけ、加速度計のオフセット
 *     誤差、制限、窓が無いときの低くした見積もりと早い推定。
 *  6. 姿勢と機体座標の測定値から上向きの加速度計の測定値。
 *  7. 閉ループ着陸（ToF 生存）の純関数: 基準推力の上限、上昇ブレーキ。
 * test_tof_silence.cpp と同じ自前の TEST/ASSERT フレームワークと main()。
 *
 * @design detailed_design.md §3.1 — ToF silent: blind descent by vertical-velocity loop [OK]
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "blind_descent.hpp"
#include "blind_landing.hpp"
#include "blind_touchdown.hpp"
#include "flight_limits.hpp"
#include "sf_math.hpp"
#include "vertical_velocity_estimator.hpp"

namespace lim = sf::limits;
constexpr float kG = sf::math::kGravity;

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-56s ", #name); \
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
        printf("FAIL: %s:%d: %g vs %g (tol %g)\n", __FILE__, __LINE__, \
               static_cast<double>(a), static_cast<double>(b), static_cast<double>(tol)); \
        throw 1; \
    }

static constexpr float kStart = 0.34f;    // [N] a hover support thrust (the SILS plant hovers at 0.34 N)
static constexpr float kDt = 0.0025f;     // [s] 400 Hz control / IMU cycle

// ----------------------------------------------------------------------------
// 1. Late fall / 後半の低下
// ----------------------------------------------------------------------------

TEST(late_factor_is_one_then_falls_to_zero)
{
    ASSERT_NEAR(sf::blindLandingLateFactor(0.0f), 1.0f, 1e-6f);
    ASSERT_NEAR(sf::blindLandingLateFactor(lim::kBlindLandingLateStartS), 1.0f, 1e-6f);
    const float one_second_later = lim::kBlindLandingLateStartS + 1.0f;
    ASSERT_NEAR(sf::blindLandingLateFactor(one_second_later), 1.0f - lim::kBlindLandingLateRampPerS, 1e-6f);
    const float t_zero = lim::kBlindLandingLateStartS + 1.0f / lim::kBlindLandingLateRampPerS;
    ASSERT_TRUE(sf::blindLandingLateFactor(t_zero + 0.01f) == 0.0f);
    ASSERT_TRUE(sf::blindLandingLateFactor(100.0f) == 0.0f);
    float previous = 1.0f;                                   // never increases / 増えない
    for (float t = 0.0f; t < 20.0f; t += 0.05f) {
        const float factor = sf::blindLandingLateFactor(t);
        ASSERT_TRUE(factor <= previous + 1e-6f);
        ASSERT_TRUE(factor >= 0.0f && factor <= 1.0f);
        previous = factor;
    }
}

TEST(time_limit_leaves_the_fall_after_zero_thrust)
{
    // The limit must be longer than the time the thrust reaches zero by at least the fall time from
    // 6 m (1.1 s), so the motors are never cut while the thrust is still holding the craft up.
    // 時間切れは推力がゼロになる時刻より、少なくとも 6 m からの落下時間（1.1 s）だけ長いこと。
    const float t_zero = lim::kBlindLandingLateStartS + 1.0f / lim::kBlindLandingLateRampPerS;
    const float fall_6m = std::sqrt(2.0f * 6.0f / kG);
    const float limit_s = static_cast<float>(lim::kBlindLandingTimeoutUs) * 1e-6f;
    ASSERT_TRUE(limit_s >= t_zero + fall_6m - 0.05f);
    ASSERT_TRUE(sf::blindLandingLateFactor(limit_s) == 0.0f);   // thrust ZERO at the limit / 時間切れでは推力ゼロ
}

TEST(late_fall_starts_after_a_normal_descent_from_the_ceiling)
{
    // A descent from the altitude ceiling at the target speed ends before the late fall begins, so
    // the late fall never acts in a normal landing.
    // 天井の高度から目標速度で降りる降下は後半の低下が始まる前に終わる: 通常の着陸では働かない。
    const float normal_s = lim::kAltitudeCeilingM / lim::kBlindDescentRateMps + 1.0f;   // + the first 1 s to get going / 立ち上がりの 1 s
    ASSERT_TRUE(normal_s < lim::kBlindLandingLateStartS);
}

// ----------------------------------------------------------------------------
// 2. Vertical velocity estimator / 鉛直速度推定器
// ----------------------------------------------------------------------------

static sf::VerticalVelocityInput veInput(float raw_force, bool at_rest, bool trusted, float eskf_vz)
{
    sf::VerticalVelocityInput in{};
    in.dt_s              = kDt;
    in.at_rest_on_ground = at_rest;
    in.eskf_trusted      = trusted;
    in.eskf_velocity_up  = eskf_vz;
    in.upward_force_raw  = raw_force;
    return in;
}

static void veRun(sf::VerticalVelocityEstimator& e, float seconds, float raw_force, bool at_rest, bool trusted, float eskf_vz)
{
    const int n = static_cast<int>(seconds / kDt + 0.5f);
    for (int i = 0; i < n; ++i) e.update(veInput(raw_force, at_rest, trusted, eskf_vz));
}

TEST(estimator_rest_is_zero_and_learns_the_offset)
{
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 1.0f, kG + 0.2f, true, false, 0.7f);    // at rest the reading is g + offset; the ESKF value is ignored / 静止: 読みは g + オフセット。ESKF の値は無視
    ASSERT_TRUE(e.velocityUp() == 0.0f);
    ASSERT_TRUE(e.source() == sf::VerticalVelocitySource::GroundZero);
    ASSERT_NEAR(e.offsetUp(), 0.2f, 1e-4f);
}

TEST(estimator_integrates_the_acceleration)
{
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG, true, false, 0.0f);          // rest: offset 0 / 静止: オフセット 0
    veRun(e, 2.0f, kG, false, false, 0.0f);         // hover reading: no motion / ホバーの読み: 動かない
    ASSERT_NEAR(e.velocityUp(), 0.0f, 1e-3f);
    ASSERT_TRUE(e.source() == sf::VerticalVelocitySource::Integrated);
    veRun(e, 0.5f, kG + 2.0f, false, false, 0.0f);  // +2 m/s^2 for 0.5 s = +1 m/s / +2 m/s^2 を 0.5 s で +1 m/s
    ASSERT_NEAR(e.velocityUp(), 1.0f, 2e-3f);
    veRun(e, 1.0f, kG - 1.0f, false, false, 0.0f);  // -1 m/s^2 for 1 s / −1 m/s^2 を 1 s
    ASSERT_NEAR(e.velocityUp(), 0.0f, 3e-3f);
}

TEST(estimator_rise_is_zero_at_rest_and_the_integral_of_the_velocity_in_the_air)
{
    // The rise starts at the take-off (the ToF health tracker uses it to tell "left the floor" from "still in the
    // sensor's dead zone"): 0 at rest, and 1 m/s for 0.5 s is 0.25 m (from a constant +2 m/s^2 for 0.5 s) + 0.5 m.
    // 上昇量は離陸から始まる（ToF 健全性トラッカーが「床を離れた」と「まだセンサの不感帯にいる」の区別に使う）:
    // 静止中は 0。+2 m/s^2 を 0.5 s で 0.25 m、続く 1 m/s の 0.5 s で 0.5 m。
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG, true, false, 0.0f);
    ASSERT_TRUE(e.riseM() == 0.0f);
    veRun(e, 0.5f, kG + 2.0f, false, false, 0.0f);
    ASSERT_NEAR(e.riseM(), 0.25f, 5e-3f);
    veRun(e, 0.5f, kG, false, false, 0.0f);
    ASSERT_NEAR(e.riseM(), 0.75f, 8e-3f);
    veRun(e, 0.1f, kG, true, false, 0.0f);          // at rest again: the rise restarts from 0 / 再び静止: 上昇量は 0 から
    ASSERT_TRUE(e.riseM() == 0.0f);
    e.reset();
    ASSERT_TRUE(e.riseM() == 0.0f);
}

TEST(estimator_align_to_the_eskf_drops_the_integral_of_the_episode)
{
    // After a long episode without a ToF the integral carries the offset error (here 0.4 m/s^2 for 10 s = 4 m/s on a craft that does not climb). When the ESKF takes the
    // ToF again, the velocity is set to the ESKF's at once (the age of the integral restarts); the rise is not touched.
    // ToF なしの長いエピソードの後、積分はオフセット誤差を持つ（ここでは 0.4 m/s^2 を 10 s で、上昇していない機体に 4 m/s）。ESKF が ToF を再び
    // 取り込んだら、速度は ESKF の値へ一度に合わせ（積分の年齢は数え直し）、上昇量には触れない。
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG, true, false, 0.0f);
    veRun(e, 1.0f, kG + 2.0f, false, false, 0.0f);          // the take-off / 離陸
    veRun(e, 10.0f, kG + 0.4f, false, false, 0.0f);         // the episode: the offset error integrates / エピソード: オフセット誤差が積分される
    ASSERT_TRUE(e.velocityUp() > 3.0f);
    const float rise = e.riseM();
    e.alignToEskf(0.05f);
    ASSERT_NEAR(e.velocityUp(), 0.05f, 1e-6f);
    ASSERT_TRUE(e.sinceResetS() == 0.0f);
    ASSERT_NEAR(e.riseM(), rise, 1e-6f);
}

TEST(estimator_time_since_reset_counts_the_integral_only)
{
    // The age of the integral: 0 at rest on the ground (rule 1) and while the ESKF is followed (rule 2); it grows only
    // while the accelerometer is integrated (rule 3), and restarts from the last reset.
    // 積分の年齢: 地上の静止（規則 1）と ESKF に従っている間（規則 2）は 0。加速度計を積分している間（規則 3）だけ増え、
    // 最後のリセットから数え直す。
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG, true, false, 0.0f);
    ASSERT_TRUE(e.sinceResetS() == 0.0f);
    veRun(e, 2.0f, kG, false, false, 0.0f);                 // integrating / 積分中
    ASSERT_NEAR(e.sinceResetS(), 2.0f, 0.01f);
    veRun(e, lim::kVerticalVelocityHandoffSettleS + 0.1f, kG, false, true, 0.0f);   // the ESKF is followed / ESKF に従う
    ASSERT_TRUE(e.source() == sf::VerticalVelocitySource::Eskf && e.sinceResetS() == 0.0f);
    veRun(e, 0.5f, kG, false, false, 0.0f);                 // the ToF is lost: the integral restarts / ToF を失う: 積分が始まる
    ASSERT_NEAR(e.sinceResetS(), 0.5f, 0.01f);
    e.reset();
    ASSERT_TRUE(e.sinceResetS() == 0.0f);
}

TEST(estimator_subtracts_the_offset_learned_at_rest)
{
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG + 0.2f, true, false, 0.0f);   // the sensor reads 0.2 m/s^2 high / センサは 0.2 m/s^2 高く読む
    veRun(e, 3.0f, kG + 0.2f, false, false, 0.0f);  // a hover reading: without the offset v would reach 0.6 m/s / オフセット無しなら 0.6 m/s
    ASSERT_NEAR(e.velocityUp(), 0.0f, 5e-3f);
}

TEST(estimator_does_not_follow_the_eskf_until_it_has_been_free_for_the_settle_time)
{
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG, true, false, 0.0f);
    // A climb at +2 m/s^2 (the accelerometer sees it) while the ESKF still says 0 (pinned, or just handed
    // off and not converged): the integral is followed, not the ESKF.
    // +2 m/s^2 の上昇（加速度計には見える）で ESKF はまだ 0（固定中、または引き継ぎ直後で未収束）: 従うのは積分。
    veRun(e, 0.5f, kG + 2.0f, false, true, 0.0f);
    ASSERT_TRUE(e.source() == sf::VerticalVelocitySource::Integrated);
    ASSERT_NEAR(e.velocityUp(), 1.0f, 2e-3f);
    // Once the ESKF has been free for the settle time the observer follows it (time constant tau).
    // ESKF が整定時間だけ自由になったら、オブザーバがそれに従う（時定数 tau）。
    veRun(e, lim::kVerticalVelocityHandoffSettleS, kG, false, true, 1.0f);
    ASSERT_TRUE(e.source() == sf::VerticalVelocitySource::Eskf);
    ASSERT_NEAR(e.velocityUp(), 1.0f, 0.05f);
}

TEST(estimator_follows_the_eskf_and_learns_the_offset_while_the_tof_is_healthy)
{
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG, true, false, 0.0f);            // rest: offset 0 / 静止: オフセット 0
    // In flight the sensor reads 0.2 m/s^2 high (vibration rectification) and the ESKF says "no motion":
    // the observer settles at v = 0 and B = 0.2 (an integral alone would have drifted 0.2 m/s per second).
    // 飛行中センサが 0.2 m/s^2 高く読み（振動による読みの偏り）、ESKF は「動いていない」: オブザーバは v = 0、B = 0.2 に
    // 落ち着く（積分だけなら 1 s に 0.2 m/s ドリフトする）。
    veRun(e, 10.0f * lim::kVerticalVelocityEskfTauS, kG + 0.2f, false, true, 0.0f);
    ASSERT_TRUE(e.source() == sf::VerticalVelocitySource::Eskf);
    ASSERT_NEAR(e.velocityUp(), 0.0f, 0.01f);
    ASSERT_NEAR(e.offsetUp(), 0.2f, 0.02f);
    // The ToF goes silent: B is frozen, the integral continues, and it does NOT drift.
    // ToF が出力停止: B は固定されたまま積分を続け、ドリフトしない。
    veRun(e, 3.0f, kG + 0.2f, false, false, 0.0f);
    ASSERT_TRUE(e.source() == sf::VerticalVelocitySource::Integrated);
    ASSERT_NEAR(e.velocityUp(), 0.0f, 0.03f);
}

TEST(estimator_silence_keeps_the_climb_the_accelerometer_saw)
{
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG, true, false, 0.0f);
    veRun(e, 4.0f, kG, false, true, 0.0f);            // hover, ESKF healthy / ホバー、ESKF 健全
    veRun(e, 0.3f, kG + 3.0f, false, true, 0.0f);     // a climb starts: the accelerometer sees it at once / 上昇開始: 加速度計は即座に見る
    ASSERT_TRUE(e.velocityUp() > 0.5f);
    veRun(e, 0.4f, kG, false, false, 0.0f);           // the ToF falls silent while the climb goes on / 上昇中に ToF が出力停止
    ASSERT_TRUE(e.source() == sf::VerticalVelocitySource::Integrated);
    ASSERT_TRUE(e.velocityUp() > 0.5f);               // an ESKF reset to 0 would have lost it / ESKF に 0 へ戻していたら失う
}

TEST(estimator_contact_sets_the_velocity_to_zero)
{
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG, true, false, 0.0f);
    veRun(e, 0.4f, kG - 4.0f, false, false, 0.0f);      // falls to -1.6 m/s / −1.6 m/s まで落ちる
    ASSERT_TRUE(e.velocityUp() < -1.0f);
    // The floor: a spike of 3 samples above the contact threshold. The integral of the spike is wrong
    // (clipped / noisy); the release sets the velocity to zero.
    // 床: 接触の閾値を超える 3 サンプルの突起。突起の積分は不正確（頭打ち・ノイズ）で、解除で速度を 0 にする。
    const float spike = lim::kBlindTouchdownAccelMps2 * 1.5f;
    for (int i = 0; i < 3; ++i) e.update(veInput(spike, false, false, 0.0f));
    ASSERT_TRUE(e.velocityUp() != 0.0f);                // still pending during the spike / 突起の間は保留
    veRun(e, 0.05f, kG, false, false, 0.0f);            // the reading falls back / 読みが戻る
    ASSERT_TRUE(e.velocityUp() == 0.0f);
    // Vibration below the threshold, or a single high sample, is not a contact.
    // 閾値未満の振動や 1 サンプルの高い値は接触でない。
    sf::VerticalVelocityEstimator f;
    f.reset();
    veRun(f, 0.5f, kG, true, false, 0.0f);
    veRun(f, 0.4f, kG - 4.0f, false, false, 0.0f);
    const float before = f.velocityUp();
    f.update(veInput(spike, false, false, 0.0f));       // one sample only / 1 サンプルだけ
    veRun(f, 0.05f, kG, false, false, 0.0f);
    ASSERT_TRUE(f.velocityUp() != 0.0f && std::fabs(f.velocityUp() - before) < 0.5f);
    // While the ToF is healthy the ESKF owns the value: no contact rule.
    // ToF が健全な間は ESKF が値を持つ: 接触の規則は働かない。
    sf::VerticalVelocityEstimator g;
    g.reset();
    veRun(g, 0.5f, kG, true, false, 0.0f);
    veRun(g, 8.0f * lim::kVerticalVelocityEskfTauS, kG, false, true, -0.4f);   // the observer has converged to the ESKF value / オブザーバが ESKF の値に収束
    ASSERT_NEAR(g.velocityUp(), -0.4f, 0.03f);
    for (int i = 0; i < 3; ++i) g.update(veInput(spike, false, true, -0.4f));
    veRun(g, 0.05f, kG, false, true, -0.4f);
    ASSERT_TRUE(g.velocityUp() < -0.2f);
}

TEST(estimator_ignores_an_unusable_reading)
{
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG, true, false, 0.0f);
    veRun(e, 0.2f, kG + 2.0f, false, false, 0.0f);
    const float before = e.velocityUp();
    veRun(e, 1.0f, 0.0f, false, false, 0.0f);         // an estimator without specific_force reads 0 / specific_force を出さない推定器は 0
    ASSERT_NEAR(e.velocityUp(), before, 1e-6f);
    veRun(e, 0.5f, 0.0f, false, true, 0.7f);          // ... and with no reading the ESKF value is copied once followed / 読みが無ければ従う段階で ESKF の値を写す
    veRun(e, lim::kVerticalVelocityHandoffSettleS, 0.0f, false, true, 0.7f);
    ASSERT_NEAR(e.velocityUp(), 0.7f, 1e-6f);
}

TEST(estimator_barometer_interface_acts_only_while_integrating)
{
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG, true, false, 0.0f);
    e.fuseExternalVelocity(1.0f, 1.0f);                // at rest: ignored / 静止中: 無視
    ASSERT_TRUE(e.velocityUp() == 0.0f);
    veRun(e, 0.5f, kG + 2.0f, false, false, 0.0f);     // integrating, v = +1 / 積分中、v = +1
    e.fuseExternalVelocity(0.0f, 0.5f);                // half way to the barometer's 0 / 気圧計の 0 へ半分寄せる
    ASSERT_NEAR(e.velocityUp(), 0.5f, 3e-3f);
    e.fuseExternalVelocity(0.0f, 5.0f);                // the gain is clamped to 1 / ゲインは 1 に制限
    ASSERT_NEAR(e.velocityUp(), 0.0f, 3e-3f);
    e.fuseExternalVelocity(9.0f, -1.0f);               // ... and to 0 from below / 負は 0 に制限
    ASSERT_NEAR(e.velocityUp(), 0.0f, 3e-3f);
}

TEST(estimator_value_is_limited)
{
    sf::VerticalVelocityEstimator e;
    e.reset();
    veRun(e, 0.5f, kG, true, false, 0.0f);
    veRun(e, 20.0f, kG + 50.0f, false, false, 0.0f);   // a runaway reading / 異常に大きい読み
    ASSERT_NEAR(e.velocityUp(), lim::kVerticalVelocityLimitMps, 1e-4f);
}

// ----------------------------------------------------------------------------
// 3. BlindDescent loop on a 1-D plant / 1 次元プラントでの BlindDescent ループ
// ----------------------------------------------------------------------------

struct DescentResult {
    bool  landed;
    float touchdown_time_s;
    float touchdown_speed;    // [m/s] down
    float peak_rise;          // [m] above the start height
    float speed_at_2s;        // [m/s] up positive, 2 s after the start
    float max_ratio;          // largest thrust / support thrust seen
};

// 1-D vertical plant: the thrust follows the command through a first-order lag (motor, 50 ms),
// a = g * (thrust / true_hover - 1). `support_over_hover` is the error of the support thrust (the start
// of the loop and the reference of the thrust ceiling) against the true hover thrust; `hover_estimate_ratio` the error of
// the hover estimate the PI gains scale with; `velocity_bias` is added to the velocity the loop sees; `h0`, `v0`
// the start; `age0_s` the time since the last reset of the velocity estimate at the start (it grows with the run).
// 1 次元の鉛直プラント: 推力は指令に 1 次遅れ（モータ、50 ms）で追従し、a = g × (推力 ÷ 真のホバー推力 − 1)。
// support_over_hover は支持推力（ループの開始と推力の上限の基準）の真のホバー推力に対する誤差、
// hover_estimate_ratio は PI のゲインが比例する hover 推力の見積もりの誤差、velocity_bias はループが見る速度に足す誤差、
// h0・v0 は開始、age0_s は開始時の速度推定の最後のリセットからの時間（走行とともに増える）。
static DescentResult descend(float support_over_hover, float hover_estimate_ratio, float velocity_bias, float h0, float v0,
                             float run_s = 40.0f, float age0_s = 0.3f)
{
    const float true_hover = 0.34f;
    const float motor_lag_s = 0.05f;
    sf::BlindDescent loop;
    sf::BlindDescentGains gains;
    loop.start(support_over_hover * true_hover, hover_estimate_ratio * true_hover, gains);
    float h = h0, v = v0, thrust_actual = true_hover, peak = h0, v2 = 0.0f, max_ratio = 0.0f;
    for (float t = 0.0f; t < run_s; t += kDt) {
        const float command = loop.update(v + velocity_bias, age0_s + t, kDt);
        if (loop.thrustRatio() > max_ratio) max_ratio = loop.thrustRatio();
        thrust_actual += (command - thrust_actual) * (kDt / motor_lag_s);
        v += kG * (thrust_actual / true_hover - 1.0f) * kDt;
        h += v * kDt;
        if (h > peak) peak = h;
        if (t < 2.0f && t + kDt >= 2.0f) v2 = v;
        if (h <= 0.0f) return {true, t, -v, peak - h0, v2, max_ratio};
    }
    return {false, run_s, 0.0f, peak - h0, v2, max_ratio};
}

TEST(loop_thrust_never_exceeds_the_ceiling_below_the_weight)
{
    // Whatever the velocity estimate says (a bias of -2 .. +2 m/s, the estimator's own drift), the thrust never exceeds
    // kNoTofThrustCapRatio x the support thrust, and a support thrust that is not above the true hover by more than the
    // margin (1 / ratio - 1 = 5.3 %) therefore never lets the craft rise from a hover.
    // 速度推定が何であれ（−2〜+2 m/s の誤差、推定器自身のドリフト）、推力は kNoTofThrustCapRatio × 支持推力を超えない。
    // 支持推力が真のホバーを余裕（1 / 比 − 1 = 5.3 %）を超えて上回らなければ、機体はホバーから上昇しない。
    for (float bias : {-2.0f, -1.0f, -0.3f, 0.0f, 0.3f, 1.0f, 2.0f}) {
        for (float error : {-0.20f, -0.05f, 0.0f, 0.01f, 0.03f}) {
            const DescentResult r = descend(1.0f + error, 1.0f, bias, 1.2f, 0.0f);
            ASSERT_TRUE(r.max_ratio <= lim::kNoTofThrustCapRatio + 1e-5f);
            ASSERT_TRUE(r.peak_rise < 0.005f);
            ASSERT_TRUE(r.landed);
        }
    }
}

TEST(loop_lands_gently_for_a_support_thrust_error_up_to_the_margin)
{
    // From a hover and from a climb at 1.3 m/s, for a support thrust 2 % below to 3 % above the true hover (the calibration
    // error is about 1 %): the craft lands from the 1.2 m ceiling within the time limit, below 1.5 m/s (the speed after
    // the PI phase is set by the ceiling: 0.05 g; a support thrust below the weight makes it faster), and the climb of
    // 1.3 m/s is stopped within a bounded rise (the stopping distance at the thrust floor plus the motor lag). A support
    // thrust far below the weight (-20 %) lands too, but hard: the ceiling is a fraction of the support thrust.
    // ホバーからも 1.3 m/s の上昇からも、支持推力が真のホバーより 2 % 低い〜3 % 高い場合（較正誤差は約 1 %）に、機体は 1.2 m
    // の天井から時間切れの内に 1.5 m/s 未満で着地し（PI の後の速度は上限 0.05 g で決まる。支持推力が重さより低いと
    // 速くなる）、1.3 m/s の上昇は有界の上昇量（推力の下限での停止距離とモータの遅れ）で止まる。重さよりずっと低い
    // 支持推力（−20 %）でも着地はするが、強く当たる: 上限は支持推力の割合だから。
    const float limit_s = static_cast<float>(lim::kBlindLandingTimeoutUs) * 1e-6f;
    for (float error : {-0.02f, 0.0f, 0.02f, 0.03f}) {
        for (float v0 : {0.0f, 1.3f}) {
            const DescentResult r = descend(1.0f + error, 1.0f + error, 0.0f, 1.2f, v0);
            ASSERT_TRUE(r.landed);
            ASSERT_TRUE(r.touchdown_time_s < limit_s);
            ASSERT_TRUE(r.touchdown_speed < 1.5f);
            ASSERT_TRUE(r.peak_rise < ((v0 > 0.0f) ? 0.25f : 0.005f));
        }
    }
    ASSERT_TRUE(descend(0.80f, 0.80f, 0.0f, 1.2f, 0.0f).landed);
}

TEST(loop_anti_windup_is_one_sided_and_acts_in_the_same_cycle)
{
    // Clamped at the ceiling by a fall (velocity -3 m/s): the integral must not rise further in ANY cycle (the
    // step is undone in the cycle it would push into the clamp), but a velocity error that pulls the thrust back
    // (the craft now rises) must unwind it while the thrust is still clamped. The old rule (integration off for the
    // next cycle, both directions) rose by one step first and could not unwind while clamped.
    // 落下（速度 −3 m/s）で上限に張り付いている間: 積分は「どの周期でも」これ以上増えない（制限の奥へ押す増分は、その
    // 周期で取り消す）。一方、推力を引き戻す向きの速度誤差（機体が上昇に転じた）では、推力がまだ制限にかかったままでも
    // 積分は戻れる。旧規則（前の周期の推力で次の周期の積分を両方向とも止める）は最初に 1 周期分増えてしまい、制限に
    // かかった間は戻れなかった。
    sf::BlindDescent loop;
    loop.start(kStart, kStart, sf::BlindDescentGains{});
    const float ceiling = lim::kNoTofThrustCapRatio * kStart;
    float previous = loop.integral();
    for (int i = 0; i < 200; ++i) {
        const float thrust = loop.update(-3.0f, 0.5f, kDt);
        ASSERT_TRUE(loop.integral() <= previous + 1e-12f);                    // never rises further / 増えない
        if (i > 0) ASSERT_NEAR(thrust, ceiling, 1e-6f);
        previous = loop.integral();
    }
    // A velocity error that lowers the thrust but not yet below the ceiling (-0.37 m/s against the -0.4 m/s target:
    // the craft sinks a little too slowly): the thrust is clamped at the ceiling, and the integral must still fall.
    // 推力を下げる向きだが、まだ上限を下回るほどでない速度誤差（目標 −0.4 m/s に対し −0.37 m/s: 沈み方がやや遅い）:
    // 推力は上限に張り付いたまま、積分は下がらなければならない。
    loop.start(kStart, kStart, sf::BlindDescentGains{});
    float thrust = 0.0f;
    for (int i = 0; i < 20; ++i) thrust = loop.update(-0.37f, 0.5f, kDt);
    ASSERT_NEAR(thrust, ceiling, 1e-6f);        // still clamped / まだ制限にかかっている
    ASSERT_TRUE(loop.integral() < -1e-4f);      // and the integrator has unwound / 積分器は戻っている
}

TEST(loop_after_the_trust_time_the_thrust_is_the_constant_ceiling)
{
    // The PI is used while the velocity estimate is within kVerticalVelocityTrustS of its last reset; after that the thrust
    // is the constant ceiling, whatever the velocity says, and the integrator does not move.
    // PI を使うのは、速度推定が最後のリセットから kVerticalVelocityTrustS 以内の間。その後は速度が何であれ推力は上限の
    // 一定値で、積分器は動かない。
    sf::BlindDescent loop;
    loop.start(kStart, kStart, sf::BlindDescentGains{});
    const float ceiling = lim::kNoTofThrustCapRatio * kStart;
    for (int i = 0; i < 100; ++i) loop.update(0.0f, 0.5f, kDt);   // trusted: the PI pulls the thrust down from a hover / 信用: PI が推力を下げる
    ASSERT_TRUE(loop.thrustRatio() < 0.9f * lim::kNoTofThrustCapRatio);
    const float integral = loop.integral();
    for (float v : {-3.0f, 0.0f, 0.4f, 3.0f}) {
        const float thrust = loop.update(v, lim::kVerticalVelocityTrustS + 0.1f, kDt);
        ASSERT_NEAR(thrust, ceiling, 1e-6f);
    }
    ASSERT_NEAR(loop.integral(), integral, 1e-9f);
    // Right at the limit it is still trusted. ちょうど境界ではまだ信用する。
    loop.start(kStart, kStart, sf::BlindDescentGains{});
    ASSERT_TRUE(loop.update(0.0f, lim::kVerticalVelocityTrustS, kDt) < ceiling - 1e-4f);
}

TEST(loop_with_an_old_estimate_descends_at_the_ceiling_and_lands)
{
    // The estimate was reset long ago (a manual flight with the ToF stopped for 30 s, then the landing begins): the PI is not
    // used from the start, the thrust is the constant ceiling and the craft sinks at 0.05 g and lands from 1.2 m.
    // 速度推定のリセットが遠い昔（ToF が止まったままの手動飛行 30 s の後に着陸が始まる）: 最初から PI を使わず、推力は
    // 上限の一定値で、機体は 0.05 g で沈み、1.2 m から着地する。
    for (float bias : {-1.0f, 0.0f, 1.0f}) {
        const DescentResult r = descend(1.0f, 1.0f, bias, 1.2f, 0.0f, 40.0f, 30.0f);
        ASSERT_TRUE(r.landed);
        ASSERT_TRUE(r.peak_rise < 0.005f);
        ASSERT_TRUE(r.touchdown_speed < 1.3f);
    }
}

TEST(loop_with_a_support_thrust_above_the_margin_ends_with_the_late_fall)
{
    // A support thrust 10 % above the true hover (the calibration failed): the ceiling is above the weight and the craft
    // drifts up slowly. Nothing in the loop can know; the late fall brings the thrust to zero and the craft lands within
    // the time limit plus the fall time.
    // 支持推力が真のホバーより 10 % 高い（較正の失敗）: 上限が重さを上回り、機体はゆっくり上昇する。ループからは
    // 判別できず、後半の低下が推力をゼロにして、時間切れ + 落下時間内に着地する。
    const float limit_s = static_cast<float>(lim::kBlindLandingTimeoutUs) * 1e-6f;
    for (float h0 : {0.5f, 1.2f}) {
        const DescentResult r = descend(1.10f, 1.0f, 0.0f, h0, 0.0f);
        ASSERT_TRUE(r.landed);
        ASSERT_TRUE(r.touchdown_time_s < limit_s + 1.5f);
    }
    ASSERT_TRUE(descend(1.10f, 1.0f, 0.0f, 1.2f, 0.0f).peak_rise > 0.1f);   // it did rise: the loop alone failed here / 上昇した: ここではループ単独は失敗
}

TEST(late_fall_brings_the_thrust_to_zero_at_the_time_limit)
{
    const float limit_s = static_cast<float>(lim::kBlindLandingTimeoutUs) * 1e-6f;
    sf::BlindDescent loop;
    loop.start(kStart, kStart, sf::BlindDescentGains{});
    float thrust = 1.0f;
    for (float t = 0.0f; t < limit_s; t += kDt) thrust = loop.update(-0.4f, 0.0f, kDt);
    ASSERT_TRUE(thrust == 0.0f);                       // the thrust is ZERO at the time limit / 時間切れで推力はゼロ
}

TEST(late_fall_time_is_counted_over_one_landing)
{
    // stop() and start() (a ToF that returns and stops again) do not restart the late-fall time; beginLanding() does.
    // stop() と start()（ToF が戻ってまた止まる）では後半の低下の時間は数え直さず、beginLanding() が数え直す。
    sf::BlindDescent loop;
    loop.start(kStart, kStart, sf::BlindDescentGains{});
    for (float t = 0.0f; t < 4.0f; t += kDt) loop.update(-0.4f, 0.0f, kDt);
    loop.stop();
    loop.start(kStart, kStart, sf::BlindDescentGains{});
    ASSERT_TRUE(loop.elapsedS() > 3.9f);
    loop.beginLanding();
    ASSERT_TRUE(loop.elapsedS() == 0.0f);
}

TEST(loop_thrust_limits_and_anti_windup)
{
    sf::BlindDescent loop;
    sf::BlindDescentGains gains;
    loop.start(kStart, kStart, gains);
    // A big climb: the thrust is held at the floor, and the integrator does not wind up behind it.
    // 大きな上昇: 推力は下限に張り付き、積分器はその後ろで巻き上がらない。
    float thrust = 0.0f;
    for (int i = 0; i < 400; ++i) thrust = loop.update(3.0f, 0.5f, kDt);   // 1 s of +3 m/s / +3 m/s を 1 s
    ASSERT_NEAR(thrust, lim::kBlindDescentThrustMinFrac * kStart, 1e-6f);
    ASSERT_TRUE(std::fabs(loop.integral()) < 0.02f * kStart + 0.05f);
    // A big fall: the thrust is held at the ceiling, BELOW the support thrust (it never brakes a fall above the weight).
    // 大きな落下: 推力は上限に張り付く。上限は支持推力「未満」（重さを超える推力で落下を止めには行かない）。
    loop.start(kStart, kStart, gains);
    for (int i = 0; i < 400; ++i) thrust = loop.update(-3.0f, 0.5f, kDt);
    ASSERT_NEAR(thrust, lim::kNoTofThrustCapRatio * kStart, 1e-6f);
    ASSERT_TRUE(thrust < kStart);
    // The thrust ratio the touchdown detector reads: commanded / support, 0 when idle.
    // 接地検出器が読む推力比: 指令 ÷ 支持推力。待機中は 0。
    ASSERT_NEAR(loop.thrustRatio(), lim::kNoTofThrustCapRatio, 1e-5f);
    loop.stop();
    ASSERT_TRUE(loop.thrustRatio() == 0.0f);
    ASSERT_TRUE(!loop.active());
}

TEST(loop_gains_scale_with_the_hover_estimate_not_the_mass)
{
    // The same loop with a hover estimate 1.5x the true one still descends (the gains are fractions of
    // the estimate): the error of the estimate scales the gains, nothing more.
    // hover 推力の見積もりが真の値の 1.5 倍でも同じループは降りる（ゲインは見積もりに対する割合）:
    // 見積もりの誤差はゲインを比例して変えるだけ。
    for (float estimate : {0.7f, 1.0f, 1.5f}) {
        const DescentResult r = descend(1.0f, estimate, 0.0f, 1.2f, 0.0f);
        ASSERT_TRUE(r.landed);
        ASSERT_TRUE(r.peak_rise < 0.005f);
        ASSERT_TRUE(r.touchdown_speed > 0.5f && r.touchdown_speed < 1.5f);
    }
}

// ----------------------------------------------------------------------------
// 4. Touchdown detector / 接地検出器
// ----------------------------------------------------------------------------

static sf::BlindTouchdownInput tdInput(float accel_mps2, float thrust_ratio, float vz_up)
{
    sf::BlindTouchdownInput in{};
    in.dt_s = kDt;
    in.accel_magnitude = accel_mps2;
    in.thrust_ratio = thrust_ratio;
    in.vertical_velocity_up = vz_up;
    return in;
}
static void tdRun(sf::BlindTouchdownDetector& d, float seconds, float accel, float ratio, float vz)
{
    const int n = static_cast<int>(seconds / kDt + 0.5f);
    for (int i = 0; i < n; ++i) d.update(tdInput(accel, ratio, vz));
}
static void tdImpact(sf::BlindTouchdownDetector& d)
{
    for (int i = 0; i < lim::kBlindTouchdownSamples; ++i) d.update(tdInput(lim::kBlindTouchdownAccelMps2 * 2.0f, 0.5f, -0.4f));
}

TEST(detector_needs_both_the_impact_and_the_stillness)
{
    sf::BlindTouchdownDetector d;
    d.init(sf::BlindTouchdownConfig{});
    tdRun(d, 2.0f, kG, 0.85f, 0.0f);                   // stillness alone (thrust ratio above the floor path's 0.7, no impact seen) / 静止だけ（推力比は床の経路の 0.7 より上、衝撃なし）
    ASSERT_TRUE(!d.touchdown());
    d.reset();
    tdImpact(d);                                       // impact alone / 衝撃だけ
    ASSERT_TRUE(d.impactSeen() && !d.touchdown());
    tdRun(d, lim::kBlindTouchdownStillS * 0.5f, kG, 0.4f, 0.0f);   // not still long enough / 静止が短い
    ASSERT_TRUE(!d.touchdown());
    tdRun(d, lim::kBlindTouchdownStillS + 0.4f, kG, 0.4f, 0.0f);   // long enough (with the low-pass settling) / 十分長い
    ASSERT_TRUE(d.touchdown());
}

TEST(detector_each_stillness_condition_is_needed)
{
    const float still = lim::kBlindTouchdownStillS + 0.6f;
    {   // still moving down (a steady descent after a vibration spike) / まだ降下中
        sf::BlindTouchdownDetector d; d.init(sf::BlindTouchdownConfig{});
        tdImpact(d); tdRun(d, still, kG, 0.85f, -0.4f);   // ratio 0.85: not the floor path / 比 0.85: 床の経路ではない
        ASSERT_TRUE(!d.touchdown());
    }
    {   // the thrust could hold the craft up (ratio >= 1, which the loop never commands) / 推力が機体を支えうる（比 1 以上。ループは出さない）
        sf::BlindTouchdownDetector d; d.init(sf::BlindTouchdownConfig{});
        tdImpact(d); tdRun(d, still, kG, 1.0f, 0.0f);
        ASSERT_TRUE(!d.touchdown());
    }
    {   // the descent's own thrust (ceiling ratio 0.95) is below the support thrust: the impact path latches / 降下自身の推力（上限比 0.95）は支持推力未満: 衝撃の経路はラッチする
        sf::BlindTouchdownDetector d; d.init(sf::BlindTouchdownConfig{});
        tdImpact(d); tdRun(d, still, kG, lim::kNoTofThrustCapRatio, 0.0f);
        ASSERT_TRUE(d.touchdown() && !d.byFloorSupport());
    }
    {   // accelerating (a falling or climbing craft reads far from 1 g) / 加速中
        sf::BlindTouchdownDetector d; d.init(sf::BlindTouchdownConfig{});
        tdImpact(d); tdRun(d, still, 0.6f * kG, 0.4f, 0.0f);
        ASSERT_TRUE(!d.touchdown());
    }
}

TEST(detector_a_bounce_restarts_the_stillness)
{
    sf::BlindTouchdownDetector d;
    d.init(sf::BlindTouchdownConfig{});
    tdImpact(d);
    tdRun(d, lim::kBlindTouchdownStillS + 0.3f - 0.1f, kG, 0.4f, 0.0f);
    tdRun(d, 0.2f, 0.3f * kG, 0.4f, 0.8f);             // the craft bounces up: moving and accelerating / 急に上がる
    ASSERT_TRUE(!d.touchdown());
    ASSERT_TRUE(d.stillSeconds() == 0.0f);
    tdRun(d, lim::kBlindTouchdownStillS + 0.4f, kG, 0.4f, 0.0f);
    ASSERT_TRUE(d.touchdown());
}

TEST(detector_latch_holds_until_reset)
{
    sf::BlindTouchdownDetector d;
    d.init(sf::BlindTouchdownConfig{});
    tdImpact(d);
    tdRun(d, lim::kBlindTouchdownStillS + 0.4f, kG, 0.4f, 0.0f);
    ASSERT_TRUE(d.touchdown());
    tdRun(d, 1.0f, 0.3f * kG, 1.2f, -1.0f);            // whatever follows / その後に何が来ても
    ASSERT_TRUE(d.touchdown());
    d.reset();
    ASSERT_TRUE(!d.touchdown() && !d.impactSeen());
}

TEST(detector_floor_holding_the_craft_up_needs_no_impact)
{
    // Thrust below 0.7 of the support thrust and |accel| near 1 g for 0.5 s: only the floor can do that. No impact needed.
    // 推力が支持推力の 0.7 未満で |加速度| が 1 g の近くが 0.5 s: 床にしかできない。衝撃は要らない。
    sf::BlindTouchdownDetector d;
    d.init(sf::BlindTouchdownConfig{});
    tdRun(d, lim::kBlindFloorS * 0.5f, kG, 0.5f, 0.0f);
    ASSERT_TRUE(!d.touchdown());
    tdRun(d, lim::kBlindFloorS * 0.5f + 0.1f, kG, 0.5f, 0.0f);
    ASSERT_TRUE(d.touchdown() && d.byFloorSupport() && !d.impactSeen());
    d.reset();
    ASSERT_TRUE(!d.touchdown() && !d.byFloorSupport() && d.floorSeconds() == 0.0f);
}

TEST(detector_floor_path_never_latches_in_the_air)
{
    // In the air a thrust below 0.7 of the weight reads below 0.7 g: free fall (0 g), a descent at the thrust floor
    // (0.25 g), the PI start (0.6 g) and the support thrust 25 % above the weight at ratio 0.69 (0.86 g) are all outside
    // 1 g +- 0.1 g for the whole time. With the thrust at the ceiling (0.95) the floor path is off.
    // 空中では重さの 0.7 倍未満の推力は 0.7 g 未満を読む: 自由落下（0 g）、推力の下限での降下（0.25 g）、PI の開始（0.6 g）、
    // 重さより 25 % 高い支持推力での比 0.69（0.86 g）は、どれも常に 1 g ± 0.1 g の外。推力が上限（0.95）なら床の経路は働かない。
    for (float reading_g : {0.0f, 0.25f, 0.6f, 0.86f}) {
        sf::BlindTouchdownDetector d;
        d.init(sf::BlindTouchdownConfig{});
        tdRun(d, 5.0f, reading_g * kG, 0.5f, -0.5f);
        ASSERT_TRUE(!d.touchdown());
    }
    sf::BlindTouchdownDetector d;
    d.init(sf::BlindTouchdownConfig{});
    tdRun(d, 5.0f, kG, lim::kNoTofThrustCapRatio, -0.4f);   // a steady descent at the ceiling thrust reads 0.95 g / 上限の推力での定常降下は 0.95 g
    ASSERT_TRUE(!d.touchdown());
    ASSERT_TRUE(lim::kBlindFloorThrustFrac < lim::kNoTofThrustCapRatio);
}

TEST(detector_vibration_and_steady_descent_never_latch)
{
    // Real hover logs: |accel| up to 1.56 g (99.9th percentile 1.50 g); aggressive flight 2.28 g. The
    // impact threshold is above the hover vibration, and even a spike that reaches it is followed by a
    // steady descent (velocity -0.4 m/s, thrust ratio 1) that is not "still".
    // 実機のホバーログ: |加速度| は 1.56 g まで（99.9 パーセンタイル 1.50 g）、激しい飛行で 2.28 g。衝撃の閾値は
    // ホバーの振動より上で、それに届く突起があっても、その後は「静止」でない定常の降下（速度 −0.4 m/s、推力比 1）。
    ASSERT_TRUE(lim::kBlindTouchdownAccelG > 1.56f);
    sf::BlindTouchdownDetector d;
    d.init(sf::BlindTouchdownConfig{});
    for (int i = 0; i < 4000; ++i) {                    // 10 s of vibration around 1 g / 1 g まわりの振動 10 s
        const float g_factor = 1.0f + 0.5f * std::sin(0.9f * static_cast<float>(i));
        d.update(tdInput(g_factor * kG, 1.0f, -0.4f));
    }
    ASSERT_TRUE(!d.touchdown());
    tdImpact(d);
    tdRun(d, 3.0f, kG, 1.0f, -0.4f);
    ASSERT_TRUE(!d.touchdown());
}

// ----------------------------------------------------------------------------
// 5. Support thrust / 支持推力
// ----------------------------------------------------------------------------

// Feed the support-thrust window n seconds of a steady condition. 定常の条件を n 秒与える。
static void sfRun(sf::SupportThrustFilter& f, float seconds, float thrust, float force, float rise)
{
    const int n = static_cast<int>(seconds / kDt + 0.5f);
    for (int i = 0; i < n; ++i) f.update(kDt, thrust, force, rise);
}

TEST(window_does_not_open_while_the_craft_is_on_the_floor)
{
    // On the floor the reading is g whatever the thrust: nothing is averaged, however long the thrust ramps.
    // 床の上では推力が何であれ読みは g: 推力がどれだけ長く上がっても何も平均しない。
    sf::SupportThrustFilter f;
    f.reset();
    for (float thrust : {0.1f, 0.2f, 0.3f, 0.45f}) sfRun(f, 1.0f, thrust, kG, 0.0f);
    ASSERT_TRUE(!f.leftFloor() && !f.calibrated() && !f.hasThrust());
    const sf::SupportThrustFilter& cf = f;
    ASSERT_NEAR(sf::blindLandingStartThrust(cf, 0.40f), lim::kBlindUncalibratedFrac * 0.40f, 1e-6f);
}

TEST(window_opens_when_the_rise_or_the_excess_reading_shows_the_craft_left_the_floor)
{
    sf::SupportThrustFilter by_rise;
    by_rise.reset();
    sfRun(by_rise, 0.5f, 0.3f, kG, 0.5f * lim::kSupportLeaveRiseM);
    ASSERT_TRUE(!by_rise.leftFloor());
    sfRun(by_rise, 0.01f, 0.3f, kG, lim::kSupportLeaveRiseM + 0.01f);
    ASSERT_TRUE(by_rise.leftFloor());
    sf::SupportThrustFilter by_force;
    by_force.reset();
    sfRun(by_force, 1.0f, 0.3f, kG + 0.5f * lim::kSupportLeaveExcessMps2, 0.0f);   // half the excess: no / 半分の超過: まだ
    ASSERT_TRUE(!by_force.leftFloor());
    sfRun(by_force, 1.0f, 0.3f, kG + 2.0f * lim::kSupportLeaveExcessMps2, 0.0f);
    ASSERT_TRUE(by_force.leftFloor());
    // The healthy ToF says airborne although an offset error has made the integral go down (rise -1.9 m) and the take-off was gentle
    // (the reading never 1 m/s^2 above g): the craft has left the floor.
    // 健全な ToF が空中と言っている。オフセット誤差で積分が下へ行き（上昇量 −1.9 m）、離陸が穏やか（読みが g を 1 m/s^2 超えない）
    // でも、機体は床を離れた。
    sf::SupportThrustFilter by_tof;
    by_tof.reset();
    for (int i = 0; i < 400; ++i) by_tof.update(kDt, 0.3f, kG + 0.4f, -1.9f, false);
    ASSERT_TRUE(!by_tof.leftFloor());
    by_tof.update(kDt, 0.3f, kG + 0.4f, -1.9f, true);
    ASSERT_TRUE(by_tof.leftFloor());
}

TEST(support_in_a_climb_from_the_floor_is_the_hover_thrust_from_the_hover_samples_only)
{
    // Take-off: 0.5 s on the floor with the thrust rising above the weight (reading g), a climb at +0.3 g with the thrust 1.3x
    // the hover and the reading 1.3 g, then the hover (thrust = hover thrust, reading g). Only the hover samples are averaged:
    // the floor samples would give 1.3x too much and the climb samples (the delivered thrust is concave in the command: the
    // reading 1.2 g for the command 1.35x) 12 % too much.
    // 離陸: 床の上で 0.5 s、推力が重さを超えて上がる（読みは g）。+0.3 g の上昇（推力はホバーの 1.3 倍、読みは 1.3 g）。
    // その後ホバー（推力 = ホバー推力、読みは g）。平均に入れるのはホバーのサンプルだけ: 床のサンプルなら 1.3 倍、上昇の
    // サンプル（出る推力は指令に対し上に凸: 指令 1.35 倍で読み 1.2 g）なら 12 % 高くなる。
    sf::SupportThrustFilter f;
    f.reset();
    sfRun(f, 0.5f, 1.3f * kStart, kG, 0.0f);
    ASSERT_TRUE(!f.leftFloor());
    sfRun(f, 1.0f, 1.35f * kStart, 1.2f * kG, 0.3f);               // the climb / 上昇
    ASSERT_TRUE(f.leftFloor() && !f.hasThrust() && !f.calibrated());
    sfRun(f, 0.5f * lim::kSupportWindowS, kStart, kG, 1.0f);       // hover, window half full / ホバー、窓が半分
    ASSERT_TRUE(!f.calibrated());
    sfRun(f, 0.5f * lim::kSupportWindowS + 0.8f, kStart, kG, 1.0f);   // + the low-pass settling and the band hold / + ローパスの整定と帯の保持
    ASSERT_TRUE(f.calibrated());
    ASSERT_NEAR(sf::blindLandingStartThrust(f, 1.2f * kStart), kStart, 0.01f * kStart);
}

TEST(support_does_not_depend_on_the_hover_estimate_when_calibrated)
{
    // A steady hover reads g: the quotient is 1, whatever the hover-thrust estimate says (within the bounds).
    // 定常ホバーは g を読む: 商は 1 で、ホバー推力の見積もりが何であれ変わらない（制限の範囲内で）。
    for (float estimate_error : {-0.10f, 0.0f, 0.10f, 0.20f, 0.40f}) {
        sf::SupportThrustFilter f;
        f.reset();
        sfRun(f, 0.2f, kStart, kG, 0.5f);
        sfRun(f, lim::kSupportWindowS + 3.0f, kStart, kG, 0.5f);
        ASSERT_TRUE(f.calibrated());
        const float hover_estimate = kStart * (1.0f + estimate_error);
        ASSERT_NEAR(sf::blindLandingStartThrust(f, hover_estimate), kStart, 1e-4f);
    }
}

TEST(means_of_thrust_and_reading_advance_only_in_the_same_cycles)
{
    // The mean thrust and the mean reading are updated only in cycles where the reading is usable, both or neither:
    // 40 cycles (0.1 s) of an unusable reading (free fall / an estimator without specific force) with a very different
    // thrust change neither mean nor the averaged time. The quotient therefore stays the one of the usable cycles.
    // 平均の推力と平均の読みは、読みが使える周期だけ、両方ともか両方とも無しで更新する: 読みが使えない 40 周期（0.1 s。
    // 自由落下、または加速度計の測定値を出さない推定器）は、推力がまったく違っても、どちらの平均も平均した時間も変えない。
    // よって商は、使える周期のものであり続ける。
    sf::SupportThrustFilter f;
    f.reset();
    sfRun(f, 0.2f, kStart, kG, 0.5f);
    sfRun(f, lim::kSupportWindowS + 1.0f, kStart, kG, 0.5f);
    ASSERT_TRUE(f.calibrated());
    const float thrust = f.thrust(), force = f.upwardForce(), age = f.nearHoverSeconds(), air = f.airborneSeconds();
    sfRun(f, 40 * kDt, 2.0f * kStart, 0.0f, 0.5f);
    ASSERT_NEAR(f.thrust(), thrust, 1e-9f);
    ASSERT_NEAR(f.upwardForce(), force, 1e-9f);
    ASSERT_NEAR(f.nearHoverSeconds(), age, 1e-9f);
    ASSERT_NEAR(f.airborneSeconds(), air, 1e-9f);
    const sf::SupportThrustFilter& cf = f;
    ASSERT_NEAR(sf::blindLandingStartThrust(cf, kStart), kStart, 1e-4f);
}

TEST(support_with_an_accelerometer_offset_error_is_off_by_the_offset_over_g)
{
    // The reading is off by -0.087 m/s^2 (the mean of two real hovers): the support thrust is 0.9 % high; by +0.2 m/s^2: 2 % low.
    // 読みが −0.087 m/s^2 ずれる（実機のホバー 2 本の平均）と支持推力は 0.9 % 高く、+0.2 m/s^2 なら 2 % 低くなる。
    for (float offset : {-0.2f, -0.087f, 0.0f, 0.087f, 0.2f}) {
        sf::SupportThrustFilter f;
        f.reset();
        sfRun(f, 0.2f, kStart, kG, 0.5f);
        sfRun(f, lim::kSupportWindowS + 3.0f, kStart, kG + offset, 0.5f);
        ASSERT_NEAR(sf::blindLandingStartThrust(f, kStart), kStart * kG / (kG + offset), 1e-4f);
        ASSERT_TRUE(sf::blindThrustCeiling(sf::blindLandingStartThrust(f, kStart)) < kStart);   // below the weight for all of them / どれでも重さ未満
    }
}

TEST(support_is_bounded_against_a_bad_reading)
{
    // A reading far from g is out of the near-hover band: it is not averaged, so the support stays the lowered estimate; and
    // the clamp [0.5, 1.5] x estimate holds for an estimate far from the thrust that held the craft.
    // g から遠い読みはホバー近傍の帯の外: 平均に入れないので、支持推力は低くした見積もりのまま。見積もりが機体を支えた推力から
    // 遠いときは、見積もりの [0.5, 1.5] 倍の制限が効く。
    sf::SupportThrustFilter far;
    far.reset();
    sfRun(far, 5.0f, kStart, 2.5f, 0.5f);
    ASSERT_TRUE(!far.calibrated());
    sfRun(far, 5.0f, kStart, 8.0f * kG, 0.5f);
    ASSERT_TRUE(!far.calibrated());
    sf::SupportThrustFilter f;
    f.reset();
    sfRun(f, 0.2f, kStart, kG, 0.5f);
    sfRun(f, lim::kSupportWindowS + 1.0f, kStart, kG, 0.5f);
    ASSERT_NEAR(sf::blindLandingStartThrust(f, 3.0f * kStart), lim::kBlindCalibratedMinFrac * 3.0f * kStart, 1e-5f);
    ASSERT_NEAR(sf::blindLandingStartThrust(f, 0.4f * kStart), lim::kBlindCalibratedMaxFrac * 0.4f * kStart, 1e-5f);
}

TEST(support_without_a_window_is_the_lowered_estimate)
{
    const float hover_estimate = 0.40f;
    const float cap = lim::kBlindUncalibratedFrac * hover_estimate;
    // Nothing at all (the craft never left the floor): the lowered estimate.
    // 何も無い（機体が床を離れていない）: 低くした見積もり。
    sf::SupportThrustFilter none;
    none.reset();
    ASSERT_NEAR(sf::blindLandingStartThrust(none, hover_estimate), cap, 1e-6f);
    // Left the floor, hover window still short: the early estimate is used AS IT IS, the lowered estimate does not cut it (the cut made
    // the ceiling 0.63-0.71 x the thrust that holds the craft in the real logs). It is bounded above by the same sanity bound as the calibrated value.
    // 床を離れたがホバーの窓がまだ短い: 早い推定を「そのまま」使い、低くした見積もりでは切らない（切ると、実機ログで上限が機体を支える
    // 推力の 0.63〜0.71 倍になった）。上は較正した値と同じ、異常値に備えた上限で抑える。
    sf::SupportThrustFilter young;
    young.reset();
    sfRun(young, 0.5f * lim::kSupportWindowS, 0.60f, 0.9f * kG, 0.2f);   // reading far below the thrust: the early estimate is above the sanity bound / 読みが推力よりずっと低い: 早い推定は異常値の上限より上
    ASSERT_TRUE(young.leftFloor() && !young.calibrated());
    ASSERT_NEAR(sf::blindLandingStartThrust(young, hover_estimate), lim::kBlindCalibratedMaxFrac * hover_estimate, 1e-6f);
    sf::SupportThrustFilter modest;
    modest.reset();
    sfRun(modest, 0.5f * lim::kSupportWindowS, 0.50f, 1.0f * kG, 0.2f);   // quotient 0.50 N: above the cap 0.30 N, below the sanity bound 0.60 N / 商 0.50 N: 上限 0.30 N より上、異常値の上限 0.60 N より下
    ASSERT_TRUE(modest.leftFloor() && !modest.calibrated());
    ASSERT_NEAR(sf::blindLandingStartThrust(modest, hover_estimate), lim::kSupportClimbDerate * 0.50f, 1e-4f);
    ASSERT_TRUE(sf::blindLandingStartThrust(modest, hover_estimate) > cap);
    // The early estimate from a climb (quotient x kSupportClimbDerate): a take-off at +0.2 g with the thrust 1.2x the hover
    // and the delivered thrust concave (the reading only 1.1 g): the quotient is 9 % high, the derated value 0.92 x 1.09 = 1.00 of the hover.
    // 上昇からの早い推定（商 × kSupportClimbDerate）: +0.2 g の離陸で推力がホバーの 1.2 倍、出る推力が上に凸（読みは 1.1 g だけ）:
    // 商は 9 % 高く、補正後の値はホバーの 0.92 × 1.09 = 1.00 倍。
    sf::SupportThrustFilter climb;
    climb.reset();
    sfRun(climb, 0.4f, 1.2f * kStart, 1.1f * kG, 0.3f);
    ASSERT_TRUE(climb.leftFloor() && !climb.calibrated());
    ASSERT_NEAR(sf::blindLandingStartThrust(climb, 1.5f * kStart), lim::kSupportClimbDerate * 1.2f * kStart / 1.1f, 2e-3f);
    ASSERT_TRUE(sf::blindThrustCeiling(sf::blindLandingStartThrust(climb, 1.5f * kStart)) < kStart);   // the ceiling (0.95 x support) stays below the weight / 上限（支持推力の 0.95 倍）は重さを下回る
    sf::SupportThrustFilter too_soon;   // fewer than kSupportClimbMinS of airborne samples: the lowered estimate / 空中のサンプルが kSupportClimbMinS 未満: 低くした見積もり
    too_soon.reset();
    sfRun(too_soon, 0.5f * lim::kSupportClimbMinS, 1.2f * kStart, 1.1f * kG, 0.3f);
    ASSERT_NEAR(sf::blindLandingStartThrust(too_soon, 1.5f * kStart), lim::kBlindUncalibratedFrac * 1.5f * kStart, 1e-6f);
    // An implausible reading (an estimator without specific_force: 0): no window, the lowered estimate.
    // 妥当でない読み（specific_force を埋めない推定器: 0）: 窓が無く、低くした見積もり。
    sf::SupportThrustFilter blind;
    blind.reset();
    sfRun(blind, 5.0f, 0.30f, 0.0f, 0.5f);
    ASSERT_TRUE(blind.leftFloor() && !blind.calibrated());
    ASSERT_NEAR(sf::blindLandingStartThrust(blind, hover_estimate), cap, 1e-6f);
}

TEST(a_thrust_ceiling_from_the_lowered_estimate_cannot_climb_unless_the_estimate_is_40_percent_high)
{
    // Without a window the ceiling is 0.95 x 0.75 x the estimate: it stays below the true hover while the estimate is
    // below 1 / (0.75 x 0.95) = 1.40 x the true hover (SILS hover.thrust_corr 1.2: 1.0-1.45).
    // 窓が無いとき、上限は見積もりの 0.95 × 0.75 倍: 見積もりが真のホバーの 1 / (0.75 × 0.95) = 1.40 倍未満の間は
    // 真のホバーを下回る（SILS の hover.thrust_corr 1.2 では 1.29 倍）。
    sf::SupportThrustFilter none;
    none.reset();
    for (float estimate_error : {-0.2f, 0.0f, 0.10f, 0.29f, 0.38f}) {
        const float estimate = kStart * (1.0f + estimate_error);
        ASSERT_TRUE(sf::blindThrustCeiling(sf::blindLandingStartThrust(none, estimate)) < kStart);
    }
    const float high = kStart * 1.42f;
    ASSERT_TRUE(sf::blindThrustCeiling(sf::blindLandingStartThrust(none, high)) > kStart);
}

TEST(an_early_estimate_gives_a_ceiling_between_84_percent_and_the_weight)
{
    // The lower bound is 0.84 x the weight: the target was lowered from 0.85 as a decision, to make the re-take-off from the floor zero (kSupportClimbDerate 0.91, flight_limits.hpp).
    // 下限は重さの 0.84 倍: 目標は、床からの再離陸を 0 にするための判断として 0.85 から下げた（kSupportClimbDerate 0.91、flight_limits.hpp）。
    // Before the calibration, the ceiling is 0.95 x kSupportClimbDerate x (the quotient thrust x g / reading). A climb whose delivered
    // thrust is linear in the command (the real logs: quotient 0.98-1.03 x the weight) gives at least 0.84 x the weight; a climb whose delivered
    // thrust is concave (SILS: the reading only 1.1 g for 1.2 x the thrust, quotient 1.09 x the weight) stays below the weight. The hover estimate
    // (here 45 % too high) does not matter: the lowered estimate no longer cuts the early estimate.
    // 較正の前の上限は 0.95 × kSupportClimbDerate × （商 = 推力 × g ÷ 読み）。出る推力が指令に比例する上昇（実機ログ: 商は重さの 0.98〜1.03 倍）では
    // 重さの 0.84 倍以上になり、出る推力が上に凸の上昇（SILS: 推力 1.2 倍で読みは 1.1 g だけ、商は重さの 1.09 倍）でも重さを下回る。
    // ホバー推力の見積もり（ここでは 45 % 高い）は関係しない: 低くした見積もりはもう早い推定を切らない。
    const float high_estimate = 1.45f * kStart;
    for (float quotient : {0.98f, 1.0f, 1.03f}) {
        sf::SupportThrustFilter linear;
        linear.reset();
        sfRun(linear, 0.4f, 1.2f * kStart * quotient, 1.2f * kG, 0.3f);   // reading 1.2 g for a thrust 1.2 x quotient x weight / 推力が重さの 1.2 × 商 倍のとき読みは 1.2 g
        ASSERT_TRUE(linear.leftFloor() && !linear.calibrated());
        const float ceiling = sf::blindThrustCeiling(sf::blindLandingStartThrust(linear, high_estimate));
        ASSERT_TRUE(ceiling >= 0.84f * kStart && ceiling < kStart);
    }
    sf::SupportThrustFilter concave;
    concave.reset();
    sfRun(concave, 0.4f, 1.2f * kStart, 1.1f * kG, 0.3f);
    const float concave_ceiling = sf::blindThrustCeiling(sf::blindLandingStartThrust(concave, high_estimate));
    ASSERT_TRUE(concave_ceiling >= 0.84f * kStart && concave_ceiling < kStart);
}

TEST(reading_limit_is_below_free_fall_and_hover)
{
    ASSERT_TRUE(sf::upwardSpecificForceValid(kG));
    ASSERT_TRUE(!sf::upwardSpecificForceValid(0.0f));
    ASSERT_TRUE(lim::kBlindMinUpwardSpecificForce < kG);
    ASSERT_TRUE(lim::kBlindCalibratedMinFrac < 1.0f && lim::kBlindCalibratedMaxFrac > 1.0f);
}

// ----------------------------------------------------------------------------
// 6. Upward specific force / 上向きの加速度計の測定値
// ----------------------------------------------------------------------------

TEST(upward_specific_force_level_and_tilted)
{
    const float level[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float rest[3]  = {0.0f, 0.0f, -kG};      // FRD, z down: a craft at rest reads -g on z / 静止は z に -g
    ASSERT_NEAR(sf::upwardSpecificForce(level, rest), kG, 1e-5f);
    const float climbing[3] = {0.0f, 0.0f, -1.5f * kG};
    ASSERT_NEAR(sf::upwardSpecificForce(level, climbing), 1.5f * kG, 1e-5f);
    // Rolled by 60 deg about x: the body z axis is tilted, the vertical component is cos(60 deg) of
    // the body-z reading (thrust 2g along the body axis -> g vertical).
    // x 軸まわりに 60 度ロール: 機体 z 軸が傾き、鉛直成分は機体 z の読みの cos(60 度)（機体軸に沿う推力 2g → 鉛直 g）。
    const float half = 0.5f * (60.0f * 3.14159265f / 180.0f);
    const float rolled[4] = {std::cos(half), std::sin(half), 0.0f, 0.0f};
    const float along_body[3] = {0.0f, 0.0f, -2.0f * kG};
    ASSERT_NEAR(sf::upwardSpecificForce(rolled, along_body), kG, 1e-3f);
}

TEST(upward_specific_force_inverted_and_zero)
{
    const float inverted[4] = {0.0f, 1.0f, 0.0f, 0.0f};     // rolled by 180 deg / 180 度ロール
    const float body[3] = {0.0f, 0.0f, -kG};
    ASSERT_TRUE(sf::upwardSpecificForce(inverted, body) < 0.0f);   // thrust points down / 推力は下向き
    const float level[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    ASSERT_TRUE(sf::upwardSpecificForce(level, zero) == 0.0f);     // an estimator without specific_force / 非対応の推定器
}

// ----------------------------------------------------------------------------
// 7. Closed-loop landing (ToF alive): base thrust bound, rise brake
//    閉ループ着陸（ToF 生存）: 基準推力の上限、上昇ブレーキ
// ----------------------------------------------------------------------------

TEST(landing_base_thrust_is_the_estimate_up_to_1_08_times_the_support_reference)
{
    // The bound is 1.08 x the support reference (kLandingBaseOverSupportMax); an estimate below it is not touched. The values are written out, not recomputed.
    // 上限は支持推力の基準の 1.08 倍（kLandingBaseOverSupportMax）で、それ以下の見積もりには触れない。値は書き下したもので、同じ式の再計算ではない。
    ASSERT_NEAR(lim::kLandingBaseOverSupportMax, 1.08f, 1e-6f);
    ASSERT_NEAR(sf::landingBaseThrust(0.34f, 0.34f), 0.34f, 1e-6f);     // estimate = support: untouched / 見積もり = 支持推力: そのまま
    ASSERT_NEAR(sf::landingBaseThrust(0.36f, 0.34f), 0.36f, 1e-6f);     // +5.9 %: inside the margin (bound 0.3672) / 余裕の内側（上限 0.3672）
    ASSERT_NEAR(sf::landingBaseThrust(0.40f, 0.34f), 0.3672f, 1e-5f);   // +17.6 %: cut to 1.08 x 0.34 / 1.08 × 0.34 まで切る
    ASSERT_NEAR(sf::landingBaseThrust(0.50f, 0.30f), 0.324f, 1e-5f);    // a far too high estimate against the lowered support reference / 低くした支持推力の基準に対して、はるかに高い見積もり
    ASSERT_NEAR(sf::landingBaseThrust(0.30f, 0.34f), 0.30f, 1e-6f);     // an estimate below the support reference is not raised / 支持推力の基準より低い見積もりは上げない
}

TEST(rise_brake_passes_the_thrust_unless_the_craft_rises_and_falls_linearly_to_half)
{
    ASSERT_NEAR(lim::kLandingRiseBrakeFrac, 0.5f, 1e-6f);
    ASSERT_NEAR(lim::kLandingRiseBrakeRampMps, 0.2f, 1e-6f);
    const float support = 0.30f;
    // Climb rate <= 0: any thrust passes (a descent, a hover, the ground where the vertical state is pinned at 0).
    // 上昇率 0 以下: どの推力も通す（降下、ホバー、鉛直状態が 0 に固定された地上）。
    for (float climb : {0.0f, -0.001f, -0.5f}) {
        ASSERT_NEAR(sf::brakeRiseThrust(0.40f, climb, support), 0.40f, 1e-6f);
    }
    // Rising: the limit is (1 - 0.5 x climb / 0.2) x support, down to 0.5 x support from 0.2 m/s on (written out per cell).
    // 上昇中: 上限は (1 − 0.5 × 上昇率 / 0.2) × 支持推力で、0.2 m/s 以上では 0.5 × 支持推力（セルごとに書き下す）。
    ASSERT_NEAR(sf::brakeRiseThrust(0.40f, 0.02f, support), 0.285f, 1e-5f);   // fraction 0.95 / 割合 0.95
    ASSERT_NEAR(sf::brakeRiseThrust(0.40f, 0.10f, support), 0.225f, 1e-5f);   // fraction 0.75 / 割合 0.75
    ASSERT_NEAR(sf::brakeRiseThrust(0.40f, 0.20f, support), 0.150f, 1e-5f);   // fraction 0.5, the end of the ramp / 割合 0.5、ランプの終わり
    ASSERT_NEAR(sf::brakeRiseThrust(0.40f, 0.60f, support), 0.150f, 1e-5f);   // and it stays there / それより上でも変わらない
    // A thrust already below the limit is not raised: a hover-like thrust at a tiny climb rate (noise around 0) is only cut by the gentle start of the ramp.
    // 上限より低い推力は上げない: ごく小さい上昇率（0 の周りの雑音）の、ホバー並みの推力は、ランプの緩い立ち上がりでしか切られない。
    ASSERT_NEAR(sf::brakeRiseThrust(0.20f, 0.10f, support), 0.20f, 1e-6f);
    ASSERT_NEAR(sf::brakeRiseThrust(0.29f, 0.002f, support), 0.29f, 1e-6f);   // limit 0.2985 / 上限 0.2985
    ASSERT_NEAR(sf::brakeRiseThrust(0.40f, 0.002f, support), 0.2985f, 1e-5f);
}

TEST(rise_brake_is_continuous_in_the_climb_rate_and_never_raises_the_thrust)
{
    // For a thrust equal to the support reference (the limit is 1 x it at a climb rate of 0) there is no jump across the climb rate: the output changes by at most 0.5 x support x step / ramp per step,
    // and never rises with the climb rate. (A thrust ABOVE the support reference passes at a climb rate <= 0 by design, so it steps down to the limit when the craft starts to rise.)
    // 支持推力の基準に等しい推力（上昇率 0 で上限が 1 倍）では、上昇率をまたいで段差が無い: 出力の 1 歩での変化は高々 0.5 × 支持推力 × 歩幅 / ランプで、上昇率とともに増えない。
    // （支持推力の基準「より高い」推力は、上昇率 0 以下では設計どおり通るので、機体が上がり始めると上限まで下がる。）
    const float support = 0.30f;
    float last = sf::brakeRiseThrust(support, 0.0f, support);
    ASSERT_NEAR(last, support, 1e-6f);
    for (int i = 1; i <= 100; ++i) {
        const float climb = 0.005f * static_cast<float>(i);
        const float out = sf::brakeRiseThrust(support, climb, support);
        ASSERT_TRUE(out <= support + 1e-6f);
        ASSERT_TRUE(last - out <= 0.5f * support * 0.005f / 0.2f + 1e-5f);   // at most 0.00375 N per 5 mm/s / 5 mm/s あたり高々 0.00375 N
        ASSERT_TRUE(out <= last + 1e-6f);                                       // never rises with the climb rate / 上昇率とともに増えない
        last = out;
    }
    ASSERT_NEAR(last, 0.5f * support, 1e-5f);
}

TEST(ramped_limit_is_one_at_the_start_falls_linearly_and_stays_at_the_end_fraction)
{
    // The shared shape of the ceiling brake, the rise brake and the settle ceiling: fraction 1 at progress 0, linear to end_fraction at ramp_width, constant above.
    // 天井ブレーキ・上昇ブレーキ・着地アシストの上限が共有する形: 進み 0 で割合 1、ramp_width で直線的に end_fraction まで下がり、それより上では一定。
    ASSERT_NEAR(sf::rampedLimit(0.0f, 0.2f, 0.5f, 0.30f), 0.30f, 1e-6f);
    ASSERT_NEAR(sf::rampedLimit(0.1f, 0.2f, 0.5f, 0.30f), 0.225f, 1e-6f);   // fraction 0.75 / 割合 0.75
    ASSERT_NEAR(sf::rampedLimit(0.2f, 0.2f, 0.5f, 0.30f), 0.150f, 1e-6f);   // fraction 0.5, the end of the ramp / 割合 0.5、ランプの終わり
    ASSERT_NEAR(sf::rampedLimit(5.0f, 0.2f, 0.5f, 0.30f), 0.150f, 1e-6f);   // and it stays there / それより上でも変わらない
    // The two brakes use it with their own constants: the ceiling brake falls from 1 to kAltitudeCeilingBrakeFrac over kAltitudeCeilingBrakeRampM of predicted excess.
    // 2 つのブレーキはそれぞれの定数で使う: 天井ブレーキは止まる高さの予測の超過 kAltitudeCeilingBrakeRampM で 1 から kAltitudeCeilingBrakeFrac まで下がる。
    ASSERT_NEAR(sf::rampedLimit(lim::kAltitudeCeilingBrakeRampM, lim::kAltitudeCeilingBrakeRampM, lim::kAltitudeCeilingBrakeFrac, 0.30f),
                lim::kAltitudeCeilingBrakeFrac * 0.30f, 1e-6f);
}

int main()
{
    printf("=== Blind descent (ToF silent) unit tests ===\n");

    run_late_factor_is_one_then_falls_to_zero();
    run_time_limit_leaves_the_fall_after_zero_thrust();
    run_late_fall_starts_after_a_normal_descent_from_the_ceiling();
    run_estimator_rest_is_zero_and_learns_the_offset();
    run_estimator_integrates_the_acceleration();
    run_estimator_subtracts_the_offset_learned_at_rest();
    run_estimator_does_not_follow_the_eskf_until_it_has_been_free_for_the_settle_time();
    run_estimator_follows_the_eskf_and_learns_the_offset_while_the_tof_is_healthy();
    run_estimator_silence_keeps_the_climb_the_accelerometer_saw();
    run_estimator_contact_sets_the_velocity_to_zero();
    run_estimator_ignores_an_unusable_reading();
    run_estimator_barometer_interface_acts_only_while_integrating();
    run_estimator_value_is_limited();
    run_estimator_rise_is_zero_at_rest_and_the_integral_of_the_velocity_in_the_air();
    run_estimator_align_to_the_eskf_drops_the_integral_of_the_episode();
    run_estimator_time_since_reset_counts_the_integral_only();
    run_loop_thrust_never_exceeds_the_ceiling_below_the_weight();
    run_loop_lands_gently_for_a_support_thrust_error_up_to_the_margin();
    run_loop_after_the_trust_time_the_thrust_is_the_constant_ceiling();
    run_loop_with_an_old_estimate_descends_at_the_ceiling_and_lands();
    run_loop_with_a_support_thrust_above_the_margin_ends_with_the_late_fall();
    run_late_fall_brings_the_thrust_to_zero_at_the_time_limit();
    run_late_fall_time_is_counted_over_one_landing();
    run_loop_thrust_limits_and_anti_windup();
    run_loop_anti_windup_is_one_sided_and_acts_in_the_same_cycle();
    run_loop_gains_scale_with_the_hover_estimate_not_the_mass();
    run_detector_needs_both_the_impact_and_the_stillness();
    run_detector_each_stillness_condition_is_needed();
    run_detector_a_bounce_restarts_the_stillness();
    run_detector_latch_holds_until_reset();
    run_detector_floor_holding_the_craft_up_needs_no_impact();
    run_detector_floor_path_never_latches_in_the_air();
    run_detector_vibration_and_steady_descent_never_latch();
    run_window_does_not_open_while_the_craft_is_on_the_floor();
    run_window_opens_when_the_rise_or_the_excess_reading_shows_the_craft_left_the_floor();
    run_support_in_a_climb_from_the_floor_is_the_hover_thrust_from_the_hover_samples_only();
    run_support_does_not_depend_on_the_hover_estimate_when_calibrated();
    run_means_of_thrust_and_reading_advance_only_in_the_same_cycles();
    run_support_with_an_accelerometer_offset_error_is_off_by_the_offset_over_g();
    run_support_is_bounded_against_a_bad_reading();
    run_support_without_a_window_is_the_lowered_estimate();
    run_a_thrust_ceiling_from_the_lowered_estimate_cannot_climb_unless_the_estimate_is_40_percent_high();
    run_an_early_estimate_gives_a_ceiling_between_84_percent_and_the_weight();
    run_reading_limit_is_below_free_fall_and_hover();
    run_upward_specific_force_level_and_tilted();
    run_upward_specific_force_inverted_and_zero();
    run_landing_base_thrust_is_the_estimate_up_to_1_08_times_the_support_reference();
    run_rise_brake_passes_the_thrust_unless_the_craft_rises_and_falls_linearly_to_half();
    run_rise_brake_is_continuous_in_the_climb_rate_and_never_raises_the_thrust();
    run_ramped_limit_is_one_at_the_start_falls_linearly_and_stays_at_the_end_fraction();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
