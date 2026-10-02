/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_flip_sequencer.cpp
 * @brief Deterministic unit tests for FlipSequencer
 *        FlipSequencer の決定論ユニットテスト
 *
 * Runs on host PC against the UNMODIFIED flip_sequencer.cpp/.hpp — no
 * ESP-IDF, no FreeRTOS, no param system (FlipSequencer never touches any of
 * them, by design — see flip_sequencer.hpp). Its own small self-contained
 * TEST/ASSERT framework and main(), mirroring test_state_manager.cpp,
 * because it is built as its OWN host binary rather than folded into
 * test_main.cpp's single TEST-registration main() (see the Makefile).
 *
 * ホストPCで無改変の flip_sequencer.cpp/.hpp に対して実行する — ESP-IDF・
 * FreeRTOS・paramシステムいずれも不要（FlipSequencer は設計上どれにも
 * 触れない — flip_sequencer.hpp 参照）。test_state_manager.cpp と同じ、
 * 自己完結した小さな TEST/ASSERT フレームワークと main() を持つ — これは
 * test_main.cpp の単一 TEST 登録 main() に混ぜ込まず、「自身の」host
 * バイナリとしてビルドするため（Makefile 参照）。
 *
 * @design docs/plans/flip-maneuver-plan.md §3.2/§3.5 — phase/abort coverage [OK]
 */

#include <cmath>
#include <cstdio>

#include "flip_sequencer.hpp"

using sf::FlipBlockReason;
using sf::FlipDirection;
using sf::FlipResult;
using sf::FlipSequencer;

// =============================================================================
// Minimal test framework (mirrors test_main.cpp / test_state_manager.cpp)
// 最小テストフレームワーク（test_main.cpp / test_state_manager.cpp と同じ）
// =============================================================================

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-40s ", #name); \
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
static constexpr float kSimDt    = 1.0f / 400.0f;   // matches ControlTask's 400Hz rate / ControlTaskの400Hzと一致

/// A "steady, safe hover" Input — every ready() gate passes. Tests that want
/// to exercise one specific gate start from this and violate only that field.
/// 「定常で安全なホバー」入力 — ready() の全判定を通過する。特定の判定だけを
/// 確認するテストは、これを起点にその1フィールドだけを崩す。
static FlipSequencer::Input nominalInput()
{
    FlipSequencer::Input in{};
    in.quat[0] = 1.0f;   // identity quaternion — level / 恒等クォータニオン=水平
    in.height_m = 2.0f;
    in.vertical_velocity_up_mps = 0.0f;
    in.horizontal_speed_mps = 0.0f;
    in.battery_v = 3.9f;
    in.estimator_ok = true;
    in.tof_valid = true;
    in.dt = kSimDt;
    return in;
}

/// Result of one full simulateFlip() run.
/// simulateFlip() 1回分の結果。
struct FlipSimResult {
    bool        done = false;
    float       elapsed_s = 0.0f;
    float       final_angle_deg = 0.0f;
    float       brake_entry_angle_deg = -1.0f;   // -1: Brake was never entered / -1: Brakeに一度も入らなかった
    FlipResult  result = FlipResult::None;
};

/// Drive a FlipSequencer to completion (or a safety timeout) against a
/// synthetic first-order-lag plant on the rotation axis: gyro chases the
/// sequencer's rate_sp with time constant config.motor_lag_ms (or stays
/// frozen at 0 if `plant_follows` is false, simulating a stalled rotation —
/// test_flip_spin_timeout below).
///
/// FlipSequencer を完了（または安全タイムアウト）まで駆動する。回転軸に
/// 合成一次遅れプラントを想定: ジャイロは列生成器の rate_sp を時定数
/// config.motor_lag_ms で追う（`plant_follows` が false なら 0 に凍結 —
/// 回転が進まない状況を模す。下の test_flip_spin_timeout 参照）。
static FlipSimResult simulateFlip(FlipSequencer::Config cfg, FlipDirection dir,
                                  bool plant_follows, float vz_up_mps,
                                  float max_sim_s)
{
    FlipSequencer seq;
    seq.config = cfg;
    seq.start(dir, 0.0f, 1.5f);

    const int axis = (dir == FlipDirection::Right || dir == FlipDirection::Left) ? 0 : 1;
    const float tau_s = cfg.motor_lag_ms * 0.001f;
    const float alpha = 1.0f - expf(-kSimDt / tau_s);
    float gyro[3] = {0.0f, 0.0f, 0.0f};

    FlipSimResult out;
    FlipSequencer::Phase prev_phase = seq.phase();

    while (out.elapsed_s < max_sim_s) {
        FlipSequencer::Input in = nominalInput();
        in.gyro[0] = gyro[0];
        in.gyro[1] = gyro[1];
        in.gyro[2] = gyro[2];
        in.vertical_velocity_up_mps = vz_up_mps;

        const FlipSequencer::Output flip_out = seq.update(in);

        if (seq.phase() == FlipSequencer::Phase::Brake &&
            prev_phase != FlipSequencer::Phase::Brake) {
            out.brake_entry_angle_deg = seq.rotationAngleDeg();
        }
        prev_phase = seq.phase();

        if (plant_follows) {
            const float target = flip_out.attitude_loop ? 0.0f : flip_out.rate_sp[axis];
            gyro[axis] += alpha * (target - gyro[axis]);
        }

        out.elapsed_s += kSimDt;
        out.final_angle_deg = seq.rotationAngleDeg();
        if (seq.done(out.result)) {
            out.done = true;
            break;
        }
    }
    return out;
}

// =============================================================================
// (1) Full maneuver against a synthetic plant: Boost->Spin->Brake->Recover
//     ->Done, landing angle 340-380 deg, under 1.0 s.
// (1) 合成プラントに対する一連の遷移: Boost→Spin→Brake→Recover→Done、
//     着地角 340〜380°、1.0s 未満。
// =============================================================================
TEST(flip_full_sequence_reaches_done)
{
    FlipSequencer::Config cfg;   // plan §5.3 defaults / plan §5.3既定
    const FlipSimResult r = simulateFlip(cfg, FlipDirection::Right,
                                         /*plant_follows=*/true,
                                         /*vz_up_mps=*/0.5f, /*max_sim_s=*/2.0f);
    std::printf("         full sequence: done=%d result=%d elapsed=%.3f s angle=%.1f deg\n",
                r.done ? 1 : 0, static_cast<int>(r.result),
                static_cast<double>(r.elapsed_s), static_cast<double>(r.final_angle_deg));
    ASSERT_TRUE(r.done);
    ASSERT_TRUE(r.result == FlipResult::Ok);
    ASSERT_TRUE(r.elapsed_s < 1.0f);
    ASSERT_TRUE(r.final_angle_deg >= 340.0f && r.final_angle_deg <= 380.0f);
}

// =============================================================================
// (2) phi_brake is computed from the MEASURED rate every cycle: dropping
//     rate_roll_dps (the flip peak) to 1200 must brake LATER — at a LARGER
//     rotation angle — than the 1500 default (flip-maneuver-plan.md §3.2/§7).
// (2) phi_brake は毎周期「計測」レートから計算される: rate_roll_dps
//     （フリップピーク）を1200へ下げると、1500の既定より「遅く」
//     （より大きな回転角で）減速に入る（plan §3.2/§7）。
// =============================================================================
TEST(flip_lower_peak_rate_brakes_at_larger_angle)
{
    FlipSequencer::Config cfg_default;   // rate_roll_dps = 1500
    FlipSequencer::Config cfg_slow;
    cfg_slow.rate_roll_dps = 1200.0f;

    const FlipSimResult fast = simulateFlip(cfg_default, FlipDirection::Right,
                                            true, 0.5f, 2.0f);
    const FlipSimResult slow = simulateFlip(cfg_slow, FlipDirection::Right,
                                            true, 0.5f, 2.0f);

    ASSERT_TRUE(fast.brake_entry_angle_deg > 0.0f);   // Brake was reached / Brakeに到達した
    ASSERT_TRUE(slow.brake_entry_angle_deg > 0.0f);
    ASSERT_TRUE(slow.brake_entry_angle_deg > fast.brake_entry_angle_deg);
}

// =============================================================================
// (3) A rotation that never picks up (stalled plant, gyro stuck at 0) aborts
//     via spin_timeout_ms with AbortedSpinTimeout, going STRAIGHT to Recover
//     (Brake is never entered — nothing to brake).
// (3) 回転が進まない（プラント停止、ジャイロ0固定）と spin_timeout_ms で
//     AbortedSpinTimeout として打ち切られ、Brake を経ずに直接 Recover へ進む
//     （減速すべき回転がない）。
// =============================================================================
TEST(flip_spin_timeout_skips_brake)
{
    FlipSequencer::Config cfg;
    const FlipSimResult r = simulateFlip(cfg, FlipDirection::Right,
                                         /*plant_follows=*/false,
                                         /*vz_up_mps=*/0.5f, /*max_sim_s=*/3.0f);
    ASSERT_TRUE(r.done);
    ASSERT_TRUE(r.result == FlipResult::AbortedSpinTimeout);
    ASSERT_TRUE(r.brake_entry_angle_deg < 0.0f);   // Brake never entered / Brakeに一度も入らなかった
}

// =============================================================================
// (4) A measured rate over gyro_abort_dps for kGyroAbortConsecutiveSamples
//     cycles in a row cuts Spin short and goes straight to Brake (plan §3.5);
//     a single sample over the limit, or a rate below it (the normal 1909 dps
//     hardware overshoot against the 1950 dps default), does NOT.
// (4) gyro_abort_dps を超える計測レートが kGyroAbortConsecutiveSamples 周期
//     連続したら Spin を打ち切り、即座に Brake へ遷移する（plan §3.5）。
//     上限超えが単発、または上限未満（既定 1950 dps に対する実機の通常の
//     オーバーシュート 1909 dps）では作動「しない」。
// =============================================================================
static constexpr int kAbortSamples = 3;   // FlipSequencer::kGyroAbortConsecutiveSamples (private) / 同左（private）

static void advanceToSpin(FlipSequencer& seq, FlipSequencer::Input& in)
{
    // Boost lasts 150 ms by default; 100 cycles at 400 Hz = 250 ms.
    // Boost は既定 150 ms、400 Hz で 100 周期 = 250 ms。
    for (int i = 0; i < 100; ++i) seq.update(in);
}

TEST(flip_gyro_abort_triggers_brake)
{
    FlipSequencer seq;
    seq.start(FlipDirection::Right, 0.0f, 1.5f);
    FlipSequencer::Input in = nominalInput();
    advanceToSpin(seq, in);
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Spin);

    in.gyro[0] = (seq.config.gyro_abort_dps + 20.0f) * kDegToRad;
    for (int i = 0; i < kAbortSamples - 1; ++i) {
        seq.update(in);
        ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Spin);   // not yet / まだ
    }
    seq.update(in);
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Brake);

    // done()'s `result` out-param is written EVERY call, even before
    // Phase::Done is reached (see flip_sequencer.hpp's doc on done()) — so
    // the abort reason is already visible here.
    // done() の `result` 出力引数は Phase::Done 到達前でも毎回書き込まれる
    // （flip_sequencer.hpp の done() ドキュメント参照）— 打ち切り理由は
    // この時点で既に見える。
    FlipResult result = FlipResult::None;
    seq.done(result);
    ASSERT_TRUE(result == FlipResult::AbortedGyroLimit);
}

TEST(flip_gyro_abort_ignores_single_sample_and_normal_overshoot)
{
    FlipSequencer seq;
    seq.start(FlipDirection::Right, 0.0f, 1.5f);
    FlipSequencer::Input in = nominalInput();
    advanceToSpin(seq, in);

    // Hardware overshoot (1909 dps) must not abort, however long it lasts.
    // 実機のオーバーシュート（1909 dps）は、何周期続いても打ち切らない。
    in.gyro[0] = 1909.0f * kDegToRad;
    for (int i = 0; i < 20; ++i) seq.update(in);
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Spin);

    // Over the limit for fewer than N cycles, interleaved with a low sample.
    // 上限超えが N 周期未満、間に低いサンプルを挟む。
    for (int round = 0; round < 5; ++round) {
        in.gyro[0] = (seq.config.gyro_abort_dps + 30.0f) * kDegToRad;
        for (int i = 0; i < kAbortSamples - 1; ++i) seq.update(in);
        in.gyro[0] = 1000.0f * kDegToRad;
        seq.update(in);
    }
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Spin);
}

// =============================================================================
// (4b) Brake ALWAYS terminates (INV-2). Hardware 2026-10-02: a gyro abort at
//      phi = 79 deg left the rotation stopped at ~207 deg; the old exit needed
//      phi >= 290 deg (or 350 deg) and Brake never ended.
// (4b) Brake は「必ず」終了する（INV-2）。実機 2026-10-02: phi = 79° のジャイロ
//      打ち切りで回転が約 207° で止まり、旧い出口は phi >= 290°（または 350°）を
//      要求したため Brake が終わらなかった。
// =============================================================================
static void enterBrakeByGyroAbort(FlipSequencer& seq, FlipSequencer::Input& in)
{
    seq.start(FlipDirection::Right, 0.0f, 1.5f);
    advanceToSpin(seq, in);
    in.gyro[0] = (seq.config.gyro_abort_dps + 20.0f) * kDegToRad;
    for (int i = 0; i < kAbortSamples; ++i) seq.update(in);
}

TEST(flip_brake_hands_off_when_rotation_stopped_at_any_angle)
{
    FlipSequencer seq;
    FlipSequencer::Input in = nominalInput();
    enterBrakeByGyroAbort(seq, in);
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Brake);
    ASSERT_TRUE(seq.rotationAngleDeg() < 60.0f);   // aborted early / 早期に打ち切り

    in.gyro[0] = 100.0f * kDegToRad;   // stopped: below handoff_rate_dps / 停止
    seq.update(in);
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Recover);
}

TEST(flip_brake_times_out_when_rate_never_decays)
{
    FlipSequencer seq;
    FlipSequencer::Input in = nominalInput();
    enterBrakeByGyroAbort(seq, in);
    in.gyro[0] = 1000.0f * kDegToRad;   // keeps rotating, but phi stays < handoff_force_deg / 回り続ける
    in.dt = kSimDt;

    // Keep the measured rate constant but small enough in angle: rotation angle
    // accumulates, so use a slow rate that cannot reach 350 deg inside the timeout.
    // 計測レートは一定。回転角が積算されるので、タイムアウト内に 350° へ届かない遅いレートにする。
    in.gyro[0] = 400.0f * kDegToRad;   // 400 dps * 0.3 s = 120 deg < 350 deg
    int cycles = 0;
    while (seq.phase() == FlipSequencer::Phase::Brake && cycles < 400) {
        seq.update(in);
        ++cycles;
    }
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Recover);
    const float timeout_cycles = seq.config.brake_timeout_ms * 0.001f / kSimDt;
    ASSERT_TRUE(static_cast<float>(cycles) >= timeout_cycles - 1.0f);
    ASSERT_TRUE(static_cast<float>(cycles) <= timeout_cycles + 2.0f);
    // Brake entered by a gyro abort: that stays the reported cause.
    // ジャイロ打ち切りで入った Brake: 報告される原因はそのまま。
    FlipResult result = FlipResult::None;
    seq.done(result);
    ASSERT_TRUE(result == FlipResult::AbortedGyroLimit);
}

TEST(flip_brake_timeout_is_reported_when_brake_was_entered_normally)
{
    // A normal flip whose Brake cannot settle in time: slow brake ramp, rate gate
    // below anything reachable, angle backstop disabled -> only the timeout can end it.
    // 通常の宙返りで Brake が時間内に収束しない場合: 遅い減速ランプ、到達できない
    // レート条件、角度の安全弁を無効化 → 終わらせられるのはタイムアウトだけ。
    FlipSequencer::Config cfg;
    cfg.brake_ramp_rps2 = 250.0f;
    cfg.handoff_rate_dps = 50.0f;
    cfg.handoff_force_deg = 360.0f;
    cfg.brake_timeout_ms = 50.0f;
    const FlipSimResult r = simulateFlip(cfg, FlipDirection::Right, true, 0.5f, 3.0f);
    ASSERT_TRUE(r.done);
    ASSERT_TRUE(r.result == FlipResult::AbortedBrakeTimeout);
}

// =============================================================================
// (4c) Recover from an inverted attitude: it must not be released to the normal
//      law while tilted (even when vz >= 0), the thrust must not push the craft
//      down while inverted, and the level error must point the short way home.
// (4c) 反転姿勢からの Recover: 傾いている間は（vz >= 0 でも）通常則へ放さない、
//      反転中の推力は機体を押し下げない、水平誤差は近い側の回転を指す。
// =============================================================================
/// Quaternion for a rotation of `deg` about body x (roll) or y (pitch).
/// 機体 x（roll）または y（pitch）まわり `deg` 回転のクォータニオン。
static void quatAboutAxis(int axis, float deg, float q[4])
{
    const float half = 0.5f * deg * kDegToRad;
    q[0] = cosf(half);
    q[1] = (axis == 0) ? sinf(half) : 0.0f;
    q[2] = (axis == 1) ? sinf(half) : 0.0f;
    q[3] = 0.0f;
}

TEST(flip_level_error_matches_euler_for_small_tilt_and_goes_the_short_way)
{
    float q[4];
    float roll = 0.0f, pitch = 0.0f;

    quatAboutAxis(0, 10.0f, q);   // roll right 10 deg / 右ロール 10°
    FlipSequencer::levelError(q, roll, pitch);
    ASSERT_NEAR(roll, 10.0f * kDegToRad, 1e-4f);
    ASSERT_NEAR(pitch, 0.0f, 1e-4f);

    quatAboutAxis(1, 10.0f, q);   // nose up 10 deg / 機首上げ 10°
    FlipSequencer::levelError(q, roll, pitch);
    ASSERT_NEAR(roll, 0.0f, 1e-4f);
    ASSERT_NEAR(pitch, 10.0f * kDegToRad, 1e-4f);

    // Past 90 deg about x the error keeps its sign and magnitude (no wrap).
    // x まわり 90° を過ぎても誤差は符号・大きさを保つ（折り返さない）。
    quatAboutAxis(0, 155.0f, q);
    FlipSequencer::levelError(q, roll, pitch);
    ASSERT_NEAR(roll, 155.0f * kDegToRad, 1e-4f);

    // Past 90 deg about y (where Euler roll/yaw jump to a 180 deg branch):
    // still a pitch error of the same size, no roll component.
    // y まわり 90° 超（オイラーの roll/yaw が 180° 側の枝へ跳ぶ所）でも、同じ大きさの
    // pitch 誤差で roll 成分は無い。
    quatAboutAxis(1, 120.0f, q);
    FlipSequencer::levelError(q, roll, pitch);
    ASSERT_NEAR(roll, 0.0f, 1e-4f);
    ASSERT_NEAR(pitch, 120.0f * kDegToRad, 1e-4f);

    // Level: zero error. / 水平: 誤差ゼロ。
    quatAboutAxis(0, 0.0f, q);
    FlipSequencer::levelError(q, roll, pitch);
    ASSERT_NEAR(roll, 0.0f, 1e-6f);
    ASSERT_NEAR(pitch, 0.0f, 1e-6f);
}

/// Bring a sequencer to Recover through an early gyro abort + stop.
/// 早期のジャイロ打ち切り＋停止で Recover まで進める。
static void enterRecover(FlipSequencer& seq, FlipSequencer::Input& in)
{
    enterBrakeByGyroAbort(seq, in);
    in.gyro[0] = 0.0f;
    seq.update(in);
}

TEST(flip_recover_waits_for_level_and_keeps_thrust_off_when_inverted)
{
    FlipSequencer seq;
    FlipSequencer::Input in = nominalInput();
    enterRecover(seq, in);
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Recover);

    // Inverted (155 deg), still "climbing" from the Boost: must stay in Recover
    // and command only the small positive floor (not thrust_spin_hi, which points down).
    // 反転（155°）、Boost の名残で上昇中: Recover に留まり、小さな正の下限だけを指令
    // （下向きになる thrust_spin_hi ではなく）。
    quatAboutAxis(0, 155.0f, in.quat);
    in.vertical_velocity_up_mps = 0.5f;
    FlipSequencer::Output out = seq.update(in);
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Recover);
    ASSERT_TRUE(out.attitude_loop);
    ASSERT_NEAR(out.thrust_n, seq.config.thrust_lo_n, 1e-6f);

    // Thrust grows with the upward component: partway at 75 deg, thrust_spin_hi
    // once within angle_a_deg (60 deg).
    // 推力は上向き成分とともに増える: 75° で途中、angle_a_deg（60°）以内で thrust_spin_hi。
    const float hi = seq.config.thrust_spin_hi_ratio * seq.config.max_thrust_n;
    quatAboutAxis(0, 75.0f, in.quat);
    out = seq.update(in);
    ASSERT_TRUE(out.thrust_n > seq.config.thrust_lo_n);
    ASSERT_TRUE(out.thrust_n < hi);
    quatAboutAxis(0, 45.0f, in.quat);
    out = seq.update(in);
    ASSERT_NEAR(out.thrust_n, hi, 1e-6f);
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Recover);

    // Level and climbing: Done.
    // 水平かつ上昇中: Done。
    quatAboutAxis(0, 0.0f, in.quat);
    seq.update(in);
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Done);
    FlipResult result = FlipResult::None;
    seq.done(result);
    ASSERT_TRUE(result == FlipResult::AbortedGyroLimit);   // abort reason preserved / 打ち切り理由は保持
}

TEST(flip_recover_times_out_while_still_tilted)
{
    FlipSequencer seq;
    FlipSequencer::Input in = nominalInput();
    enterRecover(seq, in);
    quatAboutAxis(0, 155.0f, in.quat);
    in.vertical_velocity_up_mps = 0.5f;

    int cycles = 0;
    while (seq.phase() == FlipSequencer::Phase::Recover && cycles < 1000) {
        seq.update(in);
        ++cycles;
    }
    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Done);   // INV-2: finite / 有限時間
    // First cause wins: the gyro abort that started this, not its Recover timeout.
    // 最初の原因を優先: これを始めたジャイロ打ち切りであり、その Recover タイムアウトではない。
    FlipResult result = FlipResult::None;
    seq.done(result);
    ASSERT_TRUE(result == FlipResult::AbortedGyroLimit);
}

// =============================================================================
// (5) ready() reports each execution-condition violation (flip-maneuver-
//     plan.md §3.1 C2/C3-C4/C5/C7), and accepts a fully nominal input.
// (5) ready() が各実行条件不成立（plan §3.1 C2/C3-C4/C5/C7）を報告し、
//     完全に定常な入力は受理する。
// =============================================================================
TEST(flip_ready_rejects_too_low)
{
    FlipSequencer seq;
    FlipSequencer::Input in = nominalInput();
    in.height_m = 0.5f;   // below default min_height_m = 1.0m
    FlipBlockReason reason = FlipBlockReason::None;
    ASSERT_TRUE(!seq.ready(in, reason));
    ASSERT_TRUE(reason == FlipBlockReason::TooLow);
}

TEST(flip_ready_rejects_not_steady)
{
    FlipSequencer seq;
    FlipSequencer::Input in = nominalInput();
    in.horizontal_speed_mps = 5.0f;   // above default max_hvel_mps = 0.3
    FlipBlockReason reason = FlipBlockReason::None;
    ASSERT_TRUE(!seq.ready(in, reason));
    ASSERT_TRUE(reason == FlipBlockReason::NotSteady);
}

TEST(flip_ready_rejects_battery_low)
{
    FlipSequencer seq;
    FlipSequencer::Input in = nominalInput();
    in.battery_v = 3.2f;   // below default min_voltage_v = 3.6
    FlipBlockReason reason = FlipBlockReason::None;
    ASSERT_TRUE(!seq.ready(in, reason));
    ASSERT_TRUE(reason == FlipBlockReason::BatteryLow);
}

TEST(flip_ready_rejects_cooldown)
{
    FlipSequencer seq;
    seq.start(FlipDirection::Right, 0.0f, 1.5f);
    seq.reset();   // ends the (just-started) flip and arms the C7 cooldown / 開始直後のフリップを終え C7 クールダウンを起動
    FlipSequencer::Input in = nominalInput();
    FlipBlockReason reason = FlipBlockReason::None;
    ASSERT_TRUE(!seq.ready(in, reason));
    ASSERT_TRUE(reason == FlipBlockReason::Cooldown);
}

TEST(flip_ready_accepts_nominal)
{
    FlipSequencer seq;
    FlipSequencer::Input in = nominalInput();
    FlipBlockReason reason = FlipBlockReason::None;
    ASSERT_TRUE(seq.ready(in, reason));
    ASSERT_TRUE(reason == FlipBlockReason::None);
}

// =============================================================================
// (6) Axis and sign per direction (flip-maneuver-plan.md §3.2 table): Right
//     rotates +roll, Forward rotates -pitch.
// (6) 方向別の軸と符号（plan §3.2 表）: Right は +ロール、Forward は
//     -ピッチで回転する。
// =============================================================================
TEST(flip_axis_sign_right_is_positive_roll)
{
    FlipSequencer seq;
    seq.start(FlipDirection::Right, 0.0f, 1.5f);
    FlipSequencer::Input in = nominalInput();
    FlipSequencer::Output out{};
    for (int i = 0; i < 100; ++i) out = seq.update(in);   // past Boost, into Spin / Boostを過ぎSpinへ

    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Spin);
    ASSERT_TRUE(!out.attitude_loop);
    ASSERT_TRUE(out.rate_sp[0] > 0.0f);    // roll axis, positive (Right = +p) / ロール軸、正（Right=+p）
    ASSERT_TRUE(out.rate_sp[1] == 0.0f);   // pitch axis untouched / ピッチ軸は無指令
}

TEST(flip_axis_sign_forward_is_negative_pitch)
{
    FlipSequencer seq;
    seq.start(FlipDirection::Forward, 0.0f, 1.5f);
    FlipSequencer::Input in = nominalInput();
    FlipSequencer::Output out{};
    for (int i = 0; i < 100; ++i) out = seq.update(in);   // past Boost, into Spin / Boostを過ぎSpinへ

    ASSERT_TRUE(seq.phase() == FlipSequencer::Phase::Spin);
    ASSERT_TRUE(!out.attitude_loop);
    ASSERT_TRUE(out.rate_sp[1] < 0.0f);    // pitch axis, negative (Forward = -q) / ピッチ軸、負（Forward=-q）
    ASSERT_TRUE(out.rate_sp[0] == 0.0f);   // roll axis untouched / ロール軸は無指令
}

// =============================================================================
// Main — registers and runs every TEST() above.
// メイン — 上記の全 TEST() を登録・実行する。
// =============================================================================
int main()
{
    printf("=== FlipSequencer unit tests ===\n");

    run_flip_full_sequence_reaches_done();
    run_flip_lower_peak_rate_brakes_at_larger_angle();
    run_flip_spin_timeout_skips_brake();
    run_flip_gyro_abort_triggers_brake();
    run_flip_gyro_abort_ignores_single_sample_and_normal_overshoot();
    run_flip_brake_hands_off_when_rotation_stopped_at_any_angle();
    run_flip_brake_times_out_when_rate_never_decays();
    run_flip_brake_timeout_is_reported_when_brake_was_entered_normally();
    run_flip_level_error_matches_euler_for_small_tilt_and_goes_the_short_way();
    run_flip_recover_waits_for_level_and_keeps_thrust_off_when_inverted();
    run_flip_recover_times_out_while_still_tilted();
    run_flip_ready_rejects_too_low();
    run_flip_ready_rejects_not_steady();
    run_flip_ready_rejects_battery_low();
    run_flip_ready_rejects_cooldown();
    run_flip_ready_accepts_nominal();
    run_flip_axis_sign_right_is_positive_roll();
    run_flip_axis_sign_forward_is_negative_pitch();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
