/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_tof_health.cpp
 * @brief Host tests for the ToF re-acquisition judgement (2nd independent review, D1-D6)
 *        ToF 取り込み直しの判定のホスト試験（2 回目の独立レビュー、D1〜D6）
 *
 * Pure pieces, no ESP-IDF / FreeRTOS:
 *  1. sf::ToFHealthTracker — "the estimator has re-acquired the ToF" (N adopted samples with a small
 *     innovation AND the ToF alive for kTofAliveMinUs), the stuck-invalid verdict while flying (D4),
 *     the one-shot re-acquisition request (D2), losses incl. silences beyond 36 / 72 minutes.
 *  2. sf::judgeToFSilenceReason — every cell of the verdict Failsafe publishes.
 *  3. sf::FailsafeStatusGuard — a failsafe_status nobody refreshes reads as silent (D5).
 *  4. sf::decideToFModeRefusal / sf::toFSilenceRefusesArm — the D3 "consume the edge in flight" rule
 *     and the ARM refusal in every mode.
 * Own TEST/ASSERT framework and main(), same pattern as test_tof_silence.cpp.
 *
 * @design architecture.md INV-4 — every (state x input) cell of the table is checked  [OK]
 * @design detailed_design.md §3.1 — ToF silent rows, 注 12                            [OK]
 */

#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "data_types.hpp"
#include "failsafe_status_guard.hpp"
#include "flight_limits.hpp"
#include "flight_state.hpp"
#include "tof_health_tracker.hpp"
#include "tof_silence_policy.hpp"

using sf::FlightMode;
using sf::FlightState;
using sf::ToFHealthTracker;
using sf::ToFModeRefusal;
using sf::InvalidSampleMeaning;
using sf::ToFFlightPhase;
using sf::ToFSampleFacts;
using sf::ToFSilenceReason;

static constexpr uint32_t kSamplePeriodUs = 33333;   // ~30 Hz ToF / 約 30 Hz の ToF

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-58s ", #name); \
        try { test_##name(); tests_passed++; printf("PASS\n"); } \
        catch (...) { tests_failed++; printf("FAIL\n"); } \
    } \
    static void test_##name()

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        printf("FAIL: %s:%d: condition false\n", __FILE__, __LINE__); \
        throw 1; \
    }

/// Where the craft is while the samples arrive. / サンプルが届く間の機体の状況。
struct Scene {
    ToFFlightPhase phase              = ToFFlightPhase::Ground;
    float          rise_m             = 0.0f;       // estimated rise since the craft last rested [m] / 最後の静止からの上昇量の推定 [m]
    uint32_t       takeoff_elapsed_us = 0;          // time since TAKEOFF was entered [us] / TAKEOFF に入ってからの時間 [us]
    float          distance_m         = 0.5f;       // distance of a VALID sample [m] / 有効サンプルの距離 [m]
};

static Scene sceneOf(ToFFlightPhase phase, float rise_m = 0.0f, uint32_t takeoff_elapsed_us = 0)
{
    Scene scene;
    scene.phase              = phase;
    scene.rise_m             = rise_m;
    scene.takeoff_elapsed_us = takeoff_elapsed_us;
    return scene;
}

/// A tracker driven like ImuTask drives it: tick() every 2.5 ms, onSample() every ~33 ms.
/// ImuTask が駆動するのと同じ形で駆動する追跡器: tick() は 2.5 ms ごと、onSample() は約 33 ms ごと。
struct Rig {
    ToFHealthTracker tracker;
    uint32_t now_us = 1000000;   // arbitrary non-zero start / 任意の 0 でない開始時刻

    // Advance the clock by `duration_us`, delivering samples every period when `deliver` is true.
    // 時計を duration_us 進める。deliver が true なら周期ごとにサンプルを届ける。
    void runIn(const Scene& scene, uint32_t duration_us, bool deliver, bool valid, bool adopted, float innovation_m)
    {
        uint32_t next_sample_us = now_us;
        const uint32_t end_us = now_us + duration_us;
        while (now_us < end_us) {
            tracker.tick(now_us);
            if (deliver && now_us >= next_sample_us) {
                tracker.needsReacquire(valid);   // consumed here like ImuTask does (result unused) / ImuTask と同じくここで消費
                ToFSampleFacts facts = {now_us, valid, adopted, innovation_m, scene.distance_m,
                                        scene.phase, scene.rise_m, scene.takeoff_elapsed_us};
                tracker.onSample(now_us, facts);
                next_sample_us += kSamplePeriodUs;
            }
            now_us += 2500;
        }
    }
    // `flying` = the old two-way split: FLYING or resting on the ground. / 旧来の二分: FLYING か地上静止。
    void run(uint32_t duration_us, bool deliver, bool valid, bool adopted, float innovation_m, bool flying)
    {
        runIn(sceneOf(flying ? ToFFlightPhase::Flying : ToFFlightPhase::Ground), duration_us, deliver, valid, adopted,
              innovation_m);
    }
    // Only invalid samples, in `scene`. / 無効サンプルだけを `scene` で。
    void invalidIn(const Scene& scene, uint32_t duration_us) { runIn(scene, duration_us, true, false, false, 0.0f); }
    void agree(uint32_t duration_us) { run(duration_us, true, true, true, 0.01f, false); }
};

// ----------------------------------------------------------------------------
// 1. Tracker / 追跡器
// ----------------------------------------------------------------------------

TEST(boot_is_not_reliable_until_alive_for_one_second_with_enough_agreement)
{
    Rig rig;
    rig.agree(400000);   // 0.4 s: enough samples (12) but alive too short / 0.4 s: サンプルは足りる（12 個）が生存が短い
    ASSERT_TRUE(!rig.tracker.altitudeReliable());
    rig.agree(700000);   // total 1.1 s / 合計 1.1 s
    ASSERT_TRUE(rig.tracker.altitudeReliable());
}

TEST(arrival_alone_proves_nothing_the_estimator_must_adopt_with_small_innovation)
{
    Rig rig;
    rig.run(2000000, true, true, false, 1.2f, false);   // valid samples, all gated out / 有効だが全て棄却
    ASSERT_TRUE(!rig.tracker.altitudeReliable());
    rig.run(2000000, true, true, true, 0.5f, false);    // adopted, but innovation 0.5 m: not "small" / 採用されたが 0.5 m は「小さく」ない
    ASSERT_TRUE(!rig.tracker.altitudeReliable());
    rig.agree(1500000);                                  // the estimator now agrees / 推定器が一致するようになった
    ASSERT_TRUE(rig.tracker.altitudeReliable());
}

TEST(a_large_innovation_breaks_the_streak_but_not_a_reliable_verdict)
{
    Rig rig;
    rig.agree(1500000);
    ASSERT_TRUE(rig.tracker.altitudeReliable());
    rig.run(100000, true, true, true, 0.4f, false);   // a disagreement mid-flight (maneuver) / 飛行中の不一致（機動）
    ASSERT_TRUE(rig.tracker.altitudeReliable());       // latched: only a LOSS clears it / ラッチ: 「喪失」だけが解除する
}

TEST(silence_is_a_loss_and_recovery_needs_the_full_episode)
{
    Rig rig;
    rig.agree(1500000);
    ASSERT_TRUE(rig.tracker.altitudeReliable());
    rig.run(400000, false, false, false, 0.0f, false);   // 0.4 s without a sample (> 0.3 s) / 0.4 s サンプル無し（0.3 s 超）
    ASSERT_TRUE(!rig.tracker.altitudeReliable());
    rig.agree(500000);                                    // samples are back; 0.5 s alive / サンプルが戻った。生存 0.5 s
    ASSERT_TRUE(!rig.tracker.altitudeReliable());
    rig.agree(700000);                                    // 1.2 s alive and > 10 agreeing samples / 生存 1.2 s、一致 10 個超
    ASSERT_TRUE(rig.tracker.altitudeReliable());
}

TEST(a_flickering_sensor_never_reaches_reliable)
{
    // Alive for 0.5 s, silent for 0.4 s, repeated: the alive time restarts at every return.
    // 0.5 s 生存、0.4 s 停止の繰り返し: 生存時間は復帰ごとにやり直し。
    Rig rig;
    rig.agree(1500000);
    for (int i = 0; i < 10; ++i) {
        rig.run(400000, false, false, false, 0.0f, false);
        rig.agree(500000);
        ASSERT_TRUE(!rig.tracker.altitudeReliable());
    }
}

TEST(a_gap_just_below_the_silence_threshold_is_not_a_loss)
{
    Rig rig;
    rig.agree(1500000);
    rig.run(250000, false, false, false, 0.0f, false);   // 0.25 s < 0.3 s
    ASSERT_TRUE(rig.tracker.altitudeReliable());
}

TEST(reacquire_is_requested_once_for_the_first_valid_sample_after_a_loss)
{
    Rig rig;
    rig.agree(1500000);
    ASSERT_TRUE(rig.tracker.altitudeReliable());
    rig.run(400000, false, false, false, 0.0f, false);   // loss
    rig.tracker.tick(rig.now_us);
    ASSERT_TRUE(!rig.tracker.needsReacquire(false));      // an invalid sample does not consume it / 無効サンプルは消費しない
    ASSERT_TRUE(rig.tracker.needsReacquire(true));        // the first valid one does / 最初の有効サンプルが消費する
    ASSERT_TRUE(!rig.tracker.needsReacquire(true));       // once only / 1 回だけ
}

TEST(silence_longer_than_36_and_72_minutes_is_still_a_loss)
{
    // SilenceWatch saturates, so the tracker cannot read a very long silence as healthy.
    // SilenceWatch が飽和するので、追跡器が非常に長い出力停止を健全と読むことはない。
    for (uint32_t minutes : {40u, 80u}) {
        Rig rig;
        rig.agree(1500000);
        ASSERT_TRUE(rig.tracker.altitudeReliable());
        // Observe every second (callers observe continuously) for `minutes` minutes. / 毎秒観測する。
        for (uint32_t s = 0; s < minutes * 60u; ++s) {
            rig.now_us += 1000000u;
            rig.tracker.tick(rig.now_us);
        }
        ASSERT_TRUE(!rig.tracker.altitudeReliable());
    }
}

TEST(invalid_samples_on_the_ground_are_normal_and_count_as_agreement)
{
    Rig rig;
    rig.run(1500000, true, false, false, 0.0f, false);   // below the minimum range, not flying / 最小レンジ未満、飛行中でない
    ASSERT_TRUE(rig.tracker.altitudeReliable());
    ASSERT_TRUE(!rig.tracker.invalidStuck());
}

TEST(only_invalid_samples_while_flying_become_stuck_after_the_persist_time)
{
    Rig rig;
    rig.agree(1500000);
    rig.run(sf::limits::kTofInvalidPersistUs - 100000, true, false, false, 0.0f, true);
    ASSERT_TRUE(!rig.tracker.invalidStuck());
    ASSERT_TRUE(rig.tracker.altitudeReliable());
    rig.run(200000, true, false, false, 0.0f, true);
    ASSERT_TRUE(rig.tracker.invalidStuck());
    ASSERT_TRUE(!rig.tracker.altitudeReliable());
}

TEST(a_valid_sample_inside_the_invalid_run_restarts_the_timer)
{
    Rig rig;
    rig.agree(1500000);
    for (int i = 0; i < 5; ++i) {
        rig.run(400000, true, false, false, 0.0f, true);   // 0.4 s invalid
        rig.run(100000, true, true, true, 0.01f, true);    // a valid sample
        ASSERT_TRUE(!rig.tracker.invalidStuck());
    }
}

TEST(stuck_ends_with_a_valid_sample_and_the_altitude_must_be_re_acquired)
{
    Rig rig;
    rig.agree(1500000);
    rig.run(700000, true, false, false, 0.0f, true);
    ASSERT_TRUE(rig.tracker.invalidStuck());
    rig.run(30000, true, true, true, 0.01f, true);         // ONE valid sample again, innovation small / 有効サンプル「1 つ」が再び、イノベーション小
    ASSERT_TRUE(!rig.tracker.invalidStuck());
    ASSERT_TRUE(!rig.tracker.altitudeReliable());           // not yet: the episode ends on agreement + life / まだ: 一致と生存でエピソードが終わる
    ASSERT_TRUE(rig.tracker.needsReacquire(true));           // and the estimator is asked to re-acquire / 推定器に取り込み直しを求める
    rig.run(1500000, true, true, true, 0.01f, true);
    ASSERT_TRUE(rig.tracker.altitudeReliable());
}

TEST(stuck_on_the_ground_is_not_started_and_a_started_one_survives_the_landing)
{
    // On the ground the invalid run does not even start...
    // 地上では無効の連続は始まりもしない…
    Rig fresh;
    fresh.agree(1500000);
    fresh.run(2000000, true, false, false, 0.0f, false);
    ASSERT_TRUE(!fresh.tracker.invalidStuck());
    // ...but a stuck verdict reached while flying is kept through the landing (until a valid
    // sample or the disarm), so the blind landing is not cut short by the state change.
    // …が、飛行中に成立した継続判定は着陸の間も保つ（有効サンプルか DISARM まで）。状態が変わっても
    // ToF なしの着陸が打ち切られない。
    Rig rig;
    rig.agree(1500000);
    rig.run(700000, true, false, false, 0.0f, true);
    ASSERT_TRUE(rig.tracker.invalidStuck());
    rig.invalidIn(sceneOf(ToFFlightPhase::Landing), 1000000);   // LANDING, last valid 0.5 m: counts, stays stuck / 最後の有効値 0.5 m: 数える、継続のまま
    ASSERT_TRUE(rig.tracker.invalidStuck());
    rig.invalidIn(sceneOf(ToFFlightPhase::Ground), 1000000);    // after touchdown: the verdict is kept until the disarm / 接地後: DISARM まで保つ
    ASSERT_TRUE(rig.tracker.invalidStuck());
    rig.tracker.onDisarm();
    ASSERT_TRUE(!rig.tracker.invalidStuck());
}

// ----------------------------------------------------------------------------
// 1b. What an invalid sample means by phase (C2 / C5) / 無効サンプルの意味（フェーズ別）
// ----------------------------------------------------------------------------

TEST(classify_invalid_sample_covers_every_phase)
{
    using sf::ToFSampleFacts;
    const auto classify = [](ToFFlightPhase phase, float rise_m, uint32_t elapsed_us, float last_valid_m) {
        const ToFSampleFacts facts = {0, false, false, 0.0f, 0.0f, phase, rise_m, elapsed_us};
        return sf::classifyInvalidSample(facts, last_valid_m);
    };
    const uint32_t max_us = sf::limits::kTofInvalidTakeoffMaxUs;
    const float rise_m = sf::limits::kTofInvalidTakeoffRiseM;
    const float height_m = sf::limits::kTofInvalidLandingMinHeightM;
    // Ground: always normal. / 地上: いつも正常。
    ASSERT_TRUE(classify(ToFFlightPhase::Ground, 0.0f, 0, 0.0f) == InvalidSampleMeaning::Agrees);
    ASSERT_TRUE(classify(ToFFlightPhase::Ground, 9.0f, max_us + 1, 9.0f) == InvalidSampleMeaning::Agrees);
    // Flying: counts. / FLYING: 数える。
    ASSERT_TRUE(classify(ToFFlightPhase::Flying, 0.0f, 0, 0.0f) == InvalidSampleMeaning::Counts);
    // Takeoff: ignored until the rise OR the time limit, then counts. / TAKEOFF: 上昇量か時間の上限まで無視、その後数える。
    ASSERT_TRUE(classify(ToFFlightPhase::Takeoff, 0.0f, 0, 0.0f) == InvalidSampleMeaning::Ignored);
    ASSERT_TRUE(classify(ToFFlightPhase::Takeoff, rise_m - 0.01f, max_us - 1, 0.0f) == InvalidSampleMeaning::Ignored);
    ASSERT_TRUE(classify(ToFFlightPhase::Takeoff, rise_m, 0, 0.0f) == InvalidSampleMeaning::Counts);
    ASSERT_TRUE(classify(ToFFlightPhase::Takeoff, 0.0f, max_us, 0.0f) == InvalidSampleMeaning::Counts);
    // Landing: counts only while the last valid height was high enough. / LANDING: 最後の有効な高さが十分なときだけ数える。
    ASSERT_TRUE(classify(ToFFlightPhase::Landing, 0.0f, 0, height_m) == InvalidSampleMeaning::Counts);
    ASSERT_TRUE(classify(ToFFlightPhase::Landing, 0.0f, 0, height_m - 0.01f) == InvalidSampleMeaning::Ignored);
    // Flip: never counts, whatever the rise and the last height. / FLIP: 上昇量・最後の高さに関わらず数えない。
    ASSERT_TRUE(classify(ToFFlightPhase::Flip, 9.0f, max_us + 1, 9.0f) == InvalidSampleMeaning::Ignored);
}

TEST(takeoff_before_the_rise_or_the_time_limit_never_becomes_stuck)
{
    // The real takeoff: the floor reads invalid for 186-344 ms after TAKEOFF is entered, and a slow
    // craft (10 % low estimate: 0.575 s) must not be cut short by the 0.5 s persist time.
    // 実機の離陸: TAKEOFF に入った後、床は 186〜344 ms 無効を返す。離陸の遅い機体（見積もり 10 % 低で
    // 0.575 s）が、0.5 s の継続時間で打ち切られてはならない。
    Rig rig;
    rig.agree(1500000);                                                      // on the ground: reliable / 地上で信頼できる
    rig.invalidIn(sceneOf(ToFFlightPhase::Takeoff, 0.02f, 0), 575000);        // 0.575 s, rise 2 cm / 0.575 s、上昇 2 cm
    ASSERT_TRUE(!rig.tracker.invalidStuck());
    ASSERT_TRUE(rig.tracker.altitudeReliable());                              // ignored: neither evidence nor loss / 無視: 証拠でも喪失でもない
    rig.invalidIn(sceneOf(ToFFlightPhase::Takeoff, 0.02f, sf::limits::kTofInvalidTakeoffMaxUs - 100000), 1000000);
    ASSERT_TRUE(!rig.tracker.invalidStuck());                                 // still before the time limit / まだ時間の上限の前
}

TEST(takeoff_counts_after_the_rise_and_at_the_latest_after_the_time_limit)
{
    // Once the evidence counts, TAKEOFF needs the SHORTER kTofInvalidTakeoffPersistUs (a craft that left the dead
    // zone and sees nothing is already climbing blind), not the 0.5 s of FLYING.
    // 証拠が数えられるようになったら、TAKEOFF は FLYING の 0.5 s でなく「短い」kTofInvalidTakeoffPersistUs で足りる
    // （不感帯を出たのに何も見えない機体は、すでに ToF なしで上昇している）。
    Rig by_rise;
    by_rise.agree(1500000);
    by_rise.invalidIn(sceneOf(ToFFlightPhase::Takeoff, sf::limits::kTofInvalidTakeoffRiseM + 0.01f, 400000),
                      sf::limits::kTofInvalidTakeoffPersistUs - 50000);
    ASSERT_TRUE(!by_rise.tracker.invalidStuck());
    by_rise.invalidIn(sceneOf(ToFFlightPhase::Takeoff, 0.10f, 600000), 100000);
    ASSERT_TRUE(by_rise.tracker.invalidStuck());                              // short evidence after the rise / 上昇の後の短い証拠

    Rig by_time;
    by_time.agree(1500000);
    by_time.invalidIn(sceneOf(ToFFlightPhase::Takeoff, 0.0f, sf::limits::kTofInvalidTakeoffMaxUs),
                      sf::limits::kTofInvalidTakeoffPersistUs + 100000);
    ASSERT_TRUE(by_time.tracker.invalidStuck());                              // never rose, the limit was reached / 上昇せず、上限に達した

    // FLYING keeps the long persistence: 0.2 s of invalid samples are not enough there.
    // FLYING は長い継続時間のまま: 0.2 s の無効サンプルでは足りない。
    Rig flying;
    flying.agree(1500000);
    flying.invalidIn(sceneOf(ToFFlightPhase::Flying), sf::limits::kTofInvalidTakeoffPersistUs + 50000);
    ASSERT_TRUE(!flying.tracker.invalidStuck());
}

TEST(a_loss_then_invalid_only_samples_in_landing_is_stuck_not_reliable)
{
    // The C5 case: a silence while flying, then LANDING with samples back but ALL invalid. They must
    // not count as agreement (the estimator would be declared reliable after ~1 s).
    // C5 の場合: 飛行中の出力停止の後、サンプルは戻ったが LANDING で「全部」無効。一致と数えてはならない
    // （約 1 s で推定器が信頼できると宣言されてしまう）。
    Rig rig;
    Scene flying = sceneOf(ToFFlightPhase::Flying);
    rig.runIn(flying, 1500000, true, true, true, 0.01f);                      // healthy flight at 0.5 m / 0.5 m で健全な飛行
    ASSERT_TRUE(rig.tracker.altitudeReliable());
    rig.run(400000, false, false, false, 0.0f, true);                         // silence 0.4 s: a loss / 出力停止 0.4 s: 喪失
    rig.invalidIn(sceneOf(ToFFlightPhase::Landing), 3000000);
    ASSERT_TRUE(!rig.tracker.altitudeReliable());
    ASSERT_TRUE(rig.tracker.invalidStuck());
}

TEST(invalid_samples_in_landing_near_the_floor_and_in_flip_are_not_agreement_either)
{
    for (ToFFlightPhase phase : {ToFFlightPhase::Landing, ToFFlightPhase::Flip}) {
        Rig rig;
        Scene scene = sceneOf(ToFFlightPhase::Flying);
        scene.distance_m = 0.05f;                                             // the last valid sample was 5 cm high / 最後の有効サンプルは高さ 5 cm
        rig.runIn(scene, 1500000, true, true, true, 0.01f);
        rig.run(400000, false, false, false, 0.0f, true);                     // loss
        scene.phase = phase;
        rig.runIn(scene, 3000000, true, false, false, 0.0f);                  // invalid only, 3 s / 無効だけ 3 s
        ASSERT_TRUE(!rig.tracker.altitudeReliable());                         // not agreement / 一致ではない
        ASSERT_TRUE(!rig.tracker.invalidStuck());                             // not evidence either / 証拠でもない
    }
}

TEST(a_flip_never_starts_the_stuck_run_and_the_run_restarts_after_it)
{
    Rig rig;
    rig.agree(1500000);
    rig.invalidIn(sceneOf(ToFFlightPhase::Flip), 2000000);                    // a long flip with an invalid sensor / センサが無効のままの長い宙返り
    ASSERT_TRUE(!rig.tracker.invalidStuck());
    rig.invalidIn(sceneOf(ToFFlightPhase::Flying), sf::limits::kTofInvalidPersistUs - 100000);
    ASSERT_TRUE(!rig.tracker.invalidStuck());                                 // the 0.5 s counts from FLYING, not from the flip / 0.5 s は宙返りでなく FLYING から数える
    rig.invalidIn(sceneOf(ToFFlightPhase::Flying), 200000);
    ASSERT_TRUE(rig.tracker.invalidStuck());
}

TEST(stuck_asks_for_the_re_acquisition_before_the_first_valid_sample)
{
    // The order ImuTask uses: needsReacquire(valid) BEFORE onSample(). The stuck verdict must already
    // have requested the re-acquisition, so that the first valid sample is not adopted un-reacquired.
    // ImuTask の呼び順: onSample() の「前」に needsReacquire(valid)。継続判定の時点で取り込み直しを求めて
    // おくこと。最初の有効サンプルが取り込み直しなしで採用されないように。
    Rig rig;
    rig.agree(1500000);
    rig.run(700000, true, false, false, 0.0f, true);
    ASSERT_TRUE(rig.tracker.invalidStuck());
    ASSERT_TRUE(!rig.tracker.needsReacquire(false));   // an invalid sample does not consume it / 無効サンプルは消費しない
    ASSERT_TRUE(rig.tracker.needsReacquire(true));     // the first valid one does, BEFORE onSample() / 最初の有効サンプルが、onSample() の「前」に消費する
    ASSERT_TRUE(!rig.tracker.needsReacquire(true));    // once only / 1 回だけ
}

// ----------------------------------------------------------------------------
// 2. Verdict / 判定
// ----------------------------------------------------------------------------

TEST(verdict_covers_every_cell)
{
    for (int no_sample = 0; no_sample < 2; ++no_sample) {
        for (int stuck = 0; stuck < 2; ++stuck) {
            for (int reliable = 0; reliable < 2; ++reliable) {
                const ToFSilenceReason got = sf::judgeToFSilenceReason(no_sample, stuck, reliable);
                if (no_sample) {
                    ASSERT_TRUE(got == ToFSilenceReason::NoSample);
                } else if (stuck) {
                    ASSERT_TRUE(got == ToFSilenceReason::InvalidStuck);
                } else if (!reliable) {
                    ASSERT_TRUE(got == ToFSilenceReason::Reacquiring);
                } else {
                    ASSERT_TRUE(got == ToFSilenceReason::None);
                }
            }
        }
    }
}

// ----------------------------------------------------------------------------
// 3. Stale failsafe_status / 古い failsafe_status
// ----------------------------------------------------------------------------

static sf::FailsafeStatus statusAt(uint32_t stamp_us, bool silent)
{
    sf::FailsafeStatus status = {};
    status.tof_silent = silent;
    status.tof_reason = silent ? static_cast<uint8_t>(ToFSilenceReason::NoSample) : 0;
    status.timestamp = stamp_us;
    return status;
}

TEST(a_refreshed_status_passes_through_unchanged)
{
    sf::FailsafeStatusGuard guard;
    uint32_t now = 2000000;
    for (int i = 0; i < 50; ++i) {   // 5 s at 10 Hz, "not silent"
        const sf::FailsafeStatus got = guard.guard(now, statusAt(now - 1000, false));
        ASSERT_TRUE(!got.tof_silent);
        now += 100000;
    }
}

TEST(a_status_nobody_refreshes_reads_as_silent_even_if_the_last_one_said_not_silent)
{
    sf::FailsafeStatusGuard guard;
    uint32_t now = 2000000;
    const sf::FailsafeStatus last = statusAt(now, false);
    ASSERT_TRUE(!guard.guard(now, last).tof_silent);
    now += sf::limits::kFailsafeStatusStaleUs - 100000;
    ASSERT_TRUE(!guard.guard(now, last).tof_silent);   // still within the window / まだ窓の内
    now += 200000;
    const sf::FailsafeStatus got = guard.guard(now, last);
    ASSERT_TRUE(got.tof_silent);
    ASSERT_TRUE(got.tof_reason == static_cast<uint8_t>(ToFSilenceReason::StatusStale));
    // Refreshed again: back to the published value. / 再び更新された: 発行された値に戻る。
    ASSERT_TRUE(!guard.guard(now + 100000, statusAt(now + 100000, false)).tof_silent);
}

TEST(a_status_never_published_reads_as_silent)
{
    sf::FailsafeStatusGuard guard;
    ASSERT_TRUE(guard.guard(5000000, sf::FailsafeStatus{}).tof_silent);
}

TEST(a_stale_status_stays_silent_beyond_36_and_72_minutes)
{
    for (uint32_t minutes : {40u, 80u}) {
        sf::FailsafeStatusGuard guard;
        uint32_t now = 2000000;
        const sf::FailsafeStatus last = statusAt(now, false);
        guard.guard(now, last);
        for (uint32_t s = 0; s < minutes * 60u; ++s) {
            now += 1000000u;
            guard.guard(now, last);
        }
        ASSERT_TRUE(guard.guard(now, last).tof_silent);
    }
}

TEST(the_display_staleness_test_needs_no_observation_and_catches_the_edges)
{
    const uint32_t now = 5000000;
    ASSERT_TRUE(!sf::isFailsafeStatusStaleForDisplay(now, now - 100000));                          // fresh / 新しい
    ASSERT_TRUE(!sf::isFailsafeStatusStaleForDisplay(now, now + 1000));                            // cross-task order inversion / 読み出し順序の逆転
    ASSERT_TRUE(!sf::isFailsafeStatusStaleForDisplay(now, now - sf::limits::kFailsafeStatusStaleUs + 1));
    ASSERT_TRUE(sf::isFailsafeStatusStaleForDisplay(now, now - sf::limits::kFailsafeStatusStaleUs));   // exactly the window: stale / ちょうど窓: 古い
    ASSERT_TRUE(sf::isFailsafeStatusStaleForDisplay(now, 0));                                      // never published / 未発行
    ASSERT_TRUE(sf::isFailsafeStatusStaleForDisplay(now + 2400000000u, now));                      // > 35.8 min wraps negative: stale / 35.8 分超は負に巻き戻る: 古い
}

// ----------------------------------------------------------------------------
// 4. D3 mode refusal and ARM refusal / D3 のモード拒否と ARM 拒否
// ----------------------------------------------------------------------------

TEST(altitude_mode_requests_are_consumed_in_flight_and_kept_on_the_ground)
{
    for (FlightMode mode : {FlightMode::ALT_HOLD, FlightMode::POS_HOLD}) {
        ASSERT_TRUE(sf::decideToFModeRefusal(FlightState::FLYING, mode, true) == ToFModeRefusal::Consume);
        ASSERT_TRUE(sf::decideToFModeRefusal(FlightState::IDLE_GROUND, mode, true) == ToFModeRefusal::Keep);
        ASSERT_TRUE(sf::decideToFModeRefusal(FlightState::ARMED_GROUND, mode, true) == ToFModeRefusal::Keep);
        ASSERT_TRUE(sf::decideToFModeRefusal(FlightState::FLYING, mode, false) == ToFModeRefusal::None);
    }
}

TEST(manual_mode_requests_are_never_refused)
{
    for (FlightMode mode : {FlightMode::ACRO, FlightMode::STABILIZE}) {
        for (FlightState state : {FlightState::IDLE_GROUND, FlightState::ARMED_GROUND, FlightState::FLYING}) {
            ASSERT_TRUE(sf::decideToFModeRefusal(state, mode, true) == ToFModeRefusal::None);
        }
    }
}

TEST(arm_is_refused_during_an_episode_in_every_mode)
{
    ASSERT_TRUE(sf::toFSilenceRefusesArm(true));
    ASSERT_TRUE(!sf::toFSilenceRefusesArm(false));
}

TEST(tracker_constants_are_consistent)
{
    // 10 samples at ~30 Hz must fit well inside the alive time, so the alive time is the binding
    // condition; the innovation threshold sits below the 0.15 m ground/airborne band and far below
    // the 0.5 m innovation gate; the invalid persist time is longer than the silence threshold.
    // 約 30 Hz で 10 サンプルは生存時間よりずっと短いので、拘束条件は生存時間。イノベーションの
    // しきい値は接地/空中の帯 0.15 m より下で、イノベーション判定 0.5 m よりずっと下。無効の継続時間は
    // 出力停止のしきい値より長い。
    ASSERT_TRUE(sf::limits::kTofReacquireSamples * kSamplePeriodUs < sf::limits::kTofAliveMinUs);
    ASSERT_TRUE(sf::limits::kTofReacquireInnovationM < 0.15f);
    ASSERT_TRUE(sf::limits::kTofInvalidPersistUs > sf::limits::kTofSilenceUs);
    ASSERT_TRUE(sf::limits::kFailsafeStatusStaleUs > 100000u);
    // The takeoff time limit lets a stuck verdict form inside the controller's takeoff timeout (3 s), and the
    // rise is far above the dead zone of the real sensor (3 cm) and the accelerometer integral's error (~1 cm).
    // 離陸の時間の上限は、制御器の離陸のタイムアウト（3 s）の内側で継続判定が成立できる長さ。上昇量は、実機センサの
    // 不感帯（3 cm）と加速度積分の誤差（約 1 cm）よりずっと上。
    ASSERT_TRUE(sf::limits::kTofInvalidTakeoffMaxUs + sf::limits::kTofInvalidTakeoffPersistUs <= 3000000u);   // PidController::kTakeoffMaxCycles = 3 s
    ASSERT_TRUE(sf::limits::kTofInvalidTakeoffPersistUs < sf::limits::kTofInvalidPersistUs);
    ASSERT_TRUE(sf::limits::kTofInvalidTakeoffPersistUs >= 4 * kSamplePeriodUs);   // at least 4 ToF samples / 少なくとも ToF 4 サンプル
    ASSERT_TRUE(sf::limits::kTofInvalidTakeoffRiseM >= 0.05f);
    ASSERT_TRUE(sf::limits::kTofInvalidLandingMinHeightM > 3 * 0.031f);
}

int main()
{
    printf("=== ToF re-acquisition judgement unit tests ===\n");

    run_boot_is_not_reliable_until_alive_for_one_second_with_enough_agreement();
    run_arrival_alone_proves_nothing_the_estimator_must_adopt_with_small_innovation();
    run_a_large_innovation_breaks_the_streak_but_not_a_reliable_verdict();
    run_silence_is_a_loss_and_recovery_needs_the_full_episode();
    run_a_flickering_sensor_never_reaches_reliable();
    run_a_gap_just_below_the_silence_threshold_is_not_a_loss();
    run_reacquire_is_requested_once_for_the_first_valid_sample_after_a_loss();
    run_silence_longer_than_36_and_72_minutes_is_still_a_loss();
    run_invalid_samples_on_the_ground_are_normal_and_count_as_agreement();
    run_only_invalid_samples_while_flying_become_stuck_after_the_persist_time();
    run_a_valid_sample_inside_the_invalid_run_restarts_the_timer();
    run_stuck_ends_with_a_valid_sample_and_the_altitude_must_be_re_acquired();
    run_stuck_on_the_ground_is_not_started_and_a_started_one_survives_the_landing();
    run_classify_invalid_sample_covers_every_phase();
    run_takeoff_before_the_rise_or_the_time_limit_never_becomes_stuck();
    run_takeoff_counts_after_the_rise_and_at_the_latest_after_the_time_limit();
    run_a_loss_then_invalid_only_samples_in_landing_is_stuck_not_reliable();
    run_invalid_samples_in_landing_near_the_floor_and_in_flip_are_not_agreement_either();
    run_a_flip_never_starts_the_stuck_run_and_the_run_restarts_after_it();
    run_stuck_asks_for_the_re_acquisition_before_the_first_valid_sample();
    run_verdict_covers_every_cell();
    run_a_refreshed_status_passes_through_unchanged();
    run_a_status_nobody_refreshes_reads_as_silent_even_if_the_last_one_said_not_silent();
    run_a_status_never_published_reads_as_silent();
    run_a_stale_status_stays_silent_beyond_36_and_72_minutes();
    run_the_display_staleness_test_needs_no_observation_and_catches_the_edges();
    run_altitude_mode_requests_are_consumed_in_flight_and_kept_on_the_ground();
    run_manual_mode_requests_are_never_refused();
    run_arm_is_refused_during_an_episode_in_every_mode();
    run_tracker_constants_are_consistent();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
