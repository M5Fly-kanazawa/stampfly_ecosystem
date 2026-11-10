/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_takeoff_policy.cpp
 * @brief Host tests for the TAKEOFF decisions (takeoff_policy.hpp): every cell of the normative table
 *        TAKEOFF の判断（takeoff_policy.hpp）のホスト試験: 規範表の全セル
 *
 * Pure logic, no ESP-IDF / FreeRTOS. The cells are the rows of detailed_design.md §3.1 (TAKEOFF -> FLYING manual / auto, TAKEOFF -> IDLE_GROUND, comm loss):
 *  1. decideTakeoff(): manual (ACRO / STABILIZE) x airborne; auto (ALT_HOLD / POS_HOLD) x controller_reached x floor_stays x floor_seen x elapsed (before / at the abort time),
 *     including the LANDING of a latch that does not release (not while the craft is lifting off) and the readings that straddle the ground band (every sample, every 5, 15, 29, 31 and 32 samples).
 *  2. decideCommLossLanding(): every FlightState x grace elapsed or not.
 *  2b. landingVerdictCounts(): a touchdown verdict counts only with the stamp of the present LANDING.
 *  3. TakeoffFloorWatch: the positive floor evidence (in band and not rising for the hold time; an invalid sample is neutral; a silent ToF, a reading above the band
 *     or a rise ends or restarts it), and its latch (once the evidence has held, the completion stays blocked until the craft is shown to leave the floor: valid readings above the
 *     ground band for the hold time, or one valid reading above kTakeoffAirborneBandM; an in-band reading, a silent ToF or a reset ends the run above the band).
 * The expected values are written out cell by cell from the table, not recomputed with the function's own expression.
 * Own TEST/ASSERT framework and main(), same pattern as test_tof_silence.cpp.
 *
 * @design architecture.md INV-4 — every (state x input) cell of the table is checked  [OK]
 * @design detailed_design.md §3.1 — TAKEOFF rows, comm-loss row                         [OK]
 */

#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "flight_limits.hpp"
#include "flight_state.hpp"
#include "takeoff_policy.hpp"

using sf::FlightState;
using sf::TakeoffAction;
using sf::TakeoffFacts;
using sf::TakeoffFloorWatch;

static constexpr uint32_t kAbortUs = sf::limits::kAutoTakeoffNoLiftAbortUs;
static constexpr uint32_t kHoldUs  = sf::limits::kAutoTakeoffFloorHoldUs;
static constexpr float    kBandM   = sf::limits::kTakeoffGroundBandM;
static constexpr float    kAirM    = sf::limits::kTakeoffAirborneBandM;
static constexpr float    kRiseM   = sf::limits::kAutoTakeoffFloorRiseM;

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-52s ", #name); \
        try { test_##name(); tests_passed++; printf("PASS\n"); } \
        catch (...) { tests_failed++; printf("FAIL\n"); } \
    } \
    static void test_##name()

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        printf("FAIL: %s:%d: condition false\n", __FILE__, __LINE__); \
        throw 1; \
    }

static TakeoffAction decide(bool auto_takeoff, bool airborne, bool reached, bool floor_stays, uint32_t elapsed_us, bool floor_seen = false, bool floor_leaving = false)
{
    return sf::decideTakeoff(TakeoffFacts{auto_takeoff, airborne, reached, floor_stays, floor_seen, floor_leaving, elapsed_us});
}

// ----------------------------------------------------------------------------
// 1. decideTakeoff()
// ----------------------------------------------------------------------------

TEST(manual_takeoff_completes_when_the_tof_says_airborne_and_never_aborts)
{
    // ACRO / STABILIZE: the ToF verdict alone decides (a silent ToF makes it "armed = not on the ground"); the controller's flag, the floor evidence and the time do not matter.
    // ACRO / STABILIZE: ToF の判定だけで決まる（ToF が出力停止ならそれは「ARM 中 = 接地していない」）。制御器のフラグ、床の証拠、時間は関係しない。
    for (bool reached : {false, true}) {
        for (bool floor_stays : {false, true}) {
            for (bool floor_seen : {false, true}) {   // the latch is not read either / ラッチも読まない
                for (uint32_t elapsed_us : {0u, kAbortUs - 1u, kAbortUs, kAbortUs * 4u}) {
                    ASSERT_TRUE(decide(false, true,  reached, floor_stays, elapsed_us, floor_seen) == TakeoffAction::Complete);
                    ASSERT_TRUE(decide(false, false, reached, floor_stays, elapsed_us, floor_seen) == TakeoffAction::Continue);
                }
            }
        }
    }
}

TEST(auto_takeoff_completes_when_captured_and_the_floor_is_not_shown)
{
    // The airborne verdict is not read in the auto-takeoff (the controller's capture decides). 自動離陸は airborne を読まない（制御器の捕捉が決める）。
    for (bool airborne : {false, true}) {
        for (uint32_t elapsed_us : {0u, kAbortUs - 1u, kAbortUs, kAbortUs * 4u}) {
            ASSERT_TRUE(decide(true, airborne, true,  false, elapsed_us) == TakeoffAction::Complete);
            ASSERT_TRUE(decide(true, airborne, false, false, elapsed_us) == TakeoffAction::Continue);   // no floor evidence: never aborted, the ToF-invalid rule / the controller backstop end it / 床の証拠なし: 中止しない
        }
    }
}

TEST(auto_takeoff_with_the_floor_shown_stays_and_is_aborted_after_the_abort_time)
{
    // floor_stays with the latch (floor_seen) both ways: while the evidence is there NOW the latch is set in practice, but the decision must not depend on it.
    // floor_stays に、ラッチ（floor_seen）の両方を組み合わせる: 証拠が「今」あるときラッチは実際には立っているが、判断はそれに依ってはならない。
    for (bool airborne : {false, true}) {
        for (bool reached : {false, true}) {
            for (bool floor_seen : {false, true}) {
                ASSERT_TRUE(decide(true, airborne, reached, true, 0u,           floor_seen) == TakeoffAction::Continue);   // held, even if the controller says reached / 保持（制御器が到達と言っても）
                ASSERT_TRUE(decide(true, airborne, reached, true, kAbortUs - 1, floor_seen) == TakeoffAction::Continue);
                ASSERT_TRUE(decide(true, airborne, reached, true, kAbortUs,     floor_seen) == TakeoffAction::AbortDisarm);
                ASSERT_TRUE(decide(true, airborne, reached, true, kAbortUs * 3, floor_seen) == TakeoffAction::AbortDisarm);
            }
        }
    }
}

TEST(a_slow_lift_is_never_aborted_and_the_missing_evidence_is_not_the_floor)
{
    // The case a rule on the accelerometer alone gets wrong: a craft that lifts slowly (the ToF reads above the band, so the floor evidence is absent) is not aborted at any time, whatever the
    // accelerometer would say; and the controller's backstop completes it.
    // 加速度計だけの規則が誤る場面: ゆっくり浮く機体（ToF は帯より上を読むので、床の証拠は無い）は、加速度計が何と言おうと、どの時刻でも中止されず、制御器の備えが完了させる。
    for (uint32_t elapsed_us : {kAbortUs, kAbortUs + 1000000u, 60000000u}) {
        ASSERT_TRUE(decide(true, false, false, false, elapsed_us) == TakeoffAction::Continue);
        ASSERT_TRUE(decide(true, false, true,  false, elapsed_us) == TakeoffAction::Complete);
    }
}

TEST(the_latched_floor_evidence_blocks_the_completion_but_never_causes_an_abort)
{
    // The abort needs 1 s of evidence NOW; if the completion needed only that the evidence is not there at this instant, one reading outside the band after the abort time would let the takeoff
    // complete on the floor. The latch (floor_seen) keeps the completion blocked; it is no evidence for the abort (floor_stays is what the abort reads). Until the abort time the TAKEOFF stays;
    // from the abort time on, a latch without the evidence NOW ends it by a LANDING (never by the DISARM).
    // 中止は「今」の 1 s の証拠を要する。完了が「この瞬間は証拠が無い」だけを要するなら、中止の時刻の後に帯の外へ外れた 1 つの読みで、床の上で離陸が完了する。
    // ラッチ（floor_seen）が完了を止め続ける。中止の証拠にはならない（中止が読むのは floor_stays）。中止の時刻までは TAKEOFF に留まり、その時刻からは、「今」の証拠が無いラッチは
    // LANDING で終わらせる（DISARM では決して終わらせない）。
    for (bool airborne : {false, true}) {
        for (bool reached : {false, true}) {
            for (uint32_t elapsed_us : {0u, kAbortUs - 1u}) {
                ASSERT_TRUE(decide(true, airborne, reached, false, elapsed_us, true) == TakeoffAction::Continue);   // latched, evidence not now, before the abort time: held / ラッチ中、今は証拠なし、中止の時刻の前: 保持
            }
            for (uint32_t elapsed_us : {kAbortUs, kAbortUs + 1u, kAbortUs * 4u}) {
                ASSERT_TRUE(decide(true, airborne, reached, false, elapsed_us, true) == TakeoffAction::Land);      // from the abort time: LANDING / 中止の時刻から: LANDING
            }
        }
        for (uint32_t elapsed_us : {0u, kAbortUs - 1u, kAbortUs, kAbortUs * 4u}) {
            ASSERT_TRUE(decide(true, airborne, true, false, elapsed_us, false) == TakeoffAction::Complete);        // latch released: completes / ラッチが解けた: 完了
        }
    }
    ASSERT_TRUE(decide(false, true, true, false, kAbortUs, true) == TakeoffAction::Complete);   // manual takeoff: the latch is not read / 手動離陸: ラッチは読まない
}

TEST(a_landing_is_decided_only_by_a_latch_without_the_evidence_now_after_the_abort_time_in_an_auto_takeoff)
{
    // The cells of TakeoffAction::Land, every combination of the four booleans at the abort time: LANDING exactly when auto x floor_seen x NOT floor_stays. Written out: the evidence NOW
    // gives the abort instead; no latch gives the completion (reached) or the hold (not reached); a manual takeoff never lands by this rule.
    // TakeoffAction::Land のセル。中止の時刻での 4 つの真偽の全組み合わせ: 自動 × floor_seen × floor_stays でない、のときだけ LANDING。書き下す: 「今」の証拠があれば中止、
    // ラッチが無ければ完了（到達）か保持（未到達）、手動離陸はこの規則で着陸しない。
    for (bool airborne : {false, true}) {
        for (bool reached : {false, true}) {
            ASSERT_TRUE(decide(true, airborne, reached, true,  kAbortUs, true)  == TakeoffAction::AbortDisarm);   // evidence now, latched: the abort / 今の証拠、ラッチ中: 中止
            ASSERT_TRUE(decide(true, airborne, reached, true,  kAbortUs, false) == TakeoffAction::AbortDisarm);   // evidence now, no latch (not reached in practice): the abort / 今の証拠、ラッチなし: 中止
            ASSERT_TRUE(decide(true, airborne, reached, false, kAbortUs, true)  == TakeoffAction::Land);          // the new cell / 新しいセル
            ASSERT_TRUE(decide(true, airborne, reached, false, kAbortUs, false) == (reached ? TakeoffAction::Complete : TakeoffAction::Continue));
        }
    }
    for (bool floor_stays : {false, true}) {
        for (bool floor_seen : {false, true}) {
            ASSERT_TRUE(decide(false, false, true, floor_stays, kAbortUs * 2u, floor_seen) == TakeoffAction::Continue);   // manual, not airborne: never lands, never aborts / 手動で空中でない: 着陸も中止もしない
            ASSERT_TRUE(decide(false, true,  true, floor_stays, kAbortUs * 2u, floor_seen) == TakeoffAction::Complete);
        }
    }
}

TEST(a_latched_takeoff_is_not_landed_while_the_craft_is_lifting_off)
{
    // floor_leaving (valid readings above the ground band NOW, the run in progress): a slow lift is on its way to releasing the latch (after the hold time above the band), so even from the abort time on the
    // TAKEOFF is held, not landed. Without the latch or with the evidence NOW the other cells apply as before (leaving cannot hold an abort back: the evidence NOW and a run above the band exclude each other).
    // floor_leaving（接地の帯より上の有効な読みが「今」あり、その区間が進行中）: ゆっくり浮く機体は、ラッチを解く途中（帯より上の保持時間の後）なので、中止の時刻からでも TAKEOFF は保持し、着陸させない。
    // ラッチが無い、または「今」の証拠があるときは、他のセルが従来どおり働く（leaving は中止を止めない: 「今」の証拠と帯より上の区間は両立しない）。
    for (bool airborne : {false, true}) {
        for (bool reached : {false, true}) {
            for (uint32_t elapsed_us : {0u, kAbortUs - 1u, kAbortUs, kAbortUs * 4u}) {
                ASSERT_TRUE(decide(true, airborne, reached, false, elapsed_us, true, true) == TakeoffAction::Continue);   // latched, leaving: held at every time / ラッチ中、離れつつある: どの時刻でも保持
            }
            ASSERT_TRUE(decide(true, airborne, reached, false, kAbortUs, true, false) == TakeoffAction::Land);              // the same, not leaving: LANDING / 同じで離れつつない: LANDING
        }
        for (uint32_t elapsed_us : {0u, kAbortUs, kAbortUs * 4u}) {
            ASSERT_TRUE(decide(true, airborne, true, false, elapsed_us, false, true) == TakeoffAction::Complete);          // no latch: leaving does not matter / ラッチなし: leaving は関係しない
        }
    }
    ASSERT_TRUE(decide(false, true, true, false, kAbortUs, true, true) == TakeoffAction::Complete);   // manual: not read / 手動: 読まない
}

TEST(abort_time_is_longer_than_the_controller_backstop_and_the_ordinary_takeoff)
{
    // The controller's own backstop is 3 s (kTakeoffMaxCycles 1200 at 400 Hz); an ordinary takeoff leaves the band within 1 s. / 制御器自身の備えは 3 s、普通の離陸は 1 s 以内に帯を出る。
    ASSERT_TRUE(kAbortUs > 3000000u);
    ASSERT_TRUE(kAbortUs >= 5u * 900000u);   // five times the slowest ordinary takeoff (0.9 s) / 最も遅い普通の離陸（0.9 s）の 5 倍
    ASSERT_TRUE(kAbortUs < 0x80000000u);
}

// ----------------------------------------------------------------------------
// 2. decideCommLossLanding()
// ----------------------------------------------------------------------------

TEST(comm_loss_lands_from_takeoff_and_flying_only_after_the_grace)
{
    struct Row { FlightState state; bool lands; };
    const Row rows[] = {
        {FlightState::INIT,         false},
        {FlightState::IDLE_GROUND,  false},
        {FlightState::IDLE_HELD,    false},
        {FlightState::ARMED_GROUND, false},
        {FlightState::TAKEOFF,      true},    // a TAKEOFF that does not complete must not keep climbing for ever / 完了しない TAKEOFF が上昇し続けてはならない
        {FlightState::FLYING,       true},
        {FlightState::LANDING,      false},   // already landing / 既に着陸中
        {FlightState::FLIP,         false},   // waits for FLYING / FLYING まで待つ
    };
    for (const Row& row : rows) {
        ASSERT_TRUE(sf::decideCommLossLanding(row.state, true) == row.lands);
        ASSERT_TRUE(!sf::decideCommLossLanding(row.state, false));   // before the grace: never / 猶予の前: 決して
    }
}

// ----------------------------------------------------------------------------
// 2b. landingVerdictCounts()
// ----------------------------------------------------------------------------

TEST(a_touchdown_verdict_counts_only_with_the_stamp_of_the_present_landing)
{
    const uint32_t landing_since_us = 5'000'000;   // when the present LANDING began (SystemMode.state_since_us) / 今の LANDING が始まった時刻
    const uint32_t earlier_since_us = 2'000'000;   // when the FLYING before it began / その前の FLYING が始まった時刻
    ASSERT_TRUE(sf::landingVerdictCounts(true, landing_since_us, landing_since_us));          // judged in this LANDING / この LANDING の中で判定
    ASSERT_TRUE(!sf::landingVerdictCounts(false, landing_since_us, landing_since_us));       // no verdict / 判定なし
    ASSERT_TRUE(!sf::landingVerdictCounts(true, earlier_since_us, landing_since_us));         // left over from the state before: not counted / 前の状態から残った判定: 数えない
    ASSERT_TRUE(!sf::landingVerdictCounts(true, landing_since_us + 1u, landing_since_us));    // a stamp of a later state is not it either (equality, no order) / 後の状態の刻印も違う（等しいかだけで、大小は見ない）
    ASSERT_TRUE(!sf::landingVerdictCounts(false, earlier_since_us, landing_since_us));
    ASSERT_TRUE(sf::landingVerdictCounts(true, 0u, 0u));                                      // equality includes 0 (a state entered at boot) / 0 でも等しければ数える（起動時に入った状態）
    ASSERT_TRUE(sf::landingVerdictCounts(true, 0xFFFFFFFFu, 0xFFFFFFFFu));                    // and the end of the range (wrap) / 範囲の端（巻き戻り）でも
}

// ----------------------------------------------------------------------------
// 3. TakeoffFloorWatch
// ----------------------------------------------------------------------------

static constexpr uint32_t kSampleUs = 33000;   // 30 Hz

// Feed `count` valid samples of a constant reading from `start_us`; returns the time of the last one. / 一定の読みの有効サンプルを count 個与える。
static uint32_t feed(TakeoffFloorWatch& watch, uint32_t start_us, int count, float distance_m, bool healthy = true, bool valid = true)
{
    uint32_t t = start_us;
    for (int i = 0; i < count; ++i) {
        watch.onSample(t, healthy, valid, distance_m);
        t += kSampleUs;
    }
    return t - kSampleUs;
}

TEST(floor_watch_needs_a_valid_reading_in_the_band_held_for_the_hold_time)
{
    TakeoffFloorWatch watch;
    watch.reset();
    ASSERT_TRUE(!watch.stays(10000000u, true));                          // nothing seen / 何も見ていない
    const uint32_t start = 1000000u;
    uint32_t last = feed(watch, start, 20, 0.04f);                        // 0.66 s: not yet / まだ
    ASSERT_TRUE(!watch.stays(last, true));
    last = feed(watch, last + kSampleUs, 20, 0.04f);                      // 1.3 s: held / 保持
    ASSERT_TRUE(watch.stays(last, true));
    ASSERT_TRUE(!watch.stays(last, false));                              // a ToF that is not healthy now: no evidence / ToF が今は健全でない: 証拠なし
    ASSERT_TRUE(watch.stays(start + kHoldUs, true));                     // boundary: exactly the hold time / 境界: ちょうど保持時間
    ASSERT_TRUE(!watch.stays(start + kHoldUs - 1u, true));
}

TEST(floor_watch_takes_no_evidence_from_invalid_samples_or_a_silent_tof)
{
    TakeoffFloorWatch invalid_only;
    invalid_only.reset();
    feed(invalid_only, 1000000u, 200, 0.0f, true, false);                // the real floor in the dead zone: invalid samples only / 不感帯の実際の床: 無効サンプルだけ
    ASSERT_TRUE(!invalid_only.stays(8000000u, true));
    TakeoffFloorWatch silent_tof;
    silent_tof.reset();
    feed(silent_tof, 1000000u, 100, 0.04f, false, true);                 // samples while the ToF is judged silent / ToF が出力停止と判定されている間のサンプル
    ASSERT_TRUE(!silent_tof.stays(5000000u, true));
}

TEST(floor_watch_invalid_samples_are_neutral_inside_a_run)
{
    TakeoffFloorWatch watch;
    watch.reset();
    uint32_t last = feed(watch, 1000000u, 10, 0.04f);
    last = feed(watch, last + kSampleUs, 10, 0.0f, true, false);         // a flicker of invalid samples / 無効サンプルのちらつき
    last = feed(watch, last + kSampleUs, 20, 0.04f);
    ASSERT_TRUE(watch.stays(last, true));                                // the run was not broken / 区間は切れていない
}

TEST(floor_watch_ends_when_the_reading_leaves_the_band_or_the_tof_is_not_healthy)
{
    TakeoffFloorWatch above;
    above.reset();
    uint32_t last = feed(above, 1000000u, 50, 0.04f);
    ASSERT_TRUE(above.stays(last, true));
    above.onSample(last + kSampleUs, true, true, kBandM + 0.01f);        // lifted above the band / 帯より上に浮いた
    ASSERT_TRUE(!above.stays(last + kSampleUs, true));
    last = feed(above, last + 2 * kSampleUs, 50, 0.04f);                 // a new run must hold again / 新しい区間はまた保持しなければならない
    ASSERT_TRUE(above.stays(last, true));

    TakeoffFloorWatch unhealthy;
    unhealthy.reset();
    last = feed(unhealthy, 1000000u, 50, 0.04f);
    unhealthy.onSample(last + kSampleUs, false, true, 0.04f);            // the ToF turned silent / ToF が出力停止になった
    ASSERT_TRUE(!unhealthy.stays(last + kSampleUs, true));
}

TEST(floor_watch_restarts_when_the_reading_rises_and_ignores_noise)
{
    // A craft that lifts slowly inside the band is rising, so it is not "on the floor": each rise by more than kRiseM above the lowest of the run restarts it. The steps are 0.022 m (> kRiseM = 0.02 m)
    // and every one of the three is fed, so the restart is tested three times. NOTE: on the REAL sensor (lowest valid reading 0.031 m) the band spans only 0.019 m < kRiseM, so this restart cannot
    // fire inside the band there (leaving the band is what ends the run); the test uses a sensor that reads closer (the SILS floor without the dead zone: 0.021 m), where it can.
    // The ToF noise (+-0.5 cm) never restarts it.
    // 帯の中でゆっくり浮く機体は上がっているので「床の上」ではない: 区間の最低値より kRiseM を超える上昇のたびに数え直す。段は 0.022 m（> kRiseM = 0.02 m）で、3 段すべてを与えるので、
    // 数え直しを 3 回試す。注: 「実センサ」（有効な最小の読み 0.031 m）では帯の幅が 0.019 m < kRiseM しかなく、帯の中でこの数え直しは起きない（区間を終わらせるのは帯を出ること）。試験は
    // もっと近くを読むセンサ（不感帯の無い SILS の床: 0.021 m）で行い、そこでは起きる。ToF ノイズ（±0.5 cm）では数え直さない。
    static_assert(kBandM - 0.031f < kRiseM, "the real sensor cannot rise by kRiseM inside the band");
    const float steps[] = {0.005f, 0.027f, 0.049f};
    int fed = 0;
    TakeoffFloorWatch rising;
    rising.reset();
    uint32_t t = 1000000u;
    for (float h : steps) {
        t = feed(rising, t, 25, h) + kSampleUs;                          // 0.8 s per step: never reaches the hold time within a run / 1 段 0.8 s: 区間の中では保持時間に届かない
        ASSERT_TRUE(!rising.stays(t, true));
        ++fed;
    }
    ASSERT_TRUE(fed == 3);
    TakeoffFloorWatch noisy;
    noisy.reset();
    t = 1000000u;
    for (int i = 0; i < 60; ++i) {
        noisy.onSample(t, true, true, 0.040f + ((i % 2) ? 0.005f : -0.005f));
        t += kSampleUs;
    }
    ASSERT_TRUE(noisy.stays(t, true));
}

TEST(floor_watch_latches_the_evidence_until_a_valid_reading_above_the_airborne_band)
{
    TakeoffFloorWatch watch;
    watch.reset();
    ASSERT_TRUE(!watch.seen());
    uint32_t last = feed(watch, 1000000u, 20, 0.04f);                      // 0.66 s: not held yet, no latch / まだ保持していない: ラッチなし
    ASSERT_TRUE(!watch.seen());
    last = feed(watch, last + kSampleUs, 20, 0.04f);                       // 1.3 s: held / 保持
    ASSERT_TRUE(watch.stays(last, true) && watch.seen());
    // One valid reading outside the band but below the airborne band ends the evidence NOW, not the latch: the takeoff must stay.
    // 帯の外だが空中の帯より下の有効な読み 1 つは、「今」の証拠を終わらせるが、ラッチは解かない: 離陸は完了してはならない。
    watch.onSample(last + kSampleUs, true, true, kBandM + 0.01f);
    ASSERT_TRUE(!watch.stays(last + kSampleUs, true) && watch.seen());
    watch.onSample(last + 2 * kSampleUs, true, true, kAirM);               // exactly the airborne band: not above it / ちょうど空中の帯: 超えてはいない
    ASSERT_TRUE(watch.seen());
    // The latch is not released by an invalid sample, a silent ToF, or time. / 無効サンプル、出力停止の ToF、時間では解けない。
    watch.onSample(last + 3 * kSampleUs, true, false, 0.0f);
    watch.onSample(last + 4 * kSampleUs, false, true, 0.30f);              // a reading while the ToF is judged silent does not count / 出力停止と判定されている間の読みは数えない
    watch.onSample(last + 60 * kSampleUs, true, false, 0.0f);
    ASSERT_TRUE(watch.seen());
    // Only a valid reading above the airborne band releases it. / 空中の帯より上の有効な読みだけが解く。
    watch.onSample(last + 61 * kSampleUs, true, true, kAirM + 0.01f);
    ASSERT_TRUE(!watch.seen() && !watch.stays(last + 61 * kSampleUs, true));
    // and a new run can latch it again; reset() clears it. / 新しい区間はまたラッチでき、reset() は消す。
    last = feed(watch, last + 62 * kSampleUs, 50, 0.04f);
    ASSERT_TRUE(watch.seen());
    watch.reset();
    ASSERT_TRUE(!watch.seen());
}

TEST(the_evidence_that_has_held_is_latched_when_only_invalid_samples_follow_it)
{
    // The hold time passes while the samples are invalid (the real floor reads invalid in the dead zone): stays() is true by the clock, so the latch must be set at that moment. Otherwise
    // the next reading outside the band, which ends the run, finds no latch and the TAKEOFF completes on the floor.
    // 保持時間が、サンプルが無効の間に過ぎる（実際の床は不感帯で無効を返す）: stays() は時計で真になるので、その時点でラッチしなければならない。そうしないと、区間を終わらせる
    // 次の帯の外の読みがラッチを見つけられず、床の上で TAKEOFF が完了する。
    TakeoffFloorWatch watch;
    watch.reset();
    const uint32_t start = 1000000u;
    uint32_t last = feed(watch, start, 10, 0.04f);                                           // 0.3 s of readings in the band / 帯の中の読み 0.3 s
    ASSERT_TRUE(!watch.seen());
    last = feed(watch, last + kSampleUs, 10, 0.0f, true, false);                             // 0.3 s of invalid samples: the hold time is not over / 無効サンプル 0.3 s: 保持時間はまだ
    ASSERT_TRUE(!watch.stays(last, true) && !watch.seen());
    last = feed(watch, last + kSampleUs, 30, 0.0f, true, false);                             // invalid samples past the hold time since the first reading / 最初の読みから保持時間を過ぎるまで無効サンプル
    ASSERT_TRUE(watch.stays(last, true));                                                    // by the clock / 時計で真
    ASSERT_TRUE(watch.seen());                                                               // and latched at that moment / その時点でラッチ
    watch.onSample(last + kSampleUs, true, true, kBandM + 0.01f);                            // one stray reading outside the band ends the run, not the latch / 帯の外へ外れた 1 つの読みは区間を終わらせるが、ラッチは解かない
    ASSERT_TRUE(!watch.stays(last + kSampleUs, true) && watch.seen());
    ASSERT_TRUE(decide(true, false, true, watch.stays(last + kSampleUs, true), 2000000u, watch.seen()) == TakeoffAction::Continue);   // not completed / 完了しない
    watch.onSample(last + 2 * kSampleUs, true, true, kAirM + 0.01f);                         // a reading above the airborne band: the craft really left / 空中の帯より上の読み: 本当に離れた
    ASSERT_TRUE(!watch.seen());

    // A silent ToF stops the clock of the evidence: nothing is latched by the time that passes while it is silent. / 出力停止の ToF は証拠の時計を止める: 出力停止の間に過ぎた時間では何もラッチしない。
    TakeoffFloorWatch silent;
    silent.reset();
    last = feed(silent, start, 10, 0.04f);
    last = feed(silent, last + kSampleUs, 60, 0.0f, false, false);
    ASSERT_TRUE(!silent.stays(last, true) && !silent.seen());
}

TEST(a_stray_reading_after_the_abort_time_does_not_complete_a_takeoff_on_the_floor)
{
    // The whole chain on the floor: the evidence holds past the abort time, the controller says "reached", one reading outside the band comes. Without the latch: FLYING on the floor.
    // 床の上での一連: 証拠が中止の時刻を過ぎて続き、制御器は「到達」と言い、帯の外へ外れた読みが 1 つ来る。ラッチが無ければ: 床の上で FLYING。
    TakeoffFloorWatch watch;
    watch.reset();
    uint32_t t = 1000000u;
    uint32_t last = feed(watch, t, 40, 0.04f);                            // 1.3 s of evidence (the hold is 1 s) / 1.3 s の証拠（保持は 1 s）
    ASSERT_TRUE(decide(true, false, true, watch.stays(last, true), 2000000u, watch.seen()) == TakeoffAction::Continue);   // held (reached, but the floor is shown) / 保持
    watch.onSample(last + kSampleUs, true, true, kBandM + 0.015f);        // one stray reading, the run ends / 1 つの外れた読み、区間は終わる
    const uint32_t after = last + kSampleUs;
    ASSERT_TRUE(decide(true, false, true, watch.stays(after, true), 2000000u, watch.seen()) == TakeoffAction::Continue);   // before the abort time: not completed, not aborted, held / 中止の時刻の前: 完了も中止もせず、保持
    ASSERT_TRUE(decide(true, false, true, watch.stays(after, true), kAbortUs + 1000000u, watch.seen()) == TakeoffAction::Land);   // after it: the latch that does not release ends the TAKEOFF by a LANDING, not FLYING / その後: 解けないラッチは、FLYING でなく LANDING で TAKEOFF を終わらせる
    watch.onSample(after + kSampleUs, true, true, 0.20f);                  // a valid reading above 0.15 m: the craft really left / 0.15 m を超える有効な読み: 本当に離れた
    ASSERT_TRUE(decide(true, false, true, watch.stays(after, true), kAbortUs + 1000000u, watch.seen()) == TakeoffAction::Complete);
}

TEST(the_latch_is_released_by_the_hold_time_above_the_ground_band_and_not_before)
{
    // A craft that hangs between the ground band (5 cm) and the airborne band (15 cm), e.g. in the ground effect, must not be held for ever: valid readings above the ground band for the hold
    // time (the weight of evidence the abort needs of staying) release the latch. The ORDER the decision sees: held, held, then completed.
    // 地面効果などで、帯（5 cm）と空中の帯（15 cm）の間に留まる機体は、永久には保持されない: 帯より上の有効な読みが保持時間続くと（中止が「留まる」に要るのと同じ重さの証拠）ラッチが解ける。
    // 判断が見る「順序」: 保持、保持、そして完了。
    TakeoffFloorWatch watch;
    watch.reset();
    uint32_t last = feed(watch, 1000000u, 40, 0.04f);                      // the evidence, latched / 証拠、ラッチ
    ASSERT_TRUE(watch.seen());
    const uint32_t above_start = last + kSampleUs;
    last = feed(watch, above_start, 20, 0.08f);                            // 0.66 s above the band, below the airborne band: still latched / 帯より上 0.66 s、空中の帯より下: まだラッチ
    ASSERT_TRUE(watch.seen() && !watch.stays(last, true));
    ASSERT_TRUE(decide(true, false, true, watch.stays(last, true), 3000000u, watch.seen()) == TakeoffAction::Continue);
    watch.onSample(above_start + kHoldUs - 1u, true, true, 0.08f);         // one microsecond short of the hold time / 保持時間の 1 マイクロ秒手前
    ASSERT_TRUE(watch.seen());
    watch.onSample(above_start + kHoldUs, true, true, 0.08f);              // exactly the hold time / ちょうど保持時間
    ASSERT_TRUE(!watch.seen());
    ASSERT_TRUE(decide(true, false, true, watch.stays(above_start + kHoldUs, true), 3000000u, watch.seen()) == TakeoffAction::Complete);
}

TEST(the_run_above_the_ground_band_is_ended_by_an_in_band_reading_or_a_silent_tof_and_ignores_invalid_samples)
{
    // An in-band reading ends the run above the band (the craft is back on the floor): the clock starts again. A silent ToF ends it. An invalid sample is neutral (neither extends nor ends it).
    // 帯の中の読みは、帯より上の区間を終わらせる（機体は床に戻った）: 時計は最初から。出力停止の ToF も終わらせる。無効サンプルは中立（延ばさず、終わらせない）。
    for (int ending = 0; ending < 2; ++ending) {
        TakeoffFloorWatch watch;
        watch.reset();
        uint32_t last = feed(watch, 1000000u, 40, 0.04f);
        ASSERT_TRUE(watch.seen());
        last = feed(watch, last + kSampleUs, 20, 0.08f);                   // 0.66 s above the band / 帯より上 0.66 s
        if (ending == 0) {
            watch.onSample(last + kSampleUs, true, true, 0.04f);           // back in the band / 帯の中へ戻る
        } else {
            watch.onSample(last + kSampleUs, false, true, 0.08f);          // the ToF turned silent / ToF が出力停止
        }
        last = feed(watch, last + 2 * kSampleUs, 20, 0.08f);               // another 0.66 s above: the total is more than the hold time, the run is not / さらに 0.66 s: 合計は保持時間を超えるが、区間は超えない
        ASSERT_TRUE(watch.seen());
        last = feed(watch, last + kSampleUs, 30, 0.08f);                   // 1.0 s more of the NEW run / 新しい区間があと 1.0 s
        ASSERT_TRUE(!watch.seen());
    }
    TakeoffFloorWatch invalid;
    invalid.reset();
    uint32_t last = feed(invalid, 1000000u, 40, 0.04f);
    const uint32_t above_start = last + kSampleUs;
    feed(invalid, above_start, 5, 0.08f);
    invalid.onSample(above_start + kHoldUs + 500000u, true, false, 0.0f);  // an invalid sample after the hold time: it cannot release / 保持時間の後の無効サンプル: 解けない
    ASSERT_TRUE(invalid.seen());
    invalid.onSample(above_start + kHoldUs + 500000u + kSampleUs, true, true, 0.08f);   // the next valid reading above the band: the run has lasted / 次の帯より上の有効な読み: 区間は続いていた
    ASSERT_TRUE(!invalid.seen());
}

TEST(the_run_above_the_ground_band_is_reset_with_the_watch_and_survives_a_wrap_of_the_clock)
{
    TakeoffFloorWatch watch;
    watch.reset();
    feed(watch, 0xFFFFF000u, 60, 0.04f);                                   // the latch, near the wrap / 巻き戻りの近くでラッチ
    ASSERT_TRUE(watch.seen());
    uint32_t last = feed(watch, 0xFFFFF000u + 61u * kSampleUs, 20, 0.08f); // above the band across the wrap (0.66 s) / 巻き戻りをまたいで帯より上（0.66 s）
    ASSERT_TRUE(watch.seen());
    last = feed(watch, last + kSampleUs, 15, 0.08f);                       // past the hold time across the wrap / 巻き戻りをまたいで保持時間を超える
    ASSERT_TRUE(!watch.seen());
    TakeoffFloorWatch fresh;
    fresh.reset();
    feed(fresh, 1000000u, 40, 0.04f);
    feed(fresh, 3000000u, 20, 0.08f);                                      // 0.66 s above / 帯より上 0.66 s
    fresh.reset();                                                         // forgets the run above the band, too / 帯より上の区間も忘れる
    feed(fresh, 4000000u, 40, 0.04f);                                      // latched again / またラッチ
    ASSERT_TRUE(fresh.seen());
    feed(fresh, 5000000u, 20, 0.08f);                                      // a fresh 0.66 s: not the 0.66 s + 0.66 s of before / 新しい 0.66 s: 以前の 0.66 s との合計ではない
    ASSERT_TRUE(fresh.seen());
}

TEST(readings_that_straddle_the_ground_band_end_the_takeoff_by_a_landing_never_by_flying_on_the_floor)
{
    // A latched floor evidence and then readings that alternate across the band edge (kBandM, 5 cm) in runs shorter than the hold time: neither the abort (it needs 1 s of evidence NOW) nor the release
    // of the latch (it needs 1 s above the band) is ever reached. Per sample: the watch is fed, then the decision is taken with the controller reporting "reached" and the time since the TAKEOFF began.
    // The alternation is every sample, and every 5, 15, 29 and 31 samples (a run of n samples lasts (n - 1) x 33 ms from its first to its last sample: 31 samples = 0.99 s, the longest below the hold time): the decision
    // must stay "continue" until the abort time and then be a LANDING, never the completion. With runs of 32 samples (1.02 s, above the hold time) the side above the band reaches its 1 s, releases the latch and the TAKEOFF
    // completes before the abort time.
    // 床の証拠がラッチされ、その後、読みが帯の縁（kBandM、5 cm）をまたいで、保持時間より短い区間で交互に来る: 中止（「今」の証拠 1 s を要する）にも、ラッチの解除（帯より上 1 s を要する）にも届かない。
    // サンプルごとに、ウォッチに与え、制御器が「到達」と言っている状態で、TAKEOFF の開始からの時間とともに判断する。交互は毎サンプル、5・15・29・31 サンプルごと（n サンプルの区間は最初から最後のサンプルまで
    // (n − 1) × 33 ms: 31 サンプル = 0.99 s が、保持時間未満で最長）: 判断は中止の時刻まで「継続」のままで、その後は LANDING で、完了には決してならない。32 サンプル（1.02 s、保持時間を超える）の区間では、
    // 帯より上の側が 1 s に届いてラッチが解け、TAKEOFF は中止の時刻より前に完了する。
    static_assert(30u * kSampleUs < kHoldUs && 31u * kSampleUs > kHoldUs, "the periods straddle the hold time");
    const float lo = 0.045f;   // in the band / 帯の中
    const float hi = 0.060f;   // above the band, below the airborne band / 帯より上、空中の帯より下
    for (int period : {1, 5, 15, 29, 31, 32}) {
        TakeoffFloorWatch watch;
        watch.reset();
        uint32_t t = 0;
        for (int i = 0; i < 45; ++i) {   // 1.5 s on the floor: the evidence holds and latches / 床の上 1.5 s: 証拠が立ちラッチする
            watch.onSample(t, true, true, lo);
            t += kSampleUs;
        }
        ASSERT_TRUE(watch.seen());
        const uint32_t began_us = 0;   // the TAKEOFF began with the first sample / TAKEOFF は最初のサンプルで始まった
        TakeoffAction first_end = TakeoffAction::Continue;
        uint32_t end_us = 0;
        for (int i = 0; i < 30 * 30 && first_end == TakeoffAction::Continue; ++i) {   // 30 s of samples / 30 s 分のサンプル
            const float reading = ((i / period) % 2) ? hi : lo;
            watch.onSample(t, true, true, reading);
            const uint32_t elapsed_us = t - began_us;
            const TakeoffAction action = decide(true, true, true, watch.stays(t, true), elapsed_us, watch.seen(), watch.leaving());
            if (action != TakeoffAction::Continue) {
                first_end = action;
                end_us = elapsed_us;
            }
            t += kSampleUs;
        }
        ASSERT_TRUE(first_end != TakeoffAction::Continue);                  // the TAKEOFF ends / TAKEOFF は終わる
        if (period <= 31) {
            ASSERT_TRUE(first_end == TakeoffAction::Land);                  // neither side reaches its 1 s: the LANDING, at the first in-band reading from the abort time / どちら側も 1 s に届かない: LANDING で、中止の時刻からの最初の帯の中の読み
            ASSERT_TRUE(end_us >= kAbortUs && end_us <= kAbortUs + 32u * kSampleUs);   // at the first in-band reading from the abort time on (a run above the band in progress at the abort time holds it back for less than the hold time) / 中止の時刻からの最初の帯の中の読みで（中止の時刻に帯より上の区間が進行中なら、保持時間未満だけ待つ）
        } else {
            ASSERT_TRUE(first_end == TakeoffAction::Complete);              // 1 s above the band: the latch is released, the takeoff completes (and is not held to the abort time) / 帯より上に 1 s: ラッチが解け、離陸は完了する（中止の時刻まで保持されない）
            ASSERT_TRUE(end_us < kAbortUs);
        }
    }
}

TEST(leaving_is_a_run_above_the_ground_band_that_ends_with_an_in_band_reading_a_silent_tof_or_a_reset)
{
    TakeoffFloorWatch watch;
    watch.reset();
    ASSERT_TRUE(!watch.leaving());
    uint32_t last = feed(watch, 1000000u, 40, 0.04f);                      // on the floor: not leaving / 床の上: 離れつつない
    ASSERT_TRUE(!watch.leaving());
    last = feed(watch, last + kSampleUs, 3, 0.08f);                        // valid readings above the band: leaving, long before the hold time / 帯より上の有効な読み: 離れつつある（保持時間よりずっと前から）
    ASSERT_TRUE(watch.leaving() && watch.seen());
    watch.onSample(last + kSampleUs, true, false, 0.0f);                   // an invalid sample is neutral / 無効サンプルは中立
    ASSERT_TRUE(watch.leaving());
    watch.onSample(last + 2 * kSampleUs, true, true, 0.04f);               // back in the band / 帯の中へ戻る
    ASSERT_TRUE(!watch.leaving());
    feed(watch, last + 3 * kSampleUs, 3, 0.08f);
    ASSERT_TRUE(watch.leaving());
    watch.onSample(last + 10 * kSampleUs, false, true, 0.08f);             // the ToF turned silent / ToF が出力停止
    ASSERT_TRUE(!watch.leaving());
    feed(watch, last + 11 * kSampleUs, 3, 0.08f);
    ASSERT_TRUE(watch.leaving());
    watch.reset();
    ASSERT_TRUE(!watch.leaving());
}

TEST(floor_watch_reset_forgets_the_run_and_the_wrap_of_the_clock_is_harmless)
{
    TakeoffFloorWatch watch;
    watch.reset();
    uint32_t last = feed(watch, 0xFFFFF000u, 60, 0.04f);                 // the clock wraps inside the run / 区間の途中で時計が巻き戻る
    ASSERT_TRUE(watch.stays(last, true));
    watch.reset();
    ASSERT_TRUE(!watch.stays(last, true));
}

int main()
{
    printf("=== takeoff_policy host tests ===\n");
    run_manual_takeoff_completes_when_the_tof_says_airborne_and_never_aborts();
    run_auto_takeoff_completes_when_captured_and_the_floor_is_not_shown();
    run_auto_takeoff_with_the_floor_shown_stays_and_is_aborted_after_the_abort_time();
    run_a_slow_lift_is_never_aborted_and_the_missing_evidence_is_not_the_floor();
    run_the_latched_floor_evidence_blocks_the_completion_but_never_causes_an_abort();
    run_a_landing_is_decided_only_by_a_latch_without_the_evidence_now_after_the_abort_time_in_an_auto_takeoff();
    run_a_latched_takeoff_is_not_landed_while_the_craft_is_lifting_off();
    run_abort_time_is_longer_than_the_controller_backstop_and_the_ordinary_takeoff();
    run_comm_loss_lands_from_takeoff_and_flying_only_after_the_grace();
    run_a_touchdown_verdict_counts_only_with_the_stamp_of_the_present_landing();
    run_floor_watch_needs_a_valid_reading_in_the_band_held_for_the_hold_time();
    run_floor_watch_takes_no_evidence_from_invalid_samples_or_a_silent_tof();
    run_floor_watch_invalid_samples_are_neutral_inside_a_run();
    run_floor_watch_ends_when_the_reading_leaves_the_band_or_the_tof_is_not_healthy();
    run_floor_watch_restarts_when_the_reading_rises_and_ignores_noise();
    run_floor_watch_latches_the_evidence_until_a_valid_reading_above_the_airborne_band();
    run_the_evidence_that_has_held_is_latched_when_only_invalid_samples_follow_it();
    run_a_stray_reading_after_the_abort_time_does_not_complete_a_takeoff_on_the_floor();
    run_the_latch_is_released_by_the_hold_time_above_the_ground_band_and_not_before();
    run_the_run_above_the_ground_band_is_ended_by_an_in_band_reading_or_a_silent_tof_and_ignores_invalid_samples();
    run_the_run_above_the_ground_band_is_reset_with_the_watch_and_survives_a_wrap_of_the_clock();
    run_readings_that_straddle_the_ground_band_end_the_takeoff_by_a_landing_never_by_flying_on_the_floor();
    run_leaving_is_a_run_above_the_ground_band_that_ends_with_an_in_band_reading_a_silent_tof_or_a_reset();
    run_floor_watch_reset_forgets_the_run_and_the_wrap_of_the_clock_is_harmless();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
