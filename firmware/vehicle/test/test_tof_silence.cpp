/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_tof_silence.cpp
 * @brief Host tests for the downward-ToF silence judgement and the state machine's response
 *        下向き ToF の出力停止判定と、状態機械の応答のホスト試験
 *
 * Two pure pieces, no ESP-IDF / FreeRTOS:
 *  1. sf::SilenceWatch (the judgement StateManager uses) with sf::limits::kTofSilenceUs (threshold boundary, signed
 *     time difference, uint32 wrap, never-seen, and "out of range is NOT silent").
 *  2. sf::decideToFSilenceAction / sf::toFSilenceRefusesMode — every FlightState x FlightMode x
 *     silent cell of the normative table (detailed_design.md §3.1, INV-4), including the
 *     blind-landing time and the ACRO/STABILIZE "warn, never land" rule.
 * Own TEST/ASSERT framework and main(), same pattern as test_sensor_freshness.cpp.
 *
 * 純粋な 2 つの部品で、ESP-IDF・FreeRTOS 不要:
 *  1. sf::limits::kTofSilenceUs を使う sf::SilenceWatch（StateManager が使う判定。しきい値境界、符号付き時刻差、
 *     uint32 の巻き戻り、一度も来ていない場合、「範囲外は出力停止ではない」）。
 *  2. sf::decideToFSilenceAction / sf::toFSilenceRefusesMode — 規範表（detailed_design.md §3.1、
 *     INV-4）の FlightState × FlightMode × 出力停止の全セル（ToF なしの着陸の時間と、
 *     ACRO/STABILIZE の「警告のみ・着陸させない」規則を含む）。
 * test_sensor_freshness.cpp と同じ自前の TEST/ASSERT フレームワークと main()。
 *
 * @design architecture.md INV-4 — every (state x input) cell of the table is checked  [OK]
 * @design detailed_design.md §3.1 — ToF silent rows                                     [OK]
 */

#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "flight_limits.hpp"
#include "flight_state.hpp"
#include "sensor_freshness.hpp"
#include "tof_silence_policy.hpp"

using sf::FlightState;
using sf::ToFSilenceAction;

static constexpr uint32_t kSilenceUs = sf::limits::kTofSilenceUs;   // 0.3 s / 候補値

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

static bool silent(uint32_t now_us, uint32_t last_sample_us)
{
    sf::SilenceWatch watch;                      // one observation = a stateless judgement
    watch.observe(now_us, last_sample_us);       // 観測 1 回 = 状態を持たない判定
    return watch.isSilent(kSilenceUs);
}

// ----------------------------------------------------------------------------
// 1. Silence judgement / 出力停止判定
// ----------------------------------------------------------------------------

TEST(threshold_boundary)
{
    // age == threshold is silent (complement of the strict "<" freshness test);
    // threshold - 1 us is still alive.
    // age == しきい値 は出力停止（厳密な "<" の鮮度判定の補集合）。しきい値 - 1 us はまだ動作している。
    const uint32_t now_us = 50000000u;
    ASSERT_TRUE(!silent(now_us, now_us - (kSilenceUs - 1)));
    ASSERT_TRUE(silent(now_us, now_us - kSilenceUs));
    ASSERT_TRUE(silent(now_us, now_us - (kSilenceUs + 1)));
}

TEST(normal_sample_interval_is_not_silent)
{
    // 30 Hz ToF: the largest interval measured in flight logs is 99 ms; far below 0.3 s.
    // 30 Hz の ToF: 飛行ログで測った最大間隔は 99 ms。0.3 s よりずっと小さい。
    ASSERT_TRUE(!silent(10000000u, 10000000u - 33000u));
    ASSERT_TRUE(!silent(10000000u, 10000000u - 99000u));
}

TEST(timestamp_newer_than_now_is_not_silent)
{
    // Read-order inversion across tasks: the ToF stamp can be newer than "now". The
    // difference is signed, so this is alive (the unsigned form would be ~4.29e9 = silent).
    // タスク間の読み出し順序の逆転: ToF の時刻印が「現在」より新しいことがある。差は
    // 符号付きなので動作している（符号なしだと ~4.29e9 で出力停止と誤判定される）。
    ASSERT_TRUE(!silent(10000000u, 10000001u));
    ASSERT_TRUE(!silent(10000000u, 10000005u));
    ASSERT_TRUE(!silent(10000000u, 10003000u));
}

TEST(never_seen_is_silent)
{
    // Timestamp 0 = no sample since boot: treated as silent (ARM refused until the first one).
    // 時刻印 0 = 起動後サンプル無し: 出力停止とみなす（最初の 1 個が来るまで ARM 拒否）。
    ASSERT_TRUE(silent(10000000u, 0u));
    ASSERT_TRUE(silent(100u, 0u));
}

TEST(uint32_wraparound_cases)
{
    // now just after the 2^32 wrap, sample just before it.
    // now が 2^32 の巻き戻り直後、サンプルがその直前。
    ASSERT_TRUE(!silent(100u, 0xFFFFFF00u));                     // age 356 us, alive
    ASSERT_TRUE(silent(kSilenceUs - 1u, 0xFFFFFFFFu));           // age == threshold -> silent
    ASSERT_TRUE(!silent(kSilenceUs - 2u, 0xFFFFFFFFu));          // age == threshold - 1 -> alive
    // Newer than now across the wrap.
    // 巻き戻りをまたいだ「新しい」時刻印。
    ASSERT_TRUE(!silent(0xFFFFFFFEu, 3u));
}

TEST(out_of_range_samples_are_not_silent)
{
    // 2 s of samples at 30 Hz that all say "out of range" (status != 0): the function only
    // sees WHEN samples arrive, so the sensor is alive at every tick. The status never
    // reaches it — out-of-range is a different judgement.
    // 2 s ぶんの 30 Hz サンプルがすべて「範囲外」（status != 0）: この関数はサンプルが「いつ」
    // 届くかだけを見るので、どの時点でも動作している。status はここに届かない — 範囲外は別の判定。
    const uint32_t period_us = 33333u;
    uint32_t last_sample_us = 1000000u;
    for (uint32_t now_us = 1000000u; now_us < 3000000u; now_us += 1000u) {
        if (now_us - last_sample_us >= period_us) {
            last_sample_us = now_us;   // a sample arrives (its status is irrelevant here)
        }
        ASSERT_TRUE(!silent(now_us, last_sample_us));
    }
}

TEST(stream_that_stops_is_silent_after_threshold)
{
    // Samples at 30 Hz until t = 2 s, then nothing: silent exactly kSilenceUs after the last.
    // 2 s まで 30 Hz でサンプル、以後は無し: 最後のサンプルからちょうど kSilenceUs で出力停止。
    const uint32_t last_sample_us = 2000000u;
    ASSERT_TRUE(!silent(last_sample_us + kSilenceUs - 1u, last_sample_us));
    ASSERT_TRUE(silent(last_sample_us + kSilenceUs, last_sample_us));
    ASSERT_TRUE(silent(last_sample_us + 5000000u, last_sample_us));
}

TEST(tof_silent_for_more_than_36_minutes_stays_silent)
{
    // R1: Failsafe observes the ToF stamp at 10 Hz. After the last sample the
    // watch must say "silent" from 0.3 s on and never go back to "alive" — not at 35.8 min
    // (signed 32-bit difference turns negative) and not at the 71.6 min uint32 wrap.
    // R1: Failsafe は ToF の時刻印を 10 Hz で観測する。最後のサンプルの後、0.3 s
    // 以降は「停止」で、35.8 分（符号付き 32bit 差が負になる）でも 71.6 分の uint32 巻き戻りでも
    // 「動作」に戻らないこと。
    const uint32_t last_sample_us = 3000000000u;
    const uint32_t period_us = 10000u;
    sf::SilenceWatch watch;
    uint32_t now_us = last_sample_us;
    watch.observe(now_us, last_sample_us);
    ASSERT_TRUE(!watch.isSilent(kSilenceUs));
    bool alive_again = false;
    const uint64_t total_us = 75ull * 60ull * 1000000ull;   // 75 min: wraps uint32 / 巻き戻りあり
    for (uint64_t elapsed = period_us; elapsed <= total_us; elapsed += period_us) {
        now_us = static_cast<uint32_t>(last_sample_us + elapsed);
        watch.observe(now_us, last_sample_us);
        if (elapsed >= kSilenceUs && !watch.isSilent(kSilenceUs)) {
            alive_again = true;
        }
        if (elapsed < kSilenceUs && watch.isSilent(kSilenceUs)) {
            alive_again = true;   // silent too early would also be wrong / 早すぎる停止も誤り
        }
    }
    ASSERT_TRUE(!alive_again);
    ASSERT_TRUE(watch.isSilent(kSilenceUs));
}

TEST(no_sample_since_boot_is_silent_for_any_duration)
{
    // Right after boot (stamp 0) the ToF counts as silent however long it lasts, so ARM stays
    // refused until the first sample.
    // 起動直後（時刻印 0）は、どれだけ続いても出力停止とみなす。最初のサンプルまで ARM 拒否のまま。
    sf::SilenceWatch watch;
    for (uint32_t now_us = 0u; now_us < 4000000000u; now_us += 20000000u) {
        watch.observe(now_us, 0u);
        ASSERT_TRUE(watch.isSilent(kSilenceUs));
    }
}

TEST(threshold_is_three_times_the_largest_measured_interval)
{
    // Basis check of the candidate value: >= 3 x 99 ms (flight-log maximum), < 2^31 us.
    // 候補値の根拠の確認: 99 ms（飛行ログの最大）の 3 倍以上、かつ 2^31 us 未満。
    ASSERT_TRUE(kSilenceUs >= 3u * 99000u);
    ASSERT_TRUE(kSilenceUs < 0x80000000u);
}

// ----------------------------------------------------------------------------
// 2. State machine response / 状態機械の応答
// ----------------------------------------------------------------------------

using sf::FlightMode;

static constexpr uint32_t kBlindUs = sf::limits::kBlindLandingTimeoutUs;
static constexpr uint32_t kMinSilentUs = sf::limits::kBlindLandingMinSilentUs;

// `silent_elapsed_us` defaults to the minimum guaranteed time, so that the LANDING limit alone decides unless a test says otherwise.
// `silent_elapsed_us` の既定は保証する最低時間: テストが指定しない限り、LANDING の上限だけが判断する。
static ToFSilenceAction act(FlightState state, FlightMode mode, bool is_silent,
                            uint32_t blind_elapsed_us = 0, uint32_t silent_elapsed_us = kMinSilentUs)
{
    return sf::decideToFSilenceAction(state, mode, is_silent, blind_elapsed_us, silent_elapsed_us);
}

static constexpr FlightMode kAllModes[] = {FlightMode::ACRO, FlightMode::STABILIZE,
                                           FlightMode::ALT_HOLD, FlightMode::POS_HOLD};

TEST(not_silent_never_acts_in_any_state_or_mode)
{
    for (int i = 0; i < sf::FLIGHT_STATE_COUNT; ++i) {
        for (FlightMode mode : kAllModes) {
            ASSERT_TRUE(act(static_cast<FlightState>(i), mode, false, kBlindUs * 2) ==
                        ToFSilenceAction::None);
        }
    }
}

TEST(armed_on_ground_disarms_in_every_mode)
{
    for (FlightMode mode : kAllModes) {
        ASSERT_TRUE(act(FlightState::ARMED_GROUND, mode, true) == ToFSilenceAction::Disarm);
    }
}

TEST(altitude_modes_start_landing_in_takeoff_and_flying)
{
    for (FlightState state : {FlightState::TAKEOFF, FlightState::FLYING}) {
        ASSERT_TRUE(act(state, FlightMode::ALT_HOLD, true) == ToFSilenceAction::StartLanding);
        ASSERT_TRUE(act(state, FlightMode::POS_HOLD, true) == ToFSilenceAction::StartLanding);
    }
}

TEST(manual_modes_only_warn_never_land)
{
    // The pilot owns the vertical axis in ACRO / STABILIZE: no forced landing, a warning only.
    // ACRO / STABILIZE では鉛直軸はパイロットのもの: 強制着陸せず、警告のみ。
    for (FlightState state : {FlightState::TAKEOFF, FlightState::FLYING}) {
        ASSERT_TRUE(act(state, FlightMode::ACRO, true) == ToFSilenceAction::WarnOnly);
        ASSERT_TRUE(act(state, FlightMode::STABILIZE, true) == ToFSilenceAction::WarnOnly);
    }
}

TEST(landing_completes_only_after_the_blind_time_in_every_mode)
{
    for (FlightMode mode : kAllModes) {
        ASSERT_TRUE(act(FlightState::LANDING, mode, true, 0) == ToFSilenceAction::None);
        ASSERT_TRUE(act(FlightState::LANDING, mode, true, kBlindUs - 1u) == ToFSilenceAction::None);
        ASSERT_TRUE(act(FlightState::LANDING, mode, true, kBlindUs) == ToFSilenceAction::CompleteLanding);
        ASSERT_TRUE(act(FlightState::LANDING, mode, true, kBlindUs + 1u) == ToFSilenceAction::CompleteLanding);
    }
}

TEST(landing_after_a_long_live_tof_gets_the_minimum_silent_time_before_the_motors_are_cut)
{
    // A LANDING with a live ToF that lasted longer than the limit, and then the ToF dies: the limit alone would cut the motors at once, in the air.
    // The time limit needs BOTH the LANDING time and the continuous silent time.
    // ToF が生きたまま上限より長く続いた LANDING の後に ToF が死ぬ: 上限だけなら、空中で即座にモータを切る。時間切れには
    // LANDING の時間と連続した出力停止の時間の「両方」が要る。
    for (FlightMode mode : kAllModes) {
        ASSERT_TRUE(act(FlightState::LANDING, mode, true, kBlindUs + 5000000u, 0u) == ToFSilenceAction::None);
        ASSERT_TRUE(act(FlightState::LANDING, mode, true, kBlindUs + 5000000u, kMinSilentUs - 1u) == ToFSilenceAction::None);
        ASSERT_TRUE(act(FlightState::LANDING, mode, true, kBlindUs + 5000000u, kMinSilentUs) == ToFSilenceAction::CompleteLanding);
        // the silent time alone is not enough either / 出力停止の時間だけでも足りない
        ASSERT_TRUE(act(FlightState::LANDING, mode, true, kBlindUs - 1u, kMinSilentUs * 3u) == ToFSilenceAction::None);
    }
}

TEST(flip_defers_to_flying)
{
    // The alert is re-raised while silent, so the response applies once FLIP returns to FLYING.
    // 出力停止の間はアラートが再発報されるので、FLIP が FLYING に戻った時点で応答が適用される。
    for (FlightMode mode : kAllModes) {
        ASSERT_TRUE(act(FlightState::FLIP, mode, true) == ToFSilenceAction::None);
    }
}

TEST(ground_states_do_nothing_here)
{
    // Refusal of ARM is the pre-arm gate (ArmBlock::TofSilent), not an action here.
    // ARM の拒否は ARM 前判定（ArmBlock::TofSilent）で、ここでの動作ではない。
    for (FlightMode mode : kAllModes) {
        ASSERT_TRUE(act(FlightState::INIT, mode, true) == ToFSilenceAction::None);
        ASSERT_TRUE(act(FlightState::IDLE_GROUND, mode, true) == ToFSilenceAction::None);
        ASSERT_TRUE(act(FlightState::IDLE_HELD, mode, true) == ToFSilenceAction::None);
    }
}

TEST(altitude_modes_are_refused_while_silent)
{
    // Switching INTO ALT_HOLD / POS_HOLD is refused while silent; ACRO / STABILIZE never are,
    // and nothing is refused while the ToF is alive.
    // 出力停止中は ALT_HOLD / POS_HOLD への切り替えを拒否する。ACRO / STABILIZE は拒否しない。
    // ToF が動作していれば何も拒否しない。
    ASSERT_TRUE(sf::toFSilenceRefusesMode(FlightMode::ALT_HOLD, true));
    ASSERT_TRUE(sf::toFSilenceRefusesMode(FlightMode::POS_HOLD, true));
    ASSERT_TRUE(!sf::toFSilenceRefusesMode(FlightMode::ACRO, true));
    ASSERT_TRUE(!sf::toFSilenceRefusesMode(FlightMode::STABILIZE, true));
    for (FlightMode mode : kAllModes) {
        ASSERT_TRUE(!sf::toFSilenceRefusesMode(mode, false));
    }
}

TEST(landing_time_is_counted_from_the_entry_into_landing_and_not_extended_by_a_returning_tof)
{
    // The clock runs from the entry into LANDING whether the ToF is silent or alive, and restarts only after LANDING ended:
    // a ToF that returns for a while and stops again (silent -> alive -> silent) does not extend the time limit.
    // 時計は LANDING に入った時点から、ToF が出力停止でも生きていても進み、LANDING が終わった後でだけ数え直す: ToF が
    // しばらく戻ってまた止まっても（出力停止 → 生存 → 出力停止）、時間切れは延びない。
    sf::LandingClock clock;
    uint32_t now = 1000000u;
    clock.update(false, false, now);
    ASSERT_TRUE(clock.elapsedUs() == 0u);
    const uint32_t entry = now;
    clock.update(true, true, now);                    // the first update in LANDING: the entry / LANDING での最初の更新: 入った時点
    for (int i = 0; i < 4; ++i) {                     // 4 s: ToF silent, then alive for 1.5 s, silent again / 4 s: 出力停止 → 生存 1.5 s → 出力停止
        now += 1000000u;
        clock.update(true, true, now);
    }
    ASSERT_TRUE(clock.elapsedUs() == now - entry);
    const uint32_t before = clock.elapsedUs();
    now += 6100000u;                                   // 6.1 s more: the time limit (10 s) is reached whatever the ToF did / さらに 6.1 s: ToF が何でも時間切れ
    clock.update(true, true, now);
    ASSERT_TRUE(clock.elapsedUs() == before + 6100000u);
    ASSERT_TRUE(clock.silentElapsedUs() == clock.elapsedUs());   // silent since the entry / 入ってからずっと出力停止
    ASSERT_TRUE(sf::decideToFSilenceAction(sf::FlightState::LANDING, sf::FlightMode::ALT_HOLD, true, clock.elapsedUs(),
                                           clock.silentElapsedUs()) == sf::ToFSilenceAction::CompleteLanding);
    now += 1000u;
    clock.update(false, true, now);                    // LANDING ended: cleared / LANDING が終わる: クリア
    ASSERT_TRUE(clock.elapsedUs() == 0u && clock.silentElapsedUs() == 0u);
    now += 1000u;
    clock.update(true, true, now);                     // the next LANDING starts from zero / 次の LANDING はゼロから
    ASSERT_TRUE(clock.elapsedUs() == 0u && clock.silentElapsedUs() == 0u);
    now += 500000u;
    clock.update(true, true, now);
    ASSERT_TRUE(clock.elapsedUs() == 500000u && clock.silentElapsedUs() == 500000u);
    // across a 32-bit wrap of the clock / 32 ビット時計の巻き戻りをまたぐ
    sf::LandingClock wrapped;
    wrapped.update(true, true, 0xFFFFFF00u);
    wrapped.update(true, true, 0x00000100u);
    ASSERT_TRUE(wrapped.elapsedUs() == 0x200u && wrapped.silentElapsedUs() == 0x200u);
}

TEST(landing_clock_counts_the_continuous_silence_of_this_landing)
{
    // A live ToF for 12 s in LANDING, then the ToF dies: the silent time starts at the silence (not at the entry) and restarts if the ToF returns and stops again.
    // LANDING で 12 s ToF が生きていて、その後に ToF が死ぬ: 出力停止の時間は出力停止から（入った時点からでなく）数え、ToF が戻ってまた止まれば数え直す。
    sf::LandingClock clock;
    uint32_t now = 5000000u;
    clock.update(true, false, now);
    now += 12000000u;
    clock.update(true, false, now);
    ASSERT_TRUE(clock.elapsedUs() == 12000000u && clock.silentElapsedUs() == 0u);
    clock.update(true, true, now);                     // the ToF dies / ToF が死ぬ
    ASSERT_TRUE(clock.silentElapsedUs() == 0u);
    now += 2000000u;
    clock.update(true, true, now);
    ASSERT_TRUE(clock.elapsedUs() == 14000000u && clock.silentElapsedUs() == 2000000u);
    ASSERT_TRUE(sf::decideToFSilenceAction(sf::FlightState::LANDING, sf::FlightMode::ALT_HOLD, true, clock.elapsedUs(),
                                           clock.silentElapsedUs()) == sf::ToFSilenceAction::None);   // 2 s of the minimum silent time / 最低の出力停止時間のうち 2 s
    now += 500000u;
    clock.update(true, false, now);                    // the ToF returns: the silent time is cleared / ToF が戻る: 出力停止の時間はクリア
    ASSERT_TRUE(clock.silentElapsedUs() == 0u);
    now += 500000u;
    clock.update(true, true, now);                     // and dies again: restarts / また死ぬ: 数え直す
    now += kMinSilentUs;
    clock.update(true, true, now);
    ASSERT_TRUE(clock.silentElapsedUs() == kMinSilentUs);
    ASSERT_TRUE(sf::decideToFSilenceAction(sf::FlightState::LANDING, sf::FlightMode::ALT_HOLD, true, clock.elapsedUs(),
                                           clock.silentElapsedUs()) == sf::ToFSilenceAction::CompleteLanding);
}

TEST(blind_landing_limits_are_consistent)
{
    // Basis check of the candidates: the late fall must lower the thrust (rate > 0) and start before the time
    // limit, the time limit must be positive and below the 2^31 us signed-difference range, and the impact
    // threshold of the touchdown must sit between 1 g and the 3 G crash alert (the stillness test, not the
    // threshold, separates a touchdown from vibration; real hover logs reach 1.56 g).
    // 候補値の根拠の確認: 後半の低下は推力を下げ（率 > 0）、時間切れより前に始まること、時間切れは正で符号付き差の
    // 範囲 2^31 us 未満、接地の衝撃の閾値は 1 g と 3 G の墜落アラートの間にあること（接地と振動を分けるのは閾値でなく
    // 静止の判定。実機のホバーログは 1.56 g に達する）。
    constexpr float kGravityMps2 = 9.80665f;
    ASSERT_TRUE(sf::limits::kBlindLandingLateRampPerS > 0.0f);
    ASSERT_TRUE(sf::limits::kBlindLandingLateStartS < static_cast<float>(kBlindUs) * 1e-6f);
    ASSERT_TRUE(kMinSilentUs > 0u && kMinSilentUs < kBlindUs);   // the guaranteed silent time is shorter than the LANDING limit / 保証する出力停止の時間は LANDING の上限より短い
    ASSERT_TRUE(kBlindUs > 0u && kBlindUs < 0x80000000u);
    ASSERT_TRUE(sf::limits::kBlindTouchdownAccelMps2 > 1.56f * kGravityMps2);
    ASSERT_TRUE(sf::limits::kBlindTouchdownAccelMps2 < 3.0f * kGravityMps2);
    ASSERT_TRUE(sf::limits::kBlindTouchdownSamples >= 2);
}

int main()
{
    printf("=== ToF silence judgement and state-machine response unit tests ===\n");

    run_threshold_boundary();
    run_normal_sample_interval_is_not_silent();
    run_timestamp_newer_than_now_is_not_silent();
    run_never_seen_is_silent();
    run_uint32_wraparound_cases();
    run_out_of_range_samples_are_not_silent();
    run_stream_that_stops_is_silent_after_threshold();
    run_tof_silent_for_more_than_36_minutes_stays_silent();
    run_no_sample_since_boot_is_silent_for_any_duration();
    run_threshold_is_three_times_the_largest_measured_interval();
    run_not_silent_never_acts_in_any_state_or_mode();
    run_armed_on_ground_disarms_in_every_mode();
    run_altitude_modes_start_landing_in_takeoff_and_flying();
    run_manual_modes_only_warn_never_land();
    run_landing_completes_only_after_the_blind_time_in_every_mode();
    run_landing_after_a_long_live_tof_gets_the_minimum_silent_time_before_the_motors_are_cut();
    run_flip_defers_to_flying();
    run_ground_states_do_nothing_here();
    run_altitude_modes_are_refused_while_silent();
    run_landing_time_is_counted_from_the_entry_into_landing_and_not_extended_by_a_returning_tof();
    run_landing_clock_counts_the_continuous_silence_of_this_landing();
    run_blind_landing_limits_are_consistent();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
