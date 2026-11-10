/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_sensor_freshness.cpp
 * @brief Deterministic unit tests for sf::SilenceWatch (sensor_health / comm-loss / ToF judgement)
 *        sf::SilenceWatch の決定論ユニットテスト（sensor_health・通信断・ToF の判定）
 *
 * Header-only logic (sensor_freshness.hpp) — no ESP-IDF, no FreeRTOS. Own TEST/ASSERT
 * framework and main(), same pattern as test_tilt_consistency.cpp.
 * ヘッダのみのロジック（sensor_freshness.hpp）— ESP-IDF・FreeRTOS 不要。
 * test_tilt_consistency.cpp と同じ自前の TEST/ASSERT フレームワークと main()。
 *
 * @design architecture.md §3 — R15 sensor_health 1 Hz [OK]
 */

#include <cstdint>
#include <cstdio>

#include "sensor_freshness.hpp"

// Same value as config::SENSOR_HEALTH_STALE_US (main/config.hpp) — 0.5 s.
// config::SENSOR_HEALTH_STALE_US（main/config.hpp）と同じ値 — 0.5 s。
static constexpr uint32_t kStaleWindowUs = 500000;

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

// Thin wrapper fixing the window, so tests read as the judgement they check.
// 窓を固定する薄いラッパ。テストが判定そのものとして読めるようにする。
static bool fresh(uint32_t now_us, uint32_t timestamp_us)
{
    sf::SilenceWatch watch;                 // one observation = a stateless judgement
    watch.observe(now_us, timestamp_us);    // 観測 1 回 = 状態を持たない判定
    return watch.isFresh(kStaleWindowUs);
}

// Observation period used by the stateful tests below (PowerTask polls at 1 Hz).
// 以下の状態ありテストで使う観測周期（PowerTask は 1 Hz で観測する）。
static constexpr uint32_t kPollUs = 1000000;
// Comm-loss timeout used by Failsafe::checkCommTimeout (config comm_timeout_ms = 500).
// Failsafe::checkCommTimeout が使う通信断のタイムアウト（comm_timeout_ms = 500）。
static constexpr uint32_t kCommTimeoutUs = 500000;
// 36 minutes: beyond the 2^31 us (35.8 min) limit of a signed 32-bit difference.
// 36 分: 符号付き 32bit 差の限界 2^31 us（35.8 分）を超える。
static constexpr uint32_t kThirtySixMinutesUs = 36u * 60u * 1000000u;

TEST(recent_sample_is_fresh)
{
    ASSERT_TRUE(fresh(10000000u, 9990000u));            // 10 ms old
}

TEST(old_sample_is_stale)
{
    ASSERT_TRUE(!fresh(10000000u, 9000000u));           // 1 s old
}

TEST(timestamp_newer_than_now_is_fresh)
{
    // The false-positive case: the sensor task stored a timestamp 1 us / 5 us / 3 ms
    // AFTER PowerTask read now_us. Unsigned (now - ts) would be ~4.29e9 -> stale.
    // 偽陽性ケース: センサタスクが PowerTask の now_us 読み出しより 1 us / 5 us / 3 ms
    // 「後」の時刻を保存。符号なしの (now - ts) は ~4.29e9 になり stale になってしまう。
    ASSERT_TRUE(fresh(10000000u, 10000001u));
    ASSERT_TRUE(fresh(10000000u, 10000005u));
    ASSERT_TRUE(fresh(10000000u, 10003000u));
    ASSERT_TRUE(sf::timestampAgeUs(10000000u, 10000005u) == -5);
}

TEST(boundary_exactly_at_window_is_stale)
{
    // age == window -> stale (strict "<"), age == window - 1 -> fresh, window + 1 -> stale.
    // age == 窓 → stale（厳密な "<"）、窓 - 1 → fresh、窓 + 1 → stale。
    const uint32_t now_us = 20000000u;
    ASSERT_TRUE(fresh(now_us, now_us - (kStaleWindowUs - 1)));
    ASSERT_TRUE(!fresh(now_us, now_us - kStaleWindowUs));
    ASSERT_TRUE(!fresh(now_us, now_us - (kStaleWindowUs + 1)));
}

TEST(never_published_is_not_fresh)
{
    ASSERT_TRUE(!fresh(10000000u, 0u));
    ASSERT_TRUE(!fresh(100u, 0u));
}

TEST(uint32_wraparound_normal_cases)
{
    // now just after the 2^32 wrap, sample just before it: age is small and positive.
    // now が 2^32 の巻き戻り直後、サンプルがその直前: age は小さな正の値。
    ASSERT_TRUE(fresh(100u, 0xFFFFFF00u));              // age = 356 us
    ASSERT_TRUE(sf::timestampAgeUs(100u, 0xFFFFFF00u) == 356);
}

TEST(uint32_wraparound_boundary_values)
{
    // Window boundary straddling the wrap: ts = 2^32 - 1, age = now + 1.
    // 巻き戻りをまたぐ窓境界: ts = 2^32 - 1、age = now + 1。
    ASSERT_TRUE(fresh(kStaleWindowUs - 2, 0xFFFFFFFFu));    // age = window - 1 -> fresh
    ASSERT_TRUE(!fresh(kStaleWindowUs - 1, 0xFFFFFFFFu));   // age = window     -> stale
    // Stale across the wrap: 2 s old.
    // 巻き戻りをまたいだ stale: 2 s 前。
    ASSERT_TRUE(!fresh(1000000u, 0xFFFFFFFFu - 1000000u));  // age = 2000001 us
    // Newer than now across the wrap: ts just AFTER the wrap, now just BEFORE it.
    // 巻き戻りをまたいだ「新しい」: ts が巻き戻り直後、now が直前。
    ASSERT_TRUE(fresh(0xFFFFFFFEu, 3u));                // age = -5 us
    ASSERT_TRUE(sf::timestampAgeUs(0xFFFFFFFEu, 3u) == -5);
}

TEST(long_silence_stays_silent_past_signed_limit)
{
    // The sensor stops at t0. Observed every second, it must be silent from 0.5 s on
    // and STAY silent through 36 min (a signed 32-bit difference would turn negative at
    // 35.8 min and call it "newer than now" = healthy) and through the 71.6 min uint32 wrap.
    // センサが t0 で止まる。1 秒ごとに観測し、0.5 s 以降は停止、かつ 36 分（符号付き 32bit
    // 差は 35.8 分で負になり「now より新しい」＝健全と誤る）も 71.6 分の uint32 巻き戻りも
    // またいで停止のままであること。
    const uint32_t last_sample_us = 4000000000u;          // near the uint32 limit / 上限の近く
    uint32_t now_us = last_sample_us;
    sf::SilenceWatch watch;
    watch.observe(now_us, last_sample_us);
    ASSERT_TRUE(watch.isFresh(kStaleWindowUs));
    const uint64_t total_us = 80ull * 60ull * 1000000ull;            // 80 min: wraps uint32 / 巻き戻りあり
    const uint32_t end_us = static_cast<uint32_t>(last_sample_us + total_us);
    bool ever_fresh_after_window = false;
    for (uint64_t elapsed = 0; elapsed < total_us; elapsed += kPollUs) {
        now_us += kPollUs;
        watch.observe(now_us, last_sample_us);
        if (elapsed + kPollUs >= kStaleWindowUs && !watch.isSilent(kStaleWindowUs)) {
            ever_fresh_after_window = true;
        }
    }
    ASSERT_TRUE(now_us == end_us);
    ASSERT_TRUE(!ever_fresh_after_window);
    ASSERT_TRUE(watch.silentForUs() == sf::SilenceWatch::kSaturatedUs);   // saturated, no wrap
}

TEST(silence_longer_than_36_minutes_is_silent)
{
    // The exact 36 min case, sparse observation (every 10 s).
    // ちょうど 36 分のケース。観測は疎（10 秒ごと）。
    sf::SilenceWatch watch;
    uint32_t now_us = 5000000u;
    watch.observe(now_us, now_us);                 // last sample now / 最後のサンプルは今
    for (uint32_t t = 0; t < kThirtySixMinutesUs + 10000000u; t += 10000000u) {
        now_us += 10000000u;
        watch.observe(now_us, 5000000u);
    }
    ASSERT_TRUE(watch.isSilent(kStaleWindowUs));
    ASSERT_TRUE(watch.silentForUs() > kThirtySixMinutesUs);
}

TEST(never_received_since_boot_is_silent_and_recovers)
{
    // Just after boot: stamp 0 -> silent however long we observe it; the first sample
    // flips it to fresh; stopping again flips it back.
    // 起動直後で未受信: 時刻印 0 → 何度観測しても停止。最初のサンプルで fresh になり、
    // 再び止まれば停止に戻る。
    sf::SilenceWatch watch;
    ASSERT_TRUE(watch.isSilent(kStaleWindowUs));        // never observed / 観測前
    for (uint32_t now_us = 100000u; now_us < 5000000u; now_us += 100000u) {
        watch.observe(now_us, 0u);
        ASSERT_TRUE(watch.isSilent(kStaleWindowUs));
        ASSERT_TRUE(!watch.isFresh(kStaleWindowUs));
    }
    watch.observe(5100000u, 5099000u);                  // first sample, 1 ms old
    ASSERT_TRUE(watch.isFresh(kStaleWindowUs));
    watch.observe(5600000u, 5099000u);                  // no new sample for 501 ms
    ASSERT_TRUE(watch.isSilent(kStaleWindowUs));
    watch.observe(5700000u, 5699000u);                  // a new sample revives it
    ASSERT_TRUE(watch.isFresh(kStaleWindowUs));
}

TEST(new_sample_with_future_timestamp_is_fresh_then_ages)
{
    // A stamp 3 ms newer than now (read-order inversion) is fresh, and with no further
    // sample it ages from "now" and becomes silent at the window.
    // now より 3 ms 新しい時刻印（読み出し順序の逆転）は fresh。続くサンプルが無ければ
    // 「今」から経過し、窓で停止になる。
    sf::SilenceWatch watch;
    watch.observe(10000000u, 10003000u);
    ASSERT_TRUE(watch.isFresh(kStaleWindowUs));
    ASSERT_TRUE(watch.silentForUs() == 0u);
    watch.observe(10499999u, 10003000u);
    ASSERT_TRUE(watch.isFresh(kStaleWindowUs));
    watch.observe(10500000u, 10003000u);
    ASSERT_TRUE(watch.isSilent(kStaleWindowUs));
}

TEST(comm_loss_judgement_for_failsafe)
{
    // Failsafe::checkCommTimeout: a setpoint stamped by another task a few us AFTER the
    // failsafe's now_us must NOT read as 4.29e9 us of loss (false COMM_LOST); a real loss
    // longer than 36 min must stay lost; a pause > 500 ms is a loss and recovery clears it.
    // Failsafe::checkCommTimeout: 別タスクが failsafe の now_us より数 us 「後」に刻んだ
    // setpoint を 4.29e9 us の途絶（誤った COMM_LOST）と読んではならない。36 分超の本物の途絶は
    // 途絶のまま。500 ms を超える中断は途絶で、復帰すれば解除される。
    sf::SilenceWatch watch;
    watch.observe(20000000u, 20000007u);                // 7 us in the future / 未来 7 us
    ASSERT_TRUE(!watch.isSilent(kCommTimeoutUs));
    watch.observe(20100000u, 20090000u);                // 100 ms cycle, new setpoint
    ASSERT_TRUE(!watch.isSilent(kCommTimeoutUs));
    watch.observe(20700000u, 20090000u);                // 610 ms without a packet
    ASSERT_TRUE(watch.isSilent(kCommTimeoutUs));
    watch.observe(20800000u, 20790000u);                // link back
    ASSERT_TRUE(!watch.isSilent(kCommTimeoutUs));
    // Long loss: observed at the failsafe's 10 Hz for 40 min of silence.
    // 長い途絶: failsafe の 10 Hz で 40 分間の無音を観測する。
    uint32_t now_us = 20800000u;
    for (uint32_t i = 0; i < 40u * 60u * 10u; ++i) {
        now_us += 100000u;
        watch.observe(now_us, 20790000u);
        if (i >= 5u) {
            ASSERT_TRUE(watch.isSilent(kCommTimeoutUs));
        }
    }
}

TEST(clock_not_advancing_does_not_age_silence)
{
    // A non-increasing "now" (equal or a few us backwards) adds no silent time.
    // 進まない（同値・数 us 逆行する）「現在時刻」は停止時間を増やさない。
    sf::SilenceWatch watch;
    watch.observe(1000000u, 1000000u);
    watch.observe(1300000u, 1000000u);
    ASSERT_TRUE(watch.silentForUs() == 300000u);
    watch.observe(1299990u, 1000000u);
    ASSERT_TRUE(watch.silentForUs() == 300000u);
    watch.observe(1300000u, 1000000u);                  // back to the reference: still 300 ms
    ASSERT_TRUE(watch.silentForUs() == 300000u);
    watch.observe(1300010u, 1000000u);
    ASSERT_TRUE(watch.silentForUs() == 300010u);
}

// Observation gaps (problem 3): the caller did not observe for N minutes (e.g. DISARM).
// 観測の空き（問題3）: 呼び出し側が N 分観測しなかった（例: DISARM 中）。
static bool silent_after_gap(uint32_t gap_minutes, uint32_t stamp_age_at_last_us)
{
    // A live source observed at t0, then NOT observed for gap_minutes, then observed again
    // with the SAME stamp (the source died right after the last observation).
    // 動作していた発行元を t0 で観測し、gap_minutes 分観測せず、同じ時刻印で再観測する
    // （最後の観測の直後に発行元が止まった）。
    sf::SilenceWatch watch;
    const uint32_t t0 = 3000000000u;
    const uint32_t stamp = t0 - stamp_age_at_last_us;
    watch.observe(t0, stamp);
    const uint32_t t1 = t0 + gap_minutes * 60u * 1000000u;    // wraps for long gaps / 長い空きで巻き戻る
    watch.observe(t1, stamp);
    return watch.isSilent(kStaleWindowUs);
}

TEST(observation_gap_36_40_60_minutes_is_silent)
{
    ASSERT_TRUE(silent_after_gap(36u, 1000u));
    ASSERT_TRUE(silent_after_gap(40u, 1000u));
    ASSERT_TRUE(silent_after_gap(60u, 1000u));
    // Not merely "silent for the gap": the saturated value, so it cannot read fresh later.
    // 空きの分だけでなく飽和値になること（後で fresh に戻らない）。
    sf::SilenceWatch watch;
    watch.observe(3000000000u, 2999999000u);
    watch.observe(3000000000u + 40u * 60u * 1000000u, 2999999000u);
    ASSERT_TRUE(watch.silentForUs() == sf::SilenceWatch::kSaturatedUs);
    watch.observe(3000000000u + 40u * 60u * 1000000u + kPollUs, 2999999000u);
    ASSERT_TRUE(watch.isSilent(kStaleWindowUs));
}

TEST(observation_gap_shorter_than_limit_counts_real_time)
{
    // 30 min gap (< 35.8): the elapsed time is still valid -> silent for ~30 min, not saturated.
    // 30 分の空き（< 35.8）: 経過はまだ有効 → 約 30 分の停止。飽和ではない。
    sf::SilenceWatch watch;
    watch.observe(3000000000u, 3000000000u);
    watch.observe(3000000000u + 30u * 60u * 1000000u, 3000000000u);
    ASSERT_TRUE(watch.silentForUs() == 30u * 60u * 1000000u);
}

TEST(observation_gap_then_new_sample_is_fresh_again)
{
    // After a long gap, a genuinely new sample must revive the watch (no stuck saturation)
    // and the reference clock must have been re-based (later silence is counted normally).
    // 長い空きのあとでも、本当に新しいサンプルが来れば復帰する（飽和に固着しない）。
    // 基準時刻も取り直され、以降の停止は通常どおり数える。
    sf::SilenceWatch watch;
    const uint32_t t0 = 3000000000u;
    watch.observe(t0, t0);
    const uint32_t t1 = t0 + 40u * 60u * 1000000u;
    watch.observe(t1, t0);                                   // gap -> saturated
    ASSERT_TRUE(watch.silentForUs() == sf::SilenceWatch::kSaturatedUs);
    watch.observe(t1 + kPollUs, t1 + kPollUs - 1000u);       // new sample
    ASSERT_TRUE(watch.isFresh(kStaleWindowUs));
    watch.observe(t1 + 2 * kPollUs, t1 + kPollUs - 1000u);   // 1.001 s silent
    ASSERT_TRUE(watch.silentForUs() == kPollUs + 1000u);
}

TEST(first_observation_with_old_stamp_is_silent)
{
    // The very first observation carries a stamp 36 / 40 / 60 min old: the signed difference
    // turns negative (looks "newer than now"), which must NOT read as fresh.
    // 最初の観測が 36 / 40 / 60 分前の時刻印: 符号付きの差は負（「now より新しい」に見える）に
    // なるが、fresh と読んではならない。
    const uint32_t now_us = 3500000000u;
    const uint32_t ages_min[] = {36u, 40u, 60u};
    for (uint32_t age_min : ages_min) {
        sf::SilenceWatch watch;
        watch.observe(now_us, now_us - age_min * 60u * 1000000u);
        ASSERT_TRUE(watch.isSilent(kStaleWindowUs));
        ASSERT_TRUE(watch.silentForUs() == sf::SilenceWatch::kSaturatedUs);
    }
    // 20 min old (valid range): silent for 20 min, exactly.
    // 20 分前（有効範囲）: ちょうど 20 分の停止。
    sf::SilenceWatch watch;
    watch.observe(now_us, now_us - 20u * 60u * 1000000u);
    ASSERT_TRUE(watch.silentForUs() == 20u * 60u * 1000000u);
}

TEST(future_stamp_tolerance_boundary)
{
    // A new stamp up to kClockSkewToleranceUs ahead of now is "just now" (0); further ahead
    // is the wrapped-negative ambiguity -> saturated.
    // now より kClockSkewToleranceUs までの先の新しい時刻印は「たった今」（0）。それ以上先は
    // 負に巻き戻った曖昧さ → 飽和。
    const uint32_t now_us = 10000000u;
    sf::SilenceWatch a;
    a.observe(now_us, now_us + static_cast<uint32_t>(sf::SilenceWatch::kClockSkewToleranceUs));
    ASSERT_TRUE(a.silentForUs() == 0u);
    sf::SilenceWatch b;
    b.observe(now_us, now_us + static_cast<uint32_t>(sf::SilenceWatch::kClockSkewToleranceUs) + 1u);
    ASSERT_TRUE(b.silentForUs() == sf::SilenceWatch::kSaturatedUs);
}

TEST(unchanged_stamp_with_clock_far_behind_saturates)
{
    // The clock tolerance boundary for an UNCHANGED stamp: up to tolerance backwards adds 0,
    // beyond it is a gap -> saturated.
    // 時刻印が「変化なし」のときの時計許容の境界: 許容以内の後戻りは 0 を足し、超えれば空きなので飽和。
    sf::SilenceWatch a;
    a.observe(5000000u, 5000000u);
    a.observe(5000000u - static_cast<uint32_t>(sf::SilenceWatch::kClockSkewToleranceUs), 5000000u);
    ASSERT_TRUE(a.silentForUs() == 0u);
    sf::SilenceWatch b;
    b.observe(5000000u, 5000000u);
    b.observe(5000000u - static_cast<uint32_t>(sf::SilenceWatch::kClockSkewToleranceUs) - 1u, 5000000u);
    ASSERT_TRUE(b.silentForUs() == sf::SilenceWatch::kSaturatedUs);
}

TEST(zero_stamp_after_nonzero_is_a_new_sample)
{
    // The stamp is uint32 microseconds: at the 2^32 us wrap it is exactly 0 for one tick.
    // Once a non-zero stamp has been seen, that 0 is a fresh sample, not "never published".
    // 時刻印は uint32 マイクロ秒: 2^32 us の巻き戻りでちょうど 0 になる瞬間がある。0 以外を
    // 一度見たあとのその 0 は新サンプルで、「未発行」ではない。
    sf::SilenceWatch watch;
    watch.observe(0xFFFFFF00u, 0xFFFFFF00u - 1000u);
    ASSERT_TRUE(watch.isFresh(kStaleWindowUs));
    ASSERT_TRUE(watch.hasSeenSample());
    watch.observe(100u, 0u);                          // exact wrap tick: 100 us old / ちょうど巻き戻り: 100 us 前
    ASSERT_TRUE(watch.isFresh(kStaleWindowUs));
    ASSERT_TRUE(watch.silentForUs() == 100u);
    watch.observe(kStaleWindowUs + 100u, 0u);         // unchanged 0 now ages normally / 変化なしの 0 は通常どおり経過
    ASSERT_TRUE(watch.isSilent(kStaleWindowUs));
    // Before any non-zero stamp, 0 is still "never published".
    // 0 以外を一度も見ていなければ、0 は従来どおり「未発行」。
    sf::SilenceWatch fresh_boot;
    fresh_boot.observe(100u, 0u);
    ASSERT_TRUE(fresh_boot.isSilent(kStaleWindowUs));
    ASSERT_TRUE(!fresh_boot.hasSeenSample());
}

TEST(observing_through_disarm_keeps_a_dead_link_silent)
{
    // The control-task scenario: setpoint stamp stops at t0 (link dies while DISARMED). The
    // watch is observed EVERY cycle (400 Hz here, scaled to 1 s steps for speed) through 40 min
    // and then the vehicle is armed: the link must read dead the whole way, not revive when a
    // single 40 min difference would have wrapped. Contrast: the same stamp observed only at
    // t0 and at ARM (the pre-fix behaviour) is caught by the gap rule above.
    // 制御タスクの場面: setpoint の時刻印が t0 で止まる（DISARM 中にリンクが途絶える）。監視器を
    // 毎周期観測（ここでは速度のため 1 s 刻み）して 40 分たち、ARM する: リンクは終始「途絶」で、
    // 40 分の差が巻き戻って復活してはならない。
    sf::SilenceWatch watch;
    const uint32_t t0 = 3900000000u;
    watch.observe(t0, t0);
    uint32_t now_us = t0;
    for (uint32_t i = 0; i < 40u * 60u; ++i) {
        now_us += kPollUs;
        watch.observe(now_us, t0);
        if (i >= 1u) {
            ASSERT_TRUE(watch.isSilent(kStaleWindowUs));
        }
    }
    ASSERT_TRUE(watch.silentForUs() > 39u * 60u * 1000000u);
}

int main()
{
    printf("=== sf::SilenceWatch / freshness unit tests ===\n");

    run_recent_sample_is_fresh();
    run_old_sample_is_stale();
    run_timestamp_newer_than_now_is_fresh();
    run_boundary_exactly_at_window_is_stale();
    run_never_published_is_not_fresh();
    run_uint32_wraparound_normal_cases();
    run_uint32_wraparound_boundary_values();
    run_long_silence_stays_silent_past_signed_limit();
    run_silence_longer_than_36_minutes_is_silent();
    run_never_received_since_boot_is_silent_and_recovers();
    run_new_sample_with_future_timestamp_is_fresh_then_ages();
    run_comm_loss_judgement_for_failsafe();
    run_clock_not_advancing_does_not_age_silence();
    run_observation_gap_36_40_60_minutes_is_silent();
    run_observation_gap_shorter_than_limit_counts_real_time();
    run_observation_gap_then_new_sample_is_fresh_again();
    run_first_observation_with_old_stamp_is_silent();
    run_future_stamp_tolerance_boundary();
    run_unchanged_stamp_with_clock_far_behind_saturates();
    run_zero_stamp_after_nonzero_is_a_new_sample();
    run_observing_through_disarm_keeps_a_dead_link_silent();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
