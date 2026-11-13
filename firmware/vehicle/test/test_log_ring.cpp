/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_log_ring.cpp
 * @brief Deterministic unit tests for sf::LogRing (the line buffer of the log sink)
 *        sf::LogRing（ログシンクの行バッファ）の決定論ユニットテスト
 *
 * Header-only logic (log_ring.hpp) — no ESP-IDF, no FreeRTOS. Own TEST/ASSERT framework and
 * main(), same pattern as test_sensor_freshness.cpp.
 * ヘッダのみのロジック（log_ring.hpp）— ESP-IDF・FreeRTOS 不要。test_sensor_freshness.cpp と
 * 同じ自前の TEST/ASSERT フレームワークと main()。
 *
 * @design detailed_design.md §8 — Log sink: 400 Hz tasks never wait for the console  [OK]
 */

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "log_ring.hpp"

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

// Small ring so wrap-around and overflow are reached in a few lines.
// 一周と溢れを少ない行数で起こせる小さなリング。
using SmallRing = sf::LogRing<1024>;

static bool pushText(SmallRing& ring, const std::string& text)
{
    return ring.push(text.c_str(), text.size());
}

static std::string popText(SmallRing& ring)
{
    char buffer[SmallRing::kMaxLineBytes + 1];
    uint32_t dropped_before = 0;
    const size_t length = ring.pop(buffer, SmallRing::kMaxLineBytes, &dropped_before);
    return std::string(buffer, length);
}

TEST(empty_ring_pops_nothing)
{
    SmallRing ring;
    ASSERT_TRUE(ring.empty());
    ASSERT_TRUE(popText(ring).empty());
}

TEST(lines_come_out_in_the_order_they_went_in)
{
    SmallRing ring;
    ASSERT_TRUE(pushText(ring, "I (10) a: first\n"));
    ASSERT_TRUE(pushText(ring, "W (20) b: second\n"));
    ASSERT_TRUE(pushText(ring, "E (30) c: third\n"));
    ASSERT_TRUE(popText(ring) == "I (10) a: first\n");
    ASSERT_TRUE(popText(ring) == "W (20) b: second\n");
    ASSERT_TRUE(popText(ring) == "E (30) c: third\n");
    ASSERT_TRUE(ring.empty());
}

TEST(wrap_around_keeps_every_line_whole)
{
    SmallRing ring;
    // 40 rounds of 5 lines of 100 bytes cross the end of the 1024-byte buffer many times.
    // 100 バイトの行を 5 本ずつ 40 回: 1024 バイトのバッファの端を何度もまたぐ。
    for (int round = 0; round < 40; ++round) {
        for (int line = 0; line < 5; ++line) {
            const std::string text(100, static_cast<char>('a' + (round + line) % 26));
            ASSERT_TRUE(pushText(ring, text));
        }
        for (int line = 0; line < 5; ++line) {
            const std::string expected(100, static_cast<char>('a' + (round + line) % 26));
            ASSERT_TRUE(popText(ring) == expected);
        }
    }
    ASSERT_TRUE(ring.empty());
    ASSERT_TRUE(ring.takeDropped() == 0);
}

TEST(length_prefix_split_by_the_buffer_end_is_read_back)
{
    SmallRing ring;
    // An entry of 101 bytes (8 + 93) is coprime with the 1024-byte buffer, so over 1100 lines
    // the eight prefix bytes land on every possible position, including a split by the end.
    // 101 バイトの項目（8 + 93）は 1024 バイトのバッファと互いに素なので、1100 行の間に接頭辞の 8 バイトは
    // あらゆる位置（末尾で割れる場合を含む）に来る。
    static_assert(SmallRing::kPrefixBytes == 8, "the entry size below assumes the 8-byte prefix");
    for (int i = 0; i < 1100; ++i) {
        const std::string text(93, static_cast<char>('A' + i % 26));
        ASSERT_TRUE(pushText(ring, text));
        ASSERT_TRUE(popText(ring) == text);
    }
    ASSERT_TRUE(ring.empty());
}

TEST(a_line_that_does_not_fit_is_dropped_and_counted)
{
    SmallRing ring;
    const std::string text(300, 'x');
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(pushText(ring, text));          // 3 x 308 = 924 bytes used
    ASSERT_TRUE(!pushText(ring, text));         // 1232 > 1024: refused
    ASSERT_TRUE(!pushText(ring, text));
    ASSERT_TRUE(ring.takeDropped() == 2);
    ASSERT_TRUE(ring.takeDropped() == 0);       // reading clears the count / 読むと数は 0 に戻る
    // The three stored lines are intact and the ring accepts a line again after they leave.
    // 格納済みの 3 行は無傷で、出した後はまた入る。
    ASSERT_TRUE(popText(ring) == text);
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(popText(ring) == text);
    ASSERT_TRUE(popText(ring) == text);
    ASSERT_TRUE(popText(ring) == text);
    ASSERT_TRUE(ring.empty());
}

TEST(drops_are_reported_with_the_first_line_stored_after_them)
{
    SmallRing ring;
    const std::string text(300, 'x');
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(!pushText(ring, text));
    ASSERT_TRUE(!pushText(ring, text));
    char buffer[SmallRing::kMaxLineBytes + 1];
    uint32_t dropped_before = 99;
    ASSERT_TRUE(ring.pop(buffer, SmallRing::kMaxLineBytes, &dropped_before) == text.size());
    ASSERT_TRUE(dropped_before == 0);           // the lines before the loss carry no count / 損失より前の行に数は付かない
    ASSERT_TRUE(pushText(ring, "after\n"));    // first line after the 2 drops / 2 行の落としの後の最初の行
    ASSERT_TRUE(ring.pop(buffer, SmallRing::kMaxLineBytes, &dropped_before) == text.size());
    ASSERT_TRUE(ring.pop(buffer, SmallRing::kMaxLineBytes, &dropped_before) == text.size());
    ASSERT_TRUE(ring.pop(buffer, SmallRing::kMaxLineBytes, &dropped_before) == 6);
    ASSERT_TRUE(dropped_before == 2);
    ASSERT_TRUE(ring.takeDropped() == 0);       // already attached to the line / 行に付けた分は残らない
}

TEST(line_longer_than_the_maximum_is_refused)
{
    SmallRing ring;
    const std::string longest(SmallRing::kMaxLineBytes, 'y');
    const std::string too_long(SmallRing::kMaxLineBytes + 1, 'z');
    ASSERT_TRUE(pushText(ring, longest));
    ASSERT_TRUE(!pushText(ring, too_long));
    ASSERT_TRUE(ring.takeDropped() == 1);
    ASSERT_TRUE(popText(ring) == longest);      // the longest line round-trips (2-byte length) / 最長の行も往復できる
}

TEST(empty_line_is_ignored_and_not_counted_as_a_drop)
{
    SmallRing ring;
    ASSERT_TRUE(ring.push("", 0));
    ASSERT_TRUE(ring.empty());
    ASSERT_TRUE(ring.takeDropped() == 0);
}

TEST(small_output_buffer_leaves_the_line_in_place)
{
    SmallRing ring;
    ASSERT_TRUE(pushText(ring, "0123456789"));
    char small[4];
    uint32_t unused_drops = 0;
    ASSERT_TRUE(ring.pop(small, sizeof(small), &unused_drops) == 0);   // too small: nothing removed / 小さすぎ: 取り出さない
    ASSERT_TRUE(popText(ring) == "0123456789");
}

TEST(text_with_zero_bytes_and_newlines_round_trips)
{
    SmallRing ring;
    const std::string text("a\0b\nc\n", 6);
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(popText(ring) == text);
}

// The time stamp and the truncation flag travel with the line (the WiFi exit sends them), also across the
// end of the buffer.
// 時刻と切り詰めのフラグは行と一緒に運ばれる（WiFi 側の出口が送る）。バッファの末尾をまたいでも同じ。
TEST(time_stamp_and_truncation_flag_travel_with_the_line)
{
    SmallRing ring;
    for (int i = 0; i < 1100; ++i) {
        const std::string text(93, static_cast<char>('A' + i % 26));
        const uint32_t stamp = 0xFFFFFF00u + static_cast<uint32_t>(i);   // crosses the 32-bit wrap / 32 ビットの桁あふれをまたぐ
        const bool truncated = (i % 3) == 0;
        ASSERT_TRUE(ring.push(text.c_str(), text.size(), stamp, truncated));
        char buffer[SmallRing::kMaxLineBytes + 1];
        uint32_t dropped_before = 99;
        SmallRing::LineInfo info;
        ASSERT_TRUE(ring.pop(buffer, SmallRing::kMaxLineBytes, &dropped_before, &info) == text.size());
        ASSERT_TRUE(info.timestamp_us == stamp);
        ASSERT_TRUE(info.truncated == truncated);
        ASSERT_TRUE(dropped_before == 0);
    }
}

// oldestLength() answers without removing the line, so a reader with a byte budget can leave a long
// line for the next cycle.
// oldestLength() は行を取り出さずに答えるので、バイト数の予算を持つ読み手は長い行を次の周期に残せる。
TEST(oldest_length_is_seen_without_removing_the_line)
{
    SmallRing ring;
    ASSERT_TRUE(ring.oldestLength() == 0);
    ASSERT_TRUE(pushText(ring, "short\n"));
    ASSERT_TRUE(pushText(ring, std::string(300, 'q')));
    ASSERT_TRUE(ring.oldestLength() == 6);
    ASSERT_TRUE(ring.oldestLength() == 6);      // asking twice changes nothing / 2 回聞いても変わらない
    ASSERT_TRUE(popText(ring) == "short\n");
    ASSERT_TRUE(ring.oldestLength() == 300);
    ASSERT_TRUE(popText(ring).size() == 300);
    ASSERT_TRUE(ring.oldestLength() == 0);
}

// The running total counts every refused line exactly, also past the 255 that one stored line can carry
// (the remainder stays pending for the next line), and is not reduced by reading.
// 累計は断った行をすべて正確に数える。1 行が運べる 255 を超えても同じ（残りは次の行へ持ち越す）。読んでも減らない。
TEST(total_dropped_is_exact_beyond_the_per_line_limit)
{
    SmallRing ring;
    const std::string text(300, 'x');
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(pushText(ring, text));
    for (int i = 0; i < 300; ++i) {
        ASSERT_TRUE(!pushText(ring, text));
    }
    ASSERT_TRUE(ring.totalDropped() == 300);
    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(popText(ring) == text);
    }
    ASSERT_TRUE(pushText(ring, "first after the loss\n"));
    ASSERT_TRUE(pushText(ring, "second after the loss\n"));
    char buffer[SmallRing::kMaxLineBytes + 1];
    uint32_t dropped_before = 0;
    ASSERT_TRUE(ring.pop(buffer, SmallRing::kMaxLineBytes, &dropped_before) > 0);
    ASSERT_TRUE(dropped_before == SmallRing::kMaxDroppedBefore);   // 255 carried by the first line / 最初の行が 255 を運ぶ
    ASSERT_TRUE(ring.pop(buffer, SmallRing::kMaxLineBytes, &dropped_before) > 0);
    ASSERT_TRUE(dropped_before == 45);                                // the remainder by the next one / 残りは次の行
    ASSERT_TRUE(ring.totalDropped() == 300);                          // reading does not reduce it / 読んでも減らない
}

// clear() starts a capture from "now": stored lines, pending drops and the total are all forgotten, and the
// ring works as new afterwards.
// clear() は取得を「今」から始める: 格納済みの行・持ち越しの落とし・累計をすべて忘れ、その後は新品と同じに動く。
TEST(clear_forgets_lines_and_counts)
{
    SmallRing ring;
    const std::string text(300, 'x');
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(pushText(ring, text));
    ASSERT_TRUE(!pushText(ring, text));
    ring.clear();
    ASSERT_TRUE(ring.empty());
    ASSERT_TRUE(ring.totalDropped() == 0);
    ASSERT_TRUE(ring.takeDropped() == 0);
    ASSERT_TRUE(pushText(ring, "new\n"));
    char buffer[SmallRing::kMaxLineBytes + 1];
    uint32_t dropped_before = 99;
    ASSERT_TRUE(ring.pop(buffer, SmallRing::kMaxLineBytes, &dropped_before) == 4);
    ASSERT_TRUE(dropped_before == 0);
}

// One longest line (prefix included) always fits the budget constant the sender uses.
// 最長の 1 行（接頭辞を含む）は、送信側が使う予算の定数に必ず収まる。
TEST(longest_stored_line_size_is_the_prefix_plus_the_line_limit)
{
    static_assert(SmallRing::kMaxStoredLineBytes == SmallRing::kPrefixBytes + SmallRing::kMaxLineBytes, "size");
    SmallRing ring;
    ASSERT_TRUE(pushText(ring, std::string(SmallRing::kMaxLineBytes, 'm')));
    ASSERT_TRUE(ring.usedBytes() == SmallRing::kMaxStoredLineBytes);
}

int main()
{
    printf("=== LogRing unit tests ===\n");
    run_empty_ring_pops_nothing();
    run_lines_come_out_in_the_order_they_went_in();
    run_wrap_around_keeps_every_line_whole();
    run_length_prefix_split_by_the_buffer_end_is_read_back();
    run_a_line_that_does_not_fit_is_dropped_and_counted();
    run_drops_are_reported_with_the_first_line_stored_after_them();
    run_line_longer_than_the_maximum_is_refused();
    run_empty_line_is_ignored_and_not_counted_as_a_drop();
    run_small_output_buffer_leaves_the_line_in_place();
    run_text_with_zero_bytes_and_newlines_round_trips();
    run_time_stamp_and_truncation_flag_travel_with_the_line();
    run_oldest_length_is_seen_without_removing_the_line();
    run_total_dropped_is_exact_beyond_the_per_line_limit();
    run_clear_forgets_lines_and_counts();
    run_longest_stored_line_size_is_the_prefix_plus_the_line_limit();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
