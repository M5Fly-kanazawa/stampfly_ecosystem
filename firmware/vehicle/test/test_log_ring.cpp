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
    // An entry of 101 bytes (3 + 98) is coprime with the 1024-byte buffer, so over 1100 lines
    // the three prefix bytes land on every possible position, including 1023 | 0 (split by the end).
    // 101 バイトの項目（3 + 98）は 1024 バイトのバッファと互いに素なので、1100 行の間に接頭辞の 3 バイトは
    // あらゆる位置（末尾で割れる 1023 | 0 を含む）に来る。
    for (int i = 0; i < 1100; ++i) {
        const std::string text(98, static_cast<char>('A' + i % 26));
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
    ASSERT_TRUE(pushText(ring, text));          // 3 x 303 = 909 bytes used
    ASSERT_TRUE(!pushText(ring, text));         // 1208 > 1024: refused
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

    printf("\n=== Results: %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
