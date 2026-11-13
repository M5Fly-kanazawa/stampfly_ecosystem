/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_log_sink.cpp
 * @brief Host tests of the log sink (sf_core/log_sink.cpp): one ESP_LOG entry, two exits
 *        ログシンク（sf_core/log_sink.cpp）のホスト試験: ESP_LOG の入口は 1 つ、出口は 2 つ
 *
 * log_sink.cpp is compiled unmodified. The clock and the ESP_LOG macros come from simulator/sils/compat (the
 * same host shims the SILS uses), so ESP_LOGx reaches the sink exactly as on the vehicle. The console is a
 * function of this test that collects the text. Own TEST/ASSERT framework and main(), like the other host tests.
 *
 * What is checked, by the structure of the sink:
 *   - both exits carry the same lines in the same order (one log path);
 *   - the WiFi exit is not written while no capture is active, and nothing is counted as lost then;
 *   - a capture starts from "now" (starting empties the ring and the counts);
 *   - loss is counted exactly and carried at the place where lines went missing;
 *   - an over-long line is cut, ends with a newline and is flagged on the WiFi exit;
 *   - the byte budget of the reader leaves a long line for the next cycle.
 * log_sink.cpp は無改変でコンパイルする。時計と ESP_LOG のマクロは simulator/sils/compat（SILS と同じホスト用シム）
 * から取るので、ESP_LOGx は機体と同じ経路でシンクに届く。コンソールは文字列を集めるこの試験の関数。他のホスト試験
 * と同じ自前の TEST/ASSERT フレームワークと main()。
 *
 * @design detailed_design.md §8 — Log sink: 400 Hz tasks never wait for the console or the network  [OK]
 */

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

// Angle brackets: the quoted form would pick up the stub esp_log.h / esp_timer.h next to this file; this test wants the SILS shims.
// 山括弧: 引用符の形はこのファイルの隣のスタブ esp_log.h / esp_timer.h を拾ってしまう。この試験が使うのは SILS のシム。
#include <esp_log.h>
#include <esp_timer.h>
#include "log_sink.hpp"

namespace sils {
namespace compat {
void set_virtual_time_us(int64_t now_us);
}
}  // namespace sils

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-62s ", #name); \
        try { test_##name(); tests_passed++; printf("PASS\n"); } \
        catch (...) { tests_failed++; printf("FAIL\n"); } \
    } \
    static void test_##name()

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        printf("FAIL: %s:%d: condition false: %s\n", __FILE__, __LINE__, #cond); \
        throw 1; \
    }

namespace {

const char* const kTag = "test";

std::string g_console;   // everything the console exit wrote / コンソール側の出口が書いたものすべて

int collectConsole(const char* format, va_list arguments)
{
    char buffer[1024];
    const int length = vsnprintf(buffer, sizeof(buffer), format, arguments);
    if (length > 0) {
        g_console.append(buffer, static_cast<size_t>(length));
    }
    return length;
}

/// One line taken from the WiFi exit. / WiFi 側の出口から取り出した 1 行。
struct TakenLine {
    std::string text;
    sf::log_sink::WifiLineInfo info;
};

/// Take every line the WiFi exit holds (no budget). / WiFi 側の出口が持つ行を（予算なしで）すべて取り出す。
std::vector<TakenLine> takeAllWifiLines()
{
    std::vector<TakenLine> lines;
    char text[sf::log_sink::kMaxLineBytes];
    while (true) {
        TakenLine line;
        const size_t length = sf::log_sink::takeWifiLine(text, sizeof(text), &line.info, sf::log_sink::kMaxLineBytes);
        if (length == 0) {
            break;
        }
        line.text.assign(text, length);
        lines.push_back(line);
    }
    return lines;
}

/// Put the sink into a known state: console collected by this test, no capture, nothing pending.
/// シンクを既知の状態にする: コンソールはこの試験が集め、取得なし、待っている行なし。
void resetSink()
{
    sf::log_sink::setWifiCapture(true);    // empties the WiFi ring / WiFi 用リングを空にする
    sf::log_sink::setWifiCapture(false);
    sf::log_sink::drainToConsole(0);
    g_console.clear();
}

}  // namespace

TEST(both_exits_carry_the_same_lines_in_the_same_order)
{
    resetSink();
    sf::log_sink::setWifiCapture(true);
    ESP_LOGI(kTag, "first %d", 1);
    ESP_LOGW(kTag, "second %d", 2);
    ESP_LOGE(kTag, "third %d", 3);
    sf::log_sink::drainToConsole(0);
    const std::vector<TakenLine> lines = takeAllWifiLines();
    ASSERT_TRUE(lines.size() == 3);
    std::string joined;
    for (const TakenLine& line : lines) {
        joined += line.text;
        ASSERT_TRUE(!line.info.truncated);
        ASSERT_TRUE(line.info.dropped_before == 0);
    }
    // The same text, in the same order, through the two exits (the format of the shim's ESP_LOGx is not this test's concern).
    // 2 つの出口で同じ文字列・同じ順序（シムの ESP_LOGx の書式はこの試験の関心事ではない）。
    ASSERT_TRUE(g_console == joined);
    ASSERT_TRUE(lines[0].text.find("first 1\n") != std::string::npos);
    ASSERT_TRUE(lines[1].text.find("second 2\n") != std::string::npos);
    ASSERT_TRUE(lines[2].text.find("third 3\n") != std::string::npos);
    sf::log_sink::setWifiCapture(false);
}

TEST(vehicle_time_travels_with_each_line_in_time_order)
{
    resetSink();
    sf::log_sink::setWifiCapture(true);
    sils::compat::set_virtual_time_us(1000);
    ESP_LOGI(kTag, "a");
    sils::compat::set_virtual_time_us(2500);
    ESP_LOGI(kTag, "b");
    sils::compat::set_virtual_time_us(0x1FFFFFFFFLL);   // above 32 bits: the low 32 bits go on the wire / 32 ビットを超える: 下位 32 ビットを送る
    ESP_LOGI(kTag, "c");
    const std::vector<TakenLine> lines = takeAllWifiLines();
    ASSERT_TRUE(lines.size() == 3);
    ASSERT_TRUE(lines[0].info.timestamp_us == 1000);
    ASSERT_TRUE(lines[1].info.timestamp_us == 2500);
    ASSERT_TRUE(lines[2].info.timestamp_us == 0xFFFFFFFFu);
    sf::log_sink::setWifiCapture(false);
    sils::compat::set_virtual_time_us(0);
}

TEST(no_capture_means_no_copy_and_no_loss_report)
{
    resetSink();
    // Far more than the WiFi ring holds: if the exit were written, it would overflow and count drops.
    // WiFi 用リングが抱えるよりはるかに多い: 出口に書いていれば溢れて落とし数を数えるはず。
    for (int i = 0; i < 200; ++i) {
        ESP_LOGI(kTag, "line %d %0200d", i, 0);
        sf::log_sink::drainToConsole(0);   // the console keeps up / コンソールは追いつく
    }
    ASSERT_TRUE(sf::log_sink::wifiDroppedTotal() == 0);
    ASSERT_TRUE(takeAllWifiLines().empty());
    // A capture that starts now sees nothing of the lines above. / 今始める取得は、上の行を何も見ない。
    sf::log_sink::setWifiCapture(true);
    ASSERT_TRUE(takeAllWifiLines().empty());
    ASSERT_TRUE(sf::log_sink::wifiDroppedTotal() == 0);
    sf::log_sink::setWifiCapture(false);
}

TEST(a_capture_starts_from_now_and_stopping_stops_the_copy)
{
    resetSink();
    sf::log_sink::setWifiCapture(true);
    ESP_LOGI(kTag, "old");
    sf::log_sink::setWifiCapture(true);   // restart: the pending line and the counts are forgotten / 再開: 待っている行と数は忘れる
    ESP_LOGI(kTag, "new");
    std::vector<TakenLine> lines = takeAllWifiLines();
    ASSERT_TRUE(lines.size() == 1 && lines[0].text.find("new\n") != std::string::npos);
    sf::log_sink::setWifiCapture(false);
    ESP_LOGI(kTag, "after stop");
    ASSERT_TRUE(takeAllWifiLines().empty());
    sf::log_sink::drainToConsole(0);
    ASSERT_TRUE(g_console.find("after stop") != std::string::npos);   // the console exit is unaffected / コンソール側の出口は影響を受けない
}

TEST(a_log_storm_is_counted_exactly_and_reported_at_the_place_it_happened)
{
    resetSink();
    sf::log_sink::setWifiCapture(true);
    constexpr int kLines = 60;                 // 60 x ~270 bytes: more than the 4 KB ring holds / 4 KB のリングより多い
    for (int i = 0; i < kLines; ++i) {
        ESP_LOGI(kTag, "storm %02d %0250d", i, 0);
    }
    const std::vector<TakenLine> before_gap = takeAllWifiLines();   // what the ring kept / リングが残したもの
    const uint32_t refused = sf::log_sink::wifiDroppedTotal();
    ASSERT_TRUE(!before_gap.empty());
    ASSERT_TRUE(refused > 0);
    ASSERT_TRUE(before_gap.size() + refused == static_cast<size_t>(kLines));   // every line is either delivered or counted / すべての行が、届くか数えられるかのどちらか
    ASSERT_TRUE(before_gap.front().text.find("storm 00") != std::string::npos);   // the oldest are kept, the newest refused / 古い行が残り、新しい行が断られる
    // The next line carries the number of lines lost before it. / 次の行は、その前に失われた行数を運ぶ。
    ESP_LOGI(kTag, "after the storm");
    const std::vector<TakenLine> after_gap = takeAllWifiLines();
    ASSERT_TRUE(after_gap.size() == 1);
    ASSERT_TRUE(after_gap[0].info.dropped_before == refused);
    ASSERT_TRUE(sf::log_sink::wifiDroppedTotal() == refused);   // the total is not reduced by delivery / 累計は送っても減らない
    // The console exit reports its own loss at the same kind of place. / コンソール側の出口も同じように自分の欠落を報告する。
    sf::log_sink::drainToConsole(0);
    ASSERT_TRUE(g_console.find("log sink full:") != std::string::npos);
    sf::log_sink::setWifiCapture(false);
}

TEST(an_over_long_line_is_cut_ends_with_a_newline_and_is_flagged)
{
    resetSink();
    sf::log_sink::setWifiCapture(true);
    const std::string very_long(900, 'x');
    ESP_LOGI(kTag, "%s", very_long.c_str());
    ESP_LOGI(kTag, "short");
    const std::vector<TakenLine> lines = takeAllWifiLines();
    ASSERT_TRUE(lines.size() == 2);
    ASSERT_TRUE(lines[0].info.truncated);
    ASSERT_TRUE(lines[0].text.size() == sf::log_sink::kMaxLineBytes);
    ASSERT_TRUE(lines[0].text.back() == '\n');
    ASSERT_TRUE(lines[0].text.find("xxxxxxxxxx") != std::string::npos);
    ASSERT_TRUE(!lines[1].info.truncated);                       // the flag belongs to the cut line only / フラグは切った行だけのもの
    sf::log_sink::drainToConsole(0);
    ASSERT_TRUE(g_console.find(std::string(100, 'x')) != std::string::npos);   // the console shows the cut line too / コンソールも切った行を表示する
    sf::log_sink::setWifiCapture(false);
}

TEST(the_readers_byte_budget_leaves_a_longer_line_for_the_next_packet)
{
    resetSink();
    sf::log_sink::setWifiCapture(true);
    ESP_LOGI(kTag, "%s", std::string(200, 'y').c_str());
    char text[sf::log_sink::kMaxLineBytes];
    sf::log_sink::WifiLineInfo info;
    ASSERT_TRUE(sf::log_sink::takeWifiLine(text, sizeof(text), &info, 100) == 0);   // too long for the budget: it stays / 予算に収まらない: 残る
    const size_t length = sf::log_sink::takeWifiLine(text, sizeof(text), &info, 300);
    ASSERT_TRUE(length > 200);                                                     // the next cycle's budget takes it / 次の周期の予算で取れる
    ASSERT_TRUE(sf::log_sink::takeWifiLine(text, sizeof(text), &info, 300) == 0);   // now empty / 空になった
    sf::log_sink::setWifiCapture(false);
}

int main()
{
    printf("=== log sink (ESP_LOG -> console / WiFi capture) host tests ===\n");
    // The console is this test's collector; install() takes over from it, exactly as on the vehicle.
    // コンソールはこの試験の収集関数。install() が、機体と同じようにそれを引き継ぐ。
    esp_log_set_vprintf(&collectConsole);
    sf::log_sink::install();
    run_both_exits_carry_the_same_lines_in_the_same_order();
    run_vehicle_time_travels_with_each_line_in_time_order();
    run_no_capture_means_no_copy_and_no_loss_report();
    run_a_capture_starts_from_now_and_stopping_stops_the_copy();
    run_a_log_storm_is_counted_exactly_and_reported_at_the_place_it_happened();
    run_an_over_long_line_is_cut_ends_with_a_newline_and_is_flagged();
    run_the_readers_byte_budget_leaves_a_longer_line_for_the_next_packet();

    printf("\n=== Results: %d run, %d passed, %d failed ===\n", tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
