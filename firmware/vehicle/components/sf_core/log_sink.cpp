/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file log_sink.cpp
 * @brief The log sink (see log_sink.hpp): ESP_LOG callers only copy; LogTask writes the console,
 *        TelemetryTask sends the WiFi capture
 *        ログシンク（log_sink.hpp 参照）: ESP_LOG の呼び出し側はコピーするだけ。コンソールへ書くのは
 *        LogTask、WiFi 取得を送るのは TelemetryTask
 *
 * Time added to an ESP_LOG caller by the WiFi exit, worst case: one more memcpy of at most
 * kMaxLineBytes + 8 bytes (about 2 microseconds at the ESP32-S3 copy speed of 200 MB/s, well under 1
 * microsecond for a typical 100-byte line) and one read of esp_timer (below 1 microsecond), inside the
 * critical section that already exists. Both are bounded by the line length limit; with no capture
 * active the added cost is one boolean read.
 * WiFi 側の出口が ESP_LOG の呼び出し側に足す時間（最悪）: 最大 kMaxLineBytes + 8 バイトの memcpy をもう 1 回
 * （ESP32-S3 のコピー速度 200 MB/s で約 2 µs。普通の 100 バイトの行なら 1 µs を十分下回る）と、esp_timer の
 * 読み出し 1 回（1 µs 未満）。どちらも既にあるクリティカルセクションの中で、行の長さの上限で抑えられる。
 * 取得が有効でない間に足されるのは真偽値の読み出し 1 回だけ。
 *
 * @design detailed_design.md §8 — Log sink: 400 Hz tasks never wait for the console or the network  [OK]
 * @design architecture.md §5 — Log flow: ring → console and ring → WiFi are two exits of one sink     [OK]
 */

#include "log_sink.hpp"

#include <cstdarg>
#include <cstdio>
#include <mutex>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace sf {
namespace log_sink {

namespace {

/// Tag of the "log sink full" notice. / 「log sink full」通知のタグ。
const char* const kNoticeTag = "LogTask";

/// Microseconds per millisecond (the ESP_LOG time stamp is in milliseconds).
/// 1 ミリ秒あたりのマイクロ秒（ESP_LOG の時刻表示はミリ秒）。
constexpr uint64_t kMicrosecondsPerMillisecond = 1000;

ConsoleRing     s_console_ring;                                 // lines waiting for the console / コンソールへの出力待ちの行
WifiRing        s_wifi_ring;                                    // lines waiting for the WiFi capture / WiFi 取得への送信待ちの行
bool            s_wifi_capture_active = false;                  // a capture is active; guarded by s_sink_lock / 取得が有効（s_sink_lock で守る）
portMUX_TYPE    s_sink_lock = portMUX_INITIALIZER_UNLOCKED;     // guards both rings (any task, either core) / 両方のリングを守る（どのタスク・どのコアからでも）
vprintf_like_t  s_console_vprintf = nullptr;                    // the output function that was in place before the sink / シンクの前にあった出力関数
FILE*           s_default_stdout  = nullptr;                    // stdout of LogTask = the console / LogTask の stdout ＝ コンソール
std::mutex      s_drain_mutex;                                  // one drainer at a time, so lines keep their order / 取り出しは同時に 1 つだけ（行の順序を保つ）

/// Call the console output function with a literal format (a vprintf-style function takes a
/// va_list, so the arguments are wrapped here).
/// コンソールの出力関数をリテラルの書式で呼ぶ（vprintf 形式の関数は va_list を取るので、
/// ここで引数を包む）。
int writeToConsole(const char* format, ...)
{
    va_list arguments;
    va_start(arguments, format);
    const int written = s_console_vprintf(format, arguments);
    va_end(arguments);
    return written;
}

/// Copy one finished line into the console ring and, while a capture is active, into the WiFi ring.
/// One critical section, one time stamp (read inside it, so the lines of the WiFi ring are in time
/// order even when two cores log at once). Never waits for anything but this spin lock.
/// 出来上がった 1 行をコンソール用リングへ、取得が有効な間は WiFi 用リングへもコピーする。クリティカル
/// セクションは 1 つ、時刻も 1 つ（クリティカルセクションの中で読むので、2 つのコアが同時にログを出しても
/// WiFi 用リングの行は時刻順）。このスピンロック以外は何も待たない。
void copyLineIntoRings(const char* line, size_t length, bool truncated)
{
    portENTER_CRITICAL_SAFE(&s_sink_lock);
    const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());
    s_console_ring.push(line, length, now_us, truncated);
    if (s_wifi_capture_active) {
        s_wifi_ring.push(line, length, now_us, truncated);
    }
    portEXIT_CRITICAL_SAFE(&s_sink_lock);
}

/// The output function of ESP_LOG while the sink is installed. Formats the line on the
/// caller's stack and copies it into the rings; never touches the console or the network, never waits.
/// A line longer than the rings accept is cut, still ends with a newline, and is flagged as truncated.
/// Exception: a task whose stdout was redirected (the TCP CLI session of CLITask, which
/// has no deadline) gets its line written straight to that stdout, as it would without the sink.
/// シンクを入れている間の ESP_LOG の出力関数。呼び出し側のスタックで行を整形してリングへ
/// コピーする。コンソールにもネットワークにも触れず、待たない。リングが受け付ける長さを超える行は切り、
/// 改行で終え、切り詰めのフラグを付ける。例外: stdout を差し替えているタスク（締切を持たない CLITask の
/// TCP CLI セッション）の行は、シンクが無い場合と同じくその stdout へ直接書く。
int sinkVprintf(const char* format, va_list arguments)
{
    if (stdout != s_default_stdout) {
        return vfprintf(stdout, format, arguments);
    }
    char line[kMaxLineBytes + 1];
    const int length = vsnprintf(line, sizeof(line), format, arguments);
    if (length <= 0) {
        return length;
    }
    size_t kept = static_cast<size_t>(length);
    const bool truncated = kept > kMaxLineBytes;
    if (truncated) {
        kept = kMaxLineBytes;
        line[kept - 1] = '\n';
    }
    copyLineIntoRings(line, kept, truncated);
    return length;
}

/// Take the oldest pending line out of the console ring (0 = none). dropped receives the number of
/// lines lost just before that line; when the ring is empty it receives the lines lost
/// after the last one.
/// コンソール用リングから最も古い出力待ちの行を取り出す（0 = なし）。dropped にはその行の直前に失われた
/// 行数が入る。リングが空のときは、最後の行より後に失われた行数が入る。
size_t takePendingConsoleLine(char* line, uint32_t* dropped)
{
    portENTER_CRITICAL_SAFE(&s_sink_lock);
    const size_t length = s_console_ring.pop(line, kMaxLineBytes, dropped);
    if (length == 0) {
        *dropped = s_console_ring.takeDropped();
    }
    portEXIT_CRITICAL_SAFE(&s_sink_lock);
    return length;
}

/// Tell the console that a burst larger than the ring was cut (at the place where the lines went missing).
/// リングより大きいバーストを切り捨てたことをコンソールに知らせる（行が抜けた位置で）。
void reportDroppedLines(uint32_t dropped)
{
    if (dropped == 0) {
        return;
    }
    writeToConsole("W (%lu) %s: log sink full: %lu lines dropped\n",
                   static_cast<unsigned long>(esp_timer_get_time() / kMicrosecondsPerMillisecond), kNoticeTag,
                   static_cast<unsigned long>(dropped));
}

}  // namespace

void install()
{
    s_default_stdout  = stdout;   // LogTask's own stdout is the console / LogTask 自身の stdout がコンソール
    s_console_vprintf = esp_log_set_vprintf(&sinkVprintf);
}

bool installed()
{
    return s_console_vprintf != nullptr;
}

void drainToConsole(size_t max_bytes)
{
    const std::lock_guard<std::mutex> one_drainer_at_a_time(s_drain_mutex);
    // The console output function writes to the stdout of the task that calls it. A caller
    // whose stdout was redirected (`reboot` typed on a TCP CLI session) must still send the
    // pending lines to the console, so stdout points at the console while draining.
    // コンソールの出力関数は、呼び出したタスクの stdout へ書く。stdout を差し替えている呼び出し側
    // （TCP CLI セッションで打った `reboot`）でも、出力待ちの行はコンソールへ出す必要があるので、
    // 取り出しの間だけ stdout をコンソールに向ける。
    FILE* const callers_stdout = stdout;
    stdout = s_default_stdout;
    char line[kMaxLineBytes + 1];
    size_t written_bytes = 0;
    while (max_bytes == 0 || written_bytes < max_bytes) {
        uint32_t dropped = 0;
        const size_t length = takePendingConsoleLine(line, &dropped);
        reportDroppedLines(dropped);
        if (length == 0) {
            break;
        }
        line[length] = '\0';
        writeToConsole("%s", line);
        written_bytes += length;
    }
    stdout = callers_stdout;
}

void flushToConsole()
{
    if (installed()) {
        drainToConsole(0);
    }
}

void setWifiCapture(bool active)
{
    portENTER_CRITICAL_SAFE(&s_sink_lock);
    if (active) {
        s_wifi_ring.clear();   // a capture starts from "now" / 取得は「今」から始まる
    }
    s_wifi_capture_active = active;
    portEXIT_CRITICAL_SAFE(&s_sink_lock);
}

size_t takeWifiLine(char* text, size_t text_capacity, WifiLineInfo* info, size_t max_text_bytes)
{
    size_t length = 0;
    portENTER_CRITICAL_SAFE(&s_sink_lock);
    const size_t oldest = s_wifi_ring.oldestLength();
    if (oldest != 0 && oldest <= max_text_bytes) {
        WifiRing::LineInfo stored;
        length = s_wifi_ring.pop(text, text_capacity, &info->dropped_before, &stored);
        info->timestamp_us = stored.timestamp_us;
        info->truncated    = stored.truncated;
    }
    portEXIT_CRITICAL_SAFE(&s_sink_lock);
    return length;
}

uint32_t wifiDroppedTotal()
{
    portENTER_CRITICAL_SAFE(&s_sink_lock);
    const uint32_t total = s_wifi_ring.totalDropped();
    portEXIT_CRITICAL_SAFE(&s_sink_lock);
    return total;
}

}  // namespace log_sink
}  // namespace sf
