/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file log_sink.hpp
 * @brief The log sink: one ESP_LOG entry, two exits (the USB console and the WiFi capture)
 *        ログシンク: ESP_LOG の入口は 1 つ、出口は 2 つ（USB コンソールと WiFi 取得）
 *
 * Every ESP_LOG line of the firmware goes through one output function (sinkVprintf in log_sink.cpp).
 * A caller — any task, ImuTask and ControlTask included — only formats the line on its own stack and
 * COPIES it (never touches the console or the network, never waits):
 *   - into the console ring, always. LogTask (priority 5, no deadline) writes it to the USB console.
 *   - into the WiFi ring, only while a capture is active (`sf log wifi`). TelemetryTask (the owner of
 *     the Data Stream socket) takes the lines out and sends them, at most every 100 ms and a bounded
 *     number of bytes per packet, with a non-blocking sendto.
 * Both copies are made inside the same short critical section (a spin lock, not a mutex, never held
 * across a console write or a send). So the console and the WiFi capture are two exits of the same
 * sink, not two logging mechanisms.
 *
 * Loss is never silent: a ring that is full refuses the line and counts it; the count is stored with
 * the next line that enters the ring (the console prints "log sink full: N lines dropped" at that
 * place; the WiFi exit sends it as the line's `dropped_before`), and the running total of the WiFi
 * ring goes to the PC in the status packet. A line longer than the ring accepts is cut, ends with a
 * newline, and carries the truncation flag.
 *
 * While no capture is active the WiFi ring is not written at all: no copy, no count. A capture starts
 * from "now": setWifiCapture(true) empties the ring and zeroes its counts, so lines logged while no
 * capture was active are not delivered and are not reported as lost.
 *
 * ESP_LOG の全行は 1 つの出力関数（log_sink.cpp の sinkVprintf）を通る。呼び出し側（どのタスクでも。
 * ImuTask・ControlTask を含む）は自分のスタックで行を整形して「コピーするだけ」で、コンソールにもネット
 * ワークにも触れず、待たない:
 *   - コンソール用リングへ、常に。LogTask（優先度 5、締切なし）が USB コンソールへ書く。
 *   - WiFi 用リングへ、取得が有効な間だけ（`sf log wifi`）。TelemetryTask（Data Stream のソケットの
 *     持ち主）が行を取り出し、最大 100 ms ごとに、1 パケットあたり有限のバイト数を、ノンブロッキングの sendto で送る。
 * 2 つのコピーは同じ短いクリティカルセクション（スピンロック。ミューテックスではなく、コンソールへの
 * 書き込みや送信をまたいで保持しない）の中で行う。つまりコンソールと WiFi 取得は同じシンクの 2 つの出口で
 * あり、2 つのログ機構ではない。
 *
 * 欠落は黙って起こさない: 満杯のリングは行を断って数える。数は次にリングへ入る行と一緒に格納され
 * （コンソールはその位置に「log sink full: N lines dropped」を出す。WiFi 側は行の `dropped_before` として
 * 送る）、WiFi 用リングの累計はステータスパケットで PC へ届く。リングが受け付ける長さを超える行は切り、改行で
 * 終え、切り詰めのフラグを付ける。
 *
 * 取得が有効でない間、WiFi 用リングには書かない: コピーも計数もしない。取得は「今」から始まる:
 * setWifiCapture(true) がリングを空にして数を 0 に戻すので、取得が無かった間のログは届かず、欠落としても
 * 報告されない。
 *
 * @design detailed_design.md §8 — Log sink: 400 Hz tasks never wait for the console or the network  [OK]
 * @design architecture.md §5 — Log flow: ring → console and ring → WiFi are two exits of one sink     [OK]
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "log_ring.hpp"

namespace sf {
namespace log_sink {

/// Size of the console ring [bytes]. The ARM transition logs about 600 bytes at once and boot logs
/// more; 4 KB holds several such bursts while the console is slow.
/// コンソール用リングの大きさ [バイト]。ARM の遷移は一度に約 600 バイト、起動時はそれ以上を出す。
/// 4 KB はコンソールが遅い間の数回分のバーストを入れられる。
inline constexpr size_t kConsoleRingBytes = 4096;

/// Size of the WiFi ring [bytes]. The sender takes at most kTextLogBytesPerPacket bytes of records
/// from it every 100 ms (data_stream_wire.hpp), so 4 KB is about 4 packets (400 ms) of the largest
/// drain, several times the ARM burst; a log storm that outruns the sender is counted, not hidden.
/// WiFi 用リングの大きさ [バイト]。送信側は 100 ms ごとに最大 kTextLogBytesPerPacket バイトのレコードを取り出す
/// （data_stream_wire.hpp）ので、4 KB は最大の排出で約 4 パケット（400 ms）分で、ARM のバーストの数倍。送信側を
/// 追い越すログの嵐は、隠さず数える。
inline constexpr size_t kWifiRingBytes = 4096;

using ConsoleRing = LogRing<kConsoleRingBytes>;
using WifiRing    = LogRing<kWifiRingBytes>;

/// Longest line text the sink stores (the longest line of either ring). / シンクが格納する行の本文の最大長。
inline constexpr size_t kMaxLineBytes = WifiRing::kMaxLineBytes;
static_assert(ConsoleRing::kMaxLineBytes == WifiRing::kMaxLineBytes, "both rings store the same line / 2 つのリングは同じ行を格納する");

/// What one line of the WiFi ring carries besides its text. / WiFi 用リングの 1 行が本文のほかに持つもの。
struct WifiLineInfo {
    uint32_t timestamp_us   = 0;       ///< [us] esp_timer time (low 32 bits) when the line entered the ring / 行がリングに入った時刻（esp_timer の下位 32 ビット）
    bool     truncated      = false;   ///< the text was cut to kMaxLineBytes / 本文が kMaxLineBytes に切り詰められた
    uint32_t dropped_before = 0;       ///< lines refused just before this one (up to 255) / この行の直前に断った行数（最大 255）
};

/// Put the sink in place of the ESP_LOG output function (once, when LogTask starts). Lines logged
/// before this point went straight to the console.
/// ESP_LOG の出力関数をシンクに差し替える（LogTask の起動時に 1 回）。これより前に出たログは直接コンソールへ
/// 出ている。
void install();

/// True once install() has run. / install() が実行済みなら true。
bool installed();

/// Write pending lines of the console ring to the console, at most max_bytes (0 = no limit). Only
/// LogTask calls it in normal operation (a console write can wait); flushToConsole() calls it with 0.
/// コンソール用リングの出力待ちの行をコンソールへ書く。最大 max_bytes バイト（0 = 無制限）。通常は LogTask
/// だけが呼ぶ（コンソールへの書き込みは待たされうる）。flushToConsole() は 0 で呼ぶ。
void drainToConsole(size_t max_bytes);

/// Write every pending line to the console now (blocks until done). Does nothing before install().
/// 出力待ちの行をすべて今コンソールへ書く（終わるまで待つ）。install() より前は何もしない。
void flushToConsole();

/// Start (true) or stop (false) feeding the WiFi ring. Starting empties the ring and zeroes its
/// counts. Called by the Data Stream when a capture starts or stops.
/// WiFi 用リングへの書き込みを始める（true）／止める（false）。始めるときはリングを空にして数を 0 に戻す。
/// Data Stream が取得の開始・停止で呼ぶ。
void setWifiCapture(bool active);

/// Take the oldest line out of the WiFi ring when its text is at most max_text_bytes long (0 = none,
/// or the oldest line is longer than the budget and stays for the next cycle). text must have room for
/// kMaxLineBytes. Copies under the sink's spin lock (about a microsecond for a typical line); the
/// caller sends outside it.
/// WiFi 用リングの最古の行を、本文が max_text_bytes 以下のとき取り出す（0 = なし、または最古の行が max_text_bytes より
/// 長く次の周期に残る）。text には kMaxLineBytes の領域が要る。シンクのスピンロックの下でコピーする（普通の行
/// で 1 µs 程度）。送信は呼び出し側がロックの外で行う。
size_t takeWifiLine(char* text, size_t text_capacity, WifiLineInfo* info, size_t max_text_bytes);

/// Lines the WiFi ring refused since the capture started, in total. / 取得開始以降に WiFi 用リングが断った行数の合計。
uint32_t wifiDroppedTotal();

}  // namespace log_sink
}  // namespace sf
