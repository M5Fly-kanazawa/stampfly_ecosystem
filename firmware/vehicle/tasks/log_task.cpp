/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file log_task.cpp
 * @brief Data logger + Blackbox task (100 Hz)
 *        データロガー + Blackboxタスク（100Hz）
 *
 * Drives sf::Logger, which samples the IMU / estimate / control / motor topics
 * into a binary Blackbox record on SPIFFS. The Blackbox session is gated on the
 * armed flag: one session spans one flight (ARM → DISARM), so the small SPIFFS
 * partition holds whole flights instead of ground idle. If no SPIFFS partition
 * exists, Logger::init() leaves it disabled and writes become no-ops (graceful).
 *
 * sf::Logger を駆動する。Logger は IMU/推定/制御/モータの各トピックを SPIFFS 上の
 * バイナリ Blackbox レコードに記録する。Blackbox セッションは armed フラグで判定
 * する: 1 セッション = 1 飛行（ARM→DISARM）。これにより小さな SPIFFS には地上待機で
 * なく飛行全体が残る。SPIFFS パーティションが無ければ init() が無効のままにし書き込みは
 * no-op になる（グレースフル）。
 *
 * @design architecture.md §5 — Log flow: all topics → logger          [OK]
 * @design architecture.md §6 — LogTask: Data Logger + Blackbox        [OK]
 * @design detailed_design.md §8 — LogTask: 100 Hz, priority 5         [OK]
 * @design requirements.md §7 — Blackbox (SPIFFS)                      [OK]
 * @design detailed_design.md §8 — Log sink: 400 Hz tasks never wait for the console  [OK]
 * @subscriber system_mode, sensor_imu, estimate_state, control_output, actuator_motor
 */

#include <cstdarg>
#include <cstdio>
#include <mutex>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "tasks.hpp"
#include "topics.hpp"
#include "config.hpp"
#include "logger.hpp"
#include "log_ring.hpp"   // line buffer of the log sink / ログシンクの行バッファ
#include "params.hpp"   // log.blackbox.enable gate / Blackbox 有効化の条件

static const char* TAG = "LogTask";

namespace {

// -----------------------------------------------------------------------------
// Log sink: ESP_LOG callers only copy; this task writes to the console.
// ログシンク: ESP_LOG の呼び出し側はコピーするだけ。コンソールへ書くのは本タスク。
// -----------------------------------------------------------------------------

/// Ring size [bytes]. The ARM transition logs about 600 bytes at once and boot logs more;
/// 4 KB holds several such bursts while the console is slow.
/// リングの大きさ [バイト]。ARM の遷移は一度に約 600 バイト、起動時はそれ以上を出す。
/// 4 KB はコンソールが遅い間の数回分のバーストを入れられる。
constexpr size_t kSinkRingBytes = 4096;

/// Console bytes written per LogTask period. Small enough that one period is not spent
/// waiting on a slow console (a stalled USB host makes a write wait up to 50 ms; the
/// Blackbox record of this task can then miss a period).
/// LogTask の 1 周期にコンソールへ書くバイト数。1 周期を遅いコンソールの待ちで使い切らない
/// 程度に小さくする（USB のホストが止まると書き込みは最大 50 ms 待つ。その間、本タスクの
/// Blackbox 記録が 1 周期抜けうる）。
constexpr size_t kDrainBytesPerPeriod = 512;

/// Microseconds per millisecond (the ESP_LOG time stamp is in milliseconds).
/// 1 ミリ秒あたりのマイクロ秒（ESP_LOG の時刻表示はミリ秒）。
constexpr uint64_t kMicrosecondsPerMillisecond = 1000;

using SinkRing = sf::LogRing<kSinkRingBytes>;

SinkRing        s_sink_ring;                              // pending lines / 出力待ちの行
portMUX_TYPE    s_sink_lock = portMUX_INITIALIZER_UNLOCKED;  // guards s_sink_ring (any task, either core) / s_sink_ring を守る（どのタスク・どのコアからでも）
vprintf_like_t  s_console_vprintf = nullptr;              // the output function that was in place before the sink / シンクの前にあった出力関数
FILE*           s_default_stdout  = nullptr;              // stdout of LogTask = the console / LogTask の stdout ＝ コンソール
std::mutex      s_drain_mutex;                            // one drainer at a time, so lines keep their order / 取り出しは同時に 1 つだけ（行の順序を保つ）

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

/// The output function of ESP_LOG while the sink is installed. Formats the line on the
/// caller's stack and copies it into the ring; never touches the console, never waits.
/// A line longer than the ring accepts is cut and still ends with a newline.
/// Exception: a task whose stdout was redirected (the TCP CLI session of CLITask, which
/// has no deadline) gets its line written straight to that stdout, as it would without the sink.
/// シンクを入れている間の ESP_LOG の出力関数。呼び出し側のスタックで行を整形してリングへ
/// コピーする。コンソールには触れず、待たない。リングが受け付ける長さを超える行は切り、
/// 改行で終える。例外: stdout を差し替えているタスク（締切を持たない CLITask の TCP CLI
/// セッション）の行は、シンクが無い場合と同じくその stdout へ直接書く。
int sinkVprintf(const char* format, va_list arguments)
{
    if (stdout != s_default_stdout) {
        return vfprintf(stdout, format, arguments);
    }
    char line[SinkRing::kMaxLineBytes + 1];
    const int length = vsnprintf(line, sizeof(line), format, arguments);
    if (length <= 0) {
        return length;
    }
    size_t kept = static_cast<size_t>(length);
    if (kept > SinkRing::kMaxLineBytes) {
        kept = SinkRing::kMaxLineBytes;
        line[kept - 1] = '\n';
    }
    portENTER_CRITICAL_SAFE(&s_sink_lock);
    s_sink_ring.push(line, kept);
    portEXIT_CRITICAL_SAFE(&s_sink_lock);
    return length;
}

/// Take the oldest pending line out of the ring (0 = none). dropped receives the number of
/// lines lost just before that line; when the ring is empty it receives the lines lost
/// after the last one.
/// リングから最も古い出力待ちの行を取り出す（0 = なし）。dropped にはその行の直前に失われた
/// 行数が入る。リングが空のときは、最後の行より後に失われた行数が入る。
size_t takePendingLine(char* line, uint32_t* dropped)
{
    portENTER_CRITICAL_SAFE(&s_sink_lock);
    const size_t length = s_sink_ring.pop(line, SinkRing::kMaxLineBytes, dropped);
    if (length == 0) {
        *dropped = s_sink_ring.takeDropped();
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
                   static_cast<unsigned long>(esp_timer_get_time() / kMicrosecondsPerMillisecond), TAG,
                   static_cast<unsigned long>(dropped));
}

/// Write pending lines to the console, at most max_bytes (0 = no limit).
/// 出力待ちの行をコンソールへ書く。最大 max_bytes バイト（0 = 無制限）。
void drainSink(size_t max_bytes)
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
    char line[SinkRing::kMaxLineBytes + 1];
    size_t written_bytes = 0;
    while (max_bytes == 0 || written_bytes < max_bytes) {
        uint32_t dropped = 0;
        const size_t length = takePendingLine(line, &dropped);
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

/// Put the sink in place of the ESP_LOG output function (once, when LogTask starts). Lines
/// logged before this point went straight to the console.
/// ESP_LOG の出力関数をシンクに差し替える（LogTask の起動時に 1 回）。これより前に出たログは
/// 直接コンソールへ出ている。
void installSink()
{
    s_default_stdout  = stdout;   // LogTask's own stdout is the console / LogTask 自身の stdout がコンソール
    s_console_vprintf = esp_log_set_vprintf(&sinkVprintf);
}

}  // namespace

namespace sf {
namespace tasks {

void flush_log_sink()
{
    if (s_console_vprintf != nullptr) {
        drainSink(0);
    }
}

}  // namespace tasks
}  // namespace sf

/// Logging cadence: 100 Hz (matches the 76-byte BlackboxRecord budget).
/// ロギング周期: 100Hz（76 バイト BlackboxRecord の収支に整合）。
static constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(10);

void LogTask(void* pvParameters)
{
    installSink();   // from here every ESP_LOG goes through the ring / 以後の ESP_LOG はすべてリング経由
    ESP_LOGI(TAG, "LogTask started");

    sf::Logger logger;
    logger.init();
    logger.setMode(sf::LogMode::BLACKBOX);

    bool was_armed = false;

    while (true) {
        drainSink(kDrainBytesPerPeriod);   // console writes for the lines logged since the last period / 前の周期以降に出たログのコンソール出力

        // Record while armed; flush+close on the disarm edge. update() lazy-opens
        // the session file on its first call (and rotates at the size cap), so the
        // session spans exactly one flight (ARM → DISARM).
        // ARM 中は記録、DISARM 立ち下がりで flush+close。update() が初回呼び出しで
        // セッションファイルを遅延 open する（上限でローテート）ので、セッションは
        // ちょうど 1 飛行（ARM→DISARM）になる。
        const sf::SystemMode mode = sf::system_mode.latest();

        // Blackbox is gated on log.blackbox.enable (DEFAULT OFF): each in-flight SPIFFS
        // write erases flash, which disables the flash cache and STALLS BOTH CORES for
        // ~37ms — the 400Hz control loop freezes ~every 0.5s and the craft kicks (yaw).
        // Off by default so flight is stall-free; analysis uses WiFi telemetry instead.
        // Blackbox は log.blackbox.enable で判定（既定 OFF）: 飛行中の SPIFFS 書込は
        // フラッシュ消去でキャッシュ無効化し両コアを ~37ms 停止 → 400Hz 制御が約0.5秒毎に
        // 凍結し機体がキック（ヨー）。既定 OFF で飛行をストールなしに。解析は WiFi で行う。
        int32_t blackbox_enable = 0;
        sf::params::get_int("log.blackbox.enable", blackbox_enable);

        if (mode.armed && blackbox_enable != 0) {
            logger.update();
        } else if (was_armed) {
            logger.stopSession();
        }
        was_armed = mode.armed;

        vTaskDelay(kPeriodTicks);
    }
}
