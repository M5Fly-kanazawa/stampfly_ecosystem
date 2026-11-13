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
 * LogTask also installs the log sink (sf_core/log_sink.cpp) when it starts and writes its console
 * exit every period; the WiFi exit of the same sink is read by TelemetryTask (Data Stream).
 * LogTask は起動時にログシンク（sf_core/log_sink.cpp）を入れ、毎周期そのコンソール側の出口を書く。同じシンクの
 * WiFi 側の出口は TelemetryTask（Data Stream）が読む。
 *
 * @design architecture.md §5 — Log flow: all topics → logger          [OK]
 * @design architecture.md §6 — LogTask: Data Logger + Blackbox        [OK]
 * @design detailed_design.md §8 — LogTask: 100 Hz, priority 5         [OK]
 * @design requirements.md §7 — Blackbox (SPIFFS)                      [OK]
 * @design detailed_design.md §8 — Log sink: 400 Hz tasks never wait for the console  [OK]
 * @subscriber system_mode, sensor_imu, estimate_state, control_output, actuator_motor
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tasks.hpp"
#include "topics.hpp"
#include "config.hpp"
#include "logger.hpp"
#include "log_sink.hpp"   // the log sink (ESP_LOG → rings → console / WiFi) / ログシンク（ESP_LOG → リング → コンソール / WiFi）
#include "params.hpp"   // log.blackbox.enable gate / Blackbox 有効化の条件

static const char* TAG = "LogTask";

namespace {

/// Console bytes written per LogTask period. Small enough that one period is not spent
/// waiting on a slow console (a stalled USB host makes a write wait up to 50 ms; the
/// Blackbox record of this task can then miss a period).
/// LogTask の 1 周期にコンソールへ書くバイト数。1 周期を遅いコンソールの待ちで使い切らない
/// 程度に小さくする（USB のホストが止まると書き込みは最大 50 ms 待つ。その間、本タスクの
/// Blackbox 記録が 1 周期抜けうる）。
constexpr size_t kDrainBytesPerPeriod = 512;

}  // namespace

namespace sf {
namespace tasks {

void flush_log_sink()
{
    log_sink::flushToConsole();
}

}  // namespace tasks
}  // namespace sf

/// Logging cadence: 100 Hz (matches the 76-byte BlackboxRecord budget).
/// ロギング周期: 100Hz（76 バイト BlackboxRecord の収支に整合）。
static constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(10);

void LogTask(void* pvParameters)
{
    sf::log_sink::install();   // from here every ESP_LOG goes through the sink / 以後の ESP_LOG はすべてシンク経由
    ESP_LOGI(TAG, "LogTask started");

    sf::Logger logger;
    logger.init();
    logger.setMode(sf::LogMode::BLACKBOX);

    bool was_armed = false;

    while (true) {
        sf::log_sink::drainToConsole(kDrainBytesPerPeriod);   // console writes for the lines logged since the last period / 前の周期以降に出たログのコンソール出力

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
