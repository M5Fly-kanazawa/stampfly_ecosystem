/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
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
 * バイナリ Blackbox レコードに記録する。Blackbox セッションは armed フラグでゲート
 * する: 1 セッション = 1 飛行（ARM→DISARM）。これにより小さな SPIFFS には地上待機で
 * なく飛行全体が残る。SPIFFS パーティションが無ければ init() が無効のままにし書き込みは
 * no-op になる（グレースフル）。
 *
 * @design architecture.md §5 — Log flow: all topics → logger          [OK]
 * @design architecture.md §6 — LogTask: Data Logger + Blackbox        [OK]
 * @design detailed_design.md §8 — LogTask: 100 Hz, priority 5         [OK]
 * @design requirements.md §7 — Blackbox (SPIFFS)                      [OK]
 * @subscriber system_mode, sensor_imu, estimate_state, control_output, actuator_motor
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "topics.hpp"
#include "config.hpp"
#include "logger.hpp"
#include "params.hpp"   // log.blackbox.enable gate / Blackbox 有効化ゲート

static const char* TAG = "LogTask";

/// Logging cadence: 100 Hz (matches the 76-byte BlackboxRecord budget).
/// ロギング周期: 100Hz（76 バイト BlackboxRecord の収支に整合）。
static constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(10);

void LogTask(void* pvParameters)
{
    ESP_LOGI(TAG, "LogTask started");

    sf::Logger logger;
    logger.init();
    logger.setMode(sf::LogMode::BLACKBOX);

    bool was_armed = false;

    while (true) {
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
        // Blackbox は log.blackbox.enable でゲート（既定 OFF）: 飛行中の SPIFFS 書込は
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
