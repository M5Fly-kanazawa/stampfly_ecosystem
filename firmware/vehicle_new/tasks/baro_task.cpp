/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file baro_task.cpp
 * @brief Barometer sensor task (50Hz) — BMP280 over shared I2C
 *        気圧センサタスク (50Hz) — 共有 I2C 経由の BMP280
 *
 * Reads BMP280 (pressure / temperature / altitude) via the shared I2C
 * bus owned by sf_board, and publishes to sensor_baro topic.
 *
 * sf_board が所有する共有 I2C 経由で BMP280 (気圧 / 温度 / 高度) を
 * 読み取り、sensor_baro トピックに発行する。
 *
 * @publisher  sensor_baro
 *
 * @design architecture.md §6 — BaroTask: Sensing(Baro)               [OK]
 * @design detailed_design.md §8 — BaroTask: 50Hz, priority 16        [OK]
 * @design hardware_init.md §3 — sf_board が i2c_bus を所有 (R1)      [OK]
 * @design topic_reference.md §3 — sensor_baro: Queue 2, 50Hz         [OK]
 *
 * Failure classification (hardware_init.md §5):
 *   - BMP280 init failure → Optional. ALT mode は ToF を主観測とする
 *     ため、Baro 不在でもホバー可能。本タスクは abort せず task
 *     deletion のみ (将来 sensor_present(Baro) = false 相当)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "topics.hpp"
#include "config.hpp"
#include "sf_board.hpp"
#include "bmp280.hpp"

static const char* TAG = "BaroTask";

/// Cycle period [ticks]: 20ms = 50Hz task rate. NOTE: this task publishes every
/// cycle without a data-ready gate, but the BMP280's effective conversion rate
/// (standby MS_62_5 + oversampling) is only ~13-16Hz, so the same converted value
/// is re-published ~3x. Harmless today — baro is read but NOT fused into the
/// vertical estimate (ToF-only policy) — it only means the streamed/telemetry
/// pressure is task-rate, not sensor-rate. A data_ready gate (cf. MagTask) or a
/// shorter standby would make it true 50Hz (code_review L-21).
/// 周期 [tick]: 20ms = タスク 50Hz。注: 本タスクは data-ready ゲートなしで毎周期 publish
/// するが、BMP280 の実効変換レート（standby MS_62_5 + オーバーサンプル）は約13-16Hz の
/// ため、同一変換値が約3回再発行される。現状無害 — baro は鉛直推定に融合されない
/// （ToF-only 方針）— 配信/テレメトリの気圧がセンサレートでなくタスクレートになるだけ。
/// data_ready ゲート（MagTask 参照）または standby 短縮で実 50Hz にできる (L-21)。
static constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(20);

/// Read-failure log throttle: at 50Hz, ~5s between warnings during persistent fail
/// 読み取り失敗ログ抑制: 50Hz で持続的失敗時は約 5 秒に 1 回まで警告
static constexpr uint32_t kReadFailLogIntervalCycles = 250;

/// BMP280 wrapper instance — task-local file-scope static (no extern global).
/// BMP280 ラッパー — タスクローカルなファイルスコープ static (extern グローバル禁止)。
static stampfly::BMP280 g_baro;

/// Convert wrapper output to topic format (field names differ slightly).
/// ラッパー出力をトピック形式に変換 (フィールド名のみ異なり、意味は同じ)。
static sf::BaroData toTopic(const stampfly::BaroData& src)
{
    sf::BaroData out{};
    out.pressure    = src.pressure_pa;
    out.temperature = src.temperature_c;
    out.altitude    = src.altitude_m;
    out.timestamp   = src.timestamp_us;
    return out;
}

void BaroTask(void* /*pvParameters*/)
{
    ESP_LOGI(TAG, "BaroTask started");

    // -------------------------------------------------------------------
    // Setup: borrow I2C bus from BSP and initialise BMP280
    // セットアップ: BSP から I2C バスを借用し BMP280 を初期化
    // -------------------------------------------------------------------
    stampfly::BMP280::Config cfg{};
    cfg.i2c_bus = sf::internal::board::i2c_bus();
    // NORMAL mode + sane oversampling already set as defaults in the
    // Config struct; we keep them.
    // NORMAL モード + 妥当なオーバーサンプリングが Config 既定値として
    // 設定済みなので、そのまま使う。

    esp_err_t err = g_baro.init(cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BMP280 init failed: %s — Baro disabled (Optional)",
                 esp_err_to_name(err));
        sf::internal::board::set_sensor_present(
            sf::internal::board::SensorId::Baro, false);
        vTaskDelete(NULL);
        return;
    }
    sf::internal::board::set_sensor_present(
        sf::internal::board::SensorId::Baro, true);
    ESP_LOGI(TAG, "BMP280 ready (50Hz)");

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle_count = 0;
    uint32_t last_fail_log_cycle = 0;

    while (true) {
        ++cycle_count;

        // Read pressure / temperature / altitude in one I2C transaction
        // (BMP280 driver internally chains the necessary register reads).
        // 1 回の I2C トランザクションで気圧 / 温度 / 高度を読む
        // (BMP280 ドライバが内部でレジスタ読みを連続実行)。
        stampfly::BaroData reading{};
        err = g_baro.read(reading);
        if (err == ESP_OK) {
            const sf::BaroData topic = toTopic(reading);
            sf::sensor_baro.publish(topic);
            // Report freshness to the BSP for the 1 Hz sensor_health snapshot (R15).
            // 1Hz の sensor_health 用に鮮度を BSP へ報告する (R15)。
            sf::internal::board::set_sensor_update(
                sf::internal::board::SensorId::Baro, topic.timestamp);
        } else if (cycle_count - last_fail_log_cycle >= kReadFailLogIntervalCycles) {
            ESP_LOGW(TAG, "BMP280 read failed: %s", esp_err_to_name(err));
            last_fail_log_cycle = cycle_count;
        }

        vTaskDelayUntil(&last_wake, kPeriodTicks);
    }
}
