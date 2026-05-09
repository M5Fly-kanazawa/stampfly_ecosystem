/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file flow_task.cpp
 * @brief Optical flow sensor task (100Hz) — PMW3901 sharing IMU SPI bus
 *        オプティカルフローセンサタスク (100Hz) — IMU と SPI バス共有の PMW3901
 *
 * Reads PMW3901 optical flow data and publishes delta-X / delta-Y / surface
 * quality to the sensor_flow topic. The SPI bus is owned by sf_board (R1)
 * and shared with the BMI270 IMU; the wrapper Config sets skip_bus_init=true
 * so only spi_bus_add_device() runs here.
 *
 * sf_board が所有する SPI バス (BMI270 と共有) 経由で PMW3901 のオプティカル
 * フロー (delta_x / delta_y / 表面品質) を読み、sensor_flow トピックに
 * 発行する。skip_bus_init=true により本ラッパーは spi_bus_add_device()
 * のみ実行する。
 *
 * @publisher  sensor_flow
 *
 * @design architecture.md §6 — FlowTask: Sensing(OptFlow)            [OK]
 * @design detailed_design.md §8 — FlowTask: 100Hz, priority 20       [OK]
 * @design hardware_init.md §3 — sf_board が SPI bus を所有 (R1)      [OK]
 * @design topic_reference.md §3 — sensor_flow: Queue 2, 100Hz        [OK]
 *
 * Failure classification (hardware_init.md §5):
 *   - PMW3901 init failure → Optional. POS mode で必須だが ACRO/STAB
 *     /ALT は ToF + IMU で動く。本タスクは abort せず deletion のみ
 */

#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "topics.hpp"
#include "config.hpp"
#include "pmw3901_wrapper.hpp"
#include "pmw3901_exception.hpp"

static const char* TAG = "FlowTask";

/// PMW3901 driver debug breadcrumb (referenced by sf_hal_pmw3901/src/pmw3901.c).
/// The C driver writes checkpoint codes (50..54) for crash-dump analysis.
/// Define here with C linkage so the HAL's extern declaration resolves.
///
/// PMW3901 ドライバのクラッシュダンプ用ブレッドクラム
/// (sf_hal_pmw3901/src/pmw3901.c が参照)。C ドライバが checkpoint 値
/// (50..54) を書き込む。HAL の extern 宣言を解決するため C リンケージで定義。
extern "C" volatile uint8_t g_optflow_checkpoint = 0;

/// Cycle period [ticks]: 10ms = 100Hz
/// 周期 [tick]: 10ms = 100Hz
static constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(10);

/// Read-failure log throttle: at 100Hz, ~5s between warnings
/// 読み取り失敗ログ抑制: 100Hz で約 5 秒に 1 回まで警告
static constexpr uint32_t kReadFailLogIntervalCycles = 500;

/// PMW3901 instance owned by this task. unique_ptr is used because the
/// wrapper's constructor performs initialization (Mbed-style RAII) and
/// can throw — we delay construction until the task setup phase, after
/// sf_board has brought up the SPI bus.
///
/// 本タスクが所有する PMW3901 インスタンス。コンストラクタで RAII 的に
/// 初期化し、失敗時は例外を投げる Mbed スタイル設計。sf_board の SPI バス
/// 起動が済むタスク setup phase まで構築を遅延するため unique_ptr を使う。
static std::unique_ptr<stampfly::PMW3901> g_flow;

void FlowTask(void* /*pvParameters*/)
{
    ESP_LOGI(TAG, "FlowTask started");

    // -------------------------------------------------------------------
    // Setup: construct PMW3901 (skip_bus_init=true, sf_board owns the bus)
    // セットアップ: PMW3901 を構築 (skip_bus_init=true、SPI バスは sf_board 所有)
    // -------------------------------------------------------------------
    auto cfg = stampfly::PMW3901::Config::defaultStampFly();
    // defaultStampFly() は config.skip_bus_init = true を既に設定しており、
    // sf_board が SPI バスを既に立ち上げている前提と整合する。
    // defaultStampFly() already sets skip_bus_init = true, matching the
    // assumption that sf_board has already initialised the SPI bus.

    try {
        g_flow.reset(new stampfly::PMW3901(cfg));
    } catch (const stampfly::PMW3901Exception& e) {
        ESP_LOGW(TAG, "PMW3901 init failed: %s — Flow disabled (Optional)",
                 e.what());
        vTaskDelete(NULL);
        return;
    } catch (const std::exception& e) {
        ESP_LOGW(TAG, "PMW3901 init failed (std::exception): %s — Flow disabled",
                 e.what());
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "PMW3901 ready (100Hz)");

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle_count = 0;
    uint32_t last_fail_log_cycle = 0;

    while (true) {
        ++cycle_count;

        // Use the burst read (efficient: motion + quality + shutter etc.
        // in one transaction). Throws on bus errors; convert to throttled
        // warning rather than aborting the task.
        // バースト読み (motion + quality + shutter 等を 1 トランザクションで
        // 取得、効率的) を使う。バスエラー時は例外。タスク abort はせず
        // 抑制付き警告に変換する。
        try {
            stampfly::PMW3901::MotionBurst burst = g_flow->readMotionBurst();

            sf::FlowData out{};
            out.dx        = burst.delta_x;
            out.dy        = burst.delta_y;
            out.squal     = burst.squal;
            out.timestamp = static_cast<uint32_t>(esp_timer_get_time());
            sf::sensor_flow.publish(out);
        } catch (const std::exception& e) {
            if (cycle_count - last_fail_log_cycle >= kReadFailLogIntervalCycles) {
                ESP_LOGW(TAG, "PMW3901 read failed: %s", e.what());
                last_fail_log_cycle = cycle_count;
            }
        }

        vTaskDelayUntil(&last_wake, kPeriodTicks);
    }
}
