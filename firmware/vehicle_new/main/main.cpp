/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file main.cpp
 * @brief StampFly vehicle_new — Main Entry Point
 *        StampFly vehicle_new — メインエントリポイント
 *
 * Declarative boot: app_main only names the five boot phases (NVS → BSP →
 * topics → params → tasks). Each phase's "how" lives in its owning module
 * (sf_board::init, topics_init, params::init, tasks::start_all). There are no
 * extern task handles — the pipeline tasks register their own (R3).
 *
 * 宣言的起動: app_main は 5 つの起動フェーズ（NVS → BSP → トピック → params →
 * タスク）を並べるだけ。各フェーズの「どう」は所有モジュールにある。extern タスク
 * ハンドルは持たない — パイプラインタスクが自分のハンドルを登録する（R3）。
 *
 * @design hardware_init.md §4 — 起動シーケンス (Phase 0..4)           [OK]
 * @design architecture.md §4 — State machine: INIT → IDLE             [--]
 * @design detailed_design.md §3 — onEnter(IDLE): start calibration    [--]
 */

#include "esp_log.h"
#include "nvs_flash.h"

#include "topics.hpp"
#include "params.hpp"
#include "tasks.hpp"
#include "sf_board.hpp"

static const char* TAG = "main";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== StampFly vehicle_new starting ===");

    // =========================================================================
    // Phase 0: NVS (required by WiFi and parameter persistence)
    // Phase 0: NVS（WiFi とパラメータ永続化に必要）
    // =========================================================================
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Phase 0: NVS initialized");

    // =========================================================================
    // Phase 1: BSP — sf_board owns every shared HW resource (I2C/SPI/LEDC/netif).
    // Each HAL task borrows handles from sf_board getters after Phase 4.
    // Phase 1: BSP — sf_board が共有 HW 資源を一元所有。各 HAL タスクは Phase 4 後に
    // sf_board の getter からハンドルを借用する。
    //
    // @design hardware_init.md §3/§4 — sf_board ownership + boot sequence  [OK]
    // =========================================================================
    ESP_ERROR_CHECK(sf::internal::board::init());
    ESP_LOGI(TAG, "Phase 1: BSP ready (sf_board)");

    // =========================================================================
    // Phase 2: Pub-Sub topics (buffers/mutexes ready before any task runs).
    // Phase 2: Pub-Sub トピック（タスク起動前にバッファ/mutex を準備）。
    // =========================================================================
    sf::topics_init();
    ESP_LOGI(TAG, "Phase 2: Topics initialized");

    // =========================================================================
    // Phase 3: Parameters — restore tuned values from NVS, else compile-time
    // defaults (params.cpp table[]). Must run before tasks read parameters.
    // Phase 3: パラメータ — NVS から調整値を復元、無ければコンパイル時デフォルト
    // （params.cpp の table[]）。タスクがパラメータを読む前に実行する。
    //
    // @design detailed_design.md §6 — parameter system                    [OK]
    // =========================================================================
    sf::params::init();
    ESP_LOGI(TAG, "Phase 3: Parameters loaded");

    // =========================================================================
    // Phase 4: Tasks — create all 14 FreeRTOS tasks (see sf::tasks::start_all).
    // Phase 4: タスク — 14 個の FreeRTOS タスクを生成（sf::tasks::start_all 参照）。
    // =========================================================================
    sf::tasks::start_all();
    ESP_LOGI(TAG, "=== Phase 4: tasks started — INIT complete ===");
}
