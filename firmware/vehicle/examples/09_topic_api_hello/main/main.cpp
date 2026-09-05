/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file main.cpp
 * @brief Example 09 — L1 Topic API hello: read the fused attitude estimate
 *        and print roll/pitch/yaw, without touching the HAL.
 *        サンプル09 — L1 Topic API 入門: HAL に触れずに融合済み姿勢推定を読み、
 *        roll/pitch/yaw を表示する。
 *
 * Tier map for this file: the code below the ">>> L1 TOPIC API LESSON <<<"
 * banner in app_main() (and printRollPitchYaw()) is Tier L1. Everything
 * above the banner, plus internal_sensor_feed.cpp, is Tier L2 boilerplate
 * that only exists to give the L1 code real data to read — see
 * internal_sensor_feed.hpp and README.md "How this example is built".
 * 本ファイルの Tier 対応: app_main() の ">>> L1 TOPIC API LESSON <<<" バナーより
 * 下（と printRollPitchYaw()）が Tier L1。バナーより上と internal_sensor_feed.cpp
 * は、L1 コードに読ませる本物のデータを用意するためだけに存在する Tier L2 の
 * 下ごしらえ — internal_sensor_feed.hpp と README.md「この例がどう組み立てられて
 * いるか」を参照。
 *
 * Hardware: StampFly — BMI270 on SPI2 (same wiring as examples/04_read_imu)
 * ハードウェア: StampFly — SPI2 に BMI270（examples/04_read_imu と同じ配線）
 *
 * @design architecture.md §2.5 — L1: Topic API                          [OK]
 * @design architecture.md §3 — estimate_state Topic (400Hz, Latest)     [OK]
 * @design sf_api.hpp §L1 — sf::api::estimate_latest()                   [OK]
 * @design coding_and_education.md §3 — Examples: 単独ビルド可能          [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "sf_api.hpp"
#include "sf_math.hpp"
#include "topics.hpp"
#include "internal_sensor_feed.hpp"
#include "config.hpp"

static const char* TAG = "topic_api_hello";

/// Read `estimate_state` via the L1 API and print roll/pitch/yaw in degrees.
/// This IS the lesson of this example — everything else in this file (and
/// all of internal_sensor_feed.cpp) exists only to give it real data to
/// read.
/// L1 API で `estimate_state` を読み、roll/pitch/yaw を度で表示する。これが本
/// サンプルの学習内容そのもの — このファイルの他の部分（と
/// internal_sensor_feed.cpp 全体）は、読むための本物のデータを用意するためだけに
/// 存在する。
///
/// @design architecture.md §2.5 — L1 Topic API: sf::api::* のみで完結      [OK]
static void printRollPitchYaw()
{
    // ---- The 8 lines that matter for a Tier L1 learner ----
    // ---- L1 学習者にとって重要な8行 ----
    sf::StateEstimate state = sf::api::estimate_latest();
    sf::math::Quat attitude(state.attitude[0], state.attitude[1],
                             state.attitude[2], state.attitude[3]);
    sf::math::Vec3 euler_radians = attitude.to_euler();

    float roll_degrees = euler_radians.x * config::kRadToDeg;
    float pitch_degrees = euler_radians.y * config::kRadToDeg;
    float yaw_degrees = euler_radians.z * config::kRadToDeg;

    printf("roll=%+7.2f deg  pitch=%+7.2f deg  yaw=%+7.2f deg\n",
           roll_degrees, pitch_degrees, yaw_degrees);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== Example 09: L1 Topic API hello ===");

    // Every Topic buffer must exist before anything publishes or subscribes
    // (architecture.md §3 Phase 2 pattern; topic_reference.md §5.1 confirms
    // task START ORDER after this call does not matter).
    // 全 Topic バッファは publish/subscribe の前に存在している必要がある
    // （architecture.md §3 の Phase 2 パターン。topic_reference.md §5.1 により、
    // この呼び出し後ならタスクの起動順は問わない）。
    sf::topics_init();

    esp_err_t feed_result = internal_feed::init();
    if (feed_result != ESP_OK) {
        ESP_LOGE(TAG, "Sensor feed init failed — check BMI270 wiring "
                       "(see examples/04_read_imu/README.md)");
        return;
    }

    uint32_t cycle_count = 0;
    while (true) {
        // Boilerplate: keep estimate_state fresh (Tier L2 — see file banner).
        // 下ごしらえ: estimate_state を最新に保つ（Tier L2 — ファイル冒頭のバナー参照）。
        internal_feed::step(config::kSensorFeedPeriodSeconds);

        // =====================================================================
        // >>> L1 TOPIC API LESSON STARTS HERE <<<
        // >>> L1 Topic API の学習内容はここから <<<
        // =====================================================================
        if ((cycle_count % config::kSensorFeedCyclesPerPrint) == 0) {
            printRollPitchYaw();
        }

        ++cycle_count;
        vTaskDelay(pdMS_TO_TICKS(config::kSensorFeedPeriodMilliseconds));
    }
}

// ============================================================
// Try changing! / ここを変えてみよう！
// ============================================================
// 1. Also print sf::api::is_armed() and sf::api::current_mode() on the same
//    line — both are one-line L1 calls, exactly like estimate_latest().
//    同じ行で sf::api::is_armed() と sf::api::current_mode() も表示してみよう
//    — どちらも estimate_latest() と同じ1行呼び出し。
//
// 2. Change kAttitudePrintPeriodMilliseconds in config.hpp to 20 (50 Hz) and
//    watch the console: the L1 code above never changes, only the constant
//    does.
//    config.hpp の kAttitudePrintPeriodMilliseconds を 20（50Hz）に変えてコンソール
//    を見てみよう: 上の L1 コードは一切変えず、定数だけを変える。
//
// 3. Read sf::api::power_latest() too and print battery voltage alongside
//    attitude — same one-line pattern, a different Topic.
//    sf::api::power_latest() も読み、バッテリ電圧を姿勢と並べて表示してみよう
//    — 同じ1行パターンで別の Topic。
// ============================================================
