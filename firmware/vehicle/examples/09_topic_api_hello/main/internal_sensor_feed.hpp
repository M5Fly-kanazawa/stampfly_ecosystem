/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file internal_sensor_feed.hpp
 * @brief Tier L2 boilerplate — feeds real sensor data into the Topics this
 *        example reads at Tier L1. NOT part of the L1 lesson itself.
 *        Tier L2 の下ごしらえ — 本サンプルが L1 で読む Topic に実センサデータを
 *        流し込む。L1 の学習対象そのものではない。
 *
 * WHY THIS FILE EXISTS (read this first)
 * このファイルが存在する理由（まずこれを読んでほしい）
 * ------------------------------------------------------------------
 * `sf::api::estimate_latest()` (see main.cpp) only returns whatever the last
 * publisher put on the `estimate_state` Topic (sf_core/include/topics.hpp).
 * In the real vehicle firmware that publisher is `ImuTask`
 * (tasks/imu_task.cpp, ~950 lines: boot-calibration gating, failsafe, mode-
 * transition handling, ESKF/complementary-filter switch, ...). Reusing that
 * whole task is not practical for a "hello world", and it is not even
 * possible to link as-is: the real firmware's main/CMakeLists.txt compiles
 * every file under ../tasks (imu_task.cpp, control_task.cpp, ...) straight
 * into the non-swappable `main` component, not into a component this
 * standalone example project could REQUIRE.
 *
 * `sf::api::estimate_latest()`（main.cpp 参照）が返す値は、`estimate_state`
 * Topic（sf_core/include/topics.hpp）に最後に publish された値でしかない。
 * 実ファームでの publisher は `ImuTask`（tasks/imu_task.cpp、約950行: 起動校正
 * ゲート・フェイルセーフ・モード遷移処理・ESKF/相補フィルタ切替 …）。"Hello World"
 * のためにこの全体を再利用するのは現実的でなく、そのままリンクすることも不可能 —
 * 実ファームの main/CMakeLists.txt は ../tasks 配下の全ファイル（imu_task.cpp、
 * control_task.cpp、…）を差し替え不可能な `main` コンポーネントへ直接コンパイル
 * しており、本サンプルのような単体プロジェクトが REQUIRES できるコンポーネントには
 * なっていない。
 *
 * So this file implements the smallest honest subset the design allows: read
 * the real BMI270 over SPI, run ONE `IEstimator` implementation's predict()
 * step, and publish both `sensor_imu` and `estimate_state` — exactly the two
 * Topics main.cpp's L1 code depends on. There is no calibration, failsafe, or
 * takeoff/landing logic here: this build is deliberately NOT flight-capable,
 * but the numbers it publishes come from a real, moving sensor, not a canned
 * value.
 *
 * そこで本ファイルは、設計が許す「最小の正直な部分集合」を実装する: 実際の
 * BMI270 を SPI で読み、1つの `IEstimator` 実装の predict() を1ステップ実行し、
 * `sensor_imu` と `estimate_state` の2つの Topic — main.cpp の L1 コードが依存する
 * まさにその2つ — に publish する。校正・フェイルセーフ・離着陸ロジックは無く、
 * このビルドは意図的にフライト不可能だが、publish される数値は缶詰の値ではなく
 * 実際に動くセンサに由来する。
 *
 * A Tier L1 learner never has to write a file like this one — see
 * README.md, section "この例がどう組み立てられているか / How this example is
 * built" for exactly where the L1/L2 boundary is drawn.
 * L1 学習者はこのようなファイルを自分で書く必要はない — L1/L2 の境界線が
 * どこにあるかは README.md の該当節を参照。
 *
 * @design architecture.md §2.5 — 学習者の入口（4階層アクセス）、L1 vs L2   [OK]
 * @design coding_and_education.md §3 — Examples: 単独ビルド可能           [OK]
 * @design hardware_init.md §7 — L1 学習者は sf_api.hpp のみ include       [OK]
 */

#pragma once

#include "esp_err.h"

namespace internal_feed {

/// Initialize the BMI270 IMU driver and the complementary-filter estimator.
/// Call once, after sf::topics_init(), before the first step().
/// BMI270 IMU ドライバと相補フィルタ推定器を初期化する。sf::topics_init() の
/// 後、最初の step() より前に一度だけ呼ぶこと。
///
/// @return ESP_OK on success, or the BMI270 driver's error code (commonly a
///         wiring problem — see README.md "よくあるエラーと対処").
esp_err_t init();

/// Run one read-predict-publish cycle: BMI270 -> sensor_imu -> estimator ->
/// estimate_state. Call at a steady rate (main.cpp calls it every
/// config::kSensorFeedPeriodMilliseconds).
/// 1回分の「読取→予測→発行」サイクルを実行する。一定周期で呼ぶこと
/// （main.cpp は config::kSensorFeedPeriodMilliseconds 毎に呼ぶ）。
///
/// @param dt_seconds  Time since the previous step() call [s] / 前回 step() からの経過時間
void step(float dt_seconds);

}  // namespace internal_feed
