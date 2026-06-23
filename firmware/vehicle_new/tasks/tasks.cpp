/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file tasks.cpp
 * @brief Task startup aggregation — sf::tasks::start_all()
 *        タスク起動の集約 — sf::tasks::start_all()
 *
 * Creates all 14 FreeRTOS tasks. Keeping the xTaskCreate calls here (instead of
 * inline in app_main) keeps main.cpp declarative: app_main names the boot phases,
 * this file owns the task table. No task-handle out-parameters are taken — the
 * pipeline tasks that need to notify each other register their own handle via
 * xTaskGetCurrentTaskHandle() (R3: no extern task handles).
 *
 * 14 個の FreeRTOS タスクを生成する。xTaskCreate を app_main にベタ書きせずここに
 * 集約することで main.cpp を宣言的に保つ: app_main は起動フェーズを並べるだけ、
 * 本ファイルがタスク表を所有する。ハンドルの出力引数は取らない — 相互通知が必要な
 * パイプラインタスクは自分のハンドルを xTaskGetCurrentTaskHandle() で登録する
 * （R3: extern タスクハンドル禁止）。
 *
 * @design architecture.md §6 — Task list (14 tasks)                    [OK]
 * @design detailed_design.md §8 — Task priorities and stacks           [OK]
 * @design hardware_init.md §4 — Phase 4: tasks::start_all()            [OK]
 */

#include "tasks.hpp"
#include "config.hpp"
#include "esp_log.h"

namespace sf {
namespace tasks {

namespace {
constexpr const char* TAG = "tasks";
}  // namespace

void start_all()
{
    // Core pipeline. IMU + Control live on core 1 (the strict 400Hz loop pair).
    // StateTask runs on CORE 0: it is event-driven and lightweight, and parking
    // it BELOW the 400Hz pair on the same core proved fatal on hardware — when
    // the IMU+Control cycle time approaches the 2.5ms budget, priority 22 on
    // core 1 is starved for tens of seconds and INIT→IDLE never runs (the
    // real-hardware INIT-stuck bug, 2026-06; the SIL cannot see CPU saturation).
    // On core 0, priority 22 is the HIGHEST there, so state decisions can never
    // be starved by estimation math.
    // コアパイプライン。IMU + Control はコア1（厳格な 400Hz ループのペア）。
    // StateTask は「コア0」で走らせる: イベント駆動で軽量であり、同一コアで 400Hz
    // ペアの下に置く構成は実機で致命的だった — IMU+Control の周期が 2.5ms 予算に
    // 迫るとコア1 の優先度 22 は数十秒単位で飢餓し、INIT→IDLE が走らない（実機
    // INIT 停止バグ, 2026-06。SIL は CPU 飽和を見られない）。コア0 では 22 が最高
    // 優先度なので、状態判断が推定演算に飢餓させられることはない。
    xTaskCreatePinnedToCore(StateTask, "StateTask",
        config::STACK_STATE, nullptr, config::PRIORITY_STATE, nullptr, 0);
    xTaskCreatePinnedToCore(ControlTask, "ControlTask",
        config::STACK_CONTROL, nullptr, config::PRIORITY_CONTROL, nullptr, 1);
    xTaskCreatePinnedToCore(ImuTask, "ImuTask",
        config::STACK_IMU, nullptr, config::PRIORITY_IMU, nullptr, 1);

    // Sensor tasks (core 0).
    // センサタスク (core 0)。
    xTaskCreatePinnedToCore(FlowTask, "FlowTask",
        config::STACK_OPTFLOW, nullptr, config::PRIORITY_OPTFLOW, nullptr, 0);
    xTaskCreatePinnedToCore(MagTask, "MagTask",
        config::STACK_MAG, nullptr, config::PRIORITY_MAG, nullptr, 0);
    xTaskCreatePinnedToCore(BaroTask, "BaroTask",
        config::STACK_BARO, nullptr, config::PRIORITY_BARO, nullptr, 0);
    xTaskCreatePinnedToCore(TofTask, "TofTask",
        config::STACK_TOF, nullptr, config::PRIORITY_TOF, nullptr, 0);
    xTaskCreatePinnedToCore(PowerTask, "PowerTask",
        config::STACK_POWER, nullptr, config::PRIORITY_POWER, nullptr, 0);

    // Communication and service tasks (core 0).
    // 通信・サービスタスク (core 0)。
    xTaskCreatePinnedToCore(CommTask, "CommTask",
        config::STACK_COMM, nullptr, config::PRIORITY_COMM, nullptr, 0);
    xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask",
        config::STACK_TELEMETRY, nullptr, config::PRIORITY_TELEMETRY, nullptr, 0);
    xTaskCreatePinnedToCore(ButtonTask, "ButtonTask",
        config::STACK_BUTTON, nullptr, config::PRIORITY_BUTTON, nullptr, 0);
    xTaskCreatePinnedToCore(NotifyTask, "NotifyTask",
        config::STACK_NOTIFY, nullptr, config::PRIORITY_NOTIFY, nullptr, 0);
    xTaskCreatePinnedToCore(CLITask, "CLITask",
        config::STACK_CLI, nullptr, config::PRIORITY_CLI, nullptr, 0);
    xTaskCreatePinnedToCore(LogTask, "LogTask",
        config::STACK_LOG, nullptr, config::PRIORITY_LOG, nullptr, 0);
    xTaskCreatePinnedToCore(ApiTask, "ApiTask",
        config::STACK_API, nullptr, config::PRIORITY_API, nullptr, 0);
    // TelloStateTask is independent of ApiTask so the UDP:8890 state stream keeps
    // flowing while ApiTask blocks in a move/autotune (djitellopy reads state from
    // a background thread throughout a blocking command).
    // TelloStateTask は ApiTask から独立 — ApiTask が移動/autotune でブロック中も
    // UDP:8890 状態ストリームを流し続ける（djitellopy はブロッキングコマンド中も背景
    // スレッドで状態を読む）。
    xTaskCreatePinnedToCore(TelloStateTask, "TelloStateTask",
        config::STACK_TELLO_STATE, nullptr, config::PRIORITY_TELLO_STATE, nullptr, 0);

    ESP_LOGI(TAG, "All 16 tasks started");
}

}  // namespace tasks
}  // namespace sf
