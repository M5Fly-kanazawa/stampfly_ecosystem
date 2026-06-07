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
    // Core pipeline (core 1). State first (manages transitions), then control
    // (waits for IMU notify), then IMU (highest priority, drives the pipeline).
    // コアパイプライン (core 1)。状態管理→制御 (IMU 通知待ち)→IMU (最高優先度)。
    xTaskCreatePinnedToCore(StateTask, "StateTask",
        config::STACK_STATE, nullptr, config::PRIORITY_STATE, nullptr, 1);
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

    ESP_LOGI(TAG, "All 14 tasks started");
}

}  // namespace tasks
}  // namespace sf
