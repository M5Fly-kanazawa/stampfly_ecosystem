/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (workshop firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file workshop_tasks.cpp
 * @brief Task startup aggregation — sf::workshop_tasks::start_all()
 *        タスク起動の集約 — sf::workshop_tasks::start_all()
 *
 * A near-verbatim copy of vehicle/tasks/tasks.cpp's start_all(). The ONLY
 * difference from vehicle is ControlTask -> WorkshopControlTask (the learner
 * setup()/loop_400Hz() task, workshop_control_task.cpp); every stack size,
 * priority and core assignment stays identical (config::STACK_CONTROL /
 * config::PRIORITY_CONTROL / core 1), so the timing behaviour students see
 * matches vehicle exactly.
 *
 * vehicle/tasks/tasks.cpp の start_all() をほぼそのまま複製したもの。vehicle
 * との差分は ControlTask → WorkshopControlTask（学習者の setup()/loop_400Hz()
 * を呼ぶタスク、workshop_control_task.cpp）の 1 点のみ。スタックサイズ・優先度・
 * コア割当（config::STACK_CONTROL / config::PRIORITY_CONTROL / core 1）は
 * 全て同一のため、学生が見るタイミング挙動は vehicle と一致する。
 *
 * @design W1_SPEC.md §2-4 — workshop_tasks.cpp                        [OK]
 * @design architecture.md §6 — Task list (14 tasks)                    [OK]
 */

#include "tasks.hpp"    // vehicle/tasks/tasks.hpp — reused task declarations
#include "config.hpp"
#include "esp_log.h"

/// Workshop replacement for vehicle's ControlTask — calls the learner's
/// setup()/loop_400Hz() instead of running sf::PidController. Defined in
/// workshop_control_task.cpp.
/// vehicle の ControlTask を置き換えるワークショップ版 — sf::PidController の
/// 代わりに学習者の setup()/loop_400Hz() を呼ぶ。workshop_control_task.cpp で定義。
void WorkshopControlTask(void* pvParameters);

namespace sf {
namespace workshop_tasks {

namespace {
constexpr const char* TAG = "workshop_tasks";
}  // namespace

void start_all()
{
    // Core pipeline. IMU + WorkshopControlTask live on core 1 (the strict
    // 400Hz loop pair) — identical placement/priority reasoning as vehicle
    // (see tasks.cpp): StateTask on core 0 avoids starving on the INIT-stuck
    // failure mode observed on hardware.
    // コアパイプライン。IMU + WorkshopControlTask はコア1（厳格な400Hzループの
    // ペア）— vehicle と同一の配置・優先度根拠（tasks.cpp 参照）: StateTask を
    // コア0に置くことで実機で観測された INIT 停止の飢餓モードを避ける。
    xTaskCreatePinnedToCore(StateTask, "StateTask",
        config::STACK_STATE, nullptr, config::PRIORITY_STATE, nullptr, 0);
    xTaskCreatePinnedToCore(WorkshopControlTask, "ControlTask",
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
    xTaskCreatePinnedToCore(TelloStateTask, "TelloStateTask",
        config::STACK_TELLO_STATE, nullptr, config::PRIORITY_TELLO_STATE, nullptr, 0);

    ESP_LOGI(TAG, "All 16 tasks started (workshop: ControlTask -> WorkshopControlTask)");
}

}  // namespace workshop_tasks
}  // namespace sf
