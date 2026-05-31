/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file control_task.cpp
 * @brief Control computation and actuation task (400Hz, IMU-synced)
 *        制御演算およびアクチュエーションタスク（400Hz、IMU同期）
 *
 * Waits for IMU task notification, reads the latest state estimate
 * and command setpoint, computes control output via the active
 * controller, and publishes motor duty.
 *
 * IMUタスクからの通知を待ち、最新の状態推定値と
 * コマンドセットポイントを読み取り、アクティブなコントローラで
 * 制御出力を計算し、モーターdutyを発行する。
 *
 * @design architecture.md §5 — Main pipeline: Control + Actuation     [--]
 * @design architecture.md §6 — ControlTask: Control + Actuation       [--]
 * @design detailed_design.md §8 — ControlTask: 400Hz IMU-sync, pri 23 [--]
 * @design coding_and_education.md §2 — 1 function 1 responsibility    [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "topics.hpp"
#include "controller.hpp"
#include "pid_controller.hpp"
#include "actuator.hpp"
#include "flight_state.hpp"
#include "config.hpp"

static const char* TAG = "ControlTask";

/// Task handle (set in main.cpp, used by ImuTask for notification)
/// タスクハンドル（main.cppで設定、ImuTaskが通知に使用）
TaskHandle_t g_control_task_handle = nullptr;

/// Controller instance
/// コントローラインスタンス
static sf::PidController controller;

/// Actuator: X-quad mixer + motor output. Reads control_output and publishes
/// per-motor duty on actuator_motor. The real mixer lives in sf_actuator;
/// control_task only wires it (see Step 4 below).
/// アクチュエータ: X-quad ミキサー＋モーター出力。control_output を読み、各モーター
/// duty を actuator_motor に発行する。実ミキサーは sf_actuator にあり、control_task は
/// 配線するだけ（下の Step 4 参照）。
static sf::Actuator actuator;

void ControlTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ControlTask started");

    // Initialize controller and actuator (mixer + motor HAL)
    // コントローラとアクチュエータ（ミキサー＋モーター HAL）を初期化
    controller.init();
    actuator.init();

    while (true) {
        // =====================================================================
        // Wait for IMU task notification (400Hz sync)
        // IMUタスクからの通知を待つ（400Hz同期）
        //
        // @design architecture.md §5 — IMU-synced control pipeline    [--]
        // =====================================================================

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // =====================================================================
        // Step 1: Read latest state and setpoint from topics
        // Step 1: トピックから最新の状態とセットポイントを読む
        // =====================================================================

        sf::StateEstimate state = sf::estimate_state.latest();
        sf::CommandSetpoint setpoint = sf::command_setpoint.latest();
        sf::SystemMode mode = sf::system_mode.latest();

        // =====================================================================
        // Step 2: Skip control if not armed
        // Step 2: ARM状態でなければ制御をスキップ
        // =====================================================================

        if (!sf::isArmed(static_cast<sf::FlightState>(mode.state))) {
            continue;
        }

        // =====================================================================
        // Step 3: Compute control output
        // Step 3: 制御出力を計算
        // =====================================================================

        sf::ControlOutput control = controller.compute(state, setpoint, config::IMU_DT);

        // Publish control output for logging
        // ログ用に制御出力を発行
        sf::control_output.publish(control);

        // =====================================================================
        // Step 4: Run the X-quad mixer. It reads the control_output published
        // just above, publishes per-motor duty on actuator_motor (telemetry +
        // SIL plant), and drives the motor HAL.
        // Step 4: X-quad ミキサーを実行。直上で発行した control_output を読み、各
        // モーター duty を actuator_motor に発行（テレメトリ＋SIL プラント）し、
        // モーター HAL を駆動する。
        // =====================================================================

        actuator.update();
    }
}
