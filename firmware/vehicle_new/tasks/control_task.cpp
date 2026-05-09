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
#include "flight_state.hpp"
#include "config.hpp"

static const char* TAG = "ControlTask";

/// Task handle (set in main.cpp, used by ImuTask for notification)
/// タスクハンドル（main.cppで設定、ImuTaskが通知に使用）
TaskHandle_t g_control_task_handle = nullptr;

/// Controller instance
/// コントローラインスタンス
static sf::PidController controller;

/// Apply mixer: convert thrust/torque to motor duty
/// ミキサー適用: 推力/トルクをモーターdutyに変換
///
/// @design requirements.md §6 — X-quad motor layout                   [--]
static sf::MotorOutput applyMixer(const sf::ControlOutput& control)
{
    sf::MotorOutput motors = {};
    motors.timestamp = control.timestamp;

    // TODO: Full mixer implementation
    // TODO: 完全なミキサー実装
    //
    // X-quad layout:
    // M1(FR) = T - Roll + Pitch + Yaw (CCW)
    // M2(RR) = T - Roll - Pitch - Yaw (CW)
    // M3(RL) = T + Roll - Pitch + Yaw (CCW)
    // M4(FL) = T + Roll + Pitch - Yaw (CW)

    return motors;
}

void ControlTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ControlTask started");

    // Initialize controller
    // コントローラを初期化
    controller.init();

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
        // Step 4: Apply mixer and publish motor duty
        // Step 4: ミキサー適用してモーターdutyを発行
        // =====================================================================

        sf::MotorOutput motors = applyMixer(control);
        sf::actuator_motor.publish(motors);

        // TODO: Write motor duty to hardware
        // TODO: モーターdutyをハードウェアに書き込む
    }
}
