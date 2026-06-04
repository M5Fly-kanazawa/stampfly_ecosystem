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

    // Previous arm state, to detect the disarmed→armed edge (reset PID integrators).
    // 前回の arm 状態。disarmed→armed エッジ検出用（PID 積分器をリセット）。
    bool prev_armed = false;

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
        // Step 2: Gate the motor output on the system arm state.
        // Step 2: システムの ARM 状態でモーター出力を gate する。
        //
        // When disarmed, disarm the actuator (zero the motors) and skip control.
        // When armed, arm the actuator so the motor HAL accepts duty writes — the
        // HAL silently swallows every write until armed, so without this the
        // mixer would compute correct duties that never reach the motors.
        // Both calls are idempotent (act only on the arm-state edge).
        // disarm 時はアクチュエータを disarm（モーターを 0 に）して制御をスキップ。
        // arm 時はアクチュエータを arm し HAL が duty 書き込みを受理するようにする
        // （HAL は arm されるまで全書き込みを握り潰すため、これが無いとミキサーが
        // 正しい duty を計算してもモーターへ届かない）。両呼び出しは冪等（エッジでのみ作用）。
        // =====================================================================

        if (!sf::isArmed(static_cast<sf::FlightState>(mode.state))) {
            actuator.disarm();
            prev_armed = false;
            continue;
        }

        // On the disarmed→armed edge, clear the PID integrators so a fresh ARM
        // never inherits stale integral wind-up from a previous flight. (The
        // proper home for this is a StateManager onEnter(ARMED_GROUND) callback,
        // wired in a later milestone — development_roadmap Phase C.)
        // disarmed→armed エッジで PID 積分器をクリアし、新しい ARM が前回飛行の積分
        // ワインドアップを引き継がないようにする。（本来は StateManager の
        // onEnter(ARMED_GROUND) コールバックが担うべきで、後段 Phase C で配線する。）
        if (!prev_armed) {
            controller.reset();
            prev_armed = true;
        }
        actuator.arm();

        // =====================================================================
        // Step 3: Apply the commanded flight mode, then compute control output.
        // Step 3: 指令フライトモードを反映し、制御出力を計算する。
        //
        // Mode arbitration: the controller switches ACRO/STABILIZE/ALT_HOLD/
        // POS_HOLD based on system_mode.sub_mode. onModeChange resets the relevant
        // loops on a transition (no-op when the mode is unchanged).
        // モード調停: コントローラは system_mode.sub_mode で各モードを切替える。
        // onModeChange は遷移時に該当ループをリセットする（不変時は no-op）。
        // =====================================================================

        controller.onModeChange(static_cast<sf::FlightMode>(mode.sub_mode));

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
