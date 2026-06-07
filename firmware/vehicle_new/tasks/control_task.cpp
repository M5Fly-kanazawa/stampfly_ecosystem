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
#include "tasks.hpp"
#include "controller.hpp"
#include "pid_controller.hpp"
#include "actuator.hpp"
#include "flight_state.hpp"
#include "config.hpp"

static const char* TAG = "ControlTask";

/// ControlTask handle, registered by the task itself at startup and exposed via
/// sf::tasks::control_handle(). ImuTask reads it through that accessor to wake us
/// after each estimate. Replaces the old extern global set from main.cpp
/// (R3: no extern task handles).
/// ControlTask のハンドル。タスク自身が起動時に登録し sf::tasks::control_handle()
/// で公開する。ImuTask が推定後に起こすために accessor 経由で読む。main.cpp から
/// 設定していた旧 extern グローバルを置き換える（R3: extern 排除）。
static TaskHandle_t s_control_handle = nullptr;

namespace sf {
namespace tasks {
TaskHandle_t control_handle() { return s_control_handle; }
}  // namespace tasks
}  // namespace sf

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

/// Consume controller commands published by the StateManager transition callbacks
/// (architecture §4): full reset (ARM), altitude/position reset (mode exit / soft
/// landing), and flight-mode change (FLYING sub-mode switch). This is the controller-side
/// endpoint of the reset consolidation — the state machine decides WHEN, ControlTask
/// (which owns the controller) decides HOW. Replaces the ad-hoc prev_armed edge and the
/// per-cycle onModeChange poll that used to live in the loop.
/// StateManager の遷移コールバックが発行した制御器コマンドを消費する（architecture §4）:
/// full reset（ARM）、高度/位置リセット（モード退出/soft landing）、フライトモード変更（FLYING
/// サブモード切替）。reset 集約の制御器側エンドポイント — いつは状態機械、どうは制御器を所有する
/// ControlTask が決める。ループ内にあった旧来の prev_armed エッジと毎周期 onModeChange を置き換える。
///
/// @design architecture.md §4 — reset consolidation (controller side)    [OK]
/// @design detailed_design.md §3 — FLYING sub-mode switch → onModeChange  [OK]
static void processControllerCommands(sf::IController& controller)
{
    sf::ControllerCommand cmd;
    while (sf::controller_command.read(cmd)) {
        switch (static_cast<sf::ControllerCmd>(cmd.command)) {
        case sf::ControllerCmd::Reset:
            controller.reset();
            break;
        case sf::ControllerCmd::ModeChange:
            controller.onModeChange(static_cast<sf::FlightMode>(cmd.mode));
            break;
        case sf::ControllerCmd::ResetAltPos:
            // Phase 2: altitude/position-only reset (soft landing / FLYING exit). For now
            // a no-op — a subsequent ARM issues a full Reset. Wired with soft-landing.
            // Phase 2: 高度/位置のみリセット（soft landing/FLYING退出）。現状 no-op
            // （後続 ARM が full Reset を出す）。soft-landing と共に配線。
            break;
        default:
            break;
        }
    }
}

void ControlTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ControlTask started");

    // Register our own handle so ImuTask can wake us (sf::tasks::control_handle()).
    // Done before any blocking call; ImuTask guards on null until this runs.
    // ImuTask が起こせるよう自分のハンドルを登録（sf::tasks::control_handle()）。
    // ブロッキング前に実行。これが走るまで ImuTask は null ガードで待つ。
    s_control_handle = xTaskGetCurrentTaskHandle();

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

        // Consume controller reset/mode commands from the StateManager transition
        // callbacks (architecture §4). Done first so a reset / mode change applies
        // before this cycle's compute.
        // StateManager の遷移コールバックからの制御器 reset/mode 指令を消費（architecture §4）。
        // 最初に行い、reset/モード変更がこの周期の compute 前に効くようにする。
        processControllerCommands(controller);

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
            continue;
        }

        // Arm the actuator so the motor HAL accepts duty writes. The PID integrator reset
        // on the ARM transition is now done by the onEnter(ARMED_GROUND) callback via
        // controller_command(Reset), consumed in processControllerCommands() above
        // (architecture §4) — no more prev_armed edge detection here.
        // アクチュエータを arm し、モータ HAL が duty 書き込みを受理するようにする。ARM 遷移での
        // PID 積分器リセットは onEnter(ARMED_GROUND) コールバックが controller_command(Reset)
        // 経由で行い、上の processControllerCommands() が消費する（architecture §4）— ここでの
        // prev_armed エッジ検出は廃止。
        actuator.arm();

        // =====================================================================
        // Step 3: Compute control output. The flight mode is applied by the onModeChange
        // callback (StateManager → controller_command → processControllerCommands), so
        // the controller is already configured for the current sub_mode here.
        // Step 3: 制御出力を計算する。フライトモードは onModeChange コールバック
        // （StateManager → controller_command → processControllerCommands）が反映済みで、
        // ここでは制御器が現在の sub_mode 用に構成済み。
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
