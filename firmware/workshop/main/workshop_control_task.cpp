/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (workshop firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file workshop_control_task.cpp
 * @brief WorkshopControlTask — learner code entry point (400Hz, IMU-synced)
 *        WorkshopControlTask — 学習者コードの入口（400Hz、IMU同期）
 *
 * Replaces vehicle's ControlTask (sf::PidController) with a thin shell that
 * calls the learner's setup()/loop_400Hz() (workshop_api.hpp / user_code.cpp).
 * Everything ELSE about the control cycle — IMU-notify wait + watchdog, the
 * ARM safety gate, and the 400Hz Data Stream — follows vehicle/tasks/control_task.cpp
 * as closely as possible, because that pipeline is what the rest of the reused
 * vehicle tasks (ImuTask, TelemetryTask, ...) expect to talk to.
 *
 * vehicle の ControlTask（sf::PidController）を、学習者の setup()/loop_400Hz()
 * （workshop_api.hpp / user_code.cpp）を呼ぶ薄いシェルに置き換える。制御周期の
 * 「それ以外」— IMU 通知待ち＋ウォッチドッグ、ARM 安全ゲート、400Hz Data Stream —
 * は vehicle/tasks/control_task.cpp をできる限り忠実に踏襲する。再利用している
 * 他の vehicle タスク（ImuTask、TelemetryTask、…）はこのパイプライン形状を前提に
 * 動いているため。
 *
 * @subscriber controller_command, estimate_state, sensor_imu, actuator_motor
 * @publisher  log_stream, actuator_motor (via sf::Actuator::applyTestDuties/disarm)
 * @design W1_SPEC.md §2-5 — workshop_control_task.cpp                  [OK]
 * @design architecture.md §5 — Main pipeline: Control + Actuation      [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "topics.hpp"
#include "tasks.hpp"      // sf::tasks::control_handle() declaration (vehicle/tasks/tasks.hpp)
#include "sf_api.hpp"      // sf::api::is_armed()
#include "actuator.hpp"
#include "config.hpp"

#include "workshop_api.hpp"  // extern setup()/loop_400Hz() — learner code entry points
#include "ws_internal.hpp"

static const char* TAG = "WorkshopControlTask";

/// WorkshopControlTask handle, registered by the task itself at startup and
/// exposed via sf::tasks::control_handle(). ImuTask (reused from vehicle
/// unmodified) reads it through that SAME accessor to wake us after each
/// estimate — this is why the accessor must live under namespace sf::tasks,
/// not sf::workshop_tasks (control_task.cpp:53-59 has the vehicle original).
/// WorkshopControlTask のハンドル。タスク自身が起動時に登録し
/// sf::tasks::control_handle() で公開する。ImuTask（vehicle から無改変で再利用）は
/// 推定後に起こすため「同じ」accessor を読む — これが sf::workshop_tasks でなく
/// sf::tasks 名前空間に置く理由（vehicle 原本は control_task.cpp:53-59）。
static TaskHandle_t s_control_handle = nullptr;

namespace sf {
namespace tasks {
TaskHandle_t control_handle() { return s_control_handle; }
}  // namespace tasks
}  // namespace sf

/// Mixer + motor HAL. Same object vehicle's ControlTask owns — WorkshopControlTask
/// resolves the learner's motor request (ws_internal::resolve_motor_request) instead
/// of running a PID controller, but the actuator plumbing is identical.
/// ミキサー＋モーター HAL。vehicle の ControlTask が持つのと同じオブジェクト —
/// WorkshopControlTask は PID コントローラの代わりに学習者のモータ要求を解決する
/// （ws_internal::resolve_motor_request）が、アクチュエータの配線は同一。
static sf::Actuator actuator;

/// Autonomous-landing latch (comm loss / battery emergency): once set by a
/// ControllerCmd::Landing fact, the motor-apply step below is forced off
/// regardless of sf::api::is_armed(), until the next ControllerCmd::Reset
/// (issued on the ARM transition). Workshop has no autonomous landing law of
/// its own — the safe response is simply "stop", not vehicle's level-attitude
/// descent (which needs a controller the learner does not have).
/// 自動着陸ラッチ（通信断/電池緊急）: ControllerCmd::Landing の事実で立つと、
/// 次の ControllerCmd::Reset（ARM 遷移で発行）まで sf::api::is_armed() に関係なく
/// 下のモータ適用を強制停止する。workshop には自前の自動着陸則が無い — 安全な
/// 応答は vehicle の水平姿勢降下（学習者が持たない制御器が必要）ではなく単純な
/// 「停止」で足りる。
static bool s_failsafe_latch = false;

/// Consume controller_command FACTs (StateManager transition callbacks,
/// architecture.md §4). WorkshopControlTask has no controller object to reset/
/// reconfigure — it only cares about the two verbs that affect the motor
/// safety gate below.
/// controller_command の事実を消費する（StateManager 遷移コールバック,
/// architecture.md §4）。WorkshopControlTask にはリセット/再構成すべき制御器
/// オブジェクトが無い — 下のモータ安全ゲートに関わる 2 つの verb だけを見る。
static void processControllerCommands()
{
    sf::ControllerCommand cmd;
    while (sf::controller_command.read(cmd)) {
        switch (static_cast<sf::ControllerCmd>(cmd.command)) {
        case sf::ControllerCmd::Landing:
            s_failsafe_latch = true;
            break;
        case sf::ControllerCmd::Reset:
            s_failsafe_latch = false;
            break;
        default:
            break;  // other verbs (ModeChange, Takeoff, ...) don't apply here
        }
    }
}

/// Build and publish the 400Hz Data Stream record, same shape as vehicle's
/// publishLogStream (control_task.cpp) so `sf log wifi` / Data Stream tooling
/// works unmodified on workshop. rate_ref/angle_ref come from the learner's
/// ws::set_rate_target()/set_angle_target() calls (zero if the lesson never
/// calls them — see ws_internal.hpp ControlTargets) and thrust is the mixer's
/// T input when a mixer request is active, 0 otherwise (Direct mode has no
/// thrust concept).
/// 400Hz Data Stream レコードを組んで発行する。vehicle の publishLogStream
/// （control_task.cpp）と同じ形にし、`sf log wifi` / Data Stream ツールが workshop
/// でも無改変で動くようにする。rate_ref/angle_ref は学習者の
/// ws::set_rate_target()/set_angle_target() 呼び出しに由来する（レッスンが
/// 一度も呼ばなければゼロのまま — ws_internal.hpp ControlTargets 参照）。
/// thrust は mixer 要求が有効な時だけ T、それ以外は 0（Direct モードには
/// 推力の概念が無い）。
static void publishLogStream(uint8_t state, uint8_t sub_mode)
{
    const sf::ImuData       imu   = sf::sensor_imu.latest();
    const sf::StateEstimate est   = sf::estimate_state.latest();
    const sf::MotorOutput   motor = sf::actuator_motor.latest();
    const ws_internal::MotorRequest&   req     = ws_internal::motor_request();
    const ws_internal::ControlTargets& targets = ws_internal::control_targets();

    sf::LogStreamSample sample = {};
    sample.timestamp = imu.timestamp;
    for (int i = 0; i < 3; ++i) {
        sample.gyro[i]       = imu.gyro[i];
        sample.accel[i]      = imu.accel[i];
        sample.gyro_bias[i]  = est.gyro_bias[i];
        sample.accel_bias[i] = est.accel_bias[i];
        sample.pos[i]        = est.position[i];
        sample.vel[i]        = est.velocity[i];
        sample.rate_ref[i]   = targets.rate[i];
    }
    for (int i = 0; i < 4; ++i) {
        sample.quat[i] = est.attitude[i];
        sample.duty[i] = motor.duty[i];
    }
    sample.angle_ref[0]  = targets.angle[0];
    sample.angle_ref[1]  = targets.angle[1];
    sample.thrust        = (req.mode == ws_internal::MotorMode::Mixer) ? req.thrust : 0.0f;
    sample.flight_mode    = sub_mode;
    sample.flight_state   = state;
    sf::log_stream.publish(sample);
}

void WorkshopControlTask(void* pvParameters)
{
    ESP_LOGI(TAG, "WorkshopControlTask started");

    // Register our own handle so ImuTask can wake us (sf::tasks::control_handle()).
    // Done before any blocking call — ImuTask guards on null until this runs.
    // ImuTask が起こせるよう自分のハンドルを登録（sf::tasks::control_handle()）。
    // ブロッキング前に実行。これが走るまで ImuTask は null ガードで待つ。
    s_control_handle = xTaskGetCurrentTaskHandle();

    // Initialize the actuator (mixer + motor HAL) before the learner's setup()
    // runs, so ws:: motor calls made from setup() itself are safe (though the
    // motors stay disarmed until the ARM gate below opens).
    // 学習者の setup() が走る前にアクチュエータ（ミキサー＋モーター HAL）を初期化
    // する。setup() 自身から ws:: モータ関数を呼んでも安全（下の ARM ゲートが開く
    // まではモータは disarmed のまま）。
    actuator.init();

    // Call the learner's one-time setup() exactly once.
    // 学習者の setup() を一度だけ呼ぶ。
    setup();

    uint32_t stall_count = 0;
    bool imu_pipeline_seen = false;  // see vehicle control_task.cpp for rationale

    while (true) {
        // =====================================================================
        // Wait for IMU task notification (400Hz sync) — WITH TIMEOUT. Same
        // watchdog rationale as vehicle's ControlTask: a dead IMU must not
        // leave the LEDC outputs frozen at the last written duty.
        // IMUタスクからの通知を待つ（400Hz同期）— タイムアウト付き。vehicle の
        // ControlTask と同じウォッチドッグ根拠: IMU 死亡時に LEDC 出力を最後の
        // duty で固着させない。
        // =====================================================================
        if (ulTaskNotifyTake(pdTRUE,
                             pdMS_TO_TICKS(config::CONTROL_NOTIFY_TIMEOUT_MS)) == 0) {
            actuator.disarm();
            if (imu_pipeline_seen && (stall_count++ % 100) == 0) {
                ESP_LOGE(TAG, "No IMU notification for %ums — motors forced to zero",
                         static_cast<unsigned>(config::CONTROL_NOTIFY_TIMEOUT_MS));
            }
            continue;
        }
        imu_pipeline_seen = true;
        stall_count = 0;

        // Consume Landing/Reset facts before this cycle's motor gate decision.
        // このサイクルのモータゲート判定より前に Landing/Reset の事実を消費する。
        processControllerCommands();

        const sf::SystemMode mode = sf::system_mode.latest();

        // =====================================================================
        // Learner loop — runs EVERY cycle, armed or not (sensors/LED lessons
        // work while disarmed too). loop_400Hz() may call ws::motor_* — those
        // only latch a request; the actual motor write happens below, gated.
        // 学習者ループ — armed/disarmed を問わず毎周期実行（センサ/LED レッスンは
        // disarmed でも動く）。loop_400Hz() は ws::motor_* を呼びうるが、それは
        // 要求のラッチのみ — 実際のモータ書き込みは下の安全ゲート後に行う。
        // =====================================================================
        loop_400Hz(config::IMU_DT);

        // =====================================================================
        // Motor apply — ARMED gate. applyTestDuties() arms the motor HAL, so it
        // must ONLY be reached on this branch (never while disarmed), or a
        // button-less spin-up becomes possible.
        // モータ適用 — ARM ゲート。applyTestDuties() はモータ HAL を arm するため、
        // 必ずこの分岐内でのみ呼ぶこと（disarmed 中は絶対に呼ばない）— さもないと
        // ボタン ARM なしでモータが回りうる。
        // =====================================================================
        const bool armed = sf::api::is_armed() && !s_failsafe_latch;
        if (armed) {
            float duties[4];
            ws_internal::resolve_motor_request(duties);
            actuator.applyTestDuties(duties);
        } else {
            actuator.disarm();
        }

        // Data Stream keeps recording while disarmed too (ground data matters).
        // disarmed 中も Data Stream は記録を続ける（地上データも解析に必要）。
        publishLogStream(mode.state, mode.sub_mode);
    }
}
