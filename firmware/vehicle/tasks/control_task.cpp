/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
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
 * @subscriber estimate_state, command_setpoint, controller_command, system_mode
 * @publisher control_output, controller_status, actuator_motor, log_stream, nonfinite_status
 * @design architecture.md §5 — Main pipeline: Control + Actuation     [OK]
 * @design architecture.md §6 — ControlTask: Control + Actuation       [OK]
 * @design detailed_design.md §8 — ControlTask: 400Hz IMU-sync, pri 23 [OK]
 * @design coding_and_education.md §2 — 1 function 1 responsibility    [OK]
 * @design architecture.md §3 — R13 @publisher/@subscriber annotation  [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"   // motor_test expiry check

#include "topics.hpp"
#include "tasks.hpp"
#include "controller.hpp"
#include "app_hooks.hpp"
#include "actuator.hpp"
#include "flight_state.hpp"
#include "failsafe_status_guard.hpp"   // a stale failsafe_status reads as ToF silent
#include "config.hpp"
#include "loop_timing_scope.hpp"      // period / busy-time monitor of this 400Hz loop
#include "nonfinite_scope.hpp"        // NaN / infinity check of the control path boundaries

static const char* TAG = "ControlTask";

/// ControlTask handle, registered by the task itself at startup and exposed via
/// sf::tasks::control_handle(). ImuTask reads it through that accessor to wake us
/// after each estimate. Exposed through an accessor, not an extern global
/// (R3: no extern task handles).
/// ControlTask のハンドル。タスク自身が起動時に登録し sf::tasks::control_handle()
/// で公開する。ImuTask が推定後に起こすために accessor 経由で読む。main.cpp から
/// extern グローバルでなくアクセサで公開する（R3: extern 排除）。
static TaskHandle_t s_control_handle = nullptr;

namespace sf {
namespace tasks {
TaskHandle_t control_handle() { return s_control_handle; }
}  // namespace tasks
}  // namespace sf

/// Build and publish the 400Hz Data Stream record. ControlTask is the one place
/// where the SAME cycle's IMU sample, state estimate, control setpoints and motor
/// duties are all in hand (ImuTask published them just before notifying us).
/// Published every cycle, armed or not — ground data matters for analysis too.
/// 400Hz Data Stream レコードを組んで発行する。ControlTask は「同一周期」の IMU
/// サンプル・状態推定・制御目標・モータ duty が全部手元に揃う唯一の場所（ImuTask が
/// 通知直前に発行済み）。armed/disarmed を問わず毎周期発行 — 地上データも解析に必要。
///
/// @design requirements.md §7 — Data Stream（解析用・全レート）          [OK]
/// @design architecture.md §5 — ログフロー: Data Stream                  [OK]
static void publishLogStream(const sf::ControlOutput& control, uint8_t state,
                             uint8_t sub_mode, const sf::ControllerStatus& status)
{
    const sf::ImuData       imu   = sf::sensor_imu.latest();
    const sf::StateEstimate est   = sf::estimate_state.latest();
    const sf::MotorOutput   motor = sf::actuator_motor.latest();

    sf::LogStreamSample sample = {};
    sample.timestamp = imu.timestamp;
    for (int i = 0; i < 3; ++i) {
        sample.gyro[i]       = imu.gyro[i];
        sample.accel[i]      = imu.accel[i];
        sample.gyro_bias[i]  = est.gyro_bias[i];
        sample.accel_bias[i] = est.accel_bias[i];
        sample.pos[i]        = est.position[i];
        sample.vel[i]        = est.velocity[i];
        sample.rate_ref[i]   = control.rate_ref[i];
        sample.torque[i]     = control.torque[i];
    }
    for (int i = 0; i < 4; ++i) {
        sample.quat[i] = est.attitude[i];
        sample.duty[i] = motor.duty[i];
    }
    sample.angle_ref[0]  = control.angle_ref[0];
    sample.angle_ref[1]  = control.angle_ref[1];
    sample.thrust        = control.thrust;
    sample.flight_mode   = sub_mode;
    sample.flight_state  = state;
    // Flip sequencer progress: lets a failed flip be reconstructed phase by phase.
    // フリップのシーケンサ進行: 失敗したフリップをフェーズ単位で再構成できる。
    sample.flip_phase    = status.flip_phase;
    sample.flip_result   = status.flip_result;
    sample.flip_phi_rad  = status.flip_phi_rad;
    sf::log_stream.publish(sample);
}

/// Actuator: X-quad mixer + motor output. Reads control_output and publishes
/// per-motor duty on actuator_motor. The real mixer lives in sf_actuator;
/// control_task only wires it (see Step 4 below).
/// アクチュエータ: X-quad ミキサー＋モーター出力。control_output を読み、各モーター
/// duty を actuator_motor に発行する。実ミキサーは sf_actuator にあり、control_task は
/// 配線するだけ（下の Step 4 参照）。
static sf::Actuator actuator;

/// Check the outputs of the controller for NaN / infinity: the cascade references (outer-loop output)
/// and the thrust / torque (the rate PID output). Only counts; no value is changed.
/// 制御器の出力の NaN・無限大を検査する: カスケードの目標（外側ループの出力）と推力・トルク（レート PID の
/// 出力）。数えるだけで、値は変えない。
///
/// @design detailed_design.md §8 — Non-finite number monitor (nonfinite_status)  [OK]
static void checkControllerOutputs(sf::NonFiniteMonitor& monitor, const sf::ControlOutput& control)
{
    const uint32_t reference_bad = sf::countNonFinite(control.rate_ref, 3)
                                 + sf::countNonFinite(control.angle_ref, 2);
    monitor.record(sf::NonFiniteStage::ControlReference, reference_bad);

    const uint32_t output_bad = sf::countNonFinite(&control.thrust, 1)
                              + sf::countNonFinite(control.torque, 3);
    monitor.record(sf::NonFiniteStage::ControlOutput, output_bad);
}

/// Consume controller commands published by the StateManager transition callbacks
/// (architecture §4): full reset (ARM), altitude/position reset (mode exit / soft
/// landing), and flight-mode change (FLYING sub-mode switch). This is the controller-side
/// endpoint of the reset consolidation — the state machine decides WHEN, ControlTask
/// (which owns the controller) decides HOW. The loop itself has no prev_armed edge detection
/// and no per-cycle onModeChange poll.
/// StateManager の遷移コールバックが発行した制御器コマンドを消費する（architecture §4）:
/// full reset（ARM）、高度/位置リセット（モード退出/soft landing）、フライトモード変更（FLYING
/// サブモード切替）。reset 集約の制御器側エンドポイント — いつは状態機械、どうは制御器を所有する
/// ControlTask が決める。ループ内には prev_armed のエッジ検出も毎周期の onModeChange 確認もない。
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
        case sf::ControllerCmd::Takeoff:
            // Auto-takeoff (ALT/POS mode): fixed climb + level attitude until
            // TakeoffComplete. WHEN = state machine, HOW = controller.
            // 自動離陸（ALT/POS モード）: TakeoffComplete まで固定上昇＋水平姿勢。
            // いつ=状態機械、どう=制御器。
            controller.onTakeoff();
            break;
        case sf::ControllerCmd::TakeoffComplete:
            controller.onTakeoffComplete();
            break;
        case sf::ControllerCmd::Landing:
            // Autonomous landing (comm loss / battery emergency): the controller
            // switches to level-attitude + fixed-descent and ignores the sticks.
            // 自動着陸（通信断/電池緊急）: 制御器は水平姿勢＋固定降下に切り替わり
            // スティックを無視する。
            controller.onLanding();
            break;
        case sf::ControllerCmd::ReloadParams:
            // Live tuning: re-read gains/limits in THIS task's context (the
            // params callback only publishes the verb — no cross-task touching).
            // ライブチューニング: 本タスクの文脈でゲイン/リミットを読み直す
            // （params コールバックは verb を発行するだけ — タスク跨ぎで触らない）。
            controller.reloadParams();
            break;
        case sf::ControllerCmd::Flip:
            // FLYING -> FLIP entry (flip-maneuver-plan.md §4.1): the direction
            // rides ControllerCommand.arg, same convention as
            // EstimatorCommand.arg's CovScope (data_types.hpp).
            // FLYING→FLIP突入（plan §4.1）: 方向は ControllerCommand.arg で
            // 運ぶ（EstimatorCommand.arg の CovScope と同じ流儀、
            // data_types.hpp 参照）。
            controller.onFlip(static_cast<sf::FlipDirection>(cmd.arg));
            break;
        case sf::ControllerCmd::FlipComplete:
            // FLIP -> FLYING exit (plan §4.1 onExit).
            // FLIP→FLYING退出（plan §4.1 onExit）。
            controller.onFlipComplete();
            break;
        default:
            break;
        }
    }
}

/// Publish the controller's facts on controller_status (the controller is a core component with no topic
/// access, so the task layer carries them): guidance, auto-takeoff capture, flip gate and result, and the
/// blind-descent facts the ImuTask reads. Returns what it published (the log stream records it too).
/// 制御器の事実を controller_status に発行する（制御器はトピック禁制のコア部品なので、タスク層が運ぶ）:
/// 誘導、自動離陸の捕捉、フリップの判定と結果、ImuTask が読むToF なし降下の事実。
static sf::ControllerStatus publishControllerStatus(sf::IController& controller)
{
    // Publish the controller's guidance-engaged fact so the API source can release its target when the controller
    // self-cancels guidance (pilot stick movement / mode change) — "the pilot always wins" stays observable to ApiTask
    // (M-3). The controller is a core component with no topic access, so the task layer carries the fact to the topic.
    // 制御器の誘導係合の事実を発行し、制御器がスティック動作/モード変更で誘導を自発解除したとき API ソースが目標を
    // 解放できるようにする —「パイロット優先」を ApiTask から観測可能にする (M-3)。制御器はトピック禁制のコア部品
    // ゆえ、タスク層が事実をトピックへ運ぶ。
    // takeoff_reached carries the auto-takeoff capture event to StateTask, which
    // uses it to drive TAKEOFF→FLYING for ALT/POS (decoupled from the ToF 0.15m
    // airborne edge). Same task-layer carries-the-fact pattern as guidance_active.
    // takeoff_reached は自動離陸の捕捉イベントを StateTask へ運び、ALT/POS の
    // TAKEOFF→FLYING を駆動する（ToF 0.15m 空中エッジとは分離）。guidance_active と
    // 同じ「タスク層が事実を運ぶ」パターン。
    // flip_ready/flip_block_reason/flip_active/flip_done/flip_result (plan
    // §4.1/§4.3): read from the controller every cycle, same task-layer-
    // carries-the-fact pattern as guidance_active/takeoff_reached above.
    // flip_result is NOT cleared by the controller on FLIP->FLYING (see
    // FlipSequencer::reset()'s doc) — it stays readable by the Tello API,
    // which polls this topic AFTER state_task's transition, until the
    // controller's NEXT onFlip() overwrites it.
    // flip_ready/flip_block_reason/flip_active/flip_done/flip_result
    // （plan §4.1/§4.3）: 毎周期制御器から読む。guidance_active/
    // takeoff_reached と同じ「タスク層が事実を運ぶ」パターン。
    // flip_result は制御器が FLIP→FLYING で消さない
    // （FlipSequencer::reset() のドキュメント参照）— state_task の遷移
    // 「後」に本トピックをポーリングする Tello API から、制御器の
    // 「次」の onFlip() が上書きするまで読み出せる。
    sf::ControllerStatus status{};
    status.guidance_active = controller.isGuidanceActive();
    status.takeoff_reached = controller.isTakeoffComplete();
    sf::FlipBlockReason flip_reason = sf::FlipBlockReason::None;
    status.flip_ready = controller.isFlipReady(flip_reason);
    status.flip_block_reason = static_cast<uint8_t>(flip_reason);
    status.flip_active = controller.isFlipActive();
    sf::FlipResult flip_result = sf::FlipResult::None;
    status.flip_done = controller.isFlipDone(flip_result);
    status.flip_result = static_cast<uint8_t>(flip_result);
    status.flip_phase = static_cast<uint8_t>(controller.flipPhase());
    status.flip_phi_rad = controller.flipRotationRad();
    status.vertical_velocity_up = controller.verticalVelocityUp();   // blind-touchdown detector (ImuTask) / ToF なし接地検出器（ImuTask）
    status.blind_thrust_ratio   = controller.blindThrustRatio();
    status.vertical_rise_m      = controller.verticalRiseM();   // ToF health tracker (ImuTask) / ToF 健全性トラッカー（ImuTask）
    status.timestamp = static_cast<uint32_t>(esp_timer_get_time());
    sf::controller_status.publish(status);
    return status;   // the log stream records the same facts / ログのストリームも同じ事実を記録する
}

/// Inject the ToF-silence level and the airborne fact into the controller's LOCAL copy of the state (the
/// controller is a core component: it may not read topics).
/// ToF 出力停止のレベルと airborne の事実を、制御器に渡す「ローカルコピー」の state へ注入する（制御器は
/// コア部品でトピックを読めない）。
static void injectToFFacts(sf::StateEstimate& state, sf::FailsafeStatusGuard& failsafe_guard)
{
    // The downward-ToF silence LEVEL (Failsafe's judgement, 10 Hz): the autonomous landing
    // descends blind (velocity loop on an accelerometer-based estimate) while it is set. A silent ToF is also never a
    // "usable observation": the snapshot keeps the LAST sample's validity, which would let the
    // FLIP gate trust an altitude nobody can see.
    // 下向き ToF の出力停止「レベル」（Failsafe の判定、10 Hz）: これが立っている間、自動着陸は
    // ToF なしで降りる（加速度計に基づく速度推定の速度ループ）。出力停止中の ToF は「使用可能な観測」でもない:
    // スナップショットは「最後のサンプル」の有効性を保持するので、そのままだと FLIP の判定が
    // 誰も見られない高度を信用する。
    const sf::FailsafeStatus tof_silence = failsafe_guard.guard(
        static_cast<uint32_t>(esp_timer_get_time()), sf::failsafe_status.latest());
    state.tof_silent      = tof_silence.tof_silent;
    state.tof_valid       = sf::sensor_snapshot.latest().tof_valid && !state.tof_silent;
    // The estimator's vertical state is free (not pinned by the on-ground hold): ImuTask's
    // published ground-contact verdict, which already accounts for a silent ToF.
    // 推定器の鉛直状態が自由（接地ホールドで固定されていない）: ImuTask が発行する接地の判定で、
    // ToF 出力停止も織り込み済み。
    state.airborne        = sf::system_status.latest().airborne;
}

void ControlTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ControlTask started");

    // Register our own handle so ImuTask can wake us (sf::tasks::control_handle()).
    // Done before any blocking call; ImuTask guards on null until this runs.
    // ImuTask が起こせるよう自分自身のハンドルを登録（sf::tasks::control_handle()）。
    // ブロッキング前に実行。これが実行されるまで ImuTask は null ガードで待つ。
    s_control_handle = xTaskGetCurrentTaskHandle();

    // Fetch the active controller from the app hook (L1 entry point,
    // app_hooks.hpp). Already initialized by the hook — vehicle's default
    // (app_default.cpp) returns the stock cascaded-PID controller; an
    // application (SF_APP_DIR) may return its own IController implementation
    // instead.
    // アプリフック（L1 の入口、app_hooks.hpp）からアクティブなコントローラを
    // 取得する。フック側で初期化済み — vehicle の既定（app_default.cpp）は
    // 標準のカスケードPIDコントローラを返し、アプリ（SF_APP_DIR）があれば
    // 自作の IController 実装を返せる。
    sf::IController& controller = sf::app::controller();

    // Initialize the actuator (mixer + motor HAL). The controller is
    // initialized by the app hook above, not here.
    // アクチュエータ（ミキサー＋モーター HAL）を初期化。コントローラは上の
    // アプリフックで初期化済みのため、ここでは行わない。
    actuator.init();

    // Rate limiter for the IMU-stall warning (once per second at 400Hz)
    // IMU 停止警告のレート制限（400Hz で毎秒1回）
    uint32_t stall_count = 0;

    // True after the FIRST IMU notification ever arrives. During boot the
    // BMI270 init takes ~200ms, so the watchdog below fires before the 400Hz
    // pipeline exists — expected, not an error. The forced motor zero still
    // runs (harmless while disarmed); only the ERROR log is gated, so a real
    // mid-flight IMU stall is still reported loudly.
    // 「最初の」IMU 通知が来たら true。起動時は BMI270 init に約200msかかり、
    // 400Hz パイプラインが存在する前に下のウォッチドッグが作動する — これは想定で
    // エラーではない。モータ強制ゼロは実行したまま（disarmed 中は無害）、ERROR
    // ログだけを判定する。飛行中の本物の IMU 停止は大きく報告される。
    bool imu_pipeline_seen = false;

    sf::FailsafeStatusGuard failsafe_guard;   // this task's guard: a stale status reads as ToF silent / このタスクのガード: 古い状態は ToF 出力停止と読む

    // Period monitor of this loop (control_loop_timing, read by the CLI `status`): the wakes that
    // arrive more than 1.5 periods after the previous one, and the time one cycle takes.
    // このループの周期監視（control_loop_timing、CLI の `status` が読む）: 前回から 1.5 周期を超えて
    // 来た起床と、1 周期にかかる時間。
    sf::tasks::LoopTimingScope loop_timing(sf::control_loop_timing);

    // NaN / infinity check of the control path (nonfinite_status, read by the CLI `status`): counts
    // only, in armed cycles; the motor output guard in Actuator::update() is the one that acts.
    // 制御経路の NaN・無限大の検査（nonfinite_status、CLI の `status` が読む）: ARM 中の周期だけ数える。
    // 動作するのは Actuator::update() のモータ出力の保護だけ。
    sf::tasks::NonFiniteScope nonfinite(sf::nonfinite_status);

    while (true) {
        loop_timing.endCycle();   // close the previous cycle before waiting / 待つ前に前の周期を閉じる
        // =====================================================================
        // Wait for IMU task notification (400Hz sync) — WITH TIMEOUT.
        // ControlTask is the only entity that can zero the motors, but it only
        // runs when ImuTask notifies it. If the IMU dies mid-flight (SPI fault,
        // connector), ImuTask skips the notify and ControlTask would block
        // forever while LEDC holds the last duty → flyaway. The timeout is the
        // safety net: no notification within CONTROL_NOTIFY_TIMEOUT_MS → force
        // the motors to zero and keep waiting. Recovery is automatic — the next
        // successful IMU cycle re-arms the actuator below (idempotent arm()).
        // IMUタスクからの通知を待つ（400Hz同期）— タイムアウト付き。
        // モータをゼロにできる唯一の主体は ControlTask だが、起床は ImuTask の通知
        // 頼み。飛行中に IMU が停止する（SPI 故障・コネクタ）と ImuTask は通知をスキップし、
        // ControlTask は永久ブロック、LEDC は最後の duty を保持 → フライアウェイ。
        // タイムアウトが安全網: CONTROL_NOTIFY_TIMEOUT_MS 内に通知が無ければモータを
        // 強制ゼロにして待ち続ける。復帰は自動 — 次の正常 IMU 周期で下の arm()（冪等）
        // が再アームする。
        //
        // @design architecture.md §5 — IMU-synced control pipeline    [OK]
        // =====================================================================

        if (ulTaskNotifyTake(pdTRUE,
                             pdMS_TO_TICKS(config::CONTROL_NOTIFY_TIMEOUT_MS)) == 0) {
            actuator.disarm();   // unconditional motor zero / 無条件モータゼロ
            if (imu_pipeline_seen && (stall_count++ % 100) == 0) {
                ESP_LOGE(TAG, "No IMU notification for %ums — motors forced to zero",
                         static_cast<unsigned>(config::CONTROL_NOTIFY_TIMEOUT_MS));
            }
            continue;
        }
        imu_pipeline_seen = true;
        stall_count = 0;
        sf::SystemMode mode = sf::system_mode.latest();   // read once; the loop timing and this cycle share it / 1 回だけ読む（周期監視とこの周期で共有）
        const uint32_t wake_us = static_cast<uint32_t>(esp_timer_get_time());
        loop_timing.beginCycle(wake_us, mode.armed);
        nonfinite.beginCycle(wake_us, sf::isArmed(static_cast<sf::FlightState>(mode.state)));

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
        const sf::GuidanceTarget guidance = sf::command_target.latest();   // read once: checked below, handed over in Step 3 / 1 回だけ読む（下で検査し、Step 3 で渡す）

        // Inject the two facts the estimator itself does not populate
        // (core components — the controller — may not touch Pub-Sub topics)
        // into this LOCAL copy of state, so IController::compute() can still
        // see them via its normal state parameter. Used by the flip
        // sequencer's C5/C6 gates (flip-maneuver-plan.md §3.1); see
        // data_types.hpp's doc on these two StateEstimate fields.
        // 推定器自身が埋めない2つの事実（コア部品＝制御器は Pub-Sub トピック
        // に触れない）を、この「ローカルコピー」の state へ注入する —
        // IController::compute() は通常の state 引数経由でこれらを見られる。
        // フリップ列生成器の C5/C6 判定に使う（plan §3.1）。この2フィールドの
        // ドキュメントは data_types.hpp 参照。
        state.battery_voltage = sf::sensor_power.latest().voltage;
        injectToFFacts(state, failsafe_guard);   // ToF-silence level, airborne fact / ToF 出力停止のレベル、airborne

        // Feed the link / guidance staleness watches EVERY cycle, armed or not: compute() runs
        // only while armed, and a watch that misses a 35.8 min DISARM gap could read a dead
        // link as alive when the next ARM arrives (sensor_freshness.hpp, problem 3).
        // リンク・誘導の鮮度監視器へ、ARM 中か否かに関わらず「毎周期」観測を与える。compute() は
        // ARM 中しか走らず、35.8 分の DISARM の空きを見逃した監視器は、次の ARM で途絶したリンクを
        // 生存と読みうる（sensor_freshness.hpp 問題3）。
        controller.observeLinks(state, setpoint);

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
        // （HAL は arm されるまで全書き込みを破棄するため、これが無いとミキサーが
        // 正しい duty を計算してもモーターへ届かない）。両呼び出しは冪等（エッジでのみ作用）。
        // =====================================================================

        if (!sf::isArmed(static_cast<sf::FlightState>(mode.state))) {
            // Bench motor test (DISARMED ONLY): spin the requested motor(s) at a fixed
            // duty until the command expires, otherwise keep the motors off. This branch
            // is NEVER reached while armed, so the test cannot interfere with flight. The
            // command is a CLI fact on motor_test (default inactive → normal disarm).
            // ベンチ用モータテスト（disarmed 限定）: 要求された motor を固定 duty で失効まで回し、
            // それ以外はモータ停止。armed 中は本分岐に来ないので飛行に干渉しない。コマンドは
            // motor_test トピックの CLI 事実（既定 inactive → 通常の disarm）。
            const sf::MotorTest mt = sf::motor_test.latest();
            const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());
            if (mt.active && static_cast<int32_t>(mt.expiry_us - now_us) > 0) {
                float duties[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                if (mt.motor_id == 0xFF) {
                    for (int i = 0; i < 4; ++i) duties[i] = mt.duty;
                } else if (mt.motor_id < 4) {
                    duties[mt.motor_id] = mt.duty;
                }
                actuator.applyTestDuties(duties);
            } else {
                // One-shot consume on expiry: clear the latched fact so the uint32
                // microsecond clock wrap (~71.6 min) cannot make the stale expiry
                // compare "in the future" again and silently respin the motors.
                // 失効時のワンショット消化: ラッチされた事実をクリアし、uint32 マイクロ秒
                // 時計の桁あふれ（約71.6分）で古い失効時刻が再び「未来」と判定されて
                // モータが勝手に回り出すのを防ぐ。
                if (mt.active) {
                    sf::MotorTest off = {};
                    off.active = false;
                    off.timestamp = now_us;
                    sf::motor_test.publish(off);
                }
                actuator.disarm();
            }
            // Data Stream keeps recording while disarmed (zero control record):
            // ground data — at-rest sensor noise, handling — matters for analysis.
            // disarm 中も Data Stream は記録を続ける（制御ゼロのレコード）:
            // 静止時センサノイズやハンドリング等の地上データも解析に必要。
            publishLogStream({}, mode.state, mode.sub_mode, sf::ControllerStatus{});
            continue;
        }

        // Arm the actuator so the motor HAL accepts duty writes. The PID integrator reset
        // on the ARM transition is done by the onEnter(ARMED_GROUND) callback via
        // controller_command(Reset), consumed in processControllerCommands() above
        // (architecture §4) — there is no prev_armed edge detection here.
        // アクチュエータを arm し、モータ HAL が duty 書き込みを受理するようにする。ARM 遷移での
        // PID 積分器リセットは onEnter(ARMED_GROUND) コールバックが controller_command(Reset)
        // 経由で行い、上の processControllerCommands() が消費する（architecture §4）— ここでの
        // prev_armed エッジ検出はここにはない。
        actuator.arm();
        sf::checkControllerInputs(nonfinite.monitor(), state, setpoint, guidance);

        // =====================================================================
        // Step 3: Compute control output. The flight mode is applied by the onModeChange
        // callback (StateManager → controller_command → processControllerCommands), so
        // the controller is already configured for the current sub_mode here.
        // Step 3: 制御出力を計算する。フライトモードは onModeChange コールバック
        // （StateManager → controller_command → processControllerCommands）が反映済みで、
        // ここでは制御器が現在の sub_mode 用に構成済み。
        // =====================================================================

        // Guidance target (Tello-style API / Navigator, R11): hand a NEW
        // command_target to the controller exactly once (edge on timestamp).
        // The controller validates the mode and owns the cancel-on-stick rule.
        // 誘導目標（Tello 風 API / Navigator, R11）: 新しい command_target を
        // タイムスタンプのエッジで一度だけ制御器へ渡す。モード検証とスティック
        // 解除則は制御器が所有する。
        {
            static uint32_t last_guidance_ts = 0;
            if (guidance.timestamp != 0 && guidance.timestamp != last_guidance_ts) {
                last_guidance_ts = guidance.timestamp;
                controller.setGuidanceTarget(guidance, setpoint);
            }
        }

        // Sysid excitation verbs (API `sysid` → rate-loop identification).
        // 同定励振 verb（API `sysid` → レートループ同定）。
        {
            sf::SysidCommand sysid;
            while (sf::sysid_command.read(sysid)) {
                controller.startExcitation(sysid);
            }
        }

        sf::ControlOutput control = controller.compute(state, setpoint, config::IMU_DT);

        // Publish a completed stepped-sine point (autotune): the controller is a
        // core component (no topic access — smoke builds lack FreeRTOS), so the
        // task layer carries its result to the topic.
        // 完了ステップドサイン点の発行（自動チューン）: 制御器はコア部品（トピック
        // 禁制 — smoke ビルドは FreeRTOS なし）のため、タスク層が結果を運ぶ。
        {
            sf::SysidFreqResult sysid_res;
            if (controller.fetchSysidResult(sysid_res)) {
                sf::sysid_result.publish(sysid_res);
            }
        }

        checkControllerOutputs(nonfinite.monitor(), control);

        // Publish control output for logging
        // ログ用に制御出力を発行
        sf::control_output.publish(control);

        const sf::ControllerStatus status = publishControllerStatus(controller);   // controller facts to the topic / 制御器の事実をトピックへ

        // =====================================================================
        // Step 4: Run the X-quad mixer. It reads the control_output published
        // just above, publishes per-motor duty on actuator_motor (telemetry +
        // SILS plant), and drives the motor HAL.
        // Step 4: X-quad ミキサーを実行。直上で発行した control_output を読み、各
        // モーター duty を actuator_motor に発行（テレメトリ＋SILS プラント）し、
        // モーター HAL を駆動する。
        // =====================================================================

        actuator.update(nonfinite.monitor());
        nonfinite.endCycle();

        // =====================================================================
        // Step 5: Publish the 400Hz Data Stream record (after actuator.update()
        // so actuator_motor.latest() carries THIS cycle's duties).
        // Step 5: 400Hz Data Stream レコードを発行（actuator.update() の後に行い、
        // actuator_motor.latest() が「この周期」の duty を持つようにする）。
        // =====================================================================

        publishLogStream(control, mode.state, mode.sub_mode, status);
    }
}
