/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file state_task.cpp
 * @brief State management task — event-driven (priority 22)
 *        状態管理タスク — イベント駆動（優先度22）
 *
 * Waits for events (alerts, commands) and delegates to StateManager.
 * This is the sole task that modifies flight state.
 *
 * イベント（アラート、コマンド）を待ち、StateManagerに委譲する。
 * フライト状態を変更する唯一のタスク。
 *
 * @design architecture.md §6 — StateTask: event-driven, priority 22   [--]
 * @design architecture.md §2 — State Management: sole transition owner [--]
 * @design detailed_design.md §8 — StateTask                          [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "topics.hpp"
#include "state_manager.hpp"
#include "config.hpp"

static const char* TAG = "StateTask";

/// Global state manager instance
/// グローバル状態管理インスタンス
sf::StateManager g_state_manager;

/// Register the StateManager transition callbacks (called once at task start, after
/// init()). Each callback publishes reset commands on the estimator/controller/notify
/// command topics per the detailed_design §3 transition table; the resource-owning
/// tasks (ImuTask=estimator, ControlTask=controller, NotifyTask=LED/buzzer) consume them.
/// This is the single place that encodes the transition table and makes the state
/// machine the sole owner of transition resets, replacing the ad-hoc edge detection that
/// used to live in ImuTask/ControlTask.
/// StateManager の遷移コールバックを登録する（タスク開始時に init() 後1回）。各コールバックは
/// detailed_design §3 遷移表に従い推定器/制御器/通知の指令トピックにリセット指令を publish し、
/// リソース所有タスク（ImuTask=推定器, ControlTask=制御器, NotifyTask=LED/ブザー）が消費する。
/// ここが遷移表を符号化する唯一の場所で、ImuTask/ControlTask に散らばっていたエッジ検出を
/// 置き換え、遷移リセットの唯一の所有者を状態機械にする。
///
/// @design architecture.md §4 — onExit/onEnter callbacks consolidate reset    [OK]
/// @design detailed_design.md §3 — state transition table                     [OK]
static void registerStateCallbacks(sf::StateManager& manager)
{
    using sf::FlightState;
    using sf::FlightMode;

    // onEnter — reset/init on entering the new state (detailed_design §3 onEnter column).
    // The (from, to) pair disambiguates transitions that share a destination.
    // onEnter — 新状態に入った時の reset/初期化（detailed_design §3 onEnter 列）。
    // (from, to) ペアで、行き先を共有する遷移を区別する。
    manager.onEnter([](FlightState from, FlightState to) {
        const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
        switch (to) {
        case FlightState::IDLE_GROUND:
            if (from == FlightState::INIT) {
                // INIT → IDLE_GROUND: start the boot bias calibration (callback-driven,
                // architecture §6 — moved out of ImuTask setup).
                // INIT→IDLE_GROUND: 起動バイアス校正を開始（コールバック駆動、ImuTask setup から移管）。
                sf::estimator_command.publish(
                    {static_cast<uint8_t>(sf::EstimatorCmd::Recalibrate), now});
            } else {
                // Return to ground (DISARM / land): disarm tone. The crash-return ESKF
                // reset + recalibration are wired in Phase 2 (re-fly readiness).
                // 接地復帰（DISARM/着陸）: disarm 音。墜落復帰の ESKF reset/再校正は Phase 2。
                sf::notify_command.publish(
                    {static_cast<uint8_t>(sf::NotifyEvent::DisarmTone), now});
            }
            break;
        case FlightState::ARMED_GROUND:
            if (from == FlightState::IDLE_GROUND) {
                // ARM: clear PID integrators (no stale wind-up) + arm tone.
                // ARM: PID 積分器をクリア（古いワインドアップ無し）+ arm 音。
                sf::controller_command.publish(
                    {static_cast<uint8_t>(sf::ControllerCmd::Reset), 0, now});
                sf::notify_command.publish(
                    {static_cast<uint8_t>(sf::NotifyEvent::ArmTone), now});
            }
            break;
        // TAKEOFF → FLYING: the ESKF position/velocity reset for a clean ToF lock is NOT
        // issued here. It is the ToF-synced vertical handoff in ImuTask (one-cycle
        // precise); a ~20 ms-late reset via this callback degrades POS_HOLD attitude.
        // See applyVerticalGroundHandoff(). This callback owns the controller + full-state
        // resets, not the timing-critical estimation handoff.
        // TAKEOFF→FLYING: クリーンな ToF ロックのための ESKF 位置/速度リセットはここでは出さない。
        // それは ImuTask の ToF 同期鉛直ハンドオフが担う（1サイクル精度）。このコールバック経由の
        // ~20ms 遅れ reset は POS_HOLD 姿勢を劣化させる。applyVerticalGroundHandoff() 参照。
        // 本コールバックは制御器＋full-state reset を所有し、タイミング命の estimation ハンドオフは
        // 担わない。
        default:
            break;
        }
    });

    // onModeChange — FLYING sub-mode switch: reconfigure the controller (resets the
    // relevant loops, captures hold targets). detailed_design §3 FLYING row.
    // onModeChange — FLYING サブモード切替: 制御器を再構成（該当ループ reset、保持目標捕捉）。
    manager.onModeChange([](FlightMode /*old_mode*/, FlightMode new_mode) {
        const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
        sf::controller_command.publish(
            {static_cast<uint8_t>(sf::ControllerCmd::ModeChange),
             static_cast<uint8_t>(new_mode), now});
    });
}

void StateTask(void* pvParameters)
{
    ESP_LOGI(TAG, "StateTask started");

    // Transition from INIT → IDLE_GROUND
    // INIT → IDLE_GROUNDへ遷移
    //
    // @design requirements.md §2 — INIT → IDLE_GROUND on init complete [--]
    g_state_manager.init();

    // Register the transition callbacks that consolidate all reset processing
    // (architecture §4). Must be after init() — init() zeroes the callback counts.
    // 全リセット処理を集約する遷移コールバックを登録する（architecture §4）。init() の後で
    // 行う — init() はコールバック数をゼロにするため。
    registerStateCallbacks(g_state_manager);

    // Previous ARM switch state, for rising/falling edge detection across iterations.
    // 前回の ARM スイッチ状態（立上り/立下りエッジ検出用、反復間で保持）。
    bool prev_arm = false;
    bool init_done = false;   // INIT → IDLE_GROUND done once / 初期化完了遷移を1回

    while (true) {
        // =====================================================================
        // Wake on a failsafe notification OR every 20 ms to poll the pilot's RC
        // stream (the controller sends at 50 Hz). Still event-driven for alerts;
        // the periodic tick lets us edge-detect the ARM/mode switches the pilot
        // holds without each packet having to notify this task across components.
        // フェイルセーフ通知、または 20ms 毎にウェイクしてパイロットの RC 列をポーリング
        // （コントローラは 50Hz 送信）。アラートはイベント駆動のまま、周期ティックで ARM/
        // モードのエッジ検出を行う（各パケットがコンポーネント跨ぎで通知しなくて済む）。
        //
        // @design detailed_design.md §8 — StateTask: event-driven + 50 Hz RC poll [--]
        // =====================================================================

        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));

        // =====================================================================
        // INIT → IDLE_GROUND once the estimator is producing valid output, i.e.
        // sensors are up and fused (our "initialization complete" signal). Until
        // then ARM is correctly rejected (not IDLE_GROUND).
        // 推定器が有効出力を出したら（センサ立上り＋融合済み＝「初期化完了」信号）
        // INIT → IDLE_GROUND。それまで ARM は正しく拒否される（IDLE_GROUND でない）。
        // =====================================================================
        if (!init_done && sf::sensor_imu.latest().timestamp != 0) {
            g_state_manager.notifyInitComplete();
            init_done = true;
        }

        // =====================================================================
        // Process pilot requests (ARM edge + flight-mode selection). sf_comm
        // reports the switches as facts; StateManager makes the decisions (it
        // gates ARM to IDLE_GROUND and validates mode transitions internally).
        // パイロット要求（ARM エッジ＋飛行モード選択）を処理。sf_comm はスイッチを事実
        // として報告し、判断は StateManager が行う（ARM を IDLE_GROUND に限定し、モード
        // 遷移を内部で検証）。
        //
        // @design architecture.md §2 — detection reports, state management decides [--]
        // =====================================================================

        sf::PilotRequest req = sf::pilot_request.latest();
        if (req.timestamp != 0) {                          // skip until a packet arrives
            if (req.arm && !prev_arm) {
                g_state_manager.requestArm();              // rising edge: request ARM
            } else if (!req.arm && prev_arm) {
                g_state_manager.requestDisarm();           // falling edge: request DISARM
            }
            prev_arm = req.arm;

            // Derive the requested FlightMode from the mode switches (priority:
            // POS_HOLD > ALT_HOLD > ACRO > STABILIZE default).
            // モードスイッチから要求 FlightMode を導出（優先: POS>ALT>ACRO>STABILIZE）。
            sf::FlightMode want = sf::FlightMode::STABILIZE;
            if (req.pos_hold)      want = sf::FlightMode::POS_HOLD;
            else if (req.alt_hold) want = sf::FlightMode::ALT_HOLD;
            else if (req.acro)     want = sf::FlightMode::ACRO;
            if (want != g_state_manager.getMode()) {
                g_state_manager.requestModeChange(want);
            }
        }

        // =====================================================================
        // Takeoff sequencing: ARMED_GROUND → TAKEOFF → FLYING.
        // 離陸シーケンス: ARMED_GROUND → TAKEOFF → FLYING。
        //
        // Requirements §2: ARMED_GROUND→TAKEOFF on "throttle input";
        // TAKEOFF→FLYING on "takeoff complete (altitude threshold reached)".
        // The altitude-threshold detector is the ToF-based TakeoffLandingMgr, owned by
        // ImuTask (it has the ToF and the vertical estimate); it publishes its airborne
        // state on system_status. With the vertical estimate now trustworthy (ALT/POS
        // hold, development_roadmap Phase B), this replaces the earlier estimator-
        // independent dwell. Reaching FLYING unlocks requestModeChange.
        // 要件§2: スロットル入力で離陸、高度閾値到達で離陸完了。高度検出は ImuTask 所有の
        // ToF ベース TakeoffLandingMgr（ToF と鉛直推定を持つ）で、airborne 状態を
        // system_status に発行する。鉛直推定が信頼できる今（ALT/POS 保持, ロードマップ
        // Phase B）、これが旧 estimator 非依存 dwell を置き換える。FLYING 到達で
        // requestModeChange が解放される。
        //
        // @design requirements.md §2 — ARMED_GROUND→TAKEOFF→FLYING        [OK]
        // @design development_roadmap.md §3 Layer 3 — ToF takeoff detect  [OK]
        // =====================================================================

        const sf::FlightState fs = g_state_manager.getState();
        const float throttle = sf::command_setpoint.latest().throttle;

        if (fs == sf::FlightState::ARMED_GROUND &&
            throttle > config::TAKEOFF_THROTTLE_THRESH) {
            g_state_manager.notifyTakeoff();              // → TAKEOFF
        } else if (fs == sf::FlightState::TAKEOFF &&
                   sf::system_status.latest().airborne) {
            g_state_manager.notifyTakeoffComplete();      // → FLYING (ToF: off the ground)
        }

        // =====================================================================
        // Process system alerts from failsafe
        // フェイルセーフからのシステムアラートを処理
        //
        // @design architecture.md §4 — FAILSAFE as event              [--]
        // =====================================================================

        sf::SystemAlert alert;
        while (sf::system_alert.read(alert)) {
            g_state_manager.handleAlert(alert);
        }
    }
}
