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

void StateTask(void* pvParameters)
{
    ESP_LOGI(TAG, "StateTask started");

    // Transition from INIT → IDLE_GROUND
    // INIT → IDLE_GROUNDへ遷移
    //
    // @design requirements.md §2 — INIT → IDLE_GROUND on init complete [--]
    g_state_manager.init();

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
