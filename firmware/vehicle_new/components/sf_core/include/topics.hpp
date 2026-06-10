/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file topics.hpp
 * @brief All topic definitions (single location)
 *        全トピック定義（1箇所）
 *
 * Adding a new topic requires only one line here.
 * トピック追加はここに1行追加するだけ。
 *
 * @design architecture.md §3 — Topic list                            [OK]
 * @design detailed_design.md §2 — Topic definitions                   [OK]
 * @design detailed_design.md §2 — R13/R14 annotation & overflow_count [OK]
 */

#pragma once

#include "topic.hpp"
#include "data_types.hpp"

namespace sf {

/// Initialize all topics (call once at startup)
/// 全トピックを初期化する（起動時に1回呼ぶ）
void topics_init();

// =============================================================================
// Sensor Topics
// センサトピック
// =============================================================================

extern Topic<ImuData,         RingBuffer, 8>  sensor_imu;
extern Topic<TofData,         Queue, 2>       sensor_tof;
extern Topic<FlowData,        Queue, 2>       sensor_flow;
extern Topic<MagData,         Queue, 2>       sensor_mag;
extern Topic<BaroData,        Queue, 2>       sensor_baro;
extern Topic<PowerData,       Latest, 1>      sensor_power;
extern Topic<SensorSnapshot,  Latest, 1>      sensor_snapshot;  // ImuTask mirror of async sensors (CLI/telemetry peek)

// =============================================================================
// Estimation Topics
// 推定トピック
// =============================================================================

extern Topic<StateEstimate,   Latest, 1>      estimate_state;

// =============================================================================
// Command Topics
// コマンドトピック
// =============================================================================

extern Topic<CommandSetpoint, Latest, 1>      command_setpoint;
extern Topic<PilotRequest,    Latest, 1>      pilot_request;   // arm + mode (sf_comm → StateTask)
extern Topic<ButtonEvent,     Queue, 4>       button_event;    // gesture fact (ButtonTask → StateTask)

// =============================================================================
// Control Topics
// 制御トピック
// =============================================================================

extern Topic<ControlOutput,   Latest, 1>      control_output;

// =============================================================================
// Actuation Topics
// アクチュエーショントピック
// =============================================================================

extern Topic<MotorOutput,     Latest, 1>      actuator_motor;
extern Topic<LogStreamSample, RingBuffer, 32> log_stream;        // ControlTask 400Hz → TelemetryTask (Data Stream)
extern Topic<FlowData,        RingBuffer, 8>  log_flow;          // ImuTask flow mirror → Data Stream。flow は差分量
                                                                 // （前回読み出しからの累積変位）ゆえ全サンプル配送が必須
                                                                 // Ring 32 = 80ms 分のバッファ（50Hz 排出のジッタ余裕）

// =============================================================================
// System Topics
// システムトピック
// =============================================================================

extern Topic<SystemMode,      Latest, 1>      system_mode;
extern Topic<SystemAlert,     Queue, 4>       system_alert;
extern Topic<SystemStatus,    Latest, 1>      system_status;   // boot readiness (ImuTask → pre-arm checks)

// =============================================================================
// Pairing Topics — transmitter↔vehicle binding (crosstalk prevention)
// ペアリングトピック — 送信機↔機体のバインド（混信対策）
// =============================================================================

extern Topic<PairingStatus,   Latest, 1>      pairing_state;     // decided PairingState (StateMgr → comm/notify)
extern Topic<PairingComplete, Latest, 1>      pairing_complete;  // comm bind status fact (comm → StateMgr)

// =============================================================================
// Bench / UI command Topics — CLI → owning task (future WiFi/UDP can inject too)
// ベンチ/UI コマンドトピック — CLI → 所有タスク（将来 WiFi/UDP からも注入可）
// =============================================================================

extern Topic<UiCommand,       Queue, 4>       ui_command;        // CLI → NotifyTask (sound/led)
extern Topic<MotorTest,       Latest, 1>      motor_test;        // CLI → ControlTask (motor test, disarmed only)

// =============================================================================
// Transition Command Topics — reset commands (StateManager callbacks → owning task)
// 遷移コマンドトピック — リセット指令（StateManager コールバック → 所有タスク）
//
// StateManager の onExit/onEnter コールバックが publish し、リソース所有タスクが消費する
// （architecture.md §4 コールバック集約 + R5 Pub-Sub 疎結合）。
// =============================================================================

extern Topic<EstimatorCommand,  Queue, 4>     estimator_command;   // StateMgr → ImuTask (estimator)
extern Topic<ControllerCommand, Queue, 4>     controller_command;  // StateMgr → ControlTask (controller)
extern Topic<NotifyCommand,     Queue, 8>     notify_command;      // StateMgr/Failsafe → NotifyTask

// =============================================================================
// Health Topic (R15)
// 健全性トピック（R15）
// =============================================================================

extern Topic<SensorHealth,      Latest, 1>    sensor_health;       // PowerTask ~1Hz → failsafe/telemetry
                                                                   // (PowerTask が sf_board の presence/freshness
                                                                   //  を集約して publish — 層逆転回避, power_task.cpp 参照)

// =============================================================================
// Reserved Topics — Guidance / Navigation (R11, not yet produced)
// 予約トピック — ガイダンス/ナビゲーション（R11、未発行）
// =============================================================================

extern Topic<GuidanceTarget,    Latest, 1>    command_target;      // RESERVED (M4+)
extern Topic<NavigationPath,    Queue, 4>     nav_path;            // RESERVED (Phase 6)

}  // namespace sf
