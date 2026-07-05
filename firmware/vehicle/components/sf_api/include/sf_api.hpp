/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file sf_api.hpp
 * @brief Tier L1 — Public API for learner code (推定/制御/ガイダンス学習者向け)
 *        Tier L1 — 学習者コード向けの公開 API
 *
 * 4 階層アクセス (architecture.md §2.5) における **L1 Topic API** の単一公開
 * ヘッダ。学習者は本ヘッダを include するだけで Topic / params / state /
 * mode へのアクセスが揃う。L2 (HAL) や L3 (BSP internal) を直接見る必要は
 * ない。
 *
 * Single public header for the **L1 Topic API** of the 4-tier learner access
 * model (architecture.md §2.5). Including this header alone gives a learner
 * everything needed to read sensors, observe state, and write their own
 * estimator / controller / guidance — without descending to L2 (HAL) or L3
 * (BSP internal).
 *
 * Scope (M2d):
 *   - Read-only views of all topics (latest value snapshots)
 *   - Convenience wrappers around Topic<>::latest()
 *
 * Out of scope (deferred to M5 = Workshop integration):
 *   - Actuator control (motor duty, motor mixer)
 *   - State requests (request_arm / request_disarm)
 *   - These need StateManager idempotency work (workshop_migration.md §4-2)
 *     and are handled in the same milestone.
 *
 * @design architecture.md §2.5 — 学習者の入口 (4 階層アクセス)         [OK]
 * @design coding_and_education.md §2 — Namespace 規約                 [OK]
 * @design topic_reference.md §4 — Usage patterns                      [OK]
 *
 * Cross-cutting rules / 横断ルール:
 *   R8: namespace は sf::api (L1 専用)。sf::internal::* には依存しない
 *   R9: Topic アクセスは topic_reference.md §3 の SSOT に従う
 *   R13: 学習者の publish 経路は本ヘッダを通る (lint で検出可能になる)
 */

#pragma once

#include "topics.hpp"
#include "data_types.hpp"
#include "flight_state.hpp"

namespace sf::api {

// ============================================================================
// Sensor snapshots (read-only views of latest values)
// センサ値のスナップショット (最新値の read-only ビュー)
// ============================================================================

/**
 * @brief Latest IMU reading (without consuming the ring buffer).
 *        最新の IMU 値を取得する (リングバッファを消費しない)。
 *
 * sensor_imu は RingBuffer 方式 (size=8、SPSC、ImuTask が消費) のため、
 * 通常の read() は使えない (複数 reader での動作は未定義)。本関数は
 * RingBuffer の latest() を呼び、head の 1 つ前を読むだけなので、
 * 任意のタスクから何度呼んでも安全。
 *
 * sensor_imu uses RingBuffer policy (size=8, SPSC, consumed by ImuTask),
 * so calling read() from anywhere else is undefined. This wrapper uses
 * Topic<RingBuffer>::latest() which only peeks head-1 and is safe to call
 * from any task at any frequency.
 *
 * @return Latest IMU sample. Returns zero-initialised data if no IMU
 *         publish has occurred yet (early boot).
 */
inline ImuData imu_latest()
{
    return sensor_imu.latest();
}

/**
 * @brief Latest fused state estimate (attitude / position / velocity).
 *        最新の状態推定値 (姿勢 / 位置 / 速度) を取得する。
 */
inline StateEstimate estimate_latest()
{
    return estimate_state.latest();
}

/**
 * @brief Latest pilot command setpoint (throttle / roll / pitch / yaw — the
 *        continuous stick stream; the discrete ARM/mode flags travel separately
 *        on the pilot_request topic).
 *        最新のパイロット指令セットポイント（throttle / roll / pitch / yaw — 連続的
 *        なスティック値。離散的な ARM/モードフラグは pilot_request トピックで別送）。
 */
inline CommandSetpoint command_latest()
{
    return command_setpoint.latest();
}

/**
 * @brief Latest control output (thrust / torques computed by Controller).
 *        最新の制御出力 (Controller が計算した推力 / トルク) を取得する。
 */
inline ControlOutput control_latest()
{
    return control_output.latest();
}

/**
 * @brief Latest motor duty values (after mixer).
 *        ミキサー後の最新モータ duty 値を取得する。
 */
inline MotorOutput motor_latest()
{
    return actuator_motor.latest();
}

/**
 * @brief Latest power monitor reading (battery voltage / current).
 *        最新の電源モニタ値 (バッテリ電圧 / 電流) を取得する。
 */
inline PowerData power_latest()
{
    return sensor_power.latest();
}

// ============================================================================
// System mode (read-only)
// システムモード (read-only)
// ============================================================================

/**
 * @brief Current system mode (ARM state + flight mode).
 *        現在のシステムモード (ARM 状態 + フライトモード) を取得する。
 *
 * SystemMode のフィールド (data_types.hpp で定義):
 *   - state:    FlightState enum (INIT / IDLE_GROUND / IDLE_HELD /
 *               ARMED_GROUND / TAKEOFF / FLYING / LANDING の 7 状態) を
 *               uint8_t で保持 (flight_state.hpp が SSOT)
 *   - sub_mode: FlightMode enum (ACRO / STABILIZE / ALTITUDE_HOLD /
 *               POSITION_HOLD) を uint8_t で保持
 *   - armed:    便宜上の bool フラグ (StateManager が更新)
 *
 * @return Latest system mode snapshot.
 */
inline SystemMode current_mode()
{
    return system_mode.latest();
}

/**
 * @brief Convenience: is the vehicle currently armed?
 *        便利関数: 現在 ARM 状態か?
 *
 * SystemMode::armed フラグを直接読む。FlightState 再評価より高速、
 * StateManager の publishMode() で同時に更新されるため一貫性も担保。
 */
inline bool is_armed()
{
    return current_mode().armed;
}

}  // namespace sf::api
