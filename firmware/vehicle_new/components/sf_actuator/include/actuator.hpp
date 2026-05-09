/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file actuator.hpp
 * @brief Mixer and motor output — ControlOutput to MotorOutput conversion
 *        ミキサー＋モーター出力 — ControlOutputからMotorOutputへの変換
 *
 * Implements an X-quad mixer that maps thrust + torque commands into
 * per-motor duty cycles, publishes them on the `actuator_motor` topic
 * for telemetry, and drives the LEDC PWM motor HAL.
 *
 * X-quadミキサーとして推力＋トルク指令を各モーターdutyに変換し、
 * テレメトリ用に `actuator_motor` トピックに発行し、LEDC PWM モーター
 * HAL を駆動する。
 *
 * Motor layout (X-quad, top view, M1=FR, M2=RR, M3=RL, M4=FL):
 * モーター配置（X-quad、上面図、M1=FR, M2=RR, M3=RL, M4=FL）:
 *
 *           Front
 *      M4(CW)   M1(CCW)
 *         \  ^  /
 *          \ | /
 *           \|/
 *            X
 *           /|\
 *          / | \
 *         /  |  \
 *      M3(CCW)  M2(CW)
 *           Rear
 *
 * Mixer formula (kappa applied to yaw inside the implementation):
 * ミキサー式（kappa は実装内でヨーに適用）:
 *
 *     d_FR =  thrust − roll − pitch − yaw·κ
 *     d_RR =  thrust − roll + pitch + yaw·κ
 *     d_RL =  thrust + roll + pitch − yaw·κ
 *     d_FL =  thrust + roll − pitch + yaw·κ
 *
 * @design architecture.md §5 — Actuator subsystem                       [OK]
 * @design detailed_design.md §5 — X-quad mixer                          [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#pragma once

#include "data_types.hpp"

namespace sf {

/// Actuator: mixer + motor output
/// アクチュエータ: ミキサー＋モーター出力
class Actuator {
public:
    /// Initialize actuator subsystem (configures the LEDC PWM motor HAL)
    /// アクチュエータサブシステムを初期化（LEDC PWM モーター HAL を設定）
    void init();

    /// Run mixer: read ControlOutput, compute motor duties, publish & apply
    /// ミキサー実行: ControlOutput読み取り、モーターduty計算、発行＆適用
    void update();

    /// Stop all motors immediately (safety hook for DISARM transitions)
    /// 全モーターを直ちに停止（DISARM 遷移用の安全フック）
    void disarm();
};

}  // namespace sf
