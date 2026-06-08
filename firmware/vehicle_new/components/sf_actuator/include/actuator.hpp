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
 *     d_FR (CCW) =  thrust − roll + pitch + yaw·κ
 *     d_RR (CW)  =  thrust − roll − pitch − yaw·κ
 *     d_RL (CCW) =  thrust + roll − pitch + yaw·κ
 *     d_FL (CW)  =  thrust + roll + pitch − yaw·κ
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

    /// Enable motor output (safety gate for ARM transitions). Idempotent: the
    /// motor HAL is armed only on the disarmed→armed edge. Until armed, the HAL
    /// silently swallows every duty write — so update() does nothing useful
    /// unless arm() has been called first.
    /// モーター出力を有効化（ARM 遷移用の安全ゲート）。冪等: 立上りエッジでのみ HAL を
    /// arm する。arm されるまで HAL は全 duty 書き込みを握り潰すため、arm() 後でないと
    /// update() は実際にはモーターを回さない。
    void arm();

    /// Run mixer: read ControlOutput, compute motor duties, publish & apply
    /// ミキサー実行: ControlOutput読み取り、モーターduty計算、発行＆適用
    void update();

    /// Stop all motors immediately (safety hook for DISARM transitions).
    /// Idempotent: zeroes the LEDC outputs and clears the HAL arm gate on the
    /// armed→disarmed edge.
    /// 全モーターを直ちに停止（DISARM 遷移用の安全フック）。冪等: 立下りエッジで LEDC を
    /// 0 にし HAL の arm ゲートを下げる。
    void disarm();

    /// BENCH MOTOR TEST: drive the 4 motors to the given raw duties [0,1], bypassing the
    /// mixer. Arms the HAL and publishes actuator_motor for telemetry/SIL. The CALLER
    /// (ControlTask) must gate this to the DISARMED state only — it exists for the
    /// wiring/direction check (development_roadmap Phase 2). Not used in flight.
    /// ベンチ用モータテスト: ミキサーを介さず 4 モータを生 duty [0,1] で駆動。HAL を arm し
    /// テレメトリ用に actuator_motor を発行。呼び出し側（ControlTask）が **disarmed 限定**に
    /// gate すること — 配線/回転方向確認用（development_roadmap Phase 2）。飛行では使わない。
    void applyTestDuties(const float duties[4]);

private:
    /// Actuator-level arm state, mirrors the motor HAL gate so arm()/disarm()
    /// can be called every control cycle without re-arming or log spam.
    /// アクチュエータ層の arm 状態。HAL ゲートを写し、毎制御周期に arm()/disarm() を
    /// 呼んでも再 arm やログ氾濫が起きないようにする。
    bool armed_ = false;
};

}  // namespace sf
