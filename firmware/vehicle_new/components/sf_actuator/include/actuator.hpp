/**
 * @file actuator.hpp
 * @brief Mixer and motor output — ControlOutput to MotorOutput conversion
 *        ミキサー＋モーター出力 — ControlOutputからMotorOutputへの変換
 *
 * Implements X-quad mixer that maps thrust + torque commands to
 * individual motor duties. Subscribes to control_output topic and
 * publishes to actuator_motor topic.
 *
 * X-quadミキサーとして推力＋トルク指令を各モーターdutyに変換する。
 * control_outputトピックを購読し、actuator_motorトピックに発行する。
 *
 * Motor layout (X-quad, top view):
 * モーター配置（X-quad、上面図）:
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
 * @design architecture.md §5 — Actuator subsystem                       [--]
 * @design detailed_design.md §5 — X-quad mixer                          [--]
 * @design coding_and_education.md §2 — Bilingual comments               [--]
 */

#pragma once

#include "data_types.hpp"

namespace sf {

/// Actuator: mixer + motor output
/// アクチュエータ: ミキサー＋モーター出力
class Actuator {
public:
    /// Initialize actuator subsystem
    /// アクチュエータサブシステムを初期化する
    void init();

    /// Run mixer: read ControlOutput, compute motor duties, publish & apply
    /// ミキサー実行: ControlOutput読み取り、モーターduty計算、発行＆適用
    void update();

private:
    /// Apply X-quad mixer: thrust + torque → 4 motor duties
    /// X-quadミキサー適用: 推力＋トルク → 4モーターduty
    ///
    /// M1 = T - R + P + Y  (front-right, CCW)
    /// M2 = T - R - P - Y  (rear-right,  CW)
    /// M3 = T + R - P + Y  (rear-left,   CCW)
    /// M4 = T + R + P - Y  (front-left,  CW)
    MotorOutput mix(const ControlOutput& cmd) const;

    /// Clamp motor duties to [0..1] range
    /// モーターdutyを[0..1]の範囲にクランプする
    void clampDuties(MotorOutput& out) const;
};

}  // namespace sf
