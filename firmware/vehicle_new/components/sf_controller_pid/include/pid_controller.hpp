/**
 * @file pid_controller.hpp
 * @brief PID controller implementation
 *        PIDコントローラ実装
 *
 * Cascade PID controller implementation of IController.
 * Currently a stub — returns zero thrust/torque.
 * Full PID will be ported from vehicle firmware.
 *
 * IControllerのカスケードPID実装。
 * 現在はスタブ — 零推力/トルクを返す。
 * 完全なPIDはvehicleファームから移植予定。
 *
 * @design requirements.md §4 — Component #6: replaceable control      [--]
 * @design detailed_design.md §4 — IController implementation          [--]
 */

#pragma once

#include "controller.hpp"

namespace sf {

class PidController : public IController {
public:
    /// Initialize PID gains from parameter system
    /// パラメータシステムからPIDゲインを初期化する
    void init();

    // IController interface implementation
    // IControllerインターフェース実装

    ControlOutput compute(
        const StateEstimate& state,
        const CommandSetpoint& setpoint,
        float dt
    ) override;

    void reset() override;
    void onModeChange(FlightMode new_mode) override;

private:
    FlightMode current_mode_ = FlightMode::STABILIZE;
};

}  // namespace sf
