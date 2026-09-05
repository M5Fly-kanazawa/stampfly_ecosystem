/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file learner_controller.cpp
 * @brief See learner_controller.hpp for what this class is for.
 *        このクラスの目的は learner_controller.hpp を参照。
 *
 * @design controller.hpp — IController interface (12 methods)   [OK]
 */

#include "learner_controller.hpp"

#include "config.hpp"

namespace sf {

void LearnerController::init()
{
    inner_controller_.init();
}

ControlOutput LearnerController::compute(const StateEstimate& state,
                                          const CommandSetpoint& setpoint, float dt)
{
    ControlOutput output = inner_controller_.compute(state, setpoint, dt);

    // =========================================================================
    // >>> YOUR CUSTOM CONTROL LAW GOES HERE <<<
    // >>> ここに独自の制御則を書く <<<
    // =========================================================================
    // This is the ONE line this wrapper changes relative to plain
    // PidController: scale the commanded yaw torque by a named constant. At
    // the default value (1.0, config.hpp) this is a no-op — change
    // config::kYawTorqueScale and rebuild to see the printed torque respond.
    // A real exercise would replace this line with actual control law code
    // (e.g. compute an independent yaw-rate P term instead of scaling
    // PidController's).
    // これが plain PidController に対して本ラッパーが変更する「唯一の1行」:
    // 指令ヨートルクを名前付き定数でスケールする。既定値（1.0、config.hpp）では
    // 何もしない — config::kYawTorqueScale を変えて再ビルドすると、表示される
    // トルクが反応する。実際の演習ではこの行を本物の制御則コード（例: PidController
    // の値をスケールする代わりに独立したヨーレートP項を計算する等）に置き換える。
    output.torque[2] *= config::kYawTorqueScale;

    return output;
}

void LearnerController::reset()
{
    inner_controller_.reset();
}

void LearnerController::onModeChange(FlightMode new_mode)
{
    inner_controller_.onModeChange(new_mode);
}

void LearnerController::onLanding()
{
    inner_controller_.onLanding();
}

void LearnerController::onTakeoff()
{
    inner_controller_.onTakeoff();
}

void LearnerController::onTakeoffComplete()
{
    inner_controller_.onTakeoffComplete();
}

bool LearnerController::isTakeoffComplete() const
{
    return inner_controller_.isTakeoffComplete();
}

void LearnerController::setGuidanceTarget(const GuidanceTarget& target,
                                           const CommandSetpoint& current_sticks)
{
    inner_controller_.setGuidanceTarget(target, current_sticks);
}

bool LearnerController::isGuidanceActive() const
{
    return inner_controller_.isGuidanceActive();
}

void LearnerController::startExcitation(const SysidCommand& cmd)
{
    inner_controller_.startExcitation(cmd);
}

bool LearnerController::fetchSysidResult(SysidFreqResult& out)
{
    return inner_controller_.fetchSysidResult(out);
}

void LearnerController::reloadParams()
{
    inner_controller_.reloadParams();
}

}  // namespace sf
