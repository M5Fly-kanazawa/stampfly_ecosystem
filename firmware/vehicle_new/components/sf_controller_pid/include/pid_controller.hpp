/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file pid_controller.hpp
 * @brief Cascade PID controller — IController implementation
 *        カスケードPIDコントローラ — IController実装
 *
 * @design requirements.md §4 — Component #6: replaceable control      [--]
 * @design detailed_design.md §4 — IController                        [--]
 */

#pragma once

#include "controller.hpp"
#include "pid.hpp"
#include "sf_math.hpp"

namespace sf {

class PidController : public IController {
public:
    void init();

    ControlOutput compute(
        const StateEstimate& state,
        const CommandSetpoint& setpoint,
        float dt) override;

    void reset() override;
    void onModeChange(FlightMode new_mode) override;

private:
    /// Load PID gains from parameter system / パラメータからゲインを読み込み
    void loadParams();

    FlightMode current_mode_ = FlightMode::STABILIZE;

    // Rate control PIDs (innermost loop) / レート制御PID（最内ループ）
    PID rate_roll_, rate_pitch_, rate_yaw_;

    // Attitude control PIDs (outer loop) / 姿勢制御PID（外ループ）
    PID att_roll_, att_pitch_;

    // Altitude control PIDs / 高度制御PID
    PID alt_pos_, alt_vel_;

    // Position control PIDs / 位置制御PID
    PID pos_x_, pos_y_;
    PID vel_x_, vel_y_;

    // Constants / 定数
    float max_rate_       = 1.0f;    // [rad/s] max rate setpoint
    float max_yaw_rate_   = 5.0f;    // [rad/s] max yaw rate
    float max_angle_      = 0.5236f; // [rad] max tilt (30 deg)
    float max_thrust_     = 0.672f;  // [N] 4 motors
    float hover_thrust_   = 0.363f;  // [N] mg = 0.037 * 9.81
    float max_climb_rate_ = 0.5f;    // [m/s]
    float stick_deadzone_ = 0.1f;
    float alt_setpoint_   = 0;       // [m] captured altitude
};

}  // namespace sf
