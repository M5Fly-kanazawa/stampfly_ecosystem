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
 * @design requirements.md §4 — Component #6: replaceable control      [OK]
 * @design detailed_design.md §4 — IController                        [OK]
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

    /// POS_HOLD cascade: position/velocity error → tilt setpoints (roll/pitch).
    /// Writes roll_sp/pitch_sp [rad], overriding the stick values in the caller.
    /// POS_HOLD カスケード: 位置/速度誤差 → 傾き指令（roll/pitch）。roll_sp/pitch_sp[rad]
    /// を書き込み、呼び出し側のスティック値を上書きする。
    void computePositionHold(const StateEstimate& state, float yaw, float dt,
                             float& roll_sp, float& pitch_sp);

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
    // Thrust output is PHYSICAL total thrust [N]: the B^-1 mixer (actuator.cpp)
    // allocates it across the motors and converts to duty via the motor curve.
    // max_thrust_ = 4 × max-per-motor (0.168 N) = 0.672 N (T/W ≈ 1.85); hover
    // is mg = 0.037 × 9.81 = 0.363 N (throttle ≈ 0.54 in STABILIZE).
    // スラスト出力は物理の総推力 [N]: B^-1 ミキサーが各モータに配分しモータ曲線で duty に。
    // max_thrust_ = 4×最大/モータ(0.168N) = 0.672N（T/W≈1.85）、ホバーは mg=0.363N。
    float max_thrust_     = 0.672f;  // [N] total (4 × 0.168 N per motor)
    float hover_thrust_   = 0.363f;  // [N] mg = 0.037 × 9.81
    float max_climb_rate_ = 0.5f;    // [m/s]
    float stick_deadzone_ = 0.1f;
    float alt_setpoint_   = 0;       // [m] captured altitude (ALT_HOLD target)
    bool  capture_alt_    = false;   // capture alt_setpoint on the next ALT_HOLD compute
    float gravity_        = 9.81f;   // [m/s²] for the accel→tilt mapping in POS_HOLD
    float max_pos_tilt_   = 0.1745f; // [rad] POS_HOLD tilt limit (10 deg; matches the
                                     // proven firmware/vehicle margin so altitude holds
                                     // without 1/cosθ thrust compensation, and the small
                                     // tilt keeps |a|≈g so accel-attitude stays valid)
    float pos_setpoint_x_ = 0;       // [m] captured position N (POS_HOLD target, NED)
    float pos_setpoint_y_ = 0;       // [m] captured position E (POS_HOLD target, NED)
    bool  capture_pos_    = false;   // capture pos_setpoint on the next POS_HOLD compute
};

}  // namespace sf
