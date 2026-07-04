/**
 * @file position_controller.hpp
 * @brief Position Hold Controller (Horizontal Cascade PID)
 *
 * 水平位置保持コントローラ（カスケードPID）
 *
 * Control structure (NED frame):
 *   stick -> deadzone -> vel_cmd (body) -> yaw rotation -> vel_cmd (NED)
 *                                                              |
 *                              position_setpoint += vel_cmd_ned * dt
 *                                                              |
 *   ESKF position (NED) -> [Pos PID] -> vel_target -> [Vel PID] -> angle_correction (NED)
 *                           (outer)                    (inner)           |
 *                                                               yaw inverse rotation
 *                                                                        |
 *                                                             roll_cmd, pitch_cmd [rad]
 *                                                                        |
 *                                               -> AttitudeController -> RateController -> Motors
 */

#pragma once

#include "pid.hpp"
#include "config.hpp"
#include <cmath>
#include <algorithm>

struct PositionController {
    // Cascade PID for NED X/Y axes
    // NED X/Y軸のカスケードPID
    stampfly::PID pos_x_pid;    // Outer loop: position -> velocity target (NED X = North)
    stampfly::PID pos_y_pid;    // Outer loop: position -> velocity target (NED Y = East)
    stampfly::PID vel_x_pid;    // Inner loop: velocity -> angle correction (NED X)
    stampfly::PID vel_y_pid;    // Inner loop: velocity -> angle correction (NED Y)

    float pos_setpoint_x = 0.0f;   // Position setpoint [m] (NED X = North)
    float pos_setpoint_y = 0.0f;   // Position setpoint [m] (NED Y = East)
    bool position_captured = false;
    bool initialized = false;

    /**
     * @brief Initialize position controller with default parameters
     * デフォルトパラメータで位置コントローラを初期化
     */
    void init() {
        using namespace config::position_control;

        // Position PID (outer loop): position error [m] -> velocity target [m/s]
        // 位置PID（外ループ）: 位置誤差 → 速度目標
        stampfly::PIDConfig pos_cfg;
        pos_cfg.Kp = POS_KP;
        pos_cfg.Ti = POS_TI;
        pos_cfg.Td = POS_TD;
        pos_cfg.eta = PID_ETA;
        pos_cfg.output_min = -POS_OUTPUT_MAX;
        pos_cfg.output_max = POS_OUTPUT_MAX;
        pos_cfg.derivative_on_measurement = true;
        pos_x_pid.init(pos_cfg);
        pos_y_pid.init(pos_cfg);

        // Velocity PID (inner loop): velocity error [m/s] -> angle correction [rad]
        // 速度PID（内ループ）: 速度誤差 → 角度補正
        stampfly::PIDConfig vel_cfg;
        vel_cfg.Kp = VEL_KP;
        vel_cfg.Ti = VEL_TI;
        vel_cfg.Td = VEL_TD;
        vel_cfg.eta = PID_ETA;
        vel_cfg.output_min = -VEL_OUTPUT_MAX;
        vel_cfg.output_max = VEL_OUTPUT_MAX;
        vel_cfg.derivative_on_measurement = true;
        vel_x_pid.init(vel_cfg);
        vel_y_pid.init(vel_cfg);

        initialized = true;
    }

    /**
     * @brief Reset PID states and position capture
     * PID内部状態と位置キャプチャをリセット
     */
    void reset() {
        pos_x_pid.reset();
        pos_y_pid.reset();
        vel_x_pid.reset();
        vel_y_pid.reset();
        position_captured = false;
        pos_setpoint_x = 0.0f;
        pos_setpoint_y = 0.0f;
    }

    /**
     * @brief Capture current position as setpoint
     * 現在の位置をセットポイントとしてキャプチャ
     * @param x Current NED X position [m] (North)
     * @param y Current NED Y position [m] (East)
     */
    void capturePosition(float x, float y) {
        pos_setpoint_x = x;
        pos_setpoint_y = y;
        position_captured = true;
    }

    /**
     * @brief Convert stick value to velocity command
     * スティック値を速度指令に変換
     *
     * @param stick_value Normalized stick input [-1, 1]
     * @return Velocity command [m/s]
     */
    static float stickToVelocity(float stick_value) {
        using namespace config::position_control;

        // Apply deadzone
        // デッドゾーン適用
        if (stick_value > -STICK_DEADZONE && stick_value < STICK_DEADZONE) {
            return 0.0f;  // Hold position
        }

        // Map to velocity
        // 速度にマッピング
        if (stick_value >= STICK_DEADZONE) {
            float t = (stick_value - STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
            return t * MAX_HORIZONTAL_SPEED;
        } else {
            float t = (stick_value + STICK_DEADZONE) / (1.0f - STICK_DEADZONE);
            return t * MAX_HORIZONTAL_SPEED;  // t is negative
        }
    }

    /**
     * @brief Main update: compute roll/pitch angle commands from position control
     * メイン更新: 位置制御からroll/pitch角度指令を計算
     *
     * @param vel_cmd_body_x Body-frame forward velocity command [m/s] (from pitch stick)
     * @param vel_cmd_body_y Body-frame rightward velocity command [m/s] (from roll stick)
     * @param pos_x Current NED X position [m]
     * @param pos_y Current NED Y position [m]
     * @param vel_x Current NED X velocity [m/s]
     * @param vel_y Current NED Y velocity [m/s]
     * @param yaw Current yaw angle [rad]
     * @param dt Control period [s]
     * @param out_roll_angle Output roll angle command [rad]
     * @param out_pitch_angle Output pitch angle command [rad]
     */
    void update(float vel_cmd_body_x, float vel_cmd_body_y,
                float pos_x, float pos_y,
                float vel_x, float vel_y,
                float yaw, float dt,
                float& out_roll_angle, float& out_pitch_angle)
    {
        using namespace config::position_control;

        float cos_yaw = std::cos(yaw);
        float sin_yaw = std::sin(yaw);

        // 1. Body -> NED: convert stick velocity command to NED frame
        // Body → NED変換: スティック速度指令をNED座標系に変換
        // body_x = forward (pitch direction), body_y = right (roll direction)
        float vel_cmd_ned_x = cos_yaw * vel_cmd_body_x - sin_yaw * vel_cmd_body_y;
        float vel_cmd_ned_y = sin_yaw * vel_cmd_body_x + cos_yaw * vel_cmd_body_y;

        // 2. Update position setpoint from velocity command
        // 速度指令からセットポイントを更新
        if (vel_cmd_ned_x != 0.0f || vel_cmd_ned_y != 0.0f) {
            pos_setpoint_x += vel_cmd_ned_x * dt;
            pos_setpoint_y += vel_cmd_ned_y * dt;

            // Clamp setpoint to max offset from capture point
            // キャプチャ点からの最大距離でクランプ
            // (simple per-axis clamp; could use radial clamp for circular boundary)
            pos_setpoint_x = std::clamp(pos_setpoint_x,
                                        pos_x - MAX_POSITION_OFFSET,
                                        pos_x + MAX_POSITION_OFFSET);
            pos_setpoint_y = std::clamp(pos_setpoint_y,
                                        pos_y - MAX_POSITION_OFFSET,
                                        pos_y + MAX_POSITION_OFFSET);
        }

        // 3. Position PID (outer loop): position error -> velocity target [m/s]
        // 位置PID（外ループ）: 位置誤差 → 速度目標
        float vel_target_x = pos_x_pid.update(pos_setpoint_x, pos_x, dt);
        float vel_target_y = pos_y_pid.update(pos_setpoint_y, pos_y, dt);

        // 4. Velocity PID (inner loop): velocity error -> angle correction [rad] (NED)
        // 速度PID（内ループ）: 速度誤差 → 角度補正 (NED)
        float angle_ned_x = vel_x_pid.update(vel_target_x, vel_x, dt);
        float angle_ned_y = vel_y_pid.update(vel_target_y, vel_y, dt);

        // 5. NED -> Body: convert angle correction to body frame
        // NED → Body変換: 角度補正をbody frameに変換
        // In NED: positive pitch = nose up = backward, but we want
        // positive angle_ned_x (north correction) to pitch nose DOWN (forward).
        // So negate the pitch output.
        // NED: pitch正=機首上げ=後退だが、北方向への修正には機首下げ(前進)が必要
        // → pitch出力を反転
        float pitch_body = -(cos_yaw * angle_ned_x + sin_yaw * angle_ned_y);
        float roll_body  =  -sin_yaw * angle_ned_x + cos_yaw * angle_ned_y;

        // 6. Safety clamp: limit tilt angle from position controller
        // 安全クランプ: 位置制御からの傾斜角を制限
        out_roll_angle = std::clamp(roll_body, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
        out_pitch_angle = std::clamp(pitch_body, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    }
};

// Global position controller (defined in control_task.cpp)
// グローバル位置コントローラ（control_task.cppで定義）
extern PositionController g_position_controller;
