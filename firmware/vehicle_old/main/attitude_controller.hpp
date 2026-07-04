/**
 * @file attitude_controller.hpp
 * @brief 姿勢制御（外側ループ） - Attitude Controller (Outer Loop)
 *
 * カスケード制御の外側ループ。スティック入力から角度セットポイントを計算し、
 * 内側のレートPIDへのレートセットポイントを生成する。
 *
 * Outer loop of cascade control. Converts stick input to angle setpoints
 * and generates rate setpoints for the inner rate PID loop.
 *
 * カスケード構造 / Cascade structure:
 *   stick → [Attitude PID] → rate_setpoint → [Rate PID] → motor_output
 *
 * STABILIZEモード:
 *   - Roll/Pitch: 角度制御（スティック → 角度目標 → レート目標）
 *   - Yaw: 直接レート制御（ACROと同じ）
 */

#pragma once

#include "config.hpp"
#include "pid.hpp"

/**
 * @brief 姿勢コントローラ（外側ループ）
 *
 * Roll/Pitch角度のPIDコントローラを管理し、
 * 内側のレートコントローラへのセットポイントを生成する。
 */
struct AttitudeController {
    // Roll/Pitch angle PIDs (outer loop)
    // 姿勢PID（外側ループ）
    stampfly::PID roll_angle_pid;
    stampfly::PID pitch_angle_pid;

    // Maximum tilt angles [rad]
    // 最大傾斜角 [rad]
    float max_roll_angle;
    float max_pitch_angle;

    // Yaw rate max (direct rate control)
    // Yaw最大レート（直接レート制御）
    float yaw_rate_max;

    // Initialization flag
    // 初期化フラグ
    bool initialized = false;

    /**
     * @brief 初期化（デフォルト値）
     */
    void init();

    /**
     * @brief PID内部状態リセット（ARM時・モード切替時に呼び出し）
     */
    void reset();

    /**
     * @brief 姿勢制御を更新（外側ループ）
     *
     * スティック入力と現在姿勢から、レートセットポイントを計算
     *
     * @param stick_roll  ロールスティック入力 [-1, 1]
     * @param stick_pitch ピッチスティック入力 [-1, 1]
     * @param stick_yaw   ヨースティック入力 [-1, 1]（直接レート制御）
     * @param roll_current  現在のロール角 [rad]（ESKFより）
     * @param pitch_current 現在のピッチ角 [rad]（ESKFより）
     * @param dt 制御周期 [s]
     * @param roll_rate_ref  出力: ロールレート目標 [rad/s]
     * @param pitch_rate_ref 出力: ピッチレート目標 [rad/s]
     * @param yaw_rate_ref   出力: ヨーレート目標 [rad/s]
     */
    void update(
        float stick_roll, float stick_pitch, float stick_yaw,
        float roll_current, float pitch_current,
        float dt,
        float& roll_rate_ref, float& pitch_rate_ref, float& yaw_rate_ref
    );
};

// グローバル姿勢コントローラ（control_task.cppで定義）
// Global attitude controller (defined in control_task.cpp)
extern AttitudeController g_attitude_controller;
