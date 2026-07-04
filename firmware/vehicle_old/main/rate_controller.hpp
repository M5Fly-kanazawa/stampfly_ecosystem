/**
 * @file rate_controller.hpp
 * @brief 角速度制御（Rate Control）用のコントローラ定義
 *
 * CLIからゲイン調整可能なPIDコントローラ構造体
 *
 * 物理単位モード (USE_PHYSICAL_UNITS=1):
 *   PID出力は物理トルク [Nm]
 *   ControlAllocatorでモータ推力 [N] → Duty変換
 *
 * 電圧スケールモード (USE_PHYSICAL_UNITS=0):
 *   PID出力は電圧スケール [V]
 *   レガシーミキサーでDuty変換
 */

#pragma once

#include "config.hpp"
#include "pid.hpp"
#include "control_allocation.hpp"

/**
 * @brief 角速度コントローラ
 *
 * Roll/Pitch/Yaw各軸のPIDコントローラと感度パラメータを管理
 */
struct RateController {
    stampfly::PID roll_pid;
    stampfly::PID pitch_pid;
    stampfly::PID yaw_pid;

    // 物理単位モード用コントロールアロケータ
    // Control allocator for physical units mode
    stampfly::ControlAllocator allocator;

    // 感度パラメータ（スティック最大 → 目標角速度 [rad/s]）
    float roll_rate_max;
    float pitch_rate_max;
    float yaw_rate_max;

    // 初期化フラグ
    bool initialized = false;

    /**
     * @brief 初期化（デフォルト値）
     */
    void init();

    /**
     * @brief PID内部状態リセット（ARM時に呼び出し）
     */
    void reset();
};

// グローバルレートコントローラ（control_task.cppで定義）
extern RateController g_rate_controller;
extern RateController* g_rate_controller_ptr;
