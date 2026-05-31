/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file config.hpp
 * @brief Fixed parameters (compile-time constants)
 *        固定パラメータ（コンパイル時定数）
 *
 * This file contains hardware-specific constants that do not change
 * at runtime: GPIO assignments, task priorities, stack sizes, and
 * timing constants.
 *
 * 本ファイルには実行時に変更しないハードウェア固有の定数を集約する:
 * GPIOピン割当、タスク優先度、スタックサイズ、タイミング定数。
 *
 * For tunable parameters (PID gains, ESKF settings, etc.),
 * see the parameter system in sf_core/params.hpp.
 *
 * チューニング可能なパラメータ（PIDゲイン、ESKF設定等）は
 * sf_core/params.hpp のパラメータシステムを参照。
 *
 * @design requirements.md §3 — Fixed params as constexpr              [--]
 * @design detailed_design.md §7 — config.hpp in main/                 [--]
 */

#pragma once

#include "freertos/FreeRTOS.h"

namespace config {

// =============================================================================
// GPIO Definitions
// GPIOピン定義
// =============================================================================

// SPI Bus (IMU, OptFlow)
// SPIバス（IMU、オプティカルフロー）
inline constexpr int GPIO_SPI_MOSI = 14;
inline constexpr int GPIO_SPI_MISO = 43;
inline constexpr int GPIO_SPI_SCK  = 44;
inline constexpr int GPIO_IMU_CS   = 46;
inline constexpr int GPIO_FLOW_CS  = 12;

// I2C Bus (ToF, Mag, Baro, Power)
// I2Cバス（ToF、地磁気、気圧、電源モニタ）
inline constexpr int GPIO_I2C_SDA = 3;
inline constexpr int GPIO_I2C_SCL = 4;

// ToF XSHUT pins
// ToF XSHUTピン
inline constexpr int GPIO_TOF_XSHUT_BOTTOM = 7;
inline constexpr int GPIO_TOF_XSHUT_FRONT  = 9;

// Motors (LEDC PWM) — X-quad layout
// モーター（LEDC PWM）— X-quad配置
inline constexpr int GPIO_MOTOR_M1 = 42;  // FR, CCW
inline constexpr int GPIO_MOTOR_M2 = 41;  // RR, CW
inline constexpr int GPIO_MOTOR_M3 = 10;  // RL, CCW
inline constexpr int GPIO_MOTOR_M4 = 5;   // FL, CW

// Peripherals
// 周辺機器
inline constexpr int GPIO_LED_MCU  = 21;  // M5Stamp S3 built-in LED
inline constexpr int GPIO_LED_BODY = 39;  // StampFly board LED (2x daisy-chain)
inline constexpr int GPIO_BUZZER   = 40;
inline constexpr int GPIO_BUTTON   = 0;

// =============================================================================
// Task Priorities (higher number = higher priority)
// タスク優先度（大きいほど高優先度）
//
// @design detailed_design.md §8 — Task list                           [--]
// @design architecture.md §6 — Task design                            [--]
// =============================================================================

inline constexpr UBaseType_t PRIORITY_IMU       = 24;
inline constexpr UBaseType_t PRIORITY_CONTROL   = 23;
inline constexpr UBaseType_t PRIORITY_STATE     = 22;
inline constexpr UBaseType_t PRIORITY_OPTFLOW   = 20;
inline constexpr UBaseType_t PRIORITY_MAG       = 18;
inline constexpr UBaseType_t PRIORITY_BARO      = 16;
inline constexpr UBaseType_t PRIORITY_COMM      = 15;
inline constexpr UBaseType_t PRIORITY_TOF       = 14;
inline constexpr UBaseType_t PRIORITY_TELEMETRY = 13;
inline constexpr UBaseType_t PRIORITY_POWER     = 12;
inline constexpr UBaseType_t PRIORITY_BUTTON    = 10;
inline constexpr UBaseType_t PRIORITY_NOTIFY    = 8;
inline constexpr UBaseType_t PRIORITY_CLI       = 5;
inline constexpr UBaseType_t PRIORITY_LOG       = 5;

// =============================================================================
// Task Stack Sizes [bytes]
// タスクスタックサイズ [バイト]
//
// @design detailed_design.md §8 — RAM: 92KB total stacks              [--]
// =============================================================================

inline constexpr uint32_t STACK_IMU       = 16384;
inline constexpr uint32_t STACK_CONTROL   = 8192;
inline constexpr uint32_t STACK_STATE     = 4096;
inline constexpr uint32_t STACK_OPTFLOW   = 8192;
inline constexpr uint32_t STACK_MAG       = 8192;
inline constexpr uint32_t STACK_BARO      = 8192;
inline constexpr uint32_t STACK_COMM      = 4096;
inline constexpr uint32_t STACK_TOF       = 8192;
inline constexpr uint32_t STACK_TELEMETRY = 4096;
inline constexpr uint32_t STACK_POWER     = 4096;
inline constexpr uint32_t STACK_BUTTON    = 4096;
inline constexpr uint32_t STACK_NOTIFY    = 4096;
inline constexpr uint32_t STACK_CLI       = 8192;
inline constexpr uint32_t STACK_LOG       = 4096;

// =============================================================================
// Timing Constants
// タイミング定数
//
// @design requirements.md §8 — Timing requirements                    [--]
// =============================================================================

inline constexpr float IMU_DT     = 0.0025f;   // 400Hz
inline constexpr uint32_t IMU_PERIOD_US = 2500;  // 400Hz loop period [us] (= IMU_DT·1e6)
inline constexpr float OPTFLOW_DT = 0.01f;     // 100Hz
inline constexpr float MAG_DT     = 0.04f;     // 25Hz
inline constexpr float BARO_DT    = 0.02f;     // 50Hz
inline constexpr float TOF_DT     = 0.033f;    // 30Hz

// =============================================================================
// Motor Configuration
// モーター設定
// =============================================================================

inline constexpr int MOTOR_PWM_FREQ_HZ       = 150000;
inline constexpr int MOTOR_PWM_RESOLUTION_BIT = 8;

// =============================================================================
// LED Configuration
// LED設定
// =============================================================================

inline constexpr int LED_NUM_MCU  = 1;   // M5Stamp S3 built-in
inline constexpr int LED_NUM_BODY = 2;   // StampFly board (top + bottom)

// =============================================================================
// Button Configuration
// ボタン設定
// =============================================================================

inline constexpr int BUTTON_DEBOUNCE_MS = 50;

// =============================================================================
// Buzzer Configuration
// ブザー設定
// =============================================================================

inline constexpr int BUZZER_LEDC_CHANNEL = 4;
inline constexpr int BUZZER_LEDC_TIMER   = 1;

}  // namespace config
