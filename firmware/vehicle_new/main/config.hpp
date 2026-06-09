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
 * @design requirements.md §3 — Fixed params as constexpr              [OK]
 * @design detailed_design.md §7 — config.hpp in main/                 [OK]
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
// @design detailed_design.md §8 — Task list                           [OK]
// @design architecture.md §6 — Task design                            [OK]
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
// @design detailed_design.md §8 — RAM: 92KB total stacks              [OK]
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
// @design requirements.md §8 — Timing requirements                    [OK]
// =============================================================================

inline constexpr float IMU_DT     = 0.0025f;   // 400Hz
inline constexpr uint32_t IMU_PERIOD_US = 2500;  // 400Hz loop period [us] (= IMU_DT·1e6)
inline constexpr float OPTFLOW_DT = 0.01f;     // 100Hz
inline constexpr float MAG_DT     = 0.04f;     // 25Hz
inline constexpr float BARO_DT    = 0.02f;     // 50Hz
inline constexpr float TOF_DT     = 0.033f;    // 30Hz

// ControlTask wake-up watchdog: ControlTask normally wakes on the ImuTask
// notification every IMU period (2.5 ms). If no notification arrives within
// this window (= 4 missed IMU cycles), the IMU pipeline is considered stalled
// and ControlTask forces the motors to zero. Without this, a dead IMU would
// leave the LEDC outputs frozen at the last written duty (flyaway).
// ControlTask 起床ウォッチドッグ: ControlTask は通常、IMU 周期（2.5ms）毎の ImuTask
// 通知で起きる。この窓（= IMU 4 周期分）内に通知が来なければ IMU パイプライン停止と
// みなし、モータを強制的にゼロへ。これが無いと IMU 死亡時に LEDC 出力が最後の duty で
// 固着する（フライアウェイ）。
inline constexpr uint32_t CONTROL_NOTIFY_TIMEOUT_MS = 10;  // = 4 × IMU period

// =============================================================================
// Boot Calibration — 起動キャリブレーション
//
// @design detailed_design.md §3 — onEnter(IDLE): start calibration    [OK]
// =============================================================================

// Number of at-rest IMU samples averaged for the boot gyro/accel bias estimate.
// Collected one-per-400Hz-cycle in the loop (2.5 s), well inside the on-ground window
// before ARM. More samples → lower bias variance (σ/√N).
// 起動時のジャイロ/加速度バイアス推定で平均する静止サンプル数。ループ内で 400Hz の1周期に
// 1個収集（2.5秒）、ARM 前の地上時間に十分収まる。増やすとバイアス分散が下がる（σ/√N）。
inline constexpr uint32_t CALIB_GYRO_SAMPLES = 1000;

// Settle delay before collecting calibration samples [ms]. The craft must be at TRUE
// rest: at boot a freshly-placed (or, in SIL, ground-contact-settling) craft has a
// vertical transient that biases the accel average — averaging through it produces a
// spurious accel bias that degrades the estimator. Waiting lets the transient decay
// so the average reflects gravity + the true bias only.
// 校正サンプル収集前の整定待ち [ms]。機体は「真の静止」である必要がある: 起動直後は
// 置いた直後（SIL では接地接触の整定）の鉛直過渡があり加速度平均をバイアスする — 過渡を
// 含めて平均すると spurious な加速度バイアスを生み推定器を劣化させる。待つことで過渡が
// 減衰し、平均が重力＋真のバイアスのみを反映する。
inline constexpr uint32_t CALIB_SETTLE_MS = 1000;

// Bias deadband: if the measured boot bias is below these magnitudes it is treated as
// negligible and NOT seeded into the estimator — the calibration stays a true no-op
// instead of overwriting the filter's own (already ~zero) online bias estimate. This
// keeps a clean IMU byte-neutral (the SIL plant's only residual is the gravity-constant
// rounding ~0.002 m/s²), while a real MEMS offset (~0.1–0.4 m/s²) is well above and
// applied. Seeding a near-zero "calibration" mid-run otherwise discards the filter's
// converged estimate and perturbs the (marginal) POSITION_HOLD entry.
// バイアスのデッドバンド: 測定した起動バイアスがこの大きさ未満なら無視可能として推定器に
// 種付けしない — 校正をフィルタ自身の（既にほぼゼロの）オンライン推定の上書きでなく真の
// no-op に保つ。クリーンな IMU を byte 中立に保ち（SIL プラントの残差は重力定数の丸め
// ~0.002 m/s² のみ）、実機の MEMS オフセット（~0.1–0.4 m/s²）は十分上回り適用される。
// ほぼゼロの「校正」を実行中に種付けすると、フィルタの収束済み推定を捨て、（脆弱な）
// POSITION_HOLD 入口を撹乱してしまう。
inline constexpr float CALIB_GYRO_DEADBAND  = 0.002f;  // [rad/s]
inline constexpr float CALIB_ACCEL_DEADBAND = 0.010f;  // [m/s²]

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

// =============================================================================
// Flight State Transition Constants
// フライト状態遷移定数
//
// @design requirements.md §2 — ARMED_GROUND → TAKEOFF → FLYING        [OK]
// =============================================================================

// Normalized throttle (0=stick centre/idle .. 1=full up) above which an
// ARMED_GROUND craft begins takeoff (requirements §2: "throttle input").
// Hover throttle is ≈0.54 (mg / max_thrust), so 0.5 means the pilot has
// advanced the stick toward/above hover — a deliberate intent to lift off.
// 正規化スロットル(0=中央/アイドル..1=全上げ)。ARMED_GROUND でこれを超えると離陸
// 開始(要件§2「スロットル入力」)。ホバーは≈0.54 ゆえ 0.5 は意図的な上げ＝離陸意思。
inline constexpr float TAKEOFF_THROTTLE_THRESH = 0.5f;

// =============================================================================
// Sensor Health Monitoring (R15)
// センサ健全性監視（R15）
// =============================================================================

// A sensor counts as "healthy" only if its most recent topic sample is newer than
// this window. The slowest sensor is the power monitor at 10 Hz (100 ms period), so
// 500 ms tolerates several missed samples before flagging a sensor dead — it detects
// a stalled/absent sensor without false-tripping on transient single-sample drops.
// Published at 1 Hz by PowerTask (sensor_health topic).
// センサが「健全」とみなされるのは、直近のトピックサンプルがこの窓より新しい場合のみ。
// 最も遅いセンサは電源モニタの 10Hz（周期 100ms）ゆえ、500ms なら数サンプルの欠落を
// 許容してからセンサ死亡と判定する — 単発の瞬間的欠落で誤発火せず、停止/不在センサを
// 検出する。PowerTask が 1Hz で publish（sensor_health トピック）。
inline constexpr uint32_t SENSOR_HEALTH_STALE_US = 500000;  // 0.5 s

}  // namespace config
