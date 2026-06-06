/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file data_types.hpp
 * @brief Topic data type definitions
 *        トピックデータ型定義
 *
 * All data structures exchanged between components via Pub-Sub topics.
 * Sensor topics carry raw data only — filtering is the subscriber's
 * responsibility.
 *
 * Pub-Subトピック経由でコンポーネント間で交換される全データ構造体。
 * センサトピックは生値のみを運ぶ — フィルタリングは購読者側の責務。
 *
 * @design architecture.md §3 — Topic data types                       [--]
 * @design detailed_design.md §2 — Data type definitions               [--]
 * @design coding_and_education.md §2 — Bilingual comments             [--]
 */

#pragma once

#include <cstdint>

namespace sf {

// =============================================================================
// Sensor Data Types
// センサデータ型（生値のみ）
// =============================================================================

/// IMU data from BMI270 (400Hz)
/// IMUデータ — BMI270（400Hz）
struct ImuData {
    float accel[3];       // Accelerometer [m/s²]  / 加速度
    float gyro[3];        // Gyroscope [rad/s]     / ジャイロ
    float temperature;    // Chip temperature [°C]  / チップ温度
    uint32_t timestamp;   // Microseconds [us]     / マイクロ秒
};

/// ToF distance data from VL53L3CX (30Hz)
/// ToF距離データ — VL53L3CX（30Hz）
struct TofData {
    float distance;       // Distance [m]          / 距離
    uint8_t status;       // Sensor status code    / センサステータス
    bool valid;           // Data validity flag    / データ有効フラグ
    uint32_t timestamp;   // [us]
};

/// Optical flow data from PMW3901 (100Hz)
/// オプティカルフローデータ — PMW3901（100Hz）
struct FlowData {
    int16_t dx;           // X displacement [counts] / X変位
    int16_t dy;           // Y displacement [counts] / Y変位
    uint8_t squal;        // Surface quality        / 表面品質
    uint32_t timestamp;   // [us]
};

/// Magnetometer data from BMM150 (25Hz)
/// 地磁気データ — BMM150（25Hz）
struct MagData {
    float mag[3];         // Magnetic field [uT]   / 磁場
    uint32_t timestamp;   // [us]
};

/// Barometer data from BMP280 (50Hz)
/// 気圧データ — BMP280（50Hz）
struct BaroData {
    float pressure;       // Pressure [Pa]         / 気圧
    float temperature;    // Temperature [°C]      / 温度
    float altitude;       // Pressure-derived [m]  / 気圧高度
    uint32_t timestamp;   // [us]
};

/// Power monitor data from INA3221 (10Hz)
/// 電源モニタデータ — INA3221（10Hz）
struct PowerData {
    float voltage;        // Battery voltage [V]   / バッテリー電圧
    float current;        // Current draw [mA]     / 電流
    float power;          // Power consumption [mW] / 消費電力
    uint32_t timestamp;   // [us]
};

// =============================================================================
// Estimation Data Types
// 推定データ型
// =============================================================================

/// State estimate output (400Hz)
/// 状態推定出力（400Hz）
struct StateEstimate {
    float attitude[4];    // Quaternion [w,x,y,z]  / 姿勢クォータニオン
    float position[3];    // Position [m] NED      / 位置
    float velocity[3];    // Velocity [m/s] NED    / 速度
    float gyro_bias[3];   // Gyro bias [rad/s]     / ジャイロバイアス
    float accel_bias[3];  // Accel bias [m/s²]     / 加速度バイアス
    float angular_rate[3];// Body rate [rad/s] FRD / 機体角速度（バイアス補正済み, gyro−bias）
    uint8_t sensor_mask;  // Active sensor bitmask / 有効センサマスク
    uint32_t timestamp;   // [us]
};

// =============================================================================
// Command Data Types
// コマンドデータ型
// =============================================================================

/// Pilot/API command setpoint
/// パイロット/APIコマンドセットポイント
struct CommandSetpoint {
    float throttle;       // Throttle [0..1]       / スロットル
    float roll;           // Roll command [-1..1]  / ロール指令
    float pitch;          // Pitch command [-1..1] / ピッチ指令
    float yaw;            // Yaw command [-1..1]   / ヨー指令
    uint8_t source;       // Input source ID       / 入力ソースID
    uint32_t timestamp;   // [us]
};

/// Discrete pilot requests (ARM switch + flight-mode selection), carried SEPARATELY
/// from the continuous CommandSetpoint stick stream. sf_comm decodes the ESP-NOW
/// flags byte and publishes these as FACTS (it does not decide); the StateManager —
/// the sole transition authority — edge-detects ARM and applies the mode (R5: this
/// flows by topic, not a direct call). 離散パイロット要求（ARM スイッチ＋飛行モード選択）。
/// 連続スティックの CommandSetpoint とは分離して運ぶ。sf_comm が ESP-NOW flags を
/// デコードし「事実」として発行（判断しない）。判断は唯一の遷移実行者 StateManager が
/// 行う（R5: 直接呼び出しでなくトピック経由）。
struct PilotRequest {
    bool     arm;         // ARM switch          (flags bit0) / ARM スイッチ
    bool     acro;        // ACRO/rate request   (flags bit2) / ACRO（レート）要求
    bool     alt_hold;    // ALTITUDE_HOLD switch(flags bit3) / 高度保持スイッチ
    bool     pos_hold;    // POSITION_HOLD switch(flags bit4) / 位置保持スイッチ
    uint8_t  source;      // input source ID                 / 入力ソース ID
    uint32_t timestamp;   // [us]; 0 = never published        / 0=未発行
};

// =============================================================================
// Control Data Types
// 制御データ型
// =============================================================================

/// Control output: thrust and torque in body frame
/// 制御出力: 機体座標系での推力とトルク
struct ControlOutput {
    float thrust;         // Total thrust [N]      / 全推力
    float torque[3];      // Torque [Nm] R,P,Y     / トルク
    uint32_t timestamp;   // [us]
};

// =============================================================================
// Actuation Data Types
// アクチュエーションデータ型
// =============================================================================

/// Motor duty output
/// モーターduty出力
struct MotorOutput {
    float duty[4];        // Motor duty [0..1] M1-M4 / モーターduty
    uint32_t timestamp;   // [us]
};

// =============================================================================
// System Data Types
// システムデータ型
// =============================================================================

/// Current flight state and mode
/// 現在のフライト状態とモード
struct SystemMode {
    uint8_t state;        // FlightState enum      / フライト状態
    uint8_t sub_mode;     // FlightMode enum       / フライトモード
    bool armed;           // Armed flag            / ARM状態
    uint32_t timestamp;   // [us]
};

/// System alert from failsafe
/// フェイルセーフからのシステムアラート
struct SystemAlert {
    uint8_t type;         // Alert type enum       / アラート種別
    uint8_t severity;     // Severity level        / 重要度
    uint32_t timestamp;   // [us]
};

/// Boot/system readiness status — published by ImuTask, read by the pre-arm checks
/// in StateManager::requestArm(). Lets the transition authority gate ARM on readiness
/// without reaching into a task-local object across tasks (R16-style: status via topic).
/// 起動/システム準備状態 — ImuTask が発行し、StateManager::requestArm() の ARM 前チェックが
/// 読む。遷移実行者がタスクをまたいで task-local オブジェクトに触れずに ARM を準備状態で
/// ゲートできるようにする（R16 流: 状態はトピック経由）。
struct SystemStatus {
    bool calibrated;      // boot gyro/accel bias calibration is no longer pending
                          // 起動バイアス校正が保留中でない（完了/スキップ/無効/中止）
    uint32_t timestamp;   // [us]
};

}  // namespace sf
