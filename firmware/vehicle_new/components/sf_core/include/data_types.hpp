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
    bool airborne;        // ToF-based airborne detection (TakeoffLandingMgr): off the
                          // ground. Drives TAKEOFF → FLYING. / ToF 離陸検出: 空中。
    bool held;            // ToF-based held-in-hand detection (disarmed + ToF valid above
                          // the airborne threshold). Drives IDLE_GROUND ↔ IDLE_HELD.
                          // ToF 手持ち検出（disarmed＋ToF有効＋空中閾値超）。IDLE地上↔手持ち。
    bool landing;         // landing-complete detection (low altitude + low vertical
                          // velocity sustained). Drives LANDING → IDLE_GROUND.
                          // 着陸完了検出（低高度＋低鉛直速度の持続）。LANDING→IDLE地上。
    uint32_t timestamp;   // [us]
};

// =============================================================================
// Transition Command Topics — reset commands issued by StateManager callbacks
// 遷移コマンドトピック — StateManager コールバックが発行するリセット指令
//
// architecture.md §4 mandates that reset processing be consolidated in the
// state-machine onExit/onEnter callbacks, and R5 mandates that cross-component
// communication go through Pub-Sub topics only. So a transition callback does
// not reach into the estimator/controller directly; it publishes a command and
// the owning task (ImuTask owns the estimator, ControlTask owns the controller)
// consumes it. This replaces the previous ad-hoc edge detection inside those tasks.
//
// architecture.md §4 は「リセット処理を状態機械の onExit/onEnter コールバックに集約」、
// R5 は「コンポーネント間通信は Pub-Sub トピックのみ」を要求する。よって遷移コールバックは
// 推定器/制御器を直接叩かず、コマンドを publish し、所有タスク（ImuTask=推定器、
// ControlTask=制御器）が消費して実行する。これが旧来のタスク内エッジ検出を置き換える。
// =============================================================================

/// Estimator command verbs — issued by StateManager transition callbacks
/// 推定器コマンドの種別 — StateManager の遷移コールバックが発行
enum class EstimatorCmd : uint8_t {
    None         = 0,   // no-op (default-constructed)        / 無操作
    Reset        = 1,   // full state reset (ARM, re-fly)     / 全状態リセット
    ResetPosVel  = 2,   // position/velocity reset (takeoff)  / 位置・速度のみリセット
    FreezeBias   = 3,   // freeze bias estimation (on ground) / バイアス推定を凍結
    UnfreezeBias = 4,   // unfreeze bias estimation (takeoff) / バイアス凍結を解除
    Recalibrate  = 5,   // restart boot bias calibration      / 起動バイアス校正を再実行
    InflateCov   = 6,   // inflate covariance, keep state x   / 共分散だけ膨張（推定値は保持）
};

/// Covariance-inflation scope for EstimatorCmd::InflateCov. "Inflate" means re-set the
/// selected states' covariance diagonal to its initial (uncertain) value and zero their
/// cross-covariance, WITHOUT changing the state estimate x — i.e. "I no longer trust how
/// confident I am about these states, but my best guess stands." Used by the ground→flight
/// reset-timing sweep to declare that the on-ground convergence is not flight-representative
/// without the destabilizing full-reset of the state.
/// EstimatorCmd::InflateCov の膨張対象。"膨張"=選んだ状態の共分散対角を初期(不確か)値に
/// 戻しクロス共分散をゼロ化するが、状態推定値 x は変えない=「これらの状態の自信は捨てるが
/// 最良推定は据え置く」。地上→飛行のリセットタイミングsweepで、地上収束が飛行を代表しない
/// ことを、状態の全リセット(不安定化)なしに宣言するために使う。
enum class CovScope : uint16_t {
    All        = 0,   // all 15 states                         / 全15状態
    PosVel     = 1,   // position + velocity                   / 位置+速度
    PosVelBias = 2,   // position + velocity + biases (keep att)/ 位置+速度+バイアス(姿勢据置)
    Attitude   = 3,   // attitude only (culprit isolation)     / 姿勢のみ(原因切り分け)
};

/// Estimator command — ImuTask consumes and applies to the active IEstimator.
/// `arg` carries a CovScope for InflateCov; unused (0) for the other verbs.
/// 推定器コマンド — ImuTask が消費しアクティブな IEstimator に適用。`arg` は InflateCov の
/// CovScope を運ぶ。他の verb では未使用(0)。
struct EstimatorCommand {
    uint8_t  command;     // EstimatorCmd value / EstimatorCmd の値
    uint32_t timestamp;   // [us]
    uint16_t arg;         // CovScope for InflateCov (else 0) / InflateCov の CovScope
};

/// Controller command verbs — issued by StateManager transition callbacks
/// 制御器コマンドの種別 — StateManager の遷移コールバックが発行
enum class ControllerCmd : uint8_t {
    None        = 0,   // no-op                                        / 無操作
    Reset       = 1,   // reset all integrators/filters (ARM)          / 全積分器・フィルタリセット
    ResetAltPos = 2,   // reset altitude/position loops (mode exit)    / 高度・位置ループのみリセット
    ModeChange  = 3,   // flight-mode switch: reconfigure + reset loops / モード切替: 再構成+ループreset
};

/// Controller command — ControlTask consumes and applies to the active IController
/// 制御器コマンド — ControlTask が消費しアクティブな IController に適用
struct ControllerCommand {
    uint8_t  command;     // ControllerCmd value                              / ControllerCmd の値
    uint8_t  mode;        // FlightMode value (valid when command==ModeChange) / モード値
    uint32_t timestamp;   // [us]
};

/// Notification events — issued by StateManager/Failsafe, consumed by NotifyTask
/// 通知イベント — StateManager/Failsafe が発行し NotifyTask が消費
enum class NotifyEvent : uint8_t {
    None        = 0,   // no-op                       / 無操作
    ArmTone     = 1,   // arm confirmation tone        / ARM 確認音
    DisarmTone  = 2,   // disarm tone                  / DISARM 音
    LowBattery  = 3,   // low-battery warning          / 低電圧警告
    Calibrating = 4,   // calibration in progress      / 校正中
    Ready       = 5,   // ready to arm                 / ARM 可能
};

/// Notify command — NotifyTask consumes and drives LED/buzzer (HAL direct)
/// 通知コマンド — NotifyTask が消費し LED/ブザーを駆動（HAL 直接）
struct NotifyCommand {
    uint8_t  event;       // NotifyEvent value / NotifyEvent の値
    uint32_t timestamp;   // [us]
};

// =============================================================================
// Sensor Health (R15) — published ~1Hz by sf_board
// センサ健全性（R15）— sf_board が約1Hz発行
// =============================================================================

/// Sensor identity index for SensorHealth bitmasks / SensorHealth ビットマスク用の識別
enum class SensorId : uint8_t {
    Imu = 0, Mag = 1, Baro = 2, Tof = 3, Flow = 4, Power = 5, Count = 6,
};

/// Sensor health — presence + freshness per sensor for failsafe/telemetry monitoring
/// センサ健全性 — フェイルセーフ/テレメトリ監視用のセンサ毎 presence と鮮度
struct SensorHealth {
    uint8_t  present_mask;   // bit per SensorId: hardware present     / センサ存在
    uint8_t  healthy_mask;   // bit per SensorId: producing fresh data / 鮮度OK
    uint32_t last_update_us[static_cast<int>(SensorId::Count)];  // last sample time / 最終更新
    uint32_t timestamp;      // [us]
};

// =============================================================================
// Reserved Topics — Guidance / Navigation (R11, detailed_design §9)
// 予約トピック — ガイダンス/ナビゲーション（R11、実装は将来）
//
// Placeholders so the topic location and I/O contract are fixed now (R11).
// 置き場所と入出力契約を今のうちに確定するためのプレースホルダ（R11）。
// =============================================================================

/// command_target — RESERVED (M4+). Guidance/Navigator → Controller position+yaw target.
/// 予約（M4+）。Guidance/Navigator → 制御器への位置+yaw目標。
struct GuidanceTarget {
    float    position[3];   // target position [m] NED / 目標位置
    float    yaw;           // target yaw [rad]        / 目標ヨー
    uint8_t  mode;          // guidance mode           / ガイダンスモード
    uint32_t timestamp;     // [us]
};

/// nav_path — RESERVED (Phase 6). Waypoint sequence from the Navigator.
/// 予約（Phase 6）。ナビゲータからの経路シーケンス。
struct NavigationPath {
    static constexpr int MAX_WAYPOINTS = 8;
    float    waypoints[MAX_WAYPOINTS][3];   // [m] NED / ウェイポイント
    uint8_t  count;                         // active waypoint count / 有効ウェイポイント数
    uint32_t timestamp;                     // [us]
};

}  // namespace sf
