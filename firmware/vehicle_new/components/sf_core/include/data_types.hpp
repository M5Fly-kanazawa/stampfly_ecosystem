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
 * @design architecture.md §3 — Topic data types                       [OK]
 * @design detailed_design.md §2 — Data type definitions               [OK]
 * @design coding_and_education.md §2 — Bilingual comments             [OK]
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
    bool  calibrated;     // Hard/soft-iron correction applied (MagTask) / 校正適用済み
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

/// Latest raw async-sensor readings, mirrored by ImuTask (the SPSC queue consumer)
/// into a Latest topic so monitors (CLI `sensor`, telemetry) can peek WITHOUT stealing
/// from the sensor_tof/flow/mag/baro queues (single-consumer SPSC). Updated each cycle.
/// 非同期センサの最新生値。ImuTask（SPSC キューの単一 consumer）が Latest トピックへミラーし、
/// 監視側（CLI `sensor`・テレメトリ）が sensor_tof/flow/mag/baro キューから奪わず覗けるように
/// する。毎サイクル更新。
struct SensorSnapshot {
    float    mag[3];          // Magnetometer [uT]        / 地磁気
    float    baro_pressure;   // Barometer pressure [Pa]  / 気圧
    float    baro_altitude;   // Pressure altitude [m]    / 気圧高度
    float    tof_distance;    // ToF distance [m]         / ToF 距離
    uint8_t  tof_status;      // ToF status code          / ToF ステータス
    bool     tof_valid;       // ToF validity             / ToF 有効
    int16_t  flow_dx;         // Optical flow dx [counts] / フロー dx
    int16_t  flow_dy;         // Optical flow dy [counts] / フロー dy
    uint8_t  flow_squal;      // Flow surface quality     / フロー品質
    // Per-sensor sample timestamps (the producing sensor's own stamp). The Data
    // Stream uses a CHANGE in these to detect "new sample" per sensor — the
    // shared snapshot timestamp below cannot tell which sensor updated.
    // センサ別サンプル時刻（各センサ自身の刻印）。Data Stream はこの「変化」で
    // センサ毎の新サンプルを検出する — 下の共有時刻ではどのセンサが更新したか
    // 分からない。
    uint32_t mag_timestamp;   // [us] last mag sample   / 最終 mag サンプル時刻
    uint32_t baro_timestamp;  // [us] last baro sample  / 最終 baro サンプル時刻
    uint32_t tof_timestamp;   // [us] last ToF sample   / 最終 ToF サンプル時刻
    uint32_t flow_timestamp;  // [us] last flow sample  / 最終 flow サンプル時刻
    uint32_t timestamp;       // [us] snapshot publish time / スナップショット発行時刻
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

/// Raw pilot input as delivered by the comm link, BEFORE normalization.
/// sf_comm (HAL, responsibility #9) latches the decoded wire packet here as a
/// FACT and does not interpret it; sf_command (Service, responsibility #8) reads
/// it and produces the normalized CommandSetpoint + PilotRequest. This keeps the
/// physical-layer (comm) and the command-processing layer (command) decoupled.
/// 正規化前の生パイロット入力。sf_comm（HAL・責務#9）が復号した電波パケットを「事実」
/// として保持するだけで解釈しない。sf_command（Service・責務#8）がこれを読み、正規化済み
/// CommandSetpoint と PilotRequest を生成する。物理層（comm）とコマンド処理層（command）
/// を疎結合に保つ。
struct RawControlInput {
    uint16_t throttle;    // Raw 12-bit ADC [0..4095], 2048 = zero  / 生スロットル
    uint16_t roll;        // Raw 12-bit ADC [0..4095], 2048 = centre/ 生ロール
    uint16_t pitch;       // Raw 12-bit ADC [0..4095], 2048 = centre/ 生ピッチ
    uint16_t yaw;         // Raw 12-bit ADC [0..4095], 2048 = centre/ 生ヨー
    uint8_t  flags;       // Discrete switch bits (protocol SSOT)   / スイッチビット
    uint8_t  source;      // Input source ID (CommandSource)        / 入力ソースID
    uint32_t timestamp;   // [us]; 0 = never                        / 0=未受信
};

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
/// from the continuous CommandSetpoint stick stream. sf_command decodes the flags
/// byte (delivered raw by sf_comm) and publishes these as FACTS (it does not decide);
/// the StateManager — the sole transition authority — edge-detects ARM and applies
/// the mode (R5: this flows by topic, not a direct call). 離散パイロット要求（ARM
/// スイッチ＋飛行モード選択）。連続スティックの CommandSetpoint とは分離して運ぶ。
/// sf_command が（sf_comm が生で渡した）flags をデコードし「事実」として発行（判断しない）。
/// 判断は唯一の遷移実行者 StateManager が行う（R5: 直接呼び出しでなくトピック経由）。
struct PilotRequest {
    bool     arm;         // ARM switch          (flags bit0) / ARM スイッチ
    bool     acro;        // ACRO/rate request   (flags bit2) / ACRO（レート）要求
    bool     alt_hold;    // ALTITUDE_HOLD switch(flags bit3) / 高度保持スイッチ
    bool     pos_hold;    // POSITION_HOLD switch(flags bit4) / 位置保持スイッチ
    uint8_t  source;      // input source ID                 / 入力ソース ID
    uint32_t timestamp;   // [us]; 0 = never published        / 0=未発行
};

// =============================================================================
// Input Event Data Types
// 入力イベントデータ型
// =============================================================================

/// Button gesture detected by ButtonTask at the GPIO. ButtonTask reports the gesture as
/// a FACT (it does not decide the action); the StateManager — the sole transition
/// authority — maps a click to an ARM/DISARM toggle (architecture §2: detection reports,
/// state management decides; R5: by topic, not a direct call). The 3 s / 5 s long-press
/// gestures are reserved for pairing / system-reset handling.
/// ButtonTask が GPIO で検出したボタンジェスチャ。ButtonTask はジェスチャを「事実」として
/// 報告するだけで動作を決めない。判断は唯一の遷移実行者 StateManager が行い、クリックを
/// ARM/DISARM トグルに対応づける（architecture §2: 検出は報告・判断は状態管理、R5: 直接
/// 呼び出しでなくトピック経由）。3秒/5秒の長押しはペアリング/システムリセット用に予約。
enum class ButtonGesture : uint8_t {
    None        = 0,   // no gesture                   / ジェスチャなし
    Click       = 1,   // short press (<1 s)           / 短押し
    DoubleClick = 2,   // two quick clicks             / ダブルクリック
    LongPress3s = 3,   // held 3 s (clear pairing)     / 長押し3秒
    LongPress5s = 4,   // held 5 s (system reset)      / 長押し5秒
};

/// Button event — ButtonTask publishes the detected gesture; StateTask consumes it.
/// ボタンイベント — ButtonTask が検出ジェスチャを発行し、StateTask が消費する。
struct ButtonEvent {
    uint8_t  gesture;     // ButtonGesture value         / ButtonGesture の値
    uint32_t timestamp;   // [us]; 0 = never published   / 0=未発行
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
    // Cascade setpoints, exported for the Data Stream (analysis/identification:
    // the rate-loop reference at the control rate is what closes the loop on paper).
    // カスケードの目標値。Data Stream 用に公開（解析・同定では制御周期のレート目標が
    // ループを机上で閉じるのに必要）。
    float rate_ref[3];    // Inner-loop rate setpoint [rad/s] R,P,Y / 内側ループ角速度目標
    float angle_ref[2];   // Outer-loop tilt setpoint [rad] R,P (0 in ACRO) / 外側ループ傾き目標
    uint32_t timestamp;   // [us]
};

// =============================================================================
// Data Stream (requirements §7: analysis log at the CONTROL RATE)
// データストリーム（要件§7: 制御周期の解析用ログ）
// =============================================================================

/// One consolidated 400Hz record for the Data Stream. ControlTask builds and
/// publishes one per control cycle (it is the only place where the same cycle's
/// IMU sample, estimate, and control setpoints are all in hand); the
/// TelemetryTask drains the ring at 50Hz and batches 8 records per UDP packet
/// (same wire format and bandwidth design as the proven firmware/vehicle:
/// 50 sendto/s ≈ 48 KB/s).
/// Data Stream 用の 400Hz 統合レコード。ControlTask が制御周期毎に 1 件組んで発行
/// する（同一周期の IMU サンプル・推定値・制御目標が全て手元に揃う唯一の場所）。
/// TelemetryTask が 50Hz でリングを排出し UDP 1 パケットに 8 件をバッチする
/// （実証済み firmware/vehicle と同じ電文・帯域設計: 50 sendto/s ≈ 48KB/s）。
struct LogStreamSample {
    uint32_t timestamp;     // [us] IMU sample time          / IMU サンプル時刻
    float gyro[3];          // [rad/s] raw gyro, body FRD    / 生ジャイロ
    float accel[3];         // [m/s²] raw accel, body FRD    / 生加速度
    float quat[4];          // estimator attitude [w,x,y,z]  / 推定姿勢
    float gyro_bias[3];     // [rad/s] estimator gyro bias   / ジャイロバイアス推定
    float accel_bias[3];    // [m/s²] estimator accel bias   / 加速度バイアス推定
    float pos[3];           // [m] NED position estimate     / 位置推定
    float vel[3];           // [m/s] NED velocity estimate   / 速度推定
    float rate_ref[3];      // [rad/s] inner-loop setpoint   / 内側ループ角速度目標
    float angle_ref[2];     // [rad] tilt setpoint R,P       / 傾き目標
    float thrust;           // [N] commanded total thrust    / 指令総推力
    float duty[4];          // motor duty FR,RR,RL,FL        / モータ duty
    uint8_t flight_mode;    // FlightMode value              / フライトモード
    uint8_t flight_state;   // FlightState value             / フライト状態
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
// Pairing — transmitter↔vehicle binding (crosstalk prevention)
// ペアリング — 送信機↔機体のバインド（混信対策）
//
// PairingState is a SEPARATE state machine, parallel to FlightState, single-owned
// by the StateManager (architecture.md §4). Two topics carry the split of duties:
//   - pairing_state    (StateManager → comm/notify): the decided PairingState.
//   - pairing_complete (comm → StateManager): comm's current bind status fact.
// The handshake follows the legacy vehicle: the vehicle broadcasts a PairingPacket
// advertising its own MAC; the controller learns it and unicasts ControlPackets to
// that MAC; the vehicle fixes the received src MAC as its peer (mutual MAC learning).
//
// PairingState は FlightState と並行する別の状態機械で、StateManager が単一所有する
// （architecture.md §4）。責務分担を2トピックで運ぶ:
//   - pairing_state    (StateManager → comm/notify): 決定された PairingState。
//   - pairing_complete (comm → StateManager): comm の現在のバインド状態という事実。
// ハンドシェイクは旧 vehicle 踏襲: 機体が自 MAC を広告する PairingPacket を broadcast し、
// コントローラがそれを学習して機体 MAC へ ControlPacket をユニキャスト送信、機体は受信した
// src MAC を相手として確定する（相互 MAC 学習）。
//
// @design requirements.md §2 — PairingState                            [OK]
// @design architecture.md §4 — Pairing positioning (parallel to FSM)   [OK]
// @design detailed_design.md §3 — Pairing state transitions            [OK]
// =============================================================================

/// Pairing state — parallel to FlightState, owned by StateManager
/// ペアリング状態 — FlightState と並行、StateManager が所有
enum class PairingState : uint8_t {
    NotPaired = 0,   // no bound controller (nothing in NVS)  / 未ペア（NVS保存なし）
    Pairing   = 1,   // searching; broadcasting PairingPacket / 探索中（PairingPacket送出）
    Paired    = 2,   // peer MAC fixed (saved in NVS)         / ペア確定（NVS保存済）
};

/// Pairing status — StateManager publishes the decided PairingState; comm reads it
/// to gate PairingPacket broadcast, notify reads it to drive the pairing LED/buzzer.
/// ペアリング状態 — StateManager が決定した PairingState を発行。comm は PairingPacket 送出の
/// ゲートに、notify は LED/ブザー駆動に読む。
struct PairingStatus {
    uint8_t  state;       // PairingState value            / PairingState の値
    uint32_t timestamp;   // [us]; 0 = never published     / 0=未発行
};

/// Pairing bind status — comm's fact about whether it is bound to a controller and
/// to which MAC. Published at boot (NVS restore: restored=true) and on live pairing
/// completion (restored=false), and again with bound=false when the bind is cleared
/// (re-pair). StateManager reads it to decide Paired vs (Pairing/NotPaired).
/// ペアリングのバインド状態 — comm が「相手にバインド済みか・どの MAC か」を報告する事実。
/// 起動時（NVS 復元: restored=true）と Pairing 成立時（restored=false）に発行し、バインド解除
/// （再ペア）時は bound=false で再発行する。StateManager は Paired か（Pairing/NotPaired）かの
/// 判断に読む。
struct PairingComplete {
    uint8_t  controller_mac[6];  // learned/restored transmitter MAC / 学習・復元した送信機MAC
    bool     bound;              // true: bound to a controller       / 相手にバインド済み
    bool     restored;           // true: restored from NVS at boot   / 起動時のNVS復元
    uint32_t timestamp;          // [us]; 0 = never published         / 0=未発行
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
    ReloadParams = 7,   // re-read eskf.* params, keep x and P / eskf.* を読み直す（x・P は保持）
                        // (live tuning via the param-set callback)
                        // （param set コールバック経由のライブチューニング）
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
    Landing     = 4,   // autonomous landing: level attitude + fixed descent,
                       // sticks ignored (comm loss / battery emergency);
                       // cleared by the next Reset (ARM)
                       // 自動着陸: 水平姿勢+固定降下率でスティック無視
                       // （通信断/電池緊急）。次の Reset（ARM）で解除
    ReloadParams = 5,  // re-read gains/limits from params, keep integrators
                       // (live tuning via the param-set callback)
                       // ゲイン/リミットを params から読み直す。積分器は維持
                       // （param set コールバック経由のライブチューニング）
    Takeoff      = 6,  // auto-takeoff start (TAKEOFF entry in ALT/POS mode):
                       // level attitude + fixed climb rate until airborne
                       // 自動離陸開始（ALT/POS モードの TAKEOFF 突入）:
                       // 水平姿勢＋固定上昇率で空中検知まで
    TakeoffComplete = 7, // takeoff finished (FLYING entry): engage the normal
                       // mode law (ALT_HOLD captures the current altitude)
                       // 離陸完了（FLYING 突入）: 通常のモード則を係合
                       // （ALT_HOLD は現在高度を捕捉）
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
    PairingMode = 6,   // pairing started (one-shot tone)/ ペアリング開始（単発音）
};

/// Notify command — NotifyTask consumes and drives LED/buzzer (HAL direct)
/// 通知コマンド — NotifyTask が消費し LED/ブザーを駆動（HAL 直接）
struct NotifyCommand {
    uint8_t  event;       // NotifyEvent value / NotifyEvent の値
    uint32_t timestamp;   // [us]
};

// =============================================================================
// Bench / UI commands — CLI → owning task (topic-based, so a future WiFi/UDP path
// can inject the same commands). 将来 WiFi/UDP からも同じコマンドを注入できるよう
// トピック経由にする（CLI が直接 HAL/状態に触れない）。
// =============================================================================

/// UI command verbs — CLI → NotifyTask (applies to the buzzer/LED HAL, with NVS save).
/// UI コマンドの種別 — CLI → NotifyTask（buzzer/LED HAL に適用、NVS 保存）。
enum class UiCmd : uint8_t {
    None          = 0,
    SoundMute     = 1,   // value: 1 = mute, 0 = unmute   / 1=ミュート, 0=解除
    LedBrightness = 2,   // value: 0..255 brightness       / LED 輝度
};

/// UI command — NotifyTask consumes and applies to the buzzer/LED HAL (+ NVS).
/// UI コマンド — NotifyTask が消費し buzzer/LED HAL に適用（+ NVS）。
struct UiCommand {
    uint8_t  command;     // UiCmd value                   / UiCmd の値
    uint8_t  value;       // argument (mute flag / brightness) / 引数
    uint32_t timestamp;   // [us]
};

/// Motor test command — CLI → ControlTask. Spins one/all motors at a fixed duty WHILE
/// DISARMED ONLY (bench wiring/direction check). Auto-expires at expiry_us; default
/// inactive. ControlTask ignores it entirely when the vehicle is armed (safety).
/// モータテストコマンド — CLI → ControlTask。**disarmed のときのみ**1/全モータを固定 duty で
/// 回す（配線/回転方向確認）。expiry_us で自動失効、既定 inactive。armed 時は ControlTask が
/// 完全に無視する（安全）。
/// Magnetometer-calibration verb (CLI `magcal` → MagTask, the calibrator owner).
/// Same WHEN/HOW split as the controller verbs: the CLI decides WHEN, MagTask
/// (which owns the BMM150 and the sample stream) executes HOW.
/// 地磁気校正 verb（CLI `magcal` → 校正器を所有する MagTask）。コントローラ verb と
/// 同じ「いつ/どう」分担: CLI が「いつ」を決め、BMM150 とサンプル列を所有する
/// MagTask が「どう」を実行する。
enum class MagCalCmd : uint8_t {
    None  = 0,
    Start = 1,   // begin sample collection (rotate the craft) / 収集開始（機体を回す）
    Stop  = 2,   // stop collection and COMPUTE the fit         / 収集終了し計算
    Save  = 3,   // persist to NVS (disarmed only)              / NVS 保存（disarmed のみ）
    Clear = 4,   // clear NVS + revert to uncalibrated          / NVS 消去・未校正へ
};

/// CLI → MagTask command / CLI → MagTask コマンド
struct MagCalCommand {
    uint8_t  command;     // MagCalCmd value / MagCalCmd の値
    uint32_t timestamp;   // [us]
};

/// MagTask → CLI calibration status (Latest; CLI polls while the user rotates
/// the craft). Mirrors MagCalibrator state + result so the CLI never touches
/// the calibrator object across tasks (R5).
/// MagTask → CLI 校正状態（Latest。ユーザーが機体を回す間 CLI がポーリング）。
/// MagCalibrator の状態と結果を写し、CLI がタスク跨ぎで校正器オブジェクトに
/// 触れないようにする（R5）。
struct MagCalStatus {
    uint8_t  state;          // MagCalibrator::State value / 校正器状態
    uint16_t sample_count;   // collected samples           / 収集サンプル数
    bool     valid;          // current calibration valid   / 現校正の有効性
    float    offset[3];      // hard-iron offset [uT]       / ハードアイアン
    float    scale[3];       // soft-iron scale             / ソフトアイアン
    float    fitness;        // sphere-fit quality 0-1      / 球面フィット品質
    uint32_t timestamp;      // [us]
};

struct MotorTest {
    bool     active;      // test running                  / テスト実行中
    uint8_t  motor_id;    // 0..3 = one motor, 0xFF = all  / 0..3=単体, 0xFF=全部
    float    duty;        // [0,1] PWM duty                / PWM duty
    uint32_t expiry_us;   // auto-stop time [us]; ignored after / 自動停止時刻
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

/// Discrete flight verbs from the network API (ApiTask → StateTask). The API
/// publishes FACTS ("the user asked to take off"); the StateManager — the sole
/// transition authority — validates and executes, exactly like pilot/button
/// input (architecture §2: detection reports, state management decides).
/// ネットワーク API の離散飛行 verb（ApiTask → StateTask）。API は「事実」を発行
/// （「離陸が要求された」）し、唯一の遷移権限である StateManager が検証・実行する。
/// パイロット/ボタン入力と同一の構図（architecture §2）。
enum class ApiCmd : uint8_t {
    None      = 0,
    Arm       = 1,   // arm motors (pre-arm gates still apply) / ARM（事前ゲートは有効）
    Disarm    = 2,   // disarm (ground)                        / DISARM（地上）
    Takeoff   = 3,   // mode→POS_HOLD + arm + auto-takeoff      / モード設定+ARM+自動離陸
    Land      = 4,   // autonomous landing                      / 自動着陸
    Emergency = 5,   // immediate motor cut (any state)         / 即時モータ停止
};

/// ApiTask → StateTask command / ApiTask → StateTask コマンド
struct ApiCommand {
    uint8_t  command;     // ApiCmd value / ApiCmd の値
    uint8_t  mode;        // FlightMode for Takeoff / Takeoff 用 FlightMode
    uint32_t timestamp;   // [us]
};

/// Rate-loop system-identification excitation (API `sysid` → controller). The
/// signal is ADDED to one axis' rate setpoint while the hover loops keep
/// flying (PX4-autotune style): the chirp/doublet band sits above the outer
/// loops' bandwidth, so the rate loop sees broadband excitation and the craft
/// stays in place. Logged in the Data Stream as rate_ref (post-injection) +
/// gyro at 400 Hz — exactly the u/y pair `sf sysid rate-fit` needs.
/// レートループのシステム同定励振（API `sysid` → 制御器）。ホバーループが飛行を
/// 維持したまま、1軸のレート目標に信号を「加算」する（PX4 autotune 流）: チャープ/
/// ダブレットの帯域は外側ループ帯域より上にあり、レートループだけが広帯域励振を
/// 受けて機体はその場に留まる。Data Stream に rate_ref（注入後）＋ジャイロが 400Hz
/// で記録される — `sf sysid rate-fit` が必要とする u/y 対そのもの。
struct SysidCommand {
    uint8_t  axis;        // 0=roll, 1=pitch, 2=yaw
    uint8_t  waveform;    // 0=doublet train, 1=log chirp / 0=ダブレット列, 1=対数チャープ
    float    amplitude;   // [rad/s] (clamped by the controller) / 振幅（制御器でクランプ）
    float    duration;    // [s]
    uint32_t timestamp;   // [us]
};

/// command_target — Guidance/API → Controller position+yaw target (R11 contract).
/// The controller SEEKS the position at `speed` (the setpoint walks toward the
/// target, the POS_HOLD cascade tracks the walking setpoint) and turns to `yaw`
/// with a rate-limited P loop. Published by the Tello-style API (ApiTask) and,
/// in the future, the Navigator.
/// command_target — Guidance/API → 制御器への位置+yaw目標（R11 契約）。制御器は
/// `speed` で位置目標へシーク（設定点が目標へ歩き、POS_HOLD カスケードが歩く設定点を
/// 追従）し、yaw はレート制限付き P ループで向く。Tello 風 API（ApiTask）と将来の
/// Navigator が発行する。
struct GuidanceTarget {
    float    position[3];   // target position [m] NED / 目標位置
    float    yaw;           // target yaw [rad]        / 目標ヨー
    float    speed;         // approach speed [m/s]    / 接近速度
    uint8_t  mode;          // 0=none, 1=track position+yaw / 0=なし, 1=位置+yaw追従
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
