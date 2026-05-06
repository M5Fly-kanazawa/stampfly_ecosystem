# vehicle_new Architecture Design
# vehicle_new アーキテクチャ設計書

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

本文書はvehicle_newのアーキテクチャ設計を定義する。要件定義書（requirements.md）に基づき、以下の5つのサブ工程を記述する。

| サブ工程 | 内容 |
|---------|------|
| 3-1. 責務分割 | 14コンポーネントの定義 |
| 3-2. インターフェース設計 | 軽量Pub-Sub、トピック定義、データ型 |
| 3-3. 状態機械設計 | 状態遷移の実装方針 |
| 3-4. データフロー設計 | データの流れと同期方式 |
| 3-5. タスク設計 | FreeRTOSタスクへのマッピング |

## 2. 責務分割（14コンポーネント）

### レイヤードアーキテクチャ

```
┌─────────────────────────────────────────────────┐
│  Application Layer    タスク: StateTask, etc.     │
├─────────────────────────────────────────────────┤
│  Service Layer        状態管理, 通知, ログ, 通信    │
├─────────────────────────────────────────────────┤
│  Algorithm Layer      状態推定, 制御, フィルタ      │
├─────────────────────────────────────────────────┤
│  HAL Layer            IMU, Motor, LED, ToF, etc.  │
├─────────────────────────────────────────────────┤
│  Hardware             ESP32-S3 peripherals        │
└─────────────────────────────────────────────────┘
```

### コンポーネント一覧

| # | コンポーネント | 責務 | レイヤー |
|---|---|---|---|
| 1 | センシング | センサからデータを読み、トピックに発行 | HAL |
| 2 | 状態推定 | センサデータから姿勢/位置/速度を推定（差替可能） | Algorithm |
| 3 | 状態管理 | モード遷移、ARM許可判定、onExit/onEnterコールバック | Service |
| 4 | フェイルセーフ | 異常検出、system.alert発行（状態管理より上位） | Service |
| 5 | 離着陸マネージャー | 地上/空中判定、TAKEOFF/LANDINGモード統括 | Service |
| 6 | 制御 | セットポイント追従演算（差替可能） | Algorithm |
| 7 | アクチュエーション | ミキサー＋安全チェック＋モーター出力 | HAL |
| 8 | コマンド処理 | 全入力ソース吸収・正規化・調停→セットポイント | Service |
| 9 | 通信 | ESP-NOW/UDP/WiFi/TCP送受信（物理レイヤー） | HAL |
| 10 | ナビゲーター | ウェイポイント、経路計画（将来） | Algorithm |
| 11 | キャリブレーション管理 | バイアス測定・保存・適用（タスクなし、コールバック駆動） | Service |
| 12 | パラメータ | パラメータ保持・変更・永続化 | Service |
| 13 | データロガー | Telemetry/Data Stream/Blackbox | Service |
| 14 | 通知 | LED/ブザーで状態表示（HAL直接操作） | Service |

### 設計原則

- **「検出」と「判断」の分離**: センシングが事実を報告（publish）、状態管理が判断する
- **インターフェース統一**: 制御・状態推定は差替可能な統一インターフェース
- **コールバック集約**: 状態遷移のリセット処理はonExit/onEnterに集約
- **疎結合**: コンポーネント間はPub-Subトピック経由で通信、直接依存しない

### 責務 ↔ ESP-IDF コンポーネント対応表

設計上の14責務は、ESP-IDF のコンポーネント単位に展開すると、以下の対応で実装される。1つの責務がインターフェース層 + 実装層に分かれる場合（差替可能設計のため）や、責務に直接対応しない基盤コンポーネント（Pub-Subフレームワーク、数学ライブラリ）が存在する。

| # | 責務 | ESP-IDF コンポーネント | 備考 |
|---|------|----------------------|------|
| 1 | センシング | `sf_hal_bmi270`, `sf_hal_bmm150`, `sf_hal_bmp280`, `sf_hal_pmw3901`, `sf_hal_vl53l3cx`, `sf_hal_button`, `sf_hal_power` | センサごとに HAL コンポーネント |
| 2 | 状態推定 | `sf_estimator`（インターフェース）, `sf_estimator_eskf`（ESKF実装） | 差替可能設計のため2層 |
| 3 | 状態管理 | `sf_state` | — |
| 4 | フェイルセーフ | `sf_failsafe` | — |
| 5 | 離着陸マネージャー | `sf_takeoff_landing` | — |
| 6 | 制御 | `sf_controller`（インターフェース）, `sf_controller_pid`（PID実装） | 差替可能設計のため2層 |
| 7 | アクチュエーション | `sf_actuator`, `sf_hal_motor` | ロジック層 + ハード層 |
| 8 | コマンド処理 | `sf_command` | — |
| 9 | 通信 | `sf_comm` | — |
| 10 | ナビゲーター | （未実装、将来） | — |
| 11 | キャリブレーション管理 | `sf_calibration` | — |
| 12 | パラメータ | `sf_core` 内に統合（`params.def`） | コア基盤として配置 |
| 13 | データロガー | `sf_logger`, `sf_telemetry` | Blackbox + テレメトリで分離 |
| 14 | 通知 | `sf_notify`, `sf_hal_led`, `sf_hal_buzzer` | ロジック層 + ハード層 |
| — | （責務外）Pub-Sub基盤、データ型、トピック定義、パラメータ基盤 | `sf_core` | 全コンポーネントの基盤 |
| — | （責務外）数学ライブラリ（Vector / Matrix / Quaternion） | `sf_math` | 推定/制御から共有 |

**実装コンポーネント数: 26**（HAL 10 + 責務系 14 + コア基盤 2）

## 3. インターフェース設計

### 通信方式: 軽量Pub-Sub

StampFlyの制約に最適化した独自の軽量設計。

**特徴:**
- 同一MCU内のタスク間通信のみ（プロセス間通信不要）
- シリアライズ不要（構造体のメモリ直接共有）
- 全トピックはコンパイル時に確定（動的生成不要）
- 内部実装はデータフローの特性に応じて使い分け

**内部実装の使い分け:**

| データフロー | 内部方式 | 理由 |
|---|---|---|
| IMU → 状態推定 | Lock-free SPSC Ring Buffer | ISR安全、400Hzロスなし |
| ToF/Flow/Mag/Baro → 状態推定 | FreeRTOS Queue | 低レート、シンプルで十分 |
| 推定値 → 制御 | 共有メモリ + Task Notification | 最新値のみ、最低レイテンシ |
| 全データ → ログ | Lock-free Ring Buffer | 全サンプル保持、ロスなし |
| 推定値/モード → テレメトリ | 共有メモリ（最新値） | 50Hz間引き、古い値でも可 |

**外部インターフェース（統一）:**

```cpp
// Publisher side
// パブリッシャー側
topic.publish(data);

// Subscriber side
// サブスクライバー側
topic.subscribe(callback);
// or
auto data = topic.latest();
```

### トピック一覧

| 発行元 | トピック名 | レート | 購読者 |
|--------|-----------|--------|--------|
| センシング | `sensor.imu` | 400Hz | 状態推定、ログ |
| センシング | `sensor.tof` | 30Hz | 状態推定、離着陸MGR、ログ |
| センシング | `sensor.flow` | 100Hz | 状態推定、ログ |
| センシング | `sensor.mag` | 25Hz | 状態推定、ログ |
| センシング | `sensor.baro` | 50Hz | 状態推定、ログ |
| センシング | `sensor.power` | 10Hz | フェイルセーフ、ログ |
| 状態推定 | `estimate.state` | 400Hz | 制御、テレメトリ、ログ |
| 状態管理 | `system.mode` | イベント | 制御、通知、ログ |
| コマンド処理 | `command.setpoint` | 50Hz | 制御、ログ |
| 制御 | `control.output` | 400Hz | アクチュエーション、ログ |
| アクチュエーション | `actuator.motor` | 400Hz | ログ |
| フェイルセーフ | `system.alert` | イベント | 状態管理、通知 |

- トピック追加はコンパイル時のトピック定義ファイルに1行追加で対応
- 各コンポーネントは内部で加工値トピックを追加発行可能（例: `estimate.imu_filtered`）

### データ型定義

```cpp
// =============================================================
// Sensor topics (raw data only)
// センサトピック（生値のみ — フィルタは購読者側で実施）
// =============================================================

struct ImuData {              // sensor.imu (400Hz)
    float accel[3];           // Accelerometer [m/s²]
    float gyro[3];            // Gyroscope [rad/s]
    float temperature;        // Chip temperature [°C]
    uint32_t timestamp;       // Microseconds [us]
};

struct TofData {              // sensor.tof (30Hz)
    float distance;           // Distance [m]
    uint8_t status;           // Sensor status code
    bool valid;               // Data validity flag
    uint32_t timestamp;       // [us]
};

struct FlowData {             // sensor.flow (100Hz)
    int16_t dx, dy;           // Displacement [counts]
    uint8_t squal;            // Surface quality
    uint32_t timestamp;       // [us]
};

struct MagData {              // sensor.mag (25Hz)
    float mag[3];             // Magnetic field [uT]
    uint32_t timestamp;       // [us]
};

struct BaroData {             // sensor.baro (50Hz)
    float pressure;           // Pressure [Pa]
    float temperature;        // Temperature [°C]
    float altitude;           // Pressure-derived altitude [m]
    uint32_t timestamp;       // [us]
};

struct PowerData {            // sensor.power (10Hz)
    float voltage;            // Battery voltage [V]
    float current;            // Current draw [mA]
    float power;              // Power consumption [mW]
    uint32_t timestamp;       // [us]
};

// =============================================================
// Estimation topics
// 推定トピック
// =============================================================

struct StateEstimate {        // estimate.state (400Hz)
    float attitude[4];        // Quaternion [w,x,y,z]
    float position[3];        // Position [m] NED
    float velocity[3];        // Velocity [m/s] NED
    float gyro_bias[3];       // Gyro bias estimate [rad/s]
    float accel_bias[3];      // Accel bias estimate [m/s²]
    uint8_t sensor_mask;      // Active sensor bitmask
    uint32_t timestamp;       // [us]
};

// =============================================================
// Command topics
// コマンドトピック
// =============================================================

struct CommandSetpoint {      // command.setpoint
    float throttle;           // Throttle [0..1]
    float roll;               // Roll command [-1..1]
    float pitch;              // Pitch command [-1..1]
    float yaw;                // Yaw command [-1..1]
    uint8_t source;           // Input source ID
    uint32_t timestamp;       // [us]
};

// =============================================================
// Control topics
// 制御トピック
// =============================================================

struct ControlOutput {        // control.output (400Hz)
    float thrust;             // Total thrust [N]
    float torque[3];          // Torque [Nm] roll, pitch, yaw
    uint32_t timestamp;       // [us]
};

// =============================================================
// Actuation topics
// アクチュエーショントピック
// =============================================================

struct MotorOutput {          // actuator.motor (400Hz)
    float duty[4];            // Motor duty [0..1] M1-M4
    uint32_t timestamp;       // [us]
};

// =============================================================
// System topics
// システムトピック
// =============================================================

struct SystemMode {           // system.mode (event-driven)
    uint8_t state;            // FlightState enum value
    uint8_t sub_mode;         // FlightMode enum value
    bool armed;               // Armed flag (derived from state)
    uint32_t timestamp;       // [us]
};

struct SystemAlert {          // system.alert (event-driven)
    uint8_t type;             // Alert type enum
    uint8_t severity;         // Severity level
    uint32_t timestamp;       // [us]
};
```

## 4. 状態機械設計

### FAILSAFEの位置づけ

FAILSAFEは**状態ではなくイベント**として設計する。

```
フェイルセーフコンポーネント: 異常を検出 → system.alert を発行
                                              ↓
状態管理コンポーネント: alert を購読 → 判断 → 既存モードへの遷移を実行
                                              ↓
制御/離着陸マネージャー: 遷移先のモードに従って動作
```

| 異常 | フェイルセーフが検出 | 状態管理が判断 | 結果 |
|------|-------------------|--------------|------|
| 通信途絶 | `alert: COMM_LOST` | ホバー維持 → LANDING | 離着陸MGRが自動着陸 |
| 衝撃 | `alert: IMPACT` | → IDLE_GROUND | 即DISARM |
| 低電圧 | `alert: LOW_BATTERY` | 変化なし | 通知がブザー鳴らす |
| USB給電 | `alert: USB_POWER` | ARM禁止 | — |
| ESKF発散 | `alert: ESKF_DIVERGED` | 変化なし | ESKFリセット |

### 状態遷移の実装方針

- 状態管理コンポーネントが唯一の遷移実行者
- 遷移時にonExit(旧状態)/onEnter(新状態)コールバックを実行
- コールバック内でリセット処理を集約（PIDクリア、ESKFリセット等）
- 状態管理タスクの動作不具合はフェイルセーフが検知可能な構造とする

## 5. データフロー設計

### メインパイプライン

```
sensor.imu (400Hz)
    │ Lock-free Ring
    ▼
ImuTask [状態推定]
    │ predict (400Hz) + update (各センサレートで非同期)
    │
    ├── sensor.tof (30Hz)   ← FreeRTOS Queue
    ├── sensor.flow (100Hz) ← FreeRTOS Queue
    ├── sensor.mag (25Hz)   ← FreeRTOS Queue
    └── sensor.baro (50Hz)  ← FreeRTOS Queue
    │
    ▼ estimate.state (共有メモリ + Task Notify)
    │
ControlTask [制御 + アクチュエーション]
    │ command.setpoint を参照
    │ control.output → actuator.motor
    ▼
モーター出力
```

### 同期方式

- **IMU → 推定 → 制御**: IMUタスクからTask Notificationで制御タスクを起床（400Hz同期）
- **他センサ → 推定**: 非同期。データが到着した周期のみ観測更新
- **全データ → ログ**: Lock-free Ring Bufferで全サンプル保持

### コマンドフロー

```
ESP-NOW ──┐
UDP/API ──┤
          ├──→ CommTask [通信 + コマンド処理]
SBUS ─────┘         │
(将来)               ▼ command.setpoint
                     │
              ControlTask [制御]
                     ↑
              ナビゲーター（自律モード時、将来）
```

### システムフロー

```
sensor.power ──→ PowerTask [フェイルセーフ]
                        │ system.alert
                        ▼
                 StateTask [状態管理]
                        │ system.mode
                        ▼
                 NotifyTask [通知] ──→ LED/ブザー（HAL直接操作）
```

### ログフロー

```
全トピック ──→ LogTask [データロガー]
                   ├── Telemetry: UDP送信（50Hz間引き）
                   ├── Data Stream: UDP/USB送信（全レート）
                   └── Blackbox: Flash書き込み（リングバッファ）
```

## 6. タスク設計

### タスク一覧

| タスク | 周期 | 優先度 | スタック | 含むコンポーネント |
|--------|------|--------|---------|-------------------|
| ImuTask | 400Hz | 24 | 16KB | センシング(IMU) + 状態推定 |
| ControlTask | 400Hz(IMU同期) | 23 | 8KB | 制御 + アクチュエーション |
| StateTask | イベント駆動 | 22 | 4KB | 状態管理 |
| FlowTask | 100Hz | 20 | 8KB | センシング(OptFlow) |
| MagTask | 25Hz | 18 | 8KB | センシング(Mag) |
| BaroTask | 50Hz | 16 | 8KB | センシング(Baro) |
| CommTask | 50Hz | 15 | 4KB | 通信 + コマンド処理 |
| TofTask | 30Hz | 14 | 8KB | センシング(ToF) + 離着陸マネージャー |
| TelemetryTask | 50Hz | 13 | 4KB | テレメトリ |
| PowerTask | 10Hz | 12 | 4KB | センシング(Power) + フェイルセーフ |
| ButtonTask | 50Hz | 10 | 4KB | ボタン入力 |
| NotifyTask | 30Hz | 8 | 4KB | 通知(LED/ブザー) |
| CLITask | 20Hz | 5 | 8KB | CLI + パラメータ |
| LogTask | 非同期 | 5 | 4KB | データロガー + Blackbox |

### 統合の理由

| 統合 | 理由 |
|------|------|
| IMU + 状態推定 | 400Hzで密結合、タスク切替コスト削減 |
| 制御 + アクチュエーション | 制御出力を即モーター反映、安全チェックも同タスク |
| 通信 + コマンド処理 | 受信と解釈は密接 |
| ToF + 離着陸マネージャー | ToFが離着陸判定の主入力 |
| Power + フェイルセーフ | 電圧監視が主な異常検出源 |

### 独立タスクの理由

| タスク | 理由 |
|--------|------|
| StateTask | 再構築の核心。モード遷移を一元管理。イベント駆動で即応 |
| LogTask | 記録失敗が制御系に影響してはならない。非同期で独立動作 |
| TelemetryTask | 通信遅延が制御系に波及しない |

### タスク非割当のコンポーネント

| コンポーネント | 方式 | 理由 |
|---|---|---|
| キャリブレーション管理 | 状態管理のonEnterコールバックから呼ばれる | INIT/IDLE時のみ動作、常駐不要 |
| ナビゲーター | 将来追加時に独立タスクとして追加 | 初期実装では構造のみ |

---

<a id="english"></a>

## 1. Overview

This document defines the architecture design of vehicle_new, based on the requirements definition (requirements.md). It covers five sub-phases:

| Sub-phase | Content |
|-----------|---------|
| 3-1. Responsibility Assignment | 14 component definitions |
| 3-2. Interface Design | Lightweight Pub-Sub, topic definitions, data types |
| 3-3. State Machine Design | State transition implementation policy |
| 3-4. Data Flow Design | Data flow and synchronization |
| 3-5. Task Design | FreeRTOS task mapping |

## 2. Responsibility Assignment (14 Components)

### Layered Architecture

```
┌─────────────────────────────────────────────────┐
│  Application Layer    Tasks: StateTask, etc.     │
├─────────────────────────────────────────────────┤
│  Service Layer        State Mgr, Notify, Log     │
├─────────────────────────────────────────────────┤
│  Algorithm Layer      Estimation, Control, Filter│
├─────────────────────────────────────────────────┤
│  HAL Layer            IMU, Motor, LED, ToF, etc. │
├─────────────────────────────────────────────────┤
│  Hardware             ESP32-S3 peripherals       │
└─────────────────────────────────────────────────┘
```

### Component List

| # | Component | Responsibility | Layer |
|---|-----------|---------------|-------|
| 1 | Sensing | Read sensor data, publish to topics | HAL |
| 2 | State Estimation | Estimate attitude/position/velocity (replaceable) | Algorithm |
| 3 | State Management | Mode transitions, ARM permission, onExit/onEnter callbacks | Service |
| 4 | Failsafe | Anomaly detection, system.alert publishing (higher priority) | Service |
| 5 | Takeoff/Landing Mgr | Ground/air detection, TAKEOFF/LANDING orchestration | Service |
| 6 | Control | Setpoint tracking computation (replaceable) | Algorithm |
| 7 | Actuation | Mixer + safety check + motor output | HAL |
| 8 | Command Processing | Absorb all input sources, normalize, arbitrate → setpoint | Service |
| 9 | Communication | ESP-NOW/UDP/WiFi/TCP send/receive (physical layer) | HAL |
| 10 | Navigator | Waypoints, path planning (future) | Algorithm |
| 11 | Calibration Mgr | Bias measurement, storage, application (callback-driven) | Service |
| 12 | Parameters | Parameter storage, modification, persistence | Service |
| 13 | Data Logger | Telemetry/Data Stream/Blackbox | Service |
| 14 | Notification | LED/buzzer state display (direct HAL access) | Service |

### Design Principles

- **Separate detection from decision**: Sensing reports facts (publish), State Management decides
- **Unified interfaces**: Control and State Estimation are replaceable via unified interfaces
- **Callback consolidation**: Reset processing during state transitions consolidated in onExit/onEnter
- **Loose coupling**: Components communicate via Pub-Sub topics, no direct dependencies

### Responsibility ↔ ESP-IDF Component Mapping

The 14 design responsibilities expand to ESP-IDF component granularity as follows. Some responsibilities split into interface + implementation layers (for replaceability), and some infrastructure components (Pub-Sub, math) do not map to a single responsibility.

| # | Responsibility | ESP-IDF Component(s) | Notes |
|---|----------------|---------------------|-------|
| 1 | Sensing | `sf_hal_bmi270`, `sf_hal_bmm150`, `sf_hal_bmp280`, `sf_hal_pmw3901`, `sf_hal_vl53l3cx`, `sf_hal_button`, `sf_hal_power` | One HAL component per sensor |
| 2 | State Estimation | `sf_estimator` (interface), `sf_estimator_eskf` (ESKF impl) | Two layers for replaceability |
| 3 | State Management | `sf_state` | — |
| 4 | Failsafe | `sf_failsafe` | — |
| 5 | Takeoff/Landing Mgr | `sf_takeoff_landing` | — |
| 6 | Control | `sf_controller` (interface), `sf_controller_pid` (PID impl) | Two layers for replaceability |
| 7 | Actuation | `sf_actuator`, `sf_hal_motor` | Logic + hardware |
| 8 | Command Processing | `sf_command` | — |
| 9 | Communication | `sf_comm` | — |
| 10 | Navigator | (not yet implemented) | Future |
| 11 | Calibration Mgr | `sf_calibration` | — |
| 12 | Parameters | folded into `sf_core` (`params.def`) | Located in core infrastructure |
| 13 | Data Logger | `sf_logger`, `sf_telemetry` | Blackbox + telemetry split |
| 14 | Notification | `sf_notify`, `sf_hal_led`, `sf_hal_buzzer` | Logic + hardware |
| — | (infra) Pub-Sub framework, data types, topics, parameter base | `sf_core` | Foundation for all components |
| — | (infra) Math library (Vector / Matrix / Quaternion) | `sf_math` | Shared by estimator/controller |

**Implemented component count: 26** (HAL 10 + responsibility-mapped 14 + infra 2)

## 3. Interface Design

### Communication Method: Lightweight Pub-Sub

Lightweight custom design optimized for StampFly constraints.

**Characteristics:**
- Intra-MCU task communication only (no IPC needed)
- No serialization (direct struct memory sharing)
- All topics determined at compile time (no dynamic creation)
- Internal implementation varies by data flow characteristics

**Internal Implementation Selection:**

| Data Flow | Internal Method | Reason |
|---|---|---|
| IMU → Estimation | Lock-free SPSC Ring Buffer | ISR-safe, zero-loss at 400Hz |
| ToF/Flow/Mag/Baro → Estimation | FreeRTOS Queue | Low rate, simple and sufficient |
| Estimate → Control | Shared memory + Task Notification | Latest value only, minimal latency |
| All data → Logger | Lock-free Ring Buffer | Full sample retention, zero-loss |
| Estimate/Mode → Telemetry | Shared memory (latest value) | 50Hz decimation, stale OK |

### Topic List

| Publisher | Topic | Rate | Subscribers |
|-----------|-------|------|-------------|
| Sensing | `sensor.imu` | 400Hz | Estimation, Logger |
| Sensing | `sensor.tof` | 30Hz | Estimation, TL Manager, Logger |
| Sensing | `sensor.flow` | 100Hz | Estimation, Logger |
| Sensing | `sensor.mag` | 25Hz | Estimation, Logger |
| Sensing | `sensor.baro` | 50Hz | Estimation, Logger |
| Sensing | `sensor.power` | 10Hz | Failsafe, Logger |
| Estimation | `estimate.state` | 400Hz | Control, Telemetry, Logger |
| State Mgr | `system.mode` | Event | Control, Notification, Logger |
| Command | `command.setpoint` | 50Hz | Control, Logger |
| Control | `control.output` | 400Hz | Actuation, Logger |
| Actuation | `actuator.motor` | 400Hz | Logger |
| Failsafe | `system.alert` | Event | State Mgr, Notification |

- Adding topics requires only one line in the compile-time topic definition file
- Components can publish additional processed-data topics (e.g., `estimate.imu_filtered`)

### Data Type Definitions

See Section 3 of the Japanese version for complete struct definitions.

## 4. State Machine Design

### FAILSAFE as Event

FAILSAFE is designed as an **event, not a state**.

```
Failsafe component: detect anomaly → publish system.alert
                                          ↓
State Management: subscribe alert → decide → execute transition to existing mode
                                          ↓
Control / TL Manager: operate according to target mode
```

| Anomaly | Failsafe Detects | State Mgr Decides | Result |
|---------|-----------------|-------------------|--------|
| Comm loss | `alert: COMM_LOST` | Hover hold → LANDING | TL Mgr auto-lands |
| Impact | `alert: IMPACT` | → IDLE_GROUND | Immediate DISARM |
| Low battery | `alert: LOW_BATTERY` | No change | Notification buzzer |
| USB power | `alert: USB_POWER` | ARM prohibited | — |
| ESKF divergence | `alert: ESKF_DIVERGED` | No change | ESKF reset |

## 5. Data Flow Design

### Main Pipeline

```
sensor.imu (400Hz)
    │ Lock-free Ring
    ▼
ImuTask [Estimation]
    │ predict (400Hz) + update (async per sensor rate)
    │
    ├── sensor.tof (30Hz)   ← FreeRTOS Queue
    ├── sensor.flow (100Hz) ← FreeRTOS Queue
    ├── sensor.mag (25Hz)   ← FreeRTOS Queue
    └── sensor.baro (50Hz)  ← FreeRTOS Queue
    │
    ▼ estimate.state (Shared memory + Task Notify)
    │
ControlTask [Control + Actuation]
    │ References command.setpoint
    │ control.output → actuator.motor
    ▼
Motor output
```

### Synchronization

- **IMU → Estimation → Control**: Task Notification from IMU task wakes Control task (400Hz sync)
- **Other sensors → Estimation**: Asynchronous. Observation update only when data arrives
- **All data → Logger**: Lock-free Ring Buffer retains all samples

## 6. Task Design

### Task List

| Task | Rate | Priority | Stack | Components |
|------|------|----------|-------|------------|
| ImuTask | 400Hz | 24 | 16KB | Sensing(IMU) + Estimation |
| ControlTask | 400Hz(IMU sync) | 23 | 8KB | Control + Actuation |
| StateTask | Event-driven | 22 | 4KB | State Management |
| FlowTask | 100Hz | 20 | 8KB | Sensing(OptFlow) |
| MagTask | 25Hz | 18 | 8KB | Sensing(Mag) |
| BaroTask | 50Hz | 16 | 8KB | Sensing(Baro) |
| CommTask | 50Hz | 15 | 4KB | Communication + Command Processing |
| TofTask | 30Hz | 14 | 8KB | Sensing(ToF) + TL Manager |
| TelemetryTask | 50Hz | 13 | 4KB | Telemetry |
| PowerTask | 10Hz | 12 | 4KB | Sensing(Power) + Failsafe |
| ButtonTask | 50Hz | 10 | 4KB | Button input |
| NotifyTask | 30Hz | 8 | 4KB | Notification(LED/Buzzer) |
| CLITask | 20Hz | 5 | 8KB | CLI + Parameters |
| LogTask | Async | 5 | 4KB | Data Logger + Blackbox |

### Components Without Dedicated Tasks

| Component | Method | Reason |
|-----------|--------|--------|
| Calibration Mgr | Called from State Mgr onEnter callback | Active only during INIT/IDLE |
| Navigator | Future: add as independent task | Structure only in initial implementation |
