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
┌─────────────────────────────────────────────────────┐
│  Application Layer    タスク: StateTask, etc.         │
├─────────────────────────────────────────────────────┤
│  Service Layer        状態管理, 通知, ログ, 通信        │
├─────────────────────────────────────────────────────┤
│  Algorithm Layer      状態推定, 制御, フィルタ          │
├─────────────────────────────────────────────────────┤
│  HAL Layer            IMU, Motor, LED, ToF, etc.      │
├─────────────────────────────────────────────────────┤
│  BSP Layer  (sf_board)  共有HW資源の唯一の所有者:        │
│                         I2C/SPI bus, LEDC timer,     │
│                         esp_netif, event_loop, NVS    │
├─────────────────────────────────────────────────────┤
│  Hardware             ESP32-S3 peripherals            │
└─────────────────────────────────────────────────────┘
```

**BSP（Board Support Package）層の役割:** 複数の HAL コンポーネント（ToF / Baro / Mag / Power 等が共有する I2C バス、IMU が使う SPI バス、Motor / LED / Buzzer が共有する LEDC タイマー、WiFi / ESP-NOW が必要とする esp_netif と event_loop など）を **唯一の場所で所有・初期化する**。各 HAL は `sf_board` から bus handle を借りて動作する（extern グローバル禁止）。詳細は [`hardware_init.md`](hardware_init.md) を参照。

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

### 横断ルール（v3）

旧 vehicle/ で発生したスパゲッティ徴候、組み込み制御業界標準パターン、教育用ファームとしての要請をまとめた 16 項目。実装はこれらに違反しないこと。違反を発見したら設計矛盾として即時報告する。

**HW / 起動シーケンス**

| # | ルール |
|---|------|
| R1 | `sf_board` が共有 HW 資源（I2C/SPI/LEDC/esp_netif/event_loop/NVS）の唯一の所有者である |
| R2 | HAL は Config 経由で bus handle を借用する（2 段階初期化、戻り値は `esp_err_t`） |
| R3 | `main.cpp` は線形・宣言的（Phase 0→1→2→3→4 を読めば起動順序が完全に分かる） |
| R4 | 失敗を 3 段階に分類: Critical = `abort()`、Optional = `sensor_present(id) = false` で続行、Recoverable = task ループ内 retry + ログ throttle |

**コンポーネント間通信**

| # | ルール |
|---|------|
| R5 | データ共有は Pub-Sub Topic のみ。複数 producer / 複数 consumer が同じバッファに無ロック並走することを禁止 |
| R6 | CLI コマンドはレジストリパターン（`{name, callback}` 配列で登録）。CLI 用の extern グローバルポインタを作らない |
| R7 | 同期手段は **3 種類のみ**: Pub-Sub Topic / `xTaskNotify` / `std::atomic`。`volatile bool` の散在を禁止 |

**学習者対応（4 階層アクセス）**

| # | ルール |
|---|------|
| R8 | 公開 API は `sf::api` namespace、内部 API は `sf::internal` namespace に分離。学習者は `sf::api` のみを include する |
| R9 | `docs/topic_reference.md` を Topic SSOT 文書とする。Topic 追加時は同時に表を更新する PR 必須 |
| R10 | 学習者バイパス機構を提供（`params::sim::use_true_*` 等で Estimator/Controller を素朴化、SIL/学習段階で各層を独立検証可能） |
| R11 | Guidance / Navigation Topic を予約定義（`command_target`, `nav_path`）。実装は将来でも、置き場所と入出力契約を今決める |
| R12 | `firmware/workshop/` の HAL コピーを廃止し、vehicle_new の HAL を共有する。Workshop API は L0 ラッパーとして再実装する |

**Topic 運用**

| # | ルール |
|---|------|
| R13 | 各 task ヘッダに `@publisher` / `@subscriber` アノテーションで Topic 利用関係を明記する |
| R14 | 各 Topic に `overflow_count` を内蔵し、telemetry / sensor_health から監視可能にする |
| R15 | `sensor_health` Topic を `sf_board` が 1Hz publish（各センサの presence / last_update_us / quality） |
| R16 | タイムスタンプ付きデータ（`CommandSetpoint` 等）は subscriber 側でタイムアウト判定し、Failsafe 発動の根拠とする |

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
| — | （責務外）BSP — 共有HW資源の所有・初期化 | `sf_board` | I2C/SPI/LEDC/netif/event_loop/NVS の所有者 |

**実装コンポーネント数: 27**（HAL 10 + 責務系 14 + コア基盤 3）

### 学習者の入口（4 階層アクセス）

vehicle_new は **学習者がレベルに応じて入口を選べる** 並列 API を提供する。Workshop 受講者から HW 学習者、ファーム実装者まで、全員が同じファームウェアを共有しつつ、自分のテーマに集中できる。

| 層 | 名前空間 | 典型ユーザー | できること |
|----|---------|------------|----------|
| **L0: Sketch API** | `ws::*` | Workshop 受講者・初心者 | `setup()` / `loop_400Hz(dt)`、`ws::motor_set_duty()`, `ws::gyro_x()` 等の 30+ 関数で完結。HW・タスク・Topic 知識ゼロでフライト制御まで体験 |
| **L1: Topic API** | `sf::api::*` | 推定・制御・ガイダンス学習者 | Topic を subscribe / publish して自分の ESKF / PID / Navigator を実装。`IEstimator` / `IController` を実装して既存と差替え |
| **L2: HAL Direct** | `stampfly::*Wrapper` | HW 学習者 | `BMI270Wrapper.readSensorData()` 等を直接呼び、SPI / I2C / RMT / LEDC を理解。Topic を介さない経路 |
| **L3: BSP Internal** | `sf::internal::board` | ファーム実装者・拡張者 | `sf_board` の getter で bus handle を取得、esp-idf 直叩き。起動順序や HW 資源管理を変更できる |

各層は **並列に共存する** — Workshop 受講者は L0 だけ、PID 学習者は L1 だけ、BMI270 の SPI 通信を理解したい学生は L2 まで降りる。**HW を「隠す」のではなく「学べる」** よう、どの層も完成度高く整備する。

#### HW 要素から見た入口マッピング

| HW 要素 | L0 (Sketch) | L1 (Topic) | L2 (HAL) | L3 (BSP) |
|---------|-----------|-----------|----------|----------|
| BMI270 IMU | `ws::gyro_x()` | `sensor_imu` | `BMI270Wrapper` | SPI bus |
| BMP280 Baro | `ws::baro_altitude()` | `sensor_baro` | `BMP280Wrapper` | I2C bus |
| BMM150 Mag | `ws::mag_x()` | `sensor_mag` | `BMM150Wrapper` | I2C bus |
| VL53L3CX ToF | `ws::tof_bottom()` | `sensor_tof` | `VL53L3CXWrapper` | I2C + XSHUT |
| PMW3901 Flow | `ws::flow_vx()` | `sensor_flow` | `PMW3901Wrapper` | SPI bus |
| Power monitor | `ws::battery_voltage()` | `sensor_power` | `PowerMonitor` | I2C |
| LEDC PWM motor | `ws::motor_set_duty()` | `actuator_motor` | `MotorDriver` | LEDC timer |
| WS2812 RGB LED | `ws::led_color()` | (`notify_pattern`) | `LEDDriver` | RMT |
| Buzzer | (内部) | (`notify_tone`) | `BuzzerDriver` | LEDC ch |
| Button | (内部) | (`button_event`) | `ButtonDriver` | GPIO ISR |
| ESP-NOW | (内部) | `command_setpoint` | `sf_comm` | WiFi STA |
| WiFi UDP | (内部) | (telemetry pkt) | `sf_telemetry` | netif/socket |

L0〜L2 は学習者が任意に選択し、隣接層へ階段的に降りられる構造とする。L3 はファーム実装者専用（学習者は通常触らない）。

詳細な API 設計指針と Examples 計画は [`coding_and_education.md`](coding_and_education.md) を参照。

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
    float angular_rate[3];    // Body angular rate [rad/s] FRD (gyro − bias)
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
| BSP（`sf_board`） | `app_main()` の Phase 1 で `sf::board::init()` を 1 回呼ぶ | 起動時 1 回のみ、常駐タスクなし |

## 7. ハードウェア初期化と所有権

### 設計の背景

旧 `firmware/vehicle/` では、共有 HW 資源（I2C バス、esp_netif、WiFi 等）の所有が複数モジュールに分散していた。例: I2C バスは `init.cpp` の file-scope static、`esp_netif_create_default_wifi_sta()` は `controller_comm.cpp` 内、センサ wrapper 群は `globals.hpp` の extern グローバル。これにより以下の問題が発生した。

- 初期化順序の暗黙依存（順番を間違えると crash）
- 「このグローバルは誰が所有しているか」がコードを読んで追えない
- センサ init 失敗時の挙動が WARN + fail-through で、後続の null pointer リスクがあった

vehicle_new では、これらを **`sf_board` という単一の BSP コンポーネントに集約** する。

### sf_board の責務

`sf_board` は以下の共有 HW 資源の **唯一の所有者** である。

| 資源 | 用途 | 借用する HAL |
|------|------|------------|
| I2C master bus | 30Hz〜100Hz の低レートセンサ通信 | sf_hal_bmp280, sf_hal_bmm150, sf_hal_vl53l3cx, sf_hal_power |
| SPI host (IMU) | 400Hz の IMU 通信 | sf_hal_bmi270 |
| SPI host (Flow) | 100Hz の OptFlow 通信 | sf_hal_pmw3901 |
| LEDC timer | PWM モータ・LED・ブザー駆動 | sf_hal_motor, sf_hal_led, sf_hal_buzzer |
| esp_netif + event_loop | TCP/IP スタックとイベントループ | sf_comm（WiFi/ESP-NOW実体）, sf_telemetry（UDP socket） |
| NVS partition | パラメータ・キャリブレーション永続化 | sf_calibration, params system |

各 HAL は **`sf_board` から bus handle を Config 経由で借りて** 動作する（extern グローバル禁止、R1〜R2）。

### 起動シーケンス（main.cpp の宣言的構造）

```cpp
extern "C" void app_main() {
  // Phase 0: pre-kernel resources
  ESP_ERROR_CHECK(nvs_flash_init());

  // Phase 1: BSP init — all shared HW resources
  ESP_ERROR_CHECK(sf::board::init());

  // Phase 2: Pub-Sub topics
  sf::topics_init();

  // Phase 3: parameter loading from NVS
  sf::params::loadFromNvs();

  // Phase 4: tasks
  sf::tasks::start_all();
}
```

`main.cpp` は線形・宣言的（R3）。Phase 1 の `sf::board::init()` 内部で I2C → SPI → LEDC → netif → event_loop → 各センサ HAL → アクチュエータ HAL の順に初期化される。順序変更が必要な場合は `board.cpp` 1 ファイル内のみで完結する。

### 失敗の 3 段階分類

| 分類 | 例 | 挙動 |
|------|-----|-----|
| **Critical** | BMI270, モータ HAL, NVS | `abort()` でブートを止め、LED 赤点滅でユーザーに伝達 |
| **Optional** | BMM150, Front ToF | `sensor_present(id) = false` でフラグ降ろし、機能無効で続行 |
| **Recoverable** | 一時的 I/O エラー | task ループ内で retry、`READ_FAIL_LOG_INTERVAL` で警告抑制 |

詳細な分類リスト・LED エラーパターン・HAL との接続規約・namespace 規約は [`hardware_init.md`](hardware_init.md) を参照。

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

> **v3 update note:** The Japanese §2 has been extended with cross-cutting rules R1–R16, a new BSP layer (`sf_board`), and the 4-tier learner access model (L0 Sketch / L1 Topic / L2 HAL / L3 BSP). Full English translation will be completed in milestone M1c. For the authoritative v3 design, see the Japanese sections above and [`hardware_init.md`](hardware_init.md).

### Layered Architecture

```
┌─────────────────────────────────────────────────────┐
│  Application Layer    Tasks: StateTask, etc.         │
├─────────────────────────────────────────────────────┤
│  Service Layer        State Mgr, Notify, Log         │
├─────────────────────────────────────────────────────┤
│  Algorithm Layer      Estimation, Control, Filter    │
├─────────────────────────────────────────────────────┤
│  HAL Layer            IMU, Motor, LED, ToF, etc.     │
├─────────────────────────────────────────────────────┤
│  BSP Layer  (sf_board)  Sole owner of shared HW:     │
│                         I2C/SPI bus, LEDC timer,    │
│                         esp_netif, event_loop, NVS   │
├─────────────────────────────────────────────────────┤
│  Hardware             ESP32-S3 peripherals           │
└─────────────────────────────────────────────────────┘
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
