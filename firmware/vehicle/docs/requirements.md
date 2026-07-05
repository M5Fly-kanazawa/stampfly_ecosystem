# vehicle Requirements Definition
# vehicle 要件定義書

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 目的・方針

### 上位目標

- StampFlyの制御ファームウェアとして機能する
- 拡張性を持つ
- プログラミング教材としてコードの見通しが良い
- stampfly_ecosystemの中核ソフトウェアとしての位置づけを保持する

### 直接の動機

- 旧ファームのスパゲッティ化を解消するための再構築
- コンポーネント間の責務明確化、状態管理の強化
- 意味不明の挙動を設計段階で排除

## 2. 状態モデル

### 状態遷移図

```
INIT（逐次実行、サブモードなし）
  |
IDLE
  ├─ GROUND  — ToF無効距離、ARM可、キャリブレーション可
  └─ HELD    — ToF有効距離、ARM禁止、推定テスト可
  | (ARM、GROUNDからのみ)
ARMED_GROUND
  |
TAKEOFF（シーケンス）
  |
FLYING
  ├─ ACRO         — 角速度制御
  ├─ STABILIZE     — 姿勢安定
  ├─ ALT_HOLD      — 高度維持
  └─ POS_HOLD      — 位置維持
  |
LANDING（シーケンス）
  |
IDLE_GROUND
```

### 設計原則

- ARM/DISARMはアクション（モードではない）
- 親状態が共通処理を保証
- TAKEOFF/LANDINGは専用モード（離着陸マネージャーが統括）
- シーケンス（FLIP, AUTO_LAND等）は将来拡張可能
- FAILSAFEは状態ではなくイベント — フェイルセーフコンポーネントが `system.alert` を発行し、状態管理が既存モードへの遷移で対応する

### 遷移条件

| 遷移 | トリガー |
|------|---------|
| INIT → IDLE_GROUND | 初期化完了 |
| IDLE_GROUND ↔ IDLE_HELD | ToF距離による自動判定 |
| IDLE_GROUND → ARMED_GROUND | ARMアクション（ボタン or コントローラ） |
| ARMED_GROUND → TAKEOFF | **ALT_HOLD/POS_HOLD: ARM 自体がトリガ**（スロットル入力不要）。短いスプール/整定ドウェル（0.3s, モータはゼロ）の後、自動離陸シーケンス開始: 固定上昇率＋水平姿勢で**目標高度 0.5m**（地面効果回避, config）まで。**ACRO/STABILIZE: 手動スロットル入力**（生スロットル＝推力、自動離陸なし） |
| TAKEOFF → FLYING | 離陸完了。**ALT/POS: 制御器が目標高度 0.5m を捕捉して通知**（運動量による行き過ぎでなく目標値を捕捉）。**ACRO/STABILIZE: ToF 空中検知（0.15m）**。なお ToF 空中検知 0.15m は ESKF 鉛直ハンドオフ（ImuTask, クラスB）に役割分離され、ALT/POS の離陸完了判定には用いない |
| サブモード切替 | コントローラからのモード切替コマンド。**地上（IDLE_GROUND/ARMED_GROUND）と FLYING で受理**（設置時の変更が最も安全なため。INIT/TAKEOFF/LANDING/IDLE_HELD 中は拒否し、遷移完了後に適用） |
| FLYING → LANDING | 自動着陸の起動。**(a) ALT_HOLD/POS_HOLD でのパイロット DISARM**（高度制御モードは自力で着陸できるため、空中でモータを切らず緩降下する。空中での即カットは機体を落とすだけ）、**(b) 通信途絶のホバー猶予経過**（§安全要件）。緩降下（固定降下率, config）で接地まで |
| LANDING → IDLE_GROUND | 着陸完了（ToF 接地検出 → 本当の DISARM、モータゼロ） |
| FLYING → ARMED_GROUND | 静かに着陸 / タッチアンドゴー |
| FLYING → IDLE_GROUND | **衝突検知**（緊急 DISARM）or **ACRO/STABILIZE でのパイロット DISARM**（手動推力モードは自動着陸を持たないため即カット）or **緊急停止**（API `emergency`／着陸中の再 DISARM＝中断）|
| ARMED_GROUND → IDLE_GROUND | DISARMアクション（地上は常に即カット） |

### ペアリング状態（PairingState）

FlightState とは**独立した並行状態機械**。送信機（コントローラ）と機体を1対1に束ね、複数機・複数
送信機を同一空間で運用したときの**混信**（他人の送信機で自分の機体が動く）を防ぐ。1教室で最大30機
が同時に飛ぶ運用を前提とする。状態管理（StateManager）が**単一所有**する。

```
PairingState（FlightState と並行）
  NotPaired — 相手コントローラ未確定（NVSに保存なし）
  Pairing   — 相手探索中。PairingPacket を 500ms 周期で broadcast 送出
  Paired    — 相手 MAC 確定（NVS保存済み）。相手以外の ControlPacket は破棄
```

| 遷移 | トリガー |
|------|---------|
| 起動（NVSに相手なし）→ Pairing | 自動（未ペア起動で自動的にペアリング待機）|
| 起動（NVSに相手あり）→ Paired | 自動（保存済み相手 MAC を復元）|
| Pairing → Paired | Pairing 中に相手から ControlPacket を受信し src MAC を学習 |
| 任意 → Pairing（再ペアリング）| ボタン長押し3秒（IDLE_GROUND のみ）。既存ペアを破棄して再探索 |

**制約:**
- PAIRING 突入は地上（IDLE_GROUND）でのみ可能。**Pairing 中は ARM 要求を拒否**する。
- ハンドシェイクは**相互 MAC 学習**: 機体が自 MAC を PairingPacket で広告 → コントローラが学習して
  機体 MAC 宛に ControlPacket をユニキャスト送信 → 機体が受信パケットの src MAC を相手として確定。
- 旧 vehicle のペアリングシーケンスを踏襲する（プロトコル・署名・周期を同一に保ち相互運用）。

## 3. パラメータ管理

| 項目 | 方針 |
|------|------|
| 固定パラメータ | GPIO、タスク優先度等 → `constexpr` |
| チューニングパラメータ | PIDゲイン、ESKF設定等 → ランタイム変更可能 |
| 命名規則 | 3階層: `機能.対象.パラメータ` |
| 永続化 | NVS |
| 追加手順 | 定義 + テーブル登録の2箇所 |
| アクセス手段 | WiFi + CLI |
| コールバック・バリデーション | パラメータ確定後に個別検討 |

## 4. 責務分割（14コンポーネント）

| # | コンポーネント | 責務 |
|---|---|---|
| 1 | センシング | センサからデータを読む |
| 2 | 状態推定 | 姿勢/位置/速度を推定（差替可能） |
| 3 | 状態管理 | モード遷移、ARM許可判定 |
| 4 | フェイルセーフ | 異常検出と対応アクション（状態管理より上位） |
| 5 | 離着陸マネージャー | 地上/空中判定、TAKEOFF/LANDINGモード統括、ESKF切替・リセット |
| 6 | 制御 | セットポイント追従演算（差替可能） |
| 7 | アクチュエーション | ミキサー+安全チェック+モーター出力 |
| 8 | コマンド処理 | 全入力ソース吸収・正規化・調停→セットポイント |
| 9 | 通信 | ESP-NOW/UDP/WiFiの送受信（物理レイヤー） |
| 10 | ナビゲーター | ウェイポイント、経路計画 |
| 11 | キャリブレーション管理 | バイアス測定・保存・適用 |
| 12 | パラメータ | パラメータ保持・変更・永続化 |
| 13 | データロガー | 飛行データの記録（Telemetry/Data Stream/Blackbox） |
| 14 | 通知 | LED/ブザーで状態表示 |

### 設計原則

- 「検出」と「判断」を分離（センシングが事実を報告、状態管理が判断）
- 制御はインターフェース統一（入力: 推定値+セットポイント、出力: 推力/トルク）
- 状態推定もインターフェース統一（入力: センサデータ、出力: 姿勢/位置/速度）
- 状態遷移コールバック（onExit/onEnter）でリセット処理を集約

## 5. 搭載センサ

| センサ | 型番 | 通信 | レート | 用途 |
|--------|------|------|--------|------|
| IMU | BMI270 | SPI | 400Hz | 姿勢推定の主センサ |
| 地磁気 | BMM150 | I2C | 25Hz | ヨー推定（研究対象） |
| 気圧 | BMP280 | I2C | 50Hz | 高度推定（ToF範囲外） |
| ToF（下面） | VL53L3CX | I2C | 30Hz | 高度計測 |
| ToF（前面） | VL53L3CX | I2C | 30Hz | 障害物検知・SLAM・ナビゲーション |
| OpticalFlow | PMW3901 | SPI | 100Hz | 水平位置推定 |
| 電源モニタ | INA3221 | I2C | 10Hz | バッテリー電圧・電流 |

全センサ残す（IoT教材として全アクセス可能であることが重要）

## 6. 搭載アクチュエータ

| アクチュエータ | 制御方式 | 数量 |
|---|---|---|
| モーター | LEDC PWM 150kHz 8bit | 4（X-quad配置） |
| ブザー | LEDC PWM トーン生成 | 1 |
| LED（MCU内蔵） | WS2812 RGB | 1 |
| LED（ボード上下） | WS2812 RGB デイジーチェーン | 2 |

## 7. 通信

### 機体側の通信

| 名前 | 目的 | 経路 | 初期実装 |
|------|------|------|---------|
| Telemetry | モニタリング用（状態表示） | UDP | Yes |
| Data Stream | 解析用（全センサ生値） | UDP / USB（選択） | Yes |
| Blackbox | オンボード記録（クラッシュログ） | 内部Flash（SPIFFS） | Yes |
| CLI | コマンド操作 | USB Serial / TCP | Yes |
| コマンド受信 | RC入力、API | ESP-NOW / UDP | Yes |

- 機体側からWebSocketは撤去（UDP統一、ブロッキングリスク排除）
- ブラウザ表示は `sf telemetry --web`（UDP→SSE 変換プロキシ。WebSocket の stdlib 等価）で対応
- スマホ対応は専用アプリ or WebApp

### ペアリング（混信対策）

送信機と機体を1対1に束ねる。SSOT は `protocol/spec/messages.yaml`。詳細は状態モデル §2 の
PairingState を参照。

| 項目 | 仕様 |
|------|------|
| 目的 | 複数機・複数送信機の同一空間運用での混信防止（1教室30機前提）|
| プロトコル | PairingPacket（11B: channel 1B + 機体MAC 6B + 署名 `0xAA 0x55 0x16 0x88` 4B）|
| 送出 | Pairing 中、機体が 500ms 周期で broadcast。コントローラが CH1-13 スキャンで発見 |
| アドレッシング | ControlPacket 先頭3バイト `drone_mac` ＝ 機体 MAC 下位3バイト（SSOT既存フィールド）|
| 永続化 | 相手コントローラ MAC を NVS 保存し起動時に復元 |
| フィルタ | 通常運用時、受信 ControlPacket の src MAC が相手と不一致なら破棄 |
| UI | Pairing 中: LED 青速点滅 ＋ ブザー通知。成立で解除 |

### 対応プロトコル

| プロトコル | 初期実装 |
|-----------|---------|
| ESP-NOW（デフォルトRC） | Yes |
| UDP（テレメトリ、制御） | Yes |
| TCP（CLI） | Yes |
| USB Serial（CLI + ログ） | Yes |
| TelloAPI | Yes |
| ROS2 | Yes |
| MAVLink | Yes |
| SBUS（外部RC） | 将来（構造のみ） |
| OTA | しない |

## 8. タイミング要件

| タスク | 周期 | 優先度 | スタック |
|--------|------|--------|---------|
| IMU | 400Hz | 24 | 16KB |
| Control | 400Hz（IMU同期） | 23 | 8KB |
| OptFlow | 100Hz | 20 | 8KB |
| Mag | 25Hz | 18 | 8KB |
| Baro | 50Hz | 16 | 8KB |
| Comm | 50Hz | 15 | 4KB |
| ToF | 30Hz | 14 | 8KB |
| Telemetry | 50Hz | 13 | 4KB |
| Power | 10Hz | 12 | 4KB |
| Button | 50Hz | 10 | 4KB |
| LED | 30Hz | 8 | 4KB |
| CLI | 20Hz | 5 | 8KB |

新責務（フェイルセーフ、離着陸マネージャー等）のタスクマッピングはタスク設計で決定。

## 9. 安全要件

| 条件 | 閾値 | アクション |
|------|------|-----------|
| 衝撃（加速度） | 3.0G × 連続2回 | 自動DISARM |
| 異常角速度 | 800 deg/s × 連続2回 | 自動DISARM |
| 通信途絶 | 500ms | ホバリング維持3秒 → 自動着陸 |
| LiPo低電圧 | ≤3.4V | ブザー警告のみ |
| USB給電 | ≤3.3V | ARM禁止 |
| ESKF発散 | 位置100m or 速度50m/s | ESKFリセット |

## 10. 教育・拡張性要件

| 項目 | 方針 | 初期実装 |
|------|------|---------|
| 状態推定 | 差替可能 | Yes |
| 制御 | 差替可能 | Yes |
| 入力ソース | ドライバ追加で拡張 | Yes |
| パラメータ | WiFi変更、NVS永続化 | Yes |
| ナビゲーション | 将来追加可能な構造 | 構造のみ |
| SILシミュレータ | HAL差し替えでPC上動作 | Yes |
| 単体テスト | アルゴリズム層をPC上でテスト可能 | Yes |
| コード品質 | 日英バイリンガルコメント、読みやすさ重視 | Yes |
| サンプル集 | 機能別の最小動作コード、チュートリアル的コメント | Yes |
| TelloAPI | 標準対応 | Yes |
| ROS2 | 標準対応 | Yes |
| MAVLink | 標準対応 | Yes |
| SBUS | 将来対応 | 構造のみ |

---

<a id="english"></a>

## 1. Purpose and Policy

### High-Level Goals

- Function as StampFly's control firmware
- Maintain extensibility
- Provide clear, readable code as a programming educational material
- Serve as the core software within the stampfly_ecosystem

### Direct Motivation

- Rebuild to resolve spaghetti code in legacy firmware
- Clarify component responsibilities and strengthen state management
- Eliminate undefined behavior at the design stage

## 2. State Model

### State Transition Diagram

```
INIT (sequential, no sub-modes)
  |
IDLE
  ├─ GROUND  — ToF out of range, ARM allowed, calibration allowed
  └─ HELD    — ToF in range, ARM prohibited, estimation testing possible
  | (ARM, from GROUND only)
ARMED_GROUND
  |
TAKEOFF (sequence)
  |
FLYING
  ├─ ACRO         — Rate control
  ├─ STABILIZE     — Attitude stabilization
  ├─ ALT_HOLD      — Altitude hold
  └─ POS_HOLD      — Position hold
  |
LANDING (sequence)
  |
IDLE_GROUND
```

### Design Principles

- ARM/DISARM are actions (not modes)
- Parent state guarantees common behavior
- TAKEOFF/LANDING are dedicated modes (managed by Takeoff/Landing Manager)
- Sequences (FLIP, AUTO_LAND, etc.) are extensible for future use
- FAILSAFE is an event, not a state — Failsafe component publishes `system.alert`, State Management responds with mode transitions

### Transition Conditions

| Transition | Trigger |
|------------|---------|
| INIT → IDLE_GROUND | Initialization complete |
| IDLE_GROUND ↔ IDLE_HELD | Automatic detection by ToF distance |
| IDLE_GROUND → ARMED_GROUND | ARM action (button or controller) |
| ARMED_GROUND → TAKEOFF | Throttle input |
| TAKEOFF → FLYING | Takeoff complete (altitude threshold reached) |
| FLYING sub-mode switch | Mode switch command from controller |
| FLYING → LANDING | Starts an auto-landing: **(a) pilot DISARM in ALT_HOLD/POS_HOLD** (altitude-controlled modes can land themselves, so the craft descends gradually instead of cutting motors mid-air — a mid-air cut would just drop it); **(b) comm-loss hover grace elapsed** (see Safety). Gradual descent (fixed rate, config) to touchdown |
| LANDING → IDLE_GROUND | Landing complete (ToF touchdown detection → the real DISARM, motors zero) |
| FLYING → ARMED_GROUND | Soft landing / touch-and-go |
| FLYING → IDLE_GROUND | **Crash detection** (emergency DISARM) or **pilot DISARM in ACRO/STABILIZE** (manual-thrust modes have no auto-landing, so cut immediately) or **emergency stop** (API `emergency` / a second DISARM while landing = abort) |
| ARMED_GROUND → IDLE_GROUND | DISARM action (on the ground always an immediate cut) |

### Pairing State (PairingState)

A **separate, parallel state machine** independent of FlightState. It binds one transmitter
(controller) to one vehicle to prevent **crosstalk** (someone else's transmitter moving your
vehicle) when multiple vehicles/transmitters operate in the same space — assuming up to 30
vehicles flying simultaneously in one classroom. **Single-owned** by State Management.

```
PairingState (parallel to FlightState)
  NotPaired — No bound controller yet (nothing saved in NVS)
  Pairing   — Searching for a peer; broadcasts a PairingPacket every 500ms
  Paired    — Peer MAC fixed (saved in NVS); ControlPackets from others are dropped
```

| Transition | Trigger |
|------------|---------|
| Boot (no peer in NVS) → Pairing | Automatic (unpaired boot auto-enters pairing) |
| Boot (peer in NVS) → Paired | Automatic (restore saved peer MAC) |
| Pairing → Paired | Receive a ControlPacket from the peer during Pairing, learning its src MAC |
| Any → Pairing (re-pair) | Button long-press 3s (IDLE_GROUND only); discard existing pair and re-search |

**Constraints:**
- Pairing can be entered only on the ground (IDLE_GROUND). **ARM requests are rejected while Pairing.**
- The handshake is **mutual MAC learning**: the vehicle advertises its own MAC via PairingPacket →
  the controller learns it and unicasts ControlPackets to the vehicle MAC → the vehicle fixes the
  received packet's src MAC as its peer.
- Follows the legacy vehicle's pairing sequence (same protocol, signature, and period for interop).

## 3. Parameter Management

| Item | Policy |
|------|--------|
| Fixed parameters | GPIO, task priorities, etc. → `constexpr` |
| Tuning parameters | PID gains, ESKF settings, etc. → runtime modifiable |
| Naming convention | 3-level hierarchy: `function.target.parameter` |
| Persistence | NVS |
| Addition procedure | Definition + table registration (2 locations) |
| Access methods | WiFi + CLI |
| Callbacks / validation | To be determined per parameter after finalization |

## 4. Responsibility Assignment (14 Components)

| # | Component | Responsibility |
|---|-----------|---------------|
| 1 | Sensing | Read data from sensors |
| 2 | State Estimation | Estimate attitude/position/velocity (replaceable) |
| 3 | State Management | Mode transitions, ARM permission |
| 4 | Failsafe | Anomaly detection and response (higher priority than State Management) |
| 5 | Takeoff/Landing Manager | Ground/air detection, TAKEOFF/LANDING mode orchestration, ESKF switching/reset |
| 6 | Control | Setpoint tracking computation (replaceable) |
| 7 | Actuation | Mixer + safety check + motor output |
| 8 | Command Processing | Absorb all input sources, normalize, arbitrate → setpoint |
| 9 | Communication | ESP-NOW/UDP/WiFi packet send/receive (physical layer) |
| 10 | Navigator | Waypoints, path planning |
| 11 | Calibration Management | Bias measurement, storage, application |
| 12 | Parameters | Parameter storage, modification, persistence |
| 13 | Data Logger | Flight data recording (Telemetry/Data Stream/Blackbox) |
| 14 | Notification | LED/buzzer state display |

### Design Principles

- Separate "detection" from "decision" (Sensing reports facts, State Management decides)
- Unified control interface (input: estimates + setpoints, output: thrust/torque)
- Unified estimation interface (input: sensor data, output: attitude/position/velocity)
- State transition callbacks (onExit/onEnter) consolidate reset processing

## 5. Sensors

| Sensor | Part | Bus | Rate | Purpose |
|--------|------|-----|------|---------|
| IMU | BMI270 | SPI | 400Hz | Primary sensor for attitude estimation |
| Magnetometer | BMM150 | I2C | 25Hz | Yaw estimation (research target) |
| Barometer | BMP280 | I2C | 50Hz | Altitude estimation (beyond ToF range) |
| ToF (bottom) | VL53L3CX | I2C | 30Hz | Altitude measurement |
| ToF (front) | VL53L3CX | I2C | 30Hz | Obstacle detection, SLAM, navigation |
| Optical Flow | PMW3901 | SPI | 100Hz | Horizontal position estimation |
| Power Monitor | INA3221 | I2C | 10Hz | Battery voltage/current |

All sensors retained (important for IoT educational material to have full sensor access)

## 6. Actuators

| Actuator | Control Method | Quantity |
|----------|---------------|----------|
| Motors | LEDC PWM 150kHz 8bit | 4 (X-quad layout) |
| Buzzer | LEDC PWM tone generation | 1 |
| LED (MCU built-in) | WS2812 RGB | 1 |
| LED (board top/bottom) | WS2812 RGB daisy-chain | 2 |

## 7. Communication

### Vehicle-Side Communication

| Name | Purpose | Transport | Initial Implementation |
|------|---------|-----------|----------------------|
| Telemetry | Monitoring (state display) | UDP | Yes |
| Data Stream | Analysis (all sensor raw data) | UDP / USB (selectable) | Yes |
| Blackbox | Onboard recording (crash log) | Internal Flash (SPIFFS) | Yes |
| CLI | Command operation | USB Serial / TCP | Yes |
| Command Receive | RC input, API | ESP-NOW / UDP | Yes |

- WebSocket removed from vehicle (UDP unified, eliminates blocking risk)
- Browser display via `sf telemetry --web` (UDP→SSE conversion proxy, stdlib equivalent of WebSocket)
- Smartphone support via dedicated app or WebApp

### Pairing (Crosstalk Prevention)

Binds one transmitter to one vehicle. SSOT is `protocol/spec/messages.yaml`. See PairingState in
the §2 State Model for details.

| Item | Specification |
|------|---------------|
| Purpose | Prevent crosstalk when multiple vehicles/transmitters share a space (30/classroom) |
| Protocol | PairingPacket (11B: channel 1B + vehicle MAC 6B + signature `0xAA 0x55 0x16 0x88` 4B) |
| Broadcast | While Pairing, the vehicle broadcasts every 500ms; the controller scans CH1-13 to find it |
| Addressing | ControlPacket's first 3 bytes `drone_mac` = vehicle MAC lower 3 bytes (existing SSOT field) |
| Persistence | Save the peer controller MAC to NVS and restore it at boot |
| Filter | In normal operation, drop received ControlPackets whose src MAC differs from the peer |
| UI | While Pairing: LED fast blue blink + buzzer notification; cleared on completion |

### Supported Protocols

| Protocol | Initial Implementation |
|----------|----------------------|
| ESP-NOW (default RC) | Yes |
| UDP (telemetry, control) | Yes |
| TCP (CLI) | Yes |
| USB Serial (CLI + log) | Yes |
| Tello API | Yes |
| ROS2 | Yes |
| MAVLink | Yes |
| SBUS (external RC) | Future (structure only) |
| OTA | No |

## 8. Timing Requirements

| Task | Rate | Priority | Stack |
|------|------|----------|-------|
| IMU | 400Hz | 24 | 16KB |
| Control | 400Hz (IMU-synced) | 23 | 8KB |
| OptFlow | 100Hz | 20 | 8KB |
| Mag | 25Hz | 18 | 8KB |
| Baro | 50Hz | 16 | 8KB |
| Comm | 50Hz | 15 | 4KB |
| ToF | 30Hz | 14 | 8KB |
| Telemetry | 50Hz | 13 | 4KB |
| Power | 10Hz | 12 | 4KB |
| Button | 50Hz | 10 | 4KB |
| LED | 30Hz | 8 | 4KB |
| CLI | 20Hz | 5 | 8KB |

Task mapping for new responsibilities (Failsafe, Takeoff/Landing Manager, etc.) to be determined during task design.

## 9. Safety Requirements

| Condition | Threshold | Action |
|-----------|-----------|--------|
| Impact (acceleration) | 3.0G × 2 consecutive | Auto DISARM |
| Abnormal angular rate | 800 deg/s × 2 consecutive | Auto DISARM |
| Communication loss | 500ms | Hover hold 3s → auto landing |
| LiPo low voltage | ≤3.4V | Buzzer warning only |
| USB power | ≤3.3V | ARM prohibited |
| ESKF divergence | Position 100m or velocity 50m/s | ESKF reset |

## 10. Education and Extensibility Requirements

| Item | Policy | Initial Implementation |
|------|--------|----------------------|
| State estimation | Replaceable | Yes |
| Control | Replaceable | Yes |
| Input sources | Extend by adding drivers | Yes |
| Parameters | WiFi modification, NVS persistence | Yes |
| Navigation | Future-extensible structure | Structure only |
| SIL simulator | Run on PC by replacing HAL | Yes |
| Unit testing | Algorithm layer testable on PC | Yes |
| Code quality | Bilingual comments (JP/EN), readability focus | Yes |
| Example collection | Minimal working code per feature, tutorial-style comments | Yes |
| Tello API | Standard support | Yes |
| ROS2 | Standard support | Yes |
| MAVLink | Standard support | Yes |
| SBUS | Future support | Structure only |
