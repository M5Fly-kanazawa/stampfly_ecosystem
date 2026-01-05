# StampFly Vehicle Firmware

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このプロジェクトについて

StampFly Vehicle Firmware は、StampFly ドローン機体用のフライトコントローラファームウェアです。
このファームウェアは**スケルトン（骨格）**として設計されており、飛行制御を実装したいエンジニアや学生のために、
センサー読み取り、状態推定、通信などの基盤機能をすべて提供します。

**スケルトンとは：**
- センサーの読み込み、キャリブレーション
- StampFly の各状態の一元管理
- コントローラとの通信
- CLI によるセンサ値取得・ゲイン調整
- 位置姿勢推定（ESKF: Error-State Kalman Filter）

これらが実装済みで、ユーザーは `control_task.cpp` の制御ループに自分の制御則を実装するだけで飛行させることができます。

### 対象ハードウェア

- **MCU**: M5Stamp S3（ESP32-S3）
- **フレームワーク**: ESP-IDF v5.4.1 + FreeRTOS
- **機体**: StampFly

### 主な機能

| 機能 | 説明 |
|------|------|
| 6軸IMU | BMI270（加速度・ジャイロ）400Hz |
| 地磁気センサー | BMM150 100Hz |
| 気圧センサー | BMP280 50Hz |
| ToFセンサー | VL53L3CX（底面・前方）30Hz |
| オプティカルフロー | PMW3901 100Hz |
| 電源監視 | INA3221 10Hz |
| 位置姿勢推定 | ESKF（Error-State Kalman Filter） |
| 通信 | ESP-NOW（コントローラ）+ WebSocket（テレメトリ） |
| CLI | USBシリアル経由のコマンドライン |
| LED | WS2812 RGB LED（状態表示） |
| ブザー | 状態通知用 |

---

## 2. 開発環境のセットアップ

### 必要なソフトウェア

1. **ESP-IDF v5.4.1**
   - [公式インストールガイド](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/get-started/index.html)
   - Windows の場合は ESP-IDF Tools Installer を推奨
   - macOS/Linux の場合は `install.sh` を使用

2. **USB ドライバ**
   - M5Stamp S3 は USB Serial/JTAG を使用
   - macOS/Linux では通常不要
   - Windows では CP210x ドライバが必要な場合あり

### ESP-IDF のセットアップ（macOS/Linux）

```bash
# ESP-IDF をクローン
mkdir -p ~/esp
cd ~/esp
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git

# インストールスクリプトを実行
cd esp-idf
./install.sh esp32s3

# 環境変数を設定（毎回実行するか、.bashrc/.zshrc に追加）
. ~/esp/esp-idf/export.sh
```

### プロジェクトのビルド

```bash
# プロジェクトディレクトリに移動
cd firmware/vehicle

# ビルド
idf.py build

# フラッシュ（書き込み）
idf.py flash

# シリアルモニター（ログ表示）
idf.py monitor

# ビルド・フラッシュ・モニターを一度に実行
idf.py build flash monitor
```

### シリアルポートの指定

デバイスが自動検出されない場合：

```bash
# ポートを指定してフラッシュ
idf.py -p /dev/tty.usbmodem* flash monitor

# Windows の場合
idf.py -p COM3 flash monitor
```

---

## 3. ソフトウェアアーキテクチャ

### ディレクトリ構造

```
firmware/vehicle/
├── main/                      # メインアプリケーション
│   ├── main.cpp               # エントリポイント・初期化シーケンス
│   ├── config.hpp             # 全ての設定パラメータ（GPIO、タスク優先度、ESKF等）
│   ├── globals.hpp/.cpp       # グローバル変数の宣言・定義
│   ├── init.hpp/.cpp          # 初期化関数
│   ├── rate_controller.hpp    # 角速度制御器の定義
│   └── tasks/                 # FreeRTOS タスク
│       ├── tasks.hpp          # タスク関数プロトタイプ
│       ├── imu_task.cpp       # IMU読み取り・センサーフュージョン (400Hz)
│       ├── control_task.cpp   # 制御ループ (400Hz) ★ユーザー実装箇所
│       ├── mag_task.cpp       # 地磁気センサー (100Hz)
│       ├── baro_task.cpp      # 気圧センサー (50Hz)
│       ├── tof_task.cpp       # ToFセンサー (30Hz)
│       ├── optflow_task.cpp   # オプティカルフロー (100Hz)
│       ├── comm_task.cpp      # ESP-NOW通信 (50Hz)
│       ├── power_task.cpp     # 電源監視 (10Hz)
│       ├── led_task.cpp       # LED制御 (50Hz)
│       ├── button_task.cpp    # ボタン監視 (50Hz)
│       ├── cli_task.cpp       # CLIコマンド処理
│       └── telemetry_task.cpp # WebSocketテレメトリ (50Hz)
├── components/                # ESP-IDF コンポーネント
│   ├── sf_hal_*/              # ハードウェア抽象化レイヤー（センサードライバ）
│   ├── sf_algo_*/             # アルゴリズム（ESKF、PID、フィルタ）
│   └── sf_svc_*/              # サービス（状態管理、通信、CLI）
├── tools/                     # 開発支援ツール
│   ├── scripts/               # Python解析スクリプト
│   └── eskf_debug/            # ESKF デバッグ用PCシミュレータ
├── logs/                      # フライトログ保存先
├── CMakeLists.txt             # ビルド設定
├── sdkconfig.defaults         # SDK デフォルト設定
└── partitions.csv             # パーティションテーブル
```

### コンポーネント一覧

#### HAL（Hardware Abstraction Layer）

| コンポーネント | 説明 |
|---------------|------|
| `sf_hal_bmi270` | IMU（加速度・ジャイロ）ドライバ |
| `sf_hal_bmm150` | 地磁気センサードライバ |
| `sf_hal_bmp280` | 気圧センサードライバ |
| `sf_hal_vl53l3cx` | ToFセンサードライバ |
| `sf_hal_pmw3901` | オプティカルフロードライバ |
| `sf_hal_motor` | モータードライバ（PWM制御） |
| `sf_hal_led` | WS2812 LEDドライバ |
| `sf_hal_buzzer` | ブザードライバ |
| `sf_hal_button` | ボタン入力ドライバ |
| `sf_hal_power` | 電源監視（INA3221） |

#### Algorithm

| コンポーネント | 説明 |
|---------------|------|
| `sf_algo_eskf` | Error-State Kalman Filter（位置姿勢推定） |
| `sf_algo_fusion` | センサーフュージョン管理 |
| `sf_algo_pid` | PID制御器（不完全微分付き） |
| `sf_algo_filter` | LPFなどのフィルタ |
| `sf_algo_math` | 数学ユーティリティ（ベクトル、クォータニオン） |

#### Service

| コンポーネント | 説明 |
|---------------|------|
| `sf_svc_state` | StampFlyState（状態管理シングルトン） |
| `sf_svc_comm` | ESP-NOW コントローラ通信 |
| `sf_svc_cli` | CLIコマンド処理 |
| `sf_svc_led` | LEDManager（優先度付きLED制御） |
| `sf_svc_logger` | バイナリログ記録 |
| `sf_svc_telemetry` | WebSocketテレメトリ |
| `sf_svc_health` | センサーヘルス監視 |

### タスク構成

FreeRTOS のデュアルコア構成を活用しています。

#### Core 1（高速リアルタイムタスク）

| タスク | 周期 | 優先度 | 説明 |
|--------|------|--------|------|
| IMUTask | 400Hz | 24 | IMU読み取り、ESKF更新 |
| ControlTask | 400Hz | 23 | 角速度制御、モーター出力 |
| OptFlowTask | 100Hz | 20 | オプティカルフロー読み取り |

#### Core 0（周辺・通信タスク）

| タスク | 周期 | 優先度 | 説明 |
|--------|------|--------|------|
| MagTask | 100Hz | 18 | 地磁気センサー |
| BaroTask | 50Hz | 16 | 気圧センサー |
| CommTask | 50Hz | 15 | ESP-NOW通信 |
| ToFTask | 30Hz | 14 | ToFセンサー |
| TelemetryTask | 50Hz | 13 | WebSocket送信 |
| PowerTask | 10Hz | 12 | 電源監視 |
| ButtonTask | 50Hz | 10 | ボタン入力 |
| LEDTask | 50Hz | 8 | LED更新 |
| CLITask | - | 5 | コマンド処理 |

### 起動シーケンス

```
app_main()
    │
    ├─ Phase 1: 機体を地面に置く (白色LED・3秒)
    │
    ├─ Phase 2: センサー初期化 (青色LED)
    │   ├─ I2C初期化
    │   ├─ アクチュエータ初期化（モーター、LED、ブザー）
    │   ├─ センサー初期化（IMU、Mag、Baro、ToF、OptFlow）
    │   ├─ 推定器初期化（ESKF、LPF）
    │   ├─ 通信初期化（ESP-NOW）
    │   ├─ CLI初期化
    │   ├─ Logger初期化
    │   └─ Telemetry初期化
    │
    ├─ タスク開始
    │
    ├─ Phase 3: センサー安定化待機 (マゼンタ点滅)
    │   └─ 全センサーの標準偏差が閾値以下になるまで待機
    │
    └─ Phase 4: 準備完了 (緑色LED・ビープ音)
        └─ IDLE状態へ遷移
```

---

## 4. 状態管理

### フライト状態

StampFlyState クラスが機体の状態を一元管理します。

```
INIT ─────────┐
              │ 初期化完了
              ▼
IDLE ◄────── ARMED
 │            │ ▲
 │            │ │
 │ ARM要求    │ │ DISARM要求
 │            │ │
 └──────────► │ │
              ▼ │
           FLYING
              │
              │ エラー検出
              ▼
           ERROR
```

| 状態 | 説明 | LED色 |
|------|------|-------|
| INIT | 初期化中 | 青点滅 |
| IDLE | 待機中 | 緑点灯 |
| ARMED | アーム済み（モーター有効） | 橙点灯 |
| FLYING | 飛行中 | 橙点滅 |
| ERROR | エラー | 赤点滅 |

### ARM/DISARM 操作

**ボタンによる操作：**
- IDLE状態でクリック → ARM
- ARMED状態でクリック → DISARM

**コントローラによる操作：**
- ARM フラグの立ち上がりエッジで状態遷移

**自動DISARM：**
- 衝撃検出時（加速度 > 3G または 角速度 > 800°/s）

### ペアリングモード

ボタン長押し（3秒）でペアリングモードに入ります。
- 青色高速点滅
- コントローラからのペアリング要求を受け付け

---

## 5. センサーフュージョン（ESKF）

### ESKF の概要

Error-State Kalman Filter を使用して、複数のセンサー情報を統合し、
機体の位置・速度・姿勢を高精度に推定します。

**推定状態（15次元）：**
- 位置 (x, y, z) - 3次元
- 速度 (vx, vy, vz) - 3次元
- 姿勢 (クォータニオン誤差) - 3次元
- ジャイロバイアス (bx, by, bz) - 3次元
- 加速度バイアス (ax, ay, az) - 3次元

### センサー入力

| センサー | 用途 | 更新レート |
|----------|------|------------|
| IMU（加速度・ジャイロ） | 予測ステップ | 400Hz |
| 地磁気 | ヨー補正 | 100Hz |
| 気圧 | 高度補正 | 50Hz |
| ToF | 高度補正（低高度） | 30Hz |
| オプティカルフロー | 水平速度補正 | 100Hz |

### パラメータ調整

すべてのパラメータは `config.hpp` の `namespace config::eskf` に集約されています。

```cpp
namespace eskf {
    // プロセスノイズ（値が大きい = センサーを信頼）
    inline constexpr float GYRO_NOISE = 0.009655f;
    inline constexpr float ACCEL_NOISE = 0.062885f;

    // 観測ノイズ（値が大きい = 観測を信頼しない）
    inline constexpr float BARO_NOISE = 0.1f;
    inline constexpr float TOF_NOISE = 0.002540f;
    inline constexpr float MAG_NOISE = 1.0f;
    inline constexpr float FLOW_NOISE = 0.01f;
}
```

---

## 6. 制御系の実装

### 制御タスクの構造

制御は `control_task.cpp` で実装されています。
現在は角速度制御（Rate Control）のみが実装されています。

```
コントローラ入力（スティック）
        │
        ▼
  目標角速度計算
        │
        ▼
   PID制御器        ◄─── 現在の角速度（IMUから）
        │
        ▼
  モーターミキサー
        │
        ▼
   モーター出力
```

### モーター配置

```
               Front
          FL (M4)   FR (M1)
             ╲   ▲   ╱
              ╲  │  ╱
               ╲ │ ╱
                ╲│╱
                 ╳         ← 機体中心
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲
             ╱   │   ╲
          RL (M3)    RR (M2)
                Rear

モーター回転方向:
  M1 (FR): CCW（反時計回り）
  M2 (RR): CW（時計回り）
  M3 (RL): CCW（反時計回り）
  M4 (FL): CW（時計回り）
```

### PIDゲインの調整

`config.hpp` の `namespace config::rate_control` で設定します。

```cpp
namespace rate_control {
    // Roll rate PID
    inline constexpr float ROLL_RATE_KP = 0.65f;
    inline constexpr float ROLL_RATE_TI = 0.7f;   // 積分時間 [s]
    inline constexpr float ROLL_RATE_TD = 0.01f;  // 微分時間 [s]

    // Pitch rate PID
    inline constexpr float PITCH_RATE_KP = 0.95f;
    inline constexpr float PITCH_RATE_TI = 0.7f;
    inline constexpr float PITCH_RATE_TD = 0.025f;

    // Yaw rate PID
    inline constexpr float YAW_RATE_KP = 3.0f;
    inline constexpr float YAW_RATE_TI = 0.8f;
    inline constexpr float YAW_RATE_TD = 0.01f;
}
```

### 独自の制御を実装する場合

`control_task.cpp` を編集して、独自の制御則を実装できます。

```cpp
// 例：姿勢制御（アングル制御）を追加する場合

// 1. 目標姿勢を計算
float roll_angle_target = roll_cmd * MAX_ROLL_ANGLE;
float pitch_angle_target = pitch_cmd * MAX_PITCH_ANGLE;

// 2. 現在の姿勢を取得（ESKFから）
float roll_current, pitch_current, yaw_current;
g_fusion.getEulerAngles(roll_current, pitch_current, yaw_current);

// 3. 姿勢制御PIDで目標角速度を計算
float roll_rate_target = attitude_pid.update(roll_angle_target, roll_current, dt);

// 4. 角速度制御（既存コード）
float roll_out = g_rate_controller.roll_pid.update(roll_rate_target, roll_rate_current, dt);
```

---

## 7. CLI コマンド

USBシリアル経由でCLIコマンドが使用できます。

### 基本コマンド

| コマンド | 説明 |
|----------|------|
| `help` | コマンド一覧を表示 |
| `status` | システム状態を表示 |
| `reboot` | システムを再起動 |

### センサーコマンド

| コマンド | 説明 |
|----------|------|
| `imu` | IMUの現在値を表示 |
| `mag` | 地磁気センサーの現在値を表示 |
| `baro` | 気圧センサーの現在値を表示 |
| `tof` | ToFセンサーの現在値を表示 |
| `flow` | オプティカルフローの現在値を表示 |
| `power` | 電源状態を表示 |

### 制御コマンド

| コマンド | 説明 |
|----------|------|
| `arm` | モーターをアーム |
| `disarm` | モーターをディスアーム |
| `motor <n> <duty>` | モーター n (1-4) を duty% で回転 |

### ログコマンド

| コマンド | 説明 |
|----------|------|
| `log start` | バイナリログ記録開始 |
| `log stop` | バイナリログ記録停止 |
| `log status` | ログ状態を表示 |
| `loglevel <level>` | ログレベルを設定（none/error/warn/info/debug） |

### キャリブレーションコマンド

| コマンド | 説明 |
|----------|------|
| `magcal start` | 地磁気キャリブレーション開始 |
| `magcal stop` | 地磁気キャリブレーション停止 |
| `magcal status` | キャリブレーション状態を表示 |

---

## 8. テレメトリとログ

### WebSocket テレメトリ

ESP-NOWと同時にWiFiアクセスポイントを起動し、WebSocketでテレメトリデータを配信します。

- SSID: `StampFly_XXXXXX`
- ポート: 80
- レート: 50Hz

### バイナリログ

400Hz でセンサーデータとESKF状態をバイナリ形式で記録します。

```bash
# ログ取得（Python）
python tools/scripts/log_capture.py

# ログ可視化
python tools/scripts/viz_all.py logs/flight_001.bin
```

---

## 9. GPIO 割り当て

### SPI バス

| GPIO | 機能 |
|------|------|
| 14 | MOSI |
| 43 | MISO |
| 44 | SCK |
| 46 | IMU CS |
| 12 | OptFlow CS |

### I2C バス

| GPIO | 機能 |
|------|------|
| 3 | SDA |
| 4 | SCL |

### モーター（PWM）

| GPIO | モーター | 位置 | 回転 |
|------|---------|------|------|
| 42 | M1 | FR (右前) | CCW |
| 41 | M2 | RR (右後) | CW |
| 10 | M3 | RL (左後) | CCW |
| 5 | M4 | FL (左前) | CW |

### その他

| GPIO | 機能 |
|------|------|
| 21 | MCU内蔵LED |
| 39 | ボード上LED（2個直列） |
| 40 | ブザー |
| 0 | ボタン |
| 7 | ToF XSHUT (底面) |
| 9 | ToF XSHUT (前方) |

---

## 10. トラブルシューティング

### ビルドエラー

**ESP-IDF が見つからない：**
```bash
# 環境変数を設定
. ~/esp/esp-idf/export.sh
```

**コンポーネントが見つからない：**
```bash
# managed_components を削除して再ビルド
rm -rf managed_components
idf.py build
```

### 書き込みエラー

**ポートが見つからない：**
```bash
# デバイスを確認
ls /dev/tty.usb*  # macOS
ls /dev/ttyUSB*   # Linux
```

**書き込みモードに入れない：**
1. ボタンを押しながらUSBを接続
2. `idf.py flash` を実行

### センサーが初期化されない

- I2C接続を確認（SDA/SCL）
- センサーの電源を確認
- `status` コマンドでセンサー状態を確認

---

## 11. 参考資料

### 関連リポジトリ

| リポジトリ | 説明 |
|-----------|------|
| [StampFly技術仕様](https://github.com/M5Fly-kanazawa/StampFly_technical_specification) | ハードウェア仕様書 |
| [IMUドライバ](https://github.com/kouhei1970/stampfly_imu) | BMI270 ドライバ |
| [ToFドライバ](https://github.com/kouhei1970/stampfly_tof) | VL53L3CX ドライバ |
| [OptFlowドライバ](https://github.com/kouhei1970/stampfly_opticalflow) | PMW3901 ドライバ |
| [ESKF推定器](https://github.com/kouhei1970/stampfly-eskf-estimator) | 位置姿勢推定ライブラリ |
| [コントローラ](https://github.com/M5Fly-kanazawa/Simple_StampFly_Joy) | 送信機ファームウェア（for_tdmaブランチ） |

### 設計ドキュメント

- `PLAN.md` - 本プロジェクトの設計方針

---

<a id="english"></a>

## 1. Overview

### About This Project

StampFly Vehicle Firmware is a flight controller firmware for the StampFly drone.
This firmware is designed as a **skeleton**, providing all the foundation features
such as sensor reading, state estimation, and communication for engineers and students
who want to implement their own flight control.

**Skeleton includes:**
- Sensor reading and calibration
- Centralized StampFly state management
- Controller communication
- CLI for sensor value retrieval and gain adjustment
- Pose estimation (ESKF: Error-State Kalman Filter)

With these already implemented, users only need to implement their control logic in `control_task.cpp` to fly.

### Target Hardware

- **MCU**: M5Stamp S3 (ESP32-S3)
- **Framework**: ESP-IDF v5.4.1 + FreeRTOS
- **Airframe**: StampFly

### Main Features

| Feature | Description |
|---------|-------------|
| 6-axis IMU | BMI270 (accel/gyro) 400Hz |
| Magnetometer | BMM150 100Hz |
| Barometer | BMP280 50Hz |
| ToF Sensor | VL53L3CX (bottom/front) 30Hz |
| Optical Flow | PMW3901 100Hz |
| Power Monitor | INA3221 10Hz |
| Pose Estimation | ESKF (Error-State Kalman Filter) |
| Communication | ESP-NOW (controller) + WebSocket (telemetry) |
| CLI | Command line via USB serial |
| LED | WS2812 RGB LED (status indication) |
| Buzzer | Status notification |

---

## 2. Development Environment Setup

### Required Software

1. **ESP-IDF v5.4.1**
   - [Official Installation Guide](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/get-started/index.html)
   - ESP-IDF Tools Installer recommended for Windows
   - Use `install.sh` for macOS/Linux

2. **USB Driver**
   - M5Stamp S3 uses USB Serial/JTAG
   - Usually not required on macOS/Linux
   - CP210x driver may be needed on Windows

### ESP-IDF Setup (macOS/Linux)

```bash
# Clone ESP-IDF
mkdir -p ~/esp
cd ~/esp
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git

# Run install script
cd esp-idf
./install.sh esp32s3

# Set environment variables (run each time or add to .bashrc/.zshrc)
. ~/esp/esp-idf/export.sh
```

### Building the Project

```bash
# Navigate to project directory
cd firmware/vehicle

# Build
idf.py build

# Flash
idf.py flash

# Monitor (view logs)
idf.py monitor

# Build, flash, and monitor in one command
idf.py build flash monitor
```

### Specifying Serial Port

If device is not auto-detected:

```bash
# Specify port for flashing
idf.py -p /dev/tty.usbmodem* flash monitor

# For Windows
idf.py -p COM3 flash monitor
```

---

## 3. Software Architecture

### Directory Structure

```
firmware/vehicle/
├── main/                      # Main application
│   ├── main.cpp               # Entry point, initialization sequence
│   ├── config.hpp             # All config parameters (GPIO, task priority, ESKF, etc.)
│   ├── globals.hpp/.cpp       # Global variable declarations/definitions
│   ├── init.hpp/.cpp          # Initialization functions
│   ├── rate_controller.hpp    # Rate controller definition
│   └── tasks/                 # FreeRTOS tasks
│       ├── tasks.hpp          # Task function prototypes
│       ├── imu_task.cpp       # IMU reading, sensor fusion (400Hz)
│       ├── control_task.cpp   # Control loop (400Hz) ★User implementation
│       ├── mag_task.cpp       # Magnetometer (100Hz)
│       ├── baro_task.cpp      # Barometer (50Hz)
│       ├── tof_task.cpp       # ToF sensor (30Hz)
│       ├── optflow_task.cpp   # Optical flow (100Hz)
│       ├── comm_task.cpp      # ESP-NOW communication (50Hz)
│       ├── power_task.cpp     # Power monitoring (10Hz)
│       ├── led_task.cpp       # LED control (50Hz)
│       ├── button_task.cpp    # Button monitoring (50Hz)
│       ├── cli_task.cpp       # CLI command processing
│       └── telemetry_task.cpp # WebSocket telemetry (50Hz)
├── components/                # ESP-IDF components
│   ├── sf_hal_*/              # Hardware abstraction layer (sensor drivers)
│   ├── sf_algo_*/             # Algorithms (ESKF, PID, filters)
│   └── sf_svc_*/              # Services (state management, communication, CLI)
├── tools/                     # Development tools
│   ├── scripts/               # Python analysis scripts
│   └── eskf_debug/            # ESKF debug PC simulator
├── logs/                      # Flight log storage
├── CMakeLists.txt             # Build configuration
├── sdkconfig.defaults         # SDK default settings
└── partitions.csv             # Partition table
```

### Task Configuration

Utilizes FreeRTOS dual-core configuration.

### Core 1 (High-speed Real-time Tasks)

| Task | Rate | Priority | Description |
|------|------|----------|-------------|
| IMUTask | 400Hz | 24 | IMU reading, ESKF update |
| ControlTask | 400Hz | 23 | Rate control, motor output |
| OptFlowTask | 100Hz | 20 | Optical flow reading |

### Core 0 (Peripheral/Communication Tasks)

| Task | Rate | Priority | Description |
|------|------|----------|-------------|
| MagTask | 100Hz | 18 | Magnetometer |
| BaroTask | 50Hz | 16 | Barometer |
| CommTask | 50Hz | 15 | ESP-NOW communication |
| ToFTask | 30Hz | 14 | ToF sensor |
| TelemetryTask | 50Hz | 13 | WebSocket transmission |
| PowerTask | 10Hz | 12 | Power monitoring |
| ButtonTask | 50Hz | 10 | Button input |
| LEDTask | 50Hz | 8 | LED update |
| CLITask | - | 5 | Command processing |

---

## 4. Control System Implementation

### Control Task Structure

Control is implemented in `control_task.cpp`.
Currently, only rate control (angular velocity control) is implemented.

```
Controller Input (sticks)
        │
        ▼
  Target Rate Calculation
        │
        ▼
   PID Controller      ◄─── Current Rate (from IMU)
        │
        ▼
  Motor Mixer
        │
        ▼
   Motor Output
```

### Motor Layout

```
               Front
          FL (M4)   FR (M1)
             ╲   ▲   ╱
              ╲  │  ╱
               ╲ │ ╱
                ╲│╱
                 ╳         ← Center
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲
             ╱   │   ╲
          RL (M3)    RR (M2)
                Rear

Motor Rotation:
  M1 (FR): CCW (Counter-Clockwise)
  M2 (RR): CW (Clockwise)
  M3 (RL): CCW (Counter-Clockwise)
  M4 (FL): CW (Clockwise)
```

### Implementing Custom Control

Edit `control_task.cpp` to implement your own control logic.

```cpp
// Example: Adding attitude control (angle control)

// 1. Calculate target attitude
float roll_angle_target = roll_cmd * MAX_ROLL_ANGLE;
float pitch_angle_target = pitch_cmd * MAX_PITCH_ANGLE;

// 2. Get current attitude (from ESKF)
float roll_current, pitch_current, yaw_current;
g_fusion.getEulerAngles(roll_current, pitch_current, yaw_current);

// 3. Calculate target rate with attitude PID
float roll_rate_target = attitude_pid.update(roll_angle_target, roll_current, dt);

// 4. Rate control (existing code)
float roll_out = g_rate_controller.roll_pid.update(roll_rate_target, roll_rate_current, dt);
```

---

## 5. References

### Related Repositories

| Repository | Description |
|-----------|-------------|
| [StampFly Technical Spec](https://github.com/M5Fly-kanazawa/StampFly_technical_specification) | Hardware specifications |
| [IMU Driver](https://github.com/kouhei1970/stampfly_imu) | BMI270 driver |
| [ToF Driver](https://github.com/kouhei1970/stampfly_tof) | VL53L3CX driver |
| [OptFlow Driver](https://github.com/kouhei1970/stampfly_opticalflow) | PMW3901 driver |
| [ESKF Estimator](https://github.com/kouhei1970/stampfly-eskf-estimator) | Pose estimation library |
| [Controller](https://github.com/M5Fly-kanazawa/Simple_StampFly_Joy) | Transmitter firmware (for_tdma branch) |

### Design Documents

- `PLAN.md` - Design policy for this project