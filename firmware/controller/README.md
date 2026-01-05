# StampFly コントローラ ファームウェア

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

StampFly コントローラは、M5Stack AtomS3とAtom JoyStickを使用したStampFly操縦用ファームウェアです。ESP-NOW通信でドローンと接続し、TDMA (Time Division Multiple Access) 方式により複数のコントローラが同一チャンネルで干渉なく通信できます。

### 主な機能

- **ジョイスティック入力**: 2軸×2本のアナログスティックでスロットル・ロール・ピッチ・ヨーを制御
- **ESP-NOW通信**: 低遅延の無線通信プロトコル
- **TDMA同期**: 最大10台のコントローラが同一チャンネルで動作可能
- **ペアリング機能**: ボタン操作でドローンと自動ペアリング
- **LCD表示**: 通信状態・バッテリー電圧・モード表示
- **ブザー音**: 状態通知・警告音

### ハードウェア構成

| コンポーネント | 型番 | 説明 |
|---------------|------|------|
| マイコン | M5Stack AtomS3 | ESP32-S3搭載、LCD付き |
| ジョイスティック | M5Stack Atom JoyStick | I2C接続、2軸×2本 |
| バッテリー | - | JoyStick内蔵 |

## 2. ディレクトリ構成

```
firmware/controller/
├── CMakeLists.txt              # プロジェクト設定
├── sdkconfig                   # SDK設定
├── sdkconfig.defaults          # SDK設定デフォルト値
├── partitions.csv              # パーティションテーブル
├── main/
│   ├── CMakeLists.txt          # mainコンポーネント設定
│   ├── idf_component.yml       # IDF Component Manager設定
│   └── main.cpp                # メインプログラム
├── components/
│   ├── atoms3joy/              # ジョイスティックドライバ
│   │   ├── include/atoms3joy.h
│   │   └── atoms3joy.c
│   ├── buzzer/                 # ブザー制御
│   │   ├── include/buzzer.h
│   │   └── buzzer.c
│   └── espnow_tdma/            # ESP-NOW TDMA通信
│       ├── include/espnow_tdma.h
│       └── espnow_tdma.c
├── lib/
│   ├── ATOMS3Joy/              # AtomS3Joy追加ライブラリ
│   └── FastLED/                # LED制御ライブラリ
├── managed_components/         # IDF Component Manager管理
├── docs/                       # ドキュメント
├── TDMA_USAGE.md               # TDMA詳細ガイド
└── LICENSE                     # MITライセンス
```

## 3. 開発環境のセットアップ

### 必要な環境

- **ESP-IDF**: v5.4.1
- **対応OS**: Windows、macOS、Linux
- **ハードウェア**: M5Stack AtomS3 + Atom JoyStick

### ESP-IDF のインストール

```bash
# リポジトリをクローン
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf

# インストールスクリプトを実行
cd ~/esp/esp-idf
./install.sh esp32s3

# 環境変数を設定
. ~/esp/esp-idf/export.sh
```

### プロジェクトのビルド

```bash
cd firmware/controller
idf.py build
```

### 書き込み

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

macOSの場合:
```bash
idf.py -p /dev/cu.usbmodem* flash monitor
```

## 4. タスク構成

本ファームウェアは複数のFreeRTOSタスクで構成されています。

### タスク一覧

| タスク名 | 周期 | 優先度 | Core | 役割 |
|---------|------|-------|------|------|
| InputTask | 100Hz | 最高-2 | 1 | ジョイスティック読み取り |
| DisplayTask | 10Hz | 2 | 1 | LCD表示更新 |
| BeaconTask | (通知) | 最高-1 | 1 | ビーコン送信 (マスターのみ) |
| TDMASendTask | (通知) | 最高-1 | 1 | 制御データ送信 |
| メインループ | 100Hz | - | 0 | 制御値計算・データ作成 |

### タスクフロー

```
┌──────────────┐     ┌──────────────┐
│  InputTask   │────▶│ shared_input │
│  (100Hz)     │     │   (Mutex)    │
└──────────────┘     └──────────────┘
                            │
                            ▼
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Main Loop   │────▶│ shared_send  │────▶│ TDMASendTask │
│  (100Hz)     │     │   (Mutex)    │     │   (TDMA)     │
└──────────────┘     └──────────────┘     └──────────────┘
                                                 │
                                                 ▼
                                          ┌──────────────┐
                                          │   ESP-NOW    │
                                          │  to Drone    │
                                          └──────────────┘
```

## 5. ESP-NOW TDMA通信

### TDMA方式の概要

TDMA (Time Division Multiple Access) は、時間を複数のスロットに分割し、各デバイスが決められたスロットで送信することで衝突を回避する方式です。

```
フレーム (20ms = 50Hz)
├─ ビーコン (フレーム開始500μs前、マスターのみ送信)
├─ スロット0 (2ms) 親機 ID=0 の制御データ
├─ スロット1 (2ms) 子機 ID=1 の制御データ
├─ スロット2 (2ms) 子機 ID=2 の制御データ
├─ ...
└─ スロット9 (2ms) 子機 ID=9 の制御データ
```

### パラメータ設定

`components/espnow_tdma/include/espnow_tdma.h`:

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `ESPNOW_CHANNEL` | 1 | WiFiチャンネル (1-13) |
| `TDMA_DEVICE_ID` | 0 | デバイスID (0=マスター, 1-9=スレーブ) |
| `TDMA_FRAME_US` | 20000 | フレーム周期 (20ms) |
| `TDMA_SLOT_US` | 2000 | スロット幅 (2ms) |
| `TDMA_NUM_SLOTS` | 10 | スロット数 |
| `TDMA_BEACON_ADVANCE_US` | 500 | ビーコン先行時間 |

### 複数コントローラの設定

複数のコントローラを使用する場合、各コントローラに異なる `TDMA_DEVICE_ID` を設定します。

**親機 (1台のみ)**:
```c
#define TDMA_DEVICE_ID 0
```

**子機 1**:
```c
#define TDMA_DEVICE_ID 1
```

**子機 2**:
```c
#define TDMA_DEVICE_ID 2
```

詳細は [TDMA_USAGE.md](TDMA_USAGE.md) を参照してください。

## 6. ジョイスティック

### ハードウェア

Atom JoyStick はI2C接続 (アドレス: 0x59) で、以下のデータを提供します。

| データ | レジスタ | 説明 |
|--------|---------|------|
| 左スティックX | 0x00-0x01 | 12bit (0-4095) |
| 左スティックY | 0x02-0x03 | 12bit (0-4095) |
| 右スティックX | 0x20-0x21 | 12bit (0-4095) |
| 右スティックY | 0x22-0x23 | 12bit (0-4095) |
| 左スティックボタン | 0x70 | 0/1 |
| 右スティックボタン | 0x71 | 0/1 |
| 左ボタン | 0x72 | 0/1 |
| 右ボタン | 0x73 | 0/1 |
| バッテリー電圧1 | 0x60-0x61 | mV単位 |
| バッテリー電圧2 | 0x62-0x63 | mV単位 |

### スティックモード

| モード | スロットル | エルロン | エレベータ | ラダー |
|-------|-----------|---------|-----------|--------|
| Mode 2 | 左Y | 右X | 右Y | 左X |
| Mode 3 | 右Y | 左X | 左Y | 右X |

起動時に左ボタンを押しながら電源を入れるとMode 3、それ以外はMode 2になります。

### バイアスキャリブレーション

起動時に50サンプルの平均を取り、スティック中立位置のバイアスを自動補正します。

## 7. 操作方法

### ペアリング

1. コントローラのM5ボタンを押しながら電源を入れる
2. LCD に "Pairing mode..." と表示される
3. StampFly のボタンを長押ししてペアリングモードに入る
4. ペアリングが完了するとビープ音が鳴る
5. ペアリング情報はSPIFFSに保存され、次回起動時は自動接続

### ボタン操作

| ボタン | 短押し | 長押し (400ms以上) |
|-------|-------|-------------------|
| M5ボタン | タイマー開始/停止 | タイマーリセット |
| 左スティックボタン | Arm/Disarm | - |
| 右スティックボタン | Flip | - |
| 左ボタン | 高度モード切替 | - |
| 右ボタン | 制御モード切替 | - |

### 制御モード

| モード | 説明 |
|-------|------|
| STABILIZE | 角度制御モード (初心者向け) |
| ACRO | 角速度制御モード (上級者向け) |

### 高度モード

| モード | 説明 |
|-------|------|
| Manual ALT | スロットルで直接高度を制御 |
| Auto ALT | 自動高度維持モード |

## 8. 制御パケットフォーマット

コントローラからドローンへ送信されるパケット (14バイト):

| バイト | 内容 | 説明 |
|-------|------|------|
| 0-2 | MAC下位3バイト | 宛先識別用 |
| 3-4 | Throttle | スロットル (16bit) |
| 5-6 | Phi | ロール角 (16bit) |
| 7-8 | Theta | ピッチ角 (16bit) |
| 9-10 | Psi | ヨー角 (16bit) |
| 11 | フラグ | Arm/Flip/Mode/AltMode |
| 12 | proactive_flag | 予約 |
| 13 | チェックサム | バイト0-12の合計 |

フラグバイト (11) のビット配置:
```
bit 0: Arm ボタン
bit 1: Flip ボタン
bit 2: Mode (0=STABILIZE, 1=ACRO)
bit 3: AltMode (0=Manual, 1=Auto)
```

## 9. LCD表示

LCD は10Hzで更新され、以下の情報を表示します。

```
┌────────────────────┐
│ MAC ADR XX:YY      │  ← ドローンMACアドレス下位2バイト
│ BAT 1:X.X 2:X.X    │  ← バッテリー電圧
│ MODE: 2            │  ← スティックモード
│ CH: 01 ID: 0       │  ← WiFiチャンネル、デバイスID
│ -Mnual ALT-        │  ← 高度モード
│ -STABILIZE-        │  ← 制御モード
│ Freq:  50 M        │  ← 送信周波数 (M=Master/SYNC/WAIT/LOST)
└────────────────────┘
```

### 同期状態表示

| 表示 | 意味 |
|------|------|
| M | マスター (親機) |
| SYNC | スレーブ同期中 |
| WAIT | ビーコン待機中 |
| LOST | ビーコンロスト |

## 10. ブザー音

| 音パターン | 意味 |
|-----------|------|
| 起動音 (上昇音階) | 起動完了 |
| 単発ビープ | ペアリング待機中 |
| 4000Hz 単発 | ビーコンロスト警告 |
| 3000Hz 2回 | スロットタイミングエラー |
| 1000Hz 長音 | ドローンオフライン |
| 2000Hz 3回 | Mutexタイムアウト |

## 11. トラブルシューティング

### ドローンと接続できない

1. ペアリングを再実行する (M5ボタンを押しながら起動)
2. WiFiチャンネルがドローンと一致しているか確認
3. バッテリー電圧が十分か確認

### 子機が同期しない

1. 親機 (ID=0) が起動しているか確認
2. チャンネル設定が全デバイスで一致しているか確認
3. ビーコンロスト音 (4000Hz) が鳴っていないか確認

### スティックが効かない

1. I2C接続を確認
2. シリアルモニタでジョイスティック初期化エラーがないか確認
3. 起動時にスティックを中立位置にしているか確認

### ビルドエラー

1. ESP-IDF v5.4.1がインストールされているか確認
2. `export.sh` を実行して環境変数を設定
3. `idf.py fullclean` で完全にクリーンしてから再ビルド

## ライセンス

MIT License - 詳細は [LICENSE](LICENSE) を参照

## 作者

Kouhei Ito - kouhei.ito@itolab-ktc.com

---

<a id="english"></a>

# StampFly Controller Firmware

## 1. Overview

StampFly Controller is a firmware for controlling StampFly drones using M5Stack AtomS3 and Atom JoyStick. It connects to the drone via ESP-NOW communication and supports multiple controllers on the same channel without interference using TDMA (Time Division Multiple Access).

### Main Features

- **Joystick Input**: Control throttle, roll, pitch, and yaw with 2-axis × 2 analog sticks
- **ESP-NOW Communication**: Low-latency wireless protocol
- **TDMA Synchronization**: Up to 10 controllers can operate on the same channel
- **Pairing Function**: Automatic pairing with drone via button operation
- **LCD Display**: Shows communication status, battery voltage, mode
- **Buzzer**: Audio feedback for status and warnings

### Hardware Components

| Component | Model | Description |
|-----------|-------|-------------|
| MCU | M5Stack AtomS3 | ESP32-S3, with LCD |
| Joystick | M5Stack Atom JoyStick | I2C, 2-axis × 2 |
| Battery | - | Built into JoyStick |

## 2. Directory Structure

```
firmware/controller/
├── CMakeLists.txt              # Project configuration
├── sdkconfig                   # SDK configuration
├── sdkconfig.defaults          # SDK configuration defaults
├── partitions.csv              # Partition table
├── main/
│   ├── CMakeLists.txt          # Main component configuration
│   ├── idf_component.yml       # IDF Component Manager config
│   └── main.cpp                # Main program
├── components/
│   ├── atoms3joy/              # Joystick driver
│   │   ├── include/atoms3joy.h
│   │   └── atoms3joy.c
│   ├── buzzer/                 # Buzzer control
│   │   ├── include/buzzer.h
│   │   └── buzzer.c
│   └── espnow_tdma/            # ESP-NOW TDMA communication
│       ├── include/espnow_tdma.h
│       └── espnow_tdma.c
├── lib/
│   ├── ATOMS3Joy/              # AtomS3Joy additional library
│   └── FastLED/                # LED control library
├── managed_components/         # IDF Component Manager managed
├── docs/                       # Documentation
├── TDMA_USAGE.md               # TDMA detailed guide
└── LICENSE                     # MIT License
```

## 3. Development Environment Setup

### Requirements

- **ESP-IDF**: v5.4.1
- **Supported OS**: Windows, macOS, Linux
- **Hardware**: M5Stack AtomS3 + Atom JoyStick

### Installing ESP-IDF

```bash
# Clone the repository
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf

# Run the install script
cd ~/esp/esp-idf
./install.sh esp32s3

# Set environment variables
. ~/esp/esp-idf/export.sh
```

### Building the Project

```bash
cd firmware/controller
idf.py build
```

### Flashing

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

For macOS:
```bash
idf.py -p /dev/cu.usbmodem* flash monitor
```

## 4. Task Architecture

This firmware consists of multiple FreeRTOS tasks.

### Task List

| Task Name | Period | Priority | Core | Role |
|-----------|--------|----------|------|------|
| InputTask | 100Hz | MAX-2 | 1 | Joystick reading |
| DisplayTask | 10Hz | 2 | 1 | LCD update |
| BeaconTask | (notify) | MAX-1 | 1 | Beacon transmission (master only) |
| TDMASendTask | (notify) | MAX-1 | 1 | Control data transmission |
| Main Loop | 100Hz | - | 0 | Control value calculation |

### Task Flow

```
┌──────────────┐     ┌──────────────┐
│  InputTask   │────▶│ shared_input │
│  (100Hz)     │     │   (Mutex)    │
└──────────────┘     └──────────────┘
                            │
                            ▼
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Main Loop   │────▶│ shared_send  │────▶│ TDMASendTask │
│  (100Hz)     │     │   (Mutex)    │     │   (TDMA)     │
└──────────────┘     └──────────────┘     └──────────────┘
                                                 │
                                                 ▼
                                          ┌──────────────┐
                                          │   ESP-NOW    │
                                          │  to Drone    │
                                          └──────────────┘
```

## 5. ESP-NOW TDMA Communication

### TDMA Overview

TDMA (Time Division Multiple Access) divides time into multiple slots, with each device transmitting in its assigned slot to avoid collisions.

```
Frame (20ms = 50Hz)
├─ Beacon (500μs before frame start, master only)
├─ Slot 0 (2ms) Master ID=0 control data
├─ Slot 1 (2ms) Slave ID=1 control data
├─ Slot 2 (2ms) Slave ID=2 control data
├─ ...
└─ Slot 9 (2ms) Slave ID=9 control data
```

### Parameter Settings

`components/espnow_tdma/include/espnow_tdma.h`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ESPNOW_CHANNEL` | 1 | WiFi channel (1-13) |
| `TDMA_DEVICE_ID` | 0 | Device ID (0=master, 1-9=slave) |
| `TDMA_FRAME_US` | 20000 | Frame period (20ms) |
| `TDMA_SLOT_US` | 2000 | Slot width (2ms) |
| `TDMA_NUM_SLOTS` | 10 | Number of slots |
| `TDMA_BEACON_ADVANCE_US` | 500 | Beacon advance time |

### Multi-Controller Setup

When using multiple controllers, assign different `TDMA_DEVICE_ID` to each controller.

**Master (only 1)**:
```c
#define TDMA_DEVICE_ID 0
```

**Slave 1**:
```c
#define TDMA_DEVICE_ID 1
```

**Slave 2**:
```c
#define TDMA_DEVICE_ID 2
```

See [TDMA_USAGE.md](TDMA_USAGE.md) for details.

## 6. Joystick

### Hardware

Atom JoyStick connects via I2C (address: 0x59) and provides the following data:

| Data | Register | Description |
|------|----------|-------------|
| Left Stick X | 0x00-0x01 | 12bit (0-4095) |
| Left Stick Y | 0x02-0x03 | 12bit (0-4095) |
| Right Stick X | 0x20-0x21 | 12bit (0-4095) |
| Right Stick Y | 0x22-0x23 | 12bit (0-4095) |
| Left Stick Button | 0x70 | 0/1 |
| Right Stick Button | 0x71 | 0/1 |
| Left Button | 0x72 | 0/1 |
| Right Button | 0x73 | 0/1 |
| Battery Voltage 1 | 0x60-0x61 | mV unit |
| Battery Voltage 2 | 0x62-0x63 | mV unit |

### Stick Modes

| Mode | Throttle | Aileron | Elevator | Rudder |
|------|----------|---------|----------|--------|
| Mode 2 | Left Y | Right X | Right Y | Left X |
| Mode 3 | Right Y | Left X | Left Y | Right X |

Hold the left button while powering on for Mode 3, otherwise Mode 2.

### Bias Calibration

At startup, 50 samples are averaged to automatically calibrate the stick neutral position bias.

## 7. Operation

### Pairing

1. Hold M5 button while powering on the controller
2. LCD shows "Pairing mode..."
3. Long-press the StampFly button to enter pairing mode
4. Beep sounds when pairing is complete
5. Pairing info is saved to SPIFFS and auto-connects on next boot

### Button Operations

| Button | Short Press | Long Press (400ms+) |
|--------|-------------|---------------------|
| M5 Button | Timer start/stop | Timer reset |
| Left Stick Button | Arm/Disarm | - |
| Right Stick Button | Flip | - |
| Left Button | Altitude mode toggle | - |
| Right Button | Control mode toggle | - |

### Control Modes

| Mode | Description |
|------|-------------|
| STABILIZE | Angle control mode (beginner-friendly) |
| ACRO | Rate control mode (advanced) |

### Altitude Modes

| Mode | Description |
|------|-------------|
| Manual ALT | Direct altitude control with throttle |
| Auto ALT | Automatic altitude hold mode |

## 8. Control Packet Format

Packet sent from controller to drone (14 bytes):

| Byte | Content | Description |
|------|---------|-------------|
| 0-2 | MAC lower 3 bytes | Destination ID |
| 3-4 | Throttle | Throttle value (16bit) |
| 5-6 | Phi | Roll angle (16bit) |
| 7-8 | Theta | Pitch angle (16bit) |
| 9-10 | Psi | Yaw angle (16bit) |
| 11 | Flags | Arm/Flip/Mode/AltMode |
| 12 | proactive_flag | Reserved |
| 13 | Checksum | Sum of bytes 0-12 |

Flag byte (11) bit layout:
```
bit 0: Arm button
bit 1: Flip button
bit 2: Mode (0=STABILIZE, 1=ACRO)
bit 3: AltMode (0=Manual, 1=Auto)
```

## 9. LCD Display

LCD updates at 10Hz showing the following information:

```
┌────────────────────┐
│ MAC ADR XX:YY      │  ← Drone MAC address lower 2 bytes
│ BAT 1:X.X 2:X.X    │  ← Battery voltage
│ MODE: 2            │  ← Stick mode
│ CH: 01 ID: 0       │  ← WiFi channel, device ID
│ -Mnual ALT-        │  ← Altitude mode
│ -STABILIZE-        │  ← Control mode
│ Freq:  50 M        │  ← Transmit frequency (M=Master/SYNC/WAIT/LOST)
└────────────────────┘
```

### Sync Status Display

| Display | Meaning |
|---------|---------|
| M | Master |
| SYNC | Slave synchronized |
| WAIT | Waiting for beacon |
| LOST | Beacon lost |

## 10. Buzzer Sounds

| Sound Pattern | Meaning |
|---------------|---------|
| Startup (ascending scale) | Boot complete |
| Single beep | Pairing waiting |
| 4000Hz single | Beacon lost warning |
| 3000Hz × 2 | Slot timing error |
| 1000Hz long | Drone offline |
| 2000Hz × 3 | Mutex timeout |

## 11. Troubleshooting

### Cannot Connect to Drone

1. Re-run pairing (hold M5 button while booting)
2. Verify WiFi channel matches the drone
3. Check battery voltage is sufficient

### Slave Not Synchronizing

1. Verify master (ID=0) is running
2. Confirm channel settings match across all devices
3. Check if beacon lost sound (4000Hz) is heard

### Sticks Not Responding

1. Check I2C connection
2. Check serial monitor for joystick initialization errors
3. Ensure sticks are in neutral position at startup

### Build Errors

1. Verify ESP-IDF v5.4.1 is installed
2. Run `export.sh` to set environment variables
3. Run `idf.py fullclean` and rebuild

## License

MIT License - See [LICENSE](LICENSE) for details

## Author

Kouhei Ito - kouhei.ito@itolab-ktc.com
