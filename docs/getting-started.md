# はじめに

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

このドキュメントでは、StampFly エコシステムの環境構築から初飛行までの手順を解説します。

## 1. 必要なもの

### ハードウェア

| 項目 | 型番 | 備考 |
|-----|------|------|
| StampFly 機体 | - | M5Stamp S3 搭載 |
| コントローラ | M5Stack AtomS3 + Atom JoyStick | セットで使用 |
| USB-Cケーブル | - | 書き込み用 × 2本 |
| LiPoバッテリー | 1S 3.7V | 機体用 |

### ソフトウェア

| 項目 | バージョン | 備考 |
|-----|----------|------|
| ESP-IDF | v5.4.1 | 必須 |
| Git | 最新版 | - |
| Python | 3.8以上 | ESP-IDF依存 |

## 2. ESP-IDF のインストール

### macOS / Linux

```bash
# 1. リポジトリをクローン
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf

# 2. インストールスクリプトを実行
cd ~/esp/esp-idf
./install.sh esp32s3

# 3. 環境変数を設定（毎回実行、または .bashrc/.zshrc に追加）
. ~/esp/esp-idf/export.sh
```

### Windows

1. [ESP-IDF Tools Installer](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/get-started/windows-setup.html) をダウンロード
2. インストーラを実行し、ESP-IDF v5.4.1 を選択
3. "ESP-IDF PowerShell" または "ESP-IDF Command Prompt" を使用

### インストール確認

```bash
idf.py --version
# 出力例: ESP-IDF v5.4.1
```

## 3. リポジトリのクローン

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
```

## 4. 機体ファームウェアのビルドと書き込み

### ビルド

```bash
cd firmware/vehicle
idf.py build
```

初回ビルドには数分かかります。`build/firmware_vehicle.bin` が生成されれば成功です。

### 書き込み

StampFly 機体をUSBで接続し、以下を実行:

```bash
# Linux
idf.py -p /dev/ttyACM0 flash

# macOS
idf.py -p /dev/cu.usbmodem* flash

# Windows
idf.py -p COM3 flash
```

ポート名は環境によって異なります。`ls /dev/tty*` (Linux/macOS) や デバイスマネージャー (Windows) で確認してください。

### シリアルモニタ

```bash
idf.py -p /dev/ttyACM0 monitor
```

終了は `Ctrl + ]` です。

## 5. コントローラファームウェアのビルドと書き込み

### ビルド

```bash
cd firmware/controller
idf.py build
```

`build/firmware_controller.bin` が生成されれば成功です。

### 書き込み

AtomS3 をUSBで接続し、以下を実行:

```bash
# Linux
idf.py -p /dev/ttyACM0 flash

# macOS
idf.py -p /dev/cu.usbmodem* flash

# Windows
idf.py -p COM3 flash
```

## 6. ペアリング

初回使用時、またはペアリング情報をリセットしたい場合に実行します。

### 手順

1. **コントローラ**: M5ボタン（画面下）を押しながら電源を入れる
2. LCD に "Pairing mode..." と表示され、ビープ音が鳴り始める
3. **StampFly**: ボタンを長押し（約2秒）してペアリングモードに入る
4. 両方からビープ音が鳴ればペアリング完了
5. ペアリング情報は自動保存され、次回以降は自動接続

## 7. 飛行前の確認

### チェックリスト

- [ ] バッテリーは十分に充電されているか（3.7V以上推奨）
- [ ] プロペラは正しく取り付けられているか
- [ ] 周囲に障害物がないか（最低2m四方の空間を確保）
- [ ] コントローラのスティックが中立位置にあるか

### スティックモード (Mode 2 / Mode 3)

StampFly は Mode 2 と Mode 3 に対応しています。

- **Mode 2**: 世界標準のスティック配置。多くのドローンやラジコンで採用されており、他の機体との操作感を統一したい場合に最適です。
- **Mode 3**: 作者が強く推奨するモード！スロットルが右手にあり、ゲームコントローラのような直感的な操作が可能です。ぜひ試してみてください。

#### モード切り替え方法

| 起動方法 | 選択されるモード |
|---------|----------------|
| 通常起動 | Mode 2 |
| **左ボタンを押しながら起動** | **Mode 3（推奨）** |

コントローラのLCDに `MODE: 2` または `MODE: 3` と表示されます。

## 8. 飛行方法

### スティック配置

#### Mode 2

```
        左スティック              右スティック
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │   スロットル  │          │   ピッチ    │
     │ ←ヨー    ヨー→│          │←ロール ロール→│
     │   (上昇)    │          │   (前進)    │
     │      ↓      │          │      ↓      │
     │   (下降)    │          │   (後退)    │
     └─────────────┘          └─────────────┘
```

#### Mode 3（推奨）

```
        左スティック              右スティック
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │   ピッチ     │          │   スロットル │
     │ ←ロール ロール→│          │ ←ヨー    ヨー→│
     │   (前進)    │          │   (上昇)    │
     │      ↓      │          │      ↓      │
     │   (後退)    │          │   (下降)    │
     └─────────────┘          └─────────────┘
```

Mode 3 では、右手でスロットル（高度）を操作し、左手で機体の姿勢を制御します。ゲームコントローラに慣れている方には特に直感的です。

### 基本操作

#### 1. アーム（モーター起動）

1. 機体を平らな場所に置く
2. スロットルを最下げ位置にする
3. **スロットルスティックボタン**を押す（Mode 2: 左 / Mode 3: 右）
4. モーターが回転を始める

#### 2. 離陸

1. スロットルをゆっくり上げる
2. 機体が浮き始めたら、ホバリング位置で止める
3. 目安: スロットル50%前後

#### 3. ホバリング

- **姿勢スティック**（Mode 2: 右 / Mode 3: 左）でピッチ・ロールを調整
- **スロットルスティック**の左右でヨー（機首方向）を調整
- 現在は ACRO モードのため、スティックを離しても姿勢は維持されます（自動水平復帰なし）

#### 4. 着陸

1. スロットルをゆっくり下げる
2. 機体が着地したらスロットルを最下げ
3. **スロットルスティックボタン**を押してディスアーム

### ボタン操作一覧

| ボタン | 機能 |
|-------|------|
| スロットルスティックボタン (Mode 2: 左 / Mode 3: 右) | Arm / Disarm（モーター起動/停止） |
| 姿勢スティックボタン (Mode 2: 右 / Mode 3: 左) | Flip ※将来実装予定 |
| 左ボタン | 高度モード切替 ※将来実装予定 |
| 右ボタン | 制御モード切替 ※将来実装予定 |
| M5ボタン短押し | タイマー開始/停止 |
| M5ボタン長押し | タイマーリセット |

### 制御モード（現在の状態と将来計画）

> **現在のファームウェアは ACRO モードのみで動作します。**
>
> このスケルトンファームウェアは、制御理論を学びたいエンジニアや研究者のための出発点として設計されています。STABILIZE モード（角度制御）や高度維持機能は**意図的に未実装**としており、ユーザー自身が実装することを想定しています。
>
> ぜひ、あなた自身の手で姿勢制御アルゴリズムを実装してみてください！

| モード | 説明 | 状態 |
|-------|------|------|
| ACRO | 角速度制御、スティック入力が角速度指令値になる | **現在有効** |
| STABILIZE | スティックを離すと自動で水平に戻る角度制御 | 将来実装予定 |

### 高度モード（将来実装予定）

> 高度制御も現在は未実装です。ToFセンサからのデータは取得可能ですので、PID制御やより高度な制御手法を実装してみてください。

| モード | 説明 | 状態 |
|-------|------|------|
| Manual ALT | スロットルで直接高度を制御 | **現在有効** |
| Auto ALT | 高度を自動維持（ToFセンサ使用） | 将来実装予定 |

## 9. 緊急時の対応

### モーターを緊急停止したい場合

- **スロットルスティックボタン**を押してディスアーム
- または、スロットルを最下げにして待つ

### 機体が暴走した場合

1. スロットルを最下げにする
2. スロットルスティックボタンでディスアーム
3. バッテリーを外す

### よくあるトラブル

| 症状 | 原因 | 対処 |
|-----|------|------|
| モーターが回らない | アームされていない | スロットルスティックボタンを押す |
| 機体が傾く | キャリブレーション不足 | 平らな場所で再起動 |
| ホバリングが安定しない | バッテリー電圧低下 | バッテリーを充電/交換 |
| コントローラと接続できない | ペアリング切れ | ペアリングを再実行 |

## 10. 開発者向け機能

### WiFi テレメトリー

機体はWebSocketサーバーを内蔵しており、PCやスマートフォンからリアルタイムでセンサデータを確認できます。

**接続情報:**

| 項目 | 値 |
|-----|-----|
| SSID | `StampFly` |
| パスワード | なし（オープン） |
| URL | `http://192.168.4.1/` |
| 送信レート | 50Hz |

**接続手順:**
1. 機体に電源を入れる
2. PC/スマートフォンのWiFi設定から `StampFly` に接続
3. ブラウザで `http://192.168.4.1/` を開く
4. 3Dビジュアライザで姿勢がリアルタイム表示される

> **注意:** 現在のファームウェアでは姿勢推定・位置推定アルゴリズムが未調整のため、テレメトリーで表示される姿勢・位置データは不正確な値を示すことがあります。今後のアップデートで改善予定です。

### CLI（コマンドラインインターフェース）

USB シリアル接続でCLIコンソールにアクセスできます。センサの生データ確認やパラメータ調整に使用します。

**接続方法:**
```bash
idf.py -p /dev/ttyACM0 monitor
```

**主要コマンド:**

| コマンド | 説明 |
|---------|------|
| `help` | コマンド一覧を表示 |
| `status` | システム状態を表示 |
| `sensor all` | 全センサデータを表示 |
| `ctrl watch` | コントローラ入力をリアルタイム表示 |
| `motor test 1 30` | モーター1を30%で回転 |
| `motor stop` | 全モーター停止 |
| `pair start` | ペアリングモード開始 |
| `gain` | PIDゲイン表示・設定 |

**表示例:**

```
> help

=== StampFly CLI ===
Available commands:
  help         Show available commands
  status       Show system status
  sensor       Show sensor data
  motor        Motor control
  pair         Enter pairing mode
  gain         Rate control gains [axis param value]
  ...

> status
=== System Status ===
Flight State: IDLE
Error: NONE
Battery: 3.85V
ESP-NOW: paired, connected
ESKF: initialized
Attitude: R=0.5 P=-0.3 Y=45.2 deg

> sensor all
IMU:
  Accel: X=0.12, Y=-0.05, Z=-9.78 [m/s^2]
  Gyro:  X=0.002, Y=-0.001, Z=0.000 [rad/s]
Mag: X=22.5, Y=-5.3, Z=38.1 [uT]
Baro: Pressure=101325 [Pa], Alt=0.15 [m]
ToF: Bottom=0.250 [m], Front=2.100 [m]
OptFlow: Vx=0.001, Vy=-0.002 [m/s]
Power: 3.85 [V], 120.5 [mA]
```

詳細は [firmware/vehicle/README.md](../firmware/vehicle/README.md) を参照してください。

## 11. 次のステップ

- [firmware/vehicle/README.md](../firmware/vehicle/README.md) - 機体ファームウェアの詳細
- [firmware/controller/README.md](../firmware/controller/README.md) - コントローラファームウェアの詳細
- [PROJECT_PLAN.md](../PROJECT_PLAN.md) - プロジェクト全体計画

---

<a id="english"></a>

# Getting Started

This document explains the steps from environment setup to your first flight with the StampFly ecosystem.

## 1. Requirements

### Hardware

| Item | Model | Notes |
|------|-------|-------|
| StampFly Vehicle | - | With M5Stamp S3 |
| Controller | M5Stack AtomS3 + Atom JoyStick | Used together |
| USB-C Cable | - | For flashing × 2 |
| LiPo Battery | 1S 3.7V | For vehicle |

### Software

| Item | Version | Notes |
|------|---------|-------|
| ESP-IDF | v5.4.1 | Required |
| Git | Latest | - |
| Python | 3.8+ | ESP-IDF dependency |

## 2. Installing ESP-IDF

### macOS / Linux

```bash
# 1. Clone the repository
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf

# 2. Run the install script
cd ~/esp/esp-idf
./install.sh esp32s3

# 3. Set environment variables (run each time, or add to .bashrc/.zshrc)
. ~/esp/esp-idf/export.sh
```

### Windows

1. Download [ESP-IDF Tools Installer](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/get-started/windows-setup.html)
2. Run the installer and select ESP-IDF v5.4.1
3. Use "ESP-IDF PowerShell" or "ESP-IDF Command Prompt"

### Verify Installation

```bash
idf.py --version
# Example output: ESP-IDF v5.4.1
```

## 3. Clone the Repository

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
```

## 4. Building and Flashing Vehicle Firmware

### Build

```bash
cd firmware/vehicle
idf.py build
```

The first build takes several minutes. Success when `build/firmware_vehicle.bin` is generated.

### Flash

Connect the StampFly vehicle via USB and run:

```bash
# Linux
idf.py -p /dev/ttyACM0 flash

# macOS
idf.py -p /dev/cu.usbmodem* flash

# Windows
idf.py -p COM3 flash
```

Port names vary by environment. Check with `ls /dev/tty*` (Linux/macOS) or Device Manager (Windows).

### Serial Monitor

```bash
idf.py -p /dev/ttyACM0 monitor
```

Exit with `Ctrl + ]`.

## 5. Building and Flashing Controller Firmware

### Build

```bash
cd firmware/controller
idf.py build
```

Success when `build/firmware_controller.bin` is generated.

### Flash

Connect the AtomS3 via USB and run:

```bash
# Linux
idf.py -p /dev/ttyACM0 flash

# macOS
idf.py -p /dev/cu.usbmodem* flash

# Windows
idf.py -p COM3 flash
```

## 6. Pairing

Required for first use or when resetting pairing information.

### Procedure

1. **Controller**: Hold M5 button (below screen) while powering on
2. LCD shows "Pairing mode..." and beeping starts
3. **StampFly**: Long-press button (~2 seconds) to enter pairing mode
4. Both emit beeps when pairing is complete
5. Pairing info is auto-saved and auto-connects on next boot

## 7. Pre-Flight Checks

### Checklist

- [ ] Battery sufficiently charged (3.7V+ recommended)
- [ ] Propellers correctly attached
- [ ] Clear surroundings (minimum 2m × 2m space)
- [ ] Controller sticks in neutral position

### Stick Mode (Mode 2 / Mode 3)

StampFly supports both Mode 2 and Mode 3.

- **Mode 2**: The world standard stick layout. Used by most drones and RC aircraft, ideal if you want consistent control feel across different vehicles.
- **Mode 3**: Strongly recommended by the author! Places throttle on the right hand, allowing for intuitive control similar to game controllers. Please give it a try!

#### How to Switch Modes

| Boot Method | Selected Mode |
|-------------|---------------|
| Normal boot | Mode 2 |
| **Hold left button while booting** | **Mode 3 (Recommended)** |

The controller LCD displays `MODE: 2` or `MODE: 3`.

## 8. How to Fly

### Stick Layout

#### Mode 2

```
        Left Stick               Right Stick
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │   Throttle  │          │    Pitch    │
     │ ←Yaw    Yaw→│          │←Roll    Roll→│
     │   (Ascend)  │          │  (Forward)  │
     │      ↓      │          │      ↓      │
     │  (Descend)  │          │ (Backward)  │
     └─────────────┘          └─────────────┘
```

#### Mode 3 (Recommended)

```
        Left Stick               Right Stick
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │    Pitch    │          │   Throttle  │
     │ ←Roll   Roll→│          │ ←Yaw    Yaw→│
     │  (Forward)  │          │   (Ascend)  │
     │      ↓      │          │      ↓      │
     │ (Backward)  │          │  (Descend)  │
     └─────────────┘          └─────────────┘
```

In Mode 3, the right hand controls throttle (altitude) while the left hand controls the vehicle's attitude. This is especially intuitive for those familiar with game controllers.

### Basic Operation

#### 1. Arm (Start Motors)

1. Place vehicle on a flat surface
2. Set throttle to minimum position
3. Press **throttle stick button** (Mode 2: left / Mode 3: right)
4. Motors start spinning

#### 2. Takeoff

1. Slowly raise throttle
2. Stop at hover position when vehicle lifts off
3. Estimate: around 50% throttle

#### 3. Hovering

- Use **attitude stick** (Mode 2: right / Mode 3: left) to adjust pitch and roll
- Use **throttle stick** left/right to adjust yaw (heading)
- Currently in ACRO mode only, so attitude is maintained when stick is released (no auto-leveling)

#### 4. Landing

1. Slowly lower throttle
2. Once landed, set throttle to minimum
3. Press **throttle stick button** to disarm

### Button Reference

| Button | Function |
|--------|----------|
| Throttle Stick Button (Mode 2: left / Mode 3: right) | Arm / Disarm (motor start/stop) |
| Attitude Stick Button (Mode 2: right / Mode 3: left) | Flip *Future implementation* |
| Left Button | Altitude mode toggle *Future implementation* |
| Right Button | Control mode toggle *Future implementation* |
| M5 Button Short | Timer start/stop |
| M5 Button Long | Timer reset |

### Control Modes (Current State and Future Plans)

> **The current firmware operates in ACRO mode only.**
>
> This skeleton firmware is designed as a starting point for engineers and researchers who want to learn control theory. STABILIZE mode (angle control) and altitude hold features are **intentionally not implemented**, with the expectation that users will implement them themselves.
>
> We encourage you to implement your own attitude control algorithms!

| Mode | Description | Status |
|------|-------------|--------|
| ACRO | Rate control, stick input becomes angular velocity command | **Currently Active** |
| STABILIZE | Angle control that auto-levels when stick released | Future implementation |

### Altitude Modes (Future Implementation)

> Altitude control is also currently not implemented. ToF sensor data is available, so please try implementing PID control or more advanced control methods.

| Mode | Description | Status |
|------|-------------|--------|
| Manual ALT | Direct altitude control with throttle | **Currently Active** |
| Auto ALT | Auto altitude hold (uses ToF sensor) | Future implementation |

## 9. Emergency Response

### Emergency Motor Stop

- Press **throttle stick button** to disarm
- Or set throttle to minimum and wait

### If Vehicle Goes Out of Control

1. Set throttle to minimum
2. Disarm with throttle stick button
3. Disconnect battery

### Common Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| Motors don't spin | Not armed | Press throttle stick button |
| Vehicle tilts | Calibration needed | Restart on flat surface |
| Unstable hover | Low battery voltage | Charge/replace battery |
| Can't connect to controller | Pairing lost | Re-run pairing |

## 10. Developer Features

### WiFi Telemetry

The vehicle has a built-in WebSocket server, allowing you to view sensor data in real-time from a PC or smartphone.

**Connection Info:**

| Item | Value |
|------|-------|
| SSID | `StampFly` |
| Password | None (Open) |
| URL | `http://192.168.4.1/` |
| Update Rate | 50Hz |

**Connection Steps:**
1. Power on the vehicle
2. Connect to `StampFly` WiFi from your PC/smartphone
3. Open `http://192.168.4.1/` in a browser
4. 3D visualizer displays attitude in real-time

> **Note:** The current firmware's attitude and position estimation algorithms are not yet tuned, so attitude and position data shown in telemetry may display inaccurate values. This will be improved in future updates.

### CLI (Command Line Interface)

Access the CLI console via USB serial connection. Use it to check raw sensor data and adjust parameters.

**Connection:**
```bash
idf.py -p /dev/ttyACM0 monitor
```

**Main Commands:**

| Command | Description |
|---------|-------------|
| `help` | Show command list |
| `status` | Show system status |
| `sensor all` | Show all sensor data |
| `ctrl watch` | Show controller input in real-time |
| `motor test 1 30` | Run motor 1 at 30% |
| `motor stop` | Stop all motors |
| `pair start` | Enter pairing mode |
| `gain` | Show/set PID gains |

**Example Output:**

```
> help

=== StampFly CLI ===
Available commands:
  help         Show available commands
  status       Show system status
  sensor       Show sensor data
  motor        Motor control
  pair         Enter pairing mode
  gain         Rate control gains [axis param value]
  ...

> status
=== System Status ===
Flight State: IDLE
Error: NONE
Battery: 3.85V
ESP-NOW: paired, connected
ESKF: initialized
Attitude: R=0.5 P=-0.3 Y=45.2 deg

> sensor all
IMU:
  Accel: X=0.12, Y=-0.05, Z=-9.78 [m/s^2]
  Gyro:  X=0.002, Y=-0.001, Z=0.000 [rad/s]
Mag: X=22.5, Y=-5.3, Z=38.1 [uT]
Baro: Pressure=101325 [Pa], Alt=0.15 [m]
ToF: Bottom=0.250 [m], Front=2.100 [m]
OptFlow: Vx=0.001, Vy=-0.002 [m/s]
Power: 3.85 [V], 120.5 [mA]
```

See [firmware/vehicle/README.md](../firmware/vehicle/README.md) for details.

## 11. Next Steps

- [firmware/vehicle/README.md](../firmware/vehicle/README.md) - Vehicle firmware details
- [firmware/controller/README.md](../firmware/controller/README.md) - Controller firmware details
- [PROJECT_PLAN.md](../PROJECT_PLAN.md) - Overall project plan
