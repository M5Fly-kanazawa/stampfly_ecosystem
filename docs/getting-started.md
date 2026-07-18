# はじめに

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

このドキュメントでは、StampFly エコシステムの環境構築から初飛行までの手順を解説します。

---

## 0. まずはシミュレータで遊んでみよう！

実機がなくても、コントローラとPCがあればシミュレータでドローン操縦を体験できます。**実機を飛ばす前の練習にも最適です！**

### 必要なもの

| 項目 | 説明 |
|-----|------|
| M5Stack AtomS3 + Atom JoyStick | コントローラ（USB HIDモードで使用） |
| PC | macOS / Windows / Linux |
| USB-Cケーブル | コントローラ接続用 |

### クイックスタート（5分で飛行開始！）

#### Step 1: エコシステムのインストール

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

インストーラがESP-IDFの検出・インストールも案内します。

#### Step 2: コントローラのファームウェア書き込み（初回のみ）

```bash
# 開発環境のセットアップ
source setup_env.sh

# コントローラファームウェアをビルド・書き込み
sf build controller
sf flash controller
```

#### Step 3: コントローラをUSB HIDモードに切り替え

1. コントローラの電源を入れる
2. **画面を押して**メニューを開く
3. 「**USB Mode**」を選択
4. コントローラが再起動し、USB HIDゲームパッドとして認識される

> **Tips**: USB HIDモードでは、コントローラがPCに直接ゲームパッドとして認識されます。ESP-NOWモードに戻すには、再度メニューから切り替えてください。

#### Step 4: シミュレータの起動

```bash
sf sim run vpython
```

ブラウザが自動で開き、3Dビューが表示されます。

#### Step 5: 飛ばしてみよう！

| 操作 | Mode 2（デフォルト） | Mode 3 |
|-----|---------------------|--------|
| スロットル（上昇/下降） | 左スティック上下 | 右スティック上下 |
| ロール（左右移動） | 右スティック左右 | 左スティック左右 |
| ピッチ（前後移動） | 右スティック上下 | 左スティック上下 |
| ヨー（旋回） | 左スティック左右 | 右スティック左右 |

1. **スロットルをゆっくり上げる** → ドローンが浮上
2. **スティックで姿勢を調整** → 好きな方向に飛行
3. **スロットルを下げる** → 着陸

### シミュレータのオプション

```bash
# VPythonシミュレータ（デフォルト）
sf sim run vpython

# Genesisシミュレータ（高精度物理、要別途インストール）
sf setup genesis
sf sim run genesis

# ワールドオプション
sf sim run vpython --world ringworld  # リングワールド
sf sim run vpython --seed 12345       # シード指定
```

### トラブルシューティング

| 症状 | 対処 |
|-----|------|
| コントローラが認識されない | USB HIDモードに切り替えたか確認。PCを再起動 |
| スティックがドリフトする | メニューから「Calibration」を実行 |
| vpythonが見つからない | `sf setup sim` を実行 |

シミュレータに慣れたら、実機での飛行に挑戦しましょう！

### ビルド不要で試す（Web Flasher）

開発環境を構築しなくても、ブラウザから実機にファームウェアを書き込めます。

| 項目 | 内容 |
|-----|------|
| URL | https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/ |
| 必要環境 | Chrome または Edge（Web Serial対応）＋ USB-Cケーブル（データ通信対応） |
| できること | 最新の GitHub Release のフルイメージをブラウザから直接書き込み |

#### デスクトップ版（StampFly Flasher）

ブラウザ書き込み（Web Serial）は macOS の Chrome でクラッシュする既知の不具合があるほか、
ブラウザなしでオフライン書き込みしたい教室にも便利なため、デスクトップ GUI アプリも用意されています。

| 項目 | 内容 |
|-----|------|
| 入手先 | [リリースページ](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest)（v2026.07.1 以降のリリースに添付。Linux版は v2026.07.2 以降） |
| 対応 | Windows / macOS（Apple Silicon・Intel）/ Linux（x64） |
| 使い方 | ダウンロード → 起動 → 対象を選んで書き込み（macOS初回は右クリック→開く） |
| ソースから | `python3 tools/flasher_gui/stampfly_flasher.py`（要 `pip install esptool`）または `sf flash --gui` |
| `sf` 導入済みなら | `sf flasher install` でダウンロード〜SHA256検証〜ネイティブアプリ導入を自動化（詳細: [sf flasher](commands/sf-flasher.md)） |

ソースからビルドしたい場合は、このまま以降の手順（ESP-IDF環境構築）に進んでください。

---

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
| ESP-IDF | v5.5.2 | install.shで自動インストール可 |
| Git | 最新版 | - |
| Python | 3.8以上 | - |

## 2. インストール

### Step 1: リポジトリをクローン

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
```

### Step 2: インストーラを実行

```bash
./install.sh
```

インストーラが以下を行います:
- ESP-IDFの検出（未インストールの場合はインストール案内）
- sf CLI のインストール
- シミュレータ依存（vpython, pygame等）のインストール

### Step 3: 環境をアクティブ化

```bash
source setup_env.sh
```

### Step 4: インストール確認

```bash
sf doctor
```

すべて `[OK]` と表示されれば成功です。

### Step 5: 最新版に更新したいとき

インストール後、新しい機能や修正を取り込みたくなったら `sf upgrade` を実行してください。編集中のファイルは自動的に保護されます。

```bash
sf upgrade
```

詳しい仕組みや、うまくいかない場合のトラブルシューティングは **[アップグレードガイド](guides/upgrading.md)** を参照してください。

## 3. 機体ファームウェアのビルドと書き込み

### ビルド

```bash
sf build vehicle
```

初回ビルドには数分かかります。

### 書き込み

StampFly 機体をUSBで接続し、以下を実行:

```bash
sf flash vehicle
```

ポートは自動検出されます。手動指定する場合:

```bash
sf flash vehicle -p /dev/ttyACM0    # Linux
sf flash vehicle -p /dev/cu.usbmodem*  # macOS
sf flash vehicle -p COM3            # Windows
```

### シリアルモニタ

```bash
sf monitor
```

終了は `Ctrl + ]` です。

### ビルド→書き込み→モニタを一括実行

```bash
sf build vehicle && sf flash vehicle -m
```

## 4. コントローラファームウェアのビルドと書き込み

### ビルド

```bash
sf build controller
```

### 書き込み

AtomS3 をUSBで接続し、以下を実行:

```bash
sf flash controller
```

## 5. 通信モードの選択

StampFlyは2つの通信モードをサポートしています。

| モード | 特徴 | 用途 |
|-------|------|------|
| ESP-NOW | 低遅延、TDMA同期、最大10台同時 | 複数機編隊飛行、レース |
| UDP | シンプル、WiFi AP経由、単機運用 | 開発・デバッグ、単機飛行 |

### ESP-NOWモード（デフォルト）

複数機の編隊飛行やTDMA同期が必要な場合に使用。

**ペアリング手順:**
1. **コントローラ**: M5ボタン（画面下）を押しながら電源を入れる
2. LCD に "Pairing mode..." と表示され、ビープ音が鳴り始める
3. **StampFly**: ボタンを長押し（約2秒）してペアリングモードに入る
4. 両方からビープ音が鳴ればペアリング完了
5. ペアリング情報は自動保存され、次回以降は自動接続

### UDPモード（推奨：単機運用時）

単機での飛行や開発・デバッグ時に使用。設定が簡単です。

**設定手順:**

1. **Vehicle側（CLI）:**
   ```
   comm udp
   ```
   この設定はNVSに保存され、次回起動時も維持されます。

2. **Controller側:**
   - 画面を押してメニューを開く
   - 「UDP Mode」を選択
   - コントローラが再起動し、VehicleのWiFi APに接続

**通信の仕組み:**
```
┌────────────┐      WiFi AP      ┌────────────┐
│ Controller │ ←───────────────→ │  Vehicle   │
│   (STA)    │   192.168.4.1     │   (AP)     │
└────────────┘                   └────────────┘
```

**注意:** UDPモードではペアリングは不要です。VehicleのWiFi APに自動接続します。

## 6. 飛行前の確認

### チェックリスト

- [ ] バッテリーは十分に充電されているか（3.7V以上推奨）
- [ ] プロペラは正しく取り付けられているか
- [ ] 周囲に障害物がないか（最低2m四方の空間を確保）
- [ ] コントローラのスティックが中立位置にあるか

### スティックモード (Mode 2 / Mode 3)

| 起動方法 | 選択されるモード |
|---------|----------------|
| 通常起動 | Mode 2 |
| **左ボタンを押しながら起動** | **Mode 3（推奨）** |

## 7. 飛行方法

### スティック配置

#### Mode 2

```
        左スティック              右スティック
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │   スロットル  │          │   ピッチ    │
     │ ←ヨー    ヨー→│          │←ロール ロール→│
     │      ↓      │          │      ↓      │
     └─────────────┘          └─────────────┘
```

#### Mode 3（推奨）

```
        左スティック              右スティック
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │   ピッチ     │          │   スロットル │
     │ ←ロール ロール→│          │ ←ヨー    ヨー→│
     │      ↓      │          │      ↓      │
     └─────────────┘          └─────────────┘
```

### 基本操作

#### 1. アーム（モーター起動）

1. 機体を平らな場所に置く
2. スロットルを最下げ位置にする
3. **スロットルスティックボタン**を押す
4. モーターが回転を始める

#### 2. 離陸

1. スロットルをゆっくり上げる
2. 機体が浮き始めたら、ホバリング位置で止める

#### 3. 着陸

1. スロットルをゆっくり下げる
2. 機体が着地したらスロットルを最下げ
3. **スロットルスティックボタン**を押してディスアーム

### 制御モード

| モード | 説明 | 必要センサ | 状態 |
|-------|------|-----------|------|
| ACRO | 角速度制御 | IMU | **実装済み** |
| STABILIZE | 角度制御（自動水平） | IMU | **実装済み** |
| ALTITUDE_HOLD | 高度維持 | IMU + ToF or Baro | **実装済み** |
| POSITION_HOLD | 位置保持 | IMU + ToF/Baro + Flow | **実装済み** |

> **Tips:** コントローラの右ボタンで制御モード、左ボタンで高度モードを切り替えられます。
> 必要なセンサが無効（config.hpp で OFF）の場合、自動的に STABILIZE に降格します。
> POSITION_HOLD（POS_HOLD）は実機飛行での位置保持動作まで確認済みです。

## 8. 開発者向け機能

### WiFi テレメトリー

機体はWebSocketサーバーを内蔵しており、リアルタイムでセンサデータを確認できます。

| 項目 | 値 |
|-----|-----|
| SSID | `StampFly` |
| パスワード | なし |
| URL | `http://192.168.4.1/` |

### ログキャプチャ

```bash
# WiFiでテレメトリをキャプチャ（30秒）
sf log wifi -d 30

# USBシリアルでキャプチャ
sf log capture -d 60

# ログ一覧
sf log list

# フライト解析
sf log analyze
```

### キャリブレーション

```bash
# ジャイロキャリブレーション
sf cal gyro

# 磁気キャリブレーション
sf cal mag start
# 機体を8の字に回転...
sf cal mag stop
sf cal mag save
```

### CLI（コマンドラインインターフェース）

```bash
sf monitor
```

主要コマンド:

| コマンド | 説明 |
|---------|------|
| `help` | コマンド一覧 |
| `status` | システム状態 |
| `sensor` | センサデータ表示 |
| `motor test 1 30` | モーター1を30%で回転 |
| `gain` | PIDゲイン設定 |

### Tello SDK 互換 API（プログラムからの飛行制御）

コントローラでの通常操作に加えて、Vehicle は **Tello SDK 互換の UDP コマンド**（`UDP:8889`）を受け付け、
`djitellopy` など既存の Tello 用 Python プログラムがほぼ無改変で動作します。

```bash
# 例（tools/stampfly_py 配下）
python3 tools/stampfly_py/example_djitellopy.py
```

対応コマンド一覧・安全上の注意は [`tools/stampfly_py/README.md`](../tools/stampfly_py/README.md) を
参照してください。**送信機は安全装置として中立位置でONのまま保持してください**（スティックを動かせば
いつでもパイロットが介入・停止できます）。

## 9. 次のステップ

- [コマンドリファレンス](commands/README.md) - sf CLI の全コマンド
- [firmware/vehicle/README.md](../firmware/vehicle/README.md) - 機体ファームウェア詳細
- [firmware/controller/README.md](../firmware/controller/README.md) - コントローラ詳細
- [tools/stampfly_py/README.md](../tools/stampfly_py/README.md) - Tello SDK 互換 API 詳細

---

<a id="english"></a>

# Getting Started

This document explains the steps from environment setup to your first flight with the StampFly ecosystem.

---

## 0. Try the Simulator First!

Even without the actual drone, you can experience drone piloting with just the controller and a PC.

### Requirements

| Item | Description |
|------|-------------|
| M5Stack AtomS3 + Atom JoyStick | Controller (USB HID mode) |
| PC | macOS / Windows / Linux |
| USB-C Cable | For controller connection |

### Quick Start (Flying in 5 minutes!)

#### Step 1: Install Ecosystem

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

The installer also guides ESP-IDF installation if needed.

#### Step 2: Flash Controller Firmware (First time only)

```bash
# Activate development environment
source setup_env.sh

# Build and flash controller
sf build controller
sf flash controller
```

#### Step 3: Switch Controller to USB HID Mode

1. Power on the controller
2. **Press the screen** to open menu
3. Select "**USB Mode**"
4. Controller restarts as USB HID gamepad

#### Step 4: Launch Simulator

```bash
sf sim run vpython
```

A browser opens with the 3D view.

#### Step 5: Let's Fly!

| Control | Mode 2 (Default) | Mode 3 |
|---------|------------------|--------|
| Throttle | Left stick up/down | Right stick up/down |
| Roll | Right stick left/right | Left stick left/right |
| Pitch | Right stick up/down | Left stick up/down |
| Yaw | Left stick left/right | Right stick left/right |

### Simulator Options

```bash
# VPython simulator (default)
sf sim run vpython

# Genesis simulator (high-precision, separate install)
sf setup genesis
sf sim run genesis

# World options
sf sim run vpython --world ringworld  # Ring world
sf sim run vpython --seed 12345       # Specify seed
```

### Troubleshooting

| Symptom | Solution |
|---------|----------|
| Controller not recognized | Check USB HID mode is enabled. Restart PC |
| Stick drifting | Run "Calibration" from menu |
| vpython not found | Run `sf setup sim` |

Once comfortable with the simulator, try flying the real drone!

### Try it without building (Web Flasher)

You can flash firmware to the real hardware straight from your browser — no build environment needed.

| Item | Details |
|------|---------|
| URL | https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/ |
| Requirements | Chrome or Edge (Web Serial support) + a USB-C data cable |
| What it does | Flashes the latest GitHub Release full firmware image directly from the browser |

#### Desktop App (StampFly Flasher)

A desktop GUI app is also available — the browser flasher (Web Serial) has a known bug that
crashes Chrome on macOS, and some classrooms need offline/no-browser flashing anyway.

| Item | Details |
|------|---------|
| Get it | [Releases page](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest) (attached to releases from v2026.07.1 onward; Linux build from v2026.07.2 onward) |
| Supported | Windows / macOS (Apple Silicon and Intel) / Linux (x64) |
| Usage | Download → launch → select target → flash (on macOS, right-click → Open the first time) |
| From source | `python3 tools/flasher_gui/stampfly_flasher.py` (requires `pip install esptool`) or `sf flash --gui` |
| If `sf` is set up | `sf flasher install` automates download, SHA256 verification, and native-app install (details: [sf flasher](commands/sf-flasher.md)) |

If you want to build from source, continue with the rest of this guide (ESP-IDF setup).

---

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
| ESP-IDF | v5.5.2 | Auto-install via install.sh |
| Git | Latest | - |
| Python | 3.8+ | - |

## 2. Installation

### Step 1: Clone Repository

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
```

### Step 2: Run Installer

```bash
./install.sh
```

### Step 3: Activate Environment

```bash
source setup_env.sh
```

### Step 4: Verify Installation

```bash
sf doctor
```

### Step 5: Keeping It Up to Date

Once installed, run `sf upgrade` whenever you want to pull in new features or fixes. Any files you were editing are protected automatically.

```bash
sf upgrade
```

For the full walkthrough and troubleshooting, see the **[Upgrading Guide](guides/upgrading.md)**.

## 3. Build and Flash Vehicle Firmware

### Build

```bash
sf build vehicle
```

### Flash

Connect StampFly via USB:

```bash
sf flash vehicle
```

Port is auto-detected. To specify manually:

```bash
sf flash vehicle -p /dev/ttyACM0    # Linux
sf flash vehicle -p /dev/cu.usbmodem*  # macOS
sf flash vehicle -p COM3            # Windows
```

### Serial Monitor

```bash
sf monitor
```

Exit with `Ctrl + ]`.

### Build → Flash → Monitor

```bash
sf build vehicle && sf flash vehicle -m
```

## 4. Build and Flash Controller Firmware

```bash
sf build controller
sf flash controller
```

## 5. Communication Mode Selection

StampFly supports two communication modes.

| Mode | Features | Use Case |
|------|----------|----------|
| ESP-NOW | Low latency, TDMA sync, up to 10 devices | Multi-vehicle, racing |
| UDP | Simple, via WiFi AP, single-vehicle | Development, solo flight |

### ESP-NOW Mode (Default)

For multi-vehicle formation or TDMA synchronization.

**Pairing:**
1. **Controller**: Hold M5 button (under screen) while powering on
2. LCD shows "Pairing mode..." and beeping starts
3. **StampFly**: Long-press button (~2 sec) to enter pairing mode
4. Both beep when pairing completes
5. Pairing info is saved automatically and auto-connects next time

### UDP Mode (Recommended for single-vehicle)

For solo flights or development/debugging. Simple setup.

**Setup:**

1. **Vehicle (CLI):**
   ```
   comm udp
   ```
   This setting is saved to NVS and persists after restart.

2. **Controller:**
   - Press screen to open menu
   - Select "UDP Mode"
   - Controller restarts and connects to Vehicle's WiFi AP

**How it works:**
```
┌────────────┐      WiFi AP      ┌────────────┐
│ Controller │ ←───────────────→ │  Vehicle   │
│   (STA)    │   192.168.4.1     │   (AP)     │
└────────────┘                   └────────────┘
```

**Note:** UDP mode does not require pairing. Auto-connects to Vehicle's WiFi AP.

## 6. Pre-Flight Checks

### Checklist

- [ ] Battery charged (3.7V+ recommended)
- [ ] Propellers correctly attached
- [ ] Clear surroundings (2m × 2m minimum)
- [ ] Controller sticks in neutral

### Stick Mode (Mode 2 / Mode 3)

| Startup Method | Selected Mode |
|----------------|---------------|
| Normal startup | Mode 2 |
| **Hold left button while starting** | **Mode 3 (recommended)** |

## 7. How to Fly

### Stick Layout

#### Mode 2

```
        Left Stick                Right Stick
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │   Throttle   │          │    Pitch    │
     │ ←Yaw    Yaw→ │          │←Roll   Roll→│
     │      ↓      │          │      ↓      │
     └─────────────┘          └─────────────┘
```

#### Mode 3 (Recommended)

```
        Left Stick                Right Stick
     ┌─────────────┐          ┌─────────────┐
     │      ↑      │          │      ↑      │
     │    Pitch     │          │   Throttle  │
     │←Roll   Roll→ │          │ ←Yaw    Yaw→│
     │      ↓      │          │      ↓      │
     └─────────────┘          └─────────────┘
```

### Basic Operation

#### 1. Arm (Start Motors)

1. Place the drone on a flat surface
2. Set throttle to lowest position
3. **Press the throttle stick button**
4. Motors start spinning

#### 2. Takeoff

1. Slowly raise the throttle
2. Once the drone lifts off, hold at hover position

#### 3. Landing

1. Slowly lower the throttle
2. Once landed, set throttle to lowest
3. **Press the throttle stick button** to disarm

### Control Modes

| Mode | Description | Required Sensors | Status |
|------|-------------|-----------------|--------|
| ACRO | Rate control | IMU | **Implemented** |
| STABILIZE | Angle control (auto-level) | IMU | **Implemented** |
| ALTITUDE_HOLD | Altitude hold | IMU + ToF or Baro | **Implemented** |
| POSITION_HOLD | Position hold | IMU + ToF/Baro + Flow | **Implemented** |

> **Tips:** Use the right button on the controller to switch control modes, and the left button for altitude mode.
> If required sensors are disabled (OFF in config.hpp), the mode automatically downgrades to STABILIZE.
> POSITION_HOLD (POS_HOLD) has been validated on real hardware, including in-flight position hold.

## 8. Developer Features

### WiFi Telemetry

| Item | Value |
|------|-------|
| SSID | `StampFly` |
| Password | None |
| URL | `http://192.168.4.1/` |

### Log Capture

```bash
sf log wifi -d 30      # WiFi capture (30s)
sf log capture -d 60   # USB serial capture
sf log list            # List logs
sf log analyze         # Flight analysis
```

### Calibration

```bash
sf cal gyro            # Gyro calibration
sf cal mag start       # Start mag calibration
sf cal mag save        # Save calibration
```

### CLI

```bash
sf monitor
```

### Tello SDK-Compatible API (Programmatic Flight Control)

In addition to normal controller operation, the Vehicle accepts **Tello SDK-compatible UDP
commands** (`UDP:8889`), so existing Tello Python programs (e.g. `djitellopy`) run nearly
unmodified.

```bash
# Example (under tools/stampfly_py)
python3 tools/stampfly_py/example_djitellopy.py
```

See [`tools/stampfly_py/README.md`](../tools/stampfly_py/README.md) for the full command list
and safety notes. **Keep the paired RC transmitter ON with neutral sticks as a safety device**
— the pilot can intervene and stop the vehicle at any time by moving a stick.

## 9. Next Steps

- [Command Reference](commands/README.md) - All sf CLI commands
- [firmware/vehicle/README.md](../firmware/vehicle/README.md) - Vehicle firmware details
- [firmware/controller/README.md](../firmware/controller/README.md) - Controller details
- [tools/stampfly_py/README.md](../tools/stampfly_py/README.md) - Tello SDK-compatible API details
