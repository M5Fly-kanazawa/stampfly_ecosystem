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
cd stampfly-ecosystem
./install.sh
```

インストーラがESP-IDFの検出・インストールも案内します。

#### Step 2: コントローラのファームウェア書き込み（初回のみ）

```bash
# ESP-IDF環境をアクティブ化
source ~/esp/esp-idf/export.sh

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
| Python | 3.10以上 | - |

## 2. インストール

### Step 1: リポジトリをクローン

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly-ecosystem
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
source ~/esp/esp-idf/export.sh
```

> **Tips**: `~/.bashrc` または `~/.zshrc` に `alias get_idf='source ~/esp/esp-idf/export.sh'` を追加すると便利です。

### Step 4: インストール確認

```bash
sf doctor
```

すべて `[OK]` と表示されれば成功です。

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

## 5. ペアリング

初回使用時、またはペアリング情報をリセットしたい場合に実行します。

### 手順

1. **コントローラ**: M5ボタン（画面下）を押しながら電源を入れる
2. LCD に "Pairing mode..." と表示され、ビープ音が鳴り始める
3. **StampFly**: ボタンを長押し（約2秒）してペアリングモードに入る
4. 両方からビープ音が鳴ればペアリング完了
5. ペアリング情報は自動保存され、次回以降は自動接続

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

> **現在のファームウェアは ACRO モードのみで動作します。**
> STABILIZE モードや高度維持機能は**意図的に未実装**です。
> 制御理論を学びたい方は、ぜひ自分で実装してみてください！

| モード | 説明 | 状態 |
|-------|------|------|
| ACRO | 角速度制御 | **現在有効** |
| STABILIZE | 角度制御（自動水平） | 未実装 |

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

## 9. 次のステップ

- [コマンドリファレンス](commands/README.md) - sf CLI の全コマンド
- [firmware/vehicle/README.md](../firmware/vehicle/README.md) - 機体ファームウェア詳細
- [firmware/controller/README.md](../firmware/controller/README.md) - コントローラ詳細

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
cd stampfly-ecosystem
./install.sh
```

The installer also guides ESP-IDF installation if needed.

#### Step 2: Flash Controller Firmware (First time only)

```bash
# Activate ESP-IDF environment
source ~/esp/esp-idf/export.sh

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
```

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
| Python | 3.10+ | - |

## 2. Installation

### Step 1: Clone Repository

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly-ecosystem
```

### Step 2: Run Installer

```bash
./install.sh
```

### Step 3: Activate Environment

```bash
source ~/esp/esp-idf/export.sh
```

### Step 4: Verify Installation

```bash
sf doctor
```

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

### Build → Flash → Monitor

```bash
sf build vehicle && sf flash vehicle -m
```

## 4. Build and Flash Controller Firmware

```bash
sf build controller
sf flash controller
```

## 5. Pairing

1. **Controller**: Hold M5 button while powering on
2. LCD shows "Pairing mode..."
3. **StampFly**: Long-press button (~2 sec) to enter pairing mode
4. Both beep when pairing completes

## 6. Pre-Flight Checks

- [ ] Battery charged (3.7V+ recommended)
- [ ] Propellers correctly attached
- [ ] Clear surroundings (2m × 2m minimum)
- [ ] Controller sticks in neutral

## 7. How to Fly

### Basic Operation

1. **Arm**: Press throttle stick button
2. **Takeoff**: Slowly raise throttle
3. **Land**: Lower throttle, then disarm

### Control Modes

> **Current firmware operates in ACRO mode only.**
> STABILIZE mode and altitude hold are **intentionally not implemented**.
> Implement them yourself to learn control theory!

| Mode | Description | Status |
|------|-------------|--------|
| ACRO | Rate control | **Active** |
| STABILIZE | Angle control | Not implemented |

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

## 9. Next Steps

- [Command Reference](commands/README.md) - All sf CLI commands
- [firmware/vehicle/README.md](../firmware/vehicle/README.md) - Vehicle firmware details
