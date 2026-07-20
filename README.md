# StampFly Ecosystem

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 自分の手で、ドローンの飛行制御を作りたいあなたへ

**StampFly Ecosystem** は、ドローン制御を**学び、実装し、実験する**ための
教育・研究プラットフォームです。

「PID制御を教科書で学んだけど、実際に動くものを作りたい」
「姿勢推定アルゴリズムを自分で実装して試したい」
「研究用の飛行実験プラットフォームが欲しい」

そんなあなたのために、このエコシステムは存在します。

---

## 🔗 すぐに使えるリソース

| リソース | 説明 |
|---------|------|
| [🌐 プロジェクト紹介](https://m5fly-kanazawa.github.io/stampfly_ecosystem/) | 実機3Dモデル付きランディングページ |
| [📚 ドキュメント](https://m5fly-kanazawa.github.io/stampfly_ecosystem/docs/) | 全ドキュメントを検索・閲覧 |
| [🔌 Web書き込み](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) | ビルド不要、ブラウザから実機に書き込み |
| [🖥️ StampFly Flasher](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest) | デスクトップ書き込みアプリ（Windows/macOS・Python不要） |
| [📦 ビルド済みファームウェア](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases) | GitHub Releases（vehicle / controller） |

---

## 何ができるのか？

| できること | 内容 |
|-----------|------|
| **シミュレータで練習** | 実機なしでドローン操縦を体験。制御アルゴリズムの検証にも |
| **すぐに飛ばせる** | ファームウェアをビルドして、実機で飛行 |
| **WiFi経由で制御** | PC/スマホから高レベルコマンド（jump, takeoff, land, hover）を実行 |
| **制御を自作できる** | 4つの制御モード（ACRO/STABILIZE/高度維持/位置保持）を搭載。カスケード制御を学び、カスタマイズ可能 |
| **センサーデータを見れる** | IMU、気圧、ToF、オプティカルフロー等のリアルタイムデータをCLI/WiFiで取得 |
| **実験データを解析できる** | ログを記録し、Pythonで解析・可視化 |

---

## 📦 インストール

**ターミナルを使わない方法（推奨・初心者向け）:** [Releases](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest) から
お使いのOSの **StampFly Setup**（`StampFlySetup_<バージョン>_<OS>`）をダウンロードして実行するだけで、
ウィザードが一式（sf CLI + ESP-IDF + 書き込みアプリ）を導入します。
**→ [GUI インストーラガイド](docs/guides/gui-installer.md)**

コマンドラインで導入する場合:

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

これだけで sf CLI とシミュレータ依存がインストールされます。
ESP-IDFが未インストールの場合は、インストーラが案内します。

**→ [セットアップガイド（詳細）](docs/setup/README.md)**

最新版に更新したいときは `sf upgrade` を実行するだけです（編集中のファイルは自動保護）。
**→ [アップグレードガイド](docs/guides/upgrading.md)**

---

## 🎮 まずはシミュレータで飛ばしてみよう！

**実機がなくても大丈夫。** コントローラとPCがあれば、今すぐドローン操縦を体験できます。

### VPython版（軽量・ブラウザ表示）

```bash
sf sim run vpython
```

### Genesis版（高精度物理エンジン）

```bash
sf sim run genesis
```

コントローラをUSB HIDモードに切り替えてPCに接続すれば、
3Dビューでドローンを自由に飛ばせます。

| シミュレータ | 特徴 |
|-------------|------|
| VPython版 | 軽量、センサモデル充実、SIL/HIL対応 |
| Genesis版 | 2000Hz物理演算、物理量ベース制御 |

**→ [シミュレータで遊ぶ（詳細手順）](docs/getting-started.md#0-まずはシミュレータで遊んでみよう)**

---

## 🚀 sf CLI クイックスタート

**sf CLI** は、ビルド、書き込み、モニタ、ログ取得などをシンプルに実行できる統合コマンドラインツールです。

```bash
# 環境をアクティブ化
source setup_env.sh

# 環境診断
sf doctor

# ビルド → 書き込み → モニタ
sf build vehicle && sf flash vehicle -m
```

### よく使うコマンド

| コマンド | 説明 |
|---------|------|
| `sf build vehicle` | 機体ファームウェアをビルド |
| `sf flash vehicle -m` | 書き込み後にモニタを開く |
| `sf monitor` | シリアルモニタを開く |
| `sf log wifi` | WiFiテレメトリをキャプチャ |
| `sf cal gyro` | ジャイロキャリブレーション |
| `sf sim run vpython` | シミュレータを起動 |

**→ [コマンドリファレンス](docs/commands/README.md)** | **[セットアップガイド](docs/setup/README.md)**

---

## 工場出荷状態に戻す

ワークショップやカスタムファームウェアの書き込みにより、機体・送信機のファームウェアは上書きされます。
工場出荷状態に戻したい場合は、以下のコマンドを実行してください：

```bash
# 機体を工場出荷状態に戻す
sf flash vehicle --legacy

# 送信機を工場出荷状態に戻す
sf flash controller --legacy
```

> **注意:** 事前に `source setup_env.sh` で開発環境をセットアップしてください。デバイスを USB 接続した状態で実行してください。

---

## ディレクトリ構成

```
stampfly_ecosystem/
├── docs/           # ドキュメント
├── firmware/       # 組込みファームウェア
│   ├── vehicle/     # 機体ファームウェア（主力）
│   ├── vehicle_old/ # レガシー機体ファームウェア（凍結）
│   ├── controller/  # 送信機ファームウェア
│   └── common/      # 共有コード（ESP-NOW プロトコル構造体）
├── protocol/       # 通信プロトコル仕様（構築中）
├── control/        # 制御設計資産（構築中）
├── analysis/       # 実験データ解析（構築中）
├── tools/          # 補助ツール（構築中）
├── simulator/      # 3Dフライトシミュレータ
├── ros/            # ROS連携（構築中）
├── examples/       # 学習用サンプル
└── third_party/    # 外部依存
```

---

## 始めよう

**→ [Getting Started（環境構築〜初フライト）](docs/getting-started.md)**

ESP-IDFのセットアップから、ファームウェアのビルド、
機体と送信機のペアリング、そして初フライトまで。
すべての手順をステップバイステップで解説しています。

---

## ワークショップで本格的に学ぶ

Getting Started で初フライトができたら、次は**ワークショップ**で制御の仕組みをじっくり学びましょう。

| レッスン | テーマ | 学べること |
|---------|--------|-----------|
| Lesson 1 | 環境構築 | ESP-IDF セットアップ、ビルド＆書き込み |
| Lesson 2 | コントローラ入力 | ESP-NOW 通信、スティック値の読み取り |
| Lesson 3 | LED 制御 | システム状態の可視化 |
| Lesson 4 | IMU センサー | 加速度・ジャイロデータの取得と理解 |
| Lesson 5 | モータ制御 | PWM によるモータ個別制御 |
| Lesson 6 | 姿勢推定 | 相補フィルタによる姿勢角の算出 |
| Lesson 7 | レート P 制御 | 角速度フィードバックによる安定化 |
| Lesson 8 | PID 制御 | 姿勢角の PID 制御と飛行 |

**→ [ワークショップスライド](docs/workshop/slides/README.md)** | **[ワークショップガイド](docs/workshop/)**

---

## 技術仕様

| 項目 | 仕様 |
|------|------|
| MCU | ESP32-S3（M5Stamp S3） |
| フレームワーク | ESP-IDF v5.5.2 + FreeRTOS |
| 姿勢推定 | ESKF（Error-State Kalman Filter） |
| 通信 | ESP-NOW + WiFi（テレメトリ） |
| センサー | BMI270, BMM150, BMP280, VL53L3CX, PMW3901 |

---

## ライセンス

MIT License

---

---

<a id="english"></a>

# StampFly Ecosystem

## For those who want to build their own drone control

**StampFly Ecosystem** is an educational and research platform for
**learning, implementing, and experimenting** with drone control.

"I learned PID control from textbooks, but I want to build something that actually flies."
"I want to implement my own attitude estimation algorithm and test it."
"I need a flight experiment platform for my research."

This ecosystem exists for you.

---

## 🔗 Quick Links

| Resource | Description |
|----------|-------------|
| [🌐 Project Overview](https://m5fly-kanazawa.github.io/stampfly_ecosystem/) | Landing page with interactive 3D drone model |
| [📚 Documentation](https://m5fly-kanazawa.github.io/stampfly_ecosystem/docs/) | Browse and search all documentation |
| [🔌 Web Flasher](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) | Flash real hardware from the browser, no build required |
| [🖥️ StampFly Flasher](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest) | Desktop flashing app (Windows/macOS, no Python required) |
| [📦 Pre-built Firmware](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases) | GitHub Releases (vehicle / controller binaries) |

---

## What can you do?

| Capability | Description |
|-----------|-------------|
| **Practice in simulator** | Experience drone piloting without real hardware. Also for testing control algorithms |
| **Fly immediately** | Build firmware and fly the real drone |
| **Control via WiFi** | Send high-level commands (jump, takeoff, land, hover) from PC/smartphone |
| **Build your own control** | 4 flight modes (ACRO/STABILIZE/Altitude Hold/Position Hold) included. Learn and customize cascade control |
| **View sensor data** | Real-time IMU, barometer, ToF, optical flow data via CLI/WiFi |
| **Analyze experiments** | Record flight logs and analyze with Python |

---

## 📦 Installation

**No-terminal path (recommended for beginners):** download **StampFly Setup**
(`StampFlySetup_<version>_<OS>`) for your OS from
[Releases](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest)
and run it — the wizard installs everything (sf CLI + ESP-IDF + the flashing app).
**→ [GUI Installer Guide](docs/guides/gui-installer.md)**

Command-line alternative:

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

This installs sf CLI and simulator dependencies.
If ESP-IDF is not installed, the installer will guide you.

**→ [Setup Guide (Details)](docs/setup/README.md)**

To update to the latest version later, just run `sf upgrade` (files you're editing are protected automatically).
**→ [Upgrading Guide](docs/guides/upgrading.md)**

---

## 🎮 Try the Simulator First!

**No drone needed.** With just a controller and PC, you can experience drone piloting right now.

### VPython Version (Lightweight, Browser)

```bash
sf sim run vpython
```

### Genesis Version (High-Precision Physics)

```bash
sf sim run genesis
```

Switch the controller to USB HID mode and connect to your PC.
You can fly a drone freely in the 3D view.

| Simulator | Features |
|-----------|----------|
| VPython | Lightweight, rich sensor models, SIL/HIL |
| Genesis | 2000Hz physics, physical-unit control |

**→ [Play with the Simulator (Detailed Steps)](docs/getting-started.md#0-try-the-simulator-first)**

---

## 🚀 sf CLI Quick Start

**sf CLI** is an integrated command-line tool for build, flash, monitor, log capture, and more.

```bash
# Activate environment
source setup_env.sh

# Run diagnostics
sf doctor

# Build → Flash → Monitor
sf build vehicle && sf flash vehicle -m
```

### Common Commands

| Command | Description |
|---------|-------------|
| `sf build vehicle` | Build vehicle firmware |
| `sf flash vehicle -m` | Flash and open monitor |
| `sf monitor` | Open serial monitor |
| `sf log wifi` | Capture WiFi telemetry |
| `sf cal gyro` | Gyro calibration |
| `sf sim run vpython` | Run simulator |

**→ [Command Reference](docs/commands/README.md)** | **[Setup Guide](docs/setup/README.md)**

---

## Restore Factory Firmware

Workshop lessons and custom firmware will overwrite the factory firmware on your vehicle and controller.
To restore the factory state, run:

```bash
# Restore vehicle to factory firmware
sf flash vehicle --legacy

# Restore controller to factory firmware
sf flash controller --legacy
```

> **Note:** Run `source setup_env.sh` to set up the development environment first. Connect the device via USB before running.

---

## Directory Structure

```
stampfly_ecosystem/
├── docs/           # Documentation
├── firmware/       # Embedded firmware
│   ├── vehicle/     # Vehicle firmware (primary)
│   ├── vehicle_old/ # Legacy vehicle firmware (frozen)
│   ├── controller/  # Transmitter firmware
│   └── common/      # Shared code (ESP-NOW protocol structs)
├── protocol/       # Communication protocol spec (WIP)
├── control/        # Control design assets (WIP)
├── analysis/       # Experiment data analysis (WIP)
├── tools/          # Utility tools (WIP)
├── simulator/      # 3D flight simulator
├── ros/            # ROS integration (WIP)
├── examples/       # Learning examples
└── third_party/    # External dependencies
```

---

## Get Started

**→ [Getting Started (Setup to First Flight)](docs/getting-started.md)**

From ESP-IDF setup to firmware build,
pairing vehicle and controller, and your first flight.
All steps explained step-by-step.

---

## Learn Through the Workshop

Once you've completed your first flight with Getting Started, dive deeper with the **Workshop** to learn how drone control really works.

| Lesson | Topic | What You'll Learn |
|--------|-------|-------------------|
| Lesson 1 | Environment Setup | ESP-IDF setup, build & flash |
| Lesson 2 | Controller Input | ESP-NOW communication, reading stick values |
| Lesson 3 | LED Control | Visualizing system state |
| Lesson 4 | IMU Sensor | Reading accelerometer & gyroscope data |
| Lesson 5 | Motor Control | Individual motor control via PWM |
| Lesson 6 | Attitude Estimation | Computing attitude angles with complementary filter |
| Lesson 7 | Rate P Control | Stabilization via angular rate feedback |
| Lesson 8 | PID Control | Attitude PID control and flight |

**→ [Workshop Slides](docs/workshop/slides/README.md)** | **[Workshop Guide](docs/workshop/)**

---

## Technical Specifications

| Item | Specification |
|------|---------------|
| MCU | ESP32-S3 (M5Stamp S3) |
| Framework | ESP-IDF v5.5.2 + FreeRTOS |
| Pose Estimation | ESKF (Error-State Kalman Filter) |
| Communication | ESP-NOW + WiFi (telemetry) |
| Sensors | BMI270, BMM150, BMP280, VL53L3CX, PMW3901 |

---

## License

MIT License
