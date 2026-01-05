# StampFly Ecosystem

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 自分の手で、ドローンを飛ばす制御を作りたいあなたへ

**StampFly Ecosystem** は、ドローン制御を**学び、実装し、実験する**ための
教育・研究プラットフォームです。

「PID制御を教科書で学んだけど、実際に動くものを作りたい」
「姿勢推定アルゴリズムを自分で実装して試したい」
「研究用の飛行実験プラットフォームが欲しい」

そんなあなたのために、このエコシステムは存在します。

---

## 何ができるのか？

| できること | 内容 |
|-----------|------|
| **すぐに飛ばせる** | ビルド済みファームウェアで、箱から出してすぐ飛行可能 |
| **制御を自作できる** | 角速度制御のスケルトンコードを提供。姿勢制御、位置制御を自分で実装 |
| **センサーデータを見れる** | IMU、気圧、ToF、オプティカルフロー等のリアルタイムデータをCLI/WiFiで取得 |
| **実験データを解析できる** | ログを記録し、Pythonで解析・可視化 |

---

## ディレクトリ構成

```
stampfly-ecosystem/
├── docs/           # ドキュメント
├── firmware/       # 組込みファームウェア
│   ├── vehicle/    # 機体ファームウェア
│   ├── controller/ # 送信機ファームウェア
│   └── common/     # 共有コード（構築中）
├── protocol/       # 通信プロトコル仕様（構築中）
├── control/        # 制御設計資産（構築中）
├── analysis/       # 実験データ解析（構築中）
├── tools/          # 補助ツール（構築中）
├── simulator/      # 仮想実験環境（構築中）
└── third_party/    # 外部依存
```

---

## 始めよう

**→ [Getting Started（環境構築〜初フライト）](docs/getting-started.md)**

ESP-IDFのセットアップから、ファームウェアのビルド、
機体と送信機のペアリング、そして初フライトまで。
すべての手順をステップバイステップで解説しています。

---

## 技術仕様

| 項目 | 仕様 |
|------|------|
| MCU | ESP32-S3（M5Stamp S3） |
| フレームワーク | ESP-IDF v5.4.1 + FreeRTOS |
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

## What can you do?

| Capability | Description |
|-----------|-------------|
| **Fly immediately** | Pre-built firmware lets you fly right out of the box |
| **Build your own control** | Rate control skeleton provided. Implement attitude/position control yourself |
| **View sensor data** | Real-time IMU, barometer, ToF, optical flow data via CLI/WiFi |
| **Analyze experiments** | Record flight logs and analyze with Python |

---

## Directory Structure

```
stampfly-ecosystem/
├── docs/           # Documentation
├── firmware/       # Embedded firmware
│   ├── vehicle/    # Vehicle firmware
│   ├── controller/ # Transmitter firmware
│   └── common/     # Shared code (WIP)
├── protocol/       # Communication protocol spec (WIP)
├── control/        # Control design assets (WIP)
├── analysis/       # Experiment data analysis (WIP)
├── tools/          # Utility tools (WIP)
├── simulator/      # Virtual testing environment (WIP)
└── third_party/    # External dependencies
```

---

## Get Started

**→ [Getting Started (Setup to First Flight)](docs/getting-started.md)**

From ESP-IDF setup to firmware build,
pairing vehicle and controller, and your first flight.
All steps explained step-by-step.

---

## Technical Specifications

| Item | Specification |
|------|---------------|
| MCU | ESP32-S3 (M5Stamp S3) |
| Framework | ESP-IDF v5.4.1 + FreeRTOS |
| Pose Estimation | ESKF (Error-State Kalman Filter) |
| Communication | ESP-NOW + WiFi (telemetry) |
| Sensors | BMI270, BMM150, BMP280, VL53L3CX, PMW3901 |

---

## License

MIT License
