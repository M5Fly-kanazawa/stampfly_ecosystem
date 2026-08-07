# StampFly Ecosystem 概要

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このプロジェクトについて

StampFly Ecosystem は、StampFly 機体を中心に、ドローン制御を **設計・実装・実験・解析・教育** の
すべての段階で一貫して扱うための **教育・研究用エコシステム** です。

機体ファームウェア（`firmware/vehicle/`）は ACRO（角速度制御）・STABILIZE（姿勢制御）・ALT_HOLD（高度維持）・
POS_HOLD（位置保持）の4つの飛行モードに対応しており、POS_HOLD は実機飛行での位置保持動作まで確認済みです。
コントローラからの通常のスティック操作に加えて、Tello SDK 互換の Python API（`tools/stampfly_py/`）を使い、
WiFi経由でPCから離陸・着陸・移動・回転などをプログラムで指示することもできます。
なお、旧アーキテクチャのファームウェア（`firmware/vehicle_old/`）は開発を終了し、参照用として残されています。

### 主要コンポーネント

```
┌─────────────────────────────────────────────────────────────────┐
│                    StampFly Ecosystem                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐     通信      ┌──────────────┐              │
│  │  Controller  │ ←──────────→ │   Vehicle    │              │
│  │  (AtomS3)    │  ESP-NOW/UDP │  (StampFly)  │              │
│  └──────────────┘              └──────────────┘              │
│        │                              │                        │
│        │ USB HID                      │ WebSocket              │
│        ↓                              ↓                        │
│  ┌──────────────┐              ┌──────────────┐              │
│  │  Simulator   │              │     GCS      │              │
│  │  (VPython)   │              │  (テレメトリ) │              │
│  └──────────────┘              └──────────────┘              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 2. 通信アーキテクチャ

### 通信モード

Controller と Vehicle は2つの通信モードをサポート：

| モード | プロトコル | 特徴 | 用途 |
|-------|-----------|------|------|
| ESP-NOW | ESP-NOW + TDMA | 低遅延、最大10台同時接続 | 複数機編隊飛行 |
| UDP | UDP over WiFi AP | シンプル、単機運用 | 開発・デバッグ |

### ESP-NOWモード

```
Controller ─── ESP-NOW (TDMA) ───→ Vehicle
     ↑                                ↓
  ビーコン同期              テレメトリ (ESP-NOW)
```

- **TDMA同期**: 最大10台のコントローラが衝突なく同時送信
- **ペアリング必要**: MACアドレスベースのペアリング
- **50Hz 制御周期**: 20msフレーム

### UDPモード

```
Controller ─── WiFi STA ───→ Vehicle (WiFi AP)
     │                            │
     │      192.168.4.1           │
     │                            │
  UDP 8888 ←── Control ──→ UDP Server
  UDP 8889 ←── Telemetry ──
```

- **WiFi経由**: Vehicle がAP、Controller がSTA
- **ペアリング不要**: SSIDで自動接続
- **50Hz 制御周期**: ESP-NOWと同等

### プログラムからの制御（Tello SDK 互換 API）

上記のRCスティック操作用の通信に加えて、Vehicle は **Tello SDK 互換の UDP テキストコマンド**（`UDP:8889`）
を受け付けます。既存の Tello 用 Python プログラム（`djitellopy` 等）がほぼ無改変で動作し、離陸・着陸・
移動・回転・連続操作・各種テレメトリ取得をプログラムから行えます。詳細は
[`tools/stampfly_py/README.md`](../tools/stampfly_py/README.md) を参照してください。

## 3. ディレクトリ構成

```
stampfly_ecosystem/
├── docs/                # ドキュメント
├── firmware/
│   ├── vehicle/         # 機体ファームウェア（主力）
│   ├── vehicle_old/     # レガシー機体ファームウェア（凍結、参照用）
│   ├── controller/      # コントローラファームウェア
│   └── common/          # 共通コード（プロトコル等）
├── protocol/            # 通信プロトコル仕様 (SSOT)
├── control/             # 制御設計資産
├── analysis/            # データ解析
├── tools/               # ユーティリティ
├── simulator/           # シミュレータ
└── examples/            # サンプルコード
```

## 4. 推奨ワークフロー

### 初めての飛行

1. **環境構築**: `./install.sh` で ESP-IDF をセットアップ
2. **ファームウェア書き込み**: `sf build vehicle && sf flash vehicle`
3. **通信モード選択**:
   - 単機: UDPモード (`comm udp`)
   - 複数機: ESP-NOWモード（ペアリング）
4. **飛行**: シミュレータで練習 → 実機で飛行

### 開発ワークフロー

1. **設計**: control/ で制御系を設計
2. **実装**: firmware/vehicle/ でファームウェアを実装
3. **検証**: simulator/ でSILS検証
4. **実験**: 実機でフライトテスト
5. **解析**: analysis/ でログ解析

## 5. 次のステップ

- [はじめに](getting-started.md) - 環境構築と初飛行
- [プロトコル仕様](../protocol/README.md) - 通信プロトコル詳細
- [コントローラ](../firmware/controller/README.md) - コントローラファームウェア
- [TDMA使用ガイド](architecture/tdma-usage.md) - 複数機運用
- [tools/stampfly_py/README.md](../tools/stampfly_py/README.md) - Tello SDK 互換 API 詳細

---

<a id="english"></a>

# StampFly Ecosystem Overview

## 1. Overview

### About This Project

StampFly Ecosystem is an **educational/research platform** for drone control engineering,
covering the complete workflow: **design → implementation → experimentation → analysis → education**.

The vehicle firmware (`firmware/vehicle/`) supports four flight modes — ACRO (rate control),
STABILIZE (attitude control), ALT_HOLD (altitude hold), and POS_HOLD (position hold) — with
POS_HOLD validated on real hardware. In addition to normal stick control from the controller,
a Tello-SDK-compatible Python API (`tools/stampfly_py/`) lets you command takeoff, landing,
moves, and rotation programmatically from a PC over WiFi.
A legacy firmware (`firmware/vehicle_old/`) is retained for reference only; active development
continues in `firmware/vehicle/`.

### Main Components

```
┌─────────────────────────────────────────────────────────────────┐
│                    StampFly Ecosystem                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐  Communication  ┌──────────────┐             │
│  │  Controller  │ ←─────────────→ │   Vehicle    │             │
│  │  (AtomS3)    │   ESP-NOW/UDP   │  (StampFly)  │             │
│  └──────────────┘                 └──────────────┘             │
│        │                                 │                      │
│        │ USB HID                         │ WebSocket            │
│        ↓                                 ↓                      │
│  ┌──────────────┐                 ┌──────────────┐             │
│  │  Simulator   │                 │     GCS      │             │
│  │  (VPython)   │                 │  (Telemetry) │             │
│  └──────────────┘                 └──────────────┘             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 2. Communication Architecture

### Communication Modes

Controller and Vehicle support two communication modes:

| Mode | Protocol | Features | Use Case |
|------|----------|----------|----------|
| ESP-NOW | ESP-NOW + TDMA | Low latency, up to 10 devices | Multi-vehicle formation |
| UDP | UDP over WiFi AP | Simple, single-vehicle | Development/debugging |

### ESP-NOW Mode

- **TDMA sync**: Up to 10 controllers transmit simultaneously without collision
- **Pairing required**: MAC address-based pairing
- **50Hz control rate**: 20ms frame

### UDP Mode

- **WiFi-based**: Vehicle as AP, Controller as STA
- **No pairing needed**: Auto-connect via SSID
- **50Hz control rate**: Same as ESP-NOW

### Programmatic Control (Tello SDK-Compatible API)

In addition to the RC-stick communication above, the Vehicle accepts **Tello SDK-compatible
UDP text commands** (`UDP:8889`). Existing Tello Python programs (e.g. `djitellopy`) run
nearly unmodified, letting you command takeoff, landing, moves, rotation, continuous
control, and telemetry queries programmatically. See
[`tools/stampfly_py/README.md`](../tools/stampfly_py/README.md) for details.

## 3. Directory Structure

```
stampfly_ecosystem/
├── docs/                # Documentation
├── firmware/
│   ├── vehicle/         # Vehicle firmware (primary)
│   ├── vehicle_old/     # Legacy vehicle firmware (frozen, reference only)
│   ├── controller/      # Controller firmware
│   └── common/          # Shared code (protocol, etc.)
├── protocol/            # Protocol specification (SSOT)
├── control/             # Control design assets
├── analysis/            # Data analysis
├── tools/               # Utilities
├── simulator/           # Simulator
└── examples/            # Sample code
```

## 4. Recommended Workflow

### First Flight

1. **Setup**: Run `./install.sh` to set up ESP-IDF
2. **Flash firmware**: `sf build vehicle && sf flash vehicle`
3. **Select mode**:
   - Single vehicle: UDP mode (`comm udp`)
   - Multiple vehicles: ESP-NOW mode (pairing)
4. **Fly**: Practice in simulator → Fly real drone

### Development Workflow

1. **Design**: Design control systems in control/
2. **Implement**: Implement firmware in firmware/vehicle/
3. **Verify**: SILS verification in simulator/
4. **Experiment**: Flight test with real drone
5. **Analyze**: Log analysis in analysis/

## 5. Next Steps

- [Getting Started](getting-started.md) - Setup and first flight
- [Protocol Specification](../protocol/README.md) - Communication protocol details
- [Controller](../firmware/controller/README.md) - Controller firmware
- [TDMA Guide](architecture/tdma-usage.md) - Multi-vehicle operation
- [tools/stampfly_py/README.md](../tools/stampfly_py/README.md) - Tello SDK-compatible API details
