# vehicle_new — Next Generation Vehicle Firmware
# vehicle_new — 次世代機体ファームウェア

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

StampFlyの次世代制御ファームウェア。旧ファーム（firmware/vehicle/）のスパゲッティ化を解消し、責務の明確化・状態管理の強化・拡張性を実現する再構築プロジェクト。

### 上位目標

- StampFlyの制御ファームウェアとして機能する
- 拡張性を持つ
- プログラミング教材としてコードの見通しが良い
- stampfly_ecosystemの中核ソフトウェアとしての位置づけを保持する

### ターゲット

| 項目 | 仕様 |
|------|------|
| MCU | ESP32-S3 (M5Stamp S3) |
| RTOS | FreeRTOS |
| フレームワーク | ESP-IDF v5.4 |
| 機体 | StampFly 37g Quad-rotor |

## 2. 設計の特徴

### 軽量Pub-Sub

コンポーネント間はトピック経由で通信。発行者と購読者が互いを知らない疎結合設計。

```cpp
// 発行側（IMUタスク）
sensor_imu.publish(imu_data);

// 購読側（状態推定 — IMUを知らない）
sensor_imu.subscribe([this](const ImuData& d) {
    this->onImuData(d);
});
```

### 差替可能な推定・制御

統一インターフェース（IEstimator / IController）により、ESKF→EKF、PID→MPC等の差し替えが可能。

### パラメータシステム

1行のマクロ定義でパラメータ追加が完結。WiFi/CLIからリアルタイム変更、NVS永続化。

```cpp
PARAM_FLOAT("rate.roll.kp", 1.365e-3f, 0.0f, 1.0f, on_pid_changed)
```

### 状態管理の一元化

14コンポーネントの責務を明確に分離。状態遷移はStateTaskが一元管理し、onExit/onEnterコールバックでリセット処理を集約。

## 3. 状態モデル

```
INIT → IDLE(GROUND/HELD) → ARMED_GROUND → TAKEOFF → FLYING → LANDING → IDLE
                                                       │
                                                 ┌─────┼─────┐─────┐
                                                ACRO  STAB  ALT   POS
```

## 4. コンポーネント構成（14）

| # | コンポーネント | 責務 |
|---|---|---|
| 1 | センシング | センサからデータを読み、トピックに発行 |
| 2 | 状態推定 | 姿勢/位置/速度を推定（差替可能） |
| 3 | 状態管理 | モード遷移、ARM許可判定 |
| 4 | フェイルセーフ | 異常検出、alert発行 |
| 5 | 離着陸マネージャー | TAKEOFF/LANDINGモード統括 |
| 6 | 制御 | セットポイント追従演算（差替可能） |
| 7 | アクチュエーション | ミキサー＋モーター出力 |
| 8 | コマンド処理 | 全入力ソース吸収・調停 |
| 9 | 通信 | ESP-NOW/UDP/WiFi/TCP送受信 |
| 10 | ナビゲーター | ウェイポイント、経路計画（将来） |
| 11 | キャリブレーション | バイアス測定・保存・適用 |
| 12 | パラメータ | パラメータ保持・変更・永続化 |
| 13 | データロガー | Telemetry/Data Stream/Blackbox |
| 14 | 通知 | LED/ブザーで状態表示 |

## 5. ディレクトリ構造

```
vehicle_new/
├── docs/               # 設計ドキュメント
├── main/               # app_main(), 固定パラメータ
├── components/
│   ├── sf_core/        # 基盤（Pub-Sub, パラメータ, データ型）
│   ├── sf_state/       # 状態管理
│   ├── sf_estimator/   # 推定インターフェース
│   ├── sf_estimator_eskf/  # ESKF実装
│   ├── sf_controller/  # 制御インターフェース
│   ├── sf_controller_pid/  # PID実装
│   ├── sf_hal_*/       # HALドライバ（独立コピー）
│   └── ...
├── tasks/              # FreeRTOSタスク実装
└── examples/           # サンプル集（チュートリアル付き）
```

## 6. ビルド

```bash
source setup_env.sh
cd firmware/vehicle_new
idf.py build
idf.py flash monitor
```

## 7. ドキュメント

| ドキュメント | 内容 |
|-------------|------|
| [docs/requirements.md](docs/requirements.md) | 要件定義書 |
| [docs/architecture.md](docs/architecture.md) | アーキテクチャ設計書 |
| [docs/detailed_design.md](docs/detailed_design.md) | 詳細設計書 |

## 8. ステータス

| フェーズ | 状態 |
|---------|------|
| 要件定義 | 完了 |
| アーキテクチャ設計 | 完了 |
| 詳細設計 | 完了 |
| 実装 | 未着手 |
| テスト | 未着手 |

---

<a id="english"></a>

## 1. Overview

Next-generation control firmware for StampFly. A rebuild to eliminate spaghetti code in the legacy firmware (firmware/vehicle/), achieving clear responsibility separation, robust state management, and extensibility.

### High-Level Goals

- Function as StampFly's control firmware
- Maintain extensibility
- Provide clear, readable code as programming educational material
- Serve as the core software within stampfly_ecosystem

### Target

| Item | Spec |
|------|------|
| MCU | ESP32-S3 (M5Stamp S3) |
| RTOS | FreeRTOS |
| Framework | ESP-IDF v5.4 |
| Vehicle | StampFly 37g Quad-rotor |

## 2. Design Highlights

### Lightweight Pub-Sub

Components communicate via topics. Publishers and subscribers are decoupled — neither knows the other.

### Replaceable Estimation & Control

Unified interfaces (IEstimator / IController) allow swapping ESKF→EKF, PID→MPC, etc.

### Parameter System

One-line macro definition for parameter addition. Real-time modification via WiFi/CLI with NVS persistence.

### Centralized State Management

14 components with clear responsibility separation. State transitions managed solely by StateTask with onExit/onEnter callbacks.

## 3. Components (14)

| # | Component | Responsibility |
|---|-----------|---------------|
| 1 | Sensing | Read sensors, publish to topics |
| 2 | State Estimation | Estimate attitude/position/velocity (replaceable) |
| 3 | State Management | Mode transitions, ARM permission |
| 4 | Failsafe | Anomaly detection, alert publishing |
| 5 | Takeoff/Landing Mgr | TAKEOFF/LANDING mode orchestration |
| 6 | Control | Setpoint tracking (replaceable) |
| 7 | Actuation | Mixer + motor output |
| 8 | Command Processing | Input source absorption and arbitration |
| 9 | Communication | ESP-NOW/UDP/WiFi/TCP |
| 10 | Navigator | Waypoints, path planning (future) |
| 11 | Calibration | Bias measurement, storage, application |
| 12 | Parameters | Storage, modification, persistence |
| 13 | Data Logger | Telemetry/Data Stream/Blackbox |
| 14 | Notification | LED/buzzer state display |

## 4. Documentation

| Document | Content |
|----------|---------|
| [docs/requirements.md](docs/requirements.md) | Requirements Definition |
| [docs/architecture.md](docs/architecture.md) | Architecture Design |
| [docs/detailed_design.md](docs/detailed_design.md) | Detailed Design |

## 5. Status

| Phase | Status |
|-------|--------|
| Requirements | Done |
| Architecture Design | Done |
| Detailed Design | Done |
| Implementation | Not started |
| Testing | Not started |
