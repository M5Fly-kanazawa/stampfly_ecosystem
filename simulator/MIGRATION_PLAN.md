# StampFly Simulator 移植計画

既存の [stampfly_sim](https://github.com/kouhei1970/stampfly_sim) を本リポジトリに移植し、
現在の `firmware/vehicle/` 実装と互換性のあるシミュレータに再設計する計画です。

## 1. 既存シミュレータの構成

### ソースファイル一覧

| ファイル | 役割 | 移植先 |
|---------|------|--------|
| `multicopter.py` | クアッドコプタ動力学・制御 | `core/dynamics.py` |
| `rigid_body.py` | 剛体力学計算 | `core/physics.py` |
| `aerodynamics.py` | 空力計算 | `core/aerodynamics.py` |
| `motor_prop.py` | モーター・プロペラモデル | `core/motors.py` |
| `imu_mag.py` | IMU・地磁気センサ | `sensors/imu.py`, `sensors/magnetometer.py` |
| `tof.py` | ToFセンサ | `sensors/tof.py` |
| `pid.py` | PIDコントローラ | `control/pid.py` |
| `joystick.py` | ジョイスティック入力（HID） | `interfaces/joystick.py` |
| `battery.py` | バッテリーシミュレーション | `core/battery.py` |
| `visualizer.py` | 3D可視化（VPython） | `visualization/vpython_renderer.py` |
| `test_sim.py` | メイン実行スクリプト | `scripts/run_sim.py` |
| `sandbox.py` | 開発用テスト環境 | `scripts/sandbox.py` |
| `stl2object.py` | STLファイルパーサ | `assets/loaders/stl_loader.py` |
| `StampFly.stl` | 機体3Dモデル | `assets/meshes/stampfly_v1.stl` |
| `checkerboard.png` | 地面テクスチャ | `assets/textures/checkerboard.png` |

### 現状の制限

- **Acro Mode のみ**: 角速度制御モードのみ実装
- **ATOM Joystick 専用**: HID経由のジョイスティック入力
- **単体動作**: ファームウェアとの連携なし

---

## 2. 移植後の目標構成

```
simulator/
├── README.md
├── MIGRATION_PLAN.md        # 本ドキュメント
├── requirements.txt
│
├── core/                    # 物理エンジン
│   ├── __init__.py
│   ├── physics.py           # rigid_body.py から移植
│   ├── dynamics.py          # multicopter.py から移植
│   ├── aerodynamics.py      # aerodynamics.py から移植
│   ├── motors.py            # motor_prop.py から移植
│   ├── battery.py           # battery.py から移植
│   └── environment.py       # 新規：風、地面、重力
│
├── sensors/                 # センサモデル
│   ├── __init__.py
│   ├── imu.py               # imu_mag.py から分離
│   ├── magnetometer.py      # imu_mag.py から分離
│   ├── barometer.py         # 新規：気圧計
│   ├── tof.py               # tof.py から移植
│   ├── opticalflow.py       # 新規：オプティカルフロー
│   └── noise_models.py      # 新規：Allan分散等
│
├── control/                 # 制御系
│   ├── __init__.py
│   ├── pid.py               # pid.py から移植
│   ├── rate_controller.py   # 新規：firmware互換
│   └── attitude_controller.py # 新規：firmware互換
│
├── interfaces/              # 外部接続
│   ├── __init__.py
│   ├── joystick.py          # joystick.py から移植
│   ├── protocol_bridge.py   # 新規：protocol/準拠I/O
│   ├── sil_interface.py     # 新規：PC版ESKFとの接続
│   └── hil_interface.py     # 新規：実機との接続
│
├── visualization/           # 可視化（複数バックエンド対応）
│   ├── __init__.py
│   ├── base.py              # 共通インターフェース
│   ├── vpython_backend.py   # VPython（既存、最優先）
│   ├── webgl_backend.py     # WebGL（Three.js/Panel）
│   ├── matplotlib_backend.py # Matplotlib 3D
│   ├── unity_backend.py     # Unity連携（高品質レンダリング）
│   ├── mujoco_backend.py    # MuJoCo連携（物理シミュレーション）
│   ├── headless_backend.py  # CIテスト用（描画なし）
│   └── recorder.py          # 動画録画
│
├── scenarios/               # テストシナリオ
│   ├── __init__.py
│   ├── hover.py             # 新規
│   ├── acro_flight.py       # 既存機能ベース
│   └── trajectory.py        # 新規
│
├── configs/                 # 設定
│   ├── stampfly_v1.yaml     # 機体パラメータ
│   ├── sensors.yaml         # センサノイズ設定
│   └── environments/
│       ├── default.yaml
│       └── indoor.yaml
│
├── assets/                  # リソース
│   ├── meshes/
│   │   └── stampfly_v1.stl
│   ├── textures/
│   │   └── checkerboard.png
│   └── loaders/
│       └── stl_loader.py
│
└── scripts/                 # 実行スクリプト
    ├── run_sim.py           # メインシミュレータ
    ├── run_sil.py           # SILモード
    └── sandbox.py           # 開発用
```

---

## 3. Protocol 互換性計画

### 3.1 現在のファームウェア構成

```
firmware/vehicle/
├── components/
│   ├── sf_algo_pid/         # PID実装
│   ├── sf_algo_eskf/        # ESKF実装
│   ├── sf_algo_fusion/      # センサフュージョン
│   └── sf_svc_comm/         # 通信サービス
└── main/
    ├── rate_controller.hpp  # レート制御
    └── config.hpp           # パラメータ
```

### 3.2 protocol/ 定義との整合

シミュレータは `protocol/` で定義されたメッセージ形式を使用：

| メッセージ | 用途 | シミュレータ側 |
|-----------|------|---------------|
| `SensorData` | センサ出力 | sensors/ が生成 |
| `ControlCommand` | 制御入力 | interfaces/ で受信 |
| `StateEstimate` | 状態推定 | ESKF出力と比較 |
| `TelemetryPacket` | テレメトリ | visualization/ で使用 |

### 3.3 制御モード対応計画

| モード | 既存シミュレータ | ファームウェア | 対応状況 |
|-------|-----------------|---------------|---------|
| Acro (Rate) | ○ 実装済み | ○ rate_controller | 移植対象 |
| Angle | × 未実装 | ○ 実装済み | 新規実装 |
| Altitude Hold | × 未実装 | ○ 実装済み | 新規実装 |
| Position Hold | × 未実装 | △ 開発中 | 将来実装 |

---

## 4. 移植作業フェーズ

### Phase 1: 基盤移植（必須）

**目標**: 既存シミュレータをそのまま動作させる

1. [x] ディレクトリ構造作成
2. [x] 既存ファイルをコピー・配置
3. [x] import パス修正
4. [x] requirements.txt 作成
5. [x] 動作確認（既存機能）

**成果物**: Acro Mode で飛行可能なシミュレータ ✓

### Phase 2: センサ拡張

**目標**: 実機センサ構成と一致させる

1. [ ] 気圧計（BMP280）モデル追加
2. [ ] オプティカルフロー（PMW3901）モデル追加
3. [ ] センサノイズモデルを実機特性に合わせる
4. [ ] Allan分散パラメータ導入

**成果物**: 6センサ（IMU, Mag, Baro, ToF, OptFlow）シミュレーション

### Phase 3: Protocol 統合

**目標**: protocol/ 定義に準拠したI/O

1. [ ] `protocol/messages/` の構造体を Python で実装
2. [ ] バイナリログ形式（V2）との互換性
3. [ ] `tools/log_analyzer/` でシミュレータ出力を解析可能に
4. [ ] SIL インターフェース実装

**成果物**: 実機ログと同形式のシミュレータ出力

### Phase 4: 制御系統合

**目標**: ファームウェア制御ロジックとの互換

1. [ ] `sf_algo_pid` のパラメータをシミュレータに反映
2. [ ] `rate_controller.hpp` のロジックを Python で再実装
3. [ ] Angle モード実装
4. [ ] Altitude Hold モード実装

**成果物**: 実機と同じ制御応答のシミュレータ

### Phase 5: HIL対応（オプション）

**目標**: 実機ファームウェアとの接続

1. [ ] シリアル通信インターフェース
2. [ ] リアルタイム同期機構
3. [ ] センサ注入・アクチュエータ読み取り

**成果物**: HIL（Hardware-in-the-Loop）テスト環境

---

## 5. 可視化バックエンド

### バックエンド比較

| バックエンド | 用途 | 特徴 | 優先度 |
|-------------|------|------|--------|
| **VPython** | 開発・デバッグ | 簡単セットアップ、既存コード活用 | ★★★ 最優先 |
| **WebGL** | デモ・共有 | ブラウザで動作、Three.js/Panel利用 | ★★☆ |
| **Matplotlib** | 論文・レポート | 静的プロット、軌跡可視化 | ★★☆ |
| **Unity** | 高品質レンダリング | リアルな環境、VR対応可能 | ★☆☆ |
| **MuJoCo** | 物理精度重視 | 接触・衝突の正確なシミュレーション | ★☆☆ |
| **Headless** | CI/自動テスト | 描画なし、データ出力のみ | ★★★ 必須 |

### 実装方針

1. **共通インターフェース（base.py）**
   - すべてのバックエンドは同一APIを実装
   - `render()`, `update_state()`, `close()` 等

2. **VPython（最優先）**
   - 既存 `visualizer.py` をベースに移植
   - リアルタイム3D表示

3. **WebGL**
   - Three.js をPanel経由で利用
   - Jupyter Notebook対応

4. **Matplotlib**
   - 3D trajectory プロット
   - アニメーション保存対応

5. **Unity**
   - TCP/UDP経由でPythonと通信
   - 別リポジトリとしてUnityプロジェクト管理
   - 将来的にはVR/AR対応

6. **MuJoCo**
   - `mujoco` パッケージ利用
   - XMLモデル定義（MJCF形式）
   - 高精度な接触力学シミュレーション
   - 強化学習研究との連携

---

## 6. 依存関係

### 必須
```
numpy
matplotlib
vpython
```

### オプション（HIL/拡張用）
```
pyserial
hid
pyyaml
mujoco           # MuJoCo物理エンジン連携
```

### 外部ツール（別途インストール）
```
Unity 2022.3+    # Unity連携用（C#プロジェクトとして別管理）
MuJoCo 3.0+      # MuJoCoランタイム（mujoco-pyから自動インストール可）
```

---

## 6. 検証計画

### 単体テスト
- [ ] 剛体力学の数値精度
- [ ] センサノイズ特性
- [ ] PID応答特性

### 統合テスト
- [ ] シミュレータ出力を `tools/log_analyzer/` で解析
- [ ] 実機ログとの比較（同一入力での応答差）

### 回帰テスト
- [ ] 既存 Acro Mode 動作確認
- [ ] ジョイスティック入力確認

---

## 7. 参考資料

- 既存シミュレータ: https://github.com/kouhei1970/stampfly_sim
- ファームウェア: `firmware/vehicle/`
- プロトコル定義: `protocol/`
- ESKF実装: `firmware/vehicle/components/sf_algo_eskf/`
- PID実装: `firmware/vehicle/components/sf_algo_pid/`

---

## 8. 着手チェックリスト

移植作業を開始する前に確認：

- [ ] Python 3.10+ 環境準備
- [ ] VPython インストール確認
- [ ] 既存 stampfly_sim リポジトリのクローン
- [ ] `firmware/vehicle/` の最新状態確認
- [ ] `protocol/` 定義の確認

---

*最終更新: 2026-01-05*
