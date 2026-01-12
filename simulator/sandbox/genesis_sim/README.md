# Genesis Simulator for StampFly

> **Note:** [English version follows after the Japanese section.](#english)

## 1. 概要

Genesis物理エンジンを使用したStampFlyシミュレータ環境です。物理量ベースの制御（トルク[Nm]）を実装し、2000Hz物理演算・400Hz制御ループで動作します。

### 主な特徴

- 物理量ベース制御（PID出力：トルク[Nm]）
- 2000Hz物理演算、400Hz制御ループ
- RK4ソルバによるモータ/プロペラダイナミクス
- ジョイスティック操作対応
- ボクセルワールド環境

## 2. セットアップ

### 仮想環境の作成

```bash
cd simulator/sandbox/genesis_sim
python3 -m venv venv
source venv/bin/activate
```

### 依存関係のインストール

```bash
pip install --upgrade pip
pip install genesis-world
pip install torch  # PyTorch公式サイトで適切なバージョンを確認
```

### 動作確認

```bash
cd scripts
python 25_physical_units_rate_control.py
```

## 3. スクリプト一覧

### メインスクリプト

| スクリプト | 内容 |
|-----------|------|
| `25_physical_units_rate_control.py` | **物理量ベースレート制御（推奨）** |

### 開発履歴スクリプト

段階的な開発履歴として以下のスクリプトが残っています：

| カテゴリ | スクリプト | 内容 |
|----------|-----------|------|
| 基礎テスト | `01_hello_genesis.py` | Genesis初期化テスト |
| | `02_ground_plane.py` | 地面追加テスト |
| | `03_falling_cube.py` | 立方体の自由落下 |
| | `04_load_stl.py` | STLファイル読み込み |
| | `05_stampfly_fall.py` | StampFly自由落下 |
| 座標系・モデル | `06_coordinate_test.py` | 座標系確認 |
| | `07_stl_orientation_test.py` | STL向きテスト |
| | `08_stampfly_rigid_body.py` | 剛体モデルテスト |
| | `09_stampfly_freefall.py` | StampFly自由落下 |
| | `10_single_body_freefall.py` | 単一ボディ落下 |
| URDF | `11_urdf_test.py` | URDFテスト |
| | `generate_stampfly_urdf.py` | URDF生成 |
| | `12_parts_viewer.py` | パーツビューア |
| | `13_all_parts_viewer.py` | 全パーツビューア |
| | `14_stampfly_viewer.py` | StampFlyビューア |
| | `15_stampfly_freefall.py` | URDF落下テスト |
| トルク・モータ | `16_stampfly_torque_test.py` | トルク応答テスト |
| | `17_realtime_torque_test.py` | リアルタイムトルク |
| | `18_multi_drone_torque_test.py` | マルチドローン |
| | `19_controller_torque_test.py` | コントローラ+トルク |
| | `20_controller_motor_test.py` | コントローラ+モータ |
| ワールド | `21_terrain_world.py` | 地形ワールド |
| | `22_voxel_world.py` | ボクセルワールド |
| | `23_controller_voxel_world.py` | コントローラ+ボクセル |
| 制御開発 | `24_pid_rate_control.py` | PIDレート制御（電圧スケール） |
| | `25_physical_units_rate_control.py` | **物理量ベース制御** |

### デバッグスクリプト

| スクリプト | 内容 |
|-----------|------|
| `debug_hover_only.py` | ホバリング専用テスト |
| `debug_gyro_sign.py` | ジャイロ符号確認 |
| `debug_control_loop.py` | 制御ループデバッグ |
| `debug_torque_response.py` | トルク応答確認 |
| `debug_p_control_only.py` | P制御のみ |
| `debug_no_joystick.py` | ジョイスティックなし |
| `debug_pid_terms.py` | PID各項テスト |
| `debug_viewer_timing.py` | ビューア/ヘッドレスタイミング |

## 4. 制御アーキテクチャ

```
ジョイスティック入力
    ↓
レート制御器 (400Hz)
    ↓
PID → トルク [Nm]
    ↓
制御配分 (Control Allocation)
    ↓
目標推力 [N] → デューティ比
    ↓
モータモデル (RK4, 2000Hz)
    ↓
力/モーメント [N]/[Nm]
    ↓
Genesis物理演算 (2000Hz)
```

詳細は `docs/architecture/genesis-integration.md` を参照。

## 5. トラブルシューティング

### macOSでGPUが使えない場合

CPUバックエンドを使用:
```python
gs.init(backend=gs.cpu)
```

### STLのスケールがおかしい場合

STLはmm単位なので、0.001倍してmに変換:
```python
gs.morphs.Mesh(file='...', scale=0.001)
```

### ビューアモードで発散する場合

制御ループが物理ループの外にある可能性があります。`docs/architecture/genesis-integration.md` のセクション8「リアルタイムシミュレーションのループ構造」を参照してください。

---

<a id="english"></a>

## 1. Overview

StampFly simulator environment using Genesis physics engine. Implements physical-unit-based control (torque [Nm]) with 2000Hz physics / 400Hz control loop.

### Key Features

- Physical-unit-based control (PID output: torque [Nm])
- 2000Hz physics, 400Hz control loop
- RK4 solver for motor/propeller dynamics
- Joystick support
- Voxel world environment

## 2. Setup

### Create virtual environment

```bash
cd simulator/sandbox/genesis_sim
python3 -m venv venv
source venv/bin/activate
```

### Install dependencies

```bash
pip install --upgrade pip
pip install genesis-world
pip install torch  # Check PyTorch website for appropriate version
```

### Verify installation

```bash
cd scripts
python 25_physical_units_rate_control.py
```

## 3. Scripts

### Main Script

| Script | Description |
|--------|-------------|
| `25_physical_units_rate_control.py` | **Physical-unit-based rate control (Recommended)** |

### Development History Scripts

The following scripts remain as incremental development history:

| Category | Script | Description |
|----------|--------|-------------|
| Basic Tests | `01_hello_genesis.py` | Genesis initialization |
| | `02_ground_plane.py` | Ground plane test |
| | `03_falling_cube.py` | Falling cube |
| | `04_load_stl.py` | STL loading |
| | `05_stampfly_fall.py` | StampFly free fall |
| Coordinates/Model | `06_coordinate_test.py` | Coordinate verification |
| | `07_stl_orientation_test.py` | STL orientation |
| | `08-10_*.py` | Rigid body tests |
| URDF | `11-15_*.py` | URDF development |
| Torque/Motor | `16-20_*.py` | Torque and motor tests |
| World | `21-23_*.py` | World environments |
| Control | `24_pid_rate_control.py` | PID (voltage scale) |
| | `25_physical_units_rate_control.py` | **Physical units** |

## 4. Control Architecture

```
Joystick Input
    ↓
Rate Controller (400Hz)
    ↓
PID → Torque [Nm]
    ↓
Control Allocation
    ↓
Target Thrust [N] → Duty
    ↓
Motor Model (RK4, 2000Hz)
    ↓
Force/Moment [N]/[Nm]
    ↓
Genesis Physics (2000Hz)
```

See `docs/architecture/genesis-integration.md` for details.

## 5. Troubleshooting

### GPU not available on macOS

Use CPU backend:
```python
gs.init(backend=gs.cpu)
```

### STL scale issues

STL files are in mm, multiply by 0.001 to convert to meters:
```python
gs.morphs.Mesh(file='...', scale=0.001)
```

### Divergence in viewer mode

Control loop may be outside the physics loop. See section 8 "Real-time Simulation Loop Structure" in `docs/architecture/genesis-integration.md`.
