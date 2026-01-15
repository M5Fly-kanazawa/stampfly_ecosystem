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
python run_genesis_sim.py
```

## 3. スクリプト構成

```
scripts/
├── run_genesis_sim.py    # メインスクリプト（物理量ベース制御）
└── archive/              # 開発履歴・デバッグスクリプト
    ├── 01_hello_genesis.py 〜 24_pid_rate_control.py
    ├── debug_*.py
    └── generate_stampfly_urdf.py
```

開発履歴スクリプトは `archive/` フォルダに格納されています。

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
python run_genesis_sim.py
```

## 3. Script Structure

```
scripts/
├── run_genesis_sim.py    # Main script (physical-unit control)
└── archive/              # Development history & debug scripts
    ├── 01_hello_genesis.py ... 24_pid_rate_control.py
    ├── debug_*.py
    └── generate_stampfly_urdf.py
```

Development history scripts are stored in the `archive/` folder.

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
