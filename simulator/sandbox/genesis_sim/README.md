# Genesis Simulator for StampFly

> **Note:** [English version follows after the Japanese section.](#english)

## 1. 概要

Genesis物理エンジンを使用したStampFlyシミュレータの実験環境です。

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
python scripts/01_hello_genesis.py
```

## 3. スクリプト一覧

| スクリプト | 内容 |
|-----------|------|
| `01_hello_genesis.py` | Genesis初期化テスト |
| `02_ground_plane.py` | 地面追加テスト |
| `03_falling_cube.py` | 立方体の自由落下 |
| `04_load_stl.py` | STLファイル読み込み |
| `05_stampfly_fall.py` | StampFly自由落下 |

## 4. トラブルシューティング

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

---

<a id="english"></a>

## 1. Overview

Experimental environment for StampFly simulator using Genesis physics engine.

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
python scripts/01_hello_genesis.py
```

## 3. Scripts

| Script | Description |
|--------|-------------|
| `01_hello_genesis.py` | Genesis initialization test |
| `02_ground_plane.py` | Ground plane test |
| `03_falling_cube.py` | Falling cube simulation |
| `04_load_stl.py` | STL file loading |
| `05_stampfly_fall.py` | StampFly free fall |
