# sf setup

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

オプション依存パッケージをインストールします。

## 2. サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `list` | 利用可能な依存グループ一覧 |
| `sim` | VPythonシミュレータ依存をインストール |
| `genesis` | Genesisシミュレータ依存をインストール |
| `dev` | 開発ツールをインストール |
| `full` | すべてのオプション依存をインストール（Genesis除く） |

## 3. sf setup list

利用可能な依存グループを一覧表示します。

```bash
sf setup list
```

## 4. sf setup sim

VPythonシミュレータに必要な依存をインストールします。

```bash
sf setup sim
```

### インストールされるパッケージ

| パッケージ | 説明 |
|-----------|------|
| vpython | 3D可視化ライブラリ |
| pygame | ジョイスティック入力 |
| numpy-stl | STLファイル読み込み |
| hid | HIDデバイスサポート |

## 5. sf setup genesis

Genesisシミュレータに必要な依存をインストールします。

```bash
sf setup genesis
```

### 注意事項

- **大容量ダウンロード**: PyTorch（~2GB）を含みます
- **GPU推奨**: CUDA対応GPUがあると高速に動作します
- **別途インストール**: `./install.sh`には含まれません

### インストールされるパッケージ

| パッケージ | 説明 |
|-----------|------|
| genesis-world | Genesis物理エンジン |
| torch | PyTorch（機械学習フレームワーク） |
| pygame | ジョイスティック入力 |

## 6. sf setup full

すべてのオプション依存をインストールします（Genesis除く）。

```bash
sf setup full
```

## 7. 代替方法

pipを使った代替インストール方法:

```bash
# VPythonシミュレータ依存のみ
pip install -e '.[sim]'

# すべての依存（Genesis除く）
pip install -e '.[full]'

# requirements.txtを使用（VPython含む）
pip install -r requirements.txt
```

---

<a id="english"></a>

## 1. Overview

Install optional dependencies.

## 2. Subcommands

| Subcommand | Description |
|------------|-------------|
| `list` | List available dependency groups |
| `sim` | Install VPython simulator dependencies |
| `genesis` | Install Genesis simulator dependencies |
| `dev` | Install development tools |
| `full` | Install all optional dependencies (excluding Genesis) |

## 3. sf setup sim

Install dependencies required for VPython simulator.

```bash
sf setup sim
```

### Installed Packages

| Package | Description |
|---------|-------------|
| vpython | 3D visualization library |
| pygame | Joystick input |
| numpy-stl | STL file loading |
| hid | HID device support |

## 4. sf setup genesis

Install dependencies required for Genesis simulator.

```bash
sf setup genesis
```

### Notes

- **Large download**: Includes PyTorch (~2GB)
- **GPU recommended**: Runs faster with CUDA-compatible GPU
- **Separate install**: Not included in `./install.sh`

### Installed Packages

| Package | Description |
|---------|-------------|
| genesis-world | Genesis physics engine |
| torch | PyTorch (ML framework) |
| pygame | Joystick input |

## 5. sf setup full

Install all optional dependencies (excluding Genesis).

```bash
sf setup full
```

## 6. Alternative

Alternative installation using pip:

```bash
# VPython simulator dependencies only
pip install -e '.[sim]'

# All dependencies (excluding Genesis)
pip install -e '.[full]'

# Using requirements.txt (includes VPython)
pip install -r requirements.txt
```
