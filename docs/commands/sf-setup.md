# sf setup

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

オプション依存パッケージをインストールします。

## 2. サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `list` | 利用可能な依存グループ一覧 |
| `sim` | シミュレータ依存をインストール |
| `dev` | 開発ツールをインストール |
| `full` | すべてのオプション依存をインストール |

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

## 5. sf setup full

すべてのオプション依存をインストールします。

```bash
sf setup full
```

## 6. 代替方法

pipを使った代替インストール方法:

```bash
# シミュレータ依存のみ
pip install -e '.[sim]'

# すべての依存
pip install -e '.[full]'

# requirements.txtを使用
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
| `sim` | Install simulator dependencies |
| `dev` | Install development tools |
| `full` | Install all optional dependencies |

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

## 4. Alternative

Alternative installation using pip:

```bash
# Simulator dependencies only
pip install -e '.[sim]'

# All dependencies
pip install -e '.[full]'

# Using requirements.txt
pip install -r requirements.txt
```
