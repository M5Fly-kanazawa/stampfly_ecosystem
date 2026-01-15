# sf sim

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

フライトシミュレータを起動・管理します。VPythonとGenesisの2つのバックエンドをサポートします。

## 2. サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `list` | 利用可能なシミュレータ一覧 |
| `run` | インタラクティブシミュレータ起動 |
| `headless` | ヘッドレスシミュレーション |

## 3. sf sim list

利用可能なシミュレータバックエンドを一覧表示します。

```bash
sf sim list
```

### バックエンド

| バックエンド | 説明 | 環境 |
|-------------|------|------|
| `vpython` | VPython 3D可視化 | システムPython |
| `genesis` | Genesis物理エンジン | 専用venv |

## 4. sf sim run

インタラクティブシミュレータを起動します。

```bash
sf sim run                 # VPythonシミュレータ（デフォルト）
sf sim run vpython         # VPythonシミュレータ
sf sim run genesis         # Genesisシミュレータ
sf sim run --no-joystick   # ジョイスティック無効
```

### オプション

| オプション | 説明 |
|-----------|------|
| `--no-joystick` | ジョイスティック入力を無効化 |

### コントローラ操作

| 入力 | 動作 |
|------|------|
| スロットル（Axis 0） | 推力 |
| ロール（Axis 1） | ロール指令 |
| ピッチ（Axis 2） | ピッチ指令 |
| ヨー（Axis 3） | ヨー指令 |
| Armボタン | シミュレーションリセット |
| Modeボタン | ACRO/STABILIZE切り替え |
| Qキー | 終了 |

## 5. sf sim headless

可視化なしでシミュレーションを実行します（自動テスト用）。

```bash
sf sim headless              # 10秒実行（デフォルト）
sf sim headless -d 30        # 30秒実行
sf sim headless genesis      # Genesisバックエンドで実行
sf sim headless -o log.csv   # 結果をファイル出力
```

### オプション

| オプション | 説明 | デフォルト |
|-----------|------|-----------|
| `-d, --duration` | 実行時間（秒） | 10 |
| `-o, --output` | 出力ログファイル | - |

## 6. シミュレータ詳細

### VPython バックエンド

- **物理**: 2000Hz
- **制御**: 400Hz
- **可視化**: 60Hz
- **特徴**: 軽量、即座に起動

### Genesis バックエンド

- **物理**: 2000Hz
- **制御**: 400Hz
- **可視化**: 30Hz
- **特徴**: 高精度物理、RK4モーターダイナミクス

### Genesis 環境セットアップ

```bash
cd simulator/sandbox/genesis_sim
python3 -m venv venv
source venv/bin/activate
pip install genesis-world pygame
```

---

<a id="english"></a>

## 1. Overview

Launch and manage flight simulators. Supports VPython and Genesis backends.

## 2. Subcommands

| Subcommand | Description |
|------------|-------------|
| `list` | List available simulators |
| `run` | Run interactive simulator |
| `headless` | Run headless simulation |

## 3. sf sim run

Launch interactive simulator.

```bash
sf sim run                 # VPython (default)
sf sim run genesis         # Genesis
sf sim run --no-joystick   # Disable joystick
```

### Controller

| Input | Action |
|-------|--------|
| Throttle (Axis 0) | Thrust |
| Roll (Axis 1) | Roll command |
| Pitch (Axis 2) | Pitch command |
| Yaw (Axis 3) | Yaw command |
| Arm button | Reset simulation |
| Mode button | Toggle ACRO/STABILIZE |
| Q key | Exit |

## 4. sf sim headless

Run simulation without visualization (for automated testing).

```bash
sf sim headless              # 10s (default)
sf sim headless -d 30        # 30s
sf sim headless -o log.csv   # Output to file
```
