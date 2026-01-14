# StampFly Ecosystem 移行計画

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### 目的

StampFly Ecosystemを、飛行制御の設計・実装・実験・解析・教育を包括的にサポートする自己完結型開発環境として整備する。

### 達成目標

1. **どこからでも実行可能** - コマンドをPATHに追加し、任意のディレクトリから実行
2. **環境分離** - Python/ESP-IDFをリポジトリ内に閉じ込め、システム環境を汚染しない
3. **クロスプラットフォーム** - macOS/Linux/Windows で同一のユーザー体験
4. **一元管理されたドキュメント** - コマンドリファレンス、セットアップ手順を集約

## 2. 現状

### 既存ディレクトリ構造

```
stampfly-ecosystem/
├── analysis/          # データ解析（notebooks, scripts）
├── control/           # 制御系設計（models, simulation）
├── docs/              # ドキュメント
├── examples/          # 学習用サンプル
├── firmware/          # ファームウェア（vehicle, controller）
├── protocol/          # 通信プロトコル仕様
├── simulator/         # SIL/HILシミュレータ
├── third_party/       # 外部依存
└── tools/             # ユーティリティ（log_capture, calibration等）
```

### 現在の課題

| 課題 | 詳細 |
|------|------|
| 環境依存 | `~/esp/esp-idf` など外部パス依存 |
| 分散したスクリプト | `tools/`, `simulator/scripts/` など複数箇所に散在 |
| 手動セットアップ | Python venv、ESP-IDF初期化が手動 |
| プラットフォーム非対応 | Windowsでの動作が考慮されていない |

## 3. 目標アーキテクチャ

### ディレクトリ構造（変更後）

```
stampfly-ecosystem/
├── bin/                          # グローバルコマンド（シェルラッパー）
│   ├── sf                        # Unix用エントリポイント
│   ├── sf.bat                    # Windows用エントリポイント
│   └── sf.ps1                    # PowerShell用（オプション）
│
├── lib/                          # Python実装
│   └── sfcli/                    # CLIコアライブラリ
│       ├── __init__.py
│       ├── cli.py                # メインエントリポイント
│       ├── commands/             # サブコマンド実装
│       │   ├── build.py
│       │   ├── flash.py
│       │   ├── monitor.py
│       │   ├── log.py
│       │   ├── sim.py
│       │   └── analyze.py
│       └── utils/                # 共通ユーティリティ
│           ├── config.py
│           ├── paths.py
│           └── platform.py
│
├── scripts/                      # セットアップスクリプト
│   ├── install.sh                # Unixインストーラ
│   ├── install.bat               # Windowsインストーラ
│   ├── installer.py              # 共通インストールロジック
│   ├── activate.sh               # Unix環境アクティベート
│   ├── activate.bat              # Windows環境アクティベート
│   └── uninstall.sh              # アンインストール
│
├── .venv/                        # Python仮想環境（.gitignore）
├── .esp-idf/                     # ESP-IDFインストール先（.gitignore、オプション）
│
├── docs/
│   ├── commands/                 # コマンドリファレンス
│   │   ├── README.md             # コマンド一覧・概要
│   │   ├── sf-build.md
│   │   ├── sf-flash.md
│   │   ├── sf-monitor.md
│   │   ├── sf-log.md
│   │   ├── sf-sim.md
│   │   └── sf-analyze.md
│   ├── setup/                    # セットアップガイド
│   │   ├── README.md             # クイックスタート
│   │   ├── macos.md
│   │   ├── linux.md
│   │   └── windows.md
│   └── ...（既存）
│
├── requirements.txt              # Python依存関係
├── pyproject.toml                # プロジェクトメタデータ
│
└── ...（既存ディレクトリは維持）
```

### コマンド体系

```bash
sf <command> [subcommand] [options]
```

| コマンド | 説明 |
|---------|------|
| `sf build [target]` | ファームウェアビルド（vehicle/controller） |
| `sf flash [target]` | フラッシュ書き込み |
| `sf monitor` | シリアルモニタ |
| `sf log capture` | リアルタイムログキャプチャ |
| `sf log analyze <file>` | ログファイル解析 |
| `sf sim run [scenario]` | シミュレータ起動 |
| `sf sim headless` | ヘッドレスシミュレーション |
| `sf cal mag` | 磁気キャリブレーション |
| `sf cal imu` | IMUキャリブレーション |
| `sf version` | バージョン情報表示 |
| `sf doctor` | 環境診断 |

### 環境分離の仕組み

```
┌─────────────────────────────────────────────────────────┐
│ stampfly-ecosystem/                                     │
│  ┌──────────────────────────────────────────────────┐  │
│  │ .venv/  (Python仮想環境)                         │  │
│  │  - numpy, scipy, matplotlib                      │  │
│  │  - pyserial, esptool                             │  │
│  │  - genesis-world (シミュレータ)                   │  │
│  └──────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────┐  │
│  │ .esp-idf/  (ESP-IDF、オプション)                  │  │
│  │  - または外部パスへのシンボリックリンク            │  │
│  └──────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────┐  │
│  │ bin/sf  →  lib/sfcli/  (Pythonで実装)            │  │
│  └──────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

## 4. 移行フェーズ

### Phase 1: 基盤整備

**目標**: インストーラとCLI骨格の構築

| タスク | 詳細 |
|--------|------|
| 1.1 | `requirements.txt` 作成（既存依存関係の集約） |
| 1.2 | `lib/sfcli/` ディレクトリ構造作成 |
| 1.3 | `sf` メインCLI実装（argparse/click） |
| 1.4 | `scripts/installer.py` 実装 |
| 1.5 | `scripts/install.sh` / `install.bat` 作成 |
| 1.6 | `scripts/activate.sh` / `activate.bat` 作成 |
| 1.7 | `.gitignore` 更新（.venv/, .esp-idf/） |

**成果物**:
- `./scripts/install.sh` 実行で環境構築完了
- `sf version` コマンドが動作

### Phase 2: ビルド・フラッシュコマンド

**目標**: ファームウェア開発ワークフローの統合

| タスク | 詳細 |
|--------|------|
| 2.1 | `sf build` 実装（idf.py buildラッパー） |
| 2.2 | `sf flash` 実装（idf.py flashラッパー） |
| 2.3 | `sf monitor` 実装（idf.py monitorラッパー） |
| 2.4 | `sf doctor` 実装（環境診断） |
| 2.5 | ESP-IDF自動検出・設定機能 |

**成果物**:
- どのディレクトリからでも `sf build vehicle` でビルド可能
- ESP-IDFパスの自動検出

### Phase 3: ログ・解析コマンド

**目標**: データ収集・解析ワークフローの統合

| タスク | 詳細 |
|--------|------|
| 3.1 | `sf log capture` 実装（tools/log_capture統合） |
| 3.2 | `sf log analyze` 実装（tools/log_analyzer統合） |
| 3.3 | `sf log list` 実装（ログファイル一覧） |
| 3.4 | 既存スクリプトのsfcli移行 |

**成果物**:
- `sf log capture --duration 30` でログ取得
- `sf log analyze latest --fft` で解析実行

### Phase 4: シミュレータコマンド

**目標**: シミュレーション環境の統合

| タスク | 詳細 |
|--------|------|
| 4.1 | `sf sim run` 実装（simulator/scripts統合） |
| 4.2 | `sf sim headless` 実装 |
| 4.3 | `sf sim list` 実装（シナリオ一覧） |
| 4.4 | シミュレータ設定ファイル管理 |

**成果物**:
- `sf sim run hover` でホバリングシミュレーション起動
- シナリオベースの自動テスト

### Phase 5: キャリブレーション・ユーティリティ

**目標**: ハードウェア調整ツールの統合

| タスク | 詳細 |
|--------|------|
| 5.1 | `sf cal mag` 実装（磁気キャリブレーション） |
| 5.2 | `sf cal imu` 実装（IMUキャリブレーション） |
| 5.3 | `sf cal motor` 実装（モーター特性測定） |
| 5.4 | tools/calibration の sfcli 移行 |

**成果物**:
- インタラクティブキャリブレーションウィザード

### Phase 6: ドキュメント整備

**目標**: 一元管理されたドキュメント

| タスク | 詳細 |
|--------|------|
| 6.1 | `docs/commands/` にコマンドリファレンス作成 |
| 6.2 | `docs/setup/` にプラットフォーム別ガイド作成 |
| 6.3 | README.md 更新（クイックスタート追加） |
| 6.4 | `sf help <command>` でドキュメント表示 |

**成果物**:
- 統一されたコマンドドキュメント
- `sf help build` でヘルプ表示

## 5. 技術詳細

### Python CLI実装

```python
# lib/sfcli/cli.py
import argparse
import sys
from . import commands

def main():
    parser = argparse.ArgumentParser(
        prog='sf',
        description='StampFly Ecosystem CLI'
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    # サブコマンド登録
    commands.build.register(subparsers)
    commands.flash.register(subparsers)
    commands.monitor.register(subparsers)
    commands.log.register(subparsers)
    commands.sim.register(subparsers)
    commands.cal.register(subparsers)

    args = parser.parse_args()
    return args.func(args)

if __name__ == '__main__':
    sys.exit(main())
```

### シェルラッパー（Unix）

```bash
#!/bin/bash
# bin/sf

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

# 仮想環境アクティベート
if [ -f "$VENV_DIR/bin/activate" ]; then
    source "$VENV_DIR/bin/activate"
else
    echo "Error: Virtual environment not found. Run ./scripts/install.sh first."
    exit 1
fi

# ESP-IDF環境（存在する場合）
if [ -f "$SCRIPT_DIR/.esp-idf/export.sh" ]; then
    source "$SCRIPT_DIR/.esp-idf/export.sh" > /dev/null 2>&1
elif [ -f "$HOME/esp/esp-idf/export.sh" ]; then
    source "$HOME/esp/esp-idf/export.sh" > /dev/null 2>&1
fi

# CLIを実行
python -m sfcli "$@"
```

### シェルラッパー（Windows）

```batch
@echo off
REM bin/sf.bat

set SCRIPT_DIR=%~dp0..
set VENV_DIR=%SCRIPT_DIR%\.venv

REM 仮想環境アクティベート
if exist "%VENV_DIR%\Scripts\activate.bat" (
    call "%VENV_DIR%\Scripts\activate.bat"
) else (
    echo Error: Virtual environment not found. Run scripts\install.bat first.
    exit /b 1
)

REM CLIを実行
python -m sfcli %*
```

### インストーラ（共通ロジック）

```python
# scripts/installer.py
import os
import sys
import subprocess
import platform
from pathlib import Path

class Installer:
    def __init__(self):
        self.root = Path(__file__).parent.parent
        self.venv_dir = self.root / '.venv'
        self.is_windows = platform.system() == 'Windows'

    def create_venv(self):
        """Python仮想環境を作成"""
        if self.venv_dir.exists():
            print(f"Virtual environment already exists: {self.venv_dir}")
            return

        print(f"Creating virtual environment: {self.venv_dir}")
        subprocess.run([sys.executable, '-m', 'venv', str(self.venv_dir)], check=True)

    def install_dependencies(self):
        """依存パッケージをインストール"""
        pip = self.venv_dir / ('Scripts' if self.is_windows else 'bin') / 'pip'
        requirements = self.root / 'requirements.txt'

        print("Installing dependencies...")
        subprocess.run([str(pip), 'install', '-r', str(requirements)], check=True)

    def setup_esp_idf(self):
        """ESP-IDFのセットアップ（既存を使用 or 新規インストール）"""
        # 実装省略
        pass

    def configure_shell(self):
        """シェル設定にエイリアスを追加"""
        # 実装省略
        pass

    def run(self):
        """インストール実行"""
        print("=" * 60)
        print("StampFly Ecosystem Installer")
        print("=" * 60)

        self.create_venv()
        self.install_dependencies()
        self.setup_esp_idf()
        self.configure_shell()

        print("=" * 60)
        print("Installation complete!")
        print(f"Run: source {self.root}/scripts/activate.sh")
        print("=" * 60)

if __name__ == '__main__':
    Installer().run()
```

## 6. 設定ファイル

### requirements.txt

```
# Core
numpy>=1.24.0
scipy>=1.10.0
matplotlib>=3.7.0

# Serial/Hardware
pyserial>=3.5
esptool>=4.6

# CLI
click>=8.1.0

# Simulation (optional)
# genesis-world>=0.2.0

# Analysis
pandas>=2.0.0

# Development
pytest>=7.0.0
```

### pyproject.toml

```toml
[project]
name = "stampfly-ecosystem"
version = "0.1.0"
description = "StampFly Flight Control Development Environment"
requires-python = ">=3.10"

[project.scripts]
sf = "sfcli.cli:main"

[tool.setuptools.packages.find]
where = ["lib"]
```

## 7. マイルストーン

| Phase | 目標 | 完了基準 |
|-------|------|---------|
| Phase 1 | 基盤整備 | `sf version` が動作 |
| Phase 2 | ビルド・フラッシュ | `sf build vehicle` が動作 |
| Phase 3 | ログ・解析 | `sf log capture` が動作 |
| Phase 4 | シミュレータ | `sf sim run` が動作 |
| Phase 5 | キャリブレーション | `sf cal mag` が動作 |
| Phase 6 | ドキュメント | 全コマンドのリファレンス完成 |

---

<a id="english"></a>

## 1. Overview

### Purpose

Establish StampFly Ecosystem as a self-contained development environment that comprehensively supports flight control design, implementation, experimentation, analysis, and education.

### Goals

1. **Executable from anywhere** - Add commands to PATH, run from any directory
2. **Environment isolation** - Contain Python/ESP-IDF within repository, avoid system pollution
3. **Cross-platform** - Same user experience on macOS/Linux/Windows
4. **Centralized documentation** - Consolidate command reference and setup guides

## 2. Current State

### Existing Directory Structure

```
stampfly-ecosystem/
├── analysis/          # Data analysis (notebooks, scripts)
├── control/           # Control system design (models, simulation)
├── docs/              # Documentation
├── examples/          # Learning samples
├── firmware/          # Firmware (vehicle, controller)
├── protocol/          # Communication protocol spec
├── simulator/         # SIL/HIL simulator
├── third_party/       # External dependencies
└── tools/             # Utilities (log_capture, calibration, etc.)
```

### Current Issues

| Issue | Details |
|-------|---------|
| Environment dependency | External path dependencies like `~/esp/esp-idf` |
| Scattered scripts | Distributed across `tools/`, `simulator/scripts/`, etc. |
| Manual setup | Python venv, ESP-IDF initialization done manually |
| No platform support | Windows operation not considered |

## 3. Target Architecture

### Directory Structure (After Migration)

```
stampfly-ecosystem/
├── bin/                          # Global commands (shell wrappers)
│   ├── sf                        # Unix entry point
│   ├── sf.bat                    # Windows entry point
│   └── sf.ps1                    # PowerShell (optional)
│
├── lib/                          # Python implementation
│   └── sfcli/                    # CLI core library
│       ├── __init__.py
│       ├── cli.py                # Main entry point
│       ├── commands/             # Subcommand implementations
│       └── utils/                # Common utilities
│
├── scripts/                      # Setup scripts
│   ├── install.sh                # Unix installer
│   ├── install.bat               # Windows installer
│   ├── installer.py              # Common install logic
│   ├── activate.sh               # Unix environment activation
│   └── activate.bat              # Windows environment activation
│
├── .venv/                        # Python virtual environment (.gitignore)
├── .esp-idf/                     # ESP-IDF install location (.gitignore, optional)
│
├── docs/
│   ├── commands/                 # Command reference
│   └── setup/                    # Setup guides
│
├── requirements.txt              # Python dependencies
├── pyproject.toml                # Project metadata
│
└── ... (existing directories maintained)
```

### Command System

```bash
sf <command> [subcommand] [options]
```

| Command | Description |
|---------|-------------|
| `sf build [target]` | Build firmware (vehicle/controller) |
| `sf flash [target]` | Flash firmware |
| `sf monitor` | Serial monitor |
| `sf log capture` | Real-time log capture |
| `sf log analyze <file>` | Analyze log file |
| `sf sim run [scenario]` | Start simulator |
| `sf sim headless` | Headless simulation |
| `sf cal mag` | Magnetometer calibration |
| `sf cal imu` | IMU calibration |
| `sf version` | Show version info |
| `sf doctor` | Environment diagnostics |

## 4. Migration Phases

### Phase 1: Foundation

**Goal**: Build installer and CLI skeleton

- Create `requirements.txt`
- Create `lib/sfcli/` directory structure
- Implement `sf` main CLI
- Implement `scripts/installer.py`
- Create shell wrappers

**Deliverable**: `sf version` works

### Phase 2: Build & Flash Commands

**Goal**: Integrate firmware development workflow

- Implement `sf build`, `sf flash`, `sf monitor`
- Implement `sf doctor`
- ESP-IDF auto-detection

**Deliverable**: `sf build vehicle` works from any directory

### Phase 3: Log & Analysis Commands

**Goal**: Integrate data collection and analysis workflow

- Implement `sf log capture`, `sf log analyze`
- Migrate existing scripts to sfcli

**Deliverable**: `sf log capture --duration 30` works

### Phase 4: Simulator Commands

**Goal**: Integrate simulation environment

- Implement `sf sim run`, `sf sim headless`
- Scenario-based configuration

**Deliverable**: `sf sim run hover` starts simulation

### Phase 5: Calibration & Utilities

**Goal**: Integrate hardware adjustment tools

- Implement `sf cal mag`, `sf cal imu`, `sf cal motor`
- Interactive calibration wizard

**Deliverable**: Calibration tools accessible via CLI

### Phase 6: Documentation

**Goal**: Centralized documentation

- Create command reference in `docs/commands/`
- Create platform-specific setup guides
- Update README.md

**Deliverable**: Complete command documentation

## 5. Milestones

| Phase | Goal | Completion Criteria |
|-------|------|---------------------|
| Phase 1 | Foundation | `sf version` works |
| Phase 2 | Build & Flash | `sf build vehicle` works |
| Phase 3 | Log & Analysis | `sf log capture` works |
| Phase 4 | Simulator | `sf sim run` works |
| Phase 5 | Calibration | `sf cal mag` works |
| Phase 6 | Documentation | All command references complete |
