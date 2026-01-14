# StampFly Ecosystem インストーラー設計

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### 設計思想

**ワンコマンドセットアップ**: `./install.sh` を実行するだけで、何もない環境からでも開発を始められる状態になる。

### 目標

- Python環境がなければインストール（または案内）
- ESP-IDFがなければインストール
- 複数のESP-IDFがあれば選択させる
- sfcliと依存パッケージをESP-IDFのPython環境にインストール
- すべて対話的に、ユーザーの許可を得て実行

## 2. アーキテクチャ

### Python環境の統一

```
┌─────────────────────────────────────────────────────────────┐
│ ESP-IDF Python Environment                                  │
│ (~/.espressif/python_env/idfX.X_pyX.XX_env/)               │
│                                                             │
│  ┌─────────────────────┐  ┌─────────────────────┐          │
│  │ ESP-IDF Tools       │  │ StampFly CLI        │          │
│  │ - esp_idf_monitor   │  │ - sfcli             │          │
│  │ - esptool           │  │ - numpy, scipy      │          │
│  │ - idf.py            │  │ - matplotlib        │          │
│  └─────────────────────┘  └─────────────────────┘          │
│                                                             │
│  単一のPython環境ですべてが動作                               │
└─────────────────────────────────────────────────────────────┘
```

**メリット:**
- 環境の競合が発生しない
- `env -i` などのハックが不要
- ユーザーにとって自然な構成（ESP-IDF環境をアクティベートすればすべて使える）

### ESP-IDFのPythonバージョン選択

ESP-IDFは`install.sh`実行時に、システムで利用可能なPythonを検出して仮想環境を作成する。

```
システムPython 3.12 → ESP-IDF venv (Python 3.12ベース)
システムPython 3.10 → ESP-IDF venv (Python 3.10ベース)
```

したがって、StampFly Ecosystemのインストーラーは以下の順序でチェックする：

1. **Python 3.10+** がシステムに存在するか
2. **ESP-IDF** が存在するか（存在すれば、そのPython環境を使用）

## 3. インストールフロー

```
./install.sh
     │
     ▼
┌─────────────────────────────────────────┐
│ Step 1: Python チェック                  │
│ - Python 3.10+ が存在するか？            │
│ - なければ → インストール案内/自動        │
└─────────────────────────────────────────┘
     │
     ▼
┌─────────────────────────────────────────┐
│ Step 2: ESP-IDF チェック                 │
│ - ESP-IDFが存在するか？                  │
│ - なし → インストール                    │
│ - 1つ → 確認して使用                     │
│ - 複数 → ユーザーに選択させる            │
└─────────────────────────────────────────┘
     │
     ▼
┌─────────────────────────────────────────┐
│ Step 3: sfcli インストール               │
│ - ESP-IDFのPython環境に依存関係を追加    │
│ - sfcli をインストール                   │
└─────────────────────────────────────────┘
     │
     ▼
┌─────────────────────────────────────────┐
│ Step 4: 完了                             │
│ - 設定ファイルを保存                     │
│ - 使い方を表示                           │
└─────────────────────────────────────────┘
```

## 4. 対話インターフェース

### Python未検出時

```
Checking Python...

  Python 3.10+ is required but not found.

  [1] Install Python 3.12 via Homebrew (recommended)
  [2] Open python.org download page
  [3] Cancel

  Select [1]:
```

### ESP-IDF未検出時

```
Checking ESP-IDF...

  ESP-IDF is required for StampFly development.

  [1] Install ESP-IDF v5.4 (recommended)
  [2] Specify custom path
  [3] Cancel

  Select [1]:
```

### ESP-IDF複数検出時

```
Checking ESP-IDF...

  Found 3 ESP-IDF installations:

    [1] v5.4   ~/esp/esp-idf           ← recommended
    [2] v5.1.2 ~/esp/esp-idf-v5.1
    [3] v4.4.7 ~/esp/esp-idf-v4.4      ⚠ may not be compatible
    [4] Install new ESP-IDF

  Select [1]:
```

## 5. ファイル構成

```
stampfly-ecosystem/
├── install.sh              # エントリポイント（Unix）
├── install.bat             # エントリポイント（Windows）
├── scripts/
│   └── installer.py        # メインインストーラー
├── lib/
│   └── sfcli/
│       └── installer/      # インストーラーモジュール
│           ├── __init__.py
│           ├── detector.py   # Python/ESP-IDF検出
│           ├── espidf.py     # ESP-IDFインストール
│           ├── config.py     # 設定ファイル管理
│           └── ui.py         # 対話UI
└── .sf/                    # ローカル設定（.gitignore）
    └── config.toml
```

## 6. 設定ファイル

```toml
# .sf/config.toml
[esp_idf]
path = "~/esp/esp-idf"
version = "5.4"

[project]
default_target = "vehicle"
```

## 7. 設計原則

| 原則 | 説明 |
|------|------|
| ワンコマンド | `./install.sh` だけで完結 |
| 対話的 | 重要な判断はユーザーに確認を求める |
| 冪等性 | 何度実行しても安全（既存環境を壊さない） |
| 透明性 | 何をするか説明してから実行 |
| 単一環境 | ESP-IDFのPython環境にすべてを統合 |

## 8. プラットフォーム対応

| OS | Python入手 | ESP-IDF入手 |
|----|-----------|-------------|
| macOS | Homebrew / python.org | git clone + install.sh |
| Ubuntu/Debian | apt | git clone + install.sh |
| Windows | python.org / winget | ESP-IDF Tools Installer |

## 9. 使用方法（インストール後）

```bash
# ESP-IDF環境をアクティベート
source ~/esp/esp-idf/export.sh

# StampFly CLIを使用
sf --help
sf doctor
sf build
sf flash
sf monitor
```

---

<a id="english"></a>

## 1. Overview

### Design Philosophy

**One-command setup**: Running `./install.sh` takes you from a bare environment to a fully functional development setup.

### Goals

- Install Python if not present (or provide guidance)
- Install ESP-IDF if not present
- Allow selection if multiple ESP-IDF versions exist
- Install sfcli and dependencies into ESP-IDF's Python environment
- All actions are interactive and require user confirmation

## 2. Architecture

### Unified Python Environment

```
┌─────────────────────────────────────────────────────────────┐
│ ESP-IDF Python Environment                                  │
│ (~/.espressif/python_env/idfX.X_pyX.XX_env/)               │
│                                                             │
│  ┌─────────────────────┐  ┌─────────────────────┐          │
│  │ ESP-IDF Tools       │  │ StampFly CLI        │          │
│  │ - esp_idf_monitor   │  │ - sfcli             │          │
│  │ - esptool           │  │ - numpy, scipy      │          │
│  │ - idf.py            │  │ - matplotlib        │          │
│  └─────────────────────┘  └─────────────────────┘          │
│                                                             │
│  Everything runs in a single Python environment             │
└─────────────────────────────────────────────────────────────┘
```

**Benefits:**
- No environment conflicts
- No hacks like `env -i` needed
- Natural for users (activate ESP-IDF environment and everything works)

### ESP-IDF Python Version Selection

ESP-IDF detects the system Python when `install.sh` runs and creates a virtual environment based on it.

```
System Python 3.12 → ESP-IDF venv (based on Python 3.12)
System Python 3.10 → ESP-IDF venv (based on Python 3.10)
```

Therefore, the StampFly Ecosystem installer checks in this order:

1. Does **Python 3.10+** exist on the system?
2. Does **ESP-IDF** exist? (if so, use its Python environment)

## 3. Installation Flow

```
./install.sh
     │
     ▼
┌─────────────────────────────────────────┐
│ Step 1: Python Check                     │
│ - Does Python 3.10+ exist?               │
│ - If not → Install guidance/automatic    │
└─────────────────────────────────────────┘
     │
     ▼
┌─────────────────────────────────────────┐
│ Step 2: ESP-IDF Check                    │
│ - Does ESP-IDF exist?                    │
│ - None → Install                         │
│ - One → Confirm and use                  │
│ - Multiple → Let user select             │
└─────────────────────────────────────────┘
     │
     ▼
┌─────────────────────────────────────────┐
│ Step 3: sfcli Installation               │
│ - Add dependencies to ESP-IDF Python     │
│ - Install sfcli                          │
└─────────────────────────────────────────┘
     │
     ▼
┌─────────────────────────────────────────┐
│ Step 4: Complete                         │
│ - Save configuration                     │
│ - Show usage instructions                │
└─────────────────────────────────────────┘
```

## 4. Design Principles

| Principle | Description |
|-----------|-------------|
| One-command | Just `./install.sh` does everything |
| Interactive | Ask user confirmation for important decisions |
| Idempotent | Safe to run multiple times |
| Transparent | Explain what will be done before doing it |
| Single Environment | Integrate everything into ESP-IDF's Python |

## 5. Usage (After Installation)

```bash
# Activate ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Use StampFly CLI
sf --help
sf doctor
sf build
sf flash
sf monitor
```
