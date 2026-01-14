# コマンド追加ガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

このドキュメントでは、StampFly Ecosystem CLI (`sf`) に新しいコマンドを追加する手順とルールを説明します。

## 2. クイックスタート

### スキャフォールディングツールを使用（推奨）

```bash
# 新しいコマンドを生成
sf dev new-command <command-name>

# 例: "tune" コマンドを追加
sf dev new-command tune

# サブコマンド付きで生成
sf dev new-command tune --subcommands pid,filter,motor
```

このツールは以下を自動生成します：
- `lib/sfcli/commands/<name>.py` - コマンド実装
- `docs/commands/sf-<name>.md` - ドキュメント
- `tests/commands/test_<name>.py` - テストファイル

### 手動で追加

スキャフォールディングツールを使用しない場合は、以下の手順に従ってください。

## 3. ディレクトリ構造

```
lib/sfcli/
├── commands/
│   ├── __init__.py          # コマンド登録
│   ├── build.py             # sf build
│   ├── flash.py             # sf flash
│   ├── log.py               # sf log [capture|analyze|list]
│   └── <your-command>.py    # ★ 新規追加
│
docs/commands/
├── README.md                # コマンド一覧
├── sf-build.md
├── sf-flash.md
└── sf-<your-command>.md     # ★ 新規追加
│
tests/commands/
├── test_build.py
└── test_<your-command>.py   # ★ 新規追加
```

## 4. コマンド実装ルール

### 命名規則

| 項目 | 規則 | 例 |
|------|------|-----|
| コマンド名 | 小文字、ハイフン区切り | `log`, `pid-tune`, `motor-test` |
| ファイル名 | スネークケース | `log.py`, `pid_tune.py`, `motor_test.py` |
| クラス名 | パスカルケース + Command | `LogCommand`, `PidTuneCommand` |
| 関数名 | スネークケース | `register()`, `run_capture()` |

### 必須関数

各コマンドモジュールは以下の関数を実装する必要があります：

```python
def register(subparsers) -> None:
    """コマンドをCLIに登録する"""
    pass

def run(args) -> int:
    """コマンドを実行し、終了コードを返す"""
    pass
```

### テンプレート

```python
"""
sf <command-name> - コマンドの簡潔な説明

詳細な説明（複数行可）
"""

import argparse
from ..utils import console, paths

# コマンドのメタデータ
COMMAND_NAME = "example"
COMMAND_HELP = "Example command description"


def register(subparsers: argparse._SubParsersAction) -> None:
    """コマンドをCLIに登録"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # 引数の定義
    parser.add_argument(
        "target",
        nargs="?",
        default="vehicle",
        choices=["vehicle", "controller"],
        help="Target to operate on (default: vehicle)",
    )

    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output",
    )

    # サブコマンドがある場合
    # subparsers = parser.add_subparsers(dest="subcommand")
    # _register_subcommand_a(subparsers)
    # _register_subcommand_b(subparsers)

    # 実行関数を設定
    parser.set_defaults(func=run)


def run(args: argparse.Namespace) -> int:
    """コマンドを実行

    Args:
        args: パース済み引数

    Returns:
        終了コード（0=成功、1=エラー）
    """
    try:
        # メイン処理
        console.info(f"Running example command with target: {args.target}")

        if args.verbose:
            console.debug("Verbose mode enabled")

        # 成功
        return 0

    except Exception as e:
        console.error(f"Command failed: {e}")
        return 1
```

### サブコマンドを持つコマンドのテンプレート

```python
"""
sf log - ログの取得と解析

サブコマンド:
  capture   リアルタイムログキャプチャ
  analyze   ログファイル解析
  list      ログファイル一覧
"""

import argparse
from ..utils import console

COMMAND_NAME = "log"
COMMAND_HELP = "Log capture and analysis"


def register(subparsers: argparse._SubParsersAction) -> None:
    """コマンドをCLIに登録"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # サブコマンド
    subs = parser.add_subparsers(dest="subcommand", required=True)

    # capture サブコマンド
    capture_parser = subs.add_parser("capture", help="Capture logs in real-time")
    capture_parser.add_argument("-d", "--duration", type=int, default=30,
                                 help="Capture duration in seconds")
    capture_parser.add_argument("-o", "--output", help="Output file path")
    capture_parser.set_defaults(func=run_capture)

    # analyze サブコマンド
    analyze_parser = subs.add_parser("analyze", help="Analyze log file")
    analyze_parser.add_argument("file", help="Log file to analyze")
    analyze_parser.add_argument("--fft", action="store_true", help="Run FFT analysis")
    analyze_parser.set_defaults(func=run_analyze)

    # list サブコマンド
    list_parser = subs.add_parser("list", help="List log files")
    list_parser.add_argument("-n", "--count", type=int, default=10,
                              help="Number of files to show")
    list_parser.set_defaults(func=run_list)


def run_capture(args: argparse.Namespace) -> int:
    """ログキャプチャ実行"""
    console.info(f"Capturing logs for {args.duration} seconds...")
    # 実装
    return 0


def run_analyze(args: argparse.Namespace) -> int:
    """ログ解析実行"""
    console.info(f"Analyzing {args.file}...")
    # 実装
    return 0


def run_list(args: argparse.Namespace) -> int:
    """ログ一覧表示"""
    console.info(f"Listing last {args.count} log files...")
    # 実装
    return 0
```

## 5. コマンド登録

`lib/sfcli/commands/__init__.py` にインポートを追加：

```python
from . import build
from . import flash
from . import monitor
from . import log
from . import example  # ★ 追加

__all__ = [
    "build",
    "flash",
    "monitor",
    "log",
    "example",  # ★ 追加
]
```

## 6. ドキュメント作成

### 必須セクション

`docs/commands/sf-<command>.md` に以下のセクションを含める：

```markdown
# sf <command>

> **Note:** English version follows...

## 概要

コマンドの目的と機能の簡潔な説明。

## 使用方法

\`\`\`bash
sf <command> [options] [arguments]
\`\`\`

## オプション

| オプション | 説明 | デフォルト |
|-----------|------|-----------|
| `-v, --verbose` | 詳細出力 | off |
| `-o, --output` | 出力ファイル | stdout |

## 例

\`\`\`bash
# 基本的な使い方
sf <command> target

# オプション付き
sf <command> --verbose target
\`\`\`

## 関連コマンド

- `sf other-command` - 関連する他のコマンド

---

<a id="english"></a>

(English version)
```

### ドキュメント一覧への追加

`docs/commands/README.md` のコマンド一覧テーブルに追加：

```markdown
| コマンド | 説明 |
|---------|------|
| `sf build` | ファームウェアビルド |
| `sf <new>` | 新しいコマンドの説明 |  ★ 追加
```

## 7. テスト作成

### 必須テスト

```python
# tests/commands/test_example.py
import pytest
from sfcli.commands import example


class TestExampleCommand:
    """example コマンドのテスト"""

    def test_register(self):
        """register() がエラーなく実行される"""
        import argparse
        parser = argparse.ArgumentParser()
        subparsers = parser.add_subparsers()
        example.register(subparsers)

    def test_run_success(self):
        """正常ケースで終了コード0を返す"""
        import argparse
        args = argparse.Namespace(target="vehicle", verbose=False)
        assert example.run(args) == 0

    def test_run_with_verbose(self):
        """verboseオプションが動作する"""
        import argparse
        args = argparse.Namespace(target="vehicle", verbose=True)
        assert example.run(args) == 0
```

### テスト実行

```bash
# 単一コマンドのテスト
pytest tests/commands/test_example.py -v

# 全コマンドのテスト
pytest tests/commands/ -v
```

## 8. チェックリスト

新しいコマンドを追加する際は、以下を確認してください：

- [ ] `lib/sfcli/commands/<name>.py` を作成
- [ ] `register()` と `run()` 関数を実装
- [ ] `commands/__init__.py` にインポートを追加
- [ ] `docs/commands/sf-<name>.md` を作成
- [ ] `docs/commands/README.md` の一覧を更新
- [ ] `tests/commands/test_<name>.py` を作成
- [ ] テストが全て通ることを確認
- [ ] `sf <name> --help` が正しく表示される

## 9. ユーティリティ関数

### console モジュール

```python
from ..utils import console

console.info("情報メッセージ")      # 青色
console.success("成功メッセージ")   # 緑色
console.warning("警告メッセージ")   # 黄色
console.error("エラーメッセージ")   # 赤色
console.debug("デバッグ情報")       # グレー（verbose時のみ）
```

### paths モジュール

```python
from ..utils import paths

paths.root()           # リポジトリルート
paths.firmware()       # firmware/ ディレクトリ
paths.vehicle()        # firmware/vehicle/ ディレクトリ
paths.controller()     # firmware/controller/ ディレクトリ
paths.logs()           # ログ保存ディレクトリ
paths.config()         # 設定ファイルパス
```

### platform モジュール

```python
from ..utils import platform

platform.is_windows()   # Windows環境かどうか
platform.is_macos()     # macOS環境かどうか
platform.is_linux()     # Linux環境かどうか
platform.esp_idf_path() # ESP-IDFのパス（自動検出）
```

---

<a id="english"></a>

## 1. Overview

This document explains the procedures and rules for adding new commands to the StampFly Ecosystem CLI (`sf`).

## 2. Quick Start

### Using Scaffolding Tool (Recommended)

```bash
# Generate a new command
sf dev new-command <command-name>

# Example: add "tune" command
sf dev new-command tune

# Generate with subcommands
sf dev new-command tune --subcommands pid,filter,motor
```

This tool automatically generates:
- `lib/sfcli/commands/<name>.py` - Command implementation
- `docs/commands/sf-<name>.md` - Documentation
- `tests/commands/test_<name>.py` - Test file

## 3. Naming Conventions

| Item | Convention | Example |
|------|------------|---------|
| Command name | lowercase, hyphen-separated | `log`, `pid-tune` |
| File name | snake_case | `log.py`, `pid_tune.py` |
| Class name | PascalCase + Command | `LogCommand` |
| Function name | snake_case | `register()`, `run_capture()` |

## 4. Required Functions

Each command module must implement:

```python
def register(subparsers) -> None:
    """Register command with CLI"""
    pass

def run(args) -> int:
    """Execute command and return exit code"""
    pass
```

## 5. Checklist

When adding a new command, verify:

- [ ] Create `lib/sfcli/commands/<name>.py`
- [ ] Implement `register()` and `run()` functions
- [ ] Add import to `commands/__init__.py`
- [ ] Create `docs/commands/sf-<name>.md`
- [ ] Update `docs/commands/README.md` list
- [ ] Create `tests/commands/test_<name>.py`
- [ ] All tests pass
- [ ] `sf <name> --help` displays correctly
