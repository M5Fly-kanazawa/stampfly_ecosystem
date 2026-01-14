# スキャフォールディングツール設計

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

`sf dev new-command` は、新しいコマンドを追加するためのスキャフォールディング（雛形生成）ツールです。
ボイラープレートコードを自動生成し、開発者が本質的なロジック実装に集中できるようにします。

## 2. 使用方法

### 基本的な使い方

```bash
# シンプルなコマンドを生成
sf dev new-command <name>

# 例
sf dev new-command calibrate
```

生成されるファイル:
```
lib/sfcli/commands/calibrate.py     # コマンド実装
docs/commands/sf-calibrate.md       # ドキュメント
tests/commands/test_calibrate.py    # テスト
```

### サブコマンド付きで生成

```bash
sf dev new-command <name> --subcommands <sub1>,<sub2>,...

# 例
sf dev new-command tune --subcommands pid,filter,motor
```

生成されるコード:
```python
# lib/sfcli/commands/tune.py
def register(subparsers):
    parser = subparsers.add_parser("tune", ...)
    subs = parser.add_subparsers(dest="subcommand", required=True)

    # pid サブコマンド
    pid_parser = subs.add_parser("pid", help="PID tuning")
    pid_parser.set_defaults(func=run_pid)

    # filter サブコマンド
    filter_parser = subs.add_parser("filter", help="Filter tuning")
    filter_parser.set_defaults(func=run_filter)

    # motor サブコマンド
    motor_parser = subs.add_parser("motor", help="Motor tuning")
    motor_parser.set_defaults(func=run_motor)

def run_pid(args): ...
def run_filter(args): ...
def run_motor(args): ...
```

### オプション

| オプション | 説明 | 例 |
|-----------|------|-----|
| `--subcommands`, `-s` | サブコマンドをカンマ区切りで指定 | `--subcommands a,b,c` |
| `--description`, `-d` | コマンドの説明文 | `-d "Tune PID gains"` |
| `--no-test` | テストファイルを生成しない | `--no-test` |
| `--no-doc` | ドキュメントを生成しない | `--no-doc` |
| `--force`, `-f` | 既存ファイルを上書き | `--force` |
| `--dry-run` | 生成内容をプレビュー（ファイル作成なし） | `--dry-run` |

## 3. 自動化される作業

### ファイル生成

| 生成ファイル | 内容 |
|-------------|------|
| `commands/<name>.py` | コマンド実装テンプレート |
| `docs/commands/sf-<name>.md` | バイリンガルドキュメント |
| `tests/commands/test_<name>.py` | 基本テストケース |

### 自動登録

`lib/sfcli/commands/__init__.py` に自動的にインポートを追加:

```python
# 変更前
from . import build
from . import flash

# 変更後（自動追加）
from . import build
from . import flash
from . import calibrate  # ← 追加
```

### ドキュメント一覧更新

`docs/commands/README.md` のコマンド一覧テーブルに自動追加。

## 4. テンプレートシステム

### テンプレートの場所

```
lib/sfcli/templates/
├── command.py.j2           # コマンド実装テンプレート
├── command_with_subs.py.j2 # サブコマンド付きテンプレート
├── doc.md.j2               # ドキュメントテンプレート
└── test.py.j2              # テストテンプレート
```

### テンプレート変数

| 変数 | 説明 | 例 |
|------|------|-----|
| `{{ name }}` | コマンド名（ハイフン区切り） | `pid-tune` |
| `{{ name_snake }}` | スネークケース | `pid_tune` |
| `{{ name_pascal }}` | パスカルケース | `PidTune` |
| `{{ description }}` | コマンドの説明 | `"Tune PID gains"` |
| `{{ subcommands }}` | サブコマンドリスト | `["pid", "filter"]` |
| `{{ date }}` | 生成日 | `2026-01-15` |
| `{{ author }}` | 作者（git configから） | `kouhei` |

### カスタムテンプレート

プロジェクト固有のテンプレートを使用する場合:

```bash
sf dev new-command <name> --template path/to/custom.py.j2
```

## 5. 実装例

### sf dev new-command の実装

```python
# lib/sfcli/commands/dev.py
"""
sf dev - 開発者向けコマンド

サブコマンド:
  new-command   新しいコマンドを生成
  lint          コードスタイルチェック
  test          テスト実行
"""

import argparse
from pathlib import Path
from jinja2 import Environment, FileSystemLoader
from ..utils import console, paths


COMMAND_NAME = "dev"
COMMAND_HELP = "Developer tools"


def register(subparsers):
    parser = subparsers.add_parser(COMMAND_NAME, help=COMMAND_HELP)
    subs = parser.add_subparsers(dest="subcommand", required=True)

    # new-command サブコマンド
    new_cmd = subs.add_parser("new-command", help="Generate a new command")
    new_cmd.add_argument("name", help="Command name")
    new_cmd.add_argument("-s", "--subcommands", help="Comma-separated subcommands")
    new_cmd.add_argument("-d", "--description", default="", help="Command description")
    new_cmd.add_argument("--no-test", action="store_true", help="Skip test generation")
    new_cmd.add_argument("--no-doc", action="store_true", help="Skip doc generation")
    new_cmd.add_argument("-f", "--force", action="store_true", help="Overwrite existing")
    new_cmd.add_argument("--dry-run", action="store_true", help="Preview only")
    new_cmd.set_defaults(func=run_new_command)


def run_new_command(args) -> int:
    """新しいコマンドを生成"""
    name = args.name.lower().replace("_", "-")
    name_snake = name.replace("-", "_")
    name_pascal = "".join(word.capitalize() for word in name.split("-"))

    # パス
    root = paths.root()
    cmd_path = root / "lib" / "sfcli" / "commands" / f"{name_snake}.py"
    doc_path = root / "docs" / "commands" / f"sf-{name}.md"
    test_path = root / "tests" / "commands" / f"test_{name_snake}.py"

    # 既存チェック
    if cmd_path.exists() and not args.force:
        console.error(f"Command already exists: {cmd_path}")
        console.info("Use --force to overwrite")
        return 1

    # テンプレート変数
    context = {
        "name": name,
        "name_snake": name_snake,
        "name_pascal": name_pascal,
        "description": args.description or f"{name_pascal} command",
        "subcommands": args.subcommands.split(",") if args.subcommands else [],
        "date": datetime.date.today().isoformat(),
    }

    # Jinja2環境
    template_dir = root / "lib" / "sfcli" / "templates"
    env = Environment(loader=FileSystemLoader(template_dir))

    # テンプレート選択
    if context["subcommands"]:
        cmd_template = env.get_template("command_with_subs.py.j2")
    else:
        cmd_template = env.get_template("command.py.j2")

    # 生成
    files_to_create = [
        (cmd_path, cmd_template.render(context)),
    ]

    if not args.no_doc:
        doc_template = env.get_template("doc.md.j2")
        files_to_create.append((doc_path, doc_template.render(context)))

    if not args.no_test:
        test_template = env.get_template("test.py.j2")
        files_to_create.append((test_path, test_template.render(context)))

    # ドライラン
    if args.dry_run:
        console.info("Dry run - files that would be created:")
        for path, content in files_to_create:
            console.info(f"  {path}")
            console.debug(content[:200] + "...")
        return 0

    # ファイル作成
    for path, content in files_to_create:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        console.success(f"Created: {path}")

    # __init__.py 更新
    _update_init_file(root, name_snake)

    # README.md 更新
    if not args.no_doc:
        _update_readme(root, name, context["description"])

    console.success(f"\nCommand '{name}' created successfully!")
    console.info(f"Next steps:")
    console.info(f"  1. Edit {cmd_path}")
    console.info(f"  2. Edit {doc_path}")
    console.info(f"  3. Run: pytest {test_path}")

    return 0


def _update_init_file(root: Path, name_snake: str):
    """commands/__init__.py を更新"""
    init_path = root / "lib" / "sfcli" / "commands" / "__init__.py"
    content = init_path.read_text()

    # インポート追加
    import_line = f"from . import {name_snake}"
    if import_line not in content:
        # 最後のimport行の後に追加
        lines = content.split("\n")
        last_import_idx = 0
        for i, line in enumerate(lines):
            if line.startswith("from . import"):
                last_import_idx = i

        lines.insert(last_import_idx + 1, import_line)
        init_path.write_text("\n".join(lines))
        console.success(f"Updated: {init_path}")


def _update_readme(root: Path, name: str, description: str):
    """docs/commands/README.md を更新"""
    readme_path = root / "docs" / "commands" / "README.md"
    if not readme_path.exists():
        return

    content = readme_path.read_text()
    # テーブルの最後に行を追加
    new_row = f"| `sf {name}` | {description} |"

    if new_row not in content:
        # 簡易的な実装: 最後のテーブル行の後に追加
        lines = content.split("\n")
        for i in range(len(lines) - 1, -1, -1):
            if lines[i].startswith("|") and "---" not in lines[i]:
                lines.insert(i + 1, new_row)
                break

        readme_path.write_text("\n".join(lines))
        console.success(f"Updated: {readme_path}")
```

## 6. テンプレートファイル

### command.py.j2

```python
"""
sf {{ name }} - {{ description }}
"""

import argparse
from ..utils import console, paths

COMMAND_NAME = "{{ name }}"
COMMAND_HELP = "{{ description }}"


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # TODO: Add arguments
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output",
    )

    parser.set_defaults(func=run)


def run(args: argparse.Namespace) -> int:
    """Execute {{ name }} command

    Args:
        args: Parsed arguments

    Returns:
        Exit code (0=success, 1=error)
    """
    try:
        # TODO: Implement command logic
        console.info("Running {{ name }} command...")

        return 0

    except Exception as e:
        console.error(f"Command failed: {e}")
        return 1
```

### command_with_subs.py.j2

```python
"""
sf {{ name }} - {{ description }}

Subcommands:
{% for sub in subcommands %}
  {{ sub }}   {{ sub | capitalize }} operation
{% endfor %}
"""

import argparse
from ..utils import console, paths

COMMAND_NAME = "{{ name }}"
COMMAND_HELP = "{{ description }}"


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    subs = parser.add_subparsers(dest="subcommand", required=True)

{% for sub in subcommands %}
    # {{ sub }} subcommand
    {{ sub }}_parser = subs.add_parser("{{ sub }}", help="{{ sub | capitalize }} operation")
    {{ sub }}_parser.set_defaults(func=run_{{ sub }})

{% endfor %}

{% for sub in subcommands %}
def run_{{ sub }}(args: argparse.Namespace) -> int:
    """Execute {{ sub }} subcommand"""
    try:
        console.info("Running {{ name }} {{ sub }}...")
        # TODO: Implement {{ sub }} logic
        return 0
    except Exception as e:
        console.error(f"{{ sub }} failed: {e}")
        return 1

{% endfor %}
```

## 7. メリット

| メリット | 説明 |
|---------|------|
| **一貫性** | 全コマンドが同じ構造を持つ |
| **時間短縮** | ボイラープレートを手書きしない |
| **ミス防止** | 必要なファイルの作成漏れを防ぐ |
| **学習コスト低減** | 新規開発者がすぐに貢献できる |
| **保守性** | テンプレート更新で全体を改善可能 |

---

<a id="english"></a>

## 1. Overview

`sf dev new-command` is a scaffolding tool for adding new commands.
It auto-generates boilerplate code so developers can focus on implementing core logic.

## 2. Usage

### Basic Usage

```bash
sf dev new-command <name>
sf dev new-command calibrate
```

### With Subcommands

```bash
sf dev new-command <name> --subcommands <sub1>,<sub2>,...
sf dev new-command tune --subcommands pid,filter,motor
```

### Options

| Option | Description |
|--------|-------------|
| `--subcommands`, `-s` | Comma-separated subcommands |
| `--description`, `-d` | Command description |
| `--no-test` | Skip test file generation |
| `--no-doc` | Skip documentation generation |
| `--force`, `-f` | Overwrite existing files |
| `--dry-run` | Preview without creating files |

## 3. Automated Tasks

1. Generate command implementation file
2. Generate documentation with bilingual template
3. Generate test file with basic test cases
4. Add import to `commands/__init__.py`
5. Update command list in `docs/commands/README.md`

## 4. Benefits

- **Consistency**: All commands follow the same structure
- **Time-saving**: No manual boilerplate writing
- **Error prevention**: No missing required files
- **Lower learning curve**: New developers can contribute quickly
- **Maintainability**: Template updates improve all generated code
