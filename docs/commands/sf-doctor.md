# sf doctor

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

開発環境の問題を診断し、解決方法を提案します。

## 2. 構文

```bash
sf doctor
```

## 3. チェック項目

| 項目 | 説明 |
|------|------|
| Environment | 専用環境（dedicated）か旧来環境（legacy）か。専用環境なら実行中Python・IDF_TOOLS_PATH・ESP-IDFの実在も確認（後述） |
| ESP-IDF | ESP-IDF のインストールとバージョン |
| Python | Python バージョンと依存パッケージ |
| Project | プロジェクト構造とファイル |
| Serial | シリアルポートの検出 |

### Environment（環境）節の詳細

[専用環境（dedicated environment）](../plans/dedicated-environment-plan.md)（private Python 3.12 + ESP-IDF v5.5.2 を `SF_HOME` 配下に自己完結させたもの）か、旧来環境（システムPython + 自分で用意したESP-IDF）かを `.sf/config.toml` から判定して表示します。

| 状況 | 表示内容 |
|------|---------|
| `.sf/config.toml` が無い | `No .sf/config.toml (not installed via install.sh/install.bat)` |
| v1設定（`[env]` 節が無い） | `Legacy environment (pre-v2 config). Migrate with: sf upgrade --migrate` |
| `kind = "legacy"` | `Legacy environment (system Python + your own ESP-IDF). Migrate any time with: sf upgrade --migrate` |
| `kind = "dedicated"` | `Dedicated environment: <SF_HOME>` と、以下4項目を追加で確認 |

専用環境の場合の追加チェック（いずれか異常なら警告）:

| チェック | 正常時 | 異常時のメッセージ |
|---------|--------|-------------------|
| ルートディレクトリの実在 | `root: <SF_HOME>` | `root: NOT FOUND (<SF_HOME>)` |
| 実行中インタプリタが専用Pythonか（`sys.base_prefix` が `<SF_HOME>/python` 配下） | `running Python: <path> (dedicated)` | `running Python: sf is running under a different Python than the dedicated one; open a new terminal and run setup_env again`（PATHに移行前の古い`sf`が残っている典型例） |
| `IDF_TOOLS_PATH` が設定と一致 | `IDF_TOOLS_PATH: <path>` | `IDF_TOOLS_PATH is <実際の値>, expected <設定値> -- run setup_env again` |
| 設定された ESP-IDF に `tools/idf.py` が実在 | `ESP-IDF: <path> (<version>)` | `ESP-IDF: NOT FOUND at <path>` |
| `manifest.json` の専用Python版（情報表示のみ、警告にはならない） | `Private Python <version> (release <release>)` | `manifest.json: not readable (<path>)` |

## 4. 出力例

```
[INFO] Running environment diagnostics...

Environment:
  [OK] Dedicated environment: /Users/user/.stampfly
    [OK] root: /Users/user/.stampfly
    [OK] running Python: /Users/user/.stampfly/python (dedicated)
    [OK] IDF_TOOLS_PATH: /Users/user/.stampfly/espressif
    [OK] ESP-IDF: /Users/user/.stampfly/esp-idf (v5.5.2)
    [OK] Private Python 3.12.14 (release 20260901)

ESP-IDF:
  [OK] Found: /Users/user/.stampfly/esp-idf
  [OK] Version: v5.5.2

Python:
  [OK] Version: 3.12.14
  [OK] pyserial installed
  [OK] numpy installed

Project:
  [OK] Root: /path/to/stampfly_ecosystem
  [OK] Vehicle firmware found
  [OK] Controller firmware found

Serial:
  [OK] Port found: /dev/tty.usbmodem14101

[OK] All checks passed!
```

## 5. トラブルシューティング

### ESP-IDF が見つからない

```bash
# 開発環境のセットアップ
source setup_env.sh
```

### pyserial がインストールされていない

```bash
pip install pyserial
```

### 「sf is running under a different Python than the dedicated one」と出る

専用環境へ移行済みなのに、PATH上に移行前の古い `sf`（別のPython）が残っている状態です。ターミナルを開き直すか、`setup_env` を再実行してください。

### 旧来環境（legacy）から専用環境（dedicated）へ移行したい

```bash
sf upgrade --migrate
```

詳細は [sf upgrade](sf-upgrade.md) を参照してください。

---

<a id="english"></a>

## 1. Overview

Diagnose development environment issues and suggest solutions.

## 2. Syntax

```bash
sf doctor
```

## 3. Checks

| Item | Description |
|------|-------------|
| Environment | Dedicated vs. legacy environment. For a dedicated environment, also confirms the running Python, IDF_TOOLS_PATH, and ESP-IDF are actually in place (see below) |
| ESP-IDF | ESP-IDF installation and version |
| Python | Python version and dependencies |
| Project | Project structure and files |
| Serial | Serial port detection |

### Environment section details

Determines from `.sf/config.toml` whether this checkout uses the [dedicated environment](../plans/dedicated-environment-plan.md) (a private Python 3.12 + ESP-IDF v5.5.2, self-contained under `SF_HOME`) or the legacy one (system Python + your own ESP-IDF).

| Situation | Message |
|-----------|---------|
| No `.sf/config.toml` | `No .sf/config.toml (not installed via install.sh/install.bat)` |
| v1 config (no `[env]` section) | `Legacy environment (pre-v2 config). Migrate with: sf upgrade --migrate` |
| `kind = "legacy"` | `Legacy environment (system Python + your own ESP-IDF). Migrate any time with: sf upgrade --migrate` |
| `kind = "dedicated"` | `Dedicated environment: <SF_HOME>`, plus the 4 additional checks below |

Additional checks for a dedicated environment (any failure is a warning):

| Check | OK | Failure message |
|-------|----|-----------------|
| Root directory exists | `root: <SF_HOME>` | `root: NOT FOUND (<SF_HOME>)` |
| The running interpreter is the dedicated Python (`sys.base_prefix` under `<SF_HOME>/python`) | `running Python: <path> (dedicated)` | `running Python: sf is running under a different Python than the dedicated one; open a new terminal and run setup_env again` (typically a stale `sf` still on PATH from before migrating) |
| `IDF_TOOLS_PATH` matches the configured value | `IDF_TOOLS_PATH: <path>` | `IDF_TOOLS_PATH is <actual>, expected <configured> -- run setup_env again` |
| The configured ESP-IDF has `tools/idf.py` | `ESP-IDF: <path> (<version>)` | `ESP-IDF: NOT FOUND at <path>` |
| `manifest.json`'s recorded dedicated Python version (informational only, never a warning) | `Private Python <version> (release <release>)` | `manifest.json: not readable (<path>)` |
