# tools/

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

利用者に提供するツールは **sf コマンド**として公開する（実体は `lib/sfcli/`、`docs/commands/` にリファレンス）。
`tools/` は主にその sf コマンドのバックエンド実装を置く場所であり、単独実行を前提としたスクリプトは増やさない。

## 2. sf コマンドとの対応

| ディレクトリ | sf コマンド／ツール | 補足 |
|---|---|---|
| `calibration/` | `sf cal` | センサキャリブレーション |
| `log_analyzer/` | `sf log`（capture / wifi / convert / analyze / viz） | UDP 取得の `udp_capture.py` を含む |
| `params_audit/` | `sf params check` / `generate` | 物理パラメータ（C_T/C_Q/κ/慣性 等）の整合検査・生成 |
| `sysid/` | `sf sysid` | 同定・自動チューニング |
| `flasher_gui/` | `sf flasher` | GUI 書き込み。書き込み処理の本体は `lib/sfcli/commands/flash.py` |
| `installer_gui/` | StampFly Setup | GUI インストーラ |
| `stampfly_py/` | — | 配布用 Python SDK サンプル（Tello 互換） |
| `ci/` | — | CI 補助スクリプト |

## 3. sf を経由しない補助

| 対象 | 補足 |
|---|---|
| `slides/` | `docs/events/` のスライド HTML 化。`docs/events/Makefile` から呼ばれる |
| `udev/` | Linux udev ルール |
| `terminal_launcher/` | インストーラの端末起動 |
| `extract_snippets.py` | 教材コードの抜き出し |

sf を経由しない補助は上記 4 つに限る。増やすときは `PROJECT_PLAN.md` §8 に追記する。

## 4. 新しいツールを追加するとき

利用者向けの新規ツールは sf コマンドとして追加する。手順は
[docs/contributing/adding-sf-commands.md](../docs/contributing/adding-sf-commands.md) に従う。

---

<a id="english"></a>

## 1. Overview

Tools offered to end users are published as **sf commands** (implemented in `lib/sfcli/`,
referenced under `docs/commands/`). `tools/` mainly holds the backend implementation behind those
commands; standalone scripts meant to be run on their own are not added here.

## 2. Mapping to sf Commands

| Directory | sf command / tool | Notes |
|---|---|---|
| `calibration/` | `sf cal` | Sensor calibration |
| `log_analyzer/` | `sf log` (capture / wifi / convert / analyze / viz) | Includes `udp_capture.py` for UDP capture |
| `params_audit/` | `sf params check` / `generate` | Checks/generates consistency of physical parameters (C_T/C_Q/κ/inertia, etc.) |
| `sysid/` | `sf sysid` | System identification and auto-tuning |
| `flasher_gui/` | `sf flasher` | GUI flashing. The flashing logic itself lives in `lib/sfcli/commands/flash.py` |
| `installer_gui/` | StampFly Setup | GUI installer |
| `stampfly_py/` | — | Distributed Python SDK sample (Tello-compatible) |
| `ci/` | — | CI helper scripts |

## 3. Helpers Outside sf

| Item | Notes |
|---|---|
| `slides/` | Turns `docs/events/` slides into HTML. Invoked from `docs/events/Makefile` |
| `udev/` | Linux udev rules |
| `terminal_launcher/` | Terminal launcher used by the installer |
| `extract_snippets.py` | Extracts code snippets for teaching material |

Helpers outside sf are limited to these four. Adding another requires an update to
`PROJECT_PLAN.md` §8.


## 4. Adding a New Tool

Add new user-facing tools as sf commands. Follow the procedure in
[docs/contributing/adding-sf-commands.md](../docs/contributing/adding-sf-commands.md).
