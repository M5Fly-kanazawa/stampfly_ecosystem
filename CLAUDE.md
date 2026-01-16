# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Session Rules

- **セッション開始時またはコンテキスト圧縮後に以下を読むこと:**
  - `PROJECT_PLAN.md`
  - `.claude/settings.local.json`
- **応答は日本語で行うこと**
- **コードを変更したら必ずコミットすること** - 変更をローカルに残さず、適切な単位でコミットする
- **sf CLI を積極的に使用すること** - ビルド、書き込み、診断などは `idf.py` を直接呼ぶのではなく `sf` コマンドを優先する

## Build Environment

### ESP-IDF
ESP-IDFの初期化:
```bash
source ~/esp/esp-idf/export.sh
```

ファームウェアのビルド:
```bash
cd firmware/vehicle  # or firmware/controller
idf.py build
idf.py flash monitor
```

### sf CLI（推奨）
sf CLI は ESP-IDF 環境に統合された開発ツール。`idf.py` を直接使う代わりにこちらを優先する：
```bash
source ~/esp/esp-idf/export.sh  # ESP-IDF環境をアクティブ化（sfを使うため）

sf doctor              # 環境診断（問題があればまずこれを実行）
sf build vehicle       # vehicleファームウェアをビルド
sf build controller    # controllerファームウェアをビルド
sf flash vehicle       # vehicleに書き込み
sf monitor             # シリアルモニタを開く
sf flash vehicle -m    # 書き込み後にモニタを開く
```

**sf CLI の開発・改善:**
- コマンド実装: `lib/sfcli/commands/`
- ユーティリティ: `lib/sfcli/utils/`
- 新コマンド追加時は既存コマンドのパターンに従う
- 問題発見時は積極的に修正してフレームワークを改善する

**ツール統合方針:**
- **全てのツールは sf CLI 経由で使用する** - スタンドアロンの Python スクリプトを直接実行しない
- `tools/` 配下のスクリプトは sf CLI のバックエンド実装として扱う
- 新しいツールを作成する場合は、必ず対応する sf コマンドも追加する
- ラッパースクリプト（viz_*.py 等）は非推奨、sf コマンドのオプションで対応する

**sf CLI コマンド一覧:**
| コマンド | 説明 |
|---------|------|
| `sf doctor` | 環境診断 |
| `sf build [target]` | ファームウェアビルド |
| `sf flash [target]` | 書き込み（-m でモニタ付き）|
| `sf monitor` | シリアルモニタ |
| `sf log list` | ログファイル一覧 |
| `sf log capture` | USB経由バイナリログ取得 |
| `sf log wifi` | WiFi経由400Hzテレメトリ取得 |
| `sf log convert` | バイナリ→CSV変換 |
| `sf log info` | ログファイル情報表示 |
| `sf log analyze` | フライトログ解析 |
| `sf log viz` | ログ可視化 |
| `sf cal list` | キャリブレーション一覧 |
| `sf cal gyro/accel/mag` | 各種キャリブレーション |
| `sf sim list/run` | シミュレータ操作 |

### Genesis Simulator
Genesis物理シミュレータはvenv仮想環境にインストールされている:
```bash
cd simulator/sandbox/genesis_sim
source venv/bin/activate
cd scripts
python <script_name>.py
```

## Writing Conventions

### Code Comments
コメントは英語と日本語を併記する：
```c
// Initialize the motor driver
// モータドライバを初期化
motor_init();
```

### Documentation

ドキュメントは以下のルールに従う：

#### 1. バイリンガル構成（Bilingual Structure）

- 日本語を先に、英語を後に記述
- 冒頭に英語版の存在を示す注記を入れる（`#english` へのリンク付き）
- 英語セクションは `---` で区切り、直前に `<a id="english"></a>` を設置

```markdown
# Document Title

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて
...

### 対象読者
...

## 2. 詳細

### 仕様
...

---

<a id="english"></a>

## 1. Overview

### About This Document
...

### Target Audience
...

## 2. Details

### Specifications
...
```

#### 2. 見出しフォーマット（Heading Format）

| レベル | 形式 | 例 |
|--------|------|-----|
| ドキュメントタイトル | `# Title` | `# StampFly Vehicle Firmware` |
| 章 | `## N. 章タイトル` | `## 1. 概要` / `## 1. Overview` |
| 節 | `### 節タイトル` | `### このプロジェクトについて` |
| 項 | `#### 項タイトル` | `#### パラメータ一覧` |

**注意:**
- 章には番号を付ける（`## 1.`, `## 2.`, ...）
- 節・項には番号を付けない（`## 1.1` ではなく `###`）
- 日本語と英語で同じ番号体系を使う

#### 3. 構造化情報（Structured Information）

リスト形式よりもテーブルを優先する：

```markdown
| 機能 | 説明 |
|------|------|
| IMU | BMI270（加速度・ジャイロ）400Hz |
| 気圧センサー | BMP280 50Hz |
```

#### 4. コードブロック（Code Blocks）

言語を明示する：

````markdown
```cpp
// Initialize motor
motor_init();
```

```bash
idf.py build flash monitor
```
````

#### 5. 図表（Diagrams）

ASCII アートを活用する：

```markdown
```
               Front
          FL (M4)   FR (M1)
             ╲   ▲   ╱
              ╲  │  ╱
               ╲ │ ╱
                ╲│╱
                 ╳         ← Center
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲
             ╱   │   ╲
          RL (M3)    RR (M2)
                Rear
```
```

#### 6. 参考資料（References）

外部リンクはテーブル形式で整理：

```markdown
| リポジトリ | 説明 |
|-----------|------|
| [StampFly技術仕様](https://github.com/...) | ハードウェア仕様書 |
```

## Project Overview

StampFly Ecosystem is an educational/research platform for drone control engineering. It covers the complete workflow: **design → implementation → experimentation → analysis → education**.

**Current Status:** Vehicle firmware (ACRO mode skeleton) and controller firmware are implemented and buildable.

## Architecture

The project uses a **responsibility-based directory structure**:

```
stampfly-ecosystem/
├── docs/              # Human-readable documentation
├── firmware/
│   ├── vehicle/       # Vehicle firmware
│   ├── controller/    # Transmitter firmware
│   └── common/        # Shared embedded code (protocol impl, math, utils)
├── protocol/          # Communication spec - Single Source of Truth (SSOT)
│   ├── spec/          # Machine-readable protocol definition (YAML/proto)
│   ├── generated/     # Auto-generated code from spec
│   └── tools/         # Validation and code generation
├── control/           # Control systems design (models, PID, MPC, SIL)
├── analysis/          # Data analysis (notebooks, scripts, datasets)
├── tools/             # Utilities (flashing, calibration, log capture, CI)
├── simulator/         # SIL/HIL testing environments
├── examples/          # Minimal working examples for learning
└── third_party/       # External dependencies
```

### Key Design Principles

1. **Protocol as Foundation**: All communication implementations derive from `protocol/spec/`. This is the Single Source of Truth.
2. **Responsibility Separation**: Each directory has a clear role. Don't mix concerns across boundaries.
3. **Educational Focus**: Code quality and documentation matter as much as functionality. This is built for students and researchers.

### Firmware Structure (ESP-IDF)

The `firmware/vehicle/` follows ESP-IDF component structure:
- `components/sensors/` - IMU, barometric, ToF, optical flow
- `components/estimation/` - AHRS, EKF, sensor fusion
- `components/control/` - Attitude, angular rate, position control loops
- `components/actuators/` - Motor mixing, saturation, failsafe
- `components/comms/` - Telemetry using protocol spec
- `components/system/` - State management, parameters, diagnostics, CLI
- `main/` - Task initialization and dependency management

## Build System

ESP-IDF for embedded firmware (ESP32 target)。**sf CLI を優先して使用する:**
```bash
# 推奨: sf CLI を使用
source ~/esp/esp-idf/export.sh
sf build vehicle
sf flash vehicle -m

# 代替: idf.py を直接使用（sf で問題がある場合のみ）
cd firmware/vehicle
idf.py build
idf.py flash monitor
```

## Implementation Priority

When developing this codebase, follow this order:
1. **protocol/spec/** - Define communication specification first
2. **firmware/common/protocol/** - Implement protocol encode/decode
3. **firmware/vehicle/** - Basic task structure and sensor integration
4. **examples/** - Minimal working demonstrations
5. **tools/** - Development utilities
6. **control/** and **analysis/** - Design and analysis tooling

## Language Notes

- **Firmware**: C/C++ (ESP-IDF framework)
- **Analysis/Tools**: Python (Jupyter notebooks, scripts)
- **Protocol Spec**: YAML or Protocol Buffers

## Reference

All architectural decisions are documented in `PROJECT_PLAN.md`. Consult this document before making structural changes.
