# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Session Rules

- **セッション開始時に `PROJECT_PLAN.md` を読むこと**
- **応答は日本語で行うこと**

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
│   ├── vehicle/       # Drone body firmware (the "plant" - 制御対象)
│   ├── controller/    # Remote control firmware (HMI)
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

**Planned:** ESP-IDF for embedded firmware (ESP32 target)

Build commands will be added as implementation progresses. Expected workflow:
```bash
# Firmware build (ESP-IDF)
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
