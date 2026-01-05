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
ドキュメントは以下の構成で記述する：
1. 全ての章・節を日本語で記述
2. 区切り線の後、同じ内容を英語で記述

```markdown
# 第1章 概要
## 1.1 目的
...
## 1.2 背景
...

# 第2章 設計
## 2.1 アーキテクチャ
...

---

# Chapter 1: Overview
## 1.1 Purpose
...
## 1.2 Background
...

# Chapter 2: Design
## 2.1 Architecture
...
```

## Project Overview

StampFly Ecosystem is an educational/research platform for drone control engineering. It covers the complete workflow: **design → implementation → experimentation → analysis → education**.

**Current Status:** Pre-implementation planning phase. Only `PROJECT_PLAN.md` exists; no functional code has been implemented yet.

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
