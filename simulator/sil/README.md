# StampFly SIL (host bench)

> **Note:** [English follows the Japanese section.](#english) / 日本語の後に英語版があります。
>
> **設計の正は [`RESET_PLAN.md`](RESET_PLAN.md)。** ここはその実装。

## 1. 概要

物理ベース・MuJoCo・アルゴリズム非依存の SIL（Software-in-the-Loop）。まだ飛んでいない vehicle_new を、ハードを壊さず PC 上で検証する。本体ファームを無改変でコンパイルし、決定論的な疑似 RTOS 上で走らせる（忠実案）。

### ディレクトリ

| 場所 | 役割 |
|------|------|
| `compat/` | ESP-IDF / FreeRTOS のホスト用スタブ。受動レイヤ（esp_log/esp_err/esp_timer/nvs・mutex/queue）＋能動面（`freertos/task.h`） |
| `rtos/` | 決定論的協調 RTOS エミュレータ（疑似OS、P1.1）。単一トークン＋仮想時計の離散事象スケジューラ |
| `physics/` | MuJoCo 物理＋自作のモータ/センサ/風モデル（P1.2） |
| `sim_hal/` | 合成センサを返す SIL 用 HAL ラッパー（`bmi270_wrapper` ＝ P1.1、残り ＝ P1.2） |
| `models/` | 機体の MJCF（`quad_smoke.xml`／`demo_drop.xml` ＝ P1.0、StampFly 完全版 ＝ P1.2） |
| `smoke/` | スモークテスト（`mujoco_smoke`・`cores_smoke` ＝ P1.0、`rtos_smoke` ＝ P1.1） |

### ビルド（スモークテスト）

```bash
cd simulator/sil
# 算法コア＋RTOS エミュレータ（高速・ネット不要）
cmake -S . -B build -DSIL_BUILD_MUJOCO_SMOKE=OFF
cmake --build build
./build/cores_smoke      # 本物の ESKF/PID を host で実行（P1.0）
./build/rtos_smoke       # 実タスクを疑似OS上で走らせ決定論スケジュール（P1.1）

# MuJoCo も含めて（初回は MuJoCo 3.9.0 を取得＝数分）
cmake -S . -B build
cmake --build build
./build/mujoco_smoke models/quad_smoke.xml   # 物理エンジン（P1.0）

# MuJoCo の対話ビューアで目視（GLFW を取得）
cmake -S . -B build -DSIL_MUJOCO_VIEWER=ON
cmake --build build --target simulate
./build/bin/simulate models/demo_drop.xml
```

## 2. ロードマップ

P0（更地化）✅ → **P1（骨格・本書）** → P2（差し替え実証）→ P3（CLI＋ダッシュボード）→ P4（共有用レビュー動画）。各段の詳細とゲートは [`RESET_PLAN.md`](RESET_PLAN.md)。

---

<a id="english"></a>

## 1. Overview

A physics-based, MuJoCo, algorithm-independent SIL (Software-in-the-Loop) bench. It verifies the not-yet-flown vehicle_new on a PC without risking hardware: it compiles the unmodified firmware and runs it on a deterministic emulated RTOS (the "faithful" approach). Design source of truth: [`RESET_PLAN.md`](RESET_PLAN.md).

### Build (P1.0 smoke tests)

```bash
cd simulator/sil
cmake -S . -B build -DSIL_BUILD_MUJOCO_SMOKE=OFF   # cores only (fast)
cmake --build build && ./build/cores_smoke

cmake -S . -B build                                # + MuJoCo (first run fetches 3.9.0)
cmake --build build && ./build/mujoco_smoke models/quad_smoke.xml
```
