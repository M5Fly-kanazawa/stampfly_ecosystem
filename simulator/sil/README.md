# StampFly SIL (host bench)

> **Note:** [English follows the Japanese section.](#english) / 日本語の後に英語版があります。
>
> **設計の正は [`RESET_PLAN.md`](RESET_PLAN.md)。** ここはその実装。

## 1. 概要

物理ベース・MuJoCo・アルゴリズム非依存の SIL（Software-in-the-Loop）。まだ飛んでいない vehicle_new を、ハードを壊さず PC 上で検証する。本体ファームを無改変でコンパイルし、決定論的な疑似 RTOS 上で走らせる（忠実案）。

### ディレクトリ

| 場所 | 役割 |
|------|------|
| `compat/` | ESP-IDF / FreeRTOS のホスト用スタブ（esp_log/esp_err/esp_timer/nvs ＝ P1.0、RTOS エミュレータ ＝ P1.1） |
| `rtos/` | 決定論的協調 RTOS エミュレータ（疑似OS、P1.1） |
| `physics/` | MuJoCo 物理＋自作のモータ/センサ/風モデル（P1.2） |
| `sim_hal/` | 合成センサを返す SIL 用 HAL ラッパー（P1.2） |
| `models/` | 機体の MJCF（`quad_smoke.xml` ＝ P1.0 最小、StampFly 完全版 ＝ P1.2） |
| `smoke/` | P1.0 スモークテスト（`mujoco_smoke` / `cores_smoke`） |

### P1.0 のビルド（スモークテスト）

```bash
cd simulator/sil
# 算法コアだけ（高速・ネット不要）
cmake -S . -B build -DSIL_BUILD_MUJOCO_SMOKE=OFF
cmake --build build
./build/cores_smoke

# MuJoCo も含めて（初回は MuJoCo 3.9.0 を取得＝数分）
cmake -S . -B build
cmake --build build
./build/mujoco_smoke models/quad_smoke.xml
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
