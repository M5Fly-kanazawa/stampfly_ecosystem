# StampFly SIL (host bench)

> **Note:** [English follows the Japanese section.](#english) / 日本語の後に英語版があります。
>
> **設計の正は [`RESET_PLAN.md`](RESET_PLAN.md)。** ここはその実装。

## 1. 概要

物理ベース・MuJoCo・アルゴリズム非依存の SIL（Software-in-the-Loop）。vehicle（機体ファーム）を、ハードを壊さず PC 上で検証する。本体ファームを無改変でコンパイルし、決定論的な疑似 RTOS 上で走らせる（忠実案）。

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

### Windows ネイティブビルド（MinGW-w64）

MSVC は firmware 側の指定初期化子（C++17, 約35ファイル）で失敗するため使えない。GCC/MinGW は拡張として許容するため、MSYS2 の MinGW-w64（GCC 16, posix スレッドモデル）でビルドする。`sf sil build` は Windows で MinGW を自動検出し、別ディレクトリ `build-mingw/` を使う（既存の MSVC `build/` には触れない）。

```powershell
# 初回のみ: MSYS2 導入＋ツールチェーン
winget install --id MSYS2.MSYS2 --silent --accept-package-agreements --accept-source-agreements
C:\msys64\usr\bin\bash.exe -lc "pacman -Syu --noconfirm"   # コア更新（要求されたら再実行）
C:\msys64\usr\bin\bash.exe -lc "pacman -S --noconfirm --needed mingw-w64-x86_64-toolchain mingw-w64-x86_64-cmake mingw-w64-x86_64-ninja"

# 以降は sf CLI が MinGW を自動検出（C:\msys64\mingw64\bin）
sf sil build
sf sil scenario simulator/sil/scenarios/pos_flight.scn --target vehicle
```

`sf doctor` が SIL ホストツールチェーン（MinGW-w64 の有無・スレッドモデル）を診断する（未導入は警告のみ、SIL を使わない人には必須でないため）。

**既知の制約:**
- MuJoCo 自身の `mju_error`/`mju_warning`（`%zu` 書式）と `mjz_encoder.cc`（Windows パスの `%s`/`wchar_t*` 不一致）は `-Wno-format` で抑制している（vendored コードにパッチを当てない方針）。どちらも本ベンチの実行経路（`.xml` モデル・`.mjb` 非使用）では到達しない。

**過去に発見・修正済みの Windows 固有バグ（記録として残す）:**
- `hover_smoke`/`rate_tune` は当初、離陸後の高度応答が期待値と異なった（`max_alt` 実測 0.013m、期待 0.5m）。gdb で追跡した結果、原因は Windows/MinGW 固有ではなく、`hover_smoke.cpp` が `system_mode`/`controller_command` を直接注入して StateManager をバイパスする一方、`onTakeoff()`/`onTakeoffComplete()`（PID コントローラ自身の Grounded→TakeoffClimb→Airborne フェーズ機械。通常は state_task.cpp の ARM+スプールドウェル経由で発火）を一度も呼んでいなかったこと ── フェーズが永久に Grounded のまま推力が 0 にクランプされていた。`hover_smoke.cpp` から実際の firmware ハンドシェイク（ALT_HOLD 進入で `ControllerCmd::Takeoff`、`controller_status.takeoff_reached` 確認後に `ControllerCmd::TakeoffComplete`）を発火するよう修正し、実際の自動離陸クライム時間（~3.8秒、旧スケジュールの前提 1.6秒より長い）に合わせてスケジュール定数を再調整。現在は ESKF/相補フィルタ双方・N0ノイズ下で全ゲート PASS。`.scn` シナリオ群（実 ARM/pilot_request 経路を使用）はこの問題の影響を最初から受けていなかった。

## 2. ロードマップ

P0（更地化）✅ → **P1（骨格・本書）** → P2（差し替え実証）→ P3（CLI＋ダッシュボード）→ P4（共有用レビュー動画）。各段の詳細とゲートは [`RESET_PLAN.md`](RESET_PLAN.md)。

---

<a id="english"></a>

## 1. Overview

A physics-based, MuJoCo, algorithm-independent SIL (Software-in-the-Loop) bench. It verifies the vehicle firmware on a PC without risking hardware: it compiles the unmodified firmware and runs it on a deterministic emulated RTOS (the "faithful" approach). Design source of truth: [`RESET_PLAN.md`](RESET_PLAN.md).

### Build (P1.0 smoke tests)

```bash
cd simulator/sil
cmake -S . -B build -DSIL_BUILD_MUJOCO_SMOKE=OFF   # cores only (fast)
cmake --build build && ./build/cores_smoke

cmake -S . -B build                                # + MuJoCo (first run fetches 3.9.0)
cmake --build build && ./build/mujoco_smoke models/quad_smoke.xml
```

### Windows native build (MinGW-w64)

MSVC cannot build this: the firmware uses C++17 designated initializers (~35 files) in a way MSVC rejects but GCC accepts as an extension. Build with MSYS2's MinGW-w64 (GCC 16, posix thread model) instead. `sf sil build` auto-detects MinGW on Windows and uses a separate `build-mingw/` directory (never touches an existing MSVC `build/`).

```powershell
# First time only: install MSYS2 + the toolchain
winget install --id MSYS2.MSYS2 --silent --accept-package-agreements --accept-source-agreements
C:\msys64\usr\bin\bash.exe -lc "pacman -Syu --noconfirm"   # core update (re-run if it asks you to)
C:\msys64\usr\bin\bash.exe -lc "pacman -S --noconfirm --needed mingw-w64-x86_64-toolchain mingw-w64-x86_64-cmake mingw-w64-x86_64-ninja"

# From then on, sf CLI auto-detects MinGW (C:\msys64\mingw64\bin)
sf sil build
sf sil scenario simulator/sil/scenarios/pos_flight.scn --target vehicle
```

`sf doctor` diagnoses the SIL host toolchain (MinGW-w64 presence + thread model); missing is a WARN only, since it is not required unless you use the SIL.

**Known limitations:**
- MuJoCo's own `mju_error`/`mju_warning` (`%zu` format) and `mjz_encoder.cc` (a Windows-path `%s`/`wchar_t*` mismatch) are silenced with `-Wno-format` (policy: no patching vendored code). Neither is reached by this bench's execution path (`.xml` models; the `.mjb` loader is unused).

**Windows-specific bug found and fixed previously (kept for the record):**
- `hover_smoke`/`rate_tune` initially showed a wrong post-takeoff altitude response (`max_alt` measured 0.013 m against an expected 0.5 m). Traced with gdb: the root cause was NOT Windows/MinGW-specific — `hover_smoke.cpp` injects `system_mode`/`controller_command` directly, bypassing StateManager, but never called `onTakeoff()`/`onTakeoffComplete()` (the PID controller's own Grounded→TakeoffClimb→Airborne phase machine, normally fired via state_task.cpp's ARM+spool-dwell sequence) — so the phase stayed Grounded forever with thrust clamped to 0. Fixed by having `hover_smoke.cpp` fire the real firmware handshake directly (`ControllerCmd::Takeoff` on ALT_HOLD entry; `ControllerCmd::TakeoffComplete` once `controller_status.takeoff_reached`), and re-timed the schedule constants to match the real auto-takeoff climb duration (~3.8 s, longer than the old schedule's 1.6 s assumption). All gates now PASS with both ESKF and the complementary filter, and under N0 noise. The `.scn` scenario suite (which drives the real ARM/pilot_request path) was never affected by this.
