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

### シナリオの一時パラメータ上書き（`--param`）

`sf sil scenario`（`sysid-gate` も同様）は `--param NAME=VALUE` を繰り返し指定できる。ゲイン等のファームパラメータをリビルド無しで一時的に試すためのオプションで、ファームの永続パラメータや SSOT（`params.cpp` テーブル）そのものは変更しない。

```bash
sf sil scenario simulator/sil/scenarios/acro_flight.scn --param rate.roll.kp=0.5e-3 --param rate.roll.ti=0.03
```

存在しないパラメータ名は無視され、警告付きでスキップされる（クラッシュしない）。`--param` を指定しない通常実行は従来通り（byte-identical）。

**既知の制約:**
- MuJoCo 自身の `mju_error`/`mju_warning`（`%zu` 書式）と `mjz_encoder.cc`（Windows パスの `%s`/`wchar_t*` 不一致）は `-Wno-format` で抑制している（vendored コードにパッチを当てない方針）。どちらも本ベンチの実行経路（`.xml` モデル・`.mjb` 非使用）では到達しない。

**過去に発見・修正済みの Windows 固有バグ（記録として残す）:**
- `hover_smoke`/`rate_tune` は当初、離陸後の高度応答が期待値と異なった（`max_alt` 実測 0.013m、期待 0.5m）。gdb で追跡した結果、原因は Windows/MinGW 固有ではなく、`hover_smoke.cpp` が `system_mode`/`controller_command` を直接注入して StateManager をバイパスする一方、`onTakeoff()`/`onTakeoffComplete()`（PID コントローラ自身の Grounded→TakeoffClimb→Airborne フェーズ機械。通常は state_task.cpp の ARM+スプールドウェル経由で発火）を一度も呼んでいなかったこと ── フェーズが永久に Grounded のまま推力が 0 にクランプされていた。`hover_smoke.cpp` から実際の firmware ハンドシェイク（ALT_HOLD 進入で `ControllerCmd::Takeoff`、`controller_status.takeoff_reached` 確認後に `ControllerCmd::TakeoffComplete`）を発火するよう修正し、実際の自動離陸クライム時間（~3.8秒、旧スケジュールの前提 1.6秒より長い）に合わせてスケジュール定数を再調整。現在は ESKF/相補フィルタ双方・N0ノイズ下で全ゲート PASS。`.scn` シナリオ群（実 ARM/pilot_request 経路を使用）はこの問題の影響を最初から受けていなかった。

### `sf sil fly` — リアルタイム・キーボード操縦（P6 stage 1）

「コントローラで操縦できるグラフィカルシムが実ファームに無い」の解消・第1段。ターミナルのキーボードで、無改変の実ファーム（`emu_vehicle`）をリアルタイムに操縦できる。ブラウザ/ゲームパッド対応は第2段。

```bash
sf sil build          # 初回のみ（または firmware/vehicle 変更後）
sf sil fly
```

**起動:**
- `sf sil fly` は `emu_vehicle` を `SIL_EMU_REALTIME=1`（決定論スケジューラの仮想時計を壁時計にペーシング — 進みすぎたら待ち、遅れは追いつくのみ）と `SIL_EMU_RC_STDIN=1`（stdin から `rc`/`arm`/`land`/`quit` 行を非ブロッキングで読みRCとして注入）付きで起動する。
- どちらの環境変数も未設定の通常実行（`sf sil scenario`・`sf sil regression` 等）には一切影響しない — 決定論性（byte-identical）は絶対条件として維持している。

**キー割当:**

| キー | 動作 |
|------|------|
| W / S | ピッチ 前進 / 後退 |
| A / D | ロール 左 / 右 |
| , / . | ヨー 左 / 右 |
| Space / Z | スロットル 上げ / 下げ |
| R | ARM/DISARM 切替（1回タップ — 実機送信機のモーメンタリボタンと同じ） |
| + / - | スティック振れ幅（10〜100） |
| Q | 着陸してから終了 |
| Ctrl+C | 即終了（着陸なし） |

`sf rc`（実機用キーボードモード、`lib/sfcli/commands/rc.py`）の慣例に極力揃えたが、ヨーだけ `,`/`.` にした — `Q` を「着陸＋終了」専用に予約したため（`sf rc` の `Q`/`E` は使えない）。

**ARM 手順:** ARM はファームの状態機械が「ARM ワイヤビットの立ち上がり→立ち下がりエッジ」をトグルとして扱う（`state_task.cpp`、実機送信機のモーメンタリボタンと同じ規約）。`scenarios/stab_flight.scn` の ARM シーケンス（4 秒の中立保持で起動校正完了 → ARM 押下 → スロットル投入で離陸）と同じスティック操作がキーボードでも成立することを確認済み（`simulator/tests/test_realtime_fly.py` の自動テスト参照）。手順:
1. 起動直後は中立（何も押さない）のまま数秒待ち、起動校正が完了して `IDLE_GROUND` になるのを HUD の `mode=` で確認する。
2. `R` を1回タップして ARM（`ARMED_GROUND` に遷移）。
3. `Space` でスロットルを上げると自動的に `TAKEOFF` → `FLYING` に進む。

**`--scenario <path>`:** ARM 済み（またはさらに先の状態）まで進めてからキーボード操縦を引き継ぎたい場合に使う。シナリオのタイムラインが尽きるとドライバタスクが自動終了し、以降はキーボード入力のみが効く。**既知の制約:** シナリオ再生中にスティックキーを触ると、シナリオ自身の RC 注入とキーボードの RC 注入が独立に（同じ経路で）書き込むため、同一瞬間に競合する可能性がある — シナリオ再生が終わるまでスティックには触れないこと。

**`--param NAME=VALUE`:** `sf sil scenario --param` と同じ機構（一時上書き、SSOT 非変更）。

**第2段の予告:** ブラウザ UI（`sf sil gui` 相当のライブ操縦版）とゲームパッド入力。

### Python バインディング（pybind11・`stampfly_control`、P5 stage 1）

**目的:** ファームの C++ 制御則（`firmware/vehicle/components/sf_controller_pid/include/pid.hpp` の `sf::PID`）と、その Python 再実装（`tools/log_analyzer/rate_sysid.py` の `replay_pid()` 等）は、これまで「手で同期を保つ」運用だった ── pid.hpp が変わっても両者が一致し続ける保証が構造的になく、同期ドリフトのリスクがあった。本バインディングは pid.hpp を**無改変**でコンパイルし `stampfly_control.PID` として Python から直接呼べるようにすることで、翻訳ではなく本物の C++ 実装そのものを実行し、この手動同期リスクを解消する。

**ビルド:** `sf sil build`（または手動 `cmake` ビルド）で自動的にビルドされ、`simulator/sil/build/stampfly_control.*.so`（例: `stampfly_control.cpython-312-darwin.so`）が生成される。Python 開発ヘッダが見つからない環境では `SIL_BUILD_PYBIND_CONTROL` オプションが警告付きでスキップされる（configure 自体は失敗しない）。無効化する場合: `cmake -S . -B build -DSIL_BUILD_PYBIND_CONTROL=OFF`。

**lockstep テストの実行:**
```bash
source setup_env.sh && sf sil build
pytest simulator/tests/test_pid_lockstep.py -v
```
`stampfly_control.PID`（本物のファーム）と `rate_sysid.replay_pid()`（手動移植）を同一の入力列で1ステップずつ駆動し、出力を数値比較する。詳細は同テストファイル内のコメント参照。

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

### Temporary parameter overrides for a scenario run (`--param`)

`sf sil scenario` (and `sysid-gate`) accepts a repeatable `--param NAME=VALUE`. It is meant for trying a gain or other firmware param for a single run without rebuilding — it never touches firmware's persistent params or the SSOT (`params.cpp` table).

```bash
sf sil scenario simulator/sil/scenarios/acro_flight.scn --param rate.roll.kp=0.5e-3 --param rate.roll.ti=0.03
```

Unknown param names are skipped with a warning (never a crash). A normal run without `--param` is unaffected (byte-identical).

**Known limitations:**
- MuJoCo's own `mju_error`/`mju_warning` (`%zu` format) and `mjz_encoder.cc` (a Windows-path `%s`/`wchar_t*` mismatch) are silenced with `-Wno-format` (policy: no patching vendored code). Neither is reached by this bench's execution path (`.xml` models; the `.mjb` loader is unused).

**Windows-specific bug found and fixed previously (kept for the record):**
- `hover_smoke`/`rate_tune` initially showed a wrong post-takeoff altitude response (`max_alt` measured 0.013 m against an expected 0.5 m). Traced with gdb: the root cause was NOT Windows/MinGW-specific — `hover_smoke.cpp` injects `system_mode`/`controller_command` directly, bypassing StateManager, but never called `onTakeoff()`/`onTakeoffComplete()` (the PID controller's own Grounded→TakeoffClimb→Airborne phase machine, normally fired via state_task.cpp's ARM+spool-dwell sequence) — so the phase stayed Grounded forever with thrust clamped to 0. Fixed by having `hover_smoke.cpp` fire the real firmware handshake directly (`ControllerCmd::Takeoff` on ALT_HOLD entry; `ControllerCmd::TakeoffComplete` once `controller_status.takeoff_reached`), and re-timed the schedule constants to match the real auto-takeoff climb duration (~3.8 s, longer than the old schedule's 1.6 s assumption). All gates now PASS with both ESKF and the complementary filter, and under N0 noise. The `.scn` scenario suite (which drives the real ARM/pilot_request path) was never affected by this.

### `sf sil fly` — real-time keyboard control (P6 stage 1)

Stage 1 of closing the "no graphical sim you can fly with a controller against the real firmware" gap. Pilots the real, unmodified firmware (`emu_vehicle`) in real time from the terminal keyboard. Browser/gamepad support is stage 2.

```bash
sf sil build          # once (or after changing firmware/vehicle)
sf sil fly
```

**How it works:**
- `sf sil fly` launches `emu_vehicle` with `SIL_EMU_REALTIME=1` (paces the deterministic scheduler's virtual clock to the wall clock — sleeps when ahead, never tries to catch up when behind) and `SIL_EMU_RC_STDIN=1` (reads `rc`/`arm`/`land`/`quit` lines from stdin, non-blocking, and injects them as RC).
- Neither env var touches a normal run (`sf sil scenario`, `sf sil regression`, ...) that leaves both unset — determinism (byte-identical output) is kept as an absolute non-negotiable.

**Key map:**

| Key | Action |
|-----|--------|
| W / S | Pitch forward / back |
| A / D | Roll left / right |
| , / . | Yaw left / right |
| Space / Z | Throttle up / down |
| R | ARM/DISARM toggle (tap once — mirrors a real transmitter's momentary button) |
| + / - | Stick deflection (10-100) |
| Q | Land, then quit |
| Ctrl+C | Quit immediately (no land) |

Matches `sf rc`'s keyboard-mode conventions (`lib/sfcli/commands/rc.py`) as closely as possible, except yaw moved to `,`/`.` — `Q` is reserved for "land then quit" here, so `sf rc`'s `Q`/`E` were unavailable.

**ARM sequence:** the firmware's state machine treats a rising-then-falling edge on the ARM wire bit as a toggle (`state_task.cpp`, the same convention a real transmitter's momentary button uses). The same stick sequence `scenarios/stab_flight.scn` uses (4 s neutral hold for boot calibration → ARM press → throttle up for takeoff) has been verified to work from the keyboard too (see the automated check in `simulator/tests/test_realtime_fly.py`). Steps:
1. Right after launch, hold neutral (touch nothing) for a few seconds and watch the HUD's `mode=` field reach `IDLE_GROUND` (boot calibration complete).
2. Tap `R` once to ARM (transitions to `ARMED_GROUND`).
3. Raise the throttle with `Space` — the firmware auto-advances `TAKEOFF` → `FLYING`.

**`--scenario <path>`:** use this to advance to ARMED (or further) before keyboard control takes over. The scenario driver task ends itself once its timeline is exhausted; only your keyboard input drives RC after that. **Known limitation:** touching the stick keys while the scenario is still replaying can race — the scenario's own RC injection and your keyboard's write through the same path independently, so whichever writes last within a given instant wins. Leave the sticks alone until the scenario finishes.

**`--param NAME=VALUE`:** same mechanism as `sf sil scenario --param` (temporary override, never touches the SSOT).

**Coming in stage 2:** a browser UI (a live-piloting counterpart to `sf sil gui`) and gamepad input.

### Python bindings (pybind11, `stampfly_control`, P5 stage 1)

**Purpose:** the firmware C++ control law (`sf::PID` in `firmware/vehicle/components/sf_controller_pid/include/pid.hpp`) and its Python re-implementations (e.g. `tools/log_analyzer/rate_sysid.py`'s `replay_pid()`) used to be "kept in sync by hand" — nothing enforced that the two stayed identical as pid.hpp evolved, a structural sync-drift risk. This binding compiles pid.hpp **unmodified** and exposes it as `stampfly_control.PID`, so Python calls the real C++ implementation directly instead of a translation, eliminating that hand-sync risk.

**Build:** built automatically by `sf sil build` (or a manual `cmake` build), producing `simulator/sil/build/stampfly_control.*.so` (e.g. `stampfly_control.cpython-312-darwin.so`). On a machine without Python development headers, the `SIL_BUILD_PYBIND_CONTROL` option WARNs and skips it (configure itself does not fail). Disable it explicitly with `cmake -S . -B build -DSIL_BUILD_PYBIND_CONTROL=OFF`.

**Run the lockstep test:**
```bash
source setup_env.sh && sf sil build
pytest simulator/tests/test_pid_lockstep.py -v
```
It drives `stampfly_control.PID` (the real firmware) and `rate_sysid.replay_pid()` (the hand port) step-by-step on identical inputs and compares the outputs numerically. See the comments in that test file for details.
