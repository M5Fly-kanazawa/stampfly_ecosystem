# コマンド・API 早見表

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

`sf` CLI コマンドと `ws::` Workshop API（学習者コード用）の早見表。スライド付録のチートシートと同内容を、印刷・検索しやすい表形式でまとめたもの。

## 2. sf CLI コマンド

`sf --help` の表示順。

| コマンド | 機能 |
|---------|------|
| `sf version` | バージョン情報表示 |
| `sf doctor` | 環境診断（問題があればまず実行） |
| `sf setup` | 追加依存関係のインストール |
| `sf build [vehicle\|controller]` | ファームウェアビルド |
| `sf flash [vehicle\|controller] [-m]` | 書き込み（`-m` でモニタ付き） |
| `sf flasher` | ネイティブ書き込みGUIアプリの管理 |
| `sf monitor` | シリアルモニタを開く |
| `sf telemetry [--web]` | 50Hz テレメトリのライブ表示 |
| `sf blocks [--demo]` | Blockly ブロックプログラミング（`--demo` で機体なしデモ） |
| `sf log wifi/capture/list/info/convert/analyze/viz` | ログ取得・変換・解析・可視化 |
| `sf sim run vpython\|genesis` | シミュレータ起動 |
| `sf sils build/run/scenario/regression/gate/sysid-gate/gui` | SILS ベンチ操作 |
| `sf cal gyro/accel/mag` | センサキャリブレーション |
| `sf sysid fit` | 閉ループ P 制御ログからプラント同定（実習 7） |
| `sf sysid rate-fit/rate-tune` | 閉ループ同定（ETFE + フィット）と仕様ベース PID 設計 |
| `sf sysid noise` | 静止センサログから Allan 分散でノイズ特性を推定（`--sensor gyro/accel/baro/tof/all`） |
| `sf params check` | 物理パラメータ整合検査 |
| `sf trim analyze` | ホバーログから姿勢トリムを算出 |
| `sf takeoff/land/hover/jump/up/down/cw/ccw/forward/back/left/right/stop/emergency` | Tello 風ミニフライトコマンド |
| `sf motor` | ベンチモータテスト（disarm 時のみ） |
| `sf battery/height/tof/baro/attitude/acceleration/speed` | 機体状態の問い合わせ |
| `sf rc` | RC 値の一時送信 |
| `sf lesson list/switch/solution/info/edit/build/flash` | Workshop レッスン管理。本チュートリアルの実習番号は `sf lesson switch sci2026:N`、一覧は `sf lesson list --course sci2026` |
| `sf competition hover-time/score` | ホバー耐久・スコア記録 |
| `sf app` | カスタムファームアプリの管理 |
| `sf docs` | ドキュメントサイトのビルド・配信 |
| `sf upgrade` | 最新版を pull し環境を再同期 |

**`sf sysid noise` の入力 CSV について:** `sf log wifi -o file.csv` が出す CSV は gyro/accel の列しかなく、baro/tof は含まれない。baro/tof のノイズ評価には `sf log capture`（USB経由）→ `sf log convert` で得た CSV を使うこと。また、静止区間だけを解析するために `--static-only` を付けることを推奨する。

全コマンドは `lib/sfcli/commands/` に実装がある。

## 3. ws:: Workshop API（学習者コード用、`#include "workshop_api.hpp"`、全関数は `ws::` 名前空間）

### モータ制御

| 関数 | 引数 | 説明 |
|------|------|------|
| `motor_set_duty(id, duty)` | id: 1-4, duty: 0-1 | 個別モータ duty 設定 |
| `motor_set_all(duty)` | duty: 0-1 | 全モータ同一 duty |
| `motor_stop_all()` | --- | 全モータ即時停止 |
| `motor_mixer(t, r, p, y)` | 各 float | スラスト+姿勢ミキシング |
| `arm()` / `disarm()` | --- | モータ出力の有効化／無効化 |
| `is_armed()` | --- | Arm 状態（bool） |

### コントローラ入力・モード

| 関数 | 範囲 | 説明 |
|------|------|------|
| `rc_throttle()` | 0.0〜1.0 | スロットル |
| `rc_roll()` / `rc_pitch()` / `rc_yaw()` | -1.0〜1.0 | 姿勢入力 |
| `rc_throttle_yaw_button()` / `rc_roll_pitch_button()` | bool | スティック押し込みボタン |
| `rc_stabilize_acro_mode()` / `rc_alt_mode()` / `rc_pos_mode()` | bool | 飛行モード判定 |

### センサ

| 関数 | 単位 | 説明 |
|------|------|------|
| `gyro_x/y/z()` | rad/s | 角速度（BMI270） |
| `accel_x/y/z()` | m/s² | 加速度（BMI270） |
| `baro_altitude()` / `baro_pressure()` | m / Pa | 気圧高度・気圧値（BMP280） |
| `mag_x/y/z()` | µT | 磁気（BMM150） |
| `tof_bottom()` / `tof_front()` | m | ToF 距離（front は未接続で -1） |
| `flow_vx/vy()` / `flow_quality()` | m/s / 0-255 | 光学フロー速度・品質（PMW3901） |

### 推定・制御目標のロギング

| 関数 | 単位 | 説明 |
|------|------|------|
| `estimated_roll/pitch/yaw()` | rad | ESKF 推定姿勢角 |
| `estimated_altitude()` | m | ESKF 推定高度（正=上） |
| `set_rate_target(roll, pitch, yaw)` | rad/s | 角速度目標を Data Stream の `rate_ref_*` に記録（実習 7、ロギング専用） |
| `set_angle_target(roll, pitch)` | rad | 傾き角目標を `angle_ref_*` に記録（ロギング専用） |

### LED・ユーティリティ

| 関数 | 説明 |
|------|------|
| `led_color(r, g, b)` | LED 色設定（0-255） |
| `disable_led_task()` / `enable_led_task()` | システム LED タスクの停止／再開 |
| `is_led_task_disabled()` | LED タスク停止中か |
| `millis()` | 起動からの経過時間 [ms] |
| `battery_voltage()` | バッテリー電圧 [V] |
| `print(fmt, ...)` | printf 形式のデバッグ出力（Teleplot は `>name:value` 形式） |
| `set_channel(ch)` | WiFi チャネル設定（1, 6, 11） |

---

<a id="english"></a>

## 1. Overview

### About This Document

A cheat sheet for the `sf` CLI and the `ws::` Workshop API (for learner code). Same content as the slide-appendix cheat sheets, in a print/search-friendly table form.

## 2. sf CLI Commands

In `sf --help` display order.

| Command | Function |
|---------|----------|
| `sf version` | Show version info |
| `sf doctor` | Diagnose the environment (run this first if something's wrong) |
| `sf setup` | Install optional dependencies |
| `sf build [vehicle\|controller]` | Build firmware |
| `sf flash [vehicle\|controller] [-m]` | Flash (`-m` opens the monitor afterward) |
| `sf flasher` | Manage the native flasher GUI app |
| `sf monitor` | Open the serial monitor |
| `sf telemetry [--web]` | Live 50Hz telemetry |
| `sf blocks [--demo]` | Blockly block programming (`--demo` = no hardware needed) |
| `sf log wifi/capture/list/info/convert/analyze/viz` | Log capture, conversion, analysis, visualization |
| `sf sim run vpython\|genesis` | Launch a simulator |
| `sf sils build/run/scenario/regression/gate/sysid-gate/gui` | SILS bench operations |
| `sf cal gyro/accel/mag` | Sensor calibration |
| `sf sysid fit` | Identify the plant from a closed-loop P-control log (Exercise 7) |
| `sf sysid rate-fit/rate-tune` | Closed-loop identification (ETFE + fit) and spec-based PID design |
| `sf sysid noise` | Estimate sensor noise from a static log via Allan variance (`--sensor gyro/accel/baro/tof/all`) |
| `sf params check` | Physical-parameter consistency audit |
| `sf trim analyze` | Compute attitude trim from a hover log |
| `sf takeoff/land/hover/jump/up/down/cw/ccw/forward/back/left/right/stop/emergency` | Tello-style mini flight commands |
| `sf motor` | Bench motor test (disarmed only) |
| `sf battery/height/tof/baro/attitude/acceleration/speed` | Query vehicle state |
| `sf rc` | Send RC values one-shot |
| `sf lesson list/switch/solution/info/edit/build/flash` | Workshop lesson management. This tutorial's exercise numbers: `sf lesson switch sci2026:N`, listed with `sf lesson list --course sci2026` |
| `sf competition hover-time/score` | Hover-endurance and score recording |
| `sf app` | Manage custom firmware apps |
| `sf docs` | Build/serve the documentation site |
| `sf upgrade` | Pull the latest changes and resync the environment |

**About `sf sysid noise`'s input CSV:** the CSV from `sf log wifi -o file.csv` only has gyro/accel columns -- no baro/tof. For baro/tof noise characterization, capture with `sf log capture` (USB) and convert with `sf log convert` instead. Also add `--static-only` so the analysis only uses stationary segments.

All commands are implemented under `lib/sfcli/commands/`.

## 3. ws:: Workshop API (for learner code, `#include "workshop_api.hpp"`, all functions in the `ws::` namespace)

### Motor control

| Function | Args | Description |
|----------|------|--------------|
| `motor_set_duty(id, duty)` | id: 1-4, duty: 0-1 | Set one motor's duty |
| `motor_set_all(duty)` | duty: 0-1 | Same duty on all motors |
| `motor_stop_all()` | --- | Stop all motors immediately |
| `motor_mixer(t, r, p, y)` | each float | Thrust + attitude mixing |
| `arm()` / `disarm()` | --- | Enable/disable motor output |
| `is_armed()` | --- | Arm state (bool) |

### Controller input and mode

| Function | Range | Description |
|----------|-------|--------------|
| `rc_throttle()` | 0.0-1.0 | Throttle |
| `rc_roll()` / `rc_pitch()` / `rc_yaw()` | -1.0-1.0 | Attitude input |
| `rc_throttle_yaw_button()` / `rc_roll_pitch_button()` | bool | Stick-press buttons |
| `rc_stabilize_acro_mode()` / `rc_alt_mode()` / `rc_pos_mode()` | bool | Flight-mode checks |

### Sensors

| Function | Unit | Description |
|----------|------|--------------|
| `gyro_x/y/z()` | rad/s | Angular rate (BMI270) |
| `accel_x/y/z()` | m/s² | Acceleration (BMI270) |
| `baro_altitude()` / `baro_pressure()` | m / Pa | Barometric altitude / pressure (BMP280) |
| `mag_x/y/z()` | µT | Magnetometer (BMM150) |
| `tof_bottom()` / `tof_front()` | m | ToF distance (front reads -1 if unconnected) |
| `flow_vx/vy()` / `flow_quality()` | m/s / 0-255 | Optical-flow velocity/quality (PMW3901) |

### Estimation and control-target logging

| Function | Unit | Description |
|----------|------|--------------|
| `estimated_roll/pitch/yaw()` | rad | ESKF-estimated attitude |
| `estimated_altitude()` | m | ESKF-estimated altitude (positive = up) |
| `set_rate_target(roll, pitch, yaw)` | rad/s | Record the rate target into `rate_ref_*` (Exercise 7, logging only) |
| `set_angle_target(roll, pitch)` | rad | Record the tilt target into `angle_ref_*` (logging only) |

### LED and utility

| Function | Description |
|----------|--------------|
| `led_color(r, g, b)` | Set LED color (0-255) |
| `disable_led_task()` / `enable_led_task()` | Stop/resume the system LED task |
| `is_led_task_disabled()` | Whether the LED task is stopped |
| `millis()` | Elapsed time since boot [ms] |
| `battery_voltage()` | Battery voltage [V] |
| `print(fmt, ...)` | printf-style debug output (Teleplot format: `>name:value`) |
| `set_channel(ch)` | Set the WiFi channel (1, 6, 11) |
