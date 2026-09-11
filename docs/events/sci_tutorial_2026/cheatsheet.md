# コマンド・API 早見表

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

`sf` CLI コマンドと `ws::` 実習用 API（実習コード用）の早見表。スライド付録のチートシートと同内容を、印刷・検索しやすい表形式でまとめたもの。

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
| `sf log wifi/list/info/analyze/viz` | ログ取得・解析・可視化（`capture`/`convert` は旧ファーム専用） |
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
| `sf lesson list/switch/solution/info/edit/build/flash/monitor/sils` | 実習コードの管理。本チュートリアルの実習番号は `sf lesson switch sci2026:N`、一覧は `sf lesson list --course sci2026` |
| `sf app` | カスタムファームアプリの管理 |
| `sf docs` | ドキュメントサイトのビルド・配信 |
| `sf upgrade` | 最新版を pull し環境を再同期 |

**`sf sysid noise` の入力について:** `sf log wifi` が保存するフライトログ一式（`.sflog.zip`：信号ごとの CSV をまとめた zip 1個、埋め値なし）には gyro/accel の CSV しかなく、baro/tof は含まれない。baro/tof のノイズ評価は現行ファームでは対象外（`sf log capture` → `sf log convert` は旧ファーム vehicle_old の USB バイナリログ専用で、現行ファームでは動かない）。また、静止区間だけを解析するために `--static-only` を付けることを推奨する。

全コマンドは `lib/sfcli/commands/` に実装がある。

### WiFi 接続（設定・接続の確認）

`sf telemetry`／`sf log wifi`／`param set`（飛行中のライブなゲイン変更）／`sf sysid rate-fit`・`rate-tune` の入力ログなどは、いずれも WiFi 経由でしか取得・操作できない（飛行中は USB を挿せないため）。出荷時既定は `wifi.mode`=0（STA・資格情報未設定）でテレメトリが無効なので、最初に SoftAP（機体自身が出す WiFi）へ切り替える。

| 項目 | 既定値 |
|------|--------|
| `wifi.mode` | `0`=STA（既定）／`1`=SoftAP |
| SoftAP SSID | `StampFly-XXYY`（MACアドレス末尾から自動生成） |
| SoftAP パスワード | `stampfly`（`wifi pass <secret>` で変更可） |
| 機体 IP | `192.168.10.1`（DJI Tello 互換サブネット。`sf` の各種 `--ip` の既定値でもある） |
| WiFi/ESP-NOW チャンネル | `wifi.channel`（既定 `1`、1〜13） |

機体側の初期設定（実習 1 (2/2)、機体ごとに 1 回）: `sf flash vehicle -m` の直後、USB 接続のままモニタの CLI で次を順に打つ。`N` は講師が指定するチャンネル（1 / 6 / 11 のどれか）。

```
param reset
param set wifi.mode 1
param set wifi.channel N
param save
reboot
```

続けてコントローラとペアリングする（コントローラの LCD パネルボタンを押しながら電源投入 → 機体のボタンを 3 秒以上押し続け、双方のビープで離す。5 秒以上押し続けるとシステムリセット）。PC側: WiFi設定でSSID `StampFly-XXYY` に接続（パスワードは既定 `stampfly`）。接続の確認は `sf telemetry`（IP指定不要、既定192.168.10.1で待ち受け）。

```
sf telemetry
```

会場の WiFi は使わない（不通の前提で運用する）。各自の StampFly ごとに SSID が異なる点に注意。

### sf lesson と通常コマンドの対応

`sf lesson` は実習を滞りなく進めるための近道で、正は通常コマンド。各コマンドは次を呼んでいるだけ。

| `sf lesson` コマンド | 実体 |
|----------------------|------|
| `sf lesson switch sci2026:N [--solution]` | 課題 N の `student.cpp`（`solution.cpp`）を実習ファームウェアの `user_code.cpp` にコピー |
| `sf lesson edit` | その `user_code.cpp` を開く |
| `sf lesson build` | `sf build workshop` |
| `sf lesson flash` | `sf flash workshop -m`（`--no-monitor` で `-m` 無し） |
| `sf lesson monitor` | `sf monitor workshop` |
| `sf lesson sils` | `sf sils build --target workshop` → `sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop` |
| `sf lesson sils --scenario step` | 同上でシナリオは `workshop_acro_step.scn` |

## 3. ws:: 実習用 API（実習コード用、`#include "workshop_api.hpp"`、全関数は `ws::` 名前空間）

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
| `set_channel(ch)` | WiFi チャンネル設定（1, 6, 11）。通常は使わない。チャンネルは初期設定の `param set wifi.channel` で保存済みで、呼ぶと上書きして再起動する |

---

<a id="english"></a>

## 1. Overview

### About This Document

A cheat sheet for the `sf` CLI and the `ws::` lesson API (for exercise code). Same content as the slide-appendix cheat sheets, in a print/search-friendly table form.

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
| `sf log wifi/list/info/analyze/viz` | Log capture, analysis, visualization (`capture`/`convert` are legacy-firmware only) |
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
| `sf lesson list/switch/solution/info/edit/build/flash/monitor/sils` | Lesson-code management. This tutorial's exercise numbers: `sf lesson switch sci2026:N`, listed with `sf lesson list --course sci2026` |
| `sf app` | Manage custom firmware apps |
| `sf docs` | Build/serve the documentation site |
| `sf upgrade` | Pull the latest changes and resync the environment |

**About `sf sysid noise`'s input:** the flight-log bundle (`.sflog.zip` -- a zip holding one CSV per signal at its native rate, no filled-in values) that `sf log wifi` saves only has gyro/accel CSVs -- no baro/tof. Baro/tof noise characterization is not available with the current firmware (`sf log capture` -> `sf log convert` reads the legacy vehicle_old USB binary log only and does not work with the current firmware). Also add `--static-only` so the analysis only uses stationary segments.

All commands are implemented under `lib/sfcli/commands/`.

### Connecting Over WiFi (Setup and Connectivity Check)

`sf telemetry`, `sf log wifi`, `param set` (live gain changes in flight), and the logs that feed `sf sysid rate-fit`/`rate-tune` are all only reachable over WiFi (you cannot plug in USB while flying). Out of the box, `wifi.mode` defaults to `0` (STA, no credentials configured), which leaves telemetry inert -- switch to SoftAP (the vehicle's own WiFi) first.

| Item | Default |
|------|---------|
| `wifi.mode` | `0` = STA (default) / `1` = SoftAP |
| SoftAP SSID | `StampFly-XXYY` (auto-generated from the MAC tail) |
| SoftAP password | `stampfly` (change with `wifi pass <secret>`) |
| Vehicle IP | `192.168.10.1` (DJI Tello-compatible subnet; also the default for every `sf` `--ip` flag) |
| WiFi/ESP-NOW channel | `wifi.channel` (default `1`, 1-13) |

Vehicle-side one-time setup (Exercise 1 (2/2), once per vehicle): right after `sf flash vehicle -m`, with USB still connected, type the following in the monitor CLI. `N` is the channel the instructor assigns you (1, 6 or 11).

```
param reset
param set wifi.mode 1
param set wifi.channel N
param save
reboot
```

Then pair the controller (hold the controller's LCD panel button while powering on, then hold the vehicle button for 3 s or more and release at the double beep; holding 5 s or more triggers a system reset). PC side: join SSID `StampFly-XXYY` in WiFi settings (default password `stampfly`). Connectivity check: `sf telemetry` (no `--ip` needed; listens on the default 192.168.10.1).

```
sf telemetry
```

Do not rely on venue WiFi (assume it is down). Each attendee's StampFly has a different SSID.

### How sf lesson Maps to Plain Commands

`sf lesson` is a shortcut for moving through the exercises smoothly; the plain commands are the source of truth. Each `sf lesson` command just calls one of these.

| `sf lesson` command | What it actually runs |
|----------------------|------|
| `sf lesson switch sci2026:N [--solution]` | Copies exercise N's `student.cpp` (or `solution.cpp`) to the lesson firmware's `user_code.cpp` |
| `sf lesson edit` | Opens that `user_code.cpp` |
| `sf lesson build` | `sf build workshop` |
| `sf lesson flash` | `sf flash workshop -m` (pass `--no-monitor` to drop `-m`) |
| `sf lesson monitor` | `sf monitor workshop` |
| `sf lesson sils` | `sf sils build --target workshop` -> `sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop` |
| `sf lesson sils --scenario step` | Same, but with the `workshop_acro_step.scn` scenario |

## 3. ws:: Lesson API (for exercise code, `#include "workshop_api.hpp"`, all functions in the `ws::` namespace)

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
| `set_channel(ch)` | Set the WiFi channel (1, 6, 11). Normally unused: the channel is saved by `param set wifi.channel` in the one-time setup, and calling this overwrites it and reboots |
