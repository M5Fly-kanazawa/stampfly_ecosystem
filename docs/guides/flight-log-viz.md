# フライトログの取得と可視化チュートリアル

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このチュートリアルについて

StampFly のフライトログを取得し、`sf log viz` で可視化する手順を説明します。ログの一次記録
（測定値をそのまま書いた、最初の記録）は**StampFly フライトログ v1 一式**（`.sflog.zip`。
信号ごとの CSV と `meta.json`／`schema.json` を zip にまとめたもの。以下「一式」）です。サブ
コマンドの一覧・詳細は `docs/commands/sf-log.md` を参照してください。

### 前提条件

- 開発環境がセットアップ済み（`source setup_env.sh`）
- StampFly が WiFi モードで起動している
- Python 3.x と必要なライブラリ（numpy, pandas, matplotlib）がインストール済み

## 2. ログ取得

StampFly の WiFi AP に接続した状態で取得します。

```bash
sf log wifi -d 30
```

既定では `logs/flight_<YYYYMMDD>T<HHMMSS>.sflog.zip` に保存されます。保存直後に `sf log check`
が自動実行され、電文の解析エラーや構造の異常があればその場で報告されます。

```bash
sf log list
```

```bash
sf log info
```

出力例：
```
Bundle: flight_20260911T121243.sflog.zip
  Size:    842.3 KB
  Source:  vehicle
  Created: 2026-09-11T12:12:43
  Tool:    sf log wifi 2026.09
  Capture: 192.168.10.1:8890 (requested 30s, actual 30.0s, 0 packets lost)

  Stream         Rows   NomHz  MeasHz      First(us)       Last(us)  Dur(s)
  attitude      11736   400.0   391.4              0       29996250    30.0
  imu           11736   400.0   391.4              0       29996250    30.0
  motor         11736   400.0   391.4              0       29996250    30.0
  ...
```

## 3. sf log viz による可視化

`sf log viz` は一式を読み込み、全ての信号をそれぞれの原レート（送信元パケットが本来持っていた
サンプリング周期。400 Hz の姿勢・IMU・モータ・50 Hz の操縦入力・気圧・1 Hz のバッテリ等が
混在する）のまま、1 つの描画処理で並べて表示します。

```bash
sf log viz
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --save overview.png
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --time-range 5 15
```

### パネルの一覧（`--mode all`、既定）

各パネルは、必要なストリーム（CSV）が一式に無い場合は自動的に省かれます（例: `ctrl_output.csv`
を送らないファーム版のログでは「Control Output」パネルが出ない）。

| パネル | 内容 | 単位 | 由来ストリーム |
|-------|------|------|---------------|
| Roll Rate | ロールレート実測（IMU）vs レート指令 | deg/s | `imu.csv`（`gyro_x`）+ `rate_ref.csv` |
| Pitch Rate | ピッチレート実測 vs レート指令 | deg/s | 同上（`gyro_y`） |
| Yaw Rate | ヨーレート実測 vs レート指令 | deg/s | 同上（`gyro_z`） |
| Attitude | クォータニオンから求めた roll/pitch/yaw と角度指令 | deg | `attitude.csv`（`quat_w/x/y/z`）+ `ctrl_ref.csv`（`angle_ref_roll/pitch`） |
| Acceleration | 機体座標系の加速度 x/y/z | m/s² | `imu.csv`（`accel_x/y/z`） |
| Position | NED（北・東・下）位置 x/y/z | m | `posvel.csv`（`pos_x/y/z`） |
| Velocity | NED 速度 x/y/z | m/s | `posvel.csv`（`vel_x/y/z`） |
| Motor duty | 4 モータの duty（駆動デューティ比）。`motor.csv`（400 Hz 実測）があればそれを、無ければ `ctrl_ref.csv` の 50 Hz 指令値をステップ状に表示し、パネル見出しでどちらかを明示。`total_thrust` があれば重ねて表示 | duty [0-1] / 推力 [N] | `motor.csv` または `ctrl_ref.csv` |
| Control Output | ミキサー手前の指令推力・トルク | 推力 [N] / トルク [mN·m] | `ctrl_output.csv` |
| Pilot Input | スティック入力（スロットル・ロール・ピッチ・ヨー） | -1〜1 | `pilot.csv` |
| Height / Distance | 気圧高度、下向き／前向き ToF（Time of Flight：赤外線の往復時間で距離を測るセンサ）距離 | m | `baro.csv`、`tof_bottom.csv`／`tof_front.csv` |
| Optical Flow | 光学フローの dx/dy と品質 | counts / quality | `flow.csv` |
| Magnetometer | 地磁気 x/y/z | µT | `mag.csv` |
| Gyro Bias / Flight Mode | ESKF（拡張カルマンフィルタ）のジャイロバイアス推定値と `flight_mode` | deg/s | `attitude.csv`（`gyro_bias_x/y/z`）+ `ctrl_ref.csv`（`flight_mode`） |
| Battery Status | バッテリ電圧・電流 | V / mA | `status.csv` |

### モード（`--mode`）

| モード | 含まれるパネル |
|-------|---------------|
| `all`（既定） | 上表の全パネル |
| `attitude` | Roll/Pitch/Yaw Rate、Attitude |
| `sensors` | Acceleration、Raw Gyro（`imu.csv` の `gyro_raw_x/y/z`）、Height/Distance、Optical Flow、Magnetometer |
| `position` | Position、Velocity、Height/Distance、Pilot Input |
| `eskf` | Attitude、Position、Velocity、Gyro Bias/Flight Mode、Accelerometer Bias（`attitude.csv` の `accel_bias_x/y/z`） |

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode attitude
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode sensors
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode position
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode eskf
```

### インタラクティブ表示

```bash
sf log viz logs/flight_20260911T121243.sflog.zip -i
```

`-i` は Plotly（ブラウザで動くグラフ描画ライブラリ）のダッシュボードをブラウザで開きます。
`--save FILE` と組み合わせると、ダッシュボードを PNG ではなく HTML として保存します。

## 4. 典型的なワークフロー

```bash
sf log wifi -d 60
```

```bash
sf log viz
```

```bash
sf log analyze
```

## 5. 表示ウィンドウが使えない環境での動作

matplotlib の GUI バックエンド（Tk/Qt 等の画面描画の仕組み）が使えない環境（例: GUI 無しの
仮想環境）では、ウィンドウ表示の代わりに一式の隣へ `<一式の名前>.png` を自動保存し、OS 標準の
画像ビューアで開きます。ウィンドウを開けた場合は使用したバックエンド名（`macosx`/`tkagg`/
`qtagg` 等）を 1 行表示します。GUI バックエンドが起動時の確認は通過したのに実際の描画で失敗
した場合は、同じ描画を一度だけヘッドレス（画面を使わない）で再試行してから PNG 保存に切り替え
ます。インタラクティブモード（`-i`、Plotly）はブラウザで開くためこのフォールバック（代替動作）
の対象外です。

## 6. トラブルシューティング

### WiFi 接続できない

```bash
sf log wifi -i 192.168.10.1
```

StampFly の AP（SSID: `StampFly_XXXX`）に接続されているか確認し、IP アドレスを明示的に指定
します。

### 一式が壊れている／空

```bash
sf log check logs/flight_20260911T121243.sflog.zip
```

```bash
sf log info logs/flight_20260911T121243.sflog.zip
```

### matplotlib エラー

```bash
pip install matplotlib numpy pandas scipy pyyaml
```

### プロットウィンドウが開かない／"non-interactive" 警告が出る

sf が自動で一式の隣へ PNG を保存し、既定の画像ビューアで開きます。詳しい原因と恒久的な直し方は
`docs/guides/troubleshooting.md` 第 6 章「グラフ表示（matplotlib）」を参照してください。

```bash
sf doctor
```

`sf doctor` の "Checking plot window support" で GUI バックエンドの状態を確認できます。

---

<a id="english"></a>

## 1. Overview

### About This Tutorial

This tutorial explains how to capture flight logs from StampFly and visualize them with
`sf log viz`. The primary record (the first record of raw measurements, with nothing filled in)
is the **StampFly flight-log v1 bundle** (`.sflog.zip` — a zip holding a CSV per signal plus
`meta.json`/`schema.json`; called "the bundle" below). See `docs/commands/sf-log.md` for the full
subcommand reference.

### Prerequisites

- Development environment set up (`source setup_env.sh`)
- StampFly running in WiFi mode
- Python 3.x with required libraries (numpy, pandas, matplotlib)

## 2. Log Capture

Connect to StampFly's WiFi AP and run:

```bash
sf log wifi -d 30
```

By default this saves to `logs/flight_<YYYYMMDD>T<HHMMSS>.sflog.zip`. Right after saving,
`sf log check` runs automatically and reports any wire-format parse error or structural problem.

```bash
sf log list
```

```bash
sf log info
```

## 3. Visualizing with sf log viz

`sf log viz` loads a bundle and plots every signal at its own native rate (the sampling period
the originating packet actually had — a mix of 400 Hz attitude/IMU/motor, 50 Hz pilot input/baro,
and 1 Hz battery, for example), through one renderer.

```bash
sf log viz
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --save overview.png
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --time-range 5 15
```

### Panels (`--mode all`, default)

Each panel is automatically skipped when its required stream (CSV) is absent from the bundle
(e.g. a firmware version that never sends `ctrl_output.csv` has no "Control Output" panel).

| Panel | Content | Unit | Source stream(s) |
|-------|---------|------|-------------------|
| Roll Rate | Measured roll rate (IMU) vs. rate reference | deg/s | `imu.csv` (`gyro_x`) + `rate_ref.csv` |
| Pitch Rate | Measured vs. commanded pitch rate | deg/s | same (`gyro_y`) |
| Yaw Rate | Measured vs. commanded yaw rate | deg/s | same (`gyro_z`) |
| Attitude | Roll/pitch/yaw from quaternion, plus angle reference | deg | `attitude.csv` (`quat_w/x/y/z`) + `ctrl_ref.csv` (`angle_ref_roll/pitch`) |
| Acceleration | Body-frame acceleration x/y/z | m/s^2 | `imu.csv` (`accel_x/y/z`) |
| Position | NED (North-East-Down) position x/y/z | m | `posvel.csv` (`pos_x/y/z`) |
| Velocity | NED velocity x/y/z | m/s | `posvel.csv` (`vel_x/y/z`) |
| Motor duty | The 4 motor duty cycles: `motor.csv` (native 400 Hz) if present, else the 50 Hz `ctrl_ref.csv` command drawn as a step (the panel title says which); `total_thrust` overlaid when present | duty [0-1] / thrust [N] | `motor.csv` or `ctrl_ref.csv` |
| Control Output | Pre-mixer commanded thrust and torque | thrust [N] / torque [mN*m] | `ctrl_output.csv` |
| Pilot Input | Stick input (throttle, roll, pitch, yaw) | -1 to 1 | `pilot.csv` |
| Height / Distance | Baro altitude, downward/forward ToF (Time of Flight — a sensor that measures distance from an infrared pulse's round-trip time) distance | m | `baro.csv`, `tof_bottom.csv`/`tof_front.csv` |
| Optical Flow | Optical-flow dx/dy and quality | counts / quality | `flow.csv` |
| Magnetometer | Magnetic field x/y/z | uT | `mag.csv` |
| Gyro Bias / Flight Mode | ESKF (Error-State Kalman Filter) gyro bias estimate and `flight_mode` | deg/s | `attitude.csv` (`gyro_bias_x/y/z`) + `ctrl_ref.csv` (`flight_mode`) |
| Battery Status | Battery voltage/current | V / mA | `status.csv` |

### Modes (`--mode`)

| Mode | Panels included |
|------|-------------------|
| `all` (default) | All panels above |
| `attitude` | Roll/Pitch/Yaw Rate, Attitude |
| `sensors` | Acceleration, Raw Gyro (`imu.csv`'s `gyro_raw_x/y/z`), Height/Distance, Optical Flow, Magnetometer |
| `position` | Position, Velocity, Height/Distance, Pilot Input |
| `eskf` | Attitude, Position, Velocity, Gyro Bias/Flight Mode, Accelerometer Bias (`attitude.csv`'s `accel_bias_x/y/z`) |

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode attitude
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode sensors
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode position
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode eskf
```

### Interactive display

```bash
sf log viz logs/flight_20260911T121243.sflog.zip -i
```

`-i` opens a Plotly (a browser-based plotting library) dashboard in the browser. Combined with
`--save FILE`, it saves the dashboard as HTML instead of a PNG.

## 4. Typical Workflow

```bash
sf log wifi -d 60
```

```bash
sf log viz
```

```bash
sf log analyze
```

## 5. Behavior Without a Display Window

When no matplotlib GUI backend (Tk/Qt, etc.) is usable (e.g. a headless environment), the plot is
saved as `<bundle name>.png` next to the bundle instead of opening a window, then opened with the
OS's default image viewer. When a window does open, the backend name used (`macosx`/`tkagg`/
`qtagg`, etc.) is printed on one line. If a GUI backend passes its startup probe but still fails
while actually drawing, the same plot is retried headlessly once before falling back to the PNG.
Interactive mode (`-i`, Plotly) opens in a browser and is unaffected by this fallback.

## 6. Troubleshooting

### Cannot connect over WiFi

```bash
sf log wifi -i 192.168.10.1
```

Check that you are connected to StampFly's AP (SSID: `StampFly_XXXX`), and specify the IP address
explicitly.

### Bundle is broken or empty

```bash
sf log check logs/flight_20260911T121243.sflog.zip
```

```bash
sf log info logs/flight_20260911T121243.sflog.zip
```

### matplotlib error

```bash
pip install matplotlib numpy pandas scipy pyyaml
```

### Plot window does not open / "non-interactive" warning

sf now saves a PNG next to the bundle and opens it with the default image viewer automatically.
For the cause and a permanent fix, see `docs/guides/troubleshooting.md` section 6 "Plot Window
(matplotlib)".

```bash
sf doctor
```

`sf doctor`'s "Checking plot window support" shows the GUI backend status.
