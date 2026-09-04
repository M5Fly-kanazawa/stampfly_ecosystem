# Lesson 7: システム同定 / System Identification

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このレッスンについて

フライトデータからプラントモデル $G_p(s) = K / (s(\tau_m s + 1))$ のパラメータ $K$, $\tau_m$ を同定する。
L5 の P 制御で飛行し、WiFi テレメトリでデータを取得した後、`sf sysid fit` でモデルフィッティングを行う。

### 前提知識

- L05: レート P 制御と初フライト（Kp, rate_max の値を使う）
- L06: システムモデリング（伝達関数、プラントモデル）

## 2. システム同定の仕組み

### アルゴリズム概要

$K_p$ が既知なので、テレメトリデータからプラントの入出力を復元できる。
`ws::set_rate_target(roll, pitch, yaw)` を呼ぶと、その角速度目標が 400Hz
Data Stream の `rate_ref_roll/pitch/yaw` 列に記録される（Body FRD, [rad/s]）:

```
Data Stream に記録されるデータ（sf log wifi -o *.csv）:
  rate_ref_roll : ロール角速度目標 [rad/s]（ws::set_rate_target の記録値）
  gyro_x        : ロール角速度実測 [rad/s]

プラント入出力の復元（rate_ref は既に絶対値なので rate_max 換算は不要）:
  u_plant  = Kp × (rate_ref_roll − gyro_x)     ← プラントへの入力
  y_plant  = gyro_x                             ← プラントの出力

開ループモデルをフィッティング:
  G_p(s) = K / (s·(τm·s + 1))
  minimize |y_simulated − y_plant|²   → K, τm を同定
```

`sf sysid fit` は CSV のヘッダ列から自動的にこの形式（"stream"）を判別する。
`rate_ref_*` 列が無い旧形式 CSV（`ctrl_roll` + `gyro_corrected_x` など、
`target = ctrl_roll × rate_max`）にも後方互換で対応する。

### なぜ開ループ同定が可能か

閉ループデータでも $K_p$ が既知なら、プラントへの入力 $u(t)$ を計算できる。
そのため、閉ループモデルを経由せずに開ループモデルを直接同定できる。

## 3. 手順

### ステップ 1: ファームウェア準備

1. `sf lesson switch 7` でテンプレートを `user_code.cpp` にコピー
2. `user_code.cpp` を開き、$K_p$ を設定（例: 0.5、L5 で使った値）
3. 角速度目標を計算した直後に `ws::set_rate_target(roll_target, pitch_target, yaw_target)` を呼ぶ（テンプレートに TODO ヒントあり）
4. WiFi チャンネルを設定
5. ビルド & 書き込み: `sf lesson build` → `sf lesson flash`

### ステップ 2: フライト & データ取得

1. PC でテレメトリ受信を開始（`.csv` を指定するとマージ済み Data Stream CSV を直接保存）: `sf log wifi -o flight.csv`
2. ARM → ホバリング → スティック操作でロール・ピッチ入力
3. 2〜3回のスティック操作で十分
4. 着陸 → DISARM

`flight.csv` には `timestamp_us, gyro_x/y/z, rate_ref_roll/pitch/yaw, total_thrust` 等が1周期1行で入る（拡張子を `.jsonl` にすると従来通りセンサ種別ごとの JSON Lines で保存され、`sf sysid fit` の入力には使えない）。

### ステップ 3: 同定

```bash
# 全軸を同定（--rate-max は stream 形式では無視される。rate_ref が既に絶対値のため）
sf sysid fit flight.csv --kp 0.5 --plot

# 特定軸のみ
sf sysid fit flight.csv --kp 0.5 --axis roll --plot

# 結果を YAML に保存
sf sysid fit flight.csv --kp 0.5 -o my_plant.yaml
```

### ステップ 4: L6 理論値と比較

| 軸 | K (同定) | K (L6理論) | τm (同定) | τm (理論) |
|-----|---------|-----------|----------|----------|
| Roll | ? | 102.0 | ? | 0.020 |
| Pitch | ? | 70.0 | ? | 0.020 |
| Yaw | ? | 19.0 | ? | 0.020 |

同定した K, τm から設計 Kp を計算: $K_p = 1/(4\zeta^2 K \tau_m)$

## 4. API

| 関数 | 説明 | 値域 |
|------|------|------|
| `ws::gyro_x/y/z()` | 角速度 | rad/s |
| `ws::rc_roll/pitch/yaw()` | スティック入力 | -1.0 〜 +1.0 |
| `ws::rc_throttle()` | スロットル | 0.0 〜 1.0 |
| `ws::set_rate_target(r,p,y)` | 角速度目標を Data Stream に記録（ロギング専用、制御には無関係） | rad/s |
| `ws::motor_mixer(T,R,P,Y)` | モーターミキサー | --- |
| `ws::led_color(r,g,b)` | LED 色設定 | 0〜255 |
| `ws::set_channel(ch)` | WiFi チャンネル | 1, 6, 11 |

## 5. チャレンジ

- 異なる Kp（例: 0.3, 0.7）で飛行し、同定結果がどう変わるか比較する
- `--time-range` オプションで特定区間のみ分析する
- 同定した K, τm でシミュレーション応答と実測を重ねてプロットする

---

<a id="english"></a>

## 1. Overview

### About This Lesson

Identify plant model parameters $K$ and $\tau_m$ from flight data where
$G_p(s) = K / (s(\tau_m s + 1))$.
Fly with L5's P controller, capture WiFi telemetry, then run `sf sysid fit` for model fitting.

### Prerequisites

- L05: Rate P control and first flight (need Kp, rate_max values)
- L06: System Modeling (transfer function, plant model)

## 2. How System Identification Works

### Algorithm Overview

Since $K_p$ is known, plant I/O can be reconstructed from telemetry. Calling
`ws::set_rate_target(roll, pitch, yaw)` records that rate target into the
400Hz Data Stream's `rate_ref_roll/pitch/yaw` columns (Body FRD, [rad/s]):

```
Data Stream columns (sf log wifi -o *.csv):
  rate_ref_roll : roll rate target [rad/s] (recorded by ws::set_rate_target)
  gyro_x        : measured roll rate [rad/s]

Plant I/O reconstruction (rate_ref is already absolute -- no rate_max scaling):
  u_plant  = Kp × (rate_ref_roll − gyro_x)     <- plant input
  y_plant  = gyro_x                             <- plant output

Open-loop model fitting:
  G_p(s) = K / (s·(τm·s + 1))
  minimize |y_simulated − y_plant|²   → identify K, τm
```

`sf sysid fit` auto-detects this ("stream") format from the CSV header, and
stays backward-compatible with the legacy CSV schema (`ctrl_roll` +
`gyro_corrected_x`, `target = ctrl_roll × rate_max`) when `rate_ref_*`
columns are absent.

### Why Open-Loop Identification Works

Even with closed-loop data, if $K_p$ is known, the plant input $u(t)$
can be computed directly. This allows open-loop model identification
without going through the closed-loop model.

## 3. Procedure

### Step 1: Firmware Setup

1. Run `sf lesson switch 7` to copy the template to `user_code.cpp`
2. Open `user_code.cpp` and set $K_p$ (e.g., 0.5, same as L5)
3. Right after computing the rate targets, call `ws::set_rate_target(roll_target, pitch_target, yaw_target)` (a TODO hint is in the template)
4. Set WiFi channel
5. Build & flash: `sf lesson build` → `sf lesson flash`

### Step 2: Flight & Data Capture

1. Start telemetry on PC (a `.csv` extension saves the merged Data Stream CSV directly): `sf log wifi -o flight.csv`
2. ARM → hover → apply roll/pitch stick inputs
3. 2-3 stick inputs are sufficient
4. Land → DISARM

`flight.csv` has one row per control cycle with `timestamp_us, gyro_x/y/z, rate_ref_roll/pitch/yaw, total_thrust`, etc. (a `.jsonl` extension instead saves the legacy per-sample JSON Lines format, which `sf sysid fit` cannot read).

### Step 3: Identification

```bash
# Identify all axes (--rate-max is ignored for stream-format CSVs -- rate_ref is already absolute)
sf sysid fit flight.csv --kp 0.5 --plot

# Single axis only
sf sysid fit flight.csv --kp 0.5 --axis roll --plot

# Save results to YAML
sf sysid fit flight.csv --kp 0.5 -o my_plant.yaml
```

### Step 4: Compare with L6 Theory

| Axis | K (identified) | K (L6 theory) | τm (identified) | τm (theory) |
|------|---------------|---------------|-----------------|-------------|
| Roll | ? | 102.0 | ? | 0.020 |
| Pitch | ? | 70.0 | ? | 0.020 |
| Yaw | ? | 19.0 | ? | 0.020 |

Compute design Kp from identified parameters: $K_p = 1/(4\zeta^2 K \tau_m)$

## 4. API

| Function | Description | Range |
|----------|-------------|-------|
| `ws::gyro_x/y/z()` | Angular rate | rad/s |
| `ws::rc_roll/pitch/yaw()` | Stick input | -1.0 to +1.0 |
| `ws::rc_throttle()` | Throttle | 0.0 to 1.0 |
| `ws::set_rate_target(r,p,y)` | Record the rate target into the Data Stream (logging only, no effect on control) | rad/s |
| `ws::motor_mixer(T,R,P,Y)` | Motor mixer | --- |
| `ws::led_color(r,g,b)` | LED color | 0-255 |
| `ws::set_channel(ch)` | WiFi channel | 1, 6, 11 |

## 5. Challenge

- Fly with different Kp values (e.g., 0.3, 0.7) and compare identification results
- Use `--time-range` option to analyze specific segments
- Overlay simulation response using identified K, τm with measured data
