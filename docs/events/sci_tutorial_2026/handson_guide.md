# 復習ハンズオンガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このガイドについて

チュートリアル当日に「見るだけ」で参加した内容を、後日自宅・研究室で実機またはシミュレータを使って再現するための手順書。各セッションのデモに対応するコマンド列と観察ポイントをまとめる。

### 使い方

各レッスンは共通の手順で進む。

| 手順 | コマンド | 内容 |
|------|---------|------|
| 1 | `sf lesson switch sci2026:N` | 実習 N の学習者テンプレートを `user_code.cpp` にコピー |
| 2 | `sf lesson build` | ビルド |
| 3 | `sf lesson flash` | 実機に書き込み |
| 4 | （実機がない場合）`sf lesson sils` | SILS で動作確認（VPython には実習コードは入らない） |

本チュートリアルの実習番号は `sci2026` という実習構成（コース）に対応付けられている。一覧は `sf lesson list --course sci2026` で確認できる。模範解答をそのまま試したい場合は `sf lesson switch sci2026:N --solution` を使う。実習コードと模範解答の差分だけを見たい場合は `sf lesson solution sci2026:N` を使う。

## 2. S2 の再現: 開発環境とセンサデータ

対応: 実習 2（IMU センサー）

```bash
sf doctor                    # 環境診断
sf lesson switch sci2026:2
sf lesson build
sf lesson flash
sf telemetry                 # 50Hz テレメトリのライブ表示
```

**観察ポイント:** 機体を手で傾けるとジャイロ・加速度の値が変化する。VSCode 拡張 Teleplot を使うとグラフでも確認できる。

**参照:** `firmware/vehicle/docs/architecture.md` §2（4階層アクセス）

## 3. S3 の再現: モータ制御とコントローラ入力

対応: 実習 3（モータ制御）、実習 4（コントローラ入力）

机上で行うこと。モータ回転中は手を近づけない。異常時は即 DISARM。

```bash
sf lesson switch sci2026:3
sf lesson build
sf lesson flash
```

**観察ポイント:** `motor_set_duty()` の duty を変えるとモータの回転数が変わる。

```bash
sf flash controller           # コントローラ側ファーム（初回のみ）
sf lesson switch sci2026:4
sf lesson build
sf lesson flash
```

**観察ポイント:** コントローラのスティックを倒すと `rc_roll()`/`rc_pitch()` 等の値が追従する。

**参照:** `firmware/vehicle/docs/hardware_init.md`、`protocol/spec/`（ControlPacket）

## 4. S4 の再現: フィードバック制御

対応: 実習 5〜9。**必ず机上で行い、モータ回転中は手を近づけず、低スロットルから試すこと。異常時は即 DISARM。**

### 実習 5: レート P 制御

```bash
sf lesson switch sci2026:5
sf lesson build && sf lesson flash
```

離陸してスティック操作への応答を確認する。`Kp` を変えて振動と鈍さの違いを体感する。

### 実習 6: システムモデリング（座学）

実機操作はない。スライド S4「実測パラメータ」フレーム（$K_{roll}=102$、$K_{pitch}=70$、$\tau_m\approx0.02$\,s）を確認し、$\zeta=0.7$ 設計の $K_p$ を計算しておく。

### 実習 7: システム同定

`sf lesson switch sci2026:7` の後、`user_code.cpp` に `Kp` をセットし `ws::set_rate_target()` で目標角速度を記録するコードを書いてからビルド・書き込みする。

```bash
sf lesson switch sci2026:7
sf lesson build && sf lesson flash
sf log wifi
sf sysid fit logs/flight_<timestamp>.sflog.zip --plot
```

`sf log wifi` は離陸してスティック操作しながら実行する（既定 30 秒）。取得したフライトログ一式（`.sflog.zip`：信号ごとの CSV をまとめた zip 1個、埋め値なし）は `logs/flight_<日時>.sflog.zip` に保存されるので、`<timestamp>` は実際のファイル名に置き換える。

**観察ポイント:** 同定した $K$, $\tau_m$ と実習 6 の理論値を比較する。

### 実習 8: PID 制御

```bash
sf lesson switch sci2026:8
sf lesson build && sf lesson flash
```

理想微分（振動する）→不完全微分（振動が減る）の順に試す。スライド S4「不完全微分と D-on-M」フレームを見ながらゲインを調整する。

### 実習 5/8 のコードを SILS で飛ばす

実機を飛ばす前に、書いたコードを SILS（`simulator/sils/`）で確かめられる。実機に書き込まれるのと同じソースがそのまま動く（Code Identity）ので、墜落のリスクなしに ARM・状態遷移・モータ応答の配線ミスに気付ける。`sf lesson sils` が内部で再ビルドとシナリオ実行までまとめて行うため、切替後にこの1行を打つだけでよい。

```bash
sf lesson switch sci2026:8 --solution        # または自分のコード
sf lesson sils
```

**観察ポイント:** 合格基準（`.expect`）は「離陸したか（真値高度が 0.1 m を超える）」「傾き 15° 未満か（転倒しない）」の2点。実習コードには高度ループがないため、着陸は DISARM による降下のみ。実習コードの SILS は常に `sf lesson sils` を使う（`sf sils gui` は vehicle 本体向け）。上記の手順どおりに実行すると `alt_max` ≈ 0.64 m、`tilt_max` = 0.0 で PASS になる。

### 実習 9: 姿勢推定

```bash
sf lesson switch sci2026:9
sf lesson build && sf lesson flash
```

Teleplot で `cf_roll`（自作の相補フィルタ）と `eskf_roll`（機体既定の ESKF）を重ねて表示し、一致することを確認する。

### 発展: 自動チューニング

```bash
sf sysid rate-fit logs/flight_<timestamp>.sflog.zip --axis roll -o fit.json
sf sysid rate-tune --fit fit.json --wc 25 --pm 60
```

**参照:** 本資料のスライド S4（`docs/events/sci_tutorial_2026/slides/chapters/sci_s4_pid.tex`）の「あとで読む」理論フレーム一式（フィードバック制御の基礎〜プラントモデリング〜システム同定〜PID理論〜相補フィルタ）

## 5. S5 の再現: シミュレータと解析ツール

実機がなくてもここは再現できる。

```bash
sf sim run vpython            # ブラウザ3D操縦
```

AtomS3 + Atom JoyStick を USB HID モードで持っている場合は接続して操縦できる。持っていない場合はキーボード操作にフォールバックする。

```bash
sf sils build                 # 初回のみ
sf sils gui                   # ブラウザで http://127.0.0.1:8765 が開く
```

**観察ポイント:** シナリオを1本実行し、3D 再生とグラフ、判定結果（PASS/FAIL）を確認する。「パラメータ」タブでゲインを変えて再実行し、挙動の変化を見る。

S4 で取得したフライトログ一式（`logs/` 内の最新のもの）をそのまま使う。

```bash
sf log viz
sf log analyze
```

**参照:** `docs/architecture/simulation-policy.md`、`simulator/README.md`、`simulator/sils/gui/README.md`、`docs/guides/flight-log-viz.md`

---

<a id="english"></a>

## 1. Overview

### About This Guide

A step-by-step guide for reproducing, at home or in your lab, the parts of the tutorial you watched without hands-on participation. Each section lists the commands and observation points for that session's demo.

### How to Use It

Every lesson follows the same pattern.

| Step | Command | What it does |
|------|---------|---------------|
| 1 | `sf lesson switch sci2026:N` | Copy Exercise N's student template into `user_code.cpp` |
| 2 | `sf lesson build` | Build |
| 3 | `sf lesson flash` | Flash to the vehicle |
| 4 | (no hardware) `sf lesson sils` | Check it in SILS (your exercise code does not run in VPython) |

This tutorial's exercise numbers map onto a `sci2026` exercise set (course); list them with `sf lesson list --course sci2026`. To try the reference solution directly, use `sf lesson switch sci2026:N --solution`. To see only the diff between the student code and the solution, use `sf lesson solution sci2026:N`.

## 2. Reproducing S2: Environment and Sensor Data

Corresponding to: Exercise 2 (IMU sensor)

```bash
sf doctor
sf lesson switch sci2026:2
sf lesson build
sf lesson flash
sf telemetry
```

**Watch for:** gyro and accelerometer values change as you tilt the vehicle by hand. The Teleplot VSCode extension shows them as live graphs.

**References:** `firmware/vehicle/docs/architecture.md` §2 (four-tier access)

## 3. Reproducing S3: Motor Control and Controller Input

Corresponding to: Exercise 3 (motor control), Exercise 4 (controller input)

Do this on a table. Keep hands clear of the spinning motors. DISARM immediately if anything looks wrong.

```bash
sf lesson switch sci2026:3
sf lesson build
sf lesson flash
```

**Watch for:** motor speed changes with the duty passed to `motor_set_duty()`.

```bash
sf flash controller           # controller-side firmware (once)
sf lesson switch sci2026:4
sf lesson build
sf lesson flash
```

**Watch for:** `rc_roll()`/`rc_pitch()` etc. track the controller sticks.

**References:** `firmware/vehicle/docs/hardware_init.md`, `protocol/spec/` (ControlPacket)

## 4. Reproducing S4: Feedback Control

Corresponding to: Exercises 5-9. **Do this on a table, keep hands clear of the spinning motors, and start at low throttle. DISARM immediately if anything looks wrong.**

### Exercise 5: Rate P-Control

```bash
sf lesson switch sci2026:5
sf lesson build && sf lesson flash
```

Take off and check the response to stick input. Vary `Kp` and feel the difference between oscillation and sluggishness.

### Exercise 6: System Modeling (lecture only)

No hands-on flying here. Check the measured parameters in slide S4's "Measured Parameters" frame ($K_{roll}=102$, $K_{pitch}=70$, $\tau_m\approx0.02$ s) and compute the $K_p$ for a $\zeta=0.7$ design.

### Exercise 7: System Identification

After `sf lesson switch sci2026:7`, set `Kp` in `user_code.cpp` and call `ws::set_rate_target()` to log the rate target, then build and flash.

```bash
sf lesson switch sci2026:7
sf lesson build && sf lesson flash
sf log wifi
sf sysid fit logs/flight_<timestamp>.sflog.zip --plot
```

`sf log wifi` captures while you take off and move the sticks (30 s by default). It saves a StampFly flight-log bundle (`.sflog.zip` -- a zip holding one CSV per signal at its native rate, no filled-in values) under `logs/flight_<date>T<time>.sflog.zip`; replace `<timestamp>` with the actual filename.

**Watch for:** compare the identified $K$, $\tau_m$ against the Exercise 6 theoretical values.

### Exercise 8: PID Control

```bash
sf lesson switch sci2026:8
sf lesson build && sf lesson flash
```

Try the ideal derivative (oscillates) then the incomplete-derivative filter (oscillation drops). Tune the gains against the log while consulting slide S4's "Incomplete Derivative and D-on-M" frame.

### Flying Exercise 5/8's Code in SILS

You can check your code in SILS (`simulator/sils/`) before flying it for real. The exact source that gets flashed to the vehicle runs unmodified there (Code Identity), so you can catch ARM/state-transition/motor-response wiring mistakes with zero crash risk. `sf lesson sils` handles the rebuild and scenario run internally, so typing this one line right after switching is enough.

```bash
sf lesson switch sci2026:8 --solution        # or your own code
sf lesson sils
```

**Watch for:** the pass criteria (`.expect`) check two things: (1) did it lift off (true altitude exceeds 0.1 m), and (2) does tilt stay under 15° (no tumble). The lesson firmware has no altitude loop, so landing is by DISARM descent only. Use `sf lesson sils` for exercise-code SILS runs (`sf sils gui` targets the vehicle firmware). Following the sequence above as written yields `alt_max` ~= 0.64 m and `tilt_max` = 0.0, i.e. PASS.

### Exercise 9: Attitude Estimation

```bash
sf lesson switch sci2026:9
sf lesson build && sf lesson flash
```

Overlay `cf_roll` (your hand-written complementary filter) and `eskf_roll` (the vehicle's default ESKF) in Teleplot and confirm they agree.

### Extension: Autotune

```bash
sf sysid rate-fit logs/flight_<timestamp>.sflog.zip --axis roll -o fit.json
sf sysid rate-tune --fit fit.json --wc 25 --pm 60
```

**References:** this guide's slide deck, Session 4 (`docs/events/sci_tutorial_2026/slides/chapters/sci_s4_pid.tex`) -- the full set of "read later" theory frames (feedback control basics, plant modeling, system identification, PID theory, complementary filter)

## 5. Reproducing S5: Simulator and Analysis Tools

No hardware needed here.

```bash
sf sim run vpython
```

If you have an AtomS3 + Atom JoyStick in USB HID mode, connect it to fly; otherwise it falls back to keyboard control.

```bash
sf sils build                 # once
sf sils gui                   # opens http://127.0.0.1:8765 in your browser
```

**Watch for:** run one scenario and check the 3D playback, graphs, and the pass/fail verdict. Change a gain in the "Parameters" tab and rerun to see the effect.

Reuse the flight-log bundle captured in S4 (the newest one under `logs/`).

```bash
sf log viz
sf log analyze
```

**References:** `docs/architecture/simulation-policy.md`, `simulator/README.md`, `simulator/sils/gui/README.md`, `docs/guides/flight-log-viz.md`
