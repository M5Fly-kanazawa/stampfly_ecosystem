# 復習ハンズオンガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このガイドについて

チュートリアル当日に「見るだけ」で参加した内容を、後日自宅・研究室で実機またはシミュレータを使って再現するための手順書。各セッションのデモに対応するコマンド列と観察ポイントをまとめる。

### 使い方

各レッスンは共通の手順で進む。

| 手順 | コマンド | 内容 |
|------|---------|------|
| 1 | `sf lesson switch N` | レッスン N の学習者テンプレートを `user_code.cpp` にコピー |
| 2 | `sf lesson build` | ビルド |
| 3 | `sf lesson flash` | 実機に書き込み |
| 4 | （実機がない場合）`sf sim run vpython` | シミュレータで代替 |

模範解答をそのまま試したい場合は `sf lesson switch N --solution` を使う。学習者コードと模範解答の差分だけを見たい場合は `sf lesson solution N` を使う。

## 2. S2 の再現: 開発環境とセンサデータ

対応レッスン: Lesson 4（IMU センサ）

```bash
sf doctor                    # 環境診断
sf lesson switch 4
sf lesson build
sf lesson flash
sf telemetry                 # 50Hz テレメトリのライブ表示
```

**観察ポイント:** 機体を手で傾けるとジャイロ・加速度の値が変化する。VSCode 拡張 Teleplot を使うとグラフでも確認できる。

**参照:** `firmware/vehicle/docs/architecture.md` §2（4階層アクセス）、Notebook `01_hello_stampfly.ipynb`

## 3. S3 の再現: モータ制御とコントローラ入力

対応レッスン: Lesson 1（モータ制御）、Lesson 2（コントローラ入力）

```bash
sf lesson switch 1
sf lesson build
sf lesson flash
# プロペラを外した状態で動作確認すること
```

**観察ポイント:** `motor_set_duty()` の duty を変えるとモータの回転数が変わる。

```bash
sf flash controller           # コントローラ側ファーム（初回のみ）
sf lesson switch 2
sf lesson build
sf lesson flash
```

**観察ポイント:** コントローラのスティックを倒すと `rc_roll()`/`rc_pitch()` 等の値が追従する。

**参照:** `firmware/vehicle/docs/hardware_init.md`、`protocol/spec/`（ControlPacket）

## 4. S4 の再現: フィードバック制御

対応レッスン: Lesson 5〜9。**必ず保護メガネを着用し、プロペラガードを装着した状態で低スロットルから試すこと。**

### Lesson 5: レート P 制御

```bash
sf lesson switch 5
sf lesson build && sf lesson flash
```

離陸してスティック操作への応答を確認する。`Kp` を変えて振動と鈍さの違いを体感する。

### Lesson 6: システムモデリング（座学）

実機操作はない。`docs/workshop/slides/beamer/chapters/system_modeling.tex` の実測パラメータ（$K$, $\tau_m$）を確認し、$\zeta=0.7$ 設計の $K_p$ を計算しておく。

### Lesson 7: システム同定

```bash
sf lesson switch 7
# user_code.cpp に Kp をセットし、ws::set_rate_target() で目標角速度を記録
sf lesson build && sf lesson flash
sf log wifi -o flight.csv     # 離陸してスティック操作しながら取得
sf sysid fit flight.csv --kp 0.5 --plot
```

**観察ポイント:** 同定した $K$, $\tau_m$ と Lesson 6 の理論値を比較する。

### Lesson 8: PID 制御

```bash
sf lesson switch 8
sf lesson build && sf lesson flash
```

理想微分（振動する）→不完全微分（振動が減る）の順に試す。ログから調整表（`pid_control.tex`）を見ながらゲインを調整する。

### Lesson 9: 姿勢推定

```bash
sf lesson switch 9
sf lesson build && sf lesson flash
```

Teleplot で `cf_roll`（自作の相補フィルタ）と `eskf_roll`（機体既定の ESKF）を重ねて表示し、一致することを確認する。

### 発展: 自動チューニング

```bash
sf sysid rate-fit flight.csv --axis roll
sf sysid rate-tune --fit result.yaml --wc 25 --pm 60
```

**参照:** `docs/workshop/slides/beamer/chapters/{rate_p_control,system_modeling,system_identification,pid_control,attitude_estimation}.tex`、Notebook `04_pid_theory.ipynb`, `07_system_identification.ipynb`, `08_sensor_fusion.ipynb`

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

**観察ポイント:** シナリオを1本実行し、3D 再生とグラフ、PASS/FAIL ゲートを確認する。「パラメータ」タブでゲインを変えて再実行し、挙動の変化を見る。

```bash
sf log viz flight.csv         # S4 で取得したログを再利用できる
sf log analyze flight.csv
```

**参照:** `docs/architecture/simulation-policy.md`、`simulator/README.md`、`simulator/sils/gui/README.md`、`docs/guides/flight-log-viz.md`、Notebook `15_analysis_toolkit.ipynb`

---

<a id="english"></a>

## 1. Overview

### About This Guide

A step-by-step guide for reproducing, at home or in your lab, the parts of the tutorial you watched without hands-on participation. Each section lists the commands and observation points for that session's demo.

### How to Use It

Every lesson follows the same pattern.

| Step | Command | What it does |
|------|---------|---------------|
| 1 | `sf lesson switch N` | Copy lesson N's student template into `user_code.cpp` |
| 2 | `sf lesson build` | Build |
| 3 | `sf lesson flash` | Flash to the vehicle |
| 4 | (no hardware) `sf sim run vpython` | Use the simulator instead |

To try the reference solution directly, use `sf lesson switch N --solution`. To see only the diff between the student code and the solution, use `sf lesson solution N`.

## 2. Reproducing S2: Environment and Sensor Data

Corresponding lesson: Lesson 4 (IMU sensor)

```bash
sf doctor
sf lesson switch 4
sf lesson build
sf lesson flash
sf telemetry
```

**Watch for:** gyro and accelerometer values change as you tilt the vehicle by hand. The Teleplot VSCode extension shows them as live graphs.

**References:** `firmware/vehicle/docs/architecture.md` §2 (four-tier access), Notebook `01_hello_stampfly.ipynb`

## 3. Reproducing S3: Motor Control and Controller Input

Corresponding lessons: Lesson 1 (motor control), Lesson 2 (controller input)

```bash
sf lesson switch 1
sf lesson build
sf lesson flash
# Remove the propellers before testing
```

**Watch for:** motor speed changes with the duty passed to `motor_set_duty()`.

```bash
sf flash controller           # controller-side firmware (once)
sf lesson switch 2
sf lesson build
sf lesson flash
```

**Watch for:** `rc_roll()`/`rc_pitch()` etc. track the controller sticks.

**References:** `firmware/vehicle/docs/hardware_init.md`, `protocol/spec/` (ControlPacket)

## 4. Reproducing S4: Feedback Control

Corresponding lessons: Lesson 5-9. **Wear eye protection and keep the prop guards on; start at low throttle.**

### Lesson 5: Rate P-Control

```bash
sf lesson switch 5
sf lesson build && sf lesson flash
```

Take off and check the response to stick input. Vary `Kp` and feel the difference between oscillation and sluggishness.

### Lesson 6: System Modeling (lecture only)

No hands-on flying here. Check the measured parameters ($K$, $\tau_m$) in `docs/workshop/slides/beamer/chapters/system_modeling.tex` and compute the $K_p$ for a $\zeta=0.7$ design.

### Lesson 7: System Identification

```bash
sf lesson switch 7
# set Kp in user_code.cpp, and call ws::set_rate_target() to log the rate target
sf lesson build && sf lesson flash
sf log wifi -o flight.csv     # take off and move the sticks while capturing
sf sysid fit flight.csv --kp 0.5 --plot
```

**Watch for:** compare the identified $K$, $\tau_m$ against the Lesson 6 theoretical values.

### Lesson 8: PID Control

```bash
sf lesson switch 8
sf lesson build && sf lesson flash
```

Try the ideal derivative (oscillates) then the incomplete-derivative filter (oscillation drops). Tune the gains against the log while consulting the tuning table in `pid_control.tex`.

### Lesson 9: Attitude Estimation

```bash
sf lesson switch 9
sf lesson build && sf lesson flash
```

Overlay `cf_roll` (your hand-written complementary filter) and `eskf_roll` (the vehicle's default ESKF) in Teleplot and confirm they agree.

### Extension: Autotune

```bash
sf sysid rate-fit flight.csv --axis roll
sf sysid rate-tune --fit result.yaml --wc 25 --pm 60
```

**References:** `docs/workshop/slides/beamer/chapters/{rate_p_control,system_modeling,system_identification,pid_control,attitude_estimation}.tex`, Notebooks `04_pid_theory.ipynb`, `07_system_identification.ipynb`, `08_sensor_fusion.ipynb`

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

**Watch for:** run one scenario and check the 3D playback, graphs, and PASS/FAIL gate. Change a gain in the "Parameters" tab and rerun to see the effect.

```bash
sf log viz flight.csv         # reuse the log captured in S4
sf log analyze flight.csv
```

**References:** `docs/architecture/simulation-policy.md`, `simulator/README.md`, `simulator/sils/gui/README.md`, `docs/guides/flight-log-viz.md`, Notebook `15_analysis_toolkit.ipynb`
