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
| 4 | （実機がない場合）`sf sim run vpython` | シミュレータで代替 |

本チュートリアルの実習番号は `sci2026` という実習構成（コース）に対応付けられている。一覧は `sf lesson list --course sci2026` で確認できる。模範解答をそのまま試したい場合は `sf lesson switch sci2026:N --solution` を使う。学習者コードと模範解答の差分だけを見たい場合は `sf lesson solution sci2026:N` を使う。

## 2. S2 の再現: 開発環境とセンサデータ

対応: 実習 2（IMU センサー、Workshop の Lesson 4）

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

対応: 実習 3（モータ制御、Workshop の Lesson 1）、実習 4（コントローラ入力、Workshop の Lesson 2）

```bash
sf lesson switch sci2026:3
sf lesson build
sf lesson flash
# 机上で行うこと。モータ回転中は手を近づけない。異常時は即 DISARM
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

対応: 実習 5〜9（Workshop の Lesson 5〜9）。**必ず机上で行い、モータ回転中は手を近づけず、低スロットルから試すこと。異常時は即 DISARM。**

### 実習 5: レート P 制御

```bash
sf lesson switch sci2026:5
sf lesson build && sf lesson flash
```

離陸してスティック操作への応答を確認する。`Kp` を変えて振動と鈍さの違いを体感する。

### 実習 6: システムモデリング（座学）

実機操作はない。`docs/events/stampfly_workshop/slides/chapters/system_modeling.tex` の実測パラメータ（$K$, $\tau_m$）を確認し、$\zeta=0.7$ 設計の $K_p$ を計算しておく。

### 実習 7: システム同定

```bash
sf lesson switch sci2026:7
# user_code.cpp に Kp をセットし、ws::set_rate_target() で目標角速度を記録
sf lesson build && sf lesson flash
sf log wifi -o flight.csv     # 離陸してスティック操作しながら取得
sf sysid fit flight.csv --kp 0.5 --plot
```

**観察ポイント:** 同定した $K$, $\tau_m$ と実習 6 の理論値を比較する。

### 実習 8: PID 制御

```bash
sf lesson switch sci2026:8
sf lesson build && sf lesson flash
```

理想微分（振動する）→不完全微分（振動が減る）の順に試す。ログから調整表（`pid_control.tex`）を見ながらゲインを調整する。

### 実習 5/8 のコードを SILS で飛ばす

実機を飛ばす前に、書いたコードを SILS（`simulator/sils/`）で確かめられる。実機に書き込まれるのと同じソースがそのまま動く（Code Identity）ので、墜落のリスクなしに ARM・状態遷移・モータ応答の配線ミスに気付ける。`sf lesson switch` はコピー先ファイルの更新日時を保持したままコピーするため、切替後すぐに `sf sils build` してもソースの変更が検出されず古いビルドのままになる。そのため `touch` で更新日時を進めてから再ビルドする、という手順を毎回踏むこと（省略可能な例外ではなく通常の手順）。

```bash
sf sils build --target workshop
sf lesson switch sci2026:8 --solution        # または自分のコード
touch firmware/workshop/main/user_code.cpp   # 切替はファイルの更新日時を保持するため touch する
sf sils build --target workshop              # 新しいコードを反映するため再ビルド
sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop
```

**観察ポイント:** 合格基準（`.expect`）は「離陸したか（真値高度が 0.1 m を超える）」「傾き 15° 未満か（転倒しない）」の2点。workshop ファームには高度ループがないため、着陸は DISARM による降下のみ。`sf sils gui` は現状ブラウザ画面から常に vehicle ファームを対象に実行し、workshop（学習者コード）を選ぶ操作はないので、学習者コードの実行には上記の CLI コマンドを使う。上記の手順どおりに実行すると `alt_max` ≈ 0.64 m、`tilt_max` = 0.0 で PASS になる。

### 実習 9: 姿勢推定

```bash
sf lesson switch sci2026:9
sf lesson build && sf lesson flash
```

Teleplot で `cf_roll`（自作の相補フィルタ）と `eskf_roll`（機体既定の ESKF）を重ねて表示し、一致することを確認する。

### 発展: 自動チューニング

```bash
sf sysid rate-fit flight.csv --axis roll -o fit.json
sf sysid rate-tune --fit fit.json --wc 25 --pm 60
```

**参照:** `docs/events/stampfly_workshop/slides/chapters/{rate_p_control,system_modeling,system_identification,pid_control,attitude_estimation}.tex`

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

```bash
sf log viz flight.csv         # S4 で取得したログを再利用できる
sf log analyze flight.csv
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
| 4 | (no hardware) `sf sim run vpython` | Use the simulator instead |

This tutorial's exercise numbers map onto a `sci2026` exercise set (course); list them with `sf lesson list --course sci2026`. To try the reference solution directly, use `sf lesson switch sci2026:N --solution`. To see only the diff between the student code and the solution, use `sf lesson solution sci2026:N`.

## 2. Reproducing S2: Environment and Sensor Data

Corresponding to: Exercise 2 (IMU sensor, Workshop Lesson 4)

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

Corresponding to: Exercise 3 (motor control, Workshop Lesson 1), Exercise 4 (controller input, Workshop Lesson 2)

```bash
sf lesson switch sci2026:3
sf lesson build
sf lesson flash
# Do this on a table. Keep hands clear of the spinning motors. DISARM immediately if anything looks wrong
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

Corresponding to: Exercises 5-9 (Workshop Lessons 5-9). **Do this on a table, keep hands clear of the spinning motors, and start at low throttle. DISARM immediately if anything looks wrong.**

### Exercise 5: Rate P-Control

```bash
sf lesson switch sci2026:5
sf lesson build && sf lesson flash
```

Take off and check the response to stick input. Vary `Kp` and feel the difference between oscillation and sluggishness.

### Exercise 6: System Modeling (lecture only)

No hands-on flying here. Check the measured parameters ($K$, $\tau_m$) in `docs/events/stampfly_workshop/slides/chapters/system_modeling.tex` and compute the $K_p$ for a $\zeta=0.7$ design.

### Exercise 7: System Identification

```bash
sf lesson switch sci2026:7
# set Kp in user_code.cpp, and call ws::set_rate_target() to log the rate target
sf lesson build && sf lesson flash
sf log wifi -o flight.csv     # take off and move the sticks while capturing
sf sysid fit flight.csv --kp 0.5 --plot
```

**Watch for:** compare the identified $K$, $\tau_m$ against the Exercise 6 theoretical values.

### Exercise 8: PID Control

```bash
sf lesson switch sci2026:8
sf lesson build && sf lesson flash
```

Try the ideal derivative (oscillates) then the incomplete-derivative filter (oscillation drops). Tune the gains against the log while consulting the tuning table in `pid_control.tex`.

### Flying Exercise 5/8's Code in SILS

You can check your code in SILS (`simulator/sils/`) before flying it for real. The exact source that gets flashed to the vehicle runs unmodified there (Code Identity), so you can catch ARM/state-transition/motor-response wiring mistakes with zero crash risk. `sf lesson switch` copies the file while preserving its old modification time, so building right after a switch will not pick up the change and reuses the stale build. Touching the file to bump its mtime and then rebuilding is the normal procedure every time -- not an occasional workaround.

```bash
sf sils build --target workshop
sf lesson switch sci2026:8 --solution        # or your own code
touch firmware/workshop/main/user_code.cpp   # switching preserves the mtime, so touch it
sf sils build --target workshop              # rebuild so the new code is compiled
sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop
```

**Watch for:** the pass criteria (`.expect`) check two things: (1) did it lift off (true altitude exceeds 0.1 m), and (2) does tilt stay under 15° (no tumble). The workshop firmware has no altitude loop, so landing is by DISARM descent only. `sf sils gui` currently always runs against the vehicle firmware from the browser — there is no control to select workshop (learner code) — so use the CLI command above to run learner code. Following the sequence above as written yields `alt_max` ~= 0.64 m and `tilt_max` = 0.0, i.e. PASS.

### Exercise 9: Attitude Estimation

```bash
sf lesson switch sci2026:9
sf lesson build && sf lesson flash
```

Overlay `cf_roll` (your hand-written complementary filter) and `eskf_roll` (the vehicle's default ESKF) in Teleplot and confirm they agree.

### Extension: Autotune

```bash
sf sysid rate-fit flight.csv --axis roll -o fit.json
sf sysid rate-tune --fit fit.json --wc 25 --pm 60
```

**References:** `docs/events/stampfly_workshop/slides/chapters/{rate_p_control,system_modeling,system_identification,pid_control,attitude_estimation}.tex`

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

```bash
sf log viz flight.csv         # reuse the log captured in S4
sf log analyze flight.csv
```

**References:** `docs/architecture/simulation-policy.md`, `simulator/README.md`, `simulator/sils/gui/README.md`, `docs/guides/flight-log-viz.md`
