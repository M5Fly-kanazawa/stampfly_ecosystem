# SCI チュートリアル デモ保険（SILS 由来）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

本ディレクトリは、9/10 SCI/SICE チュートリアル講座で会場の実機デモが失敗した場合の保険として、SILS（Software-in-the-Loop、MuJoCo）で飛ばした結果から事前に生成したグラフ・動画・テキストサマリをまとめたものである。すべて決定論的なシミュレーション実行（`SILS_EMU_NOISE=off`、固定シード）の結果で、講師のノート PC 上でそのまま再生でき、会場ネットワークや実機のコンディションに依存しない。

### 対象読者

- 講師本人（本番中に実機デモが失敗した場合、該当ファイルをその場で開いて見せる）
- リハーサル担当者（差し替え・再生成が必要な場合、本書 §3 と `make_fallback.py` を参照）

### 前提

- `docs/sci_tutorial/verification_checklist.md` §5「動画・ログの保険取得」の一部としてここに保険素材を置く。**実機由来の動画・ログ（S1 の POS_HOLD ホバリング実写、S4 の Lesson 5→8 実飛行）は本ディレクトリの対象外** — checklist §5 のとおり講師が別途リハーサルで取得する。ここにあるのは SILS（シミュレーション）由来の素材のみ。
- グラフは全て matplotlib、150dpi・横長・14pt 以上のフォントで、データプロジェクタ投影を想定している。

## 2. ファイル一覧

| セッション | デモ内容 | ファイル | 見せるときの一言（何を指差すか） | 生成コマンド |
|-----------|---------|----------|--------------------------------|-------------|
| S1 | POS_HOLD 水平位置保持: XY軌跡＋位置誤差 | `S1_pos_hold_xy.png` | 左図の赤い四角（POS_HOLD 捕捉点）を指し「ロールで一瞬乱しても、そこに戻って保持する」。右図で誤差が係合後 0.39 m 以内（ゲート 3.0 m）に収まっていることを指す | `sf sils scenario simulator/sils/scenarios/pos_roll.scn --target vehicle --video` |
| S1 | POS_HOLD 高度・姿勢の時系列 | `S1_altitude_attitude.png` | 上段の高度がほぼ一定、下段のロール角が外乱直後に水平へ戻る立ち上がりを指す | 同上 |
| S1 | POS_HOLD 飛行動画（MuJoCo 3D + 状態グラフ） | `S1_pos_hold_flight.mp4` | 「実機に書き込まれるのと同じ制御コードが、そのまま MuJoCo 上で飛んでいる」 | 同上（`--video` が動画を書き出す） |
| S4 | Lesson 5（P）vs Lesson 8（PID）ロール・ステップ応答比較 | `S4_roll_step_p_vs_pid.png` | 上段（P）は破線の指令に速く追いつくがステップ終了直後に -2.4 deg/s のアンダーシュート（振動）が出る。下段（PID）は追いつきはやや遅いが振動が出ない — 上下段の破線と実線のズレ方の違いを指す | 下記 §3 の S4 コマンド列（Lesson 切替を含む） |
| S4 | Lesson 5（P制御）飛行動画 | `S4_lesson5_p_flight.mp4` | 「P だけでも離陸・ホバーはできるが、ステップ応答に振動が残る」 | 同上 |
| S4 | Lesson 8（PID制御）飛行動画 | `S4_lesson8_pid_flight.mp4` | 「同じシナリオを PID にすると応答が滑らかになる」 | 同上 |
| S5 | 合否ゲート判定結果（テキスト） | `S5_gate_result.txt` | 「STABILIZE 飛行の12項目チェックが全て PASS」 | `sf sils scenario simulator/sils/scenarios/stab_flight.scn --target vehicle --video` |
| S5 | 回帰テスト最終サマリ | `S5_regression_summary.txt` | 「34本のシナリオ中 28 PASS、5件は既知の追跡中課題（KNOWN-FAIL、xfail マーカー付き）、1件は workshop 対象のスキップ — 新規の退行はゼロ」 | `sf sils regression` |
| S5 | 姿勢・角速度の時系列 | `S5_attitude_rate.png` | 「ロール+8°→-8°→ピッチ+8°→中立、の3連ステップに追従し、中立に戻すたび自己水平化する」 | `sf sils scenario simulator/sils/scenarios/stab_flight.scn --target vehicle --video` |
| S5 | STABILIZE 飛行動画（MuJoCo 3D + 状態グラフ） | `S5_stab_flight.mp4` | 「STABILIZE はスロットルを自動制御しないので、そのまま上昇し続ける（仕様通り、故障ではない）」 | 同上 |

### 数値サマリ

| 指標 | 値 |
|------|-----|
| S1 POS_HOLD 水平ドリフト最大（係合後） | 0.39 m（ゲート `< 3.0 m`、`sf sils scenario`本体の判定では窓 `[7.6,21.6]s` 基準で `horizontal_drift_max=0.55 m`、いずれも合格） |
| S4 ロールレート・ステップ目標 | 約 15.07 deg/s（スティック +0.3、デッドバンド 0.05 適用後） |
| S4 Lesson 5（P）応答 | ピーク 17.90 deg/s（オーバーシュート）、ステップ終了直後に -2.41 deg/s まで振れる（アンダーシュート） |
| S4 Lesson 8（PID）応答 | ピーク 13.23 deg/s（オーバーシュートなし）、ステップ終了後は 0.68〜10.87 deg/s の範囲で振動なし |
| S5 stab_flight ゲート | 12/12 PASS（`att_rmse=2.82<3.0`, `tilt_max=13.77<18.0`, `duty_max=1.00<1.001`） |
| S5 regression 全体 | 28 PASS + 5 KNOWN-FAIL + 1 SKIP（34本） |

## 3. 再生成方法

生成スクリプトは `make_fallback.py`。既定（引数なし）は既に生成済みの SILS バンドル・永続化済みトラジェクトリから PNG/テキストのみを再構築する（高速・シミュレーション再実行なし）。

```bash
source setup_env.sh
SF_ROOT_OVERRIDE=<このチェックアウトの絶対パス> PYTHONPATH=<同>/lib \
  python3 docs/sci_tutorial/fallback/make_fallback.py
```

S1（`pos_roll.scn`）・S5（`stab_flight.scn`）を SILS で再実行してから生成する場合（vehicle ターゲットのみ、workshop レッスン状態には触れない・安全）:

```bash
python3 docs/sci_tutorial/fallback/make_fallback.py --run
```

S4（Lesson 5 vs Lesson 8）も再実行する場合。**`sf lesson switch` で `firmware/workshop/main/user_code.cpp` を書き換える**ため、実行前のレッスンを `sf lesson list` で確認し、`--restore-lesson` で指定すること（既定は `0` = Lesson 0 student、本タスク実行時の元の状態）:

```bash
python3 docs/sci_tutorial/fallback/make_fallback.py --run --run-s4 --restore-lesson 0
```

新規シナリオ `simulator/sils/scenarios/workshop_acro_step.scn`（S4 用、`workshop_acro.scn` にロール・ステップを挿入した変種、`.expect` なし＝回帰対象外）の詳細は同ファイルのヘッダコメント参照。

## 4. 注意点

- **S4 のロールレート「実測値」は真値姿勢角の数値微分**: workshop ターゲット（`WorkshopControlTask`）は vehicle の `sf::control_output` トピックを発行しないため、モデル一致ゲートが使う `SILS_EMU_RATE_STREAM`（`rate_ref` + ジャイロ実測を記録する仕組み）は workshop では常に空になる。そのため `rate_ref`（指令）はスクリプト化したスティック値 × `rate_max_rp` から計算し、実測側は `trajectory.csv` の真値ロール角（50Hz）を数値微分して代用している。`SILS_EMU_NOISE=off` の決定論実行なので、この代用はノイズ無しジャイロの読み値と数値的に等価。
- **S1 の「外乱」はロール・スティックのステップ**であり、実際の突風（wind force injection）ではない。`pos_roll.scn` は Layer-4 POS_HOLD 回帰スイートの一本で、STABILIZE でロール右ステップを与えて横方向にドリフトさせた後 POS_HOLD に切り替え、ドリフトを止めて保持できるかを検証するシナリオ。「外乱を受けても位置保持が捕捉・保持する」というストーリーとしては S1 のデモに使える。
- **S5 の STABILIZE 飛行は高度を保持しない**（仕様通り）。動画中、機体はスロットルを上げたまま上昇し続けるが、これは ALT_HOLD/POS_HOLD ではなく手動スロットルの STABILIZE モードだからで、故障ではない。
- **`pos_flight.scn`（斜め複合）は使っていない**: ヨートルク権限飽和の既知課題（xfail、`docs/architecture/simulation-policy.md` バックログ#12）により `sf sils scenario` 単体では FAIL 判定になり `--video` が動画を書き出さない。S1 には単軸で PASS する `pos_roll.scn` を採用した。

---

<a id="english"></a>

## 1. Overview

### About this document

This directory holds pre-generated graphs, videos, and text summaries from SILS (Software-in-the-Loop, MuJoCo) flights, kept as a fallback in case the live hardware demo fails at the venue during the 9/10 SCI/SICE tutorial. Every artifact comes from a deterministic simulation run (`SILS_EMU_NOISE=off`, fixed seed), so it plays back on the instructor's laptop with no dependency on venue networking or hardware condition.

### Target audience

- The instructor (open the relevant file on the spot if a live demo fails)
- Whoever reruns the rehearsal (see §3 and `make_fallback.py` if material needs to be regenerated)

### Scope

- This is part of `docs/sci_tutorial/verification_checklist.md` §5 ("Backup Video and Logs"). **Hardware-derived video/logs (S1's real POS_HOLD hover footage, S4's real Lesson 5→8 flight) are OUT of scope here** — per checklist §5, the instructor captures those separately during rehearsal. Only SILS (simulation) material lives in this directory.
- Graphs use matplotlib at 150dpi, landscape, >=14pt fonts, sized for a data projector.

## 2. File list

| Session | Demo content | File | One-liner when presenting (what to point at) | Generation command |
|---------|--------------|------|-----------------------------------------------|---------------------|
| S1 | POS_HOLD horizontal position hold: XY trajectory + position error | `S1_pos_hold_xy.png` | Point at the red square (POS_HOLD engage point) in the left panel: "even after a momentary roll disturbance, it returns and holds." Point at the right panel: error stays within 0.39 m after engage (gate 3.0 m) | `sf sils scenario simulator/sils/scenarios/pos_roll.scn --target vehicle --video` |
| S1 | POS_HOLD altitude/attitude time series | `S1_altitude_attitude.png` | Top: altitude stays essentially flat. Bottom: roll returns to level right after the disturbance | same |
| S1 | POS_HOLD flight video (MuJoCo 3D + state graphs) | `S1_pos_hold_flight.mp4` | "The exact same control code that flashes to hardware is flying here in MuJoCo" | same (`--video` renders it) |
| S4 | Lesson 5 (P) vs Lesson 8 (PID) roll-rate step-response comparison | `S4_roll_step_p_vs_pid.png` | Top (P): catches up to the dashed command fast but undershoots to -2.4 deg/s right after the step ends. Bottom (PID): catches up a bit slower but no ringing — point at how the solid line tracks the dashed line differently in each panel | see the S4 command sequence in §3 (includes lesson switching) |
| S4 | Lesson 5 (P-control) flight video | `S4_lesson5_p_flight.mp4` | "P alone can take off and hover, but the step response still rings" | same |
| S4 | Lesson 8 (PID control) flight video | `S4_lesson8_pid_flight.mp4` | "The same scenario with PID responds more smoothly" | same |
| S5 | Pass/fail gate verdict (text) | `S5_gate_result.txt` | "All 12 checks PASS for the STABILIZE flight" | `sf sils scenario simulator/sils/scenarios/stab_flight.scn --target vehicle --video` |
| S5 | Regression suite final summary | `S5_regression_summary.txt` | "28 PASS out of 34 scenarios, 5 are known, tracked issues (KNOWN-FAIL, xfail-marked), 1 is a workshop-target skip — zero new regressions" | `sf sils regression` |
| S5 | Attitude/angular-rate time series | `S5_attitude_rate.png` | "It tracks a 3-step sequence (roll +8 -> -8 -> pitch +8 -> centre) and self-levels back to zero each time the stick centres" | `sf sils scenario simulator/sils/scenarios/stab_flight.scn --target vehicle --video` |
| S5 | STABILIZE flight video (MuJoCo 3D + state graphs) | `S5_stab_flight.mp4` | "STABILIZE has no automatic throttle/altitude control, so it just keeps climbing — that's by design, not a malfunction" | same |

### Numeric summary

| Metric | Value |
|--------|-------|
| S1 POS_HOLD max horizontal drift (post-engage) | 0.39 m (gate `< 3.0 m`; the scenario's own gate check, windowed `[7.6,21.6]s`, reports `horizontal_drift_max=0.55 m` — both pass) |
| S4 roll-rate step target | ~15.07 deg/s (stick +0.3, after the 0.05 deadband) |
| S4 Lesson 5 (P) response | peak 17.90 deg/s (overshoot); swings to -2.41 deg/s right after the step ends (undershoot) |
| S4 Lesson 8 (PID) response | peak 13.23 deg/s (no overshoot); stays within 0.68-10.87 deg/s after the step, no ringing |
| S5 stab_flight gate | 12/12 PASS (`att_rmse=2.82<3.0`, `tilt_max=13.77<18.0`, `duty_max=1.00<1.001`) |
| S5 regression overall | 28 PASS + 5 KNOWN-FAIL + 1 SKIP (34 total) |

## 3. Regenerating

`make_fallback.py` is the generator. With no flags, it rebuilds only the PNGs/text from SILS bundles and persisted trajectory data that already exist (fast, no simulation rerun).

```bash
source setup_env.sh
SF_ROOT_OVERRIDE=<absolute path to this checkout> PYTHONPATH=<same>/lib \
  python3 docs/sci_tutorial/fallback/make_fallback.py
```

To re-run S1 (`pos_roll.scn`) and S5 (`stab_flight.scn`) in SILS first (vehicle target only — never touches workshop lesson state, safe):

```bash
python3 docs/sci_tutorial/fallback/make_fallback.py --run
```

To also re-run S4 (Lesson 5 vs Lesson 8). **This rewrites `firmware/workshop/main/user_code.cpp` via `sf lesson switch`** — check the current lesson with `sf lesson list` first and pass it via `--restore-lesson` (default `0` = Lesson 0 student, the state this task found the repo in):

```bash
python3 docs/sci_tutorial/fallback/make_fallback.py --run --run-s4 --restore-lesson 0
```

See `simulator/sils/scenarios/workshop_acro_step.scn`'s header comment for the new scenario (S4's roll-step variant of `workshop_acro.scn`; no `.expect` — not part of the regression gate).

## 4. Caveats

- **S4's "measured" roll rate is a numerical derivative of the truth attitude angle.** The workshop target (`WorkshopControlTask`) never publishes vehicle's `sf::control_output` topic, so `SILS_EMU_RATE_STREAM` (the model-match gate's rate_ref + measured-gyro recorder) stays empty for it. `rate_ref` (commanded) is computed from the scripted stick value x `rate_max_rp`; the "measured" side is a numerical derivative of the 50Hz truth roll angle in `trajectory.csv`. Since the run is deterministic with `SILS_EMU_NOISE=off`, this stand-in is numerically equivalent to a noiseless gyro reading.
- **S1's "disturbance" is a scripted roll-stick step**, not an injected wind force. `pos_roll.scn` is one of the Layer-4 POS_HOLD regression scenarios: a STABILIZE roll-right step induces lateral drift, then POS_HOLD engages and must arrest and hold it. It works fine as the "disturbed then recaptured and held" story for S1.
- **S5's STABILIZE flight does not hold altitude** (by design). In the video the craft keeps climbing under sustained throttle — that is STABILIZE's manual-throttle behavior, not ALT_HOLD/POS_HOLD, and not a malfunction.
- **`pos_flight.scn` (combined diagonal) was NOT used**: it carries a known yaw torque-authority saturation issue (xfail, `docs/architecture/simulation-policy.md` backlog #12), so a standalone `sf sils scenario` run reports FAIL and `--video` never renders. S1 uses `pos_roll.scn` instead, which passes on a single axis.
