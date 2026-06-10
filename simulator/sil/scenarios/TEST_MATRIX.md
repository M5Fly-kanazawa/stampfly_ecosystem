# SIL シナリオ・テストマトリクス / SIL Scenario Test Matrix

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

vehicle_new の飛行を SIL（物理真値）で検証するシナリオスイートを、**層（Layer 1〜4）× 軸（roll/pitch/yaw/複合）× ゲート（G1〜G4）** の観点で一覧化する。発展的プラント同定（`development_roadmap.md` §3）の各層を、まず物理真値の SIL ゲートで通してから実機に進む。

### 合格判定の2系統

各 `*.scn` には `*.expect` が付き、2系統で合否を機械判定する：

| 系統 | 何を見るか | ゲート |
|------|-----------|--------|
| **ログ文字列** | 状態遷移の並び・順序（`ARM accepted` 等） | G1 |
| **数値メトリクス** | `trajectory.csv` の物理真値＋推定から算出（`metric <name> <op> <value> in <t0> <t1>`） | G2/G3/G4 |

ゲート定義（`RESET_PLAN.md` §4）：

| ゲート | 意味 | 代表メトリクス |
|--------|------|----------------|
| **G1** 起動・状態遷移 | ARM→離陸→飛行→着陸の遷移が正しく進むか | ログ（`log_contains`/`order`） |
| **G2** 推定の追従 | 推定が物理真値にどれだけ一致するか | `att_rmse` / `alt_rmse` |
| **G3** 閉ループの安定 | 姿勢・位置が発散せず有界か | `horizontal_drift_max` / `tilt_max` / `alt_band` |
| **G4** アクチュエータ健全性 | モータ指令が飽和しないか | `duty_max` |

## 2. マトリクス

### 軸別→複合の構成

各層は **個別軸を1軸ずつ → 最後に複合** で検証する（`<layer>_<axis>.scn` が個別、`<layer>_flight.scn` が複合キャップストーン）。

| 層 | モード | シナリオ | 励振軸 | G1 | G2 | G3 | G4 |
|----|--------|----------|--------|----|----|----|----|
| **L1** | ACRO（レート） | `acro_flight` | roll±/pitch±/yaw+（1本に全軸ダブレット） | ✅ | att_rmse<3 | tilt_max<25 | （注1） |
| **L2** | STABILIZE（姿勢） | `stab_flight` | roll±/pitch+（複合） | ✅ | att_rmse<3 | tilt_max<18 | duty<0.92 |
| **L3** | ALT_HOLD（高度） | `alt_flight` | 鉛直のみ | ✅ | alt_rmse<0.08 | alt_band<0.35 | duty<0.92 |
| **L4** | POS_HOLD（位置） | `pos_roll` | **ロール単独** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_pitch` | **ピッチ単独** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_flight` | **斜め複合（capstone）** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_yaw` | **ヨー回転後に保持** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |

**注1（ACRO の G4）:** ACRO のレートダブレットは設計上モータを瞬間飽和させる（広帯域励振、roadmap §3.1.3）ため、duty は意図的に未ゲート。

### その他のシナリオ（非層別）

| シナリオ | 目的 | ターゲット |
|----------|------|-----------|
| `disturb` | P7 外乱回復（横風＋モータ故障） | vehicle / vehicle_new |
| `modeswitch` | P8 飛行中モード切替（ALT↔POS）で姿勢/高度有界 | vehicle_new |
| `crash_refly` | P8 ★ロバスト再飛行（墜落→自動DISARM→物理ハンドリング→再校正→再飛行）。`--duration 33000000` 必須 | vehicle_new |
| `hover_alt` / `hover_long` | 高度保持ホバー（短/長時間） | vehicle（旧） |
| `hover_espnow` | ESP-NOW ホバー（仮想 pilot） | vehicle（旧） |
| `console_cli` | シリアル CLI 決定論検証（非飛行）。expect は旧 vehicle の CLI 出力（`StampFly RTOS` 等）にアンカーしており、emu の key 入力チャネルも vehicle_new の esp_console シムに未配線（2026-06-10 確認）。vehicle_new 対応は CLI フィーダ配線＋expect 更新が必要 | vehicle（旧） |
| `pairing` | ペアリングハンドシェイク（未ペア起動→自動Pairing→bind）＋混信拒否（誤MAC送信機のARM/離陸をフィルタが破棄）。`--unpaired` 必須 | vehicle_new |

**P8 ロバスト再飛行（`crash_refly`）が炙り出した2つのファーム欠陥（修正済）:**
1. **ESKF 姿勢の latch**: 墜落で姿勢推定が真値から大きく外れると accel-attitude χ² ゲートが補正自体を棄却し続け自己復帰しない。設置時（IDLE_HELD→IDLE_GROUND、機体が level・静止と既知）に ESKF を Reset して姿勢を level へ再初期化することで解決。
2. **モード未伝播**: 接地時の飛行モード STABILIZE リセットが `StateManager::mode_` を変えるだけで制御器に伝わらず（制御器は `ControllerCmd::ModeChange` 経由でのみモードを知る）、ALT/POS 飛行後の再離陸が古いホバー推力モードのまま上昇しない。リセット時に onModeChange を発火させて解決。

## 3. 実行方法

```bash
source setup_env.sh
sf sil build vehicle_new
sf sil scenario simulator/sil/scenarios/pos_flight.scn --target vehicle_new          # ゲート判定
sf sil scenario simulator/sil/scenarios/pos_flight.scn --target vehicle_new --video  # ＋レビュー動画
```

`metric` メトリクス名一覧（`lib/sfcli/commands/sil.py` `_traj_metric`）:
`horizontal_drift_max`, `roll_rmse`, `pitch_rmse`, `att_rmse`, `alt_rmse`,
`tilt_max`, `alt_band`, `alt_mean`, `alt_min`, `alt_max`, `duty_max`。

## 4. カバレッジ状況（2026-06-06）

- **L1〜L4 全シナリオが G1（ログ）＋ G2/G3/G4（数値）で PASS**（`SIL_EMU_NOISE=off`、決定論）。
- **L4 POS_HOLD は roll/pitch/斜め/yaw の全4軸がタイトに保持**（採用した運動加速度補償 `eskf.accel_comp.enable=1` ＋ 位置速度ループ `position.vel.kp=0.8`）。drift ≤ 1.1m・終端オフセット ≤ 1.0m、N0 センサノイズ下でも保持（drift ≤ 1.3m）。推定器の汚染を直したことで速度ゲインを 0.3→0.8 に上げられた（旧版は推定器が壊れ 0.3 が限界・斜めで発散）。
- **残課題:** n1/n2（過酷な振動）下の POS_HOLD は baseline 同様に飛び去る（振動処理は別課題）。実機 ESP-IDF ビルドは未検証（host SIL は全ソース通過）。

---

<a id="english"></a>

## 1. Overview

This document enumerates the vehicle_new SIL flight-validation scenarios along
**Layer (1–4) × axis (roll/pitch/yaw/combined) × gate (G1–G4)**. Each layer of the
layer-by-layer plant identification (`development_roadmap.md` §3) must pass the
physical-truth SIL gates before moving to hardware.

### Two verdict tracks

Each `*.scn` has a `*.expect` judged two ways: **log strings** (state-transition
order ⇒ G1) and **numerical metrics** computed from `trajectory.csv`
(`metric <name> <op> <value> in <t0> <t1>` ⇒ G2/G3/G4). Gate definitions are in
`RESET_PLAN.md` §4 (G1 boot/transitions, G2 estimate tracking, G3 closed-loop
boundedness, G4 actuator health).

## 2. Matrix

Per layer, isolate each axis first (`<layer>_<axis>.scn`), then a combined capstone
(`<layer>_flight.scn`).

| Layer | Mode | Scenario | Excited axis | G1 | G2 | G3 | G4 |
|-------|------|----------|--------------|----|----|----|----|
| **L1** | ACRO (rate) | `acro_flight` | roll±/pitch±/yaw+ (all-axis doublets) | ✅ | att_rmse<3 | tilt_max<25 | (note 1) |
| **L2** | STABILIZE (attitude) | `stab_flight` | roll±/pitch+ (combined) | ✅ | att_rmse<3 | tilt_max<18 | duty<0.92 |
| **L3** | ALT_HOLD (altitude) | `alt_flight` | vertical only | ✅ | alt_rmse<0.08 | alt_band<0.35 | duty<0.92 |
| **L4** | POS_HOLD (position) | `pos_roll` | **roll only** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_pitch` | **pitch only** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_flight` | **diagonal (capstone)** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |
| **L4** | POS_HOLD | `pos_yaw` | **hold at rotated heading** | ✅ | att_rmse<5 | drift<3, tilt<18 | duty<0.90 |

**Note 1 (ACRO G4):** ACRO rate doublets saturate the motors briefly BY DESIGN
(broadband excitation, roadmap §3.1.3), so duty is intentionally not gated.

## 3. How to run

```bash
source setup_env.sh
sf sil build vehicle_new
sf sil scenario simulator/sil/scenarios/pos_flight.scn --target vehicle_new
sf sil scenario simulator/sil/scenarios/pos_flight.scn --target vehicle_new --video
```

## 4. Coverage status (2026-06-06)

- All L1–L4 scenarios PASS on G1 (logs) + G2/G3/G4 (numerical), `SIL_EMU_NOISE=off`,
  deterministic.
- L4 POS_HOLD holds TIGHTLY on all four axes (roll/pitch/diagonal/yaw) with the adopted
  acceleration compensation (`eskf.accel_comp.enable=1`) + the position velocity loop at
  `position.vel.kp=0.8` (drift ≤ 1.1 m, final offset ≤ 1.0 m; ≤ 1.3 m under N0). Fixing the
  estimator contamination is what let the velocity gain rise from 0.3 to 0.8 (with the old
  contaminated estimate 0.3 was the limit and the diagonal diverged).
- Open: POS_HOLD under n1/n2 (severe vibration) flies away as the baseline does
  (vibration handling is separate); the on-target ESP-IDF build is unverified.
