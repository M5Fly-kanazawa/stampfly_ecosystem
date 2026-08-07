# POS_HOLD の「フラフラ揺れ」— 速度ノイズ増幅と対策（将来課題）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

POS_HOLD（位置保持）が成立した（±6〜7cm・RMS 16mm、commit `880634b`）後も残る、機体の **~1Hz の「フラフラ揺れ」** の原因を実機ログから特定し、対策候補と推奨設計を記録する。**今すぐの修正でなく、後で詰める将来課題**としての覚書。

### 一文で言うと

> **フラフラは「位置制御の不安定」ではない。オプティカルフロー由来の速度推定ノイズを、トルク効き不足を補うために上げた高い速度ループゲイン `vel.kp` が傾き指令に増幅し、姿勢ループが律儀に追従して機体が ~1Hz でティルトを繰り返している。**

### 対象読者

vehicle の POS_HOLD カスケード・ESKF 速度推定・実機チューニングに関わる人。

## 2. 観測と原因

### 2.1 周波数分解（手放しホバー、現行 pos.kp=0.4 / vel.kp=3.0 / att.ti=2、log `20260622T183043`）

| 信号 | 卓越周波数 | 中身 |
|------|-----------|------|
| 位置ドリフト | 0.05–0.3 Hz（61%） | ゆっくりした漂い（20 mm RMS）— フラフラとは別物 |
| 傾き指令 `angle_ref` | 1–8 Hz に 55% | RMS 0.51° のうち **0.43° が速度ノイズ由来** |
| ジャイロ（見える揺れ） | **1.0 Hz**（rms 0.115 rad/s, 1–2 Hz に 65%） | フラフラの実体 |
| `angle_ref` → ジャイロ コヒーレンス | **0.72** | **揺れは速度ループが「指令」している** |

### 2.2 メカニズム

```
オプティカルフローの速度ノイズ（>0.5 Hz で std ≈ 0.025 m/s）
   × vel.kp(=3.0) ÷ g          ← 速度ループが加速度（=傾き）指令に変換
   = 傾き指令ノイズ ≈ 0.43°      ← angle_ref RMS 0.51° のほぼ全部
   → 姿勢ループが忠実に追従（コヒーレンス 0.72）
   → 機体が ~1 Hz で ±0.43° ティルトを繰り返す ＝ フラフラ
```

傾きジッタ = `vel.kp · σ_v_noise / g` = 3.0 × 0.025 / 9.81 = **0.43°**。

### 2.3 なぜ vel.kp が高いのか（トレードオフの根）

`vel.kp` は **意図的に 0.8 → 3.0 へ上げた**（commit `38dc38d`, `880634b`）。理由は実機モータの**トルク効き 0.4〜0.7 倍**（[[poshold_accel_compensation]] 残課題 #5、`poshold_attID.py` で同定）を補償して発散を止め、位置を締める（RMS 31 → 16 mm）ため。その**代償が速度ノイズの増幅**。つまり「位置の締まり」と「フラフラ」は vel.kp を介した直接のトレードオフであり、ゲイン単独では両立しない。

### 2.4 att.ti=2 の寄与

パイロット好みで採用した `att.ti=2`（commit `abce8e6`）は、姿勢の動きを増やしている（ジャイロ rms 0.092 → 0.115、位置 RMS 16 → 20 mm、log `173150`=att.ti4 比）。フラフラ重視なら `att.ti` を 3〜4 に戻すと減る（が「締まった手応え」は薄まる）。

## 3. 対策候補

| 案 | やること | 効果 / 代償 | 種別 |
|----|---------|------------|------|
| **A. vel.kp を下げる** | `param set position.vel.kp 2.0`（2.5 も） | フラフラ減（ジッタ 0.43 → 0.29°）／位置が緩む（16 → 31 mm 方向） | ライブ param |
| **B. att.ti を戻す** | `attitude.*.ti 3〜4` | フラフラ減／手応え薄まる | ライブ param |
| **C. 速度をローパス（本筋）** | フロー速度を位置カスケード前で低域通過 | **締まりを保ったままフラフラだけ除去** | コード＋SILS |
| **D. 根本（ノイズ源）** | 床テクスチャ濃く／ホバー高度↓（フロー SN↑）、高効率モータ（トルク効き↑→vel.kp 下げられる） | ノイズ源そのものを減らす | ハード／環境 |

## 4. 推奨設計 — 速度ローパス（案 C）

**狙い**: `vel.kp` を下げずに（＝位置の締まりを保ったまま）、増幅されている高周波速度ノイズだけを削る。

### 4.1 周波数の分離が成立する

- 位置制御の帯域: 位置ループ ≈ 0.06 Hz、速度ループ実効クロスオーバー ≈ 0.19 Hz（vel.kp·K/g ≈ 1.2 rad/s）
- 増幅されているノイズ: ~1 Hz（>0.5 Hz）
- → **両者は十分離れている**。カットオフ ~0.5 Hz（3 rad/s）の1次ローパスなら、制御信号（<0.2 Hz）はほぼ素通し、ノイズ（>0.5 Hz）を減衰できる。

### 4.2 位相余裕への影響を必ず確認

ローパスは**速度ループの内側**に入る（位置カスケードが読む速度を平滑する）ため位相遅れを足す。速度ループ実効クロスオーバー 0.19 Hz に対しカットオフ 0.5 Hz の遅れは atan(0.19/0.5) ≈ **21°** で許容範囲だが、**K≈0.4g で既に余裕が薄い**ループゆえ、カットオフを下げ過ぎると不安定化する。設計手順:

1. `poshold_loop_design.py`（ファーム忠実 sim）にローパスを1次の状態として追加し、`vel.kp=3.0` 据え置きで閉ループ極（ω, σ）を確認。カットオフを 0.3〜0.8 Hz で振り、**worst-σ < −0.03 を K∈[2.8,7]·τ∈[50,300]ms で維持**するカットオフを選ぶ。
2. 平滑後の速度ノイズ std から傾きジッタ `vel.kp·σ/g` を再計算し、目標（例 < 0.2°）に入るか確認。
3. SILS `pos_*` 退行確認（理想フローゆえノイズ効果は出ないが安定性は確認できる）。
4. 実機で `sf log wifi` → `poshold_analysis.py` でジャイロ rms・コヒーレンスの低下を確認。

### 4.3 実装箇所

`pid_controller.cpp` `computePositionHold` が読む `state.velocity[0..1]` を、メンバの1次LPF状態で平滑してから速度ループへ渡す。カットオフは param 化（例 `position.vel_lpf_hz`）。`reset()` で LPF 状態クリア。位置（`state.position`）はフィルタしない（位置ループの低域信号はノイズが乗っていない）。

## 5. 参考

| 項目 | 場所 |
|------|------|
| 分解解析の方法 | `analysis/scripts/poshold_analysis.py`（帯域別パワー・速度ノイズ）、本文 2.1 の手順 |
| ループ設計 sim | `analysis/scripts/poshold_loop_design.py`（ファーム忠実閉ループ＋ロバスト探索） |
| 傾き達成度/フロー同定 | `analysis/scripts/poshold_attID.py` |
| 関連ログ | `logs/20260622T183043`（現行=att.ti2）, `173150`（att.ti4 比較） |
| 関連 commit | `880634b`（vel.kp 3.0 化）, `abce8e6`（att.ti 2.0 化） |
| 根本原因 | [[poshold_accel_compensation]]（トルク効き 0.4〜0.7 倍） |

---

<a id="english"></a>

## 1. Overview

### About this document

After fixed-point POS_HOLD was achieved (±6–7 cm, 16 mm RMS, commit `880634b`), a residual
**~1 Hz "floating wobble"** remains. This note records the flight-log root cause, the fix
options, and the recommended design. It is a **future-task memo, not an immediate fix**.

### In one sentence

> **The wobble is NOT position-loop instability. The high velocity-loop gain `vel.kp`
> (raised to overcome the weak motor torque effectiveness) amplifies the optical-flow
> velocity-estimate noise into tilt commands, which the attitude loop faithfully tracks,
> so the craft tilts back and forth at ~1 Hz.**

## 2. Observation and cause

### 2.1 Spectral decomposition (hands-off hover, current pos.kp=0.4 / vel.kp=3.0 / att.ti=2, log `20260622T183043`)

| Signal | Dominant freq | Content |
|--------|---------------|---------|
| Position drift | 0.05–0.3 Hz (61%) | slow wander (20 mm RMS) — separate from the wobble |
| Tilt command `angle_ref` | 55% in 1–8 Hz | of 0.51° RMS, **0.43° is velocity-noise-driven** |
| Gyro (the visible wobble) | **1.0 Hz** (rms 0.115 rad/s, 65% in 1–2 Hz) | the wobble itself |
| `angle_ref` → gyro coherence | **0.72** | **the wobble is COMMANDED by the velocity loop** |

### 2.2 Mechanism

Tilt jitter = `vel.kp · σ_v_noise / g` = 3.0 × 0.025 / 9.81 = **0.43°**, broadband ~1 Hz,
tracked by the attitude loop (coherence 0.72) → visible ~1 Hz wobble.

### 2.3 Why vel.kp is high (the trade-off)

`vel.kp` was deliberately raised 0.8 → 3.0 to compensate the real motor torque effectiveness
(~0.4–0.7×, see [[poshold_accel_compensation]] #5) and tighten the hold (31 → 16 mm). The
cost is exactly this noise amplification — "hold tightness" and "wobble" trade off through
`vel.kp`, so gains alone cannot satisfy both.

### 2.4 att.ti=2 contribution

The pilot-preferred `att.ti=2` (commit `abce8e6`) adds attitude activity (gyro rms
0.092 → 0.115, position 16 → 20 mm vs the att.ti=4 log `173150`); `att.ti` 3–4 reduces the
wobble at the cost of the firm feel.

## 3. Options

| # | Action | Effect / cost | Type |
|---|--------|---------------|------|
| A | lower `vel.kp` (3.0 → 2.0–2.5) | less wobble (jitter 0.43 → 0.29°) / looser hold | live param |
| B | restore `att.ti` (3–4) | less wobble / less firm feel | live param |
| C | **low-pass the velocity** before the position cascade | **removes the wobble while KEEPING tightness** | code + SILS |
| D | root: floor texture / lower hover (flow SNR), better motors (torque → lower vel.kp) | reduce the noise source | hw / env |

## 4. Recommended design — velocity low-pass (option C)

**Goal:** strip the amplified high-frequency velocity noise WITHOUT lowering `vel.kp`
(keep the hold tight).

**Frequency separation holds:** position-control band ≈ 0.06–0.19 Hz; amplified noise ≈ 1 Hz.
A 1st-order LPF at ~0.5 Hz passes the control signal and attenuates the noise.

**Phase-margin check is mandatory:** the LPF sits INSIDE the velocity loop and adds lag.
At the velocity crossover (0.19 Hz) a 0.5 Hz LPF adds ≈ 21° lag — acceptable, but the loop is
already marginal at K≈0.4g, so:
1. Add the LPF as a 1st-order state in `poshold_loop_design.py`; keep `vel.kp=3.0`; sweep the
   cutoff 0.3–0.8 Hz and pick the one keeping **worst-σ < −0.03 over K∈[2.8,7], τ∈[50,300] ms**.
2. Recompute the tilt jitter from the smoothed noise; confirm it meets the target (e.g. < 0.2°).
3. SILS `pos_*` regression (ideal flow won't show the noise benefit but confirms stability).
4. Hardware: `sf log wifi` → `poshold_analysis.py`; confirm lower gyro rms / coherence.

**Where:** smooth `state.velocity[0..1]` in `computePositionHold` with a 1st-order LPF state
before the velocity loop; param the cutoff (`position.vel_lpf_hz`); clear in `reset()`. Do NOT
filter `state.position` (the position-loop signal is not noisy).

## 5. References

- Decomposition method: `analysis/scripts/poshold_analysis.py` + §2.1.
- Loop-design sim: `analysis/scripts/poshold_loop_design.py`.
- Tilt-achievement / flow ID: `analysis/scripts/poshold_attID.py`.
- Logs: `logs/20260622T183043` (current, att.ti2), `173150` (att.ti4 ref).
- Commits: `880634b` (vel.kp 3.0), `abce8e6` (att.ti 2.0). Root cause: [[poshold_accel_compensation]].
