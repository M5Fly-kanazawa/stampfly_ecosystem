# 初回離陸オーバーシュートの調査（Takeoff Overshoot: first vs re-takeoff）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

実機パイロット報告（2026-06-14）「**初めて離陸する時は目標 0.5m からのオーバーシュートが大きい。一度着陸してからの再離陸とは違う。ブート後の初期化と2回目以降の離陸リセットの違いを調べる必要がありそう**」の調査結果（リファクタC）。

### 結論（先に）

- **制御器・リセット経路のバグではない。** ARM→`controller Reset`→`onTakeoff` は初回も再離陸も**同一経路**。
- 真因は**推定器（ESKF）のバイアス収束差**: **ARM は ESKF バイアスをリセットしない**（`InflateCov(Attitude)` のみ、detailed_design §3 注1）。よって**再離陸は飛行#1で収束したバイアスを継承**するが、**初回離陸は起動校正のバイアス**（残差あり）を使う。
- **SIL は起動校正が完璧（誤差ゼロ）ゆえ非対称を再現しない** → SIL では投機的修正を検証できない。実機側の課題。

## 2. 詳細

### SIL A/B 測定（`alt_double_takeoff.scn`, noise off / GE off）

1ランで「初回離陸 → 着陸 → 再離陸」を行い、各 climb のピーク高度を比較した。

| 離陸 | ピーク真高度 | オーバーシュート（−0.5m） |
|------|------------|------------------------|
| 初回（TAKEOFF #1） | 0.658 m | **+0.158 m** |
| 再離陸（TAKEOFF #2） | 0.654 m | **+0.154 m** |

→ **差は +0.004 m（ほぼ同一）**。SIL では非対称が出ない。

### なぜ SIL で出ないか

- SIL の `noise off` 経路では IMU に誤差がなく、起動校正が**真のバイアス（≈0）を完全に捕捉**する。よって初回も再離陸も ESKF バイアスは正確で同一 → climb 過渡も同一。
- 観測された ~0.16m のオーバーシュートは**両離陸に共通**で、これは**地上ブラインド窓由来の構造的過渡**（ESKF は ToF ハンドオフ 0.15m まで鉛直速度を 0 に保持するため、速度ループが初期に過励起する。detailed_design §3 注2/注4）。バイアス収束とは独立で、初回特有ではない。

### 実機で初回が大きい理由（仮説）

- 実機の起動静止校正は短時間で、**accel-Z バイアス（ba_z）に残差**が残る。飛行#1の間に ESKF がこれを真値へ収束させる。
- **ARM はバイアスをリセットしない**ので、**再離陸#2は収束済み ba_z** を、**初回#1は残差を含む起動校正 ba_z** を使う。
- 初回はブラインド窓（鉛直速度 0 ホールド区間）での鉛直加速度積分が ba_z 残差で狂い、速度ループが余分に加速 → **オーバーシュート増**。再離陸では ba_z が正確ゆえ過渡が小さい。

### noise ON で再現を試みた結果（不調・別問題を発見）

- `--noise n1` で `alt_double_takeoff` を回すと、**ARM・離陸指令は出る**（"Auto-takeoff engaged/complete"）が**真高度が 0.013m のまま＝機体が climb しない**（推定だけ上昇）。N1 振動下で ESKF 鉛直推定が上方へ発散し、速度ループが推力を絞って機体が上がらない。
- これは **C の問い（初回 vs 再離陸）とは別の「N1 ノイズ下の鉛直推定発散」問題**。要フォローアップ（本調査のスコープ外）。

## 3. 推奨対応（修正は実機検証が前提）

SIL で再現・検証できないため、**投機的な制御変更はしない**（CLAUDE.md: 制御系変更は SIL 数値裏付け後）。実機側で：

1. **起動校正の改善**（第一候補）: 静止サンプル数/窓を増やす、または ba_z を重力基準でより正確に推定して初回の残差を縮める。
2. **初回離陸の保守化**（任意）: 初回のみ `takeoff_climb_rate_` を下げる／ブラインド窓の速度ループゲインを下げ、ba_z 残差への感度を落とす。
3. **N1 鉛直発散の別調査**（上記発見）: 振動下の ESKF 鉛直推定の堅牢化（観測 R、ブラインド窓の扱い）。

いずれも**実機ログでの定量確認後**に着手する。

---

<a id="english"></a>

## 1. Overview

### About This Document

Investigation (refactor C) of the pilot report (2026-06-14): "the FIRST takeoff overshoots the 0.5 m target more than a takeoff after one landing — look at the difference between post-boot init and the takeoff reset for the 2nd+ takeoff."

### Conclusion (up front)

- **Not a controller / reset bug.** ARM → `controller Reset` → `onTakeoff` is the SAME path for the first and the re-takeoff.
- Root cause is **ESKF bias convergence**: **ARM does NOT reset the ESKF biases** (only `InflateCov(Attitude)`, §3 note 1). So the **re-takeoff inherits the biases converged during flight #1**, while the **first takeoff uses the boot-calibration biases** (with residual error).
- **SIL has a perfect (zero-error) boot calibration, so it does NOT reproduce the asymmetry** → a fix cannot be SIL-verified; it is a hardware-side item.

## 2. Details

### SIL A/B (`alt_double_takeoff.scn`, noise off / GE off)

One run does first-takeoff → land → re-takeoff and compares the climb peaks.

| Takeoff | Peak true alt | Overshoot (−0.5 m) |
|---------|---------------|--------------------|
| First (#1) | 0.658 m | **+0.158 m** |
| Re-takeoff (#2) | 0.654 m | **+0.154 m** |

→ **Difference +0.004 m (essentially identical).** No asymmetry in SIL.

### Why SIL does not show it

- On the `noise off` path the IMU has no error and the boot calibration captures the true bias (≈0) perfectly, so both takeoffs have identical, accurate ESKF biases → identical climb transient.
- The observed ~0.16 m overshoot is COMMON to both takeoffs — a structural transient from the ground-blind window (the ESKF holds vertical velocity at 0 until the 0.15 m ToF handoff, so the velocity loop over-excites initially; §3 notes 2/4). It is independent of bias convergence and is not first-takeoff-specific.

### Hardware hypothesis for the larger first overshoot

- The hardware static boot calibration is brief and leaves a **residual accel-Z bias (ba_z)**; the ESKF converges it to truth during flight #1.
- Because **ARM does not reset the biases**, **re-takeoff #2 uses the converged ba_z** while **first takeoff #1 uses the residual boot-calibration ba_z**. The blind-window vertical integration is off on the first takeoff → the velocity loop over-accelerates → larger overshoot.

### Noise-ON reproduction attempt (failed; surfaced a separate issue)

- `--noise n1` on `alt_double_takeoff`: ARM and takeoff DO fire ("Auto-takeoff engaged/complete"), but the TRUE altitude stays at 0.013 m — the craft does not climb (only the estimate rises). Under N1 vibration the ESKF vertical estimate diverges upward and the velocity loop throttles back. This is a SEPARATE "N1 vertical-estimate divergence" issue, out of scope here — follow-up needed.

## 3. Recommended action (a fix requires hardware verification)

Since SIL cannot reproduce/verify it, **no speculative control change is made** (CLAUDE.md: control changes need SIL numerical backing). On hardware:

1. **Improve the boot calibration** (first choice): more still samples / a longer window, or a more accurate gravity-referenced ba_z, to shrink the first-takeoff residual.
2. **Make the first takeoff conservative** (optional): only for the first takeoff, lower `takeoff_climb_rate_` or the blind-window velocity-loop gain to reduce sensitivity to the ba_z residual.
3. **Investigate the N1 vertical divergence** (found above): robustify the ESKF vertical estimate under vibration (observation R, blind-window handling).

All to be started only after a quantitative check on real flight logs.
