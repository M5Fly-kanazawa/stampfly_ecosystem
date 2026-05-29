# SIL による制御設計検証 — 姿勢制御（M5/M6）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

`simulator/sil/` の統合SIL（診断計器）を用いて、vehicle_new の姿勢制御設計を PC 上で定量検証した結果を記録する。これは ecosystem の3分離環境（開発 `firmware/` ／ SIL・可視化 `simulator/` ／ 制御設計 `control/`）における **`control/` = 設計根拠の受け皿** としての最初の成果物である。

検証は決定論的（固定シード）で、本体C++コア（ESKF/PID）を参照コンパイルして実行しているため、ここで得た知見は実機と同一コードに対するものである（Code Identity）。

### 検証ツール

`simulator/sil/integrated_sil.cpp`（物理 + ESKF + PID + 状態遷移 + failsafe）。評価指標・診断メトリクスを飛行後に出力する。

## 2. 評価指標

FLYING 窓で集計する制御設計の評価指標：

| 指標 | 内容 |
|------|------|
| roll/pitch RMS・max [deg] | 姿勢追従誤差（トリム飛行のため目標0からの偏差） |
| altitude RMS [mm] | 目標高度(0.5m)からの誤差 |
| norm_ratio / innov_norm / P_att | ESKF 内部診断（ノルムゲート・イノベーション・姿勢共分散） |

## 3. 診断の梯子 — 姿勢誤差の要因分離

`--feedback eskf`（ESKF推定姿勢の閉ループ）で、ノイズ層を段階的にON/OFFして姿勢誤差の要因を切り分けた。

![Diagnostic ladder](../../simulator/sil/docs/assets/fig4_diagnostic_ladder.png)

| ノイズ段 | roll RMS | 結論 |
|---------|---------|------|
| N0（ノイズなし） | 0.000° | ESKF閉ループは構造的に健全（線形化バイアスは無外乱で出ない） |
| N1（白色＋バイアス） | 0.013° | 白色ノイズはほぼ無影響 |
| N2（フル振動） | 3.321° | **スロットル結合振動が姿勢誤差の主因** |

→ 層を ON/OFF するだけで「ESKF構造の問題か、ノイズの問題か」を定量的に切り分けられた。

## 4. 制御パラメータの A/B 検証

主因と判明した振動に対し、ESKF系LPF α と加速度観測ノイズ R を SIL で A/B 検証した（`--feedback eskf --noise-stage N2`）。

![A/B sweep](../../simulator/sil/docs/assets/fig5_ab_sweep.png)

| パラメータ | 範囲 | roll RMS | 効果 |
|-----------|------|---------|------|
| LPF α | 0.32 → 0.05 | 3.32 → 2.64° | 強フィルタで ~20% 改善 |
| ESKF R | 0.3 → 10 | 3.32 → 2.93° | ~12% 改善、R≥3で頭打ち |

gust 外乱下でも α=0.05 で roll RMS 3.43→2.74°、roll max 4.43→4.00°（この外乱強度では位相遅れの副作用は顕在化せず）。

## 5. 知見と留保

- 振動起因の姿勢誤差は LPF 強化や R 増加で **限定的（~20%）にしか改善せず、根治しない**。機械的振動対策（プロペラバランス等）やノイズモデルの精緻化が必要な可能性。これは旧機の実機知見（「元ゲインが最良・ノッチNG」）とも整合する。
- 本SILは軽い外乱・無マニューバのため、**強フィルタの位相遅れの副作用は十分に顕在化していない**。強い外乱や高速マニューバでの再評価が必要。
- **重要：SILで得た改善が実機に転移するかは Model Fidelity（実機ログとの照合, ロードマップ M8）で検証してから判断する。** SIL設計検証 → Model Fidelity → 実機適用 の順を厳守する（安易な実機適用はしない）。

## 6. 再現手順

```bash
cd simulator/sil && make integrated_sil
# 診断梯子
for s in N0 N1 N2; do ./integrated_sil --feedback eskf --noise-stage $s 2>&1 >/dev/null | grep "roll :"; done
# A/B（LPF / ESKF R）
./integrated_sil --feedback eskf --noise-stage N2 --lpf-alpha 0.05 2>&1 >/dev/null | grep "roll :"
./integrated_sil --feedback eskf --noise-stage N2 --accel-noise 3.0 2>&1 >/dev/null | grep "roll :"
# グラフ再生成
python3 plot_sil_results.py   # fig4(診断梯子), fig5(A/Bスイープ) を含む
```

---

<a id="english"></a>

## 1. Overview

This document records the PC-side quantitative validation of vehicle_new's attitude-control design using the integrated SIL (`simulator/sil/`). It is the first artifact of `control/` acting as **the home for design rationale** in the ecosystem's three-environment split (firmware / simulator / control). Validation is deterministic and runs the unmodified firmware ESKF/PID core by reference compilation (Code Identity), so the findings apply to the same code that runs on hardware.

## 2. Evaluation Metrics

Over the FLYING window: roll/pitch RMS & max [deg], altitude RMS [mm], plus ESKF internal diagnostics (norm gate ratio / innovation / attitude covariance).

## 3. Diagnostic Ladder — Isolating the Cause of Attitude Error

With `--feedback eskf`, toggling the noise stage isolates the cause (see Figure in the Japanese section):

| Noise stage | roll RMS | Conclusion |
|-------------|----------|------------|
| N0 (none) | 0.000° | ESKF closed loop is structurally sound |
| N1 (white+bias) | 0.013° | White noise negligible |
| N2 (full vibration) | 3.321° | **Throttle-coupled vibration is the dominant cause** |

## 4. Control-Parameter A/B

| Parameter | Range | roll RMS | Effect |
|-----------|-------|----------|--------|
| LPF α | 0.32 → 0.05 | 3.32 → 2.64° | ~20% better with stronger filter |
| ESKF R | 0.3 → 10 | 3.32 → 2.93° | ~12% better, saturates at R≥3 |

## 5. Findings and Caveats

- Vibration-induced attitude error improves only modestly (~20%) with LPF/R tuning; it is not eliminated. Consistent with the legacy-hardware finding "stock gains best / notch NG."
- This SIL uses mild gusts and no aggressive maneuvers, so the **phase-lag side effect of stronger filtering is not yet exposed**; re-evaluate under stronger disturbances.
- **Whether SIL improvements transfer to hardware must be judged after Model Fidelity validation (roadmap M8).** Order: SIL design validation → Model Fidelity → hardware. No premature hardware application.

## 6. Reproduction

See the command block in the Japanese section (§6).
