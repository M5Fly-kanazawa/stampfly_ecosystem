# 推進系モデル刷新の計画（feat/propulsion-model-v2）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

状態: **計画中**。作成 2026-11-03、最終更新 2026-11-03。

## 1. 概要

### この文書について

推進系のモデルを作り直す作業を、新しいセッションで始めるときに最初に読む文書。何を、どの順で、何を確かめてから次へ進むかを書く。根拠の数値と決定の理由は基礎資料 `analysis/notes/propulsion_model_20261103.md`（以下「レポート」）にあり、ここでは繰り返さない。

### 方針（2026-11-03、実験者の決定）

2026-11-02〜03 の実験結果を受けて、推進系のモデルを刷新する。小手先の修正（換算だけを替える、ゲインを一律に縮める）はしない。モデル → SILS のプラント → ファームの推力 → Duty の換算 → 制御設計 → シナリオの較正 の順に、全面的に作り直す。

| 項目 | 内容 |
|------|------|
| ブランチ | `feat/propulsion-model-v2`（main から切る） |
| 基礎資料 | レポート（日英）。図入りの HTML 版はレポート冒頭の状態の行にリンクあり |
| ベンチの素データ | 隣のフォルダの `stampfly-motor-autotest/data/2026-11-02_steps.csv`（1 段 1 行の表）と `data/2026-10-15/` |
| 飛行ログ | `logs/stampfly_udp_*.jsonl` の 14 本（git の追跡外。一覧はレポート 6 章） |

## 2. 決めたこと

2026-11-03 に実験者が決めた。詳細はレポート 10 章「決めたこと」。

| 項目 | 決定 |
|------|------|
| ファームが換算に使う電圧 | オンの間の電圧 $V_\text{on} = V_\text{CLI} - \Delta V$ を物理の形で推定する。$\Delta V = R_\text{path} I + R_\text{src} I (1-D)/D$ |
| 推力の式 | べき乗 $F = F_\text{ref}(\omega/\omega_\text{ref})^n$、$\omega_\text{ref}$ = 3500 rad/s、$F_\text{ref}$ = 0.0916 N、$n$ = 2.13。全機は 1 モータの 4 倍。SILS の補正係数 $\eta$ は使わない |
| 一定の電圧降下 $V_b$ | 式に項を置き、はじめは 0。確認の後に値を入れ、$R_\text{out}$ の表を同じデータから組で作り直す |

## 3. 最初の段階

コードや基準ファイルを変える前に、次の 1〜5 の作業計画を実験者に見せて了承を得る。

| 順 | 作業 | 次へ進む前に確かめること |
|----|------|------------------------|
| 1 | main からブランチ `feat/propulsion-model-v2` を切る | — |
| 2 | 物理パラメータの基準ファイル `control/models/stampfly_physical.yaml` に、2 章の式と定数を入れる | `sf params check` が通る |
| 3 | $V_b$ の机上確認: 午後・夜の運転の $V_{DD}$ を記録電圧で補正し直し（レポート 9 章）、$V_b$ の値を決める。$R_\text{out}$ の表を $V_\text{on}$ で作り直す | 4 つの負荷で $V_b$ がそろうか。表の当てはめの残差 |
| 4 | 実機で `powf` 1 万回と、換算全体の処理時間を `esp_timer` で測る小さな試験を用意する（書き込みと実行は実験者） | 換算全体が 2.5 ms の制御周期に対して十分小さい（見込みは 1 % 未満） |
| 5 | 電池の開回路電圧と内部抵抗を、11/02 の長時間放電のデータから求める（SILS 方針 `docs/architecture/simulation-policy.md` のバックログ #19） | SILS の電池モデルに入れる値と、その範囲 |

## 4. その後の段階

レポート 10 章「ブランチの進め方の案」の 2〜6 に従う。各段階で確かめることも同じ節にある。

1. SILS のプラントを新しいモデルに替える（Genesis・VPython も同じ基準へ）
2. ファームの推力 → Duty の換算を、同じモデルの逆にする
3. 姿勢・高度の制御を新しいモデルで設計し直す（換算を正確にするとループの効きが 1.3〜2.0 倍になる）
4. シナリオの較正と基準を見直し、再確認試験（変更で既存の動作が壊れていないかを自動で確かめる試験）とモデル一致の判定を通す
5. ベンチと実機で確かめる（レポート 10 章「実機に入れるときの安全」の手順で）

## 5. 後で行うもの

| 項目 | 内容 |
|------|------|
| PWM の位相ずらし | 4 つの PWM のオンのタイミングをずらして $\Delta V$ を減らす。試しとして行う（ファームの切り替え機能とベンチの試験が要る） |
| ベンチの追加 | 別の電池で 4 モータ同時の運転を 1〜2 本、軸を止めた $V_b$ の直接の測定、Duty 0.85〜0.95、4 モータの回転数（`--audio` か各ドレイン）、プロペラの下の物の有無の比較 |
| 記録の運用 | ベンチでは毎回、室温・既知のおもり・機体の重さ・プローブの位置の写真を残す。飛行ログごとにファームのコミットを記録する |

## 6. 守ること

- `firmware/vehicle/` を変える前に、CLAUDE.md が挙げる 6 文書を読み、アーキテクチャ不変条件（INV）に照合する。
- 制御パラメータの変更は、飛行ログを使ったシミュレーションで効果を数値で確かめてから提案する。
- 換算を変えたファームを実機に入れたら、`param reset` → `param save` で古い学習値（`hover.thrust_corr` など）を消す。
- 持ち越しの未解決: SILS で換算だけを替えた試験（F1）のホバー高度が 4〜6 cm 低い原因、stab_flight の悪化。

---

<a id="english"></a>

# Propulsion Model Renewal Plan (feat/propulsion-model-v2)

Status: **Planned**. Created 2026-11-03, last updated 2026-11-03.

## 1. Overview

### About This Document

Read this first when starting the propulsion-model renewal in a new session. It states what to do, in which order, and what to check before moving on. The numbers and the reasons for the decisions are in the reference report `analysis/notes/propulsion_model_20261103.md` (below, "the report") and are not repeated here.

### Policy (2026-11-03, experimenter's decision)

Following the experiments of 2026-11-02 to 11-03, renew the propulsion model. No patch-style fixes (replacing only the conversion, shrinking gains uniformly). Rebuild everything in this order: model → SILS plant → firmware thrust-to-duty conversion → control design → scenario calibration.

| Item | Content |
|------|---------|
| Branch | `feat/propulsion-model-v2` (from main) |
| Reference | The report (Japanese and English). The HTML version with charts is linked in the report's status line |
| Bench raw data | `stampfly-motor-autotest/data/2026-11-02_steps.csv` in the sibling folder (one row per step) and `data/2026-10-15/` |
| Flight logs | The 14 files `logs/stampfly_udp_*.jsonl` (not tracked by git; listed in chapter 6 of the report) |

## 2. Decisions

Decided by the experimenter on 2026-11-03. Details in chapter 10 of the report ("Decided").

| Item | Decision |
|------|----------|
| Voltage the firmware uses for the conversion | Estimate the on-time voltage $V_\text{on} = V_\text{CLI} - \Delta V$ with the physical form $\Delta V = R_\text{path} I + R_\text{src} I (1-D)/D$ |
| Thrust formula | Power law $F = F_\text{ref}(\omega/\omega_\text{ref})^n$, $\omega_\text{ref}$ = 3500 rad/s, $F_\text{ref}$ = 0.0916 N, $n$ = 2.13. The whole vehicle is 4 times one motor. The SILS correction factor $\eta$ is not used |
| Constant voltage drop $V_b$ | Keep the term in the formula, starting at 0. Set the value after the check and remake the $R_\text{out}$ table as a pair from the same data |

## 3. First Stage

Before changing code or the reference file, show the experimenter the work plan for steps 1 to 5 below and get agreement.

| Step | Work | Check before moving on |
|------|------|------------------------|
| 1 | Create the branch `feat/propulsion-model-v2` from main | — |
| 2 | Put the formulas and constants of chapter 2 into the physical-parameter reference file `control/models/stampfly_physical.yaml` | `sf params check` passes |
| 3 | Desk check of $V_b$: correct the afternoon and night $V_{DD}$ with the logged voltage (chapter 9 of the report) and set the value of $V_b$. Remake the $R_\text{out}$ table with $V_\text{on}$ | Whether $V_b$ agrees across the four loads; the fit residual of the table |
| 4 | Prepare a small test that measures 10 000 `powf` calls and the whole conversion with `esp_timer` on the real vehicle (the experimenter flashes and runs it) | The whole conversion is small enough against the 2.5 ms control period (expected below 1 %) |
| 5 | Determine the battery's open-circuit voltage and internal resistance from the 11/02 long-discharge data (backlog #19 of the SILS policy `docs/architecture/simulation-policy.md`) | The values for the SILS battery model and their range |

## 4. Later Stages

Follow steps 2 to 6 of "Proposed course of the branch" in chapter 10 of the report; the checks for each stage are in the same section.

1. Replace the SILS plant with the new model (Genesis and VPython to the same reference)
2. Make the firmware thrust-to-duty conversion the inverse of the same model
3. Redesign the attitude and altitude control with the new model (an exact conversion raises the loop gain 1.3 to 2.0 times)
4. Review scenario calibration and references; pass the regression tests and the model-match check
5. Check on the bench and the real vehicle (following "Safety when putting it on the real vehicle" in chapter 10 of the report)

## 5. Later Work

| Item | Content |
|------|---------|
| PWM phase shifting | Shift the on-times of the four PWMs to reduce $\Delta V$. To be tried (needs a firmware switch and a bench test) |
| More bench runs | Four motors together with one or two other batteries, a direct $V_b$ measurement with the shaft held, Duty 0.85 to 0.95, rotor speed of all four motors (`--audio` or each drain), with and without objects below the propeller |
| Record keeping | On the bench, record room temperature, a known weight, the vehicle weight and a photo of the probe positions every time. Record the firmware commit for each flight log |

## 6. Rules to Keep

- Before changing `firmware/vehicle/`, read the six documents listed in CLAUDE.md and check against the architecture invariants (INV).
- Propose control-parameter changes only after confirming their effect numerically in a simulation using flight logs.
- After flashing firmware with a changed conversion, clear old learned values (such as `hover.thrust_corr`) with `param reset` → `param save`.
- Open issues carried over: why the hover altitude in the SILS test that replaced only the conversion (F1) is 4 to 6 cm low, and the degradation of stab_flight.
