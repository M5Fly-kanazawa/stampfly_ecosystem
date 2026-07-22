# StampFly シミュレーション方針（Simulation Policy）

> **【本書の位置づけ】** 本書は StampFly Ecosystem におけるシミュレーション方針の唯一の正（Single Source of Truth）である。`simulator/sil/RESET_PLAN.md`・`firmware/vehicle/docs/development_roadmap.md` 等、他文書の記述と食い違って見える場合は**本書が優先**する。食い違いに気づいたら、まず本書を更新すること。
>
> 制定: 2026-07-22。制定理由: SIL 立ち上げ期の規律（「実機データは不要」）と、実機飛行後の Model Fidelity（モデル忠実度。物理モデルが現実にどれだけ合っているかの指標）期の方針が別々の文書に分散し、両者の関係（矛盾ではなくフェーズ移行であること）が文書上追えなくなっていた。本書がこれを統合し、交通整理する。

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

本ドキュメントは、StampFly Ecosystem におけるシミュレーション（設計用の線形モデル・実ログ駆動のオフライン再生・SIL）の位置づけ、実機データの扱い方針、そして SIL のプラント（制御対象の物理モデル）が満たすべき合格基準を定める。

### 対象読者

- SIL・シミュレータを開発・改修する開発者
- 制御パラメータの変更を提案する開発者（本書 §5 の規律に従う）
- 将来のセッション（本書を読めば方針の経緯と現在地がわかるようにする）

### なぜ本書が必要か

`simulator/sil/RESET_PLAN.md` §2 方針1は「実機データは不要」「旧 M7/M8（実機ログの再生・差分診断）はやめる」と定めている。これは vehicle がまだ一度も飛んでいなかった**立ち上げ期の規律**として正しかった——実機データが存在しない段階では、SIL は実機データに依存しない設計でなければ検証手段として成立しないからである。

しかし現在（2026-07-22）は事情が変わった。実機飛行データが蓄積し（`logs/` 配下に約4.5ヶ月分・217個超のログファイル）、`firmware/vehicle/docs/development_roadmap.md` の **Phase 5（モデル校正の閉ループ運用／Model Fidelity）が進行中**である。RESET_PLAN 自身も §3 で「実機データの正しい使いどころは Model Fidelity を上げる場面だけ」と、後追いの精度向上としての実機データ活用を認めている。

つまり方針は「矛盾」しているのではなく「フェーズが移行」しただけである。だが文書が未改訂のままだと、読み手には矛盾に見える。本書がこの交通整理を担う。

## 2. シミュレーションの3層構造

StampFly Ecosystem のシミュレーションは、目的の異なる3層で構成される。

| | プラント物理 | 制御コード | 入力・外乱 |
|---|---|---|---|
| **層1** | 線形低次 $G(s)$ | 伝達関数として扱う | 設計用の想定入力 |
| **層2** | 線形低次 $G(s)$＋飽和等 | Python に移植した制御則 | 実機ログから再構成した実外乱・実指令 |
| **層3** | 非線形6自由度（MuJoCo） | 実ファーム C++ そのもの | シナリオ（`.scn`）、将来は実機ログのリプレイ |

### 層1（設計用線形モデル）

`sf sysid rate-fit` 等で実飛行ログから同定する軸別の低次モデル

$$G(s) = \frac{b\, e^{-Ls}}{s(Ts+1)}$$

を制御系設計・ループ整形の正とする。実績（`analysis/reports/altlog_20260614T201629/REPORT.md`）:

| 軸 | コヒーレンス（同定信頼度） | $L$ [ms] | PM（位相余裕） | GM（ゲイン余裕） |
|---|---|---|---|---|
| roll | 0.65（良） | 14.7 | 59° | 10.5 dB |
| pitch | 0.57（良） | 8.4 | 54° | 12.4 dB |
| yaw | 0.44（低信頼） | 11.0 | 56° | 10.2 dB |

PM 54〜59°・GM 10〜12 dB は飛行が安定している事実と整合する。ヨー軸は反トルク零点を持つ4パラメータモデルで表される（`firmware/vehicle/docs/yaw_axis_model.md`）。交差周波数における位相リード予測 +20.5〜23.3°（パラメータ再測定前後の2値）は、フライト実測 +22〜32° のリードと帯域内で整合している。

### 層2（実ログ駆動オフライン再生）

`analysis/scripts/` 配下の Python 群。層1で同定したモデルと移植した制御則を、実機ログから再構成した実外乱・実指令で駆動する閉ループ再生であり、パラメータ変更の A/B 判定の正とする。実績:

- ヨーκ（抗力/推力比）修正のリプレイ一致: 0.0〜0.7%
- ALT_HOLD 再生誤差: 約8%
- 高度 DOB（外乱オブザーバ）設計: シム予測 −37〜−56% → 実機 −67%（`analysis/scripts/alt_dob_design/README.md`）

### 層3（SIL）

MuJoCo（非線形6自由度の物理エンジン）＋実ファーム C++ を、Code Identity（SIL は本体と同じソースをそのままコンパイルして走らせる）・Param Identity（同じパラメータで走らせる）のもとで走らせる、状態機械・推定器・フェイルセーフを含むシステム全体の検証ベンチである。**新方針: プラントは実機同定値に一致する非線形モデルを目標とする**（合否は §4 のモデル一致ゲートで判定する）。

### 層の関係

層2は層3の簡略版ではなく、「層1モデルを実データ励振で駆動するもの」という独立した位置づけを持つ。層3に (a) 高忠実プラント、(b) 実機ログ入力リプレイ（§6 バックログ #9）が入った後は、層2は層3の高速・軸別の近似版という位置づけに収束していく。層1は制御設計用として恒久的に残る。

## 3. 方針の変遷（立ち上げ期 → Model Fidelity 期）

| 期間 | フェーズ | 実機データの扱い | 根拠文書 |
|---|---|---|---|
| 〜2026-06（初飛行前・SIL立ち上げ期） | 更地化・物理ベース SIL の再構築 | 実機データ不要。物理モデルの真値で機械的に検証（旧 M7/M8 の実機ログ再生・差分診断は廃止） | RESET_PLAN §2 方針1 |
| 2026-06〜（実機飛行後・Model Fidelity 期＝現在） | development_roadmap Phase 3〜5 | 実機ログで層1同定・層2 A/B・層3プラント較正を行う。実機ログの再生・突き合わせは方針違反ではなく Phase 5 の本作業そのもの | development_roadmap Phase 5 |

注: RESET_PLAN 方針2（アルゴリズムの中身に依存せず、実装でなくインターフェースに依存する）は期に依らず有効であり、本書はこれを変更しない。

## 4. モデル一致ゲート（層3の合否判定）

Code Identity のおかげで、実機同定に使ったのと同一の同定パイプライン（`sf sysid rate-fit` 等）を、SIL が生成したログにもそのまま適用できる。

**手順:**

1. SIL 内で `rate-excite` 相当の励振を行う
2. 実機と同一の同定パイプラインを適用する
3. $(b, L, T)$ を抽出する
4. 実機同定値と比較する

**合格基準**（development_roadmap Phase 3 の許容差を流用）:

| 指標 | 許容差 |
|---|---|
| ステップ応答立ち上がり時定数 | ±20% |
| gyro RMS | ±50% |

このゲートを SIL 回帰テスト（退行検出の自動テスト）に組み込み、以後のプラント改修の効果と劣化を毎回数値で判定する。

ゲート合格域に達するまでは「SIL 直接最適化禁止」の原則を維持する。SIL のプラントは理想（むだ時間小・トルク効き1.0）であり、SIL 乱流ベンチを直接最適化すると実機で位相余裕が負になるゲインに収束した教訓がある（`firmware/vehicle/docs/control_theory_overview.md` §5.4: SIL 上で Td=0.08 に最適化したゲインが実機では PM −375° に発散）。

> **初回計測（2026-07-22, `sf sil sysid-gate`）:** むだ時間0の現行プラントは全軸 FAIL — roll b +39.6% / L_total +39.0%、pitch b +109.7% / L_total +19.3%、yaw b −61.4% / L_total +95.7%。遅れの**構造**が実機と逆で、SIL は一次遅れ支配（T≈20ms=motor_tau、L≈1.5ms）、実機はむだ時間支配（L≈11〜16ms、T小）。`--motor-delay 10` で L は 1.5→10.6〜12.2ms と設計どおり動くが、L_total は 27ms 前後へ悪化する — 一致には遅延単独ではなく、バックログ#2（モータ ODE 化）・#3（係数再較正）との同時調整が必要。なお yaw の実機基準値は3パラフィット由来で最も弱い（`analysis/reports/rate_sysid_reference/README.md` の注意参照）。

## 5. 期に依らず変わらない規律

- **制御パラメータ変更は必ず実フライトログを使った数値シミュレーションで裏付ける。** 「Ti を短くすれば改善する」のような定性推測だけで提案しない。シミュレーションの結果、逆効果であれば提案しない（`control_theory_overview.md` §5.5 の鉄則）。
- **公称モデル1点への最適化をしない。** 実機はセッション間でドリフトする（同一ゲインで 5–8 Hz 帯の基準値が2.4〜2.7倍変動した実例がある）。トルク効き $\in[0.4, 0.7]$、むだ時間 $L \in [8, 15]$ ms、会場級外乱 0.2〜1 Hz を**摂動族**として持ち、ゲインの採否は族全体で悪化しないことを条件にする。
- **SIL の原理的限界は SIL では検証できない。** 並行処理の競合（複数タスクが同時に走ることによる競合）や実 WiFi/ESP-NOW の物理層は、SIL の再現性のために本来並行する処理を一本のループにまとめている構造上、原理的に再現できない（RESET_PLAN §11）。実機並行性の検証は別途行う。

## 6. SIL プラント改修バックログ（優先順）

| # | 作業 | 根拠・目標値 | 状態 |
|---|---|---|---|
| 0 | モデル一致ゲートの実装（§4） | すべての改修の物差し。最優先 | **実装済み（2026-07-22）** — `sf sil sysid-gate` |
| 1 | むだ時間の追加 | 現状 SIL 実効遅れ ~5 ms vs 実機 8.4〜14.7 ms。ゲートで SIL の現状 $L$ を実測し、差分を duty→推力経路の輸送遅れとして設定可能にする | **実装済み（2026-07-22, 既定OFF）** — `sf sil scenario --motor-delay` |
| 2 | モータモデルの ODE 化 | `simulator/genesis/motor_model.py` の電気機械 ODE $\dot\omega = \bigl[-(D_m + K_m^2/R_m)\omega - C_Q\omega^2 - Q_f + K_m V/R_m\bigr]/J_{mp}$ を SIL へ移植。実測値 $J_{mp}=1.375\times10^{-8}$ kg·m²、$C_Q=4.10\times10^{-11}$ N·m·s²/rad²、$\omega_{hover}\approx3670$ rad/s、ホバ点実効時定数 $\tau_{eff}\approx17.5$ ms | 未着手 |
| 3 | $C_T$/$C_Q$/thrust_efficiency の3点セット再較正 | 実測 $C_T=6.7\times10^{-9}$ N/(rad/s)²（2026-07-15）への差し替え。`simulator/sil/plant/plant.hpp` に保留 TODO あり——3値は Model Identity（飛行ログ較正）で連動しており**単独差し替え禁止**（ホバー推力整合を壊す） | 未着手 |
| 4 | モータ不感帯・低 duty 非線形 | 実機 ~0.9 Hz リミットサイクルの再現に必要。`analysis/datasets/motor_sweep_20260714/` のベンチデータ（3個体・プロペラ有無2条件）で同定 | 未着手 |
| 5 | 空気抵抗の追加 | 現状 MuJoCo プラントは抗力ゼロ。`sf sysid drag` の実ログ同定値を使用 | 未着手 |
| 6 | フロー品質モデル（N3） | SQUAL（オプティカルフローの表面品質指標）固定100・無ノイズが POS_HOLD 初飛行発散の盲点だった（`firmware/vehicle/docs/poshold_journey.md`: 「Code Identity でも実機で動かない」盲点の実例）。Flow/Mag ノイズは N3 tier として後段に計画済み（`simulator/sil/RESET_PLAN.md` §13） | 未着手 |
| 7 | バッテリサグの $R_{int}$ 実測較正 | 電圧依存推力誤差が高度ウォブルの主因（corr(V, 高度std)=−0.78、`analysis/reports/poshold_3min_battery_wobble_20260627.md`）。サグモデル自体は実装済みで閉ループ emu では既定 ON（2026-06-07, `b8fd27ea`）。残作業は内部抵抗 $R_{int}$（現状値 0.1 Ω は vpython 由来の仮値）の実測較正のみ | 未着手 |
| 8 | N1 振動係数を現行 vehicle ログで再同定 | 現在の軸別係数（`vib_accel_k`/`vib_gyro_k`）は旧機（legacy `firmware/vehicle`）の hover02 ログ由来のシード値 | 未着手 |
| 9 | 実機ログ入力リプレイ（`sf sil replay` 相当） | WireControl（テレメトリの制御入力構造体）50 Hz スティック入力を `.scn` シナリオへ変換し、実ログと同一プロットで比較する。層2→層3 収束の要 | 未着手 |
| 10 | 関連文書の整合維持 | 本書と RESET_PLAN・development_roadmap の食い違いに気づいたら、本書を先に更新する | 継続 |

## 7. 関連文書マップ

| 文書 | 何の正か |
|---|---|
| 本書 | シミュレーション方針（3層構造・フェーズ・モデル一致ゲート・バックログ） |
| `simulator/sil/RESET_PLAN.md` | SIL ベンチの構造・立ち上げ経緯の記録（§2 方針1 は立ち上げ期の規律） |
| `firmware/vehicle/docs/development_roadmap.md` | 開発工程全体（Phase 0〜6） |
| `firmware/vehicle/docs/control_theory_overview.md` | 制御設計の規律・同定の教訓 |
| `firmware/vehicle/docs/noise_and_vibration_model.md` | センサノイズモデル（N0〜N2、N3/N4 計画） |
| `firmware/vehicle/docs/yaw_axis_model.md` | ヨー軸モデル |
| `docs/architecture/stampfly-parameters.md` | 物理パラメータの値と実測履歴 |
| `analysis/scripts/alt_dob_design/README.md` ほか `analysis/reports/` | 層2の実施記録 |

---

<a id="english"></a>

## 1. Overview

### About This Document

This document defines the role of each simulation layer used in the StampFly Ecosystem — the design-oriented linear model, the log-driven offline replay, and the SIL (Software-In-the-Loop) bench — the policy on using real-flight data, and the pass criteria the SIL plant model must satisfy.

### Target Audience

- Developers who build or modify the SIL bench and simulator
- Developers proposing control-parameter changes (who must follow the discipline in §5)
- Future sessions (this document should make the history and current state of the policy traceable)

### Why This Document Is Needed

`simulator/sil/RESET_PLAN.md` §2, Policy 1 states that "real-flight data is not needed" and that the old M7/M8 steps (replaying and diffing against real logs) were dropped. This was correct discipline for the **bring-up phase**, before vehicle had ever flown — with no real-flight data in existence, a SIL that depended on it could not have served as a verification method.

That is no longer the situation as of 2026-07-22. Real-flight data has accumulated (roughly 4.5 months and 217+ log files under `logs/`), and `firmware/vehicle/docs/development_roadmap.md` Phase 5 (the closed-loop model-calibration operation, i.e. Model Fidelity) is now underway. RESET_PLAN itself acknowledges in §3 that the legitimate use of real-flight data is exactly to raise Model Fidelity — an after-the-fact refinement.

In other words, the policy has not "contradicted itself" — it has moved into a new phase. But an unrevised document makes that look like a contradiction. This document performs that reconciliation.

## 2. The Three Simulation Layers

The StampFly Ecosystem's simulation spans three layers with distinct purposes.

| | Plant physics | Control code | Input / disturbance |
|---|---|---|---|
| **Layer 1** | Low-order linear $G(s)$ | Treated as a transfer function | Design-intent inputs |
| **Layer 2** | Low-order linear $G(s)$ + saturation etc. | Control law ported to Python | Real disturbance/commands reconstructed from flight logs |
| **Layer 3** | Nonlinear 6-DOF (MuJoCo) | The real firmware C++ itself | Scenario files (`.scn`); future: real-log replay |

### Layer 1 (design-oriented linear model)

The per-axis low-order model identified from real-flight logs via `sf sysid rate-fit` etc.,

$$G(s) = \frac{b\, e^{-Ls}}{s(Ts+1)}$$

serves as the authority for control-system design and loop shaping. Results (`analysis/reports/altlog_20260614T201629/REPORT.md`):

| Axis | Coherence (ID confidence) | $L$ [ms] | PM (phase margin) | GM (gain margin) |
|---|---|---|---|---|
| roll | 0.65 (good) | 14.7 | 59° | 10.5 dB |
| pitch | 0.57 (good) | 8.4 | 54° | 12.4 dB |
| yaw | 0.44 (low confidence) | 11.0 | 56° | 10.2 dB |

A PM of 54–59° and GM of 10–12 dB is consistent with the fact that this log came from stable flight. The yaw axis follows a 4-parameter model with a reaction-torque zero (`firmware/vehicle/docs/yaw_axis_model.md`). The predicted phase lead at the crossover frequency, +20.5–23.3° (two values from before/after a parameter re-measurement), falls within the flight-measured lead of +22–32°.

### Layer 2 (log-driven offline replay)

The Python scripts under `analysis/scripts/`. This closed-loop replay drives the Layer-1-identified model and the ported control law with real disturbance/commands reconstructed from flight logs, and serves as the authority for A/B judgment of parameter changes. Results:

- Yaw κ (drag/thrust ratio) fix replay agreement: 0.0–0.7%
- ALT_HOLD replay error: about 8%
- Altitude DOB (disturbance observer) design: simulated prediction −37 to −56% → real flight −67% (`analysis/scripts/alt_dob_design/README.md`)

### Layer 3 (SIL)

MuJoCo (a nonlinear 6-DOF physics engine) plus the real firmware C++, run under Code Identity (the SIL compiles and runs the exact same source as the real firmware, unmodified) and Param Identity (both run with the same parameters). It is the verification bench for the whole system, including the state machine, estimator, and failsafes. **New policy: the plant should target a nonlinear model that matches real-hardware identification values** (pass/fail is judged by the model-match gate in §4).

### Relationship Between the Layers

Layer 2 is not a simplified version of Layer 3 — it stands on its own as "the Layer-1 model driven by real-data excitation." Once Layer 3 gains (a) a high-fidelity plant and (b) real-log input replay (backlog #9 in §6), Layer 2 will converge toward being a fast, per-axis approximation of Layer 3. Layer 1 remains permanently in place for control design.

## 3. Evolution of the Policy (Bring-Up Phase → Model Fidelity Phase)

| Period | Phase | Treatment of real-flight data | Governing document |
|---|---|---|---|
| Until 2026-06 (pre-first-flight, SIL bring-up) | Clean-slate rebuild of the physics-based SIL | No real-flight data needed. Verification is done mechanically against the true physical model (the old M7/M8 real-log replay/diff steps were dropped) | RESET_PLAN §2 Policy 1 |
| 2026-06 onward (post-first-flight, Model Fidelity — current) | development_roadmap Phase 3–5 | Real logs are used for Layer-1 identification, Layer-2 A/B testing, and Layer-3 plant calibration. Replaying and comparing against real logs is not a policy violation — it is exactly the work of Phase 5 | development_roadmap Phase 5 |

Note: RESET_PLAN Policy 2 (independence from algorithm internals — depend on the interface, not the implementation) remains in effect regardless of phase; this document does not change it.

## 4. The Model-Match Gate (Layer-3 Pass/Fail)

Thanks to Code Identity, the same identification pipeline used on real-hardware logs (`sf sysid rate-fit`, etc.) can be applied directly to logs generated by the SIL.

**Procedure:**

1. Run a `rate-excite`-equivalent excitation inside the SIL
2. Apply the same identification pipeline used on real hardware
3. Extract $(b, L, T)$
4. Compare against the real-hardware identified values

**Pass criteria** (reusing the tolerances from development_roadmap Phase 3):

| Metric | Tolerance |
|---|---|
| Step-response rise time constant | ±20% |
| Gyro RMS | ±50% |

This gate is wired into the SIL regression test suite (automated tests that detect regressions), so every subsequent plant modification is judged numerically, both for improvement and for regression, every time.

Until the gate is passed, the principle of "no direct optimization on the SIL" remains in force. The SIL's plant is currently ideal (small dead time, unity torque effectiveness), and directly optimizing against a SIL turbulence bench previously converged on a gain with negative phase margin on real hardware (`firmware/vehicle/docs/control_theory_overview.md` §5.4: a gain optimized to Td=0.08 on the SIL diverged to PM −375° on real hardware).

> **First measurement (2026-07-22, `sf sil sysid-gate`):** the current zero-dead-time plant FAILs on all axes — roll b +39.6% / L_total +39.0%, pitch b +109.7% / L_total +19.3%, yaw b −61.4% / L_total +95.7%. The lag **structure** is inverted vs. real hardware: the SIL is first-order-lag dominated (T≈20 ms = motor_tau, L≈1.5 ms) while the real machine is dead-time dominated (L≈11–16 ms, small T). With `--motor-delay 10`, L moves 1.5→10.6–12.2 ms exactly as designed, but L_total worsens to ~27 ms — matching requires the joint adjustment with backlog #2 (motor ODE) and #3 (coefficient recalibration), not delay alone. The yaw reference is the weakest of the three axes (3-parameter fit; see the note in `analysis/reports/rate_sysid_reference/README.md`).

## 5. Discipline That Does Not Change With Phase

- **Any control-parameter change must be backed by a numerical simulation using real flight logs.** Do not propose changes based on qualitative reasoning alone (e.g., "shortening Ti should help"). If simulation shows the change backfires, do not propose it (the rule in `control_theory_overview.md` §5.5).
- **Do not optimize for a single nominal model.** Real hardware drifts between sessions (the same gain produced a 2.4–2.7x change in the 5–8 Hz band reference value across sessions). Carry torque effectiveness $\in[0.4, 0.7]$, dead time $L \in [8, 15]$ ms, and venue-grade disturbance at 0.2–1 Hz as a **perturbation family**, and accept a gain only if it does not degrade across the whole family.
- **The SIL's inherent limitations cannot be verified within the SIL.** Concurrency conflicts (races between simultaneously running tasks) and the real WiFi/ESP-NOW physical layer cannot, in principle, be reproduced, because the SIL collapses what is normally concurrent processing into a single loop for reproducibility (RESET_PLAN §11). Real-hardware concurrency verification must be done separately.

## 6. SIL Plant Improvement Backlog (Priority Order)

| # | Item | Rationale / target value | Status |
|---|---|---|---|
| 0 | Implement the model-match gate (§4) | The yardstick for every other item. Highest priority | **Implemented (2026-07-22)** — `sf sil sysid-gate` |
| 1 | Add transport delay | Current SIL effective lag is ~5 ms vs. 8.4–14.7 ms on real hardware. Once the gate is in place, measure the SIL's current $L$ and set the difference as transport delay in the duty→thrust path | **Implemented (2026-07-22, default OFF)** — `sf sil scenario --motor-delay` |
| 2 | Convert the motor model to an ODE | Port the electromechanical ODE from `simulator/genesis/motor_model.py`: $\dot\omega = \bigl[-(D_m + K_m^2/R_m)\omega - C_Q\omega^2 - Q_f + K_m V/R_m\bigr]/J_{mp}$. Measured values: $J_{mp}=1.375\times10^{-8}$ kg·m², $C_Q=4.10\times10^{-11}$ N·m·s²/rad², $\omega_{hover}\approx3670$ rad/s, hover-point effective time constant $\tau_{eff}\approx17.5$ ms | Not started |
| 3 | Recalibrate the $C_T$/$C_Q$/thrust_efficiency triple | Replace with the measured $C_T=6.7\times10^{-9}$ N/(rad/s)² (2026-07-15). A pending TODO exists in `simulator/sil/plant/plant.hpp` — the three values are coupled via Model Identity (flight-log calibration), so **do not swap any one alone** (it breaks hover-thrust consistency) | Not started |
| 4 | Motor dead-band / low-duty nonlinearity | Needed to reproduce the ~0.9 Hz limit cycle seen on real hardware. Identify from the bench data in `analysis/datasets/motor_sweep_20260714/` (3 airframes, props on/off) | Not started |
| 5 | Add aerodynamic drag | The current MuJoCo plant has zero drag. Use the real-log-identified values from `sf sysid drag` | Not started |
| 6 | Flow-quality model (N3) | A fixed SQUAL (optical-flow surface-quality metric) of 100 with no noise was the blind spot behind the first POS_HOLD real-flight divergence (`firmware/vehicle/docs/poshold_journey.md`: a concrete case of "passes Code-Identity SIL yet fails on hardware"). Flow/mag noise is already planned as the N3 tier for a later stage (`simulator/sil/RESET_PLAN.md` §13) | Not started |
| 7 | Calibrate battery-sag $R_{int}$ from measurement | Voltage-dependent thrust error is the leading cause of altitude wobble (corr(V, altitude std) = −0.78, `analysis/reports/poshold_3min_battery_wobble_20260627.md`). The sag model itself is already implemented and already defaults ON in the closed-loop emulator (since 2026-06-07, `b8fd27ea`). The remaining work is only to calibrate the internal resistance $R_{int}$ from measurement (the current 0.1 Ω is a placeholder carried over from the vpython model) | Not started |
| 8 | Re-identify the N1 vibration coefficients on current-vehicle logs | The current per-axis coefficients (`vib_accel_k`/`vib_gyro_k`) are seed values taken from the legacy `firmware/vehicle` hover02 log | Not started |
| 9 | Real-log input replay (`sf sil replay`-equivalent) | Convert WireControl (the telemetry control-input struct) 50 Hz stick input into `.scn` scenarios, and compare against the real log on the same plots. The key step for Layer-2 → Layer-3 convergence | Not started |
| 10 | Keep related documents consistent | If a discrepancy is noticed between this document and RESET_PLAN / development_roadmap, update this document first | Ongoing |

## 7. Related Document Map

| Document | Authority for |
|---|---|
| This document | Simulation policy (three-layer structure, phases, model-match gate, backlog) |
| `simulator/sil/RESET_PLAN.md` | SIL bench structure and the bring-up history (§2 Policy 1 is bring-up-phase discipline) |
| `firmware/vehicle/docs/development_roadmap.md` | The overall development process (Phase 0–6) |
| `firmware/vehicle/docs/control_theory_overview.md` | Control-design discipline and identification lessons |
| `firmware/vehicle/docs/noise_and_vibration_model.md` | Sensor noise model (N0–N2, N3/N4 planned) |
| `firmware/vehicle/docs/yaw_axis_model.md` | The yaw-axis model |
| `docs/architecture/stampfly-parameters.md` | Physical-parameter values and measurement history |
| `analysis/scripts/alt_dob_design/README.md` and other `analysis/reports/` entries | Layer-2 execution records |
