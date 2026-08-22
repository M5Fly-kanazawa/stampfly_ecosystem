# StampFly シミュレーション方針（Simulation Policy）

> **【本書の位置づけ】** 本書は StampFly Ecosystem におけるシミュレーション方針の唯一の正（Single Source of Truth）である。`simulator/sils/RESET_PLAN.md`・`firmware/vehicle/docs/development_roadmap.md` 等、他文書の記述と食い違って見える場合は**本書が優先**する。食い違いに気づいたら、まず本書を更新すること。
>
> 制定: 2026-07-22。制定理由: SILS 立ち上げ期の規律（「実機データは不要」）と、実機飛行後の Model Fidelity（モデル忠実度。物理モデルが現実にどれだけ合っているかの指標）期の方針が別々の文書に分散し、両者の関係（矛盾ではなくフェーズ移行であること）が文書上追えなくなっていた。本書がこれを統合し、交通整理する。

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

本ドキュメントは、StampFly Ecosystem におけるシミュレーション（設計用の線形モデル・実ログ駆動のオフライン再生・SILS）の位置づけ、実機データの扱い方針、そして SILS のプラント（制御対象の物理モデル）が満たすべき合格基準を定める。

### 対象読者

- SILS・シミュレータを開発・改修する開発者
- 制御パラメータの変更を提案する開発者（本書 §5 の規律に従う）
- 将来のセッション（本書を読めば方針の経緯と現在地がわかるようにする）

### なぜ本書が必要か

`simulator/sils/RESET_PLAN.md` §2 方針1は「実機データは不要」「旧 M7/M8（実機ログの再生・差分診断）はやめる」と定めている。これは vehicle がまだ一度も飛んでいなかった**立ち上げ期の規律**として正しかった——実機データが存在しない段階では、SILS は実機データに依存しない設計でなければ検証手段として成立しないからである。

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

### 層3（SILS）

MuJoCo（非線形6自由度の物理エンジン）＋実ファーム C++ を、Code Identity（SILS は本体と同じソースをそのままコンパイルして走らせる）・Param Identity（同じパラメータで走らせる）のもとで走らせる、状態機械・推定器・フェイルセーフを含むシステム全体の検証ベンチである。**新方針: プラントは実機同定値に一致する非線形モデルを目標とする**（合否は §4 のモデル一致ゲートで判定する）。

### 層の関係

層2は層3の簡略版ではなく、「層1モデルを実データ励振で駆動するもの」という独立した位置づけを持つ。層3に (a) 高忠実プラント、(b) 実機ログ入力リプレイ（§6 バックログ #9）が入った後は、層2は層3の高速・軸別の近似版という位置づけに収束していく。層1は制御設計用として恒久的に残る。

## 3. 方針の変遷（立ち上げ期 → Model Fidelity 期）

| 期間 | フェーズ | 実機データの扱い | 根拠文書 |
|---|---|---|---|
| 〜2026-06（初飛行前・SILS立ち上げ期） | 更地化・物理ベース SILS の再構築 | 実機データ不要。物理モデルの真値で機械的に検証（旧 M7/M8 の実機ログ再生・差分診断は廃止） | RESET_PLAN §2 方針1 |
| 2026-06〜（実機飛行後・Model Fidelity 期＝現在） | development_roadmap Phase 3〜5 | 実機ログで層1同定・層2 A/B・層3プラント較正を行う。実機ログの再生・突き合わせは方針違反ではなく Phase 5 の本作業そのもの | development_roadmap Phase 5 |

注: RESET_PLAN 方針2（アルゴリズムの中身に依存せず、実装でなくインターフェースに依存する）は期に依らず有効であり、本書はこれを変更しない。

## 4. モデル一致ゲート（層3の合否判定）

Code Identity のおかげで、実機同定に使ったのと同一の同定パイプライン（`sf sysid rate-fit` 等）を、SILS が生成したログにもそのまま適用できる。

**手順:**

1. SILS 内で `rate-excite` 相当の励振を行う
2. 実機と同一の同定パイプラインを適用する
3. $(b, L, T)$ を抽出する
4. 実機同定値と比較する

**合格基準**（development_roadmap Phase 3 の許容差を流用）:

| 指標 | 許容差 |
|---|---|
| ステップ応答立ち上がり時定数 | ±20% |
| gyro RMS | ±50% |

このゲートを SILS 回帰テスト（退行検出の自動テスト）に組み込み、以後のプラント改修の効果と劣化を毎回数値で判定する。

ゲート合否は軸ごとに独立して判定する。**roll/pitch は 2026-07-26 計測（下記）で合格域に達した（yaw は未達）**。ゲイン検証に SILS を使う信頼性は roll/pitch 軸については向上したが、**全軸が合格に達するまでは**、§5 の摂動族（トルク効き $\in[0.4,0.7]$・むだ時間 $L\in[8,15]$ ms・会場級外乱 0.2〜1 Hz）による検証を SILS 検証と併用し、「SILS 単独最適化禁止」の原則を維持する。SILS 乱流ベンチを直接最適化すると実機で位相余裕が負になるゲインに収束した教訓がある（`firmware/vehicle/docs/control_theory_overview.md` §5.4: SILS 上で Td=0.08 に最適化したゲインが実機では PM −375° に発散）。

> **初回計測（2026-07-22, `sf sils sysid-gate`、むだ時間0・静的モータ曲線の旧プラント）:** 全軸 FAIL — roll b +39.6% / L_total +39.0%、pitch b +109.7% / L_total +19.3%、yaw b −61.4% / L_total +95.7%。遅れの**構造**が実機と逆で、SILS は一次遅れ支配（T≈20ms=motor_tau、L≈1.5ms）、実機はむだ時間支配（L≈11〜16ms、T小）。`--motor-delay 10` で L は 1.5→10.6〜12.2ms と設計どおり動くが、L_total は 27ms 前後へ悪化する — 一致には遅延単独ではなく、バックログ#2（モータ ODE 化）・#3（係数再較正）との同時調整が必要。なお yaw の実機基準値は3パラフィット由来で最も弱い（`analysis/reports/rate_sysid_reference/README.md` の注意参照）。
>
> **第2回計測（2026-07-26 計測, ODE プラント, `sf sils sysid-gate`）:** roll は b +27.2% / L_total +12.7% で **PASS**、pitch は b +26.6% / −1.5% で **PASS**（コヒーレンス 0.97〜0.99、良好）。yaw は b −18.2% は許容域内だが L_total が −100%（識別が退化）で **FAIL** — 実装した反トルク零点（$\tau_z\approx45.7$ ms、制御帯域内で 3.5 Hz のリード）を、現行の3パラメータ $(b,L,T)$ フィットでは表現できない構造的な問題。実機側の基準値自体も3パラフィット・コヒーレンス0.44の弱い基準である点に注意（バックログ#11 で対処予定）。

## 5. 期に依らず変わらない規律

- **制御パラメータ変更は必ず実フライトログを使った数値シミュレーションで裏付ける。** 「Ti を短くすれば改善する」のような定性推測だけで提案しない。シミュレーションの結果、逆効果であれば提案しない（`control_theory_overview.md` §5.5 の鉄則）。
- **公称モデル1点への最適化をしない。** 実機はセッション間でドリフトする（同一ゲインで 5–8 Hz 帯の基準値が2.4〜2.7倍変動した実例がある）。トルク効き $\in[0.4, 0.7]$、むだ時間 $L \in [8, 15]$ ms、会場級外乱 0.2〜1 Hz を**摂動族**として持ち、ゲインの採否は族全体で悪化しないことを条件にする。
- **SILS の原理的限界は SILS では検証できない。** 並行処理の競合（複数タスクが同時に走ることによる競合）や実 WiFi/ESP-NOW の物理層は、SILS の再現性のために本来並行する処理を一本のループにまとめている構造上、原理的に再現できない（RESET_PLAN §11）。実機並行性の検証は別途行う。

## 6. SILS プラント改修バックログ（優先順）

| # | 作業 | 根拠・目標値 | 状態 |
|---|---|---|---|
| 0 | モデル一致ゲートの実装（§4） | すべての改修の物差し。最優先 | **実装済み（2026-07-22）** — `sf sils sysid-gate` |
| 1 | むだ時間の追加 | 現状 SILS 実効遅れ ~5 ms vs 実機 8.4〜14.7 ms。ゲートで SILS の現状 $L$ を実測し、差分を duty→推力経路の輸送遅れとして設定可能にする | **実装済み（2026-07-22, 既定OFF）** — `sf sils scenario --motor-delay`。**ODE化後は追加遅延不要と判明（重畳するとL_total悪化、2026-07-26計測）。既定OFF維持** |
| 2 | モータモデルの ODE 化 | `simulator/genesis/motor_model.py` の電気機械 ODE $\dot\omega = \bigl[-(D_m + K_m^2/R_m)\omega - C_Q\omega^2 - Q_f + K_m V/R_m\bigr]/J_{mp}$ を SILS へ移植。実測値 $J_{mp}=1.375\times10^{-8}$ kg·m²、$C_Q=4.10\times10^{-11}$ N·m·s²/rad²、$\omega_{hover}\approx3670$ rad/s、ホバ点実効時定数 $\tau_{eff}\approx17.5$ ms | **実装済み（2026-07-26）** — 実測ファミリ（$C_Q=4.10\times10^{-11}$, $J_{mp}=1.375\times10^{-8}$, $K_m=5.682\times10^{-4}$, $R_m=0.593$, $D_m\approx0$, $Q_f=9.507\times10^{-6}$）で RK4 積分。反トルクは $C_Q\omega^2+J_{mp}\dot\omega$（ヨー零点を物理的に再現）。（2026-08-22 追記: この実測ファミリ `measured_2026_07` は、ファームが使う静的曲線 `legacy_motor_curve` とは別モータの記述と判明。統一は #3 のベンチ計測待ち） |
| 3 | $C_T$/$C_Q$/thrust_efficiency の3点セット再較正（ファームCt切替） | 2026-07-15 thrust stand実測の $C_T$ は、新プロペラでの電圧・回転数・推力の有効な同時計測を欠くため撤回（2026-08-03）。撤回時点でファームウェア（`actuator.cpp` の `MOTOR_CT`）は暫定採用値 $C_T=1.00\times10^{-8}$ と数値一致していたため、当時は本タスクが解消したと判断されたが、この判断は誤りだった（詳細は状態欄） | **再オープン（2026-08-22）** — 2026-08-03 の「解消」判定は誤りだった。静的曲線 Am/Bm/Cm の廃止（2026-07-26）は SILS プラント側のみで、ファームの推力→デューティ経路（`thrustToDuty()`, `firmware/vehicle/components/sf_actuator/actuator.cpp`）は現在も静的曲線を使用しており、firmware/SILSプラント間の乖離は消滅していなかった。ファームの静的曲線は SSOT `legacy_motor_curve`（$R_m=0.34$, $K_m=6.125\times10^{-4}$, $C_Q=9.71\times10^{-11}$）の代数的言い換え（$A_m=R_mC_Q/K_m$, $C_m=R_mQ_f/K_m$ で厳密再現）である一方、SILS ODEプラントは `measured_2026_07`（$R_m=0.593$, $K_m=5.682\times10^{-4}$, $C_Q=4.10\times10^{-11}$）——両者は別モータを記述していた。ホバー点でプラントはファーム指令推力の1.252倍を出し、`hover.thrust_corr=1.12` が乗って機体重量の1.402倍の推力になっていたが、プラントの `thrust_efficiency` がこの不整合を偶然打ち消していた。db65e0e5（2026-08-03, Ct撤回）がその打ち消しを失わせ、2026-08-03〜2026-08-22 の間 `sf sils regression` シナリオ33本中20本が失敗していた（main未pushでCI未検知）。**2026-08-22 対応:** ファームの静的曲線に `hover.thrust_corr` の1.12を畳み込み（$A_m$×1.12・$B_m$×$\sqrt{1.12}$・$C_m$不変、全推力域でduty出力恒等の変換）、`hover.thrust_corr` 既定を1.00に復元。SSOTに `flight_anchored_motor_curve` を新設しファームはその写しに（`legacy_motor_curve` の実測記録は数値を変えず保存）。SILSプラントの `thrust_efficiency` を0.7133（=1/1.402、理想ODEと飛行実証済みファーム+実機の差を表す明示係数）に設定。SILSシナリオのSTABILIZEスロットルを×0.8386で再較正（ALT/POSは上昇率指令のため不変）。結果: `sf sils regression` は28 PASS + 5 KNOWN-FAIL（33本）に回復。**未解決:** `legacy_motor_curve`/`measured_2026_07` のどちらが新プロペラの実体かは未決着（ベンチ V-ω-T 3量同時計測待ち、後継タスクと同一）。`thrust_efficiency=0.7133` は計測完了までの暫定値であり、計測後は両ファミリを1モータモデルに統一し `thrust_efficiency=1.0` にできるはず |
| 4 | モータ不感帯・低 duty 非線形 | 実機 ~0.9 Hz リミットサイクルの再現に必要。`analysis/datasets/motor_sweep_20260714/` のベンチデータ（3個体・プロペラ有無2条件）で同定 | 未着手 |
| 5 | 空気抵抗の追加 | 現状 MuJoCo プラントは抗力ゼロ。`sf sysid drag` の実ログ同定値を使用 | 未着手 |
| 6 | フロー品質モデル（N3） | SQUAL（オプティカルフローの表面品質指標）固定100・無ノイズが POS_HOLD 初飛行発散の盲点だった（`firmware/vehicle/docs/poshold_journey.md`: 「Code Identity でも実機で動かない」盲点の実例）。Flow/Mag ノイズは N3 tier として後段に計画済み（`simulator/sils/RESET_PLAN.md` §13） | 未着手 |
| 7 | バッテリサグの $R_{int}$ 実測較正 | 電圧依存推力誤差が高度ウォブルの主因（corr(V, 高度std)=−0.78、`analysis/reports/poshold_3min_battery_wobble_20260627.md`）。サグモデル自体は実装済みで閉ループ emu では既定 ON（2026-06-07, `b8fd27ea`）。残作業は内部抵抗 $R_{int}$（現状値 0.1 Ω は vpython 由来の仮値）の実測較正のみ | 未着手 |
| 8 | N1 振動係数を現行 vehicle ログで再同定 | 現在の軸別係数（`vib_accel_k`/`vib_gyro_k`）は旧機（legacy `firmware/vehicle`）の hover02 ログ由来のシード値 | 未着手 |
| 9 | 実機ログ入力リプレイ（`sf sils replay` 相当） | WireControl（テレメトリの制御入力構造体）50 Hz スティック入力を `.scn` シナリオへ変換し、実ログと同一プロットで比較する。層2→層3 収束の要 | 未着手 |
| 10 | 関連文書の整合維持 | 本書と RESET_PLAN・development_roadmap の食い違いに気づいたら、本書を先に更新する | 継続 |
| 11 | ヨー軸ゲートの4パラメータ化 | `rate_sysid` のヨーフィットを反トルク零点込みの4パラメータモデル（`firmware/vehicle/docs/yaw_axis_model.md`）へ拡張し、実機基準値（`analysis/reports/rate_sysid_reference/README.md` の `reference.json`）も同一パイプラインで再生成して同条件比較にする | 未着手 |
| 12 | ファームヨートルク権限の再検討 | 新基準ホバー duty（≈0.7245）下での `rate.yaw.max_torque` 差動余裕を再検討する。SILS 回帰の pos_flight/pos_yaw/yaw_hold が known-fail（`sf sils regression` の xfail マーカー）として追跡中。実機 NT金沢問題（2026-07-17 治療）と同根の可能性がある | 未着手 |
| 13 | 姿勢減衰余裕の調査 | calib（注入バイアス×新プラントの離陸動特性で 0.6-0.7 Hz 自励振動）・commloss_land_level（LANDING 水平化ゲート中のロール収束不足）で顕在化。バックログ#4（モータ不感帯）・#5（空気抵抗ゼロ）との関連を確認する。（2026-08-22 追記: #3 で判明したプラント推力過大[ホバー点でファーム指令の1.252倍、corr込みで機体重量の1.402倍]が本現象の一因だった可能性があり、thrust_efficiency 補正後に再検証する） | 未着手 |

## 7. 関連文書マップ

| 文書 | 何の正か |
|---|---|
| 本書 | シミュレーション方針（3層構造・フェーズ・モデル一致ゲート・バックログ） |
| `simulator/sils/RESET_PLAN.md` | SILS ベンチの構造・立ち上げ経緯の記録（§2 方針1 は立ち上げ期の規律） |
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

This document defines the role of each simulation layer used in the StampFly Ecosystem — the design-oriented linear model, the log-driven offline replay, and the SILS (Software-In-the-Loop) bench — the policy on using real-flight data, and the pass criteria the SILS plant model must satisfy.

### Target Audience

- Developers who build or modify the SILS bench and simulator
- Developers proposing control-parameter changes (who must follow the discipline in §5)
- Future sessions (this document should make the history and current state of the policy traceable)

### Why This Document Is Needed

`simulator/sils/RESET_PLAN.md` §2, Policy 1 states that "real-flight data is not needed" and that the old M7/M8 steps (replaying and diffing against real logs) were dropped. This was correct discipline for the **bring-up phase**, before vehicle had ever flown — with no real-flight data in existence, a SILS that depended on it could not have served as a verification method.

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

### Layer 3 (SILS)

MuJoCo (a nonlinear 6-DOF physics engine) plus the real firmware C++, run under Code Identity (the SILS compiles and runs the exact same source as the real firmware, unmodified) and Param Identity (both run with the same parameters). It is the verification bench for the whole system, including the state machine, estimator, and failsafes. **New policy: the plant should target a nonlinear model that matches real-hardware identification values** (pass/fail is judged by the model-match gate in §4).

### Relationship Between the Layers

Layer 2 is not a simplified version of Layer 3 — it stands on its own as "the Layer-1 model driven by real-data excitation." Once Layer 3 gains (a) a high-fidelity plant and (b) real-log input replay (backlog #9 in §6), Layer 2 will converge toward being a fast, per-axis approximation of Layer 3. Layer 1 remains permanently in place for control design.

## 3. Evolution of the Policy (Bring-Up Phase → Model Fidelity Phase)

| Period | Phase | Treatment of real-flight data | Governing document |
|---|---|---|---|
| Until 2026-06 (pre-first-flight, SILS bring-up) | Clean-slate rebuild of the physics-based SILS | No real-flight data needed. Verification is done mechanically against the true physical model (the old M7/M8 real-log replay/diff steps were dropped) | RESET_PLAN §2 Policy 1 |
| 2026-06 onward (post-first-flight, Model Fidelity — current) | development_roadmap Phase 3–5 | Real logs are used for Layer-1 identification, Layer-2 A/B testing, and Layer-3 plant calibration. Replaying and comparing against real logs is not a policy violation — it is exactly the work of Phase 5 | development_roadmap Phase 5 |

Note: RESET_PLAN Policy 2 (independence from algorithm internals — depend on the interface, not the implementation) remains in effect regardless of phase; this document does not change it.

## 4. The Model-Match Gate (Layer-3 Pass/Fail)

Thanks to Code Identity, the same identification pipeline used on real-hardware logs (`sf sysid rate-fit`, etc.) can be applied directly to logs generated by the SILS.

**Procedure:**

1. Run a `rate-excite`-equivalent excitation inside the SILS
2. Apply the same identification pipeline used on real hardware
3. Extract $(b, L, T)$
4. Compare against the real-hardware identified values

**Pass criteria** (reusing the tolerances from development_roadmap Phase 3):

| Metric | Tolerance |
|---|---|
| Step-response rise time constant | ±20% |
| Gyro RMS | ±50% |

This gate is wired into the SILS regression test suite (automated tests that detect regressions), so every subsequent plant modification is judged numerically, both for improvement and for regression, every time.

Pass/fail is judged independently per axis. **Roll/pitch reached the passing region in the 2026-07-26 measurement (below); yaw has not.** SILS-based gain verification is now more trustworthy for roll/pitch, but **until all axes pass**, verification via the §5 perturbation family (torque effectiveness $\in[0.4, 0.7]$, dead time $L \in [8, 15]$ ms, venue-grade disturbance 0.2–1 Hz) continues alongside SILS verification, and the principle of "no SILS-only optimization" remains in force. Directly optimizing against a SILS turbulence bench previously converged on a gain with negative phase margin on real hardware (`firmware/vehicle/docs/control_theory_overview.md` §5.4: a gain optimized to Td=0.08 on the SILS diverged to PM −375° on real hardware).

> **First measurement (2026-07-22, `sf sils sysid-gate`, the old zero-dead-time / static-motor-curve plant):** FAILs on all axes — roll b +39.6% / L_total +39.0%, pitch b +109.7% / L_total +19.3%, yaw b −61.4% / L_total +95.7%. The lag **structure** is inverted vs. real hardware: the SILS is first-order-lag dominated (T≈20 ms = motor_tau, L≈1.5 ms) while the real machine is dead-time dominated (L≈11–16 ms, small T). With `--motor-delay 10`, L moves 1.5→10.6–12.2 ms exactly as designed, but L_total worsens to ~27 ms — matching requires the joint adjustment with backlog #2 (motor ODE) and #3 (coefficient recalibration), not delay alone. The yaw reference is the weakest of the three axes (3-parameter fit; see the note in `analysis/reports/rate_sysid_reference/README.md`).
>
> **Second measurement (2026-07-26, ODE plant, `sf sils sysid-gate`):** roll **PASS**es at b +27.2% / L_total +12.7%; pitch **PASS**es at b +26.6% / −1.5% (coherence 0.97–0.99, good). Yaw's b −18.2% is within tolerance, but L_total is −100% (identification degenerates) → **FAIL** — a structural problem: the newly implemented anti-torque zero ($\tau_z\approx45.7$ ms, a lead at 3.5 Hz within the control bandwidth) cannot be represented by the current 3-parameter $(b,L,T)$ fit. Note the real-hardware reference itself is also a weak 3-parameter fit with coherence 0.44. To be addressed by backlog #11.

## 5. Discipline That Does Not Change With Phase

- **Any control-parameter change must be backed by a numerical simulation using real flight logs.** Do not propose changes based on qualitative reasoning alone (e.g., "shortening Ti should help"). If simulation shows the change backfires, do not propose it (the rule in `control_theory_overview.md` §5.5).
- **Do not optimize for a single nominal model.** Real hardware drifts between sessions (the same gain produced a 2.4–2.7x change in the 5–8 Hz band reference value across sessions). Carry torque effectiveness $\in[0.4, 0.7]$, dead time $L \in [8, 15]$ ms, and venue-grade disturbance at 0.2–1 Hz as a **perturbation family**, and accept a gain only if it does not degrade across the whole family.
- **The SILS's inherent limitations cannot be verified within the SILS.** Concurrency conflicts (races between simultaneously running tasks) and the real WiFi/ESP-NOW physical layer cannot, in principle, be reproduced, because the SILS collapses what is normally concurrent processing into a single loop for reproducibility (RESET_PLAN §11). Real-hardware concurrency verification must be done separately.

## 6. SILS Plant Improvement Backlog (Priority Order)

| # | Item | Rationale / target value | Status |
|---|---|---|---|
| 0 | Implement the model-match gate (§4) | The yardstick for every other item. Highest priority | **Implemented (2026-07-22)** — `sf sils sysid-gate` |
| 1 | Add transport delay | Current SILS effective lag is ~5 ms vs. 8.4–14.7 ms on real hardware. Once the gate is in place, measure the SILS's current $L$ and set the difference as transport delay in the duty→thrust path | **Implemented (2026-07-22, default OFF)** — `sf sils scenario --motor-delay`. **Found unnecessary after the motor-ODE conversion (#2): stacking it on top of the ODE plant WORSENS L_total (measured 2026-07-26). Kept default OFF** |
| 2 | Convert the motor model to an ODE | Port the electromechanical ODE from `simulator/genesis/motor_model.py`: $\dot\omega = \bigl[-(D_m + K_m^2/R_m)\omega - C_Q\omega^2 - Q_f + K_m V/R_m\bigr]/J_{mp}$. Measured values: $J_{mp}=1.375\times10^{-8}$ kg·m², $C_Q=4.10\times10^{-11}$ N·m·s²/rad², $\omega_{hover}\approx3670$ rad/s, hover-point effective time constant $\tau_{eff}\approx17.5$ ms | **Implemented (2026-07-26)** — RK4 integration with the measured parameter family ($C_Q=4.10\times10^{-11}$, $J_{mp}=1.375\times10^{-8}$, $K_m=5.682\times10^{-4}$, $R_m=0.593$, $D_m\approx0$, $Q_f=9.507\times10^{-6}$). Anti-torque is $C_Q\omega^2+J_{mp}\dot\omega$ (physically reproduces the yaw zero). (Added 2026-08-22: this measured family, `measured_2026_07`, turned out to describe a different motor than the static curve `legacy_motor_curve` still used by firmware; unifying the two awaits the bench measurement in #3) |
| 3 | Recalibrate the $C_T$/$C_Q$/thrust_efficiency triple (firmware Ct switch) | The 2026-07-15 thrust-stand measurement of $C_T$ was retracted (2026-08-03) for lacking a valid simultaneous voltage/RPM/thrust measurement on the new propeller. At the time of retraction, the firmware (`actuator.cpp`'s `MOTOR_CT`) already numerically matched the provisional adopted value $C_T=1.00\times10^{-8}$, which was taken at the time to mean this task was resolved — that judgment turned out to be wrong (see Status) | **Reopened (2026-08-22)** — the 2026-08-03 "Resolved" verdict was wrong. The retirement of the static Am/Bm/Cm curve (2026-07-26) applied only to the SILS plant; the firmware's thrust→duty path (`thrustToDuty()` in `firmware/vehicle/components/sf_actuator/actuator.cpp`) still uses the static curve today, so the firmware/SILS-plant divergence had never actually closed. The firmware's static curve is an algebraic restatement of the SSOT `legacy_motor_curve` family ($R_m=0.34$, $K_m=6.125\times10^{-4}$, $C_Q=9.71\times10^{-11}$; exactly reproduced via $A_m=R_mC_Q/K_m$, $C_m=R_mQ_f/K_m$), while the SILS ODE plant uses the `measured_2026_07` family ($R_m=0.593$, $K_m=5.682\times10^{-4}$, $C_Q=4.10\times10^{-11}$) — the two describe different motors. At the hover point the plant produced 1.252× the firmware's commanded thrust; with `hover.thrust_corr=1.12` layered on top, the plant delivered 1.402× the vehicle's weight in thrust, but this mismatch had been coincidentally cancelled by the value carried in the plant's `thrust_efficiency`. db65e0e5 (2026-08-03, the Ct retraction) removed that cancellation, so 20 of 33 `sf sils regression` scenarios failed from 2026-08-03 through 2026-08-22 (undetected by CI because main was unpushed). **2026-08-22 fix:** folded `hover.thrust_corr`'s 1.12 into the firmware's static curve ($A_m$×1.12, $B_m$×$\sqrt{1.12}$, $C_m$ unchanged — an identity transform for duty output across the whole thrust range), restoring `hover.thrust_corr`'s default to 1.00. Added a new `flight_anchored_motor_curve` family to the SSOT, which the firmware now mirrors (`legacy_motor_curve`'s measured record is kept unchanged). Set the SILS plant's `thrust_efficiency` to 0.7133 (=1/1.402, an explicit single coefficient standing in for the gap between the idealized ODE and the flight-proven firmware+hardware combination). Recalibrated the SILS scenarios' STABILIZE throttle by ×0.8386 (ALT/POS throttle is a climb-rate command and is unchanged). Result: `sf sils regression` recovered to 28 PASS + 5 KNOWN-FAIL (33 total). **Still open:** whether `legacy_motor_curve` or `measured_2026_07` is the true description of the new propeller remains undecided (needs a bench V-ω-T simultaneous co-measurement — same follow-up task as before). `thrust_efficiency=0.7133` is a provisional value until that measurement lands; once it does, the two families should unify into a single motor model and `thrust_efficiency` should become 1.0 |
| 4 | Motor dead-band / low-duty nonlinearity | Needed to reproduce the ~0.9 Hz limit cycle seen on real hardware. Identify from the bench data in `analysis/datasets/motor_sweep_20260714/` (3 airframes, props on/off) | Not started |
| 5 | Add aerodynamic drag | The current MuJoCo plant has zero drag. Use the real-log-identified values from `sf sysid drag` | Not started |
| 6 | Flow-quality model (N3) | A fixed SQUAL (optical-flow surface-quality metric) of 100 with no noise was the blind spot behind the first POS_HOLD real-flight divergence (`firmware/vehicle/docs/poshold_journey.md`: a concrete case of "passes Code-Identity SILS yet fails on hardware"). Flow/mag noise is already planned as the N3 tier for a later stage (`simulator/sils/RESET_PLAN.md` §13) | Not started |
| 7 | Calibrate battery-sag $R_{int}$ from measurement | Voltage-dependent thrust error is the leading cause of altitude wobble (corr(V, altitude std) = −0.78, `analysis/reports/poshold_3min_battery_wobble_20260627.md`). The sag model itself is already implemented and already defaults ON in the closed-loop emulator (since 2026-06-07, `b8fd27ea`). The remaining work is only to calibrate the internal resistance $R_{int}$ from measurement (the current 0.1 Ω is a placeholder carried over from the vpython model) | Not started |
| 8 | Re-identify the N1 vibration coefficients on current-vehicle logs | The current per-axis coefficients (`vib_accel_k`/`vib_gyro_k`) are seed values taken from the legacy `firmware/vehicle` hover02 log | Not started |
| 9 | Real-log input replay (`sf sils replay`-equivalent) | Convert WireControl (the telemetry control-input struct) 50 Hz stick input into `.scn` scenarios, and compare against the real log on the same plots. The key step for Layer-2 → Layer-3 convergence | Not started |
| 10 | Keep related documents consistent | If a discrepancy is noticed between this document and RESET_PLAN / development_roadmap, update this document first | Ongoing |
| 11 | 4-parameterize the yaw-axis gate | Extend the `rate_sysid` yaw fit to the 4-parameter model that includes the anti-torque zero (`firmware/vehicle/docs/yaw_axis_model.md`), and regenerate the real-hardware reference (`reference.json` in `analysis/reports/rate_sysid_reference/README.md`) through the same pipeline for an apples-to-apples comparison | Not started |
| 12 | Revisit firmware yaw torque authority | Re-examine the `rate.yaw.max_torque` differential headroom under the new reference hover duty (≈0.7245). SILS regression's pos_flight/pos_yaw/yaw_hold are being tracked as known-fail (xfail marker in `sf sils regression`). May share a root cause with the real-hardware NT-Kanazawa issue (treated 2026-07-17) | Not started |
| 13 | Investigate attitude damping margin | Surfaced by calib (0.6–0.7 Hz self-oscillation from injected bias × the new plant's takeoff dynamics) and commloss_land_level (insufficient roll convergence during LANDING leveling). Check for a relationship with backlog #4 (motor dead-band) and #5 (zero aerodynamic drag). (Added 2026-08-22: the plant thrust excess found in #3 [1.252× the firmware-commanded thrust at hover, 1.402× vehicle weight once thrust_corr was included] may have contributed to this; re-verify after the thrust_efficiency fix) | Not started |

## 7. Related Document Map

| Document | Authority for |
|---|---|
| This document | Simulation policy (three-layer structure, phases, model-match gate, backlog) |
| `simulator/sils/RESET_PLAN.md` | SILS bench structure and the bring-up history (§2 Policy 1 is bring-up-phase discipline) |
| `firmware/vehicle/docs/development_roadmap.md` | The overall development process (Phase 0–6) |
| `firmware/vehicle/docs/control_theory_overview.md` | Control-design discipline and identification lessons |
| `firmware/vehicle/docs/noise_and_vibration_model.md` | Sensor noise model (N0–N2, N3/N4 planned) |
| `firmware/vehicle/docs/yaw_axis_model.md` | The yaw-axis model |
| `docs/architecture/stampfly-parameters.md` | Physical-parameter values and measurement history |
| `analysis/scripts/alt_dob_design/README.md` and other `analysis/reports/` entries | Layer-2 execution records |
