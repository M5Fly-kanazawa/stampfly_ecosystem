# StampFly シミュレーション方針（Simulation Policy）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

> 制定: 2026-07-22。全面改定: 2026-09-08。改定理由: 初版は「層1 設計用線形モデル／層2 実ログ駆動再生／層3 SILS」という 3 層の枠組みで書かれていたが、これは設計者の意図した整理ではなく、設計の道具（モデル・解析手法）と実行環境（SILS）という軸の違うものを一列に並べていた。本改定では、設計者が各シミュレータに与えた役割を正とし、3 層の枠組みを廃止する。モデル一致の合否判定・規律・改修バックログは事実として引き継ぐ。

## 1. 概要

### このドキュメントについて

本ドキュメントは、StampFly Ecosystem にあるシミュレータ（SILS・VPython 版・Genesis 版）それぞれの役割と実現方法、共通の物理パラメータの扱い、実機データの使い方、SILS のプラント（制御対象の物理モデル）が満たすべき合格基準、そして今後の強化学習に向けた物理エンジンの比較を定める。シミュレーションに関する方針の正本であり、他文書と食い違ったときは本書を先に直す。

### 対象読者

- SILS・シミュレータを開発・改修する開発者
- 制御パラメータの変更を SILS や解析で裏付けようとする制御設計者
- シミュレータを教材として使う教育者、強化学習への応用を検討する研究者

### なぜ本書が必要か

シミュレータが 3 つあると「なぜ複数あるのか」「どれを何に使うのか」「物理はどこから来ているのか」「実機と同じコードが動くのはどれか」という疑問が必ず出る。答えを一か所に置き、資料や実装がこれと食い違わないようにするのが本書の目的である。

## 2. 3 つのシミュレータとその役割

| | SILS | VPython 版 | Genesis 版 |
|---|---|---|---|
| **役割** | **ファームウェアの開発と制御系の実装を、机上である程度完了させる**ための仕組み。飛ばす前の検証と合否判定 | **練習用**。比較的簡単で可読性の高い Python コードで、力学エンジン・3D 可視化・センサモデルなど「シミュレータの作り方」を学んでもらう | **強化学習**を行う上で使いやすいと判断し、選択肢として残している |
| **動く制御コード** | 実機に書き込むのと同じ C++ ファームウェア（無改変） | Python に移植した制御則 | Python の制御則 |
| **物理モデル** | MuJoCo（外部の物理エンジン。**物理計算にのみ使用**）＋自作のモータ・センサ・風モデル | 自作の 6 自由度剛体モデル（Python）＋センサモデル | Genesis（外部の高精度物理エンジン、GPU 並列） |
| **可視化** | `sf sils gui`（ブラウザ。three.js の 3D と Plotly のグラフ）。レビュー動画は事後に MuJoCo の Python レンダラで生成（`--video`） | VPython（ブラウザ 3D） | Genesis の描画 |
| **入力** | シナリオ `.scn`（操縦・外乱・故障の時系列）、キーボード操縦 | USB HID ジョイスティック（AtomS3 + Atom JoyStick） | スクリプト |
| **合否判定** | `.expect` による自動判定、`sf sils regression`（CI） | なし | なし |
| **場所・入口** | `simulator/sils/`、`sf sils build/scenario/gui` | `simulator/vpython/`、`sf sim run vpython` | `simulator/genesis/`、`sf sim run genesis` |

### なぜ 1 つでは足りないか

「実機のファームをそのまま動かして検証する」「中身を読んで作り方を学ぶ」「強化学習を回す」は要求が違い、1 つの実装では両立しない。SILS は決定論と実ファームとの同一性を最優先し、VPython 版は読みやすさを最優先し、Genesis 版は GPU 並列と学習との相性を優先する。3 つは §3 の物理パラメータを共有し、`sf params check` で食い違いを検出する。

### SILS の実現方法

- **ファームウェアは無改変**: `firmware/vehicle`（および `vehicle_old`・`workshop`）のソースをそのまま PC 向けにコンパイルする。推定・制御だけでなく状態機械やフェイルセーフも含めて実機と同一（Code Identity）。パラメータも同じ表から読む（Parameter Identity）。
- **OS の代わり**: ESP-IDF / FreeRTOS の代わりに、ホスト用の互換スタブ（`compat/`）と、単一トークン＋仮想時計の離散事象スケジューラである決定論的な疑似 RTOS（`rtos/`）の上で走らせる。同じ入力なら毎回同じ結果になる。
- **制御対象**: MuJoCo の 6 自由度剛体モデルに、自作のモータ（電気機械 ODE）・センサ・風のモデルを載せ（`physics/`, `plant/`）、400 Hz でファームと歩調を合わせる。**MuJoCo は物理計算にのみ使い、実行中の描画には使わない**。MuJoCo の対話ビューアはモデルファイルを目視確認するための任意ビルドオプション（`-DSILS_MUJOCO_VIEWER=ON`）で、シナリオ実行には関与しない。
- **試験の与え方**: シナリオ `.scn` に操縦入力・外乱・故障を時系列で書き、`.expect` の合格基準で PASS / FAIL を自動判定する。`sf sils regression` が CI で退行を検出する。
- **学習者コードも同じ土俵**: `workshop` ターゲットでは `user_code.cpp` が同じプラントでループを閉じる（`sf lesson sils`）。
- **できないこと**: 複数タスクの競合（並行処理の競合）と、実際の WiFi / ESP-NOW の物理層は、再現性のために処理を一本のループにまとめている構造上、原理的に再現できない（`simulator/sils/RESET_PLAN.md` §11）。実機でしか確かめられない。

### 設計・解析の道具はシミュレータではない

次の 2 つは以前「層 1・層 2」と呼んでいたが、機体を動かして見せる実行環境ではなく、設計と解析の道具である。本書では区別して扱う。

| 道具 | 中身 | 用途 | 場所・入口 |
|---|---|---|---|
| 設計用の線形モデル | 実飛行ログから同定した低次の伝達関数 $G(s)$ | ゲイン設計、ループ整形、仕様ベースの自動チューニング | `tools/sysid/`、`sf sysid fit` / `rate-fit` / `rate-tune` |
| 実ログ駆動の再生 | 同定モデルと Python に移植した制御則を、実機ログから再構成した外乱・指令で駆動する閉ループ再生 | パラメータ変更の A/B 判定 | `analysis/scripts/` |

## 3. 共通の物理パラメータ

質量・慣性・推力係数 $C_T$・反トルク係数 $C_Q$ などの機体物理パラメータの正本は `control/models/stampfly_physical.yaml` である。ファームウェア（`generated_params` 系ヘッダ）・SILS プラント・VPython 版・Genesis 版・`docs/architecture/stampfly-parameters.md` はここから生成または転記し、`sf params check` が転記の食い違いを検出する。値の実測履歴と採用根拠は `stampfly-parameters.md` に置く。

注意: `sf params check` は転記の一致しか見ない。「ファームの静的モータ曲線と SILS プラントの ODE が別のモータを表していた」（2026-08-22 判明）のような**モデル構造の不一致**は検出できないため、§5 の合否判定で数値的に確認する。

## 4. 実機データの扱い（立ち上げ期 → Model Fidelity 期）

| 期間 | フェーズ | 実機データの扱い | 根拠文書 |
|---|---|---|---|
| 〜2026-06（初飛行前・SILS立ち上げ期） | 更地化・物理ベース SILS の再構築 | 実機データ不要。物理モデルの真値で機械的に検証（旧 M7/M8 の実機ログ再生・差分診断は廃止） | RESET_PLAN §2 方針1 |
| 2026-06〜（実機飛行後・Model Fidelity 期＝現在） | development_roadmap Phase 3〜5 | 実機ログで線形モデルの同定・実ログ再生による A/B・SILS プラントの較正を行う。実機ログの再生・突き合わせは方針違反ではなく Phase 5 の本作業そのもの | development_roadmap Phase 5 |

注: RESET_PLAN 方針2（アルゴリズムの中身に依存せず、実装でなくインターフェースに依存する）は期に依らず有効であり、本書はこれを変更しない。

## 5. SILS のモデル一致の合否判定

Code Identity のおかげで、実機同定に使ったのと同一の同定パイプライン（`sf sysid rate-fit` 等）を、SILS が生成したログにもそのまま適用できる。

**2026-09-11 追記（フライトログ形式の統一）:** SILS と実機は、同じ「StampFly フライトログ一式」形式（拡張子 `.sflog.zip`。1 回の飛行・実行のセンサ信号一式をまとめた zip 形式のログファイルで、仕様の正本は `protocol/spec/flight_log.yaml`）でログを書き出すようになった。これにより Code Identity（コード一致: 制御・推定のソースコードが実機と SILS で同一であること）は同定パイプラインだけでなく記録形式にも及び、`sf sysid fit`/`rate-fit` と `sf log viz`/`analyze` は実機ログ・SILS ログのどちらに対しても改修なしで動く。SILS 側だけが追加で持つストリームは、MuJoCo（物理計算のみに使う外部の物理エンジン）が計算した位置・姿勢・速度・角速度の真値を収める `truth.csv` と、シナリオが注入した外乱・故障などの事象を時刻付きで記録する `events.csv` である。ロール軸のレートステップ用シナリオ `sysid_roll_step.scn`（ACRO モードでの角速度ダブレット入力、13 秒）でこの経路を実測したところ、`sf sysid fit --axis roll --mixer vehicle` はロール軸のトルク→角加速度ゲイン $K=99453$ rad/s² per N·m（設計値 $1/I_{xx}=109170$ に対し 8.9% 低い）、モータの実効時定数 $\tau_m=14$ ms（SILS のモータ ODE 自体の実効時定数は約 16 ms、同定ツール側の既定基準値は 20 ms）、決定係数（当てはまりの良さを表す指標。1 に近いほど良い）$R^2=1.00$ を得た。

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

## 6. 期に依らず変わらない規律

- **制御パラメータ変更は必ず実フライトログを使った数値シミュレーションで裏付ける。** 「Ti を短くすれば改善する」のような定性推測だけで提案しない。シミュレーションの結果、逆効果であれば提案しない（`control_theory_overview.md` §5.5 の鉄則）。
- **公称モデル1点への最適化をしない。** 実機はセッション間でドリフトする（同一ゲインで 5–8 Hz 帯の基準値が2.4〜2.7倍変動した実例がある）。トルク効き $\in[0.4, 0.7]$、むだ時間 $L \in [8, 15]$ ms、会場級外乱 0.2〜1 Hz を**摂動族**として持ち、ゲインの採否は族全体で悪化しないことを条件にする。
- **SILS の原理的限界は SILS では検証できない。** 並行処理の競合（複数タスクが同時に走ることによる競合）や実 WiFi/ESP-NOW の物理層は、SILS の再現性のために本来並行する処理を一本のループにまとめている構造上、原理的に再現できない（RESET_PLAN §11）。実機並行性の検証は別途行う。

## 7. 強化学習に向けた物理エンジンの比較（MuJoCo と Genesis）

今後、強化学習で「MuJoCo か Genesis か」という選択が出てくる。本リポジトリでの位置づけと、判断の観点を整理しておく。数値や対応状況は 2026 年 9 月時点の把握であり、採用時に最新版で再確認すること。

| 観点 | MuJoCo | Genesis |
|---|---|---|
| 開発元・実装 | Google DeepMind。C 実装＋Python バインディング。Apache-2.0 | Genesis-Embodied-AI（大学・企業の共同）。Python / PyTorch 実装。Apache-2.0 |
| 実行形態 | CPU 逐次実行が基本。GPU 大規模並列は別実装の MJX（JAX 版） | GPU 並列が前提。数千環境を一括で進め、微分可能 |
| 物理の範囲 | 剛体・関節・接触が中心 | 剛体に加え流体・柔軟体・粒子などの複数物理（対応範囲は版により異なる、要確認） |
| 決定論・再現性 | 単一スレッドで決定論的（SILS が要求する性質） | GPU 並列では演算順序により結果が揺れ得る（要確認） |
| 強化学習の周辺整備 | 成熟。Gymnasium・dm_control・MuJoCo Playground など事例が多い | 新しい（2024 年 12 月公開）。学習例は同梱されるが周辺は発展途上 |
| 本リポジトリでの現在の使い方 | SILS の物理（C API で実ファームと歩調を合わせる）。描画には使わない | `simulator/genesis/`（Python の制御則、モータ ODE の出典） |
| 実ファームとの接続 | SILS が既に接続済み（C++ 同一ソース） | 接続する仕組みは無い。学習した方策をファームへ移す工程が別途要る |
| 向く場面 | 実ファームと同じ物理で方策を検証したい、CPU で確実に回したい | 何千機を同時に学習させたい、微分可能性を使いたい |
| 課題 | 大規模並列は MJX を別途用意する必要があり、C++ ファームとは接続できない | 決定論性と成熟度。SILS の合否判定に相当する物差しが無い |

方針: 学習は Genesis（または MJX）で回し、得られた方策の検証は SILS（MuJoCo 物理、実ファーム）で行う、という分担が現実的である。両者の物理パラメータは §3 の正本で揃える。

## 8. SILS プラント改修バックログ（優先順）

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
| 9 | 実機ログ入力リプレイ（`sf sils replay` 相当） | WireControl（テレメトリの制御入力構造体）50 Hz スティック入力を `.scn` シナリオへ変換し、実ログと同一プロットで比較する。実ログ再生（解析）の結果を SILS で再現するための要 | 未着手 |
| 10 | 関連文書の整合維持 | 本書と RESET_PLAN・development_roadmap の食い違いに気づいたら、本書を先に更新する | 継続 |
| 11 | ヨー軸ゲートの4パラメータ化 | `rate_sysid` のヨーフィットを反トルク零点込みの4パラメータモデル（`firmware/vehicle/docs/yaw_axis_model.md`）へ拡張し、実機基準値（`analysis/reports/rate_sysid_reference/README.md` の `reference.json`）も同一パイプラインで再生成して同条件比較にする | 未着手 |
| 12 | ファームヨートルク権限の再検討 | 新基準ホバー duty（≈0.7245）下での `rate.yaw.max_torque` 差動余裕を再検討する。SILS 回帰の pos_flight/pos_yaw/yaw_hold が known-fail（`sf sils regression` の xfail マーカー）として追跡中。実機 NT金沢問題（2026-07-17 治療）と同根の可能性がある | 未着手 |
| 13 | 姿勢減衰余裕の調査 | calib（注入バイアス×新プラントの離陸動特性で 0.6-0.7 Hz 自励振動）・commloss_land_level（LANDING 水平化ゲート中のロール収束不足）で顕在化。バックログ#4（モータ不感帯）・#5（空気抵抗ゼロ）との関連を確認する。（2026-08-22 追記: #3 で判明したプラント推力過大[ホバー点でファーム指令の1.252倍、corr込みで機体重量の1.402倍]が本現象の一因だった可能性があり、thrust_efficiency 補正後に再検証する） | 未着手 |

## 9. 関連文書マップ

| 文書 | 何の正か |
|---|---|
| 本書 | シミュレーション方針（3 つのシミュレータの役割・SILS の実現方法・物理パラメータの正本・モデル一致の合否判定・バックログ） |
| `simulator/README.md` | VPython 版・Genesis 版の使い方 |
| `simulator/sils/README.md` | SILS ベンチの使い方（ターゲット・シナリオ・GUI） |
| `simulator/sils/RESET_PLAN.md` | SILS ベンチの構造・立ち上げ経緯の記録（§2 方針1 は立ち上げ期の規律） |
| `firmware/vehicle/docs/development_roadmap.md` | 開発工程全体（Phase 0〜6） |
| `firmware/vehicle/docs/control_theory_overview.md` | 制御設計の規律・同定の教訓 |
| `firmware/vehicle/docs/noise_and_vibration_model.md` | センサノイズモデル（N0〜N2、N3/N4 計画） |
| `firmware/vehicle/docs/yaw_axis_model.md` | ヨー軸モデル |
| `docs/architecture/stampfly-parameters.md` | 物理パラメータの値と実測履歴 |
| `analysis/scripts/alt_dob_design/README.md` ほか `analysis/reports/` | 実ログ駆動の再生の実施記録 |

---

<a id="english"></a>

> Established: 2026-07-22. Fully revised: 2026-09-08. Reason for revision: The first edition was written around a three-tier framework — "Tier 1: design-oriented linear model / Tier 2: log-driven replay / Tier 3: SILS" — but this was not the organisation the designer intended; it lined up things with different axes — design tools (models, analysis methods) and an execution environment (SILS) — in a single row. This revision treats the roles the designer assigned to each simulator as authoritative and abolishes the three-tier framework. The SILS model-match pass/fail check, the discipline, and the improvement backlog are carried forward as-is.

## 1. Overview

### About This Document

This document defines the role and implementation method of each simulator in the StampFly Ecosystem (SILS, the VPython version, and the Genesis version), how the shared physical parameters are handled, how real-flight data is used, the pass/fail criteria the SILS plant (the physical model of the controlled object) must satisfy, and a comparison of physics engines for future reinforcement learning work. It is the single source of truth for the simulation policy; when it conflicts with another document, this document is corrected first.

### Target Audience

- Developers who build and modify SILS and the simulators
- Control designers who want to back up control-parameter changes with SILS or analysis
- Educators who use the simulators as teaching material, and researchers considering applications to reinforcement learning

### Why This Document Is Needed

With three simulators, the questions "why are there several?", "which one is used for what?", "where does the physics come from?", and "which one runs the same code as the real vehicle?" inevitably come up. The purpose of this document is to put the answers in one place and keep documents and implementations from diverging from it.

## 2. The Three Simulators and Their Roles

| | SILS | VPython version | Genesis version |
|---|---|---|---|
| **Role** | A mechanism for **bringing firmware development and control-system implementation to a reasonable degree of completion on the desk (without flying)**. Verification and pass/fail checking before flight | **For practice**. Relatively simple, readable Python code for learning "how to build a simulator" — physics engine, 3D visualization, sensor models, and so on | Kept as an option because it is judged to be easy to use for **reinforcement learning** |
| **Running control code** | The same C++ firmware that is flashed onto the real vehicle (unmodified) | Control law ported to Python | Control law in Python |
| **Physical model** | MuJoCo (an external physics engine; **used only for physics computation**) plus in-house motor, sensor, and wind models | In-house 6-DOF rigid-body model (Python) plus a sensor model | Genesis (an external high-precision physics engine, GPU-parallel) |
| **Visualization** | `sf sils gui` (browser-based; 3D via three.js and graphs via Plotly). Review videos are generated afterward with MuJoCo's Python renderer (`--video`) | VPython (browser 3D) | Genesis's rendering |
| **Input** | Scenario `.scn` files (time series of stick input, disturbances, and faults), keyboard piloting | USB HID joystick (AtomS3 + Atom JoyStick) | Scripts |
| **Pass/fail check** | Automatic judgment via `.expect` files, `sf sils regression` (CI) | None | None |
| **Location / entry point** | `simulator/sils/`, `sf sils build/scenario/gui` | `simulator/vpython/`, `sf sim run vpython` | `simulator/genesis/`, `sf sim run genesis` |

### Why One Is Not Enough

"Run the real vehicle's firmware as-is to verify it," "read the internals to learn how it's built," and "run reinforcement learning" are different requirements that a single implementation cannot satisfy at once. SILS gives top priority to determinism and identity with the real firmware, the VPython version gives top priority to readability, and the Genesis version prioritizes GPU parallelism and compatibility with learning. All three share the physical parameters in §3, and `sf params check` detects discrepancies.

### How SILS Is Realised

- **Firmware is unmodified**: The source of `firmware/vehicle` (and `vehicle_old`, `workshop`) is compiled for the PC as-is. Not only estimation and control but also the state machine and failsafe logic are identical to the real vehicle (Code Identity). Parameters are also read from the same table (Parameter Identity).
- **In place of the OS**: Instead of ESP-IDF / FreeRTOS, the firmware runs on host-side compatibility stubs (`compat/`) and a deterministic pseudo-RTOS (`rtos/`) — a discrete-event scheduler with a single token and a virtual clock. The same input always produces the same result.
- **Controlled object (plant)**: On top of MuJoCo's 6-DOF rigid-body model, in-house motor (electromechanical ODE), sensor, and wind models are layered (`physics/`, `plant/`), running in step with the firmware at 400 Hz. **MuJoCo is used only for physics computation, not for rendering during execution.** MuJoCo's interactive viewer is an optional build flag (`-DSILS_MUJOCO_VIEWER=ON`) for visually inspecting the model file, and plays no part in scenario execution.
- **How tests are given**: Stick input, disturbances, and faults are written as a time series in a scenario `.scn` file, and PASS/FAIL is judged automatically against the pass criteria in an `.expect` file. `sf sils regression` detects regressions in CI.
- **Learner code runs on the same ground**: In the `workshop` target, `user_code.cpp` closes the loop against the same plant (`sf lesson sils`).
- **What it cannot do**: Contention between multiple tasks (concurrency contention) and the physical layer of actual WiFi / ESP-NOW cannot be reproduced, in principle, because of the structure that folds processing into a single loop for the sake of reproducibility (`simulator/sils/RESET_PLAN.md` §11). These can only be checked on the real vehicle.

### Design and Analysis Tools Are Not Simulators

The following two used to be called "Tier 1" and "Tier 2," but they are design and analysis tools, not execution environments that show the vehicle moving. This document treats them separately.

| Tool | Content | Purpose | Location / entry point |
|---|---|---|---|
| Design-oriented linear model | A low-order transfer function $G(s)$ identified from real flight logs | Gain design, loop shaping, specification-based automatic tuning | `tools/sysid/`, `sf sysid fit` / `rate-fit` / `rate-tune` |
| Log-driven replay | Closed-loop replay that drives the identified model and a control law ported to Python with disturbances and commands reconstructed from real-vehicle logs | A/B comparison of parameter changes | `analysis/scripts/` |

## 3. Shared Physical Parameters

The single source of truth for the vehicle's physical parameters — mass, inertia, thrust coefficient $C_T$, counter-torque coefficient $C_Q$, and so on — is `control/models/stampfly_physical.yaml`. The firmware (the `generated_params` family of headers), the SILS plant, the VPython version, the Genesis version, and `docs/architecture/stampfly-parameters.md` are generated from it or hand-copied from it, and `sf params check` detects discrepancies in the hand-copied values. The measurement history of the values and the rationale for adopting them are kept in `stampfly-parameters.md`.

Note: `sf params check` only checks that hand-copied values match; it cannot detect **a mismatch in model structure**, such as "the firmware's static motor curve and the SILS plant's ODE represented different motors" (discovered 2026-08-22). Such mismatches are confirmed numerically by the pass/fail check in §5.

## 4. Handling Real-Flight Data (Startup Phase → Model Fidelity Phase)

| Period | Phase | Handling of real-flight data | Basis document |
|---|---|---|---|
| Through 2026-06 (before first flight; SILS startup phase) | Clean-slate rebuild of a physics-based SILS | Real-flight data not required. Mechanical verification against the physical model's true values (the old M7/M8 real-log replay and differential diagnosis is discontinued) | RESET_PLAN §2, Policy 1 |
| From 2026-06 (after real flight; Model Fidelity phase = present) | development_roadmap Phase 3–5 | Identify the linear model from real-flight logs, perform A/B comparison via log-driven replay, and calibrate the SILS plant. Replaying and cross-checking against real-flight logs is not a policy violation — it is precisely the work of Phase 5 itself | development_roadmap Phase 5 |

Note: RESET_PLAN Policy 2 (not depending on the internals of an algorithm, depending on the interface rather than the implementation) remains valid regardless of phase; this document does not change it.

## 5. SILS Model-Match Pass/Fail

Thanks to Code Identity, the same identification pipeline used for real-vehicle identification (`sf sysid rate-fit`, etc.) can be applied as-is to logs generated by SILS.

**Added 2026-09-11 (flight-log format unification):** SILS and the real vehicle now write logs in the same "StampFly flight-log bundle" format (extension `.sflog.zip` — a zip-format log file bundling one flight's or one run's sensor signals; the authoritative spec is `protocol/spec/flight_log.yaml`). This extends Code Identity (the property that the control/estimation source code is identical between the real vehicle and SILS) beyond the identification pipeline to the log format itself: `sf sysid fit`/`rate-fit` and `sf log viz`/`analyze` now run unchanged on either a real-vehicle log or a SILS log. The only streams unique to SILS are `truth.csv` (the ground-truth position/attitude/velocity/angular-rate computed by MuJoCo, the external physics engine used only for physics computation) and `events.csv` (a time-stamped record of the disturbances/faults the scenario injected). A cross-check using the roll-axis rate-step scenario `sysid_roll_step.scn` (ACRO-mode angular-rate doublets, 13 s) exercised this path and found: `sf sysid fit --axis roll --mixer vehicle` identified a roll torque-to-angular-acceleration gain $K=99453$ rad/s² per N·m (8.9% below the design value $1/I_{xx}=109170$), a motor effective time constant $\tau_m=14$ ms (the SILS motor ODE's own effective time constant is about 16 ms; the identification tool's default reference is 20 ms), and a coefficient of determination (a goodness-of-fit measure, closer to 1 is better) $R^2=1.00$.

**Procedure:**

1. Perform excitation equivalent to `rate-excite` inside SILS
2. Apply the same identification pipeline used for the real vehicle
3. Extract $(b, L, T)$
4. Compare against the real-vehicle identified values

**Pass criteria** (reusing the tolerances from development_roadmap Phase 3):

| Metric | Tolerance |
|---|---|
| Step-response rise time constant | ±20% |
| gyro RMS | ±50% |

This gate is built into the SILS regression test (an automated test for detecting regressions), and the effect and degradation of every subsequent plant improvement is judged numerically each time.

Gate pass/fail is judged independently per axis. **Roll/pitch reached the passing range in the 2026-07-26 measurement (below) (yaw has not yet reached it).** Confidence in using SILS for gain verification has improved for the roll/pitch axes, but **until all axes pass**, verification using the perturbation family from §5 (torque authority $\in[0.4,0.7]$, dead time $L\in[8,15]$ ms, venue-scale disturbance 0.2〜1 Hz) is used together with SILS verification, maintaining the principle of "no SILS-only optimization." There is a lesson learned that directly optimizing against the SILS turbulence bench once converged on a gain whose phase margin went negative on the real vehicle (`firmware/vehicle/docs/control_theory_overview.md` §5.4: a gain optimized to Td=0.08 on SILS diverged to a phase margin of −375° on the real vehicle).

> **First measurement (2026-07-22, `sf sils sysid-gate`, old plant with zero dead time and a static motor curve):** All axes FAIL — roll b +39.6% / L_total +39.0%, pitch b +109.7% / L_total +19.3%, yaw b −61.4% / L_total +95.7%. The **structure** of the delay is opposite to the real vehicle: SILS is dominated by a first-order lag (T≈20ms=motor_tau, L≈1.5ms), while the real vehicle is dominated by dead time (L≈11〜16ms, small T). With `--motor-delay 10`, L moves from 1.5→10.6〜12.2ms as designed, but L_total worsens to around 27ms — matching requires simultaneous adjustment with backlog #2 (moving to a motor ODE) and #3 (coefficient recalibration), not the delay alone. Note that the real-vehicle reference value for yaw, derived from a 3-parameter fit, is the weakest (see the note in `analysis/reports/rate_sysid_reference/README.md`).
>
> **Second measurement (measured 2026-07-26, ODE plant, `sf sils sysid-gate`):** roll: b +27.2% / L_total +12.7%, **PASS**; pitch: b +26.6% / −1.5%, **PASS** (coherence 0.97〜0.99, good). yaw: b −18.2% is within tolerance, but L_total is −100% (identification degenerate), **FAIL** — a structural problem in which the implemented counter-torque zero ($\tau_z\approx45.7$ ms, a 3.5 Hz lead within the control bandwidth) cannot be represented by the current 3-parameter $(b,L,T)$ fit. Note that the real-vehicle reference value itself is also a weak reference, being a 3-parameter fit with coherence 0.44 (to be addressed in backlog #11).

## 6. Discipline That Does Not Change With Phase

- **Control parameter changes must always be backed by numerical simulation using real flight logs.** Do not propose a change based only on a qualitative guess such as "shortening Ti should improve it." If the simulation shows the change is counterproductive, do not propose it (the iron rule in `control_theory_overview.md` §5.5).
- **Do not optimize for a single nominal-model point.** The real vehicle drifts between sessions (there is an actual case where, with the same gain, the reference value in the 5–8 Hz band varied by a factor of 2.4〜2.7). Maintain torque authority $\in[0.4, 0.7]$, dead time $L \in [8, 15]$ ms, and venue-scale disturbance 0.2〜1 Hz as a **perturbation family**, and require that adopting a gain not degrade performance across the whole family.
- **SILS's fundamental limitations cannot be verified with SILS.** Concurrency contention (contention arising from multiple tasks running at the same time) and the physical layer of actual WiFi/ESP-NOW cannot be reproduced, in principle, because of the structure that folds processing that is inherently concurrent into a single loop for SILS's reproducibility (RESET_PLAN §11). Verification of real-vehicle concurrency is carried out separately.

## 7. Physics Engines for Reinforcement Learning: MuJoCo vs Genesis

Going forward, the choice of "MuJoCo or Genesis" will come up for reinforcement learning. This section organizes their positioning in this repository and the points to consider when deciding. The figures and support status reflect the understanding as of September 2026; re-check against the latest version at the time of adoption.

| Aspect | MuJoCo | Genesis |
|---|---|---|
| Developer / implementation | Google DeepMind. C implementation with Python bindings. Apache-2.0 | Genesis-Embodied-AI (a university/industry collaboration). Python/PyTorch implementation. Apache-2.0 |
| Execution form | Basically sequential CPU execution. Large-scale GPU parallelism is provided by a separate implementation, MJX (the JAX version) | Assumes GPU parallelism. Advances thousands of environments in a batch, and is differentiable |
| Scope of physics | Centered on rigid bodies, joints, and contact | Multiple physics domains in addition to rigid bodies — fluids, soft bodies, particles, etc. (coverage varies by version; needs checking) |
| Determinism / reproducibility | Deterministic on a single thread (the property SILS requires) | With GPU parallelism, results can vary with computation order (needs checking) |
| Reinforcement-learning ecosystem | Mature. Many examples such as Gymnasium, dm_control, MuJoCo Playground | New (released December 2024). Training examples are bundled, but the surrounding ecosystem is still developing |
| Current usage in this repository | SILS's physics (kept in step with the real firmware via the C API). Not used for rendering | `simulator/genesis/` (Python control law; source of the motor ODE) |
| Connection to the real firmware | Already connected via SILS (identical C++ source) | There is no mechanism to connect it. A separate process is needed to transfer a learned policy to the firmware |
| Where it fits | When you want to verify a policy under the same physics as the real firmware, or want to run reliably on CPU | When you want to train thousands of instances at once, or want to use differentiability |
| Challenges | Large-scale parallelism requires setting up MJX separately, and it cannot be connected to the C++ firmware | Determinism and maturity. There is no yardstick equivalent to SILS's pass/fail check |

Policy: a realistic division of labor is to run training with Genesis (or MJX) and verify the resulting policy with SILS (MuJoCo physics, real firmware). The physical parameters of both are kept aligned via the single source of truth in §3.

## 8. SILS Plant Improvement Backlog (Priority Order)

| # | Task | Basis / target value | Status |
|---|---|---|---|
| 0 | Implementing the model-match gate (§4) | The yardstick for every improvement. Highest priority | **Implemented (2026-07-22)** — `sf sils sysid-gate` |
| 1 | Adding dead time | Current SILS effective lag ~5 ms vs. real vehicle 8.4〜14.7 ms. Measure SILS's current $L$ with the gate, and make it possible to configure the difference as a transport delay in the duty→thrust path | **Implemented (2026-07-22, default OFF)** — `sf sils scenario --motor-delay`. **Found that after moving to an ODE, no additional delay is needed (stacking it worsens L_total, measured 2026-07-26). Default OFF is maintained** |
| 2 | Moving the motor model to an ODE | Port the electromechanical ODE from `simulator/genesis/motor_model.py`, $\dot\omega = \bigl[-(D_m + K_m^2/R_m)\omega - C_Q\omega^2 - Q_f + K_m V/R_m\bigr]/J_{mp}$, to SILS. Measured values: $J_{mp}=1.375\times10^{-8}$ kg·m², $C_Q=4.10\times10^{-11}$ N·m·s²/rad², $\omega_{hover}\approx3670$ rad/s, hover-point effective time constant $\tau_{eff}\approx17.5$ ms | **Implemented (2026-07-26)** — RK4 integration with the measured family ($C_Q=4.10\times10^{-11}$, $J_{mp}=1.375\times10^{-8}$, $K_m=5.682\times10^{-4}$, $R_m=0.593$, $D_m\approx0$, $Q_f=9.507\times10^{-6}$). Counter-torque is $C_Q\omega^2+J_{mp}\dot\omega$ (physically reproducing the yaw zero). (Added 2026-08-22: this measured family, `measured_2026_07`, turned out to describe a different motor from `legacy_motor_curve`, the static curve the firmware uses. Unification awaits the bench measurement in #3) |
| 3 | Recalibrating the $C_T$/$C_Q$/thrust_efficiency triplet (firmware Ct switchover) | The $C_T$ measured on the 2026-07-15 thrust stand was withdrawn (2026-08-03) because it lacks a valid simultaneous measurement of voltage, rotation speed, and thrust for the new propeller. At the time of withdrawal, the firmware (`MOTOR_CT` in `actuator.cpp`) numerically matched the provisionally adopted value $C_T=1.00\times10^{-8}$, so this task was judged resolved at the time — but that judgment was wrong (see the status column for details) | **Reopened (2026-08-22)** — The "resolved" judgment of 2026-08-03 was wrong. The retirement of the static curve Am/Bm/Cm (2026-07-26) applied only to the SILS-plant side; the firmware's thrust→duty path (`thrustToDuty()`, `firmware/vehicle/components/sf_actuator/actuator.cpp`) still uses the static curve, and the divergence between the firmware and the SILS plant had not disappeared. The firmware's static curve is an algebraic restatement of the SSOT `legacy_motor_curve` ($R_m=0.34$, $K_m=6.125\times10^{-4}$, $C_Q=9.71\times10^{-11}$) — exactly reproduced via $A_m=R_mC_Q/K_m$, $C_m=R_mQ_f/K_m$ — while the SILS ODE plant is `measured_2026_07` ($R_m=0.593$, $K_m=5.682\times10^{-4}$, $C_Q=4.10\times10^{-11}$): the two describe different motors. At the hover point, the plant produced 1.252 times the firmware's commanded thrust, and with `hover.thrust_corr=1.12` applied on top, this became 1.402 times the vehicle weight in thrust — but the plant's `thrust_efficiency` happened to cancel this inconsistency. db65e0e5 (2026-08-03, withdrawal of Ct) removed that cancellation, and between 2026-08-03 and 2026-08-22, 20 of the 33 `sf sils regression` scenarios were failing (undetected by CI because main had not been pushed). **2026-08-22 fix:** folded the `hover.thrust_corr` factor of 1.12 into the firmware's static curve ($A_m$×1.12, $B_m$×$\sqrt{1.12}$, $C_m$ unchanged — a transformation that leaves the duty output identical across the whole thrust range), and restored the `hover.thrust_corr` default to 1.00. Added `flight_anchored_motor_curve` to the SSOT and made the firmware a copy of it (the measurement record of `legacy_motor_curve` is kept unchanged in value). Set the SILS plant's `thrust_efficiency` to 0.7133 (=1/1.402, an explicit coefficient representing the difference between the ideal ODE and the flight-proven firmware + real vehicle). Recalibrated the STABILIZE throttle in SILS scenarios by ×0.8386 (ALT/POS are unaffected since they command climb rate). Result: `sf sils regression` recovered to 28 PASS + 5 KNOWN-FAIL (33 total). **Unresolved:** it remains undecided which of `legacy_motor_curve` / `measured_2026_07` reflects the actual new propeller (awaiting a bench measurement of V-ω-T simultaneously across all three quantities — the same as the follow-on task). `thrust_efficiency=0.7133` is a provisional value until that measurement is complete; afterward, the two families should be unified into a single motor model and `thrust_efficiency` should become 1.0 |
| 4 | Motor dead zone / low-duty nonlinearity | Needed to reproduce the real vehicle's ~0.9 Hz limit cycle. Identify from bench data in `analysis/datasets/motor_sweep_20260714/` (3 units, 2 conditions with/without propeller) | Not started |
| 5 | Adding aerodynamic drag | Currently the MuJoCo plant has zero drag. Use the value identified from real logs by `sf sysid drag` | Not started |
| 6 | Flow-quality model (N3) | A fixed SQUAL (the optical-flow surface-quality indicator) of 100 with no noise was a blind spot behind the POS_HOLD divergence on the first flight (`firmware/vehicle/docs/poshold_journey.md`: a concrete example of the blind spot "even with Code Identity, it doesn't work on the real vehicle"). Flow/Mag noise is already planned as the N3 tier for a later stage (`simulator/sils/RESET_PLAN.md` §13) | Not started |
| 7 | Measurement-based calibration of the battery-sag $R_{int}$ | Voltage-dependent thrust error is the main cause of altitude wobble (corr(V, altitude std)=−0.78, `analysis/reports/poshold_3min_battery_wobble_20260627.md`). The sag model itself is already implemented and is default ON in the closed-loop emulator (2026-06-07, `b8fd27ea`). The only remaining work is measurement-based calibration of the internal resistance $R_{int}$ (the current value of 0.1 Ω is a provisional value taken from the VPython version) | Not started |
| 8 | Re-identifying the N1 vibration coefficients from current vehicle logs | The current per-axis coefficients (`vib_accel_k`/`vib_gyro_k`) are seed values derived from the hover02 log of the old airframe (legacy `firmware/vehicle`) | Not started |
| 9 | Real-vehicle log input replay (equivalent to `sf sils replay`) | Convert 50 Hz stick input from WireControl (the telemetry control-input struct) into a `.scn` scenario and compare it against the real log on the same plot. The key piece for reproducing, in SILS, the results of real-log replay (analysis) | Not started |
| 10 | Maintaining consistency of related documents | When a discrepancy is noticed between this document and RESET_PLAN / development_roadmap, update this document first | Ongoing |
| 11 | Extending the yaw-axis gate to a 4-parameter model | Extend the `rate_sysid` yaw fit to a 4-parameter model that includes the counter-torque zero (`firmware/vehicle/docs/yaw_axis_model.md`), and regenerate the real-vehicle reference value (`reference.json` in `analysis/reports/rate_sysid_reference/README.md`) with the same pipeline so the comparison is made under matching conditions | Not started |
| 12 | Reconsidering firmware yaw torque authority | Reconsider the `rate.yaw.max_torque` differential margin under the new reference hover duty (≈0.7245). SILS regression's pos_flight/pos_yaw/yaw_hold are being tracked as known-fail (an xfail marker in `sf sils regression`). May share the same root cause as the real-vehicle NT Kanazawa issue (remedied 2026-07-17) | Not started |
| 13 | Investigating attitude damping margin | Manifested in calib (0.6〜0.7 Hz self-excited oscillation from the interaction of injected bias with the new plant's takeoff dynamics) and commloss_land_level (insufficient roll convergence during the LANDING leveling gate). Check the relationship with backlog #4 (motor dead zone) and #5 (zero aerodynamic drag). (Added 2026-08-22: the excessive plant thrust found in #3 [1.252× the firmware's commanded thrust at the hover point, 1.402× the vehicle weight once `hover.thrust_corr` is included] may have been a contributing factor in this phenomenon; re-verify after the thrust_efficiency correction) | Not started |

## 9. Related Document Map

| Document | Authoritative for |
|---|---|
| This document | Simulation policy (the role of the three simulators, how SILS is realised, the single source of truth for physical parameters, the SILS model-match pass/fail check, the backlog) |
| `simulator/README.md` | How to use the VPython and Genesis versions |
| `simulator/sils/README.md` | How to use the SILS bench (targets, scenarios, GUI) |
| `simulator/sils/RESET_PLAN.md` | The record of the SILS bench's structure and startup history (§2 Policy 1 is the discipline for the startup phase) |
| `firmware/vehicle/docs/development_roadmap.md` | The overall development process (Phase 0–6) |
| `firmware/vehicle/docs/control_theory_overview.md` | Control-design discipline and lessons from identification |
| `firmware/vehicle/docs/noise_and_vibration_model.md` | Sensor noise model (N0–N2, N3/N4 planned) |
| `firmware/vehicle/docs/yaw_axis_model.md` | The yaw-axis model |
| `docs/architecture/stampfly-parameters.md` | Physical parameter values and measurement history |
| `analysis/scripts/alt_dob_design/README.md` and others in `analysis/reports/` | Records of log-driven replay work |
