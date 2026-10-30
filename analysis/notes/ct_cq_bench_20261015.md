# プロペラ定数のベンチ実測（C_T / C_Q）を SILS とファームに採用するかの検討 (2026-10-15 実測、2026-10-30 検討)

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

**状態**: 検討完了（採用見送り）。作成 2026-10-30、最終更新 2026-10-30。

## 1. 目的と結論

### 問い

2026-10-15 に 1 モータのベンチで得た推進系の実測値を、暫定採用の $C_T=1.00\times10^{-8}$ ・$C_Q=4.10\times10^{-11}$（`control/models/stampfly_physical.yaml` の `measured_2026_07`。SILS（Software-In-the-Loop Simulation、ファームのソースをそのまま PC 上で動かすシミュレーション）のプラントとファームが使う）と置き換えるべきか。

### 結論（2026-10-30 時点）

| 項目 | 結論 | 主な理由 |
|---|---|---|
| SILS プラントへの採用 | **見送り** | モデル一致の確認（`sf sils sysid-gate`、実機のレート応答との比較）が悪化する（ロール $b$ +56.5 % → +67 / +76 %、$L_\text{total}$ +17 % → +30 / +41 %）。再確認試験（変更で既存の動作が壊れていないかを自動で確かめる試験。`sf sils regression` の 44 シナリオ）でも V1 で 1 件、V2 で 2 件が不合格になる |
| ファーム側だけの換算替え（VF） | **単独では採用不可** | SILS 上の挙動は変わらないことを確認したが、yaml を変えないと `sf params check --strict` が 4 行で不一致になる。パラメータ基準ファイルでファームの曲線とプラントの定数組を分ける方針の決定が先に要る |
| 次の手 | Duty → モータ平均電圧の関係をプラントに入れる | ベンチでは、推力の差の主因がこの関係にあることを示している（3 章） |

## 2. ベンチ実測とデータ

### ベンチ条件

| 項目 | 内容 |
|---|---|
| 実施日 | 2026-10-15 |
| 対象 | StampFly 1 機の 1 モータ（M1）、現行プロペラ |
| 駆動 | 基板上の PWM 駆動、Duty 0.1〜0.9、電池電圧 4.17〜4.38 V |
| 計測 | 回転数は整流子の切り替わりで出るモータ電圧の脈動から、推力は秤で計測 |
| 計測手段・データ | 公開リポジトリ `stampfly-motor-autotest`（GitHub `kouhei1970/stampfly-motor-autotest`）の `data/2026-10-15/` |

### 実測結果

| 量 | 値 | 意味 |
|---|---|---|
| $K_e$ | $5.90\times10^{-4}$ V·s/rad | 逆起電力定数 |
| $R$ | 0.594 Ω | 巻線抵抗 |
| $C_T$ | $7.60\times10^{-9}$ N/(rad/s)² | 推力係数（$T=C_T\omega^2$） |
| $C_Q$ | $3.38\times10^{-11}$ N·m/(rad/s)² | トルク係数（$Q=C_Q\omega^2$） |
| $J$（プロペラ込み） | $1.54\times10^{-8}$ kg·m² | ロータ慣性 |
| 無負荷電流 $I_0$ | $0.044+2.9\times10^{-9}\,\omega^2$ A | 回転数に依存する損失分を含む |
| ステップごとの $C_T$ | Duty 0.1 で $6.3\times10^{-9}$ → Duty 0.9 で $7.8\times10^{-9}$ | Duty が大きいほど増える |

**注意**: このデータの電池電流は「ファームの読み値の 10 倍」であり、独立した電流計測ではない（電流検出抵抗の誤設定をコミット d685c9fd で修正した。それ以前は 1/10 の値を報告していた）。

### 関連ファイル

| パス | 内容 |
|---|---|
| `analysis/scripts/ct_cq_bench_20261015/patches/VF.patch`, `V1.patch`, `V2.patch` | 検討に使った変更（4 章の比較条件）。いずれも `main` のコミット 884f2af6（検討時の作業ツリーの基点）に対する差分で、d685c9fd にもそのまま適用できることを確認済み |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/regression.json` | `sf sils regression` の結果（V0, VF, V1, V2）。V1・V2 の `fail` 総数には、バイパスしたパラメータ整合の事前検査が 1 件加算されている。シナリオの不合格数は V1=1、V2=2 |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/gate.json` | `sf sils sysid-gate` の生の出力（V0, VF, V1, V2） |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/refit_a.txt`, `refit_b.txt` | 同じ記録を `plant_fit.py` の定数のままで（a）、または Am/Bm を Ct に合わせて（b）再同定した結果（V0, V1, V2）。5 章参照。V0 の b は Ct を変えずに Am/Bm だけ 0.76 倍にした不整合な条件で、$b$ が約 0.76 倍に下がるだけ（逆方向の確認）であり、基準値には使わない |
| `analysis/scripts/ct_cq_bench_20261015/bench_vs_models.py` | ベンチと、ファームの静的曲線・SILS プラント定常状態との比較（3 章の表を出力） |
| `analysis/scripts/ct_cq_bench_20261015/refit_gate_bundle.py` | `sysid-gate` の記録一式（`.sflog.zip`）を、`plant_fit.py` の定数を差し替えて再同定する |

再現: `python3 -I analysis/scripts/ct_cq_bench_20261015/bench_vs_models.py <stampfly-motor-autotest の data/2026-10-15/summary.json>`

## 3. 定常特性の比較

### 比較したモデル

| モデル | 内容 |
|---|---|
| ファームの静的曲線 | `actuator.cpp` の `thrustToDuty`: $\omega=\sqrt{T/C_t}$、$V=A_m\omega^2+B_m\omega+C_m$、duty $=V/v_\text{bat}$ |
| SILS プラントのモータ常微分方程式（ODE）の定常解 | 推力 $T=\eta\,C_t\,\omega^2$（$\eta$ = `thrust_efficiency` = 0.7133） |

### 分かったこと

1. 現行の SILS プラント（$\eta=0.7133$）とファームの曲線は、どの Duty でも同じ推力を出す（2026-08-22 に等価にした）。
2. Duty × 電池電圧で駆動すると、両モデルとも低 Duty でベンチ推力を下回り（Duty 0.11 で 0.48 g 対 0.93 g）、ホバー付近では近い。**測定したモータ平均電圧 $\langle V_m\rangle$ で駆動すると、ベンチ一式の定数を入れたプラント ODE はベンチ推力を0.35 g 以内（ホバー付近の Duty 0.5〜0.7 では 0.2〜2.7 %）で再現する。** ベンチの定数同士は矛盾しておらず、ずれの所在は Duty → $\langle V_m\rangle$ の関係にある。SILS プラントはこれを $V=\text{Duty}\times v_\text{batt}$ と仮定している。
3. 区間ごとの傾き（推力 対 Duty×$V_{DD}$、および $\langle V_m\rangle$ 対 Duty×$V_{DD}$。$V_{DD}$ は計測時の電池電圧）:

| Duty 区間 | ベンチ [g/V] | モデル [g/V] | 比 | $\mathrm{d}\langle V_m\rangle/\mathrm{d}(\text{Duty}\cdot V_{DD})$ |
|---|---|---|---|---|
| 0.11→0.21 | 3.54 | 2.49 | 1.42 | 1.08 |
| 0.21→0.31 | 4.19 | 3.56 | 1.18 | 0.91 |
| 0.31→0.41 | 4.49 | 4.42 | 1.02 | 0.82 |
| 0.41→0.51 | 4.66 | 5.12 | 0.91 | 0.80 |
| 0.51→0.61 | 4.79 | 5.72 | 0.84 | 0.70 |
| 0.61→0.71 | 3.75 | 6.23 | 0.60 | 0.53 |
| 0.71→0.81 | 5.93 | 6.66 | 0.89 | 0.76 |
| 0.81→0.91 | 11.69 | 7.02 | 1.67 | 1.50 |

   ホバー付近（Duty 0.4〜0.8）では、実際の Duty 当たり推力ゲイン（傾き）はモデルの 0.60〜0.91 倍である。SILS のモデル一致の確認で出た実機/SILS のロールゲイン比 0.64（65384/102334）、および 2026-10-02 の宙返りログのフィットによる `torque_authority` 0.60 と同じ向きで、この差の大きな部分を説明する可能性がある。ただし全部を説明するとは示していない（1 モータ、$V_{DD}$ 4.2〜4.4 V、4 モータ同時の電池電圧低下なし）。
4. ホバー点で合わせた $\eta$: 1 モータ当たりのホバー推力は 0.0907 N（質量 0.037 kg）、ファームのホバー電圧は 2.581 V。プラントがこの電圧でホバーするよう $\eta$ を求め直すと次のとおり。

| 条件 | $\eta$ | ホバー $\omega$ [rad/s] | ホバー時のモータ時定数 [ms] | ファームが指令したヨートルクに対するプラントのヨートルク | ホバーでの $\mathrm{d}T/\mathrm{d}V$ [mN/V] |
|---|---|---|---|---|---|
| V0 現行 | 0.7133 | 3566 | 16.4 | 1.402 | 58.3 |
| V1 $C_T$, $C_Q$ のみ | 0.880 | 3682 | 17.3 | 1.136 | 59.5 |
| V2 ベンチ一式（$K_e$, $R$, $Q_f=K_e\cdot0.044$, $C_Q$ に $K_e\cdot2.9\times10^{-9}$ を含める, $J=1.54\times10^{-8}$） | 0.941 | 3561 | 18.4 | 1.116 | 60.3 |

   $\eta$ の補正は推力にだけ掛かるため、現状のプラントのヨー権限はファームの想定の 1.40 倍になっている。ベンチの 1 モータがホバー推力に達するのは Duty×$V_{DD}$ = 2.44 V で、ファームの 2.58 V より小さい。つまり飛行に合わせ込んだファームの曲線は、ベンチより約 7 % 多い推力能力を要求している（原因は未特定。質量、機体の空力干渉、4 モータ時の電池電圧低下が候補）。

## 4. SILS での検討

隔離した git worktree（本体のツリーと分けた作業コピー）で実施した。

### 比較した条件

| 条件 | 内容 |
|---|---|
| V0 | 基準（現行） |
| VF | ファーム側だけの換算替え: `MOTOR_CT` 7.6e-9、`MOTOR_AM` ×0.76 = 4.587968e-8、`MOTOR_BM` ×$\sqrt{0.76}$ = 5.840089e-4、`KAPPA` 4.447e-3、`rate.yaw.kp` ×1.0847 = 8.710112e-4、`rate.yaw.max_torque` 既定 1.329872e-3（テーブル上限 1.529461e-3）。推力 → 電圧の写像と、物理的なヨー推力差が数学的に同じになる換算 |
| V1 | VF + プラントの $C_T$ 7.60e-9、$C_Q$ 3.38e-11、$\eta$ 0.8802 |
| V2 | VF + プラントにベンチ一式、$\eta$ 0.9412 |

### 再確認試験（`sf sils regression`、44 シナリオ）

VF・V1・V2 では、手書きの写しが基準ファイルと食い違うため、実行前のパラメータ整合の事前検査をバイパスして実行した（6 章の再現手順を参照）。

| 条件 | PASS | FAIL | KNOWN-FAIL | XPASS | SKIP |
|---|---|---|---|---|---|
| V0 | 38 | 0 | 5 | 0 | 1 |
| VF | 38 | 0 | 5 | 0 | 1 |
| V1 | 37 | 1 | 4 | 1 | 1 |
| V2 | 36 | 2 | 4 | 1 | 1 |

（KNOWN-FAIL = 既知の不合格として期待しているシナリオ、XPASS = 既知の不合格のはずが合格したもの）

判定が変わったシナリオ（それぞれ 2 回再実行し、決定論的）:

| 条件 | シナリオ | 内容 |
|---|---|---|
| V1 | `alt_inflight_switch` FAIL | alt_max 1.1155 m（基準 < 1.1。V0 は 1.0511） |
| V2 | `commloss` FAIL | 着陸が IDLE_GROUND に到達しない（原因は未調査） |
| V2 | `stab_flight` FAIL | att_rmse 0.0527（基準 < 0.05236。V0 は 0.0489） |
| V1, V2 | `calib` XPASS | 既知の 0.6〜0.7 Hz 自励振動の失敗が合格する。SILS 方針のバックログ #13 の手がかりであって、結果ではない |
| VF | 判定変化なし | 検査した 170 指標のうち 118 は同一、52 が異なるが、すべて基準内。相対変化が最大なのは 0 に近い値（例: `rc_flip` の tilt_max 0.0064→0.0101 rad） |

### モデル一致の確認（`sf sils sysid-gate`、既定プラント）

V0 と VF は `gate.json` の生の値、V1 と V2 は 5 章の修正後の値（`refit_b.txt`）。許容は $b$ ±50 %、$L_\text{total}$ ±20 %。実機は ロール $b$=65384・$L_\text{total}$=14.36 ms、ピッチ 30724・17.29 ms、ヨー 111897・10.82 ms。**どの条件も不合格で、V0 も含む。**

| 条件 | ロール $b$ | ロール $L_\text{total}$ | ピッチ $b$ | ピッチ $L_\text{total}$ | ヨー $b$ | ヨー $L_\text{total}$ |
|---|---|---|---|---|---|---|
| V0 | +56.5 % | 16.86 ms (+17.4 %) | +211.3 % | 17.09 ms (−1.2 %) | (+74.6 %) | (0.58 ms, −94.7 %) |
| VF | +57.3 % | 16.92 ms (+17.8 %) | +212.1 % | 17.24 ms (−0.3 %) | (+75.3 %) | (0.31 ms, −97.1 %) |
| V1 | +67.0 % | 18.66 ms (+30.0 %) | +221.0 % | 18.16 ms (+5.0 %) | (+23.3 %) | (2.09 ms, −80.6 %) |
| V2 | +75.7 % | 20.22 ms (+40.8 %) | +226.2 % | 19.16 ms (+10.8 %) | (+67.9 %) | (0.84 ms, −92.3 %) |

ヨーの数値は、どの条件でも信頼できない（3 パラメータのフィットが縮退し、$L_\text{total}$ が 0 に近い。既知の課題でバックログ #11）ため括弧書きにした。ロール・ピッチでは、ベンチ定数がモータの遅れを増やし（フィットした時定数 $T$ が 15.2 ms → 17.3 / 18.9 ms）、SILS を実機からさらに遠ざける。

## 5. 見つかった解析ツールの不具合

`sf sils sysid-gate` は、ログされた duty からファームの式を逆算してトルクを復元する（`tools/sysid/plant_fit.py`）。ここで `_MOTOR_CT` と `_KAPPA` は生成されたパラメータから来る（yaml に追従する）が、`_MOTOR_AM`・`_MOTOR_BM`・`_MOTOR_CM` は手書きのリテラルで、`sf params check` は検査しない。このため yaml の Ct だけを変えると、復元される推力が Ct_new/Ct_old 倍になり、$b$ が Ct_old/Ct_new 倍に膨らむ（今回は 1/0.76 = 1.32 倍）。

| 条件 | 生の出力（`gate.json`） | Am/Bm を Ct に合わせて再同定（`refit_b.txt`） |
|---|---|---|
| V1 ロール $b$ | 143702（+119.8 %） | 109213（+67.0 %） |
| V1 ピッチ $b$ | 129788（+322.4 %） | 98639（+221.0 %） |

同じ記録の再同定で $T$・$L$ は変わらず、原因の説明と一致した。**提案（未実施）**: この 3 つのリテラルを `sf params check` の検査対象に加えるか、`actuator.cpp` と同じ元から導く。

## 6. 判断と次の手（2026-10-30 時点）

| 項目 | 内容 |
|---|---|
| プラント | 採用見送り。モデル一致の確認が悪化し（ロール $b$ +56.5 → +67 / +76 %、$L_\text{total}$ +17 → +30 / +41 %）、再確認試験が壊れる（V1 で 1 件、V2 で 2 件） |
| ファーム側の換算替え（VF） | SILS での挙動維持は確認済み。ただし yaml を変えないと `sf params check --strict` が 4 行（C_T、kappa、Am_flight_anchored、Bm_flight_anchored）で不一致になるため、単独では採用できない。採用にはパラメータ基準ファイルでファームの曲線とプラントの定数組を分ける方針の決定が要る。実機では `rate.yaw.*` の `param reset` も要る |
| 次の手 | 基板の駆動回路の Duty → モータ平均電圧の関係を SILS プラントに入れ、そのうえで $C_T$・$C_Q$・$J$ をモデル一致の確認で再評価する |
| 電流検出 | コミット d685c9fd 以前は電池電流が 1/10 で報告されていた。ベンチデータの電流は「ファームの読み値の 10 倍」 |

### 再現手順

1. `main` のコミット 884f2af6（または d685c9fd）に `analysis/scripts/ct_cq_bench_20261015/patches/<V>.patch` を適用する。
2. `sf params generate`、`sf sils build`、`sf sils regression`、`sf sils sysid-gate` の順に実行する。
3. V1・V2 は手書きの写しが基準ファイルと食い違うため、`sf sils regression` 冒頭のパラメータ整合の事前検査（`sf params check`）がエラーで止まる。この検査だけを外して実行した（VF も同様）。
4. 修正後の $b$ は、`sysid-gate` が出した記録（`gate_bundle.sflog.zip`）を `analysis/scripts/ct_cq_bench_20261015/refit_gate_bundle.py <リポジトリのルート> <記録> --am 4.587968e-8 --bm 5.840089e-4` で再同定して得る。

---

<a id="english"></a>

# Should the bench-measured propeller constants (C_T / C_Q) replace the SILS and firmware values? (measured 2026-10-15, studied 2026-10-30)

**Status**: study complete (not adopted). Created 2026-10-30, last updated 2026-10-30.

## 1. Purpose and conclusion

### Question

Should the propulsion constants measured on a single-motor bench on 2026-10-15 replace the provisional $C_T=1.00\times10^{-8}$ and $C_Q=4.10\times10^{-11}$ (`measured_2026_07` in `control/models/stampfly_physical.yaml`, used by the SILS plant and the firmware)? SILS is Software-In-the-Loop Simulation: the firmware source runs unmodified on a PC.

### Conclusion (as of 2026-10-30)

| Item | Conclusion | Main reasons |
|---|---|---|
| Adopt in the SILS plant | **Not adopted** | The model-match check (`sf sils sysid-gate`, comparison with the real rate response) gets worse (roll $b$ +56.5 % to +67 / +76 %, $L_\text{total}$ +17 % to +30 / +41 %). The regression test suite (44 scenarios of `sf sils regression`, which automatically confirms that existing behaviour is not broken) fails in 1 scenario for V1 and 2 for V2 |
| Firmware-only re-gauging (VF) | **Not adoptable alone** | SILS behaviour is preserved, but with the yaml unchanged `sf params check --strict` fails on 4 rows. A decision on how the parameter file separates the firmware curve from the plant constants must come first |
| Next step | Put the Duty to average-motor-voltage relation into the plant | The bench points to this relation as the main source of the thrust difference (chapter 3) |

## 2. Bench measurement and data

### Bench conditions

| Item | Content |
|---|---|
| Date | 2026-10-15 |
| Subject | One motor (M1) of one StampFly, current propeller |
| Drive | PWM drive on the board, Duty 0.1 to 0.9, battery 4.17 to 4.38 V |
| Measurement | Rotor speed from commutation ripple, thrust from a scale |
| Tool and data | Public repo `stampfly-motor-autotest` (GitHub `kouhei1970/stampfly-motor-autotest`), `data/2026-10-15/` |

### Results

| Quantity | Value | Meaning |
|---|---|---|
| $K_e$ | $5.90\times10^{-4}$ V·s/rad | Back-EMF constant |
| $R$ | 0.594 Ω | Winding resistance |
| $C_T$ | $7.60\times10^{-9}$ N/(rad/s)² | Thrust coefficient ($T=C_T\omega^2$) |
| $C_Q$ | $3.38\times10^{-11}$ N·m/(rad/s)² | Torque coefficient ($Q=C_Q\omega^2$) |
| $J$ (with propeller) | $1.54\times10^{-8}$ kg·m² | Rotor inertia |
| No-load current $I_0$ | $0.044+2.9\times10^{-9}\,\omega^2$ A | Includes a speed-dependent loss term |
| Per-step $C_T$ | $6.3\times10^{-9}$ at Duty 0.1 to $7.8\times10^{-9}$ at Duty 0.9 | Rises with Duty |

**Caveat**: the battery current in this data is the firmware reading multiplied by 10, not an independent current measurement (the shunt-resistor setting was fixed in commit d685c9fd; before it the current was reported at 1/10).

### Related files

| Path | Content |
|---|---|
| `analysis/scripts/ct_cq_bench_20261015/patches/VF.patch`, `V1.patch`, `V2.patch` | The changes used in the study (variants in chapters 3 and 4). Differences against `main` commit 884f2af6 (the base of the working tree at study time); verified to apply to d685c9fd as well |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/regression.json` | `sf sils regression` results (V0, VF, V1, V2). For V1 and V2 the top-level `fail` count includes one extra item for the bypassed parameter-consistency pre-check; scenario failures are V1=1, V2=2 |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/gate.json` | Raw `sf sils sysid-gate` output (V0, VF, V1, V2) |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/refit_a.txt`, `refit_b.txt` | The same recording refitted with the constants of `plant_fit.py` as they are (a), or with Am/Bm matched to Ct (b) (V0, V1, V2). See chapter 5. The V0 file b uses an inconsistent condition (Am/Bm scaled by 0.76 with Ct unchanged), only lowers $b$ by about 0.76 (the reverse check), and is not used as a reference |
| `analysis/scripts/ct_cq_bench_20261015/bench_vs_models.py` | Compares the bench with the firmware static curve and the SILS plant steady state (prints the tables of chapter 3) |
| `analysis/scripts/ct_cq_bench_20261015/refit_gate_bundle.py` | Refits a `sysid-gate` bundle with overridable constants of `plant_fit.py` |

Reproduce: `python3 -I analysis/scripts/ct_cq_bench_20261015/bench_vs_models.py <data/2026-10-15/summary.json of stampfly-motor-autotest>`

## 3. Steady-state comparison

### Models compared

| Model | Content |
|---|---|
| Firmware static curve | `thrustToDuty` in `actuator.cpp`: $\omega=\sqrt{T/C_t}$, $V=A_m\omega^2+B_m\omega+C_m$, duty $=V/v_\text{bat}$ |
| Steady state of the SILS plant motor ODE (ordinary differential equation) | thrust $T=\eta\,C_t\,\omega^2$ ($\eta$ = `thrust_efficiency` = 0.7133) |

### Findings

1. The current SILS plant ($\eta=0.7133$) and the firmware curve give the same thrust at every Duty (made equivalent on 2026-08-22).
2. Driven by Duty × battery voltage, both models under-predict the bench thrust at low Duty (0.48 g vs 0.93 g at Duty 0.11) and are close near hover. **Driven by the measured average motor voltage $\langle V_m\rangle$, the plant ODE with the full bench set reproduces the bench thrust within 0.35 g (0.2 to 2.7 % near hover, Duty 0.5 to 0.7).** The bench constants are mutually consistent; the mismatch sits in the Duty to $\langle V_m\rangle$ relation, which the SILS plant assumes to be $V=\text{Duty}\times v_\text{batt}$.
3. Per-interval slopes (thrust vs Duty×$V_{DD}$, and $\langle V_m\rangle$ vs Duty×$V_{DD}$; $V_{DD}$ is the battery voltage at measurement):

| Duty interval | bench [g/V] | model [g/V] | ratio | $\mathrm{d}\langle V_m\rangle/\mathrm{d}(\text{Duty}\cdot V_{DD})$ |
|---|---|---|---|---|
| 0.11→0.21 | 3.54 | 2.49 | 1.42 | 1.08 |
| 0.21→0.31 | 4.19 | 3.56 | 1.18 | 0.91 |
| 0.31→0.41 | 4.49 | 4.42 | 1.02 | 0.82 |
| 0.41→0.51 | 4.66 | 5.12 | 0.91 | 0.80 |
| 0.51→0.61 | 4.79 | 5.72 | 0.84 | 0.70 |
| 0.61→0.71 | 3.75 | 6.23 | 0.60 | 0.53 |
| 0.71→0.81 | 5.93 | 6.66 | 0.89 | 0.76 |
| 0.81→0.91 | 11.69 | 7.02 | 1.67 | 1.50 |

   Near hover (Duty 0.4 to 0.8) the real thrust-per-duty gain (slope) is 0.60 to 0.91 of the model's. This points the same way as the real/SILS roll gain ratio of 0.64 (65384/102334) in the SILS model-match check and the `torque_authority` of 0.60 from the 2026-10-02 flip-log fit, and plausibly explains a large part of that gap; it is not shown to explain all of it (single motor, $V_{DD}$ 4.2 to 4.4 V, no 4-motor battery voltage drop).
4. Hover anchoring: per-motor hover thrust is 0.0907 N (mass 0.037 kg) and the firmware hover voltage is 2.581 V. Re-deriving $\eta$ so that the plant hovers at that voltage:

| Variant | $\eta$ | hover $\omega$ [rad/s] | motor time constant at hover [ms] | plant yaw torque per firmware-commanded yaw torque | $\mathrm{d}T/\mathrm{d}V$ at hover [mN/V] |
|---|---|---|---|---|---|
| V0 current | 0.7133 | 3566 | 16.4 | 1.402 | 58.3 |
| V1 $C_T$, $C_Q$ only | 0.880 | 3682 | 17.3 | 1.136 | 59.5 |
| V2 full bench set ($K_e$, $R$, $Q_f=K_e\cdot0.044$, $C_Q$ including $K_e\cdot2.9\times10^{-9}$, $J=1.54\times10^{-8}$) | 0.941 | 3561 | 18.4 | 1.116 | 60.3 |

   The $\eta$ compensator applies to thrust only, so today the plant's yaw authority is 1.40 times what the firmware assumes. The bench motor reaches hover thrust at Duty×$V_{DD}$ = 2.44 V against the firmware's 2.58 V: the flight-anchored firmware needs about 7 % more thrust capability than the bench shows (cause not identified; candidates are mass, airframe interference and battery voltage drop under 4 motors).

## 4. SILS study

Run in an isolated git worktree (a working copy separate from the main tree).

### Variants

| Variant | Content |
|---|---|
| V0 | Baseline (current) |
| VF | Firmware-only re-gauging: `MOTOR_CT` 7.6e-9, `MOTOR_AM` ×0.76 = 4.587968e-8, `MOTOR_BM` ×$\sqrt{0.76}$ = 5.840089e-4, `KAPPA` 4.447e-3, `rate.yaw.kp` ×1.0847 = 8.710112e-4, `rate.yaw.max_torque` default 1.329872e-3 (table maximum 1.529461e-3). Mathematically the same thrust-to-voltage map and the same physical yaw thrust differential |
| V1 | VF + plant $C_T$ 7.60e-9, $C_Q$ 3.38e-11, $\eta$ 0.8802 |
| V2 | VF + plant with the full bench set, $\eta$ 0.9412 |

### Regression tests (`sf sils regression`, 44 scenarios)

For VF, V1 and V2 the parameter-consistency pre-check before the run had to be bypassed because the hand-maintained copies disagree with the parameter file (see the reproduction steps in chapter 6).

| Variant | PASS | FAIL | KNOWN-FAIL | XPASS | SKIP |
|---|---|---|---|---|---|
| V0 | 38 | 0 | 5 | 0 | 1 |
| VF | 38 | 0 | 5 | 0 | 1 |
| V1 | 37 | 1 | 4 | 1 | 1 |
| V2 | 36 | 2 | 4 | 1 | 1 |

(KNOWN-FAIL = scenario expected to fail; XPASS = a known failure that passed)

Verdict changes (each re-run twice, deterministic):

| Variant | Scenario | Content |
|---|---|---|
| V1 | `alt_inflight_switch` FAIL | alt_max 1.1155 m (limit < 1.1; V0 1.0511) |
| V2 | `commloss` FAIL | Landing never reaches IDLE_GROUND (cause not investigated) |
| V2 | `stab_flight` FAIL | att_rmse 0.0527 (limit < 0.05236; V0 0.0489) |
| V1, V2 | `calib` XPASS | The known 0.6 to 0.7 Hz self-oscillation failure passes. A lead for simulation-policy backlog #13, not a result |
| VF | No verdict change | Of 170 checked metrics 118 are identical and 52 differ, all inside thresholds (largest relative changes on near-zero values, e.g. `rc_flip` tilt_max 0.0064 to 0.0101 rad) |

### Model-match check (`sf sils sysid-gate`, default plant)

V0 and VF are the raw `gate.json` values; V1 and V2 are the corrected values of chapter 5 (`refit_b.txt`). Tolerances: $b$ ±50 %, $L_\text{total}$ ±20 %. Real hardware: roll $b$=65384, $L_\text{total}$=14.36 ms; pitch 30724, 17.29 ms; yaw 111897, 10.82 ms. **Every variant fails, V0 included.**

| Variant | roll $b$ | roll $L_\text{total}$ | pitch $b$ | pitch $L_\text{total}$ | yaw $b$ | yaw $L_\text{total}$ |
|---|---|---|---|---|---|---|
| V0 | +56.5 % | 16.86 ms (+17.4 %) | +211.3 % | 17.09 ms (−1.2 %) | (+74.6 %) | (0.58 ms, −94.7 %) |
| VF | +57.3 % | 16.92 ms (+17.8 %) | +212.1 % | 17.24 ms (−0.3 %) | (+75.3 %) | (0.31 ms, −97.1 %) |
| V1 | +67.0 % | 18.66 ms (+30.0 %) | +221.0 % | 18.16 ms (+5.0 %) | (+23.3 %) | (2.09 ms, −80.6 %) |
| V2 | +75.7 % | 20.22 ms (+40.8 %) | +226.2 % | 19.16 ms (+10.8 %) | (+67.9 %) | (0.84 ms, −92.3 %) |

The yaw numbers are unreliable in every variant (the 3-parameter fit degenerates and $L_\text{total}$ is near 0; known issue, backlog #11), hence the parentheses. For roll and pitch, the bench constants increase the motor lag (fitted time constant $T$ 15.2 ms to 17.3 / 18.9 ms) and move SILS further from the hardware.

## 5. Analysis-tool defect found

`sf sils sysid-gate` recovers torque from the logged duty by inverting the firmware chain (`tools/sysid/plant_fit.py`). There `_MOTOR_CT` and `_KAPPA` come from the generated parameters (they follow the yaml), but `_MOTOR_AM`, `_MOTOR_BM` and `_MOTOR_CM` are hand-typed literals that `sf params check` does not verify. Changing Ct in the yaml alone therefore scales the recovered thrust by Ct_new/Ct_old and inflates $b$ by Ct_old/Ct_new (1/0.76 = 1.32 here).

| Variant | Raw output (`gate.json`) | Refitted with Am/Bm matched to Ct (`refit_b.txt`) |
|---|---|---|
| V1 roll $b$ | 143702 (+119.8 %) | 109213 (+67.0 %) |
| V1 pitch $b$ | 129788 (+322.4 %) | 98639 (+221.0 %) |

Refitting the same recorded data leaves $T$ and $L$ unchanged, which confirms the cause. **Proposed fix (not done)**: add these three literals to `sf params check`, or derive them from the same source as `actuator.cpp`.

## 6. Decision and next steps (as of 2026-10-30)

| Item | Content |
|---|---|
| Plant | Not adopted. The model-match check gets worse (roll $b$ +56.5 to +67 / +76 %, $L_\text{total}$ +17 to +30 / +41 %) and the regression tests break (1 scenario for V1, 2 for V2) |
| Firmware re-gauging (VF) | Behaviour preservation verified in SILS. Not adoptable alone: with the yaml unchanged it fails `sf params check --strict` on 4 rows (C_T, kappa, Am_flight_anchored, Bm_flight_anchored). Adoption needs a decision on how the parameter file separates the firmware curve from the plant constants. On hardware it also needs `param reset` for `rate.yaw.*` |
| Next step | Model the Duty to average-motor-voltage relation of the board's drive circuit in the SILS plant, then re-evaluate $C_T$, $C_Q$, $J$ with the model-match check |
| Current sensing | Battery current was reported at 1/10 until commit d685c9fd; the bench data's current is "firmware reading ×10" |

### Reproduction

1. Apply `analysis/scripts/ct_cq_bench_20261015/patches/<V>.patch` to `main` commit 884f2af6 (or d685c9fd).
2. Run `sf params generate`, `sf sils build`, `sf sils regression`, `sf sils sysid-gate` in this order.
3. For V1 and V2 the parameter-consistency pre-check at the start of `sf sils regression` (`sf params check`) stops with an error because hand-maintained copies disagree with the parameter file; only this pre-check was bypassed (VF likewise).
4. Obtain the corrected $b$ by refitting the recording (`gate_bundle.sflog.zip`) written by `sysid-gate` with `analysis/scripts/ct_cq_bench_20261015/refit_gate_bundle.py <repo root> <recording> --am 4.587968e-8 --bm 5.840089e-4`.
