# プロペラ定数のベンチ実測（C_T / C_Q）を SILS とファームに採用するかの検討 (2026-10-15 実測、2026-10-30 検討)

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

**状態**: 検討の続き（2026-10-30）。C_T・C_Q だけの差し替えは採用見送り。駆動回路の平均モデル（$R_\text{out}$）は宙返りと電流の照合で支持された。電池電圧への依存の食い違いは、2026-11-02 のベンチ（$R_\text{out}$ は電池電圧によらず、一定の電圧降下 $V_b$ を加える）とベンチで実測した回転数の表で解消し、全機の推力を 10/15 の 1 モータの 4 倍とすると飛行のホバーと ±3 % で合う（6 章）。作成 2026-10-30、最終更新 2026-11-02。

## 1. 目的と結論

### 問い

2026-10-15 に 1 モータのベンチで得た推進系の実測値を、暫定採用の $C_T=1.00\times10^{-8}$ ・$C_Q=4.10\times10^{-11}$（`control/models/stampfly_physical.yaml` の `measured_2026_07`。SILS（Software-In-the-Loop Simulation、ファームのソースをそのまま PC 上で動かすシミュレーション）のプラントとファームが使う）と置き換えるべきか。

### 結論（2026-10-30 時点）

| 項目 | 結論 | 主な理由 |
|---|---|---|
| $C_T$・$C_Q$ だけの SILS プラントへの差し替え（駆動部分は Duty × 電圧のまま） | **採らない。$C_T$・$C_Q$ は駆動回路の平均モデルの一式として入れる** | 駆動部分を替えずに定数だけ替えても、宙返り・電流の照合で問題になる駆動部分の形が残る。当初の理由に挙げたモデル一致の確認（`sf sils sysid-gate`）の悪化（ロール $b$ +56.5 % → +67 / +76 %、$L_\text{total}$ +17 % → +30 / +41 %）は、6 章でこの確認がモータモデルを判別できないと分かったため、根拠にならない。再確認試験（変更で既存の動作が壊れていないかを自動で確かめる試験。`sf sils regression` の 44 シナリオ）では V1 で 1 件、V2 で 2 件が不合格になった |
| 駆動回路の平均モデル（$R_\text{out}$、開発者らの論文） | **宙返りと電流の照合で支持。電池電圧への依存の食い違いは、2026-11-02 に解消（6 章「全機の推力とホバーの釣り合い」）** | 合わせ込みなしで、宙返りの角加速度の残差は 0.29（現行モデルは 0.98〜1.02）、飛行中の電池電流は実測の 5〜6 % 以内（5 章・6 章）。ホバリング Duty の電池電圧への依存だけは予測より弱く出た |
| ファーム側だけの換算替え（VF） | **単独では採用不可（変更なし）** | SILS 上の挙動は変わらないことを確認したが、yaml を変えないと `sf params check --strict` が 4 行で不一致になる。パラメータ基準ファイルでファームの曲線とプラントの定数組を分ける方針の決定が先に要る |
| 次の手 | 8 章の決定 | 物理の実測を SILS より上位に置く。$R_\text{out}$ モデルを公開する。モデル一致の確認を作り直す。電池電圧依存の実験は方法を検討する |

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

**注意（2026-11-02 追記）**: 2026-11-02 夜に同じプロペラ・同じ高さで測り直すと、推力はどの回転数でも上の値の 0.961 倍だった（原点を通る $C_T$ は $7.27\times10^{-9}$）。空気の密度では説明できず（差は最大 0.7 %）、プロペラの下（機体は逆さまなので吸い込み側）のプローブ・配線が 10/15 より多かった可能性がある（未確認）。このベンチでは吸い込み側の障害物は推力を小さく見せる向きにしか効かないので、上の 10/15 の値を基準とし、全機の推力はその 4 倍とする（実験者の判断）。2026-11-02 の 4 モータ同時の推力は、ほかのプロペラと缶の位置関係が違うので、全機の推力の測定値としては使わない。$F/\omega^2$ は 2 日とも回転数とともに大きくなり（$\omega$ 1200 → 4700 rad/s で約 20 %）、$C_T$ を一定とは見なせない。2 日とも $F=k\,\omega^n$（$n$ = 2.14〜2.15）がよく合い、上の値はホバー付近を基準に $F=F_\text{ref}(\omega/\omega_\text{ref})^n$、$\omega_\text{ref}=3500$ rad/s、$F_\text{ref}=0.0917$ N、$n=2.15$ と書ける。ホバー付近の傾き $dF/d\omega=nF/\omega$ は、$C_T$ 一定の $2F/\omega$ より 7 % 大きい。$C_Q$ は $3.33\times10^{-11}$ で、上の値と 1.5 % の差。SILS とファームへの採用の扱いは 1 章の結論のまま（飛行ログとの突き合わせが先）。ケーブルを秤のトレイと机の両方にテープで固定する留め方は、秤のゼロを −3 g 前後ずらすだけで、推力の読みは変えないことを同じ夜に確かめた。同じ日に一度ここへ書いた「留め方で $C_T$ がずれた可能性」は誤りだった（その日の推力のずれは、秤のマイナス表示の読み落としが原因）。$K_e$・$R$・$C_Q$・$J$・$I_0$ は秤を使わない。

**注意**: このデータの電池電流は「ファームの読み値の 10 倍」であり、独立した電流計測ではない（電流検出抵抗の誤設定をコミット d685c9fd で修正した。それ以前は 1/10 の値を報告していた）。

### 関連ファイル

| パス | 内容 |
|---|---|
| `analysis/scripts/ct_cq_bench_20261015/patches/VF.patch`, `V1.patch`, `V2.patch` | 検討に使った変更（4 章の比較条件）。いずれも `main` のコミット 884f2af6（検討時の作業ツリーの基点）に対する差分で、d685c9fd にもそのまま適用できることを確認済み |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/regression.json` | `sf sils regression` の結果（V0, VF, V1, V2）。V1・V2 の `fail` 総数には、バイパスしたパラメータ整合の事前検査が 1 件加算されている。シナリオの不合格数は V1=1、V2=2 |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/gate.json` | `sf sils sysid-gate` の生の出力（V0, VF, V1, V2） |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/refit_a.txt`, `refit_b.txt` | 同じ記録を `plant_fit.py` の定数のままで（a）、または Am/Bm を Ct に合わせて（b）再同定した結果（V0, V1, V2）。7 章参照。V0 の b は Ct を変えずに Am/Bm だけ 0.76 倍にした不整合な条件で、$b$ が約 0.76 倍に下がるだけ（逆方向の確認）であり、基準値には使わない |
| `analysis/scripts/ct_cq_bench_20261015/bench_vs_models.py` | ベンチと、ファームの静的曲線・SILS プラント定常状態との比較（3 章の表を出力） |
| `analysis/scripts/ct_cq_bench_20261015/refit_gate_bundle.py` | `sysid-gate` の記録一式（`.sflog.zip`）を、`plant_fit.py` の定数を差し替えて再同定する |
| `analysis/scripts/ct_cq_bench_20261015/rout/` | $R_\text{out}(D)$ の表、SILS プラントの変更（`R1.patch`、`drive_rout_table.hpp`）と README（5 章） |
| `analysis/scripts/ct_cq_bench_20261015/crosscheck/` | 実機ログとの照合のスクリプトと結果、README（6 章）。入力ログはリポジトリに無い |

再現: `python3 -I analysis/scripts/ct_cq_bench_20261015/bench_vs_models.py <stampfly-motor-autotest の data/2026-10-15/summary.json>`

## 3. 定常特性の比較

### 比較したモデル

| モデル | 内容 |
|---|---|
| ファームの静的曲線 | `actuator.cpp` の `thrustToDuty`: $\omega=\sqrt{T/C_t}$、$V=A_m\omega^2+B_m\omega+C_m$、duty $=V/v_\text{bat}$ |
| SILS プラントのモータ常微分方程式（ODE）の定常解 | 推力 $T=\eta\,C_t\,\omega^2$（$\eta$ = `thrust_efficiency` = 0.7133） |

### 分かったこと

1. 現行の SILS プラント（$\eta=0.7133$）とファームの曲線は、どの Duty でも同じ推力を出す（2026-08-22 に等価にした）。
2. Duty × 電池電圧で駆動すると、両モデルとも低 Duty でベンチ推力を下回り（Duty 0.1 で 0.37 g 対 0.93 g）、ホバー付近では近い。**測定したモータ平均電圧 $\langle V_m\rangle$ で駆動すると、ベンチ一式の定数を入れたプラント ODE はベンチ推力を0.35 g 以内（ホバー付近の Duty 0.5〜0.7 では 0.2〜2.7 %）で再現する。** ベンチの定数同士は矛盾しておらず、ずれの所在は Duty → $\langle V_m\rangle$ の関係にある。SILS プラントはこれを $V=\text{Duty}\times v_\text{batt}$ と仮定している。なお、この比較の Duty はファームと SILS が受け取る指令値である（ベンチで波形から測った Duty は指令より 0.006〜0.015 大きい）。本ノートの初版（2026-10-30）は測定 Duty で比較しており、同日中に指令 Duty へ訂正した。
3. 区間ごとの傾き（推力 対 Duty×$V_{DD}$、および $\langle V_m\rangle$ 対 Duty×$V_{DD}$。$V_{DD}$ は計測時の電池電圧）:

| Duty 区間 | ベンチ [g/V] | モデル [g/V] | 比 | $\mathrm{d}\langle V_m\rangle/\mathrm{d}(\text{Duty}\cdot V_{DD})$ |
|---|---|---|---|---|
| 0.1→0.2 | 3.51 | 2.31 | 1.52 | 1.07 |
| 0.2→0.3 | 4.05 | 3.43 | 1.18 | 0.88 |
| 0.3→0.4 | 4.53 | 4.32 | 1.05 | 0.83 |
| 0.4→0.5 | 4.54 | 5.05 | 0.90 | 0.78 |
| 0.5→0.6 | 4.98 | 5.65 | 0.88 | 0.73 |
| 0.6→0.7 | 3.67 | 6.17 | 0.59 | 0.52 |
| 0.7→0.8 | 5.85 | 6.61 | 0.88 | 0.75 |
| 0.8→0.9 | 11.14 | 6.99 | 1.59 | 1.43 |

   ホバー付近（Duty 0.4〜0.8）では、実際の Duty 当たり推力ゲイン（傾き）はモデルの 0.59〜0.90 倍である。SILS のモデル一致の確認で出た実機/SILS のロールゲイン比 0.64（65384/102334）、および 2026-10-02 の宙返りログのフィットによる `torque_authority` 0.60 と同じ向きで、この差の大きな部分を説明する可能性がある。ただし全部を説明するとは示していない（1 モータ、$V_{DD}$ 4.2〜4.4 V、4 モータ同時の電池電圧低下なし）。
4. ホバー点で合わせた $\eta$: 1 モータ当たりのホバー推力は 0.0907 N（質量 0.037 kg）、ファームのホバー電圧は 2.581 V。プラントがこの電圧でホバーするよう $\eta$ を求め直すと次のとおり。

| 条件 | $\eta$ | ホバー $\omega$ [rad/s] | ホバー時のモータ時定数 [ms] | ファームが指令したヨートルクに対するプラントのヨートルク | ホバーでの $\mathrm{d}T/\mathrm{d}V$ [mN/V] |
|---|---|---|---|---|---|
| V0 現行 | 0.7133 | 3566 | 16.4 | 1.402 | 58.3 |
| V1 $C_T$, $C_Q$ のみ | 0.880 | 3682 | 17.3 | 1.136 | 59.5 |
| V2 ベンチ一式（$K_e$, $R$, $Q_f=K_e\cdot0.044$, $C_Q$ に $K_e\cdot2.9\times10^{-9}$ を含める, $J=1.54\times10^{-8}$） | 0.941 | 3561 | 18.4 | 1.116 | 60.3 |

   $\eta$ の補正は推力にだけ掛かるため、現状のプラントのヨー権限はファームの想定の 1.40 倍になっている。ベンチの 1 モータがホバー推力に達するのは Duty×$V_{DD}$ = 2.39 V で、ファームの 2.58 V より小さい。つまり飛行に合わせ込んだファームの曲線は、ベンチより約 8 % 多い推力能力を要求している（原因は未特定。質量、機体の空力干渉、4 モータ時の電池電圧低下が候補）。**2026-11-02 追記:** この差は、電圧の違う条件（ベンチ 4.2〜4.4 V、飛行 3.6〜4.1 V）を Duty × 電圧が一定とみなして比べたことによると考える。ベンチで実測した回転数の表を使うと、4 × 10/15 の推力はホバーの重さと ±3 % で合う（6 章「全機の推力とホバーの釣り合い」）。

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

VF・V1・V2 では、手書きの写しが基準ファイルと食い違うため、実行前のパラメータ整合の事前検査をバイパスして実行した（8 章の再現手順を参照）。

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

V0 と VF は `gate.json` の生の値、V1 と V2 は 7 章の修正後の値（`refit_b.txt`）。許容は $b$ ±50 %、$L_\text{total}$ ±20 %。実機は ロール $b$=65384・$L_\text{total}$=14.36 ms、ピッチ 30724・17.29 ms、ヨー 111897・10.82 ms。**どの条件も不合格で、V0 も含む。**

| 条件 | ロール $b$ | ロール $L_\text{total}$ | ピッチ $b$ | ピッチ $L_\text{total}$ | ヨー $b$ | ヨー $L_\text{total}$ |
|---|---|---|---|---|---|---|
| V0 | +56.5 % | 16.86 ms (+17.4 %) | +211.3 % | 17.09 ms (−1.2 %) | (+74.6 %) | (0.58 ms, −94.7 %) |
| VF | +57.3 % | 16.92 ms (+17.8 %) | +212.1 % | 17.24 ms (−0.3 %) | (+75.3 %) | (0.31 ms, −97.1 %) |
| V1 | +67.0 % | 18.66 ms (+30.0 %) | +221.0 % | 18.16 ms (+5.0 %) | (+23.3 %) | (2.09 ms, −80.6 %) |
| V2 | +75.7 % | 20.22 ms (+40.8 %) | +226.2 % | 19.16 ms (+10.8 %) | (+67.9 %) | (0.84 ms, −92.3 %) |

ヨーの数値は、どの条件でも信頼できない（3 パラメータのフィットが縮退し、$L_\text{total}$ が 0 に近い。既知の課題でバックログ #11）ため括弧書きにした。ロール・ピッチでは、ベンチ定数がモータの遅れを増やし（フィットした時定数 $T$ が 15.2 ms → 17.3 / 18.9 ms）、SILS を実機からさらに遠ざける。

## 5. 駆動回路の平均モデルを入れた SILS

### 平均モデルとは

開発者らの論文（査読対応中、StampFly のコンデンサ還流駆動回路の解析）は、StampFly の駆動回路（モータに並列のコンデンサ $C_C$ で還流する構成）を 1 周期で平均し、電池電圧と逆起電力の間に等価出力抵抗 $R_\text{out}$ が入った形で表している。$R_\text{out}$ は Duty と $C_C$ で決まり、電池電圧と回転数にはよらない。SILS の「端子電圧の平均 = Duty × 電池電圧」とは形が違う。

$$\bar I_m=\frac{V_\text{BAT}-K_e\,\omega}{R_\text{out}(D,\,C_C)},\qquad J\frac{d\omega}{dt}=K_t\,\bar I_m-C_Q\,\omega^2-K_t\,I_0(\omega),\qquad T=\eta\,C_T\,\omega^2$$

（$\bar I_m$ は平均のモータ電流 [A]、$V_\text{BAT}$ は電池電圧 [V]、$D$ は Duty、$I_0(\omega)$ は無負荷電流 [A]、$\eta$ は推力効率係数 `thrust_efficiency`）

基板の $C_C = 0.1\,\mu$F での $R_\text{out}$ は、$D$ 0.1 で 26.0 Ω、0.3 で 6.68 Ω、0.5 で 3.15 Ω、0.7 で 1.88 Ω、0.9 で 0.97 Ω、1.0 で 0.66 Ω。0.01 刻みの表（`analysis/scripts/ct_cq_bench_20261015/rout/rout_table_c01.csv`）を SILS のプラントに入れた（変更は同じ場所の `R1.patch` と `drive_rout_table.hpp`。本リポジトリには適用していない）。

定数は開発者らの論文の表（10/15 のベンチ値）。ファームは現行のまま（モデル一致の確認の逆算もずれない）。$\eta$ は 3.8 V でファームのホバリング Duty に合わせた 0.8975（R1）と、ベンチそのままの 1.0（R1-η1）の 2 通りで回した。

### ホバリングでのゲイン（Duty 1 あたりの推力変化）

| 電池電圧 | $R_\text{out}$ モデル [mN] | 現行 SILS [mN] | 比 |
|---|---|---|---|
| 3.6 V | 117 | 210 | 0.56 |
| 3.8 V | 158 | 221 | 0.71 |
| 4.0 V | 185 | 233 | 0.79 |
| 4.2 V | 203 | 245 | 0.83 |

飛行中の電池電圧では 0.56〜0.71 倍で、モデル一致の確認での実機/SILS のゲイン比 0.64、宙返りの同定で入れた補正係数 `torque_authority` 0.60 と同じ範囲にある。

### SILS の結果

| | V0 現行 | R1（$\eta$ 0.8975） | R1-η1（$\eta$ 1.0） |
|---|---|---|---|
| ロール $b$（実機との差） | +56.5 % | −17.6 % | +28.7 % |
| ピッチ $b$ | +211.3 % | +8.2 % | +84.8 % |
| ロール 遅れの合計（実機 14.4 ms） | 16.9 ms | 37.7 ms | 39.6 ms |
| ピッチ 遅れの合計（実機 17.3 ms） | 17.1 ms | 36.0 ms | 39.8 ms |
| 再確認試験（44 シナリオ） | 合格 38 | 合格 32・不合格 6 | 合格 18・不合格 20 |
| 既知の不合格が合格に | — | 3 | 0 |
| ホバリング Duty（3.6 / 3.8 / 4.2 V） | 0.717 / 0.679 / 0.614 | 0.757 / 0.679 / 0.585 | 0.654 / 0.603 / 0.533 |

| 項目 | 内容 |
|---|---|
| ゲイン | ノイズのない SILS では、R1 のロール・ピッチの $b$ が補正係数なしで許容（±50 %）に入った。ただし 6 章の較正では、振動ノイズがあると $b$ も機体の違いを判別できない。判別できたのは宙返りの照合のほうで、今の `torque_authority` 0.6 の中身はこの駆動回路の効果と考えられる |
| 遅れ | $R_\text{out}$ モデルのホバリングでの時定数は 38.8 ms（論文の値。論文自身が「応答は測っていない」と書いている）。ノイズのない SILS の同定では $T$ が 36.5 ms と出て、実機の基準値（遅れの合計 14〜17 ms）と合わないように見えた。しかし 6 章の較正で、実機と同じく振動ノイズがある条件では、時定数 16 ms の機体も 38 ms の機体も「遅れの合計 13〜14 ms」と同定されることが分かった。この基準値ではどちらとも決められない |
| 再確認試験 | R1 で新たに不合格になったのは、自動離陸の高度のばらつき 2 件、宙返り関係 3 件（うち 2 件は宙返り要求がファームの判定で拒否されて始まらない）、墜落後の再飛行 1 件。原因はまだ調べていない。シナリオの基準が旧モデルに合わせてあるのか、実機でもファームが同じ判断をするのかで意味が変わるので、既定にする前に調べる。逆に、既知の不合格の `calib`（自励振動）、`pos_flight`・`pos_yaw`（Duty が 1.0 に張り付いていたのが最大 0.72 に）が合格した。$\eta$ = 1 で 20 件不合格になるのは、ファームのホバリングの想定より推力が約 11 % 多いためと見られる（未検証） |
| 宙返りの判別 | 宙返りの同定済み設定から補正係数 0.6 を外しても、現行モデル・$R_\text{out}$ モデルのどちらでも宙返りは合格し、決め手にならなかった |
| 電池電圧への依存 | $R_\text{out}$ モデルでは、ホバリングに要る Duty が電池電圧に対してファームの想定（Duty ∝ 1/電圧）より急に変わる（3.6 V で +5.6 %、4.2 V で −4.7 %）。電池が減るにつれて、高度制御の積分器と推力の学習が吸収する量が増える、という予測になる |

実機ログとの照合は次の 6 章にまとめた。

## 6. 実機ログとの照合

方針は「ベンチ試験と開発者らの論文の式を基準とし、実機ログと突き合わせて確かめる」。各照合で、モデルが誤りならどう見えるかを先に決めてから計算した。使ったログは 2026-06-14〜09-08 の飛行 14 本と、別の PC に保存されていた 10/2 の宙返り 2 本・地上でモータを回したログ 2 本（400 Hz の Duty 付き）。プロペラは 2025 年に交換済みで、どのログも同じプロペラ。ログはこのリポジトリに無い。スクリプトと結果は `analysis/scripts/ct_cq_bench_20261015/crosscheck/`（その README に、入力ログのファイル名と各表を出すスクリプトの対応がある）。

### 同定手順の較正

実機の基準値（6/14）は、加振なしの自然な揺れから、ファームの PID 出力を計算し直して同定したもの。同じ手順を、時定数が分かっている SILS に当てた（`crosscheck/calfit.py`、`crosscheck/sysid_gate_small.scn`、結果は `crosscheck/results/sils_cal/`）。

| 条件（ロール） | 真の時定数 16 ms（現行） | 真の時定数 38 ms（$R_\text{out}$） |
|---|---|---|
| ノイズなし | T 15 + L 2 ms | T 36 + L 1 ms |
| センサノイズのみ | T 14 + L 2 ms | T 36 + L 1 ms |
| 振動ノイズあり | T 0 + L 13〜14 ms | T 0 + L 13〜14 ms |
| 実機の基準値（6/14） | T 0 + L 14.7 ms（コヒーレンス 0.65。入力と出力の信号の相関の強さで、0〜1） | 同左 |

（T は一次遅れの時定数、L はむだ時間）振動ノイズがあると、16 ms と 38 ms の機体が同じ形に同定され、実機の基準値もその形をしている。この方法ではモータの時定数を判別できない。ゲイン $b$ も同様で、ノイズありでは両方の機体が実機と同じ程度の値（ロール 61〜69k、実機 64k）になる。飛行ログ 14 本で同じ同定をしても、遅れの合計はロール 13〜19 ms で、判別の材料にならなかった（`crosscheck/logcheck/test3.py`）。

### 宙返り（大信号）

記録した 400 Hz の Duty と電池電圧を各モデルに通して、ロール角加速度を予測した。合わせ込みを一切しない予測（倍率 1、遅れ 0）と、倍率と追加の遅れを最適にした予測を比べた（`crosscheck/flipcheck/flipcheck.py`、結果は `crosscheck/results/flipcheck_results.json`）。実測（ジャイロの微分）と、現行モデル・$R_\text{out}$ モデル（ベンチ定数、$\eta$ = 1）の予測の重ね描きのデータは `crosscheck/flip_overlay.py` が `flip_overlay.json` に出す（図は本ノートに載せていない。flip_test1 は 10/2 の右ロールの宙返り）。

| モデル | 合わせ込みなしの残差 | 最適な倍率 $s$ | 最適な追加の遅れ | そのときの残差 |
|---|---|---|---|---|
| 現行モデル | 0.98〜1.02 | 0.54〜0.56 | 13〜17 ms | 0.15〜0.21 |
| $R_\text{out}$ モデル（$\eta$ = 1） | 0.29 | 0.79〜0.80 | 3〜4 ms | 0.09〜0.10 |

- 残差は、予測と実測の差の二乗平均平方根を、実測の二乗平均平方根で割った値。2 本の宙返りで同じ結果になった。
- $R_\text{out}$ モデルは、記録と通信の遅れ程度（3〜4 ms）の追加だけで合う。現行モデルは、効きを半分にして 13〜17 ms 遅らせないと合わない。宙返り用の補正（`torque_authority` 0.6、遅れ 6 ms、Duty の変化率制限）が埋めていたのは、この差と読める。
- 同じログのホバリング部分でも、追加の遅れは $R_\text{out}$ モデル 4〜8 ms、現行モデル 10〜15 ms。大信号と小信号で遅れが違って見えていた件（SILS 方針のバックログ #17(c)）は、$R_\text{out}$ モデルなら同じ小さい遅れで説明できる。
- 倍率 0.79〜0.80 のうち、1 に届かない分の候補は次のとおり。(1) 宙返り中の電池電圧の低下（ログされる電圧は 1 Hz だけで、宙返りの数百 ms の間の低下は見えない）。(2) ホバリングの釣り合いから求めた $\eta$ 0.94〜0.95（機体質量 0.037 kg で計算。2 本吊り法の測定時の質量は 0.0364 kg。論文の表の $R_\text{out}$ による値で、ベンチで実測した回転数の表では 0.97〜1.01 になる。下の「全機の推力とホバーの釣り合い」）。(3) そのほかのモデル誤差。慣性モーメントは 2 本吊り法（二本の糸で吊ってねじり振動の周期を測る方法）の実測で、記録は `control/models/stampfly_physical.yaml` にある。記録から計算すると $I_{xx}$ = 9.19e-6 kg·m² で、採用値 9.16e-6 との差は 0.3 % と小さく、ここでは主な候補ではない。

### 電池電流

$R_\text{out}$ モデルでは、平均の電池電流はモータ電流の平均に等しい。単純な平均モデルでは「Duty × モータ電流」になり、両者は約 1.5 倍違う。記録された電流（×10 で真値）から、待機中の電流（146〜200 mA）を引いて比べた（`crosscheck/logcheck/test2.py`）。

| ログ | 実測 ÷ $R_\text{out}$ モデル | 実測 ÷ 単純な平均モデル | 実測 ÷ 現行 SILS |
|---|---|---|---|
| 7/17 ホバリング（5 本、191 点） | 0.94 | 1.68 | 0.89 |
| 7/18 ホバリング（2 本、97 点） | 0.95 | 1.72 | 0.90 |
| 9/8 ホバリング | 0.95 | 1.39 | 0.84 |
| 10/2 地上でモータを回したログ（2 本） | 0.99〜1.01 | — | — |

単純な平均モデルは否定され、$R_\text{out}$ モデルはベンチでの精度（±8 %）の範囲に入った。×10 は回路図のシャント抵抗 R12 = 0.01 Ω とファームの想定 0.1 Ω の比で、エネルギー収支（7.6 倍以上が必要）とも整合する。残る不確かさはシャント抵抗の部品精度（数 %）程度。

### ホバリング Duty と電池電圧

$R_\text{out}$ モデルでは、電池電圧が下がるとホバリングに要る Duty が急に増え、Duty × 電圧が電圧とともに変わる（この電圧範囲で傾き約 −0.3〜−0.4）。単純な平均モデルとファームの想定では Duty × 電圧は一定（傾き 0）（`crosscheck/logcheck/test1.py`）。

- 電圧の幅がいちばん広いログ（6/27、0.42 V の幅）で、傾きは −0.20 ± 0.05。どちらの予測とも違い、その中間に出た。当てはまりは一定の想定のほうが良い。
- ほかの日付は電圧の幅が狭く（0.2 V 前後以下）、判別できなかった。
- 3 分間のホバリングでは電圧の低下と時間の経過が重なり、巻線の温度上昇と分けられない。ただ温度上昇は $R_\text{out}$ モデルと同じ向きに効くので、差し引くと $R_\text{out}$ モデルとの差はかえって広がる。
- これは、開発者らの論文の前提「$R_\text{out}$ は電池電圧によらない」と食い違う、実機ログで初めての結果である。10/15 のベンチは電池電圧 4.17〜4.38 V だけで、この前提を確かめた範囲は狭い。
- 影響: $R_\text{out}$ の表から推力 → Duty の変換を作ると（論文 5.3 節の提案）、電池が減るにつれてホバリングの見込みが数 % ずれるおそれがある。ファームへの採用は、電池電圧への依存を確かめる実験の後にする（実験の方針は 8 章）。論文の改訂でも「電圧によらないことは 4.2〜4.4 V で確かめた」と範囲を書いておく価値がある。
- **2026-11-02 追記:** 11/02 の長時間放電のベンチ（負荷時の電池電圧 4.27 → 3.63 V、冷えた状態どうし）で、$R_\text{out}$ の変化は 0〜−1.1 % で、電池電圧によらないと確かめた。ただし、どの Duty でも一定の電圧降下 $V_b \approx 0.11$ V（ブラシの接触に相当）を加えた式 $\bar I_m = (V_{DD} - K_e\omega - V_b)/R_\text{out}(D)$ が合う。ここで見た食い違いは、論文の表の $R_\text{out}$（D0.3〜0.7 で実測より約 10 % 小さい）と $V_b$ を入れない計算によるもので、ベンチで実測した回転数の表を使うと小さい（下の「全機の推力とホバーの釣り合い」）。

### ベンチで実測した $R_\text{out}$

各段の実測値から $R_\text{out} = (V_{DD} - K_e\,\omega)/I_\text{bat}$ を求め、論文の計算値と比べた（$V_{DD}$ はオシロで基板の近くで測った値、$I_\text{bat}$ はファームの読み値 × 10、指令 Duty）。

| Duty | $V_{DD}$ [V] | 実測 [Ω] | 計算値 [Ω] | 実測 ÷ 計算 |
|---|---|---|---|---|
| 0.1 | 4.378 | 26.19 | 25.97 | 1.01 |
| 0.2 | 4.364 | 12.07 | 11.55 | 1.05 |
| 0.3 | 4.349 | 7.33 | 6.68 | 1.10 |
| 0.4 | 4.328 | 4.88 | 4.45 | 1.10 |
| 0.5 | 4.302 | 3.479 | 3.152 | 1.10 |
| 0.5（2 回目） | 4.236 | 3.463 | 3.152 | 1.10 |
| 0.6 | 4.278 | 2.563 | 2.326 | 1.10 |
| 0.7 | 4.253 | 2.072 | 1.882 | 1.10 |
| 0.8 | 4.223 | 1.547 | 1.547 | 1.00 |
| 0.9 | 4.173 | 0.941 | 0.970 | 0.97 |

- Duty 0.5 の 2 回は $V_{DD}$ が 1.5 % 違うが、実測の $R_\text{out}$ の違いは 0.5 %。範囲は狭いが「$R_\text{out}$ は電池電圧によらない」と矛盾しない。
- Duty 0.3〜0.7 では実測が計算値より約 10 % 大きい。Duty 0.1・0.8・0.9 ではほぼ一致するので、全 Duty に一様に掛かるずれではなく、計算値の形のずれである。
- ホバリング付近で $R_\text{out}$ が 10 % 大きいと、推力は約 7 % 小さくなる（線形化による見積もり）。飛行ログのホバリングの釣り合いで要った $\eta$ 0.93〜0.95 と、電流の比 0.94〜0.95 の大部分は、これで説明できる可能性がある。
- 物理の実測を上位に置く方針（8 章）では、SILS には計算値ではなく実測の $R_\text{out}(D)$ を基準として入れる。実測は Duty 0.1 刻みの 9 点なので、間は計算値の形を実測に合わせて補う。

### 全機の推力とホバーの釣り合い（2026-11-02 追記）

2026-11-02 に、全機の推力を 10/15 の 1 モータの推力の 4 倍とすることにした（2 章の注意）。これを飛行ログのホバーで確かめた（`crosscheck/logcheck/test4.py`、結果は `crosscheck/results/test4.json` と `test4_windows.csv`）。各ホバー区間（5 秒、134 区間・計 574 秒）で、記録された 4 つの Duty と電池電圧から、2026-11-02 のベンチで実測した $\omega(D, V)$ の表で回転数を出し、10/15 のべき乗の式 $F = k\,\omega^{n}$（$n = 2.148$）で推力にして、重さ $mg$ と比べた。主張が誤り（飛行中に干渉で推力を失う）なら、比 $\sum F/(mg)$ ははっきり 1 を超える。

| 道 | 内容 |
|---|---|
| 表 A | 4 モータ同時の M1。記録と同じ位置の電圧（CLI）で引く |
| 表 B | 1 モータ。駆動回路の $V_{DD}$ で引く（$V_{DD}$ = 記録電圧 − 4 モータ時の CLI と $V_{DD}$ の差 9〜80 mV） |
| $\omega$ を使わない道 | 11/02 の 1 モータの推力（10/15 の 0.961 倍の条件）を $(D, V_{DD})$ で直接回帰 |

| 推力 ÷ 重さ（時間で重み付けした平均） | 質量 37 g | 質量 36 g |
|---|---|---|
| 表 A と 10/15 の式 | 0.986 | 1.013 |
| 表 B と 10/15 の式 | 1.006 | 1.034 |
| $\omega$ を使わない道（11/02 の推力） | 0.981 | 1.008 |

- 質量は開発者の見積もりで 36〜37 g。飛行ログは同じ形状の機体・プロペラだが、ベンチの機体とは別の個体（複数）。モータの PWM 周波数は、ログの期間のファームもベンチと同じ 150 kHz。
- 10/15 の推力の 4 倍は、ホバーの重さと ±3 % 以内で合う。飛行中のプロペラどうし・機体との干渉で、推力を 5 % 以上失っている様子はない。$\omega$ を使わない道は、$\omega$ を通した 11/02 の値（表 B で 0.966）と 1.5 % で一致した。
- $\omega$ の表は 11/02 のベンチ（プロペラの下にプローブがあり、同じ $\omega$ でトルクが 1〜4 % 大きかった条件）の値なので、$\omega$ を 0.5〜1.5 % 低く見ている可能性がある。質量の幅（2.7 %）と合わせると、この照合では 10/15 と 11/02 の推力（3.9 % の差）を区別できない。10/15 を採る理由は 2 章の注意のとおり。
- 運転ごとの比の中央値は 0.88（7/18 06:42）〜1.07（6/29）で、モデルの不確かさより大きい。個体ごとの違い（重さ・モータ・プロペラ）が候補。11/02 の 4 モータ同時のベンチでも、M1 より 4〜8 % 遅く回るモータがあった（電源電圧のスペクトルの線）。
- 1 回の飛行の中での比の電圧への傾きは、表 B で −0.44〜+0.13 /V（電圧の幅が 0.15 V 以上の 10 本のうち 7 本は +0.02〜+0.13 /V で、0.5 V で 1〜7 %）、表 A で −0.42〜+0.21 /V。電圧の低下とモータの温まり（$K_e$ が数分で 2〜4 % 下がる）が重なる。上の「ホバリング Duty と電池電圧」で見た、論文の表の $R_\text{out}$ による予測との食い違いは、ベンチで実測した表では残らない。

#### ホバー Duty の比較

同じ照合を Duty で見た（`test4.json` の `hover_duty_by_voltage`）。記録電圧 0.1 V ごとに、飛行のホバー Duty（4 モータの平均の中央値）と、ベンチから求めた「重さ 36〜37 g を支える Duty」（4 モータ同じ Duty）を並べる。

| 記録電圧 | 飛行（区間数・運転数） | 表 A と 10/15 の式 | 表 B と 10/15 の式 | 11/02 の推力を直接 |
|---|---|---|---|---|
| 3.5 V | 0.731（8・1） | 0.746〜0.763 | 0.716〜0.733 | 0.739〜0.756 |
| 3.6 V | 0.705（11・3） | 0.713〜0.729 | 0.690〜0.706 | 0.709〜0.726 |
| 3.7 V | 0.690（23・8） | 0.686〜0.702 | 0.668〜0.684 | 0.685〜0.701 |
| 3.8 V | 0.662（19・6） | 0.651〜0.666 | 0.638〜0.654 | 0.652〜0.668 |
| 3.9 V | 0.636（25・11） | 0.626〜0.640 | 0.616〜0.631 | 0.629〜0.644 |
| 4.0 V | 0.616（37・9） | 0.602〜0.615 | 0.595〜0.609 | 0.607〜0.622 |
| 4.1 V | 0.599（11・8） | 0.588〜0.601 | 0.582〜0.597 | 0.595〜0.609 |

- 時間で重み付けした平均の差（飛行 − ベンチ）は −1.9〜+2.8 %（道と質量による）。運転ごとの中央値は ±0.05 ほどばらつく（別の個体）。
- 記録電圧に対する Duty の傾き（全区間）は、飛行 −0.242 /V に対し、表 B −0.24、11/02 の推力を直接 −0.26、表 A −0.29 /V。Duty × 電圧を一定とする想定（ファームの換算）では −0.168 /V で、電圧依存が緩すぎる。
- 1 回の飛行の中（電圧の幅 0.3 V 以上の 4 本）では、飛行 −0.16〜−0.24 /V、表 B −0.23〜−0.26 /V で、飛行のほうがやや緩い。ベンチでは、同じ Duty・電圧での回転数の差は冷えた周と温まった周で ±0.6 % 以内なので、モータの温まりでは説明しきれない。原因は未特定。
- 2026-11-02 の 4 モータ同時を缶の上で秤で測った値では、37 g を支える Duty は A4M で 0.726（記録電圧で約 3.88 V）、B4M で 0.764（約 3.69 V）だった。同じ電圧の飛行（0.64、0.69）より 0.07〜0.09 大きく、飛行のホバー Duty での秤の読みは約 34 g（重さより 6〜8 % 小さい）。この測定を全機の推力として使わない判断（2 章の注意）と合う。

#### 1 モータと 4 モータ同時の回転数（オンの間の電圧）

4 つのモータは同じタイマで同時にオンになる。オンの間は 4 つ分のパルス電流（2026-11-02 で 4.1〜5.0 A）が電池と配線を流れ、M1 の $V_{DD}$ は平均より余計に下がる（ベンチのリポジトリの `tools/vdd_on_time.py`、`data/2026-11-02_steps.csv` の列 `VDD_on_V`・`VDD_off_V`）。

| | 平均 − オンの間の $V_{DD}$（中央値） | オフ − オンの $V_{DD}$ |
|---|---|---|
| 1 モータ（電池 2） | 31 mV | 101〜127 mV |
| 4 モータ同時（電池 3） | 83 mV | 203〜303 mV |

4 モータの M1 の回転数を、1 モータの表（同じ Duty・同じ $V_{DD}$）と比べた。1 モータで測った電圧の範囲の中の段だけを使う（外挿しない）。

| 表を引く電圧 | 範囲内の段 | 差の平均 | rms |
|---|---|---|---|
| 平均の $V_{DD}$ | 27 / 42 | −0.97 % | 1.24 % |
| オンの間の $V_{DD}$ | 21 / 42 | +0.09 % | 0.68 % |

（1 モータの表そのものの当てはめの rms は 0.71 %）

- 同じモータは、同じ Duty とオンの間の同じ電圧なら、4 モータ同時でも同じ回転数で回る。表 A と表 B の差（低い電圧で最大 2 %）は、この落ち込みでほぼ説明できる。C4M で −2 % に見えたのは、主に 1 モータの表の電圧範囲（3.63 V 以上）より下への外挿による。
- 記録電圧（CLI）とオンの間の $V_{DD}$ の差は、4 モータ同時で 73〜170 mV。Duty と記録電圧で表すと $\Delta V = 781 - 118\,V_\text{CLI} - 324\,D$ [mV]（残差 rms 12 mV。この電池 1 本だけの値）。パルス電流あたりでは 15〜39 mΩ で、電池が減るほど大きい。
- 飛行中の機体は、オンの間の $V_{DD}$ を直接は測れない。電池電圧を測る INA3221 は 1 回 1.1 ms の変換を 64 回平均しており（3 チャネル分で約 0.4 秒）、PWM の 1 周期 6.7 µs より桁違いに長い。
- 飛行ログに 1 モータの表を当てるなら、上の差で記録電圧をオンの間の $V_{DD}$ に直すのが筋になる。そうすると予測は表 A に近づくが、飛行の電圧依存はむしろ表 B と合っていた。この関係は未整理（電池ごとの内部抵抗の違いが候補。未確認）。

### 判定

| 照合 | 現行モデル | $R_\text{out}$ モデル | 判別できたか |
|---|---|---|---|
| 宙返りの角加速度 | 合わない（効き半分＋遅れ 15 ms が必要） | 合う（遅れ 3〜4 ms のみ） | できた |
| 飛行中の電池電流 | 1 割ずれる | 5〜6 % 以内 | できた |
| 小信号の同定（遅れ・ゲイン） | — | — | できない（振動ノイズで両方同じ形になる） |
| ホバリング Duty の電圧依存 | 傾き 0 の予測より強い | 予測より弱い | 論文の表のままでは食い違う。2026-11-02 の実験で $R_\text{out}$ は電池電圧によらないと確認し、$V_b$ を加えたベンチで実測した表では残らない |
| ホバーの釣り合い（全機 = 10/15 の 1 モータの 4 倍、2026-11-02） | — | ベンチで実測した表で ±3 % 以内 | 10/15 と 11/02 の推力は区別できない |

動特性は、電流を使わない宙返りの照合で、ベンチと論文の式が実機でも成り立つと確かめられた。電流の照合も同じ結論を支える。電池電圧への静的な依存だけは論文の前提と食い違っており、ファームの変換に使う前に実験で確かめる必要がある。**2026-11-02 追記:** 実験で確かめた。$R_\text{out}$ は電池電圧によらず、食い違いは論文の表の $R_\text{out}$ と $V_b$ の扱いによるものだった。ベンチで実測した表を使うと、ホバーの釣り合いは ±3 % 以内で合う。

## 7. 見つかった解析ツールの不具合

`sf sils sysid-gate` は、ログされた duty からファームの式を逆算してトルクを復元する（`tools/sysid/plant_fit.py`）。ここで `_MOTOR_CT` と `_KAPPA` は生成されたパラメータから来る（yaml に追従する）が、`_MOTOR_AM`・`_MOTOR_BM`・`_MOTOR_CM` は手書きのリテラルで、`sf params check` は検査しない。このため yaml の Ct だけを変えると、復元される推力が Ct_new/Ct_old 倍になり、$b$ が Ct_old/Ct_new 倍に膨らむ（今回は 1/0.76 = 1.32 倍）。

| 条件 | 生の出力（`gate.json`） | Am/Bm を Ct に合わせて再同定（`refit_b.txt`） |
|---|---|---|
| V1 ロール $b$ | 143702（+119.8 %） | 109213（+67.0 %） |
| V1 ピッチ $b$ | 129788（+322.4 %） | 98639（+221.0 %） |

同じ記録の再同定で $T$・$L$ は変わらず、原因の説明と一致した。**提案（未実施）**: この 3 つのリテラルを `sf params check` の検査対象に加えるか、`actuator.cpp` と同じ元から導く。

## 8. 判断と次の手（2026-10-30 時点）

| 項目 | 内容 |
|---|---|
| $C_T$・$C_Q$ だけの差し替え（V1・V2） | 採らない。$C_T$・$C_Q$ は駆動回路の平均モデルの一式として入れる。当初の理由（モデル一致の確認の悪化）は 6 章で根拠にならないと分かった |
| ファーム側の換算替え（VF） | SILS での挙動維持は確認済み。ただし yaml を変えないと `sf params check --strict` が 4 行（C_T、kappa、Am_flight_anchored、Bm_flight_anchored）で不一致になるため、単独では採用できない。採用にはパラメータ基準ファイルでファームの曲線とプラントの定数組を分ける方針の決定が要る。実機では `rate.yaw.*` の `param reset` も要る |
| 駆動回路の平均モデル | 5 章・6 章のとおり、宙返りと電池電流の照合で支持された。電池電圧への依存だけが論文の前提と食い違う。物理の実測を上位に置く方針により、SILS の既定にする（$R_\text{out}$ はベンチの実測値を基準にする）。再確認試験で新たに不合格になった 6 件（R1）は、SILS 側（シナリオの基準値、機体モデルのほかの部分）を疑って調べる |
| 電流検出 | コミット d685c9fd 以前は電池電流が 1/10 で報告されていた。ベンチデータの電流は「ファームの読み値の 10 倍」 |

### 決定（2026-10-30、開発者）

| 決定 | 内容 |
|---|---|
| 判断の優先順位 | 物理の実測を SILS より上位に置く。両者が食い違うときは SILS のほうを疑う |
| $R_\text{out}$ モデルの公開 | 公開する（表・SILS プラントの変更・照合スクリプトと結果を、本リポジトリの `analysis/scripts/ct_cq_bench_20261015/` に置いた） |
| モデル一致の確認（`sf sils sysid-gate`） | 作り直す。今はノイズのない SILS をノイズのある実機と比べており、モータの違いを判別できない（6 章）。記録した Duty を通す照合（宙返りのような）に切り替えるか、ノイズ条件をそろえる |
| 電池電圧依存の実験 | 電源電圧を恣意的に変えない設計にする（実験用の安定化電源では正確な結果が出ない可能性がある）。方法は検討中。案: $V_{DD}$ は各点で実測する入力なので、充電状態の違う電池で同じ Duty 列を短時間ずつ測り、各点の実測 $R_\text{out}$ が $V_{DD}$ によらないかを見る。巻線の温度上昇と分けるため段の間に冷却を入れ、電池の順番を往復させる |

### 残る課題

| 項目 | 内容 |
|---|---|
| 判定ツールの逆算定数 | 7 章の 3 つのリテラルを `sf params check` の検査対象にするか（小さな修正） |
| 再確認試験の 6 件 | R1 で新たに不合格になった原因の調査（5 章）。宙返り用の補正（0.6、6 ms、変化率制限）は外せる見込み |
| 宙返りの倍率 $s \approx 0.8$ | 6 章に挙げた候補（宙返り中の電池電圧の低下、$\eta$、そのほかのモデル誤差）の切り分け |
| 修正後のファームの電流表示 | 実機で `sensor power` を確認する（シャント修正 d685c9fd の動作確認） |
| SILS に入れる $R_\text{out}$ | ベンチの実測値を基準にし、その表で $\eta$ がほぼ 1 になるかを飛行ログのホバリングの釣り合いで確かめる（6 章） |

### 再現手順

1. `main` のコミット 884f2af6（または d685c9fd）に `analysis/scripts/ct_cq_bench_20261015/patches/<V>.patch` を適用する。
2. `sf params generate`、`sf sils build`、`sf sils regression`、`sf sils sysid-gate` の順に実行する。
3. V1・V2 は手書きの写しが基準ファイルと食い違うため、`sf sils regression` 冒頭のパラメータ整合の事前検査（`sf params check`）がエラーで止まる。この検査だけを外して実行した（VF も同様）。
4. 修正後の $b$ は、`sysid-gate` が出した記録（`gate_bundle.sflog.zip`）を `analysis/scripts/ct_cq_bench_20261015/refit_gate_bundle.py <リポジトリのルート> <記録> --am 4.587968e-8 --bm 5.840089e-4` で再同定して得る。

---

<a id="english"></a>

# Should the bench-measured propeller constants (C_T / C_Q) replace the SILS and firmware values? (measured 2026-10-15, studied 2026-10-30)

**Status**: study continued (2026-10-30). Swapping only C_T and C_Q (drive unchanged) is not taken. The averaged drive-circuit model ($R_\text{out}$) is supported by the flip and current cross-checks. The battery-voltage conflict is resolved by the 2026-11-02 bench ($R_\text{out}$ does not depend on the battery voltage, with a constant drop $V_b$ added) and the bench-measured rotor-speed map; with the vehicle thrust taken as four times the 10/15 single-motor thrust, flight hover balances within ±3 % (chapter 6). Created 2026-10-30, last updated 2026-11-02.

## 1. Purpose and conclusion

### Question

Should the propulsion constants measured on a single-motor bench on 2026-10-15 replace the provisional $C_T=1.00\times10^{-8}$ and $C_Q=4.10\times10^{-11}$ (`measured_2026_07` in `control/models/stampfly_physical.yaml`, used by the SILS plant and the firmware)? SILS is Software-In-the-Loop Simulation: the firmware source runs unmodified on a PC.

### Conclusion (as of 2026-10-30)

| Item | Conclusion | Main reasons |
|---|---|---|
| Swapping only $C_T$ and $C_Q$ in the SILS plant (drive still Duty × voltage) | **Not taken. $C_T$ and $C_Q$ go in as part of the averaged drive-circuit model** | Changing the constants without the drive keeps the drive form that the flip and current cross-checks reject. The originally cited worsening of the model-match check (`sf sils sysid-gate`; roll $b$ +56.5 % to +67 / +76 %, $L_\text{total}$ +17 % to +30 / +41 %) is no evidence, because chapter 6 shows that check cannot tell motor models apart. The regression test suite (44 scenarios of `sf sils regression`, which automatically confirms that existing behaviour is not broken) failed in 1 scenario for V1 and 2 for V2 |
| Averaged drive-circuit model ($R_\text{out}$, the developers' paper) | **Supported by the flip and current cross-checks; the battery-voltage conflict was resolved on 2026-11-02 (chapter 6, "Vehicle thrust and hover balance")** | Without any fitting, the flip angular-acceleration residual is 0.29 (0.98 to 1.02 for the current model) and the in-flight battery current is within 5 to 6 % of the measurement (chapters 5 and 6). Only the battery-voltage dependence of the hover Duty came out weaker than predicted |
| Firmware-only re-gauging (VF) | **Not adoptable alone (unchanged)** | SILS behaviour is preserved, but with the yaml unchanged `sf params check --strict` fails on 4 rows. A decision on how the parameter file separates the firmware curve from the plant constants must come first |
| Next step | Decisions in chapter 8 | Physical measurement ranks above the SILS. The $R_\text{out}$ model is published. The model-match check is rebuilt. The method of the battery-voltage experiment is to be decided |

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

**Caveat (added 2026-11-02)**: a redo with the same propeller at the same height on the evening of 2026-11-02 gave 0.961 times the thrust above at every rotor speed ($C_T = 7.27\times10^{-9}$ through the origin). Air density does not explain it (at most 0.7 %); probes and leads under the propeller (the intake side, since the vehicle sits upside down) may have been more than on 10-15 (not checked). On this bench an obstruction on the intake side can only make the thrust read lower, so the 10-15 values above are the reference and the vehicle thrust is four times the single-motor thrust (experimenter's decision). The four-motor thrust measured on 2026-11-02 is not used as a vehicle thrust measurement, because the other propellers sit differently relative to the can. $F/\omega^2$ rises with rotor speed on both days (about 20 % from 1200 to 4700 rad/s), so $C_T$ is not constant. $F=k\,\omega^n$ ($n$ = 2.14-2.15) fits both days; around hover the values above read $F=F_\text{ref}(\omega/\omega_\text{ref})^n$ with $\omega_\text{ref}=3500$ rad/s, $F_\text{ref}=0.0917$ N, $n=2.15$. The slope near hover, $dF/d\omega=nF/\omega$, is 7 % above the constant-$C_T$ value $2F/\omega$. $C_Q = 3.33\times10^{-11}$, 1.5 % from the value above. Adoption into SILS and the firmware stays as concluded in section 1 (the flight-log cross-check comes first). Taping the cables to both the scale tray and the table only shifts the scale zero by about -3 g and does not change the thrust readings (checked the same evening). An earlier note of the same day here blaming the cable routing was wrong (that day's thrust offsets came from reading negative scale values without their minus sign). $K_e$, $R$, $C_Q$, $J$ and $I_0$ do not use the scale.

**Caveat**: the battery current in this data is the firmware reading multiplied by 10, not an independent current measurement (the shunt-resistor setting was fixed in commit d685c9fd; before it the current was reported at 1/10).

### Related files

| Path | Content |
|---|---|
| `analysis/scripts/ct_cq_bench_20261015/patches/VF.patch`, `V1.patch`, `V2.patch` | The changes used in the study (variants in chapters 3 and 4). Differences against `main` commit 884f2af6 (the base of the working tree at study time); verified to apply to d685c9fd as well |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/regression.json` | `sf sils regression` results (V0, VF, V1, V2). For V1 and V2 the top-level `fail` count includes one extra item for the bypassed parameter-consistency pre-check; scenario failures are V1=1, V2=2 |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/gate.json` | Raw `sf sils sysid-gate` output (V0, VF, V1, V2) |
| `analysis/scripts/ct_cq_bench_20261015/results/<V>/refit_a.txt`, `refit_b.txt` | The same recording refitted with the constants of `plant_fit.py` as they are (a), or with Am/Bm matched to Ct (b) (V0, V1, V2). See chapter 7. The V0 file b uses an inconsistent condition (Am/Bm scaled by 0.76 with Ct unchanged), only lowers $b$ by about 0.76 (the reverse check), and is not used as a reference |
| `analysis/scripts/ct_cq_bench_20261015/bench_vs_models.py` | Compares the bench with the firmware static curve and the SILS plant steady state (prints the tables of chapter 3) |
| `analysis/scripts/ct_cq_bench_20261015/refit_gate_bundle.py` | Refits a `sysid-gate` bundle with overridable constants of `plant_fit.py` |
| `analysis/scripts/ct_cq_bench_20261015/rout/` | The $R_\text{out}(D)$ table, the SILS plant change (`R1.patch`, `drive_rout_table.hpp`) and a README (chapter 5) |
| `analysis/scripts/ct_cq_bench_20261015/crosscheck/` | Scripts, results and README of the flight-log cross-check (chapter 6). The input logs are not in the repository |

Reproduce: `python3 -I analysis/scripts/ct_cq_bench_20261015/bench_vs_models.py <data/2026-10-15/summary.json of stampfly-motor-autotest>`

## 3. Steady-state comparison

### Models compared

| Model | Content |
|---|---|
| Firmware static curve | `thrustToDuty` in `actuator.cpp`: $\omega=\sqrt{T/C_t}$, $V=A_m\omega^2+B_m\omega+C_m$, duty $=V/v_\text{bat}$ |
| Steady state of the SILS plant motor ODE (ordinary differential equation) | thrust $T=\eta\,C_t\,\omega^2$ ($\eta$ = `thrust_efficiency` = 0.7133) |

### Findings

1. The current SILS plant ($\eta=0.7133$) and the firmware curve give the same thrust at every Duty (made equivalent on 2026-08-22).
2. Driven by Duty × battery voltage, both models under-predict the bench thrust at low Duty (0.37 g vs 0.93 g at Duty 0.1) and are close near hover. **Driven by the measured average motor voltage $\langle V_m\rangle$, the plant ODE with the full bench set reproduces the bench thrust within 0.35 g (0.2 to 2.7 % near hover, Duty 0.5 to 0.7).** The bench constants are mutually consistent; the mismatch sits in the Duty to $\langle V_m\rangle$ relation, which the SILS plant assumes to be $V=\text{Duty}\times v_\text{batt}$. The Duty used in this comparison is the commanded value, which the firmware and the SILS receive (the Duty the bench measured from the drain waveform is 0.006 to 0.015 larger); the first version of this note (2026-10-30) compared at the measured Duty and was corrected the same day.
3. Per-interval slopes (thrust vs Duty×$V_{DD}$, and $\langle V_m\rangle$ vs Duty×$V_{DD}$; $V_{DD}$ is the battery voltage at measurement):

| Duty interval | bench [g/V] | model [g/V] | ratio | $\mathrm{d}\langle V_m\rangle/\mathrm{d}(\text{Duty}\cdot V_{DD})$ |
|---|---|---|---|---|
| 0.1→0.2 | 3.51 | 2.31 | 1.52 | 1.07 |
| 0.2→0.3 | 4.05 | 3.43 | 1.18 | 0.88 |
| 0.3→0.4 | 4.53 | 4.32 | 1.05 | 0.83 |
| 0.4→0.5 | 4.54 | 5.05 | 0.90 | 0.78 |
| 0.5→0.6 | 4.98 | 5.65 | 0.88 | 0.73 |
| 0.6→0.7 | 3.67 | 6.17 | 0.59 | 0.52 |
| 0.7→0.8 | 5.85 | 6.61 | 0.88 | 0.75 |
| 0.8→0.9 | 11.14 | 6.99 | 1.59 | 1.43 |

   Near hover (Duty 0.4 to 0.8) the real thrust-per-duty gain (slope) is 0.59 to 0.90 of the model's. This points the same way as the real/SILS roll gain ratio of 0.64 (65384/102334) in the SILS model-match check and the `torque_authority` of 0.60 from the 2026-10-02 flip-log fit, and plausibly explains a large part of that gap; it is not shown to explain all of it (single motor, $V_{DD}$ 4.2 to 4.4 V, no 4-motor battery voltage drop).
4. Hover anchoring: per-motor hover thrust is 0.0907 N (mass 0.037 kg) and the firmware hover voltage is 2.581 V. Re-deriving $\eta$ so that the plant hovers at that voltage:

| Variant | $\eta$ | hover $\omega$ [rad/s] | motor time constant at hover [ms] | plant yaw torque per firmware-commanded yaw torque | $\mathrm{d}T/\mathrm{d}V$ at hover [mN/V] |
|---|---|---|---|---|---|
| V0 current | 0.7133 | 3566 | 16.4 | 1.402 | 58.3 |
| V1 $C_T$, $C_Q$ only | 0.880 | 3682 | 17.3 | 1.136 | 59.5 |
| V2 full bench set ($K_e$, $R$, $Q_f=K_e\cdot0.044$, $C_Q$ including $K_e\cdot2.9\times10^{-9}$, $J=1.54\times10^{-8}$) | 0.941 | 3561 | 18.4 | 1.116 | 60.3 |

   The $\eta$ compensator applies to thrust only, so today the plant's yaw authority is 1.40 times what the firmware assumes. The bench motor reaches hover thrust at Duty×$V_{DD}$ = 2.39 V against the firmware's 2.58 V: the flight-anchored firmware needs about 8 % more thrust capability than the bench shows (cause not identified; candidates are mass, airframe interference and battery voltage drop under 4 motors). **Added 2026-11-02:** this gap is taken to come from comparing different voltages (bench 4.2-4.4 V, flight 3.6-4.1 V) as if Duty × voltage were constant. With the bench-measured rotor-speed map, four times the 10/15 thrust balances the hover weight within ±3 % (chapter 6, "Vehicle thrust and hover balance").

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

For VF, V1 and V2 the parameter-consistency pre-check before the run had to be bypassed because the hand-maintained copies disagree with the parameter file (see the reproduction steps in chapter 8).

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

V0 and VF are the raw `gate.json` values; V1 and V2 are the corrected values of chapter 7 (`refit_b.txt`). Tolerances: $b$ ±50 %, $L_\text{total}$ ±20 %. Real hardware: roll $b$=65384, $L_\text{total}$=14.36 ms; pitch 30724, 17.29 ms; yaw 111897, 10.82 ms. **Every variant fails, V0 included.**

| Variant | roll $b$ | roll $L_\text{total}$ | pitch $b$ | pitch $L_\text{total}$ | yaw $b$ | yaw $L_\text{total}$ |
|---|---|---|---|---|---|---|
| V0 | +56.5 % | 16.86 ms (+17.4 %) | +211.3 % | 17.09 ms (−1.2 %) | (+74.6 %) | (0.58 ms, −94.7 %) |
| VF | +57.3 % | 16.92 ms (+17.8 %) | +212.1 % | 17.24 ms (−0.3 %) | (+75.3 %) | (0.31 ms, −97.1 %) |
| V1 | +67.0 % | 18.66 ms (+30.0 %) | +221.0 % | 18.16 ms (+5.0 %) | (+23.3 %) | (2.09 ms, −80.6 %) |
| V2 | +75.7 % | 20.22 ms (+40.8 %) | +226.2 % | 19.16 ms (+10.8 %) | (+67.9 %) | (0.84 ms, −92.3 %) |

The yaw numbers are unreliable in every variant (the 3-parameter fit degenerates and $L_\text{total}$ is near 0; known issue, backlog #11), hence the parentheses. For roll and pitch, the bench constants increase the motor lag (fitted time constant $T$ 15.2 ms to 17.3 / 18.9 ms) and move SILS further from the hardware.

## 5. SILS with the averaged drive-circuit model

### The averaged model

The developers' paper under review (analysis of StampFly's capacitor-recirculation drive circuit) averages the drive circuit (a capacitor $C_C$ in parallel with the motor provides the recirculation path) over one PWM period and expresses it as an equivalent output resistance $R_\text{out}$ between the battery voltage and the back-EMF. $R_\text{out}$ is set by the Duty and $C_C$ and does not depend on the battery voltage or the rotor speed. This differs in form from the SILS assumption "average terminal voltage = Duty × battery voltage".

$$\bar I_m=\frac{V_\text{BAT}-K_e\,\omega}{R_\text{out}(D,\,C_C)},\qquad J\frac{d\omega}{dt}=K_t\,\bar I_m-C_Q\,\omega^2-K_t\,I_0(\omega),\qquad T=\eta\,C_T\,\omega^2$$

($\bar I_m$: average motor current [A]; $V_\text{BAT}$: battery voltage [V]; $D$: Duty; $I_0(\omega)$: no-load current [A]; $\eta$: thrust efficiency factor `thrust_efficiency`)

For the board's $C_C = 0.1\,\mu$F, $R_\text{out}$ is 26.0 ohm at $D$ = 0.1, 6.68 ohm at 0.3, 3.15 ohm at 0.5, 1.88 ohm at 0.7, 0.97 ohm at 0.9 and 0.66 ohm at 1.0. The table in 0.01 steps (`analysis/scripts/ct_cq_bench_20261015/rout/rout_table_c01.csv`) was put into the SILS plant (the change is `R1.patch` and `drive_rout_table.hpp` in the same place; it is not applied to this repository).

The constants are those of the developers' paper (the 10/15 bench values). The firmware is unchanged (so the back-calculation of the model-match check does not shift either). $\eta$ was run two ways: 0.8975 (R1), matched to the firmware hover Duty at 3.8 V, and 1.0 (R1-η1), the bench value as is.

### Gain at hover (thrust change per unit Duty)

| Battery voltage | $R_\text{out}$ model [mN] | Current SILS [mN] | Ratio |
|---|---|---|---|
| 3.6 V | 117 | 210 | 0.56 |
| 3.8 V | 158 | 221 | 0.71 |
| 4.0 V | 185 | 233 | 0.79 |
| 4.2 V | 203 | 245 | 0.83 |

At flight battery voltages the ratio is 0.56 to 0.71, the same range as the real/SILS gain ratio of 0.64 in the model-match check and the compensation factor `torque_authority` 0.60 introduced by the flip identification.

### SILS results

| | V0 current | R1 ($\eta$ 0.8975) | R1-η1 ($\eta$ 1.0) |
|---|---|---|---|
| roll $b$ (difference from real) | +56.5 % | −17.6 % | +28.7 % |
| pitch $b$ | +211.3 % | +8.2 % | +84.8 % |
| roll total delay (real 14.4 ms) | 16.9 ms | 37.7 ms | 39.6 ms |
| pitch total delay (real 17.3 ms) | 17.1 ms | 36.0 ms | 39.8 ms |
| regression tests (44 scenarios) | 38 pass | 32 pass, 6 fail | 18 pass, 20 fail |
| known failures that now pass | — | 3 | 0 |
| hover Duty (3.6 / 3.8 / 4.2 V) | 0.717 / 0.679 / 0.614 | 0.757 / 0.679 / 0.585 | 0.654 / 0.603 / 0.533 |

| Item | Content |
|---|---|
| Gain | In the noise-free SILS, the roll and pitch $b$ of R1 fall inside the tolerance (±50 %) without the compensation factor. However, the calibration in chapter 6 shows that with vibration noise $b$ cannot tell the plants apart either. What did discriminate was the flip comparison; the content of today's `torque_authority` 0.6 is considered to be this drive-circuit effect |
| Delay | The hover time constant of the $R_\text{out}$ model is 38.8 ms (the paper's value; the paper itself says the response was not measured). The noise-free SILS identification gave $T$ = 36.5 ms, which looked inconsistent with the real reference (total delay 14 to 17 ms). But the calibration of chapter 6 shows that under vibration noise, as on the real aircraft, both the 16 ms plant and the 38 ms plant are identified as "total delay 13 to 14 ms". This reference cannot decide between them |
| Regression tests | Newly failing in R1: 2 auto-takeoff altitude-spread scenarios, 3 flip-related scenarios (2 of which never start because the firmware rejects the flip request), 1 re-flight after a crash. Causes not yet investigated. Whether the scenario thresholds were tuned to the old model or the firmware would make the same decision on hardware changes the meaning, so this must be checked before making it the default. Conversely, the known failures `calib` (self-oscillation) and `pos_flight` / `pos_yaw` (Duty that was pinned at 1.0 is now at most 0.72) passed. The 20 failures at $\eta$ = 1 are probably because thrust is about 11 % above the firmware's hover assumption (unverified) |
| Flip discrimination | Removing the compensation factor 0.6 from the flip-identified settings still passes the flip with both the current and the $R_\text{out}$ model, so it did not decide |
| Battery-voltage dependence | In the $R_\text{out}$ model, the Duty needed to hover changes with battery voltage more steeply than the firmware assumes (Duty ∝ 1/voltage): +5.6 % at 3.6 V, −4.7 % at 4.2 V. The prediction is that, as the battery drains, the amount absorbed by the altitude-control integrator and the thrust learning grows |

The cross-check against flight logs is in the next chapter, 6.

## 6. Cross-check against flight logs

The policy is "take the bench test and the developers' paper equations as the reference and check them against flight logs". For each check, what a wrong model would look like was decided before computing. The logs used are 14 flights from 2026-06-14 to 09-08, plus 2 flips and 2 ground motor-spin logs of 10/2 (400 Hz with Duty) that were stored on another PC. The propeller was replaced in 2025, and all logs use the same one. The logs are not in this repository. Scripts and results are in `analysis/scripts/ct_cq_bench_20261015/crosscheck/` (its README maps the input log file names and the script behind each table).

### Calibration of the identification procedure

The real reference (6/14) was identified from natural, unexcited motion by recomputing the firmware PID output. The same procedure was applied to the SILS, whose time constant is known (`crosscheck/calfit.py`, `crosscheck/sysid_gate_small.scn`, results in `crosscheck/results/sils_cal/`).

| Condition (roll) | true time constant 16 ms (current) | true time constant 38 ms ($R_\text{out}$) |
|---|---|---|
| no noise | T 15 + L 2 ms | T 36 + L 1 ms |
| sensor noise only | T 14 + L 2 ms | T 36 + L 1 ms |
| with vibration noise | T 0 + L 13 to 14 ms | T 0 + L 13 to 14 ms |
| real reference (6/14) | T 0 + L 14.7 ms (coherence 0.65: the strength of correlation between input and output signals, 0 to 1) | same |

(T: first-order time constant, L: dead time.) With vibration noise the 16 ms and 38 ms plants are identified in the same form, and the real reference has that form too. This method cannot discriminate the motor time constant. The same holds for the gain $b$: with noise both plants give values comparable to the real one (roll 61 to 69k, real 64k). The same identification on the 14 flight logs gives total delays of 13 to 19 ms in roll, which did not discriminate either (`crosscheck/logcheck/test3.py`).

### Flips (large signal)

The recorded 400 Hz Duty and battery voltage were passed through each model to predict roll angular acceleration. A prediction with no fitting at all (scale 1, delay 0) was compared with one whose scale and extra delay are optimised (`crosscheck/flipcheck/flipcheck.py`, results in `crosscheck/results/flipcheck_results.json`). The data for overlaying the measurement (derivative of the gyro) and the predictions of the current and $R_\text{out}$ models (bench constants, $\eta$ = 1) is produced by `crosscheck/flip_overlay.py` as `flip_overlay.json` (the figure is not reproduced in this note; flip_test1 is the right-roll flip of 10/2).

| Model | residual without fitting | best scale $s$ | best extra delay | residual then |
|---|---|---|---|---|
| current model | 0.98 to 1.02 | 0.54 to 0.56 | 13 to 17 ms | 0.15 to 0.21 |
| $R_\text{out}$ model ($\eta$ = 1) | 0.29 | 0.79 to 0.80 | 3 to 4 ms | 0.09 to 0.10 |

- The residual is the RMS of (prediction − measurement) divided by the RMS of the measurement. Both flips give the same result.
- The $R_\text{out}$ model matches with only the delay of recording and communication (3 to 4 ms) added. The current model needs its effectiveness halved and 13 to 17 ms of delay. The flip-specific compensation (`torque_authority` 0.6, 6 ms delay, Duty rate limit) can be read as having filled exactly this gap.
- In the hover parts of the same logs, the extra delay is 4 to 8 ms for the $R_\text{out}$ model and 10 to 15 ms for the current model. The earlier puzzle that large and small signals showed different delays (simulation-policy backlog #17(c)) is explained by the $R_\text{out}$ model with the same small delay.
- Candidates for the part of the scale 0.79 to 0.80 that falls short of 1: (1) battery-voltage sag during the flip (the logged voltage is only 1 Hz, so a sag over the few hundred ms of the flip is invisible); (2) the hover-balance $\eta$ of 0.94 to 0.95 (computed with a mass of 0.037 kg; the mass at the bifilar measurement was 0.0364 kg; this uses the paper's $R_\text{out}$ table, and the bench-measured rotor-speed map gives 0.97 to 1.01, see "Vehicle thrust and hover balance" below); (3) other model error. The moment of inertia is the bifilar measurement (hanging by two strings and measuring the torsional period), whose record is in `control/models/stampfly_physical.yaml`. The record gives $I_{xx}$ = 9.19e-6 kg·m², 0.3 % from the adopted 9.16e-6, so it is not a main candidate here.

### Battery current

In the $R_\text{out}$ model the mean battery current equals the mean motor current. In the simple averaged model it is "Duty × motor current", and the two differ by about 1.5×. The standby current (146 to 200 mA) was subtracted from the recorded current (×10 gives the true value) before comparing (`crosscheck/logcheck/test2.py`).

| Log | measured ÷ $R_\text{out}$ model | measured ÷ simple average model | measured ÷ current SILS |
|---|---|---|---|
| 7/17 hover (5 logs, 191 points) | 0.94 | 1.68 | 0.89 |
| 7/18 hover (2 logs, 97 points) | 0.95 | 1.72 | 0.90 |
| 9/8 hover | 0.95 | 1.39 | 0.84 |
| 10/2 motors spun on the ground (2 logs) | 0.99 to 1.01 | — | — |

The simple average model is rejected, and the $R_\text{out}$ model falls within the bench accuracy (±8 %). The ×10 is the ratio of the schematic shunt R12 = 0.01 Ω to the firmware's assumed 0.1 Ω and agrees with the energy balance (7.6× or more required); the remaining uncertainty is the shunt's part tolerance (a few percent).

### Hover Duty and battery voltage

In the $R_\text{out}$ model, the Duty needed to hover rises steeply as the battery voltage falls, so Duty × voltage varies with voltage (slope about −0.3 to −0.4 over this range). In the simple average model and the firmware assumption, Duty × voltage is constant (slope 0) (`crosscheck/logcheck/test1.py`).

- In the log with the widest voltage range (6/27, 0.42 V wide) the slope is −0.20 ± 0.05: different from both predictions, between them. The constant assumption fits better.
- Other dates have narrow voltage ranges (around 0.2 V or less) and could not discriminate.
- Over three minutes of hover, voltage sag and elapsed time are confounded with winding temperature rise. The temperature rise acts in the same direction as the $R_\text{out}$ model, so removing it widens the difference from the $R_\text{out}$ model.
- This is the first result from flight logs that conflicts with the paper's premise "$R_\text{out}$ does not depend on the battery voltage". The 10/15 bench covered only 4.17 to 4.38 V, so the range over which the premise was checked is narrow.
- Implication: if a thrust-to-Duty conversion is built from the $R_\text{out}$ table (the paper's section 5.3 proposal), the hover estimate may drift by a few percent as the battery drains. Adopting it in the firmware comes after the experiment on the battery-voltage dependence (policy in chapter 8). It is also worth stating in the paper's revision that independence of voltage was checked only over 4.2 to 4.4 V.
- **Added 2026-11-02:** the long-discharge bench of 11/02 (loaded battery voltage 4.27 → 3.63 V, cold laps compared) changed $R_\text{out}$ by 0 to −1.1 %, so it does not depend on the battery voltage. A constant drop $V_b \approx 0.11$ V at every Duty (the size of a brush-contact drop) must be added: $\bar I_m = (V_{DD} - K_e\omega - V_b)/R_\text{out}(D)$. The conflict seen here came from the paper's $R_\text{out}$ table (about 10 % below the measurement at D0.3-0.7) and the missing $V_b$; with the bench-measured rotor-speed map it is small (see "Vehicle thrust and hover balance" below).

### R_out measured on the bench

From each step's measured values, $R_\text{out} = (V_{DD} - K_e\,\omega)/I_\text{bat}$, compared with the paper's calculation ($V_{DD}$ measured by the oscilloscope near the board, $I_\text{bat}$ = firmware reading × 10, commanded Duty).

| Duty | $V_{DD}$ [V] | Measured [Ω] | Calculated [Ω] | Measured / calculated |
|---|---|---|---|---|
| 0.1 | 4.378 | 26.19 | 25.97 | 1.01 |
| 0.2 | 4.364 | 12.07 | 11.55 | 1.05 |
| 0.3 | 4.349 | 7.33 | 6.68 | 1.10 |
| 0.4 | 4.328 | 4.88 | 4.45 | 1.10 |
| 0.5 | 4.302 | 3.479 | 3.152 | 1.10 |
| 0.5 (repeat) | 4.236 | 3.463 | 3.152 | 1.10 |
| 0.6 | 4.278 | 2.563 | 2.326 | 1.10 |
| 0.7 | 4.253 | 2.072 | 1.882 | 1.10 |
| 0.8 | 4.223 | 1.547 | 1.547 | 1.00 |
| 0.9 | 4.173 | 0.941 | 0.970 | 0.97 |

- The two Duty 0.5 steps differ by 1.5 % in $V_{DD}$ but only 0.5 % in measured $R_\text{out}$: a narrow range, but consistent with "$R_\text{out}$ does not depend on the battery voltage".
- At Duty 0.3 to 0.7 the measurement is about 10 % above the calculation; at Duty 0.1, 0.8 and 0.9 they agree, so this is not an error applied uniformly over all Duty values but a difference in the shape of the calculation.
- A 10 % larger $R_\text{out}$ near hover lowers the thrust by about 7 % (linearised estimate). Most of the hover-balance $\eta$ of 0.93 to 0.95 and of the current ratio of 0.94 to 0.95 in the flight logs may be explained by this.
- Under the policy that physical measurement ranks above the SILS (chapter 8), the SILS takes the measured $R_\text{out}(D)$ as its reference rather than the calculation. The measurement has 9 points at 0.1 steps, so the shape in between follows the calculation scaled to the measurement.

### Vehicle thrust and hover balance (added 2026-11-02)

On 2026-11-02 the vehicle thrust was taken as four times the 10/15 single-motor thrust (caveat in chapter 2). This was checked against hover in the flight logs (`crosscheck/logcheck/test4.py`; results in `crosscheck/results/test4.json` and `test4_windows.csv`). In each hover window (5 s; 134 windows, 574 s in all) the four logged duties and the battery voltage give the rotor speeds through the $\omega(D, V)$ map measured on the bench on 2026-11-02; the 10/15 power law $F = k\,\omega^{n}$ ($n = 2.148$) turns them into thrust, compared with the weight $mg$. If the claim were wrong (thrust lost to interference in flight), the ratio $\sum F/(mg)$ would be clearly above 1.

| Route | Content |
|---|---|
| Map A | Four motors at once, M1, against the voltage at the logged point (CLI) |
| Map B | One motor, against $V_{DD}$ at the driver ($V_{DD}$ = logged voltage − the 9-80 mV CLI-to-$V_{DD}$ difference measured with four motors) |
| Omega-free route | The 11/02 single-motor thrust (the 0.961 × 10/15 condition) regressed directly on $(D, V_{DD})$ |

| Thrust / weight (time-weighted mean) | Mass 37 g | Mass 36 g |
|---|---|---|
| Map A with the 10/15 curve | 0.986 | 1.013 |
| Map B with the 10/15 curve | 1.006 | 1.034 |
| Omega-free route (11/02 thrust) | 0.981 | 1.008 |

- The mass is the developer's estimate, 36 to 37 g. The flight logs come from vehicles and propellers of the same design, but not the bench vehicle (several individual units). The motor PWM frequency of the firmware in the log period is 150 kHz, the same as on the bench.
- Four times the 10/15 thrust balances the hover weight within ±3 %. There is no sign of a 5 % or larger thrust loss to rotor-rotor or airframe interference in flight. The omega-free route agrees within 1.5 % with the 11/02 value through omega (0.966 with map B).
- The omega map comes from the 11/02 bench (probes under the propeller; 1-4 % more torque at the same omega), so omega may read 0.5 to 1.5 % low. Together with the mass range (2.7 %), this check cannot tell the 10/15 thrust from the 11/02 thrust (3.9 % apart). The reason for taking 10/15 is given in the chapter 2 caveat.
- Per-log median ratios range from 0.88 (7/18 06:42) to 1.07 (6/29), more than the model uncertainty. Unit-to-unit differences (mass, motors, propellers) are candidates. On the 11/02 four-motor bench, too, some motor ran 4 to 8 % slower than M1 (lines in the supply-voltage spectrum).
- Within one flight, the slope of the ratio against voltage is −0.44 to +0.13 /V with map B (7 of the 10 logs with a voltage span of 0.15 V or more give +0.02 to +0.13 /V, i.e. 1 to 7 % over 0.5 V) and −0.42 to +0.21 /V with map A. Voltage sag and motor warming ($K_e$ drops 2-4 % within minutes) overlap. The conflict with the prediction from the paper's $R_\text{out}$ table ("Hover Duty and battery voltage" above) does not remain with the bench-measured map.

#### Hover duty compared

The same check seen as duty (`hover_duty_by_voltage` in `test4.json`). For each 0.1 V of logged voltage, the flight hover duty (median of the four-motor mean) is set against the duty the bench says carries 36 to 37 g (the same duty on all four motors).

| Logged voltage | Flight (windows, logs) | Map A with the 10/15 curve | Map B with the 10/15 curve | 11/02 thrust directly |
|---|---|---|---|---|
| 3.5 V | 0.731 (8, 1) | 0.746-0.763 | 0.716-0.733 | 0.739-0.756 |
| 3.6 V | 0.705 (11, 3) | 0.713-0.729 | 0.690-0.706 | 0.709-0.726 |
| 3.7 V | 0.690 (23, 8) | 0.686-0.702 | 0.668-0.684 | 0.685-0.701 |
| 3.8 V | 0.662 (19, 6) | 0.651-0.666 | 0.638-0.654 | 0.652-0.668 |
| 3.9 V | 0.636 (25, 11) | 0.626-0.640 | 0.616-0.631 | 0.629-0.644 |
| 4.0 V | 0.616 (37, 9) | 0.602-0.615 | 0.595-0.609 | 0.607-0.622 |
| 4.1 V | 0.599 (11, 8) | 0.588-0.601 | 0.582-0.597 | 0.595-0.609 |

- The time-weighted mean difference (flight − bench) is −1.9 to +2.8 % depending on the route and the mass. Per-log medians scatter by about ±0.05 (different individual vehicles).
- The slope of duty against logged voltage (all windows) is −0.242 /V in flight, against −0.24 (map B), −0.26 (11/02 thrust directly) and −0.29 /V (map A). Treating Duty × voltage as constant (the firmware conversion) gives −0.168 /V, too weak a voltage dependence.
- Within one flight (the 4 logs with a voltage span of 0.3 V or more) the flight slope is −0.16 to −0.24 /V against −0.23 to −0.26 /V for map B, slightly weaker in flight. On the bench, cold and warm laps differ by no more than ±0.6 % in speed at the same duty and voltage, so motor warming does not explain it; the cause is open.
- The 2026-11-02 four-motor measurement on the can put the duty for 37 g at 0.726 for A4M (logged voltage about 3.88 V) and 0.764 for B4M (about 3.69 V), 0.07 to 0.09 above flight at the same voltage (0.64, 0.69); at the flight hover duty the scale read about 34 g (6 to 8 % below the weight). This agrees with not using that measurement as the vehicle thrust (chapter 2 caveat).

#### One motor against four at once (on-time voltage)

All four motors switch on together on the same timer. During the on-time the pulse current of four motors (4.1 to 5.0 A on 2026-11-02) flows through the battery and the wiring, and M1's $V_{DD}$ dips further below its mean (bench repository `tools/vdd_on_time.py`; columns `VDD_on_V` and `VDD_off_V` of `data/2026-11-02_steps.csv`).

| | Mean − on-time $V_{DD}$ (median) | Off − on $V_{DD}$ |
|---|---|---|
| One motor (battery 2) | 31 mV | 101-127 mV |
| Four motors at once (battery 3) | 83 mV | 203-303 mV |

The four-motor M1 speed was compared with the one-motor map (same Duty, same $V_{DD}$), using only steps inside the voltage range measured with one motor (no extrapolation).

| Voltage the map is read with | Steps inside | Mean difference | rms |
|---|---|---|---|
| Mean $V_{DD}$ | 27 / 42 | −0.97 % | 1.24 % |
| On-time $V_{DD}$ | 21 / 42 | +0.09 % | 0.68 % |

(The one-motor map's own fit has an rms of 0.71 %.)

- The same motor runs at the same speed with four motors at once as with one, given the same Duty and the same on-time voltage. The difference between maps A and B (up to 2 % at low voltage) is almost fully explained by this dip. The −2 % seen for C4M came mainly from extrapolating below the one-motor voltage range (3.63 V and up).
- The difference between the logged voltage (CLI) and the on-time $V_{DD}$ is 73 to 170 mV with four motors; in Duty and logged voltage, $\Delta V = 781 - 118\,V_\text{CLI} - 324\,D$ [mV] (residual rms 12 mV; one battery only). Per unit pulse current it is 15 to 39 mΩ, larger as the battery drains.
- In flight the vehicle cannot measure the on-time $V_{DD}$ directly: the INA3221 that measures the battery voltage averages 64 conversions of 1.1 ms (about 0.4 s for three channels), orders of magnitude longer than the 6.7 µs PWM period.
- Applying the one-motor map to flight logs properly means converting the logged voltage to the on-time $V_{DD}$ with the difference above. That moves the prediction toward map A, while the flight voltage dependence matched map B better. This is not yet sorted out (battery-to-battery internal resistance is a candidate; not checked).

### Verdict

| Check | Current model | $R_\text{out}$ model | Discriminated? |
|---|---|---|---|
| Flip angular acceleration | does not match (needs half effectiveness + 15 ms delay) | matches (only 3 to 4 ms delay) | yes |
| In-flight battery current | off by about 10 % | within 5 to 6 % | yes |
| Small-signal identification (delay, gain) | — | — | no (vibration noise gives both the same form) |
| Voltage dependence of hover Duty | stronger than the slope-0 prediction | weaker than predicted | conflicts with the paper's table; the 2026-11-02 experiment showed $R_\text{out}$ does not depend on the battery voltage, and with $V_b$ added the bench-measured map leaves no conflict |
| Hover balance (vehicle = 4 × the 10/15 single motor, 2026-11-02) | — | within ±3 % with the bench-measured map | cannot tell the 10/15 and 11/02 thrusts apart |

The dynamics were confirmed to hold on the real aircraft, with the bench and paper equations, by the flip comparison, which does not use current. The current cross-check supports the same conclusion. Only the static dependence on battery voltage conflicts with the paper's premise, and it must be checked by experiment before it is used in the firmware conversion. **Added 2026-11-02:** checked by experiment. $R_\text{out}$ does not depend on the battery voltage; the conflict came from the paper's $R_\text{out}$ table and the missing $V_b$. With the bench-measured map, hover balances within ±3 %.

## 7. Analysis-tool defect found

`sf sils sysid-gate` recovers torque from the logged duty by inverting the firmware chain (`tools/sysid/plant_fit.py`). There `_MOTOR_CT` and `_KAPPA` come from the generated parameters (they follow the yaml), but `_MOTOR_AM`, `_MOTOR_BM` and `_MOTOR_CM` are hand-typed literals that `sf params check` does not verify. Changing Ct in the yaml alone therefore scales the recovered thrust by Ct_new/Ct_old and inflates $b$ by Ct_old/Ct_new (1/0.76 = 1.32 here).

| Variant | Raw output (`gate.json`) | Refitted with Am/Bm matched to Ct (`refit_b.txt`) |
|---|---|---|
| V1 roll $b$ | 143702 (+119.8 %) | 109213 (+67.0 %) |
| V1 pitch $b$ | 129788 (+322.4 %) | 98639 (+221.0 %) |

Refitting the same recorded data leaves $T$ and $L$ unchanged, which confirms the cause. **Proposed fix (not done)**: add these three literals to `sf params check`, or derive them from the same source as `actuator.cpp`.

## 8. Decision and next steps (as of 2026-10-30)

| Item | Content |
|---|---|
| Swapping only $C_T$ and $C_Q$ (V1, V2) | Not taken. $C_T$ and $C_Q$ go in as part of the averaged drive-circuit model. The original reason (worse model-match check) turned out to be no evidence (chapter 6) |
| Firmware re-gauging (VF) | Behaviour preservation verified in SILS. Not adoptable alone: with the yaml unchanged it fails `sf params check --strict` on 4 rows (C_T, kappa, Am_flight_anchored, Bm_flight_anchored). Adoption needs a decision on how the parameter file separates the firmware curve from the plant constants. On hardware it also needs `param reset` for `rate.yaw.*` |
| Averaged drive-circuit model | Supported by the flip and battery-current cross-checks (chapters 5 and 6); only the battery-voltage dependence conflicts with the paper's premise. Under the policy that physical measurement ranks above the SILS, it becomes the SILS default, with $R_\text{out}$ taken from the bench measurement. The 6 scenarios that newly fail in R1 are investigated with the SILS side under suspicion (scenario thresholds, other parts of the plant) |
| Current sensing | Battery current was reported at 1/10 until commit d685c9fd; the bench data's current is "firmware reading ×10" |

### Decisions (2026-10-30, the developer)

| Decision | Content |
|---|---|
| Order of judgement | Physical measurement ranks above the SILS. When the two disagree, the SILS is suspected |
| Publishing the $R_\text{out}$ model | Published (the table, the SILS plant change, and the cross-check scripts and results are in `analysis/scripts/ct_cq_bench_20261015/` of this repository) |
| Model-match check (`sf sils sysid-gate`) | To be rebuilt. It compares a noise-free SILS with a noisy real aircraft and cannot tell motors apart (chapter 6). Switch to a check that passes the recorded Duty through the model (like the flip comparison), or equalise the noise conditions |
| Battery-voltage experiment | To be designed without arbitrarily changing the supply voltage (a bench power supply may not give accurate results). The method is under study. Candidate: $V_{DD}$ is measured at every point as an input, so measure the same Duty sequence briefly with batteries at different states of charge and check whether each point's measured $R_\text{out}$ is independent of $V_{DD}$; insert cool-down between steps and run the batteries in both orders to separate winding heating |

### Remaining items

| Item | Content |
|---|---|
| Back-calculation constants of the judging tool | Whether to put the 3 literals of chapter 7 under `sf params check` (a small fix) |
| The 6 regression-test failures | Investigate the new failures in R1 (chapter 5). The flip compensation (0.6, 6 ms, rate limit) is expected to be removable |
| Flip scale $s \approx 0.8$ | Separate the candidates listed in chapter 6 (battery-voltage sag during the flip, $\eta$, other model error) |
| Current display of the fixed firmware | Check `sensor power` on hardware (verification of the shunt fix d685c9fd) |
| $R_\text{out}$ in the SILS | Use the bench measurement as the reference and check with the flight-log hover balance whether $\eta$ then comes out near 1 (chapter 6) |

### Reproduction

1. Apply `analysis/scripts/ct_cq_bench_20261015/patches/<V>.patch` to `main` commit 884f2af6 (or d685c9fd).
2. Run `sf params generate`, `sf sils build`, `sf sils regression`, `sf sils sysid-gate` in this order.
3. For V1 and V2 the parameter-consistency pre-check at the start of `sf sils regression` (`sf params check`) stops with an error because hand-maintained copies disagree with the parameter file; only this pre-check was bypassed (VF likewise).
4. Obtain the corrected $b$ by refitting the recording (`gate_bundle.sflog.zip`) written by `sysid-gate` with `analysis/scripts/ct_cq_bench_20261015/refit_gate_bundle.py <repo root> <recording> --am 4.587968e-8 --bm 5.840089e-4`.
