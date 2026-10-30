# 駆動回路の等価出力抵抗 R_out(D) の表 / Equivalent output resistance R_out(D) of the drive circuit

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 内容

| ファイル | 内容 |
|---|---|
| `rout_table_c01.csv` | 基板のコンデンサ $C_C = 0.1\,\mu\mathrm{F}$ での $R_\text{out}(D)$ [Ω]。デューティ（PWM のオン時間比）$D = 0.01 \ldots 1.00$、0.01 刻み、100 行 |
| `R1.patch` | 検討で使った SILS プラントの変更（本リポジトリには適用していない）。表を Duty から平均モータ電流への換算に使う |
| `drive_rout_table.hpp` | `R1.patch` が取り込む表の C++ ヘッダ（`simulator/sils/plant/` に置く想定） |

### 計算方法と定数

開発者らの論文（査読対応中、StampFly のコンデンサ還流駆動回路の解析）の状態遷移行列法（1 周期の状態遷移を行列で表し、周期定常の平均値を解く方法）で求めた。**電池電圧には依存しない**（構成上、$R_\text{out}$ は $D$ と $C_C$ だけで決まる）。計算コードは本リポジトリに無い。

| 定数 | 値 |
|---|---|
| $K_e$（逆起電力定数、$=K_t$） | $5.905\times10^{-4}$ V·s/rad |
| 巻線抵抗 $R_s$、インダクタンス $L_s$ | 0.59338 Ω、0.78813 µH |
| 巻線の高周波側の梯子回路 $R_1, L_1, R_2, L_2$ | 0.84158 Ω、1.96696 µH、1.37732 Ω、0.61103 µH |
| MOSFET オン抵抗 $R_\text{DS(on)}$ | 0.065 Ω |
| $C_C$ | 0.1 µF |

$D=1.00$ の行は $R_s + R_\text{DS(on)}$（0.65838 Ω）。検討で使った $K_e$ などのベンチ値は `../../../notes/ct_cq_bench_20261015.md` の 2 章。

---

<a id="english"></a>

## 1. Contents

| File | Content |
|---|---|
| `rout_table_c01.csv` | $R_\text{out}(D)$ [ohm] for the board's $C_C = 0.1\,\mu\mathrm{F}$, duty (PWM on-time ratio) $D = 0.01 \ldots 1.00$ in 0.01 steps, 100 rows |
| `R1.patch` | The SILS plant change used in the study (not applied to this repository). Uses the table to convert Duty to average motor current |
| `drive_rout_table.hpp` | C++ header of the table that `R1.patch` includes (meant for `simulator/sils/plant/`) |

### Method and constants

Computed with the state-transition-matrix method of the developers' paper under review (analysis of StampFly's capacitor-recirculation drive circuit): the one-period state transition is written as a matrix and the period-steady average is solved. **Independent of the battery voltage by construction** ($R_\text{out}$ depends only on $D$ and $C_C$). The computation code is not in this repository.

| Constant | Value |
|---|---|
| $K_e$ (back-EMF constant, $=K_t$) | $5.905\times10^{-4}$ V·s/rad |
| Winding resistance $R_s$, inductance $L_s$ | 0.59338 ohm, 0.78813 µH |
| High-frequency ladder of the winding $R_1, L_1, R_2, L_2$ | 0.84158 ohm, 1.96696 µH, 1.37732 ohm, 0.61103 µH |
| MOSFET on-resistance $R_\text{DS(on)}$ | 0.065 ohm |
| $C_C$ | 0.1 µF |

The $D=1.00$ row is $R_s + R_\text{DS(on)}$ (0.65838 ohm). The bench values such as $K_e$ are in chapter 2 of `../../../notes/ct_cq_bench_20261015.md`.
