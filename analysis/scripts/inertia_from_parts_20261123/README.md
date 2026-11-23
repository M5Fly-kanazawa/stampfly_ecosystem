# 部品の STL と実測質量からの慣性モーメント（2026-11-23）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このスクリプトについて

StampFly の慣性モーメント（ロール $I_{xx}$・ピッチ $I_{yy}$・ヨー $I_{zz}$）の基準の値を出す。2026-11-23 に開発者が、基準の値を 2点吊り法の実測値（9.16・13.3・20.4 e-6 kg·m²）から、「部品の 3D モデル（STL）と実測の部品の質量からの試算値」に、3 軸とも替えると決めた。その試算値を、リポジトリ内のファイルだけで再現する。経緯と議論は `analysis/notes/mass_inertia_measurements_20261123.md`。

### 実行

```bash
python3 -I analysis/scripts/inertia_from_parts_20261123/inertia_from_parts.py
```

Python 3 と numpy だけで動く。結果は `inertia_from_parts_output.txt`（同じ出力を保存したもの）。

## 2. 値の定義（開発者の決定）

| 項目 | 内容 |
|---|---|
| 質量 | 実測の中央の値: モータ 17/6 g ×4、電池 8 g、フレーム 3.5 g、StampS3 モジュール 3.25 g、前方 ToF 基板（STL の `battery_adapter`）1 g、プロペラ 1.01/4 g ×4。主基板 = 全質量 37 g −（ほかの全部）= 8.907 g |
| 質量の分布 | 各部品は STL の体積に一様。主基板の残りは主基板の STL の形に一様。フレームも一様 |
| 座標 | STL（mm、X = 左・Y = 上・Z = 前）から機体座標 FRD（x = +Z、y = −X、z = −Y）。重心まわり |
| 計算 | 乱数なし。中央の質量での 1 回の計算。**有効 3 桁に 1 回だけ丸めた値を採用値とする** |
| プロペラ | 羽根 4 枚の STL がリポジトリに無いので、定数で置く。軸まわりの I/m = 55.28 mm²、面内の I/m = 28.17 mm²（公開リポジトリ `kouhei1970/prop_sim` の写真の実測から作った形の値）。ロータ中心は STL の (X, Z) = (±22.80, +22.80 / −22.81) mm。羽根の重心の高さは STL の Y = 6.6252 mm（羽根の先端の高さ Y = 6.4 mm に対応する位置） |
| 部品の STL | `simulator/shared/assets/meshes/parts/` の `frame`・`pcb`・`battery`・`battery_adapter`・`m5stamps3`・`motor_fl/fr/rl/rr` |

## 3. 結果

| 量 | 丸める前（e-6 kg·m²） | 採用値（e-6 kg·m²） | 2点吊り法の値との比 |
|---|---|---|---|
| $I_{xx}$ | 11.482 | 11.5 | 0.797 |
| $I_{yy}$ | 15.304 | 15.3 | 0.869 |
| $I_{zz}$ | 22.638 | 22.6 | 0.903 |

- 採用しないが記録する量（今回は対角の 3 成分だけを替えた）: 慣性乗積 $I_{13}$ +0.276e-6、$I_{12}$ −0.016e-6、$I_{23}$ +0.017e-6 kg·m²（STL は左右対称なので $I_{12}$・$I_{23}$ は 0 に近く、実機の値は分からない）。重心は STL の原点から x −0.634 mm・y 0.055 mm・z −0.555 mm（機体座標）。
- 検算: プロペラを 1 枚 0.24 g（4 枚で 0.96 g）に戻すと 11.460 / 15.281 / 22.592 になり、解析メモ 3 章の rev4（中央の質量）の結果と 3 桁目まで一致する（スクリプトの最後に表示）。
- 幅（参考。解析メモ 3 章の値の引用で、この計算には使わない。単位 e-6 kg·m²）: 3 通りの主基板の置き方の 95 % の包絡は $I_{xx}$ 10.82〜11.84、$I_{yy}$ 14.38〜15.96、$I_{zz}$ 21.44〜23.61。これにフレームの質量分布の追加の幅が $I_{xx}$ −0.13〜+0.23、$I_{yy}$ −0.13〜+0.24、$I_{zz}$ −0.25〜+0.44。
- 物理の整合: $I_{xx} + I_{yy}$ = 26.8e-6 ≥ $I_{zz}$ = 22.6e-6（剛体では $I_{xx} + I_{yy} - I_{zz} = 2\int z^2\,dm \ge 0$ なので常に成り立つ。満たす。薄い板なら等号）。

---

<a id="english"></a>

# Moments of Inertia from the Part STL Meshes and Measured Masses (2026-11-23)

## 1. Overview

### About This Script

It computes the reference moments of inertia of the StampFly (roll $I_{xx}$, pitch $I_{yy}$, yaw $I_{zz}$). On 2026-11-23 the developer decided to replace the reference values, on all three axes, from the bifilar (2-point suspension) measurement (9.16, 13.3, 20.4 e-6 kg·m²) with the value computed from the part 3D models (STL) and the measured part masses. This script reproduces that computed value using only files in the repository. Background and discussion: `analysis/notes/mass_inertia_measurements_20261123.md`.

### Running

```bash
python3 -I analysis/scripts/inertia_from_parts_20261123/inertia_from_parts.py
```

It needs only Python 3 and numpy. The result is saved as `inertia_from_parts_output.txt` (the same output).

## 2. Definition of the Values (developer's decision)

| Item | Content |
|---|---|
| Masses | Central measured values: motor 17/6 g x4, battery 8 g, frame 3.5 g, StampS3 module 3.25 g, front ToF board (STL `battery_adapter`) 1 g, propeller 1.01/4 g x4. Main board = total 37 g - everything else = 8.907 g |
| Mass distribution | Each part is uniform over its STL volume. The main-board remainder is uniform over the main-board STL shape. The frame is uniform too |
| Coordinates | STL (mm, X = left, Y = up, Z = front) to body frame FRD (x = +Z, y = -X, z = -Y). About the centre of gravity |
| Computation | No random numbers. One computation with the central masses. **The adopted value is the result rounded once to 3 significant digits** |
| Propeller | The four-blade STL is not in the repository, so constants are used: I/m about the axis 55.28 mm^2, in-plane I/m 28.17 mm^2 (values from a shape built from the measured photographs of the public repository `kouhei1970/prop_sim`). Rotor centres at STL (X, Z) = (+-22.80, +22.80 / -22.81) mm. The blade centroid is at STL Y = 6.6252 mm (the position corresponding to a blade-tip height of Y = 6.4 mm) |
| Part STL files | `frame`, `pcb`, `battery`, `battery_adapter`, `m5stamps3`, `motor_fl/fr/rl/rr` in `simulator/shared/assets/meshes/parts/` |

## 3. Result

| Quantity | Before rounding (e-6 kg·m²) | Adopted (e-6 kg·m²) | Ratio of the bifilar value |
|---|---|---|---|
| $I_{xx}$ | 11.482 | 11.5 | 0.797 |
| $I_{yy}$ | 15.304 | 15.3 | 0.869 |
| $I_{zz}$ | 22.638 | 22.6 | 0.903 |

- Recorded but not adopted (only the three diagonal terms were replaced): products of inertia $I_{13}$ +0.276e-6, $I_{12}$ -0.016e-6, $I_{23}$ +0.017e-6 kg·m² (the STL is left-right symmetric, so $I_{12}$ and $I_{23}$ are near 0 and the real values are unknown). The centre of gravity is x -0.634 mm, y 0.055 mm, z -0.555 mm from the STL origin (body frame).
- Cross-check: with the propeller set back to 0.24 g each (0.96 g for four), the result is 11.460 / 15.281 / 22.592, matching the rev4 (central masses) result of chapter 3 of the analysis note to the third decimal (printed at the end of the script).
- Widths (reference; quoted from chapter 3 of the analysis note and not used in the computation; unit e-6 kg·m²): the 95 % envelope of the three main-board placements is $I_{xx}$ 10.82-11.84, $I_{yy}$ 14.38-15.96, $I_{zz}$ 21.44-23.61. The frame mass distribution adds $I_{xx}$ -0.13 to +0.23, $I_{yy}$ -0.13 to +0.24, $I_{zz}$ -0.25 to +0.44.
- Physical consistency: $I_{xx} + I_{yy}$ = 26.8e-6 >= $I_{zz}$ = 22.6e-6 (for any rigid body $I_{xx} + I_{yy} - I_{zz} = 2\int z^2\,dm \ge 0$; satisfied; equality for a thin plate).
