# 部品の質量・寸法の実測と、慣性モーメントの再検討（2026-11-23）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

**状態**: 検討の途中（2026-11-23 作成）。2026-11-23 に開発者が実物で測った部品の質量と寸法、それを使った慣性モーメントの試算、飛行記録からの推定の結果を記録する。**物理パラメータの基準ファイル（`control/models/stampfly_physical.yaml`）と `docs/architecture/stampfly-parameters.md` の値は、この文書では変えていない。** 値を変えるかどうかは開発者がまだ決めていない（8 章）。

## 1. 概要

### このメモについて

StampFly の慣性モーメント（ロール $I_{xx}$・ピッチ $I_{yy}$・ヨー $I_{zz}$）の基準値は、開発者が 2点吊り法（機体を 2 本の糸で水平に吊り、ねじり振動の周期から求める方法）で測った値である（9.16・13.3・20.4 e-6 kg·m²）。3D モデル（`simulator/shared/assets/meshes/parts/` の STL）から慣性を試算すると、この値より 25〜38 % 大きく出た。そこで開発者が部品を 1 つずつ量り、試算をやり直し、飛行記録からも慣性を推定した。本メモはその記録である。

各項目には次の区別を付ける。

| 区別 | 意味 |
|---|---|
| 開発者の測定 | 開発者が実物で量った値・寸法・回答。はかりは台所はかり（刻み 1 g）が基本で、読みの幅は刻みから計算した。ほかの人による独立の確認はない |
| 解析の担当による計算 | 解析を担当した AI の作業者が行った計算。元の作業ファイル（Git 管理外）から値を転記した。別の担当による計算のやり直しは、宙返りの記録（5 章）だけ行った |
| 推論 | 事実から考えたこと。強さ（強・中・弱）と、ほかに考えられる説明を添える |
| 未確認 | まだ確かめていないこと |

### 結論の要約

| 項目 | 内容 | 区別 |
|---|---|---|
| 部品の質量 | モータ 2.83 g、電池 8 g、フレーム 3.5 g、StampS3 モジュール 3.25 g、前方 ToF 基板（電池の受けと同じ部品）1 g、プロペラ 0.2525 g、全質量 37 g、主基板は残りとして約 8.9 g | 開発者の測定 |
| 試算の結果（実測の質量、中央値） | $I_{xx}$ 11.2〜11.6、$I_{yy}$ 15.0〜15.4、$I_{zz}$ 22.2〜22.9（e-6 kg·m²）。見当の質量で計算した段階（フレーム 6.3 g）より 9〜12 % 小さい | 解析の担当による計算 |
| 2点吊り法の値との比 | 2点吊り法の値は試算の中央値の 0.80（$I_{xx}$）・0.87（$I_{yy}$）・0.90（$I_{zz}$） | 解析の担当による計算 |
| 飛行記録からの推定 | 位置保持の飛行（11/22）は励振が無く、ロールは幅が大きすぎて判定に使えない。宙返りの記録（10/02）は、ロールの見かけの慣性 11.4〜12.3e-6 | 解析の担当による計算 |
| プロペラ | 0.2525 g。軸まわりの慣性は形状から 1.40e-8 kg·m²（ベンチ実測のロータの慣性の 91 %）。リポジトリの機体定義の 1 枚 0.001 kg は実物の約 4 倍。STL・Web 表示の羽根は実物と違う | 開発者の測定と計算 |
| 推論 | ロール軸は 3 つの根拠が同じ向きで、2点吊り法の値は約 2 割小さい可能性が高い。ピッチ・ヨーは 5〜8 % の開きで、確かな食い違いとは言えない | 推論（ロール: 中〜強、ピッチ・ヨー: 弱） |

### 対象読者

慣性モーメントと機体の質量配分を扱う人（SILS（ソフトウェアだけで機体と制御を模擬する試験環境）の機体モデル、制御設計、2点吊り法の再測定）。

## 2. 測定値（開発者の測定、2026-11-23）

### 部品の質量

| 部品 | まとめて量った個数と合計 | 1 個あたり | 読みの幅（刻みから） | 備考 |
|---|---|---|---|---|
| モータ | 6 個で 17 g | 2.83 g（2.8333） | 2.75〜2.92 g | 4 個分 11.33 g（11.0〜11.7 g） |
| 電池 | 1 個で 8 g | 8 g | 7.5〜8.5 g | |
| フレーム | 予備 2 個で 7 g | 3.5 g | 3.25〜3.75 g | |
| StampS3 モジュール | 4 個で 13 g | 3.25 g | 3.13〜3.38 g | |
| 前方 ToF の基板（電池の受け） | 1 個で 1 g | 1 g | 0.5〜1.5 g | 距離センサ（ToF）の基板で、電池の受けを兼ねる同じ 1 つの部品 |
| プロペラ | 4 枚で 1.01 g | 0.2525 g | 約 0.25 g | 0.01 g まで読めるはかりと思われるが、はかりの種類は**未確認** |
| 全質量 | 1 機で 37 g | 37 g | 36.5〜37.5 g | 前方 ToF の基板を含む飛行の状態 |
| 主基板（配線・はんだ・コネクタを含む） | 直接は量っていない | 約 8.9 g | 約 7.7〜10.0 g（上の幅からの 95 % 範囲） | 全質量から他の部品を引いた残り。算術: 37 − 11.33 − 8 − 3.5 − 3.25 − 1 − 1.01 = 8.91 g |

主基板の質量は残りで決めたので、他の部品の読みの幅が全部そのまま乗る。直接の実測が最も望まれる値である（8 章）。

### 寸法

| 項目 | 実物（開発者の測定） | 3D モデル（STL） | 合うか |
|---|---|---|---|
| モータの直径 | 7 mm | 6.99 mm | 合う |
| モータの缶の高さ | 16 mm | 缶 16 mm | 合う |
| モータのシャフト | 直径 0.8 mm、出ている長さ 5 mm | 全長 21.0 mm（= 16 + 5） | 合う |
| モータの体積・重心 | （缶の円柱 π × 3.5² × 16 = 615.8 mm³） | 体積 615.5 mm³、重心は上端から 12.95 mm 下 | 合う（缶の中央は上端から 13 mm） |
| プロペラの直径 | 31 mm | 円板 29.99 mm | **合わない**（実物より約 1 mm 小さい）。公開リポジトリ `kouhei1970/prop_sim` の写真からの実測 31.21 mm とは合う |
| プロペラの形 | 羽根 4 枚 | 羽根のない円板とドーム | **合わない** |
| ガードの円筒部の内径 | 約 32 mm | 31.8〜31.9 mm | 合う |
| ガードの円筒部の高さ | 約 5 mm | 円筒の高さ Y ≤ 5.3 mm | 合う |
| ガードの上の縁の直径 | 約 36 mm（上は開き、円筒部の上からほぼ線形に広がる） | 上端の外径 36.2 mm（上端 Y = 8.75 mm） | 形は整合 |
| プロペラ先端の高さ | 広がる部分の下から 1/3 ほどの高さ | 広がり始め Y ≈ 5.3 + (8.75 − 5.3)/3 ≈ 6.4 mm に当たる（STL の円板の上面は 7.8 mm） | 整合（高さの根拠に使える） |
| 先端とガードのすき間 | 半径方向に約 0.4〜0.5 mm（直径 31 mm、内径 32 mm から） | 円板（29.99 mm）では約 0.9〜1.0 mm | 円板は実物よりすき間が広い |
| フレームの体積と質量 | 質量 3.5 g | 体積 4.588 cm³（87.5 % がガードの輪とモータ受け） | 実物の材料の体積に対し 1.4〜1.6 倍相当（下の注） |

フレームの体積の注: STL の体積 4.588 cm³ に樹脂の密度 1.05〜1.20 g/cm³ を掛けると 4.8〜5.5 g で、実測の 3.5 g（3.25〜3.75 g）を超える。STL の体積と実物の材料の体積の比は 1.38〜1.57（読みの幅を入れると 1.28〜1.69）。見かけの密度は 0.763 g/cm³（0.708〜0.817）。STL の肉厚が実物より厚い可能性が高いが、実物の肉厚は**未確認**。

### 確認できた事実（開発者の回答）

| 事実 | 内容 |
|---|---|
| 電池の受けと前方 ToF の基板 | 同じ 1 つの部品。STL の `battery_adapter` がこれにあたる。したがって「3D モデルに前方 ToF の基板が無い」という途中の説明は誤りで、訂正済み |
| 2点吊り法の測定のとき | 前方 ToF の基板は付いていた |
| 2026-10-15 のベンチ測定 | プロペラを機体のフレーム（ガード）に付けたまま行った。ベンチの推力は「羽根とガードの合計」であり、ガードなしの計算とは直接比べられない |

## 3. 3D モデルと実測の質量からの慣性の試算

### 方法（解析の担当による計算）

| 項目 | 内容 |
|---|---|
| 形 | 部品ごとの STL（フレーム・主基板・電池・電池の受け・StampS3・モータ 4 個）。STL は mm 単位で X = 左・Y = 上・Z = 前。機体座標（FRD: 前・右・下）へは x = Z、y = −X、z = −Y に変換。向きの根拠は `stampfly.xml` のコメントと描画スクリプトで、実物での確認は**未確認** |
| 質量の置き方 | 部品ごとに実測の質量を STL の体積に一様に配る。主基板は全質量の残り（中央 8.957 g。プロペラは 0.24 g × 4 を使ったため、2 章の 8.91 g と 0.05 g 違う） |
| 主基板の残りの置き方（3 通り） | (a) 主基板の STL の形に一様に配る。(b) 主基板の裸の基板分 5.47 g（密度 1.85 g/cm³ と仮定）だけを STL に配り、残り 3.48 g を重心の一点に置く。(c) 配線 1.5 g を 4 本の線に沿って置き、残りを主基板に配る |
| フレームの質量の分布 | STL の肉厚が実物より厚い可能性があるので、3.5 g のうち、ガードの輪とモータ受けにどれだけ配るかを動かした（全部を輪に: 上限、アームと中央に樹脂の密度いっぱい: 下限。物理的にありえない「全部アームと中央」は参考） |
| 幅の出し方 | 質量の読みの幅の範囲で一様に引いた乱数（1 シナリオ 240 万組、同じ乱数を共通に使用）。プロペラは三角分布（0.20〜0.35 g、最頻値 0.24 g） |
| プロペラの形 | 公開リポジトリ `kouhei1970/prop_sim` の写真の実測から作った羽根 4 枚。取付角による差は 0.8 % |
| 検算 | 部品ごとの合計と一括計算のテンソルは最大 3.4e-21 kg·m² で一致 |

### 結果（重心まわり、機体座標、単位 1e-6 kg·m²）

| 量 | (a) 一様 | (b) 点質量 | (c) 配線 | 3 通りの 95 % の包絡 | フレーム分布の追加の幅 |
|---|---|---|---|---|---|
| $I_{xx}$（中央値） | 11.50 | 11.17 | 11.57 | 10.82〜11.84 | −0.13〜+0.23 |
| $I_{yy}$（中央値） | 15.32 | 15.04 | 15.42 | 14.38〜15.96 | −0.13〜+0.24 |
| $I_{zz}$（中央値） | 22.68 | 22.24 | 22.94 | 21.44〜23.61 | −0.25〜+0.44 |

| 量 | 値 |
|---|---|
| 慣性乗積 $I_{13}$（行列の要素、$I_{xz}$ に相当。符号は前が +Z の場合） | 0.26〜0.28（(b) の 95 % 範囲 0.11〜0.39）e-6 kg·m²。主軸の傾きは約 2 度。STL は左右対称なので $I_{12}$・$I_{23}$ は 0.02e-6 以下としか出ず、実機の値は分からない |
| 重心（STL の原点から） | x −0.6〜−0.7 mm（後ろ）、z −0.6〜0.0 mm（(b) は +0.04〜+0.07 mm）、y は 0.05 mm 程度 |

### 部品ごとの寄与（シナリオ (a)、中央の質量、単位 1e-6 kg·m²）

| 部品 | 質量 g | $I_{xx}$ | $I_{yy}$ | $I_{zz}$ | 全体に占める割合（$I_{xx}$ / $I_{yy}$ / $I_{zz}$） |
|---|---|---|---|---|---|
| モータ 4 個 | 11.33 | 6.683 | 6.687 | 11.861 | 58 / 44 / 53 % |
| フレーム | 3.50 | 2.037 | 2.031 | 3.815 | 18 / 13 / 17 % |
| 主基板（残り） | 8.96 | 1.244 | 1.113 | 1.549 | 11 / 7 / 7 % |
| 電池 | 8.00 | 0.236 | 3.150 | 3.077 | 2 / 21 / 14 % |
| 電池の受け（前方 ToF 基板） | 1.00 | 0.034 | 0.998 | 0.974 | 0 / 7 / 4 % |
| StampS3 モジュール | 3.25 | 0.665 | 0.741 | 0.265 | 6 / 5 / 1 % |
| プロペラ 4 枚 | 0.96 | 0.562 | 0.562 | 1.052 | 5 / 4 / 5 % |
| 合計 | 37.00 | 11.460 | 15.281 | 22.592 | 100 % |

### 見当の質量で計算していた段階からの経過

| 段階 | 質量の置き方 | 試算 $I_{xx}$ / $I_{yy}$ / $I_{zz}$（e-6） | 2点吊り法の値に対する割合 |
|---|---|---|---|
| rev3（見当） | モータ 11.0 g、フレーム 6.33 g、主基板 6.43 g、電池 8.13 g ほか | 12.52 / 16.54 / 25.05 | +37 / +24 / +23 % |
| rev4（実測、(a)） | 上の 2 章の値 | 11.46 / 15.28 / 22.59 | +25 / +15 / +11 % |

フレームの見当 6.3 g が実測 3.5 g に下がったことで、フレームの寄与が $I_{xx}$ で 1.65、$I_{zz}$ で 3.09（e-6）減った。これが変化の大半である。実測のモータ質量は見当（11.0 g）より 0.3 g 重く、「モータが軽ければ計測値に近づく」という途中の推論は否定された。

## 4. 2点吊り法の測定値との比較

### 測定記録（`control/models/stampfly_physical.yaml` の Ixx の source、2026-10-30 に開発者が提示）

式: $J = \dfrac{m g D^2 T^2}{16\, h\, \pi^2}$（$m$: 測定時の質量 = 0.0364 kg、$g$ = 9.80665 m/s²、$D$: 糸の間隔、$h$: 糸の長さ、$T$: ねじり振動の周期）。

| 軸 | $h$ (m) | $D$ (m) | $T$ (s) | 記録から計算（$m$ = 0.0364 kg） | 基準値 |
|---|---|---|---|---|---|
| x（ロール） | 0.282 | 0.078 | 0.43404 | 9.188e-6 | 9.16e-6（出所は yaml で未確認、0.3 % 違う） |
| y（ピッチ） | 0.350 | 0.078 | 0.58230 | 13.32e-6 | 13.3e-6 |
| z（ヨー） | 0.370 | 0.100 | 0.57815 | 20.42e-6 | 20.4e-6 |

（再計算は解析の担当。質量を今回の 37 g にしても 1.6 % しか変わらず、9.34 / 13.54 / 20.76 e-6 になる。）

### 試算との比較

| 軸 | 2点吊り法 (e-6) | 試算の中央値 (a) | 2点吊り法 ÷ 試算 | 包絡の下限（95 %）に対して |
|---|---|---|---|---|
| $I_{xx}$ | 9.16 | 11.50 | 0.80 | −15 % |
| $I_{yy}$ | 13.3 | 15.32 | 0.87 | −7.5 % |
| $I_{zz}$ | 20.4 | 22.68 | 0.90 | −4.8 % |

試算の側から見ると 2点吊り法に対して +25 %（$I_{xx}$）・+15 %（$I_{yy}$）・+11 %（$I_{zz}$）。(b)・(c) を含めた中央値の範囲では +22〜26 / +13〜16 / +9〜13 %。

加えて、フレームを入れる前の積み上げでも $I_{xx}$ は約 8.9e-6（モータ 4 個 6.6、プロペラ 0.57、モジュール 0.65、主基板 0.9、電池 0.2。解析の担当の寄与の表の値を、解析の進行役が足した手計算）で、2点吊り法の 9.16e-6 にほぼ届く。フレームのガードの輪は左右 ±22.8 mm にあり、2 g でも $I_{xx}$ に +1e-6 以上を足す。

## 5. 飛行記録からの推定

### (a) 2026-11-22 の位置保持などの飛行（解析の担当による計算）

| 項目 | 結果 |
|---|---|
| 方法 | ベンチの推進系の定数から推力を作り、角加速度との関係から慣性を推定。事前に規則（`plan.md`）を決めた。外部の励振が無い記録が大半なので、PWM（モータの駆動信号）の 8 bit の切り捨て誤差を外乱として使う方法を、結果を見た後に追加した |
| ロール | $I_{xx}$ = 7.2e-6（推力の尺度の補正なし 7.53e-6、あり 6.96e-6 の幾何平均）、幅 ±3.8〜5.8e-6（大半はモータの遅れの扱い）。帯域別: 6〜12 Hz で 8.6〜9.8e-6、12〜24 Hz で 7.0〜7.5e-6、24〜48 Hz で 6.0〜6.2e-6（高い帯域は合成検査で下側に偏る挙動） |
| ピッチ・ヨー | 決められない（ピッチ 0.9〜3.5e-5、ヨー 1.9〜9e-5 の範囲で、受理規則を満たす帯域が少ない） |
| 判定 | 事前の規則（不確かさ ±12 % 以内が条件）を満たさず、**判定に使えない** |
| ホバーでのベンチのモデルの推力 ÷ 重さ | 保持・離着陸の 5 記録で 1.08〜1.16、手動ホバーの 1 記録で 0.98（集計ファイルの `scale_factor` の逆数） |

### (b) 2026-10-02 の宙返りの記録（別の担当による再計算）

右ロールの宙返り（`flip_test1`・`flip_test2`）。モータは 0 と 1（デューティの下限・上限）に飽和し、ロール角速度のピークは 1875〜1926 deg/s。ベンチのモータの微分方程式（ODE）で推力を作り、ロールの見かけの慣性 $J$ を、回転中・制動中・全体の 3 つの窓で当てはめた。

| 追加の 1 次遅れ | $J$ の範囲 (e-6 kg·m²) | 残差（相対） |
|---|---|---|
| 0 ms（むだ時間 4 ms 付きの窓。残差が小さい版） | 11.49〜12.28（3 窓・2 本、尺度 1.0） | 0.03〜0.07 |
| 0 ms（むだ時間 0 ms） | 11.74〜12.47 | 0.12〜0.20 |
| 15 ms | 10.87〜11.29 | 0.12〜0.29 |
| 35 ms | 9.27〜10.10 | 0.27〜0.62 |
| 55 ms | 8.0〜10.5 | 0.33〜0.82 |

- 当てはめの窓は短い（回転中 0.12 s、制動中 0.10 s、全体 0.22 s）。「むだ時間」の列は、推力を後ろへずらした量（0 または 4 ms）。
- 遅れを 0 ms 付近で全パラメータを掃引した最良の組（`sweep`）は $J$ = 11.4〜12.3e-6（遅れ 5 ms、むだ時間 2 ms、残差 0.02〜0.06）。範囲 11.4〜12.3e-6 を本メモの代表とする。統計の幅は ±0.1〜0.3e-6。
- 宙返りの前のホバーでの推力の尺度は 1.009（`flip_test1`）・1.010（`flip_test2`）。尺度を掛けても $J$ は約 1 % しか動かない。
- 追加の遅れを大きくすると $J$ は小さくなるが、残差は悪化する。
- 推力の尺度（$s$）と $J$ は原理的に分離できない（角加速度 ∝ $s$ / $J$）。以前の照合（`ct_cq_bench_20261015.md` の「宙返り（大信号）」）で「慣性 9.16e-6 を固定すると推力の倍率 0.79〜0.80 が必要」としたものと**同じ事実**である: 9.16 / 0.80 = 11.45e-6。
- 別の既存ファイル `inertia_eff`（`crosscheck/results/test3.json`）は宙返りではなく、ホバー中の励振のレートループの当てはめの 1/b（指令のトルクを真とみなした見かけの値、15 本でロール 1.1〜3.5e-5 とばらつく）であり、使えない。
- 11/22 のホバーの推力の尺度（保持の記録で 1.08〜1.16）と、10/02 の宙返り前のホバー（1.009〜1.010）は違う。理由は**未確認**（8 章）。

## 6. プロペラ

| 項目 | 値 | 区別 |
|---|---|---|
| 質量（実測） | 0.2525 g（4 枚で 1.01 g） | 開発者の測定 |
| 形からの見積もり | 0.240 g（実測の羽根 4 枚の形）、`prop_sim` の文書は 0.26 g（面積 × 板厚 × ポリカーボネートの密度） | 解析の担当 |
| 軸まわりの慣性（形状の $I/m$ = 55.28 mm² × 0.2525 g） | $1.40\times10^{-8}$ kg·m² | 計算 |
| ベンチ実測のロータの慣性 | $1.54\times10^{-8}$ kg·m²（形状の値はその 91 %。残り $0.14\times10^{-8}$ はモータの回転子の分と読める） | 推論（中） |
| 試算 rev4 に使った質量 | 0.24 g × 4 = 0.96 g。実測（1.01 g）との差 0.05 g が機体の慣性に与える影響は 0.3 % 未満 | 解析の担当 |
| リポジトリの機体の定義（URDF） | プロペラのリンク 1 枚 0.001 kg、慣性 1e-9。実物（0.0002525 kg）の約 4 倍 | 調査 |
| STL | `propeller_{fl,fr,rl,rr}.stl`（各 690 三角形）は円板。読む側は `simulator/sils/models/stampfly.xml`（表示専用）・URDF 2 本・`mesh_inspect.cpp` | 調査 |
| Web 表示の羽根 | STL を読まず、3 枚・直径 29.98 mm をコードで生成（`landing/index.html`、`simulator/sils/gui/static/app.js`、`tools/log_analyzer/replay_video/page/replay_view.js`、`lib/sfcli/assets/telemetry_web.html`、`tools/flasher_gui/assets/gen_icon_3d.py`、`docs/events/_shared/tikz/scripts/render_stampfly_iso.py` の 6 か所）。実物は 4 枚・31 mm | 調査 |

形状データの置き換え（実測の羽根 4 枚の STL、約 2.3 万三角形、現行の約 34 倍の大きさ）は**計画の段階**で、着手していない。取付角の版の選定は、別に進めている数値流体計算（CFD）の検討の結果と開発者の決定を待っている。

## 7. 推論

| 推論 | 根拠 | 強さ | ほかの説明・弱点 |
|---|---|---|---|
| ロール軸では、2点吊り法の値は約 2 割小さい | 3 つの線が同じ向き: (1) 実測のモータ・プロペラなどの積み上げだけで 8.9e-6（フレームを入れる前）。(2) 試算 11.2〜11.6e-6。(3) 宙返り 11.4〜12.3e-6（ホバー前に推力の尺度を確かめた上で） | 中〜強 | (1)(2) は同じ質量・形状の入力を共有する。(3) は推力の尺度と分離できない（5 章）。主基板の質量が実測でない |
| ピッチ・ヨーは確かな食い違いとは言えない | 2点吊り法が試算の 0.87・0.90。包絡の下限に対して −7.5 %・−4.8 % で、フレームの分布の幅と主基板の置き方の幅に近い | 弱 | 飛行記録からは決められていない（5 章） |
| 「宙返りでは推力が 0.8 倍」は、慣性が約 1.25〜1.3 倍なら推力の補正なしで説明できる | 5 章(b) の掃引。推力の尺度 1.0 のままで $J$ = 11.4〜12.3e-6 が当てはまる | 中 | 同じ事実の別の言い方で、独立な根拠ではない。飽和時（デューティ 1）の推力はホバーでの確認の外 |

### 2点吊り法で値が小さく出る原因の候補（すべて未確認）

| 候補 | 影響の向き | 備考 |
|---|---|---|
| 糸の間隔 $D$ の読み | $J \propto D^2$ なので、読みが小さければ小さく出る | 数 % の読み違いで 2 割には届きにくい（10 % の違いで 20 %） |
| 機体側と上側で糸の間隔が違う | 式は平行な糸（上の固定点の間隔 = 機体側の取り付け点の間隔 = $D$）を仮定している。間隔が違う場合、$J$ は（上の間隔）×（機体側の間隔）に比例する。機体側のほうが広ければ、$D^2$ で計算した値は小さく出る | ロール軸の 2 割を説明するには、機体側が約 98 mm（78 mm × 1.25）である必要がある |
| 糸の曲げ・ねじりの剛性による復元力の上乗せ | 周期 $T$ が短くなり、小さく出る | 機体が軽いので、2 本の糸による復元トルクの係数は $m g D^2/(4h) \approx 1.9\times10^{-3}$ N·m/rad（ロール軸の記録の値で計算）と小さい。糸の剛性がこれに対して無視できるかは未確認 |
| 軸ごとの吊り直し | 吊る位置のずれで周期が変わる | 3 軸で吊り方が違う（$h$・$D$ が違う） |

## 8. 未確認と次の作業

| 項目 | 内容 |
|---|---|
| 主基板の質量の実測 | 残りとして 8.9 g と決めた値を直接量る。配線・はんだ・コネクタを含む |
| 2点吊り法の治具の確認 | 慣性が計算で分かる物体（円柱の棒、金属の板など）を同じ治具・手順で測り、式と手順の誤りを調べる |
| ピッチ・ヨーの独立の推定 | 外から励振を入れる試験（ホバー中に指令へ決まった入力を加える）で、ピッチとヨーの慣性を飛行記録から決める |
| 宙返りの最大の指令での推力の尺度 | 飽和時の推力の尺度は、ホバーでの確認の外 |
| ホバーの推力の尺度の違い | 11/22（保持の記録で 1.08〜1.16）と 10/02（1.009〜1.010）で違う理由 |
| 基準の値を変えるかどうかの決定 | 開発者の決定を待つ。変える場合に連動する箇所: `control/models/stampfly_physical.yaml`、SILS の機体モデル（`stampfly.xml` の慣性、`fullinertia` と重心 `pos` への置き換えも含む）、URDF（2 本）、制御設計の道具、`sf params check` |
| プロペラの形状データの置き換え | 6 章。URDF の質量、STL、Web 表示（6 か所） |
| STL の向きの物理的な確認 | X = 左・Y = 上・Z = 前の対応は実物で未確認 |
| フレームの実物の肉厚 | STL の体積が実物の 1.4〜1.6 倍相当の原因の確認 |
| はかりの種類と刻み | プロペラの質量を量ったはかりの種類 |

## 9. 作業ファイルの場所

| 内容 | 場所 |
|---|---|
| 3D モデルからの試算（rev4） | 解析の作業ファイル（Git 管理外）。`rev4_log.txt`・`rev4_compare.csv`・`rev4_parts.csv`・`rev4_frame.csv`・`rev4_mc_summary.csv` |
| 宙返りの再計算 | 解析の作業ファイル（Git 管理外）。`flipJ.log`・`sweep.log` |
| 飛行記録からの推定 | 解析の作業ファイル（Git 管理外）。事前規則の記録 `plan.md` と集計 |
| 2点吊り法の測定記録 | `control/models/stampfly_physical.yaml` の Ixx の source |
| 公開リポジトリ | `kouhei1970/prop_sim`（プロペラの写真の実測と BEMT = 翼素運動量理論）、`kouhei1970/stampfly-motor-autotest`（ベンチの自動測定） |

---

<a id="english"></a>

# Component Mass and Dimension Measurements and a Re-examination of the Moments of Inertia (2026-11-23)

**Status**: Work in progress (written 2026-11-23). This note records the component masses and dimensions the developer measured on the physical vehicle on 2026-11-23, an inertia estimate computed with them, and the result of estimating inertia from flight logs. **The values in the physical-parameter reference file (`control/models/stampfly_physical.yaml`) and in `docs/architecture/stampfly-parameters.md` are not changed by this note.** The developer has not yet decided whether to change them (chapter 8).

## 1. Overview

### About This Note

The reference moments of inertia of StampFly (roll $I_{xx}$, pitch $I_{yy}$, yaw $I_{zz}$) are the developer's bifilar (two-string torsion pendulum) measurements: 9.16, 13.3, 20.4 e-6 kg·m². An estimate from the 3D model (the STL files in `simulator/shared/assets/meshes/parts/`) came out 25 to 38 % larger. The developer therefore weighed the components one by one; the estimate was redone, and inertia was also estimated from flight logs. This note is the record.

Each item carries one of the following labels.

| Label | Meaning |
|---|---|
| Developer's measurement | Values, dimensions, and answers the developer obtained on the physical vehicle. Kitchen scale (1 g resolution) in most cases; the reading widths were computed from the resolution. Not independently confirmed by anyone else |
| Computed by the analysis assistant | Calculations by the AI worker assigned to the analysis. Values were transcribed from the working files (not under Git). Only the flip-log result (chapter 5) was recomputed by a separate assistant |
| Inference | Reasoning from the facts, with a strength (strong / medium / weak) and alternative explanations |
| Unconfirmed | Not yet checked |

### Summary of Conclusions

| Item | Content | Label |
|---|---|---|
| Component masses | Motor 2.83 g, battery 8 g, frame 3.5 g, StampS3 module 3.25 g, front ToF board (same part as the battery holder) 1 g, propeller 0.2525 g, total 37 g; the main board is about 8.9 g as the remainder | Developer's measurement |
| Estimate with measured masses (median) | $I_{xx}$ 11.2-11.6, $I_{yy}$ 15.0-15.4, $I_{zz}$ 22.2-22.9 (e-6 kg·m²); 9-12 % smaller than with the guessed masses (frame 6.3 g) | Computed by the analysis assistant |
| Ratio of the bifilar values | The bifilar values are 0.80 ($I_{xx}$), 0.87 ($I_{yy}$), 0.90 ($I_{zz}$) of the estimate medians | Computed by the analysis assistant |
| Estimate from flight logs | The position-hold flights (11/22) had no excitation; the roll width is too large for a verdict. The flip log (10/02) gives an apparent roll inertia of 11.4-12.3e-6 | Computed by the analysis assistant |
| Propeller | 0.2525 g. Spin-axis inertia from the shape: 1.40e-8 kg·m² (91 % of the bench-measured rotor inertia). The 0.001 kg per propeller in the repository's vehicle definition is about 4 times the real one. The STL and Web-display blades differ from the real propeller | Developer's measurement and calculation |
| Inference | On the roll axis three lines of evidence point the same way; the bifilar value is probably about 20 % too small. Pitch and yaw differ by 5-8 %, which is not a clear discrepancy | Inference (roll: medium to strong; pitch and yaw: weak) |

### Target Audience

Anyone working with moments of inertia and the mass distribution of the vehicle (the SILS vehicle model, control design, re-measurement by the bifilar method). SILS = a test environment that simulates the vehicle and controller in software only.

## 2. Measurements (Developer's Measurement, 2026-11-23)

### Component Masses

| Component | Count weighed together and total | Per piece | Reading width (from resolution) | Remark |
|---|---|---|---|---|
| Motor | 6 pieces, 17 g | 2.83 g (2.8333) | 2.75-2.92 g | 4 pieces: 11.33 g (11.0-11.7 g) |
| Battery | 1 piece, 8 g | 8 g | 7.5-8.5 g | |
| Frame | 2 spares, 7 g | 3.5 g | 3.25-3.75 g | |
| StampS3 module | 4 pieces, 13 g | 3.25 g | 3.13-3.38 g | |
| Front ToF board (battery holder) | 1 piece, 1 g | 1 g | 0.5-1.5 g | Board of the distance sensor (ToF); the same single part as the battery holder |
| Propeller | 4 pieces, 1.01 g | 0.2525 g | about 0.25 g | Probably a scale reading to 0.01 g, but the scale type is **unconfirmed** |
| Total mass | 1 vehicle, 37 g | 37 g | 36.5-37.5 g | Flight configuration, including the front ToF board |
| Main board (with wiring, solder, connectors) | Not weighed directly | about 8.9 g | about 7.7-10.0 g (95 % range from the widths above) | Remainder after subtracting the other parts from the total. Arithmetic: 37 − 11.33 − 8 − 3.5 − 3.25 − 1 − 1.01 = 8.91 g |

Because the main board is a remainder, the reading widths of all other parts carry over to it. A direct measurement is the most desirable one (chapter 8).

### Dimensions

| Item | Real (developer's measurement) | 3D model (STL) | Agreement |
|---|---|---|---|
| Motor diameter | 7 mm | 6.99 mm | Agrees |
| Motor can height | 16 mm | can 16 mm | Agrees |
| Motor shaft | diameter 0.8 mm, protruding 5 mm | total length 21.0 mm (= 16 + 5) | Agrees |
| Motor volume and centre of mass | (can cylinder π × 3.5² × 16 = 615.8 mm³) | volume 615.5 mm³, centre of mass 12.95 mm below the top | Agrees (can centre is 13 mm below the top) |
| Propeller diameter | 31 mm | disc 29.99 mm | **Disagrees** (about 1 mm smaller than real). Agrees with 31.21 mm measured from photos in the public repository `kouhei1970/prop_sim` |
| Propeller shape | 4 blades | bladeless disc with dome | **Disagrees** |
| Guard cylinder inner diameter | about 32 mm | 31.8-31.9 mm | Agrees |
| Guard cylinder height | about 5 mm | cylinder height Y ≤ 5.3 mm | Agrees |
| Guard top rim diameter | about 36 mm (open at the top; widens almost linearly above the cylinder) | top outer diameter 36.2 mm (top at Y = 8.75 mm) | Shapes are consistent |
| Propeller tip height | about 1/3 up the widening part | corresponds to the start of widening Y ≈ 5.3 + (8.75 − 5.3)/3 ≈ 6.4 mm (top face of the STL disc is 7.8 mm) | Consistent (usable as the height basis) |
| Tip-to-guard clearance | about 0.4-0.5 mm radially (from 31 mm diameter and 32 mm inner diameter) | about 0.9-1.0 mm with the disc (29.99 mm) | The disc leaves a larger clearance than the real one |
| Frame volume and mass | mass 3.5 g | volume 4.588 cm³ (87.5 % in the guard rings and motor seats) | Equivalent to 1.4-1.6 times the real material volume (note below) |

Note on the frame volume: multiplying the STL volume 4.588 cm³ by a resin density of 1.05-1.20 g/cm³ gives 4.8-5.5 g, exceeding the measured 3.5 g (3.25-3.75 g). The ratio of the STL volume to the real material volume is 1.38-1.57 (1.28-1.69 including the reading width). The apparent density is 0.763 g/cm³ (0.708-0.817). The STL wall is probably thicker than the real one, but the real wall thickness is **unconfirmed**.

### Confirmed Facts (Developer's Answers)

| Fact | Content |
|---|---|
| Battery holder and front ToF board | One and the same part; the STL `battery_adapter` is this part. The interim statement "the 3D model has no front ToF board" was wrong and has been corrected |
| During the bifilar measurement | The front ToF board was attached |
| 2026-10-15 bench measurement | Done with the propeller left mounted in the vehicle frame (guard). The bench thrust is the sum of blade and guard, so it cannot be compared directly with a guard-less calculation |

## 3. Inertia Estimate from the 3D Model and the Measured Masses

### Method (Computed by the Analysis Assistant)

| Item | Content |
|---|---|
| Shape | Per-part STL (frame, main board, battery, battery holder, StampS3, 4 motors). The STL is in mm with X = left, Y = up, Z = front. Converted to the body frame (FRD: forward, right, down) by x = Z, y = −X, z = −Y. The basis is a comment in `stampfly.xml` and a rendering script; confirmation on the real vehicle is **unconfirmed** |
| Mass assignment | The measured mass of each part is spread uniformly over the STL volume. The main board takes the remainder of the total (median 8.957 g; 0.05 g different from the 8.91 g of chapter 2 because 0.24 g × 4 was used for the propellers) |
| Placement of the main-board remainder (3 ways) | (a) Spread uniformly over the main-board STL shape. (b) Only the bare-board part 5.47 g (assuming 1.85 g/cm³) over the STL; the other 3.48 g as a point mass at the centre of mass. (c) 1.5 g of wiring along four lines, the rest over the main board |
| Frame mass distribution | The STL wall may be thicker than the real one, so how much of the 3.5 g goes to the guard rings and motor seats was varied (all in the rings: upper bound; arms and centre at full resin density: lower bound; "all in arms and centre" is physically impossible and given for reference) |
| Width | Random draws, uniform inside the reading ranges (2.4 million sets per scenario, same draws shared). Propeller: triangular distribution (0.20-0.35 g, mode 0.24 g) |
| Propeller shape | Four blades built from the photo measurements in the public repository `kouhei1970/prop_sim`. The effect of the pitch-angle version is 0.8 % |
| Check | Part-by-part sum and the vectorised tensor agree to at most 3.4e-21 kg·m² |

### Result (about the centre of mass, body frame, unit 1e-6 kg·m²)

| Quantity | (a) uniform | (b) point mass | (c) wiring | 95 % envelope over the 3 | Extra width from the frame distribution |
|---|---|---|---|---|---|
| $I_{xx}$ (median) | 11.50 | 11.17 | 11.57 | 10.82-11.84 | −0.13 to +0.23 |
| $I_{yy}$ (median) | 15.32 | 15.04 | 15.42 | 14.38-15.96 | −0.13 to +0.24 |
| $I_{zz}$ (median) | 22.68 | 22.24 | 22.94 | 21.44-23.61 | −0.25 to +0.44 |

| Quantity | Value |
|---|---|
| Product of inertia $I_{13}$ (matrix element, corresponds to $I_{xz}$; sign for front = +Z) | 0.26-0.28 (95 % range of (b): 0.11-0.39) e-6 kg·m². Principal-axis tilt about 2 deg. The STL is left-right symmetric, so $I_{12}$ and $I_{23}$ come out only as 0.02e-6 or below; the real values are unknown |
| Centre of mass (from the STL origin) | x −0.6 to −0.7 mm (rearward), z −0.6 to 0.0 mm ((b): +0.04 to +0.07 mm), y about 0.05 mm |

### Per-part Contributions (scenario (a), central masses, unit 1e-6 kg·m²)

| Part | Mass g | $I_{xx}$ | $I_{yy}$ | $I_{zz}$ | Share of total ($I_{xx}$ / $I_{yy}$ / $I_{zz}$) |
|---|---|---|---|---|---|
| 4 motors | 11.33 | 6.683 | 6.687 | 11.861 | 58 / 44 / 53 % |
| Frame | 3.50 | 2.037 | 2.031 | 3.815 | 18 / 13 / 17 % |
| Main board (remainder) | 8.96 | 1.244 | 1.113 | 1.549 | 11 / 7 / 7 % |
| Battery | 8.00 | 0.236 | 3.150 | 3.077 | 2 / 21 / 14 % |
| Battery holder (front ToF board) | 1.00 | 0.034 | 0.998 | 0.974 | 0 / 7 / 4 % |
| StampS3 module | 3.25 | 0.665 | 0.741 | 0.265 | 6 / 5 / 1 % |
| 4 propellers | 0.96 | 0.562 | 0.562 | 1.052 | 5 / 4 / 5 % |
| Total | 37.00 | 11.460 | 15.281 | 22.592 | 100 % |

### Progress from the Stage with Guessed Masses

| Stage | Mass assignment | Estimate $I_{xx}$ / $I_{yy}$ / $I_{zz}$ (e-6) | Relative to the bifilar value |
|---|---|---|---|
| rev3 (guess) | motors 11.0 g, frame 6.33 g, main board 6.43 g, battery 8.13 g, etc. | 12.52 / 16.54 / 25.05 | +37 / +24 / +23 % |
| rev4 (measured, (a)) | values of chapter 2 | 11.46 / 15.28 / 22.59 | +25 / +15 / +11 % |

The frame mass dropping from the guessed 6.3 g to the measured 3.5 g reduced the frame contribution by 1.65 ($I_{xx}$) and 3.09 ($I_{zz}$) (e-6); this is most of the change. The measured motor mass is 0.3 g heavier than the guess (11.0 g), so the interim inference "lighter motors would bring the estimate to the measured value" was refuted.

## 4. Comparison with the Bifilar Measurement

### Measurement Record (source of Ixx in `control/models/stampfly_physical.yaml`, presented by the developer on 2026-10-30)

Formula: $J = \dfrac{m g D^2 T^2}{16\, h\, \pi^2}$ ($m$: mass at measurement = 0.0364 kg, $g$ = 9.80665 m/s², $D$: string spacing, $h$: string length, $T$: torsional period).

| Axis | $h$ (m) | $D$ (m) | $T$ (s) | Computed from the record ($m$ = 0.0364 kg) | Reference value |
|---|---|---|---|---|---|
| x (roll) | 0.282 | 0.078 | 0.43404 | 9.188e-6 | 9.16e-6 (origin unconfirmed in the yaml; 0.3 % different) |
| y (pitch) | 0.350 | 0.078 | 0.58230 | 13.32e-6 | 13.3e-6 |
| z (yaw) | 0.370 | 0.100 | 0.57815 | 20.42e-6 | 20.4e-6 |

(Recomputed by the analysis assistant. Using today's 37 g changes the result by only 1.6 %: 9.34 / 13.54 / 20.76 e-6.)

### Comparison with the Estimate

| Axis | Bifilar (e-6) | Estimate median (a) | Bifilar ÷ estimate | Relative to the lower 95 % envelope |
|---|---|---|---|---|
| $I_{xx}$ | 9.16 | 11.50 | 0.80 | −15 % |
| $I_{yy}$ | 13.3 | 15.32 | 0.87 | −7.5 % |
| $I_{zz}$ | 20.4 | 22.68 | 0.90 | −4.8 % |

Seen from the estimate side, it is +25 % ($I_{xx}$), +15 % ($I_{yy}$), +11 % ($I_{zz}$) relative to the bifilar value (+22-26 / +13-16 / +9-13 % over the medians of (a) to (c)).

In addition, the build-up before adding the frame already gives $I_{xx}$ of about 8.9e-6 (4 motors 6.6, propellers 0.57, module 0.65, main board 0.9, battery 0.2; a hand sum of the values in the analysis assistant's contribution table, made by the analysis coordinator), nearly reaching the bifilar 9.16e-6. The guard rings of the frame lie at ±22.8 mm laterally, and even 2 g adds 1e-6 or more to $I_{xx}$.

## 5. Estimation from Flight Logs

### (a) Position-hold and Other Flights of 2026-11-22 (Computed by the Analysis Assistant)

| Item | Result |
|---|---|
| Method | Thrust is built from the bench propulsion constants and related to angular acceleration to estimate inertia. Rules were fixed beforehand (`plan.md`). Because most logs have no external excitation, a method using the 8-bit truncation error of the PWM (motor drive signal) as the disturbance was added after seeing results |
| Roll | $I_{xx}$ = 7.2e-6 (geometric mean of 7.53e-6 without and 6.96e-6 with thrust-scale correction), width ±3.8-5.8e-6 (mostly from the treatment of motor lag). By band: 8.6-9.8e-6 at 6-12 Hz, 7.0-7.5e-6 at 12-24 Hz, 6.0-6.2e-6 at 24-48 Hz (the high bands show a downward bias in the synthetic check) |
| Pitch, yaw | Not determinable (pitch 0.9-3.5e-5, yaw 1.9-9e-5; few bands satisfy the acceptance rule) |
| Verdict | Does not satisfy the preset rule (uncertainty within ±12 %); **not usable for a verdict** |
| Bench-model thrust ÷ weight at hover | 1.08-1.16 in the 5 hold / takeoff-landing logs, 0.98 in the 1 manual-hover log (inverse of `scale_factor` in the summary file) |

### (b) Flip Log of 2026-10-02 (Recomputed by a Separate Assistant)

Right-roll flips (`flip_test1`, `flip_test2`). The motors saturate at 0 and 1 (duty lower and upper limits), roll-rate peak 1875-1926 deg/s. Thrust was built with the bench motor differential equation (ODE), and the apparent roll inertia $J$ was fitted in three windows: spinning, braking, and whole.

| Extra first-order lag | Range of $J$ (e-6 kg·m²) | Relative residual |
|---|---|---|
| 0 ms (windows with 4 ms dead time; the smaller-residual version) | 11.49-12.28 (3 windows, 2 logs, scale 1.0) | 0.03-0.07 |
| 0 ms (dead time 0 ms) | 11.74-12.47 | 0.12-0.20 |
| 15 ms | 10.87-11.29 | 0.12-0.29 |
| 35 ms | 9.27-10.10 | 0.27-0.62 |
| 55 ms | 8.0-10.5 | 0.33-0.82 |

- The fitting windows are short (spinning 0.12 s, braking 0.10 s, whole 0.22 s). The dead-time entry is the shift of the thrust series (0 or 4 ms).
- The best parameter set from the sweep (`sweep`) near 0 ms lag is $J$ = 11.4-12.3e-6 (lag 5 ms, dead time 2 ms, residual 0.02-0.06). The range 11.4-12.3e-6 is taken as the representative value in this note. Statistical width ±0.1-0.3e-6.
- The thrust scale at hover before the flip is 1.009 (`flip_test1`) and 1.010 (`flip_test2`). Applying it moves $J$ by only about 1 %.
- A larger extra lag lowers $J$ but worsens the residual.
- The thrust scale ($s$) and $J$ cannot be separated in principle (angular acceleration ∝ $s$ / $J$). This is the **same fact** as the earlier check (`ct_cq_bench_20261015.md`, "Flip (large signal)"), which needed a thrust factor 0.79-0.80 with inertia fixed at 9.16e-6: 9.16 / 0.80 = 11.45e-6.
- An existing file `inertia_eff` (`crosscheck/results/test3.json`) is not from a flip but the 1/b of a fit of the rate loop to hover excitation (an apparent value that treats the commanded torque as true; 1.1-3.5e-5 scatter over 15 runs for roll) and is unusable.
- The hover thrust scale of 11/22 (1.08-1.16 in the hold logs) differs from that of 10/02 before the flip (1.009-1.010). The reason is **unconfirmed** (chapter 8).

## 6. Propeller

| Item | Value | Label |
|---|---|---|
| Mass (measured) | 0.2525 g (1.01 g for 4) | Developer's measurement |
| Estimate from the shape | 0.240 g (measured 4-blade shape); the `prop_sim` document gives 0.26 g (area × thickness × polycarbonate density) | Analysis assistant |
| Spin-axis inertia (shape $I/m$ = 55.28 mm² × 0.2525 g) | $1.40\times10^{-8}$ kg·m² | Calculation |
| Bench-measured rotor inertia | $1.54\times10^{-8}$ kg·m² (the shape value is 91 % of it; the remaining $0.14\times10^{-8}$ reads as the motor rotor) | Inference (medium) |
| Mass used in estimate rev4 | 0.24 g × 4 = 0.96 g. The 0.05 g difference from the measured 1.01 g changes the vehicle inertia by less than 0.3 % | Analysis assistant |
| Repository vehicle definition (URDF) | 0.001 kg per propeller link, inertia 1e-9; about 4 times the real 0.0002525 kg | Survey |
| STL | `propeller_{fl,fr,rl,rr}.stl` (690 triangles each) are discs. Readers: `simulator/sils/models/stampfly.xml` (display only), 2 URDF files, `mesh_inspect.cpp` | Survey |
| Web-display blades | Not read from the STL; generated in code as 3 blades of 29.98 mm diameter (`landing/index.html`, `simulator/sils/gui/static/app.js`, `tools/log_analyzer/replay_video/page/replay_view.js`, `lib/sfcli/assets/telemetry_web.html`, `tools/flasher_gui/assets/gen_icon_3d.py`, `docs/events/_shared/tikz/scripts/render_stampfly_iso.py`: 6 places). The real one has 4 blades and 31 mm | Survey |

Replacing the shape data (an STL of the measured 4 blades, about 23,000 triangles, roughly 34 times the current size) is **at the planning stage** and has not been started. The choice of the pitch-angle version awaits the result of a separately progressing computational fluid dynamics (CFD) study and the developer's decision.

## 7. Inference

| Inference | Basis | Strength | Alternatives and weaknesses |
|---|---|---|---|
| On the roll axis the bifilar value is about 20 % too small | Three lines point the same way: (1) the build-up of measured motors, propellers, etc. alone gives 8.9e-6 (before adding the frame). (2) the estimate 11.2-11.6e-6. (3) the flip 11.4-12.3e-6 (after checking the thrust scale at hover) | Medium to strong | (1) and (2) share the same mass and shape inputs. (3) cannot be separated from the thrust scale (chapter 5). The main-board mass is not measured |
| Pitch and yaw are not a clear discrepancy | The bifilar values are 0.87 and 0.90 of the estimate; −7.5 % and −4.8 % relative to the lower envelope, close to the width from the frame distribution and the main-board placement | Weak | Not determined from flight logs (chapter 5) |
| "The thrust is 0.8 times in the flip" is explained without any thrust correction if the inertia is about 1.25-1.3 times | The sweep of chapter 5(b): at thrust scale 1.0, $J$ = 11.4-12.3e-6 fits | Medium | A restatement of the same fact, not independent evidence. Thrust at saturation (duty 1) lies outside the hover check |

### Candidate Causes of the Small Bifilar Value (all unconfirmed)

| Candidate | Direction of effect | Remark |
|---|---|---|
| Reading of the string spacing $D$ | $J \propto D^2$, so a small reading gives a small value | A few percent of misreading hardly reaches 20 % (10 % gives 20 %) |
| Different string spacing at the vehicle and the upper end | The formula assumes parallel strings (spacing of the upper fixing points = spacing of the attachment points on the vehicle = $D$). When they differ, $J$ is proportional to (upper spacing) × (vehicle-side spacing). If the vehicle-side spacing is wider, the value computed with $D^2$ comes out too small | Explaining 20 % on the roll axis needs about 98 mm on the vehicle side (78 mm × 1.25) |
| Restoring torque added by string bending / torsion stiffness | Shortens the period $T$, giving a small value | The vehicle is light, so the restoring-torque coefficient of the two strings is only $m g D^2/(4h) \approx 1.9\times10^{-3}$ N·m/rad (computed from the roll-axis record). Whether the string stiffness is negligible against this has not been checked |
| Re-hanging for each axis | A shifted hanging point changes the period | The three axes use different hanging ($h$ and $D$ differ) |

## 8. Unconfirmed Items and Next Work

| Item | Content |
|---|---|
| Direct measurement of the main-board mass | Weigh directly the value fixed at 8.9 g as a remainder, including wiring, solder, and connectors |
| Check of the bifilar fixture | Measure a body whose inertia is known by calculation (a cylindrical rod, a metal plate, etc.) with the same fixture and procedure to find errors in the formula or the procedure |
| Independent estimation of pitch and yaw | An excitation test (applying a prescribed input to the command during hover) to determine pitch and yaw inertia from flight logs |
| Thrust scale at the maximum flip command | The thrust scale at saturation is outside the hover check |
| Difference in hover thrust scale | Why 11/22 (1.08-1.16 in the hold logs) and 10/02 (1.009-1.010) differ |
| Decision on changing the reference values | Awaits the developer. Linked places if changed: `control/models/stampfly_physical.yaml`, the SILS vehicle model (the inertia in `stampfly.xml`, including a switch to `fullinertia` and a centre-of-mass `pos`), the URDF files (2), the control design tools, `sf params check` |
| Replacing the propeller shape data | Chapter 6. URDF mass, STL, Web display (6 places) |
| Physical check of the STL orientation | The X = left, Y = up, Z = front correspondence is unconfirmed on the real vehicle |
| Real wall thickness of the frame | Cause of the STL volume being 1.4-1.6 times the real one |
| Scale type and resolution | The type of scale used for the propeller mass |

## 9. Locations of Working Files

| Content | Location |
|---|---|
| Estimate from the 3D model (rev4) | Analysis working files (not under Git): `rev4_log.txt`, `rev4_compare.csv`, `rev4_parts.csv`, `rev4_frame.csv`, `rev4_mc_summary.csv` |
| Flip recomputation | Analysis working files (not under Git): `flipJ.log`, `sweep.log` |
| Estimation from flight logs | Analysis working files (not under Git): the preset-rule record `plan.md` and the aggregation |
| Bifilar measurement record | Source of Ixx in `control/models/stampfly_physical.yaml` |
| Public repositories | `kouhei1970/prop_sim` (measured propeller photos and BEMT = blade element momentum theory), `kouhei1970/stampfly-motor-autotest` (automatic bench measurement) |
