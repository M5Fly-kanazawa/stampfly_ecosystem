# 推進系モデルの実験結果のまとめ（2026-11-02〜03）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

**状態**: 基礎資料（2026-11-03 作成）。推進系のモデルを刷新し、ファームの推力の換算と制御設計、SILS のプラントを作り直す作業（別ブランチで行う予定）の出発点として、2026-11-02〜03 の実験で分かったことを整理する。物理パラメータの基準ファイル（`control/models/stampfly_physical.yaml`）と SILS 方針のバックログは、この文書では変えていない。同じ内容の図入りの HTML 版: https://claude.ai/artifact/MhqcHBYiR48yuygWQDnvCJ （閲覧には共有が要る）。2026-11-03 更新: 未決だった 3 点を決めた（10 章「決めたこと」）。9 章の照合の幅の扱いを直した（合否の幅はデータを見てから決める）。重さの記述を直した（開発者が量ってあり 36〜38 g）。

## 1. 概要

### このレポートについて

StampFly の推進系（モータ・駆動回路・プロペラ）について、2026-11-02 のベンチ実験、2026-10-15 のベンチ実験のやり直し、飛行ログとの照合、SILS での試験（2026-11-03）の結果をまとめる。各項目には次の区別を付ける。

| 区別 | 意味 |
|---|---|
| 事実 | 測定値と、その計算結果。どのスクリプトの出力かを 11 章に示す。計算は Claude（AI の作業者）が行った。12 章のレビューで、主な数値は別の担当（AI）が生データから計算し直した。人による独立の確認はしていない |
| 実験者の確認・判断 | 実験者（開発者）が確かめたこと、記憶していること、決めたこと |
| 推論 | 事実から考えたこと。強さ（強・中・弱）と、ほかに考えられる説明を添える。強さは目安で、較正したものではない |
| 意見 | 次の作業への提案（10 章） |

### 結論の要約

| 項目 | 内容 | 区別 |
|---|---|---|
| 駆動回路の $R_\text{out}$ | 電池電圧への依存は小さい。4.27 → 3.63 V で 0〜−1.3 %（全体の回帰で 1 V あたり +1.6 ± 0.4 %） | 事実 |
| 一定の電圧降下 $V_b$ | どの Duty でも約 0.11 V の降下を加えた式が合う。周ごとに 0.06〜0.18 V と動く | 推論（中） |
| 回転数の再現性 | 同じモータは、同じ Duty・同じ電圧で、日をまたいで −1.2〜+0.8 %、冷えた／温まったで ±0.6 % 以内 | 事実 |
| 4 モータ同時 | 4 つのモータは同時にオンになり、オンの間の電圧が平均より余計に下がる。1 モータで測った電圧の範囲の中では、オンの間の電圧で比べると回転数の差は 0 ± 0.3 % | 事実 |
| 推力 | $F/\omega^2$ は一定でない。$F = k\,\omega^{n}$（$n$ = 2.13〜2.15、当てはめ方による）が 2 日とも合う。11/02 夜の推力は 10/15 の 0.958〜0.961 倍で、回転数によらない | 事実 |
| 採る値 | 10/15 の推力を基準とし、全機の推力はその 4 倍とする。11/02 の 4 モータ同時の推力は使わない | 実験者の判断 |
| 飛行との照合 | 4 × 10/15 の推力 ÷ 重さ（36 g と 37 g で計算。量った重さは 36〜38 g）は 0.981〜1.034。10/15 と 11/02 の推力はこの照合では区別できない | 事実 |
| ホバー Duty の電圧依存 | 飛行 −0.21〜−0.26 /V（1 回の飛行の中／全区間）、ベンチ −0.24〜−0.29 /V、今のファームの換算（Duty × 電圧が一定）−0.168 /V | 事実 |
| 今のファームの換算 | 電圧依存が緩すぎ、その分を高度制御と推力の学習が補っている（記録された指令推力が電圧の低下とともに増える） | 事実・推論（中〜強） |
| SILS | 換算をプラントの正確な逆にすると、SILS の中では電圧による高度のずれと離陸の行き過ぎが消える（作りの上で当然の部分を含む）。ループの効きは 1.3〜2.0 倍になる | 事実 |

### 対象読者

推進系のモデル、SILS のプラント、ファームの推力の換算と制御設計を作り直す人。

## 2. 用語と量の定義

### 電圧（測る位置で値が違う）

| 名前 | 測る位置・方法 | 使う場面 |
|---|---|---|
| 記録電圧（CLI 電圧） | 電池経路の電流センサ INA3221 のバス電圧。機体のコマンド（CLI）`power` と飛行ログの電池電圧はこの値 | 飛行ログ、ファームの換算 |
| 平均の $V_{DD}$ | M1（右前のモータ）の駆動回路の電源をオシロで測った平均 | ベンチの回転数の表 B |
| オンの間の $V_{DD}$ | M1 の PWM がオンの間だけの $V_{DD}$ の平均 | 4 章「4 モータ同時」 |

| 比べた量（段ごとの範囲または中央値） | 1 モータ | 4 モータ同時 |
|---|---|---|
| 記録電圧 − 平均の $V_{DD}$（範囲） | 4〜32 mV | 9〜80 mV |
| 平均 − オンの間の $V_{DD}$（中央値） | 31 mV | 83 mV |
| オフ − オンの $V_{DD}$（範囲） | 101〜127 mV | 203〜303 mV |
| 記録電圧 − オンの間の $V_{DD}$（範囲） | 33〜67 mV | 73〜170 mV |

午後以降の運転（11/02 の 4 つの負荷の比較と R15）は、$V_{DD}$ の読みの基準が午前と違う（9 章）。

### 記号

| 記号 | 意味 | 単位 |
|---|---|---|
| $D$ | Duty（ファームが PWM に渡す値。PWM は 150 kHz・8 bit） | — |
| $\omega$ | モータの回転数。整流子の切り替わりによる電圧の脈動（1 回転 10 回）から測る | rad/s |
| $K_e$, $K_t$ | 逆起電力定数とトルク定数（SI 単位では同じ値） | V·s/rad, N·m/A |
| $I_m$ | モータの平均電流（4 モータ同時では電池電流の 4 分の 1 とした） | A |
| $R_\text{out}(D)$ | 駆動回路の等価出力抵抗（Duty とコンデンサで決まる。開発者らの論文のモデル） | Ω |
| $V_b$ | 一定の電圧降下（4 章） | V |
| $I_0(\omega)$ | 無負荷電流 | A |
| $F$, $C_T$ | 1 モータの推力と、推力係数 $C_T = F/\omega^2$ | N, N/(rad/s)² |
| $k$, $n$, $F_\text{ref}$, $\omega_\text{ref}$ | 推力のべき乗の式 $F = k\,\omega^n = F_\text{ref}(\omega/\omega_\text{ref})^n$ の定数 | — |
| $C_Q$ | トルク係数 | N·m/(rad/s)² |
| $J$ | 回転部（モータ＋プロペラ）の慣性モーメント | kg·m² |
| $\eta$ | SILS のプラントの推力効率（`thrust_efficiency`、0.976） | — |

### 運転・表・試験の名前

| 名前 | 内容 |
|---|---|
| 段 | 1 つの Duty で回す 1 区切り（11/02 の長時間放電は 5 秒、R15 は 16 秒） |
| 周 | Duty の列（0.50, 0.60, 0.62 … 0.80）を 1 回たどること |
| 電池 1〜3 | 11/02 に使った 3 本の電池 |
| A1、C1〜C3 | 電池 2 の冷えた状態の周（A1 は満充電の直後、C1〜C3 はモータを 10 分冷ました後） |
| B1〜B3 | 電池 2 の 4 周続けて回した運転（2 周目以降はモータが温まった状態） |
| A4M・B4M・C4M | 電池 3 の 4 モータ同時の運転 |
| 4 つの負荷の比較 | 11/02 午後（16 時台）。プロペラなし・4 枚羽根・3 枚羽根・ピッチを立てた 4 枚羽根 |
| R15 | 11/02 夜の 10/15 の試験のやり直し |
| D-低 | Duty を段差状に変えて回転数の立ち上がりを測った運転 |
| 表 A | 4 モータ同時の M1 の回転数を、Duty と記録電圧で表した当てはめ（電池 3、42 段） |
| 表 B | 1 モータの回転数を、Duty と平均の $V_{DD}$ で表した当てはめ（電池 2、192 段）。飛行ログに当てるときは、記録電圧から 4 モータ時の「記録電圧 − 平均の $V_{DD}$」を引く |
| F1・F1g | SILS で試したファームの変更（8 章） |
| SILS | ファームのソースをそのまま PC 上で動かすシミュレーション |
| 再確認試験 | 変更で既存の動作が壊れていないかを自動で確かめる試験（`sf sils regression`、45 シナリオ） |
| モデル一致の判定 | SILS と実機の宙返りのログを比べる判定（`sf sils sysid-gate`） |

### そのほかの量

| 量 | 内容 |
|---|---|
| 推力 | 1 モータの推力 [N または g]。全機の推力は 4 モータの和 |
| 重さ | 機体の重さ。開発者が量ってあり 36〜38 g（2026-11-03 開発者の確認。初版は「量っていない見積もり 36〜37 g」と書いていた）。このレポートの照合は 37 g と 36 g の両方で計算した |

## 3. 実験とデータ

| 日付 | 実験 | 内容 |
|---|---|---|
| 2026-10-15 | 1 モータのベンチ（基準） | M1 とプロペラ。Duty 0.1〜0.9（V_DD 4.17〜4.38 V）。回転数・電流・推力・スピンダウン。録音あり |
| 2026-11-02 | 長時間放電（電池 1〜3） | 電池 2: 1 モータで Duty 0.50〜0.80（0.02 刻み）を、負荷時の $V_{DD}$ 4.27 → 3.63 V まで。電池 3: 4 モータ同時の 3 回 |
| 2026-11-02 | 4 つの負荷の比較 | 段の中で $K_e$ を測る（PWM を 0.08 秒止める） |
| 2026-11-02 夜 | R15 | 同じプロペラ・同じ高さで Duty 0.1〜0.9、スピンダウン |
| 2026-06-14〜09-08 | 飛行ログ 14 本 | 同じ形状の別の個体（ベンチの機体とは別）。13 本は 2026-08-22 のファームの換算の変更より前（9 章） |
| 2026-11-03 | SILS | 推力 → Duty の換算を替えた 2 通り（F1・F1g） |

ベンチの機体は、逆さまにして缶の上に置く。推力は機体を秤へ押し付ける向きで、プロペラの吹き出しは上へ抜け、下が吸い込み側になる。

公開データ: ベンチのリポジトリ `stampfly-motor-autotest` の `data/2026-10-15/` と `data/2026-11-02_steps.csv`（11/02 の段ごとの表）。写真とオシロの波形はリポジトリの外で保管している。手順と当日の記録は同リポジトリの `docs/procedure_long_discharge.md` 10 章。

## 4. 駆動回路（Duty・電圧 → 回転数）

### R_out の電池電圧への依存は小さい（事実）

冷えた状態の周どうし（A1: 4.27〜4.13 V、C3: 3.72〜3.63 V）で、$R_\text{out} = (V_{DD} - K_e\omega)/I_m$（$K_e = 5.7628\times10^{-4}$、11/02 のプロペラなしのスピンダウン）の C3 ÷ A1 − 1 は、Duty 0.50〜0.80 で −1.33〜+0.03 %（平均 −0.8 %）。冷えた周全体の回帰では、1 V あたり +1.6 ± 0.4 %（電圧が低いほどわずかに小さい）。

- $K_e$ を ±2 % 変えると、変化は −0.6〜−1.0 % と同じくらい動く。
- 温まった周では、同じ Duty で $R_\text{out}$ が 0.4〜0.75 % 大きくなり、電圧の効果と同じ程度の大きさになる。
- 開発者らの論文の前提「$R_\text{out}$ は Duty と回路だけで決まり、電池電圧によらない」と、1 % 程度の範囲で合う。

### 一定の電圧降下 V_b（推論、強さ：中）

電流 0 で測った $K_e$ の式のままだと、同じ Duty でプロペラなしの $R_\text{out}$ が付きより 5.7〜18.7 % 大きく出る（D 0.2〜0.5、事実）。4 つの負荷で比べると、どの Duty でも同じ一定の電圧降下 $V_b$ を加えた次の式がよく合う。

$$\bar I_m = \frac{V_{DD} - K_e\,\omega - V_b}{R_\text{out}(D)}$$

| 当てはめ（事実） | 結果 |
|---|---|
| Duty ごとの切片 | 108〜123 mV |
| 共通の $V_b$ | 110 mV（残差 rms 39.7 mV） |
| 代わりに $\omega \cdot I$ に比例する項を入れた式（パラメータの数は同じ） | 残差 rms 51.7 mV |

- $V_b$ はブラシの接触による電圧降下に相当する大きさ。駆動回路の飽和電圧や配線の降下など、ほかの由来は除けていない。
- 判別は残差 39.7 mV と 51.7 mV の差で、強くはない。
- 同じ負荷の 1 周目と 2 周目で 60〜180 mV と動き、時間（温まり方）への依存が残る。
- 4 つの負荷の比較は午後の運転で、$V_{DD}$ の読みの基準が午前と 30 mV 程度違う（9 章）。$V_b$ は最大 0.03 V 程度動くかもしれない（未確認）。

### R_out(D) の実測（Duty 0.02 刻み、事実）

冷えた周（A1・C1・C2・C3）の平均。$V_b$ を入れない式と入れた式の 2 通りと、SILS のプラントの表（10/15 の 0.1 刻みの実測で論文の計算値を補正したもの）を並べる。

| Duty | $(V_{DD}-K_e\omega)/I_m$ [Ω] | $V_b$ = 0.11 V を引いた値 [Ω] | SILS の表 [Ω] | $I_m$ [A] |
|---|---|---|---|---|
| 0.50 | 3.453 | 3.277 | 3.471 | 0.626 |
| 0.60 | 2.520 | 2.376 | 2.563 | 0.766 |
| 0.62 | 2.412 | 2.272 | 2.434 | 0.786 |
| 0.64 | 2.312 | 2.174 | 2.322 | 0.804 |
| 0.66 | 2.229 | 2.095 | 2.226 | 0.820 |
| 0.68 | 2.142 | 2.011 | 2.144 | 0.840 |
| 0.70 | 2.059 | 1.930 | 2.072 | 0.858 |
| 0.72 | 1.977 | 1.851 | 1.970 | 0.878 |
| 0.74 | 1.890 | 1.767 | 1.872 | 0.901 |
| 0.76 | 1.793 | 1.674 | 1.772 | 0.927 |
| 0.78 | 1.691 | 1.576 | 1.665 | 0.958 |
| 0.80 | 1.546 | 1.436 | 1.547 | 1.006 |

- 使える範囲: Duty 0.50〜0.80、$V_{DD}$ 3.63〜4.27 V、1 モータ。
- $V_b$ を入れない値は、SILS の表と ±1.7 % で合う。
- SILS の表は、Duty 1 あたりの傾きが 0.68〜0.70 で −3.6 Ω、0.70〜0.72 で −5.1 Ω と、D 0.70（10/15 の測定点）で折れている。0.02 刻みの実測は、同じ区間で −4.2、−4.1 Ω となめらか。
- 4 モータ同時の M1 の $R_\text{out}$（電池電流を 4 等分して計算）は、同じ Duty の 1 モータより 3〜7 % 大きい。電流の配分か配線の降下が 1 モータと同じでないことを示す（推論）。
- どちらの式の表を基準にするかは、9 章の $V_{DD}$ の基準の確認の後に決める。

### 回転数の再現性（事実）

| 比べたもの | 回転数の差 |
|---|---|
| 10/15 と 11/02（同じ M1、同じ Duty・平均 $V_{DD}$、D 0.5〜0.8） | −1.2〜+0.8 % |
| 冷えた周と温まった周（11/02、電池 2） | ±0.6 % 以内（表 B の残差の平均で +0.24 % と −0.19 %、B1 の 2〜4 周目は −0.25〜−0.59 %） |

- 10/15 の $\omega$ を録音で確かめた（録音は回転数の測り方と独立。録ったのは同じ実験者）。1 回転の線と比べて、10 段のうち 7 段で ±0.6 % 以内。D 0.5 の 2 段は一番強い線で +1.06 % と −0.85 %（ほかの線は ±1.6 % まで散る）。D 0.7 は使える線がない。
- $K_e$ は運転で温まると数分で 2〜4 % 下がる（スピンダウンと段の中の測定）。それでも、同じ Duty・電圧での回転数は ±0.6 % しか変わらない。巻線の抵抗の上昇など、逆向きの効果と打ち消し合っていると考える（推論、強さ：弱）。
- $K_e$ は 10/15 のスピンダウンで $5.90\times10^{-4}$、11/02 のプロペラなしのスピンダウンで $5.76\times10^{-4}$、11/02 夜の段の中の測定で $5.54\times10^{-4}$ と、測り方で最大 6 % 違う。同じ Duty・電圧での回転数の差（±1.2 %）には現れないので、測り方の違いによると考える（推論、強さ：中、未確認）。

### ホバー付近の感度（事実、表 B、D 0.65 付近）

| 変化 | 回転数 | 推力 |
|---|---|---|
| $V_{DD}$ +0.1 V | +1.7〜1.8 % | +3.6〜3.9 % |
| Duty +0.01 | +0.8 % | +1.7〜1.9 % |

### 立ち上がり

- 事実: Duty を段差状に変えたとき、回転数の変化の 63 % に達するまでが、上げで 35〜40 ms、下げで 50〜55 ms（D-低）。
- 推論: 上げは論文の $R_\text{out}$ モデルのホバー付近の時定数 38.8 ms とよく合う。下げが遅いのは式が非線形であることと合う。非線形の式の数値解との軌道の照合はしていない。

### 4 モータ同時（オンの間の電圧）

事実: 4 つのモータは同じタイマで同時にオンになる。オンの間は 4 つ分のパルス電流（4.1〜5.0 A）が電池と配線を流れ、M1 の $V_{DD}$ が平均より余計に下がる（2 章の表）。4 モータの M1 の回転数を、1 モータの表（同じ Duty・同じ $V_{DD}$）と比べた。

| 表を引く電圧 | 1 モータの電圧範囲の中の段 | 差の平均 | rms |
|---|---|---|---|
| 平均の $V_{DD}$ | 27 / 42 | −0.97 % | 1.24 % |
| オンの間の $V_{DD}$ | 21 / 42 | +0.09 % | 0.68 % |

- 1 モータの表そのものの当てはめの rms は 0.71 %。範囲の中の段の数が 2 行で違うのは、オンの間の電圧のほうが低く、範囲から外れる段が増えるため。
- レビューでの再計算（冷えた周だけ・4 項の式）では、範囲の中で −0.21 ± 0.15 %（21 段）、範囲の外で −0.87 %（B4M の 2 周目と C4M）。
- 推論（強さ：中〜強）: 1 モータで測った電圧の範囲の中では、同じモータは、同じ Duty とオンの間の同じ電圧なら、同時に回すモータの数によらず同じ回転数で回る。範囲の外（オンの間の $V_{DD}$ 3.61 V 未満）は確かめていない。温まった周で 0.5〜0.9 % 下がることもあるので、範囲の外の −0.9 % が温まりや負荷の違いである可能性は除けない。
- 記録電圧とオンの間の $V_{DD}$ の差（4 モータ同時）は、段ごとに 73〜170 mV。$\Delta V = 781 - 118\,V_\text{CLI} - 324\,D$ [mV]（残差 rms 12 mV、電池 3 の 1 本だけの値）。パルス電流あたりでは 15〜39 mΩ で、電池が減るほど大きい。
- 同じ差を、物理の形 $\Delta V = R_\text{path}\,I + R_\text{src}\,I\,(1-D)/D$（$I$ は電池の総電流の平均、$D$ は Duty）で当てはめ直した（2026-11-03、切片なし。計算は筆者、独立の確認なし）。第 1 項は電流センサとモータの駆動回路の間の配線の降下、第 2 項はオンの間だけ流れるパルス電流が電池と配線の抵抗に作る、平均より余計な落ち込みを表す。

| データ | $R_\text{path}$ | $R_\text{src}$ | 残差 rms |
|---|---|---|---|
| 1 モータ（電池 2、差 33〜67 mV） | 26 mΩ | 72 mΩ | 4 mV |
| 4 モータ（電池 3）、$R_\text{src}$ 一定 | 10 mΩ | 61 mΩ | 25 mV |
| 4 モータ（電池 3）、$R_\text{src}$ が電池の減りとともに増える形 | 2 mΩ | $37 + 95\,(4.2 - V_\text{CLI})$ mΩ | 13 mV |

- 推論（強さ：中）: 別の電池・別のモータの数で $R_\text{src}$ が 60〜70 mΩ 程度とそろい、1 つの形で 1 モータと 4 モータを表せる。4 モータのデータは電池 1 本だけで、電池ごとの差は分からない。切片を置いていないので、$V_{DD}$ の読みの基準のずれ（9 章）は $R_\text{path}$ に混ざる。
- 飛行中の機体はオンの間の $V_{DD}$ を直接は測れない。INA3221 は 1 回 1.1 ms の変換を 64 回平均しており（3 チャネル分で約 0.4 秒）、PWM の 1 周期 6.7 µs より桁違いに長い。
- 電源電圧のスペクトルには、1 モータの運転では出ない線があった（電池 3 の 42 段のうち 15 段。M1 の 0.92〜0.97 倍、1 段だけ 1.04 倍）。M1 より 3〜8 % 遅いモータがある（何台かは不明）。M2〜M4 の回転数は測っていない。

## 5. 推力とトルク（回転数 → 推力・トルク）

### 秤の読み（事実）

秤はゼロより下を、数字の左の横棒（マイナス記号）で示す。11/02 の当初の自動読み取りはこの記号を見ておらず、止まっているときの負の値（電池 2 で −0.54〜−3.20 g、電池 3 で −0.3〜−2.2 g）を正と読んで、推力をゼロの 2 倍だけ小さく読んでいた。記号を読むように直した（画角ごとに記号の位置を決める）。11/02 の推力の値は、すべて記号を読んだ値である。

| 確認 | 結果 |
|---|---|
| 電池 2 | 記号の位置の暗さで判定。止まっている写真の 82〜100 % に記号、回っている写真では 0 % |
| 電池 3 | 記号の位置の暗さが 0〜10 と 60〜140 にきれいに分かれた。別の担当が写真 11 枚を目で読み、11 枚とも判定と一致 |
| 夜（R15） | 夜の画角の較正を、目で読んだ 10 枚で確かめた |
| 10/15 | 写真 144 枚に記号は出ていない。手で書き写した値は 3 枚で写真と一致 |

### 推力の曲線（事実）

| モデル（パラメータの数） | 残差 rms（10/15） | 残差 rms（11/02 夜） |
|---|---|---|
| $F = C_T\,\omega^2$（1） | 0.23 g | 0.21 g |
| $F = C_T\,\omega^2 + b$（2） | 0.11 g（$b$ = −0.41 g） | 0.10 g（$b$ = −0.37 g） |
| $F = a\,\omega^2 + b\,\omega$（2） | 0.070 g | 0.066 g |
| $F = k\,\omega^{n}$（2）、対数の残差で当てはめ | 0.071 g（$n$ = 2.148） | 0.072 g（$n$ = 2.140） |
| $F = k\,\omega^{n}$（2）、推力の残差で当てはめ | 0.051 g（$n$ = 2.127 ± 0.010） | 0.064 g（$n$ = 2.122 ± 0.009） |

| $\omega$ [rad/s] | 約 1200 | 約 2400 | 約 3600 | 約 4700 |
|---|---|---|---|---|
| 段ごとの $F/\omega^2$（10/15） | 6.30e-9 | 7.18e-9 | 7.48e-9 | 7.78e-9 |
| 段ごとの $F/\omega^2$（11/02 夜） | 6.09〜6.24e-9 | 6.74〜6.79e-9 | 7.15〜7.19e-9 | 7.40〜7.43e-9 |

- $F/\omega^2$ は 2 日とも $\omega$ とともに大きくなる（1200 → 4700 rad/s で 10/15 は 23 %、11/02 は 19〜22 %）。
- 11/02 夜 ÷ 10/15 は、指数を共通（2.142）にした当てはめで 0.961、レビューでの補間による比較で 0.958（0.940〜0.973、回転数への傾き +0.0033 ± 0.0023 /(1000 rad/s) で有意でない）。D 0.9 でも 0.955〜0.958。
- 10/15 の曲線は、ホバー付近を基準に次のように書ける。使える範囲は $\omega$ 1200〜4700 rad/s。

$$F = F_\text{ref}\left(\frac{\omega}{\omega_\text{ref}}\right)^{n},\qquad \omega_\text{ref} = 3500\ \text{rad/s},\quad F_\text{ref} = 0.0916\ \text{N}\ (9.34\ \text{g}),\quad n = 2.13\ \text{（推力の残差）〜}\ 2.15\ \text{（対数の残差）}$$

- $F_\text{ref}$ は当てはめ方によらず 91.6 mN（差 0.05 %）。$n$ の差は 1200 rad/s で推力の 2 %。
- ホバー付近の傾き $dF/d\omega = nF/\omega$ は、$C_T$ 一定の $2F/\omega$ より 6〜7 % 大きい。原点を通る $C_T = 7.60\times10^{-9}$ を使うと、ホバーで推力は 1.5 % 大きく、傾きは 5.6 % 小さく、1200 rad/s では推力が約 20 % ずれる。
- $F = a\omega^2 + b\omega$ もべき乗と同じくらい合う（同じパラメータの数）。どちらを使うかは選び方の問題で、べき乗は逆算が簡単（意見）。
- 推論（強さ：弱〜中）: $F/\omega^2$ が回転数とともに上がるのは、小さいプロペラの低いレイノルズ数の効果が候補（レイノルズ数は計算していない）。ゼロのずれだけの式（$C_T\omega^2 + b$）は当てはまりが悪く、ゼロの扱いの違う 2 日で同じ形が出ることも説明しにくい。

### 10/15 と 11/02 の差

| 候補 | 結果 | 区別 |
|---|---|---|
| 秤の読み方 | 10/15 の写真に記号なし、書き写しは一致 | 事実 |
| ケーブルの留め方 | 机にも固定すると、ゼロが −3.4 g ずれ +0.1 g/分で戻るが、倍率（10 円玉 6 枚で 26.84〜26.88 g）と回転中の推力（段ごとの比 0.99〜1.01、D 0.3 だけ 1.05）は変わらない | 事実 |
| 空気の密度 | 気象庁の金沢の観測値で、室温が同じなら 11/02 ÷ 10/15 = 0.993 | 事実 |
| 室温 | 11/02 の室温は 10/15 より高くなかった（両日とも未記録）。密度の比は 0.993 より小さくならない | 実験者の記憶 |
| プロペラ・取り付けの高さ | 同じ | 実験者の確認 |
| $\omega$ の読み | 10/15 の $\omega$ は録音と合う（4 章）。$\omega$ を使わない比較（同じ Duty での電池電流あたりの推力）でも 11/02 は 0.927〜0.965 倍 | 事実 |
| プロペラの下（吸い込み側）のプローブ・配線 | 11/02 は 10/15 より多かったかもしれない | 実験者の記憶 |
| 同上の効き方 | 同じ $\omega$ で、11/02 は推力が 2〜6 % 小さく、電流（トルク）が 1〜4 % 大きい（D 0.4 以上）。吸い込みの流れの乱れと向きが合う | 推論（中、未確認） |

- 比が 0.961（同じ $\omega$）、0.927〜0.965（同じ Duty での電流あたり）、2〜6 %（同じ $\omega$ に直した Duty ごと）と違うのは、比べている量が違うため。電流あたりの比には電流の違いも入る。
- $C_Q$ がほぼ同じ（3.38 と 3.33e-11）なのに電流が 1〜4 % 大きいのは、$C_Q$ をそれぞれの日の $K_e$（$K_t$）で計算しているため（11/02 夜の $K_e$ は 6 % 小さい）。

### 採る値（実験者の判断、2026-11-02）

- 原因は特定できていない。そのうえで、同じプロペラ・同じ高さなら、このベンチの吸い込み側の障害物は推力を小さく見せる向きにしか効かないので、大きいほうの 10/15 の推力を基準とする。
- 全機の推力は、10/15 の 1 モータの推力の 4 倍とする。
- 11/02 の 4 モータ同時の推力は、ほかのプロペラと缶の位置関係が 1 モータのときと違うので、全機の推力の測定値としては使わない。実機のホバー Duty での秤の読みは約 34 g で、重さより 6〜8 % 小さかった。この不足の原因は特定していない（4 つの吹き出しの干渉、缶、M1 以外のモータが遅いことが候補）。
- 11/02 夜の 1 モータの値は、「プロペラの下にプローブ・配線がある条件の値」として残す。

### トルクと慣性（事実）

| 量 | 10/15 | 11/02 夜 |
|---|---|---|
| $C_Q$ | $3.38\times10^{-11}$ | $3.33\times10^{-11}$ |
| 段ごとの $C_Q$ | — | 1200 rad/s で $5.0\times10^{-11}$ → 4700 rad/s で $3.3\times10^{-11}$ |
| $J$（プロペラ付き） | $1.54\times10^{-8}$ kg·m² | $1.47$〜$1.53\times10^{-8}$（形を決めない当てはめ） |
| 無負荷電流 $I_0$ | $0.044 + 2.9\times10^{-9}\,\omega^2$ A | — |

- 推論（強さ：中）: 段ごとの $C_Q$ が低い回転数で大きいのは、無負荷電流（摩擦）の引き方に強く左右される。無負荷電流を $a + b\,\omega^2$ とする式はスピンダウンに合わず、一定の摩擦トルクを入れる必要がある。

## 6. 飛行ログとの照合

### ホバー区間の選び方（事実）

飛行ログ 14 本（UDP テレメトリ、Duty は 50 Hz の指令値、電池電圧は約 1 秒ごと）から、次の条件を満たす区間を選び、5〜10 秒（短い区間は 1〜5 秒）に区切った（134 区間・計 574 秒）。飛行状態が飛行中、高度 0.3 m 以上、離陸から 2 秒以降、上下の加速度 0.5 m/s² 未満（速度を 0.5 秒で平滑化して微分）、上下速度 0.2 m/s 未満、角速度 5 rad/s を超えた前後 1.5 秒を除く。区間の時間で重み付けする。6/27 02:01 の 1 本が全体の約 30 % を占める。

### ホバーの釣り合い（事実）

各区間で、記録された 4 つの Duty と電池電圧から回転数を出し、10/15 の式の推力を 4 台分足して、重さと比べた。

| 方法 | 37 g | 36 g |
|---|---|---|
| 表 A と 10/15 の式 | 0.986 | 1.013 |
| 表 B と 10/15 の式 | 1.006 | 1.034 |
| 11/02 の 1 モータの推力を Duty と $V_{DD}$ で直接回帰 | 0.981 | 1.008 |

（推力 ÷ 重さ、時間で重み付けした平均。運転ごとの中央値をさらに中央値にすると、表 A で 1.009〜1.037、表 B で 1.020〜1.049）

- 前提: 4 つのモータが M1 と同じ（M1 の表を 4 つの Duty に当てる）、定常、ベンチと飛行の機体・プロペラが同じ形状。地面効果は小さい（高度 0.3 m 以上）とした。
- 外挿: 低い電圧の区間（記録電圧 3.7 V 未満が 32 区間、最低 3.49 V）では、表 B は測った範囲（$V_{DD}$ 3.63 V 以上）の外になる。Duty を逆に解くときは 0.3〜1.0 で探す（表は 0.5〜0.8 で作った）。
- 3 つ目の方法は、表 B を通した 11/02 の推力（1.006 × 0.961 = 0.966）と 1.5 % で一致した。ただし同じベンチのデータ・同じ電圧の補正・同じ重さを使うので、独立の確認ではない。
- 運転ごとの中央値は 0.88〜1.07。個体差（重さ・モータ・プロペラ）が候補。2026-06-18 の記録では、同じ形の新しいモータは使い込んだ機体より Duty あたり約 1.34 倍の推力を出していた（ホバー Duty 0.71 → 0.53）。
- 推論（強さ：中）: 全機 = 1 モータの 4 倍は飛行と矛盾しない。飛行中に干渉で 5 % 以上推力を失っている様子はない。ただし、重さの幅（2.7 %）、回転数の表が 11/02 の条件の値であること、個体差（±10 %）があるので、10/15 と 11/02（3.9 % の差）はこの照合では区別できない。「±3 %」は結果を見た後に付けた幅で、36 g の表 B（1.034）はその外にある。

### ホバー Duty と電池電圧（事実）

| 記録電圧 | 飛行のホバー Duty（区間数・運転数） | ベンチ・表 A | ベンチ・表 B | 11/02 の推力を直接 |
|---|---|---|---|---|
| 3.5 V | 0.731（8・1） | 0.746〜0.763 | 0.716〜0.733 | 0.739〜0.756 |
| 3.6 V | 0.705（11・3） | 0.713〜0.729 | 0.690〜0.706 | 0.709〜0.726 |
| 3.7 V | 0.690（23・8） | 0.686〜0.702 | 0.668〜0.684 | 0.685〜0.701 |
| 3.8 V | 0.662（19・6） | 0.651〜0.666 | 0.638〜0.654 | 0.652〜0.668 |
| 3.9 V | 0.636（25・11） | 0.626〜0.640 | 0.616〜0.631 | 0.629〜0.644 |
| 4.0 V | 0.616（37・9） | 0.602〜0.615 | 0.595〜0.609 | 0.607〜0.622 |
| 4.1 V | 0.599（11・8） | 0.588〜0.601 | 0.582〜0.597 | 0.595〜0.609 |

（ベンチは重さ 36〜37 g を支える Duty。3.5 V の行は 1 本だけで、表 B は外挿）

| 電圧に対するホバー Duty の傾き | 1 V あたり |
|---|---|
| 飛行・全区間（区間の時間の平方根で重み付け） | −0.242 |
| 飛行・全区間（重みなし。運転単位のブートストラップで 95 % 範囲 −0.30〜−0.23） | −0.256 |
| 飛行・1 回の飛行の中（14 本の各平均を引いて合わせた値、時間で重み付け） | −0.218 |
| 同（重みなし。95 % 範囲 −0.236〜−0.178） | −0.210 |
| ベンチ・表 B | −0.24 |
| ベンチ・11/02 の推力を直接 | −0.26 |
| ベンチ・表 A | −0.29 |
| 今のファームの換算（Duty × 電圧が一定、$-D/V$） | −0.168 |

- 全区間の傾きには個体差が混ざる（低い電圧の区間は 6 月の運転が多く、高い電圧は 7 月が多い）。1 本ずつ抜いても −0.237〜−0.269 で、1 本に引きずられてはいない。
- 1 回の飛行の中の傾き（電圧の幅 0.3 V 以上の 4 本: −0.240、−0.204、−0.163、−0.173 /V）は、4 本ともベンチの表 B（−0.264、−0.238、−0.234、−0.229 /V）より緩い。理由は未特定。ベンチでは温まった周のほうが回転数がわずかに低く、温まりでは説明できない。飛行の中では電圧と時間が一緒に変わる。
- 傾きの誤差（ブートストラップ以外の ± 表記）は 5 秒区間を互いに独立とみなした値で、実際より小さい。
- 記録された指令推力を説明変数に加えると、電圧への傾きは −0.17 /V 前後になる（レビューでの計算）。ただし記録された Duty は、ファームが指令推力と電圧から換算した値そのものなので、これは換算の傾き（−0.168 /V）を取り出しているだけで、物理の傾きの反証にはならない。
- 4 モータ同時を秤で測った 37 g の Duty（A4M 0.726、B4M 0.764）は、同じ電圧の飛行より 0.07〜0.09 大きい。

### ファームの換算を替えたら指令推力は平らになるか（事実）

定常ホバーの Duty と電圧は物理で決まるので、ある換算がそれを出すのに要る指令推力は、その換算の順モデルを記録された Duty で評価した値になる。記録された指令推力は、その日のファームの値である（9 章。13 本は 2026-08-22 の換算の変更より前）。

| 換算 | 1 回の飛行の中の傾き（14 本） | 全区間の傾き | 運転ごとの中央値 |
|---|---|---|---|
| 記録された指令推力（その日のファーム） | −0.105 /V | −0.147 /V | 0.95〜1.21 |
| ベンチ・表 B | +0.050 /V | +0.006 /V | 0.90〜1.09 |
| ベンチ・表 B（オンの間の電圧で補正） | +0.042 /V | −0.004 /V | 0.88〜1.07 |
| ベンチ・表 A | +0.120 /V | +0.073 /V | 0.88〜1.07 |

（要る指令推力 ÷ 重さ 37 g の、記録電圧に対する傾き。5 秒区間を独立とみなした誤差は ±0.02 /V で、実際より小さい）

- 記録された指令推力は、電圧が 0.5 V 下がると約 5 % 増える。換算の電圧依存の不足を、速度ループとホバー推力の学習が補っている（推論、強さ：中〜強）。表 B の換算なら、1 回の飛行の中で約 2.5 % 減り（逆向き、大きさは約半分）、飛行をまたぐとほぼ 0。
- 運転ごとの中央値の幅（0.95〜1.21）には、ファームの世代の違いが入る（9 章）。2026-08-22 より後の 1 本は 1.00 だった。
- この表は 4 つの方法で同じ区間・同じ表を使っており、上のホバー Duty の傾きの比較を指令推力で言い直したものである。新しい独立の証拠ではない。

## 7. 今のファームの推力 → Duty の換算

### 構造（事実、ファームのソース）

```
 目標 ─→ [高度・姿勢・レートの制御] ─→ [ミキサー B⁻¹] ─→ [thrustToDuty] ─→ モータ
              出力: 総推力 [N]・トルク [N·m]   各モータの推力 [N]      ↑
                                                                 記録電圧（INA3221）
```

| 項目 | 内容 |
|---|---|
| 制御の出力 | `ControlOutput` の総推力 `thrust` [N] とトルク `torque[3]` [N·m] |
| ミキサー | 総推力とトルクから各モータの推力を逆算（腕の長さ 23 mm、$\kappa = C_Q/C_T = 4.10\times10^{-3}$） |
| 総推力の作り方 | ACRO・STABILIZE: スロットル × 最大推力。ALT_HOLD・POS_HOLD: ホバー推力 mg × `hover.thrust_corr` ＋ 上下速度ループの補正（±0.15 N まで） |
| 換算（`actuator.cpp` の `thrustToDuty`） | $\omega = \sqrt{T/C_T}$（$C_T = 1.00\times10^{-8}$ 暫定）、$V = A_m\omega^2 + B_m\omega + C_m$（$A_m = 6.0368\times10^{-8}$、$B_m = 6.699\times10^{-4}$、$C_m = 0.0153$）、Duty = $V$ ÷ 記録電圧 |
| ホバー推力の学習 | 定常ホバーで速度ループの出力を時定数 20 秒でホバー推力に移し、着陸の瞬間に `hover.thrust_corr` に保存 |

### Duty の変化に対する推力の変化（事実、SILS のプラントで計算）

ホバー推力（1 モータ 0.0907 N）で、Duty を少し変えたときの推力の変化の割合（局所の傾き $dT/dD$）を、SILS のプラントと今のファームの曲線で比べた（それぞれのホバー Duty で評価）。推力そのもの（$T/D$）の比はほぼ 1（0.98〜1.07）で、違うのは傾き。

| 電池電圧 | 3.60 V | 3.65 V | 3.70 V | 3.80 V | 3.90 V | 4.00 V | 4.10 V | 4.20 V |
|---|---|---|---|---|---|---|---|---|
| 傾きの比（プラント ÷ ファーム） | 0.672 | 0.679 | 0.443 | 0.528 | 0.607 | 0.662 | 0.709 | 0.755 |

- 3.65 V と 3.70 V の間の飛びは、プラントのホバー Duty が 0.70 を横切るところ（3.685 V）で、SILS の $R_\text{out}$ の表の折れ目による。レビューで表をなめらかな当てはめに替えて計算し直すと、飛びは消え、比はこの区間で約 0.55 になった。
- 推論（強さ：中）: 今のファームは、指令したトルク・推力の変化の約 0.45〜0.75 倍しか実際には出していない（SILS のプラントが実機を表すとして）。今のゲインはこの過小な出力を込みで調整されている。換算を正確にすると、ループの効きが 1.3〜2.0 倍になる（表の折れ目による 2.3 倍を除く）。この比は飛行では確かめていない。
- 推論（未確認）: 学習値は保存したときの電池電圧の影響を含む。電池が減った着陸時に保存した値を次の満充電の飛行で使うと、ホバー推力の初期値が数 % 高すぎる可能性がある。

## 8. SILS での確認（2026-11-03）

| 名前 | 中身 |
|---|---|
| 今のまま | 比較の基準 |
| F1 | `thrustToDuty` を SILS のプラントの定常の正確な逆に替えた（$\omega = \sqrt{T/(\eta C_T)}$、要る電流、要る $R_\text{out}$、表の逆引き）。逆変換の誤差は Duty で 1.7×10⁻⁶（単精度）。定数と表はプラントと同じ |
| F1g | F1 に加えて、レートループの比例ゲインとトルク上限、高度の速度ループのゲインと補正の上限を 0.593 倍（3.7〜4.1 V の傾きの比の平均。折れ目の 0.443 を含む） |

満充電から 190 秒のホバー（電池 4.19 → 3.37 V、目標高度 0.5 m）:

| 項目 | 今のまま | F1 | F1g |
|---|---|---|---|
| 離陸後の最高高度 | 0.810 m | 0.528 m | 0.596 m |
| 上下動（4 秒ごとの最大 − 最小、中央値／最大） | 2.7／8.6 cm | 1.4／3.8 cm | 1.8／4.9 cm |
| 上下動の中央値（前の 3 分の 1／後の 3 分の 1） | 3.0／2.6 cm | 0.9／1.5 cm | 1.5／2.1 cm |
| 指令推力の電圧への傾き | 重さの −0.175 倍 /V | −0.002 倍 /V | −0.003 倍 /V |
| 学習値 `hover.thrust_corr` | 0.96〜1.03 | 1.00〜1.003 | 1.00〜1.003 |
| ホバー中の平均高度 | 0.487 m | 0.451 m | 0.439 m |
| 高度が目標 ±5 cm に収まるまで | 20.6 s | 収まらない | 収まらない |

再確認試験（45 シナリオ）: 今のまま 41 合格・既知の不合格 3・対象外 1。F1 は 11 不合格、F1g は 10 不合格。モデル一致の判定は 3 つとも合格。

| 分類 | シナリオ | 根拠 |
|---|---|---|
| 較正のずれ（F1・F1g） | acro_flight、alt_flight、modeswitch、pos_pitch、pos_roll、stab_ground_windup | 開ループのスロットル入力を 1.0625 倍にすると、元の基準で合格 |
| 本物の変化（F1） | alt_auto_takeoff、pos_auto_takeoff | 離陸の行き過ぎが消え、実機の行き過ぎに合わせた下限（上下動 0.05 m 以上）を割る |
| 本物の変化（F1・F1g） | stab_flight | 姿勢の追従誤差 0.061〜0.065（上限 0.052、今のまま 0.047）。入力をずらしても不合格 |
| 本物の変化（F1g） | flip_abort_recover | ゲインを縮めた分、宙返り後に高度を戻す力が足りない |
| 原因未特定（F1・F1g） | calib_side_lying、crash_refly | 入力をずらしても不合格 |
| 既知の不合格（3 つとも） | commloss_land_level、pos_yaw、yaw_hold | 不合格の指標が変わった |

（F1 の 11 = 較正 6 + 離陸 2 + stab_flight 1 + 未特定 2、F1g の 10 = 較正 6 + stab_flight 1 + flip_abort_recover 1 + 未特定 2）

- 事実: F1 で電池モデルを切ると、上下動の中央値は 0.23 cm（今のまま 0.26 cm）。
- この SILS の限界: F1 はファームの換算をプラントの逆にしたので、SILS の中で電圧によるずれが消えるのは作りの上で当然である。分かったのは、SILS のホバーの高度のずれと離陸の行き過ぎが、換算とプラントの電圧依存の食い違いから来ていたこと。実機で残る誤差（6 章の逆向きの約 2.5 %）は SILS では確かめられない。
- 推論: 効きが上がった F1 でも、ホバーの揺れは出なかった（姿勢の rms は 1e-10° 程度）。stab_flight の悪化は F1g でも直らないので、効きの上昇とは別の原因と考える。一律にゲインを縮める（F1g）のは、宙返りの後の回復を弱くした。
- 未特定: F1・F1g でホバーの高度が目標より 4〜6 cm 低い（推定高度も低い）。高度の制御には積分があり、学習値も 1.00 なので、定常のずれが残るのは不自然で、原因を調べる必要がある。今のままの 20.6 s は 11/01 の記録（約 8 s）と違い、理由は未特定。
- 実機の離陸の行き過ぎ（2026-06-27 で約 0.8 m、SILS 方針の 2026-11-01 の記録）も、同じ換算のずれが原因の可能性がある（推論、未確認）。

## 9. 計測と照合の注意点（次の作業への引き継ぎ）

| 項目 | 起きたこと | 次からの対応 |
|---|---|---|
| 秤のマイナス表示 | 記号を読まず、負の値を正と読んだ | `read_scale.py` で記号を判定。画角を変えたら記号の位置を決め直す |
| 重さ | 初版では量っていないと書いたが、開発者が量ってあり 36〜38 g だった（2026-11-03 確認） | 機体ごとの値を記録に残す |
| 室温 | 10/15・11/02 とも記録していない | 運転の前後に記録する |
| 既知のおもり | 10/15 は確認していない | 毎回、測定の前後に確かめる |
| 録音 | 11/02 はどの運転でも録っていない | `SF_MIC_NAME` を設定し `--audio` を付ける（4 つのモータの回転数を音で分けられる） |
| $V_{DD}$ の基準 | 記録電圧 − $V_{DD}$ が、午前の 1 モータで −11〜−21 mV + 38〜46 mV/A、午後（4 つの負荷、プロペラ付き）で +17〜+23 mV + 13〜27 mV/A、夜（R15）で +9〜+21 mV + 55〜96 mV/A と違う | オシロの設定とプローブの位置を写真で残す。午後・夜の $V_{DD}$ を記録電圧で補正し直して $V_b$ を確かめる |
| プロペラの下の物 | プローブ・配線が推力を下げた可能性 | どけた場合と置いた場合を同じ Duty で比べる |
| Duty の範囲 | 長時間放電は 0.50〜0.80 だけ（0.9 は夜の 2 段のみ） | 0.85〜0.95 を足す |
| 4 モータの回転数 | M1 しか測っていない | 録音か、M2〜M4 のドレインにオシロを当てる |
| 飛行ログのファームの世代 | 13 本は 2026-08-22 の換算の変更（モータ曲線を 1.12 倍、`hover.thrust_corr` の既定 1.12 → 1.00）より前。6/18 に `hover.thrust_corr` がパラメータになり、7/04 にホバー推力の学習、7/18 に高度の外乱オブザーバが既定で入った。Duty を使う結果は影響を受けないが、記録された指令推力の水準は世代で違う | 飛行ログごとにファームのコミットを記録する |
| 照合の幅 | 「±3 %」は結果を見た後に付けた幅 | 合否の幅は実験の前には決めず、データを見てから決める。前に決めた幅は予想どおりの結果にしか使えず、予想外の結果に対応できないことがある（2026-11-03 実験者） |

## 10. モデル刷新への引き継ぎ（意見）

### モデルに入れる要素の候補

| 要素 | 内容 | 使える範囲・根拠 |
|---|---|---|
| 駆動回路 | $\bar I_m = (V_\text{on} - K_e\omega - V_b)/R_\text{out}(D)$。$R_\text{out}$ は 0.02 刻みの実測でなめらかな表にする | D 0.50〜0.80・$V_{DD}$ 3.63〜4.27 V（4 章）。$V_b$ は式に項を置き、はじめは 0（決めたこと 3） |
| 電源 | 電池の開回路電圧と内部抵抗。4 モータ同時のパルス電流による、オンの間の電圧の落ち込み $\Delta V = R_\text{path} I + R_\text{src} I (1-D)/D$（決めたこと 1） | 4 章。11/02 の長時間放電のデータで決められるが、まだ解析していない（SILS 方針 `docs/architecture/simulation-policy.md` 8 章のバックログ #19） |
| 推力 | $F = F_\text{ref}(\omega/\omega_\text{ref})^n$、10/15 の値、全機は 4 倍 | $\omega$ 1200〜4700 rad/s（5 章・6 章） |
| トルク | $C_Q$ と、一定の摩擦トルクを含む無負荷の損失 | 5 章 |
| 温度 | $K_e$ は温まると下がるが、同じ Duty・電圧での回転数への影響は ±0.6 % 以内 | D 0.50〜0.80 で確かめた範囲では、入れなくてよい |
| 個体差 | 同じ形の機体で推力が ±10 % 程度違う | 6 章。モデルは平均的な機体として作り、個体差は学習で吸収する |

### 定数の候補（測り方と条件付き）

| 量 | 値 | 測り方・条件 |
|---|---|---|
| $K_e$ | $5.90\times10^{-4}$／$5.76\times10^{-4}$／$5.54\times10^{-4}$ V·s/rad | 10/15 のスピンダウン／11/02 のプロペラなしのスピンダウン／11/02 夜の段の中。どれを使っても同じ Duty・電圧での回転数の再現は ±1.2 % |
| 巻線抵抗 $R$ | 0.594 Ω | 10/15 |
| $R_\text{out}(D)$ | 4 章の表（2 通り） | D 0.50〜0.80。外は SILS の表（10/15 の 0.1 刻み） |
| $V_b$ | はじめは 0。候補 0.11 V | 4 つの負荷の比較（推論）。周ごとに 0.06〜0.18 V。$V_{DD}$ の基準を確かめてから値を入れる |
| $I_0$ | $0.044 + 2.9\times10^{-9}\,\omega^2$ A | 10/15。一定の摩擦トルクの形に直す必要あり |
| 推力 | $F_\text{ref} = 0.0916$ N、$\omega_\text{ref} = 3500$ rad/s、$n$ = 2.13〜2.15 | 10/15、$\omega$ 1200〜4700 rad/s |
| $C_Q$ | $3.38\times10^{-11}$ | 10/15（11/02 夜は 3.33e-11） |
| $J$ | $1.54\times10^{-8}$ kg·m² | 10/15（11/02 夜は 1.47〜1.53e-8） |
| 記録電圧 → オンの間の $V_{DD}$ | $\Delta V = R_\text{path} I + R_\text{src} I (1-D)/D$、4 モータで $R_\text{path}$ 2 mΩ・$R_\text{src} = 37 + 95\,(4.2 - V_\text{CLI})$ mΩ | 電池 3 の 1 本だけ。1 モータ（電池 2）は 26 mΩ・72 mΩ。同じ 4 モータのデータに当てはめた式 $781 - 118\,V_\text{CLI} - 324\,D$ mV は残差 12 mV だが、1 モータや別の電池には使えない |
| 重さ | 36〜38 g | 開発者が量った値（照合の計算は 36 g と 37 g） |

### すでに決めたこと（実験者の判断）

10/15 の推力を基準、全機は 4 倍。11/02 の 4 モータ同時の推力は使わない。重さは 36〜38 g（開発者が量った値）。

### 決めたこと（2026-11-03、実験者の判断）

| 項目 | 決定 | 内容 |
|---|---|---|
| 1. ファームが換算に使う電圧 | 物理の形で落ち込みを推定する | $V_\text{on} = V_\text{CLI} - \Delta V$、$\Delta V = R_\text{path} I + R_\text{src} I (1-D)/D$（$I$ は INA3221 の電池電流）。定数は 11/02 の値（4 章）を初期値とし、ファームと SILS の電池モデルに同じ式を入れる。別の電池で 4 モータ同時のベンチを 1〜2 本取り、電池ごとの差を確かめる |
| 2. 推力の式の形 | べき乗 | $F = F_\text{ref}(\omega/\omega_\text{ref})^n$、$\omega_\text{ref}$ = 3500 rad/s、$F_\text{ref}$ = 0.0916 N、$n$ = 2.13（10/15、推力の残差で当てはめ）。SILS の補正係数 $\eta$ は使わない。反トルクは $C_Q\omega^2$ と摩擦の損失の形のままとし、摩擦の当てはめ直しのときに見直す |
| 3. $V_b$ | 式に項を置き、はじめは 0 | 確認の後に値を入れ、$R_\text{out}$ の表を同じデータから組で作り直す。表はどちらにしても $V_\text{on}$ で作り直す。ホバーの範囲では、入れるかどうかの差は推力で 1 % 未満の見込み（筆者の概算、未確認） |

- 4 つの PWM の位相をずらして落ち込みを減らす方法は、試しとして後で行う（ファームの切り替え機能とベンチの試験が要る）。
- べき乗の式の計算（モータごとに `powf` 1 回）の処理時間は、ブランチの最初に実機で測る（見込みは 4 回で十数 µs、2.5 ms の制御周期の 1 % 未満。未確認）。

### まだ要る測定

Duty 0.85〜0.95 の範囲、モータごとの回転数（録音か各ドレイン）、プロペラの下の物の有無の比較、毎回の室温と既知のおもり、機体の重さ、4 つの PWM の位相をずらす試験（解析メモ `analysis/notes/ct_cq_bench_20261015.md` 8 章の計画）、午後・夜の $V_{DD}$ の基準の確認、電池の開回路電圧と内部抵抗の解析、別の電池での 4 モータ同時の運転（1〜2 本）、軸を止めた $V_b$ の直接の測定（プロペラなし・$\omega$ = 0・低い Duty で電圧と電流の直線の切片を見る）。

### 制御への影響

- 換算を正確にすると、ループの効きが今の 1.3〜2.0 倍になる（7 章）。ゲインは一律に縮めるのではなく、新しいモデルで設計し直す（F1g の一律の縮小は宙返りの後の回復を弱くした）。
- 換算に連動して見直すもの: Duty の飽和と余裕（全推力・低い電圧）、上げと下げで違う立ち上がり（35〜40 ms と 50〜55 ms）、ヨーのトルク上限 `rate.yaw.max_torque` とミキサーの $\kappa$、高度の外乱オブザーバ、ホバー推力の学習の時定数（20 秒）と新しい効きの関係。
- 開ループのスロットルを使うシナリオ（担当の数えで約 18）は較正をやり直す。離陸の行き過ぎに合わせた下限など、旧モデルの挙動に合わせたシナリオの基準も見直す。

### 実機に入れるときの安全

- 換算を変えたファームを書き込んだら、`param reset` → `param save` で保存済みの値（`hover.thrust_corr` の学習値など）を消す（2026-08-22 の変更と同じ）。古い学習値のまま飛ばすと推力が 10 % 以上ずれる。
- 最初はプロペラを外すか機体を糸でつないで、Duty の出方と、モータの回転の上げ下げを確かめる。Duty の上限を低めにして浮上させる。
- 個体ごとにホバー推力の学習値を取り直す。低い電圧（3.5 V 以下）での余裕を確かめる。
- 元に戻す手順（前のファームとパラメータ）を用意してから飛ばす。

### ブランチの進め方の案

各段階で、次へ進む前に確かめることを括弧に書く。

1. モデルの基準ファイル（`stampfly_physical.yaml`）に新しいモデルの定数と式を入れる。あわせて、午後・夜の $V_{DD}$ を記録電圧で補正し直して $V_b$ の値を決め、$R_\text{out}$ の表を $V_\text{on}$ で作り直す。実機で換算全体の処理時間を測る（`sf params check` が通る）。
2. SILS のプラントを新しいモデルに替える。Genesis・VPython も同じ基準へ（バックログ #18）（ベンチの段ごとの表を SILS で再現する）。
3. ファームの推力 → Duty の換算を、同じモデルの逆にする（逆変換の誤差を確かめる）。
4. 姿勢・高度の制御を新しいモデルで設計し直す（SILS で安定余裕、Duty の飽和、宙返り、離陸を確かめる）。
5. シナリオの較正と基準を見直し、再確認試験とモデル一致の判定を通す。
6. ベンチと実機で確かめる（上の安全の手順。ホバーの電圧依存、離陸の行き過ぎ、揺れ）。

## 11. データとスクリプトの対応

| 内容 | 出力・スクリプト |
|---|---|
| $R_\text{out}$ の電池電圧依存、冷えた周の表 | ベンチのリポジトリ `tools/analyze_long.py`（11/02 の電気の集計）、`data/2026-11-02_steps.csv` |
| 4 つの負荷と $V_b$ | ベンチの手順書 10 章（11/02 の解析） |
| 段ごとの表（電池 2・3、R15 の D 0.9） | ベンチのリポジトリ `data/2026-11-02_steps.csv`（コミット 2d5f782） |
| オンの間の $V_{DD}$、1 モータと 4 モータの比較 | ベンチのリポジトリ `tools/vdd_on_time.py` |
| 推力（電池 2・3、マイナス記号） | ベンチのリポジトリ `tools/analyze_thrust.py`、`tools/read_scale.py`、`tools/scale_calib_2026-11-02_bat3.json` |
| 10/15 のやり直し（$C_T$・$C_Q$・$K_e$・$J$） | ベンチのリポジトリ `tools/analyze_constants_long.py` |
| ホバーの釣り合い、ホバー Duty の比較 | 本リポジトリ `analysis/scripts/ct_cq_bench_20261015/crosscheck/logcheck/test4.py`（`results/test4.json`、`test4_windows.csv`） |
| 換算の確認 | 同 `test5.py`（`results/test5.json`） |
| ホバー区間の選び方 | 同 `hover.py` |
| SILS の F1・F1g | 同 `analysis/scripts/ct_cq_bench_20261015/patches/F1.patch`・`F1g.patch`、`results/base_1103/`・`F1/`・`F1g/`、`hover_180s.scn`、`hover_metrics.py`（本リポジトリのコミット 3601ac28） |
| 解析の経緯 | 本リポジトリ `analysis/notes/ct_cq_bench_20261015.md` 6 章 |

## 12. このレポートのレビュー（2026-11-03）

本文を書いた後、別の担当（AI）4 つに次のレビューをさせ、指摘を本文に取り込んだ。

| 層 | 内容 | 主な結果 |
|---|---|---|
| 反証テスト | 主張ごとに「偽ならどう見えるか」を決め、生データから計算し直す | $R_\text{out}$ の電圧依存、推力のべき乗と 2 日の比、ホバー Duty の傾き、傾きの比の飛びと表の折れ目は再現した。$n$ は推力の残差なら 2.12〜2.13。録音との一致は 7 段で ±0.6 %、D 0.5 は ±1.6 % まで散る。4 モータ同時は 1 モータの電圧範囲の外では確かめられない |
| コードの点検 | 計算スクリプトの選び方・外挿・決め打ちの値・循環 | 重さの決め打ち、表の外挿、傾きの誤差の過小評価、ホバー区間の条件が出力に残っていないこと。F1 の定数と表はプラントと一致 |
| 文脈を知らない読者 | 文書だけを読んで批評 | 運転名・表・記号の定義、主張の強さ、ラベルの付け方、定数の一覧、使える範囲 |
| 分野のチェックリスト | 同定・統計・構成管理・制御設計・安全 | 飛行ログのファームの世代、機体の同一性、効きの比の上限が表の折れ目に由来すること、安全の節、独立の確認と言い過ぎた箇所 |

「独立の確認」と書いたもの（写真の目視、録音、逆変換の誤差）以外は、計算者と同じ AI の再計算であり、人による確認ではない。
---

<a id="english"></a>

**Status**: Basic reference (written 2026-11-03). It collects what the experiments of 2026-11-02 to 11-03 showed, as the starting point for the work (planned on a separate branch) of renewing the propulsion model, redoing the firmware's thrust conversion and control design, and rebuilding the SILS plant. This document does not change the reference file of physical parameters (`control/models/stampfly_physical.yaml`) or the SILS policy backlog. An HTML version with charts: https://claude.ai/artifact/MhqcHBYiR48yuygWQDnvCJ (sharing is needed to view it). Updated 2026-11-03: the three open points are decided (chapter 10, "Decided"). The handling of the check width in chapter 9 is corrected (the pass width is decided after seeing the data). The weight is corrected (weighed by the developer, 36 to 38 g).

## 1. Overview

### About this report

This report summarizes the propulsion system of the StampFly (motor, drive circuit, propeller): the bench experiment of 2026-11-02, the redo of the 2026-10-15 bench experiment, the cross-check against flight logs, and the SILS tests (2026-11-03). Each item carries one of the following labels.

| Label | Meaning |
|---|---|
| Fact | A measured value and its calculation. Chapter 11 shows which script produced each output. The calculations were done by Claude (an AI worker). In the review of chapter 12, another worker (an AI) recomputed the main numbers from the raw data. No human has confirmed them independently |
| Confirmed or decided by the experimenter | What the experimenter (the developer) checked, remembers, or decided |
| Inference | What was reasoned from the facts. It carries a strength (strong, medium, weak) and alternative explanations. The strength is a rough guide, not a calibrated value |
| Opinion | A proposal for the next work (chapter 10) |

### Summary of conclusions

| Item | Content | Label |
|---|---|---|
| Drive circuit $R_\text{out}$ | Dependence on battery voltage is small. From 4.27 to 3.63 V it changes by 0 to −1.3 % (+1.6 ± 0.4 % per volt in the fit over all laps) | Fact |
| Constant voltage drop $V_b$ | A formula with a drop of about 0.11 V at every Duty fits. It varies from 0.06 to 0.18 V between laps | Inference (medium) |
| Reproducibility of rotor speed | For the same motor at the same Duty and voltage: −1.2 to +0.8 % across days, within ±0.6 % between cold and warm | Fact |
| Four motors together | The four motors switch on at the same time, and the on-time voltage drops more than the mean voltage. Within the voltage range measured with one motor, the speed difference is 0 ± 0.3 % when compared at the on-time voltage | Fact |
| Thrust | $F/\omega^2$ is not constant. $F = k\,\omega^{n}$ ($n$ = 2.13 to 2.15, depending on the fitting method) fits on both days. The thrust of the 11/02 night is 0.958 to 0.961 times that of 10/15, independent of rotor speed | Fact |
| Values to adopt | Use the 10/15 thrust as the reference, and take the thrust of the whole vehicle as 4 times it. Do not use the 11/02 four-motor thrust | Experimenter's decision |
| Cross-check against flight | 4 × the 10/15 thrust ÷ weight (calculated for 36 g and 37 g; the weighed value is 36 to 38 g) is 0.981 to 1.034. This check cannot tell the 10/15 and 11/02 thrust apart | Fact |
| Voltage dependence of hover Duty | Flight −0.21 to −0.26 /V (within one flight / all segments), bench −0.24 to −0.29 /V, current firmware conversion (Duty × voltage constant) −0.168 /V | Fact |
| Current firmware conversion | Its voltage dependence is too weak, and the altitude control and the thrust learning make up the difference (the logged commanded thrust rises as the voltage drops) | Fact and inference (medium to strong) |
| SILS | If the conversion is made the exact inverse of the plant, the voltage-dependent altitude offset and the takeoff overshoot disappear inside the SILS (partly by construction). The loop gain effect becomes 1.3 to 2.0 times | Fact |

### Target readers

People who rebuild the propulsion model, the SILS plant, and the firmware's thrust conversion and control design.

## 2. Terms and quantities

### Voltages (the value depends on where it is measured)

| Name | Where and how it is measured | Where it is used |
|---|---|---|
| Logged voltage (CLI voltage) | Bus voltage of the current sensor INA3221 on the battery path. The vehicle command (CLI) `power` and the battery voltage in flight logs show this value | Flight logs, firmware conversion |
| Mean $V_{DD}$ | The mean of the supply of the M1 (front right motor) drive circuit, measured with an oscilloscope | Map B of bench rotor speed |
| On-time $V_{DD}$ | The mean of $V_{DD}$ only while the M1 PWM is on | Chapter 4, "Four motors together" |

| Quantity compared (range or median over steps) | One motor | Four motors together |
|---|---|---|
| Logged voltage − mean $V_{DD}$ (range) | 4 to 32 mV | 9 to 80 mV |
| Mean − on-time $V_{DD}$ (median) | 31 mV | 83 mV |
| Off − on $V_{DD}$ (range) | 101 to 127 mV | 203 to 303 mV |
| Logged voltage − on-time $V_{DD}$ (range) | 33 to 67 mV | 73 to 170 mV |

For the runs from the afternoon on (the comparison of four loads on 11/02 and R15), the reference of the $V_{DD}$ reading differs from the morning (chapter 9).

### Symbols

| Symbol | Meaning | Unit |
|---|---|---|
| $D$ | Duty (the value the firmware gives to the PWM. The PWM is 150 kHz, 8 bit) | — |
| $\omega$ | Rotor speed of the motor. Measured from the voltage ripple caused by commutator switching (10 per revolution) | rad/s |
| $K_e$, $K_t$ | Back-EMF constant and torque constant (the same value in SI units) | V·s/rad, N·m/A |
| $I_m$ | Mean motor current (for four motors together, taken as one quarter of the battery current) | A |
| $R_\text{out}(D)$ | Equivalent output resistance of the drive circuit (set by Duty and the capacitor. The model of the developers' paper) | Ω |
| $V_b$ | Constant voltage drop (chapter 4) | V |
| $I_0(\omega)$ | No-load current | A |
| $F$, $C_T$ | Thrust of one motor, and the thrust coefficient $C_T = F/\omega^2$ | N, N/(rad/s)² |
| $k$, $n$, $F_\text{ref}$, $\omega_\text{ref}$ | Constants of the power law $F = k\,\omega^n = F_\text{ref}(\omega/\omega_\text{ref})^n$ for thrust | — |
| $C_Q$ | Torque coefficient | N·m/(rad/s)² |
| $J$ | Moment of inertia of the rotating part (motor + propeller) | kg·m² |
| $\eta$ | Thrust efficiency of the SILS plant (`thrust_efficiency`, 0.976) | — |

### Names of runs, maps, and tests

| Name | Content |
|---|---|
| Step | One segment run at one Duty (5 s in the 11/02 long discharge, 16 s in R15) |
| Lap | One pass through the Duty sequence (0.50, 0.60, 0.62 … 0.80) |
| Batteries 1 to 3 | The three batteries used on 11/02 |
| A1, C1 to C3 | Cold laps of battery 2 (A1 right after a full charge, C1 to C3 after the motor cooled for 10 minutes) |
| B1 to B3 | Runs of battery 2 with four laps in a row (from the second lap on, the motor is warm) |
| A4M, B4M, C4M | Runs of battery 3 with four motors together |
| Comparison of four loads | The afternoon of 11/02 (16:00 hour). No propeller, 4-blade, 3-blade, and 4-blade with raised pitch |
| R15 | The redo of the 10/15 test on the night of 11/02 |
| D-low | A run that changed Duty in a step and measured the rise of rotor speed |
| Map A | A fit of the M1 rotor speed with four motors together, as a function of Duty and logged voltage (battery 3, 42 steps) |
| Map B | A fit of the one-motor rotor speed as a function of Duty and mean $V_{DD}$ (battery 2, 192 steps). When applied to flight logs, subtract the four-motor "logged voltage − mean $V_{DD}$" from the logged voltage |
| F1, F1g | Firmware changes tried in the SILS (chapter 8) |
| SILS | A simulation that runs the firmware source as it is on a PC |
| Regression tests | Automatic tests that check a change did not break existing behaviour (`sf sils regression`, 45 scenarios) |
| Model-match check | A check that compares the SILS and real-vehicle flip logs (`sf sils sysid-gate`) |

### Other quantities

| Quantity | Content |
|---|---|
| Thrust | Thrust of one motor [N or g]. The thrust of the whole vehicle is the sum of the four motors |
| Weight | Weight of the vehicle. The developer has weighed it: 36 to 38 g (confirmed by the developer on 2026-11-03; the first version said "not weighed, estimate 36 to 37 g"). The checks in this report were calculated for both 37 g and 36 g |

## 3. Experiments and data

| Date | Experiment | Content |
|---|---|---|
| 2026-10-15 | One-motor bench (reference) | M1 with propeller. Duty 0.1 to 0.9 ($V_{DD}$ 4.17 to 4.38 V). Rotor speed, current, thrust, spin-down. Audio recorded |
| 2026-11-02 | Long discharge (batteries 1 to 3) | Battery 2: one motor, Duty 0.50 to 0.80 (steps of 0.02), until the loaded $V_{DD}$ fell from 4.27 to 3.63 V. Battery 3: three runs with four motors together |
| 2026-11-02 | Comparison of four loads | $K_e$ measured inside a step (PWM stopped for 0.08 s) |
| Night of 2026-11-02 | R15 | Same propeller, same height, Duty 0.1 to 0.9, spin-down |
| 2026-06-14 to 09-08 | 14 flight logs | Another unit of the same design (not the bench vehicle). 13 logs predate the firmware conversion change of 2026-08-22 (chapter 9) |
| 2026-11-03 | SILS | Two variants with a different thrust → Duty conversion (F1, F1g) |

The bench vehicle is placed upside down on a can. The thrust pushes the vehicle against the scale, the propeller outflow goes up, and the intake side is below.

Published data: `data/2026-10-15/` and `data/2026-11-02_steps.csv` (table per step of 11/02) in the bench repository `stampfly-motor-autotest`. Photos and oscilloscope waveforms are kept outside the repository. The procedure and the record of the day are in chapter 10 of `docs/procedure_long_discharge.md` in the same repository.

## 4. Drive circuit (duty and voltage to rotor speed)

### The dependence of R_out on battery voltage is small (fact)

Between cold laps (A1: 4.27 to 4.13 V, C3: 3.72 to 3.63 V), C3 ÷ A1 − 1 of $R_\text{out} = (V_{DD} - K_e\omega)/I_m$ ($K_e = 5.7628\times10^{-4}$, from the 11/02 spin-down without propeller) is −1.33 to +0.03 % at Duty 0.50 to 0.80 (mean −0.8 %). In the fit over all cold laps it is +1.6 ± 0.4 % per volt (slightly smaller at lower voltage).

- Changing $K_e$ by ±2 % moves the result by about the same amount, −0.6 to −1.0 %.
- In warm laps, $R_\text{out}$ at the same Duty is 0.4 to 0.75 % larger, which is as large as the voltage effect.
- This agrees within about 1 % with the assumption of the developers' paper that "$R_\text{out}$ is set only by Duty and the circuit and does not depend on battery voltage".

### Constant voltage drop V_b (inference, strength: medium)

With the formula using $K_e$ measured at zero current, $R_\text{out}$ without propeller comes out 5.7 to 18.7 % larger than with propeller at the same Duty (D 0.2 to 0.5, fact). Comparing the four loads, the following formula, which adds the same constant voltage drop $V_b$ at every Duty, fits well.

$$\bar I_m = \frac{V_{DD} - K_e\,\omega - V_b}{R_\text{out}(D)}$$

| Fit (fact) | Result |
|---|---|
| Intercept per Duty | 108 to 123 mV |
| Common $V_b$ | 110 mV (residual rms 39.7 mV) |
| Formula with a term proportional to $\omega \cdot I$ instead (same number of parameters) | Residual rms 51.7 mV |

- $V_b$ has the size of a voltage drop from brush contact. Other origins, such as the saturation voltage of the drive circuit or the wiring drop, are not excluded.
- The discrimination rests on the difference between the residuals 39.7 mV and 51.7 mV, which is not strong.
- Between the first and second lap of the same load it varies from 60 to 180 mV, so a dependence on time (how warm the motor is) remains.
- The comparison of four loads is an afternoon run, and its $V_{DD}$ reading reference differs from the morning by about 30 mV (chapter 9). $V_b$ may move by up to about 0.03 V (not checked).

### Measured R_out(D) (steps of 0.02 in Duty, fact)

Mean of the cold laps (A1, C1, C2, C3). The table shows the formula without $V_b$ and the formula with it, next to the SILS plant table (the paper's calculated values corrected with the 10/15 measurement in steps of 0.1).

| Duty | $(V_{DD}-K_e\omega)/I_m$ [Ω] | Value with $V_b$ = 0.11 V subtracted [Ω] | SILS table [Ω] | $I_m$ [A] |
|---|---|---|---|---|
| 0.50 | 3.453 | 3.277 | 3.471 | 0.626 |
| 0.60 | 2.520 | 2.376 | 2.563 | 0.766 |
| 0.62 | 2.412 | 2.272 | 2.434 | 0.786 |
| 0.64 | 2.312 | 2.174 | 2.322 | 0.804 |
| 0.66 | 2.229 | 2.095 | 2.226 | 0.820 |
| 0.68 | 2.142 | 2.011 | 2.144 | 0.840 |
| 0.70 | 2.059 | 1.930 | 2.072 | 0.858 |
| 0.72 | 1.977 | 1.851 | 1.970 | 0.878 |
| 0.74 | 1.890 | 1.767 | 1.872 | 0.901 |
| 0.76 | 1.793 | 1.674 | 1.772 | 0.927 |
| 0.78 | 1.691 | 1.576 | 1.665 | 0.958 |
| 0.80 | 1.546 | 1.436 | 1.547 | 1.006 |

- Range of use: Duty 0.50 to 0.80, $V_{DD}$ 3.63 to 4.27 V, one motor.
- The values without $V_b$ agree with the SILS table within ±1.7 %.
- The SILS table has a kink at D 0.70 (a measured point of 10/15): the slope per unit Duty is −3.6 Ω for 0.68 to 0.70 and −5.1 Ω for 0.70 to 0.72. The measurement in steps of 0.02 is smooth over the same interval, with −4.2 and −4.1 Ω.
- $R_\text{out}$ of M1 with four motors together (calculated by dividing the battery current by 4) is 3 to 7 % larger than for one motor at the same Duty. This shows that the current sharing or the wiring drop is not the same as for one motor (inference).
- Which formula's table to use as the reference will be decided after the check of the $V_{DD}$ reference in chapter 9.

### Reproducibility of rotor speed (fact)

| Compared | Rotor speed difference |
|---|---|
| 10/15 and 11/02 (same M1, same Duty and mean $V_{DD}$, D 0.5 to 0.8) | −1.2 to +0.8 % |
| Cold and warm laps (11/02, battery 2) | Within ±0.6 % (mean residuals of map B are +0.24 % and −0.19 %; laps 2 to 4 of B1 are −0.25 to −0.59 %) |

- The 10/15 $\omega$ was checked against the audio recording (the recording is independent of the way rotor speed was measured; it was made by the same experimenter). Compared with the one-per-revolution line, 7 of 10 steps are within ±0.6 %. The two steps at D 0.5 are +1.06 % and −0.85 % on the strongest line (other lines scatter up to ±1.6 %). At D 0.7 there is no usable line.
- $K_e$ falls by 2 to 4 % within a few minutes as the motor warms up in operation (measured by spin-down and inside steps). Even so, the rotor speed at the same Duty and voltage changes by only ±0.6 %. We think it is cancelled by an opposite effect such as the rise of winding resistance (inference, strength: weak).
- $K_e$ differs by up to 6 % depending on the method: $5.90\times10^{-4}$ from the 10/15 spin-down, $5.76\times10^{-4}$ from the 11/02 spin-down without propeller, and $5.54\times10^{-4}$ from the measurement inside the steps of the 11/02 night. It does not show up in the rotor speed difference at the same Duty and voltage (±1.2 %), so we think it comes from the difference in method (inference, strength: medium, not checked).

### Sensitivity near hover (fact, map B, around D 0.65)

| Change | Rotor speed | Thrust |
|---|---|---|
| $V_{DD}$ +0.1 V | +1.7 to 1.8 % | +3.6 to 3.9 % |
| Duty +0.01 | +0.8 % | +1.7 to 1.9 % |

### Rise

- Fact: When Duty was changed in a step, the time to reach 63 % of the change in rotor speed was 35 to 40 ms for an increase and 50 to 55 ms for a decrease (D-low).
- Inference: The increase agrees well with the time constant of 38.8 ms near hover of the paper's $R_\text{out}$ model. The slower decrease is consistent with the formula being nonlinear. The trajectory was not compared with a numerical solution of the nonlinear formula.

### Four motors together (on-time voltage)

Fact: The four motors switch on at the same time from the same timer. While on, the pulse current of four motors (4.1 to 5.0 A) flows through the battery and wiring, and the $V_{DD}$ of M1 drops more than its mean (table in chapter 2). The M1 rotor speed with four motors was compared with the one-motor map (same Duty, same $V_{DD}$).

| Voltage used to read the map | Steps inside the one-motor voltage range | Mean difference | rms |
|---|---|---|---|
| Mean $V_{DD}$ | 27 / 42 | −0.97 % | 1.24 % |
| On-time $V_{DD}$ | 21 / 42 | +0.09 % | 0.68 % |

- The rms of the fit of the one-motor map itself is 0.71 %. The number of steps inside the range differs between the two rows because the on-time voltage is lower, so more steps fall outside the range.
- In the recalculation in the review (cold laps only, four-term formula), the result is −0.21 ± 0.15 % inside the range (21 steps) and −0.87 % outside it (lap 2 of B4M and C4M).
- Inference (strength: medium to strong): Within the voltage range measured with one motor, the same motor turns at the same speed at the same Duty and the same on-time voltage, regardless of how many motors run together. Outside the range (on-time $V_{DD}$ below 3.61 V) it is not checked. Warm laps can also be 0.5 to 0.9 % lower, so we cannot exclude that the −0.9 % outside the range is due to warming or a different load.
- The difference between logged voltage and on-time $V_{DD}$ (four motors together) is 73 to 170 mV per step. $\Delta V = 781 - 118\,V_\text{CLI} - 324\,D$ [mV] (residual rms 12 mV, value for battery 3 only). Per pulse current it is 15 to 39 mΩ, and it is larger as the battery runs down.
- The same difference was refitted with the physical form $\Delta V = R_\text{path}\,I + R_\text{src}\,I\,(1-D)/D$ ($I$ is the mean total battery current, $D$ the Duty) (2026-11-03, no intercept; computed by the author, not independently checked). The first term is the wiring drop between the current sensor and the motor drive circuit; the second is the extra drop below the mean that the pulse current, flowing only during the on-time, makes across the battery and wiring resistance.

| Data | $R_\text{path}$ | $R_\text{src}$ | Residual rms |
|---|---|---|---|
| One motor (battery 2, difference 33 to 67 mV) | 26 mΩ | 72 mΩ | 4 mV |
| Four motors (battery 3), constant $R_\text{src}$ | 10 mΩ | 61 mΩ | 25 mV |
| Four motors (battery 3), $R_\text{src}$ growing as the battery runs down | 2 mΩ | $37 + 95\,(4.2 - V_\text{CLI})$ mΩ | 13 mV |

- Inference (strength: medium): with a different battery and a different number of motors, $R_\text{src}$ agrees at about 60 to 70 mΩ, so one form describes one motor and four motors. The four-motor data come from one battery only, and the difference between batteries is not known. As there is no intercept, the shift of the $V_{DD}$ reading reference (chapter 9) is mixed into $R_\text{path}$.
- A vehicle in flight cannot measure the on-time $V_{DD}$ directly. The INA3221 averages 64 conversions of 1.1 ms each (about 0.4 s for three channels), which is orders of magnitude longer than one PWM period of 6.7 µs.
- The spectrum of the supply voltage had lines that do not appear in one-motor runs (15 of the 42 steps of battery 3; 0.92 to 0.97 times the M1 line, and 1.04 times in one step). Some motor (the number is unknown) is 3 to 8 % slower than M1. The rotor speeds of M2 to M4 were not measured.

## 5. Thrust and torque (rotor speed to thrust and torque)

### Scale reading (fact)

The scale shows values below zero with a horizontal bar (a minus sign) to the left of the digits. The original automatic reading of 11/02 did not look at this sign. It read the negative values at rest (−0.54 to −3.20 g for battery 2, −0.3 to −2.2 g for battery 3) as positive, and so read the thrust smaller by twice the zero offset. The reading was fixed to read the sign (the position of the sign is set for each camera view). All thrust values of 11/02 are values read with the sign.

| Check | Result |
|---|---|
| Battery 2 | Judged by the darkness at the sign position. The sign appears in 82 to 100 % of the photos at rest and in 0 % of the photos while turning |
| Battery 3 | The darkness at the sign position separated cleanly into 0 to 10 and 60 to 140. Another worker read 11 photos by eye, and all 11 matched the judgement |
| Night (R15) | The calibration of the night's camera view was checked with 10 photos read by eye |
| 10/15 | No sign appears in the 144 photos. The values copied by hand matched the photos in 3 photos |

### Thrust curve (fact)

| Model (number of parameters) | Residual rms (10/15) | Residual rms (11/02 night) |
|---|---|---|
| $F = C_T\,\omega^2$ (1) | 0.23 g | 0.21 g |
| $F = C_T\,\omega^2 + b$ (2) | 0.11 g ($b$ = −0.41 g) | 0.10 g ($b$ = −0.37 g) |
| $F = a\,\omega^2 + b\,\omega$ (2) | 0.070 g | 0.066 g |
| $F = k\,\omega^{n}$ (2), fitted on log residuals | 0.071 g ($n$ = 2.148) | 0.072 g ($n$ = 2.140) |
| $F = k\,\omega^{n}$ (2), fitted on thrust residuals | 0.051 g ($n$ = 2.127 ± 0.010) | 0.064 g ($n$ = 2.122 ± 0.009) |

| $\omega$ [rad/s] | about 1200 | about 2400 | about 3600 | about 4700 |
|---|---|---|---|---|
| $F/\omega^2$ per step (10/15) | 6.30e-9 | 7.18e-9 | 7.48e-9 | 7.78e-9 |
| $F/\omega^2$ per step (11/02 night) | 6.09 to 6.24e-9 | 6.74 to 6.79e-9 | 7.15 to 7.19e-9 | 7.40 to 7.43e-9 |

- $F/\omega^2$ grows with $\omega$ on both days (from 1200 to 4700 rad/s it grows by 23 % on 10/15 and by 19 to 22 % on 11/02).
- 11/02 night ÷ 10/15 is 0.961 in a fit with a common exponent (2.142), and 0.958 in the comparison by interpolation in the review (0.940 to 0.973; the slope against rotor speed is +0.0033 ± 0.0023 /(1000 rad/s), not significant). At D 0.9 it is also 0.955 to 0.958.
- The 10/15 curve can be written around hover as follows. Range of use: $\omega$ 1200 to 4700 rad/s.

$$F = F_\text{ref}\left(\frac{\omega}{\omega_\text{ref}}\right)^{n},\qquad \omega_\text{ref} = 3500\ \text{rad/s},\quad F_\text{ref} = 0.0916\ \text{N}\ (9.34\ \text{g}),\quad n = 2.13\ \text{(thrust residuals) to}\ 2.15\ \text{(log residuals)}$$

- $F_\text{ref}$ is 91.6 mN regardless of the fitting method (difference 0.05 %). The difference in $n$ is 2 % of thrust at 1200 rad/s.
- The slope near hover $dF/d\omega = nF/\omega$ is 6 to 7 % larger than $2F/\omega$ for constant $C_T$. With $C_T = 7.60\times10^{-9}$ through the origin, thrust at hover is 1.5 % larger, the slope is 5.6 % smaller, and at 1200 rad/s the thrust is off by about 20 %.
- $F = a\omega^2 + b\omega$ fits about as well as the power law (same number of parameters). Which to use is a matter of choice, and the power law is easy to invert (opinion).
- Inference (strength: weak to medium): The rise of $F/\omega^2$ with rotor speed may be a low Reynolds number effect of a small propeller (the Reynolds number was not calculated). The formula with only a zero offset ($C_T\omega^2 + b$) fits poorly, and it is hard to explain why the same shape appears on two days with different zero handling.

### Difference between 10/15 and 11/02

| Candidate | Result | Label |
|---|---|---|
| Scale reading | No sign in the 10/15 photos, hand-copied values matched | Fact |
| Cable fixing | When the cable is also fixed to the desk, the zero shifts by −3.4 g and returns at +0.1 g/min, but the scale factor (26.84 to 26.88 g with six 10-yen coins) and the thrust while turning (ratio per step 0.99 to 1.01, only D 0.3 is 1.05) do not change | Fact |
| Air density | With the Japan Meteorological Agency observations for Kanazawa, at the same room temperature 11/02 ÷ 10/15 = 0.993 | Fact |
| Room temperature | The room temperature on 11/02 was not higher than on 10/15 (not recorded on either day). The density ratio does not go below 0.993 | Experimenter's recollection |
| Propeller and mounting height | Same | Confirmed by the experimenter |
| Reading of $\omega$ | The 10/15 $\omega$ agrees with the audio (chapter 4). A comparison without $\omega$ (thrust per battery current at the same Duty) also gives 0.927 to 0.965 times for 11/02 | Fact |
| Probe and wiring below the propeller (intake side) | There may have been more on 11/02 than on 10/15 | Experimenter's recollection |
| How the above acts | At the same $\omega$, the thrust on 11/02 is 2 to 6 % smaller and the current (torque) is 1 to 4 % larger (D 0.4 and above). This matches a disturbed intake flow | Inference (medium, not checked) |

- The ratios differ (0.961 at the same $\omega$, 0.927 to 0.965 per current at the same Duty, 2 to 6 % per Duty converted to the same $\omega$) because the compared quantities differ. The ratio per current also contains the difference in current.
- $C_Q$ is almost the same (3.38 and 3.33e-11) although the current is 1 to 4 % larger, because $C_Q$ is calculated with the $K_e$ ($K_t$) of each day (the $K_e$ of the 11/02 night is 6 % smaller).

### Values to adopt (experimenter's decision, 2026-11-02)

- The cause has not been identified. Given that, with the same propeller and height, an obstacle on the intake side of this bench only makes the thrust look smaller, the larger thrust of 10/15 is taken as the reference.
- The thrust of the whole vehicle is 4 times the one-motor thrust of 10/15.
- The 11/02 four-motor thrust is not used as a measurement of the thrust of the whole vehicle, because the position of the other propellers and the can differs from the one-motor case. The scale reading at the real vehicle's hover Duty was about 34 g, 6 to 8 % smaller than the weight. The cause of this shortfall has not been identified (candidates are interference of the four outflows, the can, and motors other than M1 being slower).
- The one-motor values of the 11/02 night are kept as "values for the condition with a probe and wiring below the propeller".

### Torque and inertia (fact)

| Quantity | 10/15 | 11/02 night |
|---|---|---|
| $C_Q$ | $3.38\times10^{-11}$ | $3.33\times10^{-11}$ |
| $C_Q$ per step | — | $5.0\times10^{-11}$ at 1200 rad/s → $3.3\times10^{-11}$ at 4700 rad/s |
| $J$ (with propeller) | $1.54\times10^{-8}$ kg·m² | $1.47$ to $1.53\times10^{-8}$ (fit without a fixed shape) |
| No-load current $I_0$ | $0.044 + 2.9\times10^{-9}\,\omega^2$ A | — |

- Inference (strength: medium): The larger $C_Q$ per step at low rotor speed depends strongly on how the no-load current (friction) is subtracted. A formula with the no-load current as $a + b\,\omega^2$ does not fit the spin-down, and a constant friction torque has to be included.

## 6. Cross-check against flight logs

### How hover segments were selected (fact)

From 14 flight logs (UDP telemetry; Duty is the 50 Hz command value, battery voltage about every second), segments meeting the following conditions were selected and cut into 5 to 10 s pieces (1 to 5 s for short segments): 134 segments, 574 s in total. Flight state is flying, altitude 0.3 m or more, 2 s or more after takeoff, vertical acceleration below 0.5 m/s² (velocity smoothed over 0.5 s and differentiated), vertical velocity below 0.2 m/s, and the 1.5 s before and after angular velocity exceeding 5 rad/s excluded. Segments are weighted by their duration. One log of 6/27 02:01 makes up about 30 % of the total.

### Hover balance (fact)

For each segment, the rotor speeds were computed from the four logged Duty values and the battery voltage, the thrust of the 10/15 formula was summed over four motors, and the result was compared with the weight.

| Method | 37 g | 36 g |
|---|---|---|
| Map A and the 10/15 formula | 0.986 | 1.013 |
| Map B and the 10/15 formula | 1.006 | 1.034 |
| Fit of the 11/02 one-motor thrust directly on Duty and $V_{DD}$ | 0.981 | 1.008 |

(Thrust ÷ weight, time-weighted mean. Taking the median of the per-run medians gives 1.009 to 1.037 for map A and 1.020 to 1.049 for map B.)

- Assumptions: the four motors are the same as M1 (the M1 map is applied to the four Duty values), steady state, and the bench and flight vehicles and propellers have the same design. Ground effect is small (altitude 0.3 m or more).
- Extrapolation: in low-voltage segments (32 segments with logged voltage below 3.7 V, the lowest 3.49 V), map B is outside the measured range ($V_{DD}$ 3.63 V and above). When solving Duty backwards, the search runs over 0.3 to 1.0 (the map was built over 0.5 to 0.8).
- The third method agreed within 1.5 % with the 11/02 thrust passed through map B (1.006 × 0.961 = 0.966). It is not an independent check, because it uses the same bench data, the same voltage correction, and the same weight.
- The median per run is 0.88 to 1.07. Unit-to-unit variation (weight, motor, propeller) is a candidate. In the record of 2026-06-18, a new motor of the same design gave about 1.34 times the thrust per Duty of a well-used vehicle (hover Duty 0.71 → 0.53).
- Inference (strength: medium): "Whole vehicle = 4 times one motor" is not inconsistent with flight. There is no sign of losing 5 % or more of thrust to interference in flight. However, given the weight range (2.7 %), the fact that the rotor speed map holds for the 11/02 conditions, and the unit-to-unit variation (±10 %), this check cannot tell 10/15 and 11/02 (a 3.9 % difference) apart. The "±3 %" is a width set after seeing the result, and map B at 36 g (1.034) is outside it.

### Hover Duty and battery voltage (fact)

| Logged voltage | Flight hover Duty (segments, runs) | Bench, map A | Bench, map B | 11/02 thrust directly |
|---|---|---|---|---|
| 3.5 V | 0.731 (8, 1) | 0.746 to 0.763 | 0.716 to 0.733 | 0.739 to 0.756 |
| 3.6 V | 0.705 (11, 3) | 0.713 to 0.729 | 0.690 to 0.706 | 0.709 to 0.726 |
| 3.7 V | 0.690 (23, 8) | 0.686 to 0.702 | 0.668 to 0.684 | 0.685 to 0.701 |
| 3.8 V | 0.662 (19, 6) | 0.651 to 0.666 | 0.638 to 0.654 | 0.652 to 0.668 |
| 3.9 V | 0.636 (25, 11) | 0.626 to 0.640 | 0.616 to 0.631 | 0.629 to 0.644 |
| 4.0 V | 0.616 (37, 9) | 0.602 to 0.615 | 0.595 to 0.609 | 0.607 to 0.622 |
| 4.1 V | 0.599 (11, 8) | 0.588 to 0.601 | 0.582 to 0.597 | 0.595 to 0.609 |

(The bench columns are the Duty that supports a weight of 36 to 37 g. The 3.5 V row comes from one log only, and map B is extrapolated there.)

| Slope of hover Duty against voltage | Per volt |
|---|---|
| Flight, all segments (weighted by the square root of segment duration) | −0.242 |
| Flight, all segments (unweighted; 95 % range −0.30 to −0.23 from a bootstrap by run) | −0.256 |
| Flight, within one flight (value after subtracting the mean of each of the 14 logs, time-weighted) | −0.218 |
| Same (unweighted; 95 % range −0.236 to −0.178) | −0.210 |
| Bench, map B | −0.24 |
| Bench, 11/02 thrust directly | −0.26 |
| Bench, map A | −0.29 |
| Current firmware conversion (Duty × voltage constant, $-D/V$) | −0.168 |

- The slope over all segments mixes in unit-to-unit variation (low-voltage segments are mostly June runs, high-voltage ones mostly July). Leaving out one log at a time gives −0.237 to −0.269, so it is not dragged by a single log.
- The slopes within one flight (four logs with a voltage span of 0.3 V or more: −0.240, −0.204, −0.163, −0.173 /V) are all gentler than the bench map B (−0.264, −0.238, −0.234, −0.229 /V). The reason is not identified. On the bench, warm laps give a slightly lower rotor speed, so warming does not explain it. In flight, voltage and time change together.
- The errors of the slopes (the ± notation other than the bootstrap) treat the 5 s segments as independent and are smaller than the real ones.
- If the logged commanded thrust is added as an explanatory variable, the slope against voltage becomes about −0.17 /V (calculation in the review). However, the logged Duty is itself the value the firmware converted from the commanded thrust and the voltage, so this only extracts the slope of the conversion (−0.168 /V) and does not refute the physical slope.
- The Duty for 37 g measured on the scale with four motors together (A4M 0.726, B4M 0.764) is 0.07 to 0.09 larger than in flight at the same voltage.

### Would the commanded thrust become flat if the firmware conversion were replaced? (fact)

The steady hover Duty and voltage are set by physics, so the commanded thrust that a conversion needs to produce them is the forward model of that conversion evaluated at the logged Duty. The logged commanded thrust is the firmware value of that day (chapter 9; 13 logs predate the conversion change of 2026-08-22).

| Conversion | Slope within one flight (14 logs) | Slope over all segments | Median per run |
|---|---|---|---|
| Logged commanded thrust (firmware of that day) | −0.105 /V | −0.147 /V | 0.95 to 1.21 |
| Bench, map B | +0.050 /V | +0.006 /V | 0.90 to 1.09 |
| Bench, map B (corrected with the on-time voltage) | +0.042 /V | −0.004 /V | 0.88 to 1.07 |
| Bench, map A | +0.120 /V | +0.073 /V | 0.88 to 1.07 |

(Slope against logged voltage of needed commanded thrust ÷ weight 37 g. The error treating 5 s segments as independent is ±0.02 /V and is smaller than the real one.)

- The logged commanded thrust increases by about 5 % when the voltage drops by 0.5 V. The speed loop and the hover thrust learning make up for the shortage of voltage dependence in the conversion (inference, strength: medium to strong). With the conversion of map B, it would decrease by about 2.5 % within one flight (opposite direction, about half the size) and be almost 0 across flights.
- The width of the per-run medians (0.95 to 1.21) includes the difference of firmware generations (chapter 9). One log after 2026-08-22 was 1.00.
- This table uses the same segments and the same maps for the four methods, and restates the comparison of hover Duty slopes above in terms of commanded thrust. It is not new independent evidence.

## 7. The current firmware's thrust-to-duty conversion

### Structure (fact, firmware source)

```
 target ─→ [altitude, attitude, rate control] ─→ [mixer B⁻¹] ─→ [thrustToDuty] ─→ motor
              output: total thrust [N], torque [N·m]   thrust per motor [N]      ↑
                                                                 logged voltage (INA3221)
```

| Item | Content |
|---|---|
| Control output | Total thrust `thrust` [N] and torque `torque[3]` [N·m] of `ControlOutput` |
| Mixer | Computes the thrust of each motor backwards from the total thrust and the torque (arm length 23 mm, $\kappa = C_Q/C_T = 4.10\times10^{-3}$) |
| How total thrust is made | ACRO, STABILIZE: throttle × maximum thrust. ALT_HOLD, POS_HOLD: hover thrust mg × `hover.thrust_corr` + correction from the vertical speed loop (up to ±0.15 N) |
| Conversion (`thrustToDuty` in `actuator.cpp`) | $\omega = \sqrt{T/C_T}$ ($C_T = 1.00\times10^{-8}$, provisional), $V = A_m\omega^2 + B_m\omega + C_m$ ($A_m = 6.0368\times10^{-8}$, $B_m = 6.699\times10^{-4}$, $C_m = 0.0153$), Duty = $V$ ÷ logged voltage |
| Hover thrust learning | At steady hover, moves the speed loop output into the hover thrust with a time constant of 20 s, and saves it to `hover.thrust_corr` at the moment of landing |

### Change of thrust against a change of Duty (fact, calculated with the SILS plant)

At hover thrust (0.0907 N per motor), the rate of change of thrust for a small change of Duty (local slope $dT/dD$) was compared between the SILS plant and the current firmware curve (each evaluated at its own hover Duty). The ratio of the thrust itself ($T/D$) is almost 1 (0.98 to 1.07); the slope is what differs.

| Battery voltage | 3.60 V | 3.65 V | 3.70 V | 3.80 V | 3.90 V | 4.00 V | 4.10 V | 4.20 V |
|---|---|---|---|---|---|---|---|---|
| Slope ratio (plant ÷ firmware) | 0.672 | 0.679 | 0.443 | 0.528 | 0.607 | 0.662 | 0.709 | 0.755 |

- The jump between 3.65 V and 3.70 V is where the hover Duty of the plant crosses 0.70 (3.685 V), and comes from the kink in the SILS $R_\text{out}$ table. When the review replaced the table with a smooth fit and recalculated, the jump disappeared and the ratio was about 0.55 in this interval.
- Inference (strength: medium): The current firmware actually produces only about 0.45 to 0.75 times the commanded change of torque and thrust (assuming the SILS plant represents the real vehicle). The current gains were tuned with this too-small output included. Making the conversion exact raises the loop gain effect by 1.3 to 2.0 times (excluding the factor 2.3 caused by the kink in the table). This ratio has not been checked in flight.
- Inference (not checked): The learned value includes the effect of the battery voltage at the time it was saved. If a value saved at landing with a run-down battery is used in the next flight with a full battery, the initial hover thrust may be several percent too high.

## 8. SILS check (2026-11-03)

| Name | Content |
|---|---|
| As is | Reference for the comparison |
| F1 | `thrustToDuty` replaced by the exact steady-state inverse of the SILS plant ($\omega = \sqrt{T/(\eta C_T)}$, the needed current, the needed $R_\text{out}$, inverse lookup of the table). The error of the inverse is 1.7×10⁻⁶ in Duty (single precision). Constants and table are the same as the plant |
| F1g | In addition to F1, the proportional gain and torque limit of the rate loop, and the gain and correction limit of the altitude speed loop, are multiplied by 0.593 (mean of the slope ratios for 3.7 to 4.1 V; includes the 0.443 at the kink) |

190 s hover from full charge (battery 4.19 → 3.37 V, target altitude 0.5 m):

| Item | As is | F1 | F1g |
|---|---|---|---|
| Maximum altitude after takeoff | 0.810 m | 0.528 m | 0.596 m |
| Vertical motion (max − min per 4 s, median / max) | 2.7 / 8.6 cm | 1.4 / 3.8 cm | 1.8 / 4.9 cm |
| Median vertical motion (first third / last third) | 3.0 / 2.6 cm | 0.9 / 1.5 cm | 1.5 / 2.1 cm |
| Slope of commanded thrust against voltage | −0.175 times weight /V | −0.002 times /V | −0.003 times /V |
| Learned value `hover.thrust_corr` | 0.96 to 1.03 | 1.00 to 1.003 | 1.00 to 1.003 |
| Mean altitude during hover | 0.487 m | 0.451 m | 0.439 m |
| Time until altitude stays within target ±5 cm | 20.6 s | does not settle | does not settle |

Regression tests (45 scenarios): as is, 41 pass, 3 known failures, 1 out of scope. F1 has 11 failures and F1g has 10. The model-match check passes for all three.

| Class | Scenario | Basis |
|---|---|---|
| Calibration offset (F1, F1g) | acro_flight, alt_flight, modeswitch, pos_pitch, pos_roll, stab_ground_windup | Multiplying the open-loop throttle input by 1.0625 passes against the original reference |
| Real change (F1) | alt_auto_takeoff, pos_auto_takeoff | The takeoff overshoot disappears and falls below the lower limit set to match the real vehicle's overshoot (vertical motion 0.05 m or more) |
| Real change (F1, F1g) | stab_flight | Attitude tracking error 0.061 to 0.065 (limit 0.052, as is 0.047). Shifting the input still fails |
| Real change (F1g) | flip_abort_recover | With the reduced gain, the force to bring the altitude back after a flip is not enough |
| Cause not identified (F1, F1g) | calib_side_lying, crash_refly | Shifting the input still fails |
| Known failure (all three) | commloss_land_level, pos_yaw, yaw_hold | The failing metric changed |

(F1's 11 = calibration 6 + takeoff 2 + stab_flight 1 + unidentified 2; F1g's 10 = calibration 6 + stab_flight 1 + flip_abort_recover 1 + unidentified 2.)

- Fact: With the battery model switched off in F1, the median vertical motion is 0.23 cm (as is 0.26 cm).
- Limit of this SILS: F1 makes the firmware conversion the inverse of the plant, so it is true by construction that the voltage-dependent offset disappears inside the SILS. What was learned is that the hover altitude offset and the takeoff overshoot in the SILS came from the mismatch between the conversion and the voltage dependence of the plant. The error that remains on the real vehicle (the opposite-direction 2.5 % in chapter 6) cannot be checked in the SILS.
- Inference: Even with F1, where the effect is larger, no hover oscillation appeared (attitude rms about 1e-10°). The worsening of stab_flight is not fixed by F1g either, so we think it has a cause other than the larger effect. Shrinking all gains uniformly (F1g) weakened the recovery after a flip.
- Not identified: In F1 and F1g the hover altitude is 4 to 6 cm below the target (the estimated altitude is also low). The altitude control has an integrator and the learned value is 1.00, so a steady offset is unnatural, and the cause must be investigated. The 20.6 s of "as is" differs from the record of 11/01 (about 8 s), and the reason is not identified.
- The takeoff overshoot of the real vehicle (about 0.8 m on 2026-06-27, in the 2026-11-01 record of the SILS policy) may also be caused by the same conversion offset (inference, not checked).

## 9. Notes on measurement and cross-checking (hand-over)

| Item | What happened | What to do from now on |
|---|---|---|
| Minus sign on the scale | The sign was not read, and negative values were read as positive | Judge the sign with `read_scale.py`. Set the sign position again when the camera view changes |
| Weight | The first version said the vehicle was not weighed; the developer had weighed it, 36 to 38 g (confirmed 2026-11-03) | Keep the value for each vehicle in the record |
| Room temperature | Not recorded on 10/15 or 11/02 | Record before and after a run |
| Known weight | Not checked on 10/15 | Check before and after every measurement |
| Audio recording | Not recorded in any run on 11/02 | Set `SF_MIC_NAME` and add `--audio` (the rotor speeds of the four motors can be separated by sound) |
| Reference of $V_{DD}$ | Logged voltage − $V_{DD}$ differs: −11 to −21 mV + 38 to 46 mV/A for one motor in the morning, +17 to +23 mV + 13 to 27 mV/A in the afternoon (four loads, with propeller), and +9 to +21 mV + 55 to 96 mV/A at night (R15) | Keep the oscilloscope settings and the probe position as photos. Correct the afternoon and night $V_{DD}$ with the logged voltage again and check $V_b$ |
| Objects below the propeller | The probe and wiring may have lowered the thrust | Compare the case with them removed and with them in place at the same Duty |
| Duty range | The long discharge covers only 0.50 to 0.80 (0.9 only in two steps at night) | Add 0.85 to 0.95 |
| Rotor speed of four motors | Only M1 was measured | Use audio, or put the oscilloscope on the drains of M2 to M4 |
| Firmware generation of flight logs | 13 logs predate the conversion change of 2026-08-22 (motor curve 1.12 times, default of `hover.thrust_corr` 1.12 → 1.00). `hover.thrust_corr` became a parameter on 6/18, hover thrust learning was added on 7/04, and the altitude disturbance observer became default on 7/18. Results that use Duty are not affected, but the level of the logged commanded thrust differs by generation | Record the firmware commit for each flight log |
| Width of the check | The "±3 %" is a width set after seeing the result | Do not fix the pass width before the experiment; decide it after seeing the data. A width fixed in advance works only for an expected result and may not handle an unexpected one (experimenter, 2026-11-03) |

## 10. Hand-over to the model redesign (opinion)

### Candidate elements for the model

| Element | Content | Range of use and basis |
|---|---|---|
| Drive circuit | $\bar I_m = (V_\text{on} - K_e\omega - V_b)/R_\text{out}(D)$. Make $R_\text{out}$ a smooth table from the measurement in steps of 0.02 | D 0.50 to 0.80, $V_{DD}$ 3.63 to 4.27 V (chapter 4). $V_b$ is kept as a term in the formula, starting at 0 (decision 3) |
| Power supply | Open-circuit voltage and internal resistance of the battery. The drop of the on-time voltage due to the pulse current of four motors together, $\Delta V = R_\text{path} I + R_\text{src} I (1-D)/D$ (decision 1) | Chapter 4. It can be determined from the 11/02 long-discharge data, but has not been analyzed yet (backlog #19 in chapter 8 of the SILS policy `docs/architecture/simulation-policy.md`) |
| Thrust | $F = F_\text{ref}(\omega/\omega_\text{ref})^n$, the 10/15 values, 4 times for the whole vehicle | $\omega$ 1200 to 4700 rad/s (chapters 5 and 6) |
| Torque | $C_Q$, and the no-load loss including a constant friction torque | Chapter 5 |
| Temperature | $K_e$ falls when warm, but the effect on rotor speed at the same Duty and voltage is within ±0.6 % | Within the range checked at D 0.50 to 0.80, it need not be included |
| Unit-to-unit variation | Thrust differs by about ±10 % among vehicles of the same design | Chapter 6. Build the model as an average vehicle and absorb the variation by learning |

### Candidate constants (with method and conditions)

| Quantity | Value | Method and conditions |
|---|---|---|
| $K_e$ | $5.90\times10^{-4}$ / $5.76\times10^{-4}$ / $5.54\times10^{-4}$ V·s/rad | 10/15 spin-down / 11/02 spin-down without propeller / inside the steps of the 11/02 night. Whichever is used, the reproduction of rotor speed at the same Duty and voltage is within ±1.2 % |
| Winding resistance $R$ | 0.594 Ω | 10/15 |
| $R_\text{out}(D)$ | Table in chapter 4 (two versions) | D 0.50 to 0.80. Outside it, the SILS table (10/15, steps of 0.1) |
| $V_b$ | 0 at first. Candidate 0.11 V | Comparison of four loads (inference). 0.06 to 0.18 V between laps. Set the value after checking the $V_{DD}$ reference |
| $I_0$ | $0.044 + 2.9\times10^{-9}\,\omega^2$ A | 10/15. Needs to be rewritten in the form of a constant friction torque |
| Thrust | $F_\text{ref} = 0.0916$ N, $\omega_\text{ref} = 3500$ rad/s, $n$ = 2.13 to 2.15 | 10/15, $\omega$ 1200 to 4700 rad/s |
| $C_Q$ | $3.38\times10^{-11}$ | 10/15 (3.33e-11 on the 11/02 night) |
| $J$ | $1.54\times10^{-8}$ kg·m² | 10/15 (1.47 to 1.53e-8 on the 11/02 night) |
| Logged voltage → on-time $V_{DD}$ | $\Delta V = R_\text{path} I + R_\text{src} I (1-D)/D$; for four motors $R_\text{path}$ 2 mΩ, $R_\text{src} = 37 + 95\,(4.2 - V_\text{CLI})$ mΩ | One battery only (battery 3). One motor (battery 2): 26 mΩ and 72 mΩ. The fitted formula $781 - 118\,V_\text{CLI} - 324\,D$ mV on the same four-motor data has a residual of 12 mV, but cannot be used for one motor or another battery |
| Weight | 36 to 38 g | Weighed by the developer (the checks were calculated for 36 g and 37 g) |

### Already decided (experimenter's decision)

Use the 10/15 thrust as the reference, and 4 times it for the whole vehicle. Do not use the 11/02 four-motor thrust. The weight is 36 to 38 g (weighed by the developer).

### Decided (2026-11-03, experimenter's decision)

| Item | Decision | Content |
|---|---|---|
| 1. Voltage the firmware uses for the conversion | Estimate the drop with the physical form | $V_\text{on} = V_\text{CLI} - \Delta V$, $\Delta V = R_\text{path} I + R_\text{src} I (1-D)/D$ ($I$ is the INA3221 battery current). The 11/02 values (chapter 4) are the initial constants; the same formula goes into the firmware and the SILS battery model. Take one or two more four-motor bench runs with other batteries to see the difference between batteries |
| 2. Form of the thrust formula | Power law | $F = F_\text{ref}(\omega/\omega_\text{ref})^n$, $\omega_\text{ref}$ = 3500 rad/s, $F_\text{ref}$ = 0.0916 N, $n$ = 2.13 (10/15, fitted on the thrust residual). The SILS correction factor $\eta$ is not used. The reaction torque keeps the form $C_Q\omega^2$ plus friction loss, to be reviewed when the friction is refitted |
| 3. $V_b$ | Keep the term in the formula, starting at 0 | Set the value after the check, and remake the $R_\text{out}$ table as a pair from the same data. The table is remade with $V_\text{on}$ in either case. In the hover range, including it or not changes the thrust by less than 1 % (author's rough estimate, not checked) |

- Shifting the phases of the four PWMs to reduce the drop will be tried later (it needs a switch in the firmware and a bench test).
- The processing time of the power law (one `powf` per motor) is measured on the real vehicle at the start of the branch (expected: a dozen µs or so for four calls, below 1 % of the 2.5 ms control period; not checked).

### Measurements still needed

The range of Duty 0.85 to 0.95, rotor speed per motor (audio or each drain), the comparison with and without objects below the propeller, room temperature and a known weight every time, the weight of the vehicle, the test that shifts the phases of the four PWMs (the plan in chapter 8 of the analysis note `analysis/notes/ct_cq_bench_20261015.md`), the check of the afternoon and night $V_{DD}$ reference, the analysis of the battery's open-circuit voltage and internal resistance, four-motor runs with other batteries (one or two), and a direct measurement of $V_b$ with the shaft held (no propeller, $\omega$ = 0, low Duty; the intercept of the voltage-current line).

### Effect on control

- Making the conversion exact raises the loop gain effect to 1.3 to 2.0 times the current one (chapter 7). Do not shrink the gains uniformly; redesign them with the new model (the uniform shrinking of F1g weakened the recovery after a flip).
- Items to review together with the conversion: the Duty saturation and margin (full thrust, low voltage), the rise that differs between increase and decrease (35 to 40 ms and 50 to 55 ms), the yaw torque limit `rate.yaw.max_torque` and the mixer's $\kappa$, the altitude disturbance observer, and the relation between the hover thrust learning time constant (20 s) and the new effect.
- Scenarios that use open-loop throttle (about 18 by the worker's count) need to be recalibrated. Scenario references matched to the old model's behavior, such as the lower limit set to match the takeoff overshoot, also need review.

### Safety when putting it on the real vehicle

- After flashing the firmware with the changed conversion, clear the saved values (such as the learned `hover.thrust_corr`) with `param reset` → `param save` (as in the change of 2026-08-22). Flying with an old learned value shifts the thrust by 10 % or more.
- At first, remove the propellers or tether the vehicle with a string, and check how Duty comes out and how the motors speed up and down. Lift off with a low Duty limit.
- Take the learned hover thrust again for each unit. Check the margin at low voltage (3.5 V and below).
- Prepare the way back (the previous firmware and parameters) before flying.

### Proposed course of the branch

At each stage, what to check before going on is written in parentheses.

1. Put the constants and formulas of the new model into the model's reference file (`stampfly_physical.yaml`). Along with it, correct the afternoon and night $V_{DD}$ with the logged voltage to set the value of $V_b$, remake the $R_\text{out}$ table with $V_\text{on}$, and measure the processing time of the whole conversion on the real vehicle (`sf params check` passes).
2. Replace the SILS plant with the new model. Bring Genesis and VPython to the same reference (backlog #18) (reproduce the bench table per step in the SILS).
3. Make the firmware's thrust → Duty conversion the inverse of the same model (check the error of the inverse).
4. Redesign the attitude and altitude control with the new model (check the stability margin, Duty saturation, flip, and takeoff in the SILS).
5. Review the calibration and references of the scenarios, and pass the regression tests and the model-match check.
6. Check on the bench and the real vehicle (the safety steps above. Voltage dependence of hover, takeoff overshoot, oscillation).

## 11. Data and scripts

| Content | Output or script |
|---|---|
| Battery voltage dependence of $R_\text{out}$, table of cold laps | Bench repository `tools/analyze_long.py` (electrical summary of 11/02), `data/2026-11-02_steps.csv` |
| Four loads and $V_b$ | Chapter 10 of the bench procedure (analysis of 11/02) |
| Table per step (batteries 2 and 3, D 0.9 of R15) | Bench repository `data/2026-11-02_steps.csv` (commit 2d5f782) |
| On-time $V_{DD}$, comparison of one and four motors | Bench repository `tools/vdd_on_time.py` |
| Thrust (batteries 2 and 3, minus sign) | Bench repository `tools/analyze_thrust.py`, `tools/read_scale.py`, `tools/scale_calib_2026-11-02_bat3.json` |
| Redo of 10/15 ($C_T$, $C_Q$, $K_e$, $J$) | Bench repository `tools/analyze_constants_long.py` |
| Hover balance, comparison of hover Duty | This repository `analysis/scripts/ct_cq_bench_20261015/crosscheck/logcheck/test4.py` (`results/test4.json`, `test4_windows.csv`) |
| Check of the conversion | Same `test5.py` (`results/test5.json`) |
| Selection of hover segments | Same `hover.py` |
| SILS F1 and F1g | Same `analysis/scripts/ct_cq_bench_20261015/patches/F1.patch` and `F1g.patch`, `results/base_1103/`, `F1/`, `F1g/`, `hover_180s.scn`, `hover_metrics.py` (commit 3601ac28 of this repository) |
| History of the analysis | This repository `analysis/notes/ct_cq_bench_20261015.md` chapter 6 |

## 12. Review of this report (2026-11-03)

After the text was written, four other workers (AIs) were made to do the following reviews, and their findings were taken into the text.

| Layer | Content | Main result |
|---|---|---|
| Falsification test | For each claim, decide "what it would look like if false" and recompute from the raw data | The voltage dependence of $R_\text{out}$, the power law of thrust and the ratio of the two days, the slope of hover Duty, and the jump in the slope ratio and the kink in the table were reproduced. $n$ is 2.12 to 2.13 with thrust residuals. The agreement with the audio is ±0.6 % in 7 steps, and D 0.5 scatters up to ±1.6 %. Four motors together cannot be checked outside the voltage range of one motor |
| Code audit | Choices, extrapolation, hard-coded values, and circularity in the calculation scripts | Hard-coded weight, extrapolation of the table, underestimated slope errors, and the fact that the hover segment conditions are not left in the output. The constants and table of F1 match the plant |
| Reader without context | Read only the document and criticize it | Definitions of run names, maps, and symbols, strength of claims, how labels are attached, list of constants, range of use |
| Field checklist | Identification, statistics, configuration management, control design, safety | Firmware generation of the flight logs, identity of the vehicle, the fact that the upper limit of the gain-effect ratio comes from the kink in the table, the safety section, and places where "independent confirmation" was overstated |

Except for what is written as "independent confirmation" (visual reading of photos, audio, error of the inverse), the checks are recalculations by the same kind of AI as the calculator, not confirmation by a human.
