# SILS プラントの順モデル照合（記録 duty → 角加速度）(2026-11-01)

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

**状態**: 実施済み（バックログ #21 の第一段）。判定基準（合否の閾値）は決めていない。作成 2026-11-01、最終更新 2026-11-01。実機ログ（`flip_test1/2`）はリポジトリに含めない。

## 1. 目的と結論

### 問い

実機フライトログに記録された 4 モータの duty から、SILS（ファームのソースをそのまま PC 上で動かすシミュレーション）のプラントと同じ計算で機体の角加速度を予測し（順モデル）、実機の角加速度と比べることで、プラントの推進系（$R_\text{out}$ モデル）の良し悪しを判定できるか。本書は、(A) 予測モジュールが SILS プラントを正確に再現すること、(B) ホバーで低周波側の感度が落ちる原因の仮説、(C) 実機ログへの適用結果、をまとめる。

### 結論

| 項目 | 結論 |
|---|---|
| A. モジュールの再現精度 | SILS 自身の真値（`truth.csv` の角速度の差分）に対し、空中の行で残差 **0.00〜0.02 %**（ロール・ピッチ・ヨー）。大振幅の宙返りでも 0.02 %。8 bit の PWM 量子化（モータ駆動の duty を 255 段階に切り捨てる処理）を入れないと 1.6〜4.9 % に悪化するので、この項は実際に効いている。ログ時刻との 1 substep（0.25 ms）のずれは原因を追っていない（未確認） |
| B. 低周波で \|H\| が小さい理由 | 仮説 H1（外乱トルクを制御が打ち消す）を支持する観測が出た。外乱なしのホバーでは \|H\| = 1.000（0.5〜40 Hz の全帯域、コヒーレンス 1.00）で、H2（モデル自体の低周波ゲイン誤差）は SILS では起きない。外乱トルクを加えると外乱のある帯域だけ \|H\| が 0.09〜0.6 に落ちる。ただし実機にそのような外乱トルクが実際にあることは、この実験では確認できない（未確認） |
| C. 実機宙返り | 合わせ込みなしの残差は $R_\text{out}$ 0.29〜0.31（旧 D×V モデル 0.97〜1.04）。最良スケール 0.79〜0.81、追加遅れ 3〜4 ms（旧は 0.54〜0.57、14〜17 ms）。以前の Python 照合の値と一致 |
| C. 実機ホバー | 5〜10 Hz の \|H\| は $R_\text{out}$ 0.78〜0.84（旧 0.36〜0.38）、遅れ 4 ms（旧 14 ms）。0.5〜5 Hz は両モデルとも 0.06〜0.45 で、モデルの判別には使えない。10 Hz 以上は、同一時刻の IMU 行の扱いで \|H\| が 0.5〜1.6 まで動き、現状では使えない |
| 判定基準 | 決めていない。候補の指標と、$R_\text{out}$ と旧モデルが分かれる箇所を 6 章に示す |

## 2. SILS プラントの項目と、モジュールの対応

プラント（`simulator/sils/plant/plant.cpp`）が各モータ・機体に適用する項を、行番号つきで列挙した。モジュール `tools/sysid/forward_model.py` はこれと同じ式を使い、定数は `tools/sysid/_generated_params.py`（`control/models/stampfly_physical.yaml` から自動生成）だけから取る。

| 項 | プラントでの位置 | モジュール |
|---|---|---|
| duty の経路 | 輸送遅れ（既定 OFF）`plant.cpp:577-585`、スルー制限（既定 OFF）`:586-596`、[0,1] クランプ `:608` | 遅れとスルー制限は未実装（既定 OFF のため）。クランプは実装 |
| PWM 量子化 | ファーム `motor_driver.cpp:162-163`（255 倍して切り捨て）、SILS ボード `virtual_board.cpp:501-510`（255 で割り戻し）。分解能は `motor_hw.hpp:53` の 8 bit | `quantise_duty()`（既定 8 bit、float32 で切り捨て） |
| モータ ODE（常微分方程式） | $I_m=(V-K_e\omega)/R_\text{out}(D)$（$D<0.01$ で 0）`:384-391`、$J\dot\omega=K_tI_m-C_q\omega^2-K_t(I_{0a}+I_{0b}\omega^2)$ `:393-399`、$R_\text{out}$ は 101 点の表の線形補間 `:365-382` | `rout_ohm()`・`motor_current()`・`omega_dot()` |
| 積分 | 1 substep = 0.25 ms（`stampfly.xml:29`）、古典 RK4（4 次ルンゲ・クッタ）`:613-617`、$\omega\ge0$ に制限 `:618`。400 Hz の 1 周期に 10 substep（`:516-531`） | `simulate_motors()`（同じ手順） |
| 供給電圧 | 電池モデル $V=\text{OCV(SoC)}-I_\text{total}R_\text{int}$、substep ごとにクーロンカウントで更新 `:457-468`、`:564`、`:713`。状態は float32 | `voltage_mode="battery"`（既定、float32 も再現）／`"log"`（ログの 1 Hz 値）／数値（定電圧） |
| 推力 | $T_i=\eta C_t\omega_i^2\cdot\text{health}_i\cdot\text{ge\_mult}$ `:622`（$\eta$=0.976）。health=1、地面効果 OFF（`ge_gain=0`）を仮定 | `body_torque()` |
| ロール・ピッチのトルク | MuJoCo が各推力をロータ位置（$\pm0.023,\pm0.023,0.005$ m、`stampfly.xml:131-134`、`gear` は `:141-144`）の +Z 方向に加え、$r\times F$ が腕のモーメントになる。`torque_authority`=1 は素通し `:701-703` | $\tau_x=\sum y_iT_i$、$\tau_y=-\sum x_iT_i$（FLU→FRD で符号反転） |
| ヨーのトルク | 各モータ $(C_q\omega^2+J\dot\omega)$、CCW（FR・RL）は −Z、CW（RR・FL）は +Z `:645-647`。$\dot\omega$ は 1 substep の実現値 | 同じ（ロータ慣性の反作用項 $J\dot\omega$ を含む） |
| 剛体 | 慣性 diag(9.16e-6, 13.3e-6, 20.4e-6) kg·m²（`stampfly.xml:86`）。ジャイロ項 $\omega\times I\omega$ は MuJoCo が積分 | `angular_accel()`（ジャイロ項は記録角速度を渡したときだけ含める） |
| 乱流トルク・風 | 乱流トルク `:727-735`（既定 0）、風は重心への力のみでトルクなし | 未実装（既定シナリオで 0） |
| モータ順 | 添字 0..3 = M1 FR, M2 RR, M3 RL, M4 FL（ログ列 `duty_FR/RR/RL/FL` と同じ） | 並べ替えなし |

実機ログに適用するときの違い: プラントの duty は SILS ではログ値の量子化後、実機ではファームが同じ切り捨てを行うので、量子化はどちらにも適用する。一方、4 章の 1 substep の整列ずれ（`SILS_LOG_ALIGNMENT_DELAY_S`）は SILS 専用で、実機には使わない。

## 3. A: モジュールが SILS プラントを再現する

### 方法

SILS で `sysid_gate.scn`（POS_HOLD ホバー中にロール・ピッチ・ヨーのチャープ、44 s）と `api_flip_roll.scn`（ロール宙返り、39 s、`--param flip.enable=1`）を実行し、バンドルの `motor.csv`（duty）から予測した $\alpha_\text{pred}$ を、(a) `truth.csv` の機体角速度の差分、(b) ログされた `gyro` の差分、と比べた。残差は $\text{rms}(\alpha_\text{ref}-\alpha_\text{pred})/\text{rms}(\alpha_\text{ref})$、空中（高度 0.05 m 超）の行のみ。接地の瞬間は接触力がモジュールに無いので除外した（含めると接地の 1 点で残差が 19 % になる）。「最も動的な窓」は 0.5 s の窓で $\alpha_\text{ref}$ の二乗平均が最大のもの。`alt_flight.scn` は 4 つのモータが常に対称で $\alpha\approx0$（約 1e-14 rad/s²）になるため、この検査には使えない。

### 結果（予測を 1 substep 遅らせた場合）

| シナリオ・軸 | 対 truth 全体 | 対 truth 最も動的な窓 | 最良の追加遅れ・スケール | 対 gyro 差分 全体 | 対 gyro 差分 窓 |
|---|---|---|---|---|---|
| sysid_gate ロール | 0.00 % | 0.00 % | 0 ms・1.0000 | 3.32 % | 0.77 % |
| sysid_gate ピッチ | 0.01 % | 0.00 % | 0 ms・1.0000 | 4.97 % | 1.91 % |
| sysid_gate ヨー | 0.00 % | 0.00 % | 0 ms・1.0000 | 4.43 % | 1.76 % |
| api_flip_roll ロール（$\alpha$ の rms 28.9 rad/s²） | 0.02 % | 0.00 % | 0 ms・1.0000 | 0.36 % | 0.12 % |
| api_flip_roll ピッチ・ヨー | 励振なし（\|α\| < 1e-13 rad/s²）のため判定不能 | | | | |

### 読み方

- 対 truth の残差は 0.02 % 以下で、モジュールは SILS プラントを実質正確に再現している。追加遅れ 0〜40 ms の探索でも最良は 0 ms、スケールは 1.0000（ヨー 0.9971 は窓の残差が 0.00 % なので誤差内）。
- 対 gyro 差分の残差 3〜5 % は、センサ経路（gyro の量子化など）による。原因は未確認だが、追加遅れ・スケールは 0 ms・1.0000 で、モジュールの欠陥ではない。
- **1 substep のずれ**: 整列ずれなしでは残差は sysid_gate で 1.26 / 1.46 / 2.38 %（ロール／ピッチ／ヨー）、宙返りで 0.76 %。予測を 0.25 ms（1 substep）遅らせると上表の値になる。ログの時刻刻みと SILS の物理刻みの順序によると思われるが、原因は追っていない（未確認）。実機ログには適用しない。
- **反証の確認**: PWM 量子化を外すと整列ずれなし・ジャイロ項なしの条件で 4.85 / 3.58 / 3.45 %（sysid_gate）、1.58 %（宙返り）となり、同条件の量子化ありの 1.26 / 1.49 / 2.38 %、0.76 % より悪い。ログの 1 Hz 電圧を保持して使う方式は 7.3 %（宙返り）で、電池モデルの再計算が優る。
- 再確認試験（変更で既存の動作が壊れていないかを自動で確かめる試験）として `tools/sysid/test_forward_model.py` を追加した（7 件、約 22 s。SILS 未ビルドなら SILS を使う 3 件は skip）。許容値は実測の約 10 倍（残差 3 %、スケール ±1 %）。

## 4. B: ホバーで低周波の \|H\| が小さい理由

### 仮説と、事前に書いた予測

実機ホバーでは、$H=S_{\text{pred,meas}}/S_{\text{pred,pred}}$（Welch 法、Hann 窓、nperseg=256、400 Hz。$S_{xy}$ は $x$ から $y$ へのクロススペクトル）が 5〜10 Hz で 0.7〜0.8 なのに 0.5〜5 Hz で 0.08〜0.41 だった。

| 仮説 | 内容 | 計算前に書いた予測 |
|---|---|---|
| H1（外乱の打ち消し） | duty には、モデルにない外乱トルク（重心ずれ、モータ間のばらつき、空気）を制御が打ち消す成分が含まれる。外乱のないモデルに通すと、実際には打ち消されて存在しないトルクを予測する | 外乱なしの SILS ホバーでは \|H\| ≈ 1（0.5〜40 Hz 全域）。外乱を加えると、外乱のある帯域で \|H\| < 1 |
| H2（低周波のゲイン誤差） | モデルのゲインが低周波で誤っている | 外乱なしでも \|H\| < 1 |

### 実験

`make_hover_scn.py` で、POS_HOLD ホバー（`sysid_gate.scn` と同じ離陸と上昇、t = 18〜74 s がホバー）に 0.23〜1.9 Hz の水平の突風力（重心への力、振幅 0.01 N × 5 成分）を加えた 2 条件を作り、SILS に流した。

| 条件 | 内容 |
|---|---|
| N（外乱トルクなし） | 突風力だけ。風は重心に働く力でトルクを生まないので、実際のトルクはすべてモータが出す |
| D（低周波の外乱トルク） | N に加え、モータ 0・1・3 の推力健全度を 0.7・1.3・0.4 Hz で最大 3 % 変調（モータ間のばらつきの時間変化）。モジュールは health=1 を仮定するので、この外乱トルクを知らない |
| N + 乱流 | N に `--turbulence 0.03`（3.7〜5.9 Hz の外乱トルク + 1〜3 Hz の水平力） |

### 観測（t = 20〜74 s、\|H\| のバンド合計）

| 条件・軸 | 0.5〜2 Hz | 2〜5 Hz | 5〜10 Hz | 10〜20 Hz | 20〜40 Hz |
|---|---|---|---|---|---|
| N ロール | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| N ピッチ | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| D ロール | **0.09**（コヒーレンス 0.14） | 0.59 | 1.08 | 1.51 | 2.79 |
| D ピッチ | **0.29** | 0.57 | 0.96 | 1.08 | 2.52 |
| D ヨー | 0.25 | 0.28 | 0.64 | 0.58 | 0.77 |
| N + 乱流 ロール | 0.28 | 0.35 | 0.57 | 0.99 | 1.00 |
| N + 乱流 ピッチ | 0.33 | 0.38 | 0.50 | 0.99 | 1.00 |

N のコヒーレンスは全帯域・全軸で 1.00。

### 判定

- **観測されたのは H1 の予測**。N では \|H\| = 1.000 が全帯域で成立し、H2（モデル自体の低周波ゲイン誤差）は SILS では起きない（モジュールがプラントと同一なので当然だが、予測が外れなかったことの確認になる）。D と乱流では外乱の周波数帯域だけ \|H\| が落ち、外乱のない帯域（乱流の 10 Hz 以上）では 1 に戻る。D の \|H\| = 0.09〜0.29（0.5〜2 Hz）は、実機の 0.08〜0.41 と同程度の落ち方である。
- 制御が外乱を打ち消すと、モータが出すトルクは $-d$ に近づき、観測される角加速度には現れない。予測は duty から $-d$ 相当を作るので $\alpha_\text{pred}$ が過大になり、$H=1+S_{\text{pred},d}/S_{\text{pred,pred}}\to0$ になる。
- D の 10 Hz 以上の \|H\| > 1 は、健全度の階段状の更新（100 ms ごと）が広帯域のトルクを足すため。原因の詳細は未確認。
- **この実験でできないこと**: 実機に同じ大きさの外乱トルクが実際に存在するかは分からない（未確認）。SILS では「外乱があれば実機と同様の \|H\| の落ち方が出る」ことまでしか示せない。実機でもう一つ見るべきは、制御トルクが支配する宙返りで、低周波に偏らず一定のスケール（0.79〜0.81）が出ている点で、これは H1 と矛盾しない。ただし 0.8 という欠損自体は、本書では説明できていない。

## 5. C: 実機ログへの適用

### 方法と前提

- 対象: `flip_test1`・`flip_test2`（機体は飛行中。地上拘束の 2 本は使わない）。実機ログは IMU の時刻が±0.5 ms ゆらぎ、前の IMU サンプルを使い回す行（同じ時刻・同じ gyro・新しい duty）がある。2〜4 % の行（宙返りの窓を含む全体）、ホバー窓では 9〜16 %。扱い方を 3 通り用意した（`load_real_log()`）。

| 扱い | 内容 |
|---|---|
| timestamp | 同じ時刻の行を 1 点に畳む（最後の行）。畳んだ行の duty は捨てる（以前の `flipcheck.py` と同じ） |
| spread | 全行の duty を残し、同じ時刻の行を直前の停滞区間に等間隔で並べ直す。gyro は 1 点 |
| raw | 行を等間隔と見なし、使い回された古い gyro もそのまま使う（素朴な扱い） |

- 規定値は timestamp。電圧はステータスの 1 Hz 値を線形補間（INA3221）。予測と実測の比較は 400 Hz の一様な格子上で行う。
- 宙返りの窓（`flip_test1` 15.85〜16.25 s、`flip_test2` 6.66〜7.10 s）は、以前の照合と同じ平滑化（Savitzky–Golay 9 点・2 次）で $\alpha_\text{meas}$ と $\alpha_\text{pred}$ を求める。ホバー窓（`flip_test1` 5.5〜15.5 s、`flip_test2` 4.4〜6.4 s）は格子上の前進差分。$H$ は $|S_{\text{pm}}|/S_{\text{pp}}$、逆向きの括り $S_{\text{mm}}/|S_{\text{pm}}|$、位相、コヒーレンス $|S_\text{pm}|^2/(S_\text{pp}S_\text{mm})$、遅れ $-\angle S_\text{pm}/(2\pi f_c)$（$f_c$ は帯域内のパワー重心周波数）。

### 宙返りの窓（扱い: timestamp）

| ログ・窓 | モデル | 合わせ込みなし残差 | 最良の追加遅れ | 最良スケール | 最良時の残差 |
|---|---|---|---|---|---|
| flip_test1 ロール | $R_\text{out}$（ログ電圧、8 bit PWM） | 0.311 | 4 ms | 0.790 | 0.095 |
| flip_test1 ロール | 旧 D×V | 1.039 | 14 ms | 0.537 | 0.149 |
| flip_test2 ロール | $R_\text{out}$ | 0.285 | 3 ms | 0.807 | 0.136 |
| flip_test2 ロール | 旧 D×V | 0.967 | 17 ms | 0.568 | 0.216 |
| flip_test2 ピッチ（6.95〜7.45 s） | $R_\text{out}$ | 1.398 | 40 ms（探索の上限） | 0.335 | 0.883 |

- 以前の Python 照合（残差 0.29、旧 0.98〜1.02、スケール 0.79〜0.80、遅れ 3〜4 ms）を再現した。
- ピッチの窓は、残差が 1 を超え、遅れが探索の上限に張り付き、スケールが 0.3 にしかならない。宙返りの後の復帰制御が支配しており、この指標では判定できない。
- **扱い方の依存**: spread は timestamp と 1 % 以内で一致する。raw は `flip_test1` で残差 0.917（扱いで宙返りの窓の gyro 差分が壊れる）、`flip_test2` で 0.437 になり、同じ窓でも結果が大きく変わる。使い回された行を無視した扱いは使えない。
- **電圧の仮定への依存**（`flip_test1`／`flip_test2` の合わせ込みなし残差、最良スケール）

| 電圧の与え方 | flip_test1 | flip_test2 |
|---|---|---|
| ログの 1 Hz 値（窓内の平均 3.90 V） | 0.311、0.790 | 0.285、0.807 |
| プラントの電池モデル（ログ全体の平均 4.06 / 4.14 V） | 0.294、0.799 | 0.256、0.820 |
| 定電圧 3.7 V | 0.216、0.867 | 0.198、0.888 |

  電圧の置き方でスケールが 0.79〜0.89 動く。0.8 前後の欠損が電圧の仮定に左右されることは、判定基準を作る際に見ておく必要がある。PWM 量子化の有無（8 bit／なし）の影響は残差で 0.005 以下。

### ホバー窓（扱い: timestamp、ロール／ピッチ）

|\|H\|（$|S_\text{pm}|/S_\text{pp}$）、括弧は逆向きの括り $S_\text{mm}/|S_\text{pm}|$

| ログ・軸・モデル | 0.5〜2 Hz | 2〜5 Hz | 5〜10 Hz | 10〜20 Hz | 20〜40 Hz |
|---|---|---|---|---|---|
| flip_test1 ロール $R_\text{out}$ | 0.08 (0.15) | 0.25 (0.54) | **0.78 (0.87)** | 0.51 (1.04) | 0.41 (4.77) |
| flip_test1 ロール 旧 | 0.06 (0.10) | 0.19 (0.26) | 0.38 (0.40) | 0.39 (1.01) | 0.26 (2.99) |
| flip_test2 ロール $R_\text{out}$ | 0.13 (0.26) | 0.45 (0.62) | **0.84 (0.88)** | 1.18 (2.83) | 0.62 (36.9) |
| flip_test2 ロール 旧 | 0.08 (0.16) | 0.23 (0.28) | 0.36 (0.37) | 0.51 (0.94) | 0.32 (5.82) |
| flip_test1 ピッチ $R_\text{out}$ | 0.20 (0.48) | 0.25 (0.59) | 0.41 (0.69) | 0.59 (0.90) | 0.80 (1.68) |
| flip_test1 ピッチ 旧 | 0.13 (0.26) | 0.16 (0.30) | 0.24 (0.33) | 0.31 (0.45) | 0.40 (0.85) |
| flip_test2 ピッチ $R_\text{out}$ | 0.16 (0.27) | 0.34 (0.61) | 0.76 (0.94) | 0.85 (1.37) | 1.25 (2.90) |
| flip_test2 ピッチ 旧 | 0.09 (0.18) | 0.16 (0.30) | 0.30 (0.35) | 0.29 (0.48) | 0.36 (0.84) |

コヒーレンス（$R_\text{out}$ ロール）: flip_test1 0.54 / 0.46 / 0.89 / 0.49 / 0.09、flip_test2 0.50 / 0.73 / 0.95 / 0.42 / 0.02（旧は 5〜10 Hz で 0.93 / 0.97）。遅れ（$R_\text{out}$ ロール）は 5〜10 Hz で 4.4 / 4.2 ms（旧 14.4 / 14.4 ms、ピッチは $R_\text{out}$ 4.9 / 11.9 ms、旧 19.9 / 18.9 ms）。0.5〜2 Hz の位相は −46〜−52°、遅れに換算すると 80〜90 ms になるが、これは外乱成分の位相で、モータの遅れではない。ヨー軸の実機照合は行っていない（未確認）。

### 同一時刻の IMU 行の扱いによる 5〜20 Hz の変化（$R_\text{out}$、\|H\|／コヒーレンス）

| ログ・軸 | 扱い | 5〜10 Hz | 10〜20 Hz | 20〜40 Hz |
|---|---|---|---|---|
| flip_test1 ロール | timestamp | 0.78／0.89 | 0.51／0.49 | 0.41／0.09 |
| | spread | 0.77／0.88 | 0.51／0.39 | 0.34／0.04 |
| | raw | 0.84／0.96 | 0.95／0.66 | 1.60／0.22 |
| flip_test2 ロール | timestamp | 0.84／0.95 | 1.18／0.42 | 0.62／0.02 |
| | spread | 0.83／0.94 | 1.12／0.39 | 0.68／0.02 |
| | raw | 0.86／0.93 | 1.51／0.71 | 1.75／0.06 |
| flip_test1 ピッチ | timestamp／spread／raw | 0.41／0.46／0.59 | 0.59／0.62／0.89 | 0.80／0.88／1.55 |
| flip_test2 ピッチ | timestamp／spread／raw | 0.76／0.84／0.89 | 0.85／0.91／1.01 | 1.25／1.49／2.39 |

5〜10 Hz は扱いによる動きが 0.07 以内（ロール）で安定している。10 Hz 以上は扱いで \|H\| が 2 倍近く動き、コヒーレンスも低い（0.02〜0.7）。raw のコヒーレンスが高めに出るのは、古い gyro 行の階段状の変化が予測側の duty の更新と同じ周期で入るためと考えられるが、確認していない（未確認）。どの扱いが実際の時間関係を最もよく表すかは、制御周期ごとの実時刻がログに無いため決められない（未確認）。ログに制御の実時刻（またはタイマ値）を足せば判別できる。

## 6. 候補の指標と、$R_\text{out}$ と旧モデルが分かれる箇所

合否の閾値は決めない。以下は候補。

| 候補指標 | $R_\text{out}$ | 旧 D×V | 分離 | 備考 |
|---|---|---|---|---|
| 宙返りの合わせ込みなし残差（ロール） | 0.20〜0.31 | 0.97〜1.04 | 明確（約 3〜5 倍） | 電圧の置き方で 0.20〜0.31 と動く |
| 宙返りの最良スケール | 0.79〜0.89 | 0.54〜0.57 | 明確 | 1 から離れる（0.8 前後）理由は未解明。電圧の仮定に左右される |
| 宙返りの追加遅れ | 3〜4 ms | 14〜17 ms | 明確 | 時刻整列（ログのゆらぎ）の誤差は 1 ms 級 |
| ホバー 5〜10 Hz の \|H\|（ロール） | 0.78〜0.84 | 0.36〜0.38 | 明確 | コヒーレンスは両者とも 0.89〜0.97。ピッチは窓による差が大きい |
| ホバー 5〜10 Hz の遅れ | 4.2〜4.4 ms | 14.4 ms | 明確 | ロールのみ（ピッチは 5〜12 ms と窓でばらつく） |
| ホバー 0.5〜5 Hz の \|H\| | 0.08〜0.45 | 0.06〜0.23 | 判別に使えない | 外乱の打ち消しが支配（4 章）。SILS でも外乱があれば同様に落ちる |
| ホバー 10 Hz 以上の \|H\| | 0.4〜1.3 | 0.26〜0.51 | 判別に使えない | 同一時刻行の扱いで 2 倍動き、コヒーレンスが低い |

SILS 側へ同じ手順を適用して実機と並べる（同じ操作を SILS で飛ばし、同じ \|H\| と宙返り残差を出す）ことは、本書では行っていない（未確認）。

## 7. 未確認事項と制約

| 項目 | 状態 |
|---|---|
| ログ時刻と SILS 物理刻みの 1 substep（0.25 ms）のずれの原因 | 未確認（SILS 専用の経験的な補正） |
| 対 gyro 差分の 3〜5 % の原因（センサ経路） | 未確認 |
| 実機に H1 が想定する外乱トルクが実在し、その大きさが足りるか | 未確認 |
| 宙返りの最良スケール 0.8 の原因（推力欠損、電圧、他） | 未確認 |
| 同一時刻行のどの扱いが実時間関係に最も近いか | 未確認（制御の実時刻がログに無い） |
| 実機ヨー軸の照合 | 未確認（ホバー窓・宙返り窓とも未実施） |
| D 条件の 10 Hz 以上で \|H\| > 1 になる原因 | 未確認（健全度の階段状更新の影響と推測） |
| 地面効果・接触・モータ遅れ・スルー制限・乱流トルクの予測への影響 | モジュールに未実装。実機窓は空中のみ |
| GitHub への push | 実施できなかった（トークン無効、`gh auth login` が必要）。コミットはローカルのブランチ `feat/forward-model-check` にある |

## 8. 再現手順

| 手順 | コマンド |
|---|---|
| SILS のビルドと実行 | `sf sils build`、`sf sils scenario simulator/sils/scenarios/sysid_gate.scn --target vehicle --duration 44000000` |
| A の集計 | `python analysis/scripts/forward_model_check_20261101/selfcheck_sils.py <bundle.sflog.zip>` |
| B のシナリオ生成と H | `make_hover_scn.py N\|D <out.scn>`、`sf sils scenario <out.scn> --duration 83000000`（乱流は `--turbulence 0.03`）、`sils_hover_h.py <bundle>` |
| C | `SF_LOGS=<ログのディレクトリ> python analysis/scripts/forward_model_check_20261101/real_logs.py flip\|hover\|reuse` |
| 試験 | `python -m pytest tools/sysid/test_forward_model.py` |

いずれも `PYTHONPATH=<リポジトリ直下>:<リポジトリ直下>/lib` が必要。

---

<a id="english"></a>

# Forward-model check of the SILS plant (logged duty to angular acceleration) (2026-11-01)

**Status**: done (first step of backlog #21). No pass/fail thresholds are set. Created 2026-11-01, last updated 2026-11-01. The real flight logs (`flip_test1/2`) are not in the repository.

## 1. Purpose and conclusions

### Question

Can the SILS plant's propulsion model (the $R_\text{out}$ model) be judged by predicting body angular acceleration from the logged 4-motor duty with exactly the plant's computation (forward model) and comparing it with what the real vehicle did? This note covers (A) that the module reproduces the plant, (B) a hypothesis for the low-frequency drop of the sensitivity in hover, (C) the application to the real logs.

### Conclusions

| Item | Conclusion |
|---|---|
| A. Reproduction | Against the plant's own truth (derivative of `truth.csv` body rates), airborne rows: residual **0.00–0.02 %** (roll, pitch, yaw), also for the large-signal flip. Without the 8-bit PWM quantisation it degrades to 1.6–4.9 %, so that term does real work. A 1-substep (0.25 ms) alignment offset to the log time is not traced (unverified). |
| B. Low-frequency \|H\| | The observation matches H1 (the controller cancels unmodelled disturbance torque). Without a torque disturbance \|H\| = 1.000 in all bands 0.5–40 Hz (coherence 1.00), so H2 (model gain error at low frequency) does not occur in SILS. With disturbance torque, \|H\| drops to 0.09–0.6 only in the disturbed bands. Whether the real vehicle carries such torques is not testable here (unverified). |
| C. Real flips | No-fit residual $R_\text{out}$ 0.29–0.31 (old D×V 0.97–1.04); best scale 0.79–0.81, extra delay 3–4 ms (old 0.54–0.57, 14–17 ms). Reproduces the earlier Python check. |
| C. Real hover | 5–10 Hz \|H\| is 0.78–0.84 for $R_\text{out}$ (old 0.36–0.38), delay 4 ms (old 14 ms). 0.5–5 Hz is 0.06–0.45 for both models and does not discriminate. Above 10 Hz \|H\| moves between 0.5 and 1.6 with the treatment of rows that reuse an IMU sample: not usable now. |
| Criteria | Not decided. Candidate metrics and where the models separate are in chapter 6. |

## 2. Plant terms and module correspondence

See the Japanese table in chapter 2 (same content). Summary: every term of `plant.cpp` — PWM quantisation (8 bit, truncating; `motor_driver.cpp:162-163`, `virtual_board.cpp:501-510`), duty clamp (`:608`), motor ODE with $R_\text{out}(D)$ (`:365-399`), RK4 at 0.25 ms (`:613-618`, `stampfly.xml:29`), battery model $V=\text{OCV}-IR_\text{int}$ (`:457-468`, float32 state), thrust $\eta C_t\omega^2$ (`:622`), arm moments from rotor sites (`stampfly.xml:131-134`), yaw reaction $C_q\omega^2+J\dot\omega$ (`:645-647`), rigid body (`stampfly.xml:86`) — is in `tools/sysid/forward_model.py` with constants only from `_generated_params.py`. Not implemented (default OFF/0 in the plant): transport delay, slew limit, ground effect, health, `torque_authority` ≠ 1, turbulence torque.

## 3. A: the module reproduces the SILS plant

`sysid_gate.scn` (hover with roll/pitch/yaw chirps, 44 s) and `api_flip_roll.scn` (roll flip, 39 s) were run in SILS. Residual $=\text{rms}(\alpha_\text{ref}-\alpha_\text{pred})/\text{rms}(\alpha_\text{ref})$ on airborne rows (altitude > 0.05 m; ground contact is not modelled). `alt_flight.scn` is axis-symmetric ($\alpha\approx10^{-14}$ rad/s²) and unusable here.

| Case, axis | vs truth, whole | vs truth, most dynamic 0.5 s | best extra delay, scale | vs gyro derivative, whole | vs gyro derivative, window |
|---|---|---|---|---|---|
| sysid_gate roll | 0.00 % | 0.00 % | 0 ms, 1.0000 | 3.32 % | 0.77 % |
| sysid_gate pitch | 0.01 % | 0.00 % | 0 ms, 1.0000 | 4.97 % | 1.91 % |
| sysid_gate yaw | 0.00 % | 0.00 % | 0 ms, 1.0000 | 4.43 % | 1.76 % |
| api_flip_roll roll (α rms 28.9 rad/s²) | 0.02 % | 0.00 % | 0 ms, 1.0000 | 0.36 % | 0.12 % |
| api_flip_roll pitch, yaw | not excited | | | | |

- The prediction is delayed by one plant substep (0.25 ms) for SILS logs; without it the residual is 1.26/1.46/2.38 % (sysid_gate) and 0.76 % (flip). The cause is untraced (unverified); it is not applied to real logs.
- Falsification: removing the PWM quantisation gives 4.85/3.58/3.45 % and 1.58 % (no alignment shift, no gyroscopic term), worse than 1.26/1.49/2.38 % and 0.76 % under the same conditions. Using the 1 Hz logged voltage instead of the re-run battery model gives 7.3 % on the flip.
- 3–5 % against the gyro derivative is the sensor path (cause unverified); the best delay and scale are 0 ms and 1.0000.
- `tools/sysid/test_forward_model.py` (7 tests, about 22 s; the 3 SILS-based tests skip if the SILS is not built) is a regression test (an automatic check that existing behaviour did not break); tolerances are about 10× the measured values.

## 4. B: why |H| is small at low frequency in hover

| Hypothesis | Content | Prediction written before computing |
|---|---|---|
| H1 | The duty contains the controller's response to unmodelled disturbance torque; a disturbance-free model predicts torque that, in reality, cancels the disturbance | No disturbance: \|H\| ≈ 1 at 0.5–40 Hz. With disturbance: \|H\| < 1 in the disturbed band |
| H2 | Model gain error at low frequency | \|H\| < 1 even without disturbance |

Scenarios (`make_hover_scn.py`): POS_HOLD hover, t = 18–74 s, with 0.23–1.9 Hz force gusts on the centre of mass (no torque). Case N: gusts only. Case D: N plus thrust-health modulation of motors 0/1/3 at 0.7/1.3/0.4 Hz (up to 3 %), a torque disturbance unknown to the module. Case N+turbulence: N with `--turbulence 0.03` (3.7–5.9 Hz torque).

| Case, axis | 0.5–2 Hz | 2–5 Hz | 5–10 Hz | 10–20 Hz | 20–40 Hz |
|---|---|---|---|---|---|
| N roll / pitch | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| D roll | 0.09 (coh. 0.14) | 0.59 | 1.08 | 1.51 | 2.79 |
| D pitch | 0.29 | 0.57 | 0.96 | 1.08 | 2.52 |
| D yaw | 0.25 | 0.28 | 0.64 | 0.58 | 0.77 |
| N+turbulence roll | 0.28 | 0.35 | 0.57 | 0.99 | 1.00 |
| N+turbulence pitch | 0.33 | 0.38 | 0.50 | 0.99 | 1.00 |

H1 was observed: \|H\| = 1.000 everywhere without torque disturbance (coherence 1.00), \|H\| drops only in the disturbed bands and returns to 1 above them (turbulence). A drop of 0.09–0.29 at 0.5–2 Hz is of the same size as the real logs (0.08–0.41). What this cannot show: whether the real vehicle has disturbance torques of that size (unverified). \|H\| > 1 above 10 Hz in case D is probably the staircase update of the health (unverified). The flat scale of 0.79–0.81 in the real flips is consistent with H1, but the 0.8 deficit itself is unexplained.

## 5. C: application to the real logs

Real logs have jittered IMU stamps and rows that reuse the previous IMU sample (2–4 % overall, 9–16 % in the hover windows). Treatments in `load_real_log()`: `timestamp` (one sample per distinct stamp, duty of the other rows dropped; default, as in `flipcheck.py`), `spread` (all duty rows kept, rows sharing a stamp spread evenly over the preceding stall), `raw` (rows assumed uniform, stale gyro kept). Voltage: the 1 Hz status reading, linearly interpolated.

Flip windows (timestamp; same smoothing as before):

| Log, window | Model | No-fit residual | Best extra delay | Best scale | Residual at best |
|---|---|---|---|---|---|
| flip_test1 roll | $R_\text{out}$ | 0.311 | 4 ms | 0.790 | 0.095 |
| flip_test1 roll | old D×V | 1.039 | 14 ms | 0.537 | 0.149 |
| flip_test2 roll | $R_\text{out}$ | 0.285 | 3 ms | 0.807 | 0.136 |
| flip_test2 roll | old D×V | 0.967 | 17 ms | 0.568 | 0.216 |
| flip_test2 pitch | $R_\text{out}$ | 1.398 | 40 ms (scan limit) | 0.335 | 0.883 |

The pitch window is dominated by the post-flip recovery and is not judgeable. `spread` agrees with `timestamp` within 1 %; `raw` gives 0.917 (flip_test1) and 0.437 (flip_test2). Voltage assumption: logged 1 Hz value (mean 3.90 V) 0.311/0.790 and 0.285/0.807 (flip_test1/flip_test2: no-fit residual, scale); plant battery model 0.294/0.799 and 0.256/0.820; constant 3.7 V 0.216/0.867 and 0.198/0.888. PWM quantisation changes the residual by ≤ 0.005.

Hover windows (timestamp): see the Japanese tables in chapter 5 for \|H\| with the opposite-direction bracket, coherence and delay per band. Key rows (roll, $R_\text{out}$ / old): 5–10 Hz \|H\| 0.78 / 0.38 (flip_test1) and 0.84 / 0.36 (flip_test2), coherence 0.89–0.95 (old 0.93–0.97), delay 4.4 / 14.4 ms. 0.5–5 Hz: 0.08–0.45 (old 0.06–0.23), dominated by disturbance cancellation. Yaw on real logs: not evaluated (unverified).

Dependence on the reused-row treatment (roll, $R_\text{out}$, |H| / coherence): flip_test1 5–10 Hz 0.78/0.89 (timestamp), 0.77/0.88 (spread), 0.84/0.96 (raw); 10–20 Hz 0.51/0.49, 0.51/0.39, 0.95/0.66; 20–40 Hz 0.41/0.09, 0.34/0.04, 1.60/0.22. flip_test2 5–10 Hz 0.84, 0.83, 0.86; 10–20 Hz 1.18, 1.12, 1.51. The 5–10 Hz band is stable (within 0.07); above 10 Hz the treatment moves \|H\| by up to a factor of 2 with low coherence. Which treatment matches the true timing cannot be decided because the log carries no real control-tick time (unverified).

## 6. Candidate metrics and where the models separate

| Candidate metric | $R_\text{out}$ | Old D×V | Separation | Note |
|---|---|---|---|---|
| Flip no-fit residual (roll) | 0.20–0.31 | 0.97–1.04 | clear (3–5×) | depends on the voltage assumption |
| Flip best scale | 0.79–0.89 | 0.54–0.57 | clear | deficit of ≈0.8 unexplained, depends on voltage |
| Flip extra delay | 3–4 ms | 14–17 ms | clear | alignment error ~1 ms |
| Hover 5–10 Hz \|H\| (roll) | 0.78–0.84 | 0.36–0.38 | clear | coherence 0.89–0.97 for both; pitch varies by window |
| Hover 5–10 Hz delay | 4.2–4.4 ms | 14.4 ms | clear | roll only |
| Hover 0.5–5 Hz \|H\| | 0.08–0.45 | 0.06–0.23 | not discriminating | disturbance cancellation dominates (chapter 4) |
| Hover ≥ 10 Hz \|H\| | 0.4–1.3 | 0.26–0.51 | not discriminating | factor-2 dependence on row treatment, low coherence |

Applying the same procedure to a SILS flight of the same manoeuvre and comparing it with the real log side by side was not done (unverified).

## 7. Unverified items and limits

Cause of the 1-substep alignment offset; cause of the 3–5 % residual against the gyro derivative; existence and size of the real disturbance torques; cause of the 0.8 flip scale; which reused-row treatment is closest to the true timing; real yaw axis; cause of |H| > 1 above 10 Hz in case D; effects of ground effect, contact, motor delay, slew limit and turbulence torque (not implemented). The branch could not be pushed (GitHub token invalid; `gh auth login` needed); commits are local on `feat/forward-model-check`.

## 8. Reproduction

`sf sils build`; `sf sils scenario <scn> --target vehicle --duration <us>`; `analysis/scripts/forward_model_check_20261101/{selfcheck_sils.py, make_hover_scn.py, sils_hover_h.py, real_logs.py}` (`real_logs.py` needs `SF_LOGS`); `python -m pytest tools/sysid/test_forward_model.py`. All need `PYTHONPATH=<repo root>:<repo root>/lib`.
