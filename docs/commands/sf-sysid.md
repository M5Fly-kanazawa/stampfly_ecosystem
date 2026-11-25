# sf sysid

フライトログからのシステム同定コマンド。

バンドル（一式）を指定する引数は拡張子を省略でき、裸の名前は `logs/` 内も検索対象になります
（例: `sf sysid rate-fit flight_20260912T093015 --axis roll`）。

## 構文

```bash
sf sysid <subcommand> [options]
```

### サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `noise` | センサノイズ特性解析（アラン分散） |
| `inertia` | 慣性モーメント推定（ステップ応答） |
| `motor` | モータダイナミクス同定（Ct, Cq, tau_m） |
| `drag` | 空力抗力係数推定 |
| `params` | パラメータ管理（表示・差分・エクスポート） |
| `validate` | バリデーション・整合性チェック |
| `plan` | 飛行試験計画の生成 |
| `rate-fit` | レート制御ループのプラント b・T・L を、励振飛行のフライトログから同定（励振区間だけを使う） |
| `thrust-excite` | 鉛直の推力励振の飛行を API で自動化（離陸 → 高さ 0.7 m → 励振 → 着陸） |
| `thrust-fit` | 推力 → 上向き加速度のプラント・ToF の遅れ・加速度計の雑音を記録から同定 |

## 使用例

```bash
sf sysid noise flight_log.csv
sf sysid motor flight_log.csv
sf sysid params show
```

> 詳細なドキュメントは今後追加予定です（推力励振の 2 コマンドは下記）。
> [English version of the rate-fit and thrust-excitation sections follows below.](#english-thrust)

## レート同定（rate-fit）

```bash
sf sysid rate-fit <バンドル> --axis roll --f-lo 1 --f-hi 25
```

既定では、フライトログ内のテキストログ（`text_log`）にある `Sysid excitation start/done` の行から、指定した軸の励振の区間を探し、その前後に `--pad` で決まる余白を足した区間だけを同定に使う。実機では地上・離陸・励振していないホバーの区間がスペクトル平均の大半を占め、閉ループとセンサ雑音のせいで周波数応答が物理的でなくなるため。励振が複数あれば最後のものを使い、その旨を表示する。`done` 行が無いときは開始 + `dur` を終了とする。使った区間（出所・開始と終了の秒・サンプル数・`nperseg`）は毎回表示する。

**Welch 法の区間と余白（`--pad auto`）。** 周波数応答は Welch 法（ログを一定の長さの区間に切り、各区間に Hann 窓（両端をなだらかに 0 へ落とす重み）をかけてスペクトルを平均する方法）で求める。区間の長さは `nperseg` サンプル（400 Hz で 1024 サンプル = 2.56 s）、隣の区間とは半分ずつ重ねる。区間は切り出した窓の先頭から並べ、最後に区間 1 つ分に満たない端数のサンプルは捨てる。そのため余白が短いと、励振の終わり近くが一部しか使われない。励振 10 s（対数チャープ 1〜25 Hz）に前後 0.5 s を足した窓は 11 s = 4400 サンプルで、区間は 7 つ。最後の区間の終わりは励振の開始から 9.74 s（約 23 Hz）なので、23〜25 Hz は一度も使われず、約 18 Hz（励振の開始から約 8.9 s）より上は重みも半分未満になる。開始側も、最初の約 0.3 s は重みが半分未満である。実機のロール角速度の励振飛行（2026-11-23）では、20〜25 Hz のコヒーレンス（入出力の線形な関係の強さ。1 に近いほど信頼できる）が 0.12〜0.56 に落ちた。励振のあとに 1.5 s 残して同定すると 0.68〜0.93 になり、周波数応答はモデルと約 3 dB 以内で合った。

既定の `--pad auto` は、励振の前に区間の半分（1 ホップ。`nperseg`/2 サンプル）、励振のあとに区間 1 つ分（`nperseg` サンプル）を足す（1024 サンプルなら 1.28 s と 2.56 s）。励振の全サンプルが 2 つの区間の重なる所に入り、各サンプルの重み（その点を含む区間の Hann 窓の 2 乗の和）が 0.5 以上になる。`nperseg` を自動にしたときは、余白を足した窓が 4096 サンプル以上なら 1024、未満なら 512（余白は 0.64 s と 1.28 s）とし、余白と `nperseg` を一緒に決める。`--nperseg` を指定したときは、その値から余白を決める。`--pad 0.5` のように数値を与えると、従来どおり前後に同じ秒数を足す。

実際に使う行の時刻で重みを計算し、使った区間の表示（`window:` の行）の次の行に、覆い具合を表示する（励振のうち重みが 0.5 以上の範囲、重みの最小値、最後に使った行の時刻。励振がチャープなら対応する周波数も）。励振の端の 0.05 s を超える部分が重み 0.5 未満のとき（励振のあとのログが短い、`--pad` の数値が小さい、など）は警告を出し、失われる範囲（例: 「チャープの 17.7 Hz より上」）と対処（励振のあとのログを長く残す、`--pad` を大きくする）を示す。`--time-range` と `--window all` では励振の位置が分からないので、この行は出さない。

| オプション | 既定 | 意味 |
|-----------|------|------|
| `--window {auto,all}` | `auto` | `auto` は励振区間（見つからなければ警告してログ全体）、`all` はログ全体 |
| `--time-range T0 T1` | - | ログ先頭行からの秒で区間を直接指定（`--window` より優先） |
| `--pad {auto,秒}` | `auto` | 励振区間の前後に足す余白。`auto` は前に `nperseg`/2・後に `nperseg` サンプル分（1024 で 1.28 s / 2.56 s）、数値は前後に同じ秒数 |
| `--nperseg` | 自動 | Welch 法の区間長。余白を足した窓が 4096 サンプル以上なら 1024、未満なら 512 |

電圧（status ストリームは 1 Hz で、400 Hz のストリームより遅れて始まる）が先頭で欠けているときは、近い有効値で補い、補った行数を `duty_reason` に残す。周波数応答に有限な点が無い、またはコヒーレンスが有限でないときは、初期値を返さずエラーで止まる。

---

## 推力励振の同定（thrust-excite / thrust-fit）

高さの制御（ALT_HOLD / POS_HOLD）を閉じたまま、総推力の指令へ小さな多重正弦波（8 本: 0.4・0.7・1.1・1.7・2.7・4.1・5.9・8.9 Hz）または階段を足して、推力 → 鉛直の加速度のむだ時間と遅れ、下向き距離センサ（ToF）の遅れ、飛行中の加速度計の雑音を測る。ファームウェア側の設計は `firmware/vehicle/docs/detailed_design.md` の「推力励振」節、波形の基準は `protocol/spec/thrust_excitation.yaml`。

### 手順

```bash
sf log wifi -d 60
sf sysid thrust-excite --takeoff --land
sf sysid thrust-fit logs/flight_<timestamp>.sflog.zip --plot out/ --json-out out/thrust_fit.json
```

1 行目は別の端末で先に始める（機体から 400 Hz の記録を取る）。励振の信号そのものは記録に入らない。PC が基準の表から再生成する。

### thrust-excite のオプション

| オプション | 既定 | 説明 |
|-----------|------|------|
| `--waveform multisine\|steps` | multisine | 波形 |
| `--amp` | 5 | 振幅［ホバー推力の %］ |
| `--dur` | 20 | 多重正弦波の長さ［s］ |
| `--half-period` | 0.6 | 階段: 正または負の 1 段の長さ［s］ |
| `--count` | 4 | 階段: 正負の対の数 |
| `--height` | 0.7 | `--takeoff` の後に上る高さ［m］（離陸直後は 0.5 m。`up`/`down` は 10〜300 cm 単位） |
| `--takeoff` / `--land` | なし | 離陸する / 最後に着陸する（`--takeoff` なしでは今の高さで励振する） |
| `--ip` | 192.168.10.1 | 機体の IP |

機体の応答は `ok` / `error excitation rejected: <理由>` / `error excitation aborted: <理由>` / `error flight ended`。エラーのとき `--land` が付いていれば着陸を試みる。

### thrust-fit の出力

| 項目 | 内容 |
|------|------|
| `K`, `tau_s`, `L_s` と標準誤差 | G(s) = K·exp(−L·s)/(τ·s+1)。入力は励振後の総推力 `ctrl_output.thrust` ［N］、出力は重力を引いた上向きの加速度 ［m/s²］。L は加速度計の経路の遅れを含む |
| `tones` | 各音の周波数・ゲイン・位相・コヒーレンス（周波数ごとの信頼度）・信号対雑音比（SNR）・当てはめに使ったか・使わない理由。当てはめを省いたときも必ず出す。コヒーレンス 0.8（= 音の電力 ÷ 雑音の電力が 4 以上）未満の音は当てはめから外す |
| `fit_status` | 当てはめを行ったか、省いたなら理由（使える音が何本で、基準は何か）の 1 行 |
| `tof.status` | ToF の遅れを求めたか、省いたなら理由（SNR 10 以上の音が何本か）の 1 行 |
| `warnings`, `run_data` | 設計行列の条件数（後述）が 100 を超えたときの警告。窓の長さ・フーリエ分解能 1/T・条件数・雑音の見積もりに使った点数 |
| `tof.tof_delay_relative_to_accelerometer_s` | ToF の遅れ（**加速度計に対する相対値**。絶対値ではない）。加速度計の 2 階積分と ToF の位相差の傾きから求める。ToF の振幅が小さい高い周波数の音は SNR（音の電力 ÷ 近傍の電力）が 10 未満で外す |
| `accelerometer_noise` | 励振の前のホバー（待機の 1 s 前までの 10 s、だめなら 5 s）と励振の最中（音を引いた後）の、帯域ごとの実効値（0.4–1・1–3・3–6・6–9.5 Hz）と床の値［(m/s²)/√Hz］ |
| `start` | 励振の開始の周期の位置（50 Hz の変わり目から何周期ずれたか）と相関値 |
| 階段 | 区間の切り出しと、推力に対する加速度の立ち上がり（20 % 通過）の遅れだけ（参考値） |

閉ループの偏りを避けるため、G は励振 r を基準にした S_ra/S_ru で求める（u を基準にした S_ua/S_uu は、加速度の雑音がループを回って推力に戻る分で偏る）。8 本の音は 0.1 Hz の整数倍なので、各音の cos・sin と多項式の傾向（5 次）を最小二乗で当てはめ、複素振幅の比を取る。ToF は標本の時刻そのもので当てはめ、零次保持の補間はしない。

### 出力の読み方

`sf sysid thrust-fit` は、当てはめが成功しても省かれても、次の順に必ず出す。

1. `fit:` の行 — (K, τ, L) の当てはめを行ったか。省いたときは「使える音が N 本、基準は コヒーレンス 0.8 以上、3 本以上必要」の形で理由を出す。
2. 音ごとの表 — 周波数、`|G|`（ゲイン）、位相、`SNR_a(run)`（加速度の SNR、励振の最中の雑音を基準）、`SNR_a(hover)`（同、励振の前のホバーの雑音を基準）、`SNR_u(run)`（推力の SNR）、コヒーレンス、使える/使えない（使えない理由つき）。
3. `ToF:` の行 — ToF の遅れを求めたか。省いたときは理由を出す。
4. 窓の長さ・分解能 1/T・設計行列の条件数。`WARNING:` が付いたら下の限界の節を読む。
5. 加速度計の雑音（励振の前のホバーと励振の最中）。

**SNR の基準は「励振の最中の雑音」（`run`）が第一**。励振の前のホバーの雑音（`hover`）は、励振の最中の雑音より小さい（実機の最初の記録で、加速度の雑音の電力が 1.4〜11 倍。励振でループが動き、遅い変動や振動が増えるため）ので、`hover` を基準にすると楽観的になる。当てはめに使う判定は `run` だけで行い、`hover` は比較のために並べる。

**雑音の見積もり**: 音の cos・sin と低次の傾向を当てはめた残差を、音の周りの窓のフーリエ格子 k/T（T は窓の長さ。他の音から 1/T 以上離れた最も近い点のうち、傾向に吸収されない 6 点まで）で同じ推定器にかけ、電力の平均を雑音の電力とする。格子は 1/T の間隔で互いに直交するので、窓が短くても設計行列は悪条件にならない。合成データ（真の応答が既知。窓 10・20・30 s、24 通りの雑音）では、見積もった雑音の電力が音の振幅の実測の分散の 0.7〜1.3 倍に入った（最も低い音は 2〜3 倍の保守側になることがある）。

### 限界（短い記録・小さい振幅）

| 状況 | 起きること | 対処 |
|------|-----------|------|
| 励振が短い（窓が短い） | 分解能 1/T が粗く（窓 8.9 s で 0.112 Hz）、音の振幅の不確かさが増す。同じ飛行条件の合成データで、10 s の記録の τ の散らばり（1 sigma）は 4.4 ms、L は 3.4 ms、20 s で 2.4 ms と 2.1 ms、30 s で 1.9 ms と 1.4 ms。τ と L は互いに分けにくく、個別の値の誤差は大きい | `--dur 20` 以上。短い記録の τ と L は標準誤差を添えて読む |
| 振幅が小さい（`--amp` が小さい） | 加速度の音の電力が雑音に埋もれ、コヒーレンスが 0.8 に届かない音が増える。使える音が 3 本未満なら当てはめを省く（`fit_status` に理由）。実機の 2 %・10 s の記録では、使える音は 0 本だった（SNR_a(run) は 0.6〜2.7） | `--amp` を上げる（5 % が既定）、`--dur` を延ばす（SNR は窓の長さにほぼ比例して上がる） |
| ToF の SNR が低い | SNR 10 未満の音は ToF の遅れに使わない。2 本未満なら省く（`tof.status` に理由） | 同上。ToF の高さは加速度の 2 階積分に相当し、高い周波数の音ほど振幅が小さい |
| 条件数が 100 を超える | 設計行列がほぼ一次従属で、音の振幅が壊れる。この道具の標準の設計（8 本の音と 5 次の傾向）では窓 8.9 s で約 4、18.9 s で約 3.4 で、通常は出ない | `WARNING:` が出たら結果を使わない。音の間隔が窓の分解能 1/T より狭い設計にした場合に出る |

> 2026-11-23 の修正: 旧来の道具は、雑音の見積もりのために音の ±0.1・0.2・0.3 Hz を足して同時に当てはめていた。この間隔が 10 s の記録の窓（8.9 s、1/T = 0.112 Hz）の分解能より細かく、設計行列の条件数が 5.4×10¹⁰ になり、全部の音が「使えない」と判定されて当てはめが黙って省かれていた。上の方法に改め、省いたときは理由を出すようにした。

---

<a id="english-thrust"></a>

## Thrust-excitation identification (thrust-excite / thrust-fit)

With the altitude loop (ALT_HOLD / POS_HOLD) closed, a small multisine (eight tones: 0.4, 0.7, 1.1, 1.7, 2.7, 4.1, 5.9, 8.9 Hz) or a step sequence is added to the total thrust command. This measures the dead time and lag from thrust to vertical acceleration, the delay of the downward distance sensor (ToF), and the accelerometer noise in flight. Firmware design: "thrust excitation" in `firmware/vehicle/docs/detailed_design.md`; waveform source: `protocol/spec/thrust_excitation.yaml`.

### Workflow

```bash
sf log wifi -d 60
sf sysid thrust-excite --takeoff --land
sf sysid thrust-fit logs/flight_<timestamp>.sflog.zip --plot out/ --json-out out/thrust_fit.json
```

Start the first line in another terminal beforehand (400 Hz capture from the vehicle). The excitation signal itself is not logged; the PC regenerates it from the shared table.

### thrust-excite options

| Option | Default | Description |
|--------|---------|-------------|
| `--waveform multisine\|steps` | multisine | waveform |
| `--amp` | 5 | amplitude [% of hover thrust] |
| `--dur` | 20 | multisine length [s] |
| `--half-period` | 0.6 | steps: length of one positive or negative step [s] |
| `--count` | 4 | steps: number of positive/negative pairs |
| `--height` | 0.7 | height [m] to climb to after `--takeoff` (0.5 m right after takeoff; `up`/`down` move 10-300 cm) |
| `--takeoff` / `--land` | off | take off first / land at the end (without `--takeoff` the excitation runs at the current height) |
| `--ip` | 192.168.10.1 | vehicle IP |

Vehicle replies: `ok` / `error excitation rejected: <reason>` / `error excitation aborted: <reason>` / `error flight ended`. On an error, landing is attempted when `--land` is given.

### thrust-fit output

| Item | Meaning |
|------|---------|
| `K`, `tau_s`, `L_s` with standard errors | G(s) = K exp(-L s)/(tau s + 1). Input: total thrust after the excitation, `ctrl_output.thrust` [N]; output: upward acceleration minus gravity [m/s^2]. L includes the accelerometer path delay |
| `tones` | per tone: frequency, gain, phase, coherence (per-frequency reliability), signal-to-noise ratio (SNR), whether used in the fit, and why not. Always printed, also when the fit is skipped. Tones with coherence below 0.8 (= tone power / noise power below 4) are left out |
| `fit_status` | one line: whether the fit ran and, if not, why (how many tones are usable and what the criterion is) |
| `tof.status` | one line: whether the ToF delay was computed and, if not, why (how many tones have SNR of 10 or more) |
| `warnings`, `run_data` | a warning when the design-matrix condition number (below) exceeds 100; window length, Fourier resolution 1/T, condition number and the number of probes used for the noise |
| `tof.tof_delay_relative_to_accelerometer_s` | ToF delay RELATIVE to the accelerometer (not an absolute delay), from the slope of the phase between the double-integrated acceleration and the ToF height. Tones whose SNR (tone power / neighbor power) is below 10 are dropped |
| `accelerometer_noise` | hover before the excitation (10 s ending 1 s before the standby, else 5 s) and during it (tones subtracted): band RMS (0.4-1, 1-3, 3-6, 6-9.5 Hz) and floor [(m/s^2)/sqrt(Hz)] |
| `start` | start cycle of the excitation (offset from the 50 Hz state edge) and the correlation |
| steps | cut-out and the rise delay (20 % crossing) of the acceleration after the thrust only (reference value) |

To avoid the closed-loop bias, G is S_ra/S_ru with the excitation r as the reference (S_ua/S_uu is biased by acceleration noise that returns through the loop into the thrust). The eight tones are multiples of 0.1 Hz, so cos, sin of each tone and a polynomial trend (order 5) are fitted by least squares and the complex amplitudes divided. The ToF is fitted at its own sample times; no zero-order-hold interpolation is used.

### Reading the output

`sf sysid thrust-fit` always prints, in this order, whether the fit succeeded or was skipped:

1. The `fit:` line: whether the (K, tau, L) fit ran. If skipped, the reason ("N tones are usable, criterion coherence >= 0.8, at least 3 are needed").
2. The per-tone table: frequency, `|G|` (gain), phase, `SNR_a(run)` (acceleration SNR against the noise during the excitation), `SNR_a(hover)` (the same against the hover before the excitation), `SNR_u(run)` (thrust SNR), coherence, usable or not (with the reason).
3. The `ToF:` line: whether the ToF delay was computed; the reason if skipped.
4. Window length, resolution 1/T and design-matrix condition number. If a `WARNING:` appears, read the limits section below.
5. Accelerometer noise (hover before the excitation, and during it).

**The reference for the SNR is the noise during the excitation (`run`).** The hover noise before the excitation (`hover`) is smaller than the noise during it (on the first real record the acceleration noise power was 1.4 to 11 times smaller: the excitation moves the loop and raises the slow wander and vibration), so using it would be optimistic. Only `run` decides which tones are used in the fit; `hover` is printed for comparison.

**Noise estimate:** the residual of the tones-plus-low-order-trend fit is passed through the same estimator at the Fourier grid k/T around each tone (T is the window length; the nearest bins at least 1/T from every tone and not absorbed by the trend, up to 6), and the mean power is the noise power. The grid points are 1/T apart and mutually orthogonal, so the design matrix is well conditioned however short the window is. On synthetic data with a known response (windows of 10, 20 and 30 s, 24 noise seeds) the estimated noise power was 0.7 to 1.3 times the measured variance of the tone amplitude (up to 2 to 3 times, on the conservative side, at the lowest tone).

### Limits (short runs, small amplitudes)

| Situation | What happens | What to do |
|-----------|--------------|------------|
| Short excitation (short window) | The resolution 1/T is coarse (0.112 Hz for an 8.9 s window) and the tone amplitudes are less certain. On synthetic data under the same conditions the scatter (1 sigma) of tau is 4.4 ms and of L 3.4 ms for a 10 s run, 2.4 and 2.1 ms for 20 s, 1.9 and 1.4 ms for 30 s. tau and L are hard to separate, so each has a large error | `--dur 20` or more. Read tau and L of a short run together with their standard errors |
| Small amplitude (small `--amp`) | The acceleration tone power sinks into the noise and more tones miss coherence 0.8. With fewer than 3 usable tones the fit is skipped (reason in `fit_status`). On the real 2 %, 10 s record no tone was usable (SNR_a(run) 0.6 to 2.7) | Raise `--amp` (default 5 %), lengthen `--dur` (the SNR rises roughly in proportion to the window length) |
| Low ToF SNR | Tones with SNR below 10 are not used for the ToF delay; with fewer than 2 the delay is skipped (reason in `tof.status`). The ToF height corresponds to the double integral of the acceleration, so high tones are small | Same as above |
| Condition number above 100 | The design matrix is nearly rank-deficient and the tone amplitudes break. With the standard design (eight tones, order-5 trend) it is about 4 for an 8.9 s window and about 3.4 for 18.9 s, so it normally does not appear | Do not use a result with a `WARNING:`. It appears when a design has tone spacings finer than the window resolution 1/T |

> Fix of 2026-11-23: the earlier tool added neighbors at +-0.1, 0.2 and 0.3 Hz of every tone to the same least-squares fit to estimate the noise. That spacing is finer than the resolution of a 10 s run's window (8.9 s, 1/T = 0.112 Hz), the condition number reached 5.4e10, every tone was judged unusable and the fit was silently skipped. The method above replaces it, and a skipped fit now states its reason.


---

## Rate-loop identification (rate-fit)

```bash
sf sysid rate-fit <bundle> --axis roll --f-lo 1 --f-hi 25
```

By default the command finds the excitation window of the chosen axis from the `Sysid excitation start/done` lines in the bundle's text log (`text_log`) and fits only that window, padded as set by `--pad`. On hardware the ground, takeoff and unexcited hover segments dominate the spectral average (closed loop plus sensor noise) and give an unphysical frequency response. With several excitations the last one is used and the output says so; without a `done` line the end is start + `dur`. The window actually used (source, start and end in s, number of samples, `nperseg`) is always printed.

**Welch segments and the padding (`--pad auto`).** The frequency response is estimated with Welch averaging: the log is cut into segments of `nperseg` samples (1024 at 400 Hz = 2.56 s, overlapping by half), each segment is multiplied by a Hann window (a weight that tapers smoothly to zero at both ends), and the spectra are averaged. Segments are laid out from the first sample of the window, and leftover samples at the end that do not fill a whole segment are dropped. A short pad therefore leaves the end of the excitation partly unused. A 10 s log chirp (1-25 Hz) padded by 0.5 s on each side is an 11 s = 4400-sample window with 7 segments; the last one ends 9.74 s after the excitation start (about 23 Hz), so 23-25 Hz is never analysed and above about 18 Hz (8.9 s into the chirp) the weight is below one half. The start is weak too: the first ~0.3 s has less than half weight. On the roll-rate excitation flights of 2026-11-23 the coherence (how strongly input and output are linearly related; 1 = fully reliable) at 20-25 Hz was 0.12-0.56; with 1.5 s kept after the chirp it became 0.68-0.93 and the frequency response matched the model within about 3 dB.

The default `--pad auto` adds one Welch hop (`nperseg`/2 samples) before the excitation and one Welch segment (`nperseg` samples) after it (1.28 s and 2.56 s for 1024). Every excitation sample then lies where two segments overlap, so its weight (the sum of the squared Hann windows of the segments containing it) is at least 0.5. With `nperseg` on auto, the window is 1024 if the padded window has at least 4096 samples, else 512 (pads 0.64 s and 1.28 s); the pads and `nperseg` are decided together. With `--nperseg N` the pads follow N. A number such as `--pad 0.5` keeps the old meaning: that many seconds on both sides.

The weights are computed on the rows actually used (with their real timestamps), and one line after the window line reports the coverage (the part of the excitation with weight >= 0.5, the minimum weight, the time of the last analysed row; for a chirp also the corresponding frequencies). If more than 0.05 s at either end of the excitation has weight < 0.5 (the log ends too soon after the excitation, or a small numeric `--pad`), a warning says what is lost (for example "the chirp above 17.7 Hz") and what to do (keep the log longer after the excitation, or raise `--pad`). With `--time-range` or `--window all` the excitation position is unknown, so the line is not printed.

| Option | Default | Meaning |
|--------|---------|---------|
| `--window {auto,all}` | `auto` | `auto` = the excitation window (the whole log with a warning if none is found); `all` = the whole log |
| `--time-range T0 T1` | - | Window in seconds from the first row of the log (overrides `--window`) |
| `--pad {auto,SECONDS}` | `auto` | Padding around the excitation window. `auto` = `nperseg`/2 samples before and `nperseg` after (1.28 s / 2.56 s for 1024); a number = that many seconds on both sides |
| `--nperseg` | auto | Welch segment length: 1024 if the padded window has at least 4096 samples, else 512 |

Battery voltage (the status stream is 1 Hz and starts later than the 400 Hz streams) that is missing at the start is filled from the nearest valid sample, and the number of filled rows is reported in `duty_reason`. If the frequency response has no finite points or the coherence is not finite, the command stops with an error instead of returning the start values.
