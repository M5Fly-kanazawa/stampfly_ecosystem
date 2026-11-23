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
| `thrust-excite` | 鉛直の推力励振の飛行を API で自動化（離陸 → 高さ 0.7 m → 励振 → 着陸） |
| `thrust-fit` | 推力 → 上向き加速度のプラント・ToF の遅れ・加速度計の雑音を記録から同定 |

## 使用例

```bash
sf sysid noise flight_log.csv
sf sysid motor flight_log.csv
sf sysid params show
```

> 詳細なドキュメントは今後追加予定です（推力励振の 2 コマンドは下記）。
> [English version of the thrust-excitation section follows below.](#english-thrust)

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
| `tones` | 各音の周波数・ゲイン・位相・コヒーレンス（周波数ごとの信頼度）・当てはめに使ったか。コヒーレンス 0.8 未満の音は当てはめから外す |
| `tof.tof_delay_relative_to_accelerometer_s` | ToF の遅れ（**加速度計に対する相対値**。絶対値ではない）。加速度計の 2 階積分と ToF の位相差の傾きから求める。ToF の振幅が小さい高い周波数の音は SNR（音の電力 ÷ 近傍の電力）が 10 未満で外す |
| `accelerometer_noise` | 励振の前のホバー（待機の 1 s 前までの 10 s、だめなら 5 s）と励振の最中（音を引いた後）の、帯域ごとの実効値（0.4–1・1–3・3–6・6–9.5 Hz）と床の値［(m/s²)/√Hz］ |
| `start` | 励振の開始の周期の位置（50 Hz の変わり目から何周期ずれたか）と相関値 |
| 階段 | 区間の切り出しと、推力に対する加速度の立ち上がり（20 % 通過）の遅れだけ（参考値） |

閉ループの偏りを避けるため、G は励振 r を基準にした S_ra/S_ru で求める（u を基準にした S_ua/S_uu は、加速度の雑音がループを回って推力に戻る分で偏る）。8 本の音は 0.1 Hz の整数倍なので、各音の cos・sin と多項式の傾向を最小二乗で当てはめ、複素振幅の比を取る。ToF は標本の時刻そのもので当てはめ、零次保持の補間はしない。

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
| `tones` | per tone: frequency, gain, phase, coherence (per-frequency reliability), whether used in the fit. Tones with coherence below 0.8 are left out |
| `tof.tof_delay_relative_to_accelerometer_s` | ToF delay RELATIVE to the accelerometer (not an absolute delay), from the slope of the phase between the double-integrated acceleration and the ToF height. Tones whose SNR (tone power / neighbor power) is below 10 are dropped |
| `accelerometer_noise` | hover before the excitation (10 s ending 1 s before the standby, else 5 s) and during it (tones subtracted): band RMS (0.4-1, 1-3, 3-6, 6-9.5 Hz) and floor [(m/s^2)/sqrt(Hz)] |
| `start` | start cycle of the excitation (offset from the 50 Hz state edge) and the correlation |
| steps | cut-out and the rise delay (20 % crossing) of the acceleration after the thrust only (reference value) |

To avoid the closed-loop bias, G is S_ra/S_ru with the excitation r as the reference (S_ua/S_uu is biased by acceleration noise that returns through the loop into the thrust). The eight tones are multiples of 0.1 Hz, so cos, sin of each tone and a polynomial trend are fitted by least squares and the complex amplitudes divided. The ToF is fitted at its own sample times; no zero-order-hold interpolation is used.
