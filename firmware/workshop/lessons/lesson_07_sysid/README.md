# Lesson 7: システム同定 / System Identification

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このレッスンについて

フライトデータからプラントモデル $G_p(s) = K / (s(\tau_m s + 1))$ のパラメータ $K$, $\tau_m$ を同定する。
L5 の P 制御で飛行し、WiFi テレメトリでデータを取得した後、`sf sysid fit` でモデルフィッティングを行う。

### 前提知識

- L05: レート P 制御と初フライト（Kp, rate_max の値を使う）
- L06: システムモデリング（伝達関数、プラントモデル）

## 2. システム同定の仕組み

### アルゴリズム概要

`sf sysid fit` は既定（`--input auto`）で **間接閉ループ方式**
（`--input indirect`）を使う。一式の `motor.csv` に 400Hz で記録される4モータの
`duty_FR/RR/RL/FL` 列から短タップの FIR 回帰で瞬時比例ゲイン相当
（$K_p$）を自動推定し、その $K_p$ で target→gyro の閉ループ伝達関数
そのものを直接フィットする。$K_p$ の値を知る必要も `--kp` を指定する
必要もない（duty 列は「$u(t)$ を直接復元する」旧来の使い方ではなく、
$K_p$ を自動推定するための材料として使われる）:

```
一式（.sflog.zip: sf log wifi が書く、信号ごとのCSVとmeta.json/schema.jsonをzipにまとめた
StampFlyフライトログv1一式）に記録されるデータ:
  motor.csv の duty_FR/RR/RL/FL  : 4モータ duty [0,1]（400Hz、Kp自動推定に使用）
  rate_ref.csv の rate_ref_roll/pitch/yaw : 角速度目標 [rad/s]（target、閉ループの入力）
  imu.csv の gyro_x               : ロール角速度実測 [rad/s]（閉ループの出力）

間接閉ループ同定（既定、--kp 不要）:
  1. duty 列から短タップ FIR 回帰で Kp を自動推定
  2. target(t) → gyro(t) の閉ループ応答を，推定した Kp で
     シミュレーションし，実測 gyro と比較して K, τm をフィット
```

400Hz duty 列が無い、または FIR 推定の当てはまりが悪い（推定の R² が
閾値未満）ログでは、次の順にフォールバックする: (1) `control_output`
（プリミキサー指令、400Hz、`--mixer` 不要）、(2) **duty優先方式**
（ミキサー逆算で $u(t)=$ mixer\_inverse(duty) を直接復元し、開ループで
フィット。`--kp` は不要だが `--mixer` の指定が正しいことが必須）、
(3) 明示的な `--kp` による旧来の直接フィット
（$u=K_p(\text{target}-\text{gyro})$ を外部入力として使う、
`--input kp --kp 0.5`）。`--kp` を明示的に渡した場合は FIR 推定を
スキップし、そのまま指定した $K_p$ で間接閉ループ方式を使う（既知の
$K_p$ との比較や、duty 列の無い旧ログでの再現に使う）。

人間の操縦では持続的な高周波（〜8Hz）励振を安全に作れないため、
$u=K_p(\text{target}-\text{gyro})$ を外部入力として直接フィットする
旧来の "kp"/duty優先方式は実飛行データで破綻しやすい（実測で R² < 0、
K が理論値から1〜3桁ズレる例あり）。既定の間接閉ループ方式はこの問題を
回避し、実飛行データで K を理論値の数%〜数十%程度まで復元できる
（ただし $\tau_m$ は、人間操縦データからは高周波成分が不足するため、
K ほど頑健には決まらない — 詳細は下の重要事項を参照）。

### なぜ閉ループのまま同定できるか

間接閉ループ方式は、target(t)（パイロットのスティック由来、gyro(t) と
代数的に絡み合っていない真に外部の信号）を直接シミュレーションの入力に
使い、実測 gyro と比較する。閉ループを経由せず $u(t)$ を単独で復元
しようとする duty優先方式・旧来の "kp" 方式よりも、実飛行データに対して
遥かに頑健である。duty優先方式自体は、閉ループ制御の外側にあるモータ
指令そのもの（4モータ duty）を直接観測して逆算するため、$K_p$ の値にも
フィードバック則の仮定にも依存しない、という別の利点を持つ。

> **重要（励振不足の落とし穴 / τm の識別性）:** duty優先方式・旧来の
> "kp" 方式では、スティックをほとんど動かさずに一定方向へ持ち続けた
> 区間があると、閉ループの P 制御則 $u = K_p(\text{target} - y)$ が
> $u \approx \text{定数} - K_p y$ に潰れ、$u$ と $y$ が「プラントの
> 動特性やハードウェアの符号とは無関係に」強く負相関して見える
> （閉ループ同定の典型的な落とし穴）。`sf sysid fit` はこの状態を検出
> すると該当区間を除外し警告する。既定の間接閉ループ方式は K について
> この罠を構造的に回避するが、**$\tau_m$ は別問題として、励振が弱いと
> 本質的に決まりにくい**（実測では、複数の $\tau_m$ 候補で当てはまり
> の良さ R² がほとんど変わらない「識別性の低い」状態になりうる —
> 「間接方式を使えば $\tau_m$ も自動的に正確になる」わけではない）。
> 良い $\tau_m$ を得るには、K の場合よりもさらに大きめ・高頻度・広帯域な
> 励振が要る。いずれにせよステップ2の励振の指示に従うこと。

## 3. 手順

### ステップ 1: ファームウェア準備

1. `sf lesson switch 7` でテンプレートを `user_code.cpp` にコピー
2. `user_code.cpp` を開き、$K_p$ を設定（例: 0.5、L5 で使った値）
3. 角速度目標を計算した直後に `ws::set_rate_target(roll_target, pitch_target, yaw_target)` を呼ぶ（テンプレートに TODO ヒントあり）
4. WiFi チャンネルを設定
5. ビルド & 書き込み: `sf lesson build` → `sf lesson flash`

### ステップ 2: フライト & データ取得

1. PC でテレメトリ受信を開始（既定で一式 `.sflog.zip` として保存される）: `sf log wifi -d 30`
2. ARM → ホバリング → スティック操作でロール・ピッチ・ヨー入力
3. **フライト全体を通じて、各軸のスティックを大きめの振幅で連続的に、
   ランダムっぽく動かし続けること。** 一定方向に持ち続ける時間を作らない
   （2〜3回軽く動かすだけでは全く足りない — 目安として、記録全体での
   `rate_ref` の標準偏差が `rate_max` の 10% 未満だと `sf sysid fit` が
   励振不足として区間を除外し、同定に失敗する）
4. 着陸 → DISARM

保存先は既定で `logs/flight_<YYYYMMDD>T<HHMMSS>.sflog.zip`。取得直後に `sf log check` が
自動実行され、電文の解析エラーがあればその場で報告される。中身を1枚の表として見たい場合は
`sf log convert logs/flight_<日時>.sflog.zip --aligned` で派生の整列 CSV（`timestamp_us`,
`gyro_x/y/z`, `rate_ref_roll/pitch/yaw`, `duty_FR/RR/RL/FL` 等が1周期1行）を作れるが、
`sf sysid fit` 自身はこの変換を内部で行うため、通常は一式をそのまま渡せばよい。

### ステップ 3: 同定

`--mixer` の既定値 `legacy`（`ws_internal.hpp` の線形Xクアッドミキサー）は本レッスンの
`firmware/workshop` ファームに対応するため、明示指定は不要。

全軸を同定する（既定で間接閉ループ方式が自動選択される。`--kp` も不要）:

```bash
sf sysid fit logs/flight_20260911T121243.sflog.zip --plot
```

特定軸のみ同定する:

```bash
sf sysid fit logs/flight_20260911T121243.sflog.zip --axis roll --plot
```

結果を YAML に保存する:

```bash
sf sysid fit logs/flight_20260911T121243.sflog.zip -o my_plant.yaml
```

既知の Kp を明示指定したい場合（FIR自動推定をスキップしてそのKpを使う）:

```bash
sf sysid fit logs/flight_20260911T121243.sflog.zip --kp 0.5 --plot
```

400Hz duty 列の無い旧ログなど、間接方式が使えない場合の直接fit（比較・デバッグ用。`--mixer`
が正しいログにのみ有効）:

```bash
sf sysid fit logs/flight_20260911T121243.sflog.zip --input kp --kp 0.5 --plot
```

取得した一式は `sf log viz logs/flight_20260911T121243.sflog.zip` で可視化できる
（`docs/guides/flight-log-viz.md` 参照）。

### ステップ 4: L6 理論値と比較

| 軸 | K (同定) | K (L6理論) | τm (同定) | τm (理論) |
|-----|---------|-----------|----------|----------|
| Roll | ? | 102.0 | ? | 0.020 |
| Pitch | ? | 70.0 | ? | 0.020 |
| Yaw | ? | 8.0 | ? | 0.020 |

同定した K, τm から設計 Kp を計算: $K_p = 1/(4\zeta^2 K \tau_m)$

**τm が理論値から大きくズレていても、必ずしも失敗ではない:** 上の重要事項
で触れた通り、τm は K よりも励振に敏感で、通常のスティック操作では
理論値0.02sの数倍（0.05〜0.08s程度）に振れることがある（複数の候補τmで
R² がほとんど変わらない、識別性の低い状態）。K の一致度をまず確認し、
τm の大きなズレは「励振をさらに強めて再挑戦する」動機として扱うこと。

## 4. API

| 関数 | 説明 | 値域 |
|------|------|------|
| `ws::gyro_x/y/z()` | 角速度 | rad/s |
| `ws::rc_roll/pitch/yaw()` | スティック入力 | -1.0 〜 +1.0 |
| `ws::rc_throttle()` | スロットル | 0.0 〜 1.0 |
| `ws::set_rate_target(r,p,y)` | 角速度目標を Data Stream に記録（ロギング専用、制御には無関係） | rad/s |
| `ws::motor_mixer(T,R,P,Y)` | モーターミキサー | --- |
| `ws::led_color(r,g,b)` | LED 色設定 | 0〜255 |
| `ws::set_channel(ch)` | WiFi チャンネル | 1, 6, 11 |

## 5. チャレンジ

- 異なる Kp（例: 0.3, 0.7）で飛行し、同定結果がどう変わるか比較する
- `--time-range` オプションで特定区間のみ分析する
- 同定した K, τm でシミュレーション応答と実測を重ねてプロットする

---

<a id="english"></a>

## 1. Overview

### About This Lesson

Identify plant model parameters $K$ and $\tau_m$ from flight data where
$G_p(s) = K / (s(\tau_m s + 1))$.
Fly with L5's P controller, capture WiFi telemetry, then run `sf sysid fit` for model fitting.

### Prerequisites

- L05: Rate P control and first flight (need Kp, rate_max values)
- L06: System Modeling (transfer function, plant model)

## 2. How System Identification Works

### Algorithm Overview

By default (`--input auto`), `sf sysid fit` uses the **indirect
closed-loop method** (`--input indirect`). It auto-estimates the
instantaneous proportional gain ($K_p$) from the bundle's 400Hz
`motor.csv` `duty_FR/RR/RL/FL` columns via a short-tap FIR regression, then
fits the closed-loop target->gyro transfer function directly using that
$K_p$. There is no need to know $K_p$, or to pass `--kp` (the duty
columns aren't used the old way, to reconstruct $u(t)$ directly -- they're
just the material the FIR regression uses to estimate $K_p$):

```
Bundle columns (.sflog.zip -- a StampFly flight-log v1 bundle, one CSV
per signal plus meta.json/schema.json, written by sf log wifi):
  motor.csv: duty_FR/RR/RL/FL  : 4 motor duties [0,1] (400Hz, feeds Kp auto-estimate)
  rate_ref.csv: rate_ref_roll/pitch/yaw : rate target [rad/s] (target, the closed-loop input)
  imu.csv: gyro_x               : measured roll rate [rad/s] (the closed-loop output)

Indirect closed-loop identification (default, no --kp needed):
  1. Auto-estimate Kp from the duty columns via a short-tap FIR regression
  2. Simulate the target(t) -> gyro(t) closed-loop response with that Kp
     and fit K, tau_m against the measured gyro
```

For logs without the 400Hz duty columns, or where the FIR estimate's own
fit is poor (its R² is below a threshold), `sf sysid fit` falls back in
this order: (1) `control_output` (the pre-mixer command, 400Hz, no
`--mixer` needed), (2) the **duty-first method** (reconstruct
$u(t)=$ mixer\_inverse(duty) directly and fit it open-loop -- no `--kp`
needed, but `--mixer` must be correct), (3) the older direct fit with an
explicit `--kp` ($u=K_p(\text{target}-\text{gyro})$ used as a direct
external input, `--input kp --kp 0.5`). Passing `--kp` explicitly skips
the FIR estimate and uses that $K_p$ for the indirect method instead
(useful to compare against a known $K_p$, or to reproduce an older log
without duty columns).

A human pilot cannot safely sustain the persistent high-frequency (~8Hz)
excitation the direct "kp"/duty-first methods need when used as an
external-input fit, so fitting $u=K_p(\text{target}-\text{gyro})$ directly
tends to fail on real flight data (observed R² < 0, K off by 1-3 orders of
magnitude in practice). The default indirect closed-loop method avoids
this and recovers K within a few percent to a few tens of percent of
theory on real flight data (though $\tau_m$ is not as robustly determined
as K -- see the important note below).

### Why This Stays Well-Posed Without Opening the Loop

The indirect closed-loop method feeds target(t) -- a genuinely external
signal (the pilot's stick), not algebraically entangled with gyro(t) --
directly into the simulation and compares against the measured gyro. This
is far more robust on real flight data than the duty-first/older "kp"
methods, which try to reconstruct $u(t)$ on its own outside the loop. The
duty-first method has its own separate advantage: it directly observes the
motor command itself (4 motor duties), so it needs neither $K_p$ nor any
assumption about the feedback law.

> **Important (the insufficient-excitation pitfall / $\tau_m$'s
> identifiability):** with the duty-first or older "kp" methods, a stretch
> where the stick barely moves and is held in one direction collapses the
> closed-loop P-control law $u = K_p(\text{target} - y)$ into
> $u \approx \text{const} - K_p y$, making $u$ and $y$ look strongly and
> misleadingly *negatively* correlated -- regardless of the true plant
> dynamics or hardware sign convention (a classic closed-loop
> identifiability pitfall). `sf sysid fit` detects and drops such segments
> with a warning. The default indirect method structurally sidesteps this
> trap for K, but **$\tau_m$ is a separate problem: it stays poorly
> determined whenever excitation is weak** (in practice, several candidate
> $\tau_m$ values can give nearly the same R² -- a low-identifiability
> situation. Using the indirect method does NOT automatically make
> $\tau_m$ accurate). Getting a good $\tau_m$ needs even larger, more
> frequent, broader-bandwidth excitation than K does. Either way, follow
> Step 2's excitation guidance.

## 3. Procedure

### Step 1: Firmware Setup

1. Run `sf lesson switch 7` to copy the template to `user_code.cpp`
2. Open `user_code.cpp` and set $K_p$ (e.g., 0.5, same as L5)
3. Right after computing the rate targets, call `ws::set_rate_target(roll_target, pitch_target, yaw_target)` (a TODO hint is in the template)
4. Set WiFi channel
5. Build & flash: `sf lesson build` → `sf lesson flash`

### Step 2: Flight & Data Capture

1. Start telemetry on PC (saved by default as a bundle, `.sflog.zip`): `sf log wifi -d 30`
2. ARM → hover → apply roll/pitch/yaw stick inputs
3. **Keep moving each axis's stick continuously, with large amplitude, in a
   quasi-random pattern throughout the whole flight.** Never hold one
   direction for long (a couple of light stick taps is nowhere near
   enough -- as a rule of thumb, `sf sysid fit` drops segments as
   insufficiently excited, and fitting fails, when the recorded
   `rate_ref`'s standard deviation is below 10% of `rate_max`)
4. Land → DISARM

The default save path is `logs/flight_<YYYYMMDD>T<HHMMSS>.sflog.zip`. Right after capture,
`sf log check` runs automatically and reports any wire-format parse error immediately. To view
the contents as one table, `sf log convert logs/flight_<timestamp>.sflog.zip --aligned` builds a
derived aligned CSV (`timestamp_us`, `gyro_x/y/z`, `rate_ref_roll/pitch/yaw`,
`duty_FR/RR/RL/FL`, etc., one row per control cycle) -- but `sf sysid fit` itself does this
alignment internally, so normally the bundle can just be passed as-is.

### Step 3: Identification

`--mixer`'s default, `legacy` (the linear X-quad mixer in `ws_internal.hpp`), already matches
this lesson's `firmware/workshop` firmware, so no explicit override is needed.

Identify all axes (indirect closed-loop method is auto-selected by default -- no `--kp` needed):

```bash
sf sysid fit logs/flight_20260911T121243.sflog.zip --plot
```

Single axis only:

```bash
sf sysid fit logs/flight_20260911T121243.sflog.zip --axis roll --plot
```

Save results to YAML:

```bash
sf sysid fit logs/flight_20260911T121243.sflog.zip -o my_plant.yaml
```

Pass a known Kp explicitly to skip the FIR auto-estimate and use that Kp:

```bash
sf sysid fit logs/flight_20260911T121243.sflog.zip --kp 0.5 --plot
```

Direct fit for logs without the 400Hz duty columns, or wherever the indirect method isn't usable
(comparison/debugging; `--mixer` must be correct for the log):

```bash
sf sysid fit logs/flight_20260911T121243.sflog.zip --input kp --kp 0.5 --plot
```

The captured bundle can be visualized with `sf log viz logs/flight_20260911T121243.sflog.zip`
(see `docs/guides/flight-log-viz.md`).

### Step 4: Compare with L6 Theory

| Axis | K (identified) | K (L6 theory) | τm (identified) | τm (theory) |
|------|---------------|---------------|-----------------|-------------|
| Roll | ? | 102.0 | ? | 0.020 |
| Pitch | ? | 70.0 | ? | 0.020 |
| Yaw | ? | 8.0 | ? | 0.020 |

Compute design Kp from identified parameters: $K_p = 1/(4\zeta^2 K \tau_m)$

**A large $\tau_m$ deviation from theory is not necessarily a failure:**
as the important note above explains, $\tau_m$ is more sensitive to
excitation than K and can land several times the theoretical 0.02s
(0.05-0.08s or so) under normal stick handling (a low-identifiability
situation where several candidate $\tau_m$ values give nearly the same
R²). Check how well K matches first, and treat a large $\tau_m$ deviation
as a reason to fly with stronger excitation and retry, not as a bug.

## 4. API

| Function | Description | Range |
|----------|-------------|-------|
| `ws::gyro_x/y/z()` | Angular rate | rad/s |
| `ws::rc_roll/pitch/yaw()` | Stick input | -1.0 to +1.0 |
| `ws::rc_throttle()` | Throttle | 0.0 to 1.0 |
| `ws::set_rate_target(r,p,y)` | Record the rate target into the Data Stream (logging only, no effect on control) | rad/s |
| `ws::motor_mixer(T,R,P,Y)` | Motor mixer | --- |
| `ws::led_color(r,g,b)` | LED color | 0-255 |
| `ws::set_channel(ch)` | WiFi channel | 1, 6, 11 |

## 5. Challenge

- Fly with different Kp values (e.g., 0.3, 0.7) and compare identification results
- Use `--time-range` option to analyze specific segments
- Overlay simulation response using identified K, τm with measured data
