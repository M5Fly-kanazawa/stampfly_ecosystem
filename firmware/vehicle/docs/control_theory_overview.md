# 制御理論に立脚した設計概要 — 状態推定・モデル導出・カスケード制御・ゲイン調整

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

> StampFly `vehicle` の飛行制御を、**(1) 状態推定（ESKF）、(2) 制御対象モデルの導出、(3) カスケード制御の設計、(4) ゲイン調整の方法論** の4本柱で、制御工学の言葉で体系的にまとめた報告書。各節は実装（`sf_estimator_eskf`, `sf_controller_pid`, `sf_autotune`）と1対1に対応する。POS_HOLD 実現までの「物語」は [`poshold_journey.md`](poshold_journey.md)、ヨー軸の詳細は [`yaw_axis_model.md`](yaw_axis_model.md) を参照。

## 1. 概要

### このドキュメントについて

37g の超小型ドローンが GPS 無しの屋内で姿勢安定化から定点ホバリングまでを成立させるには、**「いま機体はどこで・どう向いているか」を推定する器（推定器）** と、**それを目標へ駆動する器（制御器）** の両輪が要る。本書は両輪の理論的骨格 ──状態推定のモデル、制御対象（プラント）の物理モデル、それを制御する多段ループの設計、そしてゲインの決め方── を、なぜそうしたのかの根拠とともに記す。

### 設計を貫く3つの同一性（Identity）原則

開発ロードマップ（`development_roadmap.md`）が掲げる、SILS（Software-in-the-Loop, 実機ファームをそのままPC上の物理シミュレータで走らせる試験）と実機を貫く3原則：

| 原則 | 意味 |
|------|------|
| **Code Identity** | SILS も実機も**同一のC++ソース**を走らせる。テスト用の書き直しをしない |
| **Param Identity** | ゲイン・フィルタ定数は単一の出所（params SSOT）から両者へ供給する |
| **Model Identity** | プラントモデルは推測でなく**実機データの同定**で決める。モデルの妥当性が SILS→実機の転送可能性を保証する |

この3原則が、後述する「SILS では成立するが実機で発散」という乖離を**意識的に扱える**枠組みを与えている。

## 2. 状態推定 — 誤差状態カルマンフィルタ（ESKF）

### 2.1 なぜ誤差状態（Error-State）か

姿勢を含む推定では、姿勢を直接 KF の状態に乗せると2つの困難が生じる：

1. **多様体の問題**：回転（クォータニオン）は4成分だが自由度は3。カルマン更新の線形和 `x + Kδ` がクォータニオンの単位ノルム拘束を破る。
2. **線形化の不正確さ**：姿勢誤差が大きいと回転の非線形性で予測共分散がずれる。

**ESKF（誤差状態KF）** はこれを、状態を **名目状態（nominal）** と **誤差状態（error）** に分けて解く：

- **名目状態** `x̂ = (p, v, q, b_g, b_a)`：非線形のまま厳密に積分（クォータニオンは積で更新しノルム保存）
- **誤差状態** `δx = (δp, δv, δθ, δb_g, δb_a) ∈ ℝ¹⁵`：原点近傍の**微小量**ゆえ線形 KF が正確。姿勢誤差 `δθ`（3次元の回転ベクトル）は最小表現で拘束問題が消える
- 更新後、誤差を名目へ**注入（inject）** してリセット：`q ← q ⊗ δq(δθ)`、`δx ← 0`

これにより「クォータニオンの拘束を保ったまま線形 KF の精度を得る」。状態定義は実装と一致（`eskf_core.hpp`）：

```
δx = [ δp(3)   位置誤差   NED [m]
       δv(3)   速度誤差   NED [m/s]
       δθ(3)   姿勢誤差   回転ベクトル [rad]
       δb_g(3) ジャイロバイアス誤差 [rad/s]
       δb_a(3) 加速度バイアス誤差   [m/s²] ]   ∈ ℝ¹⁵
```

### 2.2 プロセスモデル（予測） — IMU を入力とする運動学

IMU（加速度計・ジャイロ）を**入力**として名目状態を積分する（`predict()`）。バイアス補正後の比力 `a = a_raw − b_a`、角速度 `ω = ω_raw − b_g` を用い：

$$
\dot p = v, \qquad
\dot v = R(q)\,a + g, \qquad
\dot q = \tfrac12\, q \otimes \begin{bmatrix}0\\ \omega\end{bmatrix}, \qquad
\dot b_g = 0,\;\; \dot b_a = 0
$$

- `R(q)`：機体→NED の回転行列、`g = [0,0,+g]ᵀ`（NED, 下向き正）
- バイアスはランダムウォーク（駆動はプロセスノイズ `Q`）

**誤差状態の線形化**（連続系 `δẋ = A δx + w`）で、`A` の非ゼロブロックは4つだけ：

$$
A_{\delta p,\delta v}=I,\quad
A_{\delta v,\delta\theta}=-R[a\times],\quad
A_{\delta v,\delta b_a}=-R,\quad
A_{\delta\theta,\delta b_g}=-I
$$

ここで `[a×]` は比力ベクトルの歪対称行列（外積を行列で表したもの）。物理的意味：

- `A_{δv,δθ} = −R[a×]`：**姿勢が傾いていると比力が誤った方向にNEDへ投影され速度を汚す** ←§2.5 の比力の罠の線形化版
- `A_{δθ,δb_g} = −I`：ジャイロバイアスがそのまま姿勢ドリフトになる

離散化は `F = I + A·dt`。共分散伝搬 `P' = F P Fᵀ + Q` を、`A` が **225要素中24要素しか非ゼロでない疎構造**を使って2パスのインプレース演算に落とす（密な 2×15³≈6750 積和 → ~720 積和）。これは飾りではなく **400Hz でコアを飽和させないための必須最適化**であり、実機の「INIT で停止」バグの根治策だった（密ループ × `-Og` で StateTask が飢餓）。

### 2.3 観測モデル（更新） — 各センサの `h(x)` と `H`

各観測は `z = h(x) + ν`、`ν ~ N(0, R)`。イノベーション `r = z − h(x̂)`、ヤコビアン `H = ∂h/∂δx`、カルマンゲイン `K = P Hᵀ (H P Hᵀ + R)⁻¹` で更新する。更新は**ジョセフ形** `P' = (I−KH)P(I−KH)ᵀ + KRKᵀ` で行い、共分散の対称・正定値性を数値的に守る。

| 観測 | センサ | `h(x)`（観測する量） | 種別 |
|------|--------|---------------------|------|
| **高度** | ToF (VL53, 30Hz) | `−p_z`（傾き補正・`tof_tilt_threshold` 超で棄却） | スカラー |
| **鉛直速度** | ToF 微分 | `−v_z` | スカラー |
| **水平速度** | オプティカルフロー (PMW3901) | フロー変位 → 機体水平速度（ジャイロ回転成分を除去・高さでスケール） | スカラー×2 |
| **姿勢（重力基準）** | 加速度計 | `R(q)ᵀ(−g)` ＝ 機体で見た重力方向（§2.5） | 3次元 |
| 気圧高度 | BMP280 | `−p_z`（既定 off, ToF 優先） | スカラー |
| 地磁気方位 | 磁気センサ | `R(q)ᵀ m_ref`（既定 off, 校正後のみ） | 3次元 |

**鉛直は ToF のみ**を一次情報源とする設計判断（baro 却下）。理由は、baro を有効化すると hover 中の気圧ドリフトで高度がランウェイする SILS 事象を観測したため。フローは加速度計と**独立**な水平速度源で、これが §2.5 の運動加速度補償の鍵になる。

### 2.4 ロバスト化の4つの仕掛け

教科書的 ESKF をそのまま積むと実機で破綻する。**「無関係なセンサの異常が、クロス共分散を通じて全状態を汚す」**のが KF の弱点で、これを有界化する4機構を入れた：

| 機構 | 何をするか | なぜ要るか |
|------|-----------|-----------|
| **χ² 外れ値ゲート** | 3次元観測で正規化イノベーション二乗 `rᵀS⁻¹r` が χ²(3,0.95)=7.81 を超えたら**棄却** | 衝撃・外れ値・一時的な不整合が状態を蹴飛ばすのを防ぐ |
| **適応 R** | accel 姿勢更新で `R ← R·(1 + k·|a−g|²)`（k=10）。**運動加速度が大きいほど信用を下げる** | ホバー中は信用し、マニューバ中は弱める（ハードゲートでなく連続減重） |
| **active_mask による P 隔離** | off のセンサ群が触る状態は P 行列で**隔離**し、観測のたびに復活しないようにする | 観測 ON/OFF が共分散構造を壊さない（旧 ESKF の構造的欠陥の根治） |
| **ジャイロバイアス偏差クランプ** | `b_g` を起動校正値の **±0.03 rad/s** 内にのみ更新許可（PX4流） | 磁気外乱・悪い床のフローがレートループに使うバイアスを引きずるのを有界化。予算: BMI270 温度ドリフト ~0.01rad/s に対し3倍の余裕 |

**設計思想**：可観測性のために全観測をバイアスへ結合させる（クロス共分散）が、その結合が同時に汚染経路にもなる。だから**結合は保ちつつ被害を有界化する**（クランプ・減重・隔離）。これが「弱めるのでなく直す／有界化する」という本プロジェクト一貫の方針。

### 2.5 比力の罠と運動加速度補償（α-β トラッカ）

加速度計が測るのは加速度でなく **比力（specific force）** `f = a_kin − g`。素の姿勢更新は `a_kin = 0`（ホバー）を仮定して `h(x) = R(q)ᵀ(−g)` とするが、**水平加速中は `a_kin ≠ 0`** で「見かけの重力」`atan(a_kin/g)` に推定が張り付き、POS_HOLD が正のフィードバックで飛び去る（詳細は [`poshold_journey.md`](poshold_journey.md) §3）。

対策は、**フロー速度（加速度計と独立）から `a_kin` を推定して比力予測に織り込む**：

$$
h(x) = g_\text{expected} + R(q)^{\mathsf T} a_\text{kin}
\quad\Rightarrow\quad
r = f_\text{meas} - h(x) = (\text{真の姿勢誤差のみ})
$$

`a_kin` は **α-β フィルタ**（速度＋加速度の2状態追跡器）でフロー速度から得る：

$$
v_\text{pred} = v + a\,dt,\quad
r = v_\text{flow}-v_\text{pred},\quad
v \mathrel{+}= \alpha r,\quad
a \mathrel{+}= \tfrac{\beta}{dt} r
$$

α-β は残差を**加速度状態に積分**するので、単純微分（高域通過）が持続加速度（DC）を washout するのと違い、**バイアスの真因である持続ドリフト加速度を捉える**（`β=0.02` でノイズに強く）。ホバーでは `a_kin≈0` に収束し素の更新へ自然退化する。

## 3. 制御対象（プラント）モデルの導出

### 3.1 角速度（レート）軸のモデル ── 物理からの導出

最内ループが制御する**角速度応答**を、物理から導く。ロール/ピッチは推力差動でトルクを作る：

1. **モータ/ESC の動特性**：指令から実推力（=モータ角速度²）までは1次遅れ。時定数 `T = τ_m`（全軸共通、同じモータ）。
2. **むだ時間**：センサ→処理→駆動の遅れ `L`（〜5ms、全軸共通）。
3. **剛体の積分**：トルク → 角加速度 → 角速度は `1/(I s)`。

これらを直列にして、**指令 → 角速度**の伝達関数：

$$
\boxed{\;G_\text{rate}(s) = \frac{b\,e^{-Ls}}{s\,(T s + 1)}\;}
\qquad (\text{roll / pitch})
$$

- 積分器 `1/s`（剛体）＋ 1次極 `1/(Ts+1)`（モータ）＋ むだ時間 `e^{-Ls}`
- `b`：実効ゲイン（推力差動の強さ ÷ 慣性）

### 3.2 ヨー軸の反トルク零点

ヨーは生成機構が異なり、**抗力トルク**（モータ速度に比例・定常）に加えて**反トルク**（モータ角加速度 `ω̇` に比例・瞬時、角運動量保存の蹴り返し）が乗る。これが分子に零点を生む：

$$
G_\text{yaw}(s) = \frac{b\,(1+\tau_z s)\,e^{-Ls}}{s\,(T s + 1)},
\qquad \tau_z = \frac{I_r}{2 k_Q \omega_0}
$$

物理計算（計測パラメータから独立検証）で **`τ_z/T = 1 + (K_m²/R_m + D_m)/(2C_q ω_0) = 3.0`（慣性に無関係）** ＝零点は常に極の3倍低域＝**最小位相（LHP）の位相リード**。交差周波数 2.9Hz で +20.5°リードとなり、フライトデータの +22〜32° と一致。**「ヨーだけ反トルク零点・ゲイン `b` は roll/pitch の約1/4〜1/5」**が要点（詳細・同定の限界は [`yaw_axis_model.md`](yaw_axis_model.md)）。

### 3.3 推力・ミキサ・モータ曲線

- **総推力 → 各モータ**：`B⁻¹` ミキサ（`actuator.cpp`）が総推力＋3軸トルクを4モータの推力配分へ逆変換し、モータ曲線で duty に変換する。物理量（総推力 [N], トルク [Nm]）で制御し、duty 変換は最終段に閉じ込める。
- **ホバー推力補正**：`hover_thrust = mg × 1.12`。1.12 は**飛行実測**の補正係数（モータ曲線が実機で推力を約12%過大に見積もる。複数ログで 1.11〜1.13 と安定）。
- **トルク効き 0.4〜0.7倍**：実機モータが理論トルクの0.4〜0.7倍しか出さない**ハード事実**。これがレートゲインを実効的に下げ、§5・[`poshold_journey.md`](poshold_journey.md) §4 の「実機で姿勢ループ権限が頭打ち」の根。

### 3.4 Model Identity ── 同定でモデルを決める

上記の `(b, T, L, τ_z)` は推測でなく**実飛行データから同定**する（§5）。SILS のプラントはこのモデルの理想版（むだ時間・トルク効き 1.0・理想フロー）であり、**SILS と実機の差はこのモデルパラメータの差として理解できる**。これが Model Identity の実体である。

## 4. カスケード制御の設計

### 4.1 ループ階層と時間スケール分離

位置から最終のモータ配分まで、**外側ほど遅く・内側ほど速い**多段カスケードを組む（`pid_controller.cpp::compute()`、INV-1 で全モード単一パイプライン）：

```
[位置ループ]  pos_setpoint − p   →(P)→  v_sp      （NED 水平・最も遅い）
     │
[速度ループ]  v_sp − v          →(PID)→ a_sp      （NED 水平加速度）
     │  a ≈ g·tilt の写像 ＋ ヨー回転で機体座標へ
[姿勢ループ]  tilt_sp − θ        →(PID)→ ω_sp      （角速度目標）
     │
[レートループ] ω_sp − ω         →(PID)→ τ          （3軸トルク）
     │
[ミキサ B⁻¹]  (総推力, τ)        →        4×duty
```

鉛直は別系統で `[高度ループ] → [鉛直速度ループ] → 推力補正 + hover_thrust`。**時間スケール分離**（外/内ループの帯域比）を保つことが安定の条件で、これが崩れると発散する（§5・実機 POS_HOLD の真因）。

### 4.2 各ループの制御則と根拠

| ループ | 制御則 | 設計上の要点 |
|--------|--------|-------------|
| **レート（最内）** | PID, **D項は測定値微分（D-on-measurement）** | 目標ステップで微分キックを出さない。出力は**物理トルク [Nm]** にクランプし積分器も同じ上限でゲート（アンチワインドアップ）。上限は幾何最大トルク以下＝差動トルクで総推力を枯渇させない |
| **姿勢（角度）** | P（または PID）→ 角速度目標 | 出力上限 `max_att_rate_sp = 3.0 rad/s` は ACRO スティックスケール(1.0)とは**別物**：30°誤差×kp=5 は 2.6rad/s を要求するので 1.0 で切ると STABILIZE 復元が死ぬ |
| **鉛直速度** | PI → 推力補正 | 出力を `±0.15N` にクランプ（実績ゲインがこの飽和と組で調整・ホバー推力偏りの積分引き込みも抑制） |
| **高度** | P → 鉛直速度目標 | 離陸時は速度を `±takeoff_climb_rate` にクランプし**目標近傍で減速→捕捉**（オーバーシュート無し） |
| **水平速度** | PID → 水平加速度 | POS_HOLD の内ループ。トルク効き不足を補うため `vel.kp` を高めに（§5・代償はフラフラ） |
| **位置（最外）** | P → 水平速度目標 | POS_HOLD の外ループ。**スティックを倒すと速度指令に切替（位置ループ迂回）し、離すと現在位置を再捕捉**（スティック速度リポジショニング＝倒して動かし、離して保持） |

### 4.3 加速度↔傾きの写像

水平位置制御の心臓は、**水平加速度を傾き角に翻訳する**小角近似（`computePositionHold`）：

$$
a_\text{horiz} \approx g\,\theta
\quad\Rightarrow\quad
\theta_\text{pitch} = -\frac{a_x^\text{body}}{g},\quad
\theta_\text{roll} = +\frac{a_y^\text{body}}{g}
$$

NED で計算した目標加速度をヨーで機体座標へ回し、`÷g` で傾き目標に変換、`±10°` にクランプ（小傾きを保つことで `|f|≈g` が成り立ち accel 姿勢更新の前提も守られる）。**この `g` という係数が、実効「傾き→速度」ゲイン K と理想 g のズレ（実機 K≈0.4g）として §5 の発散に直結する。**

### 4.4 アーキテクチャ不変条件（INV）

場当たり的な並列パッチを防ぐため、設計文書が**不変条件**を定義し全変更で照合する：

- **INV-1**：全鉛直フェーズ（Grounded/離陸/空中/着陸）が**単一の姿勢+レートパイプライン**を通る。フェーズが変えてよいのは鉛直チャネル（推力/上昇/降下）と自身の脱出条件のみ。フェーズ別の制御関数を持たない
- **INV-2**：リンク生存中はパイロットが姿勢を保つ。水平化は通信途絶（設定点の陳腐化）時のみ
- **姿勢トリム**：平衡傾きを**全モード共通の1点**で角度目標に加算（CG オフセット等の定常ドリフトを推力を余分に食わず打ち消す）。ホバー限定の常時オンボード学習で自己トリム

## 5. ゲイン調整の方法論

### 5.1 周波数領域でのシステム同定

ゲインを**勘でなく同定された伝達関数から**決める。手段は2系統：

**(a) オフライン同定（`sf sysid`）**：飛行中に1軸のレート目標へ**チャープ／ダブレット**を加算励振し、ログから周波数応答 `G(jω)` を **ETFE（経験的伝達関数推定）** で求め、`b·e^{-Ls}/(s(Ts+1))` 形でフィットする。入力 `u` は実トルクを厳密に再構成する。

**(b) オンボード自動チューン（`autotune`, `sf_autotune`）**：飛行中に**ステップドサイン**（各周波数の正弦波を整定後にロックイン）を掃引し、各点で `(u=実トルク, y=ジャイロ)` の **I/Q 相関**を積算して `G(jω)` を1点ずつ測る。整定の過渡は捨てる。

### 5.2 ロックインのロバスト化

実飛行は外乱・雑音だらけ。autotune はこれに対し：

- **コヒーレンス／SNR ゲート**：**オフ音**（励振していない近傍周波数）でジャイロ I/Q を積算して**雑音床**を測り、`coh = on/(on+off)` でフィットの重みを下げる
- **除トレンド**：近DC外乱（CW/CCW ヨートリム）をレートの遅い走査平均で差し引いてからロックイン

### 5.3 ループ整形 ── PM/ωc 仕様を PID 形で解く

同定した `G(jω)` に対し、**目標の交差周波数 ωc と位相余裕 PM** を満たす PID ゲインを解く（`tunePid`）。PID 位相は Td に対し非単調になりうるため、二分法でなく**ピーク走査**で安定に解く（過去のバグ：二分法が上端で誤解）。設計後、`evalMargins` で実際のゲイン余裕 GM・位相余裕 PM を評価し、**安全ゲート**を通してからライブ適用（NVS 非保存）：

| ゲート | 基準 | 意図 |
|--------|------|------|
| 位相余裕 PM | ≥ 規定値 | 不安定設計を弾く |
| ゲイン余裕 GM | ≥ 6 dB（全軸統一） | ヨーの旧8dBは未モデル零点の保守値→零点モデル化で6dBに |
| ヨー ωc 上限 | `0.3/τ_z` | 非最小位相の帯域限界を避ける（リードの場合は保守的余裕） |

### 5.4 SILS 対実機 ── 同定の決定的役割

最大の教訓は **「SILS で最適化したゲインが実機で発振する」**こと。SILS のプラントは理想（むだ時間小・トルク効き1.0）で、SILS 乱流ベンチを直接最適化すると**実機で位相余裕が負になるゲイン**（例: Td=0.08 で実機 PM −375°）に収束する。

→ **必ず実機同定したプラント上でループ整形する**。ACRO レート発振の解決（実遅れ12-16ms・トルク効き0.36-0.71倍を同定→再設計で実機の振動峰消失、roll PM57°/pitch59° を飛行ログで裏付け）、POS_HOLD の実機再設計（実効ゲイン K≈0.4g を3手法で同定→`vel.kp 0.8→3.0`/`pos.kp 1.0→0.4` で発散停止）は、いずれも**同定がゲイン決定を駆動した**。

### 5.5 制御パラメータ変更の鉄則

> **制御系パラメータ（PIDゲイン・フィルタ定数・リミット）の変更提案は、必ず実フライトログを使った数値シミュレーションで効果を定量確認してから行う。** 「Ti を短くすれば改善する」式の定性推測だけで提案しない。シミュレーションの結果、逆効果なら提案しない。

これは推測の混入を防ぐプロジェクトの規律で、§5.1–5.4 の同定駆動アプローチと一体である。

## 6. まとめ ── この設計の特徴

| 観点 | 本設計の立場 |
|------|-------------|
| **推定** | 教科書 ESKF に留まらず、χ²・適応R・P隔離・バイアスクランプ・運動加速度補償の**ロバスト化層**で実機の汚染経路を有界化 |
| **モデル** | プラントは推測でなく**実機同定**（Model Identity）。SILS と実機の乖離をモデルパラメータ差として扱える |
| **制御** | 多段カスケードの**時間スケール分離**を、非力なモータ（トルク効き0.4〜0.7倍）という制約下で成立させる。INV で並列パッチを排除 |
| **調整** | ゲインは**同定された伝達関数からループ整形**で決め、安全ゲートを通す。SILS でなく**実機プラントで設計**する規律 |

理論（ESKF・伝達関数・ループ整形）と実機の物理（比力・トルク効き・むだ時間）を**同定で橋渡し**する ── これが本ファームの制御設計を貫く一本の筋である。

---

<a id="english"></a>

# Control-Theory Design Overview — Estimation, Modelling, Cascade Control, Tuning

> A systematic, control-engineering account of the StampFly `vehicle` flight control across four pillars: **(1) state estimation (ESKF), (2) plant-model derivation, (3) cascade-control design, (4) gain-tuning methodology.** Each section maps 1:1 to the implementation (`sf_estimator_eskf`, `sf_controller_pid`, `sf_autotune`). For the POS_HOLD "story" see [`poshold_journey.md`](poshold_journey.md); for the yaw axis see [`yaw_axis_model.md`](yaw_axis_model.md).

## 1. Overview

For a 37 g drone to go from attitude stabilisation to fixed-point hover indoors without GPS, it needs both an **estimator** ("where am I and how am I oriented") and a **controller** ("drive that toward the target"). This report records the theoretical skeleton of both — the estimation model, the plant physics, the multi-loop design, and how the gains are chosen — with the rationale for each.

### The three Identity principles

From `development_roadmap.md`, spanning SILS (the firmware compiled and run on a PC physics simulator) and hardware:

| Principle | Meaning |
|-----------|---------|
| **Code Identity** | SILS and hardware run the SAME C++ source — no test rewrites |
| **Param Identity** | gains/filter constants come from one source (params SSOT) to both |
| **Model Identity** | the plant model is IDENTIFIED from real data, not guessed — its validity is what makes SILS→hardware transfer trustworthy |

These let us consciously handle the "passes in SILS, diverges on hardware" gap discussed in §5.

## 2. State Estimation — Error-State Kalman Filter (ESKF)

### 2.1 Why an error state

Putting attitude directly in the KF state has two problems: the quaternion's 4 components carry 3 DOF (the linear update `x+Kδ` breaks the unit-norm constraint), and large attitude errors make the linearised covariance inaccurate. The **ESKF** splits the state into a **nominal** part `x̂=(p,v,q,b_g,b_a)` (integrated exactly, nonlinear, quaternion-product update preserving norm) and an **error** part `δx=(δp,δv,δθ,δb_g,δb_a)∈ℝ¹⁵` (a small quantity near the origin where a linear KF is accurate; the 3-vector attitude error `δθ` is a minimal representation with no constraint). After each update the error is **injected** into the nominal (`q ← q⊗δq(δθ)`) and reset to zero — getting linear-KF accuracy while preserving the quaternion constraint.

### 2.2 Process model (predict) — IMU as input

The IMU drives the nominal integration (`predict()`), with bias-corrected specific force `a=a_raw−b_a` and rate `ω=ω_raw−b_g`:

$$
\dot p=v,\quad \dot v=R(q)a+g,\quad \dot q=\tfrac12 q\otimes[0,\omega]^{\mathsf T},\quad \dot b_g=\dot b_a=0
$$

The error-state dynamics `δẋ=Aδx+w` have only four nonzero blocks:
$A_{\delta p,\delta v}=I$, $A_{\delta v,\delta\theta}=-R[a\times]$, $A_{\delta v,\delta b_a}=-R$, $A_{\delta\theta,\delta b_g}=-I$ — where `-R[a×]` is the linearised "a tilt mis-projects the specific force into NED and corrupts velocity" (the linearised form of the §2.5 specific-force trap). Discretised as `F=I+A·dt`, the covariance propagation `P'=FPFᵀ+Q` exploits `A`'s sparsity (24 of 225 entries) to become two in-place passes (~720 vs the dense ~6750 multiply-adds) — not cosmetic but **required to not saturate a core at 400 Hz**, and the fix for the real "stuck at INIT" bug (a dense loop at `-Og` starved StateTask).

### 2.3 Measurement models (update)

Each observation `z=h(x)+ν`, with innovation `r=z−h(x̂)`, Jacobian `H=∂h/∂δx`, gain `K=PHᵀ(HPHᵀ+R)⁻¹`, updated in **Joseph form** `P'=(I−KH)P(I−KH)ᵀ+KRKᵀ` for numerical symmetry/positive-definiteness.

| Observation | Sensor | `h(x)` | Type |
|-------------|--------|--------|------|
| **Altitude** | ToF (VL53, 30 Hz) | `−p_z` (tilt-compensated; rejected above `tof_tilt_threshold`) | scalar |
| **Vertical velocity** | ToF derivative | `−v_z` | scalar |
| **Horizontal velocity** | optical flow (PMW3901) | flow → body horizontal velocity (gyro-rotation removed, height-scaled) | scalar×2 |
| **Attitude (gravity ref)** | accelerometer | `R(q)ᵀ(−g)` = gravity direction in body (§2.5) | 3-D |
| Baro altitude | BMP280 | `−p_z` (default off; ToF preferred) | scalar |
| Heading | magnetometer | `R(q)ᵀ m_ref` (default off; only if calibrated) | 3-D |

**Vertical uses ToF only** as the primary source (baro rejected: enabling it produced a SILS altitude runaway from pressure drift in hover). Flow is a horizontal-velocity source **independent of the accelerometer** — the key to §2.5.

### 2.4 Four robustness mechanisms

A textbook ESKF breaks on hardware: the KF's weakness is that **a misbehaving "irrelevant" sensor drags every state through the cross-covariances.** Four mechanisms bound the damage:

| Mechanism | What it does | Why |
|-----------|--------------|-----|
| **χ² outlier gate** | reject a 3-D update if `rᵀS⁻¹r > χ²(3,0.95)=7.81` | stops shocks/outliers from kicking the state |
| **Adaptive R** | accel-attitude `R ← R·(1+k·|a−g|²)` (k=10): trust less the larger the kinematic accel | trust at hover, down-weight in maneuvers (continuous, not a hard gate) |
| **active_mask P-isolation** | states touched by OFF sensors are isolated in P, not revived each update | observation ON/OFF can't corrupt the covariance structure |
| **Gyro-bias deviation clamp** | `b_g` may move only ±0.03 rad/s from the boot calibration (PX4-style) | bounds a contaminated rate-feedback bias; 3× headroom over BMI270 thermal drift |

**Design philosophy:** the coupling that makes the bias observable is also the contamination path, so **keep the coupling but bound the damage** — the project's consistent "don't weaken, correct/bound" stance.

### 2.5 The specific-force trap and acceleration compensation (α-β tracker)

The accelerometer measures **specific force** `f=a_kin−g`. The plain attitude update assumes `a_kin=0` (hover), but accelerating horizontally (`a_kin≠0`) sticks the estimate at the "apparent gravity" angle `atan(a_kin/g)` and POS_HOLD flies away by positive feedback ([`poshold_journey.md`](poshold_journey.md) §3). The fix estimates `a_kin` from the flow velocity (independent of the accelerometer) and folds it into the prediction:

$$
h(x)=g_\text{expected}+R(q)^{\mathsf T}a_\text{kin}\ \Rightarrow\ r=f_\text{meas}-h(x)=(\text{true attitude error only})
$$

`a_kin` comes from an **α-β filter** on the flow velocity ($v\mathrel{+}=\alpha r$, $a\mathrel{+}=\tfrac{\beta}{dt}r$): integrating the residual into the acceleration state captures the SUSTAINED drift acceleration (the bias's real source) instead of washing out the DC like a naive derivative; `β=0.02` rejects noise; at hover it degenerates to the plain update.

## 3. Plant-Model Derivation

### 3.1 The rate axis — derived from physics

Roll/pitch make torque by thrust differential. Series-connect the motor/ESC first-order lag (time constant `T`, common to all axes), the transport delay `L` (~5 ms), and the rigid-body integrator `1/(Is)`:

$$
\boxed{G_\text{rate}(s)=\frac{b\,e^{-Ls}}{s(Ts+1)}}\quad(\text{roll/pitch})
$$

integrator + one motor pole + dead time; `b` = effective gain (thrust differential ÷ inertia).

### 3.2 The yaw reaction-torque zero

Yaw adds a **reaction torque** (∝ motor angular acceleration, instantaneous) to the drag torque, producing a numerator zero:

$$
G_\text{yaw}(s)=\frac{b(1+\tau_z s)e^{-Ls}}{s(Ts+1)},\quad \tau_z=\frac{I_r}{2k_Q\omega_0}
$$

A physics check gives `τ_z/T = 3.0` (inertia-independent) — the zero is always 3× below the pole, a minimum-phase (LHP) phase LEAD (+20.5° at the 2.9 Hz crossover, matching the +22–32° in flight data). Key point: **only yaw has the reaction zero, and its gain `b` is ~1/4–1/5 of roll/pitch** ([`yaw_axis_model.md`](yaw_axis_model.md)).

### 3.3 Thrust, mixer, motor curve

Total thrust → per-motor via the `B⁻¹` mixer (`actuator.cpp`), which inverts total-thrust + 3-axis torque to four motor thrusts and a duty via the motor curve — control is done in physical units ([N],[Nm]) with duty confined to the last stage. **Hover thrust** = `mg×1.12`, where 1.12 is a flight-measured correction (the curve over-promises thrust ~12% on hardware, stable 1.11–1.13 across logs). **Torque effectiveness 0.4–0.7×** is the hardware fact that effectively lowers the rate gain and caps the attitude-loop authority (§5, [`poshold_journey.md`](poshold_journey.md) §4).

### 3.4 Model Identity

`(b,T,L,τ_z)` are IDENTIFIED from real flight (§5), not guessed. The SILS plant is the ideal version (small delay, torque effectiveness 1.0, ideal flow), so **the SILS-vs-hardware difference is exactly a difference in these model parameters** — that is what Model Identity means in practice.

## 4. Cascade-Control Design

### 4.1 Loop hierarchy and time-scale separation

From position to motor mix, outer loops slow and inner loops fast (`compute()`, one pipeline for all modes by INV-1):

```
[position]  pos_sp − p  →(P)→  v_sp     (NED horiz, slowest)
[velocity]  v_sp − v     →(PID)→ a_sp
   a ≈ g·tilt mapping + yaw rotation to body
[attitude]  tilt_sp − θ  →(PID)→ ω_sp
[rate]      ω_sp − ω     →(PID)→ τ        (3-axis torque)
[mixer B⁻¹] (thrust, τ)  →        4×duty
```

Vertical is a parallel chain `[altitude]→[vertical-velocity]→thrust_correction + hover_thrust`. Preserving **time-scale separation** (the outer/inner bandwidth ratio) is the stability condition — break it and it diverges (§5, the real POS_HOLD root cause).

### 4.2 Each loop's law and rationale

| Loop | Law | Key point |
|------|-----|-----------|
| **Rate (innermost)** | PID, **D-on-measurement** | no derivative kick on a setpoint step; output clamped to physical torque [Nm], integrator gated to the same limit (anti-windup), set below the geometric max so differential torque can't starve collective |
| **Attitude (angle)** | P(ID) → rate sp | output limit `max_att_rate_sp=3.0 rad/s` is DISTINCT from the ACRO stick scale (1.0): a 30° error × kp=5 wants 2.6 rad/s, so clamping at 1.0 would cripple STABILIZE recovery |
| **Vertical velocity** | PI → thrust correction | output clamped to `±0.15 N` (the proven gains were tuned against this saturation; also bounds the hover-bias integrator pull) |
| **Altitude** | P → vertical-velocity sp | on takeoff, velocity clamped to `±takeoff_climb_rate` so it DECELERATES near the target and CAPTURES it (no overshoot) |
| **Horizontal velocity** | PID → horizontal accel | POS_HOLD inner loop; `vel.kp` raised to offset weak torque (§5; the cost is the wobble) |
| **Position (outermost)** | P → horizontal-velocity sp | POS_HOLD outer loop; a deflected stick switches to a velocity command (bypassing the position loop) and releasing re-captures the current position (velocity-command repositioning — deflect to move, release to hold) |

### 4.3 The acceleration↔tilt mapping

The heart of horizontal control is the small-angle translation of horizontal acceleration into tilt (`computePositionHold`):

$$
a_\text{horiz}\approx g\theta\ \Rightarrow\ \theta_\text{pitch}=-\frac{a_x^\text{body}}{g},\ \theta_\text{roll}=+\frac{a_y^\text{body}}{g}
$$

The NED accel is rotated to body by yaw, divided by `g`, and clamped to ±10° (small tilt keeps `|f|≈g`, also honoring the accel-attitude assumption). **This `g` factor is exactly where the real effective "tilt→velocity" gain K (≈0.4 g on hardware) departs from the ideal g and drives the §5 divergence.**

### 4.4 Architectural invariants (INV)

To prevent ad-hoc parallel patches, the design defines invariants checked on every change: **INV-1** — all vertical phases share ONE attitude+rate pipeline (a phase may change only the vertical channel and its own exit condition; no per-phase control function); **INV-2** — the pilot keeps attitude while the link is live, leveling only on comm-loss (stale setpoints); plus an **attitude trim** added at a single confluence for every mode (cancelling steady drift with no extra thrust), self-learned hover-gated.

## 5. Gain-Tuning Methodology

### 5.1 Frequency-domain identification

Gains come from an IDENTIFIED transfer function, not intuition. Two paths: **(a) offline (`sf sysid`)** — chirp/doublet excitation added to one axis' rate setpoint, ETFE of the logged response, fit to `b·e^{-Ls}/(s(Ts+1))` with `u` reconstructed as the exact torque; **(b) onboard autotune (`sf_autotune`)** — a stepped-sine sweep that lock-in-measures `G(jω)` one point at a time via I/Q correlation of `(u=actual torque, y=gyro)`, discarding the settle transient.

### 5.2 Robust lock-in

Real flight is full of disturbance/noise, so autotune adds a **coherence/SNR gate** (accumulate gyro I/Q at an OFF-tone to measure the noise floor; `coh=on/(on+off)` down-weights the fit) and **detrending** (subtract a slow running mean of the rate — the near-DC CW/CCW yaw trim — before lock-in).

### 5.3 Loop shaping to PM/ωc specs

Given `G(jω)`, solve the PID gains meeting a target crossover ωc and phase margin PM (`tunePid`). Since the PID phase can be non-monotonic in Td, solve by a peak scan, not bisection (a past bug: bisection misread the upper end). Then `evalMargins` evaluates the achieved GM/PM and a **safety gate** must pass before live (non-persisted) application: PM ≥ spec; GM ≥ 6 dB (unified across axes; yaw's old 8 dB was a conservative pad for the unmodelled zero); yaw ωc cap `0.3/τ_z` to stay clear of the bandwidth limit.

### 5.4 SILS vs hardware — identification is decisive

The biggest lesson: **gains optimised in SILS oscillate on hardware.** The SILS plant is ideal (small delay, torque 1.0), so directly optimising a SILS turbulence bench converges to gains with NEGATIVE phase margin on hardware (e.g. Td=0.08 → real PM −375°). So **always loop-shape on the hardware-identified plant.** Both the ACRO rate-oscillation fix (identified 12–16 ms delay, 0.36–0.71× torque → redesign removed the vibration peak, roll PM 57°/pitch 59° confirmed from flight logs) and the POS_HOLD hardware redesign (K≈0.4 g identified by 3 methods → `vel.kp 0.8→3.0`/`pos.kp 1.0→0.4` stopped the divergence) were **driven by identification.**

### 5.5 The rule for control-parameter changes

> **Every proposed change to a control parameter (PID gain, filter constant, limit) must be quantitatively confirmed by a numerical simulation on real flight-log data BEFORE it is proposed.** No qualitative guessing ("shorten Ti and it'll improve"). If the simulation shows it backfires, it is not proposed.

This discipline against guesswork is of a piece with the identification-driven approach of §5.1–5.4.

## 6. Summary — what's distinctive

| Aspect | This design's stance |
|--------|----------------------|
| **Estimation** | beyond a textbook ESKF: a robustness layer (χ², adaptive R, P-isolation, bias clamp, acceleration compensation) bounds the hardware contamination paths |
| **Modelling** | the plant is IDENTIFIED, not guessed (Model Identity); the SILS-hardware gap is a model-parameter difference |
| **Control** | a multi-loop cascade with time-scale separation made to work under weak motors (0.4–0.7× torque); INV rules out parallel patches |
| **Tuning** | gains come from loop-shaping an IDENTIFIED transfer function through safety gates, designed on the HARDWARE plant, not SILS |

Bridging theory (ESKF, transfer functions, loop shaping) and hardware physics (specific force, torque effectiveness, dead time) **through identification** — that is the single thread running through this firmware's control design.
