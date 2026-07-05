# POS_HOLD 実現までの技術的概要 — 何がすごく、何が大変だったか

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

> 37g の超小型ドローン StampFly が、GPS もモーションキャプチャも使わず、オプティカルフロー＋ToF＋IMU だけで **手放し定点ホバリング（±6〜7cm, RMS 16mm）** を成立させるまでの記録。詳細な根本原因と数値は [`poshold_accel_compensation.md`](poshold_accel_compensation.md)（運動加速度補償）と [`poshold_wobble_velocity_noise.md`](poshold_wobble_velocity_noise.md)（残留フラフラ）を参照。本書はその全体像をまとめた概要である。

## 1. 概要

### このドキュメントについて

vehicle の **POS_HOLD（位置保持）** を成立させるまでに越えた2つの大きな壁 ──「加速度センサの比力の罠」と「SIL と実機の乖離」── を、物理・制御工学の言葉で順に説明する。なぜ難しいのか、何を試して何が効かなかったのか、最終的にどう成立させたのか、そして何が残っているのかを俯瞰する。

### 対象読者

ドローンの姿勢推定・位置制御・SIL テストに関わる人。GPS の無い屋内で「その場に張り付くホバリング」がなぜ難しいのかを学びたい人。制御工学・ハードウェアの素養を前提とし、ソフトウェア固有の用語には補足を添える。

### 一文で言うと

> **POS_HOLD は「位置→速度→姿勢→レート」の4段カスケードを全部成立させる到達点であり、その途中で (1) 加速度センサが水平加速中に「見かけの重力」を指す原理的問題と、(2) 理想化された SIL では再現できない実機モータの権限不足、という2つの壁に突き当たった。前者は運動加速度補償（α-β トラッカ）で根治し、後者は実機同定でプラントを作り直して設計し直すことで、定点保持 ±6〜7cm を達成した。**

## 2. そもそも POS_HOLD はなぜ難しいのか

### 多段カスケードの時間スケール分離

**POS_HOLD** は「機体を空中の一点に留め続ける」モードである。手を離しても外乱で流されず、その場に張り付く。これを実現するには多段の **カスケード**（出力が次の段の指令になる入れ子のループ）を全段成立させる必要がある。

```
位置誤差 → [位置ループ] → 速度指令 → [速度ループ] → 傾き指令（姿勢目標）
        → [姿勢ループ] → 角速度指令 → [レートループ] → モータ配分（ミキサ）
```

外側ほど遅く、内側ほど速い ── この **時間スケール分離** がカスケードの大原則であり、これが崩れると発散する。POS_HOLD が成立するということは、**この4段すべてが安定し、かつ正しい帯域配分で噛み合っている**ことの証明になる。

### この機体固有のハンデ

| ハンデ | 内容 |
|--------|------|
| **絶対位置センサが無い** | GPS 無し・屋内。位置は **オプティカルフロー（床を見るカメラ）速度の積分** と ToF 高度のみ。原理的にドリフトしやすい |
| **加速度センサの原理的限界** | 水平加速中に「重力の向き」を見失う（§3 で詳述） |
| **モータが非力** | 実機モータの **トルク効きが理論値の 0.4〜0.7 倍**。制御権限そのものが足りない（§4 で詳述） |
| **37g の軽量機** | 外乱に弱く、リミットサイクル（自励振動）を起こしやすい |

## 3. 第一の壁 — 加速度センサが「嘘の重力」を指す

### 3.1 比力の罠

加速度センサが出すのは加速度そのものではなく **比力（specific force）** `f = a_kin − g` である。

- `a_kin` … 機体の運動加速度（ニュートンの第2法則の加速度）
- `g` … 重力加速度ベクトル

ホバー中は `a_kin = 0` なので `f = −g`、つまり加速度センサは **真下（重力方向）** を指す。これによりジャイロ積分のドリフトを補正できる。これが姿勢推定の基本原理である。

ところが POS_HOLD は位置を戻すために機体を傾けて横に加速する → `a_kin ≠ 0`。すると比力は重力からズレた **「見かけの重力」** を指す。

```
        ホバー中                    水平加速中（ドリフトを止めようと傾斜）

          │ f = −g                    ╲  f = a_kin − g
          │（真下＝重力）              ╲（重力から θ 傾く＝見かけの重力）
          ▼                            ◢θ
        [機体]                       [機体]  → 横加速 a_kin

   加速度センサは正しく            加速度センサは「見かけの下」を
   重力を指す                       指す → 姿勢推定が θ だけ誤る
```

ドリフト加速度 ≈ 1.1 m/s² のとき `θ = atan(1.1/9.81) ≈ 6.4°`。推定 roll/pitch が ±6.5° に **張り付く** 現象として現れた。

### 3.2 正のフィードバックで飛び去る

最悪なのは、これが **正のフィードバックループ** を作ること。

```
推定が「ほぼ水平」と誤認 → 制御は「もっと傾けないと」と判断
   → 実機はさらに傾く → さらに加速 → さらに見かけの重力がズレる
      → 推定はますます水平と誤認 …（発散）
```

ジャイロ積分は本当の傾き（真値）を追えているのに、加速度更新が毎周期「お前は水平だ」と引き戻すため推定が真値に追従できず張り付く。**旧 vehicle（87飛行の実績機）も同じ限界で「かろうじて保持」止まり**だった。

### 3.3 効かなかった対策 — 「accel を弱める」は全滅

| 案 | 考え方 | なぜダメか |
|----|--------|-----------|
| 推力補償（適応R減重） | 推力モデルで加速を検知し accel を弱める | **鶏卵問題**：減重量を汚染された推定姿勢から計算 → 推定は水平と思い込む → 減重が効かない |
| accel バイアス凍結 | 飛行中バイアス推定を止める | バイアスは無関係。自由度を奪うと更に不安定 |
| 速度ゲート減重 | フロー速度が大きい時 accel を弱める | **アンカーを外す** とジャイロ単独ではドリフトが勝ち全軸が流れる（90〜260m 飛び去り） |

**教訓：accel の重みを下げる方向は全部失敗する。** ジャイロ単独では姿勢を保てないので加速度アンカーは必須。問題は accel が **間違った方向** を指すことなので、弱めるのではなく **正しく直す** 必要がある。

### 3.4 効いた解 — α-β トラッカで運動加速度を差し引く

正攻法は、**運動加速度 `a_kin` を独立な情報源（オプティカルフロー速度）から推定して比力から引く** ことだった。

```
予測する比力  f_pred = g_expected + R^T · a_kin
イノベーション innov = f_measured − f_pred = (本当の姿勢誤差だけ)
```

フロー速度は加速度センサと独立なので循環（自分で自分を補正）にならない。ホバー中は `a_kin ≈ 0` で素の更新に一致し、加速中だけ補正が働く。

ただしここで **軸非対称バグ** が炙り出された。当初はフロー速度の単純微分で `a_kin` を作っていたところ：

| 軸 | 単純微分の結果 |
|----|---------------|
| roll | ✅ 保持 |
| **pitch** | ❌ **発散（98m）** |
| diagonal | ❌ 発散（21m） |

**コードは左右対称なのにピッチだけ発散。** 原因は単純微分（高域通過）の二重欠陥：①不規則なフロー周期で微分が ±30 m/s² に暴れる、②高域通過は **持続加速度（DC 成分）を washout** し振動しか通さない。

解決は **α-β フィルタ**（速度＋加速度の2状態追跡器）への置き換え：

```
予測:  v_pred = v_est + a_est · dt
補正:  残差 r = v_flow − v_pred
       v_est += α · r          （速度状態）
       a_est += (β/dt) · r     （加速度状態 ← これが a_kin）
```

α-β は残差を **加速度状態に積分** するので、高域微分のように DC を捨てず、**持続ドリフト加速度（バイアスの真因そのもの）を捉える**。`β=0.02` でノイズに強くし、**生フロー速度** を使う（融合速度 `vel_` だと predict で accel を再注入してピッチ軸が自己強化する）。

結果、**SIL clean で全4軸タイト保持（drift ≤ 1.1m, att_rmse ≤ 3.1°）** を達成した。

### 3.5 SIL が炙り出した方法論の教訓

この軸非対称バグは **単軸テストでは隠れていた**（当初は roll 単軸しか試していなかった）。**個別軸（pos_roll / pos_pitch / pos_diag / pos_yaw）→ 複合（pos_flight）** の網羅テスト構成と、物理真値の **数値ゲート**（`.expect` に `metric <name> <op> <value> in <t0> <t1>`）を導入して初めて、ピッチ軸の発散という重大バグが見えた。

> **教訓：単軸テストは軸非対称のバグを隠す。** 各軸を1軸ずつ試し、最後に複合で試す構成にして初めてバグが見える。

## 4. 第二の壁 — SIL では飛ぶのに実機では発散する

### 4.1 Code Identity の哲学

この開発の特徴は、**SIL（Software-in-the-Loop, 本体C++コードをそのままコンパイルして物理シミュレータ上で走らせる試験環境）** が、テスト用に書き直した別コードではなく **実機と同一のソース** を走らせること（Code Identity）。これにより SIL で見つけたバグは実機のバグと等価になる ── はずだった。

ところが POS_HOLD は **SIL では完璧に保持するのに、初の実機飛行で発散した**。これが本件の核心的教訓である。

### 4.2 実機の症状 — 成長する不安定リミットサイクル

狭い部屋・手放しで POS_HOLD →

- SIL のような即タンブルではなく、**周期約9.4秒・振幅 ±0.37→0.50→0.62m と成長する緩い水平振動**（不安定リミットサイクル, ω≈0.68 rad/s, σ≈+0.057/s）で壁に激突
- 同時刻の ALT_HOLD は単調ドリフトのみで **振動なし** → 振動は **閉じた位置ループ固有** と切り分け
- フローは健全（SQUAL 79–184）、姿勢の張り付きも無し → **第一の壁（α-β 補償）は実機でも有効**

### 4.3 真因 — 実効「傾き→速度」ゲインが 0.4g しかない

3手法（開ループ fit・振幅関係・較正）が一致して、**指令傾き→実測水平速度の実効ゲイン K ≈ 0.4g**（理想は g）と同定。内訳：

- **姿勢ループの傾き未達**（指令傾きに対し実傾きが ≈0.58 しか出ない）← 主犯
- フロー速度の過小読み（≈0.66g）

そしてその姿勢未達の根は、**実機モータのトルク効き 0.4〜0.7 倍**（ACRO 発振解決時から判明していたハード事実）。レートゲインが約 1/4 に減衰し、姿勢ループの権限が頭打ちになる。

制御理論的に重要な区別：

```
前向き経路（傾き→速度）の不足  →  内ループ(速度)帯域が外ループ(位置)より下がる
                              →  カスケード時間スケール分離が崩壊  →  不安定化（発散）
```

**K = g（理想）なら現ゲインは安定（σ −0.124）= SIL 合格・実機墜落の正体**。SIL は理想フロー（plant.cpp で SQUAL=100固定・ノイズ無し・トルク効き1.0）ゆえ、この **ハード由来の権限不足を再現できなかった**。「Code Identity でも実機で動かない」盲点の実例である。

### 4.4 対策と「ハード限界」という結論

実機で同定したプラント上で閉ループ設計をやり直し（`analysis/scripts/poshold_loop_design.py`）：

| パラメータ | 変更 | 狙い |
|-----------|------|------|
| `position.vel.kp` | 0.8 → **3.0** | 速度ループの権限回復（K 不足を内ループ強化で補償） |
| `position.pos.kp` | 1.0 → **0.4** | 外ループを遅くし時間スケール分離を回復 |

K∈[2.8,7]・τ∈[50,300ms] でロバスト安定。実機検証：

| 指標 | 結果 |
|------|------|
| 発散振動 | **完全消失** |
| 保持精度 | **±6〜7cm, RMS 16mm** |
| 定常ドリフト | 31 → 16mm に締まり（vel.kp 強化で改善） |
| 有界性 | 前半 RMS 0.028m ≈ 後半 0.031m（非成長＝有界） |

ただし重要な結論として、**att.kp を上げると 1.2Hz 共振が出るだけ、att.ti を下げても僅か** ── 姿勢ループのチューニングに伸びしろは無い。**±6〜7cm がこの37g機のトルク効きでの実用限界** で、これ以上のタイトホールドはハード（高効率モータ/プロペラ）の話。SIL は理想トルク効きゆえこの限界を再現できず「直せない」。

## 5. 仕上げ — スティックで動かせる POS モード

定点保持が完成したので、**スティック速度リポジショニング**（roll/pitch を倒すと水平速度を指令し、中立に戻すと現在位置を再捕捉して止まる方式 ＝「倒して動かし、離して保持」）のスティック操作を実装（commit `ef3f854`）：

- roll/pitch スティック → 水平速度指令（機体座標、STABILIZE の傾き方向と一致）を速度ループへ注入
- **中立に戻すと、離した位置で停止して再捕捉**（位置目標を更新して止まる）
- `position.stick_vel`（既定 0.4 m/s）でライブ調整

「定点に張り付くが、動かしたい時はスティックで動かせて、離せばまた止まる」── これで実用的な位置制御モードになった。SIL `pos_reposition`（東へ 0.70m 移動→新位置 6cm 保持）19/19 PASS、実機でも良好。INV-1（全鉛直フェーズが単一姿勢パイプラインを通る不変条件）も維持。

## 6. すごさ — 何が達成されたのか

| # | 達成 | なぜすごいか |
|---|------|------------|
| 1 | **絶対位置センサ無しで定点ホバリング** | GPS/モーキャップ無し、床カメラ＋ToF＋IMU だけで ±6〜7cm。GPS 依存の商用機とは別次元の難しさ |
| 2 | **比力の罠を正攻法で根治** | 姿勢推定の最も深い原理的問題を、減重などの対症療法でなく独立情報源からの運動加速度補償（α-β トラッカ）で解いた。旧実績機の「かろうじて保持」を越えた |
| 3 | **4段カスケードの完全成立** | 位置→速度→姿勢→レートの時間スケール分離を、非力なモータという制約下で噛み合わせた |
| 4 | **全発見が数値裏付き** | 「Ti を縮めれば直る」式の定性推測ゼロ。軸非対称バグも実効ゲイン 0.4g もフラフラ 0.43° も、すべて数値同定とログ解析で定量特定してから対策 |

## 7. 何が大変だったのか — 本質的な困難

| 困難 | 中身 |
|------|------|
| **物理の罠が制御を直撃** | 加速度センサの比力という原理が、位置制御の傾き動作と本質的に干渉する。制御だけ・推定だけでは解けず、両者をまたいで理解する必要があった |
| **対症療法が全部裏目** | 「accel を弱める」直感は鶏卵問題で全滅。**正しく直す** しかなく、独立情報源（フロー）という発想転換が必要だった |
| **対称コードの非対称バグ** | ピッチだけ発散。単軸テストでは隠れ、軸別→複合の網羅テスト＋数値ゲートを作って初めて見えた |
| **SIL と実機の乖離** | SIL 完璧→実機発散。理想フロー・理想トルクの SIL は **ハード由来の権限不足を再現しない**。実機同定でプラントを作り直して設計し直す必要があった |
| **チューニングでは越えられない壁** | 最後はモータのトルク効き 0.4〜0.7 倍というハード限界に突き当たる。ソフトの伸びしろが尽きる地点を見極めること自体が難しい |

## 8. 残っている宿題 — 既知・ハード律速

POS_HOLD 成立後も残る2つの微小振動。**どちらもハードが律速で、ループ調整は底**：

| # | 症状 | 真因 | 対策の方向 |
|---|------|------|-----------|
| #6 | 水平 ~1Hz のフラフラ（±0.43°） | 高 vel.kp が **フロー速度ノイズを傾き指令に増幅**（コヒーレンス 0.72）。`vel.kp·σ_v/g = 3.0×0.025/9.81 = 0.43°` | **速度ローパス**（~0.5Hz）で締まりを保ったままノイズだけ除去。位相余裕の確認必須（[`poshold_wobble_velocity_noise.md`](poshold_wobble_velocity_noise.md)） |
| #7 | 高度の長周期上下動（±6cm） | **モータ/プロペラ応答遅れ ~110ms** に対し高度ループの減衰不足（ToF 30Hz・ESKF 鉛直速度は健全・無遅れと実測済み） | `alt.kp 0.6→0.45` で 38% 緩和済。精密化には鉛直の同定飛行が必要 |

## 9. 確定パラメータ（2026-06-22 時点）

| パラメータ | 値 | 備考 |
|-----------|-----|------|
| `position.pos.kp` | 0.4 | 外ループを遅く（時間スケール分離） |
| `position.vel.kp` | 3.0 | 内ループ強化（K 不足補償） |
| `position.stick_vel` | 0.4 m/s | スティック再配置速度 |
| `attitude.*.ti` | 2.0 | パイロット好み |
| `altitude.alt.kp` | 0.45 | 高度上下動 38% 緩和 |
| `eskf.accel_comp.*` | enable 1 / alpha 0.2 / beta 0.02 / max 5.0 | 運動加速度補償 |

> ⚠️ **実機適用は WiFi で `param set` → `param save` が必須。** トリム学習器が着陸ごとに自動 save するため、再フラッシュ単独では旧値（NVS）で上書きされる。詳細は [`../README`](../) 系の param SSOT 規約参照。

---

<a id="english"></a>

# The Road to POS_HOLD — What's Impressive and What Was Hard

> A record of how the 37 g StampFly micro-drone achieved **hands-off fixed-point hover (±6–7 cm, 16 mm RMS)** using only optical flow + ToF + IMU — no GPS, no motion capture. For the detailed root causes and numbers, see [`poshold_accel_compensation.md`](poshold_accel_compensation.md) (acceleration compensation) and [`poshold_wobble_velocity_noise.md`](poshold_wobble_velocity_noise.md) (residual wobble). This document is the high-level overview.

## 1. Overview

### About this document

It walks through the two big walls crossed on the way to vehicle **POS_HOLD** — the "accelerometer specific-force trap" and the "SIL-vs-hardware gap" — in physics and control-engineering terms: why it is hard, what was tried and failed, how it was finally made to work, and what remains.

### In one sentence

> **POS_HOLD is the milestone of getting an entire position→velocity→attitude→rate cascade to hold together. Along the way it hit two walls: (1) the accelerometer pointing at "apparent gravity" while accelerating horizontally, and (2) the real motor authority shortfall that an idealised SIL cannot reproduce. The first was cured with kinematic-acceleration compensation (an α-β tracker); the second was solved by re-identifying the plant from real flight and redesigning the loop — yielding ±6–7 cm fixed-point hold.**

## 2. Why POS_HOLD is hard

### Time-scale separation of a nested cascade

POS_HOLD keeps the craft pinned to one point in the air even hands-off. Achieving it requires every stage of a nested **cascade** to be stable and correctly band-separated:

```
position error → [pos loop] → velocity cmd → [vel loop] → tilt cmd (attitude target)
              → [attitude loop] → rate cmd → [rate loop] → motor mix
```

Outer loops slow, inner loops fast — break this **time-scale separation** and it diverges. POS_HOLD working is proof that all four stages are stable and meshed at the right bandwidths.

### Handicaps specific to this craft

| Handicap | Content |
|----------|---------|
| **No absolute position sensor** | No GPS, indoor. Position comes only from integrating optical-flow velocity plus ToF height — inherently drift-prone |
| **Accelerometer's fundamental limit** | Loses the gravity direction while accelerating horizontally (§3) |
| **Weak motors** | Real torque effectiveness is 0.4–0.7× the modelled value — the control authority itself is short (§4) |
| **37 g lightweight** | Susceptible to disturbance and limit cycles |

## 3. Wall #1 — the accelerometer points at "false gravity"

### 3.1 The specific-force trap

The accelerometer outputs **specific force** `f = a_kin − g`, not acceleration itself (`a_kin` = kinematic acceleration, `g` = gravity vector). At hover `a_kin = 0`, so `f = −g` and it points straight down — that is how it corrects gyro drift. But POS_HOLD tilts the craft to push back, which accelerates it sideways (`a_kin ≠ 0`), so the specific force deviates from gravity by `θ = atan(a_drift / g)`. With a measured drift ≈ 1.1 m/s², `θ ≈ 6.4°`, and the estimated roll/pitch stuck near ±6.5°.

### 3.2 Positive-feedback fly-away

The gyro tracks the true (growing) tilt, but the accel update keeps yanking the estimate back to "level", so it STICKS instead of tracking truth. The position cascade trusts that wrong attitude, commands more tilt → more acceleration → more deviation → diverges. The proven old vehicle (87 flights) had the same limit ("marginally holds").

### 3.3 What did NOT work — every "weaken the accel" idea

| Attempt | Idea | Why it failed |
|---------|------|---------------|
| Thrust-model down-weight | detect "accelerating", trust accel less | chicken-and-egg: down-weight computed from the contaminated estimate → est thinks it is level → no down-weight |
| Accel-bias freeze | freeze the bias estimate | the bias was never the source; removing a DOF destabilises further |
| Velocity-gated down-weight | trust accel less when flow speed is high | removing the anchor lets gyro drift dominate (90–260 m fly-away) |

**Lesson: weakening the accel always fails.** The gyro alone cannot hold attitude, so the anchor is needed; it points the WRONG WAY, so it must be CORRECTED, not weakened.

### 3.4 What worked — an α-β tracker

Estimate the kinematic acceleration `a_kin` from an INDEPENDENT source (optical-flow velocity) and subtract it: predict `f_pred = g_expected + R^T·a_kin`, so the innovation is the true attitude error. At hover `a_kin ≈ 0` and it reduces to the plain update.

A naive flow-velocity derivative exposed an **axis-asymmetric bug**: it held roll/yaw but diverged on pitch (98 m) and the diagonal — symmetric code, asymmetric result — because the derivative spiked on jittery timing and, being a high-pass, washed out the SUSTAINED (DC) acceleration. The fix is an **α-β filter** (a velocity+acceleration two-state tracker) on the RAW flow velocity (the fused `vel_` re-injects the accel and makes pitch self-reinforce). It integrates the residual into the acceleration state, capturing the sustained drift acceleration. Result: all four axes hold tight in clean SIL (drift ≤ 1.1 m, att_rmse ≤ 3.1°).

### 3.5 Methodology lesson the SIL forced out

The asymmetric bug was hidden by single-axis testing. Only the **per-axis → combined** scenario structure (`pos_roll/pitch/diag/flight`) plus **numerical truth gates** (`metric <name> <op> <value> in <t0> <t1>` in `.expect`) revealed the pitch divergence. **A single-axis test hides axis-asymmetric bugs.**

## 4. Wall #2 — flies in SIL, diverges on hardware

### 4.1 The Code Identity philosophy

The SIL compiles the actual firmware C++ and runs it on a physics simulator — the same source as the real craft (Code Identity) — so a SIL bug should equal a hardware bug. Yet POS_HOLD held perfectly in SIL but **diverged on the first real flight**. That is the core lesson here.

### 4.2 The hardware symptom — a growing unstable limit cycle

Hands-off in a small room: not an instant tumble like SIL, but a **slow horizontal oscillation of period ≈ 9.4 s, amplitude ±0.37→0.50→0.62 m growing** (ω ≈ 0.68 rad/s, σ ≈ +0.057/s) until it hit a wall. The simultaneous ALT_HOLD only drifted monotonically (no oscillation), isolating the oscillation as **intrinsic to the closed position loop**. Flow was healthy (SQUAL 79–184) and the attitude was not stuck — so Wall #1's α-β compensation works on hardware too.

### 4.3 Root cause — the effective "tilt→velocity" gain is only 0.4 g

Three methods agreed on an effective gain **K ≈ 0.4 g** (ideal is g) for commanded-tilt → measured-horizontal-velocity. Breakdown: the **attitude loop under-delivers the tilt** (≈0.58 of command — the main culprit) plus flow-velocity under-read (≈0.66 g). The root of the under-delivery is the real **motor torque effectiveness of 0.4–0.7×** (a known hardware fact since the ACRO-oscillation fix): rate gains are cut ~4×, capping the attitude loop's authority.

The control-theoretic distinction that matters:

```
shortfall in the forward path (tilt→vel)  →  inner(velocity)-loop bandwidth drops below the outer(position) loop
                                          →  cascade time-scale separation collapses  →  instability (divergence)
```

**With K = g (ideal) the current gains are stable (σ −0.124) — that is exactly why SIL passes and hardware crashes.** SIL uses ideal flow (plant.cpp pins SQUAL=100, no noise, torque effectiveness 1.0), so it **cannot reproduce the hardware authority shortfall** — a concrete example of "passes Code-Identity SIL yet fails on hardware".

### 4.4 The fix and the "hardware limit" conclusion

Redesign the closed loop on the hardware-identified plant (`analysis/scripts/poshold_loop_design.py`):

| Parameter | Change | Purpose |
|-----------|--------|---------|
| `position.vel.kp` | 0.8 → **3.0** | restore velocity-loop authority (compensate the K shortfall) |
| `position.pos.kp` | 1.0 → **0.4** | slow the outer loop, restore time-scale separation |

Robustly stable over K∈[2.8,7], τ∈[50,300 ms]. On hardware: divergence gone, hold ±6–7 cm / 16 mm RMS, steady drift tightened 31 → 16 mm, bounded (first-half RMS 0.028 m ≈ second-half 0.031 m). Crucially, **raising att.kp only excites a 1.2 Hz resonance and lowering att.ti barely helps** — the attitude loop has no tuning headroom left. **±6–7 cm is the practical limit for this 37 g craft's torque effectiveness**; tighter hold is a hardware question (more efficient motors/props). SIL's ideal torque cannot reproduce this limit, so it cannot "fix" it.

## 5. The finishing touch — a stick-movable POS mode

With fixed-point hold complete, a **velocity-command repositioning** stick scheme was added (commit `ef3f854`) — deflect to move, release to hold: roll/pitch sticks inject a horizontal velocity command (body frame, matching the STABILIZE tilt direction) into the velocity loop; returning to neutral re-captures the stopped position as the new target. `position.stick_vel` (default 0.4 m/s) is live-tunable. SIL `pos_reposition` (move 0.70 m east → hold the new spot within 6 cm) passes 19/19; good on hardware; INV-1 (single attitude pipeline for all vertical phases) preserved.

## 6. What's impressive

| # | Achievement | Why it's impressive |
|---|-------------|---------------------|
| 1 | **Fixed-point hover without an absolute position sensor** | ±6–7 cm from floor-camera + ToF + IMU only — a different class of difficulty from GPS-based commercial drones |
| 2 | **Cured the specific-force trap properly** | Solved the deepest attitude-estimation problem with an independent-source kinematic-acceleration compensation (α-β tracker), not a band-aid down-weight; beat the old craft's "marginal hold" |
| 3 | **A complete four-stage cascade** | Meshed position→velocity→attitude→rate time-scale separation under weak-motor constraints |
| 4 | **Every finding is numerically backed** | No "shorten Ti and it'll fix" guessing — the axis-asymmetric bug, the 0.4 g effective gain, the 0.43° wobble were all quantitatively identified before any change |

## 7. What was hard

| Difficulty | Content |
|------------|---------|
| **Physics trap hits control directly** | The accelerometer's specific force intrinsically interferes with the position-loop's tilting action — unsolvable from control alone or estimation alone |
| **Every band-aid backfired** | The intuitive "weaken the accel" all failed (chicken-and-egg); the only way was to CORRECT it, requiring the independent-source (flow) reframing |
| **Asymmetric bug in symmetric code** | Pitch alone diverged; hidden by single-axis tests, only the per-axis→combined matrix with numerical gates exposed it |
| **SIL-vs-hardware gap** | Perfect in SIL, diverged on hardware; ideal flow/torque SIL does not reproduce the hardware authority shortfall — required re-identifying the plant and redesigning |
| **A wall tuning cannot cross** | It ends at the hardware limit of 0.4–0.7× torque effectiveness; recognising where software headroom runs out is itself hard |

## 8. Remaining homework — known, hardware-limited

| # | Symptom | Root cause | Direction |
|---|---------|------------|-----------|
| #6 | Horizontal ~1 Hz wobble (±0.43°) | high vel.kp amplifies flow-velocity noise into tilt commands (coherence 0.72); `vel.kp·σ_v/g = 3.0×0.025/9.81 = 0.43°` | velocity low-pass (~0.5 Hz) to strip the noise while keeping tightness; mandatory phase-margin check ([`poshold_wobble_velocity_noise.md`](poshold_wobble_velocity_noise.md)) |
| #7 | Slow altitude bob (±6 cm) | ~110 ms motor/prop actuation lag vs under-damped altitude loop (ToF 30 Hz and ESKF vertical velocity are healthy and lag-free, measured) | `alt.kp 0.6→0.45` damped 38%; precise tuning needs a vertical identification flight |

## 9. Final parameters (as of 2026-06-22)

| Parameter | Value | Note |
|-----------|-------|------|
| `position.pos.kp` | 0.4 | slow outer loop (time-scale separation) |
| `position.vel.kp` | 3.0 | inner-loop authority (K-shortfall compensation) |
| `position.stick_vel` | 0.4 m/s | stick reposition speed |
| `attitude.*.ti` | 2.0 | pilot-preferred |
| `altitude.alt.kp` | 0.45 | altitude bob damped 38% |
| `eskf.accel_comp.*` | enable 1 / alpha 0.2 / beta 0.02 / max 5.0 | acceleration compensation |

> ⚠️ **Applying to hardware requires `param set` → `param save` over WiFi.** The trim learner auto-saves on each landing, so re-flashing alone is overwritten by the old NVS values.
