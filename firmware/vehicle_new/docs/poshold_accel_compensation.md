# POSITION_HOLD と運動加速度補償 — 問題・試行錯誤・解・残課題

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

vehicle_new の **POSITION_HOLD（位置保持）** が SIL で「飛び去る／タンブルする」問題を、原因究明から解決まで追った記録である。なぜ飛び去ったのか（物理）、何を試して何が効かなかったのか、最終的にどう直したのか、そして何が残っているのかを、制御工学の言葉で順に説明する。

### 一文で言うと

> **加速度センサは「重力」と「機体の運動による加速度」を区別できない。水平に加速している間は、姿勢推定が本当の重力でなく『見かけの重力』の方を向いてしまい、制御が嘘の姿勢で動いて飛び去る。** これを、オプティカルフローから運動加速度を推定して差し引くことで解決した。

### 対象読者

vehicle_new の推定（ESKF）・制御（PID カスケード）・SIL テストに関わる人。ドローンの姿勢推定がなぜ難しいかを学びたい人。

## 2. 問題 — なぜ POSITION_HOLD が飛び去ったか

### 2.1 加速度センサが測るのは「比力」

加速度センサが出すのは加速度そのものではなく **比力（specific force）** `f = a_kin − g` である。

- `a_kin` … 機体の運動加速度（ニュートンの第2法則の加速度）
- `g` … 重力加速度ベクトル

静止・ホバー中は `a_kin = 0` なので `f = −g`、つまり加速度センサは**重力の向き**をそのまま指す。だからホバー中は加速度センサで「下はどっち」が分かり、ジャイロ積分のドリフトを補正できる。

### 2.2 水平に加速すると「見かけの重力」を向く

POSITION_HOLD は位置を保つために機体を傾けて押し戻す。傾ければ機体は**横に加速する** → `a_kin ≠ 0`。すると比力は重力からズレる：

```
        ホバー中                    水平加速中（ドリフトを止めようと傾斜）
                                    
          │ f = −g                    ╲  f = a_kin − g
          │（真下＝重力）              ╲（重力から θ だけ傾く＝見かけの重力）
          ▼                            ◢θ
        [機体]                       [機体]  → 横加速 a_kin

   加速度センサは正しく            加速度センサは「見かけの下」を
   重力を指す                       指す → 姿勢推定が θ だけ誤る
```

定常的に `a_drift` で横にドリフトしていると、姿勢推定は重力でなく**見かけの重力の角度** `θ = atan(a_drift / g)` に張り付く。実測でドリフト加速度 ≈ 1.1 m/s² のとき `atan(1.1/9.81) ≈ 6.4°` で、推定 roll/pitch が ±6.5° に固まる現象として現れた。

### 2.3 嘘の姿勢で制御が動く → 正のフィードバックで飛び去る

ジャイロ積分は本当の傾き（−48° などに増えていく真値）を追えるのに、accel-attitude 更新が「お前は水平（±6.5°）だ」と毎周期引き戻すため、**推定が真値に追従せず張り付く**。位置カスケードはその嘘の姿勢を信じて傾き指令を出すので、

```
推定が「ほぼ水平」と誤認 → 制御は「もっと傾けないと」と判断
   → 実機はさらに傾く → さらに加速 → さらに見かけの重力がズレる
      → 推定はますます水平と誤認 …（正のフィードバックで発散）
```

これが POSITION_HOLD の飛び去り／タンブルの正体だった。旧 vehicle（87飛行）も同じ限界を持ち「かろうじて保持」止まりだった。

## 3. 試行錯誤の記録

pos_flight シナリオ（ロール単軸のステップ擾乱→保持）で 3 案を比較し、さらに網羅テストで本命を見つけた。

### 3.1 診断 — バイアスの発生源はどこか

2つの仮説を SIL のデータで切り分けた：

| 仮説 | 内容 | 判定 |
|------|------|------|
| **H1** | accel_bias_y が定常傾斜を吸収して est_roll をバイアス | **棄却**（定常で ba_y ≈ 0.008 m/s²、含意 roll 0.05°。16° のバイアスを全く説明しない） |
| **H2** | 比力（推力方向＝見かけの重力）による姿勢汚染 | **確定**（est_roll が ±6.5° に張り付き、真値が発散） |

### 3.2 効かなかった案（減重・凍結系は全滅）

| 案 | 考え方 | 結果 | なぜダメか |
|----|--------|------|-----------|
| **A. 推力補償（適応R減重）** | 推力モデルで「加速中」を検知し accel を弱める | 全 k で悪化 | **鶏卵問題**：減重量を「汚染された推定姿勢」から計算 → 推定は水平と思い込む → 減重が効かず accel が水平へ引く悪循環 |
| **B. accel-bias フリーズ** | 飛行中に accel バイアス推定を凍結 | 悪化 | H1 棄却どおりバイアスは無関係。自由度を奪うと更に不安定 |
| **D. 速度ゲート減重** | フロー速度が大きい時 accel を弱める | 全滅（90〜260m） | accel の**アンカーを外す**とジャイロ積分のドリフトが勝って全軸が流れる |

**共通の教訓：accel の重みを下げる（アンカーを外す）方向は全て失敗する。** ジャイロ単独では姿勢を保てないため、accel は必要。問題は accel が**間違った方向**を指すことなので、弱めるのでなく**正しく直す**必要がある。

### 3.3 効いた案 — Exp C: 運動加速度補償

accel を弱めるのでなく、**運動加速度 `a_kin` を独立な情報源から推定して比力から差し引く**：

```
予測する比力  f_pred = g_expected + R^T · a_kin
イノベーション innov = f_measured − f_pred
                      = (本当の姿勢誤差) ← a_kin が正しければ運動項が消える
```

`a_kin` は **オプティカルフロー速度の微分**から得る。フロー速度は加速度センサと独立なので、循環（自分で自分を補正する）にならない。ホバー中は `a_kin ≈ 0` で素の更新に一致し、加速中だけ補正が働く。

### 3.4 網羅テストが「軸非対称バグ」を炙り出した（重要）

ここで決定的な発見があった。それまでの pos_flight は**ロール単軸**しか試していなかった。**個別軸のシナリオ（pos_roll / pos_pitch / pos_diag / pos_yaw）と数値ゲート**を新設して全軸で検証したところ：

| 軸 | Exp C（単純微分）の結果 |
|----|------------------------|
| roll | ✅ 保持（5.4m） |
| **pitch** | ❌ **発散（98m）** |
| diagonal | ❌ 発散（21m） |
| yaw | ✅ 保持（3.4m） |

**ロールは直るのにピッチが発散**。コードは左右対称なのに非対称が出た。原因は **フロー速度の単純微分（高域通過）の二重欠陥**：

1. **スパイク**：不規則なフロー dt で微分が ±30 m/s²（非物理）に暴れる
2. **DC 欠落**：高域通過は**持続する加速度（DC 成分）を washout** し、振動成分しか通さない。単軸はたまたま保持できても、両軸が同時に効く斜め複合で破綻

→ 単純微分では「軸別はギリギリ保持・複合で発散」という脆い挙動になり、パラメータをいくら振っても全軸を同時に満たす点が存在しなかった（カオス的）。

### 3.5 決定打 — α-β トラッカ

単純微分をやめ、**α-β フィルタ**（状態 = 速度＋加速度の2状態追跡器）で `a_kin` を推定した：

```
予測:  v_pred = v_est + a_est · dt
補正:  残差 r = v_flow − v_pred
       v_est += α · r          （速度状態）
       a_est += (β/dt) · r     （加速度状態 ← これが a_kin）
```

α-β は残差を**加速度状態に積分**するので、高域微分のように DC を捨てず、**持続ドリフト加速度（est バイアスの真因そのもの）を捉える**。β を小さくするとノイズに強くなる。

| 試行 | 結果 |
|------|------|
| 単純微分 | roll◎ pitch✗ diag✗（脆弱・カオス） |
| 融合速度 vel_ の微分 | diag◎ pitch✗（循環でピッチ自己強化） |
| **α-β トラッカ（β=0.02）** | **全4軸◎（drift ≤ 3.8m, 安定基盤）** |

**ポイント：融合速度 `vel_` でなく生フロー速度を使う**こと。融合速度は predict で accel を再注入するため、ピッチ軸が自己強化して発散する。

### 3.6 タイト化 — 制御ゲインを上げられるようになった

推定器が全軸で真値に追従するようになって初めて、位置カスケードのゲインを上げられた：

| 構成 | 位置速度ゲイン vel.kp | 斜め複合の挙動 |
|------|----------------------|----------------|
| 旧（汚染推定） | 0.3 が限界（0.5 で発散） | marginal hold（~5m 大振動、原点に収束せず残留3m） |
| **新（推定修正後）** | **0.8**（安定基盤 0.7〜0.9） | **タイト収束（~1m 塊、残留 ±0.1m）** |

「制御調整は限界、律速は推定」という以前の結論どおり、**推定を直すことが制御を解放した**。

## 4. 採用した解

### 4.1 パラメータ（SSOT・既定 ON）

| パラメータ | 値 | 意味 |
|-----------|-----|------|
| `eskf.accel_comp.enable` | 1（ON） | 運動加速度補償の有効化 |
| `eskf.accel_comp.alpha` | 0.2 | α-β 速度ゲイン |
| `eskf.accel_comp.beta` | 0.02 | α-β 加速度ゲイン（小＝DC 捕捉） |
| `eskf.accel_comp.max` | 5.0 | a_kin の物理クランプ [m/s²] |
| `position.vel.kp` | 0.8 | 位置速度ループ（推定修正で 0.3→0.8 可能に） |

### 4.2 実装箇所

| ファイル | 内容 |
|---------|------|
| `eskf_core.cpp` `updateFlowRaw` | α-β トラッカで a_kin を推定 |
| `eskf_core.cpp` `updateAccelAttitude` | `h_vec = g_expected + R^T·a_kin` を予測に使用 |
| `params.cpp` | 上記パラメータを登録 |
| `pid_controller.cpp` | 位置カスケード（vel.kp） |

### 4.3 結果（clean SIL・全4軸タイト保持）

| 軸 | 最大ドリフト | 終端オフセット | 推定 att_rmse |
|----|------------|---------------|---------------|
| roll | 0.67m | ≤1.0m | 1.5° |
| pitch | 1.11m | ≤1.0m | 1.8° |
| diagonal | 0.85m | ≤1.0m | 3.1° |
| yaw | 0.90m | ≤1.0m | 1.7° |

N0 ノイズ下でも drift ≤ 1.3m。決定論的（同シードで byte-identical）。

## 5. テスト方法（網羅テストの作り方）

今回の鍵は「ロール単軸だけ試して満足しない」こと。**個別軸 → 複合**の構成と**物理真値の数値ゲート**を導入した。詳細は `simulator/sil/scenarios/TEST_MATRIX.md`。

- **個別軸 → 複合**：`pos_roll`（ロール単独）→ `pos_pitch`（ピッチ単独）→ `pos_flight`（斜め複合 capstone）→ `pos_yaw`（機首を回して保持）
- **数値ゲート**：`.expect` に `metric <name> <op> <value> in <t0> <t1>` を追加し、`trajectory.csv` の真値＋推定から G2（推定追従）/G3（有界・タイト）/G4（非飽和）を機械判定。従来はログ文字列（≈G1）だけだった

> **教訓：単軸テストは軸非対称のバグを隠す。** 各軸を1軸ずつ試し、最後に複合で試す構成にして初めて、ピッチ軸の発散という重大バグが見えた。

## 6. 残課題

| # | 課題 | 説明 | 方向性 |
|---|------|------|--------|
| 1 | **入口過渡の振動** | 擾乱直後に水平 ~0.85m まで振れ、~10秒かけて減衰してから定点に収まる（「即収束」でなく「減衰しながら収束」） | 位置/速度ループの減衰を上げる（vel の微分項 td、または位置の積分 ti）。ただしゲインは脆弱な領域があるので SIL で慎重に |
| 2 | **過酷振動（n1/n2）下の飛び去り** | スロットル依存振動 n1／帯域制限 n2 では baseline 同様に飛び去る。フロー速度がノイズで汚れ a_kin が劣化するため | 振動処理（ノッチフィルタ、フローノイズモデルの精緻化）が必要。clean + N0 が主検証レベルで、n1/n2 は別フェーズの課題 |
| 3 | **実機 ESP-IDF ビルド未検証** | host SIL は全ソースを通すが、ESP32 ターゲットでのビルドは未確認 | `sf build vehicle_new` で確認 |
| 4 | **実機飛行未検証** | SIL（物理真値）で成立したが、実機 POSITION_HOLD は未飛行 | development_roadmap Phase 4.3 の手順（SIL→実機 Code Identity）で進める |

### 残課題の優先度

1 と 3 は短く、2 と 4 は別フェーズ。今回の修正は**根本原因（姿勢推定の汚染による飛び去り）の解消**であり、入口過渡の整定をさらに詰めるのは制御チューニングの上積み（独立した課題）である。

### 更新 — 初の実機 POS_HOLD 飛行（2026-06-22, log 20260622T161055）

残課題#4（実機未検証）に着手。狭い部屋・手放しで POS_HOLD 飛行 → SIL のような即タンブルではなく、**周期約9.4秒・振幅 ±0.37→0.62m と成長する緩い水平振動（不安定リミットサイクル）**で壁に激突。同時刻の ALT_HOLD は単調ドリフトのみ（振動なし）で、振動が**閉じた位置ループ固有**であることを切り分けた。フローは健全（SQUAL 79–184）、姿勢の張り付きも無し（α-β 補償は実機でも有効）。

**真因**: 同定（`analysis/scripts/poshold_loop_design.py`）で、実機の「指令傾き→実測水平速度」の実効ゲインが**約 0.4 g しかない**（傾き未達＋フロー速度の過小読み）。これが内側(速度)ループ帯域を外側(位置)ループより下げ、カスケードの時間スケール分離を壊して発散。**SIL は理想フロー（K=g, ノイズ無し）ゆえ安定 = 残課題#1「入口過渡」の実機での正体はこのゲイン不足による不安定**だった。

**暫定対策（実装済み・SIL退行なし）**: 同定プラント上で `position.vel.kp 0.8→2.0`（≈0.8·g/K、速度ループの権限回復）＋`position.pos.kp 1.0→0.3`（外ループを遅く）。K∈[2.4,9.8]・τ∈[50,300]ms でロバスト安定。実機の外乱 0.19 m/s² へのピーク偏差は約0.27m。**ただしこれは不足プラントへの band-aid**。真の tight hold には**ゲイン不足の根治**（姿勢ループの傾き達成度の確認・フロー速度スケール/ToF高さの校正）が必要 = 残課題#5（新規）。

| # | 課題 | 方向性 |
|---|------|--------|
| 5 | **実効「傾き→速度」ゲイン不足（~0.4 g）** | 姿勢ループが指令傾きを達成しているか（STABILIZE 飛行で同定）、フロー速度スケール（rad/pixel・ToF高さ）の校正。K→g に戻れば暫定ゲインを上げ tight hold 可能 |

---

<a id="english"></a>

## 1. Overview

A record of diagnosing and fixing the vehicle_new **POSITION_HOLD** "fly-away / tumble"
problem in SIL: why it flew away (the physics), what was tried and what failed, how it
was finally fixed, and what remains — explained in control-engineering terms.

### In one sentence

> **An accelerometer cannot tell gravity from the craft's own kinematic acceleration.
> While accelerating horizontally, the attitude estimate points at the "apparent gravity"
> instead of true gravity, so the controller flies on a wrong attitude and runs away.**
> Fixed by estimating the kinematic acceleration from optical flow and subtracting it.

## 2. The problem — why POSITION_HOLD flew away

### 2.1 An accelerometer measures specific force

The accelerometer outputs **specific force** `f = a_kin − g`, not acceleration itself
(`a_kin` = kinematic acceleration, `g` = gravity vector). At hover `a_kin = 0`, so
`f = −g`: it points along gravity, which is how it corrects the gyro's drift.

### 2.2 Accelerating horizontally → it points at "apparent gravity"

POSITION_HOLD tilts the craft to push back against drift; the tilt makes it accelerate
sideways (`a_kin ≠ 0`), so the specific force deviates from gravity by
`θ = atan(a_drift / g)`. With a measured drift of ~1.1 m/s², `θ ≈ 6.4°`, and the estimated
roll/pitch stuck near ±6.5°.

### 2.3 The controller flies on a wrong attitude → positive-feedback fly-away

The gyro integration tracks the true (growing) tilt, but the accel-attitude update keeps
pulling the estimate back to "level", so the estimate STICKS instead of tracking truth.
The position cascade trusts that wrong attitude and commands more tilt → more acceleration
→ more deviation → diverges. The proven old vehicle (87 flights) had the same limit
("marginally holds").

## 3. The attempts

### 3.1 Diagnosis — where is the bias born?

- **H1** (accel_bias_y absorbs the steady tilt): **rejected** — ba_y ≈ 0.008 m/s² explains
  none of the 16° bias.
- **H2** (specific-force / apparent-gravity contamination): **confirmed** — est sticks at
  ±6.5° while truth diverges.

### 3.2 What did NOT work (all the down-weight / freeze ideas)

| Attempt | Idea | Result | Why it failed |
|---------|------|--------|---------------|
| **A. thrust-model down-weight** | detect "accelerating" from a thrust model, trust accel less | worse at all gains | chicken-and-egg: the down-weight is computed from the CONTAMINATED estimate → est thinks it is level → no down-weight → accel keeps pulling to level |
| **B. accel-bias freeze** | freeze the accel-bias estimate in flight | worse | the bias was never the source (H1); removing a DOF destabilises further |
| **D. velocity-gated down-weight** | trust accel less when flow speed is high | diverges (90–260 m) | removing the accel ANCHOR lets the gyro drift dominate |

**Lesson: every "weaken the accel" approach fails.** The gyro alone cannot hold attitude,
so the accel anchor is needed; the problem is that it points the WRONG WAY, so it must be
CORRECTED, not weakened.

### 3.3 What worked — Exp C: acceleration compensation

Estimate the kinematic acceleration `a_kin` from an INDEPENDENT source (optical-flow
velocity) and subtract it: predict `f_pred = g_expected + R^T·a_kin`, so the innovation is
the TRUE attitude error. At hover `a_kin ≈ 0` and it reduces to the plain update.

### 3.4 A comprehensive test exposed an axis-asymmetric bug (key)

The old pos_flight tested ROLL ONLY. Adding per-axis scenarios (pos_roll/pitch/diag/yaw)
with numerical gates showed Exp C held roll/yaw but **diverged on pitch (98 m) and the
diagonal** — symmetric code, asymmetric result. The cause was the **naive flow-velocity
derivative**: (1) it spiked to ±30 m/s² on jittery flow timing, and (2) being a high-pass,
it **washed out the SUSTAINED (DC) acceleration** and passed only oscillation. Each axis
held alone, but the diagonal (both axes at once) diverged, with no robust parameter point.

### 3.5 The fix — an α-β tracker

Replace the derivative with an **α-β filter** (a two-state tracker: velocity + acceleration)
on the flow velocity. It integrates the residual into the acceleration state, so it captures
the sustained drift acceleration (the actual source of the bias) instead of washing it out.
Small β rejects noise. **Use the RAW flow velocity, not the fused `vel_`** (which re-injects
the accel via predict and makes the pitch axis self-reinforce). Result: all four axes hold.

### 3.6 Tightening — the estimator fix unlocked higher control gains

Only once the estimate tracked truth could the position cascade be tuned: `position.vel.kp`
went 0.3 → 0.8 (the old contaminated estimate capped it at 0.3, the diagonal diverged at
0.5), turning the marginal ~5 m swing into a tight ~1 m hold.

## 4. The adopted solution

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `eskf.accel_comp.enable` | 1 | enable acceleration compensation |
| `eskf.accel_comp.alpha` | 0.2 | α-β velocity gain |
| `eskf.accel_comp.beta` | 0.02 | α-β acceleration gain (small = capture DC) |
| `eskf.accel_comp.max` | 5.0 | physical clamp on a_kin [m/s²] |
| `position.vel.kp` | 0.8 | position velocity loop (raised from 0.3) |

Result (clean SIL): all four axes hold tight — drift ≤ 1.1 m, final offset ≤ 1.0 m,
att_rmse ≤ 3.1°; ≤ 1.3 m under N0 noise; deterministic.

## 5. Test methodology

The key was not to stop at a single-axis test. Use **isolated-axis → combined** scenarios
(`pos_roll`/`pos_pitch`/`pos_flight`/`pos_yaw`) and **physical-truth numerical gates**
(`metric <name> <op> <value> in <t0> <t1>` in `.expect`, computed from `trajectory.csv` for
G2/G3/G4 — the DSL was log-only ≈ G1). See `simulator/sil/scenarios/TEST_MATRIX.md`.

> **Lesson: a single-axis test hides axis-asymmetric bugs.** Only the per-axis-then-combined
> structure revealed the pitch-axis divergence.

## 6. Remaining issues

| # | Issue | Description | Direction |
|---|-------|-------------|-----------|
| 1 | **Entry transient** | a ~0.85 m swing that damps over ~10 s before settling (damped, not instant convergence) | add velocity-loop damping (td) or position integral (ti); tune carefully in SIL (fragile regions exist) |
| 2 | **Fly-away under n1/n2** | severe vibration corrupts the flow velocity → a_kin degrades; flies away like the baseline | needs vibration handling (notch, better flow-noise model); clean + N0 are the primary levels |
| 3 | **On-target ESP-IDF build unverified** | host SIL compiles all sources; the ESP32 build is unchecked | run `sf build vehicle_new` |
| 4 | **Real-flight unverified** | holds in SIL (physical truth); the real POSITION_HOLD is unflown | proceed via development_roadmap Phase 4.3 (SIL→hardware Code Identity) |

Items 1 and 3 are short; 2 and 4 are separate phases. This fix resolves the ROOT CAUSE
(fly-away from attitude contamination); further damping the entry transient is incremental
control tuning, a separate concern.
