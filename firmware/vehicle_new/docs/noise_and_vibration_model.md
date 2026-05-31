# Noise and Vibration Model Design
# ノイズ・振動モデル設計書

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

本文書は **SIL の合成センサに与えるノイズモデルと振動モデル**の設計を定義する。
大学制御工学教育レベルの物理的妥当性を目指す。

物理エンジンは **MuJoCo**（剛体運動＋接触、RESET_PLAN §6）が担い、**IMU・ToF・フロー・気圧の合成、モータ、風は自前で実装する**。本文書は、その自作センサ・モータモデルに載せるノイズ／振動の仕様である。SIL の作り方の全体像は `simulator/sil/RESET_PLAN.md` を参照。

### 重要な認識

データシートのノイズ値と実機のノイズは**桁違いに異なる**。

| パラメータ | データシート理想値 | StampFly実測チューニング値 | 倍率 |
|-----------|-----------------|------------------------|------|
| gyro_noise | 0.000122 rad/s/√Hz | 0.009655 rad/s/√Hz | ×79 |
| accel_noise | 0.00157 m/s²/√Hz | 0.3 m/s²/√Hz | ×191 |

この差の主因は**モーター/プロペラ振動**であり、静的ノイズモデルだけでは不十分。

## 2. プロセスノイズ（Q行列）の理論

### 連続時間ノイズ密度 → 離散Q

```
Q_d = σ_c² × Δt    [単位²]
```

- σ_c: 連続時間ノイズ密度 [単位/√Hz]（データシート記載値）
- Δt: サンプリング間隔 [s]

### ESKF 15状態のQ行列

> **SIL が出すのは §3〜§4 の物理的なセンサノイズだけ**で、各推定器（ESKF／相補／Madgwick／MPC）はそこから自分の Q/R・ゲインを導く（アルゴリズム非依存、RESET_PLAN 方針2）。下の Q/R 表は**現行 ESKF の例示**であり、同じ物理ノイズが一つの推定器の行列にどう写るかを示すもの。SIL の義務は 15 状態 ESKF 形の構造を出すことではなく、物理ノイズを出すことである。

| 状態 | Q要素 | StampFly値 |
|------|-------|-----------|
| POS (0-2) | 0（速度積分で自動増大） | 0 |
| VEL (3-5) | accel_noise² × Δt | 2.25e-4 |
| ATT (6-8) | gyro_noise² × Δt | 2.3e-7 |
| BG (9-11) | gyro_bias_noise² × Δt | 4.2e-13 |
| BA (12-14) | accel_bias_noise² × Δt | 2.5e-11 |

### バイアスのランダムウォーク

```
bias(t) = bias_startup + Σ w[k]
w[k] ~ N(0, σ_rw² × Δt)
```

- bias_startup: 起動時に固定される初期バイアス（毎回異なる）
- σ_rw: ランダムウォーク密度（Allan分散の長τ成分から推定）

## 3. 観測ノイズ（R行列）

| センサ | 観測量 | データシートσ | StampFly設定σ | R = σ² |
|--------|-------|-------------|-------------|--------|
| ToF | 高度 [m] | 0.005-0.03 | 0.03 | 9.0e-4 |
| Baro | 高度 [m] | 0.013 | 0.1 | 1.0e-2 |
| Mag | 磁場 [µT] | 0.3 | 1.0 | 1.0 |
| Flow | 速度 [m/s] | 高度依存 | 0.30 | 9.0e-2 |
| AccelAtt | 姿勢 [m/s²] | 0.00157 | 0.06 | 3.6e-3 |

## 4. モーター/プロペラ振動モデル

### 振動の周波数特性（StampFly推定）

| パラメータ | ホバリング時推定 | 最大時推定 |
|-----------|---------------|----------|
| モーターRPM | 15,000-20,000 | 35,000-45,000 |
| 1次振動 | 250-333 Hz | 583-750 Hz |
| BPF（2枚） | 500-667 Hz | 1167-1500 Hz |
| IMU ODR | 400 Hz (Nyquist=200Hz) | — |

**注意:** 1次振動（250-333Hz）はNyquist周波数（200Hz）を超えており、
エイリアシングが発生する。BMI270の内蔵AAFの設定が極めて重要。

### 振動の振幅（スロットル依存）

| 状態 | 加速度振動 rms [m/s²] | 角速度振動 rms [°/s] |
|------|---------------------|---------------------|
| モーターOFF | 0.01-0.05 | 0.01-0.1 |
| ホバリング (duty 30-50%) | 2-10 | 5-20 |
| 高スロットル (duty 70-90%) | 5-30+ | 10-50+ |

スロットル依存性: `振幅 ≈ K × duty²`

### SILでの振動再現レベル（Noise Model Stage）

> **Note:** 本節の `N0〜N4` はノイズモデルの理論的段階を指す（教材としての複雑度区分）。<br>
> 開発工程の `Phase 0〜6` やプラント同定層 `Layer 1〜4`（`development_roadmap.md`）とは別概念で、衝突を避けるため `N` プレフィックスを用いる。

| 段階 | モデル | 教育用途 |
|-------|-------|---------|
| N0 | ホワイトガウスノイズ（固定σ） | KFの基礎理解 |
| N1 | スロットル依存ガウスノイズ | ノイズ-スロットル相関の理解 |
| **N2** | **N1 + 帯域制限ノイズ** | **LPFの必要性理解（推奨）** |
| N3 | N2 + 正弦波（BPF成分） | ノッチフィルタ設計演習 |
| N4 | 実機FFTプロファイル注入 | 実機との定量比較（Model Fidelity、実機飛行後） |

### 推奨: N2（帯域制限スロットル依存ノイズ）

```cpp
// SIL 合成センサのノイズモデル構成
// SIL synthetic-sensor noise model structure

struct SensorNoiseModel {
    // 1. Static noise (datasheet)
    // 1. 静的ノイズ（データシート）
    float gyro_noise_density;     // [rad/s/√Hz]
    float accel_noise_density;    // [m/s²/√Hz]

    // 2. Startup bias (random per boot)
    // 2. 起動時バイアス（毎起動ランダム）
    float gyro_bias[3];           // [rad/s]
    float accel_bias[3];          // [m/s²]

    // 3. Bias random walk
    // 3. バイアスランダムウォーク
    float gyro_bias_rw;           // [rad/s/√s]
    float accel_bias_rw;          // [m/s²/√s]

    // 4. Throttle-dependent vibration (per-axis)
    // 4. スロットル依存振動（軸ごと）
    //
    // Per-axis K is required because hover02 backtest showed gyro Z is 16x
    // smaller than X/Y, accel Z is 13% larger than X/Y. Isotropic K under-
    // or over-estimates each axis by 1.1-16x.
    // 軸別Kが必要: hover02 ログ実測でジャイロZがX/Yより16倍小、
    // 加速度ZがX/Yより13%大。等方Kは各軸を1.1〜16倍で誤推定する。
    float vib_accel_k[3];         // K_accel per axis (X,Y,Z): σ_axis = K[axis] × duty²
    float vib_gyro_k[3];          // K_gyro  per axis (X,Y,Z)
    float vib_freq_low;           // [Hz] bandpass lower bound
    float vib_freq_high;          // [Hz] bandpass upper bound
};

// Starting values seeded from a legacy-hardware flight log (firmware/vehicle,
// hover02, 2026-04-13). They give a realistic initial noise model for the new
// SIL; refining them against vehicle_new's own logs is Model Fidelity work,
// done only after vehicle_new flies (development_roadmap.md Phase 5).
// 旧機（firmware/vehicle, hover02, 2026-04-13）のログから採った初期値。
// 新 SIL の現実的な初期ノイズモデルになる。vehicle_new 自身のログでの精緻化は
// Model Fidelity（実機飛行後＝development_roadmap.md Phase 5）で行う。
//   vib_accel_k = {3.96, 2.35, 5.64}  [m/s²]
//   vib_gyro_k  = {1.08, 0.83, 0.15}  [rad/s]
//   Backtest residuals all within 1.1-1.2x of the legacy-data σ
//   バックテスト残差: 全軸で旧機データσの 1.1〜1.2倍以内
```

## 5. 合成センサ・モータモデルの要件（MuJoCo 上に自作）

MuJoCo が剛体運動（並進・回転、オイラー方程式・ジャイロスコピック項を含む）と接触を計算する。**自作するのは、その状態量から作る合成センサと、モータ／風モデル**である。MuJoCo の内蔵センサ（`<accelerometer>`／`<gyro>`／`<rangefinder>`）・内蔵アクチュエータ（`<motor>`）は、**自作実装が正しいかを照らし合わせるリファレンス**として使う（RESET_PLAN §6）。

### 加速度計が測る加速度（specific force）の正しい計算 ← 合成加速度計

加速度計は、重力への反作用を含んだ加速度（＝加速度計が実際に測る量。慣性航法でいう specific force）を出力する。**符号と回転行列はファーム本体の規約に厳密に合わせること**（裏取り済み、`eskf_core.cpp` / `calibration.cpp`）:

- **StampFly の生加速度計は水平静止で `[0,0,+9.81]`（反作用＝proper-acceleration 規約、+g）を読む。** これは標準的な定義 `f = R_bn(a_world − g_ned)`（水平静止で −g）の**符号を反転**したもの。
- 回転は **`R_bn = R_nb^T`（NED→body）**。`q`（`attitude`）は `q_nb`（body→NED）なので、NED 量を body へ移すには `inv_rotate`（=`R_bn`）を使う。`g_ned = [0,0,+9.81]`（+Z 下）。

```
合成加速度計（生・body・+g 規約）
  raw_body = R_bn × (g_ned − a_world)
           = R_bn × (g_ned − (thrust_ned + drag) / mass)
  ただし R_bn = inv_rotate(by q_nb) = R_nb^T,  g_ned = [0,0,+9.81]
  水平静止: a_world=0, R_bn=I → raw_body = [0,0,+9.81]
```

ファームはこの生値（+g）を受け、起動時の `ba_z ≈ +2g` バイアス（次項）で内部的に −g（標準の符号）へ変換する。**SIL はこのバイアス初期化も再現すること**（さもないと重力を誤符号で積分）。

旧 SIL の合成センサは推力加速度を含めていなかった（致命的バグ）。**自作加速度計では推力寄与を必ず含める。** MuJoCo 内蔵 `<accelerometer>`（site=body 系で加速度計が測る加速度を返す）と突き合わせて検算する。

### バイアス初期化 ← 起動キャリブレーションの再現

vehicle ファームは起動時に `setAttitudeReference()` で `ba_z ≈ 2g` をセットする。
SIL でも起動キャリブレーションフローを再現し、推定器が実機と同じ初期化を踏むようにする。

### 推力の二乗則 ← 自作モータモデル

モータモデルは duty を推力に変換する。線形ではなく二乗則。

```
thrust = k_thrust × duty²
```

これに加えて、モータ遅れ（一次遅れ τ）・健全度／劣化（推力スケール）を持たせる（RESET_PLAN §5 の「外乱（風）・モータ健全度」を最初から実装）。生成した推力／トルクは MuJoCo の機体に力として与える。

### オイラー方程式のジャイロスコピック項 ← MuJoCo が担当

剛体回転（下式のジャイロスコピック結合を含む）は **MuJoCo の剛体積分が計算する**。自作はしない。合成ジャイロは MuJoCo の機体角速度 ω を読み、§4 のノイズ／振動を載せて出力する。式は教材・検算の参照として残す。

```
I_xx × ω̇_x = τ_x - (I_zz - I_yy) × ω_y × ω_z
I_yy × ω̇_y = τ_y - (I_xx - I_zz) × ω_z × ω_x
I_zz × ω̇_z = τ_z - (I_yy - I_xx) × ω_x × ω_y
```

## 6. パラメータ同定方法

| パラメータ | 同定方法 | 必要データ |
|-----------|---------|-----------|
| K_accel, K_gyro | ホバリングログのRMS vs duty回帰 | 複数スロットルの定常データ |
| f_low, f_high | FFT PSDの-10dBポイント | 定常ホバリングのFFT |
| バイアスドリフト | Allan分散解析 | 長時間静置データ |

> **実機ログは SIL を作る・動かす前提ではない（RESET_PLAN §2 方針1）。** §4 のレガシー値は「現実的な初期値の**任意のシード**」にすぎず、SIL はデータシート値×経験的倍率のような妥当な初期値でも build/run してゲートまで到達できる。旧機（既に飛んだ**別の**機体）のログがたまたま存在するのでシードに使うだけで、vehicle_new 自身での同定は実機飛行後（Model Fidelity、Phase 5）に行う。

## 7. 実装ロードマップ

SIL の合成センサ整備の順序。RESET_PLAN の **P1（物理ベース SIL の骨格）**の中で進め、N3/N4 は教材・Model Fidelity として後段に置く。

| 段階 | 内容 | 教育目標 | 対応（RESET_PLAN / Phase） |
|------|------|---------|--------------------------|
| 基礎 | 加速度計の式の修正 + バイアス初期化（合成IMUが正しく動く） | SILが正常に飛ぶ | P1 |
| N0 | IMUガウスノイズ + バイアス | KFの基礎 | P1 |
| N1 | スロットル依存ノイズ | 振動とノイズの関係 | P1 |
| N2 | 帯域制限 + ToF/Baroノイズ | フィルタ設計の動機 | P1 |
| N3 | 正弦波BPF + Flow | ノッチフィルタ、高度依存ノイズ | 教材 |
| N4 | 実機FFTプロファイル | モデル検証 | Phase 5（実機飛行後） |

---

<a id="english"></a>

## 1. Overview

This document defines the **sensor noise and vibration models fed to the SIL's synthetic sensors**.
Target: university-level control engineering education with physical validity.

The physics engine is **MuJoCo** (rigid-body + contact, RESET_PLAN §6); the **IMU/ToF/flow/baro synthesis, the motor, and wind are written by hand**. This document is the noise/vibration spec layered on top of those self-written sensor/motor models. For the full SIL design, see `simulator/sil/RESET_PLAN.md`.

### Key Insight

Datasheet noise values and actual in-flight noise differ by **orders of magnitude**.
The primary cause is **motor/propeller vibration**, making static noise models insufficient.

## 2. Specifications (see Japanese §2–§7 for full detail)

### Summary Tables

**Q Matrix (Process Noise):**
- VEL: accel_noise² × Δt = 2.25e-4
- ATT: gyro_noise² × Δt = 2.3e-7
- BG: gyro_bias_noise² × Δt = 4.2e-13
- BA: accel_bias_noise² × Δt = 2.5e-11

**Noise Model Stages (N0-N4):**

> Note: `N0-N4` denote noise-model complexity tiers (educational). They are distinct from the development `Phase 0-6` and the plant-identification `Layer 1-4` in `development_roadmap.md`. The `N` prefix is used to avoid collision.

- N0: White Gaussian (basic KF understanding)
- N1: Throttle-dependent (noise-throttle correlation)
- **N2: N1 + band-limited (LPF motivation) — Recommended**
- N3: N2 + sinusoidal BPF (notch filter design)
- N4: Real FFT profile injection (validation — Model Fidelity, after first real flight)

**Self-written sensor/motor model requirements (on top of MuJoCo):**
1. Specific force: the synthetic accelerometer must include the thrust contribution (the old SIL's bug); cross-check against MuJoCo's built-in `<accelerometer>`
2. Startup calibration (bias initialization, `ba_z ≈ 2g`)
3. Quadratic thrust motor model (`thrust = k·duty²`) plus motor lag and health/degradation; rigid-body rotation (Euler/gyroscopic terms) is handled by MuJoCo, not hand-integrated
4. Sensor noise (Gaussian + bias + per-axis vibration), seeded from legacy logs, refined later via Model Fidelity
