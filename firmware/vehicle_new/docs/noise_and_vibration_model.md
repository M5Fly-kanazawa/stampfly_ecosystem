# Noise and Vibration Model Design
# ノイズ・振動モデル設計書

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

本文書はSILシミュレータのセンサノイズモデルと振動モデルの設計を定義する。
大学制御工学教育レベルの物理的妥当性を目指す。

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

### SILでの振動再現レベル

| レベル | モデル | 教育用途 |
|-------|-------|---------|
| L0 | ホワイトガウスノイズ（固定σ） | KFの基礎理解 |
| L1 | スロットル依存ガウスノイズ | ノイズ-スロットル相関の理解 |
| **L2** | **L1 + 帯域制限ノイズ** | **LPFの必要性理解（推奨）** |
| L3 | L2 + 正弦波（BPF成分） | ノッチフィルタ設計演習 |
| L4 | 実機FFTプロファイル注入 | 実機との定量比較 |

### 推奨: L2（帯域制限スロットル依存ノイズ）

```cpp
// SILセンサモデルの構成
// SIL sensor model structure

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

    // 4. Throttle-dependent vibration
    // 4. スロットル依存振動
    float vib_accel_k;            // K_accel: σ = K × duty²
    float vib_gyro_k;             // K_gyro
    float vib_freq_low;           // [Hz] bandpass lower bound
    float vib_freq_high;          // [Hz] bandpass upper bound
};
```

## 5. SIL物理モデルの修正事項

### 比力（specific force）の正しい計算

```
加速度計出力 = R_nb × (a_ned - g_ned)
            = R_nb × ((thrust_ned + drag) / mass)
```

現在の`getSensors()`は推力加速度を含んでいない。修正必須。

### バイアス初期化

vehicleファームは`setAttitudeReference()`で`ba_z ≈ 2g`をセットする。
SILでも起動キャリブレーションフローを再現する必要がある。

### 推力の二乗則

```
thrust = k_thrust × duty²    （現在は線形: k_thrust × duty）
```

### オイラー方程式のジャイロスコピック項

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

## 7. 実装ロードマップ

| Phase | 内容 | 教育目標 |
|-------|------|---------|
| 0 | 比力修正 + バイアス初期化 | SILが正常に飛ぶ |
| 1 | IMUガウスノイズ + バイアス | KFの基礎 |
| 2 | スロットル依存ノイズ | 振動とノイズの関係 |
| 3 | 帯域制限 + ToF/Baroノイズ | フィルタ設計の動機 |
| 4 | 正弦波BPF + Flow | ノッチフィルタ、高度依存ノイズ |
| 5 | 実機FFTプロファイル | モデル検証 |

---

<a id="english"></a>

## 1. Overview

This document defines the sensor noise and vibration models for the SIL simulator.
Target: university-level control engineering education with physical validity.

### Key Insight

Datasheet noise values and actual in-flight noise differ by **orders of magnitude**.
The primary cause is **motor/propeller vibration**, making static noise models insufficient.

## 2-7. (See Japanese sections above for complete specifications)

### Summary Tables

**Q Matrix (Process Noise):**
- VEL: accel_noise² × Δt = 2.25e-4
- ATT: gyro_noise² × Δt = 2.3e-7
- BG: gyro_bias_noise² × Δt = 4.2e-13
- BA: accel_bias_noise² × Δt = 2.5e-11

**Vibration Model Levels:**
- L0: White Gaussian (basic KF understanding)
- L1: Throttle-dependent (noise-throttle correlation)
- **L2: L1 + band-limited (LPF motivation) — Recommended**
- L3: L2 + sinusoidal BPF (notch filter design)
- L4: Real FFT profile injection (validation)

**SIL Fix Priority:**
1. Fix specific force calculation (include thrust)
2. Add startup calibration (bias initialization)
3. Add sensor noise (Gaussian + bias + vibration)
4. Quadratic thrust model
5. Euler equation gyroscopic terms
