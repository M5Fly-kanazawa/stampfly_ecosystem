# Lesson 6: システムモデリング / System Modeling

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このレッスンについて

StampFly の1軸角速度制御系を伝達関数でモデル化し、2次系理論に基づいて P ゲインを設計する。
L05 で経験的に選んだ `Kp=0.5` がモデル上どのような減衰比 ζ を与えるかを確認し、
目標 ζ から逆算して各軸に最適な Kp を求める。

### 前提知識

- L05: レート P 制御と初フライト

## 2. プラント伝達関数

### ブロック図

```
u ──▶ [Mixer + Motor: Km/(τm·s+1)] ──τ──▶ [Body: 1/(I·s)] ──▶ ω
```

### 統合伝達関数

```
G_p(s) = K / (s * (τ_m * s + 1)),    K = K_m / I
```

| ブロック | 物理的意味 | 伝達関数 |
|---------|----------|---------|
| Mixer + Motor | duty → トルク（1次遅れ） | Km / (τm·s + 1) |
| Body | トルク → 角速度（積分器） | 1 / (I·s) |

## 3. P 制御の閉ループ

### 閉ループ伝達関数

```
G_cl(s) = Kp·K / (τ_m·s² + s + Kp·K)
```

### 2次系パラメータ

```
ωn = √(Kp·K / τ_m)
ζ  = 1 / (2·√(Kp·K·τ_m))
```

### 設計式

```
Kp = 1 / (4·ζ²·K·τ_m)
```

## 4. StampFly のパラメータ

| パラメータ | 記号 | Roll | Pitch | Yaw | 単位 |
|-----------|------|------|-------|-----|------|
| 慣性モーメント | I | 9.16e-6 | 13.3e-6 | 20.4e-6 | kg·m² |
| モータ時定数 | τ_m | ~0.05（暫定） | ~0.05（暫定） | ~0.05（暫定）† | s |
| 実効プラントゲイン | K | ~102 | ~70 | ~19 | rad/s² |

> **【訂正 2026-07-15】** 旧値 τ_m = 0.02 s は出所未確認（従来からの思い込みの可能性）のため上書き。
> 新値はフォトインタラプタによる回転過渡実測（Duty10%、63%立ち上がり 0.20 s）と
> モータドライバ論文の同定値（J=5.31e-8 kg·m²・暫定）から求めたホバ点の局所時定数 ≈ 0.05 s。
> J はコーストダウン試験等での再確定待ちのため**暫定**。確定後、下のゲイン設計表も再計算すること。
> † ヨーは加速反トルクの零点により実効的にさらに速い可能性がある（yaw_axis_model.md の零点符号は検証中）。

### ゲイン設計表

| 軸 | K | Kp=0.5 の ζ | ζ=0.7 の Kp | ζ=1.0 の Kp |
|---|---|-----------|-----------|-----------|
| Roll | 102 | 0.50 | 0.25 | 0.12 |
| Pitch | 70 | 0.60 | 0.36 | 0.18 |
| Yaw | 19 | 1.15 | 1.34 | 0.66 |

### なぜ軸ごとに Kp を変えるべきか

- Roll: `Kp=0.5` → `ζ=0.50`（やや振動的、~16% オーバーシュート）
- Pitch: `Kp=0.5` → `ζ=0.60`（適度なオーバーシュート）
- Yaw: `Kp=0.5` → `ζ=1.15`（過減衰、応答が遅い）

全軸同じ Kp では最適な応答が得られない。モデルベースの設計で軸ごとに調整する。

## 5. ループシェイピング（周波数領域）

### 開ループ伝達関数

```
L(s) = Kp · G_p(s) = Kp·K / (s·(τm·s + 1))
```

- ωgc: |L(jωgc)| = 1 となるゲイン交差周波数
- 位相余裕 PM = 90° − arctan(τm·ωgc)
- Kp を上げると ωgc ↑ → PM ↓ → 振動的

### 無駄時間の影響

実システムには制御ループ遅延 τd ≈ 5ms（@400Hz）が存在する:

```
L_delay(s) = L(s) · e^(−τd·s)
```

- ゲインは変わらない（|e^(−jωτd)| = 1）
- 位相が −τd·ω [rad] だけ追加で低下

| 設定 | モデル PM | 実機 PM（τd=5ms） | 判定 |
|------|---------|----------------|------|
| Kp=0.5（ωgc=40） | 51° | ≈40° | 60° 未達 |
| Kp=0.25（ωgc=23） | 65° | ≈59° | ギリギリ |

- 実用上 PM ≥ 60° が必要
- ζ=0.7 設計でも遅延込みでボーダーライン
- 「モデル上は安定」でも実機で振動する理由の説明になる

## 6. 実習手順

### ステップ 1: 設計式を実装

`user_code.cpp` に設計式を書く:

```cpp
float zeta = 0.7f;
float Kp_roll  = 1.0f / (4.0f * zeta * zeta * K_roll  * tau_m);
float Kp_pitch = 1.0f / (4.0f * zeta * zeta * K_pitch * tau_m);
float Kp_yaw   = 1.0f / (4.0f * zeta * zeta * K_yaw   * tau_m);
```

### ステップ 2: 比較飛行

1. まず `zeta = 0.7` で飛行
2. `zeta = 0.5` に変更して飛行（L05 の Kp=0.5 相当）
3. `zeta = 1.0` に変更して飛行（臨界減衰）
4. 違いを体感する

### チャレンジ

- `sf log wifi` でステップ応答を記録し、理論値と比較する
- Roll と Yaw の応答の違いを ζ の差で説明する

## 7. 参考資料

| リソース | 説明 |
|---------|------|
| `tools/sysid/defaults.py` | デフォルト物理パラメータ |
| `sf sim run loop_shaping` | インタラクティブ PID チューニング |
| `sf log wifi` | リアルタイムテレメトリ取得 |

---

<a id="english"></a>

## 1. Overview

### About This Lesson

Model the single-axis angular rate control of StampFly using transfer functions,
then design P gains based on second-order system theory.
Verify what damping ratio ζ the empirical `Kp=0.5` from L05 actually provides,
and compute optimal per-axis Kp from a target ζ.

### Prerequisites

- L05: Rate P control and first flight

## 2. Plant Transfer Function

### Block Diagram

```
u ──▶ [Mixer + Motor: Km/(τm·s+1)] ──τ──▶ [Body: 1/(I·s)] ──▶ ω
```

### Combined Transfer Function

```
G_p(s) = K / (s * (τ_m * s + 1)),    K = K_m / I
```

| Block | Physical Meaning | Transfer Function |
|-------|-----------------|-------------------|
| Mixer + Motor | duty → torque (1st-order lag) | Km / (τm·s + 1) |
| Body | torque → angular rate (integrator) | 1 / (I·s) |

## 3. Closed-Loop with P Control

### Closed-Loop Transfer Function

```
G_cl(s) = Kp·K / (τ_m·s² + s + Kp·K)
```

### Second-Order Parameters

```
ωn = √(Kp·K / τ_m)
ζ  = 1 / (2·√(Kp·K·τ_m))
```

### Design Formula

```
Kp = 1 / (4·ζ²·K·τ_m)
```

## 4. StampFly Parameters

| Parameter | Symbol | Roll | Pitch | Yaw | Unit |
|-----------|--------|------|-------|-----|------|
| Moment of inertia | I | 9.16e-6 | 13.3e-6 | 20.4e-6 | kg·m² |
| Motor time constant | τ_m | ~0.05 (prov.) | ~0.05 (prov.) | ~0.05 (prov.)† | s |
| Effective plant gain | K | ~102 | ~70 | ~19 | rad/s² |

> **[Correction 2026-07-15]** The old value τ_m = 0.02 s had no verified provenance and is
> superseded. The new value is the hover-point local time constant ≈ 0.05 s derived from the
> photointerrupter spin-up measurement (Duty 10%, 63% rise 0.20 s) and the motor-driver paper's
> identified parameters (J = 5.31e-8 kg·m², provisional). J awaits confirmation (coast-down test);
> recompute the gain design tables below once J is settled.
> † Yaw may be effectively faster due to the acceleration-reaction zero (zero sign under review).

### Gain Design Table

| Axis | K | ζ at Kp=0.5 | Kp for ζ=0.7 | Kp for ζ=1.0 |
|------|---|-------------|-------------|-------------|
| Roll | 102 | 0.50 | 0.25 | 0.12 |
| Pitch | 70 | 0.60 | 0.36 | 0.18 |
| Yaw | 19 | 1.15 | 1.34 | 0.66 |

### Why Per-Axis Kp Matters

- Roll: `Kp=0.5` → `ζ=0.50` (slightly oscillatory, ~16% overshoot)
- Pitch: `Kp=0.5` → `ζ=0.60` (moderate overshoot)
- Yaw: `Kp=0.5` → `ζ=1.15` (overdamped, sluggish response)

A single Kp for all axes cannot achieve optimal response. Model-based design enables per-axis tuning.

## 5. Loop Shaping (Frequency Domain)

### Open-Loop Transfer Function

```
L(s) = Kp · G_p(s) = Kp·K / (s·(τm·s + 1))
```

- ωgc: gain crossover frequency where |L(jωgc)| = 1
- Phase margin PM = 90° − arctan(τm·ωgc)
- Increasing Kp → higher ωgc → lower PM → more oscillatory

### Dead Time Effect

Real systems have a control loop delay τd ≈ 5ms (@400Hz):

```
L_delay(s) = L(s) · e^(−τd·s)
```

- Magnitude is unchanged (|e^(−jωτd)| = 1)
- Phase drops by an additional −τd·ω [rad]

| Setting | Model PM | Actual PM (τd=5ms) | Verdict |
|---------|----------|--------------------|---------|
| Kp=0.5 (ωgc=40) | 51° | ≈40° | Below 60° |
| Kp=0.25 (ωgc=23) | 65° | ≈59° | Borderline |

- Practical requirement: PM ≥ 60°
- Even ζ=0.7 design is borderline with delay
- Explains why "stable on paper" can oscillate on real hardware

## 6. Hands-on Procedure

### Step 1: Implement the Design Formula

Write the design formula in `user_code.cpp`:

```cpp
float zeta = 0.7f;
float Kp_roll  = 1.0f / (4.0f * zeta * zeta * K_roll  * tau_m);
float Kp_pitch = 1.0f / (4.0f * zeta * zeta * K_pitch * tau_m);
float Kp_yaw   = 1.0f / (4.0f * zeta * zeta * K_yaw   * tau_m);
```

### Step 2: Comparison Flights

1. Fly with `zeta = 0.7` first
2. Change to `zeta = 0.5` (equivalent to L05's Kp=0.5 for roll)
3. Change to `zeta = 1.0` (critically damped)
4. Feel the difference

### Challenge

- Record step responses with `sf log wifi` and compare with theoretical values
- Explain the difference between Roll and Yaw responses using ζ

## 7. References

| Resource | Description |
|----------|-------------|
| `tools/sysid/defaults.py` | Default physical parameters |
| `sf sim run loop_shaping` | Interactive PID tuning |
| `sf log wifi` | Real-time telemetry capture |
