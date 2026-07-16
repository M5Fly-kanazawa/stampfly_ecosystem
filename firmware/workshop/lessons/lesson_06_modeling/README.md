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
| モータ時定数 | τ_m | 0.02 | 0.02 | 0.02† | s |
| 実効プラントゲイン | K | ~102 | ~70 | ~19 | rad/s² |

> **【確定 2026-07-15】** τ_m = 0.02 s は一連の実測で**裏付けが取れた**。
> コーストダウン試験（J/C_Q = 335 s·rad の直接測定）、プロペラ上面写真の画素直接積分
> （J_prop = 1.03e-8）、回転子実測諸元（J_rotor = 3.45e-9）、Ke = 5.33e-4（コーストダウン
> 開回路EMFの直接測定。独立の無負荷V-ω同定 5.35e-4 と0.5%一致）から、ホバ点の実効時定数は
> 完全導通（γ=1）で ≈0.018 s、飛行同定の 0.02〜0.034 s は γ≈0.2〜0.8 に相当し、
> 本表の 0.02 は実測と整合する。
> （経緯: 一時 J の旧推定 5.31e-8 に基づき 0.05 と誤訂正した。J の旧値は
> プロペラ2倍・回転子10倍の過大と判明し、正しくは J = 1.375e-8。ゲイン設計表は
> 0.02 前提のまま有効。詳細は multicopter_introduction/notes/qa_log.md Q4-9〜Q4-13）
> † ヨーは加速反トルクの零点（τ_z ≈ 46 ms）により実効的にさらに速い（零点符号は検証中）。

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
| Motor time constant | τ_m | 0.02 | 0.02 | 0.02† | s |
| Effective plant gain | K | ~102 | ~70 | ~19 | rad/s² |

> **[Confirmed 2026-07-15]** τ_m = 0.02 s is now backed by measurement: the coast-down test
> (direct J/C_Q = 335 s·rad), pixel-integration of the propeller photo (J_prop = 1.03e-8),
> measured rotor-cup dimensions (J_rotor = 3.45e-9), and Ke = 5.5e-4 (no-load V-ω; cross-checked by
> open-circuit coast-down EMF after instrument-gain correction) give a hover-point effective time
> constant τ_eff ≈ 0.017 s at full conduction (γ=1); the flight-identified 0.02–0.034 s corresponds
> to γ≈0.2–0.76, consistent with 0.02 in this table.
> (History: briefly mis-corrected to 0.05 based on the old J = 5.31e-8, which turned out to be
> 2× too large for the prop and 10× for the rotor; the correct J is 1.375e-8. The gain design
> tables below remain valid as computed with 0.02. See multicopter_introduction qa_log Q4-9..13.)
> † Yaw is effectively faster still, due to the acceleration-reaction zero (τ_z ≈ 46 ms; zero sign under review).

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
