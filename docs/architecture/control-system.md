# StampFly 制御系設計

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

本ドキュメントは、StampFlyの制御系設計の数学的基礎を記述する。
マルチコプターの6自由度運動方程式、キネマティクス、姿勢表現について解説し、
シミュレータおよびファームウェアの実装根拠を示す。

### 座標系の定義

本ドキュメントでは以下の座標系を使用する：

```
慣性座標系 (Inertial Frame, I)        機体座標系 (Body Frame, B)
      Z_I (上)                              Z_B (上)
       ↑                                     ↑
       │                                     │    X_B (前)
       │                                     │   ╱
       │                                     │  ╱
       └──────→ Y_I (東)                     └──────→ Y_B (右)
      ╱
     ╱
    ↙
   X_I (北)

NED (North-East-Down) ではなく NWU (North-West-Up) 系を採用
```

| 座標系 | 原点 | 軸 | 用途 |
|--------|------|-----|------|
| 慣性座標系 (I) | 地表の固定点 | X:北, Y:西, Z:上 | 位置・速度の記述 |
| 機体座標系 (B) | 機体重心 | X:前, Y:左, Z:上 | 力・モーメントの記述 |

### 状態変数

| 記号 | 説明 | 座標系 | 単位 |
|------|------|--------|------|
| **位置** | | | |
| x, y, z | 位置 | 慣性系 | m |
| **速度** | | | |
| ẋ, ẏ, ż | 慣性系速度 | 慣性系 | m/s |
| u, v, w | 機体系速度 | 機体系 | m/s |
| **姿勢** | | | |
| φ, θ, ψ | オイラー角 (Roll, Pitch, Yaw) | - | rad |
| q₀, q₁, q₂, q₃ | クォータニオン | - | - |
| **角速度** | | | |
| p, q, r | 機体角速度 | 機体系 | rad/s |

## 2. 6自由度運動方程式

### 並進運動方程式

機体座標系における並進運動方程式は、ニュートンの第2法則より：

```
m(dV_B/dt + ω × V_B) = F_B
```

ここで：
- m : 機体質量 [kg]
- V_B = [u, v, w]ᵀ : 機体系速度ベクトル [m/s]
- ω = [p, q, r]ᵀ : 機体角速度ベクトル [rad/s]
- F_B : 機体系に作用する力 [N]

成分で展開すると：

```
m(u̇ + qw - rv) = F_x
m(v̇ + ru - pw) = F_y
m(ẇ + pv - qu) = F_z
```

整理して：

```
u̇ = F_x/m - qw + rv
v̇ = F_y/m - ru + pw
ẇ = F_z/m - pv + qu
```

行列形式：

```
┌   ┐   ┌     ┐   ┌        ┐   ┌   ┐
│ u̇ │   │ F_x │   │  0  r -q│   │ u │
│ v̇ │ = │ F_y │/m - │ -r  0  p│ × │ v │
│ ẇ │   │ F_z │   │  q -p  0│   │ w │
└   ┘   └     ┘   └        ┘   └   ┘
```

### 回転運動方程式

機体座標系における回転運動方程式は、オイラーの運動方程式より：

```
I(dω/dt) + ω × (Iω) = τ
```

ここで：
- I : 慣性テンソル [kg·m²]
- τ = [L, M, N]ᵀ : 機体系に作用するモーメント [N·m]

StampFlyは対称形状のため、慣性テンソルは対角行列：

```
    ┌           ┐
I = │ I_xx  0    0  │
    │  0   I_yy  0  │
    │  0    0  I_zz │
    └           ┘
```

成分で展開すると：

```
I_xx·ṗ + (I_zz - I_yy)·q·r = L
I_yy·q̇ + (I_xx - I_zz)·p·r = M
I_zz·ṙ + (I_yy - I_xx)·p·q = N
```

整理して：

```
ṗ = [L - (I_zz - I_yy)·q·r] / I_xx
q̇ = [M - (I_xx - I_zz)·p·r] / I_yy
ṙ = [N - (I_yy - I_xx)·p·q] / I_zz
```

### 作用する力とモーメント

#### 推力

4つのモーターからの合計推力（機体Z軸負方向）：

```
F_thrust = [0, 0, -(T₁ + T₂ + T₃ + T₄)]ᵀ
```

各モーターの推力：
```
T_i = C_t · ω_i²
```

| 記号 | 値 | 単位 | 説明 |
|------|-----|------|------|
| C_t | 1.00×10⁻⁸ | N/(rad/s)² | 推力係数 |

#### 重力

慣性系での重力を機体系に変換：

```
F_gravity_I = [0, 0, mg]ᵀ  (上向き正のため正)
F_gravity_B = R_BI · F_gravity_I
```

ここで R_BI は慣性系から機体系への回転行列（DCMの転置）。

#### モーメント

各モーターが生成するモーメント：

```
τ_total = Σ(r_i × F_i) + Σ(τ_reaction_i)
```

1. **推力によるモーメント（アーム長 × 推力）**

```
L = d·(T₃ + T₄ - T₁ - T₂)  (Roll)
M = d·(T₁ + T₄ - T₂ - T₃)  (Pitch)
```

2. **反トルク（モーター回転の反作用）**

```
N = C_q·(ω₂² + ω₄² - ω₁² - ω₃²)  (Yaw)
```

| モーター | 位置 | 回転方向 | Roll寄与 | Pitch寄与 | Yaw寄与 |
|---------|------|---------|---------|----------|---------|
| M1 (FR) | (+,+) | CCW | - | + | + |
| M2 (RR) | (-,+) | CW | - | - | - |
| M3 (RL) | (-,-) | CCW | + | - | + |
| M4 (FL) | (+,-) | CW | + | + | - |

#### 空気抵抗

並進抵抗（速度の2乗に比例）：

```
F_drag_u = -C_d · sign(u) · u²
F_drag_v = -C_d · sign(v) · v²
F_drag_w = -C_d · sign(w) · w²
```

回転抵抗（角速度の2乗に比例）：

```
τ_drag_p = -C_r · sign(p) · p²
τ_drag_q = -C_r · sign(q) · q²
τ_drag_r = -C_r · sign(r) · r²
```

| 記号 | 値 | 単位 | 説明 |
|------|-----|------|------|
| C_d | 0.1 | - | 並進抗力係数 |
| C_r | 1×10⁻⁵ | - | 回転抗力係数 |

## 3. キネマティクス（位置・姿勢の算出）

### 位置の算出

慣性系位置は、機体系速度を回転行列で変換して積分：

```
┌   ┐       ┌   ┐
│ ẋ │       │ u │
│ ẏ │ = R · │ v │
│ ż │       │ w │
└   ┘       └   ┘
```

ここで R は機体系から慣性系への回転行列（DCM: Direction Cosine Matrix）。

### 姿勢の算出

姿勢の時間発展には2つの表現方法がある。

## 4. 姿勢表現：オイラー角

### 定義

オイラー角は3つの連続回転で姿勢を表現する。
本システムでは **Z-Y-X（Yaw-Pitch-Roll）** 順序を採用：

```
1. ψ (Yaw)   : Z軸周りに回転
2. θ (Pitch) : Y軸周りに回転
3. φ (Roll)  : X軸周りに回転
```

### 回転行列（DCM）

オイラー角から回転行列への変換：

```
R = R_x(φ) · R_y(θ) · R_z(ψ)
```

各軸周りの基本回転行列：

```
         ┌              ┐
R_x(φ) = │ 1    0      0   │
         │ 0   cos(φ) -sin(φ)│
         │ 0   sin(φ)  cos(φ)│
         └              ┘

         ┌               ┐
R_y(θ) = │ cos(θ)  0  sin(θ) │
         │   0     1    0    │
         │-sin(θ)  0  cos(θ) │
         └               ┘

         ┌              ┐
R_z(ψ) = │ cos(ψ) -sin(ψ)  0 │
         │ sin(ψ)  cos(ψ)  0 │
         │   0      0      1 │
         └              ┘
```

合成した回転行列（機体系→慣性系）：

```
    ┌                                                    ┐
R = │ cθcψ           cθsψ            -sθ                │
    │ sφsθcψ-cφsψ    sφsθsψ+cφcψ     sφcθ               │
    │ cφsθcψ+sφsψ    cφsθsψ-sφcψ     cφcθ               │
    └                                                    ┘
```

（cφ = cos(φ), sφ = sin(φ) の略記）

### オイラー角の時間微分

角速度 [p, q, r]ᵀ からオイラー角の時間微分への変換：

```
┌   ┐   ┌                          ┐   ┌   ┐
│ φ̇ │   │ 1   sin(φ)tan(θ)   cos(φ)tan(θ) │   │ p │
│ θ̇ │ = │ 0      cos(φ)        -sin(φ)     │ · │ q │
│ ψ̇ │   │ 0   sin(φ)/cos(θ)  cos(φ)/cos(θ)│   │ r │
└   ┘   └                          ┘   └   ┘
```

**注意：ジンバルロック**

θ = ±90° のとき cos(θ) = 0 となり、変換行列が特異になる。
これを **ジンバルロック** と呼び、オイラー角表現の根本的な制限である。

| 条件 | 問題 |
|------|------|
| θ → ±90° | tan(θ) → ∞, 1/cos(θ) → ∞ |
| 結果 | φ̇, ψ̇ が発散、姿勢の一意性喪失 |

→ **解決策：クォータニオンを使用する**

## 5. 姿勢表現：クォータニオン

### 定義

クォータニオンは4つの成分で姿勢を表現する：

```
q = q₀ + q₁i + q₂j + q₃k = [q₀, q₁, q₂, q₃]ᵀ
```

ここで：
- q₀ : スカラー部（実部）
- q₁, q₂, q₃ : ベクトル部（虚部）
- 単位クォータニオン制約：q₀² + q₁² + q₂² + q₃² = 1

### 回転の幾何学的意味

単位クォータニオンは、軸 **n** = [nₓ, nᵧ, n_z]ᵀ 周りの角度 **θ** の回転を表す：

```
q₀ = cos(θ/2)
q₁ = nₓ · sin(θ/2)
q₂ = nᵧ · sin(θ/2)
q₃ = n_z · sin(θ/2)
```

### クォータニオンから回転行列への変換

```
    ┌                                            ┐
R = │ q₀²+q₁²-q₂²-q₃²   2(q₁q₂-q₀q₃)     2(q₁q₃+q₀q₂)   │
    │ 2(q₁q₂+q₀q₃)     q₀²-q₁²+q₂²-q₃²   2(q₂q₃-q₀q₁)   │
    │ 2(q₁q₃-q₀q₂)     2(q₂q₃+q₀q₁)     q₀²-q₁²-q₂²+q₃² │
    └                                            ┘
```

### オイラー角からクォータニオンへの変換

```
q₀ = cos(φ/2)cos(θ/2)cos(ψ/2) + sin(φ/2)sin(θ/2)sin(ψ/2)
q₁ = sin(φ/2)cos(θ/2)cos(ψ/2) - cos(φ/2)sin(θ/2)sin(ψ/2)
q₂ = cos(φ/2)sin(θ/2)cos(ψ/2) + sin(φ/2)cos(θ/2)sin(ψ/2)
q₃ = cos(φ/2)cos(θ/2)sin(ψ/2) - sin(φ/2)sin(θ/2)cos(ψ/2)
```

### クォータニオンからオイラー角への変換

```
φ = atan2(2(q₀q₁ + q₂q₃), q₀² - q₁² - q₂² + q₃²)
θ = asin(2(q₀q₂ - q₁q₃))
ψ = atan2(2(q₀q₃ + q₁q₂), q₀² + q₁² - q₂² - q₃²)
```

**注意：** asin の引数が ±1 を超える場合はクリッピングが必要。

### クォータニオンの時間微分

角速度 [p, q, r]ᵀ からクォータニオンの時間微分：

```
┌    ┐       ┌              ┐   ┌    ┐
│ q̇₀ │       │ 0  -p  -q  -r │   │ q₀ │
│ q̇₁ │ = 1/2 │ p   0   r  -q │ · │ q₁ │
│ q̇₂ │       │ q  -r   0   p │   │ q₂ │
│ q̇₃ │       │ r   q  -p   0 │   │ q₃ │
└    ┘       └              ┘   └    ┘
```

この微分方程式は **特異点を持たない**（ジンバルロックが発生しない）。

### 正規化

数値積分の誤差蓄積により単位制約が崩れるため、各ステップで正規化：

```
q ← q / |q|
```

### オイラー角とクォータニオンの比較

| 項目 | オイラー角 | クォータニオン |
|------|-----------|---------------|
| パラメータ数 | 3 | 4 |
| 特異点 | あり（ジンバルロック） | なし |
| 直感性 | 高い（角度で理解しやすい） | 低い |
| 計算コスト | 三角関数多用 | 乗算主体 |
| 補間 | 困難 | SLERP可能 |
| 制約 | なし | |q| = 1 |
| 用途 | 表示・入出力 | 内部演算・積分 |

**StampFlyでの使い分け：**
- 内部計算（シミュレータ・AHRS）：クォータニオン
- ユーザー表示・ログ出力：オイラー角に変換

## 6. 数値積分

### Runge-Kutta 4次法（RK4）

シミュレータでは4次のRunge-Kutta法を使用：

```
k₁ = f(t, y)
k₂ = f(t + h/2, y + h·k₁/2)
k₃ = f(t + h/2, y + h·k₂/2)
k₄ = f(t + h, y + h·k₃)

y(t+h) = y(t) + h·(k₁ + 2k₂ + 2k₃ + k₄)/6
```

| パラメータ | 値 | 単位 |
|-----------|-----|------|
| 積分刻み幅 h | 0.001 | s (1 kHz) |
| 精度 | O(h⁴) | - |

### 積分対象

RK4で積分する状態変数：

| 変数 | 微分方程式 |
|------|-----------|
| u, v, w | 並進運動方程式 |
| p, q, r | 回転運動方程式 |
| q₀, q₁, q₂, q₃ | クォータニオン運動学 |
| x, y, z | 位置運動学 |

## 7. 実装リファレンス

### シミュレータコード

| 機能 | ファイル | クラス/関数 |
|------|---------|------------|
| 剛体運動 | `simulator/core/physics.py` | `rigidbody` |
| マルチコプター | `simulator/core/dynamics.py` | `multicopter` |
| モーター | `simulator/core/motors.py` | `motor_prop` |

### 主要メソッド

```python
# physics.py
rigidbody.uvw_dot()      # 並進加速度
rigidbody.pqr_dot()      # 角加速度
rigidbody.quat_dot()     # クォータニオン微分
rigidbody.position_dot() # 速度→位置
rigidbody.step()         # RK4積分

# 座標変換
rigidbody.quat_dcm()     # クォータニオン→DCM
rigidbody.euler2quat()   # オイラー角→クォータニオン
rigidbody.quat2euler()   # クォータニオン→オイラー角
```

---

<a id="english"></a>

## 1. Overview

### About This Document

This document describes the mathematical foundations of StampFly's control system design.
It covers the 6-DOF equations of motion, kinematics, and attitude representations,
providing the rationale for simulator and firmware implementations.

### Coordinate System Definitions

This document uses the following coordinate systems:

```
Inertial Frame (I)                    Body Frame (B)
      Z_I (Up)                              Z_B (Up)
       ↑                                     ↑
       │                                     │    X_B (Front)
       │                                     │   ╱
       │                                     │  ╱
       └──────→ Y_I (East)                   └──────→ Y_B (Right)
      ╱
     ╱
    ↙
   X_I (North)

NWU (North-West-Up) system adopted, not NED
```

| Frame | Origin | Axes | Usage |
|-------|--------|------|-------|
| Inertial (I) | Fixed point on ground | X:North, Y:West, Z:Up | Position, velocity |
| Body (B) | Vehicle CG | X:Front, Y:Left, Z:Up | Forces, moments |

### State Variables

| Symbol | Description | Frame | Unit |
|--------|-------------|-------|------|
| **Position** | | | |
| x, y, z | Position | Inertial | m |
| **Velocity** | | | |
| ẋ, ẏ, ż | Inertial velocity | Inertial | m/s |
| u, v, w | Body velocity | Body | m/s |
| **Attitude** | | | |
| φ, θ, ψ | Euler angles (Roll, Pitch, Yaw) | - | rad |
| q₀, q₁, q₂, q₃ | Quaternion | - | - |
| **Angular velocity** | | | |
| p, q, r | Body angular velocity | Body | rad/s |

## 2. 6-DOF Equations of Motion

### Translational Equations

From Newton's second law in the body frame:

```
m(dV_B/dt + ω × V_B) = F_B
```

Where:
- m : Vehicle mass [kg]
- V_B = [u, v, w]ᵀ : Body velocity vector [m/s]
- ω = [p, q, r]ᵀ : Body angular velocity vector [rad/s]
- F_B : Forces in body frame [N]

Expanded:

```
u̇ = F_x/m - qw + rv
v̇ = F_y/m - ru + pw
ẇ = F_z/m - pv + qu
```

### Rotational Equations

From Euler's equation of motion in the body frame:

```
I(dω/dt) + ω × (Iω) = τ
```

For diagonal inertia tensor (symmetric vehicle):

```
ṗ = [L - (I_zz - I_yy)·q·r] / I_xx
q̇ = [M - (I_xx - I_zz)·p·r] / I_yy
ṙ = [N - (I_yy - I_xx)·p·q] / I_zz
```

### Forces and Moments

#### Thrust

Total thrust from 4 motors (negative Z in body frame):

```
F_thrust = [0, 0, -(T₁ + T₂ + T₃ + T₄)]ᵀ
T_i = C_t · ω_i²
```

#### Moments

```
L = d·(T₃ + T₄ - T₁ - T₂)  (Roll)
M = d·(T₁ + T₄ - T₂ - T₃)  (Pitch)
N = C_q·(ω₂² + ω₄² - ω₁² - ω₃²)  (Yaw - reaction torque)
```

## 3. Kinematics

### Position Update

Position in inertial frame from body velocity:

```
[ẋ, ẏ, ż]ᵀ = R · [u, v, w]ᵀ
```

Where R is the DCM (Direction Cosine Matrix) from body to inertial frame.

## 4. Attitude Representation: Euler Angles

### Definition

Euler angles represent attitude as three sequential rotations.
This system uses **Z-Y-X (Yaw-Pitch-Roll)** order:

1. ψ (Yaw): Rotation about Z-axis
2. θ (Pitch): Rotation about Y-axis
3. φ (Roll): Rotation about X-axis

### Rotation Matrix (DCM)

Combined rotation matrix (body → inertial):

```
    ┌                                                    ┐
R = │ cθcψ           cθsψ            -sθ                │
    │ sφsθcψ-cφsψ    sφsθsψ+cφcψ     sφcθ               │
    │ cφsθcψ+sφsψ    cφsθsψ-sφcψ     cφcθ               │
    └                                                    ┘
```

### Euler Angle Derivatives

```
┌   ┐   ┌                          ┐   ┌   ┐
│ φ̇ │   │ 1   sin(φ)tan(θ)   cos(φ)tan(θ) │   │ p │
│ θ̇ │ = │ 0      cos(φ)        -sin(φ)     │ · │ q │
│ ψ̇ │   │ 0   sin(φ)/cos(θ)  cos(φ)/cos(θ)│   │ r │
└   ┘   └                          ┘   └   ┘
```

**Gimbal Lock:** At θ = ±90°, cos(θ) = 0 causes singularity.

## 5. Attitude Representation: Quaternion

### Definition

Quaternion represents attitude with 4 components:

```
q = [q₀, q₁, q₂, q₃]ᵀ
```

With unit constraint: q₀² + q₁² + q₂² + q₃² = 1

### Quaternion to DCM

```
    ┌                                            ┐
R = │ q₀²+q₁²-q₂²-q₃²   2(q₁q₂-q₀q₃)     2(q₁q₃+q₀q₂)   │
    │ 2(q₁q₂+q₀q₃)     q₀²-q₁²+q₂²-q₃²   2(q₂q₃-q₀q₁)   │
    │ 2(q₁q₃-q₀q₂)     2(q₂q₃+q₀q₁)     q₀²-q₁²-q₂²+q₃² │
    └                                            ┘
```

### Quaternion Derivatives

```
┌    ┐       ┌              ┐   ┌    ┐
│ q̇₀ │       │ 0  -p  -q  -r │   │ q₀ │
│ q̇₁ │ = 1/2 │ p   0   r  -q │ · │ q₁ │
│ q̇₂ │       │ q  -r   0   p │   │ q₂ │
│ q̇₃ │       │ r   q  -p   0 │   │ q₃ │
└    ┘       └              ┘   └    ┘
```

This equation has **no singularity** (no gimbal lock).

### Comparison

| Aspect | Euler Angles | Quaternion |
|--------|--------------|------------|
| Parameters | 3 | 4 |
| Singularity | Yes (gimbal lock) | None |
| Intuition | High | Low |
| Computation | Trigonometric | Multiplication |
| Interpolation | Difficult | SLERP possible |
| Constraint | None | \|q\| = 1 |
| Usage | Display/I/O | Internal/integration |

## 6. Numerical Integration

### Runge-Kutta 4th Order (RK4)

The simulator uses 4th order Runge-Kutta:

```
k₁ = f(t, y)
k₂ = f(t + h/2, y + h·k₁/2)
k₃ = f(t + h/2, y + h·k₂/2)
k₄ = f(t + h, y + h·k₃)

y(t+h) = y(t) + h·(k₁ + 2k₂ + 2k₃ + k₄)/6
```

| Parameter | Value | Unit |
|-----------|-------|------|
| Step size h | 0.001 | s (1 kHz) |
| Accuracy | O(h⁴) | - |

## 7. Implementation Reference

### Simulator Code

| Function | File | Class/Method |
|----------|------|--------------|
| Rigid body | `simulator/core/physics.py` | `rigidbody` |
| Multicopter | `simulator/core/dynamics.py` | `multicopter` |
| Motor | `simulator/core/motors.py` | `motor_prop` |

### Key Methods

```python
# physics.py
rigidbody.uvw_dot()      # Translational acceleration
rigidbody.pqr_dot()      # Angular acceleration
rigidbody.quat_dot()     # Quaternion derivative
rigidbody.position_dot() # Velocity to position
rigidbody.step()         # RK4 integration

# Coordinate transforms
rigidbody.quat_dcm()     # Quaternion to DCM
rigidbody.euler2quat()   # Euler to Quaternion
rigidbody.quat2euler()   # Quaternion to Euler
```
