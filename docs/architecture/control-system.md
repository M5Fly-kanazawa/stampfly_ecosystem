# StampFly 制御系設計

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

本ドキュメントは、StampFlyの制御系設計の数学的基礎を記述する。
マルチコプターの6自由度運動方程式、キネマティクス、姿勢表現について解説し、
シミュレータおよびファームウェアの実装根拠を示す。

### 座標系の定義

本ドキュメントでは **NED（North-East-Down）座標系** を採用する（右手系）：

```
慣性座標系 (Inertial Frame, I)        機体座標系 (Body Frame, B)

       X_I (北)                             X_B (前)
        ↑                                    ↑
        │                                    │
        │                                    │
        └──────→ Y_I (東)                    └──────→ Y_B (右)
         ╲                                    ╲
          ╲                                    ╲
           ↘                                    ↘
            Z_I (下)                             Z_B (下)
```

| 座標系 | 原点 | 軸 | 用途 |
|--------|------|-----|------|
| 慣性座標系 (I) | 地表の固定点 | X:北, Y:東, Z:下 | 位置・速度の記述 |
| 機体座標系 (B) | 機体重心 | X:前, Y:右, Z:下 | 力・モーメントの記述 |

**NED座標系の特徴：**
- 右手系（$\mathbf{X} \times \mathbf{Y} = \mathbf{Z}$）
- 高度が上がると $z$ は負になる
- 航空・ドローン分野で標準的に使用される

### 状態変数

| 記号 | 説明 | 座標系 | 単位 |
|------|------|--------|------|
| **位置** | | | |
| $x, y, z$ | 位置 | 慣性系 | m |
| **速度** | | | |
| $\dot{x}, \dot{y}, \dot{z}$ | 慣性系速度 | 慣性系 | m/s |
| $u, v, w$ | 機体系速度 | 機体系 | m/s |
| **姿勢** | | | |
| $\phi, \theta, \psi$ | オイラー角 (Roll, Pitch, Yaw) | - | rad |
| $q_0, q_1, q_2, q_3$ | クォータニオン | - | - |
| **角速度** | | | |
| $p, q, r$ | 機体角速度 | 機体系 | rad/s |

## 2. 6自由度運動方程式

### 並進運動方程式

機体座標系における並進運動方程式は、ニュートンの第2法則より：

$$
m\left(\frac{d\mathbf{V}_B}{dt} + \boldsymbol{\omega} \times \mathbf{V}_B\right) = \mathbf{F}_B
$$

ここで：
- $m$ : 機体質量 [kg]
- $\mathbf{V}_B = [u, v, w]^\top$ : 機体系速度ベクトル [m/s]
- $\boldsymbol{\omega} = [p, q, r]^\top$ : 機体角速度ベクトル [rad/s]
- $\mathbf{F}_B$ : 機体系に作用する力 [N]

成分で展開すると：

$$
\begin{align}
m(\dot{u} + qw - rv) &= F_x \\
m(\dot{v} + ru - pw) &= F_y \\
m(\dot{w} + pv - qu) &= F_z
\end{align}
$$

整理して：

$$
\begin{align}
\dot{u} &= \frac{F_x}{m} - qw + rv \\
\dot{v} &= \frac{F_y}{m} - ru + pw \\
\dot{w} &= \frac{F_z}{m} - pv + qu
\end{align}
$$

行列形式：

$$
\begin{bmatrix} \dot{u} \\ \dot{v} \\ \dot{w} \end{bmatrix} =
\frac{1}{m}\begin{bmatrix} F_x \\ F_y \\ F_z \end{bmatrix} -
\begin{bmatrix} 0 & -r & q \\ r & 0 & -p \\ -q & p & 0 \end{bmatrix}
\begin{bmatrix} u \\ v \\ w \end{bmatrix}
$$

### 回転運動方程式

機体座標系における回転運動方程式は、オイラーの運動方程式より：

$$
\mathbf{I}\frac{d\boldsymbol{\omega}}{dt} + \boldsymbol{\omega} \times (\mathbf{I}\boldsymbol{\omega}) = \boldsymbol{\tau}
$$

ここで：
- $\mathbf{I}$ : 慣性テンソル [kg·m²]
- $\boldsymbol{\tau} = [L, M, N]^\top$ : 機体系に作用するモーメント [N·m]

StampFlyは対称形状のため、慣性テンソルは対角行列：

$$
\mathbf{I} = \begin{bmatrix} I_{xx} & 0 & 0 \\ 0 & I_{yy} & 0 \\ 0 & 0 & I_{zz} \end{bmatrix}
$$

成分で展開すると：

$$
\begin{align}
I_{xx}\dot{p} + (I_{zz} - I_{yy})qr &= L \\
I_{yy}\dot{q} + (I_{xx} - I_{zz})pr &= M \\
I_{zz}\dot{r} + (I_{yy} - I_{xx})pq &= N
\end{align}
$$

整理して：

$$
\begin{align}
\dot{p} &= \frac{L - (I_{zz} - I_{yy})qr}{I_{xx}} \\
\dot{q} &= \frac{M - (I_{xx} - I_{zz})pr}{I_{yy}} \\
\dot{r} &= \frac{N - (I_{yy} - I_{xx})pq}{I_{zz}}
\end{align}
$$

### 作用する力とモーメント

#### 推力

4つのモーターからの合計推力（機体Z軸負方向＝上向き）：

$$
\mathbf{F}_{thrust} = \begin{bmatrix} 0 \\ 0 \\ -(T_1 + T_2 + T_3 + T_4) \end{bmatrix}
$$

各モーターの推力：

$$
T_i = C_t \omega_i^2
$$

| 記号 | 値 | 単位 | 説明 |
|------|-----|------|------|
| $C_t$ | $1.00 \times 10^{-8}$ | N/(rad/s)² | 推力係数 |

**注意（NED座標系）：** Z軸は下向きが正のため、上向きの推力は負の値となる。

#### 重力

慣性系での重力を機体系に変換：

$$
\mathbf{F}_{gravity}^I = \begin{bmatrix} 0 \\ 0 \\ mg \end{bmatrix} \quad \text{(NED: 下向きが正)}
$$

$$
\mathbf{F}_{gravity}^B = \mathbf{R}^\top \mathbf{F}_{gravity}^I
$$

ここで $\mathbf{R}^\top$ は慣性系から機体系への回転行列（DCMの転置）。

**注意（NED座標系）：** 重力は+Z方向（下向き）に作用する。

#### モーメント

各モーターが生成するモーメント：

$$
\boldsymbol{\tau}_{total} = \sum_i (\mathbf{r}_i \times \mathbf{F}_i) + \sum_i \boldsymbol{\tau}_{reaction,i}
$$

1. **推力によるモーメント（アーム長 × 推力）**

$$
\begin{align}
L &= d(T_3 + T_4 - T_1 - T_2) \quad \text{(Roll)} \\
M &= d(T_1 + T_4 - T_2 - T_3) \quad \text{(Pitch)}
\end{align}
$$

2. **反トルク（モーター回転の反作用）**

$$
N = C_q(\omega_2^2 + \omega_4^2 - \omega_1^2 - \omega_3^2) \quad \text{(Yaw)}
$$

| モーター | 位置 | 回転方向 | Roll寄与 | Pitch寄与 | Yaw寄与 |
|---------|------|---------|---------|----------|---------|
| M1 (FR) | (+,+) | CCW | - | + | + |
| M2 (RR) | (-,+) | CW | - | - | - |
| M3 (RL) | (-,-) | CCW | + | - | + |
| M4 (FL) | (+,-) | CW | + | + | - |

#### 空気抵抗

並進抵抗（速度の2乗に比例）：

$$
\begin{align}
F_{drag,u} &= -C_d \cdot \text{sign}(u) \cdot u^2 \\
F_{drag,v} &= -C_d \cdot \text{sign}(v) \cdot v^2 \\
F_{drag,w} &= -C_d \cdot \text{sign}(w) \cdot w^2
\end{align}
$$

回転抵抗（角速度の2乗に比例）：

$$
\begin{align}
\tau_{drag,p} &= -C_r \cdot \text{sign}(p) \cdot p^2 \\
\tau_{drag,q} &= -C_r \cdot \text{sign}(q) \cdot q^2 \\
\tau_{drag,r} &= -C_r \cdot \text{sign}(r) \cdot r^2
\end{align}
$$

| 記号 | 値 | 単位 | 説明 |
|------|-----|------|------|
| $C_d$ | 0.1 | - | 並進抗力係数 |
| $C_r$ | $1 \times 10^{-5}$ | - | 回転抗力係数 |

## 3. キネマティクス（位置・姿勢の算出）

### 位置の算出

慣性系位置は、機体系速度を回転行列で変換して積分：

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{z} \end{bmatrix} =
\mathbf{R} \begin{bmatrix} u \\ v \\ w \end{bmatrix}
$$

ここで $\mathbf{R}$ は機体系から慣性系への回転行列（DCM: Direction Cosine Matrix）。

### 姿勢の算出

姿勢の時間発展には2つの表現方法がある。

## 4. 姿勢表現：オイラー角

### 定義

オイラー角は3つの連続回転で姿勢を表現する。
本システムでは **Z-Y-X（Yaw-Pitch-Roll）** 順序を採用：

1. $\psi$ (Yaw) : Z軸周りに回転
2. $\theta$ (Pitch) : Y軸周りに回転
3. $\phi$ (Roll) : X軸周りに回転

### 回転行列（DCM）

オイラー角から回転行列への変換：

$$
\mathbf{R} = \mathbf{R}_x(\phi) \cdot \mathbf{R}_y(\theta) \cdot \mathbf{R}_z(\psi)
$$

各軸周りの基本回転行列：

$$
\mathbf{R}_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}
$$

$$
\mathbf{R}_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}
$$

$$
\mathbf{R}_z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}
$$

合成した回転行列（機体系→慣性系）：

$$
\mathbf{R} = \begin{bmatrix}
c_\theta c_\psi & c_\theta s_\psi & -s_\theta \\
s_\phi s_\theta c_\psi - c_\phi s_\psi & s_\phi s_\theta s_\psi + c_\phi c_\psi & s_\phi c_\theta \\
c_\phi s_\theta c_\psi + s_\phi s_\psi & c_\phi s_\theta s_\psi - s_\phi c_\psi & c_\phi c_\theta
\end{bmatrix}
$$

（$c_\phi = \cos\phi$, $s_\phi = \sin\phi$ の略記）

### オイラー角の時間微分

角速度 $[p, q, r]^\top$ からオイラー角の時間微分への変換：

$$
\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} =
\begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta
\end{bmatrix}
\begin{bmatrix} p \\ q \\ r \end{bmatrix}
$$

**注意：ジンバルロック**

$\theta = \pm 90°$ のとき $\cos\theta = 0$ となり、変換行列が特異になる。
これを **ジンバルロック** と呼び、オイラー角表現の根本的な制限である。

| 条件 | 問題 |
|------|------|
| $\theta \to \pm 90°$ | $\tan\theta \to \infty$, $1/\cos\theta \to \infty$ |
| 結果 | $\dot{\phi}, \dot{\psi}$ が発散、姿勢の一意性喪失 |

→ **解決策：クォータニオンを使用する**

## 5. 姿勢表現：クォータニオン

### 定義

クォータニオンは4つの成分で姿勢を表現する：

$$
\mathbf{q} = q_0 + q_1 i + q_2 j + q_3 k = [q_0, q_1, q_2, q_3]^\top
$$

ここで：
- $q_0$ : スカラー部（実部）
- $q_1, q_2, q_3$ : ベクトル部（虚部）
- 単位クォータニオン制約：$q_0^2 + q_1^2 + q_2^2 + q_3^2 = 1$

### 回転の幾何学的意味

単位クォータニオンは、軸 $\mathbf{n} = [n_x, n_y, n_z]^\top$ 周りの角度 $\theta$ の回転を表す：

$$
\begin{align}
q_0 &= \cos(\theta/2) \\
q_1 &= n_x \sin(\theta/2) \\
q_2 &= n_y \sin(\theta/2) \\
q_3 &= n_z \sin(\theta/2)
\end{align}
$$

### クォータニオンから回転行列への変換

$$
\mathbf{R} = \begin{bmatrix}
q_0^2+q_1^2-q_2^2-q_3^2 & 2(q_1 q_2 - q_0 q_3) & 2(q_1 q_3 + q_0 q_2) \\
2(q_1 q_2 + q_0 q_3) & q_0^2-q_1^2+q_2^2-q_3^2 & 2(q_2 q_3 - q_0 q_1) \\
2(q_1 q_3 - q_0 q_2) & 2(q_2 q_3 + q_0 q_1) & q_0^2-q_1^2-q_2^2+q_3^2
\end{bmatrix}
$$

### オイラー角からクォータニオンへの変換

$$
\begin{align}
q_0 &= \cos(\phi/2)\cos(\theta/2)\cos(\psi/2) + \sin(\phi/2)\sin(\theta/2)\sin(\psi/2) \\
q_1 &= \sin(\phi/2)\cos(\theta/2)\cos(\psi/2) - \cos(\phi/2)\sin(\theta/2)\sin(\psi/2) \\
q_2 &= \cos(\phi/2)\sin(\theta/2)\cos(\psi/2) + \sin(\phi/2)\cos(\theta/2)\sin(\psi/2) \\
q_3 &= \cos(\phi/2)\cos(\theta/2)\sin(\psi/2) - \sin(\phi/2)\sin(\theta/2)\cos(\psi/2)
\end{align}
$$

### クォータニオンからオイラー角への変換

$$
\begin{align}
\phi &= \text{atan2}(2(q_0 q_1 + q_2 q_3), q_0^2 - q_1^2 - q_2^2 + q_3^2) \\
\theta &= \arcsin(2(q_0 q_2 - q_1 q_3)) \\
\psi &= \text{atan2}(2(q_0 q_3 + q_1 q_2), q_0^2 + q_1^2 - q_2^2 - q_3^2)
\end{align}
$$

**注意：** $\arcsin$ の引数が $\pm 1$ を超える場合はクリッピングが必要。

### クォータニオンの時間微分

角速度 $[p, q, r]^\top$ からクォータニオンの時間微分：

$$
\begin{bmatrix} \dot{q}_0 \\ \dot{q}_1 \\ \dot{q}_2 \\ \dot{q}_3 \end{bmatrix} =
\frac{1}{2} \begin{bmatrix}
0 & -p & -q & -r \\
p & 0 & r & -q \\
q & -r & 0 & p \\
r & q & -p & 0
\end{bmatrix}
\begin{bmatrix} q_0 \\ q_1 \\ q_2 \\ q_3 \end{bmatrix}
$$

この微分方程式は **特異点を持たない**（ジンバルロックが発生しない）。

### 正規化

数値積分の誤差蓄積により単位制約が崩れるため、各ステップで正規化：

$$
\mathbf{q} \leftarrow \frac{\mathbf{q}}{|\mathbf{q}|}
$$

### オイラー角とクォータニオンの比較

| 項目 | オイラー角 | クォータニオン |
|------|-----------|---------------|
| パラメータ数 | 3 | 4 |
| 特異点 | あり（ジンバルロック） | なし |
| 直感性 | 高い（角度で理解しやすい） | 低い |
| 計算コスト | 三角関数多用 | 乗算主体 |
| 補間 | 困難 | SLERP可能 |
| 制約 | なし | $\|\mathbf{q}\| = 1$ |
| 用途 | 表示・入出力 | 内部演算・積分 |

**StampFlyでの使い分け：**
- 内部計算（シミュレータ・AHRS）：クォータニオン
- ユーザー表示・ログ出力：オイラー角に変換

## 6. 数値積分

### Runge-Kutta 4次法（RK4）

シミュレータでは4次のRunge-Kutta法を使用：

$$
\begin{align}
\mathbf{k}_1 &= f(t, \mathbf{y}) \\
\mathbf{k}_2 &= f(t + h/2, \mathbf{y} + h\mathbf{k}_1/2) \\
\mathbf{k}_3 &= f(t + h/2, \mathbf{y} + h\mathbf{k}_2/2) \\
\mathbf{k}_4 &= f(t + h, \mathbf{y} + h\mathbf{k}_3)
\end{align}
$$

$$
\mathbf{y}(t+h) = \mathbf{y}(t) + \frac{h}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)
$$

| パラメータ | 値 | 単位 |
|-----------|-----|------|
| 積分刻み幅 $h$ | 0.001 | s (1 kHz) |
| 精度 | $O(h^4)$ | - |

### 積分対象

RK4で積分する状態変数：

| 変数 | 微分方程式 |
|------|-----------|
| $u, v, w$ | 並進運動方程式 |
| $p, q, r$ | 回転運動方程式 |
| $q_0, q_1, q_2, q_3$ | クォータニオン運動学 |
| $x, y, z$ | 位置運動学 |

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

This document adopts the **NED (North-East-Down) coordinate system** (right-handed):

```
Inertial Frame (I)                    Body Frame (B)

       X_I (North)                          X_B (Front)
        ↑                                    ↑
        │                                    │
        │                                    │
        └──────→ Y_I (East)                  └──────→ Y_B (Right)
         ╲                                    ╲
          ╲                                    ╲
           ↘                                    ↘
            Z_I (Down)                           Z_B (Down)
```

| Frame | Origin | Axes | Usage |
|-------|--------|------|-------|
| Inertial (I) | Fixed point on ground | X:North, Y:East, Z:Down | Position, velocity |
| Body (B) | Vehicle CG | X:Front, Y:Right, Z:Down | Forces, moments |

**NED Coordinate System Characteristics:**
- Right-handed ($\mathbf{X} \times \mathbf{Y} = \mathbf{Z}$)
- Altitude increase results in negative $z$
- Standard in aviation and drone applications

### State Variables

| Symbol | Description | Frame | Unit |
|--------|-------------|-------|------|
| **Position** | | | |
| $x, y, z$ | Position | Inertial | m |
| **Velocity** | | | |
| $\dot{x}, \dot{y}, \dot{z}$ | Inertial velocity | Inertial | m/s |
| $u, v, w$ | Body velocity | Body | m/s |
| **Attitude** | | | |
| $\phi, \theta, \psi$ | Euler angles (Roll, Pitch, Yaw) | - | rad |
| $q_0, q_1, q_2, q_3$ | Quaternion | - | - |
| **Angular velocity** | | | |
| $p, q, r$ | Body angular velocity | Body | rad/s |

## 2. 6-DOF Equations of Motion

### Translational Equations

From Newton's second law in the body frame:

$$
m\left(\frac{d\mathbf{V}_B}{dt} + \boldsymbol{\omega} \times \mathbf{V}_B\right) = \mathbf{F}_B
$$

Where:
- $m$ : Vehicle mass [kg]
- $\mathbf{V}_B = [u, v, w]^\top$ : Body velocity vector [m/s]
- $\boldsymbol{\omega} = [p, q, r]^\top$ : Body angular velocity vector [rad/s]
- $\mathbf{F}_B$ : Forces in body frame [N]

Expanded:

$$
\begin{align}
\dot{u} &= \frac{F_x}{m} - qw + rv \\
\dot{v} &= \frac{F_y}{m} - ru + pw \\
\dot{w} &= \frac{F_z}{m} - pv + qu
\end{align}
$$

### Rotational Equations

From Euler's equation of motion in the body frame:

$$
\mathbf{I}\frac{d\boldsymbol{\omega}}{dt} + \boldsymbol{\omega} \times (\mathbf{I}\boldsymbol{\omega}) = \boldsymbol{\tau}
$$

For diagonal inertia tensor (symmetric vehicle):

$$
\begin{align}
\dot{p} &= \frac{L - (I_{zz} - I_{yy})qr}{I_{xx}} \\
\dot{q} &= \frac{M - (I_{xx} - I_{zz})pr}{I_{yy}} \\
\dot{r} &= \frac{N - (I_{yy} - I_{xx})pq}{I_{zz}}
\end{align}
$$

### Forces and Moments

#### Thrust

Total thrust from 4 motors (negative Z in body frame = upward):

$$
\mathbf{F}_{thrust} = \begin{bmatrix} 0 \\ 0 \\ -(T_1 + T_2 + T_3 + T_4) \end{bmatrix}
$$

$$
T_i = C_t \omega_i^2
$$

**Note (NED):** Z-axis is positive downward, so upward thrust is negative.

#### Gravity

Gravity in inertial frame transformed to body frame:

$$
\mathbf{F}_{gravity}^I = \begin{bmatrix} 0 \\ 0 \\ mg \end{bmatrix} \quad \text{(NED: positive downward)}
$$

$$
\mathbf{F}_{gravity}^B = \mathbf{R}^\top \mathbf{F}_{gravity}^I
$$

**Note (NED):** Gravity acts in +Z direction (downward).

#### Moments

$$
\begin{align}
L &= d(T_3 + T_4 - T_1 - T_2) \quad \text{(Roll)} \\
M &= d(T_1 + T_4 - T_2 - T_3) \quad \text{(Pitch)} \\
N &= C_q(\omega_2^2 + \omega_4^2 - \omega_1^2 - \omega_3^2) \quad \text{(Yaw)}
\end{align}
$$

## 3. Kinematics

### Position Update

Position in inertial frame from body velocity:

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{z} \end{bmatrix} =
\mathbf{R} \begin{bmatrix} u \\ v \\ w \end{bmatrix}
$$

Where $\mathbf{R}$ is the DCM (Direction Cosine Matrix) from body to inertial frame.

## 4. Attitude Representation: Euler Angles

### Definition

Euler angles represent attitude as three sequential rotations.
This system uses **Z-Y-X (Yaw-Pitch-Roll)** order:

1. $\psi$ (Yaw): Rotation about Z-axis
2. $\theta$ (Pitch): Rotation about Y-axis
3. $\phi$ (Roll): Rotation about X-axis

### Rotation Matrix (DCM)

Combined rotation matrix (body → inertial):

$$
\mathbf{R} = \begin{bmatrix}
c_\theta c_\psi & c_\theta s_\psi & -s_\theta \\
s_\phi s_\theta c_\psi - c_\phi s_\psi & s_\phi s_\theta s_\psi + c_\phi c_\psi & s_\phi c_\theta \\
c_\phi s_\theta c_\psi + s_\phi s_\psi & c_\phi s_\theta s_\psi - s_\phi c_\psi & c_\phi c_\theta
\end{bmatrix}
$$

### Euler Angle Derivatives

$$
\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} =
\begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta
\end{bmatrix}
\begin{bmatrix} p \\ q \\ r \end{bmatrix}
$$

**Gimbal Lock:** At $\theta = \pm 90°$, $\cos\theta = 0$ causes singularity.

## 5. Attitude Representation: Quaternion

### Definition

Quaternion represents attitude with 4 components:

$$
\mathbf{q} = [q_0, q_1, q_2, q_3]^\top
$$

With unit constraint: $q_0^2 + q_1^2 + q_2^2 + q_3^2 = 1$

### Quaternion to DCM

$$
\mathbf{R} = \begin{bmatrix}
q_0^2+q_1^2-q_2^2-q_3^2 & 2(q_1 q_2 - q_0 q_3) & 2(q_1 q_3 + q_0 q_2) \\
2(q_1 q_2 + q_0 q_3) & q_0^2-q_1^2+q_2^2-q_3^2 & 2(q_2 q_3 - q_0 q_1) \\
2(q_1 q_3 - q_0 q_2) & 2(q_2 q_3 + q_0 q_1) & q_0^2-q_1^2-q_2^2+q_3^2
\end{bmatrix}
$$

### Quaternion Derivatives

$$
\begin{bmatrix} \dot{q}_0 \\ \dot{q}_1 \\ \dot{q}_2 \\ \dot{q}_3 \end{bmatrix} =
\frac{1}{2} \begin{bmatrix}
0 & -p & -q & -r \\
p & 0 & r & -q \\
q & -r & 0 & p \\
r & q & -p & 0
\end{bmatrix}
\begin{bmatrix} q_0 \\ q_1 \\ q_2 \\ q_3 \end{bmatrix}
$$

This equation has **no singularity** (no gimbal lock).

### Comparison

| Aspect | Euler Angles | Quaternion |
|--------|--------------|------------|
| Parameters | 3 | 4 |
| Singularity | Yes (gimbal lock) | None |
| Intuition | High | Low |
| Computation | Trigonometric | Multiplication |
| Interpolation | Difficult | SLERP possible |
| Constraint | None | $\|\mathbf{q}\| = 1$ |
| Usage | Display/I/O | Internal/integration |

## 6. Numerical Integration

### Runge-Kutta 4th Order (RK4)

The simulator uses 4th order Runge-Kutta:

$$
\begin{align}
\mathbf{k}_1 &= f(t, \mathbf{y}) \\
\mathbf{k}_2 &= f(t + h/2, \mathbf{y} + h\mathbf{k}_1/2) \\
\mathbf{k}_3 &= f(t + h/2, \mathbf{y} + h\mathbf{k}_2/2) \\
\mathbf{k}_4 &= f(t + h, \mathbf{y} + h\mathbf{k}_3)
\end{align}
$$

$$
\mathbf{y}(t+h) = \mathbf{y}(t) + \frac{h}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)
$$

| Parameter | Value | Unit |
|-----------|-------|------|
| Step size $h$ | 0.001 | s (1 kHz) |
| Accuracy | $O(h^4)$ | - |

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
