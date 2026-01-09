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

## 6. DCモータモデル

### 概要

マルチコプターのアクチュエータであるDCモータ（ブラシレスDCモータ含む）の動特性をモデル化する。
最初に電気系・機械系の完全な微分方程式を導出し、最終的にインダクタンスを無視した近似モデルを示す。

### 等価回路

DCモータの電気系は以下の等価回路で表される：

```
    R          L
   ─┴─       ─┴─
──►│├───────►│├───────►┬────────
   ─┬─       ─┬─       │
    V_a                 │  ┌─────┐
    (印加電圧)           └──┤  e  │  逆起電力
                           │ =Kω │
                           └─────┘
```

### 記号の定義

| 記号 | 説明 | 単位 |
|------|------|------|
| $V_a$ | 印加電圧（Armature voltage） | V |
| $i$ | 電機子電流（Armature current） | A |
| $R$ | 電機子抵抗（Armature resistance） | Ω |
| $L$ | 電機子インダクタンス（Armature inductance） | H |
| $e$ | 逆起電力（Back-EMF） | V |
| $K_e$ | 逆起電力定数（Back-EMF constant） | V/(rad/s) |
| $K_t$ | トルク定数（Torque constant） | N·m/A |
| $\omega$ | 回転角速度（Angular velocity） | rad/s |
| $\tau_m$ | モータトルク（Motor torque） | N·m |
| $J_m$ | ロータ慣性モーメント（Rotor inertia） | kg·m² |
| $b$ | 粘性摩擦係数（Viscous friction） | N·m·s/rad |
| $\tau_L$ | 負荷トルク（Load torque） | N·m |

**注意：** 理想的なDCモータでは $K_e = K_t$ （SI単位系において）。

### 完全な微分方程式

#### 電気系の方程式

キルヒホッフの電圧則より、電機子回路の電圧方程式：

$$
V_a = Ri + L\frac{di}{dt} + e
$$

逆起電力は回転角速度に比例：

$$
e = K_e \omega
$$

したがって：

$$
L\frac{di}{dt} = V_a - Ri - K_e \omega
$$

整理して電流の微分方程式：

$$
\frac{di}{dt} = \frac{1}{L}(V_a - Ri - K_e \omega)
$$

#### 機械系の方程式

ニュートンの回転運動方程式より：

$$
J_m \frac{d\omega}{dt} = \tau_m - b\omega - \tau_L
$$

モータトルクは電流に比例：

$$
\tau_m = K_t i
$$

したがって：

$$
\frac{d\omega}{dt} = \frac{1}{J_m}(K_t i - b\omega - \tau_L)
$$

#### 状態方程式形式

状態変数 $\mathbf{x} = [i, \omega]^\top$、入力 $u = V_a$、外乱 $d = \tau_L$ として：

$$
\frac{d}{dt}\begin{bmatrix} i \\ \omega \end{bmatrix} =
\begin{bmatrix}
-\frac{R}{L} & -\frac{K_e}{L} \\
\frac{K_t}{J_m} & -\frac{b}{J_m}
\end{bmatrix}
\begin{bmatrix} i \\ \omega \end{bmatrix} +
\begin{bmatrix} \frac{1}{L} \\ 0 \end{bmatrix} V_a +
\begin{bmatrix} 0 \\ -\frac{1}{J_m} \end{bmatrix} \tau_L
$$

### 時定数の分析

DCモータには2つの時定数が存在する：

| 時定数 | 式 | 意味 |
|--------|-----|------|
| 電気時定数 | $\tau_e = \frac{L}{R}$ | 電流の応答速度 |
| 機械時定数 | $\tau_m = \frac{J_m R}{K_t K_e}$ | 回転速度の応答速度 |

一般的なDCモータでは：

$$
\tau_e \ll \tau_m
$$

（電気時定数は数ms、機械時定数は数十〜数百ms）

### インダクタンスを無視した近似モデル

#### 近似の妥当性

小型DCモータでは $\tau_e \ll \tau_m$ が成り立つため、電気系の過渡応答は機械系に比べて十分速い。
このとき、電流は瞬時に定常値に達すると仮定できる（準静的近似）：

$$
L\frac{di}{dt} \approx 0
$$

#### 簡略化された電気系

インダクタンスを無視すると：

$$
V_a = Ri + K_e \omega
$$

電流について解くと：

$$
i = \frac{V_a - K_e \omega}{R}
$$

#### 簡略化された機械系

電流の式を機械系に代入：

$$
J_m \frac{d\omega}{dt} = K_t \cdot \frac{V_a - K_e \omega}{R} - b\omega - \tau_L
$$

整理すると：

$$
J_m \frac{d\omega}{dt} = \frac{K_t}{R} V_a - \left(\frac{K_t K_e}{R} + b\right)\omega - \tau_L
$$

#### 1次遅れ系への帰着

負荷トルク $\tau_L = 0$ のとき、等価時定数と等価ゲインを定義：

$$
\tau_{eq} = \frac{J_m R}{K_t K_e + bR}
$$

$$
K_{eq} = \frac{K_t}{K_t K_e + bR}
$$

すると角速度の微分方程式は1次遅れ系となる：

$$
\tau_{eq} \frac{d\omega}{dt} + \omega = K_{eq} V_a
$$

#### 伝達関数

ラプラス変換すると：

$$
\frac{\Omega(s)}{V_a(s)} = \frac{K_{eq}}{\tau_{eq} s + 1}
$$

粘性摩擦が十分小さい場合（$bR \ll K_t K_e$）：

$$
\tau_{eq} \approx \frac{J_m R}{K_t K_e}, \quad K_{eq} \approx \frac{1}{K_e}
$$

### プロペラ負荷の考慮

マルチコプターでは、モータはプロペラを駆動する。プロペラの空力負荷トルクは角速度の2乗に比例：

$$
\tau_L = C_q \omega^2
$$

このとき微分方程式は：

$$
J_m \frac{d\omega}{dt} = \frac{K_t}{R}(V_a - K_e \omega) - b\omega - C_q \omega^2
$$

#### 定常状態

定常状態（$\frac{d\omega}{dt} = 0$）では：

$$
\frac{K_t}{R}(V_a - K_e \omega_{ss}) = b\omega_{ss} + C_q \omega_{ss}^2
$$

これは $\omega_{ss}$ についての2次方程式であり、与えられた電圧 $V_a$ に対する定常回転数を求められる。

### StampFlyでのモータパラメータ

| パラメータ | 記号 | 値 | 単位 |
|-----------|------|-----|------|
| モータ定数 | $K_t = K_e$ | 0.0042 | N·m/A, V/(rad/s) |
| 電機子抵抗 | $R$ | 0.5 | Ω |
| ロータ慣性 | $J_m$ | $1.0 \times 10^{-7}$ | kg·m² |
| 電気時定数 | $\tau_e$ | ~0.1 | ms (無視可能) |

**注意：** 実際のブラシレスDCモータはESC（電子速度コントローラ）で制御されるため、
より複雑な動特性を持つが、制御設計では上記の近似モデルで十分な場合が多い。

### まとめ

| モデル | 特徴 | 用途 |
|--------|------|------|
| 完全モデル（2次系） | 電気・機械両方の過渡応答を表現 | 高精度シミュレーション |
| 近似モデル（1次系） | インダクタンス無視、計算が簡単 | 制御設計、リアルタイムシミュレーション |

制御系設計では近似モデルで十分であり、StampFlyのシミュレータでもこの近似を採用している。

## 7. 線形化と伝達関数

### 概要

制御系設計では、非線形な運動方程式を平衡点周りで線形化し、伝達関数を用いて解析・設計を行う。
本セクションでは、ホバリング状態を平衡点として線形化を行い、角速度制御ループの伝達関数を導出する。

### ホバリング平衡点

ホバリング状態における平衡点を定義する：

| 状態変数 | 平衡値 | 説明 |
|---------|-------|------|
| $\phi_0, \theta_0, \psi_0$ | $0$ | 水平姿勢 |
| $p_0, q_0, r_0$ | $0$ | 角速度ゼロ |
| $u_0, v_0, w_0$ | $0$ | 機体速度ゼロ |
| $T_0$ | $mg/4$ | 各モータの推力（重力釣り合い） |
| $\omega_{m0}$ | $\sqrt{mg/(4C_t)}$ | 各モータの回転速度 |

### 回転運動方程式の線形化

#### 非線形方程式

セクション2で導出した回転運動方程式：

$$
\begin{align}
\dot{p} &= \frac{L - (I_{zz} - I_{yy})qr}{I_{xx}} \\
\dot{q} &= \frac{M - (I_{xx} - I_{zz})pr}{I_{yy}} \\
\dot{r} &= \frac{N - (I_{yy} - I_{xx})pq}{I_{zz}}
\end{align}
$$

#### 小摂動の仮定

平衡点周りの小摂動を仮定：

$$
p = p_0 + \delta p, \quad q = q_0 + \delta q, \quad r = r_0 + \delta r
$$

平衡点 $(p_0, q_0, r_0) = (0, 0, 0)$ において、2次の微小項 $qr$, $pr$, $pq$ を無視すると：

$$
\begin{align}
\delta\dot{p} &= \frac{\delta L}{I_{xx}} \\
\delta\dot{q} &= \frac{\delta M}{I_{yy}} \\
\delta\dot{r} &= \frac{\delta N}{I_{zz}}
\end{align}
$$

#### 線形化されたモーメント

モーメントの摂動は推力の摂動で表される：

$$
\begin{align}
\delta L &= d(\delta T_3 + \delta T_4 - \delta T_1 - \delta T_2) \\
\delta M &= d(\delta T_1 + \delta T_4 - \delta T_2 - \delta T_3) \\
\delta N &= C_q(2\omega_{m0})(\delta\omega_2 + \delta\omega_4 - \delta\omega_1 - \delta\omega_3)
\end{align}
$$

推力の線形化（$T_i = C_t \omega_i^2$ より）：

$$
\delta T_i = 2 C_t \omega_{m0} \cdot \delta\omega_i = k_T \cdot \delta\omega_i
$$

ここで $k_T = 2 C_t \omega_{m0}$ は推力ゲイン。

### モータダイナミクスとの結合

#### モータの伝達関数

セクション6で導出した1次遅れモデル（インダクタンス無視）：

$$
\frac{\Omega_i(s)}{V_{a,i}(s)} = \frac{K_{eq}}{\tau_{eq} s + 1}
$$

PWMデューティ $\delta_i$ から回転速度への関係（電源電圧 $V_{bat}$ を考慮）：

$$
\frac{\Omega_i(s)}{\Delta_i(s)} = \frac{K_m}{\tau_m s + 1}
$$

ここで：
- $K_m = K_{eq} \cdot V_{bat}$ : モータゲイン [rad/s]
- $\tau_m = \tau_{eq}$ : モータ時定数 [s]

#### 制御入力の定義

4つのモータへの入力を、仮想的な制御入力に変換：

$$
\begin{bmatrix} \delta_T \\ \delta_\phi \\ \delta_\theta \\ \delta_\psi \end{bmatrix} =
\begin{bmatrix} 1 & 1 & 1 & 1 \\ -1 & -1 & 1 & 1 \\ 1 & -1 & -1 & 1 \\ 1 & -1 & 1 & -1 \end{bmatrix}
\begin{bmatrix} \delta_1 \\ \delta_2 \\ \delta_3 \\ \delta_4 \end{bmatrix}
$$

| 入力 | 説明 |
|------|------|
| $\delta_T$ | 推力（スロットル） |
| $\delta_\phi$ | ロールモーメント |
| $\delta_\theta$ | ピッチモーメント |
| $\delta_\psi$ | ヨーモーメント |

### 角速度制御ループの伝達関数

#### ロール軸の伝達関数

ロール軸について、制御入力 $\delta_\phi$ から角速度 $p$ への伝達関数を導出する。

**モータダイナミクス：**

$$
\frac{\Delta\Omega_\phi(s)}{\Delta_\phi(s)} = \frac{K_m}{\tau_m s + 1}
$$

**推力からモーメントへ：**

$$
\delta L = 2 d \cdot k_T \cdot \Delta\omega_\phi
$$

**角速度ダイナミクス（ラプラス変換）：**

$$
s \cdot P(s) = \frac{\delta L(s)}{I_{xx}}
$$

**全体の伝達関数：**

$$
\boxed{
G_p(s) = \frac{P(s)}{\Delta_\phi(s)} = \frac{K_p}{s(\tau_m s + 1)}
}
$$

ここで：

$$
K_p = \frac{2 d \cdot k_T \cdot K_m}{I_{xx}} = \frac{4 d \cdot C_t \cdot \omega_{m0} \cdot K_m}{I_{xx}}
$$

#### ピッチ軸の伝達関数

対称性より、ピッチ軸も同様の形式：

$$
\boxed{
G_q(s) = \frac{Q(s)}{\Delta_\theta(s)} = \frac{K_q}{s(\tau_m s + 1)}
}
$$

$$
K_q = \frac{4 d \cdot C_t \cdot \omega_{m0} \cdot K_m}{I_{yy}}
$$

$I_{xx} = I_{yy}$ の場合、$K_p = K_q$。

#### ヨー軸の伝達関数

ヨー軸は反トルクによる制御：

$$
\boxed{
G_r(s) = \frac{R(s)}{\Delta_\psi(s)} = \frac{K_r}{s(\tau_m s + 1)}
}
$$

$$
K_r = \frac{4 C_q \cdot \omega_{m0} \cdot K_m}{I_{zz}}
$$

#### 伝達関数の構造

各軸の角速度伝達関数は **積分器とモータ1次遅れの直列結合** となる：

```
                    ┌─────────────┐     ┌─────────┐
  δ_cmd  ───────────►│  Motor      │────►│ 1/I·s   │────► ω
                    │  Km/(τm·s+1)│     │(積分器) │
                    └─────────────┘     └─────────┘
```

この構造は、角加速度がモーメントに比例し、角速度はその積分であることを反映している。

### 姿勢角への拡張

#### 角速度から姿勢角への関係

小角度の仮定のもと、オイラー角と角速度の関係は単純化される：

$$
\dot{\phi} \approx p, \quad \dot{\theta} \approx q, \quad \dot{\psi} \approx r
$$

したがって、角速度から姿勢角への伝達関数は積分器：

$$
\frac{\Phi(s)}{P(s)} = \frac{1}{s}, \quad \frac{\Theta(s)}{Q(s)} = \frac{1}{s}, \quad \frac{\Psi(s)}{R(s)} = \frac{1}{s}
$$

#### 姿勢角制御ループの伝達関数

制御入力から姿勢角までの全体伝達関数（ロール軸の例）：

$$
\boxed{
G_\phi(s) = \frac{\Phi(s)}{\Delta_\phi(s)} = \frac{K_p}{s^2(\tau_m s + 1)}
}
$$

これは **2次積分器とモータ1次遅れの直列結合** であり、開ループでは不安定である。
安定化にはフィードバック制御（PIDなど）が必要となる。

### 制御設計への応用

#### ブロック線図

角速度制御ループ（レートモード）のブロック線図：

```
              ┌─────────────────────────────────────────┐
              │            角速度制御ループ              │
              │                                         │
 ω_cmd  ──►(+)───►│Controller│───►│Motor│───►│1/I·s│───┬──► ω
           -↑     │  C(s)    │    │G_m(s)│   │     │   │
            │     └──────────┘    └──────┘   └─────┘   │
            │                                          │
            └──────────────────────────────────────────┘
                              （角速度フィードバック）
```

#### PID制御器設計の指針

| パラメータ | 設計指針 |
|-----------|---------|
| $K_p$ (比例ゲイン) | 応答速度を決定。モータ時定数 $\tau_m$ を考慮 |
| $K_i$ (積分ゲイン) | 定常偏差を除去。ただし積分器が既に存在するため注意 |
| $K_d$ (微分ゲイン) | ダンピングを追加。モータ遅れを補償 |

**注意：** プラントに積分器が含まれるため、I制御の追加は慎重に行う必要がある。
典型的には PD または P 制御で設計し、外乱補償が必要な場合のみ I を追加する。

### パラメータ例（StampFly）

| パラメータ | 記号 | 値 | 単位 |
|-----------|------|-----|------|
| アーム長 | $d$ | 0.033 | m |
| 推力係数 | $C_t$ | $1.0 \times 10^{-8}$ | N/(rad/s)² |
| トルク係数 | $C_q$ | $1.0 \times 10^{-10}$ | N·m/(rad/s)² |
| 慣性モーメント | $I_{xx}, I_{yy}$ | $2.0 \times 10^{-5}$ | kg·m² |
| 慣性モーメント | $I_{zz}$ | $3.5 \times 10^{-5}$ | kg·m² |
| モータ時定数 | $\tau_m$ | 0.02 | s |
| ホバリング回転数 | $\omega_{m0}$ | 4000 | rad/s |

### 数値計算例

上記パラメータを用いて、具体的な伝達関数を導出する。

#### モータゲインの仮定

PWMデューティ 0→1 で回転数 0→8000 rad/s と仮定：

$$
K_m = 8000 \text{ rad/s}
$$

#### 推力ゲイン

$$
k_T = 2 C_t \omega_{m0} = 2 \times 1.0 \times 10^{-8} \times 4000 = 8.0 \times 10^{-5} \text{ N/(rad/s)}
$$

#### 各軸のゲイン計算

**ロール/ピッチ軸：**

$$
K_p = K_q = \frac{2 d \cdot k_T \cdot K_m}{I_{xx}} = \frac{2 \times 0.033 \times 8.0 \times 10^{-5} \times 8000}{2.0 \times 10^{-5}} = 2112 \text{ [1/s]}
$$

**ヨー軸：**

$$
K_r = \frac{4 C_q \cdot \omega_{m0} \cdot K_m}{I_{zz}} = \frac{4 \times 1.0 \times 10^{-10} \times 4000 \times 8000}{3.5 \times 10^{-5}} = 365.7 \text{ [1/s]}
$$

#### 数値入り伝達関数

**ロール軸（$\delta_\phi \to p$）：**

$$
\boxed{G_p(s) = \frac{2112}{s(0.02s + 1)} = \frac{105600}{s(s + 50)}}
$$

**ピッチ軸（$\delta_\theta \to q$）：**

$$
\boxed{G_q(s) = \frac{2112}{s(0.02s + 1)} = \frac{105600}{s(s + 50)}}
$$

**ヨー軸（$\delta_\psi \to r$）：**

$$
\boxed{G_r(s) = \frac{365.7}{s(0.02s + 1)} = \frac{18285}{s(s + 50)}}
$$

#### 特性まとめ

| 軸 | ゲイン | 極 | 特徴 |
|----|-------|-----|------|
| Roll | 2112 | $s=0, -50$ | 積分器 + 時定数20ms |
| Pitch | 2112 | $s=0, -50$ | Rollと同一 |
| Yaw | 365.7 | $s=0, -50$ | ゲインがRoll/Pitchの約1/6 |

**考察：** ヨー軸はトルク係数 $C_q$ がロール/ピッチの推力によるモーメントより小さく、
かつ $I_{zz} > I_{xx}$ のため、ゲインが小さくなる。
これはヨー軸の応答が他軸より遅いことを意味し、制御器設計時に考慮が必要である。

## 8. 数値積分

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

## 9. 実装リファレンス

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

## 6. DC Motor Model

### Overview

This section models the dynamics of DC motors (including brushless DC motors) used as actuators in multicopters.
We first derive the complete differential equations for electrical and mechanical systems, then present the simplified model that ignores inductance.

### Equivalent Circuit

The electrical system of a DC motor is represented by the following equivalent circuit:

```
    R          L
   ─┴─       ─┴─
──►│├───────►│├───────►┬────────
   ─┬─       ─┬─       │
    V_a                 │  ┌─────┐
    (Applied voltage)   └──┤  e  │  Back-EMF
                           │ =Kω │
                           └─────┘
```

### Symbol Definitions

| Symbol | Description | Unit |
|--------|-------------|------|
| $V_a$ | Armature voltage | V |
| $i$ | Armature current | A |
| $R$ | Armature resistance | Ω |
| $L$ | Armature inductance | H |
| $e$ | Back-EMF | V |
| $K_e$ | Back-EMF constant | V/(rad/s) |
| $K_t$ | Torque constant | N·m/A |
| $\omega$ | Angular velocity | rad/s |
| $\tau_m$ | Motor torque | N·m |
| $J_m$ | Rotor inertia | kg·m² |
| $b$ | Viscous friction coefficient | N·m·s/rad |
| $\tau_L$ | Load torque | N·m |

**Note:** For an ideal DC motor, $K_e = K_t$ (in SI units).

### Complete Differential Equations

#### Electrical Equation

From Kirchhoff's voltage law, the armature circuit equation:

$$
V_a = Ri + L\frac{di}{dt} + e
$$

Back-EMF is proportional to angular velocity:

$$
e = K_e \omega
$$

Therefore:

$$
\frac{di}{dt} = \frac{1}{L}(V_a - Ri - K_e \omega)
$$

#### Mechanical Equation

From Newton's rotational equation:

$$
J_m \frac{d\omega}{dt} = \tau_m - b\omega - \tau_L
$$

Motor torque is proportional to current:

$$
\tau_m = K_t i
$$

Therefore:

$$
\frac{d\omega}{dt} = \frac{1}{J_m}(K_t i - b\omega - \tau_L)
$$

#### State-Space Form

With state $\mathbf{x} = [i, \omega]^\top$, input $u = V_a$, and disturbance $d = \tau_L$:

$$
\frac{d}{dt}\begin{bmatrix} i \\ \omega \end{bmatrix} =
\begin{bmatrix}
-\frac{R}{L} & -\frac{K_e}{L} \\
\frac{K_t}{J_m} & -\frac{b}{J_m}
\end{bmatrix}
\begin{bmatrix} i \\ \omega \end{bmatrix} +
\begin{bmatrix} \frac{1}{L} \\ 0 \end{bmatrix} V_a +
\begin{bmatrix} 0 \\ -\frac{1}{J_m} \end{bmatrix} \tau_L
$$

### Time Constant Analysis

DC motors have two time constants:

| Time Constant | Formula | Meaning |
|---------------|---------|---------|
| Electrical | $\tau_e = \frac{L}{R}$ | Current response speed |
| Mechanical | $\tau_m = \frac{J_m R}{K_t K_e}$ | Velocity response speed |

For typical DC motors:

$$
\tau_e \ll \tau_m
$$

(Electrical: ~ms, Mechanical: ~tens to hundreds of ms)

### Simplified Model (Ignoring Inductance)

#### Validity of Approximation

For small DC motors where $\tau_e \ll \tau_m$, the electrical transient is much faster than the mechanical system.
The current can be assumed to reach steady-state instantaneously (quasi-static approximation):

$$
L\frac{di}{dt} \approx 0
$$

#### Simplified Electrical System

Ignoring inductance:

$$
V_a = Ri + K_e \omega
$$

Solving for current:

$$
i = \frac{V_a - K_e \omega}{R}
$$

#### Simplified Mechanical System

Substituting into the mechanical equation:

$$
J_m \frac{d\omega}{dt} = \frac{K_t}{R} V_a - \left(\frac{K_t K_e}{R} + b\right)\omega - \tau_L
$$

#### First-Order System

With $\tau_L = 0$, defining equivalent time constant and gain:

$$
\tau_{eq} = \frac{J_m R}{K_t K_e + bR}, \quad K_{eq} = \frac{K_t}{K_t K_e + bR}
$$

The angular velocity follows a first-order system:

$$
\tau_{eq} \frac{d\omega}{dt} + \omega = K_{eq} V_a
$$

#### Transfer Function

In Laplace domain:

$$
\frac{\Omega(s)}{V_a(s)} = \frac{K_{eq}}{\tau_{eq} s + 1}
$$

When viscous friction is small ($bR \ll K_t K_e$):

$$
\tau_{eq} \approx \frac{J_m R}{K_t K_e}, \quad K_{eq} \approx \frac{1}{K_e}
$$

### Propeller Load

For multicopters, the aerodynamic load torque from propellers is proportional to the square of angular velocity:

$$
\tau_L = C_q \omega^2
$$

The differential equation becomes:

$$
J_m \frac{d\omega}{dt} = \frac{K_t}{R}(V_a - K_e \omega) - b\omega - C_q \omega^2
$$

### Summary

| Model | Characteristics | Application |
|-------|----------------|-------------|
| Complete (2nd order) | Captures both electrical and mechanical transients | High-fidelity simulation |
| Simplified (1st order) | Ignores inductance, simpler computation | Control design, real-time simulation |

The simplified model is sufficient for control design and is used in the StampFly simulator.

## 7. Linearization and Transfer Functions

### Overview

In control system design, nonlinear equations of motion are linearized around an equilibrium point,
and transfer functions are used for analysis and design.
This section linearizes around the hovering state and derives the transfer functions for the angular velocity control loop.

### Hovering Equilibrium Point

Define the equilibrium point at hovering state:

| State Variable | Equilibrium Value | Description |
|---------------|-------------------|-------------|
| $\phi_0, \theta_0, \psi_0$ | $0$ | Level attitude |
| $p_0, q_0, r_0$ | $0$ | Zero angular velocity |
| $u_0, v_0, w_0$ | $0$ | Zero body velocity |
| $T_0$ | $mg/4$ | Thrust per motor (gravity balance) |
| $\omega_{m0}$ | $\sqrt{mg/(4C_t)}$ | Motor angular velocity |

### Linearization of Rotational Equations

#### Nonlinear Equations

From Section 2, the rotational equations of motion:

$$
\begin{align}
\dot{p} &= \frac{L - (I_{zz} - I_{yy})qr}{I_{xx}} \\
\dot{q} &= \frac{M - (I_{xx} - I_{zz})pr}{I_{yy}} \\
\dot{r} &= \frac{N - (I_{yy} - I_{xx})pq}{I_{zz}}
\end{align}
$$

#### Small Perturbation Assumption

Assume small perturbations around the equilibrium:

$$
p = p_0 + \delta p, \quad q = q_0 + \delta q, \quad r = r_0 + \delta r
$$

At equilibrium $(p_0, q_0, r_0) = (0, 0, 0)$, ignoring second-order terms $qr$, $pr$, $pq$:

$$
\begin{align}
\delta\dot{p} &= \frac{\delta L}{I_{xx}} \\
\delta\dot{q} &= \frac{\delta M}{I_{yy}} \\
\delta\dot{r} &= \frac{\delta N}{I_{zz}}
\end{align}
$$

#### Linearized Moments

Moment perturbations expressed in terms of thrust perturbations:

$$
\begin{align}
\delta L &= d(\delta T_3 + \delta T_4 - \delta T_1 - \delta T_2) \\
\delta M &= d(\delta T_1 + \delta T_4 - \delta T_2 - \delta T_3) \\
\delta N &= C_q(2\omega_{m0})(\delta\omega_2 + \delta\omega_4 - \delta\omega_1 - \delta\omega_3)
\end{align}
$$

Thrust linearization (from $T_i = C_t \omega_i^2$):

$$
\delta T_i = 2 C_t \omega_{m0} \cdot \delta\omega_i = k_T \cdot \delta\omega_i
$$

Where $k_T = 2 C_t \omega_{m0}$ is the thrust gain.

### Coupling with Motor Dynamics

#### Motor Transfer Function

From the first-order model in Section 6 (ignoring inductance):

$$
\frac{\Omega_i(s)}{V_{a,i}(s)} = \frac{K_{eq}}{\tau_{eq} s + 1}
$$

Relationship from PWM duty $\delta_i$ to angular velocity (considering battery voltage $V_{bat}$):

$$
\frac{\Omega_i(s)}{\Delta_i(s)} = \frac{K_m}{\tau_m s + 1}
$$

Where:
- $K_m = K_{eq} \cdot V_{bat}$ : Motor gain [rad/s]
- $\tau_m = \tau_{eq}$ : Motor time constant [s]

#### Virtual Control Input Definition

Transform individual motor inputs to virtual control inputs:

$$
\begin{bmatrix} \delta_T \\ \delta_\phi \\ \delta_\theta \\ \delta_\psi \end{bmatrix} =
\begin{bmatrix} 1 & 1 & 1 & 1 \\ -1 & -1 & 1 & 1 \\ 1 & -1 & -1 & 1 \\ 1 & -1 & 1 & -1 \end{bmatrix}
\begin{bmatrix} \delta_1 \\ \delta_2 \\ \delta_3 \\ \delta_4 \end{bmatrix}
$$

| Input | Description |
|-------|-------------|
| $\delta_T$ | Thrust (throttle) |
| $\delta_\phi$ | Roll moment |
| $\delta_\theta$ | Pitch moment |
| $\delta_\psi$ | Yaw moment |

### Angular Velocity Control Loop Transfer Functions

#### Roll Axis Transfer Function

Derive the transfer function from control input $\delta_\phi$ to angular velocity $p$ for the roll axis.

**Motor dynamics:**

$$
\frac{\Delta\Omega_\phi(s)}{\Delta_\phi(s)} = \frac{K_m}{\tau_m s + 1}
$$

**Thrust to moment:**

$$
\delta L = 2 d \cdot k_T \cdot \Delta\omega_\phi
$$

**Angular velocity dynamics (Laplace transform):**

$$
s \cdot P(s) = \frac{\delta L(s)}{I_{xx}}
$$

**Overall transfer function:**

$$
\boxed{
G_p(s) = \frac{P(s)}{\Delta_\phi(s)} = \frac{K_p}{s(\tau_m s + 1)}
}
$$

Where:

$$
K_p = \frac{2 d \cdot k_T \cdot K_m}{I_{xx}} = \frac{4 d \cdot C_t \cdot \omega_{m0} \cdot K_m}{I_{xx}}
$$

#### Pitch Axis Transfer Function

By symmetry, the pitch axis has the same form:

$$
\boxed{
G_q(s) = \frac{Q(s)}{\Delta_\theta(s)} = \frac{K_q}{s(\tau_m s + 1)}
}
$$

$$
K_q = \frac{4 d \cdot C_t \cdot \omega_{m0} \cdot K_m}{I_{yy}}
$$

When $I_{xx} = I_{yy}$, then $K_p = K_q$.

#### Yaw Axis Transfer Function

The yaw axis is controlled by reaction torque:

$$
\boxed{
G_r(s) = \frac{R(s)}{\Delta_\psi(s)} = \frac{K_r}{s(\tau_m s + 1)}
}
$$

$$
K_r = \frac{4 C_q \cdot \omega_{m0} \cdot K_m}{I_{zz}}
$$

#### Transfer Function Structure

Each axis angular velocity transfer function is a **series connection of integrator and motor first-order lag**:

```
                    ┌─────────────┐     ┌─────────┐
  δ_cmd  ───────────►│  Motor      │────►│ 1/I·s   │────► ω
                    │  Km/(τm·s+1)│     │(integr.)│
                    └─────────────┘     └─────────┘
```

This structure reflects that angular acceleration is proportional to moment, and angular velocity is its integral.

### Extension to Attitude Angles

#### Angular Velocity to Attitude Angle Relationship

Under small angle assumption, Euler angles and angular velocity are simply related:

$$
\dot{\phi} \approx p, \quad \dot{\theta} \approx q, \quad \dot{\psi} \approx r
$$

Therefore, the transfer function from angular velocity to attitude angle is an integrator:

$$
\frac{\Phi(s)}{P(s)} = \frac{1}{s}, \quad \frac{\Theta(s)}{Q(s)} = \frac{1}{s}, \quad \frac{\Psi(s)}{R(s)} = \frac{1}{s}
$$

#### Attitude Angle Control Loop Transfer Function

Overall transfer function from control input to attitude angle (roll axis example):

$$
\boxed{
G_\phi(s) = \frac{\Phi(s)}{\Delta_\phi(s)} = \frac{K_p}{s^2(\tau_m s + 1)}
}
$$

This is a **series connection of double integrator and motor first-order lag**, which is unstable in open loop.
Feedback control (such as PID) is required for stabilization.

### Application to Control Design

#### Block Diagram

Block diagram of angular velocity control loop (rate mode):

```
              ┌─────────────────────────────────────────┐
              │     Angular Velocity Control Loop       │
              │                                         │
 ω_cmd  ──►(+)───►│Controller│───►│Motor│───►│1/I·s│───┬──► ω
           -↑     │  C(s)    │    │G_m(s)│   │     │   │
            │     └──────────┘    └──────┘   └─────┘   │
            │                                          │
            └──────────────────────────────────────────┘
                         (angular velocity feedback)
```

#### PID Controller Design Guidelines

| Parameter | Design Guideline |
|-----------|-----------------|
| $K_p$ (Proportional gain) | Determines response speed. Consider motor time constant $\tau_m$ |
| $K_i$ (Integral gain) | Eliminates steady-state error. Note: integrator already exists in plant |
| $K_d$ (Derivative gain) | Adds damping. Compensates motor delay |

**Note:** Since the plant contains an integrator, adding I control requires caution.
Typically, design with PD or P control, adding I only when disturbance rejection is needed.

### Parameter Example (StampFly)

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Arm length | $d$ | 0.033 | m |
| Thrust coefficient | $C_t$ | $1.0 \times 10^{-8}$ | N/(rad/s)² |
| Torque coefficient | $C_q$ | $1.0 \times 10^{-10}$ | N·m/(rad/s)² |
| Moment of inertia | $I_{xx}, I_{yy}$ | $2.0 \times 10^{-5}$ | kg·m² |
| Moment of inertia | $I_{zz}$ | $3.5 \times 10^{-5}$ | kg·m² |
| Motor time constant | $\tau_m$ | 0.02 | s |
| Hovering angular velocity | $\omega_{m0}$ | 4000 | rad/s |

### Numerical Example

Using the above parameters, derive concrete transfer functions.

#### Motor Gain Assumption

Assuming PWM duty 0→1 corresponds to angular velocity 0→8000 rad/s:

$$
K_m = 8000 \text{ rad/s}
$$

#### Thrust Gain

$$
k_T = 2 C_t \omega_{m0} = 2 \times 1.0 \times 10^{-8} \times 4000 = 8.0 \times 10^{-5} \text{ N/(rad/s)}
$$

#### Gain Calculation for Each Axis

**Roll/Pitch axis:**

$$
K_p = K_q = \frac{2 d \cdot k_T \cdot K_m}{I_{xx}} = \frac{2 \times 0.033 \times 8.0 \times 10^{-5} \times 8000}{2.0 \times 10^{-5}} = 2112 \text{ [1/s]}
$$

**Yaw axis:**

$$
K_r = \frac{4 C_q \cdot \omega_{m0} \cdot K_m}{I_{zz}} = \frac{4 \times 1.0 \times 10^{-10} \times 4000 \times 8000}{3.5 \times 10^{-5}} = 365.7 \text{ [1/s]}
$$

#### Transfer Functions with Numerical Values

**Roll axis ($\delta_\phi \to p$):**

$$
\boxed{G_p(s) = \frac{2112}{s(0.02s + 1)} = \frac{105600}{s(s + 50)}}
$$

**Pitch axis ($\delta_\theta \to q$):**

$$
\boxed{G_q(s) = \frac{2112}{s(0.02s + 1)} = \frac{105600}{s(s + 50)}}
$$

**Yaw axis ($\delta_\psi \to r$):**

$$
\boxed{G_r(s) = \frac{365.7}{s(0.02s + 1)} = \frac{18285}{s(s + 50)}}
$$

#### Characteristics Summary

| Axis | Gain | Poles | Characteristics |
|------|------|-------|-----------------|
| Roll | 2112 | $s=0, -50$ | Integrator + 20ms time constant |
| Pitch | 2112 | $s=0, -50$ | Same as Roll |
| Yaw | 365.7 | $s=0, -50$ | Gain is ~1/6 of Roll/Pitch |

**Discussion:** The yaw axis has a smaller gain because the torque coefficient $C_q$ is smaller
than the moment generated by thrust in roll/pitch, and $I_{zz} > I_{xx}$.
This means the yaw axis response is slower than the other axes, which must be considered in controller design.

## 8. Numerical Integration

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

## 9. Implementation Reference

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
