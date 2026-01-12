# 物理単位ベース制御アロケーションの理論と移行計画

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### 目的

シミュレータで採用している**物理単位ベースの制御アロケーション**をファームウェアに移行し、以下を実現する：

1. **シミュレータとファームウェアの制御系統一**
2. **物理的に意味のある単位での制御設計**
3. **パラメータチューニングの容易化**

### 現状の違い

| 項目 | ファームウェア（現状） | シミュレータ |
|------|----------------------|-------------|
| 入力単位 | 電圧スケール (±3.7V) | 物理単位 (N, Nm) |
| ミキサー係数 | 経験的 (0.25/3.7) | 幾何学ベース (B⁻¹) |
| スケーリング | 暗黙的 | 明示的 |

---

## 2. 理論的背景

### 制御アロケーション行列

X-Quadの制御アロケーションは、仮想制御入力 `u` と個別モータ推力 `T` の関係で定義される。

#### 順方向アロケーション（Forward Allocation）

```
u = B × T

[uₜ ]   [  1      1      1      1   ] [T₁]
[u_φ] = [-y₁   -y₂   -y₃   -y₄  ] [T₂]
[u_θ]   [ x₁    x₂    x₃    x₄  ] [T₃]
[u_ψ]   [-κσ₁  -κσ₂  -κσ₃  -κσ₄ ] [T₄]
```

**変数定義：**
- `uₜ`: 総推力 [N]
- `u_φ`: ロールトルク [Nm]（X軸周り）
- `u_θ`: ピッチトルク [Nm]（Y軸周り）
- `u_ψ`: ヨートルク [Nm]（Z軸周り）
- `Tᵢ`: モータi の推力 [N]
- `xᵢ, yᵢ`: モータ位置 [m]（NED機体座標系）
- `κ`: トルク/推力比 = Cq/Ct [m]
- `σᵢ`: 回転方向（CW=+1, CCW=-1）

#### 逆方向ミキシング（Inverse Mixing）

```
T = B⁻¹ × u
```

対称X-Quadの場合、逆行列は解析的に計算可能：

```
      [1   -1/d   +1/d   +1/κ]
B⁻¹ = [1   -1/d   -1/d   -1/κ] × (1/4)
      [1   +1/d   -1/d   +1/κ]
      [1   +1/d   +1/d   -1/κ]
```

### StampFlyの物理パラメータ

| パラメータ | 記号 | 値 | 単位 |
|-----------|------|-----|------|
| モータ間距離 | r | 32.5 | mm |
| モーメントアーム | d = r/√2 | 23.0 | mm |
| 推力係数 | Ct | 1.00×10⁻⁸ | N/(rad/s)² |
| トルク係数 | Cq | 9.71×10⁻¹¹ | Nm/(rad/s)² |
| トルク/推力比 | κ = Cq/Ct | 9.71×10⁻³ | m |
| バッテリ電圧 | Vbat | 3.7 | V |
| 機体質量 | m | 35 | g |
| ホバー推力 | Thover = mg | 0.343 | N |

### モータ配置（NED機体座標系）

```
              Front (+X)
         FL(M4)     FR(M1)
           CW   ▲    CCW
             ╲  │  ╱
              ╲ │ ╱
               ╲│╱
        -Y ←────╳────→ +Y
               ╱│╲
              ╱ │ ╲
             ╱  │  ╲
           CCW  │   CW
         RL(M3)     RR(M2)
              Rear (-X)
```

| モータ | 位置 | x [m] | y [m] | 回転 | σ |
|--------|------|-------|-------|------|---|
| M1 (FR) | 前右 | +0.023 | +0.023 | CCW | -1 |
| M2 (RR) | 後右 | -0.023 | +0.023 | CW | +1 |
| M3 (RL) | 後左 | -0.023 | -0.023 | CCW | -1 |
| M4 (FL) | 前左 | +0.023 | -0.023 | CW | +1 |

### 具体的なアロケーション行列

StampFlyのパラメータを代入：

```
      [  1.000    1.000    1.000    1.000  ]
B =   [ -0.023   -0.023   +0.023   +0.023  ]
      [ +0.023   -0.023   -0.023   +0.023  ]
      [+0.00971 -0.00971 +0.00971 -0.00971 ]
```

逆行列：

```
       [ 0.25   -10.87   +10.87   +25.75 ]
B⁻¹ =  [ 0.25   -10.87   -10.87   -25.75 ]
       [ 0.25   +10.87   -10.87   +25.75 ]
       [ 0.25   +10.87   +10.87   -25.75 ]
```

**係数の解釈：**
- `10.87 = 1/(4d) = 1/(4×0.023)`
- `25.75 = 1/(4κ) = 1/(4×0.00971)`

---

## 3. 推力-Duty変換

### 定常状態近似

モータ動特性の定常状態（dω/dt = 0）から、所望推力に必要な電圧を逆算：

```
ω = √(T / Ct)                    ... (1) 推力→角速度

V = Rm[(Dm + Km²/Rm)ω + Cqω² + Qf] / Km  ... (2) 角速度→電圧

duty = V / Vbat                  ... (3) 電圧→Duty
```

### モータパラメータ（実測値）

| パラメータ | 記号 | 値 | 単位 |
|-----------|------|-----|------|
| 抵抗 | Rm | 0.34 | Ω |
| モータ定数 | Km | 6.125×10⁻⁴ | V·s/rad |
| 粘性抵抗 | Dm | 3.69×10⁻⁸ | Nm·s/rad |
| 摩擦トルク | Qf | 2.76×10⁻⁵ | Nm |

### ホバー時の検証

```
Thover = 0.343 N (35g × 9.81)
T_per_motor = 0.343 / 4 = 0.0858 N

ω = √(0.0858 / 1.0e-8) = 2930 rad/s

V ≈ 2.78 V
duty = 2.78 / 3.7 = 0.75 (75%)
```

---

## 4. PID出力からトルクへの変換

### 現状ファームウェア

```cpp
// PID出力: ±3.7V（電圧スケール）
// ミキサー: duty = thrust + 0.25*(roll + pitch + yaw)/3.7
```

**問題点：**
- PID出力の物理的意味が不明確
- トルク→推力→Dutyの変換が暗黙的

### 提案：物理単位ベースアプローチ

```
PID出力 [rad/s誤差] → トルク [Nm] → 推力 [N] → Duty [0-1]
```

**変換式：**

```
τ = Kp × ε_ω                    ... PID P項（簡略化）

T = B⁻¹ × [uₜ, τ_φ, τ_θ, τ_ψ]ᵀ  ... ミキシング

duty = thrust_to_duty(T)        ... 推力→Duty
```

### PIDゲインの物理的解釈

現在のファームウェアゲイン（電圧スケール出力）：

| 軸 | Kp | Ti | Td | 出力上限 |
|----|----|----|-----|---------|
| Roll | 0.65 | 0.7s | 0.01s | ±3.7V |
| Pitch | 0.95 | 0.7s | 0.025s | ±3.7V |
| Yaw | 3.0 | 0.8s | 0.01s | ±3.7V |

**物理単位への変換：**

電圧出力をトルクに変換するスケーリング係数：

```
k_τ = (0.25 / 3.7) × T_max × d
    = 0.0676 × 0.15 × 0.023
    ≈ 2.33×10⁻⁴ Nm/V
```

これにより：
- Roll最大トルク: 3.7V × 2.33×10⁻⁴ ≈ 0.86 mNm
- 角加速度: τ/Ixx = 0.86e-3 / 9.16e-6 ≈ 94 rad/s²

---

## 5. 移行計画

### Phase 1: 制御アロケーションモジュール作成

**新規ファイル:** `firmware/vehicle/components/sf_algo_control/control_allocation.hpp`

```cpp
namespace stampfly {

struct QuadConfig {
    float d = 0.023f;           // Moment arm [m]
    float kappa = 9.71e-3f;     // Cq/Ct [m]
    float motor_x[4] = {0.023f, -0.023f, -0.023f, 0.023f};
    float motor_y[4] = {0.023f, 0.023f, -0.023f, -0.023f};
    int motor_dir[4] = {-1, 1, -1, 1};
};

class ControlAllocator {
public:
    void init(const QuadConfig& config);

    // 制御入力 [N, Nm] → モータ推力 [N]
    void mix(float u_thrust, float u_roll, float u_pitch, float u_yaw,
             float* thrusts_out);

    // モータ推力 [N] → Duty [0-1]
    void thrusts_to_duties(const float* thrusts, float* duties_out);

private:
    float B_inv_[4][4];  // ミキシング行列
};

} // namespace stampfly
```

### Phase 2: モータモデル統合

**新規ファイル:** `firmware/vehicle/components/sf_algo_control/motor_model.hpp`

```cpp
namespace stampfly {

struct MotorParams {
    float Ct = 1.0e-8f;    // Thrust coefficient
    float Cq = 9.71e-11f;  // Torque coefficient
    float Rm = 0.34f;      // Resistance
    float Km = 6.125e-4f;  // Motor constant
    float Vbat = 3.7f;     // Battery voltage
};

// 推力→Duty変換（定常状態近似）
float thrust_to_duty(float thrust, const MotorParams& params);

} // namespace stampfly
```

### Phase 3: PIDゲイン再設計

物理単位出力用のゲイン設計：

| 軸 | 慣性モーメント | 目標帯域 | Kp (物理) |
|----|--------------|---------|----------|
| Roll | 9.16×10⁻⁶ kg·m² | 30 rad/s | ~0.008 Nm/(rad/s) |
| Pitch | 13.3×10⁻⁶ kg·m² | 30 rad/s | ~0.012 Nm/(rad/s) |
| Yaw | 20.4×10⁻⁶ kg·m² | 15 rad/s | ~0.009 Nm/(rad/s) |

**導出：**
```
閉ループ帯域 ω_c = Kp × Kt / I
Kp = ω_c × I / Kt

ここで Kt ≈ 1 (角速度→トルクの物理ゲイン)
```

### Phase 4: 段階的移行

1. **Step 1**: 新アロケーションモジュールを追加（既存と並存）
2. **Step 2**: コンパイルスイッチで切り替え可能に
3. **Step 3**: シミュレータで検証
4. **Step 4**: 実機テスト
5. **Step 5**: 旧コード削除

---

## 6. 変更対象ファイル

| ファイル | 変更内容 |
|---------|---------|
| `components/sf_algo_control/control_allocation.hpp` | **新規作成** |
| `components/sf_algo_control/control_allocation.cpp` | **新規作成** |
| `components/sf_algo_control/motor_model.hpp` | **新規作成** |
| `components/sf_algo_control/motor_model.cpp` | **新規作成** |
| `components/sf_hal_motor/motor_driver.hpp` | setMixerOutput廃止、setMotors追加 |
| `components/sf_hal_motor/motor_driver.cpp` | 同上 |
| `main/tasks/control_task.cpp` | 新アロケータ使用 |
| `main/config.hpp` | 物理パラメータ追加 |

---

## 7. 検証方法

### シミュレータ検証

1. 同一PIDゲインでシミュレータとファームウェアの応答比較
2. ステップ応答の一致確認
3. ホバー時のDuty値比較

### 実機検証

1. ホバーテスト：安定性確認
2. ステップ入力：姿勢応答確認
3. 外乱応答：リカバリ性能確認

---

<a id="english"></a>

## 1. Overview

### Objective

Migrate the **physical units-based control allocation** from the simulator to firmware:

1. **Unify control systems between simulator and firmware**
2. **Control design with physically meaningful units**
3. **Simplified parameter tuning**

### Current Differences

| Item | Firmware (Current) | Simulator |
|------|-------------------|-----------|
| Input units | Voltage scale (±3.7V) | Physical units (N, Nm) |
| Mixer coefficients | Empirical (0.25/3.7) | Geometry-based (B⁻¹) |
| Scaling | Implicit | Explicit |

---

## 2. Theoretical Background

### Control Allocation Matrix

X-Quad control allocation is defined by the relationship between virtual control inputs `u` and individual motor thrusts `T`.

#### Forward Allocation

```
u = B × T

[uₜ ]   [  1      1      1      1   ] [T₁]
[u_φ] = [-y₁   -y₂   -y₃   -y₄  ] [T₂]
[u_θ]   [ x₁    x₂    x₃    x₄  ] [T₃]
[u_ψ]   [-κσ₁  -κσ₂  -κσ₃  -κσ₄ ] [T₄]
```

#### Inverse Mixing

```
T = B⁻¹ × u
```

For symmetric X-Quad:

```
      [1   -1/d   +1/d   +1/κ]
B⁻¹ = [1   -1/d   -1/d   -1/κ] × (1/4)
      [1   +1/d   -1/d   +1/κ]
      [1   +1/d   +1/d   -1/κ]
```

---

## 5. Migration Plan

### Phase 1: Create Control Allocation Module

**New file:** `firmware/vehicle/components/sf_algo_control/control_allocation.hpp`

### Phase 2: Integrate Motor Model

**New file:** `firmware/vehicle/components/sf_algo_control/motor_model.hpp`

### Phase 3: Redesign PID Gains

Design gains for physical unit output based on inertia and target bandwidth.

### Phase 4: Gradual Migration

1. Add new allocation module (coexist with existing)
2. Enable compile-time switch
3. Verify in simulator
4. Test on hardware
5. Remove legacy code

---

## 7. Verification

### Simulator Verification

1. Compare responses with identical PID gains
2. Confirm step response matching
3. Compare hover duty values

### Hardware Verification

1. Hover test: stability check
2. Step input: attitude response
3. Disturbance response: recovery performance
