# 物理単位ベース制御アロケーションの理論と移行計画

> **【2026-08-03: κ再改定】** 本書は §2〜§7 で κ=9.71×10⁻³（移行計画当時の値）を使った移行記録として書かれている。
> その後 2026-07-17 に κ は thrust stand実測由来の $C_T$ を使った値へ更新されたが、この $C_T$ は撤回された
> （新プロペラでの電圧・回転数・推力の有効な同時計測を欠くため）。撤回に伴い κ は 2026-08-03 に **4.10×10⁻³**
> （$C_Q$=4.10e-11 のコーストダウン実測・確定 ÷ $C_T$=1.00e-8 の暫定採用値）へ再改定された。ファームウェアは
> `rate.yaw.kp`・`rate.yaw.max_torque` を旧κとの比で同率再スケールしており、飛行挙動は不変。**本書自体（§2〜§7の
> 数値例・コード片）は移行計画当時の記録として変更せず保存する** — 現在の採用値は
> `docs/architecture/stampfly-parameters.md` §3、現在の実装は `firmware/vehicle/components/sf_actuator/actuator.cpp` を参照。

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

> **現状について（2026年promotion後の注記）:** 本ドキュメントは、シミュレータの物理単位ベース制御アロケーションをファームウェアへ移行する計画として書かれた。この移行は完了しており、現行の主力ファーム `firmware/vehicle`（旧 `vehicle_new`。実機でのPOS_HOLD位置保持検証を機に2026年にpromotionされ、レイヤード構成の旧ファームは `firmware/vehicle_old` として凍結）では、物理単位（推力[N]・トルク[Nm]）ベースの制御アロケーションが**唯一の実装**であり、移行前の電圧スケール方式や `USE_PHYSICAL_UNITS` のようなコンパイルスイッチは存在しない。本ドキュメントの理論的背景（§2〜§5）は現在も有効だが、§6「移行計画」・§7「変更対象ファイル」に登場するファイルパス（`sf_algo_control` 等、旧レイヤード命名）は、当時の実装対象であった `firmware/vehicle_old` を指しており、現行の `firmware/vehicle` 側の実装は `sf_actuator/actuator.cpp`（ミキサー＋モータ曲線を1ファイルに統合、`ControlAllocator`/`motor_model.hpp`のような分離クラスはない）に相当する。詳細は各節の注記を参照。

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

V ≈ 2.33 V
duty = 2.33 / 3.7 = 0.63 (63%)
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

電圧出力をトルクに変換するスケーリング係数（ホバー動作点での線形化）：

```
k_τ = 4 × (0.25 / 3.7) × (dT/dDuty)_hover × d
    ≈ 4 × 0.0676 × 0.23 × 0.023
    ≈ 1.4×10⁻³ Nm/V
```

**注意:**
- 4モータ全ての寄与を考慮（factor of 4）
- 非線形モータモデルをホバー点で線形化: (dT/dDuty)_hover ≈ 0.23 N/duty
- 以前の導出（k_τ = 2.33×10⁻⁴）は1モータ分かつT_max=0.15を使用しており誤り

これにより：
- Roll最大トルク: 3.7V × 1.4×10⁻³ ≈ 5.2 mNm
- 角加速度: τ/Ixx = 5.2e-3 / 9.16e-6 ≈ 570 rad/s²

---

## 5. PIDゲイン変換の理論的根拠

### ファームウェアPID形式の確認

ファームウェア（当時 `sf_algo_pid/pid.cpp`、現行 `firmware/vehicle` では `sf_controller_pid/include/pid.hpp` に相当。この点は現行でも同じ）は**標準形式（ISA形式）**を採用：

```cpp
output = P_ + I_ + D_
       = Kp_ * error_ + Kp_ * integral_ + Kp_ * deriv_filtered_
       = Kp_ × [e + integral_ + deriv_filtered_]
```

連続時間表現：

```
u = Kp × [e + (1/Ti)∫e dt + Td × de/dt]
```

### なぜTi, Tdは変換不要か：数学的証明

**旧システム（電圧出力 u_V）：**
```
u_V = Kp_old × [e + (1/Ti_old)∫e dt + Td_old × de/dt]
```

**新システム（トルク出力 u_τ）：**
```
u_τ = Kp_new × [e + (1/Ti_new)∫e dt + Td_new × de/dt]
```

等価動作条件 `u_τ = k × u_V`（kはスケーリング係数）を満たすには：

```
Kp_new × [e + (1/Ti_new)∫e dt + Td_new × de/dt]
    = k × Kp_old × [e + (1/Ti_old)∫e dt + Td_old × de/dt]
```

任意のe(t)で成立するためには、各項の係数が一致：

| 項 | 左辺係数 | 右辺係数 | 結論 |
|----|---------|---------|------|
| e | Kp_new | k × Kp_old | Kp_new = k × Kp_old |
| ∫e dt | Kp_new/Ti_new | k × Kp_old/Ti_old | Ti_new = Ti_old |
| de/dt | Kp_new × Td_new | k × Kp_old × Td_old | Td_new = Td_old |

**証明（Ti）：**
```
Kp_new/Ti_new = k × Kp_old/Ti_old
(k × Kp_old)/Ti_new = k × Kp_old/Ti_old
∴ Ti_new = Ti_old
```

**証明（Td）：**
```
Kp_new × Td_new = k × Kp_old × Td_old
(k × Kp_old) × Td_new = k × Kp_old × Td_old
∴ Td_new = Td_old
```

### 物理的解釈

| パラメータ | 単位 | 物理的意味 | 変換 |
|-----------|------|-----------|------|
| Kp | [出力単位/誤差単位] | 比例ゲイン（出力スケール） | **要変換** |
| Ti | [s] | 積分時定数（P項と同等になる時間） | 不変 |
| Td | [s] | 微分時定数（予測時間） | 不変 |
| η | [-] | 微分フィルタ係数（無次元） | 不変 |

**Ti, Tdが不変な理由：**
- 時定数（秒）であり、**時間領域の動的挙動**を定義
- P項、I項、D項の**相対的な寄与比率**を決定
- 出力の物理単位には依存しない

### 結論

```
新Kp = k_scale × 旧Kp
新Ti = 旧Ti（変更不要）
新Td = 旧Td（変更不要）
新η  = 旧η（変更不要）
```

### ゲイン変換対応表

スケーリング係数（ホバー動作点での線形化、4モータ寄与）：
```
k_τ_roll/pitch = 4 × (0.25/3.7) × 0.23 × 0.023 ≈ 1.4×10⁻³ Nm/V
k_τ_yaw        = 4 × (0.25/3.7) × 0.23 × 0.00971 ≈ 5.9×10⁻⁴ Nm/V
```

| 軸 | 旧Kp [V/(rad/s)] | k_τ [Nm/V] | 新Kp [Nm/(rad/s)] |
|----|-----------------|------------|-------------------|
| Roll | 0.65 | 1.4×10⁻³ | **9.1×10⁻⁴** |
| Pitch | 0.95 | 1.4×10⁻³ | **1.33×10⁻³** |
| Yaw | 3.0 | 5.9×10⁻⁴ | **1.77×10⁻³** |

| 軸 | 旧Ti [s] | 新Ti [s] | 旧Td [s] | 新Td [s] |
|----|---------|---------|---------|---------|
| Roll | 0.7 | **0.7** | 0.01 | **0.01** |
| Pitch | 0.7 | **0.7** | 0.025 | **0.025** |
| Yaw | 0.8 | **0.8** | 0.01 | **0.01** |

| 軸 | 旧出力上限 [V] | 新出力上限 [Nm] |
|----|--------------|----------------|
| Roll | ±3.7 | **±5.2×10⁻³** |
| Pitch | ±3.7 | **±5.2×10⁻³** |
| Yaw | ±3.7 | **±2.2×10⁻³** |

**注記：** η（微分フィルタ係数）= 0.125 は変更不要

---

## 6. 移行計画

> **注記:** 以下 Phase 1〜4 は `firmware/vehicle_old`（旧レイヤード命名の当時の `firmware/vehicle`）での実装記録。現行の `firmware/vehicle` では、この移行計画のゴールである「物理単位ベース制御アロケーション」がコンパイルスイッチなしの唯一の実装として `sf_actuator/actuator.cpp` に存在する（`ControlAllocator`/`motor_model.hpp`のような分離クラス構成ではなく、ミキサー式と推力→duty変換を1ファイル内の関数群で実装）。

### Phase 1: 制御アロケーションモジュール作成 ✅ 完了（`firmware/vehicle_old` にて）

**実装ファイル（`firmware/vehicle_old`）:** `components/sf_algo_control/`

- `include/control_allocation.hpp` - QuadConfig, ControlAllocatorクラス
- `control_allocation.cpp` - B行列、B⁻¹行列の構築とミキシング実装

```cpp
namespace stampfly {

struct QuadConfig {
    float d = 0.023f;           // Moment arm [m]
    float kappa = 9.71e-3f;     // Cq/Ct [m]
    float motor_x[4] = {0.023f, -0.023f, -0.023f, 0.023f};
    float motor_y[4] = {0.023f, 0.023f, -0.023f, -0.023f};
    int motor_dir[4] = {-1, 1, -1, 1};
    float max_thrust_per_motor = 0.15f;
};

class ControlAllocator {
public:
    void init(const QuadConfig& config);
    void setMotorParams(const MotorParams& params);

    // 制御入力 [N, Nm] → モータ推力 [N]
    bool mix(const float control[4], float thrusts_out[4]) const;

    // モータ推力 [N] → 制御入力 [N, Nm]
    void allocate(const float thrusts[4], float control_out[4]) const;

    // モータ推力 [N] → Duty [0-1]
    void thrustsToDuties(const float thrusts[4], float duties_out[4]) const;

private:
    float B_[4][4];      // アロケーション行列
    float B_inv_[4][4];  // ミキシング行列
};

} // namespace stampfly
```

### Phase 2: モータモデル統合 ✅ 完了（`firmware/vehicle_old` にて）

**実装ファイル（`firmware/vehicle_old`）:** `components/sf_algo_control/`

- `include/motor_model.hpp` - MotorParams, スタンドアロン変換関数
- `motor_model.cpp` - DEFAULT_MOTOR_PARAMS定義

```cpp
namespace stampfly {

struct MotorParams {
    float Ct = 1.0e-8f;     // Thrust coefficient [N/(rad/s)²]
    float Cq = 9.71e-11f;   // Torque coefficient [Nm/(rad/s)²]
    float Rm = 0.34f;       // Motor resistance [Ω]
    float Km = 6.125e-4f;   // Motor constant [V·s/rad]
    float Dm = 3.69e-8f;    // Viscous damping [Nm·s/rad]
    float Qf = 2.76e-5f;    // Friction torque [Nm]
    float Jm = 1.0e-9f;     // Motor+propeller inertia [kg·m²]
    float Vbat = 3.7f;      // Battery voltage [V]
};

// スタンドアロン変換関数
inline float thrustToDuty(float thrust, const MotorParams& params);
inline float thrustToOmega(float thrust, float Ct);
inline float omegaToVoltage(float omega, const MotorParams& params);
inline float dutyToThrust(float duty, const MotorParams& params, int max_iter = 10);

extern const MotorParams DEFAULT_MOTOR_PARAMS;

} // namespace stampfly
```

### Phase 3: PIDゲイン再設計 ✅ 完了（`firmware/vehicle_old` にて）

**実装ファイル（`firmware/vehicle_old`）:** `main/config.hpp`

コンパイルスイッチ `USE_PHYSICAL_UNITS` による切り替え（`firmware/vehicle_old` 当時の実装。現行 `firmware/vehicle` にこのスイッチはなく、常時物理単位。現行のゲインSSOTは `sf_core/params.cpp` の `table[]`、パラメータ名は `rate.roll.kp` 等 — 値は `docs/architecture/stampfly-parameters.md` 参照）：

```cpp
namespace rate_control {

// 物理単位モード切り替え (1: トルク出力, 0: 電圧出力)
#define USE_PHYSICAL_UNITS 1

#if USE_PHYSICAL_UNITS
// 物理単位ベースゲイン [Nm/(rad/s)]
// k_τ = 4 × (0.25/3.7) × 0.23 × d ≈ 1.4e-3 Nm/V (roll/pitch)
inline constexpr float ROLL_RATE_KP = 9.1e-4f;    // 0.65 × 1.4e-3
inline constexpr float PITCH_RATE_KP = 1.33e-3f;  // 0.95 × 1.4e-3
inline constexpr float YAW_RATE_KP = 1.77e-3f;    // 3.0 × 5.9e-4
inline constexpr float ROLL_OUTPUT_LIMIT = 5.2e-3f;   // [Nm] = 3.7 × 1.4e-3
inline constexpr float PITCH_OUTPUT_LIMIT = 5.2e-3f;  // [Nm]
inline constexpr float YAW_OUTPUT_LIMIT = 2.2e-3f;    // [Nm] = 3.7 × 5.9e-4
#else
// 電圧スケールゲイン（レガシー）
inline constexpr float ROLL_RATE_KP = 0.65f;
// ...
#endif

// Ti, Td は不変（時定数）
inline constexpr float ROLL_RATE_TI = 0.7f;
inline constexpr float ROLL_RATE_TD = 0.01f;
// ...

} // namespace rate_control
```

### Phase 4: 段階的移行 ✅ 完了（コード統合、`firmware/vehicle_old` にて）

**実装内容（`firmware/vehicle_old`）:**

1. `motor_driver.hpp/cpp` - `setMotorDuties()` 関数追加
2. `control_task.cpp` - ControlAllocator統合、条件コンパイル対応
3. `rate_controller.hpp` - ControlAllocatorメンバ追加

**制御フロー（物理単位モード）:**

```
スロットル [0-1] → 総推力 [N] = throttle × 4 × 0.15
PID出力 [Nm] → ロール/ピッチ/ヨートルク
         ↓
    ControlAllocator.mix()
         ↓
    モータ推力 [N] × 4
         ↓
    thrustsToDuties()
         ↓
    モータDuty [0-1] × 4
         ↓
    setMotorDuties()
```

| ステップ | 内容 | 状態（`firmware/vehicle_old` 時点） |
|---------|------|------|
| Step 1 | 新アロケーションモジュールを追加（既存と並存） | ✅ 完了 |
| Step 2 | コンパイルスイッチで切り替え可能に | ✅ 完了 |
| Step 3 | control_task.cppに統合 | ✅ 完了 |
| Step 4 | シミュレータで検証 | 🔄 未実施 |
| Step 5 | 実機テスト | 🔄 未実施 |
| Step 6 | 旧コード削除（オプション） | 🔄 未実施 |

> **その後の展開:** 実機テスト・移行完了は、`firmware/vehicle_old` 上でのStep 5/6としてではなく、並行して開発されていた新設計ファーム（`vehicle_new`）側で達成された。`vehicle_new` は実機でのPOS_HOLD位置保持検証を経て2026年に `firmware/vehicle`（主力機）へpromotionされ、物理単位ベースの制御アロケーションが `sf_actuator/actuator.cpp` として恒久実装になっている。`firmware/vehicle_old`（旧レイヤード構成）は87フライトの実績を持つ凍結版として保持され、新規開発の対象ではない。

---

## 7. 変更対象ファイル

> **注記:** 以下は `firmware/vehicle_old`（当時の `firmware/vehicle`）における変更履歴。現行 `firmware/vehicle` の対応実装は `sf_actuator/actuator.cpp`（ミキサー＋モーター曲線）・`sf_core/params.cpp`（ゲインSSOT）・`sf_controller_pid/pid_controller.cpp`（PID統合）である。

| ファイル（`firmware/vehicle_old`） | 変更内容 | 状態 |
|---------|---------|------|
| `components/sf_algo_control/include/control_allocation.hpp` | 新規作成 | ✅ |
| `components/sf_algo_control/control_allocation.cpp` | 新規作成 | ✅ |
| `components/sf_algo_control/include/motor_model.hpp` | 新規作成 | ✅ |
| `components/sf_algo_control/motor_model.cpp` | 新規作成 | ✅ |
| `components/sf_algo_control/CMakeLists.txt` | 新規作成 | ✅ |
| `main/config.hpp` | 物理単位PIDゲイン追加、USE_PHYSICAL_UNITSスイッチ | ✅ |
| `components/sf_hal_motor/motor_driver.hpp` | setMotorDuties()追加 | ✅ |
| `components/sf_hal_motor/motor_driver.cpp` | setMotorDuties()実装 | ✅ |
| `main/rate_controller.hpp` | ControlAllocatorメンバ追加 | ✅ |
| `main/tasks/control_task.cpp` | ControlAllocator統合、条件コンパイル | ✅ |
| `main/CMakeLists.txt` | sf_algo_control依存追加 | ✅ |
| `components/sf_svc_cli/CMakeLists.txt` | sf_algo_control依存追加 | ✅ |

---

## 8. 検証方法

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

> **Note on current status (post-2026 promotion):** This document was written as a plan to migrate the simulator's physical-units-based control allocation into firmware. That migration is complete: the current primary firmware `firmware/vehicle` (formerly `vehicle_new`, promoted after real-hardware POS_HOLD validation in 2026; the earlier layered firmware is now frozen as `firmware/vehicle_old`) implements physical-units (thrust [N] / torque [Nm]) control allocation as the **only** implementation — there is no voltage-scale mode and no `USE_PHYSICAL_UNITS`-style compile switch. The theoretical background (§2-§5) remains valid, but the file paths in §6 "Migration Plan" / §7 "Files Changed" (`sf_algo_control`, etc., the old layered naming) refer to `firmware/vehicle_old`, where that work was actually done; the current `firmware/vehicle` equivalent is `sf_actuator/actuator.cpp` (mixer + motor curve merged into one file — no separate `ControlAllocator`/`motor_model.hpp` classes).

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

## 5. Theoretical Basis for PID Gain Conversion

### Firmware PID Form

The firmware (then `sf_algo_pid/pid.cpp`; the current `firmware/vehicle` equivalent is `sf_controller_pid/include/pid.hpp` — this point still holds today) uses **Standard Form (ISA Form)**:

```
u = Kp × [e + (1/Ti)∫e dt + Td × de/dt]
```

### Mathematical Proof: Why Ti and Td Remain Unchanged

For equivalent behavior `u_τ = k × u_V` (k = scaling factor):

| Term | LHS Coefficient | RHS Coefficient | Result |
|------|-----------------|-----------------|--------|
| e | Kp_new | k × Kp_old | Kp_new = k × Kp_old |
| ∫e dt | Kp_new/Ti_new | k × Kp_old/Ti_old | **Ti_new = Ti_old** |
| de/dt | Kp_new × Td_new | k × Kp_old × Td_old | **Td_new = Td_old** |

**Conclusion:**
- Ti, Td are **time constants** [seconds] defining dynamic behavior
- They determine **relative contribution ratios** of P, I, D terms
- Only **Kp needs scaling** (it has output units)

### Gain Conversion Table

Scaling factors (linearized at hover operating point, 4 motor contributions):
```
k_τ_roll/pitch = 4 × (0.25/3.7) × 0.23 × 0.023 ≈ 1.4×10⁻³ Nm/V
k_τ_yaw        = 4 × (0.25/3.7) × 0.23 × 0.00971 ≈ 5.9×10⁻⁴ Nm/V
```

| Axis | Old Kp [V/(rad/s)] | k_τ [Nm/V] | New Kp [Nm/(rad/s)] |
|------|-------------------|------------|---------------------|
| Roll | 0.65 | 1.4×10⁻³ | **9.1×10⁻⁴** |
| Pitch | 0.95 | 1.4×10⁻³ | **1.33×10⁻³** |
| Yaw | 3.0 | 5.9×10⁻⁴ | **1.77×10⁻³** |

| Axis | Old Ti [s] | New Ti [s] | Old Td [s] | New Td [s] |
|------|-----------|-----------|-----------|-----------|
| Roll | 0.7 | **0.7** | 0.01 | **0.01** |
| Pitch | 0.7 | **0.7** | 0.025 | **0.025** |
| Yaw | 0.8 | **0.8** | 0.01 | **0.01** |

| Axis | Old Output Limit [V] | New Output Limit [Nm] |
|------|---------------------|----------------------|
| Roll | ±3.7 | **±5.2×10⁻³** |
| Pitch | ±3.7 | **±5.2×10⁻³** |
| Yaw | ±3.7 | **±2.2×10⁻³** |

**Note:** η (derivative filter coefficient) = 0.125 remains unchanged

---

## 6. Migration Plan

> **Note:** Phases 1-4 below were carried out in `firmware/vehicle_old` (the layered firmware that was then called `firmware/vehicle`). In the current `firmware/vehicle`, the goal of this migration — physical-units control allocation with no compile switch — is the sole implementation, in `sf_actuator/actuator.cpp` (mixer formula and thrust-to-duty conversion as a set of functions in one file, not the separate `ControlAllocator`/`motor_model.hpp` classes below).

### Phase 1: Create Control Allocation Module (done in `firmware/vehicle_old`)

**File (`firmware/vehicle_old`):** `components/sf_algo_control/control_allocation.hpp`

### Phase 2: Integrate Motor Model (done in `firmware/vehicle_old`)

**File (`firmware/vehicle_old`):** `components/sf_algo_control/motor_model.hpp`

### Phase 3: Redesign PID Gains (done in `firmware/vehicle_old`)

Design gains for physical unit output based on inertia and target bandwidth. In the current `firmware/vehicle`, the gain SSOT is the `table[]` in `sf_core/params.cpp` (names like `rate.roll.kp`); see `docs/architecture/stampfly-parameters.md` for current values.

### Phase 4: Gradual Migration (done in `firmware/vehicle_old`)

1. Add new allocation module (coexist with existing)
2. Enable compile-time switch
3. Verify in simulator
4. Test on hardware
5. Remove legacy code

**What happened next:** Hardware testing and full migration were not completed as Steps 4-6 above in `firmware/vehicle_old`; instead they were achieved in the parallel clean-slate redesign (`vehicle_new`), which reached real-hardware POS_HOLD validation and was promoted to `firmware/vehicle` (the primary firmware) in 2026, with physical-units control allocation as its permanent `sf_actuator/actuator.cpp` implementation. `firmware/vehicle_old` (the earlier layered design) is retained as a frozen reference with 87 real flights, not a target for new development.

---

## 8. Verification

> **Note:** This section describes the verification plan as originally written for `firmware/vehicle_old`. The physical-units approach itself has since been verified end-to-end (including real-hardware flight) via the separate `vehicle_new` → `firmware/vehicle` promotion path described above.

### Simulator Verification

1. Compare responses with identical PID gains
2. Confirm step response matching
3. Compare hover duty values

### Hardware Verification

1. Hover test: stability check
2. Step input: attitude response
3. Disturbance response: recovery performance
