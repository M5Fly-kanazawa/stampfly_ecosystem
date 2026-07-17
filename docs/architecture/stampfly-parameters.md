# StampFly 物理パラメータリファレンス（シミュレータ＆ファームウェア）

> **【実測値ノート 2026-07-15】** モータ+プロペラ系の物理パラメータが再測定で確定した:
> $C_T$=6.7e-9, $C_Q$=4.10e-11（κ=6.12e-3 m）, $J_{mp}$=1.375e-8 kg·m², $\omega_{hover}$=3670 rad/s,
> ホバ点実効時定数 ≈18 ms（詳細: multicopter_introduction/notes/qa_log.md Q4-9..13、
> 影響一覧: analysis/reports/param_correction_impact_20260715.md）。
> 本文書の数値例のうち旧値（$C_t$=1.0e-8, $C_q$=9.71e-11, $\omega_{m0}$=2930 等）に基づくものは
> ファームウェア実装の記述としては正確だが、物理値としては上記が正。

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

本ドキュメントは、StampFly のシミュレータおよびファームウェアで使用される物理定数・パラメータの一覧である。
シミュレータのパラメータは実機の測定・同定結果に基づいており、各モジュールに散在している値を一箇所にまとめたものである。
ファームウェア（`firmware/vehicle`。旧 `vehicle_new`、実機POS_HOLD検証を機に2026年promotionで現行機に昇格。レイヤード構成の旧ファームは `firmware/vehicle_old` として凍結）のチューニング可能なパラメータ（PIDゲイン・ESKF設定等）は、`firmware/vehicle/components/sf_core/params.cpp` 内の `table[]`（名前・変数・既定値/最小/最大/変更コールバックを結ぶ明示テーブル）を Single Source of Truth（SSOT）として記載している。GPIOピン割当やタスク優先度等の**変更しない**固定定数は別途 `firmware/vehicle/main/config.hpp` にある（本ドキュメントでは扱わない）。

### 対象読者

- シミュレータを使用・改良する開発者
- ファームウェアの制御パラメータを確認・調整する開発者
- 制御系を設計する学生・研究者
- モデルパラメータの根拠を確認したい人

### PID 表記の違い

シミュレータとファームウェアでは PID ゲインの表記形式が異なる。

| 項目 | シミュレータ | ファームウェア |
|------|------------|--------------|
| 表記形式 | Kp / Ki / Kd | Kp / Ti / Td |
| 名称 | 標準形式 | 時定数形式 |

ファームウェアの時定数形式の伝達関数：

```
C(s) = Kp × (1 + 1/(Ti·s) + Td·s / (η·Td·s + 1))
```

標準形式との変換関係：

| 変換 | 式 |
|------|-----|
| Ki → Ti | Ti = Kp / Ki |
| Kd → Td | Td = Kd / Kp |
| Ti → Ki | Ki = Kp / Ti |
| Td → Kd | Kd = Kp × Td |

η は不完全微分フィルタ係数（デフォルト 0.125）。微分項の高周波ゲインを 1/η に制限する。

### 参照ファイル

#### シミュレータ

| モジュール | ファイルパス |
|-----------|-------------|
| 機体ダイナミクス | `simulator/core/dynamics.py` |
| モーター・プロペラ | `simulator/core/motors.py` |
| 剛体物理 | `simulator/core/physics.py` |
| 空気力学 | `simulator/core/aerodynamics.py` |
| IMUセンサ | `simulator/sensors/imu.py` |
| 気圧センサ | `simulator/sensors/barometer.py` |
| モーターミキサー | `simulator/control/motor_mixer.py` |

#### ファームウェア

| モジュール | ファイルパス |
|-----------|-------------|
| パラメータ設定（SSOT、可変値） | `firmware/vehicle/components/sf_core/params.cpp` |
| 固定定数（GPIO・タスク優先度等） | `firmware/vehicle/main/config.hpp` |
| カスケードPID（レート/姿勢/高度/位置 全段） | `firmware/vehicle/components/sf_controller_pid/pid_controller.cpp` |
| ESKF | `firmware/vehicle/components/sf_estimator_eskf/include/eskf_estimator.hpp` |
| PIDコア | `firmware/vehicle/components/sf_controller_pid/include/pid.hpp` |
| 制御割当（ミキサー＋モーター曲線） | `firmware/vehicle/components/sf_actuator/actuator.cpp` |

## 2. 機体パラメータ

### 質量特性

| パラメータ | 記号 | シミュレータ | ファームウェア | 単位 | 備考 |
|-----------|------|------------|--------------|------|------|
| 機体質量 | m | 0.035 | 0.037 | kg | バッテリー込み |
| Roll慣性モーメント | Ixx | 9.16×10⁻⁶ | - | kg·m² | |
| Pitch慣性モーメント | Iyy | 13.3×10⁻⁶ | - | kg·m² | |
| Yaw慣性モーメント | Izz | 20.4×10⁻⁶ | - | kg·m² | |

> **注記:** ファームウェアの機体質量 0.037 kg は高度制御の重力補償（`sf_controller_pid`, `PidController::kMassG` = 0.037×9.80665）で使用される。シミュレータの 0.035 kg との差は個体差・バッテリー重量の違いによる。

### 機体形状

```
               Front
          FL (M4)   FR (M1)
             ╲   ▲   ╱
              ╲  │  ╱
               ╲ │ ╱
                ╲│╱
                 ╳         ← Center
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲
             ╱   │   ╲
          RL (M3)    RR (M2)
                Rear
```

| パラメータ | 値 | 単位 | 備考 |
|-----------|-----|------|------|
| モーター間距離（対角） | 0.065 | m | 2 × アーム長 |
| アーム長（中心→モーター） | 0.0325 | m | √(x² + y²) = 0.023 × √2 |
| モーメントアーム | 0.023 | m | X/Y座標オフセット（= アーム長/√2） |
| モーター高さ（重心から） | 0.005 | m | |

### 空気抵抗

| パラメータ | 値 | 単位 | 備考 |
|-----------|-----|------|------|
| 並進抗力係数 | 0.1 | - | F_drag = 0.1 × v² |
| 回転抗力係数 | 1×10⁻⁵ | - | τ_drag = 1e-5 × ω² |
| 空気密度 | 1.225 | kg/m³ | 標準大気 |

## 3. モーター・プロペラパラメータ

### モーター配置と回転方向

| モーター | 位置 | X座標 (m) | Y座標 (m) | 回転方向 |
|---------|------|-----------|-----------|----------|
| M1 (FR) | 前右 | +0.023 | +0.023 | CCW |
| M2 (RR) | 後右 | -0.023 | +0.023 | CW |
| M3 (RL) | 後左 | -0.023 | -0.023 | CCW |
| M4 (FL) | 前左 | +0.023 | -0.023 | CW |

### モーター電気特性

| パラメータ | 記号 | 値 | 単位 | 測定方法 |
|-----------|------|-----|------|----------|
| 巻線抵抗 | Rm | 0.34 | Ω | LCRメータ測定 |
| 巻線インダクタンス | Lm | 1.0×10⁻⁶ | H | LCRメータ測定 |
| 回転子慣性モーメント | Jmp | 2.01×10⁻⁸ | kg·m² | 形状・重量から推定 |

### 回転数-電圧特性

電圧 V と角速度 ω の関係は以下のモデルで記述される：

```
V = Am × ω² + Bm × ω + Cm
```

| パラメータ | 記号 | 値 | 単位 | 備考 |
|-----------|------|-----|------|------|
| 2次係数 | Am | 5.39×10⁻⁸ | V/(rad/s)² | 実験同定 |
| 1次係数 | Bm | 6.33×10⁻⁴ | V/(rad/s) | 実験同定 |
| 定数項 | Cm | 1.53×10⁻² | V | 実験同定 |

### 推力・トルク特性

| パラメータ | 記号 | 値 | 単位 | 備考 |
|-----------|------|-----|------|------|
| 推力係数 | Ct | 1.00×10⁻⁸ | N/(rad/s)² | T = Ct × ω² |
| トルク係数 | Cq | 9.71×10⁻¹¹ | N·m/(rad/s)² | Q = Cq × ω² |
| トルク/推力比 | κ | 6.12×10⁻³ | m | κ = Cq/Ct。2026-07-15実測（コーストダウン Cq=4.10e-11 / 推力測定 Ct=6.7e-9）。2026-07-17 にファームウェア B⁻¹ ミキサー（actuator.cpp）・SILプラント（plant.hpp）へ反映。上の Ct/Cq 行はファームウェアのモータ曲線実装（較正済みチェーン）の記述として旧値のまま |

### 派生パラメータ

モデルから導出されるパラメータ：

| パラメータ | 記号 | 計算式 | 値 |
|-----------|------|--------|-----|
| 逆起電力定数 | Km | Cq×Rm/Am | 6.12×10⁻⁴ V/(rad/s) |
| 粘性摩擦係数 | Dm | (Bm - Cq×Rm/Am)×(Cq/Am) | 計算値 |
| 静止摩擦 | Qf | Cm×Cq/Am | 計算値 |

### ホバリング条件

機体重量 0.035 kg × 9.81 m/s² = 0.343 N を4モーターで分担：

| 条件 | 値 | 単位 |
|------|-----|------|
| 1モーターあたり推力 | 0.0858 | N |
| ホバリング角速度 | 約2930 | rad/s |
| ホバリング電圧 | 約2.1 | V |

## 4. センサパラメータ

### IMU (BMI270)

| パラメータ | 値 | 単位 | 備考 |
|-----------|-----|------|------|
| サンプルレート | 400 | Hz | |
| ジャイロフルスケール | 2000 | deg/s | 設定可能: 125/250/500/1000/2000 |
| 加速度フルスケール | 8 | g | 設定可能: 2/4/8/16 |
| ジャイロ分解能 | 16 | bit | |
| 加速度分解能 | 16 | bit | |
| ジャイロノイズ密度 | 0.007 | deg/s/√Hz | データシート値 |
| 加速度ノイズ密度 | 120 | µg/√Hz | データシート値 |
| バイアス不安定性（ジャイロ） | 0.1 | deg/s | |
| バイアス不安定性（加速度） | 0.002 | g | |

### 気圧計 (BMP280)

| パラメータ | 値 | 単位 | 備考 |
|-----------|-----|------|------|
| 気圧分解能 | 0.16 | Pa | X16オーバーサンプリング時 |
| 温度分解能 | 0.01 | °C | |
| 気圧ノイズ | 1.3 | Pa RMS | X16オーバーサンプリング時 |
| 温度ノイズ | 0.005 | °C | |
| 基準海面気圧 | 101325 | Pa | 標準大気 |

### 物理定数

| 定数 | 記号 | シミュレータ | ファームウェア | 単位 | 備考 |
|------|------|------------|--------------|------|------|
| 重力加速度 | g | 9.80665 | 9.81 | m/s² | シミュレータは精密値、FWは簡易値 |
| 空気モル質量 | M | 0.0289644 | - | kg/mol | 気圧高度計算用 |
| 気体定数 | R | 8.31447 | - | J/(mol·K) | 気圧高度計算用 |
| 温度減率 | L | 0.0065 | - | K/m | 気圧高度計算用 |

## 5. シミュレータ制御パラメータ

### モーターミキサー

```
m1 = throttle + scale × (-roll + pitch + yaw)   # FR (M1)
m2 = throttle + scale × (-roll - pitch - yaw)   # RR (M2)
m3 = throttle + scale × (+roll - pitch + yaw)   # RL (M3)
m4 = throttle + scale × (+roll + pitch - yaw)   # FL (M4)
```

| パラメータ | 値 | 備考 |
|-----------|-----|------|
| 出力スケール | 0.0676 | 0.25/3.7 |
| アイドル出力 | 0.05 | ARM時スロットル0で5% |
| 最大推力/モーター | 0.15 N | 推定値 |

### PIDゲイン（シミュレータデフォルト）

> **注記:** シミュレータは標準形式（Kp/Ki/Kd）を使用する。ファームウェアの時定数形式との変換は「PID 表記の違い」節を参照。

#### 姿勢制御（角度→角速度）

| 軸 | Kp | Ki | Kd |
|----|-----|-----|-----|
| Roll | 5.0 | 1.0 | 0.0 |
| Pitch | 5.0 | 1.0 | 0.0 |
| Yaw | 0.5 | 0.01 | 0.0 |

#### 角速度制御（角速度→電圧）

| 軸 | Kp | Ki | Kd |
|----|-----|-----|-----|
| Roll rate | 0.2 | 10.0 | 0.002 |
| Pitch rate | 0.2 | 10.0 | 0.002 |
| Yaw rate | 1.0 | 2.0 | 0.001 |

#### 高度制御

| Kp | Ki | Kd |
|-----|-----|-----|
| 10.0 | 5.0 | 5.0 |

### 制御タイミング

| パラメータ | 値 | 単位 |
|-----------|-----|------|
| シミュレーション刻み | 0.001 | s (1 kHz) |
| 制御周期 | 0.01 | s (100 Hz) |
| バッテリー電圧（公称） | 3.7 | V |

## 6. 外乱モデル

シミュレーションでは以下の外乱を付加可能：

| パラメータ | デフォルト値 | 単位 | 備考 |
|-----------|-------------|------|------|
| モーメント外乱（L） | 4.4×10⁻⁶ | N·m | 正規分布σ |
| モーメント外乱（M） | 4.4×10⁻⁶ | N·m | 正規分布σ |
| モーメント外乱（N） | 4.0×10⁻⁶ | N·m | 正規分布σ |
| 力外乱（X, Y, Z） | 1×10⁻⁶ | N | 正規分布σ |

## 7. ファームウェア制御パラメータ

ファームウェアのチューニング可能パラメータは `firmware/vehicle/components/sf_core/params.cpp` の `table[]`（SSOT）で一元管理されている。パラメータ名は `<カテゴリ>.<サブカテゴリ>.<項目>` のドット区切り（例: `rate.roll.kp`）で、CLI `param get/set/save` から参照・変更できる。GPIOピン割当やタスク優先度等の固定定数は別ファイル `firmware/vehicle/main/config.hpp` にあり、本節では扱わない。
PID は時定数形式（Kp/Ti/Td）を使用する（「PID 表記の違い」節参照）。**物理単位（出力=トルク[Nm]・力[N]）が唯一のモードであり、コンパイルスイッチ（旧`USE_PHYSICAL_UNITS`のようなもの）や電圧出力のレガシーモードは存在しない。**

### レート制御

パラメータ名前空間: `rate.*`（`sf_controller_pid` のカスケードPID、出力は `sf_actuator` のB⁻¹ミキサー向け物理トルク [Nm]）

| 軸 | Kp [Nm/(rad/s)] | Ti [s] | Td [s] |
|----|-----------------|--------|--------|
| Roll | 9.76×10⁻⁴ | 0.7 | 0.01 |
| Pitch | 1.43×10⁻³ | 0.7 | 0.025 |
| Yaw | 1.90×10⁻³ | 0.8 | 0.01 |

不完全微分フィルタ係数: η = 0.125（`sf_controller_pid/include/pid.hpp` の固定値、パラメータ化されていない）

> **注記:** 上表は2026-06-27に実機検証の上で採用された値（実機ふらつき対策のゲイン再設計、Kpのみ変更・Ti/Tdは据置き）。

### 姿勢制御

パラメータ名前空間: `attitude.*`

| 軸 | Kp [(rad/s)/rad] | Ti [s] | Td [s] |
|----|-----------------|--------|--------|
| Roll | 5.0 | 2.0 | 0.04 |
| Pitch | 5.0 | 2.0 | 0.04 |

| パラメータ | 値 | 単位 | 備考 |
|-----------|-----|------|------|
| 姿勢ループ出力上限 | 3.0 | rad/s | 角速度指令の上限（`max_att_rate_sp_`、固定値） |
| η | 0.125 | - | 不完全微分フィルタ係数（固定値） |

#### 姿勢トリム・ヘディングホールド（新規）

| パラメータ | 既定値 | 範囲 | 備考 |
|-----------|-------|------|------|
| `attitude.roll.trim` | 0.0 rad | ±0.1 | 平衡傾き（`sf trim analyze` で飛行同定、オンボード自動学習あり） |
| `attitude.pitch.trim` | 0.0 rad | ±0.1 | 同上 |
| `attitude.trim.learn` | 1（有効） | 0/1 | オンボード自動トリム学習の有効/無効 |
| `attitude.yawhold.kp` | 3.0 | 0〜10 | ヘディングホールドPゲイン（0で無効） |
| `attitude.yawhold.rate_max` | 2.0 rad/s | 0.1〜5 | ヘディング補正レート上限 |

### 高度制御

パラメータ名前空間: `altitude.*`（PIのみ、Tdなし）

| パラメータ | 値 | 単位 |
|-----------|-----|------|
| 機体質量（`kMassG` 計算用） | 0.037 | kg |
| 重力加速度 | 9.80665 | m/s² |

#### 高度 PID（位置→速度）

| Kp | Ti [s] | OutputMax [m/s] | 備考 |
|-----|--------|-----------------|------|
| 0.45 | 7.0 | 0.5（`altitude.climb_rate`） | 下降レート上限は別パラメータ `altitude.descent_rate`（既定 0.5 m/s） |

#### 速度 PID（速度→推力補正）

| Kp | Ti [s] | OutputMax [N] |
|-----|--------|---------------|
| 0.1 | 2.5 | 0.15 |

#### ホバー推力フィードフォワード（新規）

| パラメータ | 既定値 | 範囲 | 備考 |
|-----------|-------|------|------|
| `hover.thrust_corr` | 1.12 | 0.5〜2.0 | モータ曲線が実機推力を約12%過大に見積もる分の補正係数（飛行実測） |
| `hover.thrust.learn` | 1（有効） | 0/1 | オンボード自動ホバー推力学習（着陸時にNVS保存） |

### 位置制御

パラメータ名前空間: `position.*`（PIのみ、Tdなし）

#### 位置 PID（位置→速度）

| Kp | Ti [s] | OutputMax [m/s] |
|-----|--------|-----------------|
| 0.4 | 5.0 | 1.0 |

#### 速度 PID（速度→水平加速度）

| Kp | Ti [s] | OutputMax [m/s²] | 備考 |
|-----|--------|-------------------|------|
| 3.0 | 2.0 | ≈1.71（= g × 傾き上限10°） | 出力は水平加速度。傾き角へは g で除算（POS_HOLD傾き上限10°でクランプ） |

> **注記:** 上記は2026-06-22の実機POS_HOLD飛行を経て再調整された値（初期値 pos.kp=1.0/vel.kp=0.3 から変更）。実機では「傾き指令→実測水平速度」の実効ゲインが理論値の約0.4倍しかなく、内側（速度）ループの権限を引き上げ外側（位置）ループを遅くすることで、成長する発散振動を解消した。

| パラメータ | 既定値 | 範囲 | 備考 |
|-----------|-------|------|------|
| `position.stick_vel` | 0.4 m/s | 0.05〜2.0 | POS_HOLDスティック再配置（倒して動かし、離して保持）の速度スケール |

### ESKF パラメータ

パラメータ名前空間: `eskf.*`

#### プロセスノイズ

| パラメータ | 値 | 備考 |
|-----------|-----|------|
| `eskf.process.gyro_noise` | 0.009655 | ジャイロ測定ノイズ |
| `eskf.process.accel_noise` | 0.3 | 加速度測定ノイズ |
| `eskf.process.gyro_bias` | 0.000013 | ジャイロバイアスランダムウォーク |
| `eskf.process.accel_bias` | 0.0001 | 加速度バイアスランダムウォーク |
| `eskf.bias.gyro_dev_max` | 0.03 rad/s | ジャイロバイアス偏差クランプ（起動校正値からの許容偏差） |

#### 観測ノイズ

| パラメータ | 値 | 備考 |
|-----------|-----|------|
| `eskf.obs.baro_noise` | 0.1 | 気圧高度 |
| `eskf.obs.tof_noise` | 0.01 | ToF測距 |
| `eskf.obs.mag_noise` | 1.0 | 地磁気 |
| `eskf.obs.flow_noise` | 0.30 | オプティカルフロー |
| `eskf.obs.accel_att_noise` | 1.2 | 加速度ベース姿勢 |
| `eskf.obs.accel_att_lpf` | 30.0 Hz | 加速度ベース姿勢の重力基準ローパス（機体振動除去） |

> **注記:** 上記は複数回のチューニングを経た現在値であり、初期設計時の値（例: accel_noise=0.062885、accel_att_noise=0.06）から大きく変わっている。特に `accel_att_noise` はχ²ゲート過剰棄却の根治を経て 0.06 → 0.8 → 1.2 と改定された。

#### センサ有効/無効

| パラメータ | 既定値 | 備考 |
|-----------|-------|------|
| `eskf.use_tof` | 1（有効） | |
| `eskf.use_flow` | 1（有効） | |
| `eskf.use_baro` | 0（無効） | 鉛直はToF専用（設計方針、気圧は不使用） |
| `eskf.use_mag` | 0（無効） | |

#### ゲート閾値

観測ごとに自由度別のχ²閾値を持つ旧方式から、以下の方式へ再設計されている:

| パラメータ | 値 | 備考 |
|-----------|-----|------|
| `eskf.gate.mahalanobis` | 15.0 | 汎用マハラノビス距離ゲート |
| `eskf.gate.tof_innov` | 0.5 m | ToFイノベーションクランプ |
| `eskf.gate.baro_innov` | 0.5 m | 気圧イノベーションクランプ |
| `eskf.gate.flow_clamp` | 0.3 | オプティカルフローイノベーションクランプ |
| `eskf.gate.flow_squal` | 10 | オプティカルフロー品質（SQUAL）最小閾値 |
| `eskf.att.chi2_gate` | 7.81 | 加速度ベース姿勢更新専用のχ²閾値（自由度3、旧ACCEL_ATT閾値を継承） |
| `eskf.att.k_adaptive` | 10.0 | 加速度ベース姿勢の適応ゲイン |
| `eskf.att.corr_clamp` | 0.05 rad | 加速度ベース姿勢補正クランプ |

#### 加速度補償姿勢推定（POS_HOLD向け、新規）

α-βフロー加速度トラッカーで姿勢推定を補償する機構（旧設計にはない）:

| パラメータ | 既定値 | 備考 |
|-----------|-------|------|
| `eskf.accel_comp.enable` | 1（有効） | |
| `eskf.accel_comp.alpha` | 0.2 | |
| `eskf.accel_comp.beta` | 0.02 | |
| `eskf.accel_comp.max` | 5.0 | 補償クランプ |

### LPF・ノッチフィルタ

現行の `firmware/vehicle` には、旧設計にあった汎用IMU LPF（ACCEL_CUTOFF_HZ / GYRO_CUTOFF_HZ）やジャイロノッチフィルタに相当するコンポーネント・パラメータ（旧 `sf_algo_filter`）は存在しない。ESKF内の加速度ベース姿勢専用ローパス（上記 `eskf.obs.accel_att_lpf`）のみが現存する。

### 安全パラメータ

パラメータ名前空間: `safety.*`

| パラメータ | 値 | 単位 | 備考 |
|-----------|-----|------|------|
| `safety.impact.accel_g` | 3.0 | G | 衝撃検出閾値 |
| `safety.impact.gyro_dps` | 800 | deg/s | 衝撃検出閾値 |
| `safety.comm.timeout_ms` | 500 | ms | 通信途絶（コムロス）タイムアウト |
| `safety.battery.low_v` | 3.4 | V | バッテリー低電圧警告 |
| `safety.battery.usb_v` | 3.3 | V | USB給電判定電圧 |

---

<a id="english"></a>

## 1. Overview

### About This Document

This document provides a comprehensive list of physical constants and parameters used in both the StampFly simulator and firmware.
Simulator parameters are based on measurements and system identification from the actual aircraft, consolidated from various simulator modules.
Firmware (`firmware/vehicle`, formerly `vehicle_new`; promoted to the primary firmware in 2026 after real-hardware POS_HOLD validation, with the earlier layered firmware frozen as `firmware/vehicle_old`) tunable parameters (PID gains, ESKF settings, etc.) are documented with the `table[]` in `firmware/vehicle/components/sf_core/params.cpp` (an explicit table binding name -> variable -> default/min/max/callback) as the Single Source of Truth (SSOT). Fixed constants that never change at runtime (GPIO assignments, task priorities, etc.) live separately in `firmware/vehicle/main/config.hpp` and are out of scope here.

### Target Audience

- Developers using or improving the simulator
- Developers checking or tuning firmware control parameters
- Students and researchers designing control systems
- Anyone verifying the basis of model parameters

### PID Notation Differences

The simulator and firmware use different PID gain notations.

| Item | Simulator | Firmware |
|------|-----------|----------|
| Notation | Kp / Ki / Kd | Kp / Ti / Td |
| Name | Standard form | Time-constant form |

Firmware time-constant form transfer function:

```
C(s) = Kp × (1 + 1/(Ti·s) + Td·s / (η·Td·s + 1))
```

Conversion between forms:

| Conversion | Formula |
|-----------|---------|
| Ki → Ti | Ti = Kp / Ki |
| Kd → Td | Td = Kd / Kp |
| Ti → Ki | Ki = Kp / Ti |
| Td → Kd | Kd = Kp × Td |

η is the incomplete derivative filter coefficient (default 0.125). It limits the high-frequency gain of the derivative term to 1/η.

### Reference Files

#### Simulator

| Module | File Path |
|--------|-----------|
| Vehicle Dynamics | `simulator/core/dynamics.py` |
| Motor & Propeller | `simulator/core/motors.py` |
| Rigid Body Physics | `simulator/core/physics.py` |
| Aerodynamics | `simulator/core/aerodynamics.py` |
| IMU Sensor | `simulator/sensors/imu.py` |
| Barometric Sensor | `simulator/sensors/barometer.py` |
| Motor Mixer | `simulator/control/motor_mixer.py` |

#### Firmware

| Module | File Path |
|--------|-----------|
| Parameter Config (SSOT, tunable values) | `firmware/vehicle/components/sf_core/params.cpp` |
| Fixed constants (GPIO, task priorities, etc.) | `firmware/vehicle/main/config.hpp` |
| Cascade PID (all stages: rate/attitude/altitude/position) | `firmware/vehicle/components/sf_controller_pid/pid_controller.cpp` |
| ESKF | `firmware/vehicle/components/sf_estimator_eskf/include/eskf_estimator.hpp` |
| PID core | `firmware/vehicle/components/sf_controller_pid/include/pid.hpp` |
| Control Allocation (mixer + motor curve) | `firmware/vehicle/components/sf_actuator/actuator.cpp` |

## 2. Vehicle Parameters

### Mass Properties

| Parameter | Symbol | Simulator | Firmware | Unit | Notes |
|-----------|--------|-----------|----------|------|-------|
| Vehicle Mass | m | 0.035 | 0.037 | kg | Including battery |
| Roll Moment of Inertia | Ixx | 9.16×10⁻⁶ | - | kg·m² | |
| Pitch Moment of Inertia | Iyy | 13.3×10⁻⁶ | - | kg·m² | |
| Yaw Moment of Inertia | Izz | 20.4×10⁻⁶ | - | kg·m² | |

> **Note:** The firmware mass of 0.037 kg is used for gravity compensation in altitude control (`sf_controller_pid`, `PidController::kMassG` = 0.037 × 9.80665). The difference from the simulator's 0.035 kg is due to unit variation and battery weight differences.

### Vehicle Geometry

```
               Front
          FL (M4)   FR (M1)
             ╲   ▲   ╱
              ╲  │  ╱
               ╲ │ ╱
                ╲│╱
                 ╳         ← Center
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲
             ╱   │   ╲
          RL (M3)    RR (M2)
                Rear
```

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Motor-to-motor distance (diagonal) | 0.065 | m | 2 × arm length |
| Arm length (center to motor) | 0.0325 | m | √(x² + y²) = 0.023 × √2 |
| Moment arm | 0.023 | m | X/Y coordinate offset (= arm length/√2) |
| Motor height (from CG) | 0.005 | m | |

### Aerodynamic Drag

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Translational drag coefficient | 0.1 | - | F_drag = 0.1 × v² |
| Rotational drag coefficient | 1×10⁻⁵ | - | τ_drag = 1e-5 × ω² |
| Air density | 1.225 | kg/m³ | Standard atmosphere |

## 3. Motor & Propeller Parameters

### Motor Layout and Rotation Direction

| Motor | Position | X (m) | Y (m) | Rotation |
|-------|----------|-------|-------|----------|
| M1 (FR) | Front-Right | +0.023 | +0.023 | CCW |
| M2 (RR) | Rear-Right | -0.023 | +0.023 | CW |
| M3 (RL) | Rear-Left | -0.023 | -0.023 | CCW |
| M4 (FL) | Front-Left | +0.023 | -0.023 | CW |

### Motor Electrical Characteristics

| Parameter | Symbol | Value | Unit | Method |
|-----------|--------|-------|------|--------|
| Winding Resistance | Rm | 0.34 | Ω | LCR meter |
| Winding Inductance | Lm | 1.0×10⁻⁶ | H | LCR meter |
| Rotor Inertia | Jmp | 2.01×10⁻⁸ | kg·m² | Estimated from geometry |

### Speed-Voltage Relationship

The relationship between voltage V and angular velocity ω:

```
V = Am × ω² + Bm × ω + Cm
```

| Parameter | Symbol | Value | Unit | Notes |
|-----------|--------|-------|------|-------|
| Quadratic coefficient | Am | 5.39×10⁻⁸ | V/(rad/s)² | Experimental |
| Linear coefficient | Bm | 6.33×10⁻⁴ | V/(rad/s) | Experimental |
| Constant term | Cm | 1.53×10⁻² | V | Experimental |

### Thrust & Torque Characteristics

| Parameter | Symbol | Value | Unit | Notes |
|-----------|--------|-------|------|-------|
| Thrust coefficient | Ct | 1.00×10⁻⁸ | N/(rad/s)² | T = Ct × ω² |
| Torque coefficient | Cq | 9.71×10⁻¹¹ | N·m/(rad/s)² | Q = Cq × ω² |
| Torque/Thrust ratio | κ | 6.12×10⁻³ | m | κ = Cq/Ct. Measured 2026-07-15 (coast-down Cq=4.10e-11 / thrust-stand Ct=6.7e-9). Applied to the firmware B⁻¹ mixer (actuator.cpp) and the SIL plant (plant.hpp) on 2026-07-17. The Ct/Cq rows above keep the legacy values as a description of the firmware's calibrated motor-curve chain |

### Derived Parameters

| Parameter | Symbol | Formula | Value |
|-----------|--------|---------|-------|
| Back-EMF constant | Km | Cq×Rm/Am | 6.12×10⁻⁴ V/(rad/s) |
| Viscous friction | Dm | (Bm - Cq×Rm/Am)×(Cq/Am) | computed |
| Static friction | Qf | Cm×Cq/Am | computed |

### Hover Conditions

Vehicle weight 0.035 kg × 9.81 m/s² = 0.343 N shared by 4 motors:

| Condition | Value | Unit |
|-----------|-------|------|
| Thrust per motor | 0.0858 | N |
| Hover angular velocity | ~2930 | rad/s |
| Hover voltage | ~2.1 | V |

## 4. Sensor Parameters

### IMU (BMI270)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Sample rate | 400 | Hz | |
| Gyro full scale | 2000 | deg/s | Configurable: 125/250/500/1000/2000 |
| Accel full scale | 8 | g | Configurable: 2/4/8/16 |
| Gyro resolution | 16 | bit | |
| Accel resolution | 16 | bit | |
| Gyro noise density | 0.007 | deg/s/√Hz | From datasheet |
| Accel noise density | 120 | µg/√Hz | From datasheet |
| Gyro bias instability | 0.1 | deg/s | |
| Accel bias instability | 0.002 | g | |

### Barometer (BMP280)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Pressure resolution | 0.16 | Pa | X16 oversampling |
| Temperature resolution | 0.01 | °C | |
| Pressure noise | 1.3 | Pa RMS | X16 oversampling |
| Temperature noise | 0.005 | °C | |
| Sea level pressure | 101325 | Pa | Standard atmosphere |

### Physical Constants

| Constant | Symbol | Simulator | Firmware | Unit | Notes |
|----------|--------|-----------|----------|------|-------|
| Gravity | g | 9.80665 | 9.81 | m/s² | Simulator uses precise value, FW uses simplified |
| Molar mass of air | M | 0.0289644 | - | kg/mol | For barometric altitude |
| Gas constant | R | 8.31447 | - | J/(mol·K) | For barometric altitude |
| Temperature lapse rate | L | 0.0065 | - | K/m | For barometric altitude |

## 5. Simulator Control Parameters

### Motor Mixer

```
m1 = throttle + scale × (-roll + pitch + yaw)   # FR (M1)
m2 = throttle + scale × (-roll - pitch - yaw)   # RR (M2)
m3 = throttle + scale × (+roll - pitch + yaw)   # RL (M3)
m4 = throttle + scale × (+roll + pitch - yaw)   # FL (M4)
```

| Parameter | Value | Notes |
|-----------|-------|-------|
| Output scale | 0.0676 | 0.25/3.7 |
| Idle output | 0.05 | 5% when armed at zero throttle |
| Max thrust/motor | 0.15 N | Estimated |

### PID Gains (Simulator Default)

> **Note:** The simulator uses standard form (Kp/Ki/Kd). See "PID Notation Differences" for conversion to firmware time-constant form.

#### Attitude Control (angle → rate)

| Axis | Kp | Ki | Kd |
|------|-----|-----|-----|
| Roll | 5.0 | 1.0 | 0.0 |
| Pitch | 5.0 | 1.0 | 0.0 |
| Yaw | 0.5 | 0.01 | 0.0 |

#### Rate Control (rate → voltage)

| Axis | Kp | Ki | Kd |
|------|-----|-----|-----|
| Roll rate | 0.2 | 10.0 | 0.002 |
| Pitch rate | 0.2 | 10.0 | 0.002 |
| Yaw rate | 1.0 | 2.0 | 0.001 |

#### Altitude Control

| Kp | Ki | Kd |
|-----|-----|-----|
| 10.0 | 5.0 | 5.0 |

### Control Timing

| Parameter | Value | Unit |
|-----------|-------|------|
| Simulation step | 0.001 | s (1 kHz) |
| Control period | 0.01 | s (100 Hz) |
| Battery voltage (nominal) | 3.7 | V |

## 6. Disturbance Model

The following disturbances can be added in simulation:

| Parameter | Default | Unit | Notes |
|-----------|---------|------|-------|
| Moment disturbance (L) | 4.4×10⁻⁶ | N·m | Normal dist. σ |
| Moment disturbance (M) | 4.4×10⁻⁶ | N·m | Normal dist. σ |
| Moment disturbance (N) | 4.0×10⁻⁶ | N·m | Normal dist. σ |
| Force disturbance (X, Y, Z) | 1×10⁻⁶ | N | Normal dist. σ |

## 7. Firmware Control Parameters

Firmware tunable parameters are centrally managed in the `table[]` (SSOT) inside `firmware/vehicle/components/sf_core/params.cpp`. Parameter names use dot notation `<category>.<subcategory>.<item>` (e.g. `rate.roll.kp`) and can be read/written via the `param get/set/save` CLI. Fixed constants (GPIO assignments, task priorities, etc.) live separately in `firmware/vehicle/main/config.hpp` and are not covered here.
PID uses time-constant form (Kp/Ti/Td) — see "PID Notation Differences" for details. **Physical units (torque [Nm] / force [N] output) are the only mode — there is no compile-time switch (like the earlier `USE_PHYSICAL_UNITS`) and no legacy voltage-output mode.**

### Rate Control

Parameter namespace: `rate.*` (the `sf_controller_pid` cascade PID; output is the physical torque [Nm] fed to `sf_actuator`'s B⁻¹ mixer)

| Axis | Kp [Nm/(rad/s)] | Ti [s] | Td [s] |
|------|-----------------|--------|--------|
| Roll | 9.76×10⁻⁴ | 0.7 | 0.01 |
| Pitch | 1.43×10⁻³ | 0.7 | 0.025 |
| Yaw | 1.90×10⁻³ | 0.8 | 0.01 |

Incomplete derivative filter coefficient: η = 0.125 (a fixed constant in `sf_controller_pid/include/pid.hpp`, not a tunable parameter)

> **Note:** The table above reflects values adopted after real-flight verification on 2026-06-27 (a gain redesign addressing hardware oscillation; only Kp changed, Ti/Td unchanged).

### Attitude Control

Parameter namespace: `attitude.*`

| Axis | Kp [(rad/s)/rad] | Ti [s] | Td [s] |
|------|-----------------|--------|--------|
| Roll | 5.0 | 2.0 | 0.04 |
| Pitch | 5.0 | 2.0 | 0.04 |

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Attitude-loop output limit | 3.0 | rad/s | Rate command upper limit (`max_att_rate_sp_`, fixed constant) |
| η | 0.125 | - | Incomplete derivative filter coefficient (fixed constant) |

#### Attitude Trim / Heading Hold (new)

| Parameter | Default | Range | Notes |
|-----------|---------|-------|-------|
| `attitude.roll.trim` | 0.0 rad | ±0.1 | Equilibrium tilt (flight-identified via `sf trim analyze`; also onboard auto-learned) |
| `attitude.pitch.trim` | 0.0 rad | ±0.1 | Same |
| `attitude.trim.learn` | 1 (on) | 0/1 | Enable/disable onboard automatic trim learning |
| `attitude.yawhold.kp` | 3.0 | 0-10 | Heading-hold P gain (0 disables) |
| `attitude.yawhold.rate_max` | 2.0 rad/s | 0.1-5 | Heading correction rate limit |

### Altitude Control

Parameter namespace: `altitude.*` (PI only — no Td)

| Parameter | Value | Unit |
|-----------|-------|------|
| Vehicle mass (used for `kMassG`) | 0.037 | kg |
| Gravity | 9.80665 | m/s² |

#### Altitude PID (position → velocity)

| Kp | Ti [s] | OutputMax [m/s] | Notes |
|-----|--------|-----------------|-------|
| 0.45 | 7.0 | 0.5 (`altitude.climb_rate`) | Descent rate limit is a separate parameter, `altitude.descent_rate` (default 0.5 m/s) |

#### Velocity PID (velocity → thrust correction)

| Kp | Ti [s] | OutputMax [N] |
|-----|--------|---------------|
| 0.1 | 2.5 | 0.15 |

#### Hover-Thrust Feedforward (new)

| Parameter | Default | Range | Notes |
|-----------|---------|-------|-------|
| `hover.thrust_corr` | 1.12 | 0.5-2.0 | Correction factor for the motor curve over-predicting real thrust by ~12% (flight-measured) |
| `hover.thrust.learn` | 1 (on) | 0/1 | Onboard automatic hover-thrust learning (persisted to NVS on landing) |

### Position Control

Parameter namespace: `position.*` (PI only — no Td)

#### Position PID (position → velocity)

| Kp | Ti [s] | OutputMax [m/s] |
|-----|--------|-----------------|
| 0.4 | 5.0 | 1.0 |

#### Velocity PID (velocity → horizontal acceleration)

| Kp | Ti [s] | OutputMax [m/s²] | Notes |
|-----|--------|-------------------|-------|
| 3.0 | 2.0 | ≈1.71 (= g × 10° tilt limit) | Output is horizontal acceleration; divided by g to get tilt angle (clamped to the 10° POS_HOLD tilt limit) |

> **Note:** These gains were re-tuned after the first real POS_HOLD flight on 2026-06-22 (from the initial pos.kp=1.0/vel.kp=0.3). On real hardware, the tilt-command-to-measured-velocity gain was only ~0.4x the theoretical value; raising the inner (velocity) loop's authority and slowing the outer (position) loop resolved a growing divergent oscillation.

| Parameter | Default | Range | Notes |
|-----------|---------|-------|-------|
| `position.stick_vel` | 0.4 m/s | 0.05-2.0 | Speed scale for POS_HOLD stick repositioning (deflect to move, release to hold) |

### ESKF Parameters

Parameter namespace: `eskf.*`

#### Process Noise

| Parameter | Value | Notes |
|-----------|-------|-------|
| `eskf.process.gyro_noise` | 0.009655 | Gyro measurement noise |
| `eskf.process.accel_noise` | 0.3 | Accelerometer measurement noise |
| `eskf.process.gyro_bias` | 0.000013 | Gyro bias random walk |
| `eskf.process.accel_bias` | 0.0001 | Accelerometer bias random walk |
| `eskf.bias.gyro_dev_max` | 0.03 rad/s | Gyro bias deviation clamp (allowed deviation from the boot-calibration nominal) |

#### Observation Noise

| Parameter | Value | Notes |
|-----------|-------|-------|
| `eskf.obs.baro_noise` | 0.1 | Barometric altitude |
| `eskf.obs.tof_noise` | 0.01 | ToF ranging |
| `eskf.obs.mag_noise` | 1.0 | Magnetometer |
| `eskf.obs.flow_noise` | 0.30 | Optical flow |
| `eskf.obs.accel_att_noise` | 1.2 | Accelerometer-based attitude |
| `eskf.obs.accel_att_lpf` | 30.0 Hz | Gravity-reference lowpass for accel-based attitude (rejects airframe vibration) |

> **Note:** These are current values after several rounds of tuning, and differ substantially from the original design values (e.g. accel_noise=0.062885, accel_att_noise=0.06). In particular, `accel_att_noise` was revised 0.06 → 0.8 → 1.2 while fixing chi-squared gate over-rejection.

#### Sensor Enable Flags

| Parameter | Default | Notes |
|-----------|---------|-------|
| `eskf.use_tof` | 1 (on) | |
| `eskf.use_flow` | 1 (on) | |
| `eskf.use_baro` | 0 (off) | Vertical estimation is ToF-only by design; barometer unused |
| `eskf.use_mag` | 0 (off) | |

#### Gate Thresholds

Redesigned from the earlier scheme of one chi-squared threshold per observation (by degrees of freedom) to:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `eskf.gate.mahalanobis` | 15.0 | General-purpose Mahalanobis-distance gate |
| `eskf.gate.tof_innov` | 0.5 m | ToF innovation clamp |
| `eskf.gate.baro_innov` | 0.5 m | Barometer innovation clamp |
| `eskf.gate.flow_clamp` | 0.3 | Optical-flow innovation clamp |
| `eskf.gate.flow_squal` | 10 | Optical-flow quality (SQUAL) minimum threshold |
| `eskf.att.chi2_gate` | 7.81 | Chi-squared threshold specific to the accel-based attitude update (3 DoF; carried over from the old ACCEL_ATT threshold) |
| `eskf.att.k_adaptive` | 10.0 | Adaptive gain for accel-based attitude |
| `eskf.att.corr_clamp` | 0.05 rad | Accel-based attitude correction clamp |

#### Acceleration-Compensated Attitude (for POS_HOLD; new)

An alpha-beta flow-acceleration tracker compensates the attitude estimate (not present in the original design):

| Parameter | Default | Notes |
|-----------|---------|-------|
| `eskf.accel_comp.enable` | 1 (on) | |
| `eskf.accel_comp.alpha` | 0.2 | |
| `eskf.accel_comp.beta` | 0.02 | |
| `eskf.accel_comp.max` | 5.0 | Compensation clamp |

### LPF / Notch Filter

The current `firmware/vehicle` has no component or parameters equivalent to the original design's general-purpose IMU LPF (ACCEL_CUTOFF_HZ / GYRO_CUTOFF_HZ) or gyro notch filter (the old `sf_algo_filter`). Only the accel-based-attitude-specific lowpass inside the ESKF (`eskf.obs.accel_att_lpf`, above) remains.

### Safety Parameters

Parameter namespace: `safety.*`

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| `safety.impact.accel_g` | 3.0 | G | Impact detection threshold |
| `safety.impact.gyro_dps` | 800 | deg/s | Impact detection threshold |
| `safety.comm.timeout_ms` | 500 | ms | Communication-loss timeout |
| `safety.battery.low_v` | 3.4 | V | Battery low-voltage warning |
| `safety.battery.usb_v` | 3.3 | V | USB-power detection voltage |
