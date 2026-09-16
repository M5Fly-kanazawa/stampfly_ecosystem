# Flip 平面モデル設計検討（Flip Planar Design Study）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### この検討について

宙返り（Flip）マニューバの制御案（`docs/plans/flip-maneuver-plan.md` §3・§7）を、実装前に数値で裏付けるための
平面 2 自由度モデル（鉛直位置 z とロール角 φ）の検討。2026-09-16 の Phase 0 で作成した。
利用者向けのツールではなく **設計根拠を残すための検討スクリプト**である（`sf` コマンド化は Phase 1 で判断する）。

### 対象読者

Flip の制御パラメータ（回転速度・推力配分・予備上昇時間）を見直す人。

## 2. 内容

| ファイル | 内容 |
|---------|------|
| `flip_sim.py` | v1: 物理モデル（剛体 2 自由度 + モータ電気機械 ODE、SILS `plant.cpp` と同式）、ミキサー飽和方針 A/B、角度スケジュール型の制御、掃引の枠組み |
| `flip_sim_v2.py` | v2: 減速開始角 φ_brake を計測角速度から毎周期導出、推力 3 段階（T_boost / T_spin_hi / T_lo）、飽和方針 C（モータごとのクランプ = 現行 `actuator.cpp`）、IMU 加速度ノルム |
| `flip_sim_v3.py` | v3: 計測ピーク角速度 ≤ 1700 °/s の制約の下での推奨組合せ確定（24 ケース）と T_boost = 電気的上限 × 0.9 の比較 |
| `flip_sweep_results*.csv` / `flip_sweep_summary*.txt` | 各版の掃引結果と要約 |

物理パラメータは `control/models/stampfly_physical.yaml`（`measured_2026_07`）と
`firmware/vehicle/components/sf_controller_pid/include/pid_controller.hpp`（`max_roll_pitch_torque_`、`max_thrust_`）から読み取った値。

### 実行

```bash
cd control/design/flip_planar_study
python3 flip_sim_v3.py   # v2・v1 を import する。numpy が必要
```

### 主な結論

- 高度落ち込みに最も効くのは予備上昇時間 t_boost と回転速度 ω_flip。加速・減速窓の集合推力を最大にすると差動トルクの余裕がゼロになり回転しない（推力 3 段階の根拠）
- レート指令はランプ（300 rad/s²）がステップより良い。ステップは 10〜11 % オーバーシュートし、ω_flip 1800 °/s ではジャイロ計測範囲に余裕が無くなる
- 推奨組合せ（ランプ 300 rad/s²、T_spin_hi 0.50 N、T_lo 0.03 N、t_boost 0.15 s、T_boost = 電気的上限 × 0.9）では開始高度を下回らず、最大上昇 21〜30 cm
- 詳細と限界（空力抗力なし、ジャイロ理想、モータの低 duty 非線形なし）は計画文書 §5.3

---

<a id="english"></a>

## 1. Overview

### About This Study

A planar two-degree-of-freedom model (vertical position z and roll angle φ) used to back the flip maneuver control
proposal (`docs/plans/flip-maneuver-plan.md` §3 and §7) with numbers before implementation. Written in Phase 0 on
2026-09-16. It is a **design-rationale study**, not a user-facing tool (whether to wrap it as an `sf` command is
decided in Phase 1).

### Target Audience

Anyone revisiting the flip control parameters (rotation rate, thrust schedule, boost duration).

## 2. Contents

| File | Content |
|------|---------|
| `flip_sim.py` | v1: physics (rigid 2-DOF + motor electromechanical ODE, same equations as SILS `plant.cpp`), mixer saturation policies A/B, angle-scheduled control, sweep framework |
| `flip_sim_v2.py` | v2: brake angle φ_brake derived every cycle from the measured rate, three thrust levels (T_boost / T_spin_hi / T_lo), policy C (per-motor clamp = current `actuator.cpp`), IMU acceleration norm |
| `flip_sim_v3.py` | v3: recommended combination under the measured-peak-rate ≤ 1700 deg/s constraint (24 cases) and the T_boost = 0.9 × electrical ceiling variant |
| `flip_sweep_results*.csv` / `flip_sweep_summary*.txt` | Sweep results and summaries per version |

Physical parameters are read from `control/models/stampfly_physical.yaml` (`measured_2026_07`) and
`firmware/vehicle/components/sf_controller_pid/include/pid_controller.hpp` (`max_roll_pitch_torque_`, `max_thrust_`).

### Running

```bash
cd control/design/flip_planar_study
python3 flip_sim_v3.py   # imports v2 and v1; requires numpy
```

### Main Conclusions

- Altitude loss is dominated by the boost duration t_boost and the rotation rate ω_flip. Commanding maximum collective in the acceleration/braking windows leaves no differential-torque headroom and the craft does not rotate (why three thrust levels are used)
- A ramped rate command (300 rad/s²) beats a step; the step overshoots by 10-11 %, and at 1800 deg/s the gyro range has no margin left
- With the recommended set (ramp 300 rad/s², T_spin_hi 0.50 N, T_lo 0.03 N, t_boost 0.15 s, T_boost = 0.9 × electrical ceiling) the craft never goes below its start altitude; maximum rise 21-30 cm
- Details and limits (no aerodynamic drag, ideal gyro, no low-duty motor nonlinearity) are in plan §5.3
