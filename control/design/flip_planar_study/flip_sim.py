#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StampFly roll-flip maneuver -- numerical verification & parameter sweep.
StampFly ロール宙返り（フリップ）制御案 -- 数値検証とパラメータ掃引。

This is a STANDALONE analysis script (NOT part of the stampfly_ecosystem repo
build). It never modifies any file under the repository; all outputs go next
to this script.
これは stampfly_ecosystem リポジトリのビルドに含まれない単独の解析スクリプトで
ある（リポジトリ側のファイルは一切変更しない）。出力は全て本スクリプトと同じ
ディレクトリに置く。

Planar (y-z plane) 2-DOF rigid body model (z: altitude, phi: cumulative roll
angle, y: optional lateral position) driven by an X-frame quad's roll
differential. Two representative motor electromechanical ODEs (one for the
"right" pair, one for the "left" pair -- the two motors within a pair are
commanded identically for a pure roll maneuver, so they are numerically
redundant and are collapsed to one state each).
平面（y-z面）2自由度剛体モデル（z: 高度、phi: 累積ロール角、y: 横位置は任意）を
X配置クアッドのロール差動で駆動する。モータの電気機械ODEは「右ペア」「左ペア」の
代表2状態のみ持つ（純ロール運動ではペア内の2モータへの指令が常に同一になるため、
数値的に冗長な状態を持たず集約できる）。

Physical parameters are read from (and cross-checked against):
物理パラメータの出所（突き合わせ確認済み）:
  - control/models/stampfly_physical.yaml
      constants.mass / Ixx / Iyy / arm_offset
      calibration_sets.measured_2026_07 (Ct/Cq/Jmp/Dm/Qf/Rm/Km -- adopted)
  - simulator/sils/plant/generated_params.hpp (machine-generated from the
    yaml above; numeric cross-check)
  - simulator/sils/plant/plant.cpp / plant.hpp
      omegaDot()/steadyStateOmega()/dutyToThrust(): the electromechanical
      motor ODE this script reproduces exactly, and thrust_efficiency=0.7133.
  - firmware/vehicle/components/sf_controller_pid/include/pid_controller.hpp
      max_roll_pitch_torque_ = 5.2e-3 N*m (rate-loop output limit)
      max_thrust_ = 0.672 N = 4 x 0.168 N/motor (current firmware collective
      ceiling, T/W ~ 1.85)
These are read-only references; nothing in the repository is written by this
script (see the CLAUDE.md instruction and the task's own instruction).
これらは読み取り専用の参照であり、本スクリプトはリポジトリに一切書き込まない。

Usage / 実行方法:
    python3 flip_sim.py
Outputs / 出力:
    flip_baseline_timeseries.png   -- baseline case time-series plot
    flip_sweep_results.csv          -- full sweep result table
    flip_sweep_summary.txt          -- printed summary (also on stdout)
"""

import os
import csv
import math
from dataclasses import dataclass, replace, asdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# =====================================================================
# 1. Physical parameters (StampFly, SSOT: control/models/stampfly_physical.yaml)
#    物理パラメータ（SSOT: 上記 yaml）
# =====================================================================
MASS = 0.037            # kg (vehicle mass)
IXX = 9.16e-6            # kg*m^2 (roll moment of inertia)
IYY = 13.3e-6            # kg*m^2 (pitch moment of inertia; used for the "what if
                          # this were done about the pitch axis" inertia sweep)
ARM_D = 0.023            # m (roll moment arm, ARM_OFFSET)
G = 9.81                 # m/s^2

# Motor/propeller electromechanical ODE coefficients (measured_2026_07, adopted).
# モータ/プロペラ電気機械ODE係数（measured_2026_07、採用値）。
CT = 1.0e-8               # N/(rad/s)^2      T = Ct * omega^2
CQ = 4.10e-11             # N*m/(rad/s)^2    (aero-drag term in the ODE)
JMP = 1.375e-8            # kg*m^2           rotor inertia
DM = 0.0                  # N*m*s/rad        viscous damping (0, per 2026-07-26 refit)
QF = 9.507e-6             # N*m              Coulomb friction torque
RM = 0.593                # Ohm              winding resistance
KM = 5.682e-4             # V/(rad/s)        back-EMF constant
ETA = 0.7133              # thrust_efficiency (plant.hpp Config::thrust_efficiency)

HOVER_THRUST_TOTAL = MASS * G                  # N,  = 0.3630 N
HOVER_THRUST_PER_MOTOR = HOVER_THRUST_TOTAL / 4.0

# --- Two thrust-ceiling conditions (added per coordinator follow-up) ---
# 推力上限の2条件（コーディネータからの追加指示）。
#   "firmware": current firmware's actual ceiling (pid_controller.hpp
#                max_thrust_ = 0.672 N = 4 x 0.168 N/motor, NO margin retained
#                -- T_boost IS the ceiling itself).
#   "physical": the propeller's stated mechanical limit (0.2 N/motor, 0.8 N
#                total), with the original design's 10% margin retained for
#                T_boost (T_boost = 0.9 x 4 x 0.2 = 0.72 N).
THRUST_MODES = {
    "firmware": dict(motor_t_max=0.168, t_boost=0.672),
    "physical": dict(motor_t_max=0.200, t_boost=0.720),
}

# --- Two rate-loop torque-output-limit conditions ---
# レートループのトルク出力上限の2条件。
#   "firmware": pid_controller.hpp max_roll_pitch_torque_ = 5.2e-3 N*m.
#   "unlimited": effectively no software cap; only the mixer's geometric
#                (thrust-based) saturation can limit torque.
TAU_C_LIMITS = {
    "firmware": 5.2e-3,
    "unlimited": 1.0,   # N*m, never reached in this problem -> geometric-only
}

MOTOR_T_MIN = 0.0

# =====================================================================
# 2. Motor electromechanical ODE (verbatim port of plant.cpp's omegaDot /
#    steadyStateOmega, see file header for source).
#    モータ電気機械ODE（plant.cpp の omegaDot/steadyStateOmega を移植）。
# =====================================================================

def motor_omega_dot(omega, v_motor):
    """dw/dt = [-(Dm + Km^2/Rm)*w - Cq*w^2 - Qf + Km*V/Rm] / Jmp"""
    drag = (DM + KM * KM / RM) * omega + CQ * omega ** 2 + QF
    drive = KM * v_motor / RM
    return (drive - drag) / JMP


def motor_steady_state_omega(v_motor):
    """Positive root of Cq*w^2 + (Dm+Km^2/Rm)*w + (Qf - Km*V/Rm) = 0."""
    a = CQ
    b = DM + KM * KM / RM
    c = QF - KM * v_motor / RM
    disc = np.maximum(b * b - 4.0 * a * c, 0.0)
    omega = (-b + np.sqrt(disc)) / (2.0 * a)
    return np.maximum(omega, 0.0)


def thrust_from_omega(omega):
    """T = eta * Ct * omega^2 (per motor)."""
    return ETA * CT * omega ** 2


def voltage_for_thrust(thrust):
    """Inverse map: desired steady-state per-motor thrust [N] -> required
    terminal voltage [V] (before v_supply clipping).
    推力指令[N] -> 必要な端子電圧[V]（電源電圧によるクリップ前）への逆写像。"""
    thrust = np.maximum(thrust, 0.0)
    omega = np.sqrt(thrust / (ETA * CT))
    drag = (DM + KM * KM / RM) * omega + CQ * omega ** 2 + QF
    return np.maximum(drag * RM / KM, 0.0)


# =====================================================================
# 3. Mixer: collective thrust Tc [N] + roll torque tau_c [N*m] -> per-motor
#    thrust commands, saturated by one of two policies.
#    ミキサー: 集合推力Tcとロールトルクtau_cを配分し、2方針のいずれかで飽和。
# =====================================================================

def mix_motors(Tc, tau_c, policy_is_B, motor_t_max, d=ARM_D):
    """
    4-motor X-frame roll mixer, collapsed to the 2 independent "right"/"left"
    pair states (right = 2 motors sharing one command, left = 2 motors
    sharing the other). Returns PER-MOTOR thrust (one motor's worth, not the
    pair total).
    4モータX配置のロールミキサーを、独立な「右」「左」ペア状態2つに集約したもの
    （右2モータ・左2モータはロール運動では常に同一指令）。戻り値は「1モータ分」
    の推力（ペア合計ではない）。

    policy A (thrust priority / 集合推力優先): keep Tc, scale down tau_c.
    policy B (torque priority / トルク優先):    keep tau_c, scale down Tc.
    """
    x = tau_c / (4.0 * d)      # half-differential term [N]
    Tc4 = Tc / 4.0
    Tmax = motor_t_max
    Tmin = MOTOR_T_MIN

    right_raw = Tc4 + x
    left_raw = Tc4 - x
    sat_raw = (right_raw > Tmax + 1e-12) | (right_raw < Tmin - 1e-12) | \
              (left_raw > Tmax + 1e-12) | (left_raw < Tmin - 1e-12)

    # --- Policy A: |s*x| <= min(Tmax-Tc4, Tc4-Tmin), s in [0,1] ---
    boundA = np.clip(np.minimum(Tmax - Tc4, Tc4 - Tmin), 0.0, None)
    absx = np.abs(x)
    sA = np.where(absx < 1e-12, 1.0, np.clip(boundA / np.maximum(absx, 1e-12), 0.0, 1.0))
    rightA = Tc4 + sA * x
    leftA = Tc4 - sA * x

    # --- Policy B: t in [0,1] scaling only the collective term ---
    eps = 1e-12
    safeTc4 = np.where(Tc4 > eps, Tc4, 1.0)
    t_upper = np.minimum((Tmax - x) / safeTc4, (Tmax + x) / safeTc4)
    tB = np.clip(t_upper, 0.0, 1.0)
    tB = np.where(Tc4 > eps, tB, 0.0)
    rightB = tB * Tc4 + x
    leftB = tB * Tc4 - x

    right = np.where(policy_is_B, rightB, rightA)
    left = np.where(policy_is_B, leftB, leftA)

    # Final safety clip (handles the physically-impossible corner cases,
    # e.g. pure differential demanded at zero collective).
    # 最終安全クリップ（無収束端点、例: 集合推力ゼロでの純差動要求）。
    right_c = np.clip(right, Tmin, Tmax)
    left_c = np.clip(left, Tmin, Tmax)

    Tc_ach = 2.0 * (right_c + left_c)
    tau_ach = 2.0 * d * (right_c - left_c)
    return right_c, left_c, sat_raw, Tc_ach, tau_ach


# =====================================================================
# 4. Rigid-body + motor dynamics (RK4 integration).
#    剛体+モータ動力学（RK4積分）。
# =====================================================================

def derivatives(state, Vr, Vl, I_roll, c_damp):
    z, vz, y, vy, phi, omega_body, wr, wl = state
    Tr = thrust_from_omega(wr)     # per-motor thrust, right pair
    Tl = thrust_from_omega(wl)     # per-motor thrust, left pair
    T_total = 2.0 * Tr + 2.0 * Tl
    tau_roll = 2.0 * ARM_D * (Tr - Tl)

    dz = vz
    dvz = (T_total / MASS) * np.cos(phi) - G
    dy = vy
    dvy = (T_total / MASS) * np.sin(phi)
    dphi = omega_body
    domega = (tau_roll - c_damp * omega_body) / I_roll
    dwr = motor_omega_dot(wr, Vr)
    dwl = motor_omega_dot(wl, Vl)
    return (dz, dvz, dy, dvy, dphi, domega, dwr, dwl)


def rk4_step(state, dt, Vr, Vl, I_roll, c_damp):
    k1 = derivatives(state, Vr, Vl, I_roll, c_damp)
    s2 = tuple(s + 0.5 * dt * k for s, k in zip(state, k1))
    k2 = derivatives(s2, Vr, Vl, I_roll, c_damp)
    s3 = tuple(s + 0.5 * dt * k for s, k in zip(state, k2))
    k3 = derivatives(s3, Vr, Vl, I_roll, c_damp)
    s4 = tuple(s + dt * k for s, k in zip(state, k3))
    k4 = derivatives(s4, Vr, Vl, I_roll, c_damp)
    return tuple(s + (dt / 6.0) * (a + 2 * b + 2 * c + d)
                 for s, a, b, c, d in zip(state, k1, k2, k3, k4))


# =====================================================================
# 5. Control design constants (rate loop / attitude loop / phase logic).
#    制御設計定数（レートループ／姿勢ループ／フェーズロジック）。
# =====================================================================
DT_CTRL = 1.0 / 400.0     # s, 400 Hz control loop
DT_PHYS = 1.0 / 4000.0    # s, 4 kHz RK4 physics integration
N_SUB = round(DT_CTRL / DT_PHYS)   # 10 physics substeps per control step
T_MAX = 1.5                # s, failure cutoff

PHASE_BOOST, PHASE_SPIN, PHASE_BRAKE_RATE, PHASE_BRAKE_ANGLE, PHASE_RECOVER, \
    PHASE_DONE, PHASE_FAILED = range(7)


def deg2rad(x):
    return np.asarray(x) * math.pi / 180.0


def rad2deg(x):
    return np.asarray(x) * 180.0 / math.pi


# Rate-loop P gain: sized so a 500 deg/s error saturates the FIRMWARE torque
# limit (5.2e-3 N*m). This gain itself does not change across the
# tau_c_limit_mode sweep -- only the output CLIP does (that is the point of
# the sweep: same tuning, cap on/off).
# レートループPゲイン: 誤差500°/sでファーム側トルク上限(5.2e-3 N*m)に達する
# ように設計。tau_c_limit_mode の掃引はこのゲインではなく出力クリップのみを
# 変える（同じチューニングでソフト制限の有無を比較するのが狙い）。
KP_RATE = TAU_C_LIMITS["firmware"] / deg2rad(500.0)   # N*m / (rad/s)

T_LAG = 0.016                  # s, assumed motor/ESC lag
ALPHA_BRAKE_FRAC = 0.6         # fraction of the achievable ang. accel. used for braking margin
PHI_ERR_ENTER_RECOVER_DEG = 20.0
OMEGA_ENTER_RECOVER_DPS = 200.0
PHI_SAT_DESIGN_DEG = 30.0      # design choice: attitude-loop P gain saturates at this error

# Method (ii) "rate-brake" exit thresholds (fixed, from coordinator spec).
# 方式(ii)「レートブレーキ」の抜け条件（コーディネータ指定の固定値）。
BRAKE_RATE_EXIT_PHI_LOOSE_DEG = 290.0
BRAKE_RATE_EXIT_OMEGA_LOOSE_DPS = 300.0
BRAKE_RATE_EXIT_PHI_HARD_DEG = 350.0


def compute_delta_phi_brake_deg(omega_flip_dps, I_roll, tau_max_effective):
    """Delta_phi_brake = omega^2/(2*alpha_brake) + omega*t_lag,
    alpha_brake = 0.6 * tau_max_effective / I_roll.
    Used both as method(i)'s "phi_handoff = 360 - Delta_phi_brake" (auto) and
    as method(ii)'s brake-trigger angle "phi_brake = 360 - Delta_phi_brake"
    (identical formula; coordinator confirmed these coincide)."""
    omega_rad = deg2rad(omega_flip_dps)
    alpha_achievable = tau_max_effective / I_roll
    alpha_brake = ALPHA_BRAKE_FRAC * alpha_achievable
    delta_rad = (omega_rad ** 2) / (2.0 * alpha_brake) + omega_rad * T_LAG
    return rad2deg(delta_rad)


# =====================================================================
# 6. Case definition
# =====================================================================

@dataclass
class Case:
    label: str = "baseline"
    omega_flip_dps: float = 1500.0
    T_idle: float = 0.06
    phi_a_deg: float = 60.0
    phi_b_deg: float = 300.0
    brake_method: str = "rate"      # "rate" (ii, primary) | "angle" (i)
    handoff_mode: str = "auto"      # only used when brake_method=="angle": "auto"|"270"|"300"|"330"
    policy: str = "B"               # "A" (thrust priority) | "B" (torque priority)
    v_supply: float = 3.7
    I_roll: float = IXX
    t_boost: float = 0.100
    omega_att_max_dps: float = 600.0
    c_damp: float = 0.0
    thrust_limit_mode: str = "firmware"   # "firmware" | "physical"
    tau_c_limit_mode: str = "firmware"    # "firmware" | "unlimited"
    spin_cmd_mode: str = "step"           # "step" (omega_cmd jumps to omega_flip) | "ramp" (linear ramp at alpha_cmd)
    alpha_cmd_radss: float = 300.0        # rad/s^2, only used when spin_cmd_mode=="ramp"


BASE = Case(label="baseline")


# =====================================================================
# 7. Vectorized batch simulator: runs N cases simultaneously (numpy arrays
#    over the case axis), so sweeps of O(100) cases finish in seconds.
#    ベクトル化バッチシミュレータ: N ケースを numpy 配列で同時に進める。
# =====================================================================

def run_batch(cases, dt_ctrl=DT_CTRL, dt_phys=DT_PHYS, t_max=T_MAX, store_traj=False):
    N = len(cases)

    omega_flip = np.array([c.omega_flip_dps for c in cases], dtype=float)
    T_idle = np.array([c.T_idle for c in cases], dtype=float)
    phi_a = np.array([c.phi_a_deg for c in cases], dtype=float)
    phi_b = np.array([c.phi_b_deg for c in cases], dtype=float)
    policy_is_B = np.array([c.policy == "B" for c in cases], dtype=bool)
    v_supply = np.array([c.v_supply for c in cases], dtype=float)
    I_roll = np.array([c.I_roll for c in cases], dtype=float)
    t_boost = np.array([c.t_boost for c in cases], dtype=float)
    omega_att_max = np.array([c.omega_att_max_dps for c in cases], dtype=float)
    c_damp = np.array([c.c_damp for c in cases], dtype=float)
    is_rate_method = np.array([c.brake_method == "rate" for c in cases], dtype=bool)
    is_ramp = np.array([c.spin_cmd_mode == "ramp" for c in cases], dtype=bool)
    alpha_cmd = np.array([c.alpha_cmd_radss for c in cases], dtype=float)

    motor_t_max = np.array([THRUST_MODES[c.thrust_limit_mode]["motor_t_max"] for c in cases], dtype=float)
    t_boost_N = np.array([THRUST_MODES[c.thrust_limit_mode]["t_boost"] for c in cases], dtype=float)
    tau_c_limit = np.array([TAU_C_LIMITS[c.tau_c_limit_mode] for c in cases], dtype=float)

    # Effective per-motor thrust ceiling = min(mechanical cap, electrical
    # ceiling at this case's v_supply). At v_supply=3.7 V the steady-state
    # electrical ceiling is only ~0.163 N/motor -- BELOW both the "firmware"
    # (0.168 N) and "physical" (0.2 N) mechanical caps. The mixer and the
    # brake-trigger-angle formula below both use this effective (usually
    # electrically-bound) ceiling so their notion of "available torque" is
    # physically honest.
    # モータ1個あたりの実効推力上限 = min(機械上限, その電源電圧での電気的定常上限)。
    # 3.7V では電気的定常上限が約0.163N/motorしかなく、ファーム機械上限(0.168N)にも
    # 物理上限(0.2N)にも届かない。ミキサーとブレーキ判定角の計算は両方ともこの
    # 実効上限（大抵は電気的上限で決まる）を使い、「使えるトルク」の見積もりを
    # 物理的に誠実なものにする。
    t_elec_max = thrust_from_omega(motor_steady_state_omega(v_supply))
    effective_t_max = np.minimum(motor_t_max, t_elec_max)

    geometric_tau_max = 2.0 * ARM_D * effective_t_max
    tau_max_effective = np.minimum(geometric_tau_max, tau_c_limit)

    delta_phi_brake_deg = compute_delta_phi_brake_deg(omega_flip, I_roll, tau_max_effective)
    auto_trigger_deg = 360.0 - delta_phi_brake_deg

    handoff_deg = auto_trigger_deg.copy()
    for mode_val in ["270", "300", "330"]:
        m = np.array([(c.brake_method == "angle") and (c.handoff_mode == mode_val) for c in cases])
        handoff_deg[m] = float(mode_val)
    # method (ii) always uses the auto (Delta_phi_brake) trigger.
    # 方式(ii)は常に自動計算(Delta_phi_brake)のトリガ角を使う。
    handoff_deg[is_rate_method] = auto_trigger_deg[is_rate_method]

    # spin_cmd_mode=="ramp": trigger angle uses Delta_phi_brake = omega^2/(2*alpha_cmd)
    # + omega*t_lag with the RAMP's own programmed alpha_cmd (not the torque-derived
    # alpha_brake) -- and always enters the rate-brake phase (ramping the command back
    # down), regardless of the brake_method OFAT axis.
    # spin_cmd_mode=="ramp": トリガ角は Delta_phi_brake=omega^2/(2*alpha_cmd)+omega*t_lag
    # をランプ自身の計画加速度 alpha_cmd で計算（トルク由来の alpha_brake ではない）。
    # brake_method 軸に関わらず、常にレートブレーキ相当フェーズ（指令を alpha_cmd で
    # 引き戻す）へ入る。
    omega_flip_rad = deg2rad(omega_flip)
    ramp_delta_phi_brake_deg = rad2deg(omega_flip_rad ** 2 / (2.0 * alpha_cmd) + omega_flip_rad * T_LAG)
    ramp_trigger_deg = 360.0 - ramp_delta_phi_brake_deg
    handoff_deg = np.where(is_ramp, ramp_trigger_deg, handoff_deg)
    is_rate_method = is_rate_method | is_ramp

    insufficient_margin = handoff_deg < (phi_b + 5.0)
    handoff_deg = np.clip(handoff_deg, phi_b + 5.0, 359.0)

    Kphi = omega_att_max / PHI_SAT_DESIGN_DEG   # 1/s

    omega0 = np.sqrt(HOVER_THRUST_PER_MOTOR / (ETA * CT)) * np.ones(N)   # rad/s, hover motor speed

    z = np.zeros(N); vz = np.zeros(N)
    y = np.zeros(N); vy = np.zeros(N)
    phi = np.zeros(N)
    omega_body = np.zeros(N)
    wr = omega0.copy()
    wl = omega0.copy()
    omega_cmd_state = np.zeros(N)   # rad/s, ramp state for spin_cmd_mode=="ramp" cases only

    phase = np.zeros(N, dtype=np.int64)   # PHASE_BOOST
    n_boost_steps = np.round(t_boost / dt_ctrl).astype(int)

    min_z = np.zeros(N)
    max_phi_deg = np.zeros(N)
    max_omega_dps = np.zeros(N)
    sat_count = np.zeros(N)
    ctrl_count = np.zeros(N)
    tracking_err_max_dps = np.zeros(N)
    done_time = np.full(N, np.nan)
    done_z = np.full(N, np.nan)
    done_phierr = np.full(N, np.nan)
    failed = np.zeros(N, dtype=bool)

    n_ctrl_steps = int(round(t_max / dt_ctrl))

    traj = None
    if store_traj:
        traj = {k: [] for k in ["t", "phi_deg", "omega_dps", "omega_cmd_dps", "Tc", "tau_c",
                                 "Tr_pair", "Tl_pair", "z", "vz", "y", "phase", "sat"]}

    for step in range(n_ctrl_steps):
        active = (phase != PHASE_DONE) & (phase != PHASE_FAILED)
        if not np.any(active):
            break
        t_now = step * dt_ctrl
        phi_deg = rad2deg(phi)
        omega_dps = rad2deg(omega_body)
        phi_err_deg = phi_deg - 360.0

        # ---- phase transitions (based on state at the START of this control step) ----
        # ---- フェーズ遷移判定（本制御周期の開始時点の状態を用いる） ----
        is_boost = (phase == PHASE_BOOST)
        phase = np.where(is_boost & (step >= n_boost_steps), PHASE_SPIN, phase)

        is_spin = (phase == PHASE_SPIN)
        spin_next = np.where(is_rate_method, PHASE_BRAKE_RATE, PHASE_BRAKE_ANGLE)
        phase = np.where(is_spin & (phi_deg >= handoff_deg), spin_next, phase)

        is_brake_rate = (phase == PHASE_BRAKE_RATE)
        brake_rate_done = is_brake_rate & (
            ((phi_deg >= BRAKE_RATE_EXIT_PHI_LOOSE_DEG) & (np.abs(omega_dps) < BRAKE_RATE_EXIT_OMEGA_LOOSE_DPS))
            | (phi_deg >= BRAKE_RATE_EXIT_PHI_HARD_DEG)
        )
        phase = np.where(brake_rate_done, PHASE_BRAKE_ANGLE, phase)

        is_brake_angle = (phase == PHASE_BRAKE_ANGLE)
        brake_angle_done = is_brake_angle & (np.abs(phi_err_deg) <= PHI_ERR_ENTER_RECOVER_DEG) & \
            (np.abs(omega_dps) < OMEGA_ENTER_RECOVER_DPS)
        phase = np.where(brake_angle_done, PHASE_RECOVER, phase)

        is_recover = (phase == PHASE_RECOVER)
        recover_done = is_recover & (vz >= 0.0)
        newly_done = recover_done & np.isnan(done_time)
        done_time = np.where(newly_done, t_now, done_time)
        done_z = np.where(newly_done, z, done_z)
        done_phierr = np.where(newly_done, phi_err_deg, done_phierr)
        phase = np.where(recover_done, PHASE_DONE, phase)

        still_active_for_fail = (phase != PHASE_DONE) & (phase != PHASE_FAILED)
        will_fail = still_active_for_fail & (t_now >= t_max)
        failed = failed | will_fail
        phase = np.where(will_fail, PHASE_FAILED, phase)

        active = (phase != PHASE_DONE) & (phase != PHASE_FAILED)

        # ---- command computation ----
        phi_err_rad = deg2rad(phi_err_deg)
        omega_att_cmd = np.clip(Kphi * (-phi_err_rad), -deg2rad(omega_att_max), deg2rad(omega_att_max))

        # spin_cmd_mode=="ramp": omega_cmd ramps linearly at +-alpha_cmd instead of
        # stepping straight to omega_flip / 0. Non-ramp cases leave this state at 0
        # (unused).
        # spin_cmd_mode=="ramp": omega_cmd はステップではなく +-alpha_cmd で直線ランプ
        # する（ランプでないケースはこの状態を使わず0のまま）。
        ramp_up_val = np.minimum(omega_flip_rad, omega_cmd_state + alpha_cmd * dt_ctrl)
        ramp_down_val = np.maximum(0.0, omega_cmd_state - alpha_cmd * dt_ctrl)
        new_ramp_state = np.where(phase == PHASE_SPIN, ramp_up_val,
                          np.where(phase == PHASE_BRAKE_RATE, ramp_down_val, omega_cmd_state))
        omega_cmd_state = np.where(is_ramp, new_ramp_state, omega_cmd_state)

        Tc_spin_sched = np.where(phi_deg < phi_a, t_boost_N,
                         np.where(phi_deg < phi_b, T_idle, t_boost_N))

        omega_cmd_spin = np.where(is_ramp, omega_cmd_state, omega_flip_rad)
        omega_cmd_brake_rate = np.where(is_ramp, omega_cmd_state, np.zeros(N))

        phase_list = [phase == PHASE_BOOST, phase == PHASE_SPIN, phase == PHASE_BRAKE_RATE,
                      phase == PHASE_BRAKE_ANGLE, phase == PHASE_RECOVER]
        Tc = np.select(phase_list,
                        [t_boost_N, Tc_spin_sched, t_boost_N, t_boost_N, t_boost_N],
                        default=HOVER_THRUST_TOTAL)
        omega_cmd = np.select(phase_list,
                               [np.zeros(N), omega_cmd_spin, omega_cmd_brake_rate, omega_att_cmd, omega_att_cmd],
                               default=0.0)

        tau_c = KP_RATE * (omega_cmd - omega_body)
        tau_c = np.clip(tau_c, -tau_c_limit, tau_c_limit)

        tracking_err_max_dps = np.maximum(
            tracking_err_max_dps, np.where(active, np.abs(rad2deg(omega_cmd) - omega_dps), tracking_err_max_dps))

        right_c, left_c, sat_flag, Tc_ach, tau_ach = mix_motors(Tc, tau_c, policy_is_B, motor_t_max)

        Vr_raw = voltage_for_thrust(right_c)
        Vl_raw = voltage_for_thrust(left_c)
        elec_sat = (Vr_raw > v_supply) | (Vl_raw > v_supply)
        Vr_cmd = np.clip(Vr_raw, 0.0, v_supply)
        Vl_cmd = np.clip(Vl_raw, 0.0, v_supply)
        any_sat = sat_flag | elec_sat

        sat_count += np.where(active, any_sat.astype(float), 0.0)
        ctrl_count += active.astype(float)

        if store_traj:
            traj["t"].append(t_now)
            traj["phi_deg"].append(float(phi_deg[0]))
            traj["omega_dps"].append(float(omega_dps[0]))
            traj["omega_cmd_dps"].append(float(rad2deg(omega_cmd)[0]))
            traj["Tc"].append(float(Tc_ach[0]))
            traj["tau_c"].append(float(tau_ach[0]))
            traj["Tr_pair"].append(float(2.0 * right_c[0]))
            traj["Tl_pair"].append(float(2.0 * left_c[0]))
            traj["z"].append(float(z[0])); traj["vz"].append(float(vz[0])); traj["y"].append(float(y[0]))
            traj["phase"].append(int(phase[0]))
            traj["sat"].append(bool(any_sat[0]))

        for _ in range(N_SUB):
            state = (z, vz, y, vy, phi, omega_body, wr, wl)
            state = rk4_step(state, dt_phys, Vr_cmd, Vl_cmd, I_roll, c_damp)
            (z, vz, y, vy, phi, omega_body, wr, wl) = state

        min_z = np.minimum(min_z, np.where(active, z, min_z))
        max_phi_deg = np.maximum(max_phi_deg, np.where(active, rad2deg(phi), max_phi_deg))
        max_omega_dps = np.maximum(max_omega_dps, np.where(active, np.abs(rad2deg(omega_body)), max_omega_dps))

    still_running = (phase != PHASE_DONE) & (phase != PHASE_FAILED)
    failed = failed | still_running
    phase = np.where(still_running, PHASE_FAILED, phase)

    sat_fraction = np.divide(sat_count, np.maximum(ctrl_count, 1.0))
    altitude_drop = -np.minimum(min_z, 0.0)

    return dict(
        handoff_deg=handoff_deg, insufficient_margin=insufficient_margin,
        done_time=done_time, done_z=done_z, done_phierr=done_phierr,
        max_phi_deg=max_phi_deg, max_omega_dps=max_omega_dps,
        min_z=min_z, altitude_drop=altitude_drop, sat_fraction=sat_fraction,
        failed=failed, tau_max_effective=tau_max_effective,
        tracking_err_max_dps=tracking_err_max_dps,
        traj=traj,
    )


# =====================================================================
# 8. Sweep case construction: baseline + OFAT (one-factor-at-a-time) +
#    a small curated "risk" grid + a dedicated omega_flip x inertia grid
#    (to answer "does the recommended omega_flip change for the pitch axis").
#    掃引ケース構築: baseline + OFAT + 小規模リスクグリッド +
#    「ピッチ軸で推奨omega_flipが変わるか」専用の omega_flip x 慣性 グリッド。
# =====================================================================

def variant(group, axis, value_label, **kwargs):
    c = replace(BASE, label=f"{axis}={value_label}", **kwargs)
    return (group, axis, str(value_label), c)


def make_cases():
    cases = [("baseline", "baseline", "baseline", replace(BASE, label="baseline"))]

    for v in [1000.0, 1200.0, 1800.0]:
        cases.append(variant("OFAT", "omega_flip_dps", v, omega_flip_dps=v))

    for v in [0.0, 0.12, 0.20]:
        cases.append(variant("OFAT", "T_idle", v, T_idle=v))

    for v in [40.0, 80.0]:
        cases.append(variant("OFAT", "phi_a_deg", v, phi_a_deg=v))

    for v in [260.0, 340.0]:
        cases.append(variant("OFAT", "phi_b_deg", v, phi_b_deg=v))

    # brake method comparison: baseline is "rate" (ii); compare against "angle" (i)
    # with auto trigger and 3 fixed trigger angles.
    cases.append(variant("OFAT", "brake_method", "angle_auto", brake_method="angle", handoff_mode="auto"))
    for hm in ["270", "300", "330"]:
        cases.append(variant("OFAT", "brake_method", f"angle_{hm}", brake_method="angle", handoff_mode=hm))

    cases.append(variant("OFAT", "policy", "A", policy="A"))
    cases.append(variant("OFAT", "v_supply", 3.5, v_supply=3.5))
    cases.append(variant("OFAT", "I_roll", "Iyy", I_roll=IYY))

    for v in [0.050, 0.150, 0.200]:
        cases.append(variant("OFAT", "t_boost", v, t_boost=v))

    for v in [400.0, 800.0]:
        cases.append(variant("OFAT", "omega_att_max_dps", v, omega_att_max_dps=v))

    cases.append(variant("OFAT", "c_damp", "small", c_damp=5e-7))

    cases.append(variant("OFAT", "thrust_limit_mode", "physical", thrust_limit_mode="physical"))
    cases.append(variant("OFAT", "tau_c_limit_mode", "unlimited", tau_c_limit_mode="unlimited"))

    # spin-command shape: (a) step to omega_flip [baseline] vs (b) linear ramp at
    # alpha_cmd, decelerating at the same alpha_cmd (coordinator follow-up #1).
    # スピン指令の与え方: (a)ステップ[基準] vs (b)alpha_cmdで直線ランプ(加減速とも同じ勾配)。
    for a in [150.0, 300.0, 450.0]:
        cases.append(variant("OFAT", "spin_cmd_mode", f"ramp_a{int(a)}",
                              spin_cmd_mode="ramp", alpha_cmd_radss=a, brake_method="rate"))

    # --- curated risk grid: aggressive spin + weak battery + all combinations
    # of {thrust limit mode} x {torque limit mode} x {mixer policy} x
    # {brake method} -- targets "dangerous combination" hunting. ---
    # 危険な組合せ探索用の小規模グリッド: 積極的な回転速度＋低電圧のもとで、
    # 推力上限モード x トルク上限モード x ミキサー方針 x ブレーキ方式 の全組合せ。
    for tlm in ["firmware", "physical"]:
        for tcl in ["firmware", "unlimited"]:
            for pol in ["A", "B"]:
                for bm in ["rate", "angle"]:
                    lbl = f"risk_tlm{tlm}_tcl{tcl}_pol{pol}_bm{bm}"
                    kw = dict(omega_flip_dps=1800.0, v_supply=3.5, T_idle=0.12,
                              thrust_limit_mode=tlm, tau_c_limit_mode=tcl,
                              policy=pol, brake_method=bm)
                    if bm == "angle":
                        kw["handoff_mode"] = "auto"
                    c = replace(BASE, label=lbl, **kw)
                    cases.append(("risk_grid", lbl, lbl, c))

    # --- dedicated omega_flip x inertia grid (roll Ixx vs "as if pitch" Iyy) ---
    # omega_flip x 慣性 専用グリッド（ロールIxx vs 「ピッチ軸だったら」Iyy）。
    for wf in [1000.0, 1200.0, 1500.0, 1800.0]:
        for ir_name, ir in [("Ixx", IXX), ("Iyy", IYY)]:
            lbl = f"pitchcheck_wf{int(wf)}_{ir_name}"
            c = replace(BASE, label=lbl, omega_flip_dps=wf, I_roll=ir)
            cases.append(("pitch_axis_check", lbl, lbl, c))

    return cases


# =====================================================================
# 9. Diagnostics, plotting, CSV output, main()
# =====================================================================

def print_electrical_diagnostics():
    print("=" * 78)
    print("Motor electrical-vs-mechanical ceiling diagnostics")
    print("モータの電気的上限 vs 機械的上限の診断")
    print("=" * 78)
    for mode, p in THRUST_MODES.items():
        for vs in [3.7, 3.5]:
            omega_max = motor_steady_state_omega(np.array([vs]))[0]
            t_max_elec = float(thrust_from_omega(omega_max))
            print(f"  thrust_limit_mode={mode:9s} v_supply={vs} V -> "
                  f"steady-state max thrust/motor = {t_max_elec:.4f} N "
                  f"(mechanical cap = {p['motor_t_max']:.3f} N, "
                  f"boost/motor cmd = {p['t_boost']/4:.4f} N)")
    v_for_mech_max = float(voltage_for_thrust(np.array([0.200]))[0])
    v_for_fw_max = float(voltage_for_thrust(np.array([0.168]))[0])
    print(f"  voltage required for 0.200 N/motor (physical cap) = {v_for_mech_max:.3f} V")
    print(f"  voltage required for 0.168 N/motor (firmware cap) = {v_for_fw_max:.3f} V")
    print(f"  hover thrust/motor = {HOVER_THRUST_PER_MOTOR:.4f} N, "
          f"hover omega = {math.sqrt(HOVER_THRUST_PER_MOTOR/(ETA*CT)):.1f} rad/s")
    print(f"  KP_RATE = {KP_RATE:.6e} N*m/(rad/s)")
    print("=" * 78)


def plot_baseline(traj, path):
    t = np.array(traj["t"])
    phi = np.array(traj["phi_deg"])
    omega = np.array(traj["omega_dps"])
    Tc = np.array(traj["Tc"])
    tau_c = np.array(traj["tau_c"])
    Tr = np.array(traj["Tr_pair"])
    Tl = np.array(traj["Tl_pair"])
    z = np.array(traj["z"])
    vz = np.array(traj["vz"])
    phase = np.array(traj["phase"])

    fig, axes = plt.subplots(4, 1, figsize=(9, 12), sharex=True)

    ax = axes[0]
    ax.plot(t, phi, label="phi [deg] (cumulative roll)")
    ax.axhline(360, color="gray", lw=0.7, ls="--")
    ax2 = ax.twinx()
    ax2.plot(t, omega, color="tab:orange", label="omega [deg/s]")
    ax.set_ylabel("phi [deg]")
    ax2.set_ylabel("omega [deg/s]", color="tab:orange")
    ax.set_title("StampFly flip: baseline case (rate-brake method, firmware limits)")
    ax.legend(loc="upper left"); ax2.legend(loc="upper right")

    ax = axes[1]
    ax.plot(t, Tc, label="Tc (achieved collective) [N]")
    ax.plot(t, Tr, label="right pair thrust [N]", alpha=0.7)
    ax.plot(t, Tl, label="left pair thrust [N]", alpha=0.7)
    ax.set_ylabel("thrust [N]")
    ax.legend(loc="upper right")

    ax = axes[2]
    ax.plot(t, tau_c * 1e3, color="tab:green", label="tau_c (achieved) [mN*m]")
    ax.set_ylabel("torque [mN*m]")
    ax.legend(loc="upper right")

    ax = axes[3]
    ax.plot(t, z, label="z [m]")
    ax2 = ax.twinx()
    ax2.plot(t, vz, color="tab:red", label="vz [m/s]")
    ax.set_ylabel("z [m]")
    ax2.set_ylabel("vz [m/s]", color="tab:red")
    ax.set_xlabel("time [s]")
    ax.legend(loc="upper left"); ax2.legend(loc="lower right")

    # shade phases on all subplots
    phase_names = {PHASE_BOOST: "Boost", PHASE_SPIN: "Spin", PHASE_BRAKE_RATE: "Brake(rate)",
                   PHASE_BRAKE_ANGLE: "Brake(angle)", PHASE_RECOVER: "Recover",
                   PHASE_DONE: "Done", PHASE_FAILED: "Failed"}
    colors = {PHASE_BOOST: "#dbeafe", PHASE_SPIN: "#fee2e2", PHASE_BRAKE_RATE: "#fef3c7",
              PHASE_BRAKE_ANGLE: "#fde68a", PHASE_RECOVER: "#d1fae5", PHASE_DONE: "#e5e7eb",
              PHASE_FAILED: "#000000"}
    change_idx = np.where(np.diff(phase) != 0)[0] + 1
    bounds = [0] + list(change_idx) + [len(t) - 1]
    for ax in axes:
        for i in range(len(bounds) - 1):
            i0, i1 = bounds[i], bounds[i + 1]
            ax.axvspan(t[i0], t[min(i1, len(t) - 1)], color=colors.get(phase[i0], "white"), alpha=0.25, lw=0)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def write_csv(cases, results, path):
    fieldnames = ["group", "axis", "value", "label",
                  "omega_flip_dps", "T_idle_N", "phi_a_deg", "phi_b_deg",
                  "brake_method", "handoff_mode", "policy", "v_supply_V",
                  "I_roll", "t_boost_s", "omega_att_max_dps", "c_damp",
                  "thrust_limit_mode", "tau_c_limit_mode",
                  "spin_cmd_mode", "alpha_cmd_radss",
                  "trigger_deg_used", "insufficient_brake_margin",
                  "tau_max_effective_Nm",
                  "total_time_s", "max_omega_dps", "max_phi_deg",
                  "altitude_drop_m", "done_altitude_m", "attitude_residual_deg",
                  "motor_sat_fraction", "tracking_err_max_dps", "failed"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for i, (group, axis, value, c) in enumerate(cases):
            I_name = "Ixx" if abs(c.I_roll - IXX) < 1e-12 else ("Iyy" if abs(c.I_roll - IYY) < 1e-12 else f"{c.I_roll:.3e}")
            w.writerow(dict(
                group=group, axis=axis, value=value, label=c.label,
                omega_flip_dps=c.omega_flip_dps, T_idle_N=c.T_idle,
                phi_a_deg=c.phi_a_deg, phi_b_deg=c.phi_b_deg,
                brake_method=c.brake_method, handoff_mode=c.handoff_mode,
                policy=c.policy, v_supply_V=c.v_supply, I_roll=I_name,
                t_boost_s=c.t_boost, omega_att_max_dps=c.omega_att_max_dps,
                c_damp=c.c_damp, thrust_limit_mode=c.thrust_limit_mode,
                tau_c_limit_mode=c.tau_c_limit_mode,
                spin_cmd_mode=c.spin_cmd_mode, alpha_cmd_radss=c.alpha_cmd_radss,
                trigger_deg_used=round(float(results["handoff_deg"][i]), 2),
                insufficient_brake_margin=bool(results["insufficient_margin"][i]),
                tau_max_effective_Nm=round(float(results["tau_max_effective"][i]), 6),
                total_time_s=("" if np.isnan(results["done_time"][i]) else round(float(results["done_time"][i]), 4)),
                max_omega_dps=round(float(results["max_omega_dps"][i]), 1),
                max_phi_deg=round(float(results["max_phi_deg"][i]), 1),
                altitude_drop_m=round(float(results["altitude_drop"][i]), 4),
                done_altitude_m=("" if np.isnan(results["done_z"][i]) else round(float(results["done_z"][i]), 4)),
                attitude_residual_deg=("" if np.isnan(results["done_phierr"][i]) else round(float(results["done_phierr"][i]), 2)),
                motor_sat_fraction=round(float(results["sat_fraction"][i]), 3),
                tracking_err_max_dps=round(float(results["tracking_err_max_dps"][i]), 1),
                failed=bool(results["failed"][i]),
            ))


def run_case_O(v_supply=3.7, I_roll=IXX, thrust_limit_mode="firmware",
               tau_c_limit_mode="firmware", policy="B"):
    """
    Case O -- "factory-firmware-equivalent" open-loop comparison profile
    (coordinator follow-up #2). Purely time-triggered, NO attitude control --
    rotation angle is whatever the physics produces from the prescribed
    omega_cmd(t)/Tc(t) schedule, run through the SAME rate loop (KP_RATE,
    tau_c_limit), mixer and motor/rigid-body physics as the rest of this
    script, so it is directly comparable to the baseline "flip design" case.
    ケースO -- 「工場出荷ファーム相当」の時間駆動オープンループ比較プロファイル
    （コーディネータ追加指示#2）。姿勢制御は使わず、回転角は所定の
    omega_cmd(t)/Tc(t) スケジュールを他ケースと同じレートループ・ミキサー・
    モータ/剛体物理に通した結果として決まる -- 基準（フリップ案）ケースと
    直接比較できる。

    Timeline (coordinator spec):
      [0, 0.375s)              level hold,        Tc = 1.2 * T_hover
      [0.375, 0.575s)          omega_cmd ramps up   at +0.3534 rad/tick (400Hz)
      [0.575, 0.775s)          omega_cmd ramps down at the same slope
        (peak omega_cmd = 80 ticks * 0.3534 = 28.272 rad/s; the commanded
         profile's own integral is a 324 deg triangle -- NOT the actual
         achieved rotation, which the rate loop/motors may not track exactly)
        Tc during this 0.4 s window, split into 4 x 0.1s quarters:
          1.05, 1.0, 1.0, 1.4  (x T_hover)
      [0.775, 1.075s)          omega_cmd = 0, Tc = 1.4 * T_hover

    NOTE: the coordinator's message says results should be read "at 0.875s
    after [start]", but the stated segment durations (0.375+0.2+0.2+0.3s)
    sum to 1.075 s, not 0.875 s. This implementation uses the segment
    durations verbatim (unambiguous) and reports the state at the END of
    that 1.075 s profile; the 0.875 s figure looks like an arithmetic slip
    and is flagged here rather than silently guessed at.
    注記: コーディネータの指示文は「0.875s後」に結果を読むとあるが、明記された
    区間長（0.375+0.2+0.2+0.3s）の合計は1.075sであり0.875sと一致しない。本実装
    は曖昧さのない区間長の方をそのまま使い、その1.075sプロファイル終了時点の
    状態を報告する（0.875sは計算上の行き違いと見て、ここに明記する）。
    """
    motor_t_max = THRUST_MODES[thrust_limit_mode]["motor_t_max"]
    tau_c_limit = TAU_C_LIMITS[tau_c_limit_mode]
    t_elec_max = float(thrust_from_omega(motor_steady_state_omega(np.array([v_supply])))[0])
    effective_t_max = np.array([min(motor_t_max, t_elec_max)])
    policy_is_B = np.array([policy == "B"])

    T_HOVER = HOVER_THRUST_TOTAL   # 0.363 N (mg), matches coordinator's stated value

    t_level, t_up, t_down, t_hold = 0.375, 0.200, 0.200, 0.300
    n_level = int(round(t_level / DT_CTRL))
    n_up = int(round(t_up / DT_CTRL))
    n_down = int(round(t_down / DT_CTRL))
    n_hold = int(round(t_hold / DT_CTRL))
    n_total = n_level + n_up + n_down + n_hold
    n_quarter = int(round(0.1 / DT_CTRL))
    thrust_quarters = [1.05, 1.0, 1.0, 1.4]
    RATE_STEP_PER_TICK = 0.3534   # rad/s added (or removed) per control tick

    z = 0.0; vz = 0.0; y = 0.0; vy = 0.0; phi = 0.0; omega_body = 0.0
    wr = math.sqrt(HOVER_THRUST_PER_MOTOR / (ETA * CT))
    wl = wr
    omega_cmd_state = 0.0
    min_z = 0.0

    traj = {k: [] for k in ["t", "phi_deg", "omega_dps", "Tc", "z", "vz"]}

    for step in range(n_total):
        t_now = step * DT_CTRL
        if step < n_level:
            Tc = 1.2 * T_HOVER
            omega_cmd = 0.0
        elif step < n_level + n_up + n_down:
            k = step - n_level                      # 0..(n_up+n_down-1), spans the full 0.4 s
            q = min(k // n_quarter, 3)
            Tc = thrust_quarters[q] * T_HOVER
            if step < n_level + n_up:
                omega_cmd_state = min(60.0, omega_cmd_state + RATE_STEP_PER_TICK)   # ramp up
            else:
                omega_cmd_state = max(0.0, omega_cmd_state - RATE_STEP_PER_TICK)    # ramp down
            omega_cmd = omega_cmd_state
        else:
            omega_cmd = 0.0
            Tc = 1.4 * T_HOVER

        tau_c = KP_RATE * (omega_cmd - omega_body)
        tau_c = float(np.clip(tau_c, -tau_c_limit, tau_c_limit))

        right_c, left_c, _sat, Tc_ach, _tau_ach = mix_motors(
            np.array([Tc]), np.array([tau_c]), policy_is_B, effective_t_max)
        Vr = float(np.clip(voltage_for_thrust(right_c), 0.0, v_supply)[0])
        Vl = float(np.clip(voltage_for_thrust(left_c), 0.0, v_supply)[0])

        traj["t"].append(t_now)
        traj["phi_deg"].append(math.degrees(phi))
        traj["omega_dps"].append(math.degrees(omega_body))
        traj["Tc"].append(float(Tc_ach[0]))
        traj["z"].append(z); traj["vz"].append(vz)

        for _ in range(N_SUB):
            state = (z, vz, y, vy, phi, omega_body, wr, wl)
            z, vz, y, vy, phi, omega_body, wr, wl = rk4_step(
                state, DT_PHYS, Vr, Vl, I_roll, 0.0)
            min_z = min(min_z, z)

    phi_deg_final = math.degrees(phi)
    nearest_rev = 360.0 * round(phi_deg_final / 360.0)
    residual_deg = phi_deg_final - nearest_rev

    return dict(
        traj=traj, total_rotation_deg=phi_deg_final,
        altitude_drop_m=-min(min_z, 0.0), min_z=min_z,
        final_z=z, final_vz=vz, attitude_residual_deg=residual_deg,
        nearest_revolution_deg=nearest_rev, t_end=n_total * DT_CTRL,
        commanded_integral_deg=324.0,
    )


def plot_case_O(traj, path):
    t = np.array(traj["t"]); phi = np.array(traj["phi_deg"]); omega = np.array(traj["omega_dps"])
    Tc = np.array(traj["Tc"]); z = np.array(traj["z"]); vz = np.array(traj["vz"])
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    ax = axes[0]
    ax.plot(t, phi, label="phi [deg]")
    ax2 = ax.twinx(); ax2.plot(t, omega, color="tab:orange", label="omega [deg/s]")
    ax.set_ylabel("phi [deg]"); ax2.set_ylabel("omega [deg/s]", color="tab:orange")
    ax.set_title("Case O: factory-firmware-equivalent open-loop profile")
    ax.legend(loc="upper left"); ax2.legend(loc="upper right")
    ax = axes[1]
    ax.plot(t, Tc, label="Tc (achieved) [N]"); ax.set_ylabel("thrust [N]"); ax.legend()
    ax = axes[2]
    ax.plot(t, z, label="z [m]")
    ax2 = ax.twinx(); ax2.plot(t, vz, color="tab:red", label="vz [m/s]")
    ax.set_ylabel("z [m]"); ax2.set_ylabel("vz [m/s]", color="tab:red"); ax.set_xlabel("time [s]")
    ax.legend(loc="upper left"); ax2.legend(loc="lower right")
    fig.tight_layout(); fig.savefig(path, dpi=150); plt.close(fig)


def main():
    print_electrical_diagnostics()

    # ---- baseline single-case run with trajectory for plotting ----
    baseline_case = replace(BASE, label="baseline")
    base_res = run_batch([baseline_case], store_traj=True)
    plot_path = os.path.join(OUT_DIR, "flip_baseline_timeseries.png")
    plot_baseline(base_res["traj"], plot_path)
    print(f"\nBaseline trajectory plot saved: {plot_path}")
    print(f"Baseline: total_time={base_res['done_time'][0]:.4f}s "
          f"max_omega={base_res['max_omega_dps'][0]:.1f} deg/s "
          f"altitude_drop={base_res['altitude_drop'][0]*100:.2f} cm "
          f"done_altitude={base_res['done_z'][0]*100:.2f} cm "
          f"attitude_residual={base_res['done_phierr'][0]:.2f} deg "
          f"sat_fraction={base_res['sat_fraction'][0]*100:.1f}% "
          f"trigger_deg={base_res['handoff_deg'][0]:.1f} "
          f"failed={bool(base_res['failed'][0])}")

    # ---- full sweep ----
    cases_meta = make_cases()
    cases_only = [c for (_, _, _, c) in cases_meta]
    print(f"\nRunning sweep with {len(cases_only)} cases ...")
    results = run_batch(cases_only, store_traj=False)

    csv_path = os.path.join(OUT_DIR, "flip_sweep_results.csv")
    write_csv(cases_meta, results, csv_path)
    print(f"Sweep CSV saved: {csv_path}")

    # ---- text summary ----
    summary_path = os.path.join(OUT_DIR, "flip_sweep_summary.txt")
    lines = []
    lines.append(f"Total cases: {len(cases_only)}")
    n_failed = int(np.sum(results["failed"]))
    lines.append(f"Failed cases: {n_failed} / {len(cases_only)}")
    lines.append("")
    lines.append("Failed case labels:")
    for i, (group, axis, value, c) in enumerate(cases_meta):
        if results["failed"][i]:
            lines.append(f"  [{group}] {c.label}: max_omega={results['max_omega_dps'][i]:.0f}dps "
                          f"max_phi={results['max_phi_deg'][i]:.0f}deg sat={results['sat_fraction'][i]*100:.0f}%")
    lines.append("")
    lines.append("Worst altitude drop (top 10):")
    order = np.argsort(-results["altitude_drop"])
    for idx in order[:10]:
        group, axis, value, c = cases_meta[idx]
        lines.append(f"  [{group}] {c.label}: drop={results['altitude_drop'][idx]*100:.2f}cm "
                      f"done_z={results['done_z'][idx]*100 if not np.isnan(results['done_z'][idx]) else float('nan'):.2f}cm "
                      f"sat={results['sat_fraction'][idx]*100:.0f}% failed={results['failed'][idx]}")
    lines.append("")
    lines.append("Highest motor saturation fraction (top 10):")
    order = np.argsort(-results["sat_fraction"])
    for idx in order[:10]:
        group, axis, value, c = cases_meta[idx]
        lines.append(f"  [{group}] {c.label}: sat={results['sat_fraction'][idx]*100:.0f}% "
                      f"drop={results['altitude_drop'][idx]*100:.2f}cm failed={results['failed'][idx]}")
    lines.append("")
    lines.append("pitch_axis_check group (omega_flip x inertia):")
    for i, (group, axis, value, c) in enumerate(cases_meta):
        if group == "pitch_axis_check":
            I_name = "Ixx" if abs(c.I_roll - IXX) < 1e-12 else "Iyy"
            t_done = results['done_time'][i]
            lines.append(f"  wf={c.omega_flip_dps:.0f}dps I={I_name}: "
                          f"time={'FAIL' if np.isnan(t_done) else f'{t_done:.3f}s'} "
                          f"drop={results['altitude_drop'][i]*100:.2f}cm "
                          f"sat={results['sat_fraction'][i]*100:.0f}% "
                          f"trigger={results['handoff_deg'][i]:.1f}deg "
                          f"failed={results['failed'][i]}")

    lines.append("")
    lines.append("spin_cmd_mode: step (baseline) vs ramp (alpha_cmd sweep):")
    baseline_idx = 0   # cases_meta[0] is always the baseline row
    lines.append(f"  step (baseline): tracking_err_max={results['tracking_err_max_dps'][baseline_idx]:.0f}dps "
                  f"sat={results['sat_fraction'][baseline_idx]*100:.0f}% "
                  f"time={results['done_time'][baseline_idx]:.3f}s "
                  f"drop={results['altitude_drop'][baseline_idx]*100:.2f}cm "
                  f"failed={results['failed'][baseline_idx]}")
    for i, (group, axis, value, c) in enumerate(cases_meta):
        if c.spin_cmd_mode == "ramp":
            t_done = results['done_time'][i]
            lines.append(f"  ramp alpha_cmd={c.alpha_cmd_radss:.0f}rad/s^2: "
                          f"tracking_err_max={results['tracking_err_max_dps'][i]:.0f}dps "
                          f"sat={results['sat_fraction'][i]*100:.0f}% "
                          f"time={'FAIL' if np.isnan(t_done) else f'{t_done:.3f}s'} "
                          f"drop={results['altitude_drop'][i]*100:.2f}cm "
                          f"trigger={results['handoff_deg'][i]:.1f}deg "
                          f"failed={results['failed'][i]}")

    # ---- Case O: factory-firmware-equivalent open-loop comparison ----
    # ケースO: 工場出荷ファーム相当のオープンループ比較。
    caseO = run_case_O()
    caseO_plot_path = os.path.join(OUT_DIR, "flip_caseO_timeseries.png")
    plot_case_O(caseO["traj"], caseO_plot_path)
    lines.append("")
    lines.append("Case O (factory-firmware-equivalent, open-loop, no attitude control):")
    lines.append(f"  profile end t={caseO['t_end']:.3f}s (segment durations sum to 1.075s; "
                 f"coordinator text said 0.875s -- see docstring note, flagged not silently fixed)")
    lines.append(f"  actual total rotation = {caseO['total_rotation_deg']:.1f} deg "
                 f"(commanded profile's own integral = {caseO['commanded_integral_deg']:.0f} deg -- "
                 f"NOT the same thing; actual rotation lags the open-loop command)")
    lines.append(f"  altitude drop = {caseO['altitude_drop_m']*100:.2f} cm, "
                 f"final z = {caseO['final_z']*100:.2f} cm, final vz = {caseO['final_vz']:.3f} m/s")
    lines.append(f"  attitude residual (from nearest 360deg multiple, "
                 f"nearest={caseO['nearest_revolution_deg']:.0f}deg) = {caseO['attitude_residual_deg']:.1f} deg")
    lines.append(f"  plot saved: {caseO_plot_path}")
    lines.append("")
    lines.append("Case O vs baseline (flip-design, rate-brake, firmware limits) side by side:")
    lines.append(f"  {'case':30s} {'time[s]':>8s} {'alt_drop[cm]':>13s} {'end_z[cm]':>10s} "
                 f"{'end_vz[m/s]':>12s} {'att_resid[deg]':>15s}")
    lines.append(f"  {'baseline (my design)':30s} {base_res['done_time'][0]:8.3f} "
                 f"{base_res['altitude_drop'][0]*100:13.2f} {base_res['done_z'][0]*100:10.2f} "
                 f"{'0.000 (vz>=0 trigger)':>12s} {base_res['done_phierr'][0]:15.2f}")
    lines.append(f"  {'case O (factory-equiv)':30s} {caseO['t_end']:8.3f} "
                 f"{caseO['altitude_drop_m']*100:13.2f} {caseO['final_z']*100:10.2f} "
                 f"{caseO['final_vz']:12.3f} {caseO['attitude_residual_deg']:15.2f}")

    summary_text = "\n".join(lines)
    with open(summary_path, "w") as f:
        f.write(summary_text + "\n")
    print(f"\nSummary saved: {summary_path}\n")
    print(summary_text)


if __name__ == "__main__":
    main()
