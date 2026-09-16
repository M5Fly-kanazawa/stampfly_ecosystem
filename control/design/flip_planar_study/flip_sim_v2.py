#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StampFly roll-flip maneuver -- v2 (coordinator review fixes).
StampFly ロール宙返り（フリップ）制御案 -- v2（検収コメント反映）。

This is a STANDALONE analysis script (NOT part of the stampfly_ecosystem repo
build); it never modifies any repository file. It reuses the low-level
physics/motor primitives from flip_sim.py (v1, kept unmodified alongside this
file so v1's own CSV/plots stay reproducible) and reimplements the
control/phase logic with the coordinator's two corrections:
これは stampfly_ecosystem リポジトリのビルドに含まれない単独スクリプトで、
リポジトリのファイルは変更しない。低レベルの物理・モータ関数は flip_sim.py
（v1、本ファイルと並置して変更せず残す＝v1のCSV/図の再現性を保つ）から再利用し、
制御/フェーズロジックはコーディネータの2件の修正を反映して再実装する:

  1. phi_b は固定値ではなく phi_brake (=360 - Delta_phi_brake) から
     phi_b = phi_brake - 20 deg として毎周期（現在の omega_body から）導出する。
     以前のように phi_b+5 へクリップする処理はしない（v1 の
     insufficient_brake_margin 問題の根本原因だった）。
     phi_b is now DERIVED every control cycle from the CURRENT omega_body as
     phi_b = phi_brake - 20 deg (no more clipping to a phi_b+5 floor -- that
     clipping was the root cause of v1's "insufficient_brake_margin" issue).
  2. 集合推力を3段階に分離: T_boost（Boost/Recoverのみ、最大推力）、
     T_spin_hi（Spinの加速窓 phi<phi_a と減速準備窓 phi_b<=phi<phi_brake、
     さらに実減速中の Brake フェーズでも使用。ホバー程度でトルク余裕を確保）、
     T_lo（phi_a<=phi<phi_b の反転コースト区間）。
     Collective thrust is now split into 3 tiers: T_boost (Boost/Recover
     only, max thrust), T_spin_hi (Spin's acceleration window phi<phi_a AND
     its brake-prep window phi_b<=phi<phi_brake, AND continues through the
     actual Brake phase -- roughly hover level, to preserve torque margin),
     T_lo (the inverted coast window phi_a<=phi<phi_b).

Additional changes per the coordinator's review:
  3. Mixer saturation policy "C" added (and made the new baseline): clamp
     each motor independently to [0, T_motor_max] with NO redistribution --
     this matches firmware/vehicle/components/sf_actuator/actuator.cpp's
     clampDuties(). Policies A/B are kept for comparison.
     ミキサー飽和方針「C」を追加（新基準）: 差動を保たず、モータ毎に
     独立して[0, T_motor_max]へクランプするだけ -- 現行ファームの
     actuator.cpp の clampDuties() と同じ方式。A/Bは比較用に残す。
  4. IMU位置（回転軸から1cm）での加速度計ノルムのピークと3G超過時間を
     出力する: |a| = sqrt((T/m)^2 + (omega^2*r)^2 + (alpha*r)^2)（3項を
     直交とみなした合成、保守的な近似 -- コーディネータの式をそのまま
     literal に実装）。
     Peak IMU-position accelerometer norm (IMU assumed 1 cm off the roll
     axis) and time above 3G are now tracked and reported: |a| =
     sqrt((T/m)^2 + (omega^2*r)^2 + (alpha*r)^2) (the three terms combined
     as if mutually orthogonal -- a conservative reading of the coordinator's
     formula, implemented literally).
  5. Re-swept per the coordinator's axis list (baseline fixed, one axis at a
     time -- no combinatorial risk grid this round, per "1軸ずつ").

Outputs (v2-suffixed so v1's files are untouched):
    flip_baseline_timeseries_v2.png
    flip_sweep_results_v2.csv
    flip_sweep_summary_v2.txt
(Case O is unchanged -- coordinator: "ケースOはそのままでよい" -- so it is
NOT rerun here; see v1's flip_caseO_timeseries.png / flip_sim.py::run_case_O.)
"""

import os
import csv
import math
from dataclasses import dataclass, replace

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import flip_sim as v1   # reuse physics/motor primitives + constants (v1 untouched)

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# Re-exported constants / primitives from v1 (single source of truth for the
# physics; only the control/phase logic below is new).
# v1からの再利用定数/関数（物理はv1が単一の出典。以下は制御/フェーズロジックのみ新規）。
MASS, IXX, IYY, ARM_D, G = v1.MASS, v1.IXX, v1.IYY, v1.ARM_D, v1.G
ETA, CT = v1.ETA, v1.CT
HOVER_THRUST_TOTAL = v1.HOVER_THRUST_TOTAL
HOVER_THRUST_PER_MOTOR = v1.HOVER_THRUST_PER_MOTOR
THRUST_MODES = v1.THRUST_MODES
TAU_C_LIMITS = v1.TAU_C_LIMITS
MOTOR_T_MIN = v1.MOTOR_T_MIN
motor_steady_state_omega = v1.motor_steady_state_omega
thrust_from_omega = v1.thrust_from_omega
voltage_for_thrust = v1.voltage_for_thrust
rk4_step = v1.rk4_step
deg2rad, rad2deg = v1.deg2rad, v1.rad2deg
DT_CTRL, DT_PHYS, N_SUB, T_MAX = v1.DT_CTRL, v1.DT_PHYS, v1.N_SUB, v1.T_MAX
KP_RATE = v1.KP_RATE
T_LAG, ALPHA_BRAKE_FRAC = v1.T_LAG, v1.ALPHA_BRAKE_FRAC
PHI_ERR_ENTER_RECOVER_DEG = v1.PHI_ERR_ENTER_RECOVER_DEG
OMEGA_ENTER_RECOVER_DPS = v1.OMEGA_ENTER_RECOVER_DPS
PHI_SAT_DESIGN_DEG = v1.PHI_SAT_DESIGN_DEG
BRAKE_RATE_EXIT_PHI_LOOSE_DEG = v1.BRAKE_RATE_EXIT_PHI_LOOSE_DEG
BRAKE_RATE_EXIT_OMEGA_LOOSE_DPS = v1.BRAKE_RATE_EXIT_OMEGA_LOOSE_DPS
BRAKE_RATE_EXIT_PHI_HARD_DEG = v1.BRAKE_RATE_EXIT_PHI_HARD_DEG
run_case_O = v1.run_case_O

R_IMU = 0.01   # m, assumed IMU offset from the roll axis (coordinator spec)
G_MPS2 = 9.81  # m/s^2, for the "G" normalization of the accelerometer norm

PHASE_BOOST, PHASE_SPIN, PHASE_BRAKE_RATE, PHASE_BRAKE_ANGLE, PHASE_RECOVER, \
    PHASE_DONE, PHASE_FAILED = range(7)

POLICY_CODE = {"A": 0, "B": 1, "C": 2}


# =====================================================================
# Mixer v2: adds policy "C" (raw per-motor clamp, matches firmware
# actuator.cpp clampDuties -- no redistribution).
# ミキサーv2: 方針C（生のクランプのみ、現行ファーム clampDuties 相当）を追加。
# =====================================================================

def mix_motors_v2(Tc, tau_c, policy_code, motor_t_max, d=ARM_D):
    x = tau_c / (4.0 * d)
    Tc4 = Tc / 4.0
    Tmax = motor_t_max
    Tmin = MOTOR_T_MIN

    right_raw = Tc4 + x
    left_raw = Tc4 - x
    sat_raw = (right_raw > Tmax + 1e-12) | (right_raw < Tmin - 1e-12) | \
              (left_raw > Tmax + 1e-12) | (left_raw < Tmin - 1e-12)

    # Policy A: preserve Tc, scale tau.
    boundA = np.clip(np.minimum(Tmax - Tc4, Tc4 - Tmin), 0.0, None)
    absx = np.abs(x)
    sA = np.where(absx < 1e-12, 1.0, np.clip(boundA / np.maximum(absx, 1e-12), 0.0, 1.0))
    rightA = Tc4 + sA * x
    leftA = Tc4 - sA * x

    # Policy B: preserve tau, scale Tc.
    eps = 1e-12
    safeTc4 = np.where(Tc4 > eps, Tc4, 1.0)
    t_upper = np.minimum((Tmax - x) / safeTc4, (Tmax + x) / safeTc4)
    tB = np.clip(t_upper, 0.0, 1.0)
    tB = np.where(Tc4 > eps, tB, 0.0)
    rightB = tB * Tc4 + x
    leftB = tB * Tc4 - x

    # Policy C: raw per-motor clamp, no redistribution (firmware clampDuties).
    rightC = right_raw
    leftC = left_raw

    right = np.select([policy_code == 0, policy_code == 1, policy_code == 2], [rightA, rightB, rightC])
    left = np.select([policy_code == 0, policy_code == 1, policy_code == 2], [leftA, leftB, leftC])

    right_c = np.clip(right, Tmin, Tmax)
    left_c = np.clip(left, Tmin, Tmax)
    Tc_ach = 2.0 * (right_c + left_c)
    tau_ach = 2.0 * d * (right_c - left_c)
    return right_c, left_c, sat_raw, Tc_ach, tau_ach


# =====================================================================
# Case definition (v2): phi_b removed (now derived), T_idle renamed T_lo,
# T_spin_hi added, policy default now "C", brake_method removed (always the
# rate-brake state machine; spin_cmd_mode covers step/ramp).
# =====================================================================

@dataclass
class CaseV2:
    label: str = "baseline"
    omega_flip_dps: float = 1500.0
    T_lo: float = 0.06
    T_spin_hi: float = 0.36
    phi_a_deg: float = 60.0
    policy: str = "C"
    v_supply: float = 3.7
    I_roll: float = IXX
    t_boost: float = 0.100
    omega_att_max_dps: float = 600.0
    c_damp: float = 0.0
    thrust_limit_mode: str = "firmware"
    tau_c_limit_mode: str = "firmware"
    spin_cmd_mode: str = "step"        # "step" | "ramp"
    alpha_cmd_radss: float = 300.0     # only used when spin_cmd_mode=="ramp"


BASE = CaseV2(label="baseline")


# =====================================================================
# Vectorized batch simulator (v2).
# =====================================================================

def run_batch(cases, dt_ctrl=DT_CTRL, dt_phys=DT_PHYS, t_max=T_MAX, store_traj=False):
    N = len(cases)

    omega_flip = np.array([c.omega_flip_dps for c in cases], dtype=float)
    T_lo = np.array([c.T_lo for c in cases], dtype=float)
    T_spin_hi = np.array([c.T_spin_hi for c in cases], dtype=float)
    phi_a = np.array([c.phi_a_deg for c in cases], dtype=float)
    policy_code = np.array([POLICY_CODE[c.policy] for c in cases], dtype=int)
    v_supply = np.array([c.v_supply for c in cases], dtype=float)
    I_roll = np.array([c.I_roll for c in cases], dtype=float)
    t_boost = np.array([c.t_boost for c in cases], dtype=float)
    omega_att_max = np.array([c.omega_att_max_dps for c in cases], dtype=float)
    c_damp = np.array([c.c_damp for c in cases], dtype=float)
    is_ramp = np.array([c.spin_cmd_mode == "ramp" for c in cases], dtype=bool)
    alpha_cmd = np.array([c.alpha_cmd_radss for c in cases], dtype=float)

    motor_t_max = np.array([THRUST_MODES[c.thrust_limit_mode]["motor_t_max"] for c in cases], dtype=float)
    t_boost_N = np.array([THRUST_MODES[c.thrust_limit_mode]["t_boost"] for c in cases], dtype=float)
    tau_c_limit = np.array([TAU_C_LIMITS[c.tau_c_limit_mode] for c in cases], dtype=float)

    # Effective per-motor thrust ceiling = min(mechanical, electrical @ v_supply)
    # -- see v1 for the rationale (at 3.7V the electrical ceiling, ~0.163N, is
    # already below both mechanical caps).
    t_elec_max = thrust_from_omega(motor_steady_state_omega(v_supply))
    effective_t_max = np.minimum(motor_t_max, t_elec_max)
    geometric_tau_max = 2.0 * ARM_D * effective_t_max
    tau_max_effective = np.minimum(geometric_tau_max, tau_c_limit)

    # alpha_brake source: ramp mode uses its own programmed alpha_cmd; step
    # mode uses 0.6 * tau_max_effective / I_roll (coordinator spec, both
    # unchanged in form from v1 -- computed once per case, independent of the
    # instantaneous Tc).
    # alpha_brakeの出典: ランプ方式は自身のalpha_cmd、ステップ方式は
    # 0.6*tau_max_effective/I_roll（コーディネータ指定の式。ケース毎に1回
    # 計算する定数で、瞬時のTcには依存しない）。
    alpha_brake_source = np.where(is_ramp, alpha_cmd, ALPHA_BRAKE_FRAC * tau_max_effective / I_roll)

    Kphi = omega_att_max / PHI_SAT_DESIGN_DEG
    omega_flip_rad = deg2rad(omega_flip)

    omega0 = np.sqrt(HOVER_THRUST_PER_MOTOR / (ETA * CT)) * np.ones(N)
    z = np.zeros(N); vz = np.zeros(N)
    y = np.zeros(N); vy = np.zeros(N)
    phi = np.zeros(N)
    omega_body = np.zeros(N)
    wr = omega0.copy(); wl = omega0.copy()
    omega_cmd_state = np.zeros(N)

    phase = np.zeros(N, dtype=np.int64)
    n_boost_steps = np.round(t_boost / dt_ctrl).astype(int)

    min_z = np.zeros(N)
    max_phi_deg = np.zeros(N)
    max_omega_dps = np.zeros(N)
    sat_count = np.zeros(N)
    ctrl_count = np.zeros(N)
    tracking_err_max_dps = np.zeros(N)
    max_accel_g = np.zeros(N)
    time_above_3g = np.zeros(N)
    done_time = np.full(N, np.nan)
    done_z = np.full(N, np.nan)
    done_phierr = np.full(N, np.nan)
    brake_trigger_deg_actual = np.full(N, np.nan)
    failed = np.zeros(N, dtype=bool)

    n_ctrl_steps = int(round(t_max / dt_ctrl))

    traj = None
    if store_traj:
        traj = {k: [] for k in ["t", "phi_deg", "omega_dps", "Tc", "tau_c",
                                 "Tr_pair", "Tl_pair", "z", "vz", "y", "phase",
                                 "sat", "phi_brake_now", "accel_g"]}

    for step in range(n_ctrl_steps):
        active = (phase != PHASE_DONE) & (phase != PHASE_FAILED)
        if not np.any(active):
            break
        t_now = step * dt_ctrl
        phi_deg = rad2deg(phi)
        omega_dps = rad2deg(omega_body)
        phi_err_deg = phi_deg - 360.0

        # ---- dynamic brake-trigger angle, recomputed EVERY cycle from the
        # CURRENT omega_body (not a fixed omega_flip target) ----
        # ---- 動的ブレーキトリガ角: 固定のomega_flipではなく、現在の
        # omega_bodyから毎周期再計算する ----
        omega_for_brake = np.abs(omega_body)
        delta_phi_brake_deg = rad2deg(omega_for_brake ** 2 / (2.0 * alpha_brake_source)
                                       + omega_for_brake * T_LAG)
        phi_brake_now = 360.0 - delta_phi_brake_deg
        phi_b_now = phi_brake_now - 20.0   # derived, NOT clipped to any floor

        # ---- phase transitions ----
        is_boost = (phase == PHASE_BOOST)
        phase = np.where(is_boost & (step >= n_boost_steps), PHASE_SPIN, phase)

        is_spin = (phase == PHASE_SPIN)
        spin_trigger = is_spin & (phi_deg >= phi_brake_now)
        newly_triggered = spin_trigger & np.isnan(brake_trigger_deg_actual)
        brake_trigger_deg_actual = np.where(newly_triggered, phi_deg, brake_trigger_deg_actual)
        phase = np.where(spin_trigger, PHASE_BRAKE_RATE, phase)

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

        # ---- ramp state ----
        ramp_up_val = np.minimum(omega_flip_rad, omega_cmd_state + alpha_cmd * dt_ctrl)
        ramp_down_val = np.maximum(0.0, omega_cmd_state - alpha_cmd * dt_ctrl)
        new_ramp_state = np.where(phase == PHASE_SPIN, ramp_up_val,
                          np.where(phase == PHASE_BRAKE_RATE, ramp_down_val, omega_cmd_state))
        omega_cmd_state = np.where(is_ramp, new_ramp_state, omega_cmd_state)

        # ---- 3-tier collective schedule ----
        Tc_spin_sched = np.where(phi_deg < phi_a, T_spin_hi,
                         np.where(phi_deg < phi_b_now, T_lo, T_spin_hi))
        phase_list = [phase == PHASE_BOOST, phase == PHASE_SPIN, phase == PHASE_BRAKE_RATE,
                      phase == PHASE_BRAKE_ANGLE, phase == PHASE_RECOVER]
        Tc = np.select(phase_list,
                        [t_boost_N, Tc_spin_sched, T_spin_hi, T_spin_hi, t_boost_N],
                        default=HOVER_THRUST_TOTAL)

        phi_err_rad = deg2rad(phi_err_deg)
        omega_att_cmd = np.clip(Kphi * (-phi_err_rad), -deg2rad(omega_att_max), deg2rad(omega_att_max))
        omega_cmd_spin = np.where(is_ramp, omega_cmd_state, omega_flip_rad)
        omega_cmd_brake_rate = np.where(is_ramp, omega_cmd_state, np.zeros(N))
        omega_cmd = np.select(phase_list,
                               [np.zeros(N), omega_cmd_spin, omega_cmd_brake_rate, omega_att_cmd, omega_att_cmd],
                               default=0.0)

        tau_c = KP_RATE * (omega_cmd - omega_body)
        tau_c = np.clip(tau_c, -tau_c_limit, tau_c_limit)
        tracking_err_max_dps = np.maximum(
            tracking_err_max_dps, np.where(active, np.abs(rad2deg(omega_cmd) - omega_dps), tracking_err_max_dps))

        right_c, left_c, sat_flag, Tc_ach, tau_ach = mix_motors_v2(Tc, tau_c, policy_code, effective_t_max)

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
            traj["Tc"].append(float(Tc_ach[0]))
            traj["tau_c"].append(float(tau_ach[0]))
            traj["Tr_pair"].append(float(2.0 * right_c[0]))
            traj["Tl_pair"].append(float(2.0 * left_c[0]))
            traj["z"].append(float(z[0])); traj["vz"].append(float(vz[0])); traj["y"].append(float(y[0]))
            traj["phase"].append(int(phase[0]))
            traj["sat"].append(bool(any_sat[0]))
            traj["phi_brake_now"].append(float(phi_brake_now[0]))

        accel_g_last = np.zeros(N)
        for _ in range(N_SUB):
            omega_before = omega_body.copy()
            state = (z, vz, y, vy, phi, omega_body, wr, wl)
            state = rk4_step(state, dt_phys, Vr_cmd, Vl_cmd, I_roll, c_damp)
            (z, vz, y, vy, phi, omega_body, wr, wl) = state

            alpha_est = (omega_body - omega_before) / dt_phys
            T_total_now = 2.0 * thrust_from_omega(wr) + 2.0 * thrust_from_omega(wl)
            accel_mag = np.sqrt((T_total_now / MASS) ** 2
                                 + (omega_body ** 2 * R_IMU) ** 2
                                 + (alpha_est * R_IMU) ** 2)
            accel_g_now = accel_mag / G_MPS2
            accel_g_last = accel_g_now
            max_accel_g = np.maximum(max_accel_g, np.where(active, accel_g_now, max_accel_g))
            time_above_3g += np.where(active & (accel_g_now > 3.0), dt_phys, 0.0)

        if store_traj:
            traj["accel_g"].append(float(accel_g_last[0]))

        min_z = np.minimum(min_z, np.where(active, z, min_z))
        max_phi_deg = np.maximum(max_phi_deg, np.where(active, rad2deg(phi), max_phi_deg))
        max_omega_dps = np.maximum(max_omega_dps, np.where(active, np.abs(rad2deg(omega_body)), max_omega_dps))

    still_running = (phase != PHASE_DONE) & (phase != PHASE_FAILED)
    failed = failed | still_running
    phase = np.where(still_running, PHASE_FAILED, phase)

    sat_fraction = np.divide(sat_count, np.maximum(ctrl_count, 1.0))
    altitude_drop = -np.minimum(min_z, 0.0)

    return dict(
        brake_trigger_deg_actual=brake_trigger_deg_actual,
        tau_max_effective=tau_max_effective,
        done_time=done_time, done_z=done_z, done_phierr=done_phierr,
        max_phi_deg=max_phi_deg, max_omega_dps=max_omega_dps,
        min_z=min_z, altitude_drop=altitude_drop, sat_fraction=sat_fraction,
        failed=failed, tracking_err_max_dps=tracking_err_max_dps,
        max_accel_g=max_accel_g, time_above_3g=time_above_3g,
        traj=traj,
    )


# =====================================================================
# Sweep case construction (v2): baseline fixed, ONE axis at a time, per
# the coordinator's explicit list (no combinatorial risk grid this round).
# =====================================================================

def variant(axis, value_label, **kwargs):
    c = replace(BASE, label=f"{axis}={value_label}", **kwargs)
    return (axis, str(value_label), c)


def make_cases():
    cases = [("baseline", "baseline", replace(BASE, label="baseline"))]

    for wf in [1000.0, 1200.0, 1500.0, 1800.0]:
        for ir_name, ir in [("Ixx", IXX), ("Iyy", IYY)]:
            lbl = f"wf{int(wf)}_{ir_name}"
            cases.append(("omega_flip_x_inertia", lbl,
                           replace(BASE, label=lbl, omega_flip_dps=wf, I_roll=ir)))

    for a in [150.0, 300.0, 450.0]:
        cases.append(variant("alpha_cmd", a, spin_cmd_mode="ramp", alpha_cmd_radss=a))

    for v in [0.25, 0.50]:
        cases.append(variant("T_spin_hi", v, T_spin_hi=v))

    for v in [0.0, 0.12]:
        cases.append(variant("T_lo", v, T_lo=v))

    for v in [0.050, 0.150, 0.200]:
        cases.append(variant("t_boost", v, t_boost=v))

    cases.append(variant("v_supply", 3.5, v_supply=3.5))

    for pol in ["A", "B"]:
        cases.append(variant("policy", pol, policy=pol))

    return cases


# =====================================================================
# Diagnostics / plotting / CSV / main
# =====================================================================

def plot_baseline(traj, path):
    t = np.array(traj["t"])
    phi = np.array(traj["phi_deg"]); omega = np.array(traj["omega_dps"])
    Tc = np.array(traj["Tc"]); tau_c = np.array(traj["tau_c"])
    Tr = np.array(traj["Tr_pair"]); Tl = np.array(traj["Tl_pair"])
    z = np.array(traj["z"]); vz = np.array(traj["vz"])
    phase = np.array(traj["phase"]); accel_g = np.array(traj["accel_g"])

    fig, axes = plt.subplots(5, 1, figsize=(9, 14), sharex=True)

    ax = axes[0]
    ax.plot(t, phi, label="phi [deg]")
    ax.axhline(360, color="gray", lw=0.7, ls="--")
    ax2 = ax.twinx(); ax2.plot(t, omega, color="tab:orange", label="omega [deg/s]")
    ax.set_ylabel("phi [deg]"); ax2.set_ylabel("omega [deg/s]", color="tab:orange")
    ax.set_title("StampFly flip v2: baseline (dynamic phi_brake, 3-tier thrust, policy C)")
    ax.legend(loc="upper left"); ax2.legend(loc="upper right")

    ax = axes[1]
    ax.plot(t, Tc, label="Tc (achieved) [N]")
    ax.plot(t, Tr, label="right pair [N]", alpha=0.7)
    ax.plot(t, Tl, label="left pair [N]", alpha=0.7)
    ax.set_ylabel("thrust [N]"); ax.legend(loc="upper right")

    ax = axes[2]
    ax.plot(t, tau_c * 1e3, color="tab:green", label="tau_c (achieved) [mN*m]")
    ax.set_ylabel("torque [mN*m]"); ax.legend(loc="upper right")

    ax = axes[3]
    ax.plot(t, z, label="z [m]")
    ax2 = ax.twinx(); ax2.plot(t, vz, color="tab:red", label="vz [m/s]")
    ax.set_ylabel("z [m]"); ax2.set_ylabel("vz [m/s]", color="tab:red")
    ax.legend(loc="upper left"); ax2.legend(loc="lower right")

    ax = axes[4]
    ax.plot(t, accel_g, color="tab:purple", label="IMU accel norm [G] (1cm offset)")
    ax.axhline(3.0, color="red", lw=0.8, ls="--", label="3G")
    ax.set_ylabel("accel [G]"); ax.set_xlabel("time [s]"); ax.legend(loc="upper right")

    phase_names = {PHASE_BOOST: "Boost", PHASE_SPIN: "Spin", PHASE_BRAKE_RATE: "Brake(rate)",
                   PHASE_BRAKE_ANGLE: "Brake(angle)", PHASE_RECOVER: "Recover"}
    colors = {PHASE_BOOST: "#dbeafe", PHASE_SPIN: "#fee2e2", PHASE_BRAKE_RATE: "#fef3c7",
              PHASE_BRAKE_ANGLE: "#fde68a", PHASE_RECOVER: "#d1fae5"}
    change_idx = np.where(np.diff(phase) != 0)[0] + 1
    bounds = [0] + list(change_idx) + [len(t) - 1]
    for ax in axes:
        for i in range(len(bounds) - 1):
            i0, i1 = bounds[i], bounds[i + 1]
            ax.axvspan(t[i0], t[min(i1, len(t) - 1)], color=colors.get(phase[i0], "white"), alpha=0.25, lw=0)

    fig.tight_layout(); fig.savefig(path, dpi=150); plt.close(fig)


def write_csv(cases_meta, results, path):
    fieldnames = ["group", "axis", "value", "label",
                  "omega_flip_dps", "T_lo_N", "T_spin_hi_N", "phi_a_deg",
                  "policy", "v_supply_V", "I_roll", "t_boost_s",
                  "omega_att_max_dps", "c_damp",
                  "thrust_limit_mode", "tau_c_limit_mode",
                  "spin_cmd_mode", "alpha_cmd_radss",
                  "tau_max_effective_Nm", "brake_trigger_deg_actual", "phi_b_at_trigger_deg",
                  "total_time_s", "max_omega_dps", "max_phi_deg",
                  "altitude_drop_m", "done_altitude_m", "attitude_residual_deg",
                  "motor_sat_fraction", "tracking_err_max_dps",
                  "max_accel_g", "time_above_3g_s", "failed"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for i, (axis, value, c) in enumerate(cases_meta):
            I_name = "Ixx" if abs(c.I_roll - IXX) < 1e-12 else ("Iyy" if abs(c.I_roll - IYY) < 1e-12 else f"{c.I_roll:.3e}")
            trig = results["brake_trigger_deg_actual"][i]
            w.writerow(dict(
                group=axis, axis=axis, value=value, label=c.label,
                omega_flip_dps=c.omega_flip_dps, T_lo_N=c.T_lo, T_spin_hi_N=c.T_spin_hi,
                phi_a_deg=c.phi_a_deg, policy=c.policy, v_supply_V=c.v_supply, I_roll=I_name,
                t_boost_s=c.t_boost, omega_att_max_dps=c.omega_att_max_dps, c_damp=c.c_damp,
                thrust_limit_mode=c.thrust_limit_mode, tau_c_limit_mode=c.tau_c_limit_mode,
                spin_cmd_mode=c.spin_cmd_mode, alpha_cmd_radss=c.alpha_cmd_radss,
                tau_max_effective_Nm=round(float(results["tau_max_effective"][i]), 6),
                brake_trigger_deg_actual=("" if np.isnan(trig) else round(float(trig), 2)),
                phi_b_at_trigger_deg=("" if np.isnan(trig) else round(float(trig) - 20.0, 2)),
                total_time_s=("" if np.isnan(results["done_time"][i]) else round(float(results["done_time"][i]), 4)),
                max_omega_dps=round(float(results["max_omega_dps"][i]), 1),
                max_phi_deg=round(float(results["max_phi_deg"][i]), 1),
                altitude_drop_m=round(float(results["altitude_drop"][i]), 4),
                done_altitude_m=("" if np.isnan(results["done_z"][i]) else round(float(results["done_z"][i]), 4)),
                attitude_residual_deg=("" if np.isnan(results["done_phierr"][i]) else round(float(results["done_phierr"][i]), 2)),
                motor_sat_fraction=round(float(results["sat_fraction"][i]), 3),
                tracking_err_max_dps=round(float(results["tracking_err_max_dps"][i]), 1),
                max_accel_g=round(float(results["max_accel_g"][i]), 3),
                time_above_3g_s=round(float(results["time_above_3g"][i]), 4),
                failed=bool(results["failed"][i]),
            ))


def main():
    print("=" * 78)
    print("flip_sim_v2: dynamic phi_brake / 3-tier thrust / policy C / IMU accel")
    print("=" * 78)

    baseline_case = replace(BASE, label="baseline")
    base_res = run_batch([baseline_case], store_traj=True)
    plot_path = os.path.join(OUT_DIR, "flip_baseline_timeseries_v2.png")
    plot_baseline(base_res["traj"], plot_path)
    print(f"\nBaseline plot saved: {plot_path}")
    print(f"Baseline: total_time={base_res['done_time'][0]:.4f}s "
          f"max_omega={base_res['max_omega_dps'][0]:.1f}dps "
          f"max_phi={base_res['max_phi_deg'][0]:.1f}deg "
          f"altitude_drop={base_res['altitude_drop'][0]*100:.2f}cm "
          f"done_altitude={base_res['done_z'][0]*100:.2f}cm "
          f"attitude_residual={base_res['done_phierr'][0]:.2f}deg "
          f"sat_fraction={base_res['sat_fraction'][0]*100:.1f}% "
          f"brake_trigger={base_res['brake_trigger_deg_actual'][0]:.1f}deg "
          f"max_accel={base_res['max_accel_g'][0]:.2f}G "
          f"time_above_3G={base_res['time_above_3g'][0]*1000:.1f}ms "
          f"failed={bool(base_res['failed'][0])}")

    cases_meta = make_cases()
    cases_only = [c for (_, _, c) in cases_meta]
    print(f"\nRunning v2 sweep with {len(cases_only)} cases ...")
    results = run_batch(cases_only, store_traj=False)

    csv_path = os.path.join(OUT_DIR, "flip_sweep_results_v2.csv")
    write_csv(cases_meta, results, csv_path)
    print(f"Sweep CSV saved: {csv_path}")

    summary_path = os.path.join(OUT_DIR, "flip_sweep_summary_v2.txt")
    lines = []
    lines.append(f"Total cases: {len(cases_only)}")
    n_failed = int(np.sum(results["failed"]))
    lines.append(f"Failed cases: {n_failed} / {len(cases_only)}")
    lines.append("")
    lines.append("Failed case labels:")
    for i, (axis, value, c) in enumerate(cases_meta):
        if results["failed"][i]:
            lines.append(f"  {c.label}: max_omega={results['max_omega_dps'][i]:.0f}dps "
                          f"max_phi={results['max_phi_deg'][i]:.0f}deg sat={results['sat_fraction'][i]*100:.0f}%")
    lines.append("")
    lines.append("All cases (axis=value): time[s] drop[cm] end_resid[deg] sat[%] track_err[dps] max_accel[G] t>3G[ms] trigger[deg] failed")
    for i, (axis, value, c) in enumerate(cases_meta):
        t_done = results['done_time'][i]
        time_str = "FAIL" if np.isnan(t_done) else f"{t_done:.3f}"
        phierr = results['done_phierr'][i]
        resid_str = "nan" if np.isnan(phierr) else f"{phierr:.1f}"
        lines.append(f"  {c.label:28s} "
                      f"time={time_str:>6s} "
                      f"drop={results['altitude_drop'][i]*100:6.2f} "
                      f"resid={resid_str:>6s} "
                      f"sat={results['sat_fraction'][i]*100:5.1f} "
                      f"track_err={results['tracking_err_max_dps'][i]:6.0f} "
                      f"max_accel={results['max_accel_g'][i]:5.2f} "
                      f"t>3G={results['time_above_3g'][i]*1000:6.1f} "
                      f"trigger={results['brake_trigger_deg_actual'][i]:6.1f} "
                      f"failed={results['failed'][i]}")

    lines.append("")
    lines.append("omega_flip x inertia grid (Ixx vs Iyy):")
    for i, (axis, value, c) in enumerate(cases_meta):
        if axis == "omega_flip_x_inertia":
            I_name = "Ixx" if abs(c.I_roll - IXX) < 1e-12 else "Iyy"
            t_done = results['done_time'][i]
            lines.append(f"  wf={c.omega_flip_dps:.0f}dps I={I_name}: "
                          f"time={'FAIL' if np.isnan(t_done) else f'{t_done:.3f}s'} "
                          f"drop={results['altitude_drop'][i]*100:.2f}cm "
                          f"max_accel={results['max_accel_g'][i]:.2f}G "
                          f"trigger={results['brake_trigger_deg_actual'][i]:.1f}deg "
                          f"failed={results['failed'][i]}")

    lines.append("")
    lines.append("policy A/B/C comparison (all other params at baseline):")
    for i, (axis, value, c) in enumerate(cases_meta):
        if axis == "policy" or c.label == "baseline":
            t_done = results['done_time'][i]
            lines.append(f"  policy={c.policy}: time={'FAIL' if np.isnan(t_done) else f'{t_done:.3f}s'} "
                          f"drop={results['altitude_drop'][i]*100:.2f}cm sat={results['sat_fraction'][i]*100:.0f}% "
                          f"failed={results['failed'][i]}")

    summary_text = "\n".join(lines)
    with open(summary_path, "w") as f:
        f.write(summary_text + "\n")
    print(f"\nSummary saved: {summary_path}\n")
    print(summary_text)


if __name__ == "__main__":
    main()
