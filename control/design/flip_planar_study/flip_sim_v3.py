#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StampFly roll-flip maneuver -- v3 (final confirmation sweep).
StampFly ロール宙返り（フリップ）制御案 -- v3（最終確認掃引）。

Standalone analysis script; never modifies the repository. Reuses v1's
physics/motor primitives (via v2) and v2's control/phase state machine
(dynamic phi_brake, 3-tier collective, policy A/B/C mixer), unmodified.
v3 adds exactly what the coordinator asked for in the final review round:
リポジトリは変更しない単独スクリプト。v1の物理/モータ関数（v2経由）と、
v2の制御/フェーズ状態機械（動的phi_brake、3段階推力、方針A/B/Cミキサー）を
無改変のまま再利用する。v3では最終レビューで指示された以下を追加する:

  1. 設計制約 |omega|max <= 1700 deg/s を全ケースについて判定・出力する
     （打ち切りしきい値1800度/secと、ジャイロ計測レンジ2000dpsへの余裕）。
  2. 推奨組合せ確定用の全組合せ掃引: alpha_cmd=300 rad/s^2 (ramp),
     T_spin_hi=0.50N, t_boost=0.15s, 方針Cを固定し、
     omega_flip in {1400,1500,1600} x T_lo in {0, 0.03} x
     inertia in {Ixx, Iyy} x v_supply in {3.7, 3.5} の24ケース全組合せ。
  3. 追加出力: (a) 開始高度からの最大上昇 [m]、(b) 最大落ち込み [m]、
     (c) 最低点からRecover完了(v_z>=0)までの時間。
  4. T_boost = 0.9 x 電気的上限（Boost/Recover両方）の変種を、
     omega_flip=1500, T_lo=0.03, 3.7V, Ixx/Iyyの2ケースで比較。

Outputs (v3-suffixed; v2's own files are untouched):
    flip_sweep_results_v3.csv
    flip_sweep_summary_v3.txt
"""

import os
import csv
import numpy as np

import flip_sim_v2 as v2   # reuse v2's control/phase logic + v1's physics (both untouched)

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# Re-exported constants/primitives.
MASS, IXX, IYY, ARM_D, G = v2.MASS, v2.IXX, v2.IYY, v2.ARM_D, v2.G
ETA, CT = v2.ETA, v2.CT
HOVER_THRUST_TOTAL, HOVER_THRUST_PER_MOTOR = v2.HOVER_THRUST_TOTAL, v2.HOVER_THRUST_PER_MOTOR
THRUST_MODES, TAU_C_LIMITS, MOTOR_T_MIN = v2.THRUST_MODES, v2.TAU_C_LIMITS, v2.MOTOR_T_MIN
motor_steady_state_omega = v2.motor_steady_state_omega
thrust_from_omega = v2.thrust_from_omega
voltage_for_thrust = v2.voltage_for_thrust
rk4_step = v2.rk4_step
deg2rad, rad2deg = v2.deg2rad, v2.rad2deg
DT_CTRL, DT_PHYS, N_SUB, T_MAX = v2.DT_CTRL, v2.DT_PHYS, v2.N_SUB, v2.T_MAX
KP_RATE = v2.KP_RATE
T_LAG, ALPHA_BRAKE_FRAC = v2.T_LAG, v2.ALPHA_BRAKE_FRAC
PHI_ERR_ENTER_RECOVER_DEG, OMEGA_ENTER_RECOVER_DPS = v2.PHI_ERR_ENTER_RECOVER_DEG, v2.OMEGA_ENTER_RECOVER_DPS
PHI_SAT_DESIGN_DEG = v2.PHI_SAT_DESIGN_DEG
BRAKE_RATE_EXIT_PHI_LOOSE_DEG = v2.BRAKE_RATE_EXIT_PHI_LOOSE_DEG
BRAKE_RATE_EXIT_OMEGA_LOOSE_DPS = v2.BRAKE_RATE_EXIT_OMEGA_LOOSE_DPS
BRAKE_RATE_EXIT_PHI_HARD_DEG = v2.BRAKE_RATE_EXIT_PHI_HARD_DEG
R_IMU, G_MPS2 = v2.R_IMU, v2.G_MPS2
PHASE_BOOST, PHASE_SPIN, PHASE_BRAKE_RATE, PHASE_BRAKE_ANGLE, PHASE_RECOVER, \
    PHASE_DONE, PHASE_FAILED = v2.PHASE_BOOST, v2.PHASE_SPIN, v2.PHASE_BRAKE_RATE, \
    v2.PHASE_BRAKE_ANGLE, v2.PHASE_RECOVER, v2.PHASE_DONE, v2.PHASE_FAILED
POLICY_CODE = v2.POLICY_CODE
mix_motors_v2 = v2.mix_motors_v2

OMEGA_MAX_LIMIT_DPS = 1700.0   # design constraint (coordinator, this round)

from dataclasses import dataclass, replace


@dataclass
class CaseV3:
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
    spin_cmd_mode: str = "step"
    alpha_cmd_radss: float = 300.0
    boost_thrust_mode: str = "firmware"   # "firmware" (mechanical cap, as v1/v2) | "elec90" (0.9 x electrical ceiling)


# Fixed "recommended combination" settings from the v2 review, per this round's instruction 2.
BASE = CaseV3(label="baseline", spin_cmd_mode="ramp", alpha_cmd_radss=300.0,
              T_spin_hi=0.50, t_boost=0.150, policy="C")


def run_batch(cases, dt_ctrl=DT_CTRL, dt_phys=DT_PHYS, t_max=T_MAX):
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
    is_elec90 = np.array([c.boost_thrust_mode == "elec90" for c in cases], dtype=bool)

    motor_t_max = np.array([THRUST_MODES[c.thrust_limit_mode]["motor_t_max"] for c in cases], dtype=float)
    t_boost_N_firmware = np.array([THRUST_MODES[c.thrust_limit_mode]["t_boost"] for c in cases], dtype=float)
    tau_c_limit = np.array([TAU_C_LIMITS[c.tau_c_limit_mode] for c in cases], dtype=float)

    t_elec_max = thrust_from_omega(motor_steady_state_omega(v_supply))
    effective_t_max = np.minimum(motor_t_max, t_elec_max)

    # T_boost tier: "firmware" (mechanical cap, unchanged from v1/v2) or
    # "elec90" = 0.9 x (4 x electrical ceiling at this case's v_supply) --
    # deliberately leaves 10% margin below what the motors can physically
    # reach, so Boost/Recover retain differential-torque headroom (the
    # coordinator's point: what does that headroom cost in climb thrust?).
    # T_boost段: "firmware"（機械上限、v1/v2のまま）または
    # "elec90" = 0.9 x (4 x この電圧での電気的上限) -- 意図的に電気的上限の
    # 10%下に留め、Boost/Recoverに差動トルクの余裕を残す（その余裕の代償=
    # 上昇推力の減少を見るのが目的）。
    t_boost_N_elec90 = 0.9 * 4.0 * t_elec_max
    t_boost_N = np.where(is_elec90, t_boost_N_elec90, t_boost_N_firmware)

    geometric_tau_max = 2.0 * ARM_D * effective_t_max
    tau_max_effective = np.minimum(geometric_tau_max, tau_c_limit)
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
    max_z = np.zeros(N)
    time_at_min_z = np.zeros(N)
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

    for step in range(n_ctrl_steps):
        active = (phase != PHASE_DONE) & (phase != PHASE_FAILED)
        if not np.any(active):
            break
        t_now = step * dt_ctrl
        phi_deg = rad2deg(phi)
        omega_dps = rad2deg(omega_body)
        phi_err_deg = phi_deg - 360.0

        omega_for_brake = np.abs(omega_body)
        delta_phi_brake_deg = rad2deg(omega_for_brake ** 2 / (2.0 * alpha_brake_source)
                                       + omega_for_brake * T_LAG)
        phi_brake_now = 360.0 - delta_phi_brake_deg
        phi_b_now = phi_brake_now - 20.0

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

        ramp_up_val = np.minimum(omega_flip_rad, omega_cmd_state + alpha_cmd * dt_ctrl)
        ramp_down_val = np.maximum(0.0, omega_cmd_state - alpha_cmd * dt_ctrl)
        new_ramp_state = np.where(phase == PHASE_SPIN, ramp_up_val,
                          np.where(phase == PHASE_BRAKE_RATE, ramp_down_val, omega_cmd_state))
        omega_cmd_state = np.where(is_ramp, new_ramp_state, omega_cmd_state)

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
            max_accel_g = np.maximum(max_accel_g, np.where(active, accel_g_now, max_accel_g))
            time_above_3g += np.where(active & (accel_g_now > 3.0), dt_phys, 0.0)

        is_new_min = active & (z < min_z)
        time_at_min_z = np.where(is_new_min, t_now + dt_ctrl, time_at_min_z)
        min_z = np.minimum(min_z, np.where(active, z, min_z))
        max_z = np.maximum(max_z, np.where(active, z, max_z))
        max_phi_deg = np.maximum(max_phi_deg, np.where(active, rad2deg(phi), max_phi_deg))
        max_omega_dps = np.maximum(max_omega_dps, np.where(active, np.abs(rad2deg(omega_body)), max_omega_dps))

    still_running = (phase != PHASE_DONE) & (phase != PHASE_FAILED)
    failed = failed | still_running
    phase = np.where(still_running, PHASE_FAILED, phase)

    sat_fraction = np.divide(sat_count, np.maximum(ctrl_count, 1.0))
    altitude_drop = -np.minimum(min_z, 0.0)
    max_climb = np.maximum(max_z, 0.0)
    recover_duration_from_min = done_time - time_at_min_z   # NaN where done_time is NaN (failed)

    return dict(
        brake_trigger_deg_actual=brake_trigger_deg_actual,
        tau_max_effective=tau_max_effective,
        done_time=done_time, done_z=done_z, done_phierr=done_phierr,
        max_phi_deg=max_phi_deg, max_omega_dps=max_omega_dps,
        min_z=min_z, altitude_drop=altitude_drop, max_climb=max_climb,
        time_at_min_z=time_at_min_z, recover_duration_from_min=recover_duration_from_min,
        sat_fraction=sat_fraction, failed=failed, tracking_err_max_dps=tracking_err_max_dps,
        max_accel_g=max_accel_g, time_above_3g=time_above_3g,
        t_boost_N_used=t_boost_N,
    )


# =====================================================================
# Case construction
# =====================================================================

def make_cases_main24():
    """Item 2: full factorial, 3 x 2 x 2 x 2 = 24 cases, all other params
    fixed at the confirmed recommended combination (BASE)."""
    cases = []
    for wf in [1400.0, 1500.0, 1600.0]:
        for tlo in [0.0, 0.03]:
            for ir_name, ir in [("Ixx", IXX), ("Iyy", IYY)]:
                for vs in [3.7, 3.5]:
                    lbl = f"wf{int(wf)}_Tlo{tlo}_{ir_name}_{vs}V"
                    c = replace(BASE, label=lbl, omega_flip_dps=wf, T_lo=tlo, I_roll=ir, v_supply=vs)
                    cases.append(("main24", lbl, c))
    return cases


def make_cases_elec90():
    """Item 4: T_boost = 0.9 x electrical ceiling, vs the firmware-cap
    baseline, for omega_flip=1500, T_lo=0.03, 3.7V, Ixx/Iyy (2+2 cases)."""
    cases = []
    for ir_name, ir in [("Ixx", IXX), ("Iyy", IYY)]:
        lbl_fw = f"elec90cmp_wf1500_Tlo0.03_{ir_name}_3.7V_firmware"
        c_fw = replace(BASE, label=lbl_fw, omega_flip_dps=1500.0, T_lo=0.03, I_roll=ir,
                       v_supply=3.7, boost_thrust_mode="firmware")
        cases.append(("elec90_compare", lbl_fw, c_fw))
        lbl_e9 = f"elec90cmp_wf1500_Tlo0.03_{ir_name}_3.7V_elec90"
        c_e9 = replace(BASE, label=lbl_e9, omega_flip_dps=1500.0, T_lo=0.03, I_roll=ir,
                       v_supply=3.7, boost_thrust_mode="elec90")
        cases.append(("elec90_compare", lbl_e9, c_e9))
    return cases


# =====================================================================
# CSV / summary / main
# =====================================================================

def write_csv(cases_meta, results, path):
    fieldnames = ["group", "label", "omega_flip_dps", "T_lo_N", "T_spin_hi_N",
                  "policy", "v_supply_V", "I_roll", "t_boost_s", "boost_thrust_mode",
                  "t_boost_N_used", "spin_cmd_mode", "alpha_cmd_radss",
                  "max_omega_dps", "constraint_1700_violated",
                  "max_climb_m", "altitude_drop_m", "time_min_to_recover_s",
                  "total_time_s", "done_altitude_m", "attitude_residual_deg",
                  "motor_sat_fraction", "tracking_err_max_dps",
                  "max_accel_g", "time_above_3g_s", "brake_trigger_deg_actual", "failed"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for i, (group, label, c) in enumerate(cases_meta):
            I_name = "Ixx" if abs(c.I_roll - IXX) < 1e-12 else ("Iyy" if abs(c.I_roll - IYY) < 1e-12 else f"{c.I_roll:.3e}")
            trig = results["brake_trigger_deg_actual"][i]
            rdur = results["recover_duration_from_min"][i]
            w.writerow(dict(
                group=group, label=label, omega_flip_dps=c.omega_flip_dps, T_lo_N=c.T_lo,
                T_spin_hi_N=c.T_spin_hi, policy=c.policy, v_supply_V=c.v_supply, I_roll=I_name,
                t_boost_s=c.t_boost, boost_thrust_mode=c.boost_thrust_mode,
                t_boost_N_used=round(float(results["t_boost_N_used"][i]), 4),
                spin_cmd_mode=c.spin_cmd_mode, alpha_cmd_radss=c.alpha_cmd_radss,
                max_omega_dps=round(float(results["max_omega_dps"][i]), 1),
                constraint_1700_violated=bool(results["max_omega_dps"][i] > OMEGA_MAX_LIMIT_DPS),
                max_climb_m=round(float(results["max_climb"][i]), 4),
                altitude_drop_m=round(float(results["altitude_drop"][i]), 4),
                time_min_to_recover_s=("" if np.isnan(rdur) else round(float(rdur), 4)),
                total_time_s=("" if np.isnan(results["done_time"][i]) else round(float(results["done_time"][i]), 4)),
                done_altitude_m=("" if np.isnan(results["done_z"][i]) else round(float(results["done_z"][i]), 4)),
                attitude_residual_deg=("" if np.isnan(results["done_phierr"][i]) else round(float(results["done_phierr"][i]), 2)),
                motor_sat_fraction=round(float(results["sat_fraction"][i]), 3),
                tracking_err_max_dps=round(float(results["tracking_err_max_dps"][i]), 1),
                max_accel_g=round(float(results["max_accel_g"][i]), 3),
                time_above_3g_s=round(float(results["time_above_3g"][i]), 4),
                brake_trigger_deg_actual=("" if np.isnan(trig) else round(float(trig), 2)),
                failed=bool(results["failed"][i]),
            ))


def main():
    print("=" * 78)
    print("flip_sim_v3: omega constraint check + 24-case confirmation grid + elec90 variant")
    print("=" * 78)

    cases_meta = make_cases_main24() + make_cases_elec90()
    cases_only = [c for (_, _, c) in cases_meta]
    print(f"Running {len(cases_only)} cases ({len(make_cases_main24())} main24 + "
          f"{len(make_cases_elec90())} elec90_compare) ...")
    results = run_batch(cases_only)

    csv_path = os.path.join(OUT_DIR, "flip_sweep_results_v3.csv")
    write_csv(cases_meta, results, csv_path)
    print(f"CSV saved: {csv_path}")

    lines = []
    lines.append(f"Total cases: {len(cases_only)}")
    n_failed = int(np.sum(results["failed"]))
    n_violate = int(np.sum(results["max_omega_dps"] > OMEGA_MAX_LIMIT_DPS))
    lines.append(f"Failed: {n_failed} / {len(cases_only)}   "
                 f"omega>1700dps constraint violated: {n_violate} / {len(cases_only)}")
    lines.append("")

    lines.append("=== main24 (omega_flip x T_lo x inertia x v_supply, full factorial) ===")
    for i, (group, label, c) in enumerate(cases_meta):
        if group != "main24":
            continue
        I_name = "Ixx" if abs(c.I_roll - IXX) < 1e-12 else "Iyy"
        viol = "VIOLATE" if results["max_omega_dps"][i] > OMEGA_MAX_LIMIT_DPS else "ok"
        t_done = results['done_time'][i]
        rdur = results['recover_duration_from_min'][i]
        lines.append(
            f"  wf={c.omega_flip_dps:.0f} Tlo={c.T_lo:.2f} I={I_name} Vs={c.v_supply}V | "
            f"max_omega={results['max_omega_dps'][i]:7.1f}dps[{viol}] "
            f"climb={results['max_climb'][i]*100:6.2f}cm drop={results['altitude_drop'][i]*100:6.2f}cm "
            f"t_min->recover={('n/a' if np.isnan(rdur) else f'{rdur:.3f}s'):>7s} "
            f"time={('FAIL' if np.isnan(t_done) else f'{t_done:.3f}s'):>7s} "
            f"resid={results['done_phierr'][i]:6.2f}deg sat={results['sat_fraction'][i]*100:4.0f}% "
            f"failed={results['failed'][i]}")

    lines.append("")
    lines.append("=== elec90_compare (T_boost=firmware-cap vs 0.9x electrical ceiling, "
                 "wf=1500 Tlo=0.03 3.7V Ixx/Iyy) ===")
    for i, (group, label, c) in enumerate(cases_meta):
        if group != "elec90_compare":
            continue
        I_name = "Ixx" if abs(c.I_roll - IXX) < 1e-12 else "Iyy"
        t_done = results['done_time'][i]
        lines.append(
            f"  I={I_name} boost_mode={c.boost_thrust_mode:9s} T_boost_used={results['t_boost_N_used'][i]:.4f}N | "
            f"max_omega={results['max_omega_dps'][i]:7.1f}dps "
            f"climb={results['max_climb'][i]*100:6.2f}cm drop={results['altitude_drop'][i]*100:6.2f}cm "
            f"time={('FAIL' if np.isnan(t_done) else f'{t_done:.3f}s'):>7s} "
            f"resid={results['done_phierr'][i]:6.2f}deg sat={results['sat_fraction'][i]*100:4.0f}% "
            f"failed={results['failed'][i]}")

    lines.append("")
    lines.append("=== best (min altitude_drop) among constraint-satisfying (omega<=1700dps) "
                 "main24 cases, split by roll/pitch x voltage ===")
    for I_name, ir in [("Ixx(roll)", IXX), ("Iyy(pitch)", IYY)]:
        for vs in [3.7, 3.5]:
            best_idx = None
            best_drop = None
            for i, (group, label, c) in enumerate(cases_meta):
                if group != "main24":
                    continue
                if abs(c.I_roll - ir) > 1e-12 or abs(c.v_supply - vs) > 1e-9:
                    continue
                if results["max_omega_dps"][i] > OMEGA_MAX_LIMIT_DPS:
                    continue
                if results["failed"][i]:
                    continue
                d = results["altitude_drop"][i]
                if best_drop is None or d < best_drop:
                    best_drop = d
                    best_idx = i
            if best_idx is None:
                lines.append(f"  {I_name} @ {vs}V: NO constraint-satisfying case found")
            else:
                group, label, c = cases_meta[best_idx]
                lines.append(f"  {I_name} @ {vs}V: wf={c.omega_flip_dps:.0f}dps T_lo={c.T_lo:.2f}N -> "
                             f"drop={results['altitude_drop'][best_idx]*100:.2f}cm "
                             f"climb={results['max_climb'][best_idx]*100:.2f}cm "
                             f"max_omega={results['max_omega_dps'][best_idx]:.1f}dps "
                             f"time={results['done_time'][best_idx]:.3f}s "
                             f"resid={results['done_phierr'][best_idx]:.2f}deg "
                             f"sat={results['sat_fraction'][best_idx]*100:.0f}%")

    summary_text = "\n".join(lines)
    summary_path = os.path.join(OUT_DIR, "flip_sweep_summary_v3.txt")
    with open(summary_path, "w") as f:
        f.write(summary_text + "\n")
    print(f"\nSummary saved: {summary_path}\n")
    print(summary_text)


if __name__ == "__main__":
    main()
