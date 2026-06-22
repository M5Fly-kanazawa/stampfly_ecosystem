#!/usr/bin/env python3
"""
poshold_analysis.py — POS_HOLD / ALT_HOLD flight-log diagnosis.
POS_HOLD / ALT_HOLD 飛行ログ診断。

Answers the bring-up question "why does POS_HOLD not hold its point?" by
quantifying, over the actual flight window, the chain POS_HOLD depends on:

  optical flow (PMW3901) --> ESKF horizontal velocity --> dead-reckoned position
                                                       --> position cascade --> tilt cmd

決定木:
  1. フロー品質 (SQUAL / dx,dy)   … 速度推定の入力が健全か
  2. 速度推定 vel_x/y のノイズ・バイアス
  3. 位置ドリフト pos vs pos_sp   … 真の保持性能
  4. 推定姿勢(quat) vs angle_ref vs ジャイロ積分 … 見かけ重力汚染の指紋
  5. angle_ref の 10deg 飽和
  6. accel_raw 振動 (n1/n2)
  7. ToF 高さ (フロー速度スケール)

Usage:
  python3 analysis/scripts/poshold_analysis.py <log.jsonl> [out_dir]
  python3 analysis/scripts/poshold_analysis.py --compare <pos.jsonl> <alt.jsonl> [out_dir]
"""
import sys, os, json, math
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

R2D = 180.0 / math.pi
G = 9.80665
FLOW_MIN_SQUAL = 10          # firmware gate (eskf_core.hpp)
MAX_POS_TILT_DEG = 10.0      # firmware max_pos_tilt_ (0.1745 rad)
MODE_NAME = {0: "ACRO", 1: "STABILIZE", 2: "ALT_HOLD", 3: "POS_HOLD"}
STATE_NAME = {0: "INIT", 1: "IDLE_GROUND", 2: "IDLE_HELD", 3: "ARMED_GROUND",
              4: "TAKEOFF", 5: "FLYING", 6: "LANDING"}


def load(path):
    """Group JSONL records by their "id" field."""
    S = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except Exception:
                continue
            S.setdefault(d.get("id", "?"), []).append(d)
    return S


def quat_to_euler(q):
    """[w,x,y,z] -> (roll, pitch, yaw) [rad], aerospace ZYX."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def arr(records, key):
    return np.array([r[key] for r in records], dtype=float)


def ts_s(records, t0):
    return np.array([(r["ts"] - t0) / 1e6 for r in records], dtype=float)


def analyze(path):
    S = load(path)
    name = os.path.basename(path)
    if "imu" not in S or "ctrl_ref" not in S:
        print(f"[{name}] missing imu/ctrl_ref records, skip")
        return None
    t0 = S["imu"][0]["ts"]

    # ---- IMU (400 Hz): attitude, raw accel (vibration), biases ----
    t_imu = ts_s(S["imu"], t0)
    quat = arr(S["imu"], "quat")
    euler = np.array([quat_to_euler(q) for q in quat])      # roll,pitch,yaw [rad]
    gyro = arr(S["imu"], "gyro")
    accel_raw = arr(S["imu"], "accel_raw")
    abias = arr(S["imu"], "accel_bias")

    # ---- pos/vel (400 Hz): ESKF horizontal estimate ----
    t_pv = ts_s(S["posvel"], t0)
    pos = arr(S["posvel"], "pos")
    vel = arr(S["posvel"], "vel")

    # ---- ctrl_ref (50 Hz): mode, tilt cmd, setpoint, thrust, duty ----
    t_cr = ts_s(S["ctrl_ref"], t0)
    mode = arr(S["ctrl_ref"], "mode")
    angle_ref = arr(S["ctrl_ref"], "angle_ref")             # roll,pitch [rad]
    thrust = arr(S["ctrl_ref"], "total_thrust")
    duty = arr(S["ctrl_ref"], "motor_duty")
    pos_sp = arr(S["ctrl_ref"], "pos_sp")
    alt_sp = arr(S["ctrl_ref"], "alt_sp")

    # ---- sticks (ctrl): confirm hands-off ----
    have_ctrl = "ctrl" in S
    if have_ctrl:
        t_ct = ts_s(S["ctrl"], t0)
        st_roll = arr(S["ctrl"], "roll")
        st_pitch = arr(S["ctrl"], "pitch")
        st_yaw = arr(S["ctrl"], "yaw")
        st_thr = arr(S["ctrl"], "throttle")

    # ---- flow (PMW3901): quality + raw motion ----
    have_flow = "flow" in S
    if have_flow:
        t_fl = ts_s(S["flow"], t0)
        fl_q = arr(S["flow"], "quality")
        fl_dx = arr(S["flow"], "dx")
        fl_dy = arr(S["flow"], "dy")

    # ---- ToF bottom: altitude source for flow scaling ----
    have_tof = "tof_b" in S
    if have_tof:
        t_tof = ts_s(S["tof_b"], t0)
        tof_d = arr(S["tof_b"], "distance")
        tof_st = arr(S["tof_b"], "status")

    # ---- status (1 Hz): flight_state, voltage ----
    fstate = arr(S["status"], "flight_state") if "status" in S else np.array([])
    volt = arr(S["status"], "voltage") if "status" in S else np.array([])

    # =====================================================================
    # Flight window: thrust > 0 (motors live). Takeoff -> last live sample.
    # =====================================================================
    live = thrust > 0.02
    if not live.any():
        live = duty.max(axis=1) > 0.02
    if not live.any():
        print(f"[{name}] never spun motors (thrust/duty all ~0)")
        return None
    i_to = int(np.argmax(live))                  # first live
    i_end = len(live) - 1 - int(np.argmax(live[::-1]))  # last live
    t_to, t_end = t_cr[i_to], t_cr[i_end]
    dur = t_end - t_to

    flight_mode = int(np.median(mode[i_to:i_end + 1])) if i_end > i_to else int(mode[i_to])

    def win(t, lo=t_to, hi=t_end):
        return (t >= lo) & (t <= hi)

    # mask helpers on each stream's own grid
    m_pv = win(t_pv)
    m_imu = win(t_imu)
    m_cr = win(t_cr)

    # Early hold window: first 2.5 s after takeoff (before wall impact) —
    # the cleanest view of the hold attempt. 入口の保持挙動。
    t_e0, t_e1 = t_to, min(t_to + 2.5, t_end)
    me_pv = win(t_pv, t_e0, t_e1)
    me_imu = win(t_imu, t_e0, t_e1)

    M = {"log": name, "flight_mode": MODE_NAME.get(flight_mode, flight_mode),
         "takeoff_s": round(t_to, 2), "end_s": round(t_end, 2),
         "duration_s": round(dur, 2)}

    # ---- horizontal drift: pos vs setpoint (interp setpoint onto pv grid) ----
    psp_x = np.interp(t_pv, t_cr, pos_sp[:, 0])
    psp_y = np.interp(t_pv, t_cr, pos_sp[:, 1])
    drift = np.hypot(pos[:, 0] - psp_x, pos[:, 1] - psp_y)
    if m_pv.any():
        M["drift_max_m"] = round(float(drift[m_pv].max()), 3)
        M["drift_end_m"] = round(float(drift[m_pv][-1]), 3)
    spd = np.hypot(vel[:, 0], vel[:, 1])
    if m_pv.any():
        M["horiz_speed_max_ms"] = round(float(spd[m_pv].max()), 3)

    # ---- velocity noise/bias in the early hold window ----
    if me_pv.any():
        M["vel_x_mean_ms"] = round(float(vel[me_pv, 0].mean()), 3)
        M["vel_y_mean_ms"] = round(float(vel[me_pv, 1].mean()), 3)
        M["vel_x_std_ms"] = round(float(vel[me_pv, 0].std()), 3)
        M["vel_y_std_ms"] = round(float(vel[me_pv, 1].std()), 3)

    # ---- estimated attitude (deg) + sticking signature ----
    roll_d = euler[:, 0] * R2D
    pitch_d = euler[:, 1] * R2D
    if m_imu.any():
        M["est_roll_range_deg"] = [round(float(roll_d[m_imu].min()), 1),
                                   round(float(roll_d[m_imu].max()), 1)]
        M["est_pitch_range_deg"] = [round(float(pitch_d[m_imu].min()), 1),
                                    round(float(pitch_d[m_imu].max()), 1)]
        M["est_tilt_max_deg"] = round(float(np.hypot(roll_d, pitch_d)[m_imu].max()), 1)

    # ---- angle_ref (tilt command) saturation ----
    ar_r = angle_ref[:, 0] * R2D
    ar_p = angle_ref[:, 1] * R2D
    if m_cr.any():
        ar_mag = np.hypot(ar_r, ar_p)[m_cr]
        M["angle_ref_max_deg"] = round(float(ar_mag.max()), 1)
        M["angle_ref_sat_pct"] = round(float((ar_mag >= MAX_POS_TILT_DEG - 0.3).mean() * 100), 1)

    # ---- gyro-integrated tilt (independent attitude reference) ----
    # Integrate body roll/pitch rate from takeoff; over a few seconds this is a
    # decent independent check of whether the ESKF attitude is being pulled wrong
    # by the apparent-gravity (specific-force) contamination.
    if m_imu.any():
        idx = np.where(m_imu)[0]
        ti = t_imu[idx]
        gx = gyro[idx, 0]
        gy = gyro[idx, 1]
        r0, p0 = euler[idx[0], 0], euler[idx[0], 1]
        gi_roll = r0 + np.concatenate([[0], np.cumsum(0.5 * (gx[1:] + gx[:-1]) * np.diff(ti))])
        gi_pitch = p0 + np.concatenate([[0], np.cumsum(0.5 * (gy[1:] + gy[:-1]) * np.diff(ti))])
        # divergence between ESKF attitude and gyro-integrated attitude
        d_roll = (roll_d[idx] - gi_roll * R2D)
        d_pitch = (pitch_d[idx] - gi_pitch * R2D)
        M["eskf_vs_gyro_roll_maxdiff_deg"] = round(float(np.abs(d_roll).max()), 1)
        M["eskf_vs_gyro_pitch_maxdiff_deg"] = round(float(np.abs(d_pitch).max()), 1)

    # ---- flow quality ----
    if have_flow:
        mf = win(t_fl)
        if mf.any():
            q = fl_q[mf]
            M["flow_squal_min"] = int(q.min())
            M["flow_squal_med"] = int(np.median(q))
            M["flow_squal_below_gate_pct"] = round(float((q < FLOW_MIN_SQUAL).mean() * 100), 1)
            mot = (np.abs(fl_dx[mf]) + np.abs(fl_dy[mf]))
            M["flow_zero_motion_pct"] = round(float((mot == 0).mean() * 100), 1)
            M["flow_dx_absmax"] = int(np.abs(fl_dx[mf]).max())
            M["flow_dy_absmax"] = int(np.abs(fl_dy[mf]).max())

    # ---- vibration: raw accel std in early window (per axis) ----
    if me_imu.any():
        for k, ax in enumerate("xyz"):
            M[f"accel_raw_std_{ax}"] = round(float(accel_raw[me_imu, k].std()), 3)
        # crude dominant vibration freq on z (most thrust-coupled)
        az = accel_raw[me_imu, 2]
        az = az - az.mean()
        if len(az) > 32:
            dt = np.median(np.diff(t_imu[me_imu]))
            ffreq = np.fft.rfftfreq(len(az), dt)
            psd = np.abs(np.fft.rfft(az)) ** 2
            psd[0] = 0
            M["accel_z_dom_freq_hz"] = round(float(ffreq[int(np.argmax(psd))]), 1)

    # ---- accel bias drift over flight ----
    if m_imu.any():
        M["accel_bias_x_range"] = [round(float(abias[m_imu, 0].min()), 3),
                                   round(float(abias[m_imu, 0].max()), 3)]
        M["accel_bias_y_range"] = [round(float(abias[m_imu, 1].min()), 3),
                                   round(float(abias[m_imu, 1].max()), 3)]

    # ---- ToF altitude ----
    if have_tof:
        mt = win(t_tof)
        if mt.any():
            M["tof_dist_range_m"] = [round(float(tof_d[mt].min()), 3),
                                     round(float(tof_d[mt].max()), 3)]
            M["tof_valid_pct"] = round(float((tof_st[mt] == 0).mean() * 100), 1)

    # ---- sticks: confirm hands-off ----
    if have_ctrl:
        mc = win(t_ct)
        if mc.any():
            M["stick_roll_absmax"] = round(float(np.abs(st_roll[mc]).max()), 3)
            M["stick_pitch_absmax"] = round(float(np.abs(st_pitch[mc]).max()), 3)
            M["stick_yaw_absmax"] = round(float(np.abs(st_yaw[mc]).max()), 3)

    if len(volt):
        M["voltage"] = [round(float(volt[0]), 2), round(float(volt[-1]), 2)]

    # ====================== figures ======================
    data = dict(name=name, t_to=t_to, t_end=t_end, flight_mode=flight_mode,
                t_pv=t_pv, pos=pos, vel=vel, psp_x=psp_x, psp_y=psp_y, drift=drift,
                t_imu=t_imu, roll_d=roll_d, pitch_d=pitch_d, accel_raw=accel_raw,
                t_cr=t_cr, ar_r=ar_r, ar_p=ar_p, thrust=thrust, duty=duty,
                m_pv=m_pv, m_imu=m_imu, m_cr=m_cr)
    if have_flow:
        data.update(t_fl=t_fl, fl_q=fl_q, fl_dx=fl_dx, fl_dy=fl_dy)
    if have_tof:
        data.update(t_tof=t_tof, tof_d=tof_d)
    return M, data


def make_figure(data, out_png):
    name = data["name"]
    t_to, t_end = data["t_to"], data["t_end"]
    pad = 0.5
    lo, hi = t_to - pad, t_end + pad
    fig, ax = plt.subplots(4, 1, figsize=(11, 13), sharex=True)
    fig.suptitle(f"{name}  [{MODE_NAME.get(data['flight_mode'])}]  "
                 f"flight {t_to:.1f}-{t_end:.1f}s", fontsize=11)

    # (1) horizontal position vs setpoint + drift
    a = ax[0]
    a.plot(data["t_pv"], data["pos"][:, 0], label="pos N (est)", lw=0.8)
    a.plot(data["t_pv"], data["pos"][:, 1], label="pos E (est)", lw=0.8)
    a.plot(data["t_pv"], data["psp_x"], "--", label="setpoint N", lw=0.8)
    a.plot(data["t_pv"], data["psp_y"], "--", label="setpoint E", lw=0.8)
    a.plot(data["t_pv"], data["drift"], "k", label="|drift|", lw=1.2)
    a.axvline(t_to, color="g", ls=":", lw=0.8); a.axvline(t_end, color="r", ls=":", lw=0.8)
    a.set_ylabel("position [m]"); a.legend(fontsize=7, ncol=5, loc="upper left")
    a.grid(alpha=0.3); a.set_title("horizontal position vs hold setpoint", fontsize=9)

    # (2) est attitude vs tilt command
    a = ax[1]
    a.plot(data["t_imu"], data["roll_d"], label="est roll", lw=0.8)
    a.plot(data["t_imu"], data["pitch_d"], label="est pitch", lw=0.8)
    a.plot(data["t_cr"], data["ar_r"], "--", label="angle_ref roll (cmd)", lw=0.9)
    a.plot(data["t_cr"], data["ar_p"], "--", label="angle_ref pitch (cmd)", lw=0.9)
    a.axhline(MAX_POS_TILT_DEG, color="r", ls=":", lw=0.6)
    a.axhline(-MAX_POS_TILT_DEG, color="r", ls=":", lw=0.6)
    a.axvline(t_to, color="g", ls=":", lw=0.8); a.axvline(t_end, color="r", ls=":", lw=0.8)
    a.set_ylabel("angle [deg]"); a.legend(fontsize=7, ncol=2, loc="upper left")
    a.grid(alpha=0.3); a.set_title("estimated attitude vs position-loop tilt command", fontsize=9)

    # (3) flow quality + velocity
    a = ax[2]
    if "fl_q" in data:
        a.plot(data["t_fl"], data["fl_q"], "m", label="flow SQUAL", lw=0.8)
        a.axhline(FLOW_MIN_SQUAL, color="m", ls=":", lw=0.6, label="gate=10")
    a.set_ylabel("SQUAL"); a.legend(fontsize=7, loc="upper left")
    a2 = a.twinx()
    a2.plot(data["t_pv"], data["vel"][:, 0], "c", label="vel N", lw=0.7)
    a2.plot(data["t_pv"], data["vel"][:, 1], "orange", label="vel E", lw=0.7)
    a2.set_ylabel("velocity [m/s]"); a2.legend(fontsize=7, loc="upper right")
    a.axvline(t_to, color="g", ls=":", lw=0.8); a.axvline(t_end, color="r", ls=":", lw=0.8)
    a.grid(alpha=0.3); a.set_title("optical flow quality + ESKF horizontal velocity", fontsize=9)

    # (4) thrust/duty + vibration
    a = ax[3]
    a.plot(data["t_cr"], data["thrust"], "k", label="total thrust [N]", lw=0.9)
    a.set_ylabel("thrust [N]"); a.legend(fontsize=7, loc="upper left")
    a3 = a.twinx()
    az = data["accel_raw"][:, 2]
    a3.plot(data["t_imu"], az, "gray", lw=0.4, alpha=0.7, label="accel_raw z")
    a3.set_ylabel("accel_raw z [m/s^2]"); a3.legend(fontsize=7, loc="upper right")
    a.axvline(t_to, color="g", ls=":", lw=0.8); a.axvline(t_end, color="r", ls=":", lw=0.8)
    a.grid(alpha=0.3); a.set_title("thrust + raw accel (vibration)", fontsize=9)
    a.set_xlim(lo, hi); a.set_xlabel("time [s]")

    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def print_metrics(M):
    print(f"\n===== {M['log']}  [{M['flight_mode']}] =====")
    order = ["takeoff_s", "end_s", "duration_s", "voltage",
             "stick_roll_absmax", "stick_pitch_absmax", "stick_yaw_absmax",
             "drift_max_m", "drift_end_m", "horiz_speed_max_ms",
             "vel_x_mean_ms", "vel_y_mean_ms", "vel_x_std_ms", "vel_y_std_ms",
             "flow_squal_min", "flow_squal_med", "flow_squal_below_gate_pct",
             "flow_zero_motion_pct", "flow_dx_absmax", "flow_dy_absmax",
             "est_roll_range_deg", "est_pitch_range_deg", "est_tilt_max_deg",
             "angle_ref_max_deg", "angle_ref_sat_pct",
             "eskf_vs_gyro_roll_maxdiff_deg", "eskf_vs_gyro_pitch_maxdiff_deg",
             "accel_raw_std_x", "accel_raw_std_y", "accel_raw_std_z", "accel_z_dom_freq_hz",
             "accel_bias_x_range", "accel_bias_y_range",
             "tof_dist_range_m", "tof_valid_pct"]
    for k in order:
        if k in M:
            print(f"  {k:32s}: {M[k]}")


def main(argv):
    if len(argv) >= 2 and argv[0] == "--compare":
        paths = argv[1:3]
        out = argv[3] if len(argv) > 3 else "analysis/out/poshold"
    else:
        paths = argv[0:1]
        out = argv[1] if len(argv) > 1 else "analysis/out/poshold"
    os.makedirs(out, exist_ok=True)
    allM = {}
    for p in paths:
        res = analyze(p)
        if res is None:
            continue
        M, data = res
        print_metrics(M)
        png = os.path.join(out, os.path.splitext(os.path.basename(p))[0] + ".png")
        make_figure(data, png)
        print(f"  figure -> {png}")
        allM[M["log"]] = M
    with open(os.path.join(out, "metrics.json"), "w") as f:
        json.dump(allM, f, indent=2)
    print(f"\nmetrics -> {os.path.join(out, 'metrics.json')}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1:])
