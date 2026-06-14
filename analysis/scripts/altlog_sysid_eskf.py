#!/usr/bin/env python3
"""
altlog_sysid_eskf.py — ALT_HOLD flight-log analysis: rate-loop system ID, current/
suggested PID margins, and ESKF consistency. Produces PNG figures + a metrics JSON.

ALT_HOLD 飛行ログ解析: レートループのシステム同定、現/推奨 PID の余裕、ESKF の整合性。
PNG 図と metrics JSON を出力する。

Reuses the proven rate-loop tooling (tools/log_analyzer/rate_sysid.py): the firmware
PID replay reconstructs the rate-loop torque u from rate_ref + gyro + the FLOWN gains
(logged at 1 Hz in the status record), then ETFE(u, gyro) gives the plant.

Usage: python3 analysis/scripts/altlog_sysid_eskf.py <log.jsonl> <out_dir>
"""
import sys, os, json, math
import numpy as np

# Reuse the flight-proven rate-loop identification/tuning backend.
# 実績あるレートループ同定/チューニングのバックエンドを再利用。
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "tools", "log_analyzer"))
import rate_sysid as rs  # noqa: E402

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

R2D = 180.0 / math.pi
AXES = ["roll", "pitch", "yaw"]
LIMITS = {"roll": 5.2e-3, "pitch": 5.2e-3, "yaw": 2.2e-3}  # rate-loop output limit [Nm]


# ---------------------------------------------------------------------------
# Load + resample onto a uniform grid (ETFE needs evenly-sampled data).
# 読込 + 一様グリッドへ再サンプル（ETFE は等間隔データが前提）。
# ---------------------------------------------------------------------------
def load(path):
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
            S.setdefault(d["id"], []).append(d)
    return S


def main(path, out):
    os.makedirs(out, exist_ok=True)
    S = load(path)
    t0 = S["imu"][0]["ts"]
    M = {"log": os.path.basename(path)}

    # --- flown gains (median over the flight) from the 1 Hz status record ---
    g = {a: np.median([s[f"pid_{a}"] for s in S["status"]], axis=0) for a in AXES}
    flown = {a: {"kp": float(g[a][0]), "ti": float(g[a][1]), "td": float(g[a][2])} for a in AXES}
    M["flown_gains"] = flown
    M["voltage"] = {"start": S["status"][0]["voltage"], "end": S["status"][-1]["voltage"]}

    # --- uniform-grid arrays over the FLYING segment (state==5) ---
    imu = S["imu"]
    t_imu = np.array([(d["ts"] - t0) / 1e6 for d in imu])
    gyro = np.array([d["gyro"] for d in imu])
    accel = np.array([d["accel"] for d in imu])
    quat = np.array([d["quat"] for d in imu])
    gbias = np.array([d["gyro_bias"] for d in imu])
    abias = np.array([d["accel_bias"] for d in imu])
    rr = np.array([d["rate_ref"] for d in S["rate_ref"]])
    t_rr = np.array([(d["ts"] - t0) / 1e6 for d in S["rate_ref"]])
    pos = np.array([d["pos"] for d in S["posvel"]])
    vel = np.array([d["vel"] for d in S["posvel"]])
    t_pv = np.array([(d["ts"] - t0) / 1e6 for d in S["posvel"]])

    # FLYING window: skip takeoff transient; use a clean hover/maneuver span.
    # 飛行窓: 離陸過渡を避け、ホバー/操縦の区間を使う。
    t_start = 6.0
    t_end = t_imu[-1] - 0.5
    fs = 1.0 / np.median(np.diff(t_imu))
    M["fs_imu"] = float(fs)
    tg = np.arange(t_start, t_end, 1.0 / fs)   # uniform time grid
    gyro_u = np.vstack([np.interp(tg, t_imu, gyro[:, k]) for k in range(3)]).T
    rr_u = np.vstack([np.interp(tg, t_rr, rr[:, k]) for k in range(3)]).T

    # =====================================================================
    # 1. Overview figure
    # =====================================================================
    def q2eul(q):
        w, x, y, z = q
        return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * R2D,
                math.asin(max(-1, min(1, 2 * (w * y - z * x)))) * R2D,
                math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * R2D)
    eul = np.array([q2eul(q) for q in quat])
    alt_eskf = -pos[:, 2]
    tof = np.array([[(d["ts"] - t0) / 1e6, d["distance"], d["status"]] for d in S["tof_b"]])
    baro = np.array([[(d["ts"] - t0) / 1e6, d["altitude"]] for d in S["baro"]])
    cref = S["ctrl_ref"]
    t_cr = np.array([(d["ts"] - t0) / 1e6 for d in cref])
    thrust = np.array([d["total_thrust"] for d in cref])
    duty = np.array([d["motor_duty"] for d in cref])
    alt_sp = np.array([d["alt_sp"] for d in cref])

    fig, ax = plt.subplots(3, 2, figsize=(13, 10))
    ax[0, 0].plot(t_imu, eul[:, 0], label="roll"); ax[0, 0].plot(t_imu, eul[:, 1], label="pitch")
    ax[0, 0].plot(t_imu, eul[:, 2], label="yaw"); ax[0, 0].set_title("Attitude [deg]"); ax[0, 0].legend(); ax[0, 0].grid(alpha=.3)
    for k, nm in enumerate(AXES):
        ax[0, 1].plot(t_imu, gyro[:, k] * R2D, label=f"gyro {nm}", lw=.6)
    ax[0, 1].set_title("Body rates [deg/s]"); ax[0, 1].legend(fontsize=7); ax[0, 1].grid(alpha=.3)
    ax[1, 0].plot(t_pv, alt_eskf, label="ESKF", lw=1.2)
    ax[1, 0].plot(tof[:, 0], tof[:, 1], '.', ms=2, label="ToF", alpha=.5)
    ax[1, 0].plot(t_cr, alt_sp, '--', label="alt setpoint", lw=.8)
    ax[1, 0].set_title("Altitude [m]"); ax[1, 0].legend(); ax[1, 0].grid(alpha=.3)
    ax[1, 1].plot(baro[:, 0], baro[:, 1] - np.median(baro[:, 1]), label="baro−median", lw=.6)
    ax[1, 1].set_title("Baro altitude (relative) [m]"); ax[1, 1].legend(); ax[1, 1].grid(alpha=.3)
    ax[2, 0].plot(t_cr, thrust, label="total_thrust [N]")
    ax[2, 0].plot(t_cr, np.mean(duty, axis=1), label="mean duty")
    ax[2, 0].set_title("Thrust / duty"); ax[2, 0].legend(); ax[2, 0].grid(alpha=.3)
    vt = np.array([[(s["ts"] - t0) / 1e6, s["voltage"]] for s in S["status"]])
    ax[2, 1].plot(vt[:, 0], vt[:, 1], '-o', ms=3)
    ax[2, 1].set_title("Battery [V]"); ax[2, 1].grid(alpha=.3)
    for a in ax.ravel():
        a.set_xlabel("t [s]")
    fig.tight_layout(); fig.savefig(f"{out}/01_overview.png", dpi=110); plt.close(fig)

    # =====================================================================
    # 2. Rate-loop system ID per axis
    # =====================================================================
    M["sysid"] = {}
    fig, ax = plt.subplots(3, 3, figsize=(15, 11))
    for r, a in enumerate(AXES):
        kp, ti, td = flown[a]["kp"], flown[a]["ti"], flown[a]["td"]
        u = rs.replay_pid(rr_u[:, r], gyro_u[:, r], kp, ti, td, LIMITS[a], dt=1.0 / fs)
        w, G, coh = rs.etfe(u, gyro_u[:, r], fs=fs, f_lo=0.8, f_hi=30.0)
        # closed-loop FRF (gain-independent cross-check): rate_ref -> gyro
        wcl, Gcl, cohcl = rs.etfe(rr_u[:, r], gyro_u[:, r], fs=fs, f_lo=0.8, f_hi=30.0)
        fit = rs.fit_plant(w, G, coh, axis=a)
        cur = rs.loop_margins(fit["b"], fit["T"], fit["L"], kp, ti, td)
        M["sysid"][a] = {"fit": {k: fit[k] for k in ("b", "T", "L", "inertia_eff", "coherence_mean")},
                         "current_margins": cur, "exc_std_dps": float(rr_u[:, r].std() * R2D)}
        fhz = w / (2 * math.pi)
        Gm = rs.plant_response(w, fit["b"], fit["T"], fit["L"])
        ax[r, 0].loglog(fhz, np.abs(G), '.', ms=3, label="ETFE plant", alpha=.5)
        ax[r, 0].loglog(fhz, np.abs(Gm), '-', label="fit b·e^{-Ls}/(s(Ts+1))")
        ax[r, 0].set_title(f"{a}: plant |G| (J_eff={fit['inertia_eff']:.2e})"); ax[r, 0].legend(fontsize=7); ax[r, 0].grid(alpha=.3, which="both")
        ax[r, 1].semilogx(fhz, np.unwrap(np.angle(G)) * R2D, '.', ms=3, alpha=.5)
        ax[r, 1].semilogx(fhz, np.angle(Gm) * R2D, '-')
        ax[r, 1].set_title(f"{a}: plant phase [deg]"); ax[r, 1].grid(alpha=.3, which="both")
        ax[r, 2].semilogx(fhz, coh, label=f"coh(u,gyro)"); ax[r, 2].semilogx(wcl / (2 * math.pi), cohcl, label="coh(ref,gyro)", alpha=.6)
        ax[r, 2].axhline(0.6, color='r', ls=':', lw=.8)
        ax[r, 2].set_title(f"{a}: coherence (exc {rr_u[:,r].std()*R2D:.1f}dps)"); ax[r, 2].set_ylim(0, 1); ax[r, 2].legend(fontsize=7); ax[r, 2].grid(alpha=.3, which="both")
        for c in range(3):
            ax[r, c].set_xlabel("f [Hz]")
    fig.tight_layout(); fig.savefig(f"{out}/02_sysid.png", dpi=110); plt.close(fig)

    # --- suggested gains for a target PM/wc (per axis), VERIFIED numerically ---
    # 目標 PM/wc の推奨ゲイン（数値検証付き）。実機適用は SIL 検証が前提。
    TARGET = {"roll": (40.0, 55.0), "pitch": (40.0, 55.0), "yaw": (25.0, 55.0)}  # (wc rad/s, PM deg)
    for a in AXES:
        f = M["sysid"][a]["fit"]
        wc, pm = TARGET[a]
        try:
            tuned = rs.tune_pid(f["b"], f["T"], f["L"], wc, pm)
            M["sysid"][a]["suggested"] = {"target_wc": wc, "target_pm": pm,
                                          "kp": tuned["kp"], "ti": tuned["ti"], "td": tuned["td"],
                                          "achieved": tuned["achieved"]}
        except Exception as e:
            M["sysid"][a]["suggested"] = {"error": str(e)}

    # =====================================================================
    # 3. ESKF consistency
    # =====================================================================
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    # (a) ESKF alt vs ToF residual (interp ToF onto posvel grid, status==0/valid only)
    tof_valid = tof[tof[:, 2] == 0]
    if len(tof_valid) > 5:
        tof_i = np.interp(t_pv, tof_valid[:, 0], tof_valid[:, 1])
        res_alt = alt_eskf - tof_i
        flying = t_pv > 6.0
        M["eskf_alt_vs_tof_resid_std_m"] = float(np.std(res_alt[flying]))
        ax[0, 0].plot(t_pv, alt_eskf, label="ESKF alt"); ax[0, 0].plot(t_pv, tof_i, label="ToF", alpha=.6)
        ax[0, 0].plot(t_pv, res_alt, label=f"residual (σ={np.std(res_alt[flying])*100:.1f}cm)", lw=.7)
    ax[0, 0].set_title("Vertical: ESKF vs ToF [m]"); ax[0, 0].legend(fontsize=8); ax[0, 0].grid(alpha=.3)
    # (b) attitude: accel-derived tilt vs quat tilt residual
    roll_acc = np.arctan2(-accel[:, 1], -accel[:, 2]) * R2D
    pitch_acc = np.arctan2(accel[:, 0], np.sqrt(accel[:, 1] ** 2 + accel[:, 2] ** 2)) * R2D
    res_roll = roll_acc - eul[:, 0]
    res_pitch = pitch_acc - eul[:, 1]
    amag = np.linalg.norm(accel, axis=1)
    quiet = np.abs(amag - 9.81) < 0.5   # near-1g samples = trustworthy accel tilt
    M["eskf_att_resid_deg"] = {"roll_std": float(np.std(res_roll[quiet])),
                               "pitch_std": float(np.std(res_pitch[quiet])),
                               "accel_excess_frac": float(np.mean(~quiet))}
    # Scatter only the near-1g (trustworthy accel) samples: accel-tilt vs ESKF-tilt.
    # Consistency ⇒ points on y=x. The flight is too dynamic (most samples have specific
    # acceleration) for a time-series residual, so use the quiet subset.
    # near-1g（信頼できる accel）サンプルのみの散布図: accel-tilt vs ESKF-tilt。整合なら y=x 上。
    ax[0, 1].scatter(eul[quiet, 0], roll_acc[quiet], s=3, alpha=.3, label="roll")
    ax[0, 1].scatter(eul[quiet, 1], pitch_acc[quiet], s=3, alpha=.3, label="pitch")
    lim = 12
    ax[0, 1].plot([-lim, lim], [-lim, lim], 'k--', lw=.8, label="y=x")
    ax[0, 1].set_xlim(-lim, lim); ax[0, 1].set_ylim(-lim, lim); ax[0, 1].set_aspect("equal")
    ax[0, 1].set_xlabel("ESKF tilt [deg]"); ax[0, 1].set_ylabel("accel tilt [deg]")
    ax[0, 1].set_title(f"Attitude consistency, near-1g only ({np.mean(quiet)*100:.0f}% of samples)")
    ax[0, 1].legend(fontsize=8); ax[0, 1].grid(alpha=.3)
    # (c) bias time series
    ax[1, 0].plot(t_imu, gbias[:, 0] * R2D, label="gx"); ax[1, 0].plot(t_imu, gbias[:, 1] * R2D, label="gy"); ax[1, 0].plot(t_imu, gbias[:, 2] * R2D, label="gz")
    ax[1, 0].set_title("Gyro bias [deg/s]"); ax[1, 0].legend(fontsize=8); ax[1, 0].grid(alpha=.3)
    ax2 = ax[1, 0].twinx()
    for k, nm in enumerate(["ax", "ay", "az"]):
        ax2.plot(t_imu, abias[:, k], '--', lw=.7, label=f"accel {nm}")
    ax2.legend(fontsize=7, loc="upper right")
    M["bias"] = {"gyro_last": gbias[-1].tolist(), "accel_first": abias[0].tolist(), "accel_last": abias[-1].tolist(),
                 "accel_bias_drift": float(np.linalg.norm(abias[-1] - abias[0]))}
    # (d) gyro PSD (oscillation peaks)
    from numpy.fft import rfft, rfftfreq
    M["psd_peak_hz"] = {}
    for k, nm in enumerate(AXES):
        sig = gyro_u[:, k] - gyro_u[:, k].mean()
        nper = 2048
        acc = None
        cnt = 0
        win = np.hanning(nper)
        for s0 in range(0, len(sig) - nper, nper // 2):
            P = np.abs(rfft(sig[s0:s0 + nper] * win)) ** 2
            acc = P if acc is None else acc + P
            cnt += 1
        psd = acc / cnt
        fr = rfftfreq(nper, 1.0 / fs)
        ax[1, 1].semilogy(fr, psd, label=nm, lw=.7)
        band = (fr > 2) & (fr < 60)
        pk = fr[band][np.argmax(psd[band])]
        M["psd_peak_hz"][nm] = float(pk)
    ax[1, 1].set_xlim(0, 60); ax[1, 1].set_title("Gyro PSD (oscillation check)"); ax[1, 1].set_xlabel("f [Hz]"); ax[1, 1].legend(fontsize=8); ax[1, 1].grid(alpha=.3, which="both")
    for a in (ax[0, 0], ax[1, 0]):
        a.set_xlabel("t [s]")
    fig.tight_layout(); fig.savefig(f"{out}/03_eskf.png", dpi=110); plt.close(fig)

    # =====================================================================
    # 4. Specific-acceleration / accel-attitude innovation (grounds accel_att_noise)
    #    比力 / accel 姿勢イノベーション（accel_att_noise の接地）
    #
    # The accel-attitude update assumes accel = gravity. The deviation (specific
    # acceleration = measured accel − predicted gravity reaction) is what R must cover.
    # Measured FROM the dynamic flight (a still flight is impossible AND would under-
    # estimate this). For near-1g samples the adaptive R term ~1, so their spread is the
    # BASELINE that accel_att_noise should match.
    # accel 姿勢更新は accel=重力を仮定。そのズレ（比力＝accel−予測重力反力）こそ R が覆う量。
    # 動的飛行から実測する（静止飛行は不可能でかつ過小評価になる）。near-1g サンプルは適応R≈1 ゆえ
    # その広がりが accel_att_noise を合わせるべきベースライン。
    g = 9.80665
    def g_expected(q):
        w, x, y, z = q
        return np.array([2 * (x * z - w * y) * -g, 2 * (y * z + w * x) * -g, (1 - 2 * (x * x + y * y)) * -g])
    aspec = np.array([accel[i] - g_expected(quat[i]) for i in range(len(quat))])
    amag = np.linalg.norm(accel, axis=1)
    gdiff = np.abs(amag - g)
    flyi = t_imu > 6.0
    near1g = flyi & (gdiff < 0.3)            # adaptive-R term ≈ 1 here
    base = aspec[near1g]
    M["accel_innov"] = {
        "accel_att_noise_current": 0.8,
        "near1g_frac": float(np.mean(near1g[flyi])),
        "baseline_std_xyz": [float(base[:, k].std()) for k in range(3)],
        "baseline_horiz_std": float(np.linalg.norm(base[:, :2], axis=1).std()),
        "gdiff_p50": float(np.percentile(gdiff[flyi], 50)),
        "gdiff_p90": float(np.percentile(gdiff[flyi], 90)),
        "x_over_y_anisotropy": float(base[:, 0].std() / max(base[:, 1].std(), 1e-6)),
    }
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    for k, nm in enumerate(["x→pitch", "y→roll", "z→vert"]):
        ax[0].hist(base[:, k], bins=80, alpha=.5, label=f"{nm} σ={base[:,k].std():.2f}", density=True)
    for s in (0.8, 1.5, 2.0):
        ax[0].axvline(s, color='k', ls=':', lw=.7); ax[0].axvline(-s, color='k', ls=':', lw=.7)
    ax[0].set_title("Specific accel, near-1g [m/s²]\n(dotted = accel_att_noise candidates 0.8/1.5/2.0)")
    ax[0].set_xlim(-8, 8); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
    ax[1].hist(gdiff[flyi], bins=80, density=True)
    ax[1].axvline(np.percentile(gdiff[flyi], 90), color='r', ls='--', label=f"p90={np.percentile(gdiff[flyi],90):.1f}")
    ax[1].set_title("|accel|−g [m/s²] (drives adaptive R)\nlarge ⇒ R inflates, χ² may reject"); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
    ax[2].plot(t_imu, aspec[:, 0], lw=.4, label="x→pitch"); ax[2].plot(t_imu, aspec[:, 1], lw=.4, label="y→roll")
    ax[2].set_title("Specific accel time series\n(x-axis anisotropy = vibration)"); ax[2].set_xlabel("t [s]"); ax[2].set_ylim(-15, 15); ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(f"{out}/04_accel_innov.png", dpi=110); plt.close(fig)

    # =====================================================================
    # 5. Flow / position / velocity quality (toward POS_HOLD)
    #    フロー / 位置 / 速度の品質（POS_HOLD に向けて）
    #
    # ALT_HOLD holds altitude only — horizontal pos/vel are DEAD-RECKONED from accel +
    # optical flow. Their quality here predicts whether POS_HOLD can hold. We look at:
    #  - flow SQUAL (quality) — is the flow usable?
    #  - ESKF horizontal velocity noise at "hover" (drives POS_HOLD's inner loop)
    #  - horizontal position drift over the flight (no pos aiding in ALT_HOLD ⇒ the drift
    #    IS the dead-reckoning error; accel-bias x,y integrates straight into it)
    #  - flow-rate vs ESKF horizontal velocity (does flow track the motion?)
    # ALT_HOLD は高度のみ保持 — 水平 pos/vel は accel＋フローのデッドレコニング。その品質が
    # POS_HOLD 成立可否を予告する。
    # =====================================================================
    flow = S["flow"]
    t_fl = np.array([(d["ts"] - t0) / 1e6 for d in flow])
    dxdy = np.array([[d["dx"], d["dy"]] for d in flow], dtype=float)
    squal = np.array([d["quality"] for d in flow], dtype=float)
    flyv = t_pv > 6.0
    vh = np.linalg.norm(vel[:, :2], axis=1)          # ESKF horizontal speed
    ph = pos[:, :2] - pos[flyv][0, :2]               # horizontal position vs start-of-flight
    drift = np.linalg.norm(ph, axis=1)
    flyf = t_fl > 6.0
    M["posvel"] = {
        "flow_squal_mean": float(squal[flyf].mean()), "flow_squal_p10": float(np.percentile(squal[flyf], 10)),
        "flow_squal_below_30_frac": float(np.mean(squal[flyf] < 30)),
        "eskf_hvel_std_mps": float(vh[flyv].std()), "eskf_hvel_p90_mps": float(np.percentile(vh[flyv], 90)),
        "horiz_drift_end_m": float(drift[flyv][-1]),
        "horiz_drift_rate_mps": float(drift[flyv][-1] / (t_pv[flyv][-1] - t_pv[flyv][0])),
        "accel_bias_xy_end": [float(abias[-1, 0]), float(abias[-1, 1])],
    }
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    ax[0, 0].plot(t_fl, squal, lw=.6); ax[0, 0].axhline(30, color='r', ls='--', label="SQUAL=30 (typical floor)")
    ax[0, 0].set_title(f"Optical-flow quality (SQUAL)  mean={squal[flyf].mean():.0f}"); ax[0, 0].legend(fontsize=8); ax[0, 0].grid(alpha=.3)
    ax[0, 1].plot(t_pv, vel[:, 0], lw=.6, label="vx"); ax[0, 1].plot(t_pv, vel[:, 1], lw=.6, label="vy")
    ax[0, 1].set_title(f"ESKF horizontal velocity [m/s]  (σ={vh[flyv].std():.3f}, p90={np.percentile(vh[flyv],90):.2f})"); ax[0, 1].legend(fontsize=8); ax[0, 1].grid(alpha=.3)
    ax[1, 0].plot(ph[flyv, 0], ph[flyv, 1], lw=.7); ax[1, 0].plot(0, 0, 'go', label="start"); ax[1, 0].plot(ph[flyv, 0][-1], ph[flyv, 1][-1], 'rs', label=f"end ({drift[flyv][-1]:.1f}m)")
    ax[1, 0].set_aspect("equal"); ax[1, 0].set_title("Horizontal position (dead-reckoned, ALT_HOLD)"); ax[1, 0].set_xlabel("x [m]"); ax[1, 0].set_ylabel("y [m]"); ax[1, 0].legend(fontsize=8); ax[1, 0].grid(alpha=.3)
    # flow rate (px/s) vs ESKF horizontal speed — correlation = flow tracks motion
    dt_fl = np.diff(t_fl, prepend=t_fl[0] - 1.0 / 98)
    flow_rate = np.linalg.norm(dxdy, axis=1) / np.maximum(dt_fl, 1e-3)
    vh_at_fl = np.interp(t_fl, t_pv, vh)
    ax[1, 1].scatter(vh_at_fl[flyf], flow_rate[flyf], s=3, alpha=.3)
    ax[1, 1].set_xlabel("ESKF horiz speed [m/s]"); ax[1, 1].set_ylabel("flow rate [px/s]")
    ax[1, 1].set_title("Flow rate vs ESKF speed (correlation ⇒ flow tracks motion)"); ax[1, 1].grid(alpha=.3)
    for a in (ax[0, 0], ax[0, 1]):
        a.set_xlabel("t [s]")
    fig.tight_layout(); fig.savefig(f"{out}/05_posvel_flow.png", dpi=110); plt.close(fig)

    # =====================================================================
    # 6. Vibration spectrum (x-axis anisotropy source) — high-freq accel FFT
    #    振動スペクトル（x軸異方性の源）— 高周波 accel FFT
    # =====================================================================
    acc_u = np.vstack([np.interp(tg, t_imu, accel[:, k]) for k in range(3)]).T
    from numpy.fft import rfft, rfftfreq
    fig, ax = plt.subplots(1, 1, figsize=(11, 5))
    M["vib_peak_hz"] = {}
    nper = 2048
    win = np.hanning(nper)
    fr = rfftfreq(nper, 1.0 / fs)
    for k, nm in enumerate(["accel x→pitch", "accel y→roll", "accel z→vert"]):
        sig = acc_u[:, k] - acc_u[:, k].mean()
        acc = None
        cnt = 0
        for s0 in range(0, len(sig) - nper, nper // 2):
            P = np.abs(rfft(sig[s0:s0 + nper] * win)) ** 2
            acc = P if acc is None else acc + P
            cnt += 1
        psd = acc / cnt
        ax.semilogy(fr, psd, lw=.8, label=nm)
        band = (fr > 8) & (fr < fs / 2)
        M["vib_peak_hz"][nm] = float(fr[band][np.argmax(psd[band])])
    ax.set_xlim(0, fs / 2); ax.set_title("Accel PSD (vibration) — x-axis carries the energy; peak ⇒ prop/motor source")
    ax.set_xlabel("f [Hz]"); ax.set_ylabel("PSD [(m/s²)²/Hz]"); ax.legend(); ax.grid(alpha=.3, which="both")
    fig.tight_layout(); fig.savefig(f"{out}/06_vibration.png", dpi=110); plt.close(fig)

    with open(f"{out}/metrics.json", "w") as fp:
        json.dump(M, fp, indent=2)
    print(json.dumps({"accel_innov": M["accel_innov"], "posvel": M["posvel"], "vib_peak_hz": M["vib_peak_hz"]}, indent=2))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
