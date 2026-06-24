#!/usr/bin/env python3
"""
acro_rate_analysis.py — ACRO (angular-rate) control-loop diagnosis.
ACRO（角速度制御）ループ診断。

Question being answered:
  "I have to fly with constant counter-stick and it feels unstable in ACRO."
  当て舵気味でないと飛ばない / 安定感が不足する、の原因をログから定量化する。

In ACRO the pilot stick IS the body-rate command. So the whole story lives in
two telemetry streams that share the 400 Hz IMU timestamp:

  rate_ref  (rad/s)  = commanded body rate  (= stick command in ACRO)
  imu.gyro  (rad/s)  = measured  body rate  (bias-corrected)

We quantify, per axis (roll/pitch/yaw), over the airborne window(s):

  1. Neutral-stick residual rate   -> does the craft drift on its own?
                                       (this is the literal "counter-stick" signature)
  2. Tracking error  gyro - rate_ref  (RMS, bias)
  3. Command->measurement lag  (cross-correlation)  -> sluggish loop / PIO risk
  4. Closed-loop frequency response (ETFE)  -> bandwidth, peaking(resonance), coherence
  5. Positive/negative command asymmetry  -> trim / motor imbalance fingerprint
  6. Sign / cross-axis sanity  -> wiring / sign errors
  7. Attitude (quat) drift while hands-off

Usage:
  python3 analysis/scripts/acro_rate_analysis.py <log.jsonl> [out_dir]
"""
import sys, os, json, math
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

R2D = 180.0 / math.pi
AXES = ["roll", "pitch", "yaw"]


# ---------------------------------------------------------------- load
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
            S.setdefault(d.get("id", "?"), []).append(d)
    return S


def quat_to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def arr(recs, key):
    return np.array([r[key] for r in recs], dtype=float)


# ---------------------------------------------------------------- segment
def airborne_segments(t_tof, tof_d, t_grid, thr_m=0.12, min_dur=2.0):
    """Return list of (t0,t1) airborne windows mapped onto the IMU grid time."""
    fly = tof_d > thr_m
    # interpolate the boolean onto the imu grid (nearest)
    fly_g = np.interp(t_grid, t_tof, fly.astype(float)) > 0.5
    segs = []
    i = 0
    n = len(fly_g)
    while i < n:
        if fly_g[i]:
            j = i
            while j < n and fly_g[j]:
                j += 1
            t0, t1 = t_grid[i], t_grid[j - 1]
            if t1 - t0 >= min_dur:
                segs.append((t0, t1))
            i = j
        else:
            i += 1
    return segs


# ---------------------------------------------------------------- spectral
def etfe(u, y, dt, nseg=8):
    """Welch-averaged transfer function y/u with coherence. u=cmd, y=meas."""
    N = len(u)
    seg = N // nseg
    if seg < 64:
        seg = N
        nseg = 1
    win = np.hanning(seg)
    f = np.fft.rfftfreq(seg, dt)
    Suu = np.zeros(len(f)); Syy = np.zeros(len(f))
    Suy = np.zeros(len(f), dtype=complex)
    cnt = 0
    step = seg // 2 if nseg > 1 else seg
    for s in range(0, N - seg + 1, step):
        U = np.fft.rfft((u[s:s+seg] - u[s:s+seg].mean()) * win)
        Y = np.fft.rfft((y[s:s+seg] - y[s:s+seg].mean()) * win)
        Suu += (U * np.conj(U)).real
        Syy += (Y * np.conj(Y)).real
        Suy += Y * np.conj(U)
        cnt += 1
    H = Suy / np.maximum(Suu, 1e-30)
    coh = (np.abs(Suy) ** 2) / np.maximum(Suu * Syy, 1e-30)
    return f, H, coh


def best_lag(u, y, dt, max_ms=80):
    """Cross-correlation lag (ms) that best aligns measured y to command u.
    Positive = measurement lags command (loop is slow)."""
    u = u - u.mean(); y = y - y.mean()
    n = len(u)
    maxlag = int(max_ms / 1000.0 / dt)
    if maxlag < 1 or n < 8:
        return float("nan"), float("nan")
    lags = np.arange(-maxlag, maxlag + 1)
    c = np.correlate(y, u, mode="full")
    mid = n - 1
    seg = c[mid - maxlag: mid + maxlag + 1]
    norm = np.sqrt(np.sum(u*u) * np.sum(y*y)) + 1e-30
    seg = seg / norm
    k = int(np.argmax(seg))
    return lags[k] * dt * 1000.0, float(seg[k])


# ---------------------------------------------------------------- analyze
def analyze(path, out):
    S = load(path)
    name = os.path.basename(path)
    if "imu" not in S or "rate_ref" not in S:
        print(f"[{name}] no imu/rate_ref, skip"); return
    t0 = S["imu"][0]["ts"]
    t_imu = (arr(S["imu"], "ts") - t0) / 1e6
    gyro = arr(S["imu"], "gyro")              # rad/s, bias corrected
    quat = arr(S["imu"], "quat")
    gbias = arr(S["imu"], "gyro_bias")

    # rate_ref shares the imu timestamp; align by ts dict
    RR = {r["ts"]: r["rate_ref"] for r in S["rate_ref"]}
    ts_imu = arr(S["imu"], "ts")
    rr = np.array([RR.get(int(t), [np.nan, np.nan, np.nan]) for t in ts_imu], dtype=float)

    dt = float(np.median(np.diff(t_imu)))
    fs = 1.0 / dt

    # ToF for airborne segmentation
    if "tof_b" in S:
        t_tof = (arr(S["tof_b"], "ts") - t0) / 1e6
        tof_d = arr(S["tof_b"], "distance")
    else:
        t_tof = t_imu; tof_d = np.full_like(t_imu, 0.5)

    segs = airborne_segments(t_tof, tof_d, t_imu)
    if not segs:
        # fall back: where rate command or rate active
        active = (np.nanmax(np.abs(rr), axis=1) > np.radians(15)) | (np.max(np.abs(gyro), axis=1) > np.radians(30))
        if active.any():
            segs = [(t_imu[active][0], t_imu[active][-1])]
    print(f"\n{'='*72}\n{name}   fs={fs:.0f}Hz  dur={t_imu[-1]:.1f}s")
    print(f"airborne segments (ToF>0.12m, >=2s): "
          + ", ".join(f"{a:.1f}-{b:.1f}s" for a, b in segs) if segs else "NONE")
    if not segs:
        print("no flight window found"); return

    # pick the longest segment for the main loop analysis
    seg = max(segs, key=lambda ab: ab[1] - ab[0])
    m = (t_imu >= seg[0]) & (t_imu <= seg[1])
    print(f"main window: {seg[0]:.1f}-{seg[1]:.1f}s  ({seg[1]-seg[0]:.1f}s, N={m.sum()})")

    g = gyro[m] * R2D     # deg/s
    c = rr[m] * R2D       # deg/s command
    tt = t_imu[m]
    valid = ~np.isnan(c).any(axis=1)
    g = g[valid]; c = c[valid]; tt = tt[valid]

    metrics = {"log": name, "window_s": [round(seg[0], 1), round(seg[1], 1)], "fs_hz": round(fs)}
    print("\n  per-axis ACRO rate-loop diagnosis  (deg/s unless noted)")
    print("  " + "-"*86)
    hdr = ("axis", "cmd_max", "cmd_rms", "meas_rms", "err_rms", "err_bias",
           "neutral_drift", "lag_ms", "xcorr")
    print("  " + "".join(f"{h:>12s}" for h in hdr))
    rows = {}
    for k, ax in enumerate(AXES):
        ck = c[:, k]; gk = g[:, k]
        err = gk - ck
        # neutral window: |cmd| small -> craft should hold rate ~0
        neu = np.abs(ck) < 8.0   # deg/s
        neutral_drift = float(np.median(gk[neu])) if neu.sum() > 20 else float("nan")
        lag_ms, xc = best_lag(ck, gk, dt)
        row = dict(
            cmd_max=float(np.abs(ck).max()),
            cmd_rms=float(np.sqrt(np.mean(ck**2))),
            meas_rms=float(np.sqrt(np.mean(gk**2))),
            err_rms=float(np.sqrt(np.mean(err**2))),
            err_bias=float(np.mean(err)),
            neutral_drift=neutral_drift,
            lag_ms=lag_ms, xcorr=xc,
        )
        rows[ax] = row
        print("  " + f"{ax:>12s}" + "".join(
            f"{row[h]:>12.2f}" if not math.isnan(row[h]) else f"{'nan':>12s}"
            for h in ("cmd_max", "cmd_rms", "meas_rms", "err_rms", "err_bias",
                      "neutral_drift", "lag_ms", "xcorr")))
    metrics["axes"] = rows

    # frequency response per axis (only if there is real excitation)
    print("\n  closed-loop freq response  rate_ref->gyro  (where coherence>0.5)")
    print("  " + "-"*70)
    fr = {}
    for k, ax in enumerate(AXES):
        ck = c[:, k]; gk = g[:, k]
        if np.std(ck) < 2.0:
            print(f"  {ax:>6s}: command too small (std={np.std(ck):.1f} deg/s) — skip")
            continue
        f, H, coh = etfe(ck, gk, dt, nseg=8)
        mag = np.abs(H)
        good = (coh > 0.5) & (f > 0.3)
        if good.sum() < 3:
            print(f"  {ax:>6s}: low coherence everywhere (max={coh[f>0.3].max():.2f}) — noisy/poor tracking")
            fr[ax] = dict(note="low_coherence", coh_max=float(coh[f>0.3].max()))
            continue
        # -3dB bandwidth: first freq where mag drops below 1/sqrt2 of DC-ish (use mag near lowest good f)
        dcmag = np.median(mag[good][:3])
        bw = float("nan")
        below = (mag < dcmag / math.sqrt(2)) & good
        if below.any():
            bw = float(f[below][0])
        peak = float(mag[good].max())
        peak_f = float(f[good][np.argmax(mag[good])])
        # phase at the frequency where mag crosses ~ -3dB (proxy)
        fr[ax] = dict(bw_hz=bw, peak_gain=peak, peak_f_hz=peak_f,
                      coh_max=float(coh[good].max()))
        print(f"  {ax:>6s}: -3dB BW={bw:.1f}Hz  peak|H|={peak:.2f}@{peak_f:.1f}Hz  "
              f"coh_max={coh[good].max():.2f}"
              + ("   <-- RESONANT PEAK" if peak > 1.3 else ""))
    metrics["freq"] = fr

    # attitude drift hands-off: integrate not needed, use quat euler over neutral windows
    eul = np.array([quat_to_euler(q) for q in quat[m]]) * R2D
    eul = eul[valid]
    metrics["att"] = dict(
        roll_range=[round(float(eul[:, 0].min()), 1), round(float(eul[:, 0].max()), 1)],
        pitch_range=[round(float(eul[:, 1].min()), 1), round(float(eul[:, 1].max()), 1)],
    )
    print(f"\n  attitude over window: roll {metrics['att']['roll_range']}deg  "
          f"pitch {metrics['att']['pitch_range']}deg")
    print(f"  gyro_bias (last): {np.round(gbias[-1], 4).tolist()} rad/s")

    # ----------------- figures -----------------
    os.makedirs(out, exist_ok=True)
    fig, ax = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    for k, axis in enumerate(AXES):
        a = ax[k]
        a.plot(tt, c[:, k], label=f"{axis} cmd (rate_ref)", lw=0.9, color="tab:orange")
        a.plot(tt, g[:, k], label=f"{axis} meas (gyro)", lw=0.7, color="tab:blue", alpha=0.85)
        a.axhline(0, color="k", lw=0.4)
        a.set_ylabel(f"{axis} [deg/s]"); a.grid(alpha=0.3)
        a.legend(fontsize=8, loc="upper right")
    ax[0].set_title(f"{name}  ACRO rate tracking  cmd vs measured  [{seg[0]:.1f}-{seg[1]:.1f}s]", fontsize=10)
    ax[-1].set_xlabel("time [s]")
    fig.tight_layout()
    p1 = os.path.join(out, os.path.splitext(name)[0] + "_track.png")
    fig.savefig(p1, dpi=110); plt.close(fig)

    # scatter cmd vs meas (instant) per axis -> gain & asymmetry
    fig, ax = plt.subplots(1, 3, figsize=(15, 5))
    for k, axis in enumerate(AXES):
        a = ax[k]
        a.scatter(c[:, k], g[:, k], s=2, alpha=0.2)
        lim = max(np.abs(c[:, k]).max(), np.abs(g[:, k]).max(), 1)
        a.plot([-lim, lim], [-lim, lim], "r--", lw=0.8, label="ideal y=x")
        a.set_xlabel(f"{axis} cmd [deg/s]"); a.set_ylabel(f"{axis} meas [deg/s]")
        a.set_title(axis); a.grid(alpha=0.3); a.legend(fontsize=8)
        a.set_aspect("equal", "box")
    fig.suptitle(f"{name}  cmd vs measured rate (instantaneous)", fontsize=10)
    fig.tight_layout()
    p2 = os.path.join(out, os.path.splitext(name)[0] + "_scatter.png")
    fig.savefig(p2, dpi=110); plt.close(fig)
    print(f"\n  figures -> {p1}\n            {p2}")

    with open(os.path.join(out, os.path.splitext(name)[0] + "_metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    return metrics


def main(argv):
    if not argv:
        print(__doc__); return
    path = argv[0]
    out = argv[1] if len(argv) > 1 else "analysis/out/acro"
    analyze(path, out)


if __name__ == "__main__":
    main(sys.argv[1:])
