#!/usr/bin/env python3
"""
acro_rate_spectral.py — spectral / band-split follow-up to acro_rate_analysis.

acro_rate_analysis showed measured body-rate RMS >> commanded RMS with very low
command/measurement coherence: the body rate is dominated by something the pilot
did NOT command. This script localizes that "something":

  A. PSD of gyro per axis        -> dominant oscillation frequency
  B. PSD of accel_raw per axis   -> structural vibration vs control limit-cycle
  C. band split (pilot <Fc Hz | oscillation >Fc Hz):
        - does cmd track meas in the PILOT band?  (loop healthy in-band?)
        - how much energy is in the OSCILLATION band?
  D. cross-axis cmd->meas correlation matrix  -> axis-swap / sign sanity
  E. spectrogram of worst axis  -> sustained limit cycle vs transient?

Usage:
  python3 analysis/scripts/acro_rate_spectral.py <log.jsonl> [out_dir] [t0 t1]
"""
import sys, os, json, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

try:
    from scipy import signal as ss
    HAVE_SCIPY = True
except Exception:
    HAVE_SCIPY = False

R2D = 180.0 / math.pi
AXES = ["roll", "pitch", "yaw"]
FC = 4.0   # pilot-band / oscillation-band split [Hz]

_trap = getattr(np, "trapezoid", getattr(np, "trapz", None))


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


def arr(recs, key):
    return np.array([r[key] for r in recs], dtype=float)


def welch_psd(x, fs, nseg=1024):
    if HAVE_SCIPY:
        f, p = ss.welch(x, fs=fs, nperseg=min(nseg, len(x)))
        return f, p
    # numpy fallback
    seg = min(nseg, len(x))
    win = np.hanning(seg)
    step = seg // 2
    f = np.fft.rfftfreq(seg, 1/fs)
    psd = np.zeros(len(f)); cnt = 0
    for s in range(0, len(x)-seg+1, step):
        X = np.fft.rfft((x[s:s+seg]-x[s:s+seg].mean())*win)
        psd += (X*np.conj(X)).real; cnt += 1
    return f, psd/max(cnt, 1)


def butter_split(x, fs, fc):
    """Return (low, high) band components."""
    if HAVE_SCIPY:
        b, a = ss.butter(4, fc/(fs/2), "low")
        lo = ss.filtfilt(b, a, x)
        return lo, x - lo
    # crude FFT brickwall fallback
    X = np.fft.rfft(x - x.mean())
    f = np.fft.rfftfreq(len(x), 1/fs)
    Xl = X.copy(); Xl[f > fc] = 0
    lo = np.fft.irfft(Xl, n=len(x)) + x.mean()
    return lo, x - lo


def xcorr_peak(u, y):
    u = u - u.mean(); y = y - y.mean()
    n = (np.sqrt(np.sum(u*u)*np.sum(y*y)) + 1e-30)
    return float(np.sum(u*y)/n)   # zero-lag normalized correlation


def main(argv):
    if not argv:
        print(__doc__); return
    path = argv[0]
    out = argv[1] if len(argv) > 1 else "analysis/out/acro"
    os.makedirs(out, exist_ok=True)
    S = load(path)
    name = os.path.basename(path)
    t0 = S["imu"][0]["ts"]
    t = (arr(S["imu"], "ts") - t0) / 1e6
    gyro = arr(S["imu"], "gyro") * R2D
    graw = arr(S["imu"], "gyro_raw") * R2D
    araw = arr(S["imu"], "accel_raw")
    RR = {r["ts"]: r["rate_ref"] for r in S["rate_ref"]}
    ts = arr(S["imu"], "ts")
    rr = np.array([RR.get(int(x), [np.nan]*3) for x in ts]) * R2D
    fs = 1.0/np.median(np.diff(t))

    # window
    if len(argv) >= 4:
        w0, w1 = float(argv[2]), float(argv[3])
    else:
        if "tof_b" in S:
            tt = (arr(S["tof_b"], "ts")-t0)/1e6
            td = arr(S["tof_b"], "distance")
            fly = td > 0.12
            w0 = tt[fly][0] if fly.any() else t[0]
            w1 = tt[fly][-1] if fly.any() else t[-1]
        else:
            w0, w1 = t[0], t[-1]
    m = (t >= w0) & (t <= w1)
    g = gyro[m]; gr = graw[m]; ar = araw[m]; c = rr[m]; tw = t[m]
    v = ~np.isnan(c).any(axis=1)
    g, gr, ar, c, tw = g[v], gr[v], ar[v], c[v], tw[v]
    print(f"\n{'='*72}\n{name}  fs={fs:.0f}Hz  window {w0:.1f}-{w1:.1f}s  N={len(g)}  scipy={HAVE_SCIPY}")

    # ---- A/B: dominant frequencies ----
    print("\n  A. gyro PSD dominant peaks (per axis)")
    for k, ax in enumerate(AXES):
        f, p = welch_psd(g[:, k], fs)
        sel = f > 0.5
        kk = np.argmax(p[sel]); fp = f[sel][kk]
        tot = _trap(p[sel], f[sel])
        hi = _trap(p[sel & (f > FC)], f[sel & (f > FC)])
        print(f"     {ax:>6s}: peak={fp:5.1f}Hz   energy>{FC:.0f}Hz = {100*hi/max(tot,1e-9):4.0f}%   "
              f"rms={np.sqrt(np.mean(g[:,k]**2)):5.1f} deg/s")
    print("\n  B. accel_raw PSD dominant peaks (vibration)")
    for k, ax in enumerate("xyz"):
        f, p = welch_psd(ar[:, k]-ar[:, k].mean(), fs)
        sel = f > 0.5
        kk = np.argmax(p[sel]); fp = f[sel][kk]
        print(f"     accel_{ax}: peak={fp:5.1f}Hz   std={ar[:,k].std():5.2f} m/s^2")
    # gyro vs gyro_raw difference => is there gyro filtering?
    dlt = np.sqrt(np.mean((g-gr)**2, axis=0))
    print(f"\n  gyro vs gyro_raw RMS diff (deg/s): roll={dlt[0]:.2f} pitch={dlt[1]:.2f} yaw={dlt[2]:.2f}"
          f"   ({'filtered' if dlt.max()>0.5 else 'identical=no LPF/notch on logged gyro'})")

    # ---- C: band split ----
    print(f"\n  C. band split at {FC}Hz  (pilot band vs oscillation band)")
    print(f"     {'axis':>6s}{'cmd_track_low':>16s}{'cmd_track_full':>16s}"
          f"{'meas_rms_low':>14s}{'meas_rms_high':>14s}{'cmd_rms':>10s}")
    for k, ax in enumerate(AXES):
        clo, chi = butter_split(c[:, k], fs, FC)
        glo, ghi = butter_split(g[:, k], fs, FC)
        xc_low = xcorr_peak(clo, glo)
        xc_full = xcorr_peak(c[:, k], g[:, k])
        print(f"     {ax:>6s}{xc_low:>16.2f}{xc_full:>16.2f}"
              f"{np.sqrt(np.mean(glo**2)):>14.1f}{np.sqrt(np.mean(ghi**2)):>14.1f}"
              f"{np.sqrt(np.mean(c[:,k]**2)):>10.1f}")

    # ---- D: cross-axis matrix (zero-lag corr cmd_i vs meas_j) ----
    print("\n  D. cross-axis corr  cmd(row) -> meas(col)   [diagonal should dominate]")
    print("            meas_roll  meas_pitch   meas_yaw")
    for i, axi in enumerate(AXES):
        cells = []
        for j in range(3):
            clo, _ = butter_split(c[:, i], fs, FC)
            glo, _ = butter_split(g[:, j], fs, FC)
            cells.append(xcorr_peak(clo, glo))
        print(f"   cmd_{axi:5s}" + "".join(f"{x:>11.2f}" for x in cells))

    # ---- E: figures: PSD + spectrogram of worst axis (roll) ----
    fig, ax = plt.subplots(2, 1, figsize=(11, 9))
    for k, axn in enumerate(AXES):
        f, p = welch_psd(g[:, k], fs)
        ax[0].semilogy(f[f>0.5], p[f>0.5], lw=1.0, label=f"gyro {axn}")
    ax[0].axvline(FC, color="k", ls=":", lw=0.8, label=f"{FC}Hz split")
    ax[0].set_xlim(0, 60); ax[0].set_xlabel("Hz"); ax[0].set_ylabel("PSD (deg/s)^2/Hz")
    ax[0].set_title(f"{name}  gyro PSD  [{w0:.1f}-{w1:.1f}s]"); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    # spectrogram of roll
    if HAVE_SCIPY:
        f, tspec, Sxx = ss.spectrogram(g[:, 0], fs=fs, nperseg=256, noverlap=192)
        ax[1].pcolormesh(tspec+w0, f, 10*np.log10(Sxx+1e-9), shading="auto", cmap="magma")
        ax[1].set_ylim(0, 60); ax[1].set_ylabel("Hz"); ax[1].set_xlabel("time [s]")
        ax[1].set_title("roll-gyro spectrogram (dB)")
    fig.tight_layout()
    p1 = os.path.join(out, os.path.splitext(name)[0]+"_spectral.png")
    fig.savefig(p1, dpi=110); plt.close(fig)
    print(f"\n  figure -> {p1}")


if __name__ == "__main__":
    main(sys.argv[1:])
