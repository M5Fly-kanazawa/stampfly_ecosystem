#!/usr/bin/env python3
"""A2: Log C burst at raw ts ~57.86-57.92s: real dynamic event vs measurement artifact.
判別: 実力学イベントなら posvel/ToF に Δvz≈-2.7 m/s・高度数十cm低下が見えるはず。
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/yaw_nt_kanazawa")
from yawlib import load_jsonl

LOG = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs/stampfly_udp_20260627T164611.jsonl"
OUT = "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/cb1a9c28-b337-4d5f-8a34-584035013b55/scratchpad"

d = load_jsonl(LOG)
T0, T1 = 55.86, 59.92          # +-2s around 57.86-57.92
BC0, BC1 = 57.80, 58.00        # burst core window (slightly padded)

imu = d["imu"]
m = (imu["ts"] >= T0) & (imu["ts"] <= T1)
t_i = imu["ts"][m]
acc = imu["accel"][m]          # raw (bias-uncorrected); diffs cancel bias
gyr = imu["gyro"][m]

# (i) adjacent-sample accel diffs
dacc = np.diff(acc, axis=0)
dacc_norm = np.linalg.norm(dacc, axis=1)
t_d = t_i[1:]
i_max = int(np.argmax(dacc_norm))
print("=== (i) accel adjacent-sample diffs, window %.2f-%.2fs ===" % (T0, T1))
print("max |d accel| (vector norm) = %.2f m/s^2 at ts=%.4fs" % (dacc_norm[i_max], t_d[i_max]))
for ax, name in enumerate("xyz"):
    j = int(np.argmax(np.abs(dacc[:, ax])))
    print("  axis %s: max|diff|=%.2f m/s^2 at ts=%.4f  (window std of diff=%.3f)" %
          (name, abs(dacc[j, ax]), t_d[j], np.std(dacc[:, ax])))
# background: same stats excluding burst core
bg = (t_d < BC0) | (t_d > BC1)
print("background (excl. %.2f-%.2fs): max|d accel|=%.2f, std=%.3f m/s^2" %
      (BC0, BC1, dacc_norm[bg].max(), np.std(dacc_norm[bg])))
core = ~bg
print("burst core %.2f-%.2fs: n=%d samples, max|d accel|=%.2f m/s^2" %
      (BC0, BC1, core.sum(), dacc_norm[core].max() if core.any() else float("nan")))

# raw accel z (and norm) levels in the core vs background
acc_norm = np.linalg.norm(acc, axis=1)
print("accel vector-norm: core min/max = %.2f / %.2f ; background mean=%.2f std=%.2f" %
      (acc_norm[1:][core].min(), acc_norm[1:][core].max(),
       acc_norm[1:][bg].mean(), acc_norm[1:][bg].std()))

# (ii) gyro in the same window
dgyr = np.diff(gyr, axis=0)
dgyr_norm = np.linalg.norm(dgyr, axis=1)
print("\n=== (ii) gyro same window ===")
print("gyro |sample| max in core = %.3f rad/s ; background max = %.3f rad/s" %
      (np.linalg.norm(gyr[1:][core], axis=1).max(), np.linalg.norm(gyr[1:][bg], axis=1).max()))
print("gyro adjacent diff: core max=%.3f rad/s, background max=%.3f rad/s, background std=%.4f" %
      (dgyr_norm[core].max(), dgyr_norm[bg].max(), np.std(dgyr_norm[bg])))

# (iii) posvel alt and vz
pv = d["posvel"]
mp = (pv["ts"] >= T0) & (pv["ts"] <= T1)
t_p = pv["ts"][mp]
alt = -pv["pos"][mp, 2]
vz = -pv["vel"][mp, 2]
core_p = (t_p >= BC0) & (t_p <= BC1)
bg_p = ~core_p
print("\n=== (iii) posvel alt / vz ===")
print("alt: window min/max = %.3f / %.3f m (p2p %.1f mm)" % (alt.min(), alt.max(), (alt.max()-alt.min())*1000))
if core_p.any():
    print("alt in burst core: min/max = %.3f / %.3f m; alt right before core=%.3f, right after=%.3f  (change %.1f mm)"
          % (alt[core_p].min(), alt[core_p].max(),
             alt[t_p < BC0][-1] if (t_p < BC0).any() else np.nan,
             alt[t_p > BC1][0] if (t_p > BC1).any() else np.nan,
             ((alt[t_p > BC1][0] - alt[t_p < BC0][-1]) * 1000) if ((t_p > BC1).any() and (t_p < BC0).any()) else np.nan))
    print("vz  in burst core: min/max = %.3f / %.3f m/s" % (vz[core_p].min(), vz[core_p].max()))
print("vz: window min/max = %.3f / %.3f m/s; max |dvz| over any 0.4s = ...", )
# largest vz change over any 0.4s span in the window
dt_med = np.median(np.diff(t_p))
n04 = max(1, int(round(0.4 / dt_med)))
dvz04 = vz[n04:] - vz[:-n04]
jmax = int(np.argmax(np.abs(dvz04)))
print("largest |Delta vz| over 0.4s anywhere in window = %.3f m/s at ts=%.2fs (predicted if real: ~2.7 m/s)"
      % (abs(dvz04[jmax]), t_p[jmax]))
# same restricted to burst core start
in_core_start = (t_p[:-n04] >= BC0 - 0.1) & (t_p[:-n04] <= BC1)
if in_core_start.any():
    print("largest |Delta vz| over 0.4s starting inside burst core = %.3f m/s"
          % np.max(np.abs(dvz04[in_core_start])))

# (iv) ToF
tof = d["tof_b"]
mt = (tof["ts"] >= T0) & (tof["ts"] <= T1)
t_t = tof["ts"][mt]
dist = tof["distance"][mt]
print("\n=== (iv) tof_b ===")
print("n=%d samples; distance min/max = %.3f / %.3f (p2p %.1f mm)"
      % (mt.sum(), dist.min(), dist.max(), (dist.max()-dist.min())*1000))
core_t = (t_t >= BC0) & (t_t <= BC1)
if core_t.any():
    print("tof in burst core: min/max = %.3f / %.3f" % (dist[core_t].min(), dist[core_t].max()))
before = dist[t_t < BC0]
after = dist[t_t > BC1]
if len(before) and len(after):
    print("tof last-before=%.3f first-after=%.3f (change %.1f mm)"
          % (before[-1], after[0], (after[0]-before[-1])*1000))

# telemetry gaps in the window (any of the streams)
print("\n=== telemetry continuity in window ===")
for name, ts in (("imu", t_i), ("posvel", t_p), ("tof_b", t_t)):
    gaps = np.diff(ts)
    print("%s: median dt=%.4fs, max gap=%.3fs at ts=%.3f" % (name, np.median(gaps), gaps.max(), ts[np.argmax(gaps)]))

# plot
fig, axes = plt.subplots(5, 1, figsize=(11, 12), sharex=True)
axes[0].plot(t_i, acc[:, 0], lw=0.5, label="ax")
axes[0].plot(t_i, acc[:, 1], lw=0.5, label="ay")
axes[0].plot(t_i, acc[:, 2], lw=0.5, label="az")
axes[0].set_ylabel("accel raw [m/s2]"); axes[0].legend(fontsize=7)
axes[1].plot(t_d, dacc_norm, lw=0.5, color="tab:red")
axes[1].set_ylabel("|d accel| adj [m/s2]")
axes[2].plot(t_i, gyr[:, 0], lw=0.5, label="gx")
axes[2].plot(t_i, gyr[:, 1], lw=0.5, label="gy")
axes[2].plot(t_i, gyr[:, 2], lw=0.5, label="gz")
axes[2].set_ylabel("gyro [rad/s]"); axes[2].legend(fontsize=7)
axes[3].plot(t_p, alt, lw=0.8, label="alt=-pos_z")
axes[3].plot(t_t, dist, ".", ms=3, label="tof_b dist")
axes[3].set_ylabel("alt / tof [m]"); axes[3].legend(fontsize=7)
axes[4].plot(t_p, vz, lw=0.8, color="tab:green")
axes[4].set_ylabel("vz=-vel_z [m/s]"); axes[4].set_xlabel("raw ts [s]")
for ax in axes:
    ax.axvspan(BC0, BC1, color="orange", alpha=0.2)
    ax.grid(alpha=0.3)
fig.suptitle("Log C burst window: accel/gyro/posvel/ToF, raw ts %.2f-%.2fs (orange=burst core)" % (T0, T1))
fig.tight_layout()
fig.savefig(OUT + "/a2_burst_window.png", dpi=130)
print("\nfigure: %s/a2_burst_window.png" % OUT)
