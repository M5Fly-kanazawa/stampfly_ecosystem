#!/usr/bin/env python3
"""Coarse timeline of a SILS scenario bundle: altitude, tilt, commanded thrust / mg, duty, vbat, rates.
Usage: scn_timeline.py <out_scn_dir> [t0 t1 [step_s]]"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "lib"))
import sflog  # noqa: E402

MG = 0.037 * 9.80665
d = Path(sys.argv[1])
t0 = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
t1 = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
step = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5
log = sflog.load(sorted(d.glob("*.sflog.zip"))[-1])
S = log.streams
tr = S["truth"].sort_values("timestamp_us")
t = tr["timestamp_us"].to_numpy() * 1e-6
alt = -tr["pos_z"].to_numpy()
q = tr[["quat_w", "quat_x", "quat_y", "quat_z"]].to_numpy()
tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
rate = np.degrees(np.linalg.norm(tr[["rate_x", "rate_y", "rate_z"]].to_numpy(), axis=1))
co = S["ctrl_output"].sort_values("timestamp_us")
tc = co["timestamp_us"].to_numpy() * 1e-6
th = co["thrust"].to_numpy() / MG
mo = S["motor"].sort_values("timestamp_us")
tm = mo["timestamp_us"].to_numpy() * 1e-6
D = mo[["duty_FR", "duty_RR", "duty_RL", "duty_FL"]].to_numpy()
st = S["status"].sort_values("timestamp_us")
ts = st["timestamp_us"].to_numpy() * 1e-6
vb = st["voltage"].to_numpy()
print("%6s %6s %6s %7s %6s %6s %6s %7s" % ("t", "alt", "tilt", "thr/mg", "dutyMx", "dutyMn", "vbat", "rate"))
x = max(t0, t[0])
while x < min(t1, t[-1]):
    m = (t >= x) & (t < x + step)
    mc = (tc >= x) & (tc < x + step)
    mm = (tm >= x) & (tm < x + step)
    ms = (ts >= x) & (ts < x + step)
    if m.any():
        print("%6.2f %6.3f %6.1f %7.3f %6.3f %6.3f %6.3f %7.1f" % (
            x, alt[m].mean(), tilt[m].max(), th[mc].mean() if mc.any() else np.nan,
            D[mm].max() if mm.any() else np.nan, D[mm].min() if mm.any() else np.nan,
            vb[ms].mean() if ms.any() else np.nan, rate[m].max()))
    x += step
