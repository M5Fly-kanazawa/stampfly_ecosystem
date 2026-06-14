#!/usr/bin/env python3
"""
wobble_bench.py — measure the hover wobble from a wobble_bench SIL run's trajectory.csv.

Window [12,28] s of wobble_bench.scn (POS_HOLD hover under 1-3 Hz turbulence): reports the
attitude RMS (the wobble), rate RMS, motor-duty RMS (effort/vibration), and position drift.
Compare control methods (M0 baseline → M1 …) under the SAME --turbulence/--noise/--seed.

ふらつき最小化研究の測定。wobble_bench の trajectory.csv から窓[12,28]s の姿勢RMS等を出す。

Usage: python3 wobble_bench.py <out_scn_wobble_bench/trajectory.csv> [t0 t1]
"""
import sys
import numpy as np


def main(path, t0=12.0, t1=28.0):
    d = np.genfromtxt(path, delimiter=",", names=True)
    t = d["t"]
    m = (t >= t0) & (t <= t1)
    if m.sum() < 10 or d["alt"][m].mean() < 0.3:
        print(f"  NOT HOVERING (alt mean={d['alt'][m].mean():.3f}m) — run failed?")
        return
    roll, pitch, tt = d["roll"][m], d["pitch"][m], t[m]
    dr, dp = np.gradient(roll, tt), np.gradient(pitch, tt)
    duty = np.vstack([d["m0"][m], d["m1"][m], d["m2"][m], d["m3"][m]])
    print(f"window [{t0},{t1}]s  n={m.sum()}  alt={d['alt'][m].mean():.2f}m")
    print(f"  ATT RMS   : roll={roll.std():.3f} deg  pitch={pitch.std():.3f} deg   <- wobble")
    print(f"  RATE RMS  : droll={dr.std():.1f}  dpitch={dp.std():.1f} deg/s")
    print(f"  DUTY std  : {duty.std(axis=1).mean():.4f}   (effort / vibration)")
    print(f"  POS drift : px={d['px'][m].max()-d['px'][m].min():.2f}  py={d['py'][m].max()-d['py'][m].min():.2f} m")


if __name__ == "__main__":
    args = sys.argv[1:]
    path = args[0]
    t0, t1 = (float(args[1]), float(args[2])) if len(args) >= 3 else (12.0, 28.0)
    main(path, t0, t1)
