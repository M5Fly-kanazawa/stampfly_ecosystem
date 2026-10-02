#!/usr/bin/env python3
"""Plot the first flip of two SILS runs (baseline vs chosen profile) on the same plant.
同じプラントでの 2 つの SILS 実行（基準 vs 採用プロファイル）の最初の宙返りを描く。

Evidence for docs/plans/flip-maneuver-plan.md section 5.7. Panels: rate target vs true rate,
rotation angle about the flip axis, flip-axis torque with the 7 mN*m limit. Time origin = the
rate target leaving 0.
flip-maneuver-plan.md 5.7 節の根拠。パネル: レート目標と真のレート、回転軸まわりの回転角、
回転軸トルクと 7 mN·m 上限。時刻の原点 = レート目標が 0 を離れた時刻。

Usage: python3 flip_profile_plots.py --axis roll --out flip_profile_roll.png \
         --run "unfixed trapezoid (fails like the hardware)=<bundle.sflog.zip>" \
         --run "planned 360 deg profile=<bundle.sflog.zip>"
"""
import argparse
import os
import shutil
import tempfile
import zipfile

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

TORQUE_LIMIT_NM = 7.0e-3


def load(path, axis):
    d = tempfile.mkdtemp()
    try:
        zipfile.ZipFile(path).extractall(d)
        rd = lambda n: np.genfromtxt(os.path.join(d, n + ".csv"), delimiter=",", names=True)
        tr, rr, co = rd("truth"), rd("rate_ref"), rd("ctrl_output")
    finally:
        shutil.rmtree(d, ignore_errors=True)
    key = "roll" if axis == "roll" else "pitch"
    t_rr = rr["timestamp_us"] / 1e6
    ref = np.degrees(rr[f"rate_ref_{key}"])
    t0 = t_rr[np.where((np.abs(ref) > 1.0) & (t_rr > 20.0))[0][0]]
    t_tr = tr["timestamp_us"] / 1e6
    rate = np.degrees(tr["rate_x" if axis == "roll" else "rate_y"])
    sign = 1.0 if ref[np.argmax(np.abs(ref) * (t_rr > t0) * (t_rr < t0 + 1))] > 0 else -1.0
    win = (t_tr >= t0) & (t_tr <= t0 + 1.2)
    dt = np.median(np.diff(t_tr))
    angle = np.cumsum(sign * rate[win]) * dt
    t_co = co["timestamp_us"] / 1e6
    tq = co["torque_roll" if axis == "roll" else "torque_pitch"] * 1e3
    return dict(t_ref=t_rr - t0, ref=sign * ref, t=t_tr[win] - t0, rate=sign * rate[win], angle=angle,
                t_tq=t_co - t0, tq=sign * tq)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", choices=("roll", "pitch"), default="roll")
    ap.add_argument("--out", required=True)
    ap.add_argument("--run", action="append", required=True, help="LABEL=PATH (.sflog.zip)")
    args = ap.parse_args()
    runs = []
    for item in args.run:
        label, path = item.rsplit("=", 1)
        runs.append((label, load(path, args.axis)))

    fig, ax = plt.subplots(3, 1, figsize=(8.5, 9), sharex=True)
    colors = ["tab:red", "tab:blue", "tab:green"]
    for (label, r), c in zip(runs, colors):
        ax[0].plot(r["t_ref"], r["ref"], "--", color=c, lw=1.0, label=f"{label}: target")
        ax[0].plot(r["t"], r["rate"], "-", color=c, lw=1.6, label=f"{label}: actual")
        ax[1].plot(r["t"], r["angle"], "-", color=c, lw=1.6, label=label)
        ax[2].plot(r["t_tq"], r["tq"], "-", color=c, lw=1.2, label=label)
    ax[0].set_ylabel(f"{args.axis} rate [deg/s]")
    ax[0].set_xlim(-0.05, 1.0)
    ax[0].legend(fontsize=7, loc="upper right")
    ax[1].axhline(360, color="k", lw=0.8, ls=":")
    ax[1].set_ylabel("rotation angle [deg]")
    ax[1].legend(fontsize=8, loc="upper left")
    ax[2].axhline(TORQUE_LIMIT_NM * 1e3, color="k", lw=0.8, ls=":")
    ax[2].axhline(-TORQUE_LIMIT_NM * 1e3, color="k", lw=0.8, ls=":", label="torque limit +-7 mN*m")
    ax[2].set_ylabel("flip-axis torque [mN*m]")
    ax[2].set_xlabel("time from rate target leaving 0 [s]")
    ax[2].legend(fontsize=8, loc="upper right")
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle(f"{args.axis} flip on the fitted SILS plant (flip_fit_2026_10), first flip", fontsize=10)
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)


if __name__ == "__main__":
    main()
