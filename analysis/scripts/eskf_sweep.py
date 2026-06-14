#!/usr/bin/env python3
"""
eskf_sweep.py — sweep ESKF parameters on the offline replay harness and plot the
consistency scores, to pick the best params from the DYNAMIC real flight.

オフライン再生ハーネスで ESKF パラメータを掃引し整合性スコアを描画、動的実飛行から最良値を選ぶ。

Build the harness first (g++), then sweep. Usage:
  python3 eskf_sweep.py <events.txt> <out_dir>
"""
import sys, os, subprocess
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(__file__)
ROOT = os.path.join(HERE, "..", "..")
BIN = "/tmp/eskf_replay"


def build():
    inc = ["firmware/vehicle_new/test",
           "firmware/vehicle_new/components/sf_math/include",
           "firmware/vehicle_new/components/sf_estimator_eskf/include",
           "firmware/vehicle_new/components/sf_core/include"]
    cmd = ["g++", "-std=c++17", "-O2"]
    for i in inc:
        cmd += ["-I", os.path.join(ROOT, i)]
    cmd += [os.path.join(HERE, "eskf_replay.cpp"),
            os.path.join(ROOT, "firmware/vehicle_new/components/sf_estimator_eskf/eskf_core.cpp"),
            "-lm", "-o", BIN]
    subprocess.run(cmd, check=True)


def run(events, aatt, tof, lpf=0.0, notch=0.0):
    out = subprocess.run([BIN, str(aatt), str(tof), str(lpf), str(notch)],
                         stdin=open(events), capture_output=True, text=True).stdout
    for line in out.splitlines():
        p = line.split()
        if len(p) >= 13:                       # the CSV result line (skip the ESKF init log)
            v = [float(x) for x in p]
            return dict(aatt=v[0], tof=v[1], lpf=v[2], notch=v[3], ba_drift=v[4],
                        bax=v[5], bay=v[6], baz=v[7], roll_std=v[8], pitch_std=v[9],
                        rej=v[10], tof_nis=v[11], innov=v[12])
    return None


def main(events, out):
    os.makedirs(out, exist_ok=True)
    build()
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))

    # (1) accel_att_noise sweep: χ² reject fraction (target ~5%) + bias drift
    aatts = [0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
    R = [run(events, a, 0.03) for a in aatts]
    rej = [r["rej"] * 100 for r in R]
    bad = [r["ba_drift"] for r in R]
    ax[0, 0].plot(aatts, rej, "-o", label="accel χ² reject %")
    ax[0, 0].axhline(5, color="r", ls="--", label="ideal ~5%")
    ax[0, 0].axvline(0.8, color="gray", ls=":", label="flown 0.8")
    ax[0, 0].set_xlabel("accel_att_noise [m/s²]"); ax[0, 0].set_ylabel("reject %")
    ax[0, 0].set_title("Accel-attitude χ² rejection vs accel_att_noise"); ax[0, 0].legend(fontsize=8); ax[0, 0].grid(alpha=.3)
    ax[0, 1].plot(aatts, bad, "-o", color="tab:green")
    ax[0, 1].axvline(0.8, color="gray", ls=":")
    ax[0, 1].set_xlabel("accel_att_noise [m/s²]"); ax[0, 1].set_ylabel("accel-bias drift [m/s²]")
    ax[0, 1].set_title("Accel-bias drift vs accel_att_noise"); ax[0, 1].grid(alpha=.3)
    # interpolate the accel_att_noise that gives ~5% rejection
    rej_arr = np.array(rej)
    target = None
    for i in range(len(aatts) - 1):
        if (rej_arr[i] - 5) * (rej_arr[i + 1] - 5) <= 0:
            f = (5 - rej_arr[i]) / (rej_arr[i + 1] - rej_arr[i])
            target = aatts[i] + f * (aatts[i + 1] - aatts[i]); break

    # (2) tof_noise sweep: NIS (target ~1) at the chosen accel_att_noise
    a_use = target if target else 1.2
    tofs = [0.008, 0.01, 0.015, 0.02, 0.03, 0.05]
    Rt = [run(events, a_use, tn) for tn in tofs]
    tof_nis = [r["tof_nis"] for r in Rt]
    ax[1, 0].plot(tofs, tof_nis, "-o")
    ax[1, 0].axhline(1.0, color="r", ls="--", label="ideal NIS ~1")
    ax[1, 0].axvline(0.03, color="gray", ls=":", label="flown 0.03")
    ax[1, 0].set_xlabel("tof_noise [m]"); ax[1, 0].set_ylabel("ToF innovation NIS")
    ax[1, 0].set_title(f"ToF NIS vs tof_noise (accel_att={a_use:.2f})"); ax[1, 0].legend(fontsize=8); ax[1, 0].grid(alpha=.3)

    # (3) accel pre-filter for the attitude update: does an LPF / a 12 Hz notch help?
    base = run(events, a_use, 0.03)
    variants = {"none": run(events, a_use, 0.03, 0, 0),
                "LPF 30Hz": run(events, a_use, 0.03, 30, 0),
                "LPF 15Hz": run(events, a_use, 0.03, 15, 0),
                "notch 12Hz": run(events, a_use, 0.03, 0, 12)}
    names = list(variants.keys())
    rejv = [variants[n]["rej"] * 100 for n in names]
    badv = [variants[n]["ba_drift"] for n in names]
    x = np.arange(len(names))
    ax[1, 1].bar(x - 0.2, rejv, 0.4, label="χ² reject %")
    ax2 = ax[1, 1].twinx(); ax2.bar(x + 0.2, badv, 0.4, color="tab:green", label="bias drift")
    ax[1, 1].set_xticks(x); ax[1, 1].set_xticklabels(names, fontsize=8)
    ax[1, 1].set_ylabel("reject %"); ax2.set_ylabel("bias drift [m/s²]")
    ax[1, 1].set_title(f"Accel pre-filter for attitude update (accel_att={a_use:.2f})")
    ax[1, 1].legend(fontsize=8, loc="upper left"); ax2.legend(fontsize=8, loc="upper right"); ax[1, 1].grid(alpha=.3)

    fig.tight_layout(); fig.savefig(f"{out}/07_eskf_sweep.png", dpi=110); plt.close(fig)

    print(f"=== accel_att_noise giving ~5% rejection: {target:.2f} ===" if target else "no 5% crossing")
    print("aatt  rej%  ba_drift  roll_std pitch_std")
    for r in R:
        print(f"  {r['aatt']:.1f}  {r['rej']*100:5.1f}  {r['ba_drift']:.3f}   {r['roll_std']:.2f}    {r['pitch_std']:.2f}")
    print("tof_noise  NIS")
    for r in Rt:
        print(f"  {r['tof']:.3f}    {r['tof_nis']:.3f}")
    print("filter      rej%  ba_drift  roll_std pitch_std")
    for n in names:
        v = variants[n]
        print(f"  {n:10s}  {v['rej']*100:5.1f}  {v['ba_drift']:.3f}   {v['roll_std']:.2f}    {v['pitch_std']:.2f}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
