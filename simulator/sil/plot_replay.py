#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
#
# M7 differential diagnosis — visualize log_replay output.
# Reads the per-cycle comparison CSV emitted by ./log_replay and produces:
#   * per-axis RMSE table (SIL legacy / SIL redesign vs the flight-log reference)
#   * attitude / position / velocity time-series figures
#   * PSD (power spectral density) of the attitude residual (where, in frequency,
#     the SIL fails to track the flight)
# and prints / writes a verdict.
#
# M7 差分診断 — log_replay 出力の可視化。
# ./log_replay が出す周期別比較CSVを読み、軸別RMSE表・姿勢/位置/速度の時系列図・
# 姿勢残差のPSD(パワースペクトル密度=どの周波数で追従に失敗しているか)を生成し、
# 判定を出力する。

import csv
import math
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from scipy.signal import welch
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False

FS = 400.0  # replay sample rate [Hz] / 再生サンプルレート


def wrap180(a):
    """Wrap degrees to [-180, 180]. 角度を[-180,180]度に折り返す。"""
    return (a + 180.0) % 360.0 - 180.0


def load(path):
    cols = {}
    with open(path) as f:
        r = csv.DictReader(f)
        names = r.fieldnames
        for n in names:
            cols[n] = []
        for row in r:
            for n in names:
                cols[n].append(float(row[n]))
    return {k: np.asarray(v) for k, v in cols.items()}


def rmse(err):
    return float(np.sqrt(np.mean(err ** 2))) if len(err) else 0.0


def main():
    ap = argparse.ArgumentParser(description="Plot log_replay diagnostics (M7).")
    ap.add_argument("diag", help="log_replay comparison CSV")
    ap.add_argument("--skip", type=float, default=6.0,
                    help="seconds to skip before RMSE (warm-start + takeoff)")
    ap.add_argument("--outdir", default="docs/images", help="figure output dir")
    ap.add_argument("--tag", default="run", help="figure filename prefix")
    args = ap.parse_args()

    d = load(args.diag)
    t = d["t"]
    mask = t >= args.skip
    os.makedirs(args.outdir, exist_ok=True)

    # ----- RMSE table (legacy & redesign vs reference) -----
    axes = [
        ("roll  [deg]", "ref_roll", "old_roll", "new_roll", True),
        ("pitch [deg]", "ref_pitch", "old_pitch", "new_pitch", True),
        ("yaw   [deg]", "ref_yaw", "old_yaw", "new_yaw", True),
        ("pos_x [m]  ", "ref_px", "old_px", "new_px", False),
        ("pos_y [m]  ", "ref_py", "old_py", "new_py", False),
        ("pos_z [m]  ", "ref_pz", "old_pz", "new_pz", False),
        ("vel_x [m/s]", "ref_vx", "old_vx", "new_vx", False),
        ("vel_y [m/s]", "ref_vy", "old_vy", "new_vy", False),
        ("vel_z [m/s]", "ref_vz", "old_vz", "new_vz", False),
    ]
    print(f"\n=== M7 RMSE (t >= {args.skip:.1f}s) ===")
    print(f"{'axis':<12}| {'legacy(old)':>13} | {'redesign(new)':>13}")
    md_rows = []
    att_old, att_new = [], []
    for label, ref, old, new, is_ang in axes:
        if is_ang:
            eo = wrap180(d[old][mask] - d[ref][mask])
            en = wrap180(d[new][mask] - d[ref][mask])
        else:
            eo = d[old][mask] - d[ref][mask]
            en = d[new][mask] - d[ref][mask]
        ro, rn = rmse(eo), rmse(en)
        print(f"{label:<12}| {ro:>13.4f} | {rn:>13.4f}")
        md_rows.append(f"| {label.strip()} | {ro:.4f} | {rn:.4f} |")
        if is_ang:
            att_old.append(ro)
            att_new.append(rn)

    att_old_mean = float(np.mean(att_old))
    verdict = ("再現 (REPRODUCED): 推定器はソフト的に決定論的、残差はHW由来"
               if att_old_mean < 1.0 else
               "非再現 (NOT reproduced): ログESKF入力の外の要因に依存")
    print(f"\nlegacy mean attitude RMSE = {att_old_mean:.4f} deg -> {verdict}")

    # Emit a markdown snippet for pasting into the journal.
    # ジャーナル貼り付け用の markdown 断片を出力。
    md = ["| 軸 | 旧ESKF(SIL) RMSE | 新ESKF(SIL) RMSE |",
          "|----|------------------|------------------|"]
    md += md_rows
    md_path = os.path.join(args.outdir, f"{args.tag}_rmse.md")
    with open(md_path, "w") as f:
        f.write("\n".join(md) + "\n")
    print(f"[plot_replay] RMSE markdown -> {md_path}")

    # ----- Figure 1: attitude time series -----
    fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for i, (name, ref, old, new) in enumerate([
            ("roll", "ref_roll", "old_roll", "new_roll"),
            ("pitch", "ref_pitch", "old_pitch", "new_pitch"),
            ("yaw", "ref_yaw", "old_yaw", "new_yaw")]):
        ax[i].plot(t, d[ref], "k-", lw=1.2, label="flight log (ref)")
        ax[i].plot(t, d[old], "C3-", lw=0.9, alpha=0.8, label="SIL legacy (old)")
        ax[i].plot(t, d[new], "C0-", lw=0.9, alpha=0.8, label="SIL redesign (new)")
        ax[i].set_ylabel(f"{name} [deg]")
        ax[i].grid(True, alpha=0.3)
        if i == 0:
            ax[i].legend(loc="upper right", fontsize=8, ncol=3)
    ax[-1].set_xlabel("time [s]")
    fig.suptitle(f"Attitude: SIL replay vs flight log  ({args.tag})")
    fig.tight_layout()
    f1 = os.path.join(args.outdir, f"{args.tag}_attitude.png")
    fig.savefig(f1, dpi=110)
    plt.close(fig)

    # ----- Figure 2: position & velocity (left=full incl. divergence, right=zoom) -----
    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    for col, (lbl, ref, old, new) in enumerate([
            ("pos_x [m]", "ref_px", "old_px", "new_px"),
            ("vel_x [m/s]", "ref_vx", "old_vx", "new_vx")]):
        # full scale (shows legacy divergence)
        a0 = ax[col][0]
        a0.plot(t, d[ref], "k-", lw=1.2, label="flight log")
        a0.plot(t, d[old], "C3-", lw=1.0, label="SIL legacy")
        a0.plot(t, d[new], "C0-", lw=1.0, label="SIL redesign")
        a0.set_ylabel(lbl)
        a0.set_title(f"{lbl} — full scale")
        a0.grid(True, alpha=0.3)
        a0.legend(fontsize=8)
        # zoom on reference + redesign only
        a1 = ax[col][1]
        a1.plot(t, d[ref], "k-", lw=1.2, label="flight log")
        a1.plot(t, d[new], "C0-", lw=1.0, label="SIL redesign")
        lo = min(d[ref].min(), d[new].min())
        hi = max(d[ref].max(), d[new].max())
        pad = 0.1 * (hi - lo + 1e-6)
        a1.set_ylim(lo - pad, hi + pad)
        a1.set_title(f"{lbl} — zoom (ref + redesign)")
        a1.grid(True, alpha=0.3)
        a1.legend(fontsize=8)
    for a in ax[-1]:
        a.set_xlabel("time [s]")
    fig.suptitle(f"Position & velocity: SIL replay vs flight log  ({args.tag})")
    fig.tight_layout()
    f2 = os.path.join(args.outdir, f"{args.tag}_posvel.png")
    fig.savefig(f2, dpi=110)
    plt.close(fig)

    # ----- Figure 3: PSD of attitude residual -----
    f3 = None
    if HAVE_SCIPY:
        fig, ax = plt.subplots(1, 3, figsize=(13, 4))
        for i, (name, ref, old, new) in enumerate([
                ("roll", "ref_roll", "old_roll", "new_roll"),
                ("pitch", "ref_pitch", "old_pitch", "new_pitch"),
                ("yaw", "ref_yaw", "old_yaw", "new_yaw")]):
            eo = wrap180(d[old][mask] - d[ref][mask])
            en = wrap180(d[new][mask] - d[ref][mask])
            nper = min(4096, len(eo))
            if nper < 16:
                continue
            fo, po = welch(eo, fs=FS, nperseg=nper)
            fn, pn = welch(en, fs=FS, nperseg=nper)
            ax[i].loglog(fo, po, "C3-", label="legacy residual")
            ax[i].loglog(fn, pn, "C0-", label="redesign residual")
            ax[i].set_title(f"{name} residual PSD")
            ax[i].set_xlabel("frequency [Hz]")
            ax[i].set_ylabel("PSD [deg²/Hz]")
            ax[i].grid(True, which="both", alpha=0.3)
            if i == 0:
                ax[i].legend(fontsize=8)
        fig.suptitle(f"Attitude residual PSD (SIL − flight)  ({args.tag})")
        fig.tight_layout()
        f3 = os.path.join(args.outdir, f"{args.tag}_psd.png")
        fig.savefig(f3, dpi=110)
        plt.close(fig)
    else:
        print("[plot_replay] scipy not available -> skipping PSD figure", file=sys.stderr)

    print(f"[plot_replay] figures -> {f1}\n                         {f2}" +
          (f"\n                         {f3}" if f3 else ""))


if __name__ == "__main__":
    main()
