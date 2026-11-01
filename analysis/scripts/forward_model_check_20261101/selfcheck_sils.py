#!/usr/bin/env python3
"""
Self-consistency of tools/sysid/forward_model.py on SILS bundles (part A of the report).
SILS バンドルでの forward_model.py の自己無矛盾性 (報告書 A 部)。

Usage: python selfcheck_sils.py <bundle.sflog.zip> [more bundles...]
Run with PYTHONPATH=<repo root>:<repo root>/lib. / PYTHONPATH にリポジトリ直下と lib を通す。
"""
import sys
import numpy as np
from tools.sysid import forward_model as fm

AXES = ("roll", "pitch", "yaw")
DELAYS_S = np.arange(0, 41) * 1e-3          # extra delay scan 0..40 ms, 1 ms steps
WINDOW_S = 0.5                               # "most dynamic window" length
ABS_FLOOR = 1e-6                             # rad/s^2: below this an axis is not excited


def best_fit(ref, preds, mask, delays):
    """(delay, scale, residual at scale 1 and best delay, residual after scale)."""
    return fm.best_delay_and_scale(ref, preds, delays, mask)


def report(path):
    motion = fm.load_logged_motion(path)
    t_tr, rate_tr, alt = fm.load_truth(path)
    trace, torque = fm.predict_torque_trace(motion)
    air = np.interp(motion.t_s, t_tr, alt) > fm.AIRBORNE_MIN_ALT_M
    refs = {
        "truth": fm.accel_from_rate(motion.t_s, rate_tr, motion.dt, t_src=t_tr),
        "gyro": fm.accel_from_rate(motion.t_s, motion.gyro, motion.dt),
    }
    # plant-aligned prediction (one substep) and a prediction swept over extra delay
    base = fm.accel_from_trace(motion, trace, torque, fm.SILS_LOG_ALIGNMENT_DELAY_S)
    print(f"== {path}\n rows {len(air)}, airborne {int(air.sum())}")
    for name, ref in refs.items():
        for a, axis in enumerate(AXES):
            rms = float(np.sqrt(np.nanmean(ref[air, a] ** 2)))
            if rms < ABS_FLOOR:
                print(f" [{name:5s}] {axis:5s}: not excited (rms {rms:.2e} rad/s^2) -> n/a")
                continue
            whole = fm.relative_residual(ref, base, air)[a]
            # most dynamic window
            nwin = int(WINDOW_S / motion.dt)
            e = np.where(air, ref[:, a] ** 2, 0.0)
            e[~np.isfinite(e)] = 0.0
            cs = np.concatenate([[0], np.cumsum(e)])
            k0 = int(np.argmax(cs[nwin:] - cs[:-nwin]))
            win = np.zeros(len(air), bool); win[k0:k0 + nwin] = True; win &= air
            res_win = fm.relative_residual(ref, base, win)[a]
            preds = [fm.accel_from_trace(motion, trace, torque, fm.SILS_LOG_ALIGNMENT_DELAY_S + d)[:, a]
                     for d in DELAYS_S]
            d_b, s_b, r_b, r_bs = fm.best_delay_and_scale(ref[:, a], preds, DELAYS_S, air)
            print(f" [{name:5s}] {axis:5s}: rms ref {rms:9.3g} | whole {whole:.4f} | "
                  f"window t={motion.t_s[k0]:.2f}s {res_win:.4f} | no-fit(no alignment shift) "
                  f"{fm.relative_residual(ref, fm.accel_from_trace(motion, trace, torque, 0.0), air)[a]:.4f} | "
                  f"best extra delay (on top of alignment) {d_b*1e3:.0f} ms scale {s_b:.4f} res {r_b:.4f}->{r_bs:.4f}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        report(p)
