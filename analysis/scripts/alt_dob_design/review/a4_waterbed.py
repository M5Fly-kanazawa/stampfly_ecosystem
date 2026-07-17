#!/usr/bin/env python3
"""A4: Direct waterbed check on log A (avg025s init): Welch PSD band-power ratio
DOB fc1.5 / baseline for alt and for thrust_cmd, 4 bands.
"""
import sys
import numpy as np
from scipy.signal import welch

sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/alt_dob_design")
sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/yaw_nt_kanazawa")
sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts")
import step2_dob_design as s2
import step1_actuation_id as s1

FS = s2.FS
dA = s1.load_jsonl(str(s2.LOG_A))
fw = s1.detect_flight_window(dA["ctrl_ref"]["ts"], dA["ctrl_ref"]["motor_duty"])
win = (fw[0] + 3.0, fw[1] - 3.0)
L = s2.load_full_log(s2.LOG_A, win)
dext = s2.compute_dext_all(L, "acc")

cfg_base = s2.default_cfg(dob_enabled=False)
cfg_dob = s2.default_cfg(dob_enabled=True, dob_fc=1.5, dob_order=2, washout_enabled=True,
                         dob_init_mode="avg025s")
tr_base = s2.replay_all(L, dext, cfg_base)
tr_dob = s2.replay_all(L, dext, cfg_dob)


def avg_psd(traces, key, sp_sub):
    """Duration-weighted Welch PSD across segments, nperseg up to 50 s so the
    0.02-0.1 Hz band is resolvable (0.02 Hz resolution)."""
    f_common, Pxx_sum, dur_sum = None, None, 0.0
    for t in traces:
        x = t[key] - (sp_sub if sp_sub is not None else np.mean(t[key]))
        n = len(x)
        nper = min(n, int(FS * 50))
        f, Pxx = welch(x, fs=FS, nperseg=nper)
        if f_common is None:
            f_common = f
        Pxx_sum = (np.interp(f_common, f, Pxx) * n if Pxx_sum is None
                   else Pxx_sum + np.interp(f_common, f, Pxx) * n)
        dur_sum += n
    return f_common, Pxx_sum / dur_sum


BANDS = [(0.02, 0.1), (0.1, 0.5), (0.5, 2.0), (2.0, 5.0)]

for key, label, sp in (("alt", "alt [m]", L["sp"]), ("thrust_cmd", "thrust_cmd [N]", None)):
    f_b, P_b = avg_psd(tr_base, key, sp)
    f_d, P_d = avg_psd(tr_dob, key, sp)
    print("== %s ==" % label)
    print("%-14s %14s %14s %10s" % ("band [Hz]", "base RMS", "DOB RMS", "power ratio"))
    for lo, hi in BANDS:
        m = (f_b >= lo) & (f_b <= hi)
        pb = np.trapezoid(P_b[m], f_b[m])
        pd = np.trapezoid(np.interp(f_b[m], f_d, P_d), f_b[m])
        unit = 1000.0  # mm or mN
        print("%-14s %11.2f %s %11.2f %s %10.3f %s" %
              ("%.2f-%.1f" % (lo, hi), np.sqrt(pb) * unit, "mm" if key == "alt" else "mN",
               np.sqrt(pd) * unit, "mm" if key == "alt" else "mN",
               pd / pb, "<-- WORSE" if pd / pb > 1.0 else ""))
    print()
