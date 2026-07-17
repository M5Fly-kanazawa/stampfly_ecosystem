#!/usr/bin/env python3
"""A3: Is the DOB -56% gain an artifact of the filter-state init mode?
Log A longest valid segment, replay baseline & DOB fc=1.5 with sample0 and avg025s init,
report alt std over the LATE 50% of the segment only.
"""
import sys
import numpy as np

sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/alt_dob_design")
sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/yaw_nt_kanazawa")
sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts")
import step2_dob_design as s2
import step1_actuation_id as s1

# Reproduce main()'s log-A window: auto-detected flight window, 3s edge trim
dA = s1.load_jsonl(str(s2.LOG_A))
fw = s1.detect_flight_window(dA["ctrl_ref"]["ts"], dA["ctrl_ref"]["motor_duty"])
win = (fw[0] + 3.0, fw[1] - 3.0)
L = s2.load_full_log(s2.LOG_A, win)
print("log A window %.1f-%.1fs, segments:" % win)
for s, e in L["segs"]:
    print("  %.2f-%.2fs (%.1fs)" % (L["tg"][s], L["tg"][e], (e - s) / s2.FS))

dext = s2.compute_dext_all(L, "acc")
lens = [e - s for s, e, _, _ in dext]
k = int(np.argmax(lens))
s0, e0, d_seg, n_seg = dext[k]
seg_len_s = (e0 - s0) / s2.FS
print("longest segment: idx %d, %.2f-%.2fs (%.1fs)" % (k, L["tg"][s0], L["tg"][e0], seg_len_s))

results = {}
for init in ("sample0", "avg025s"):
    for case, cfg in (("baseline", s2.default_cfg(dob_enabled=False, dob_init_mode=init)),
                      ("dob_fc1.5", s2.default_cfg(dob_enabled=True, dob_fc=1.5, dob_order=2,
                                                   washout_enabled=True, dob_init_mode=init))):
        tr = s2.replay_segment_dob(L, s0, e0, d_seg, n_seg, cfg)
        n = len(tr["alt"])
        half = n // 2
        alt_late = tr["alt"][half:]
        alt_full = tr["alt"]
        results[(init, case)] = dict(
            late_std_mm=float(np.std(alt_late)) * 1000,
            late_rms_mm=float(np.sqrt(np.mean((alt_late - L["sp"]) ** 2))) * 1000,
            full_std_mm=float(np.std(alt_full)) * 1000,
        )

print("\n%-10s %-10s %12s %12s %12s" % ("init", "case", "late_std_mm", "late_rms_mm", "full_std_mm"))
for (init, case), r in results.items():
    print("%-10s %-10s %12.1f %12.1f %12.1f" % (init, case, r["late_std_mm"], r["late_rms_mm"], r["full_std_mm"]))

for init in ("sample0", "avg025s"):
    b = results[(init, "baseline")]["late_std_mm"]
    d = results[(init, "dob_fc1.5")]["late_std_mm"]
    print("init=%s: late-half DOB gain = %+.1f%% (baseline %.1f -> DOB %.1f mm)" % (init, 100 * (d / b - 1), b, d))
