#!/usr/bin/env python3
"""
Ablation test of claim (4): "400Hz->low-rate decimation requires anti-alias
filtering; without it, amplitude inflates ~2x."

Method: import step1_actuation_id.py as a library (module-level code only
defines functions/constants -- safe, does not run main()), then reproduce
step4's d_ext_acc computation for logs A and C with the SAME code path except
antialias_lowpass() is bypassed (identity), and compare the 0.02-1Hz band RMS
and band_ratio_acc_over_kin to the values already in step1_results.json.
"""
import sys
import numpy as np

sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/alt_dob_design")
sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts")
sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/yaw_nt_kanazawa")

import step1_actuation_id as s1  # noqa: E402
import alt_ti_venue_sim as avs   # noqa: E402
from yawlib import load_jsonl    # noqa: E402


def extend_log_with_fup_NO_AA(path, win):
    """Same as s1.extend_log_with_fup but WITHOUT the antialias_lowpass call
    before decimating native 400Hz f_up onto the FS=100Hz grid (naive
    np.interp decimation only, mirrors what a careless implementation would
    do)."""
    L = avs.load_log(str(path), win)
    d = load_jsonl(str(path))
    imu = d["imu"]
    accel_corr = s1.bias_corrected_accel(imu)
    f_up_native, _ = s1.body_accel_to_f_up(accel_corr, imu["quat"])
    # NOTE: no antialias_lowpass() here -- this is the ablation.
    L["fupg"] = np.interp(L["tg"], imu["ts"], f_up_native)
    return L


def run_one(name, path, win, model):
    L_with = s1.extend_log_with_fup(path, win)     # script's real path (AA filter)
    L_without = extend_log_with_fup_NO_AA(path, win)  # ablated (no AA filter)

    for label, L in [("WITH_AA", L_with), ("WITHOUT_AA", L_without)]:
        per_seg_kin, per_seg_acc, per_seg_pairs = [], [], []
        d_kin_all, d_acc_all = [], []
        for s, e in L["segs"]:
            tg_seg, d_kin_seg, _ = avs.dext_for_segment(L, s, e)
            d_acc_seg = s1.dext_acc_for_segment(L, s, e, model)
            per_seg_kin.append((tg_seg, d_kin_seg))
            per_seg_acc.append((tg_seg, d_acc_seg))
            per_seg_pairs.append((tg_seg, d_kin_seg, d_acc_seg))
            d_kin_all.append(d_kin_seg)
            d_acc_all.append(d_acc_seg)
        rms_kin_band, _, _ = s1.band_rms_from_series_segments(per_seg_kin, avs.FS, 0.02, 1.0)
        rms_acc_band, _, _ = s1.band_rms_from_series_segments(per_seg_acc, avs.FS, 0.02, 1.0)
        coh_band = s1.coherence_band_from_segments(per_seg_pairs, avs.FS, 0.02, 1.0)
        d_acc_std = np.std(np.concatenate(d_acc_all)) * 1000
        d_kin_std = np.std(np.concatenate(d_kin_all)) * 1000
        print(f"  [{name}:{label}] band_rms_kin={rms_kin_band*1000:.2f}mN "
              f"band_rms_acc={rms_acc_band*1000:.2f}mN "
              f"ratio_acc/kin={rms_acc_band/rms_kin_band:.3f} "
              f"coherence(0.02-1Hz)={coh_band:.3f} "
              f"[std_kin={d_kin_std:.2f}mN std_acc={d_acc_std:.2f}mN, full-band]")


if __name__ == "__main__":
    model = dict(L=0.0623989024, T=0.02, g=1.0)  # matches step4_model_used in step1_results.json

    print("=" * 100)
    print("Log A (low-gap continuous hover)")
    print("=" * 100)
    run_one("A", s1.LOG_A, (58.742436, 199.447425), model)

    print("=" * 100)
    print("Log C (venue, up to 25% telemetry loss -- script's stated worst case for AA)")
    print("=" * 100)
    run_one("C", s1.LOG_C, s1.WIN_C, model)
