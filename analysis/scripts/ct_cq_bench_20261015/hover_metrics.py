#!/usr/bin/env python3
"""Hover metrics for a SILS scenario bundle (F1 / F1g experiment).

Usage: python hover_metrics.py <out_scn_dir> <out.json>
  <out_scn_dir> = simulator/sils/viz/out_scn_<scenario>/ (holds the *.sflog.zip and console.log)

Metrics are computed from the TRUE altitude (truth stream), the firmware commanded total
thrust (ctrl_output), duty (motor), battery voltage (status) and, when the instrumented
firmware printed them, the learned hover thrust correction (console.log "HOVERDBG" lines).
真値高度・総推力指令・duty・電池電圧と、計装版ファームが出力する学習値から算出する。
"""
import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "lib"))
import sflog  # noqa: E402

MASS = 0.037
G = 9.80665
MG = MASS * G
TARGET = 0.5


def main():
    d = Path(sys.argv[1])
    z = sorted(d.glob("*.sflog.zip"))[-1]
    log = sflog.load(z)
    S = log.streams
    tr = S["truth"].sort_values("timestamp_us")
    t = tr["timestamp_us"].to_numpy() * 1e-6
    alt = -tr["pos_z"].to_numpy()
    res = {"bundle": z.name}

    # takeoff start: first time altitude > 5 cm
    i0 = int(np.argmax(alt > 0.05))
    t_to = float(t[i0])
    res["t_takeoff_s"] = t_to
    # end of hover: last time altitude > 0.4 m
    # scenario timeline: boot 6 + ALT 1.5 + ARM 1 + hold 190 s -> hold ends at 198.5 s; stop 0.2 s early
    # (the DISARM press after that drops the craft, which must not enter the hover statistics)
    t_end = 198.3
    i_end = int(np.searchsorted(t, t_end))
    res["t_hover_end_s"] = t_end
    res["hover_duration_s"] = t_end - t_to

    a = alt[i0:i_end + 1]
    ta = t[i0:i_end + 1]
    res["alt_max_m"] = float(a.max())
    res["overshoot_m"] = float(a.max() - TARGET)
    res["t_alt_max_after_takeoff_s"] = float(ta[np.argmax(a)] - t_to)
    # settling: after the last time |alt - 0.5| > band
    for band in (0.05, 0.02):
        out = np.where(np.abs(a - TARGET) > band)[0]
        res["settle_time_%dcm_s" % int(band * 100)] = float(ta[out[-1]] - t_to) if len(out) else 0.0
    res["settle_note"] = "time from first alt>5cm to the LAST sample with |alt-0.5| > band, within the hover window"

    # 4-s max-min windows after t_takeoff + 20 s
    t_w0 = t_to + 20.0
    bands = []
    ws = t_w0
    while ws + 4.0 <= t_end - 1.0:
        m = (t >= ws) & (t < ws + 4.0)
        bands.append(float(alt[m].max() - alt[m].min()))
        ws += 4.0
    bands = np.array(bands)
    res["band4s_n"] = int(len(bands))
    res["band4s_median_cm"] = float(np.median(bands) * 100)
    res["band4s_max_cm"] = float(bands.max() * 100)
    res["band4s_min_cm"] = float(bands.min() * 100)
    res["band4s_window_start_s"] = t_w0
    # first/last 60-s thirds to show the trend
    k = len(bands) // 3
    res["band4s_median_cm_first_third"] = float(np.median(bands[:k]) * 100)
    res["band4s_median_cm_last_third"] = float(np.median(bands[-k:]) * 100)
    m = (t >= t_w0) & (t <= t_end - 1.0)
    res["alt_std_cm"] = float(alt[m].std() * 100)
    res["alt_mean_hover_m"] = float(alt[m].mean())

    # battery voltage
    st = S["status"].sort_values("timestamp_us")
    ts = st["timestamp_us"].to_numpy() * 1e-6
    vb = st["voltage"].to_numpy()
    mv = (ts >= t_w0) & (ts <= t_end - 1.0)
    res["vbat_hover_min_V"] = float(vb[mv].min())
    res["vbat_hover_max_V"] = float(vb[mv].max())
    res["vbat_at_takeoff_V"] = float(vb[np.argmin(np.abs(ts - t_to))])
    res["vbat_whole_run_min_V"] = float(vb.min())
    res["vbat_whole_run_max_V"] = float(vb.max())

    # commanded total thrust vs battery voltage during hover
    co = S["ctrl_output"].sort_values("timestamp_us")
    tc = co["timestamp_us"].to_numpy() * 1e-6
    th = co["thrust"].to_numpy()
    mc = (tc >= t_w0) & (tc <= t_end - 1.0)
    tcw, thw = tc[mc], th[mc]
    vw = np.interp(tcw, ts, vb)
    # average over 1-s bins first (rate loops dither; the trend is the question)
    nb = int((tcw[-1] - tcw[0]) // 1.0)
    bt = np.array([thw[(tcw >= tcw[0] + j) & (tcw < tcw[0] + j + 1)].mean() for j in range(nb)])
    bv = np.array([vw[(tcw >= tcw[0] + j) & (tcw < tcw[0] + j + 1)].mean() for j in range(nb)])
    p = np.polyfit(bv, bt / MG, 1)
    res["thrust_cmd_vs_vbat_slope_per_V_of_mg"] = float(p[0])
    res["thrust_cmd_vs_vbat_corr"] = float(np.corrcoef(bv, bt)[0, 1])
    res["thrust_cmd_hover_mean_over_mg"] = float(thw.mean() / MG)
    res["thrust_cmd_first30s_over_mg"] = float(thw[tcw < tcw[0] + 30].mean() / MG)
    res["thrust_cmd_last30s_over_mg"] = float(thw[tcw > tcw[-1] - 30].mean() / MG)

    # duty
    mo = S["motor"].sort_values("timestamp_us")
    tm = mo["timestamp_us"].to_numpy() * 1e-6
    D = mo[["duty_FR", "duty_RR", "duty_RL", "duty_FL"]].to_numpy()
    dmax = D.max(axis=1)
    res["duty_max_whole_run"] = float(dmax.max())
    mm = (tm >= t_w0) & (tm <= t_end - 1.0)
    res["duty_max_hover"] = float(dmax[mm].max())
    res["duty_mean_hover_first30s"] = float(D[mm & (tm < t_w0 + 30)].mean())
    res["duty_mean_hover_last30s"] = float(D[mm & (tm > t_end - 31)].mean())
    # time at saturation: any motor duty >= 0.999 (clamp) / >= 0.95
    dt_m = np.diff(tm, append=tm[-1])
    for thr in (0.999, 0.95):
        sat = dmax >= thr
        res["time_at_duty_ge_%g_s_whole_run" % thr] = float(dt_m[sat].sum())
        res["time_at_duty_ge_%g_s_hover" % thr] = float(dt_m[sat & mm].sum())
    res["duty_note"] = "motor.csv rows exist only for armed control cycles; dt from successive timestamps"

    # attitude ringing: roll/pitch rate RMS and tilt in hover (truth)
    q = tr[["quat_w", "quat_x", "quat_y", "quat_z"]].to_numpy()
    roll = np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
    pitch = np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1))
    mt = (t >= t_w0) & (t <= t_end - 1.0)
    res["tilt_rms_deg_hover"] = float(np.degrees(np.sqrt(np.mean(roll[mt] ** 2 + pitch[mt] ** 2))))
    res["tilt_max_deg_whole_run"] = float(np.degrees(np.max(np.hypot(roll, pitch))))
    rates = tr[["rate_x", "rate_y", "rate_z"]].to_numpy()
    res["body_rate_rms_dps_hover"] = [float(np.degrees(np.sqrt(np.mean(rates[mt, k] ** 2)))) for k in range(3)]
    res["tilt_max_deg_during_takeoff"] = float(np.degrees(np.max(np.hypot(roll[(t >= t_to) & (t < t_to + 20)], pitch[(t >= t_to) & (t < t_to + 20)]))))

    # learned hover thrust + velocity-loop correction (instrumented firmware only)
    cl = d / "console.log"
    dbg = []
    if cl.exists():
        for line in cl.read_text(errors="replace").splitlines():
            mm_ = re.search(r"HOVERDBG t=(\d+) corr=([-0-9.eE+]+) vel=([-0-9.eE+]+)", line)
            if mm_:
                dbg.append((int(mm_.group(1)) * 1e-6, float(mm_.group(2)), float(mm_.group(3))))
    if dbg:
        dbg = np.array(dbg)
        # firmware time base may differ from log time base by a constant; use order only
        res["hoverdbg_n"] = int(len(dbg))
        res["hover_thrust_corr_start"] = float(dbg[0, 1])
        res["hover_thrust_corr_end"] = float(dbg[-1, 1])
        res["hover_thrust_corr_min"] = float(dbg[:, 1].min())
        res["hover_thrust_corr_max"] = float(dbg[:, 1].max())
        res["hover_thrust_corr_every_20pct"] = [float(dbg[int(f * (len(dbg) - 1)), 1]) for f in np.linspace(0, 1, 6)]
        n = len(dbg)
        res["velloop_correction_N_mean_second_half"] = float(dbg[n // 2:, 2].mean())
        res["velloop_correction_N_mean_all"] = float(dbg[:, 2].mean())
        res["velloop_correction_over_mg_mean_second_half"] = float(dbg[n // 2:, 2].mean() / MG)
    else:
        res["hoverdbg"] = "no HOVERDBG lines (uninstrumented firmware)"

    # ---- same metrics restricted to battery voltage >= 3.5 V (flight/bench range) ----
    # cutoff = first time the 1-s mean of the sensed voltage falls below 3.5 V (after hover start)
    tv = None
    for tt in np.arange(t_w0, t_end, 1.0):
        mm1 = (ts >= tt) & (ts < tt + 1.0)
        if mm1.any() and vb[mm1].mean() < 3.5:
            tv = float(tt)
            break
    if tv is None:
        tv = t_end - 1.0
    r35 = {"cutoff_time_s": tv, "window": [t_w0, tv]}
    if tv - t_w0 >= 8.0:
        b35 = []
        ws = t_w0
        while ws + 4.0 <= tv:
            m = (t >= ws) & (t < ws + 4.0)
            b35.append(float(alt[m].max() - alt[m].min()))
            ws += 4.0
        b35 = np.array(b35)
        r35["band4s_n"] = int(len(b35))
        r35["band4s_median_cm"] = float(np.median(b35) * 100)
        r35["band4s_max_cm"] = float(b35.max() * 100)
        m35c = (tc >= t_w0) & (tc <= tv)
        t35, h35 = tc[m35c], th[m35c]
        v35 = np.interp(t35, ts, vb)
        nb35 = int((t35[-1] - t35[0]) // 1.0)
        bt35 = np.array([h35[(t35 >= t35[0] + j) & (t35 < t35[0] + j + 1)].mean() for j in range(nb35)])
        bv35 = np.array([v35[(t35 >= t35[0] + j) & (t35 < t35[0] + j + 1)].mean() for j in range(nb35)])
        r35["vbat_min_V"] = float(bv35.min())
        r35["vbat_max_V"] = float(bv35.max())
        if bv35.max() - bv35.min() > 0.05:
            r35["thrust_cmd_vs_vbat_slope_per_V_of_mg"] = float(np.polyfit(bv35, bt35 / MG, 1)[0])
            r35["thrust_cmd_vs_vbat_corr"] = float(np.corrcoef(bv35, bt35)[0, 1])
        m35m = (tm >= t_w0) & (tm <= tv)
        r35["duty_max"] = float(dmax[m35m].max())
        r35["duty_mean_first"] = float(D[m35m].mean(axis=1)[:400 * 5].mean())
        r35["duty_mean_last"] = float(D[m35m].mean(axis=1)[-400 * 5:].mean())
        r35["alt_mean_m"] = float(alt[(t >= t_w0) & (t <= tv)].mean())
        r35["alt_std_cm"] = float(alt[(t >= t_w0) & (t <= tv)].std() * 100)
        if dbg is not None and len(dbg) if isinstance(dbg, list) else len(dbg):
            # align: last HOVERDBG sample ~ hover end (DISARM), one per second
            t_d0 = 198.5 - len(dbg)
            sel = [(t_d0 + i) for i in range(len(dbg))]
            sel = np.array(sel)
            mk = (sel >= t_w0) & (sel <= tv)
            if mk.any():
                r35["hover_thrust_corr_at_start"] = float(dbg[mk, 1][0])
                r35["hover_thrust_corr_at_cutoff"] = float(dbg[mk, 1][-1])
                r35["velloop_correction_N_mean"] = float(dbg[mk, 2].mean())
                r35["velloop_correction_over_mg_mean"] = float(dbg[mk, 2].mean() / MG)
            r35["dbg_alignment_note"] = "HOVERDBG time base assumed to end at hover end 198.5 s (approximate, +-2 s)"
    res["restricted_vbat_ge_3p5"] = r35

    Path(sys.argv[2]).parent.mkdir(parents=True, exist_ok=True)
    Path(sys.argv[2]).write_text(json.dumps(res, indent=1, ensure_ascii=False) + "\n")
    print(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
