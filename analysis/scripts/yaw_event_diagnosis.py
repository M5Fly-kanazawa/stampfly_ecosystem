#!/usr/bin/env python3
"""Spontaneous yaw-rotation diagnosis from a POS/STAB UDP JSONL log.

Distinguishes the cause of sudden yaw rotations (~90-180 deg that return) by:
  - confirming they are UNcommanded (yaw stick ~0)
  - confirming they are REAL physical rotation (bias-corrected gyro, not an ESKF jump)
  - testing command vs measured yaw rate (controller DRIVES vs REACTS to the rotation)
  - the steady yaw trim CCW(M1+M3) vs CW(M2+M4) and which motors saturate per event
A disturbance-driven event (command opposes measured, controller saturates) plus a
large persistent CW-low trim points to a physical motor/prop fault, not control tuning.

突発ヨー回転の原因切り分け（操縦由来か / 実回転か推定ジャンプか / 制御が駆動か反応か /
持続ヨートリムとイベント時のモータ飽和）。stdlib + numpy。

Usage: python3 analysis/scripts/yaw_event_diagnosis.py <log.jsonl> [more.jsonl ...]
"""
import json, sys
import numpy as np

def analyze(LOG):
    S = {'imu': [], 'ctrl_ref': [], 'ctrl': [], 'rate_ref': [], 'status': []}
    for line in open(LOG):
        try: d = json.loads(line)
        except Exception: continue
        if d.get('id') in S: S[d['id']].append(d)
    t0 = S['imu'][0]['ts']
    ts = lambda r: np.array([(x['ts'] - t0) / 1e6 for x in r])
    ti = ts(S['imu'])
    gyro = np.array([r['gyro'] for r in S['imu']]); gb = np.array([r['gyro_bias'] for r in S['imu']])
    gz = np.degrees(gyro[:, 2] - gb[:, 2])                    # bias-corrected yaw rate [deg/s]
    fs = 1 / np.median(np.diff(ti))
    tcr = ts(S['ctrl_ref']); duty = np.array([r['motor_duty'] for r in S['ctrl_ref']])  # M1FR,M2RR,M3RL,M4FL
    tr = ts(S['rate_ref']); ycmd = np.degrees(np.array([r['rate_ref'] for r in S['rate_ref']])[:, 2])
    yst = np.abs(np.array([r['yaw'] for r in S['ctrl']])).max() if S['ctrl'] else float('nan')
    py = S['status'][0].get('pid_yaw') if S['status'] else None

    print(f"\n=== {LOG.split('/')[-1]}  ({ti[-1]:.0f}s) ===")
    if py: print(f"yaw PID kp/ti/td = {py[0]:.3e}/{py[1]:.2f}/{py[2]:.3f}")
    print(f"yaw stick |max| = {yst:.3f}   (≈0 → spontaneous, not commanded)")
    fly = (tcr > 10) & (tcr < ti[-1] - 5)
    ccw, cw = duty[fly, 0] + duty[fly, 2], duty[fly, 1] + duty[fly, 3]
    print(f"steady yaw trim  CCW(M1+M3)={ccw.mean():.2f} vs CW(M2+M4)={cw.mean():.2f}  "
          f"(diff {ccw.mean()-cw.mean():+.2f} → counters a steady CW moment)")

    idx = np.where(np.abs(gz) > 120)[0]
    if len(idx) == 0:
        print("  no large yaw events (>120 deg/s)"); return
    groups = np.split(idx, np.where(np.diff(idx) > int(fs))[0] + 1)
    print(f"  {len(groups)} event(s) (|yaw rate|>120 deg/s):")
    print(f"    {'t[s]':>6} {'yaw_meas':>8} {'yaw_cmd':>7} {'rel':>9} | "
          f"{'M1FR':>5} {'M2RR':>5} {'M3RL':>5} {'M4FL':>5} {'sat':>7}")
    for g in groups[:10]:
        pk = g[np.argmax(np.abs(gz[g]))]; t = ti[pk]
        c = float(np.interp(t, tr, ycmd))
        rel = "OPPOSES" if c * gz[pk] < 0 else "follows"   # opposes → controller reacts (disturbance)
        d = duty[np.argmin(np.abs(tcr - t))]
        sat = ','.join(f"M{j+1}" for j in range(4) if d[j] > 0.95) or '-'
        print(f"    {t:6.1f} {gz[pk]:>8.0f} {c:>7.0f} {rel:>9} | "
              f"{d[0]:>5.2f} {d[1]:>5.2f} {d[2]:>5.2f} {d[3]:>5.2f} {sat:>7}")

if __name__ == '__main__':
    for L in (sys.argv[1:] or ["logs/stampfly_udp_20260627T165713.jsonl",
                               "logs/stampfly_udp_20260627T164611.jsonl"]):
        analyze(L)
