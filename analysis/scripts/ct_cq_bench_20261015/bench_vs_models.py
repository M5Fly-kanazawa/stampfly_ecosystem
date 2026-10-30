#!/usr/bin/env python3
"""Compare the single-motor bench data with the firmware static curve and the SILS plant steady state.
単一モータのベンチ実測を、ファームの静的曲線と SILS プラントの定常状態と比較する。

Usage / 使い方:
    python3 -I bench_vs_models.py <summary.json>
    e.g. stampfly-motor-autotest/data/2026-10-15/summary.json
         (public repo kouhei1970/stampfly-motor-autotest)

Models / モデル:
  fw     firmware static curve (actuator.cpp thrustToDuty): V = Am w^2 + Bm w + Cm, T = Ct w^2
  now    SILS plant motor ODE steady state, current set, T = eta Ct w^2 (eta = 0.7133)
  ctcq   plant with bench Ct, Cq only (eta = 1)
  full   plant with the full bench set (K_e, R, Q_f, Cq incl. K_e*2.9e-9) (eta = 1)
  full(Vm) same as full but driven by the measured average motor voltage <Vm>
The values below are copied from control/models/stampfly_physical.yaml and
firmware/vehicle/components/sf_actuator/actuator.cpp as of 2026-10-30.
"""
import json
import math
import sys

G = 9.80665
# Firmware static curve (actuator.cpp) / ファームの静的曲線
AM, BM, CM, CTF = 6.0368e-8, 6.699042e-4, 1.53e-2, 1.00e-8
MASS = 0.037


def fw(volt):
    w = (-BM + math.sqrt(BM * BM - 4 * AM * (CM - volt))) / (2 * AM)
    return w, CTF * w * w


def plant(volt, Ct, Cq, Rm, Km, Qf, eta, Dm=0.0):
    """Steady state of Km V/Rm = (Dm + Km^2/Rm) w + Cq w^2 + Qf (motor ODE)."""
    a, b, c = Cq, Dm + Km * Km / Rm, Qf - Km * volt / Rm
    w = (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)
    return w, eta * Ct * w * w


P_NOW = dict(Ct=1e-8, Cq=4.1e-11, Rm=0.593, Km=5.682e-4, Qf=9.507e-6, eta=0.7133)
P_CTCQ = dict(Ct=7.6e-9, Cq=3.38e-11, Rm=0.593, Km=5.682e-4, Qf=9.507e-6, eta=1.0)
P_FULL = dict(Ct=7.6e-9, Cq=3.38e-11 + 5.90e-4 * 2.9e-9, Rm=0.594, Km=5.90e-4, Qf=5.90e-4 * 0.044, eta=1.0)


def main(path):
    # Use the COMMANDED duty ('duty'): the firmware and the SILS receive it. 'duty_meas' (from the drain
# waveform) is 0.006-0.015 larger and is not used. <Vm> stays as measured.
    # 指令 Duty（'duty'）を使う。ファームと SILS が受け取るのは指令値。duty_meas は使わない。
    rows = sorted(json.load(open(path)), key=lambda r: r['duty'])
    k = 1000 / G  # N -> g
    print(f"{'d':>5} {'VDD':>5} {'dV':>5} {'Vm':>5} | {'w_b':>5} {'T_b':>5} | {'w_fw':>5} {'T_fw':>5} | "
          f"{'w_now':>5} {'T_now':>5} | {'w_cq':>5} {'T_cq':>5} | {'w_fl':>5} {'T_fl':>5} | {'T_fl(Vm)':>8}")
    table = []
    for r in rows:
        d, vdd, vm = r['duty'], r['VDD'], r['Vm_avg']
        wb, tb = r['omega'], r['thrust_g']
        v = d * vdd
        wf, tf = fw(v)
        wn, tn = plant(v, **P_NOW)
        wc, tc = plant(v, **P_CTCQ)
        wl, tl = plant(v, **P_FULL)
        wv, tv = plant(vm, **P_FULL)
        print(f"{d:5.3f} {vdd:5.2f} {v:5.2f} {vm:5.2f} | {wb:5.0f} {tb:5.2f} | {wf:5.0f} {tf*k:5.2f} | "
              f"{wn:5.0f} {tn*k:5.2f} | {wc:5.0f} {tc*k:5.2f} | {wl:5.0f} {tl*k:5.2f} | {tv*k:8.2f}")
        if not r['tag'].endswith('_rep'):  # skip the repeat run at Duty 0.5 / 繰り返し計測は傾きから除く
            table.append((d, v, vm, tb, tf * k))

    # Per-interval slopes against Duty x V_DD / Duty x V_DD に対する区間ごとの傾き
    print('\nper-interval slope / 区間ごとの傾き')
    print(f"{'interval':>11} {'bench[g/V]':>10} {'model[g/V]':>10} {'ratio':>6} {'d<Vm>/d(Duty*VDD)':>18}")
    for a, b in zip(table, table[1:]):
        dv = b[1] - a[1]
        sb = (b[3] - a[3]) / dv
        sm = (b[4] - a[4]) / dv
        sv = (b[2] - a[2]) / dv
        print(f"{a[0]:.2f}->{b[0]:.2f} {sb:10.2f} {sm:10.2f} {sb/sm:6.2f} {sv:18.2f}")

    # Hover per motor: eta that makes the plant hover at the firmware hover voltage
    # ホバー: プラントがファームのホバー電圧でホバーするよう η を再導出
    th = MASS * G / 4
    w_h = math.sqrt(th / CTF)
    v_h = AM * w_h ** 2 + BM * w_h + CM
    print('\nhover thrust per motor [g]', round(th * k, 3), ' firmware hover V', round(v_h, 3))
    for name, p in [('now', P_NOW), ('ctcq', P_CTCQ), ('full', P_FULL)]:
        w, _ = plant(v_h, **dict(p, eta=1.0))
        eta = th / (p['Ct'] * w * w)
        print(f"{name}: re-derived eta {eta:.4f}  hover omega {w:.0f} rad/s")
    pts = sorted((r['thrust_g'], r['duty'] * r['VDD'], r['omega'], r['Vm_avg']) for r in rows)
    thg = th * k
    for p0, p1 in zip(pts, pts[1:]):
        if p0[0] <= thg <= p1[0]:
            f = (thg - p0[0]) / (p1[0] - p0[0])
            print('bench hover: Duty*VDD', round(p0[1] + f * (p1[1] - p0[1]), 3),
                  'omega', round(p0[2] + f * (p1[2] - p0[2])), 'Vm', round(p0[3] + f * (p1[3] - p0[3]), 3))


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
