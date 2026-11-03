#!/usr/bin/env python3
"""F1 experiment: (1) inverse∘forward identity check of the exact plant inverse,
(2) hover gain ratio r(V) of plant vs firmware thrust-per-duty.

F1 実験: (1) プラント定常特性の厳密な逆写像の恒等性確認、(2) ホバー推力でのゲイン比 r(V)。

Constants are read from simulator/sils/plant/generated_params.hpp (namespace plant)
and actuator.cpp (firmware curve), not typed by hand.
定数は generated_params.hpp / actuator.cpp から読む（手書きしない）。

Usage: python f1_identity_rv.py [outdir]
"""
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
GP = (ROOT / "simulator/sils/plant/generated_params.hpp").read_text()
ACT = (ROOT / "firmware/vehicle/components/sf_actuator/actuator.cpp").read_text()


def scal(name, text=GP):
    m = re.search(r"constexpr float %s = ([0-9.eE+-]+)f;" % name, text)
    return float(m.group(1))


plant_ns = GP[GP.index("namespace plant {"):GP.index("}  // namespace plant")]
KE, KT, CQ, CT = (scal(n, plant_ns) for n in ("KE", "KT", "CQ", "CT"))
I0A, I0B, ETA = (scal(n, plant_ns) for n in ("I0A", "I0B", "THRUST_EFFICIENCY"))
ROUT = [float(x) for x in re.search(r"ROUT_OHM\[ROUT_N\] = \{(.*?)\};", plant_ns, re.S).group(1).replace("f", "").split(",") if x.strip()]
assert len(ROUT) == 101
MASS = scal("MASS")
FW_AM, FW_BM, FW_CM = (float(re.search(r"MOTOR_%s = ([0-9.e+-]+)f" % n, ACT).group(1)) for n in ("AM", "BM", "CM"))
FW_CT = float(re.search(r"MOTOR_CT = ([0-9.e+-]+)f", ACT).group(1))
G = 9.80665
T_HOVER = MASS * G / 4.0


def rout(d):
    if d <= 0.01:
        return ROUT[1]
    if d >= 1.0:
        return ROUT[100]
    pos = d / 0.01
    i = min(int(pos), 99)
    fr = pos - i
    return ROUT[i] + fr * (ROUT[i + 1] - ROUT[i])


def plant_omega(d, v):
    """Plant steady-state omega (Plant::steadyStateOmega)."""
    if d < 0.01:
        return 0.0
    r = rout(min(d, 1.0))
    a = CQ + KT * I0B
    b = KT * KE / r
    c = KT * I0A - KT * v / r
    disc = b * b - 4 * a * c
    return max(0.0, (-b + math.sqrt(disc)) / (2 * a)) if disc > 0 else 0.0


def plant_thrust(d, v):
    w = plant_omega(d, v)
    return ETA * CT * w * w


def inv_duty(t, v):
    """F1 exact inverse: thrust [N], vbat [V] -> duty (same edges as actuator.cpp)."""
    if t <= 0.0:
        return 0.0
    w = math.sqrt(t / (ETA * CT))
    drive = v - KE * w
    if drive <= 0.0:
        return 1.0
    i_req = (CQ * w * w + KT * (I0A + I0B * w * w)) / KT
    r_req = drive / i_req
    if r_req >= ROUT[1]:
        return 0.01
    if r_req <= ROUT[100]:
        return 1.0
    lo, hi = 1, 100
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ROUT[mid] >= r_req:
            lo = mid
        else:
            hi = mid
    frac = (ROUT[lo] - r_req) / (ROUT[lo] - ROUT[lo + 1])
    return (lo + frac) * 0.01


def fw_duty(t, v):
    if t <= 0:
        return 0.0
    w = math.sqrt(t / FW_CT)
    return (FW_AM * w * w + FW_BM * w + FW_CM) / v


def fw_thrust(d, v):
    """Thrust the firmware curve assumes at duty d (inverse of fw_duty)."""
    volts = d * v
    # AM w^2 + BM w + (CM - volts) = 0
    c = FW_CM - volts
    disc = FW_BM ** 2 - 4 * FW_AM * c
    w = (-FW_BM + math.sqrt(disc)) / (2 * FW_AM)
    return FW_CT * w * w


def bisect(f, lo, hi, target):
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if f(mid) < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def main():
    out = []
    P = out.append
    P("constants: KE=%g KT=%g CQ=%g CT=%g I0A=%g I0B=%g ETA=%g MASS=%g" % (KE, KT, CQ, CT, I0A, I0B, ETA, MASS))
    P("firmware: CT=%g AM=%g BM=%g CM=%g" % (FW_CT, FW_AM, FW_BM, FW_CM))
    P("per-motor hover thrust T_h = m g / 4 = %.6f N" % T_HOVER)
    mono = all(ROUT[k] > ROUT[k + 1] for k in range(1, 100))
    P("R_out strictly decreasing for D>=0.01: %s" % mono)

    # --- (1) identity: D -> forward (plant) -> T -> inverse -> D' ---
    worst = (0.0, None)
    n = 0
    for di in range(5, 101):          # D 0.05 .. 1.00 step 0.01
        for vi in range(0, 91):       # V 3.4 .. 4.3 step 0.01
            d = di / 100.0
            v = 3.4 + vi * 0.01
            t = plant_thrust(d, v)
            dd = inv_duty(t, v)
            e = abs(dd - d)
            n += 1
            if e > worst[0]:
                worst = (e, (d, v, t, dd))
    P("")
    P("== (1) identity check inverse(forward(D,V)) == D, D 0.05..1.00 step 0.01, V 3.4..4.3 step 0.01 (%d points, float64 python)" % n)
    P("max |duty error| = %.3e at D=%.2f V=%.2f (T=%.5f N, recovered D=%.6f)" % ((worst[0],) + worst[1]))
    # also off-grid D (random-ish)
    worst2 = 0.0
    for k in range(20000):
        d = 0.05 + 0.95 * ((k * 0.6180339887) % 1.0)
        v = 3.4 + 0.9 * ((k * 0.7548776662) % 1.0)
        worst2 = max(worst2, abs(inv_duty(plant_thrust(d, v), v) - d))
    P("off-grid (20000 quasi-random points) max |duty error| = %.3e" % worst2)
    # reverse direction: T -> D -> T'
    worst3 = 0.0
    for ti in range(1, 200):
        t = ti * 0.001
        for vi in range(0, 91, 5):
            v = 3.4 + vi * 0.01
            d = inv_duty(t, v)
            if 0.01 < d < 1.0:
                worst3 = max(worst3, abs(plant_thrust(d, v) - t))
    P("reverse (T in 1..199 mN -> D -> plant T) interior max |thrust error| = %.3e N" % worst3)

    # --- (2) hover gain ratio ---
    P("")
    P("== (2) hover gain ratio r(V) = (dT/dD)_plant / (dT/dD)_firmware")
    P("A (task definition): plant slope at plant hover duty D_p; firmware slope at firmware hover duty D_f = fw_duty(T_h,V).")
    P("B (closed-loop coupling): both slopes at the SAME duty D_p, i.e. d T_plant / d T_cmd through baseline duty=fw_duty(T_cmd).")
    P("central difference step h = 0.002 in D")
    h = 0.002
    P("%5s %8s %8s %10s %10s %7s %7s" % ("V", "D_plant", "D_fw", "dTp/dD", "dTf/dD", "rA", "rB"))
    rows = []
    v = 3.60
    while v < 4.2001:
        dp = bisect(lambda d: plant_thrust(d, v), 0.011, 1.0, T_HOVER)
        df = fw_duty(T_HOVER, v)
        sp = (plant_thrust(dp + h, v) - plant_thrust(dp - h, v)) / (2 * h)
        sf = (fw_thrust(df + h, v) - fw_thrust(df - h, v)) / (2 * h)
        sf_b = (fw_thrust(dp + h, v) - fw_thrust(dp - h, v)) / (2 * h)
        ra, rb = sp / sf, sp / sf_b
        rows.append((v, dp, df, sp, sf, ra, rb))
        P("%5.2f %8.4f %8.4f %10.5f %10.5f %7.4f %7.4f" % (v, dp, df, sp, sf, ra, rb))
        v += 0.05
    sel = [r for r in rows if 3.70 - 1e-9 <= r[0] <= 4.10 + 1e-9]
    ra_bar = sum(r[5] for r in sel) / len(sel)
    rb_bar = sum(r[6] for r in sel) / len(sel)
    P("mean over 3.70..4.10 V (%d points): rA_bar = %.4f   rB_bar = %.4f" % (len(sel), ra_bar, rb_bar))
    # firmware conversion + plant level mismatch at hover (thrust the plant gives for fw duty)
    P("")
    P("hover level check: plant thrust at firmware hover duty / T_h")
    for vv in (3.6, 3.8, 4.0, 4.2):
        P("  V=%.1f: %.4f" % (vv, plant_thrust(fw_duty(T_HOVER, vv), vv) / T_HOVER))
    # non-smoothness: fine V scan of rA, count slope-sign changes and max jump
    P("")
    P("non-smoothness scan: rA at 0.005 V steps (3.6..4.2), step h = 0.0005 in D")
    hh = 0.0005
    prev = None
    jumps = []
    vals = []
    v = 3.60
    while v < 4.2001:
        dp = bisect(lambda d: plant_thrust(d, v), 0.011, 1.0, T_HOVER)
        df = fw_duty(T_HOVER, v)
        sp = (plant_thrust(dp + hh, v) - plant_thrust(dp - hh, v)) / (2 * hh)
        sf = (fw_thrust(df + hh, v) - fw_thrust(df - hh, v)) / (2 * hh)
        r = sp / sf
        vals.append((v, r, dp))
        if prev is not None:
            jumps.append((abs(r - prev[1]), v))
        prev = (v, r)
        v += 0.005
    P("rA fine scan min %.4f max %.4f; largest step-to-step change %.4f at V=%.3f (typical %.4f)" % (
        min(x[1] for x in vals), max(x[1] for x in vals), max(jumps)[0], max(jumps)[1],
        sum(j[0] for j in jumps) / len(jumps)))
    # D_p crossing table-grid knots
    knots = [round(x[2] / 0.01) for x in vals]
    P("plant hover duty runs %.4f..%.4f, i.e. crosses %d R_out table knots (0.01 spacing); the slope of the linear interpolation is piecewise constant between knots, so rA is piecewise-smooth with kinks at each knot" % (
        min(x[2] for x in vals), max(x[2] for x in vals), len(set(knots)) - 1))
    txt = "\n".join(out)
    print(txt)
    if len(sys.argv) > 1:
        Path(sys.argv[1]).mkdir(parents=True, exist_ok=True)
        (Path(sys.argv[1]) / "identity_and_rv.txt").write_text(txt + "\n")


if __name__ == "__main__":
    main()
