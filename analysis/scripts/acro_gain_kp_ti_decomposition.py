#!/usr/bin/env python3
"""ACRO rate-loop stability-margin decomposition: current vs original-converted gains.

Splits the original->vehicle gain conversion into its Kp-increase and Ti-lengthening
parts and computes phase/gain margins on a physically-grounded rate-loop plant, under
both a small-lag and a large-lag assumption. Self-contained (stdlib only).

レートループ安定余裕の分解: 現行 vs オリジナル換算ゲイン。換算を「Kp増加分」と
「Ti延長分」に分け、URDF慣性＋同定遅れに基づくプラント上で位相/ゲイン余裕を計算する。
"""
import math, cmath

ETA = 0.125  # derivative-filter coefficient (vehicle pid.hpp) / 微分フィルタ係数

# ---- rate PID (standard form) and rate-loop plant frequency responses ----
def Cjw(w, kp, ti, td):
    s = 1j*w
    return kp*(1.0 + 1.0/(ti*s) + (td*s)/(ETA*td*s + 1.0))

def Gjw(w, I, tau_m, L, eta_t):
    # torque[Nm] -> angular rate[rad/s]:  eta_t*e^{-Ls} / (I s (tau_m s + 1))
    s = 1j*w
    return eta_t*cmath.exp(-L*s) / (I*s*(tau_m*s + 1.0))

def Ljw(w, g, I, tau_m, L, eta_t):
    return Cjw(w, *g)*Gjw(w, I, tau_m, L, eta_t)

def margins(g, I, tau_m, L, eta_t):
    ws = [10**(x/200) for x in range(-200, 700)]      # 0.01 .. ~3000 rad/s
    mag = [abs(Ljw(w, g, I, tau_m, L, eta_t)) for w in ws]
    ph = [math.degrees(cmath.phase(Ljw(w, g, I, tau_m, L, eta_t))) for w in ws]
    for i in range(1, len(ph)):                        # unwrap
        while ph[i]-ph[i-1] > 180:  ph[i] -= 360
        while ph[i]-ph[i-1] < -180: ph[i] += 360
    wc = pm = gm = None
    for i in range(1, len(ws)):                        # gain crossover -> PM
        if (mag[i-1]-1)*(mag[i]-1) < 0:
            f = (1-mag[i-1])/(mag[i]-mag[i-1])
            wc = ws[i-1]*(ws[i]/ws[i-1])**f
            pm = 180 + ph[i-1]+f*(ph[i]-ph[i-1]); break
    for i in range(1, len(ws)):                        # phase crossover -> GM
        if (ph[i-1]+180)*(ph[i]+180) < 0:
            f = (-180-ph[i-1])/(ph[i]-ph[i-1])
            magc = mag[i-1]*(mag[i]/mag[i-1])**f
            gm = -20*math.log10(magc); break
    return wc, pm, gm

# ---- physical constants ----
I = {'roll': 9.16e-6, 'pitch': 13.3e-6, 'yaw': 20.4e-6}   # URDF stampfly.urdf [kg m^2]
CUR = (3.40e-4, 0.4, 0.017658)                            # current vehicle roll

def cal_eta(g_roll, tau_m, L, target=55.0):
    # calibrate effective torque so current-roll PM matches the real ~55 deg (flight-id)
    best = None
    for et in [x/400 for x in range(2, 321)]:
        wc, pm, gm = margins(g_roll, I['roll'], tau_m, L, et)
        if pm and (best is None or abs(pm-target) < abs(best[1]-target)):
            best = (et, pm)
    return best[0]

VARIANTS = {
    'A current        (kp3.4e-4 ti0.4 td0.018)': (3.40e-4,     0.4, 0.017658),
    'B Ti lengthen    (kp3.4e-4 ti0.7 td0.018)': (3.40e-4,     0.7, 0.017658),
    'C Kp x2.8        (kp9.8e-4 ti0.4 td0.018)': (9.759795e-4, 0.4, 0.017658),
    'D converted(all) (kp9.8e-4 ti0.7 td0.01 )': (9.759795e-4, 0.7, 0.01),
}

if __name__ == '__main__':
    for tau_m, L, tag in [(0.02, 0.012, 'small lag: tau_m=20ms L=12ms (rate-loop id)'),
                          (0.03, 0.06,  'large lag: tau_m=30ms L=60ms (pessimistic)')]:
        et = cal_eta(CUR, tau_m, L)
        print('='*72)
        print(f'{tag}   [calibrated eta_t={et:.2f} -> current-roll PM=55deg]')
        print('='*72)
        print(f"  {'variant':42} {'wc[Hz]':>7} {'PM[deg]':>8} {'GM[dB]':>7}")
        for name, g in VARIANTS.items():
            wc, pm, gm = margins(g, I['roll'], tau_m, L, et)
            print(f"  {name:42} {wc/6.283 if wc else 0:>7.2f} "
                  f"{pm if pm else 0:>8.0f} {gm if gm is not None else 99:>7.0f}")
        print()
