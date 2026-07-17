#!/usr/bin/env python3
"""A5: Independent recomputation (no step2 import) of
(a) velocity-loop open-loop margins  L_v = Cv * G/(M s) * E
(b) DOB inner-loop robustness        min |1 + W Q (G' - G)|
All transfer functions written from scratch.
"""
import numpy as np

M = 0.037
TI = 1.5
T_ACT = 0.020
TAU_E = 0.0276
W_HZ = np.logspace(-3, 2, 200000)   # 1e-3 .. 100 Hz, dense
W = 2 * np.pi * W_HZ


def G_of(w, L, T=T_ACT, g=1.0):
    s = 1j * w
    return g * np.exp(-s * L) / (1 + s * T)


def margins(w_hz, Lw):
    """GM at the first -180 deg crossing of the unwrapped phase; PM at the
    (last) gain crossover |L|=1."""
    mag = np.abs(Lw)
    ph = np.degrees(np.unwrap(np.angle(Lw)))
    gm_db = pm = f_gc = f_pc = np.nan
    over = np.where(mag >= 1.0)[0]
    if len(over) and over[-1] < len(mag) - 1:
        i = over[-1]
        # linear interp of phase at |L|=1 crossing
        j = i + 1
        frac = (1.0 - mag[i]) / (mag[j] - mag[i])
        ph_c = ph[i] + frac * (ph[j] - ph[i])
        pm = 180.0 + ph_c
        f_gc = w_hz[i] + frac * (w_hz[j] - w_hz[i])
    cross = np.where(np.diff(np.sign(ph + 180.0)) != 0)[0]
    if len(cross):
        i = cross[0]
        j = i + 1
        frac = (-180.0 - ph[i]) / (ph[j] - ph[i])
        mag_c = mag[i] + frac * (mag[j] - mag[i])
        gm_db = -20 * np.log10(mag_c)
        f_pc = w_hz[i] + frac * (w_hz[j] - w_hz[i])
    return gm_db, pm, f_gc, f_pc


print("=== (a) velocity-loop margins, from scratch ===")
print("L_v(s) = kp(1+1/(s Ti)) * [g e^{-Ls}/(1+sT)] / (M s) * 1/(1+s tau_e)")
print("Ti=%.2f  T=%.3f  tau_e=%.4f  M=%.3f" % (TI, T_ACT, TAU_E, M))
for L_delay in (0.060, 0.0624):
    print("-- pure delay L = %.4f s --" % L_delay)
    for kp in (0.1, 0.15, 0.2):
        s = 1j * W
        Cv = kp * (1 + 1 / (s * TI))
        Lv = Cv * G_of(W, L_delay) / (M * s) * (1 / (1 + s * TAU_E))
        gm, pm, fgc, fpc = margins(W_HZ, Lv)
        print("  kp=%.2f: GM = %6.2f dB @ %.3f Hz   PM = %6.2f deg @ %.3f Hz" % (kp, gm, fpc, pm, fgc))

print()
print("README claims: kp0.1 GM 15.3dB / PM 59deg ; kp0.15 11.7dB/55deg ; kp0.2 GM 9.2dB / PM 49deg")

print()
print("=== (b) DOB inner-loop min|1 + W(s)Q(s)(G'(s)-G(s))|, from scratch ===")
WASH = 2 * np.pi * 0.03


def Q2(w, fc):
    wc = 2 * np.pi * fc
    s = 1j * w
    return wc ** 2 / (s ** 2 + np.sqrt(2) * wc * s + wc ** 2)


def Wash(w):
    s = 1j * w
    return s / (s + WASH)


for L_nom in (0.060, 0.0624):
    print("-- nominal delay L = %.4f s, perturbation = pure delay +50 ms --" % L_nom)
    for fc in (1.5, 3.0):
        Gn = G_of(W, L_nom)
        Gp = G_of(W, L_nom + 0.050)
        Ldob = Wash(W) * Q2(W, fc) * (Gp - Gn)
        dist = np.abs(1 + Ldob)
        # full range and step2's 0.01-20Hz range
        m2 = (W_HZ >= 0.01) & (W_HZ <= 20)
        i_all = int(np.argmin(dist))
        i_r = np.where(m2)[0][int(np.argmin(dist[m2]))]
        print("  fc=%.1f: min|1+L| = %.4f at %.2f Hz (1e-3..100Hz) ; %.4f at %.2f Hz (0.01-20Hz) ; max|L|=%.3f"
              % (fc, dist[i_all], W_HZ[i_all], dist[i_r], W_HZ[i_r], np.abs(Ldob).max()))
print()
print("README claims (delay +50ms row): fc=1.5 -> 0.716 ; fc=3.0 -> 0.566")
