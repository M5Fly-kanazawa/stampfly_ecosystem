#!/usr/bin/env python3
"""POS_HOLD altitude-bob: disturbance identification + closed-loop replay sim.

From a POS_HOLD UDP JSONL log, this:
  1. Identifies the net vertical-force disturbance d_ext driving the altitude bob
     (d_ext = M*az - (thrust_lagged*cos(tilt) - M*g)), and shows it grows with
     battery sag.
  2. Closed-loop REPLAYS the firmware altitude cascade (alt_pos PI -> vel_sp ->
     alt_vel PI -> thrust) against the SAME d_ext, with the ESKF vz estimation
     noise injected into the feedback, and VALIDATES it reproduces the measured
     bob with the current gains.
  3. Compares candidate gain sets on the validated replay.

CAVEAT: the replay validates at the CURRENT (low-bandwidth) operating point. As
loop gain rises the velocity crossover enters the actuation-lag region, which this
first-order-lag + injected-noise model under-represents — so high-gain results are
OPTIMISTIC and conflict with the firmware's real-flight note ("raising vel.kp is
counterproductive due to the ~110ms lag"). Do NOT adopt a gain increase from this
sim alone; it requires a real vertical sysid + SIL design.
注意: 本再生は現行(低帯域)動作点で検証済み。高ゲイン域は遅れの影響を過小評価し実機知見と
矛盾するため、本シミュ単独でゲイン増を採用しないこと(要 鉛直sysid + SIL設計)。

Usage: python3 analysis/scripts/poshold_alt_loop_sim.py [log.jsonl]
"""
import json, sys, numpy as np

LOG = sys.argv[1] if len(sys.argv) > 1 else "logs/stampfly_udp_20260627T020137.jsonl"
M, G, HOVER_CORR = 0.037, 9.80665, 1.12
HOVER = M*G*HOVER_CORR
TAU = 0.11            # actuation lag [s] (firmware comment, hover-log measured)
FS = 100.0            # uniform resample rate
WIN = (108.0, 178.0)  # bad (low-battery) window

S = {'posvel': [], 'ctrl_ref': [], 'imu': []}
with open(LOG) as f:
    for line in f:
        try: d = json.loads(line)
        except Exception: continue
        if d.get('id') in S: S[d['id']].append(d)
t0 = S['posvel'][0]['ts']
def ts(r): return np.array([(x['ts']-t0)/1e6 for x in r])
tpv = ts(S['posvel']); pos = np.array([r['pos'] for r in S['posvel']]); vel = np.array([r['vel'] for r in S['posvel']])
alt = -pos[:, 2]; vz = -vel[:, 2]
tcr = ts(S['ctrl_ref']); thr = np.array([r['total_thrust'] for r in S['ctrl_ref']])
tim = ts(S['imu']); q = np.array([r['quat'] for r in S['imu']]); cosT = 1 - 2*(q[:, 1]**2 + q[:, 2]**2)

def smooth(x, w):
    return np.convolve(x, np.ones(w)/w, mode='same')
def lpf(x, tau, fs):
    a = np.exp(-1/(tau*fs)); y = np.zeros_like(x); y[0] = x[0]
    for i in range(1, len(x)): y[i] = a*y[i-1] + (1-a)*x[i]
    return y

A, B = WIN; tg = np.arange(A, B, 1/FS)
altg = np.interp(tg, tpv, alt); vzg = np.interp(tg, tpv, vz)
thrg = np.interp(tg, tcr, thr); cosg = np.interp(tg, tim, cosT)
az = smooth(np.gradient(smooth(vzg, 30), 1/FS), 30)
d_ext = M*az - (lpf(thrg, TAU, FS)*cosg - M*G)
noise = vzg - smooth(vzg, int(FS/1.5))   # ESKF vz estimation noise (>1.5 Hz)

class PI:
    def __init__(s, kp, ti, lim): s.kp, s.ti, s.lim, s.I, s.pe = kp, ti, lim, 0.0, 0.0
    def step(s, sp, meas, dt):
        e = sp-meas; P = s.kp*e
        if s.ti >= 0.01:
            inext = s.I + (s.kp/s.ti)*(e+s.pe)*dt*0.5; ot = P+inext
            if not ((ot > s.lim and e > 0) or (ot < -s.lim and e < 0)): s.I = inext
            s.I = float(np.clip(s.I, -s.lim, s.lim))
        s.pe = e; return float(np.clip(P+s.I, -s.lim, s.lim))

def replay(akp, ati, vkp, vti, tilt_ff=False):
    dt = 1/FS; n = len(tg); a_ = PI(akp, ati, 0.5); v_ = PI(vkp, vti, 0.15)
    al = np.zeros(n); vc = np.zeros(n); al[0] = altg[0]; vc[0] = vzg[0]
    sp = float(np.median(altg)); thr_state = HOVER; ac = np.exp(-1/(TAU*FS))
    for i in range(1, n):
        vz_meas = vc[i-1] + noise[i]                       # inject estimation noise
        vsp = a_.step(sp, al[i-1], dt); corr = v_.step(vsp, vz_meas, dt)
        tcmd = HOVER + corr
        if tilt_ff: tcmd /= max(cosg[i], 0.5)
        tcmd = float(np.clip(tcmd, 0, 0.6)); thr_state = ac*thr_state + (1-ac)*tcmd
        azc = (thr_state*cosg[i] - M*G + d_ext[i])/M
        vc[i] = vc[i-1] + azc*dt; al[i] = al[i-1] + vc[i]*dt
    return al, vc

if __name__ == '__main__':
    meas = np.std(altg)*1000
    al, _ = replay(0.45, 7.0, 0.1, 2.5)
    print(f"d_ext (bob-driving net force) std = {np.std(d_ext)*1000:.1f} mN   vz-noise std = {np.std(noise)*1000:.0f} mm/s")
    print(f"[validation] measured alt std = {meas:.0f} mm   replay(current gains) = {np.std(al)*1000:.0f} mm\n")
    print(f"  {'candidate':30} {'alt std':>8}  {'vs meas':>7}")
    for name, g in [("current alt0.45 vel0.1", (0.45, 7.0, 0.1, 2.5, False)),
                    ("alt.kp 0.70", (0.70, 7.0, 0.1, 2.5, False)),
                    ("vel.kp 0.20", (0.45, 7.0, 0.2, 2.5, False)),
                    ("vel.kp 0.40", (0.45, 7.0, 0.4, 2.5, False)),
                    ("tilt FF thrust/cos", (0.45, 7.0, 0.1, 2.5, True))]:
        s = np.std(replay(*g)[0])*1000
        print(f"  {name:30} {s:>6.0f}mm {s/meas*100:>6.0f}%")
    print("\nNOTE: high-gain rows are OPTIMISTIC (model under-represents lag at higher")
    print("crossover; conflicts with real-flight tuning). Do not adopt without sysid+SIL.")
