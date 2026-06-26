#!/usr/bin/env python3
"""POS_HOLD long-flight wobble vs battery-sag analysis.

Time-segments a POS_HOLD UDP JSONL log and correlates horizontal/altitude wobble
with battery voltage sag and hover duty (thrust->duty compensation). Complements
poshold_analysis.py (which gives whole-flight metrics + a figure).

POS_HOLD 長尺ログの「位置・高度ふらつき」を時間分割し、電池電圧サグ・ホバーduty と
の相関を出す。stdlib + numpy のみ。

Usage: python3 analysis/scripts/poshold_battery_wobble.py [log.jsonl]
"""
import json, sys, numpy as np
LOG = sys.argv[1] if len(sys.argv) > 1 else "logs/stampfly_udp_20260627T020137.jsonl"
S={'posvel':[],'ctrl_ref':[],'status':[],'tof_b':[]}
with open(LOG) as f:
    for line in f:
        try: d=json.loads(line)
        except: continue
        if d.get('id') in S: S[d['id']].append(d)
t0=S['posvel'][0]['ts']
def ts(r): return np.array([(x['ts']-t0)/1e6 for x in r])
# --- series ---
tpv=ts(S['posvel']); pos=np.array([r['pos'] for r in S['posvel']]); vel=np.array([r['vel'] for r in S['posvel']])
alt=-pos[:,2]                                   # NED z down -> altitude up
tcr=ts(S['ctrl_ref'])
psp=np.array([r['pos_sp'] for r in S['ctrl_ref']])
duty=np.array([r['motor_duty'] for r in S['ctrl_ref']])
thr=np.array([r['total_thrust'] for r in S['ctrl_ref']])
tst=ts(S['status']); volt=np.array([r['voltage'] for r in S['status']]); fst=np.array([r['flight_state'] for r in S['status']])
# 飛行窓: fs==5 (FLYING)。離陸後 t>=8s 〜 t<=178s に限定(離着陸過渡除外)
T0,T1=8.0,178.0
m=(tpv>=T0)&(tpv<=T1)
# pos_sp を posvel 時刻へ補間
pspx=np.interp(tpv,tcr,psp[:,0]); pspy=np.interp(tpv,tcr,psp[:,1])
drift=np.hypot(pos[:,0]-pspx, pos[:,1]-pspy)
def stat(x): return float(np.mean(x)),float(np.std(x)),float(np.max(x)-np.min(x))
def domfreq(x,fs_hz):
    x=x-np.mean(x); n=len(x)
    if n<16: return 0.0
    F=np.fft.rfft(x*np.hanning(n)); f=np.fft.rfftfreq(n,1/fs_hz); P=np.abs(F)**2
    P[0]=0; 
    return float(f[np.argmax(P)])
def bandfrac(x,fs_hz,lo,hi):
    x=x-np.mean(x); n=len(x); F=np.fft.rfft(x*np.hanning(n)); f=np.fft.rfftfreq(n,1/fs_hz); P=np.abs(F)**2; P[0]=0
    tot=P.sum(); return float(P[(f>=lo)&(f<=hi)].sum()/tot) if tot>0 else 0.0
fs_pv=1/np.median(np.diff(tpv))   # posvel sample rate

print("="*72)
print("POS_HOLD 3分ログ解析  stampfly_udp_20260627T020137  (換算ゲイン)")
print(f"  飛行窓 {T0}-{T1}s  posvel fs≈{fs_pv:.0f}Hz  保持高度≈{np.median(alt[m]):.3f}m")
print("="*72)
# --- 全体 ---
print("\n[全体ふらつき(飛行窓)]")
am,asd,ap2p=stat(alt[m]); print(f"  高度  : 平均{am:.3f}m  std {asd*1000:.1f}mm  p2p {ap2p*1000:.0f}mm  卓越{domfreq(alt[m],fs_pv):.2f}Hz")
dm,dsd,_=stat(drift[m]); print(f"  水平drift: 平均{dm*1000:.0f}mm  std {dsd*1000:.1f}mm  最大{drift[m].max()*1000:.0f}mm")
vxm,vxs,_=stat(vel[m,0]); vym,vys,_=stat(vel[m,1])
print(f"  水平速度: vx {vxm:+.3f}±{vxs:.3f}  vy {vym:+.3f}±{vys:.3f} m/s  速さmax {np.hypot(vel[m,0],vel[m,1]).max():.3f}")
print(f"  高度0.2-2Hz帯エネルギー比 {bandfrac(alt[m],fs_pv,0.2,2)*100:.0f}%  水平drift卓越 {domfreq(drift[m],fs_pv):.2f}Hz")

# --- 時間分割 (30s窓) で 電圧 vs ふらつき vs 制御出力 ---
print("\n[30s窓: 電圧サグ と ふらつき・ホバーduty の推移]")
print(f"  {'窓[s]':>10} {'V[V]':>6} {'高度std':>8} {'高度p2p':>8} {'drift_rms':>9} {'vel_std':>8} {'duty平均':>8} {'duty_std':>8} {'thrust':>7}")
edges=np.arange(T0,T1+1,30)
rows=[]
for a,b in zip(edges[:-1],edges[1:]):
    mm=(tpv>=a)&(tpv<b); mc=(tcr>=a)&(tcr<b); mv=(tst>=a)&(tst<b)
    if mm.sum()<10: continue
    V=float(np.mean(volt[mv])) if mv.sum() else float(np.interp((a+b)/2,tst,volt))
    altsd=np.std(alt[mm]); altp2p=alt[mm].max()-alt[mm].min()
    drms=np.sqrt(np.mean(drift[mm]**2))
    vsd=np.std(np.hypot(vel[mm,0],vel[mm,1]))
    dutym=float(np.mean(duty[mc])); dutysd=float(np.mean(np.std(duty[mc],axis=1)))  # avg per-motor spread
    thrm=float(np.mean(thr[mc]))
    rows.append((a,b,V,altsd,altp2p,drms,vsd,dutym,dutysd,thrm))
    print(f"  {a:4.0f}-{b:<4.0f} {V:>6.3f} {altsd*1000:>7.1f}m {altp2p*1000:>7.0f}m {drms*1000:>8.0f}m {vsd:>8.3f} {dutym:>8.3f} {dutysd:>8.4f} {thrm:>7.3f}")
# 相関
R=np.array([r[2:] for r in rows])  # V,altsd,altp2p,drms,vsd,dutym,dutysd,thr
def corr(a,b): return float(np.corrcoef(a,b)[0,1])
print("\n[電圧 vs 各量の相関(窓平均)]")
print(f"  corr(V, 高度std)   = {corr(R[:,0],R[:,1]):+.2f}")
print(f"  corr(V, drift_rms) = {corr(R[:,0],R[:,3]):+.2f}")
print(f"  corr(V, vel_std)   = {corr(R[:,0],R[:,4]):+.2f}")
print(f"  corr(V, ホバーduty)= {corr(R[:,0],R[:,5]):+.2f}   (Vサグでduty上昇=thrust→duty補償)")
