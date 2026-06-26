import math, cmath

# ---- レートループ プラント: torque[Nm] -> angular rate[rad/s] ----
#   G(s) = eta_t * e^{-L s} / ( I * s * (tau_m s + 1) )
#   I=慣性(URDF), tau_m=モータ1次遅れ, L=純むだ時間(計算/通信), eta_t=実トルク効き
# ---- レートPID(vehicle_new標準形, η=0.125) ----
#   C(s) = Kp ( 1 + 1/(Ti s) + Td s/(0.125 Td s + 1) )
ETA = 0.125
def Cjw(w, kp, ti, td):
    s = 1j*w
    return kp*(1.0 + 1.0/(ti*s) + (td*s)/(ETA*td*s + 1.0))
def Gjw(w, I, tau_m, L, eta_t):
    s = 1j*w
    return eta_t*cmath.exp(-L*s) / (I*s*(tau_m*s + 1.0))
def Ljw(w, g, I, tau_m, L, eta_t):
    kp,ti,td = g
    return Cjw(w,kp,ti,td)*Gjw(w,I,tau_m,L,eta_t)

def margins(g, I, tau_m, L, eta_t):
    # 周波数グリッド(対数)
    ws=[10**(x/200) for x in range(-200, 700)]  # 0.01 .. ~3000 rad/s
    mag=[abs(Ljw(w,g,I,tau_m,L,eta_t)) for w in ws]
    ph =[math.degrees(cmath.phase(Ljw(w,g,I,tau_m,L,eta_t))) for w in ws]
    # unwrap phase
    for i in range(1,len(ph)):
        while ph[i]-ph[i-1]>180: ph[i]-=360
        while ph[i]-ph[i-1]<-180: ph[i]+=360
    # gain crossover (|L|=1)
    wc=None; pm=None
    for i in range(1,len(ws)):
        if (mag[i-1]-1)*(mag[i]-1)<0:
            # linear interp in log
            f=(1-mag[i-1])/(mag[i]-mag[i-1])
            wc=ws[i-1]*(ws[i]/ws[i-1])**f
            phc=ph[i-1]+f*(ph[i]-ph[i-1])
            pm=180+phc
            break
    # phase crossover (phase=-180) -> GM
    gm=None; wpc=None
    for i in range(1,len(ws)):
        if (ph[i-1]+180)*(ph[i]+180)<0:
            f=(-180-ph[i-1])/(ph[i]-ph[i-1])
            wpc=ws[i-1]*(ws[i]/ws[i-1])**f
            magc=mag[i-1]*(mag[i]/mag[i-1])**f
            gm=-20*math.log10(magc)
            break
    return wc,pm,gm,wpc

# 慣性 (URDF stampfly.urdf)
I={'roll':9.16e-6,'pitch':13.3e-6,'yaw':20.4e-6}
cur={'roll':(3.40e-4,0.4,0.017658),'pitch':(5.16e-4,0.4,0.017155),'yaw':(5.31e-3,1.6,0.01)}
conv={'roll':(9.759795e-4,0.7,0.01),'pitch':(1.426432e-3,0.7,0.025),'yaw':(1.901691e-3,0.8,0.01)}

# --- プラント較正: 現行rollが実機既知PM≈55°になるよう eta_t を合わせる ---
tau_m=0.02; L=0.012   # rate-loop: モータ1次遅れ~20ms + むだ時間~12ms(同定)
print("プラント較正(現行rollがPM≈55°になるeta_tを探索, tau_m=20ms,L=12ms):")
best=None
for et in [x/100 for x in range(5,101)]:
    wc,pm,gm,wpc=margins(cur['roll'],I['roll'],tau_m,L,et)
    if pm and best is None or (pm and abs(pm-55)<abs(best[1]-55)):
        best=(et,pm,wc,gm)
eta_t=best[0]
print(f"  -> eta_t={eta_t:.2f} で 現行roll PM={best[1]:.1f}° wc={best[2]:.1f}rad/s({best[2]/6.283:.2f}Hz) GM={best[3]:.1f}dB")
print(f"  (eta_t=実効トルク効き; 実機0.4-0.7の範囲なら妥当)")

print("\n"+"="*78)
print(f"レートループ 安定余裕比較 (同一較正プラント eta_t={eta_t:.2f}, tau_m=20ms, L=12ms)")
print("="*78)
print(f"{'axis':5} {'gains':10} {'wc[Hz]':>8} {'PM[deg]':>9} {'GM[dB]':>8} {'位相交差[Hz]':>11}")
print("-"*78)
for ax in ['roll','pitch','yaw']:
    for label,gset in [('現行',cur),('換算',conv)]:
        wc,pm,gm,wpc=margins(gset[ax],I[ax],tau_m,L,eta_t)
        wcs=f"{wc/6.283:.2f}" if wc else "  -"
        pms=f"{pm:.1f}" if pm else "  -"
        gms=f"{gm:.1f}" if gm else " inf"
        wpcs=f"{wpc/6.283:.2f}" if wpc else "  -"
        print(f"{ax:5} {label:10} {wcs:>8} {pms:>9} {gms:>8} {wpcs:>11}")
    print()

print("\n"+"="*78)
print("頑健性チェック: 異なる遅れ仮定で現行rollをPM55°に較正→換算rollと比較")
print("="*78)
print(f"{'tau_m[ms]':>9} {'L[ms]':>6} {'eta_t':>6} | {'現行 wc/PM/GM':>22} | {'換算 wc/PM/GM':>22}")
print("-"*78)
for tau_m,L in [(0.02,0.012),(0.05,0.02),(0.02,0.04),(0.08,0.0),(0.03,0.06)]:
    # 較正
    best=None
    for et in [x/200 for x in range(2,161)]:
        wc,pm,gm,wpc=margins(cur['roll'],I['roll'],tau_m,L,et)
        if pm:
            if best is None or abs(pm-55)<abs(best[1]-55): best=(et,pm,wc,gm)
    et=best[0]
    wc1,pm1,gm1,_=margins(cur['roll'],I['roll'],tau_m,L,et)
    wc2,pm2,gm2,_=margins(conv['roll'],I['roll'],tau_m,L,et)
    def _f(wc,pm,gm):
        return f"{wc/6.283:.2f}Hz/{pm:.0f}°/{gm:.0f}dB" if (wc and pm is not None and gm is not None) else "      (none)"
    s1=_f(wc1,pm1,gm1); s2=_f(wc2,pm2,gm2)
    print(f"{tau_m*1000:>9.0f} {L*1000:>6.0f} {et:>6.2f} | {s1:>22} | {s2:>22}")
print("\n判定: 全条件で 換算rollは現行より PM大 かつ wc高 なら相対傾向は頑健")
