import math

# ---- 物理定数 (SIL plant.hpp + actuator.cpp, = 実機StampFly同定値) ----
Am   = 5.39e-8   # V = Am w^2 + Bm w + Cm
Bm   = 6.33e-4
Cm   = 1.53e-2
Ct   = 1.00e-8   # T = Ct w^2  [N]
eff  = 1.0/1.12  # 実推力効率 0.893
d    = 0.023     # arm [m]
kappa= 9.71e-3   # Cq/Ct [m]
mass = 0.037
g    = 9.81
Vbat = 3.7

# ---- ホバー動作点 omega0 (実推力 mg/4 を満たす実ロータ速度) ----
T_hover = mass*g/4.0                 # per-motor real thrust [N]
w0 = math.sqrt(T_hover/(eff*Ct))     # real rotor speed at hover
V0 = Am*w0*w0 + Bm*w0 + Cm
print(f"T_hover/motor = {T_hover*1000:.2f} mN, w0 = {w0:.1f} rad/s, V0 = {V0:.3f} V, duty0 = {V0/Vbat:.3f}")

# ---- g_VT = dT_ideal/dV at w0  [N/V] (理想曲線の傾き=ファームthrustToDutyの線形化点) ----
g_VT = (2.0*Ct*w0) / (2.0*Am*w0 + Bm)
print(f"g_VT (dT/dV) = {g_VT:.5f} N/V")
print()

# ---- 橋渡し係数 ----
br_rp  = d     * g_VT   # roll/pitch:  Kp_vn = d*g_VT * Kp_orig   [Nm/V]
br_yaw = kappa * g_VT   # yaw:         Kp_vn = kappa*g_VT * Kp_orig
print(f"bridge roll/pitch  d*g_VT     = {br_rp:.6e} Nm/V")
print(f"bridge yaw         kappa*g_VT = {br_yaw:.6e} Nm/V")
print()

# ---- オリジナル M5StampFly レートゲイン ----
orig = {
    'roll' : dict(kp=0.65, ti=0.7, td=0.01,  br=br_rp),
    'pitch': dict(kp=0.95, ti=0.7, td=0.025, br=br_rp),
    'yaw'  : dict(kp=3.0,  ti=0.8, td=0.01,  br=br_yaw),
}
# ---- 現行 vehicle_new (params.cpp) ----
cur = {
    'roll' : dict(kp=3.40e-4, ti=0.4, td=0.017658),
    'pitch': dict(kp=5.16e-4, ti=0.4, td=0.017155),
    'yaw'  : dict(kp=5.31e-3, ti=1.6, td=0.01),
}

print(f"{'axis':6} | {'orig kp':>8} {'->vn kp':>11} | {'cur vn kp':>11} | {'ratio orig/cur':>14} | ti o->v  td o->v")
print("-"*92)
for ax in ['roll','pitch','yaw']:
    o=orig[ax]; c=cur[ax]
    kp_vn = o['br']*o['kp']
    ratio = kp_vn/c['kp']
    print(f"{ax:6} | {o['kp']:8.3f} {kp_vn:11.4e} | {c['kp']:11.4e} | {ratio:14.2f}x | {c['ti']}->{o['ti']}  {c['td']:.4f}->{o['td']}")
