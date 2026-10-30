import numpy as np, csv, pathlib
from scipy.optimize import brentq
Ke=Kt=5.905e-4; Cq=3.38e-11; Ct=7.60e-9; a0=0.0443; b0=2.915e-9; Rs=0.593; Ron=0.065
_t=np.array([[float(r['D']),float(r['R_out_ohm'])] for r in csv.DictReader(open(pathlib.Path(__file__).resolve().parents[2]/'rout'/'rout_table_c01.csv'))])
def Rout(D): return float(np.interp(D,_t[:,0],_t[:,1]))
def _solve(V_eff,R,Ke_,Kt_,Cq_,i0,i1):
    # Kt*(V_eff-Ke*w)/R = Cq w^2 + Kt(i0+i1 w^2)
    f=lambda w: Kt_*(V_eff-Ke_*w)/R-(Cq_*w*w+Kt_*(i0+i1*w*w))
    if f(0.0)<=0: return 0.0
    return brentq(f,0.0,V_eff/Ke_)
def rout(D,V):
    R=Rout(D); w=_solve(V,R,Ke,Kt,Cq,a0,b0); return w,(V-Ke*w)/R
def chop(D,V):
    R=Rs+Ron; w=_solve(D*V,R,Ke,Kt,Cq,a0,b0); return w,D*(D*V-Ke*w)/R
def sils(D,V):
    Km,Rm,Cq2,Qf=5.682e-4,0.593,4.10e-11,9.507e-6
    f=lambda w: Km*(D*V-Km*w)/Rm-(Cq2*w*w+Qf)
    if f(0.0)<=0: return 0.0,0.0
    w=brentq(f,0.0,D*V/Km); return w,(D*V-Km*w)/Rm
def thrust_rout(D,V): return Ct*rout(D,V)[0]**2
def duty_rout_for_thrust(T,V):
    g=lambda D: thrust_rout(D,V)-T
    lo,hi=0.05,1.0
    if g(lo)>0 or g(hi)<0: return np.nan
    return brentq(g,lo,hi)
