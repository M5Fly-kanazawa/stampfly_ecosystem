#!/usr/bin/env python3
"""
Part B: transfer H(f) from the module's alpha_pred to the plant's truth alpha in SILS hover.
B 部: SILS ホバーでモジュールの α_pred からプラント真値 α への伝達 H(f)。

Usage: python sils_hover_h.py <bundle.sflog.zip> [t0 t1]    (PYTHONPATH=<repo>:<repo>/lib)
Bundles come from make_hover_scn.py scenarios (case N, case D, case N with --turbulence 0.03).
H = S_pred,truth / S_pred,pred, Welch Hann nperseg 256 at 400 Hz, band sums.
バンドルは make_hover_scn.py のシナリオ (N, D, N + --turbulence 0.03) から得る。
"""
import sys
import numpy as np
from scipy import signal
from tools.sysid import forward_model as fm
BANDS=[(0.5,2),(2,5),(5,10),(10,20),(20,40)]
def H_bands(pred, meas, fs=400.0, nper=256):
    f,Spp=signal.welch(pred,fs,'hann',nper,axis=0)
    _,Smm=signal.welch(meas,fs,'hann',nper,axis=0)
    _,Spm=signal.csd(pred,meas,fs,'hann',nper,axis=0)
    return f,Spp,Smm,Spm
def run(path, t0, t1, delay=fm.SILS_LOG_ALIGNMENT_DELAY_S, tag=''):
    motion=fm.load_logged_motion(path); tt,rate,alt=fm.load_truth(path)
    trace,tq=fm.predict_torque_trace(motion)
    pred=fm.accel_from_trace(motion,trace,tq,delay)
    truth=fm.accel_from_rate(motion.t_s,rate,motion.dt,t_src=tt)
    k=(motion.t_s>=t0)&(motion.t_s<t1); k[-1]=False
    f,Spp,Smm,Spm=H_bands(pred[k],truth[k])
    print(tag,path.split('/')[-2],f'[{t0},{t1}] rms pred',np.sqrt((pred[k]**2).mean(0)).round(2),'rms truth',np.sqrt((truth[k]**2).mean(0)).round(2))
    for a,ax in enumerate(('roll','pitch','yaw')):
        row=[]
        for lo,hi in BANDS:
            b=(f>=lo)&(f<hi)
            H=Spm[b,a].sum()/Spp[b,a].sum()   # S_pred,meas / S_pred,pred
            coh=np.abs(Spm[b,a].sum())**2/(Spp[b,a].sum()*Smm[b,a].sum())
            row.append(f'{lo}-{hi}Hz |H|={abs(H):.3f} coh={coh:.2f} P={Spp[b,a].sum():.2e}')
        print(' ',ax,' | '.join(row))
if __name__ == "__main__":
    t0, t1 = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (20.0, 74.0)
    run(sys.argv[1], t0, t1, tag="")
