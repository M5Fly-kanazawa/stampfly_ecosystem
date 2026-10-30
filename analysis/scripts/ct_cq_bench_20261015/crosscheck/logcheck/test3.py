import sys, os; sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from data import *; from hover import *
import rate_sysid as rs
from scipy import signal
import warnings, traceback; warnings.filterwarnings('ignore')
FREQS=[2,4,6,8,10]
YAW_LIMIT=lambda name: 2.2e-3 if name.startswith('202606') else (1.83e-3 if name.startswith('2026071') else 1.226e-3)
DIFF={'roll':lambda c:(c[:,2]+c[:,3])-(c[:,0]+c[:,1]),'pitch':lambda c:(c[:,0]+c[:,3])-(c[:,1]+c[:,2]),'yaw':lambda c:(c[:,0]+c[:,2])-(c[:,1]+c[:,3])}
GYR={'roll':0,'pitch':1,'yaw':2}
def frf(u,y,fs,fmodel=None):
    nper=int(2*fs) if fs<=50 else 800   # 50 Hz: 100 samples (2 s) ; 400 Hz: 800 samples (2 s) -> df=0.5 Hz
    if len(u)<nper*2: return None
    f,Puu=signal.welch(u,fs,nperseg=nper,noverlap=nper//2,window='hann')
    _,Pyy=signal.welch(y,fs,nperseg=nper,noverlap=nper//2,window='hann')
    _,Puy=signal.csd(u,y,fs,nperseg=nper,noverlap=nper//2,window='hann')
    H=Puy/Puu; coh=np.abs(Puy)**2/(Puu*Pyy)
    out=[]
    for fq in FREQS:
        i=int(np.argmin(abs(f-fq)))
        d=dict(f=float(f[i]),phase_deg=float(np.degrees(np.angle(H[i]))),mag=float(abs(H[i])),coh=float(coh[i]))
        if fmodel: T,L=fmodel; d['phase_model_deg']=float((-90-np.degrees(np.arctan(2*np.pi*f[i]*T))-360*f[i]*L+180)%360-180)
        out.append(d)
    return dict(nseg_len=nper,n_samples=len(u),points=out)
def span_mask(R,df,name):
    ok,fs,diag,tfl=hover_mask(R,flip_exclude=name in BUNDLE_FLIP)
    return ok,diag
res={}
for name in JSONL+BUNDLE_FLIP:
    R=get(name); t=R['c_t']
    df=load_aligned(bundle_path(name)); tdf=df.timestamp_us.to_numpy()*1e-6
    # airborne span: FLYING, >= SKIP after takeoff, before flip (flip logs) -- no altitude/accel gate (rate loop is identified in flight)
    idx=np.searchsorted(R['s_t'],tdf,side='right')-1
    fsd=np.where(idx>=0,R['s_fs'][np.clip(idx,0,None)],-1)
    fly=fsd==5; tfl=tdf[fly][0]
    gn=np.linalg.norm(df[['gyro_x','gyro_y','gyro_z']].to_numpy(),axis=1)
    big=np.flatnonzero(gn>GYR_T) if False else np.flatnonzero(gn>5.0)
    tflip=tdf[big[0]] if (name in BUNDLE_FLIP and len(big)) else np.inf
    air=fly&(tdf>=tfl+2.0)&(tdf<tflip-1.0)
    r=dict(log=name,group=group(name),gains_source='status.pid_* (median over log; constant in each log)' ,gains={a:R['pidk'][a].tolist() for a in R['pidk']},air_seconds=float(air.sum()/400.0),fits={})
    gains={a:dict(kp=float(R['pidk'][a][0]),ti=float(R['pidk'][a][1]),td=float(R['pidk'][a][2]),**({'limit':YAW_LIMIT(name)} if a=='yaw' else {})) for a in R['pidk']}
    variants=[('air',df[air].reset_index(drop=True))]
    if name not in BUNDLE_FLIP: variants.append(('whole',df))
    modes=['kp']+(['duty'] if name in BUNDLE_FLIP else [])
    for vn,d in variants:
        for mode in modes:
            for axis in ('roll','pitch','yaw'):
                key=f'{vn}/{mode}/{axis}'
                try:
                    f=rs.fit_from_df(d,axis,gains=gains[axis],input_mode=mode)
                    col={'roll':'rate_ref_roll','pitch':'rate_ref_pitch','yaw':'rate_ref_yaw'}[axis]; gcol={'roll':'gyro_x','pitch':'gyro_y','yaw':'gyro_z'}[axis]
                    # exact fit span rows
                    ok_=d[[col,gcol]].notna().all(axis=1).to_numpy(); ii=np.flatnonzero(ok_)
                    seg=d.iloc[ii[0]:ii[-1]+1]
                    r['fits'][key]=dict(b=float(f['b']),T=float(f['T']),L=float(f['L']),T_plus_L=float(f['T']+f['L']),coh_mean=float(f['coherence_mean']),
                        rate_ref_std=float(seg[col].std()),gyro_std=float(seg[gcol].std()),n=int(len(seg)),input_mode=f.get('input_mode'),**({'fit_extra':{k:float(v) for k,v in f.items() if isinstance(v,(int,float)) and k not in('b','T','L')}}))
                except Exception as e:
                    r['fits'][key]=dict(error=str(e)[:200])
    # FRF
    r['frf']={}
    airt=(t>=tdf[air][0])&(t<=tdf[air][-1]) if air.any() else np.zeros(len(t),bool)
    ti=R['i_t']; G=R['i_gyro']
    if airt.sum()>0:
        for axis in ('roll','pitch','yaw'):
            u=DIFF[axis](R['c_duty'])[airt]
            fs_=R['rate']
            if fs_<=50:
                ker=np.ones(8)/8; gf=np.convolve(G[:,GYR[axis]],ker,mode='same')   # 8-sample boxcar anti-alias then sample at ctrl times
                y=np.interp(t[airt],ti,gf)
            else:
                y=np.interp(t[airt],ti,G[:,GYR[axis]])
            fm=None
            kf=r['fits'].get(f'air/kp/{axis}')
            if kf and 'T' in kf: fm=(kf['T'],kf['L'])
            r['frf'][f'{axis}_{int(fs_)}Hz']=frf(u,y,fs_,fm)
            if name in BUNDLE_FLIP:
                # also 50 Hz variant from the same data (decimated) for comparison
                pass
    res[name]=r
    print(name,'air %.1fs'%r['air_seconds'],{k:(round(v['b']),round(v['T']*1e3,2),round(v['L']*1e3,2),round(v['coh_mean'],2)) if 'b' in v else v for k,v in r['fits'].items() if '/yaw' not in k})
    sys.stdout.flush()
json.dump(res,open(OUT+'/test3.json','w'),indent=1)
