import sys,os,json; sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from common import *; from models import *
import numpy as np
OUT=RESULTS
AZ_MAX=0.5; VZ_MAX=0.2; ALT_MIN=0.3; SKIP=2.0; MINWIN=1.0; CHUNK=5.0
def smooth(x,n): return np.convolve(x,np.ones(n)/n,mode='same')
def windows(R):
    t=R['c_t']
    fs=np.interp(t,R['s_t'],R['s_fs'].astype(float),left=np.nan,right=np.nan)  # note: linear interp of state codes; use hold instead
    idx=np.searchsorted(R['s_t'],t,side='right')-1
    fs=np.where(idx>=0,R['s_fs'][np.clip(idx,0,None)],-1)
    fly=(fs==5)
    # takeoff completion: first FLYING status
    tfl=t[fly][0] if fly.any() else np.inf
    alt=np.interp(t,R['p_t'],-R['p_pos'][:,2])
    vz=np.interp(t,R['p_t'],-R['p_vel'][:,2])
    # vertical accel: central diff of vz (smoothed 0.5 s) over +-0.25s
    vzs=smooth(vz,25); az=np.gradient(vzs,t)
    az=smooth(az,25)
    ok=fly&(alt>ALT_MIN)&(t>=tfl+SKIP)&(np.abs(az)<AZ_MAX)&(np.abs(vzs)<VZ_MAX)
    segs=[];i=0;n=len(t)
    while i<n:
        if ok[i]:
            j=i
            while j+1<n and ok[j+1] and t[j+1]-t[j]<0.1: j+=1
            segs.append((t[i],t[j])); i=j+1
        else: i+=1
    wins=[]
    for a,b in segs:
        if b-a<MINWIN: continue
        k=max(1,int(np.floor((b-a)/CHUNK)))
        edges=np.linspace(a,b,k+1)
        for u,v in zip(edges[:-1],edges[1:]):
            if v-u>=MINWIN: wins.append((u,v))
    return wins,dict(tfl=tfl,ok_frac=float(ok.mean()),alt_max=float(alt.max()),segs=segs)
res={}
for s in LOGS:
    R=load(s); t=R['c_t']
    wins,info=windows(R)
    rows=[]
    for a,b in wins:
        m=(t>=a)&(t<=b)
        D=R['c_duty'][m].mean(0)
        Vm=np.mean(np.interp(t[m],R['s_t'],R['s_v']))
        rows.append(dict(t0=a-t[0],t1=b-t[0],dur=b-a,Dbar=float(D.mean()),Dm=D.tolist(),V=float(Vm),thr=float(R['c_thr'][m].mean())))
    r=dict(date=s,n_win=len(rows),rows=rows,info=dict(ok_frac=info['ok_frac'],alt_max=info['alt_max'],nseg=len(info['segs']),seg_total=sum(b-a for a,b in info['segs'])))
    if len(rows)>=2:
        V=np.array([x['V'] for x in rows]);D=np.array([x['Dbar'] for x in rows]);P=D*V
        r['Vspan']=float(V.max()-V.min())
        X=np.vstack([np.ones_like(V),V-V.mean()]).T
        beta,*_=np.linalg.lstsq(X,P,rcond=None)
        resd=P-X@beta; dof=len(V)-2
        s2=resd@resd/dof if dof>0 else np.nan
        se=np.sqrt(s2/((V-V.mean())@(V-V.mean()))) if dof>0 else np.nan
        from scipy import stats
        tc=stats.t.ppf(0.975,dof) if dof>0 else np.nan
        r['DV_mean']=float(P.mean()); r['DV_slope']=float(beta[1]); r['DV_slope_ci95']=float(tc*se); r['DV_resid_rms']=float(np.sqrt(np.mean(resd**2)))
        r['DV_slope_rel_pct_per_V']=float(beta[1]/P.mean()*100)
        # hypotheses, normalised through log mean point
        Vb,Db=V.mean(),D.mean()
        c=Db*Vb; Dchop=c/V
        Tref=thrust_rout(Db,Vb)
        Drout=np.array([duty_rout_for_thrust(Tref,v) for v in V])
        r['Tref_per_motor_N']=float(Tref); r['Tref_4x_N']=float(4*Tref)
        r['rms_chop']=float(np.sqrt(np.mean((D-Dchop)**2))); r['rms_rout']=float(np.sqrt(np.mean((D-Drout)**2)))
        r['Dchop_slope_dD_dV']=float(np.polyfit(V,Dchop,1)[0]); r['Drout_slope_dD_dV']=float(np.polyfit(V,Drout,1)[0])
        r['meas_slope_dD_dV']=float(np.polyfit(V,D,1)[0])
        # CI on measured dD/dV
        X2=np.vstack([np.ones_like(V),V-V.mean()]).T; b2,*_=np.linalg.lstsq(X2,D,rcond=None); rr=D-X2@b2
        s22=rr@rr/dof if dof>0 else np.nan; se2=np.sqrt(s22/((V-V.mean())@(V-V.mean()))) if dof>0 else np.nan
        r['meas_slope_ci95']=float(tc*se2)
        r['too_small_span']=bool(r['Vspan']<0.15)
        r['Dchop_curve']=Dchop.tolist(); r['Drout_curve']=Drout.tolist()
        # thrust command vs V (extra)
        T=np.array([x['thr'] for x in rows]); r['thr_slope_per_V']=float(np.polyfit(V,T,1)[0]); r['thr_mean']=float(T.mean())
    res[s]=r
json.dump(res,open(OUT+'/test1.json','w'),indent=1)
for s,r in res.items():
    print(s,'nwin',r['n_win'],{k:(round(v,4) if isinstance(v,float) else v) for k,v in r.items() if k not in('rows','Dchop_curve','Drout_curve','info')},r['info'])
