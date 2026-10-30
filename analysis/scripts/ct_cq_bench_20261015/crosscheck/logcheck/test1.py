import sys, os; sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from data import *; from hover import *; from models import *
from scipy import stats
CHUNK=5.0
res={}
for name,relax in [(n,False) for n in JSONL+BUNDLE_FLIP]+[(n,True) for n in BUNDLE_FLIP]:
    R=get(name); t=R['c_t']
    ok,fs,diag,tfl=hover_mask(R, flip_exclude=name in BUNDLE_FLIP, **(dict(az_max=3.0,vz_max=0.6) if relax else {}))
    if relax: name=name+'_relaxed'
    segs=segments(t,ok)
    wins=[]
    for a,b in segs:
        if b-a<MINWIN: continue
        k=max(1,int(np.floor((b-a)/CHUNK))); e=np.linspace(a,b,k+1)
        wins+= [(u,v) for u,v in zip(e[:-1],e[1:]) if v-u>=MINWIN]
    rows=[]
    for a,b in wins:
        m=(t>=a)&(t<=b); D=R['c_duty'][m].mean(0)
        rows.append(dict(t0=float(a-t[0]),t1=float(b-t[0]),dur=float(b-a),Dbar=float(D.mean()),V=float(np.mean(np.interp(t[m],R['s_t'],R['s_v']))),thr=float(R['c_thr'][m].mean())))
    r=dict(log=name,group=group(name),duty_source=R['src'],n_win=len(rows),diag=diag,rows=rows)
    if len(rows)>=1:
        r['hover_s_in_windows']=float(sum(x['dur'] for x in rows))
        V=np.array([x['V'] for x in rows]);D=np.array([x['Dbar'] for x in rows]);P=D*V
        r['Vspan']=float(V.max()-V.min()); r['too_small_span']=bool(r['Vspan']<0.15)
        r['DV_mean']=float(P.mean())
        if len(rows)>=3:
            dof=len(V)-2; Vc=V-V.mean(); Sxx=Vc@Vc
            def fit(y):
                b=(Vc@(y-y.mean()))/Sxx; res_=y-y.mean()-b*Vc; s=np.sqrt(res_@res_/dof/Sxx); return b,stats.t.ppf(.975,dof)*s,res_
            b,ci,rs=fit(P); r.update(DV_slope=float(b),DV_slope_ci95=float(ci),DV_resid_rms=float(np.sqrt(np.mean(rs**2))),DV_slope_rel_pct_per_V=float(b/P.mean()*100))
            b2,ci2,_=fit(D); r.update(meas_dDdV=float(b2),meas_dDdV_ci95=float(ci2))
        Vb,Db=V.mean(),D.mean()
        Dchop=Db*Vb/V; Tref=thrust_rout(Db,Vb)
        Drout=np.array([duty_rout_for_thrust(Tref,v) for v in V])
        r.update(Tref_4x_N=float(4*Tref),rms_chop=float(np.sqrt(np.mean((D-Dchop)**2))),rms_rout=float(np.sqrt(np.mean((D-Drout)**2))))
        if len(V)>=2:
            r.update(chop_dDdV=float(np.polyfit(V,Dchop,1)[0]),rout_dDdV=float(np.polyfit(V,Drout,1)[0]))
        r['Dchop']=Dchop.tolist(); r['Drout']=Drout.tolist()
    res[name]=r
    print(name,r['group'],'nwin',r['n_win'],{k:(round(v,4) if isinstance(v,float) else v) for k,v in r.items() if k not in('rows','Dchop','Drout','diag','log','group')}); print('   ',{k:(round(v,3) if isinstance(v,float) else v) for k,v in diag.items()})
json.dump(res,open(OUT+'/test1.json','w'),indent=1)
