import sys, os; sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from data import *; from hover import *; from models import *
import warnings; warnings.filterwarnings('ignore')
def I_of(fn,D,V):
    return 0.0 if D<0.005 else fn(D,V)[1]
FN={'rout':rout,'chop':chop,'sils':sils}
def pred(D4,V): return {k:sum(I_of(f,d,V) for d in D4) for k,f in FN.items()}
def run(name,mode,relax=False):
    R=get(name); t=R['c_t']
    if mode=='hover':
        ok,fs,diag,tfl=hover_mask(R,flip_exclude=name in BUNDLE_FLIP,**(dict(az_max=3.0,vz_max=0.6) if relax else {}))
    else: ok=np.ones(len(t),bool)
    cur=R['s_cur']*10.0  # true mA
    # overhead: IDLE state (fs 1/2) with all duties zero for the previous 3 s
    off=[]
    for k,(ts,fs_) in enumerate(zip(R['s_t'],R['s_fs'])):
        if fs_ in (0,1,2) and not np.isnan(cur[k]):
            m=(t>ts-3)&(t<=ts)
            if m.sum()>0 and R['c_duty'][m].max()==0: off.append(cur[k])
    ovh=float(np.mean(off)) if off else np.nan
    rows=[]
    step=max(1,int(R['rate']//50))
    for k,ts in enumerate(R['s_t']):
        if np.isnan(cur[k]): continue
        m=(t>ts-1)&(t<=ts)
        if m.sum()<0.9*R['rate']: continue
        if mode=='hover' and not ok[m].all(): continue
        D4=R['c_duty'][m].mean(0); V=R['s_v'][k]
        P=pred(D4,V)
        # variant: average of instantaneous predictions (subsampled to 50 Hz)
        idx=np.flatnonzero(m)[::step]
        Pi={kk:np.mean([sum(I_of(f,d,V) for d in R['c_duty'][i]) for i in idx]) for kk,f in FN.items()}
        # variant: centred window (ts-0.5, ts+0.5)
        mc=(t>ts-0.5)&(t<=ts+0.5)
        Pc=pred(R['c_duty'][mc].mean(0),V) if mc.sum()>0.9*R['rate']/1 *0.9 else None
        rows.append(dict(t=float(ts-t[0]),V=float(V),Dbar=float(D4.mean()),D4=D4.tolist(),I_raw_mA=float(cur[k]),I_meas_A=float((cur[k]-ovh)/1000) if not np.isnan(ovh) else None,
            pred=P,pred_inst=Pi,pred_centred=Pc))
    return dict(log=name,group=group(name),mode=mode,relaxed=relax,overhead_mA=ovh,n_overhead_samples=len(off),n=len(rows),rows=rows)
def summarize(res,key='pred',dmin=0.0):
    out={}
    rr=[r for r in res['rows'] if r['I_meas_A'] is not None and r['Dbar']>=dmin and r[key] is not None]
    for h in ('rout','chop','sils'):
        ra=np.array([r['I_meas_A']/r[key][h] for r in rr if r[key][h]>0.02])
        if len(ra)==0: out[h]=None; continue
        out[h]=dict(n=int(len(ra)),median=float(np.median(ra)),p16=float(np.percentile(ra,16)),p84=float(np.percentile(ra,84)),std=float(ra.std()),
            sum_ratio=float(sum(r['I_meas_A'] for r in rr if r[key][h]>0.02)/sum(r[key][h] for r in rr if r[key][h]>0.02)))
    return out
if __name__=='__main__':
    allres={}
    jobs=[(n,'hover',False) for n in JSONL[6:]+BUNDLE_FLIP]+[(n,'hover',True) for n in BUNDLE_FLIP]+[(n,'all',False) for n in BUNDLE_BENCH]
    for name,mode,relax in jobs:
        r=run(name,mode,relax); key=name+('_relaxed' if relax else '')
        r['summary']={k:summarize(r,k) for k in ('pred','pred_inst','pred_centred')}
        allres[key]=r
        print(key,mode,'ovh %.1f mA (n=%d)'%(r['overhead_mA'],r['n_overhead_samples']),'n',r['n'])
        for k in ('pred','pred_inst','pred_centred'):
            print('   ',k,{h:(None if v is None else (v['n'],round(v['median'],3),round(v['p16'],3),round(v['p84'],3),round(v['sum_ratio'],3))) for h,v in r['summary'][k].items()})
    json.dump(allres,open(OUT+'/test2.json','w'),indent=1)
