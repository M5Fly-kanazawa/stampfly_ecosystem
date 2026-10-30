import sys, os; sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from data import *; from hover import *; from models import *
out={}
for name in JSONL+BUNDLE_FLIP+BUNDLE_BENCH:
    R=get(name); ti=R['i_t']; d=np.diff(ti); tc=R['c_t']; dc=np.diff(tc)
    hv=hover_mask(R,flip_exclude=name in BUNDLE_FLIP)[2] if name not in BUNDLE_BENCH else {}
    s=dict(duration_s=float(ti[-1]-ti[0]),imu_n=len(ti),imu_dup_ts=int((d<=0).sum()),imu_gaps_gt20ms=int((d>0.02).sum()),imu_max_gap_s=float(d.max()),
      duty_series_dup_ts=int((dc<=0).sum()),duty_gaps_gt60ms=int((dc>0.06).sum()),status_n=len(R['s_t']),
      status_dt_unique_s=sorted(set(np.round(np.diff(R['s_t']),2).tolist()))[:6],
      V_min=float(R['s_v'].min()),V_max=float(R['s_v'].max()),has_current=bool(np.isfinite(R['s_cur']).any()),
      modes=sorted(set(np.asarray(R['c_mode']).astype(int).tolist())) if 'c_mode' in R else None,
      gyro_norm_max=float(np.linalg.norm(R['i_gyro'],axis=1).max()),airborne_s=float((np.interp(tc,R['s_t'],R['s_fs'])>4.5).sum()/R['rate']))
    out[name]=s; print(name,s)
# sweep description from motor stream
df=load_aligned(bundle_path('motor_sweep_bench')); t=(df.timestamp_us-df.timestamp_us.iloc[0])/1e6
steps={}
for c in ['duty_FR','duty_RR','duty_RL','duty_FL']:
    x=df[c].to_numpy(); on=np.flatnonzero(x>0)
    peak=x.max(); steps[c]=dict(t_start=float(t[on[0]]),t_end=float(t[on[-1]]),dur=float(t[on[-1]]-t[on[0]]),peak=float(peak),t_peak=float(t[np.argmax(x)]))
    print(c,steps[c])
out['sweep_steps']=steps
# distinct duty values in sweep
x=df['duty_FR'].to_numpy(); print('FR samples',np.round(x[(t>13)&(t<17.5)][::40],3))
print(df[['flight_mode','flight_state']].drop_duplicates().to_string())
json.dump(out,open(OUT+'/sanity.json','w'),indent=1,default=str)
