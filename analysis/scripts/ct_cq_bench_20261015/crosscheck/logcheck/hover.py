import numpy as np
AZ_MAX=0.5; VZ_MAX=0.2; ALT_MIN=0.3; SKIP=2.0; MINWIN=1.0; GYRO_FLIP=5.0; FLIP_PAD=1.0
def smooth(x,n): return np.convolve(x,np.ones(n)/n,mode='same')
def hover_mask(R, flip_exclude=False, az_max=None, vz_max=None):
    az_max=AZ_MAX if az_max is None else az_max; vz_max=VZ_MAX if vz_max is None else vz_max
    t=R['c_t']
    idx=np.searchsorted(R['s_t'],t,side='right')-1
    fs=np.where(idx>=0,R['s_fs'][np.clip(idx,0,None)],-1)
    fly=(fs==5)
    tfl=t[fly][0] if fly.any() else np.inf
    alt=np.interp(t,R['p_t'],R['p_alt']); vz=np.interp(t,R['p_t'],R['p_vz'])
    # vertical accel from vz: 0.5 s moving average then derivative then 0.5 s average
    n=max(3,int(0.5*R['rate'])|1)
    vzs=smooth(vz,n); az=smooth(np.gradient(vzs)*R['rate'],n)  # nominal spacing: timestamps contain duplicates
    gn=np.linalg.norm(R['i_gyro'],axis=1); gmax=float(gn.max())
    tflip=np.inf
    if flip_exclude:
        big=np.flatnonzero(gn>GYRO_FLIP)
        if len(big): tflip=R['i_t'][big[0]]
    ok_fly=fly; ok_alt=alt>ALT_MIN; ok_t=t>=tfl+SKIP; ok_az=np.abs(az)<az_max; ok_vz=np.abs(vzs)<vz_max; ok_flip=t<tflip-FLIP_PAD
    # local exclusion of violent-rotation spikes in any log (gyro norm > GYRO_FLIP rad/s): +-1.5 s
    ti=R['i_t']; spike=np.zeros(len(t),bool)
    for ts_ in ti[gn>GYRO_FLIP]:
        spike|=(np.abs(t-ts_)<1.5)
    ok_flip=ok_flip&~spike
    ok=ok_fly&ok_alt&ok_t&ok_az&ok_vz&ok_flip
    diag=dict(frac_flying=float(ok_fly.mean()),frac_alt=float((ok_fly&ok_alt).mean()),frac_t=float((ok_fly&ok_alt&ok_t).mean()),
      frac_az=float((ok_fly&ok_az).mean()),frac_vz=float((ok_fly&ok_vz).mean()),frac_ok=float(ok.mean()),
      t_flying_s=float(fly.sum()/R['rate']),alt_max=float(alt.max()),gyro_norm_max=gmax,t_flip=None if not np.isfinite(tflip) else float(tflip-t[0]),
      hover_total_s=float(ok.sum()/R['rate']),frac_spike_excl=float(spike.mean()))
    return ok,fs,diag,tfl
def segments(t,ok,gap=0.1):
    segs=[];i=0;n=len(t)
    while i<n:
        if ok[i]:
            j=i
            while j+1<n and ok[j+1] and t[j+1]-t[j]<gap: j+=1
            segs.append((t[i],t[j])); i=j+1
        else: i+=1
    return segs
