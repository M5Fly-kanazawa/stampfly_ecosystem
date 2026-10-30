import sys, os, json, numpy as np
sys.path[:0]=[os.path.join(REPO,'tools'),os.path.join(REPO,'lib'),os.path.join(REPO,'tools','log_analyzer')]
from common import load as load_jsonl, REPO, LOGS_DIR, RESULTS
from sysid.loader import load_aligned
OUT=RESULTS
JSONL=["20260614T201629","20260614T214537","20260627T020137","20260627T164611","20260627T165713","20260629T145526",
"20260717T211648","20260717T223206","20260717T224508","20260717T224906","20260717T231940","20260718T022929","20260718T064222","20260908T121243"]
BUNDLE_FLIP=["flip_test1","flip_test2"]; BUNDLE_BENCH=["motor_sweep_bench","ground_spin_stab"]
def group(name):
    return {'2026':name[:8]}['2026'] if name[0]=='2' else '20261002'
def bundle_path(name):
    return f'{LOGS_DIR}/{name}.sflog.zip' if name in BUNDLE_FLIP+BUNDLE_BENCH else f'{LOGS_DIR}/{name}.sflog.zip'
def get(name):
    """Unified record dict. c_* = commanded duty series (50 Hz ctrl_ref for JSONL logs, 400 Hz motor stream for native bundles)."""
    if name in JSONL:
        R=load_jsonl(name); R['rate']=50.0; R['src']='ctrl_ref 50Hz'
        R['p_alt']=-R['p_pos'][:,2]; R['p_vz']=-R['p_vel'][:,2]
        R['pidk']={a:np.median(R['s_pid'][a],axis=0) for a in ('roll','pitch','yaw')}
        return R
    df=load_aligned(bundle_path(name))
    R={'rate':400.0,'src':'motor 400Hz'}
    d=df.dropna(subset=['duty_FR','duty_RR','duty_RL','duty_FL'])
    R['c_t']=d.timestamp_us.to_numpy()*1e-6
    R['c_duty']=d[['duty_FR','duty_RR','duty_RL','duty_FL']].to_numpy(float)
    R['c_thr']=d.total_thrust.to_numpy(float)
    s=df.dropna(subset=['status_timestamp_us','voltage']).drop_duplicates('status_timestamp_us')
    R['s_t']=s.status_timestamp_us.to_numpy()*1e-6; R['s_v']=s.voltage.to_numpy(float)
    R['s_cur']=s.current_ma.to_numpy(float); R['s_fs']=s.flight_state.to_numpy(float)
    R['s_pid']={a:s[[f'pid_{a}_kp',f'pid_{a}_ti',f'pid_{a}_td']].to_numpy(float) for a in ('roll','pitch','yaw')}
    R['pidk']={a:np.median(R['s_pid'][a],axis=0) for a in ('roll','pitch','yaw')}
    R['p_t']=df.timestamp_us.to_numpy()*1e-6; R['p_alt']=-df.pos_z.to_numpy(float); R['p_vz']=-df.vel_z.to_numpy(float)
    R['i_t']=R['p_t']; R['i_gyro']=df[['gyro_x','gyro_y','gyro_z']].to_numpy(float)
    R['mode']=df.flight_mode.to_numpy()
    R['c_mode']=d.flight_mode.to_numpy()
    return R
