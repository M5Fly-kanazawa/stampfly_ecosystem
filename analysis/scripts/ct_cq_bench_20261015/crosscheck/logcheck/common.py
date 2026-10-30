import json, numpy as np
import os, pathlib
HERE = pathlib.Path(__file__).resolve().parent
REPO = os.environ.get('SF_REPO') or str(HERE.parents[4])  # repository root / リポジトリのルート
RESULTS = str(HERE.parent / 'results')  # JSON outputs / 出力先
ROUT_CSV = str(HERE.parents[1] / 'rout' / 'rout_table_c01.csv')
LOGS_DIR = os.environ.get('SF_LOGS') or REPO + '/logs'  # logs are not in the repository / ログはリポジトリに無い
LOGS=["20260614T201629","20260614T214537","20260627T020137","20260627T164611","20260627T165713","20260629T145526",
"20260717T211648","20260717T223206","20260717T224508","20260717T224906","20260717T231940","20260718T022929","20260718T064222","20260908T121243"]
def load(s):
    d={k:[] for k in ['imu','posvel','tof_b','ctrl_ref','status','rate_ref']}
    for l in open(f'{LOGS_DIR}/stampfly_udp_{s}.jsonl'):
        try:o=json.loads(l)
        except Exception: continue
        if o.get('id') in d: d[o['id']].append(o)
    R={}
    ctr=d['ctrl_ref']
    R['c_t']=np.array([o['ts'] for o in ctr])*1e-6
    R['c_mode']=np.array([o['mode'] for o in ctr])
    R['c_thr']=np.array([o['total_thrust'] for o in ctr])
    R['c_duty']=np.array([o['motor_duty'] for o in ctr],float)
    st=d['status']
    R['s_t']=np.array([o['ts'] for o in st])*1e-6
    R['s_v']=np.array([o['voltage'] for o in st],float)
    R['s_fs']=np.array([o['flight_state'] for o in st])
    R['s_cur']=np.array([o.get('current_ma',np.nan) for o in st],float)
    R['s_pid']={a:np.array([o['pid_'+a] for o in st],float) for a in ('roll','pitch','yaw')}
    R['t_t']=np.array([o['ts'] for o in d['tof_b']])*1e-6
    R['t_d']=np.array([o['distance'] for o in d['tof_b']],float)
    R['t_s']=np.array([o['status'] for o in d['tof_b']])
    R['p_t']=np.array([o['ts'] for o in d['posvel']])*1e-6
    R['p_pos']=np.array([o['pos'] for o in d['posvel']],float)
    R['p_vel']=np.array([o['vel'] for o in d['posvel']],float)
    R['i_t']=np.array([o['ts'] for o in d['imu']])*1e-6
    R['i_gyro']=np.array([o['gyro'] for o in d['imu']],float)
    R['i_acc']=np.array([o['accel'] for o in d['imu']],float)
    return R
