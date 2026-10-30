import sys, os; from common import REPO, LOGS_DIR
sys.path[:0]=[os.path.join(REPO,'tools'),os.path.join(REPO,'lib')]
from sysid.loader import load_aligned
import numpy as np, pandas as pd
pd.set_option('display.width',250); pd.set_option('display.max_rows',500)
for f in ['motor_sweep_bench','ground_spin_stab','flip_test1','flip_test2']:
    df=load_aligned(f'{LOGS_DIR}/{f}.sflog.zip')
    t=(df.timestamp_us-df.timestamp_us.iloc[0])/1e6
    print('--',f,'dur',t.iloc[-1],'rows',len(df))
    g=df.assign(t=t.round(0)).groupby('t')[['duty_FR','duty_RR','duty_RL','duty_FL','voltage','current_ma','flight_state','flight_mode','distance','gyro_x']].agg(lambda x:x.mean())
    g['gm']=df.assign(t=t.round(0)).groupby('t').apply(lambda d: np.sqrt((d[['gyro_x','gyro_y','gyro_z']]**2).sum(1)).max())
    print(g.round(3).to_string())
    print(df.attrs)
