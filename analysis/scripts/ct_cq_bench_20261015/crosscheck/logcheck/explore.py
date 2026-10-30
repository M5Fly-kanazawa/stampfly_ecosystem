import sys, os; sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from common import *
import numpy as np
for s in LOGS:
    R=load(s)
    t0=R['c_t'][0]
    print('==',s,'dur ctrl %.1f'%(R['c_t'][-1]-t0),'status n',len(R['s_t']))
    print(' fs counts',{int(k):int((R['s_fs']==k).sum()) for k in np.unique(R['s_fs'])},'modes',{int(k):int((R['c_mode']==k).sum()) for k in np.unique(R['c_mode'])})
    print(' V %.3f..%.3f'%(R['s_v'].min(),R['s_v'].max()),'tof max %.2f'%R['t_d'].max(),'posz min %.2f'%R['p_pos'][:,2].min(),'duty mean max %.3f'%R['c_duty'].mean(1).max())
    print(' dt ctrl med %.4f max %.3f ; imu med %.5f max %.3f'%(np.median(np.diff(R['c_t'])),np.diff(R['c_t']).max(),np.median(np.diff(R['i_t'])),np.diff(R['i_t']).max()))
    print(' status fs seq:', ''.join(str(x) for x in R['s_fs']))
    print(' pid changes:', {a:len(np.unique(R['s_pid'][a],axis=0)) for a in R['s_pid']}, 'cur', np.nanmin(R['s_cur']) if np.isfinite(R['s_cur']).any() else None, np.nanmax(R['s_cur']) if np.isfinite(R['s_cur']).any() else None)
