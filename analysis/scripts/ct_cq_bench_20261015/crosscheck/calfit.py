"""usage: calfit.py <label> <bundle.sflog.zip> [repo_root] -- fits duty and kp modes; prints roll rate std in the roll chirp window."""
import sys, os, json, zipfile
import numpy as np
# repository root (default: this checkout) / リポジトリのルート
W = sys.argv[3] if len(sys.argv) > 3 else os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..'))
sys.path.insert(0, W + '/lib'); sys.path.insert(0, W + '/tools'); sys.path.insert(0, W + '/tools/log_analyzer')
from sysid.loader import load_aligned
import rate_sysid as rs

label, zp = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(zp) as z:
    g = json.loads(z.read('gains.json'))
df = load_aligned(zp)
t = df['timestamp_us'].to_numpy() * 1e-6


def window_std(ref_col, gyro_col):
    ref = df[ref_col].to_numpy()
    ok = np.isfinite(ref)
    # excitation window: rows where |rate_ref| exceeds 5 % of its max (chirp active)
    t0, t1 = (18.0, 22.0) if ref_col == 'rate_ref_roll' else (24.0, 28.0)   # nominal chirp windows from sysid_gate.scn
    sel = (t >= t0) & (t <= t1)
    gy = df[gyro_col].to_numpy()[sel]
    tr = df[{'gyro_x':'rate_x','gyro_y':'rate_y'}[gyro_col]].to_numpy()[sel]
    return t0, t1, float(np.degrees(np.nanstd(gy))), float(np.degrees(np.nanstd(tr)))


for ax, rc, gc in (('roll', 'rate_ref_roll', 'gyro_x'), ('pitch', 'rate_ref_pitch', 'gyro_y')):
    t0, t1, sd, rmax = window_std(rc, gc)
    print('%s %s excitation window %.2f-%.2f s: gyro std %.2f deg/s, truth rate std %.2f deg/s' % (label, ax, t0, t1, sd, rmax))
for mode in ('duty', 'kp'):
    for ax in ('roll', 'pitch', 'yaw'):
        try:
            f = rs.fit_from_df(df, ax, gains=g[ax], input_mode=mode)
            print('%s %-5s %-4s b=%.0f T_ms=%.2f L_ms=%.2f T+L_ms=%.2f coh=%.3f' % (
                label, ax, mode, f['b'], f['T'] * 1e3, f['L'] * 1e3, (f['T'] + f['L']) * 1e3, f['coherence_mean']))
        except Exception as e:
            print('%s %-5s %-4s ERROR %s' % (label, ax, mode, repr(e)[:150]))
