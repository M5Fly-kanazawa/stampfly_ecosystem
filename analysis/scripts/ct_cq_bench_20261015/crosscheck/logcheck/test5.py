# Would a bench-based thrust-to-duty conversion make the commanded hover thrust flat in battery voltage?
# ベンチに基づく推力→Duty の換算なら、ホバーの指令推力は電池電圧によらず平らになるか。
#
# In steady hover the duties and the voltage are set by physics. The commanded thrust a conversion would need to
# produce them is that conversion's forward model evaluated at the logged duties; for the current firmware it is
# the logged command itself (ctrl_ref total_thrust). A good conversion gives a ratio to the weight that is flat in
# voltage, both within one flight and across flights.
# 定常ホバーの Duty と電圧は物理で決まる。ある換算がそれを出すのに要る指令推力は、その換算の順モデルを記録された
# Duty で評価した値で、今のファームでは記録された指令値（ctrl_ref の total_thrust）そのもの。よい換算なら、重さとの
# 比が 1 回の飛行の中でも、飛行をまたいでも電圧によらず平らになる。
#
# Conversions / 換算: current (logged command), A (four-motor M1 map, logged voltage), B (one-motor map, V_DD =
# logged voltage - four-motor CLI-to-V_DD offset), B_on (one-motor map on the on-time V_DD).
# Inputs as test4.py (SF_BENCH). Output: results/test5.json, results/test5_windows.csv
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, pandas as pd
from data import get, JSONL, BUNDLE_FLIP, REPO, OUT
from hover import hover_mask, segments, MINWIN
BENCH = os.environ.get('SF_BENCH') or os.path.join(os.path.dirname(REPO), 'stampfly-motor-autotest', 'data')
G = 9.80665; M = 0.037      # standard gravity, nominal mass [kg] / 標準重力、標準質量
s = pd.DataFrame(json.load(open(os.path.join(BENCH, '2026-10-15', 'summary.json'))))
n, lk = np.polyfit(np.log(s.omega), np.log(s.thrust_g * G * 1e-3), 1)
F = lambda w: np.exp(lk) * w ** n
st = pd.read_csv(os.path.join(BENCH, '2026-11-02_steps.csv')); b2, b3 = st[st.battery == 'bat2'], st[st.battery == 'bat3']
def fit(D, V, y):
    X = np.c_[np.ones_like(D), D, V, D * V, D ** 2]; b, *_ = np.linalg.lstsq(X, y, rcond=None)
    return lambda d, v: np.c_[np.ones_like(d), d, v, d * v, d ** 2] @ b
mA = fit(b3.duty.values, b3.CLI_V.values, b3.omega_rad_s.values)
mB = fit(b2.duty.values, b2.VDD_V.values, b2.omega_rad_s.values)
mBon = fit(b2.duty.values, b2.VDD_on_V.values, b2.omega_rad_s.values)              # one motor, on-time V_DD
off = fit(b3.duty.values, b3.CLI_V.values, (b3.CLI_V - b3.VDD_V).values)            # CLI - mean V_DD, four motors
offon = fit(b3.duty.values, b3.CLI_V.values, (b3.CLI_V - b3.VDD_on_V).values)        # CLI - on-time V_DD, four motors
models = {'A': lambda d, v: F(mA(d, v)), 'B': lambda d, v: F(mB(d, v - off(d, v))),
          'B_on': lambda d, v: F(mBon(d, v - offon(d, v)))}
rows = []
for name in JSONL + BUNDLE_FLIP:
    R = get(name); t = R['c_t']
    ok, *_ = hover_mask(R, flip_exclude=name in BUNDLE_FLIP)
    for a, b in segments(t, ok):
        if b - a < MINWIN: continue
        k = max(1, int(np.floor((b - a) / 5.0))); e = np.linspace(a, b, k + 1)
        for u, v in zip(e[:-1], e[1:]):
            if v - u < MINWIN: continue
            m = (t >= u) & (t <= v); D = R['c_duty'][m].mean(0); V = float(np.mean(np.interp(t[m], R['s_t'], R['s_v']))); Vv = np.full(4, V)
            r = dict(log=name, dur=v - u, V=V, D=D.mean(), current=float(R['c_thr'][m].mean()) / (M * G))
            for kname, f in models.items(): r[kname] = float(f(D, Vv).sum()) / (M * G)
            rows.append(r)
d = pd.DataFrame(rows); d.to_csv(os.path.join(OUT, 'test5_windows.csv'), index=False, float_format='%.5g')
cols = ['current', 'A', 'B', 'B_on']
print('required commanded thrust / weight (median) and slope against voltage within each long flight [per V]')
print('  log               V range     ' + '  '.join(f'{c:>14s}' for c in cols) + '   duty slope')
for lg, g in d.groupby('log'):
    if g.V.max() - g.V.min() < 0.15 or len(g) < 4: continue
    sl = {c: np.polyfit(g.V, g[c], 1)[0] for c in cols}
    print(f'  {lg} {g.V.min():.2f}-{g.V.max():.2f} ' + '  '.join(f'{g[c].median():.3f} {sl[c]:+.3f}' for c in cols)
          + f'   {np.polyfit(g.V, g.D, 1)[0]:+.3f}')
# within-flight pooled slope (each flight's mean removed), weighted by time / 飛行ごとの平均を引いて合わせた傾き
x = d.groupby('log').V.transform(lambda z: z - z.mean())
print('\npooled within-flight slope [per V] (14 logs, each flight mean removed):')
for c in cols + ['D']:
    y = d.groupby('log')[c].transform(lambda z: z - z.mean())
    b = np.sum(d.dur * x * y) / np.sum(d.dur * x * x)
    r = y - b * x; se = np.sqrt(np.sum(d.dur * r * r) / np.sum(d.dur) / (len(d) - 15) * len(d) / np.sum(d.dur * x * x) * np.sum(d.dur) / len(d))
    print(f'  {c:8s} {b:+.3f} +- {se:.3f}')
print(f'  firmware D x V constant would give duty slope {-np.average(d.D, weights=d.dur) / np.average(d.V, weights=d.dur):+.3f}')
print('\nacross all windows (slope [per V]): ' + ', '.join(f'{c} {np.polyfit(d.V, d[c], 1, w=np.sqrt(d.dur))[0]:+.3f}' for c in cols))
print('spread of per-log medians: ' + ', '.join(f'{c} {d.groupby("log")[c].median().min():.3f}-{d.groupby("log")[c].median().max():.3f}' for c in cols))

# summary for the note / ノート用のまとめ
xw = d.groupby('log').V.transform(lambda z: z - z.mean())
res = dict(n_windows=len(d), within_flight_slope_per_V={}, across_slope_per_V={}, per_log_median_range={})
for c in cols + ['D']:
    y = d.groupby('log')[c].transform(lambda z: z - z.mean())
    res['within_flight_slope_per_V'][c] = float(np.sum(d.dur * xw * y) / np.sum(d.dur * xw * xw))
for c in cols:
    res['across_slope_per_V'][c] = float(np.polyfit(d.V, d[c], 1, w=np.sqrt(d.dur))[0])
    med = d.groupby('log')[c].median(); res['per_log_median_range'][c] = [float(med.min()), float(med.max())]
json.dump(res, open(os.path.join(OUT, 'test5.json'), 'w'), indent=1)
