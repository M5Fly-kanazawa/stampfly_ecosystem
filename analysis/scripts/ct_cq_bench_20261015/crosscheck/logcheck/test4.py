# Hover thrust balance: is the vehicle thrust four times the 2026-10-15 single-motor thrust curve?
# ホバーの推力の釣り合い: 全機の推力は 2026-10-15 の 1 モータの推力の曲線の 4 倍か。
#
# For every steady-hover window (5 s, hover.py) the four logged duties and the logged battery voltage give each
# rotor speed through a map omega(D, V) measured on the bench on 2026-11-02, and the 10-15 power law F = k*omega^n
# turns it into thrust. The ratio sum(F)/(m*g) should be near 1 and flat in voltage if the claim holds; a loss of
# thrust in flight (rotor-rotor or airframe interference) would push it clearly above 1.
# 各ホバー区間で、記録された 4 つの Duty と電池電圧から、2026-11-02 のベンチで測った ω(D, V) の表で回転数を出し、
# 10/15 のべき乗の式で推力にする。主張が正しければ ΣF/(m·g) は 1 に近く、電圧によらない。飛行中に干渉で推力を
# 失っていれば、はっきり 1 を超える。
#
# Maps / 表:  A = four motors at once (M1, against the logged-voltage point CLI_V)
#                 4 モータ同時の M1（記録と同じ位置の電圧 CLI_V で引く）
#             B = one motor (against V_DD at the driver; V_DD = V_log - (CLI_V - V_DD) from the four-motor runs)
#                 1 モータ（駆動回路の V_DD で引く。V_DD = 記録電圧 - 4 モータ時の CLI_V - V_DD）
# An omega-free route regresses the measured one-motor thrust of 11-02 on (D, V_DD) directly.
# ω を使わない別の道として、11/02 の 1 モータの推力を (D, V_DD) で直接回帰したものも当てる。
#
# Inputs / 入力: the flight logs (common.LOGS_DIR) and the bench repository's data folder (env SF_BENCH, default
# <repo>/../stampfly-motor-autotest/data): 2026-10-15/summary.json and 2026-11-02_steps.csv.
# Output / 出力: results/test4.json, results/test4_windows.csv
import sys, os, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, pandas as pd
from scipy.optimize import brentq
from data import get, JSONL, BUNDLE_FLIP, REPO, OUT
from hover import hover_mask, segments, MINWIN

G = 9.80665                 # standard gravity [m/s^2] / 標準重力加速度
CHUNK = 5.0                 # window length [s], as in test1 / 区間の長さ（test1 と同じ）
MASSES = (0.037, 0.036)     # vehicle mass range [kg] estimated by the developer (36-37 g) / 機体質量の範囲（開発者の見積もり 36〜37 g）
BENCH = os.environ.get('SF_BENCH') or os.path.join(os.path.dirname(REPO), 'stampfly-motor-autotest', 'data')


def fitmap(D, V, y):
    # y = b0 + b1 D + b2 V + b3 D V + b4 D^2 over the bench steps (D 0.5-0.8) / ベンチの段で当てはめる
    X = np.c_[np.ones_like(D), D, V, D * V, D ** 2]
    b, *_ = np.linalg.lstsq(X, y, rcond=None)
    return (lambda d, v: np.c_[np.ones_like(d), d, v, d * v, d ** 2] @ b), float(np.sqrt(np.mean((y - X @ b) ** 2)))


# 10-15 thrust curve F = k omega^n [N] / 10/15 の推力の曲線
s = pd.DataFrame(json.load(open(os.path.join(BENCH, '2026-10-15', 'summary.json'))))
N_PL, LK = np.polyfit(np.log(s.omega), np.log(s.thrust_g * G * 1e-3), 1)
def thrust(w): return np.exp(LK) * np.asarray(w) ** N_PL

st = pd.read_csv(os.path.join(BENCH, '2026-11-02_steps.csv'))
b2, b3 = st[st.battery == 'bat2'], st[st.battery == 'bat3']
mapA, rmsA = fitmap(b3.duty.values, b3.CLI_V.values, b3.omega_rad_s.values)
mapB, rmsB = fitmap(b2.duty.values, b2.VDD_V.values, b2.omega_rad_s.values)
offs, rmsO = fitmap(b3.duty.values, b3.CLI_V.values, (b3.CLI_V - b3.VDD_V).values)
fmap, rmsF = fitmap(b2.duty.values, b2.VDD_V.values, b2.thrust_g.values * G * 1e-3)

rows = []
for name in JSONL + BUNDLE_FLIP:
    R = get(name); t = R['c_t']
    ok, fs, diag, tfl = hover_mask(R, flip_exclude=name in BUNDLE_FLIP)
    for a, b in segments(t, ok):
        if b - a < MINWIN:
            continue
        k = max(1, int(np.floor((b - a) / CHUNK))); e = np.linspace(a, b, k + 1)
        for u, v in zip(e[:-1], e[1:]):
            if v - u < MINWIN:
                continue
            # mean duty per motor and mean logged voltage over the window / 区間の平均 Duty（モータごと）と平均電圧
            m = (t >= u) & (t <= v); D = R['c_duty'][m].mean(0)
            V = float(np.mean(np.interp(t[m], R['s_t'], R['s_v']))); Vv = np.full(4, V)
            Vdd = Vv - offs(D, Vv)
            wA, wB = mapA(D, Vv), mapB(D, Vdd)
            rows.append(dict(log=name, t0_s=float(u - t[0]), dur_s=float(v - u), V=V, D_mean=float(D.mean()),
                             omega_A=float(wA.mean()), omega_B=float(wB.mean()),
                             F_A_N=float(thrust(wA).sum()), F_B_N=float(thrust(wB).sum()),
                             F_direct_1102_N=float(fmap(D, Vdd).sum())))
d = pd.DataFrame(rows)
for mass in MASSES:
    for col in ('A', 'B', 'direct_1102'):
        d[f'ratio_{col}_{mass}'] = d[f'F_{col}_N'] / (mass * G)

# Duty the bench says is needed to carry the weight at the logged voltage (same duty on all four motors)
# 記録電圧で重さを支えるのにベンチから要る Duty（4 モータ同じ Duty とする）
ROUTES = {
    'A': lambda dd, v: thrust(mapA(dd, v)),
    'B': lambda dd, v: thrust(mapB(dd, v - offs(dd, v))),
    'direct_1102': lambda dd, v: fmap(dd, v - offs(dd, v)),
}


def duty_needed(route, v, mass):
    f = lambda x: 4 * float(ROUTES[route](np.array([x]), np.array([v]))[0]) - mass * G
    return brentq(f, 0.3, 1.0)


for mass in MASSES:
    for col in ROUTES:
        d[f'D_need_{col}_{mass}'] = [duty_needed(col, v, mass) for v in d.V]
d.to_csv(os.path.join(OUT, 'test4_windows.csv'), index=False, float_format='%.5g')


def wmean(x): return float(np.average(x, weights=d.dur_s))


res = dict(power_law=dict(n=float(N_PL), F_at_3500_N=float(thrust(3500.0))),
           map_rms=dict(A_rad_s=rmsA, B_rad_s=rmsB, cli_minus_vdd_V=rmsO, direct_thrust_N=rmsF),
           n_windows=len(d), hover_s=float(d.dur_s.sum()), V_range=[float(d.V.min()), float(d.V.max())], summary={}, per_log={})
for mass in MASSES:
    for col in ('A', 'B', 'direct_1102'):
        c = d[f'ratio_{col}_{mass}']
        res['summary'][f'{col}_{mass}'] = dict(time_weighted_mean=wmean(c), median_of_log_medians=float(d.groupby('log')[c.name].median().median()))
for lg, g in d.groupby('log', sort=False):
    r = dict(n=len(g), hover_s=float(g.dur_s.sum()), V_median=float(g.V.median()), D_median=float(g.D_mean.median()),
             ratio_A=float(g['ratio_A_0.037'].median()), ratio_B=float(g['ratio_B_0.037'].median()))
    if g.V.max() - g.V.min() >= 0.15 and len(g) >= 4:
        # slope of the ratio against voltage within one flight (voltage drop and motor warming overlap)
        # 1 回の飛行の中での比の電圧への傾き（電圧の低下とモータの温まりが重なる）
        r['slope_A_per_V'] = float(np.polyfit(g.V, g['ratio_A_0.037'], 1)[0])
        r['slope_B_per_V'] = float(np.polyfit(g.V, g['ratio_B_0.037'], 1)[0])
    res['per_log'][lg] = r

# Hover duty by logged voltage (0.1 V bins): flight median against the bench duty for 36-37 g
# 記録電圧ごと（0.1 V 刻み）のホバー Duty: 飛行の中央値と、36〜37 g を支えるベンチの Duty
d['V_bin'] = (d.V / 0.1).round() * 0.1
res['hover_duty_by_voltage'] = [
    dict(V_bin=float(vb), n=len(g), n_logs=int(g.log.nunique()), flight=float(g.D_mean.median()),
         **{f'bench_{col}_{mass}': float(g[f'D_need_{col}_{mass}'].median()) for mass in MASSES for col in ROUTES})
    for vb, g in d.groupby('V_bin')]
res['hover_duty_diff'] = {f'{col}_{mass}': dict(
    time_weighted_mean=wmean(d.D_mean - d[f'D_need_{col}_{mass}']),
    relative_pct=100 * wmean((d.D_mean - d[f'D_need_{col}_{mass}']) / d[f'D_need_{col}_{mass}']))
    for mass in MASSES for col in ROUTES}
# slope of hover duty against logged voltage; Duty x V constant would give -D/V
# 記録電圧に対するホバー Duty の傾き。Duty × 電圧が一定なら -D/V
res['hover_duty_slope_per_V'] = dict(
    flight=float(np.polyfit(d.V, d.D_mean, 1, w=np.sqrt(d.dur_s))[0]),
    **{f'bench_{col}_{mass}': float(np.polyfit(d.V, d[f'D_need_{col}_{mass}'], 1)[0]) for mass in MASSES for col in ROUTES},
    duty_times_V_constant=float(-wmean(d.D_mean) / wmean(d.V)))
res['hover_duty_slope_within_flight'] = {
    lg: dict(V=[float(g.V.min()), float(g.V.max())], flight=float(np.polyfit(g.V, g.D_mean, 1)[0]),
             bench_B_0_037=float(np.polyfit(g.V, g['D_need_B_0.037'], 1)[0]))
    for lg, g in d.groupby('log') if g.V.max() - g.V.min() >= 0.3 and len(g) >= 8}
json.dump(res, open(os.path.join(OUT, 'test4.json'), 'w'), indent=1)
print(json.dumps(res['summary'], indent=1))
for lg, r in res['per_log'].items():
    print(lg, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
