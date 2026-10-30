#!/usr/bin/env python3
"""Flip / hover forward-model check of two motor models against flip_test1/2 logs.
Run: python3 -I flipcheck.py   (read-only on the repo; writes JSON to ../results)
Logs: flip_test1 / flip_test2 .sflog.zip in <repo>/logs (or $SF_LOGS) (not in the repository) / ログはリポジトリに含まれない
"""
import json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.environ.get("SF_REPO") or os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))  # repository root / リポジトリのルート
OUT = os.path.join(HERE, "..", "results")
ROUT_CSV = os.path.join(HERE, "..", "..", "rout", "rout_table_c01.csv")
sys.path[:0] = [REPO, REPO + "/lib"]
import numpy as np
from scipy.signal import savgol_filter
from tools.sysid.loader import load_aligned

DT = 0.0025
SUB = 5  # RK4 substeps per 2.5 ms -> 2 kHz
ARM, IXX, IYY, MASS, G = 0.023, 9.16e-6, 13.3e-6, 0.037, 9.80665
SR = np.array([-1, -1, 1, 1.0])   # roll torque sign for [FR,RR,RL,FL]
SP = np.array([1, -1, -1, 1.0])   # pitch torque sign

rt = np.genfromtxt(ROUT_CSV, delimiter=",", names=True)
RD, RR_ = rt["D"], rt["R_out_ohm"]

MODELS = {
    "M_now": dict(kind="now", Ct=1.00e-8, eta=0.7133),
    "M_rout_eta1.0": dict(kind="rout", Ct=7.60e-9, eta=1.0),
    "M_rout_eta0.8975": dict(kind="rout", Ct=7.60e-9, eta=0.8975),
}
NOW = dict(J=1.375e-8, Km=5.682e-4, Rm=0.593, Cq=4.10e-11, Qf=9.507e-6)
ROUT = dict(J=1.54e-8, Ke=5.905e-4, Kt=5.905e-4, Cq=3.38e-11, a0=0.0443, b0=2.915e-9)


def deriv(kind, w, D, V):
    if kind == "now":
        p = NOW
        return (p["Km"] * (D * V - p["Km"] * w) / p["Rm"] - p["Cq"] * w * w - p["Qf"]) / p["J"]
    p = ROUT
    R = np.interp(D, RD, RR_)
    Im = np.where(D < 0.01, 0.0, (V - p["Ke"] * w) / R)
    return (p["Kt"] * Im - p["Cq"] * w * w - p["Kt"] * (p["a0"] + p["b0"] * w * w)) / p["J"]


def rk4(kind, w, D, V, h):
    k1 = deriv(kind, w, D, V); k2 = deriv(kind, w + .5 * h * k1, D, V)
    k3 = deriv(kind, w + .5 * h * k2, D, V); k4 = deriv(kind, w + h * k3, D, V)
    return np.maximum(w + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4), 0)


def simulate(kind, D, V):
    """RK4 at 2 kHz, ZOH duty. Returns (w_before, w_after) per grid sample (state at start / end of the ZOH interval)."""
    N = len(D); h = DT / SUB
    w = np.zeros(4)
    D0, V0 = np.clip(D[0], 0, 1), V[0]
    for _ in range(int(1.0 / h)):   # 1 s pre-run at first sample = steady state
        w = rk4(kind, w, D0, V0, h)
    before = np.zeros((N, 4)); after = np.zeros((N, 4))
    for n in range(N):
        before[n] = w
        Dn = np.clip(D[n], 0, 1)
        for _ in range(SUB):
            w = rk4(kind, w, Dn, V[n], h)
        after[n] = w
    return before, after


def load(name):
    d = load_aligned(f"{os.environ.get('SF_LOGS') or REPO + '/logs'}/{name}.sflog.zip")
    ts = d.timestamp_us.values.astype(float)
    t0 = ts[0]; t = (ts - t0) / 1e6
    st = d.drop_duplicates("status_timestamp_us").dropna(subset=["status_timestamp_us"])
    tv = (st.status_timestamp_us.values - t0) / 1e6; vv = st.voltage.values
    keep = np.r_[np.diff(ts) != 0, True]   # last row of each identical-timestamp group (IMU re-use)
    dd = d[keep].reset_index(drop=True); tu = t[keep]
    grid = np.arange(tu[0], tu[-1], DT)
    cols = {c: np.interp(grid, tu, dd[c].values) for c in ("gyro_x", "gyro_y", "gyro_z", "accel_z", "vel_z")}
    idx = np.clip(np.searchsorted(tu, grid, side="right") - 1, 0, len(tu) - 1)
    duty = np.c_[[dd[f"duty_{m}"].values[idx] for m in ("FR", "RR", "RL", "FL")]].T
    V = np.interp(grid, tv, vv)
    return dict(t=grid, duty=duty, V=V, **cols), (tv, vv)


def alpha_meas(w): return savgol_filter(w, 9, 2, deriv=1, delta=DT)
def smooth(x): return savgol_filter(x, 9, 2)
def shift(x, t, delta): return np.interp(t - delta, t, x)


def fit_window(tau, am, t, inertia, win, deltas=np.arange(0, 41) * 1e-3):
    m = am[win]; rows = []
    for dl in deltas:
        p = smooth(shift(tau, t, dl) / inertia)[win]
        pp = float(p @ p); s = float(p @ m / pp)
        rows.append((dl, s, float(np.sqrt(np.mean((m - s * p) ** 2))), float(np.sqrt(np.mean((m - p) ** 2))), np.sqrt(pp)))
    rows = np.array(rows)
    ib = int(np.argmin(rows[:, 2])); i1 = int(np.argmin(rows[:, 3]))
    rm = float(np.sqrt(np.mean(m ** 2)))
    return dict(
        n=int(win.sum()), alpha_rms=rm,
        best=dict(delta_ms=rows[ib, 0] * 1e3, s=rows[ib, 1], rms=rows[ib, 2], rel=rows[ib, 2] / rm,
                  s_se_nominal=float(rows[ib, 2] / rows[ib, 4])),
        d0_sfree=dict(s=rows[0, 1], rms=rows[0, 2], rel=rows[0, 2] / rm),
        s1_dfree=dict(delta_ms=rows[i1, 0] * 1e3, rms=rows[i1, 3], rel=rows[i1, 3] / rm),
        s1_d0=dict(rms=rows[0, 3], rel=rows[0, 3] / rm),
        curve=[(float(a * 1e3), float(b), float(c)) for a, b, c, _, _ in rows])


def main():
    res = {}
    WIN = {
        "flip_test1": dict(roll_flip=(15.85, 16.25), roll_flip_core=(15.88, 16.10), hover=(5.5, 15.5)),
        "flip_test2": dict(roll_flip=(6.66, 7.10), roll_flip_core=(6.66, 6.90), pitch_after=(6.95, 7.45), hover=(4.4, 6.40)),
    }
    for name in ("flip_test1", "flip_test2"):
        L, (tv, vv) = load(name); t = L["t"]
        res[name] = dict(voltage_samples=[(round(float(a), 2), float(b)) for a, b in zip(tv, vv)])
        gx, gy = L["gyro_x"], L["gyro_y"]
        amx, amy = alpha_meas(gx), alpha_meas(gy)
        for mname, mp in MODELS.items():
            w, _ = simulate(mp["kind"], L["duty"], L["V"])
            T = mp["eta"] * mp["Ct"] * w * w
            tau_r = ARM * (T @ SR); tau_p = ARM * (T @ SP)
            r = dict(A={}, B={}, C={})
            for wn, (a, b) in WIN[name].items():
                win = (t >= a) & (t <= b)
                if wn.startswith("roll_flip"):
                    f = fit_window(tau_r, amx, t, IXX, win); f["win"] = (a, b)
                    f["max_gyro_dps"] = float(np.degrees(np.abs(gx[win]).max())); r["A"][wn] = f
                elif wn == "pitch_after":
                    f = fit_window(tau_p, amy, t, IYY, win); f["win"] = (a, b)
                    f["max_gyro_dps"] = float(np.degrees(np.abs(gy[win]).max())); r["A"][wn] = f
                else:
                    for ax, tau, am, I, g in (("roll", tau_r, amx, IXX, gx), ("pitch", tau_p, amy, IYY, gy)):
                        f = fit_window(tau, am, t, I, win); f["win"] = (a, b)
                        f["rate_std_dps"] = float(np.degrees(g[win].std()))
                        f["tau_pred_std_mNm"] = float(tau[win].std() * 1e3); r["B"][ax] = f
                    tot = T.sum(axis=1)[win] / MASS; meas = -L["accel_z"][win]
                    r["C"] = dict(win=(a, b), mean_pred_ms2=float(tot.mean()), mean_meas_ms2=float(meas.mean()),
                                  ratio_pred_over_meas=float(tot.mean() / meas.mean()), ratio_pred_over_g=float(tot.mean() / G),
                                  mean_vel_z=float(L["vel_z"][win].mean()), mean_duty=float(L["duty"][win].mean()),
                                  mean_V=float(L["V"][win].mean()))
            res[name][mname] = r
    # cross-check with prior script style (rate-level, M_now, V=3.90, post-step state, flip_test1)
    L, _ = load("flip_test1"); t = L["t"]
    _, w = simulate("now", L["duty"], np.full_like(t, 3.90))
    tau = ARM * ((0.7133e-8 * w * w) @ SR)
    sel = (t > 15.6) & (t < 16.3); tw = (t > 15.85) & (t < 16.25); i0 = np.where(sel)[0][0]; gm = L["gyro_x"]
    xs = []
    for dl in np.arange(0, 41) * 1e-3:
        al = shift(tau, t, dl) / IXX
        q = np.cumsum(np.r_[0, .5 * (al[1:] + al[:-1]) * DT]); q -= q[i0]
        mm = (gm - gm[i0])[tw]; qq = q[tw]
        s = float(qq @ mm / (qq @ qq))
        xs.append((dl * 1e3, s, np.degrees(np.sqrt(np.mean((mm - s * qq) ** 2)))))
    xs = np.array(xs); ib = int(np.argmin(xs[:, 2]))
    res["prior_crosscheck"] = dict(best_delta_ms=xs[ib, 0], s=xs[ib, 1], rms_dps=xs[ib, 2])
    json.dump(res, open(os.path.join(OUT, "flipcheck_results.json"), "w"), indent=1, default=float)
    print("done")

if __name__ == "__main__":
    main()
