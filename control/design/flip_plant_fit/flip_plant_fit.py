#!/usr/bin/env python3
"""Flip plant fit: compare a real flip log with SILS runs and re-derive the fit.
宙返りのプラントフィット: 実機の宙返りログと SILS 実行を比較し、フィットを再導出する。

Evidence for docs/architecture/simulation-policy.md §5 ("2026-10-02 追記") and
docs/plans/flip-maneuver-plan.md §5.6. Analysis helper only (not a user tool).
simulation-policy.md §5 と flip-maneuver-plan.md §5.6 の根拠。解析補助のみ（利用者向けツールではない）。

Inputs / 入力 (each a .sflog.zip or an extracted directory):
  --hw    real flip log (e.g. logs/flip_test1.sflog.zip)
  --sils  LABEL=PATH  one or more SILS runs of simulator/sils/scenarios/api_flip_roll.scn
          (run with `--param flip.enable=1`; first flip = roll right)
          例: --sils before=out_before.sflog.zip --sils after=out_after.sflog.zip
Outputs / 出力 (--out DIR): flip_fit_overlay.png, flip_fit_openloop.png, flip_fit_metrics.txt

Method / 手法:
  1. Metrics: spin-up/peak/abort/brake/torque-saturation/duty-pinning per run
     (time origin = rate_ref leaving 0, back-extrapolated from the 300 rad/s^2 ramp).
  2. Regression: alpha_meas = b * tau_cmd(t-L) through a first-order lag T (least squares).
  3. Open-loop forward fit on the real log: logged duties -> motor ODE (SSOT constants)
     -> roll acceleration, unknowns (s, L, kJ): torque scale, extra delay, rotor-inertia scale.
"""
import argparse
import glob
import os
import shutil
import tempfile
import zipfile

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

# Physical constants copied from control/models/stampfly_physical.yaml (checked by `sf params check`
# through generated_params; kept here only for the stand-alone forward model).
# control/models/stampfly_physical.yaml の値（スタンドアロンの順方向モデル用の写し）。
CT_EFF = 0.7133e-8      # plant thrust_efficiency * Ct
CQ, JMP, DM, QF, RM, KM = 4.10e-11, 1.375e-8, 0.0, 9.507e-6, 0.593, 5.682e-4
ARM, IXX = 0.023, 9.16e-6
V_BATT_HW = 3.90        # mean logged voltage during the flip (1 Hz status, unloaded 4.03 V after crash)
DT = 0.0025
RAMP_DPS2 = 300.0 * 180.0 / np.pi  # flip.rate_ramp_rps2 = 300 rad/s^2


class Run:
    """One flight log (CSV streams of a StampFly flight-log bundle). / フライトログ一式の CSV。"""

    def __init__(self, path, shift_t0):
        d = path
        if path.endswith(".zip"):
            d = tempfile.mkdtemp()
            zipfile.ZipFile(path).extractall(d)
        rd = lambda n: np.genfromtxt(os.path.join(d, n + ".csv"), delimiter=",", names=True)
        self.imu, self.rr, self.co, self.mo, self.pv = (rd(n) for n in ("imu", "rate_ref", "ctrl_output", "motor", "posvel"))
        self.t0 = self.imu["timestamp_us"][0] / 1e6 if shift_t0 else 0.0
        sec = lambda x: x["timestamp_us"] / 1e6 - self.t0
        self.t, self.tr, self.tc, self.tm = sec(self.imu), sec(self.rr), sec(self.co), sec(self.mo)
        self.g = np.degrees(np.c_[self.imu["gyro_x"], self.imu["gyro_y"], self.imu["gyro_z"]])
        self.duty = np.c_[self.mo["duty_FR"], self.mo["duty_RR"], self.mo["duty_RL"], self.mo["duty_FL"]]

    def spin_start(self, after):
        r = np.degrees(self.rr["rate_ref_roll"])
        i = np.where((r > 300) & (self.tr > after))[0][0]
        return self.tr[i] - r[i] / RAMP_DPS2

    def metrics(self, ts):
        g, t = self.g[:, 0], self.t
        phi = np.cumsum(g * np.diff(t, prepend=t[0] - DT))
        phi -= phi[np.searchsorted(t, ts) - 1]
        m = {}
        w = (t > ts) & (t < ts + 0.6)
        k = int(np.argmax(g * w))
        m["peak_dps"] = g[k]
        for lv in (1000, 1500, 1800):
            j = np.where((g > lv) & (t > ts))[0]
            ok = len(j) and t[j[0]] < ts + 0.6
            m[f"t{lv}_ms"] = (t[j[0]] - ts) * 1e3 if ok else np.nan
            m[f"phi@{lv}"] = phi[j[0]] if ok else np.nan
        kz = k + int(np.where(g[k:] <= 0)[0][0])
        m["t_stop_ms"], m["phi_stop"] = (t[kz] - ts) * 1e3, phi[kz]
        sel = np.arange(k, kz)
        s2 = sel[(g[sel] < 0.8 * g[k]) & (g[sel] > 0.2 * g[k])]
        m["brake_decel_rad_s2"] = np.polyfit(t[s2], np.radians(g[s2]), 1)[0]
        a, b = np.searchsorted(t, ts + 0.0225), np.searchsorted(t, ts + 0.0925)
        m["spinup_rad_s2"] = np.polyfit(t[a:b], np.radians(g[a:b]), 1)[0]
        tq = self.co["torque_roll"]
        sat, iv, i = np.abs(tq) >= 6.99e-3, [], 0
        while i < len(sat):
            if sat[i] and ts - 0.05 < self.tc[i] < ts + 0.6:
                j = i
                while j < len(sat) and sat[j]:
                    j += 1
                iv.append((round(float(self.tc[i] - ts), 4), round(float(self.tc[j - 1] - ts), 4), int(np.sign(tq[i]))))
                i = j
            else:
                i += 1
        m["torque_sat_s"] = iv
        w2 = (self.tm > ts) & (self.tm < ts + 0.25)
        d = self.duty[w2]
        m["duty_any_pinned"] = float(np.mean(np.any((d <= 0.001) | (d >= 0.999), axis=1)))
        m["phi"] = phi
        return m

    def regress_b(self, ts):
        """alpha = b * lag(tau_cmd(t-L), T) over [ts, ts+0.4]. / ロール軸の b, L, T 回帰。"""
        ga = savgol_filter(np.radians(self.g[:, 0]), 9, 2, deriv=1, delta=DT)
        win = (self.t > ts) & (self.t < ts + 0.4)

        def model(x):
            b, L, T = x
            u = np.interp(self.t - L, self.tc, self.co["torque_roll"])
            y = np.zeros_like(u)
            a = DT / (T + DT)
            for k in range(1, len(u)):
                y[k] = y[k - 1] + a * (u[k] - y[k - 1])
            return b * y

        r = least_squares(lambda x: (model(x) - ga)[win], [1e5, 0.02, 0.01],
                          bounds=([1e3, 0, 1e-4], [5e5, 0.08, 0.05]), x_scale=[1e4, 0.01, 0.01])
        cov = np.linalg.pinv(r.jac.T @ r.jac) * np.sum(r.fun ** 2) / (len(r.fun) - 3)
        return r.x, np.sqrt(np.diag(cov))


def omega_eq(v):
    a, b, c = CQ, DM + KM ** 2 / RM, QF - KM * v / RM
    return (-b + np.sqrt(b * b - 4 * a * c)) / (2 * a)


def motors(duty, kj, vb=V_BATT_HW, sub=10):
    """RK4 of the plant's motor ODE for logged duties (ZOH per sample). / ログ duty でモータ ODE を RK4 積分。"""
    h, jm = DT / sub, JMP * kj
    f = lambda w, v: (-(DM + KM ** 2 / RM) * w - CQ * w * w - QF + KM * v / RM) / jm
    w = omega_eq(duty[0] * vb) * np.ones(4)
    out = np.zeros_like(duty)
    for k in range(len(duty)):
        v = np.clip(duty[k] * vb, 0, vb)
        for _ in range(sub):
            k1 = f(w, v); k2 = f(w + .5 * h * k1, v); k3 = f(w + .5 * h * k2, v); k4 = f(w + h * k3, v)
            w = np.maximum(w + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4), 0)
        out[k] = w
    return out


def forward_fit(hw, a=15.60, b=16.30, fa=15.85, fb=16.25):
    """Open-loop forward fit (s, L, kJ) of the real log. / 実機ログの開ループ順方向フィット。"""
    sel = (hw.tm > a) & (hw.tm < b)
    tm, duty = hw.tm[sel], hw.duty[sel]
    gm = np.interp(tm, hw.t, np.radians(hw.g[:, 0]))
    sr = np.array([-1, -1, 1, 1.0])
    tw = (tm > fa) & (tm < fb)

    def predict(x):
        s, L, kj = x
        d = np.c_[[np.interp(tm - L, tm, duty[:, i]) for i in range(4)]].T
        w = motors(d, kj)
        al = s * ARM * ((CT_EFF * w * w) @ sr) / IXX
        p = np.zeros_like(al)
        p[0] = gm[0]
        for k in range(1, len(al)):
            p[k] = p[k - 1] + .5 * (al[k] + al[k - 1]) * DT
        return p

    r = least_squares(lambda x: (predict(x) - gm)[tw], [0.5, 0.02, 0.5], bounds=([0.1, 0, 0.3], [3, 0.06, 5]),
                      x_scale=[0.5, 0.02, 0.5])
    cov = np.linalg.inv(r.jac.T @ r.jac) * np.sum(r.fun ** 2) / (len(r.fun) - 3)
    rms = np.degrees(np.sqrt(np.mean(r.fun ** 2)))
    return r.x, np.sqrt(np.diag(cov)), rms, tm, gm, predict


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hw", required=True)
    ap.add_argument("--sils", action="append", default=[], metavar="LABEL=PATH")
    ap.add_argument("--out", required=True)
    ap.add_argument("--hw-after", type=float, default=15.0, help="search start for the spin [s since log start]")
    ap.add_argument("--sils-after", type=float, default=22.9, help="same for SILS (flip r at t=23 s)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runs = [("HW flip_test1", Run(args.hw, True), args.hw_after, "k")]
    for i, spec in enumerate(args.sils):
        lab, path = spec.split("=", 1)
        runs.append((f"SILS {lab}", Run(path, False), args.sils_after, f"C{i * 3}"))

    lines = []
    fig, ax = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    for lab, run, after, col in runs:
        ts = run.spin_start(after)
        m = run.metrics(ts)
        x, se = run.regress_b(ts)
        lines.append(f"[{lab}] spin start t={ts:.4f} s")
        for k, v in m.items():
            if k != "phi":
                lines.append(f"  {k}: {v if not isinstance(v, float) else round(v, 3)}")
        lines.append(f"  regression b={x[0]:.0f}+-{se[0]:.0f} rad/s2/Nm  L={x[1] * 1e3:.1f} ms  T={x[2] * 1e3:.1f} ms")
        tt = run.t - ts
        ax[0].plot(tt * 1e3, run.g[:, 0], color=col, label=lab)
        ax[0].plot((run.tr - ts) * 1e3, np.degrees(run.rr["rate_ref_roll"]), color=col, ls=":", lw=1)
        ax[1].plot(tt * 1e3, m["phi"], color=col, label=lab)
        ax[2].plot((run.tc - ts) * 1e3, run.co["torque_roll"] * 1e3, color=col, label=lab)
        d = run.duty
        ax[3].plot((run.tm - ts) * 1e3, (d[:, 2] + d[:, 3] - d[:, 0] - d[:, 1]) / 2, color=col, lw=.8, label=lab)
    for a_, yl in zip(ax, ("roll rate [dps] (dotted: rate_ref)", "phi [deg] (gyro integral)",
                           "torque_roll cmd [mNm]", "duty (left - right)/2")):
        a_.set_ylabel(yl); a_.grid(alpha=.3); a_.legend(fontsize=8)
    ax[0].axhline(1800, color="gray", lw=.5); ax[1].axhline(360, color="gray", lw=.5)
    ax[3].set_xlabel("time since spin start [ms]"); ax[0].set_xlim(-50, 600)
    ax[0].set_title("Roll flip: hardware vs SILS plant (unfixed flip logic)")
    plt.tight_layout(); plt.savefig(os.path.join(args.out, "flip_fit_overlay.png"), dpi=90); plt.close()

    hw = runs[0][1]
    x, se, rms, tm, gm, predict = forward_fit(hw)
    lines.append(f"[open-loop forward fit, real log] s={x[0]:.3f}+-{se[0]:.3f} L={x[1] * 1e3:.1f}+-{se[1] * 1e3:.1f} ms "
                 f"kJ={x[2]:.2f}+-{se[2]:.2f} residual rms {rms:.0f} dps (window 15.85-16.25 s)")
    plt.figure(figsize=(10, 4))
    plt.plot(tm, np.degrees(gm), "k", label="gyro HW")
    plt.plot(tm, np.degrees(predict([1, 0, 1])), label="duty->ODE nominal (s=1, L=0)")
    plt.plot(tm, np.degrees(predict(x)), label="fit")
    plt.xlim(15.8, 16.3); plt.legend(); plt.grid(alpha=.3); plt.ylabel("roll rate [dps]"); plt.xlabel("t [s]")
    plt.tight_layout(); plt.savefig(os.path.join(args.out, "flip_fit_openloop.png"), dpi=90); plt.close()

    with open(os.path.join(args.out, "flip_fit_metrics.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
