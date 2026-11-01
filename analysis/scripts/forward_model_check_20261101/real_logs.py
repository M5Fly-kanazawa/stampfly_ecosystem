#!/usr/bin/env python3
"""
Part C: apply tools/sysid/forward_model.py to the real flip / hover logs.
C 部: forward_model.py を実機の宙返り・ホバーのログに適用する。

Usage: python real_logs.py {flip|hover|reuse|all}     (SF_LOGS = directory with the logs)
Needs PYTHONPATH=<repo>:<repo>/lib. The old D*V model comes from flipcheck.py (read only).
旧 D×V モデルは flipcheck.py (読み取りのみ) のものを使う。
"""
import importlib.util
import os
import sys
import numpy as np
from scipy import signal
from scipy.signal import savgol_filter
from tools.sysid import forward_model as fm
from tools.sysid import _generated_params as gp

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
LOGS = os.environ.get("SF_LOGS") or os.path.join(REPO, "logs")
FLIPCHECK = os.path.join(REPO, "analysis", "scripts", "ct_cq_bench_20261015", "crosscheck", "flipcheck", "flipcheck.py")
FLIP_WINDOWS = {
    "flip_test1": {"roll flip": (15.85, 16.25)},
    "flip_test2": {"roll flip": (6.66, 7.10), "pitch after flip": (6.95, 7.45)},
}
HOVER_WINDOWS = {"flip_test1": (5.5, 15.5), "flip_test2": (4.4, 6.4)}
BANDS_HZ = [(0.5, 2), (2, 5), (5, 10), (10, 20), (20, 40)]
DELAYS_S = np.arange(0, 41) * 1e-3
NPERSEG = 256
SG_WINDOW, SG_ORDER = 9, 2          # same smoother as flipcheck.py
INERTIA = np.array([gp.EXPECTED_IXX, gp.EXPECTED_IYY, gp.EXPECTED_IZZ])
AXES = ("roll", "pitch", "yaw")


def path(name):
    return os.path.join(LOGS, f"{name}.sflog.zip")


def old_model_torque(motion):
    """Old D*V motor model through flipcheck.py's simulate(); body torque [roll,pitch,yaw=0].
    旧モデル (flipcheck.py の simulate) による機体トルク。ヨーは未計算 (0)。"""
    spec = importlib.util.spec_from_file_location("flipcheck", FLIPCHECK)
    os.environ["SF_REPO"] = REPO
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    volt = motion.voltage if motion.voltage is not None else np.full(len(motion.t_s), 3.9)
    before, after = mod.simulate("now", np.clip(motion.duty, 0, 1), volt)
    omega = 0.5 * (before + after)
    thrust = 0.7133 * 1.00e-8 * omega ** 2           # MODELS["M_now"]: eta*Ct
    tau = np.zeros((len(motion.t_s), 3))
    tau[:, 0] = mod.ARM * (thrust @ mod.SR)
    tau[:, 1] = mod.ARM * (thrust @ mod.SP)
    return tau


def new_model_alpha(motion, delay_s=0.0, pwm_bits=fm.PWM_RESOLUTION_BITS, voltage_mode="log"):
    trace, torque = fm.predict_torque_trace(motion, voltage_mode, pwm_bits=pwm_bits)
    return fm.accel_from_trace(motion, trace, torque, delay_s)


def new_model_fn(motion, **kwargs):
    """Simulate the motors once; return delay -> alpha (cheap to scan). / 1 回だけ積分し遅れ→α を返す。"""
    trace, torque = fm.predict_torque_trace(motion, kwargs.get("voltage_mode", "log"),
                                            pwm_bits=kwargs.get("pwm_bits", fm.PWM_RESOLUTION_BITS))
    return lambda d: fm.accel_from_trace(motion, trace, torque, d)


def shifted(series, delay_s, dt):
    """Delay a row series by `delay_s` (linear interpolation).  / 行系列を線形補間で遅らせる。"""
    t = np.arange(len(series)) * dt
    return np.stack([np.interp(t - delay_s, t, series[:, a]) for a in range(series.shape[1])], axis=1)


def flip_fit(meas, pred_fn, win, axis):
    """flipcheck-style fit: smoothed prediction vs smoothed-derivative measurement.
    Returns no-fit residual (delay 0, scale 1) and the best (delay, scale) fit."""
    rows = []
    for d in DELAYS_S:
        p = savgol_filter(pred_fn(d)[:, axis], SG_WINDOW, SG_ORDER)[win]
        m = meas[win]
        s = float(p @ m / (p @ p))
        rows.append((d, s, np.sqrt(np.mean((m - s * p) ** 2)), np.sqrt(np.mean((m - p) ** 2))))
    rows = np.array(rows)
    rm = float(np.sqrt(np.mean(meas[win] ** 2)))
    ib = int(np.argmin(rows[:, 2]))
    i1 = int(np.argmin(rows[:, 3]))
    return dict(rms=rm, nofit=rows[0, 3] / rm, best_delay_ms=rows[ib, 0] * 1e3, best_scale=rows[ib, 1],
                best_res=rows[ib, 2] / rm, s1_delay_ms=rows[i1, 0] * 1e3, s1_res=rows[i1, 3] / rm)


def flips(treatment=fm.REUSE_TIMESTAMP):
    print(f"# FLIP windows (reuse treatment: {treatment})")
    print("log | window | model | no-fit res | best delay ms | best scale | res at best | res (s=1, free delay, ms)")
    for name, wins in FLIP_WINDOWS.items():
        motion = fm.load_real_log(path(name), treatment)
        t = motion.t_s - motion.t_s[0]
        omega_meas = {a: savgol_filter(motion.gyro[:, a], SG_WINDOW, SG_ORDER, deriv=1, delta=motion.dt) for a in range(3)}
        old_tau = old_model_torque(motion)
        models = {
            "R_out (module, 8-bit PWM, logged V)": new_model_fn(motion),
            "R_out (module, no PWM quantisation)": new_model_fn(motion, pwm_bits=None),
            "R_out (module, V = 3.7 V const)": new_model_fn(motion, voltage_mode=3.7),
            "R_out (module, plant battery model)": new_model_fn(motion, voltage_mode="battery"),
            "old D*V (flipcheck)": lambda d: shifted(old_tau / INERTIA, d, motion.dt),
        }
        for wname, (a, b) in wins.items():
            axis = 0 if wname.startswith("roll") else 1
            win = (t >= a) & (t <= b)
            for mname, fn in models.items():
                r = flip_fit(omega_meas[axis], fn, win, axis)
                print(f"{name} | {wname} {a}-{b}s ({AXES[axis]}) | {mname} | {r['nofit']:.3f} | {r['best_delay_ms']:.0f} | "
                      f"{r['best_scale']:.3f} | {r['best_res']:.3f} | {r['s1_res']:.3f} @ {r['s1_delay_ms']:.0f}")


def band_stats(pred, meas, fs):
    f, Spp = signal.welch(pred, fs, "hann", NPERSEG, axis=0)
    _, Smm = signal.welch(meas, fs, "hann", NPERSEG, axis=0)
    _, Spm = signal.csd(pred, meas, fs, "hann", NPERSEG, axis=0)
    out = []
    for lo, hi in BANDS_HZ:
        b = (f >= lo) & (f < hi)
        res = []
        for a in range(pred.shape[1]):
            spp, smm, spm = Spp[b, a].sum(), Smm[b, a].sum(), Spm[b, a].sum()
            fc = float((f[b] * Spp[b, a]).sum() / spp)
            res.append(dict(Hfwd=abs(spm) / spp, Hrev=smm / abs(spm), coh=abs(spm) ** 2 / (spp * smm),
                            phase_deg=np.degrees(np.angle(spm)), delay_ms=-np.angle(spm) / (2 * np.pi * fc) * 1e3,
                            fc=fc))
        out.append(res)
    return out


def hover(treatment=fm.REUSE_TIMESTAMP, quiet=False, models=("R_out", "old")):
    results = {}
    for name, (a, b) in HOVER_WINDOWS.items():
        motion = fm.load_real_log(path(name), treatment)
        t = motion.t_s - motion.t_s[0]
        win = (t >= a) & (t < b)
        win[-1] = False
        meas = fm.accel_from_rate(motion.t_s, motion.gyro, motion.dt)
        for mname in models:
            pred = new_model_alpha(motion) if mname == "R_out" else old_model_torque(motion) / INERTIA
            stats = band_stats(pred[win], meas[win], 1.0 / motion.dt)
            results[(name, mname)] = stats
            if quiet:
                continue
            print(f"## {name} hover {a}-{b}s model={mname} treatment={treatment} rows={int(win.sum())}")
            for (lo, hi), per_axis in zip(BANDS_HZ, stats):
                cells = " | ".join(f"{AXES[i][0]}: |H|={r['Hfwd']:.2f} rev={r['Hrev']:.2f} coh={r['coh']:.2f} "
                                   f"ph={r['phase_deg']:.0f} d={r['delay_ms']:.1f}ms" for i, r in enumerate(per_axis[:2]))
                print(f"  {lo}-{hi} Hz | {cells}")
    return results


def reuse():
    print("# 5-20 Hz dependence on the reused-IMU-row treatment (R_out model, roll/pitch)")
    for tr in fm.REUSE_TREATMENTS:
        hover(tr, models=("R_out",))


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("flip", "all"):
        for tr in fm.REUSE_TREATMENTS:
            flips(tr)
    if what in ("hover", "all"):
        hover()
    if what in ("reuse", "all"):
        reuse()
