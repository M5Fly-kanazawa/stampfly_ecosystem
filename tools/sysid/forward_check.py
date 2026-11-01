#!/usr/bin/env python3
"""
forward_check.py - judge a SILS plant propulsion model against real flight logs by the
forward-model check (simulation-policy.md section 5, backlog #0 / #21).
forward_check.py - SILS プラントの推進系モデルを、順モデル照合で実機ログに対して判定する
（simulation-policy.md 5章、バックログ #0・#21）。

Method / 方法:
  The logged 4-motor duty is run through the plant's own motor and torque equations
  (tools/sysid/forward_model.py) to predict body angular acceleration, which is compared
  with what the real vehicle did (gyro derivative).  Four metrics on the roll axis:
    1. flip window  : no-fit residual  rms(a_meas - a_pred) / rms(a_meas)
    2. flip window  : best extra delay (search, with the best scale at each delay)
    3. hover window : gain  |S_pm| / S_pp   in the 5-10 Hz band
    4. hover window : delay -angle(S_pm) / (2 pi f_c) in the same band
  Pitch is computed too and reported as reference (not judged).
  Windows, thresholds and treatments come from analysis/datasets/flightlog/forward_check.yaml;
  there are no numeric constants in this module.
  ログの 4 モータ duty をプラント自身のモータ・トルク式 (forward_model.py) に通して機体の
  角加速度を予測し、実機の角加速度 (ジャイロの微分) と比べる。ロール軸の 4 指標:
  宙返り窓の合わせ込みなし残差・最良の追加遅れ、ホバー窓 5-10 Hz のゲイン・遅れ。
  ピッチも計算して参考表示する (判定しない)。窓・閾値・扱いは forward_check.yaml から読み、
  本モジュールには数値定数を置かない。
"""

from __future__ import annotations

import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import yaml
from scipy import signal
from scipy.signal import savgol_filter

from . import forward_model as fm

AXES = ("roll", "pitch", "yaw")
STATUS_PASS, STATUS_FAIL, STATUS_REFERENCE = "PASS", "FAIL", "reference"

# A model factory takes the logged motion and returns a function delay_s -> alpha (N, 3)
# [rad/s^2, FRD].  Cheap to scan: the motors are integrated once per motion.
# モデル工場: ログ動作を受け取り、遅れ → 角加速度 (N,3) の関数を返す。モータの積分は 1 回だけ。
ModelFactory = Callable[[fm.LoggedMotion], Callable[[float], np.ndarray]]


@dataclass
class Metric:
    """One row of the result table. / 結果表の 1 行。"""

    log: str
    axis: str
    name: str
    value: float
    unit: str
    criterion: str
    status: str          # PASS / FAIL / reference
    detail: str = ""


def default_criteria_path(repo_root: Path) -> Path:
    return repo_root / "analysis" / "datasets" / "flightlog" / "forward_check.yaml"


def load_spec(path) -> dict:
    """Read the criteria / windows file. / 基準・窓ファイルを読む。"""
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


# --------------------------------------------------------------------------
# Model factories / モデル工場
# --------------------------------------------------------------------------
def rout_model(voltage_mode="log", motor_delay_ms: float = 0.0) -> ModelFactory:
    """The SILS plant's R_out model (forward_model.py). / SILS の R_out モデル。"""

    def factory(motion: fm.LoggedMotion):
        trace, torque = fm.predict_torque_trace(motion, voltage_mode, motor_delay_ms=motor_delay_ms)
        return lambda delay_s: fm.accel_from_trace(motion, trace, torque, delay_s)

    return factory


# --------------------------------------------------------------------------
# Metrics / 指標
# --------------------------------------------------------------------------
def _shift_scan(spec: dict) -> np.ndarray:
    search = spec["flip_delay_search_ms"]
    return np.arange(search["min"], search["max"] + 0.5 * search["step"], search["step"]) * 1e-3


def flip_window_metrics(motion, alpha_fn, window, axis: int, spec: dict) -> dict:
    """No-fit residual at delay 0 / scale 1, and the best extra delay (with the least-squares
    scale at each delay).  Smoother: Savitzky-Golay, applied to the prediction and (as a
    derivative) to the measured rate.
    遅れ 0・スケール 1 での残差と、遅れごとに最小二乗スケールを当てたときの最良の追加遅れ。"""
    sg = spec["flip_smoother"]
    t = motion.t_s - motion.t_s[0]
    win = (t >= window[0]) & (t <= window[1])
    meas = savgol_filter(motion.gyro[:, axis], sg["window"], sg["order"], deriv=1, delta=motion.dt)[win]
    rms = float(np.sqrt(np.mean(meas ** 2)))
    delays = _shift_scan(spec)
    rows = []
    for delay in delays:
        pred = savgol_filter(alpha_fn(delay)[:, axis], sg["window"], sg["order"])[win]
        scale = float(pred @ meas / (pred @ pred))
        fit_res = float(np.sqrt(np.mean((meas - scale * pred) ** 2)))
        raw_res = float(np.sqrt(np.mean((meas - pred) ** 2)))
        rows.append((delay, scale, fit_res, raw_res))
    rows = np.array(rows)
    best = int(np.argmin(rows[:, 2]))
    return dict(
        nofit_residual=float(rows[0, 3] / rms),
        best_delay_ms=float(rows[best, 0] * 1e3),
        best_scale=float(rows[best, 1]),
        best_residual=float(rows[best, 2] / rms),
        at_search_edge=bool(best == len(delays) - 1),
        rows=int(win.sum()),
    )


def hover_band_metrics(motion, alpha_pred: np.ndarray, window, axis: int, spec: dict) -> dict:
    """Gain |S_pm|/S_pp, delay -angle(S_pm)/(2 pi f_c) and coherence in the judged band.
    S: Welch spectra (Hann, nperseg from the file) of the predicted (p) and measured (m)
    angular acceleration on the uniform grid; f_c = power-weighted centre frequency.
    判定帯域のゲイン・遅れ・コヒーレンス。S は予測 p と実測 m の Welch スペクトル、
    f_c は帯域内のパワー重心周波数。"""
    psd = spec["hover_psd"]
    lo, hi = spec["hover_band_hz"]
    t = motion.t_s - motion.t_s[0]
    win = (t >= window[0]) & (t < window[1])
    win[-1] = False                     # accel_from_rate: last row is NaN / 最終行は NaN
    meas = fm.accel_from_rate(motion.t_s, motion.gyro, motion.dt)
    fs = 1.0 / motion.dt
    p, m = alpha_pred[win][:, axis], meas[win][:, axis]
    f, spp = signal.welch(p, fs, psd["window"], psd["nperseg"])
    _, smm = signal.welch(m, fs, psd["window"], psd["nperseg"])
    _, spm = signal.csd(p, m, fs, psd["window"], psd["nperseg"])
    band = (f >= lo) & (f < hi)
    spp_b, smm_b, spm_b = spp[band].sum(), smm[band].sum(), spm[band].sum()
    f_c = float((f[band] * spp[band]).sum() / spp_b)
    return dict(
        gain=float(abs(spm_b) / spp_b),
        delay_ms=float(-np.angle(spm_b) / (2 * np.pi * f_c) * 1e3),
        coherence=float(abs(spm_b) ** 2 / (spp_b * smm_b)),
        centre_hz=f_c,
        rows=int(win.sum()),
    )


# --------------------------------------------------------------------------
# Criteria / 判定
# --------------------------------------------------------------------------
def _judge(value: float, crit: dict, judged: bool) -> tuple:
    """Return (criterion text, status). / (基準の文字列, 合否) を返す。"""
    lo, hi = crit.get("min"), crit.get("max")
    if lo is not None and hi is not None:
        text, ok = f"{lo:g} - {hi:g}", lo <= value <= hi
    elif hi is not None:
        text, ok = f"< {hi:g}", value < hi
    else:
        text, ok = f"> {lo:g}", value > lo
    if not judged:
        return text, STATUS_REFERENCE
    return text, STATUS_PASS if ok else STATUS_FAIL


def evaluate_log(path, log_spec: dict, spec: dict, model: ModelFactory) -> list:
    """All metrics of one reference log. / 1 本の基準ログの全指標。"""
    path = Path(path)
    motion = fm.load_real_log(path, spec["treatment"])
    alpha_fn = model(motion)
    judged_axis = spec["judged_axis"]
    crit = spec["criteria"]
    name = path.name
    out = []
    for axis_name, window in log_spec.get("flip_windows", {}).items():
        axis = AXES.index(axis_name)
        r = flip_window_metrics(motion, alpha_fn, window, axis, spec)
        judged = axis_name == judged_axis
        note = f"window {window[0]}-{window[1]} s, best scale {r['best_scale']:.3f}"
        for metric, key, unit in (("flip_nofit_residual", "nofit_residual", ""),
                                  ("flip_best_delay_ms", "best_delay_ms", "ms")):
            text, status = _judge(r[key], crit[metric], judged)
            detail = note + ("; at search edge" if key == "best_delay_ms" and r["at_search_edge"] else "")
            out.append(Metric(name, axis_name, metric, r[key], unit, text, status, detail))
    hover_window = log_spec["hover_window"]
    pred = alpha_fn(0.0)
    for axis_name in (judged_axis, "pitch"):
        axis = AXES.index(axis_name)
        r = hover_band_metrics(motion, pred, hover_window, axis, spec)
        judged = axis_name == judged_axis
        note = f"window {hover_window[0]}-{hover_window[1]} s, coherence {r['coherence']:.2f}"
        for metric, key, unit in (("hover_gain", "gain", ""), ("hover_delay_ms", "delay_ms", "ms")):
            text, status = _judge(r[key], crit[metric], judged)
            out.append(Metric(name, axis_name, metric, r[key], unit, text, status, note))
    return out


def evaluate_reference_logs(spec_path, model: Optional[ModelFactory] = None, log_dir=None) -> list:
    """Evaluate every reference log of the criteria file (default model: R_out, logged V).
    基準ログすべてを評価する (既定のモデルは R_out・ログ電圧)。"""
    spec_path = Path(spec_path)
    spec = load_spec(spec_path)
    model = model or rout_model(spec["voltage_mode"])
    base = Path(log_dir) if log_dir else spec_path.parent
    rows = []
    for log_spec in spec["reference_logs"]:
        rows.extend(evaluate_log(base / log_spec["file"], log_spec, spec, model))
    return rows


def overall_pass(rows: list) -> bool:
    """True when no judged metric failed and at least one was judged. / 判定対象が全て PASS。"""
    judged = [r for r in rows if r.status != STATUS_REFERENCE]
    return bool(judged) and all(r.status == STATUS_PASS for r in judged)


# --------------------------------------------------------------------------
# Self-consistency against the SILS plant / SILS プラントとの自己無矛盾性
# --------------------------------------------------------------------------
def self_consistency(bundle, spec: dict, motor_delay_ms: float = 0.0) -> list:
    """Check the Python forward model against the SILS plant's own truth: the angular
    acceleration predicted from the LOGGED duty must equal the truth.csv body-rate
    derivative on airborne rows.  Returns one dict per axis with residual / scale / status.
    ログされた duty から予測した角加速度が、truth.csv の角速度微分 (空中の行) に一致するか。
    軸ごとに残差・スケール・合否を返す。"""
    cfg = spec["self_consistency"]
    motion = fm.load_logged_motion(bundle)
    t_truth, rate_truth, altitude = fm.load_truth(bundle)
    truth = fm.accel_from_rate(motion.t_s, rate_truth, motion.dt, t_src=t_truth)
    trace, torque = fm.predict_torque_trace(motion, motor_delay_ms=motor_delay_ms)
    pred = fm.accel_from_trace(motion, trace, torque, fm.SILS_LOG_ALIGNMENT_DELAY_S)
    airborne = np.interp(motion.t_s, t_truth, altitude) > fm.AIRBORNE_MIN_ALT_M
    airborne &= motion.t_s > motion.t_s[0] + cfg["settle_s"]
    residual = fm.relative_residual(truth, pred, airborne)
    rows = []
    for axis, axis_name in enumerate(AXES):
        ok = airborne & np.isfinite(truth[:, axis]) & np.isfinite(pred[:, axis])
        rms = float(np.sqrt(np.mean(truth[ok, axis] ** 2))) if ok.any() else 0.0
        if rms < cfg["excited_min_rms"]:
            rows.append(dict(axis=axis_name, rms=rms, residual=None, scale=None, status="not excited"))
            continue
        scale = float(truth[ok, axis] @ pred[ok, axis] / (pred[ok, axis] @ pred[ok, axis]))
        good = residual[axis] < cfg["residual_max"] and abs(scale - 1.0) < cfg["scale_tolerance"]
        rows.append(dict(axis=axis_name, rms=rms, residual=float(residual[axis]), scale=scale,
                         status=STATUS_PASS if good else STATUS_FAIL))
    return rows


def self_consistency_pass(rows: list) -> bool:
    judged = [r for r in rows if r["status"] != "not excited"]
    return bool(judged) and all(r["status"] == STATUS_PASS for r in judged)


def bundle_has_truth(bundle) -> bool:
    with zipfile.ZipFile(bundle) as zf:
        return "truth.csv" in zf.namelist()
