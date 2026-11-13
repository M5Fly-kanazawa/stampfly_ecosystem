#!/usr/bin/env python3
"""
baro_offset_study.py — real-log study of the BMP280 against the downward ToF, and the offline
evaluation of the firmware's barometric height estimator (sf::BarometricHeightEstimator).

実機ログで BMP280 を下向き ToF と突き合わせる調査と、ファームの気圧高度推定器
（sf::BarometricHeightEstimator）のオフライン評価。

Sub-commands / サブコマンド:
  study              per-log table: (a) step when the motors start / at takeoff, (b) drift, (c) noise,
                     (d) lag of the barometer against the ToF
                     ログごとの表: (a) モータ始動・離陸時のステップ、(b) ドリフト、(c) ノイズ、(d) ToF に対する遅れ
  eval [--taus ...]  the C++ estimator replays every log; at each start time t0 (every 0.5 s of airborne
                     time) the ToF is declared silent, the offset is held, and the error against the real
                     ToF height over the next 2 / 5 / 10 s is reported: per log and pooled, for the floor
                     zero only and for several learning time constants
                     C++ の推定器が全ログを再生する。各開始時刻 t0（空中時間の 0.5 s ごと）で ToF が出力停止したと
                     し、オフセットを保持して、以後 2 / 5 / 10 s の実機 ToF の高さに対する誤差を出す:
                     ログごと・全ログ合算、床のゼロだけの場合と学習時定数ごと
  velocity           stage B: does a barometer velocity improve the accelerometer-only vertical velocity of the blind descent?
                     段階 B: 気圧計の速度は、ToF なしの降下の加速度計だけの鉛直速度を改善するか
  events LOG OUT     write the event file for one log (the input of baro_offset_eval.cpp)
                     1 つのログのイベントファイルを書く（baro_offset_eval.cpp の入力）

Run (from the repository root, sf environment active):
    python analysis/scripts/baro_offset_study.py study
    python analysis/scripts/baro_offset_study.py eval

Logs / ログ: 5 captures of the vehicle (logs/*.sflog.zip, analysis/datasets/flightlog/vehicle_hover_*) and the
legacy UDP captures logs/stampfly_udp_*.jsonl that carry a barometer. The two vehicle_roll_flip_* bundles
of analysis/datasets/flightlog are byte-identical copies of logs/flip_test1/2 and are not counted twice.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "lib"))
import sflog  # noqa: E402

BUNDLES = {
    "flip1": "logs/flip_test1.sflog.zip",
    "flip2": "logs/flip_test2.sflog.zip",
    "yaw360": "logs/front_tof_poshold_yaw360.sflog.zip",
    "yaw4x90": "logs/front_tof_poshold_yaw4x90.sflog.zip",
    "hover0908": "analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip",
}
ARMED_STATES = (3, 4, 5, 6, 7)          # ARMED_GROUND, TAKEOFF, FLYING, LANDING, FLIP
MOTOR_ON_DUTY = 0.02                    # mean duty above this = motors running
AIRBORNE_HEIGHT_M = 0.15                # ground/air threshold of the takeoff/landing manager
T0_STEP_S = 0.5                         # grid of the ToF-silence start times
HORIZONS_S = (2.0, 5.0, 10.0)
MIN_TRUTH_COVERAGE = 0.5                # a window needs this fraction of its samples with a ToF truth
TILT_LIMIT_RAD = 0.52                   # the ToF height is a truth only below this tilt (30 degrees)
TRUTH_MAX_HEIGHT_M = 1.6                # ... and below the ToF range


# ---------------------------------------------------------------------------------------------
# Loading / 読み込み
# ---------------------------------------------------------------------------------------------
def _from_bundle(path: str) -> dict:
    streams = sflog.load(path).streams
    t0 = streams["imu"].timestamp_us.iloc[0]
    sec = lambda x: (np.asarray(x, dtype=float) - t0) / 1e6
    d = {"imu_t": sec(streams["imu"].timestamp_us)}
    d["baro_t"] = sec(streams["baro"].timestamp_us)
    d["baro_alt"] = streams["baro"].altitude.values.astype(float)
    d["baro_p"] = streams["baro"].pressure.values.astype(float)      # Pa (flight-log convention)
    tof = streams["tof_bottom"]
    d["tof_t"], d["tof_d"], d["tof_s"] = sec(tof.timestamp_us), tof.distance.values.astype(float), tof.status.values
    d["quat"] = streams["attitude"][["quat_w", "quat_x", "quat_y", "quat_z"]].values.astype(float)
    d["accel"] = streams["imu"][["accel_x", "accel_y", "accel_z"]].values.astype(float)
    d["eskf_vz_up"] = -streams["posvel"].vel_z.values.astype(float)
    ctrl = streams["ctrl_ref"]
    d["duty_t"] = sec(ctrl.timestamp_us)
    d["duty"] = ctrl[["duty_FR", "duty_RR", "duty_RL", "duty_FL"]].values.astype(float).mean(axis=1)
    if "flight_phase" in streams:
        d["state_t"], d["state"] = d["imu_t"], streams["flight_phase"].flight_state.values
    else:
        d["state_t"], d["state"] = sec(streams["status"].timestamp_us), streams["status"].flight_state.values
    return d


def _from_jsonl(path: str) -> dict | None:
    rows: dict[str, list] = {}
    for line in open(path):
        try:
            x = json.loads(line)
        except ValueError:
            continue
        rows.setdefault(x["id"], []).append(x)
    if "baro" not in rows or "tof_b" not in rows:
        return None
    t0 = rows["imu"][0]["ts"]
    sec = lambda v: (np.asarray(v, dtype=float) - t0) / 1e6
    d = {"imu_t": sec([x["ts"] for x in rows["imu"]])}
    d["baro_t"] = sec([x["ts"] for x in rows["baro"]])
    d["baro_alt"] = np.array([x["altitude"] for x in rows["baro"]])
    d["baro_p"] = np.array([x["pressure"] for x in rows["baro"]]) * 100.0   # hPa on the wire -> Pa
    d["tof_t"] = sec([x["ts"] for x in rows["tof_b"]])
    d["tof_d"] = np.array([x["distance"] for x in rows["tof_b"]])
    d["tof_s"] = np.array([x["status"] for x in rows["tof_b"]])
    d["quat"] = np.array([x["quat"] for x in rows["imu"]])
    d["accel"] = np.array([x["accel"] for x in rows["imu"]])
    d["eskf_vz_up"] = -np.array([x["vel"][2] for x in rows["posvel"]])
    d["duty_t"] = sec([x["ts"] for x in rows["ctrl_ref"]])
    d["duty"] = np.array([x["motor_duty"] for x in rows["ctrl_ref"]]).mean(axis=1)
    d["state_t"] = sec([x["ts"] for x in rows["status"]])
    d["state"] = np.array([x["flight_state"] for x in rows["status"]])
    return d


def load_all() -> dict[str, dict]:
    logs = {name: _from_bundle(os.path.join(ROOT, p)) for name, p in BUNDLES.items()}
    for path in sorted(glob.glob(os.path.join(ROOT, "logs", "stampfly_udp_*.jsonl"))):
        d = _from_jsonl(path)
        if d is not None:
            logs["j" + os.path.basename(path)[13:28]] = d
    for d in logs.values():
        _derive(d)
    return logs


def _derive(d: dict) -> None:
    """Tilt-corrected ToF height, armed interval, unique baro samples, motor start.
    傾き補正した ToF の高さ、ARM の区間、一意な気圧サンプル、モータ始動。"""
    q = d["quat"]
    cos_tilt = 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)
    c = np.interp(d["tof_t"], d["imu_t"], cos_tilt)
    d["tof_valid"] = (d["tof_s"] == 0) & (d["tof_d"] > 0)
    d["tof_h"] = d["tof_d"] * c
    d["tof_tilt"] = np.arccos(np.clip(c, -1, 1))
    changed = np.r_[True, np.diff(d["baro_p"]) != 0]
    d["ubaro_t"], d["ubaro_alt"] = d["baro_t"][changed], d["baro_alt"][changed]
    running = np.where(d["duty"] > MOTOR_ON_DUTY)[0]
    d["motor_start"], d["motor_end"] = d["duty_t"][running[0]], d["duty_t"][running[-1]]
    armed_idx = np.where(np.isin(d["state"], ARMED_STATES))[0]
    d["arm_start"] = min(d["state_t"][armed_idx[0]], d["motor_start"]) if len(armed_idx) else d["motor_start"]
    d["arm_end"] = max(d["state_t"][armed_idx[-1]], d["motor_end"]) if len(armed_idx) else d["motor_end"]


# ---------------------------------------------------------------------------------------------
# Events for the C++ evaluator / C++ 評価プログラム用のイベント
# ---------------------------------------------------------------------------------------------
def silence_start_times(d: dict) -> list[float]:
    """t0 every T0_STEP_S of airborne time: the ToF valid, above the ground/air threshold, craft armed.
    空中時間の T0_STEP_S ごとの t0: ToF が有効で、接地/空中のしきい値より上で、ARM 中。"""
    ok = d["tof_valid"] & (d["tof_h"] > AIRBORNE_HEIGHT_M) & (d["tof_h"] <= TRUTH_MAX_HEIGHT_M) & (d["tof_tilt"] < TILT_LIMIT_RAD) & (d["tof_t"] > d["arm_start"]) & (d["tof_t"] < d["arm_end"])
    times = d["tof_t"][ok]
    if not len(times):
        return []
    return [float(t) for t in np.arange(times[0], times[-1], T0_STEP_S) if np.min(np.abs(times - t)) < 0.1]


def write_events(d: dict, path: str) -> int:
    lines = []
    for t, alt, p in zip(d["baro_t"], d["baro_alt"], d["baro_p"]):
        armed = int(d["arm_start"] <= t <= d["arm_end"])
        lines.append((t, f"B {t:.6f} {alt:.5f} {p:.4f} {armed} {1 - armed}"))
    for t, h, tilt, ok in zip(d["tof_t"], d["tof_h"], d["tof_tilt"], d["tof_valid"]):
        lines.append((t, f"T {t:.6f} {h:.5f} {tilt:.5f} {int(ok)}"))
        if ok and tilt < TILT_LIMIT_RAD and h <= TRUTH_MAX_HEIGHT_M:
            lines.append((t, f"Q {t:.6f} {h:.5f}"))
    t0s = silence_start_times(d)
    for t0 in t0s:
        lines.append((t0 + 1e-7, f"S {t0:.6f}"))
    lines.sort(key=lambda x: x[0])
    with open(path, "w") as f:
        f.write("\n".join(s for _, s in lines) + "\n")
    return len(t0s)


def build_evaluator() -> str:
    exe = os.path.join(tempfile.gettempdir(), "baro_offset_eval")
    src = os.path.join(ROOT, "analysis", "scripts", "baro_offset_eval.cpp")
    cmd = ["g++", "-std=c++17", "-O2", "-I", os.path.join(ROOT, "firmware/vehicle/components/sf_core/include"),
           "-I", os.path.join(ROOT, "firmware/vehicle/components/sf_math/include"), src, "-o", exe]
    subprocess.run(cmd, check=True)
    return exe


def run_silence(exe: str, events: str, tau: float, learning: bool) -> dict[float, tuple[np.ndarray, np.ndarray]]:
    out = subprocess.run([exe, events, "silence", str(tau), "1" if learning else "0"], capture_output=True, text=True, check=True).stdout
    per_t0: dict[float, list] = {}
    for line in out.splitlines():
        _, t0, dt, err = line.split()
        per_t0.setdefault(float(t0), []).append((float(dt), float(err)))
    return {t0: (np.array([r[0] for r in v]), np.array([r[1] for r in v])) for t0, v in per_t0.items()}


def window_errors(d: dict, per_t0: dict, horizon: float, min_age_s: float = 0.0) -> np.ndarray:
    """All truth-covered errors within `horizon` of every t0 whose window is complete.
    窓が揃った全 t0 の、時間幅 horizon 以内の真値のある誤差。"""
    errors = []
    last = d["baro_t"][-1]
    first_airborne = silence_start_times(d)[0]
    for t0, (dt, err) in per_t0.items():
        if t0 + horizon > last or t0 - first_airborne < min_age_s:
            continue
        sel = dt <= horizon
        if sel.sum() == 0 or np.isfinite(err[sel]).mean() < MIN_TRUTH_COVERAGE:
            continue
        errors.append(err[sel][np.isfinite(err[sel])])
    return np.concatenate(errors) if errors else np.array([])


def stats(e: np.ndarray) -> str:
    if not len(e):
        return "     -     -     -"
    a = np.abs(e)
    return f"{np.sqrt(np.mean(e ** 2)):5.2f} {np.percentile(a, 95):5.2f} {a.max():5.2f}"


def command_eval(taus: list[float], min_age_s: float) -> None:
    logs = load_all()
    exe = build_evaluator()
    configs = [("floor zero only", 0.0, False)] + [(f"learned tau={tau:g} s", tau, True) for tau in taus]
    pooled = {c[0]: {h: [] for h in HORIZONS_S} for c in configs}
    with tempfile.TemporaryDirectory() as tmp:
        for name, d in logs.items():
            events = os.path.join(tmp, name + ".txt")
            n_t0 = write_events(d, events)
            if n_t0 == 0:
                continue
            print(f"\n{name}: {n_t0} start times; error [m] = baro height - ToF height: rms / 95th percentile of |e| / max |e|")
            print("  config                 " + "".join(f"   {int(h):2d} s window     " for h in HORIZONS_S))
            for label, tau, learning in configs:
                per_t0 = run_silence(exe, events, tau, learning)
                cells = []
                for h in HORIZONS_S:
                    e = window_errors(d, per_t0, h, min_age_s)
                    pooled[label][h].append(e)
                    cells.append(stats(e))
                print(f"  {label:22s} " + "   ".join(cells))
    print(f"\nPOOLED over all logs (every sample of every window; start times >= {min_age_s:g} s after the first airborne time)")
    print("  config                 " + "".join(f"   {int(h):2d} s window     " for h in HORIZONS_S))
    for label, _, _ in configs:
        cells = [stats(np.concatenate(pooled[label][h]) if pooled[label][h] else np.array([])) for h in HORIZONS_S]
        print(f"  {label:22s} " + "   ".join(cells))


# ---------------------------------------------------------------------------------------------
# Stage B: vertical velocity, accelerometer only vs accelerometer + barometer
# 段階 B: 鉛直速度、加速度計だけ vs 加速度計 + 気圧計
# ---------------------------------------------------------------------------------------------
VELOCITY_WINDOW_S = 0.25                # reference velocity: slope of a line fitted to the ToF heights within +-this of each sample
VELOCITY_MIN_POINTS = 8
VELOCITY_HORIZONS_S = (1, 2, 4, 6, 8, 10)
TOF_ALIVE_AGE_S = 0.3                   # the ToF counts as alive (eskf_trusted) while its newest valid sample is this young


def reference_velocity(d: dict) -> list[tuple[float, float]]:
    """Vertical velocity (up positive) from the real ToF: the least-squares slope of the tilt-corrected valid heights
    within +-VELOCITY_WINDOW_S of each valid sample (>= VELOCITY_MIN_POINTS points, tilt < limit, height <= TRUTH_MAX_HEIGHT_M).
    実機 ToF からの鉛直速度（上が正）: 各有効サンプルの前後 VELOCITY_WINDOW_S の傾き補正した高さへの最小二乗直線の傾き。"""
    ok = d["tof_valid"] & (d["tof_tilt"] < TILT_LIMIT_RAD) & (d["tof_h"] <= TRUTH_MAX_HEIGHT_M)
    t, h = d["tof_t"][ok], d["tof_h"][ok]
    out = []
    for i, ti in enumerate(t):
        lo, hi = np.searchsorted(t, ti - VELOCITY_WINDOW_S), np.searchsorted(t, ti + VELOCITY_WINDOW_S, side="right")
        if hi - lo >= VELOCITY_MIN_POINTS and t[hi - 1] - t[lo] > VELOCITY_WINDOW_S:
            out.append((ti, float(np.polyfit(t[lo:hi] - ti, h[lo:hi], 1)[0])))
    return out


def write_velocity_events(d: dict, path: str) -> int:
    alive_t = d["tof_t"][d["tof_valid"] & (d["tof_h"] > AIRBORNE_HEIGHT_M)]
    lines = []
    for k, t in enumerate(d["imu_t"]):
        q, a = d["quat"][k], d["accel"][k]
        j = np.searchsorted(alive_t, t) - 1
        trusted = int(j >= 0 and t - alive_t[j] < TOF_ALIVE_AGE_S and t > d["motor_start"])
        lines.append((t, f"I {t:.5f} {a[0]:.5f} {a[1]:.5f} {a[2]:.5f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f} {d['eskf_vz_up'][k]:.5f} {int(t < d['motor_start'])} {trusted}"))
    for t, alt in zip(d["ubaro_t"], d["ubaro_alt"]):
        lines.append((t, f"B {t:.6f} {alt:.5f}"))
    for t, v in reference_velocity(d):
        lines.append((t, f"R {t:.6f} {v:.5f}"))
    t0s = [t for t in silence_start_times(d) if t + VELOCITY_HORIZONS_S[-1] <= d["imu_t"][-1]]
    for t0 in t0s:
        lines.append((t0 + 1e-7, f"S {t0:.6f}"))
    lines.sort(key=lambda x: x[0])
    with open(path, "w") as f:
        f.write("\n".join(x for _, x in lines) + "\n")
    return len(t0s)


def build_velocity_evaluator() -> str:
    exe = os.path.join(tempfile.gettempdir(), "baro_velocity_eval")
    cmd = ["g++", "-std=c++17", "-O2", "-I", os.path.join(ROOT, "firmware/vehicle/components/sf_core/include"),
           "-I", os.path.join(ROOT, "firmware/vehicle/components/sf_math/include"),
           os.path.join(ROOT, "analysis", "scripts", "baro_velocity_eval.cpp"), "-o", exe]
    subprocess.run(cmd, check=True)
    return exe


def command_velocity(taus: list[float], gains: list[float]) -> None:
    logs = load_all()
    exe = build_velocity_evaluator()
    configs = [("(i) accel only", 1.0, 0.0)] + [(f"(ii) tau={tau:g} s gain={g:g}", tau, g) for tau in taus for g in gains]
    pooled = {c[0]: {h: [] for h in VELOCITY_HORIZONS_S} for c in configs}
    with tempfile.TemporaryDirectory() as tmp:
        for name, d in logs.items():
            events = os.path.join(tmp, name + ".txt")
            if write_velocity_events(d, events) == 0:
                continue
            for label, tau, gain in configs:
                out = subprocess.run([exe, events, str(tau), str(gain)], capture_output=True, text=True, check=True).stdout
                for line in out.splitlines():
                    _, _, h, v, ref, _ = line.split()
                    if np.isfinite(float(ref)):
                        pooled[label][int(float(h))].append(float(v) - float(ref))
    print("vertical-velocity error [m/s] (estimate - ToF-derived reference), pooled over all logs and start times: rms / 95th percentile of |e| / max |e|")
    print("  config                        " + "".join(f"  {h:2d} s          " for h in VELOCITY_HORIZONS_S))
    for label, _, _ in configs:
        cells = [stats(np.array(pooled[label][h])) for h in VELOCITY_HORIZONS_S]
        print(f"  {label:28s}" + "  ".join(cells))
    n = [len(pooled[configs[0][0]][h]) for h in VELOCITY_HORIZONS_S]
    print("  samples per horizon:", n)


# ---------------------------------------------------------------------------------------------
# Study table / 調査の表
# ---------------------------------------------------------------------------------------------
def _moving_average(x: np.ndarray, n: int) -> np.ndarray:
    return np.convolve(x, np.ones(n) / n, mode="same")


def tof_height_at(d: dict, t: np.ndarray) -> np.ndarray:
    """ToF height interpolated to t; NaN where the valid samples around t are more than 0.15 s apart.
    t での ToF の高さ（補間）。前後の有効サンプルが 0.15 s より離れていれば NaN。"""
    valid_t, valid_h = d["tof_t"][d["tof_valid"]], d["tof_h"][d["tof_valid"]]
    if len(valid_t) < 2:
        return np.full(len(t), np.nan)
    idx = np.clip(np.searchsorted(valid_t, t), 1, len(valid_t) - 1)
    gap_ok = (valid_t[idx] - valid_t[idx - 1]) < 0.15
    return np.where(gap_ok, np.interp(t, valid_t, valid_h, left=np.nan, right=np.nan), np.nan)


def study_row(name: str, d: dict) -> dict:
    t, alt = d["ubaro_t"], d["ubaro_alt"]
    pre = (t < d["motor_start"] - 0.2) & (t > d["motor_start"] - 3.0)
    zero = alt[pre].mean()
    h_at = tof_height_at(d, t)
    air = h_at > 0.3
    row = {"log": name, "duration_s": d["imu_t"][-1], "pre_noise_m": (alt[pre] - np.polyval(np.polyfit(t[pre], alt[pre], 1), t[pre])).std()}
    on = (t > d["motor_start"] + 0.3) & (t < d["motor_start"] + 1.0) & ~(h_at > 0.05)
    row["step_motors_on_floor_m"] = alt[on].mean() - zero if on.sum() >= 2 else np.nan
    if air.sum() > 50:
        diff = alt[air] - zero - h_at[air]
        ta = t[air]
        first = ta < ta[0] + 3.0
        row.update(air_s=ta[-1] - ta[0], step_takeoff_m=diff[first].mean(), mean_m=diff.mean(), sd_m=diff.std(), max_abs_m=np.abs(diff).max(),
                   drift_mm_s=np.polyfit(ta, diff, 1)[0] * 1000 if ta[-1] - ta[0] > 5 else np.nan,
                   duty=np.interp(ta, d["duty_t"], d["duty"]).mean(),
                   hp_noise_m=(diff - _moving_average(diff, 13))[7:-7].std())
    post = t > d["motor_end"] + 3
    row["ground_after_minus_before_m"] = alt[post].mean() - zero if post.sum() > 30 else np.nan
    row["ground_slope_mm_s"] = np.polyfit(t[post], alt[post], 1)[0] * 1000 if post.sum() > 30 else np.nan
    row["unique_rate_hz"] = len(t) / t[-1]
    return row


def command_study() -> None:
    logs = load_all()
    cols = ["duration_s", "air_s", "duty", "step_motors_on_floor_m", "step_takeoff_m", "mean_m", "sd_m", "max_abs_m", "drift_mm_s",
            "ground_slope_mm_s", "ground_after_minus_before_m", "pre_noise_m", "hp_noise_m", "unique_rate_hz"]
    print("(a) step_motors_on_floor_m: baro rise 0.3-1.0 s after the motors start, craft still on the floor (relative to the 3 s before)")
    print("    step_takeoff_m / mean_m / sd_m / max_abs_m: (baro - floor zero) - ToF height, first 3 s of flight / whole flight, ToF valid and > 0.3 m")
    print("(b) drift_mm_s: slope of that difference in flight; ground_slope_mm_s, ground_after_minus_before_m: motors off after the flight")
    print("(c) pre_noise_m: rms about a line on the floor (3 s); hp_noise_m: rms of the difference minus its 1 s moving average in flight")
    print("    unique_rate_hz: distinct pressure values per second (the effective conversion rate)")
    print(f"{'log':18s}" + "".join(f"{c[:11]:>12s}" for c in cols))
    for name, d in logs.items():
        r = study_row(name, d)
        print(f"{name:18s}" + "".join(f"{r.get(c, np.nan):12.3f}" for c in cols))


NOISE_BANDS_HZ = ((0.05, 0.5), (0.5, 2.0), (2.0, 6.5))   # up to the Nyquist frequency of 13.3 samples/s
NOISE_GRID_HZ = 13.3


def band_rms(t: np.ndarray, x: np.ndarray) -> list[float]:
    """rms of x in each NOISE_BANDS_HZ band (Hann-windowed FFT of x resampled to NOISE_GRID_HZ).
    x の各周波数帯の rms（NOISE_GRID_HZ に再サンプルして Hann 窓の FFT）。"""
    grid = np.arange(t[0], t[-1], 1 / NOISE_GRID_HZ)
    y = np.interp(grid, t, x)
    y = y - y.mean()
    w = np.hanning(len(y))
    power = np.abs(np.fft.rfft(y * w)) ** 2 / (np.sum(w ** 2) * NOISE_GRID_HZ)
    freq = np.fft.rfftfreq(len(y), 1 / NOISE_GRID_HZ)
    df = freq[1] - freq[0]
    return [float(np.sqrt(2 * power[(freq >= lo) & (freq < hi)].sum() * df)) for lo, hi in NOISE_BANDS_HZ]


def command_noise() -> None:
    """(c) noise by frequency band: on the floor with the motors off (barometer alone) and in flight
    (barometer minus ToF height, over uninterrupted ToF stretches of at least 8 s).
    (c) 周波数帯ごとのノイズ: モータ停止の床の上（気圧計だけ）と、飛行中（気圧計 − ToF の高さ、ToF が 8 s 以上途切れない区間）。"""
    logs = load_all()
    head = "".join(f"  {lo:g}-{hi:g} Hz" for lo, hi in NOISE_BANDS_HZ)
    print(f"\n(c) noise rms [m] by band;  floor, motors off (>= 20 s) |  in flight (baro - ToF height, stretches >= 8 s)")
    print(f"{'log':18s}{head}   |{head}")
    for name, d in logs.items():
        t, alt = d["ubaro_t"], d["ubaro_alt"]
        post = t > d["motor_end"] + 3
        floor = band_rms(t[post], alt[post]) if post.sum() > 20 * 13 else [np.nan] * 3
        h_at = tof_height_at(d, t)
        ok = np.isfinite(h_at) & (h_at > 0.3)
        edges = np.r_[0, np.where(np.diff(np.where(ok)[0]) > 3)[0] + 1, ok.sum()]
        idx = np.where(ok)[0]
        pieces = [idx[a:b] for a, b in zip(edges[:-1], edges[1:]) if b - a > 8 * 13]
        flight = np.nanmean([band_rms(t[i], alt[i] - h_at[i]) for i in pieces], axis=0) if pieces else [np.nan] * 3
        print(f"{name:18s}" + "".join(f"{v:10.3f}" for v in floor) + "   |" + "".join(f"{v:10.3f}" for v in flight))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("study")
    ev = sub.add_parser("eval")
    ev.add_argument("--taus", type=float, nargs="+", default=[1, 2, 5, 10, 20, 40])
    ev.add_argument("--min-age", type=float, default=0.0, help="only start times at least this long after the first airborne time [s]")
    vel = sub.add_parser("velocity", help="stage B: vertical velocity, accelerometer only vs accelerometer + barometer")
    vel.add_argument("--taus", type=float, nargs="+", default=[0.5, 1.0, 2.0])
    vel.add_argument("--gains", type=float, nargs="+", default=[0.005, 0.01, 0.02, 0.05])
    ex = sub.add_parser("events")
    ex.add_argument("log")
    ex.add_argument("out")
    args = ap.parse_args()
    if args.command == "study":
        command_study()
        command_noise()
    elif args.command == "velocity":
        command_velocity(args.taus, args.gains)
    elif args.command == "eval":
        command_eval(args.taus, args.min_age)
    else:
        logs = load_all()
        print(write_events(logs[args.log], args.out), "start times")


if __name__ == "__main__":
    main()
