#!/usr/bin/env python3
"""
Stream-parse a StampFly stampfly_udp_*.jsonl flight log for the V(T) motor
identification study. Extends analysis/scripts/ct_switch_study_20260727/parse_log.py
(read-only reference, NOT imported — copied+extended here since the repo must
not be modified) with the "imu" topic (400Hz quat/accel/accel_bias), which the
CT-switch study did not need.

StampFly 実飛行ログ(stampfly_udp_*.jsonl)をストリーム解析。V(T)同定に必要な
imuトピック(400Hz、quat/accel/accel_bias)を追加で取り込む。

Wire-format ground truth (data_stream_wire.hpp, read 2026-07-29):
  - WireImuEskf.accel[] and .accel_raw[] are literally copies of the SAME
    source sample (UnifiedPacketBuilder::begin: imu.accel[axis] = s.accel[axis];
    imu.accel_raw[axis] = s.accel[axis]) -- i.e. NOT bias-corrected on the wire.
    accel_bias[] is the ESKF's separately-estimated bias (int16 x10000 on the
    wire, already de-quantized to float by the PC-side decoder before it lands
    in the .jsonl, confirmed by inspecting a sample: accel_bias ~ [-0.25,0.42,-0.01]
    which is a physically plausible bias, not a raw int16 magnitude).
    -> bias-corrected specific force must be computed here as accel_raw - accel_bias,
       matching eskf_core.cpp EskfCore::predict(): "accel = accel_raw - ba_".
  - quat[4] wire order is [w,x,y,z] (data_stream_wire.hpp comment, and
    sf_math.hpp Quat{w,x,y,z}).
  - eskf_core.cpp EskfCore::predict(): accel_ned = R(q) * accel_body_corrected;
    vel += (accel_ned + gravity)*dt with gravity=[0,0,+g] (NED, down-positive).
    q is therefore the BODY-TO-NED (world-from-body) rotation. R(q) is built by
    Quat::to_dcm() (sf_math.hpp) -- ported verbatim into `quat_to_dcm` below so
    the vertical-specific-force projection is provably identical to what the
    firmware itself computes internally (not a re-derivation that could get a
    sign wrong).
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd


def parse_log(path: Path) -> dict:
    """Single streaming pass. Returns dict of lists per topic."""
    ctrl_ref = {"ts": [], "mode": [], "total_thrust": [], "duty": []}
    ctrl = {"ts": [], "throttle": [], "roll": [], "pitch": [], "yaw": []}
    posvel = {"ts": [], "pos": [], "vel": []}
    status = {"ts": [], "voltage": [], "flight_state": [], "current_ma": []}
    imu = {"ts": [], "accel_raw": [], "accel_bias": [], "quat": []}

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            i = d.get("id")
            if i == "ctrl_ref":
                ctrl_ref["ts"].append(d["ts"])
                ctrl_ref["mode"].append(d.get("mode", -1))
                ctrl_ref["total_thrust"].append(d.get("total_thrust", 0.0))
                ctrl_ref["duty"].append(d.get("motor_duty", [0, 0, 0, 0]))
            elif i == "ctrl":
                ctrl["ts"].append(d["ts"])
                ctrl["throttle"].append(d.get("throttle", 0.0))
                ctrl["roll"].append(d.get("roll", 0.0))
                ctrl["pitch"].append(d.get("pitch", 0.0))
                ctrl["yaw"].append(d.get("yaw", 0.0))
            elif i == "posvel":
                posvel["ts"].append(d["ts"])
                posvel["pos"].append(d.get("pos", [0, 0, 0]))
                posvel["vel"].append(d.get("vel", [0, 0, 0]))
            elif i == "status":
                status["ts"].append(d["ts"])
                status["voltage"].append(d.get("voltage", np.nan))
                status["flight_state"].append(d.get("flight_state", -1))
                status["current_ma"].append(d.get("current_ma", np.nan))
            elif i == "imu":
                imu["ts"].append(d["ts"])
                # NOTE: "accel" field on this topic is a wire-format duplicate
                # of accel_raw (see module docstring) -- read accel_raw only.
                imu["accel_raw"].append(d.get("accel_raw", d.get("accel", [0, 0, 0])))
                imu["accel_bias"].append(d.get("accel_bias", [0, 0, 0]))
                imu["quat"].append(d.get("quat", [1, 0, 0, 0]))

    return {"ctrl_ref": ctrl_ref, "ctrl": ctrl, "posvel": posvel,
            "status": status, "imu": imu}


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """Vectorized port of sf_math.hpp Quat::to_dcm(). q: (N,4) [w,x,y,z].
    Returns R: (N,3,3) such that v_ned = R @ v_body (body-to-NED)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    n = q.shape[0]
    R = np.empty((n, 3, 3), dtype=np.float64)
    R[:, 0, 0] = 1 - 2 * (yy + zz)
    R[:, 0, 1] = 2 * (xy - wz)
    R[:, 0, 2] = 2 * (xz + wy)
    R[:, 1, 0] = 2 * (xy + wz)
    R[:, 1, 1] = 1 - 2 * (xx + zz)
    R[:, 1, 2] = 2 * (yz - wx)
    R[:, 2, 0] = 2 * (xz - wy)
    R[:, 2, 1] = 2 * (yz + wx)
    R[:, 2, 2] = 1 - 2 * (xx + yy)
    return R


def nearest_merge(ref_ts: np.ndarray, src_ts: np.ndarray, src_vals: np.ndarray, max_gap_us: float) -> np.ndarray:
    """For each ref_ts, find the nearest src_ts sample (assumes src_ts sorted
    ascending) and return src_vals at that index, or NaN if the source is
    empty or the nearest sample is farther than max_gap_us away."""
    if len(src_ts) == 0:
        return np.full((len(ref_ts),) + src_vals.shape[1:], np.nan)
    idx = np.searchsorted(src_ts, ref_ts)
    idx = np.clip(idx, 1, len(src_ts) - 1)
    left = idx - 1
    right = idx
    left_gap = np.abs(ref_ts - src_ts[left])
    right_gap = np.abs(src_ts[right] - ref_ts)
    use_left = left_gap <= right_gap
    chosen = np.where(use_left, left, right)
    gap = np.where(use_left, left_gap, right_gap)
    out = src_vals[chosen].astype(float)
    bad = gap > max_gap_us
    out[bad] = np.nan
    return out


def interp_merge(ref_ts: np.ndarray, src_ts: np.ndarray, src_vals: np.ndarray, max_extrap_us: float) -> np.ndarray:
    """Linear interpolation (np.interp) for slowly-varying scalar sources
    (battery voltage). Used instead of nearest_merge for voltage because the
    V(T) fit target V_bar = duty*Vbat is directly sensitive to Vbat precision,
    and status packets are sparse (~1Hz) so nearest-sample can be off by up to
    0.5s of battery droop. Edge samples beyond max_extrap_us of the nearest
    src sample are marked NaN (no reliable extrapolation)."""
    if len(src_ts) < 2:
        return np.full(len(ref_ts), np.nan)
    out = np.interp(ref_ts, src_ts, src_vals, left=np.nan, right=np.nan)
    # np.interp already returns NaN for out-of-range points via left/right;
    # but only if ref_ts strictly outside [src_ts[0], src_ts[-1]]. Additionally
    # guard near-edge points using nearest gap.
    idx = np.clip(np.searchsorted(src_ts, ref_ts), 1, len(src_ts) - 1)
    gap = np.minimum(np.abs(ref_ts - src_ts[idx - 1]), np.abs(src_ts[idx] - ref_ts))
    out = np.where(gap > max_extrap_us, np.nan, out)
    return out


def windowed_mean_std(ref_ts: np.ndarray, src_ts: np.ndarray, src_vals: np.ndarray,
                       half_window_us: float):
    """For each ref_ts, mean/std/count of src_vals with |src_ts - ref_ts| <= half_window_us.
    src_ts must be sorted ascending. O(N log M) via searchsorted + prefix sums."""
    n = len(ref_ts)
    if len(src_ts) == 0:
        nan = np.full(n, np.nan)
        return nan, nan, np.zeros(n, dtype=int)

    lo = np.searchsorted(src_ts, ref_ts - half_window_us, side="left")
    hi = np.searchsorted(src_ts, ref_ts + half_window_us, side="right")
    count = hi - lo

    cs = np.concatenate([[0.0], np.cumsum(src_vals, dtype=np.float64)])
    cs2 = np.concatenate([[0.0], np.cumsum(src_vals.astype(np.float64) ** 2)])
    s = cs[hi] - cs[lo]
    s2 = cs2[hi] - cs2[lo]

    mean = np.where(count > 0, s / np.maximum(count, 1), np.nan)
    var = np.where(count > 1, s2 / np.maximum(count, 1) - mean ** 2, np.nan)
    var = np.maximum(var, 0.0)
    std = np.sqrt(var)
    return mean, std, count


def build_dataframe(parsed: dict) -> pd.DataFrame:
    cr = parsed["ctrl_ref"]
    if len(cr["ts"]) == 0:
        return pd.DataFrame()

    ts = np.asarray(cr["ts"], dtype=np.int64)
    order = np.argsort(ts)
    ts = ts[order]
    mode = np.asarray(cr["mode"])[order]
    total_thrust = np.asarray(cr["total_thrust"])[order]
    duty = np.asarray(cr["duty"])[order]  # (N,4) FR,RR,RL,FL

    df = pd.DataFrame({
        "ts_us": ts,
        "mode": mode,
        "total_thrust_logged": total_thrust,
        "duty_m1": duty[:, 0],
        "duty_m2": duty[:, 1],
        "duty_m3": duty[:, 2],
        "duty_m4": duty[:, 3],
    })
    df["duty_mean"] = duty.mean(axis=1)
    df["duty_std_across_motors"] = duty.std(axis=1)

    # ctrl (stick) — nearest merge, 50Hz vs 50Hz so gap should be tiny
    c = parsed["ctrl"]
    if len(c["ts"]) > 0:
        c_ts = np.asarray(c["ts"], dtype=np.int64)
        c_order = np.argsort(c_ts)
        c_ts = c_ts[c_order]
        for key in ("throttle", "roll", "pitch", "yaw"):
            vals = np.asarray(c[key])[c_order]
            df[f"stick_{key}"] = nearest_merge(ts, c_ts, vals, max_gap_us=100_000)
    else:
        for key in ("throttle", "roll", "pitch", "yaw"):
            df[f"stick_{key}"] = np.nan

    # posvel — nearest merge (400Hz source, dense, gap should be tiny)
    pv = parsed["posvel"]
    if len(pv["ts"]) > 0:
        pv_ts = np.asarray(pv["ts"], dtype=np.int64)
        pv_order = np.argsort(pv_ts)
        pv_ts = pv_ts[pv_order]
        pos = np.asarray(pv["pos"])[pv_order]
        vel = np.asarray(pv["vel"])[pv_order]
        alt = -pos[:, 2]  # NED z-down -> altitude up
        velz_up = -vel[:, 2]
        df["altitude"] = nearest_merge(ts, pv_ts, alt, max_gap_us=100_000)
        df["velz_up"] = nearest_merge(ts, pv_ts, velz_up, max_gap_us=100_000)
    else:
        df["altitude"] = np.nan
        df["velz_up"] = np.nan

    # status (battery voltage etc.) — sparse (~1Hz): linear interpolation for
    # voltage (precision-critical for this study), nearest for the rest.
    st = parsed["status"]
    if len(st["ts"]) > 0:
        st_ts = np.asarray(st["ts"], dtype=np.int64)
        st_order = np.argsort(st_ts)
        st_ts = st_ts[st_order]
        voltage = np.asarray(st["voltage"], dtype=float)[st_order]
        flight_state = np.asarray(st["flight_state"], dtype=float)[st_order]
        current_ma = np.asarray(st["current_ma"], dtype=float)[st_order]
        df["voltage"] = interp_merge(ts, st_ts, voltage, max_extrap_us=3_000_000)
        df["voltage_nearest"] = nearest_merge(ts, st_ts, voltage, max_gap_us=2_000_000)
        df["flight_state"] = nearest_merge(ts, st_ts, flight_state, max_gap_us=2_000_000)
        df["current_ma"] = nearest_merge(ts, st_ts, current_ma, max_gap_us=2_000_000)
    else:
        df["voltage"] = np.nan
        df["voltage_nearest"] = np.nan
        df["flight_state"] = np.nan
        df["current_ma"] = np.nan

    # imu (400Hz quat + accel_raw + accel_bias) — reconstruct actual thrust
    # via vertical projection of bias-corrected specific force, block-averaged
    # over a +-10ms window centered on each 50Hz ctrl_ref sample (== the
    # firmware's own ctrl_ref period, so ~8 IMU samples/window; averaging
    # also suppresses motor-vibration noise in the raw accelerometer).
    im = parsed["imu"]
    if len(im["ts"]) > 0:
        im_ts = np.asarray(im["ts"], dtype=np.int64)
        im_order = np.argsort(im_ts)
        im_ts = im_ts[im_order]
        accel_raw = np.asarray(im["accel_raw"])[im_order]
        accel_bias = np.asarray(im["accel_bias"])[im_order]
        quat = np.asarray(im["quat"])[im_order]

        accel_body = accel_raw - accel_bias  # bias-corrected specific force, body FRD
        R = quat_to_dcm(quat)  # body->NED
        accel_ned = np.einsum("nij,nj->ni", R, accel_body)
        f_up = -accel_ned[:, 2]          # vertical (up-positive) specific force [m/s^2]
        tilt_rad = np.arccos(np.clip(R[:, 2, 2], -1.0, 1.0))  # body-z vs world-down axis

        half_win = 10_000  # us, matches 50Hz ctrl_ref half-period
        f_up_mean, f_up_std, f_up_n = windowed_mean_std(ts, im_ts, f_up, half_win)
        tilt_mean, _, _ = windowed_mean_std(ts, im_ts, np.degrees(tilt_rad), half_win)

        df["f_up_mps2"] = f_up_mean
        df["f_up_std_mps2"] = f_up_std
        df["f_up_n_samples"] = f_up_n
        df["tilt_deg"] = tilt_mean

        # Raw (uncorrected, no bias subtraction) variant kept for a sensitivity
        # check on how much the ESKF accel-bias estimate matters to T_actual.
        accel_ned_nobias = np.einsum("nij,nj->ni", R, accel_raw)
        f_up_nobias = -accel_ned_nobias[:, 2]
        f_up_nobias_mean, _, _ = windowed_mean_std(ts, im_ts, f_up_nobias, half_win)
        df["f_up_nobias_mps2"] = f_up_nobias_mean
    else:
        df["f_up_mps2"] = np.nan
        df["f_up_std_mps2"] = np.nan
        df["f_up_n_samples"] = 0
        df["tilt_deg"] = np.nan
        df["f_up_nobias_mps2"] = np.nan

    df["t_s"] = (df["ts_us"] - df["ts_us"].iloc[0]) / 1e6
    return df


def load_or_parse(path: Path, cache_dir: Path) -> pd.DataFrame:
    cache = cache_dir / (path.stem + ".vt.pkl")
    if cache.exists() and cache.stat().st_mtime > path.stat().st_mtime:
        return pd.read_pickle(cache)
    parsed = parse_log(path)
    df = build_dataframe(parsed)
    cache_dir.mkdir(parents=True, exist_ok=True)
    df.to_pickle(cache)
    return df


if __name__ == "__main__":
    p = Path(sys.argv[1])
    cache_dir = Path(__file__).parent / "cache"
    df = load_or_parse(p, cache_dir)
    print(f"{p.name}: {len(df)} ctrl_ref samples, duration {df['t_s'].max():.1f}s")
    print(df[["t_s", "mode", "duty_mean", "voltage", "f_up_mps2", "f_up_n_samples",
              "tilt_deg", "altitude", "velz_up"]].describe())
    print(df["mode"].value_counts())
