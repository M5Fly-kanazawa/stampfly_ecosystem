"""
replay_data.py - flight log -> per-video-frame data for `sf log video`
フライトログ -> 動画の1コマごとのデータ（`sf log video` 用）

Turns a flight-log v1 bundle (lib/sflog FlightLog) into a JSON-able dict
sampled on a uniform grid at exactly the video frame times
    t_k = start + k * speed / fps      (seconds since the first sample)
so the page (page/replay_view.js) never has to interpolate.
フライトログ v1 一式（lib/sflog の FlightLog）を、動画の各コマの時刻
    t_k = start + k * speed / fps      （先頭サンプルからの秒）
にぴったり合わせた等間隔の格子へ再サンプルした、JSON 化できる辞書にする。
ページ側（page/replay_view.js）は補間をしなくてよい。

CONVENTIONS (the JS uses the same words) / 規約（JS 側でも同じ言葉で書く）
  * attitude quaternion (w,x,y,z) rotates BODY FRD vectors into NED
    (R body->NED).  attitude の四元数 (w,x,y,z) は機体 FRD のベクトルを
    NED へ回す（R 機体->NED）。
  * posvel position is NED in metres; altitude h = -pos_z.
    posvel の位置は NED [m]、高度 h = -pos_z。
  * motor duty order is FR, RR, RL, FL (= M1..M4). Prefer the `motor` stream
    (400 Hz); otherwise the ctrl_ref duties (50 Hz).
    モータ duty の並びは FR, RR, RL, FL（= M1..M4）。`motor`（400 Hz）を優先し、
    無ければ ctrl_ref の duty（50 Hz）。
  * flight mode 0 ACRO, 1 STABILIZE, 2 ALT_HOLD, 3 POS_HOLD; flight state
    0 INIT .. 7 FLIP (firmware sf_state/flight_state.hpp).
    飛行モード 0 ACRO, 1 STABILIZE, 2 ALT_HOLD, 3 POS_HOLD。飛行状態は
    0 INIT .. 7 FLIP（ファーム sf_state/flight_state.hpp）。
  * roll/pitch/yaw are the true (not exaggerated) ZYX Euler angles in
    degrees. roll/pitch/yaw は誇張なしの ZYX オイラー角 [deg]。
  * battery voltage = status (about 1 Hz), held until the next sample.
    電池電圧は status（約 1 Hz）を次のサンプルまで保持。
  * setpoint = ctrl_ref pos_setpoint / alt_setpoint while in POS_HOLD, else
    the takeoff point (position at the window start).
    目標点は POS_HOLD 中の ctrl_ref の pos_setpoint / alt_setpoint、
    無ければ離陸点（窓の開始位置）。
"""

import math
from typing import Optional, Sequence, Tuple

import numpy as np

FLIGHT_MODE_NAMES = ["ACRO", "STABILIZE", "ALT_HOLD", "POS_HOLD"]
FLIGHT_STATE_NAMES = [
    "INIT", "IDLE_GROUND", "IDLE_HELD", "ARMED_GROUND",
    "TAKEOFF", "FLYING", "LANDING", "FLIP",
]
MOTOR_ORDER = ["FR", "RR", "RL", "FL"]

THRUST_THRESHOLD_N = 0.02   # "flying" when total thrust exceeds this / この推力を超えたら飛行中
WINDOW_PAD_S = 1.0          # padding on each side of the auto window / 自動窓の前後の余白
MODE_NONE = -1              # flight mode unknown / 飛行モード不明
STATE_NONE = -1             # flight state unknown / 飛行状態不明
ROUND_DIGITS = 6            # JSON precision / JSON の桁数


# ---------------------------------------------------------------- time base
def bundle_t0_us(log) -> int:
    """Time zero: the smallest first timestamp over every stream, the same
    reference `sf log viz` uses for its time axis.
    時刻の原点: 全ストリームの先頭タイムスタンプの最小値（`sf log viz` の
    時間軸と同じ基準）。"""
    firsts = [
        int(df["timestamp_us"].iloc[0])
        for df in log.streams.values()
        if not df.empty and "timestamp_us" in df.columns
    ]
    return min(firsts) if firsts else 0


def _seconds(df, t0_us: int) -> np.ndarray:
    """Stream timestamps in seconds since `t0_us`.
    ストリームのタイムスタンプを `t0_us` からの秒にする。"""
    return (df["timestamp_us"].to_numpy(dtype=np.int64) - t0_us) / 1e6


def _increasing(t: np.ndarray) -> np.ndarray:
    """Boolean mask that keeps strictly increasing timestamps (a repeated or
    backwards timestamp would break np.interp).
    単調増加なタイムスタンプだけを残すマスク（重複・逆行は np.interp を壊す）。"""
    keep = np.ones(len(t), dtype=bool)
    if len(t) > 1:
        keep[1:] = np.diff(t) > 0
    return keep


# -------------------------------------------------------------- resampling
def _interp_columns(t_src: np.ndarray, values: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    """Linear interpolation of each column of `values` (N,C) onto `t_grid`;
    NaN samples are dropped per column, ends are held.
    `values` (N,C) の各列を `t_grid` へ線形補間する。NaN は列ごとに除き、
    両端は保持する。"""
    out = np.empty((len(t_grid), values.shape[1]))
    for c in range(values.shape[1]):
        ok = np.isfinite(values[:, c])
        out[:, c] = np.interp(t_grid, t_src[ok], values[ok, c]) if ok.any() else np.nan
    return out


def _hold_previous(t_src: np.ndarray, values: np.ndarray, t_grid: np.ndarray, default):
    """Zero-order hold: the latest source sample at or before each grid time
    (`default` before the first sample). Used for enums such as the mode.
    零次ホールド: 各格子時刻以前の最新サンプル（最初のサンプルより前は
    `default`）。モード等の列挙値に使う。"""
    idx = np.searchsorted(t_src, t_grid, side="right") - 1
    out = np.where(idx >= 0, values[np.clip(idx, 0, None)], default)
    return out


def hemisphere_continuous(quat: np.ndarray) -> np.ndarray:
    """Flip the sign of quaternions so consecutive samples have a positive
    dot product (q and -q are the same rotation; without this, linear
    interpolation can swing through the wrong half of the sphere).
    隣り合うサンプルの内積が正になるよう四元数の符号を揃える（q と -q は
    同じ回転。揃えないと線形補間が球面の反対側を通ってしまう）。"""
    quat = np.array(quat, dtype=float)
    if len(quat) < 2:
        return quat
    dots = np.einsum("ij,ij->i", quat[1:], quat[:-1])
    # The sign of sample i relative to the raw previous sample, accumulated.
    # 直前の生サンプルに対する符号を累積して、各サンプルの符号を決める。
    flips = np.where(dots < 0, -1.0, 1.0)
    signs = np.concatenate([[1.0], np.cumprod(flips)])
    return quat * signs[:, None]


def nlerp_quaternions(t_src: np.ndarray, quat: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    """Normalised linear interpolation of quaternions (w,x,y,z) onto
    `t_grid`, after making them hemisphere-continuous.
    四元数 (w,x,y,z) を半球連続にしてから `t_grid` へ正規化線形補間する。"""
    q = hemisphere_continuous(quat)
    out = _interp_columns(t_src, q, t_grid)
    norm = np.linalg.norm(out, axis=1, keepdims=True)
    norm[norm == 0] = 1.0
    return out / norm


def euler_zyx_deg(quat: np.ndarray) -> np.ndarray:
    """True roll/pitch/yaw [deg] (ZYX) of body->NED quaternions (N,4 w,x,y,z).
    body->NED 四元数 (N,4: w,x,y,z) の真の roll/pitch/yaw [deg]（ZYX）。"""
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.degrees(np.stack([roll, pitch, yaw], axis=1))


# ------------------------------------------------------------------ window
def _thrust_series(log, t0_us: int) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
    """Total-thrust time series [N] used to find the flight window:
    ctrl_output.thrust (400 Hz), else ctrl_ref.total_thrust (50 Hz).
    飛行窓を探すための総推力 [N] の時系列: ctrl_output.thrust（400 Hz）、
    無ければ ctrl_ref.total_thrust（50 Hz）。"""
    for name, col in (("ctrl_output", "thrust"), ("ctrl_ref", "total_thrust")):
        df = log.streams.get(name)
        if df is not None and not df.empty and col in df.columns:
            return _seconds(df, t0_us), df[col].to_numpy(dtype=float), f"{name}.{col}"
    return None


def longest_run(t: np.ndarray, mask: np.ndarray) -> Optional[Tuple[float, float]]:
    """(start, end) seconds of the longest contiguous True run of `mask`.
    `mask` の最長の連続 True 区間の (開始, 終了) 秒。"""
    best = None
    i, n = 0, len(mask)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and mask[j + 1]:
            j += 1
        if best is None or t[j] - t[i] > best[1] - best[0]:
            best = (float(t[i]), float(t[j]))
        i = j + 1
    return best


def default_window(log, t0_us: int, data_end_s: float) -> Tuple[float, float, str]:
    """Default video window: the largest contiguous segment with total thrust
    above THRUST_THRESHOLD_N, padded by WINDOW_PAD_S each side and clipped to
    the data. Falls back to the whole log (with a note) when no thrust
    information or no such segment exists.
    既定の動画の窓: 総推力が THRUST_THRESHOLD_N を超える最大の連続区間を
    前後 WINDOW_PAD_S だけ広げ、データ範囲に収めたもの。推力情報や該当区間が
    無ければログ全体（注記付き）にする。"""
    series = _thrust_series(log, t0_us)
    if series is None:
        return 0.0, data_end_s, "no thrust stream: whole log used"
    t, thrust, label = series
    run = longest_run(t, np.nan_to_num(thrust) > THRUST_THRESHOLD_N)
    if run is None:
        return 0.0, data_end_s, f"{label} never above {THRUST_THRESHOLD_N} N: whole log used"
    start = max(0.0, run[0] - WINDOW_PAD_S)
    end = min(data_end_s, run[1] + WINDOW_PAD_S)
    return start, end, f"window from {label} > {THRUST_THRESHOLD_N} N, +-{WINDOW_PAD_S:g} s"


# --------------------------------------------------------------- the pieces
def _motor_duties(log, t0_us: int, t_grid: np.ndarray) -> Tuple[np.ndarray, str]:
    """Four motor duties (FR, RR, RL, FL) on the grid: `motor` stream if
    present, else ctrl_ref, else zeros.
    格子上の 4 モータ duty（FR, RR, RL, FL）: `motor` ストリームがあればそれ、
    無ければ ctrl_ref、それも無ければ 0。"""
    cols = ["duty_FR", "duty_RR", "duty_RL", "duty_FL"]
    for name in ("motor", "ctrl_ref"):
        df = log.streams.get(name)
        if df is None or df.empty or not set(cols) <= set(df.columns):
            continue
        t = _seconds(df, t0_us)
        keep = _increasing(t)
        duty = _interp_columns(t[keep], df[cols].to_numpy(dtype=float)[keep], t_grid)
        return np.clip(np.nan_to_num(duty), 0.0, 1.0), name
    return np.zeros((len(t_grid), 4)), "none"


def _flight_state(log, t0_us: int, t_grid: np.ndarray) -> np.ndarray:
    """Flight state on the grid (zero-order hold): flight_phase (400 Hz) if
    present, else status (1 Hz).
    格子上の飛行状態（零次ホールド）: flight_phase（400 Hz）があればそれ、
    無ければ status（1 Hz）。"""
    for name in ("flight_phase", "status"):
        df = log.streams.get(name)
        if df is not None and not df.empty and "flight_state" in df.columns:
            t = _seconds(df, t0_us)
            vals = df["flight_state"].to_numpy(dtype=float)
            vals = np.where(np.isfinite(vals), vals, STATE_NONE)
            return _hold_previous(t, vals.astype(int), t_grid, STATE_NONE)
    return np.full(len(t_grid), STATE_NONE, dtype=int)


def _flight_mode(log, t0_us: int, t_grid: np.ndarray) -> np.ndarray:
    """Flight mode on the grid (zero-order hold of ctrl_ref.flight_mode).
    格子上の飛行モード（ctrl_ref.flight_mode の零次ホールド）。"""
    df = log.streams.get("ctrl_ref")
    if df is None or df.empty or "flight_mode" not in df.columns:
        return np.full(len(t_grid), MODE_NONE, dtype=int)
    vals = df["flight_mode"].to_numpy(dtype=float)
    vals = np.where(np.isfinite(vals), vals, MODE_NONE).astype(int)
    return _hold_previous(_seconds(df, t0_us), vals, t_grid, MODE_NONE)


def _voltage(log, t0_us: int, t_grid: np.ndarray) -> np.ndarray:
    """Battery voltage [V] on the grid: the status sample (about 1 Hz) held
    until the next sample arrives -- no smoothing, no invented values in
    between. NaN before the first sample or when status is absent.
    格子上の電池電圧 [V]: status のサンプル（約 1 Hz）を次のサンプルまで保持
    する（平滑化も、サンプル間の作り値もしない）。最初のサンプルより前や
    status が無いときは NaN。"""
    df = log.streams.get("status")
    if df is None or df.empty or "voltage" not in df.columns:
        return np.full(len(t_grid), np.nan)
    t = _seconds(df, t0_us)
    return _hold_previous(t, df["voltage"].to_numpy(dtype=float), t_grid, np.nan)


def _setpoint(log, t0_us: int, start_s: float, end_s: float, takeoff_ned: np.ndarray) -> dict:
    """Target point of the position hold. While POS_HOLD the ctrl_ref
    pos_setpoint (N, E) and alt_setpoint are averaged over the window;
    otherwise the takeoff point is used.
    位置保持の目標点。POS_HOLD 中の ctrl_ref の pos_setpoint (N, E) と
    alt_setpoint を窓内で平均する。無ければ離陸点を使う。"""
    df = log.streams.get("ctrl_ref")
    need = {"flight_mode", "pos_setpoint_x", "pos_setpoint_y", "alt_setpoint"}
    if df is not None and not df.empty and need <= set(df.columns):
        t = _seconds(df, t0_us)
        sel = (df["flight_mode"].to_numpy() == 3) & (t >= start_s) & (t <= end_s)
        if sel.any():
            return {
                "ned": [float(np.nanmedian(df["pos_setpoint_x"].to_numpy()[sel])),
                        float(np.nanmedian(df["pos_setpoint_y"].to_numpy()[sel]))],
                "h": float(np.nanmedian(df["alt_setpoint"].to_numpy()[sel])),
                "source": "ctrl_ref",
            }
    return {"ned": [float(takeoff_ned[0]), float(takeoff_ned[1])],
            "h": float(-takeoff_ned[2]), "source": "takeoff_point"}


# --------------------------------------------------------------- main entry
def frame_times(start_s: float, end_s: float, fps: float, speed: float) -> np.ndarray:
    """Frame times t_k = start + k*speed/fps for every frame inside the
    window (k = 0 .. floor((end-start)*fps/speed)).
    窓内の全コマの時刻 t_k = start + k*speed/fps（k = 0 .. floor((end-start)*fps/speed)）。"""
    n = int(math.floor((end_s - start_s) * fps / speed + 1e-9)) + 1
    return start_s + np.arange(max(n, 1)) * speed / fps


def build_replay_data(
    log,
    *,
    fps: float = 30.0,
    speed: float = 1.0,
    time_range: Optional[Sequence[float]] = None,
    source_name: str = "",
) -> dict:
    """FlightLog -> JSON-able dict on the video frame grid (see module doc).
    FlightLog -> 動画コマ格子上の JSON 化できる辞書（モジュール説明を参照）。

    Raises ValueError when the bundle has no `attitude` or no `posvel`.
    `attitude` か `posvel` が無い一式は ValueError。
    """
    for need in ("attitude", "posvel"):
        if need not in log.streams or log.streams[need].empty:
            raise ValueError(
                f"the log has no '{need}' stream, so the vehicle pose cannot be drawn")
    if fps <= 0 or speed <= 0:
        raise ValueError("fps and speed must be positive")

    t0_us = bundle_t0_us(log)
    att, pv = log.streams["attitude"], log.streams["posvel"]
    t_att, t_pv = _seconds(att, t0_us), _seconds(pv, t0_us)
    k_att, k_pv = _increasing(t_att), _increasing(t_pv)
    t_att, t_pv = t_att[k_att], t_pv[k_pv]
    data_end = float(min(t_att[-1], t_pv[-1]))
    data_begin = float(max(t_att[0], t_pv[0]))

    notes = []
    if time_range is not None:
        start_s, end_s = float(time_range[0]), float(time_range[1])
        if end_s <= start_s:
            raise ValueError("--time-range END must be larger than START")
        start_s, end_s = max(start_s, data_begin), min(end_s, data_end)
        notes.append("window: --time-range")
    else:
        start_s, end_s, note = default_window(log, t0_us, data_end)
        start_s = max(start_s, data_begin)
        notes.append(note)
    if end_s <= start_s:
        raise ValueError("the chosen time window holds no pose data")

    t = frame_times(start_s, end_s, fps, speed)
    pos = _interp_columns(
        t_pv, pv[["pos_x", "pos_y", "pos_z"]].to_numpy(dtype=float)[k_pv], t)
    quat = nlerp_quaternions(
        t_att, att[["quat_w", "quat_x", "quat_y", "quat_z"]].to_numpy(dtype=float)[k_att], t)
    euler = euler_zyx_deg(quat)
    motor, motor_source = _motor_duties(log, t0_us, t)
    notes.append(f"motor duties from {motor_source}")
    setpoint = _setpoint(log, t0_us, start_s, end_s, pos[0])

    def r(a):
        return np.round(np.asarray(a, dtype=float), ROUND_DIGITS)

    voltage = _voltage(log, t0_us, t)
    return {
        "t": r(t).tolist(),
        "pos": r(pos).tolist(),
        "quat": r(quat).tolist(),
        "motor": r(motor).tolist(),
        "h": r(-pos[:, 2]).tolist(),
        "roll": r(euler[:, 0]).tolist(),
        "pitch": r(euler[:, 1]).tolist(),
        "yaw": r(euler[:, 2]).tolist(),
        "mode": _flight_mode(log, t0_us, t).tolist(),
        "flight_state": _flight_state(log, t0_us, t).tolist(),
        "voltage": [None if not math.isfinite(v) else round(float(v), 4) for v in voltage],
        "setpoint": setpoint,
        "meta": {
            "source": source_name,
            "start": round(start_s, ROUND_DIGITS),
            "end": round(end_s, ROUND_DIGITS),
            "fps": fps,
            "speed": speed,
            "n_frames": int(len(t)),
            "motor_order": MOTOR_ORDER,
            "mode_names": FLIGHT_MODE_NAMES,
            "flight_state_names": FLIGHT_STATE_NAMES,
            "notes": notes,
        },
    }
