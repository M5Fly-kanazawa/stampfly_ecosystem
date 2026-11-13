"""
stats.py - Checkout summaries of a flight-log bundle (the implementation behind `sf log stats`).
stats.py - フライトログ一式の確認用要約（`sf log stats` の実体）。

Each mode reads a few streams of a bundle and answers one question of the hardware checkout
(firmware/vehicle/docs/hardware_checkout.md): how high the floor ToF reads while ARMED_GROUND, how much it
scatters, how the armed 400 Hz loop counters grew, when the motors stopped after LANDING, how far a turn went,
how the barometric height differs from the ToF, why the vehicle booted. A mode returns the numbers (a dict, for
tests and scripts) and the printable lines (a table with units). It states facts only: a pass/fail line belongs
to the procedure, not to this module.
各モードは一式の数本のストリームを読み、実機確認の 1 つの問いに答える（firmware/vehicle/docs/hardware_checkout.md）:
ARMED_GROUND 中の床の ToF の高さ、そのばらつき、ARM 中の 400 Hz ループのカウンタの増え方、LANDING の後にモータが
止まった時刻、回頭の大きさ、気圧高度と ToF の差、起動の理由。モードは数値（辞書。試験とスクリプト用）と表示用の行
（単位つきの表）を返す。事実だけを述べる。合否の線は手順書の側にあり、本モジュールには無い。

A missing stream raises StatsError naming the stream (an older firmware or a non-WiFi bundle lacks some).
無いストリームは、そのストリーム名を含む StatsError にする（旧ファームや WiFi 取得でない一式には無いものがある）。

@design docs/commands/sf-log.md (sf log stats); firmware/vehicle/docs/hardware_checkout.md
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import pandas as pd

#: Microseconds per second. / 1 秒あたりのマイクロ秒。
_US_PER_S = 1_000_000

#: FlightState values (components/sf_state/include/flight_state.hpp). / FlightState の値。
STATE_ARMED_GROUND = 3
STATE_LANDING = 6

#: ToF range status of a valid reading. / 有効な ToF の読みの status。
TOF_STATUS_VALID = 0

#: kTakeoffGroundBandM [m]: a valid floor reading above this is not "on the floor".
#: kTakeoffGroundBandM [m]: これを超える有効な床の読みは「床の上」と判定されない。
GROUND_BAND_M = 0.05
#: kLandingRestBandM [m]: the spread that counts a ToF run as at rest. / ToF の区間を静止とみなす幅。
REST_BAND_M = 0.02
#: kAutoTakeoffTargetAltitudeM [m]. / 自動離陸の目標高度。
DEFAULT_TARGET_M = 0.5
#: Half-width of the band around the target that counts as settled [m] (POS_HOLD hold accuracy, +-6-7 cm).
#: 目標の周りの整定とみなす半幅 [m]（POS_HOLD の保持精度 ±6〜7 cm）。
DEFAULT_SETTLE_BAND_M = 0.07
#: A motor duty below this counts as stopped. / これ未満の duty は停止とみなす。
DUTY_OFF = 1e-3
#: The last seconds before the motor stop in which invalid ToF samples are counted [s].
#: モータ停止の直前の、無効な ToF の標本を数える区間 [s]。
INVALID_LOOKBACK_S = 0.5
#: Maximum nearest-neighbour gap between a barometric sample and a ToF sample [s].
#: 気圧の標本と ToF の標本を対応づけてよい時刻差の上限 [s]。
BARO_TOF_PAIR_GAP_S = 0.1
#: Margin around the window in which ToF samples are read for the barometric comparison [s].
#: 気圧との比較で ToF の標本を読む、区間の前後の余白 [s]。
BARO_TOF_MARGIN_S = 1.0
#: Rows listed at most in a table (the rest is counted). / 表に並べる行数の上限（残りは件数で示す）。
MAX_LISTED_ROWS = 20

#: esp_reset_reason() values the vehicle records. / 機体が記録する esp_reset_reason() の値。
RESET_REASON_NAMES = {
    0: "UNKNOWN", 1: "POWERON", 2: "EXT", 3: "SW", 4: "PANIC", 5: "INT_WDT",
    6: "TASK_WDT", 7: "WDT", 8: "DEEPSLEEP", 9: "BROWNOUT", 10: "SDIO",
}
#: The reasons a healthy checkout shows: power on, and `reboot` (software reset).
#: 健全な実機確認で現れる理由: 電源投入と `reboot`（ソフトウェアリセット）。
EXPECTED_RESET_REASONS = (1, 3)

MODES = ("tof", "spread", "loop", "events", "turn", "baro", "boot")
#: Modes that need an explicit window. / 区間の指定が必須のモード。
WINDOW_REQUIRED_MODES = ("spread", "turn", "baro")


class StatsError(Exception):
    """A summary cannot be made (a stream is missing, the window is empty, ...); the message says why.
    要約を作れない（ストリームが無い、区間が空、など）。理由をメッセージに書く。
    """


@dataclass
class StatsResult:
    """The numbers (values) and the printable lines (lines) of one mode.
    1 つのモードの数値（values）と表示用の行（lines）。
    """

    values: dict = field(default_factory=dict)
    lines: list = field(default_factory=list)


# =============================================================================
# helpers
# =============================================================================


def _stream(log, name: str) -> pd.DataFrame:
    """The stream's rows, sorted by time; StatsError naming the stream when it is absent or empty.
    ストリームの行（時刻順）。無い・空なら、そのストリーム名を含む StatsError。
    """
    if name not in log.streams or len(log.streams[name]) == 0:
        raise StatsError(
            f"stream '{name}' is missing or empty in this bundle (an older firmware, or a capture "
            f"that did not receive it)"
        )
    return log.streams[name].sort_values("timestamp_us", kind="stable").reset_index(drop=True)


def _seconds(df: pd.DataFrame) -> np.ndarray:
    """Vehicle time of each row [s]. / 各行の機体時刻 [s]。"""
    return df["timestamp_us"].to_numpy(dtype=float) / _US_PER_S


def _in_window(seconds: np.ndarray, since: Optional[float], until: Optional[float]) -> np.ndarray:
    """Mask of the rows with since <= t <= until (an open end when None). / 区間内の行のマスク。"""
    mask = np.ones(len(seconds), dtype=bool)
    if since is not None:
        mask &= seconds >= since
    if until is not None:
        mask &= seconds <= until
    return mask


def _need_window(mode: str, since: Optional[float], until: Optional[float]) -> None:
    """StatsError when a mode that needs a window gets none. / 区間が必須のモードで未指定なら StatsError。"""
    if mode in WINDOW_REQUIRED_MODES and (since is None or until is None):
        raise StatsError(
            f"mode '{mode}' needs both --since and --until (vehicle time in seconds since boot; "
            f"the time stamps of `sf log info <bundle> --text`)"
        )
    if since is not None and until is not None and until <= since:
        raise StatsError(f"--until ({until}) must be greater than --since ({since})")


def _valid_tof(log, since: Optional[float], until: Optional[float]):
    """(time [s], distance [m]) of the valid downward-ToF samples in the window.
    区間内の有効な下向き ToF の標本の（時刻 [s]、距離 [m]）。
    """
    tof = _stream(log, "tof_bottom")
    seconds = _seconds(tof)
    mask = _in_window(seconds, since, until) & (tof["status"].to_numpy() == TOF_STATUS_VALID)
    return seconds[mask], tof["distance"].to_numpy(dtype=float)[mask]


def _state_runs(log, state: int):
    """[(start_s, end_s)] of the runs where flight_phase.flight_state == state. A run ends at the first row
    with another state; a run that lasts to the end of the stream ends at its last row.
    flight_phase.flight_state == state が続く区間の [(開始 s, 終了 s)]。区間は別の状態の最初の行で終わる。
    ストリームの終わりまで続く区間は最後の行で終わる。
    """
    phase = _stream(log, "flight_phase")
    seconds = _seconds(phase)
    active = phase["flight_state"].to_numpy() == state
    runs = []
    start = None
    for t, is_active in zip(seconds, active):
        if is_active and start is None:
            start = t
        elif not is_active and start is not None:
            runs.append((start, t))
            start = None
    if start is not None:
        runs.append((start, seconds[-1]))
    return runs


def _fmt(value: float, digits: int = 1) -> str:
    return f"{value:.{digits}f}"


# =============================================================================
# mode: tof
# =============================================================================


def stats_tof(log, since=None, until=None) -> StatsResult:
    """Floor ToF while ARMED_GROUND: valid minimum and maximum [cm], the readings above the ground band.
    ARMED_GROUND 中の床の ToF: 有効な読みの最小・最大 [cm] と、床の帯を超えた読み。
    """
    tof = _stream(log, "tof_bottom")
    runs = [
        (max(a, since) if since is not None else a, min(b, until) if until is not None else b)
        for a, b in _state_runs(log, STATE_ARMED_GROUND)
    ]
    runs = [(a, b) for a, b in runs if b > a]
    if not runs:
        raise StatsError(
            f"stream 'flight_phase' never shows ARMED_GROUND (flight_state {STATE_ARMED_GROUND}) in the window"
        )
    seconds = _seconds(tof)
    inside = np.zeros(len(seconds), dtype=bool)
    for a, b in runs:
        inside |= (seconds >= a) & (seconds < b)
    distance = tof["distance"].to_numpy(dtype=float)[inside]
    valid = tof["status"].to_numpy()[inside] == TOF_STATUS_VALID
    armed_s = sum(b - a for a, b in runs)
    values = {
        "armed_s": armed_s, "samples": int(inside.sum()), "valid": int(valid.sum()),
        "invalid": int((~valid).sum()), "above_band": 0,
        "valid_min_cm": None, "valid_max_cm": None, "above_list": [],
    }
    lines = [
        "Floor ToF while ARMED_GROUND",
        f"  ARMED_GROUND time       : {_fmt(armed_s)} s in {len(runs)} run(s), from {_fmt(runs[0][0])} s",
        f"  ToF samples             : {values['samples']} (valid {values['valid']}, invalid {values['invalid']})",
    ]
    if valid.any():
        good = distance[valid]
        times = seconds[inside][valid]
        above = [(float(t), float(d)) for t, d in zip(times, good) if d > GROUND_BAND_M]
        values.update(valid_min_cm=float(good.min() * 100), valid_max_cm=float(good.max() * 100),
                      above_band=len(above), above_list=above)
        lines += [
            f"  valid minimum           : {_fmt(values['valid_min_cm'])} cm",
            f"  valid maximum           : {_fmt(values['valid_max_cm'])} cm",
            f"  valid above {GROUND_BAND_M * 100:.0f} cm        : {len(above)}",
        ]
        for t, d in above[:MAX_LISTED_ROWS]:
            lines.append(f"    {_fmt(t, 3):>10s} s  {_fmt(d * 100)} cm")
        if len(above) > MAX_LISTED_ROWS:
            lines.append(f"    ... {len(above) - MAX_LISTED_ROWS} more")
    else:
        lines.append("  valid samples           : none (every reading was invalid: the floor is in the dead zone)")
    return StatsResult(values, lines)


# =============================================================================
# mode: spread
# =============================================================================


def stats_spread(log, since, until) -> StatsResult:
    """Scatter (maximum - minimum) of the valid ToF samples in each 1 s window from `since` [cm].
    `since` から 1 秒ごとの窓での、有効な ToF の標本のばらつき（最大 - 最小）[cm]。
    """
    seconds, distance = _valid_tof(log, since, until)
    rows = []
    for k in range(int(until - since)):
        lo = since + k
        window = distance[(seconds >= lo) & (seconds < lo + 1)]
        if len(window) >= 2:
            rows.append((lo, len(window), float((window.max() - window.min()) * 100)))
    if not rows:
        raise StatsError("no 1 s window of the given range holds two valid ToF samples")
    worst = max(r[2] for r in rows)
    lines = ["ToF scatter in 1 s windows", f"  {'window start':>12s}  {'samples':>7s}  {'spread':>9s}"]
    for lo, n, spread in rows[:MAX_LISTED_ROWS]:
        lines.append(f"  {_fmt(lo):>10s} s  {n:>7d}  {_fmt(spread):>6s} cm")
    if len(rows) > MAX_LISTED_ROWS:
        lines.append(f"  ... {len(rows) - MAX_LISTED_ROWS} more windows")
    lines.append(f"  largest 1 s spread      : {_fmt(worst)} cm (rest band {REST_BAND_M * 100:.0f} cm)")
    return StatsResult({"windows": rows, "largest_cm": worst}, lines)


# =============================================================================
# mode: loop
# =============================================================================

#: The armed counters whose growth the `loop` table shows. / `loop` の表が増加を示す ARM 中のカウンタ。
_ARMED_COUNTERS = ("late_count", "over_budget_count")


def stats_loop(log, since=None, until=None) -> StatsResult:
    """Armed 400 Hz loop counters (how they grew, their last values), the non-finite count, the log-path losses.
    ARM 中の 400 Hz ループのカウンタ（増え方と最後の値）、有限でない数の数え、ログ経路の欠落。
    """
    loop = _stream(log, "loop_timing")
    loop = loop[_in_window(_seconds(loop), since, until)]
    nonfinite = _stream(log, "nonfinite")
    nonfinite = nonfinite[_in_window(_seconds(nonfinite), since, until)]
    link = _stream(log, "link_health")
    link = link[_in_window(_seconds(link), since, until)]
    if len(loop) == 0 or len(nonfinite) == 0 or len(link) == 0:
        raise StatsError("no row of loop_timing / nonfinite / link_health lies in the window")

    keys = [f"{task}_armed_{c}" for task in ("imu", "control") for c in _ARMED_COUNTERS]
    changes = []
    previous = None
    for _, row in loop.iterrows():
        now = tuple(int(row[k]) for k in keys)
        if now != previous and any(now):
            changes.append((row["timestamp_us"] / _US_PER_S, now))
        previous = now

    values = {"changes": len(changes)}
    lines = ["Armed 400 Hz loop (counters are totals since boot)"]
    if changes:
        lines.append(f"  rows where an armed counter changed ({len(changes)}):")
        lines.append(f"  {'time':>10s}  " + "  ".join(f"{k.replace('_armed_', ' ').replace('_count', ''):>20s}" for k in keys))
        for t, now in changes[:MAX_LISTED_ROWS]:
            lines.append(f"  {_fmt(t):>8s} s  " + "  ".join(f"{n:>20d}" for n in now))
        if len(changes) > MAX_LISTED_ROWS:
            lines.append(f"  ... {len(changes) - MAX_LISTED_ROWS} more")
    else:
        lines.append("  no armed counter changed (late and over budget stayed 0 while ARMed)")
    last = loop.iloc[-1]
    for task in ("imu", "control"):
        entry = {c: int(last[f"{task}_armed_{c}"]) for c in
                 ("cycles", "late_count", "over_budget_count", "period_max_us", "busy_max_us")}
        values[task] = entry
        lines.append(
            f"  {task:<8s} armed: cycles {entry['cycles']}, late {entry['late_count']}, "
            f"over budget {entry['over_budget_count']}, period max {entry['period_max_us'] / 1000:.2f} ms, "
            f"busy max {entry['busy_max_us'] / 1000:.2f} ms"
        )
    detected = int(nonfinite.iloc[-1]["detected_cycles"])
    checked = int(nonfinite.iloc[-1]["checked_cycles"])
    first = nonfinite[nonfinite["detected_cycles"] > 0]
    values.update(nonfinite_detected=detected, nonfinite_checked=checked)
    lines.append(
        f"  non-finite              : detected {detected} of {checked} armed cycles"
        + (f", first at {_fmt(first.iloc[0]['timestamp_us'] / _US_PER_S)} s" if len(first) else "")
    )
    last_link = link.iloc[-1]
    paths = {k: int(last_link[k]) for k in ("log_stream_overflow", "send_drops", "entry_drops", "text_dropped_lines")}
    values["log_paths"] = paths
    lines.append("  log paths               : " + ", ".join(f"{k}={v}" for k, v in paths.items()))
    lines += _loop_optional_lines(log, since, until, values)
    return StatsResult(values, lines)


def _loop_optional_lines(log, since, until, values) -> list:
    """The lines of the `loop` mode that come from optional streams (text_log, tof_silence).
    `loop` モードのうち、任意のストリーム（text_log、tof_silence）から作る行。
    """
    lines = []
    if "text_log" in log.streams and len(log.streams["text_log"]) > 0:
        text = log.streams["text_log"]
        text = text[_in_window(_seconds(text), since, until)]
        lost = int(text["dropped_before"].fillna(0).sum())
        values["text_lines_dropped"] = lost
        lines.append(f"  text log                : {len(text)} lines, {lost} dropped before a line (loss markers)")
    else:
        values["text_lines_dropped"] = None
        lines.append("  text log                : no text_log stream in this bundle")
    if "tof_silence" in log.streams and len(log.streams["tof_silence"]) > 0:
        silence = log.streams["tof_silence"]
        silence = silence[_in_window(_seconds(silence), since, until)]
        silent = int((silence["tof_silent"] == 1).sum())
        values["tof_silent_rows"] = silent
        lines.append(f"  ToF output stopped      : tof_silent=1 in {silent} of {len(silence)} rows")
    else:
        values["tof_silent_rows"] = None
        lines.append("  ToF output stopped      : no tof_silence stream in this bundle")
    return lines


# =============================================================================
# mode: events
# =============================================================================


def stats_events(log, since=None, until=None, target_m=DEFAULT_TARGET_M, band_m=DEFAULT_SETTLE_BAND_M) -> StatsResult:
    """Peak ToF height, the last time outside target +- band after the peak, and (after LANDING starts) the
    time until all motors stopped and the last valid ToF before the stop.
    最高到達の ToF の高さ、ピーク後に target ± band の外にあった最後の時刻、そして（LANDING が始まった後の）全モータが
    止まるまでの時間と、停止の前の最後の有効な ToF。
    """
    seconds, distance = _valid_tof(log, since, until)
    if len(seconds) == 0:
        raise StatsError("stream 'tof_bottom' has no valid sample in the window")
    peak = int(np.argmax(distance))
    values = {"peak_m": float(distance[peak]), "peak_s": float(seconds[peak])}
    lines = ["Flight events", f"  peak ToF height         : {values['peak_m']:.3f} m at {values['peak_s']:.3f} s"]

    runs = [(a, b) for a, b in _state_runs(log, STATE_LANDING) if (since is None or a >= since) and (until is None or a <= until)]
    landings = [a for a, _ in runs]
    end = landings[0] if landings else seconds[-1]
    outside = seconds[(seconds >= seconds[peak]) & (seconds <= end) & (np.abs(distance - target_m) > band_m)]
    values["last_outside_s"] = float(outside[-1]) if len(outside) else None
    where = f"after the peak, before {'landing' if landings else 'the end'}"
    lines.append(
        f"  last time outside {target_m:.2f} +- {band_m:.2f} m : "
        + (f"{outside[-1]:.3f} s ({where})" if len(outside) else f"never ({where}: inside the band from the peak on)")
    )
    if not landings:
        lines.append("  LANDING                 : never started in the window")
        return StatsResult(values, lines)
    values["landing_end_s"] = float(runs[0][1])
    lines += _landing_lines(log, landings[0], runs[0][1], until, values)
    return StatsResult(values, lines)


def _landing_lines(log, landing_s: float, landing_end_s: float, until: Optional[float], values: dict) -> list:
    """Lines about the motor stop after the LANDING start at `landing_s` (the stop time, the last valid ToF).
    `landing_s` の LANDING 開始の後のモータ停止についての行（停止の時刻、最後の有効な ToF）。
    A bundle that records the motors only while ARMed (a SILS bundle) has no all-zero row after the stop; the
    line then says so and gives the time the LANDING state ended instead.
    ARM 中だけモータを記録する一式（SILS の一式）には、停止の後の全 duty 0 の行が無い。その場合はそう述べ、代わりに
    LANDING の状態が終わった時刻を示す。
    """
    motor = _stream(log, "motor")
    motor_s = _seconds(motor)
    duties = motor[["duty_FR", "duty_RR", "duty_RL", "duty_FL"]].to_numpy(dtype=float)
    stopped = (motor_s >= landing_s) & np.all(duties < DUTY_OFF, axis=1)
    if until is not None:
        stopped &= motor_s <= until
    values["landing_s"] = float(landing_s)
    if not stopped.any():
        values["stop_after_s"] = None
        return [
            f"  LANDING                 : started at {landing_s:.3f} s; no motor row with all duties 0 after it "
            f"(the motor stream ends at {motor_s[-1]:.3f} s)",
            f"  LANDING state ended     : at {landing_end_s:.3f} s, {landing_end_s - landing_s:.3f} s after the start",
        ]
    stop_s = float(motor_s[stopped][0])
    values["stop_after_s"] = stop_s - landing_s
    lines = [f"  LANDING                 : started at {landing_s:.3f} s; all motors stopped {stop_s - landing_s:.3f} s later"]
    tof = _stream(log, "tof_bottom")
    tof_s = _seconds(tof)
    between = (tof_s >= landing_s) & (tof_s <= stop_s)
    valid = between & (tof["status"].to_numpy() == TOF_STATUS_VALID)
    if valid.any():
        index = np.flatnonzero(valid)[-1]
        invalid_late = int((between & ~valid & (tof_s >= stop_s - INVALID_LOOKBACK_S)).sum())
        values.update(last_valid_cm=float(tof["distance"].iloc[index] * 100),
                      last_valid_before_stop_s=float(stop_s - tof_s[index]), invalid_last_half_s=invalid_late)
        lines.append(
            f"  last valid ToF before stop: {values['last_valid_cm']:.1f} cm, "
            f"{values['last_valid_before_stop_s']:.3f} s before the stop; "
            f"invalid samples in the last {INVALID_LOOKBACK_S} s: {invalid_late}"
        )
    else:
        values["last_valid_cm"] = None
        lines.append("  last valid ToF before stop: none (no valid sample between the LANDING start and the stop)")
    return lines


# =============================================================================
# mode: turn
# =============================================================================


def _yaw_deg(attitude: pd.DataFrame) -> np.ndarray:
    """Yaw angle [deg] of each attitude quaternion (w, x, y, z), in (-180, 180].
    各姿勢クォータニオン (w, x, y, z) のヨー角 [deg]。(-180, 180] の範囲。
    """
    w, x, y, z = (attitude[f"quat_{c}"].to_numpy(dtype=float) for c in "wxyz")
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def stats_turn(log, since, until) -> StatsResult:
    """Heading change (unwrapped, signed), ToF height range and horizontal drift of a window (a turn).
    区間（回頭）の向きの変化（巻き戻しなし・符号つき）、ToF の高さの範囲、水平の流れ。
    """
    attitude = _stream(log, "attitude")
    attitude = attitude[_in_window(_seconds(attitude), since, until)]
    if len(attitude) < 2:
        raise StatsError("stream 'attitude' has fewer than 2 samples in the window")
    yaw = _yaw_deg(attitude)
    steps = (np.diff(yaw) + 180.0) % 360.0 - 180.0
    path = np.concatenate([[0.0], np.cumsum(steps)])
    values = {"yaw_change_deg": float(path[-1]), "yaw_min_deg": float(path.min()), "yaw_max_deg": float(path.max())}
    lines = [
        "Turn in the window",
        f"  yaw change              : {path[-1]:+.1f} deg (min {path.min():+.1f}, max {path.max():+.1f}); "
        f"positive = clockwise seen from above",
    ]
    _, height = _valid_tof(log, since, until)
    if len(height):
        values.update(tof_min_m=float(height.min()), tof_max_m=float(height.max()))
        lines.append(f"  ToF height              : min {height.min():.3f} m, max {height.max():.3f} m")
    else:
        values.update(tof_min_m=None, tof_max_m=None)
        lines.append("  ToF height              : no valid sample in the window")
    posvel = _stream(log, "posvel")
    posvel = posvel[_in_window(_seconds(posvel), since, until)]
    if len(posvel):
        x = posvel["pos_x"].to_numpy(dtype=float)
        y = posvel["pos_y"].to_numpy(dtype=float)
        drift = float(np.max(np.hypot(x - x[0], y - y[0])))
        values["drift_m"] = drift
        lines.append(f"  horizontal drift        : max {drift:.3f} m from the first position (ESKF estimate)")
    else:
        values["drift_m"] = None
        lines.append("  horizontal drift        : no posvel sample in the window")
    return StatsResult(values, lines)


# =============================================================================
# mode: baro
# =============================================================================


def stats_baro(log, since, until) -> StatsResult:
    """Barometric floor-relative height minus the ToF height (rms, maximum), and the learned offset.
    気圧から求めた床基準の高さと ToF の高さの差（rms・最大）と、学習したオフセット。
    """
    baro = _stream(log, "baro_height")
    tof_s, tof_d = _valid_tof(log, since - BARO_TOF_MARGIN_S, until + BARO_TOF_MARGIN_S)
    baro_s = _seconds(baro)
    inside = _in_window(baro_s, since, until) & (baro["valid"].to_numpy() == 1)
    errors = []
    if len(tof_s):
        for t, h in zip(baro_s[inside], baro["height"].to_numpy(dtype=float)[inside]):
            nearest = int(np.argmin(np.abs(tof_s - t)))
            if abs(tof_s[nearest] - t) < BARO_TOF_PAIR_GAP_S:
                errors.append(float(h - tof_d[nearest]))
    learned = _in_window(baro_s, since, until) & (baro["learned"].to_numpy() == 1)
    offsets = baro["offset"].to_numpy(dtype=float)[learned]
    ages = baro["offset_age"].to_numpy(dtype=float)[learned]
    values = {"pairs": len(errors), "rms_m": None, "max_abs_m": None,
              "offset_min_m": None, "offset_max_m": None, "age_max_s": None}
    lines = ["Barometric height against the ToF (observation only)"]
    if errors:
        e = np.array(errors)
        values.update(rms_m=float(np.sqrt(np.mean(e * e))), max_abs_m=float(np.max(np.abs(e))))
        lines.append(f"  baro height - ToF       : rms {values['rms_m']:.3f} m, max {values['max_abs_m']:.3f} m, {len(e)} pairs")
    else:
        lines.append("  baro height - ToF       : no valid pair in the window (valid baro sample within 0.1 s of a valid ToF sample)")
    if len(offsets):
        values.update(offset_min_m=float(offsets.min()), offset_max_m=float(offsets.max()), age_max_s=float(ages.max()))
        lines.append(
            f"  learned offset          : min {offsets.min():.2f} m, max {offsets.max():.2f} m, "
            f"age max {ages.max():.1f} s"
        )
    else:
        lines.append("  learned offset          : never learned in the window")
    return StatsResult(values, lines)


# =============================================================================
# mode: boot
# =============================================================================


def stats_boot(log, since=None, until=None) -> StatsResult:
    """Reset reasons seen in status.csv and the battery voltage range.
    status.csv に現れたリセット理由と、電池電圧の範囲。
    """
    status = _stream(log, "status")
    status = status[_in_window(_seconds(status), since, until)]
    if len(status) == 0:
        raise StatsError("stream 'status' has no row in the window")
    reasons = sorted({int(r) for r in status["reset_reason"].dropna()})
    unexpected = [r for r in reasons if r not in EXPECTED_RESET_REASONS]
    volts = status["voltage"].dropna()
    values = {"reasons": reasons, "unexpected": unexpected,
              "voltage_min_v": float(volts.min()) if len(volts) else None,
              "voltage_max_v": float(volts.max()) if len(volts) else None}
    names = ", ".join(f"{r} {RESET_REASON_NAMES.get(r, '?')}" for r in reasons)
    lines = [
        "Boot and battery",
        f"  reset reasons           : {names}",
        "  unexpected reasons      : "
        + (", ".join(f"{r} {RESET_REASON_NAMES.get(r, '?')}" for r in unexpected) if unexpected
           else "none (only 1 POWERON and 3 SW)"),
    ]
    if len(volts):
        lines.append(f"  battery voltage         : min {volts.min():.2f} V, max {volts.max():.2f} V")
    return StatsResult(values, lines)


# =============================================================================
# entry point
# =============================================================================


def compute(log, mode: str, since=None, until=None, target_m=DEFAULT_TARGET_M, band_m=DEFAULT_SETTLE_BAND_M) -> StatsResult:
    """Run one mode on a loaded bundle. Raises StatsError when it cannot be answered.
    読み込んだ一式に 1 つのモードを実行する。答えられなければ StatsError。
    """
    if mode not in MODES:
        raise StatsError(f"unknown mode '{mode}' (choose from {', '.join(MODES)})")
    _need_window(mode, since, until)
    if mode == "tof":
        return stats_tof(log, since, until)
    if mode == "spread":
        return stats_spread(log, since, until)
    if mode == "loop":
        return stats_loop(log, since, until)
    if mode == "events":
        return stats_events(log, since, until, target_m, band_m)
    if mode == "turn":
        return stats_turn(log, since, until)
    if mode == "baro":
        return stats_baro(log, since, until)
    return stats_boot(log, since, until)
