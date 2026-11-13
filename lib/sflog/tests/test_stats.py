"""
test_stats.py - sflog.stats (the implementation behind `sf log stats`): every mode against a hand-computed case
sflog.stats（`sf log stats` の実体）の試験: 各モードを手計算した事例で確かめる

Each test builds a tiny synthetic bundle whose answer is worked out by hand (the arithmetic is written next to
the data), runs the mode and compares the numbers. The missing-stream and window rules are tested per mode.
各試験は答えを手で計算した小さな合成の一式を作り、モードを実行して数値を比べる（計算はデータの隣に書く）。
ストリームが無い場合と区間の規則も、モードごとに試験する。
"""

import math

import pandas as pd
import pytest

from sflog import stats
from sflog.bundle import FlightLog
from sflog.stats import StatsError


def _frame(columns, rows):
    return pd.DataFrame(rows, columns=columns)


def _tof(rows):
    """rows: (time [s], distance [m], status) -> tof_bottom. / 行: (時刻 [s], 距離 [m], status)。"""
    return _frame(["timestamp_us", "distance", "status"], [(round(t * 1e6), d, s) for t, d, s in rows])


def _phase(rows):
    """rows: (time [s], flight_state) -> flight_phase. / 行: (時刻 [s], flight_state)。"""
    return _frame(["timestamp_us", "flight_state"], [(round(t * 1e6), s) for t, s in rows])


def _log(**streams):
    return FlightLog(streams=streams)


# ---------------------------------------------------------------- tof


def test_tof_counts_only_the_armed_ground_run_and_lists_the_reading_above_the_band():
    # ARMED_GROUND (state 3) from 2 s to 5 s (the first row of another state ends it, exclusive): 3.0 s.
    # Inside [2, 5): 2.0 s 0.032 m, 2.5 s 0.034 m, 3.0 s invalid (254), 3.5 s 0.052 m, 4.0 s 0.031 m.
    # 1.5 s (before) and 5.0 s (the end) are outside. -> 5 samples, 4 valid, 1 invalid, min 3.1 cm, max 5.2 cm,
    # one above 5 cm: 5.2 cm at 3.5 s.
    log = _log(
        flight_phase=_phase([(1, 2), (2, 3), (3, 3), (4, 3), (5, 6), (6, 6)]),
        tof_bottom=_tof([(1.5, 0.04, 0), (2.0, 0.032, 0), (2.5, 0.034, 0), (3.0, 0.0, 254),
                         (3.5, 0.052, 0), (4.0, 0.031, 0), (5.0, 0.5, 0)]),
    )
    result = stats.stats_tof(log)
    v = result.values
    assert v["armed_s"] == pytest.approx(3.0)
    assert (v["samples"], v["valid"], v["invalid"]) == (5, 4, 1)
    assert v["valid_min_cm"] == pytest.approx(3.1) and v["valid_max_cm"] == pytest.approx(5.2)
    assert v["above_band"] == 1 and v["above_list"] == [(3.5, pytest.approx(0.052))]
    text = "\n".join(result.lines)
    assert "valid minimum           : 3.1 cm" in text and "valid above 5 cm        : 1" in text
    assert "3.500 s  5.2 cm" in text


def test_tof_window_cuts_the_run_and_all_invalid_is_reported():
    log = _log(flight_phase=_phase([(1, 3), (9, 6)]), tof_bottom=_tof([(2, 0.0, 254), (3, 0.0, 254), (8, 0.04, 0)]))
    result = stats.stats_tof(log, since=1.5, until=4)     # run cut to [1.5, 4): 2.5 s; only the two invalid rows
    assert result.values["armed_s"] == pytest.approx(2.5) and result.values["valid"] == 0
    assert "none (every reading was invalid" in "\n".join(result.lines)


def test_tof_without_armed_ground_names_flight_phase():
    log = _log(flight_phase=_phase([(1, 2), (2, 2)]), tof_bottom=_tof([(1, 0.03, 0)]))
    with pytest.raises(StatsError, match="flight_phase.*ARMED_GROUND"):
        stats.stats_tof(log)


# ---------------------------------------------------------------- spread


def test_spread_is_max_minus_min_of_the_valid_samples_per_1s_window():
    # since 10, until 12: windows [10,11) and [11,12).
    # [10,11): 0.030, 0.032, 0.036 (the 10.9 s one) -> spread 0.6 cm, n = 3 (the invalid 10.95 s sample is skipped).
    # [11,12): 0.050 (11.2 s), 0.0524 (11.6 s) -> spread 0.24 cm, n = 2; 11.5 s is invalid and ignored.
    log = _log(tof_bottom=_tof([(10.0, 0.030, 0), (10.4, 0.032, 0), (10.9, 0.036, 0), (10.95, 0.0, 254),
                                (11.2, 0.050, 0), (11.5, 0.0, 254), (11.6, 0.0524, 0)]))
    result = stats.stats_spread(log, 10, 12)
    assert [(lo, n) for lo, n, _ in result.values["windows"]] == [(10, 3), (11, 2)]
    assert [s for _, _, s in result.values["windows"]] == [pytest.approx(0.6), pytest.approx(0.24)]
    assert result.values["largest_cm"] == pytest.approx(0.6)
    assert "largest 1 s spread      : 0.6 cm (rest band 2 cm)" in "\n".join(result.lines)


def test_spread_with_no_pair_is_an_error():
    log = _log(tof_bottom=_tof([(10.0, 0.03, 0), (11.0, 0.03, 0)]))
    with pytest.raises(StatsError, match="two valid ToF samples"):
        stats.stats_spread(log, 10, 12)


# ---------------------------------------------------------------- loop


def _loop_log():
    columns = ["timestamp_us"] + [f"{t}_armed_{c}" for t in ("imu", "control")
                                  for c in ("cycles", "late_count", "over_budget_count", "period_max_us", "busy_max_us")]
    rows = [  # time, then imu (cycles, late, over, period max, busy max), control (...)
        (1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (2, 400, 0, 0, 2500, 100, 400, 0, 0, 2500, 90),
        (3, 800, 2, 0, 7500, 100, 800, 0, 0, 2500, 90),
        (4, 1200, 2, 0, 7500, 100, 1200, 0, 1, 11000, 2600),
    ]
    loop = _frame(columns, [(round(r[0] * 1e6),) + r[1:] for r in rows])
    nonfinite = _frame(["timestamp_us", "checked_cycles", "detected_cycles"],
                       [(1_000_000, 0, 0), (2_000_000, 400, 0), (3_000_000, 800, 5), (4_000_000, 1200, 5)])
    link = _frame(["timestamp_us", "log_stream_overflow", "send_drops", "entry_drops", "text_dropped_lines"],
                  [(1_000_000, 0, 0, 0, 0), (4_000_000, 0, 1, 0, 7)])
    return loop, nonfinite, link


def test_loop_lists_the_rows_where_an_armed_counter_changed_and_the_last_values():
    loop, nonfinite, link = _loop_log()
    text_log = _frame(["timestamp_us", "truncated", "dropped_before", "text"],
                      [(1_500_000, 0, 0, "a"), (2_500_000, 0, 3, "b"), (3_500_000, 0, 4, "c")])
    silence = _frame(["timestamp_us", "tof_silent"], [(1_000_000, 0), (2_000_000, 1), (3_000_000, 1), (4_000_000, 0)])
    result = stats.stats_loop(_log(loop_timing=loop, nonfinite=nonfinite, link_health=link,
                                   text_log=text_log, tof_silence=silence))
    v = result.values
    # Changes: row at 3 s (imu late 0 -> 2) and row at 4 s (control over budget 0 -> 1; imu late stays 2, so a
    # row where nothing changes is not listed). The 2 s row changed nothing nonzero. -> 2 listed rows.
    assert v["changes"] == 2
    assert v["imu"] == {"cycles": 1200, "late_count": 2, "over_budget_count": 0, "period_max_us": 7500, "busy_max_us": 100}
    assert v["control"]["over_budget_count"] == 1 and v["control"]["busy_max_us"] == 2600
    assert (v["nonfinite_detected"], v["nonfinite_checked"]) == (5, 1200)
    assert v["log_paths"] == {"log_stream_overflow": 0, "send_drops": 1, "entry_drops": 0, "text_dropped_lines": 7}
    assert v["text_lines_dropped"] == 7 and v["tof_silent_rows"] == 2      # 3 + 4 dropped; 2 of 4 rows silent
    text = "\n".join(result.lines)
    assert "imu      armed: cycles 1200, late 2, over budget 0, period max 7.50 ms, busy max 0.10 ms" in text
    assert "control  armed: cycles 1200, late 0, over budget 1, period max 11.00 ms, busy max 2.60 ms" in text
    assert "detected 5 of 1200 armed cycles, first at 3.0 s" in text
    assert "tof_silent=1 in 2 of 4 rows" in text


def test_loop_without_changes_and_without_optional_streams_says_so():
    loop, nonfinite, link = _loop_log()
    loop[[c for c in loop.columns if c != "timestamp_us"]] = 0
    result = stats.stats_loop(_log(loop_timing=loop, nonfinite=nonfinite, link_health=link))
    assert result.values["changes"] == 0
    text = "\n".join(result.lines)
    assert "no armed counter changed" in text
    assert "no text_log stream" in text and "no tof_silence stream" in text


@pytest.mark.parametrize("missing", ["loop_timing", "nonfinite", "link_health"])
def test_loop_names_the_missing_required_stream(missing):
    loop, nonfinite, link = _loop_log()
    streams = {"loop_timing": loop, "nonfinite": nonfinite, "link_health": link}
    del streams[missing]
    with pytest.raises(StatsError, match=f"stream '{missing}'"):
        stats.stats_loop(_log(**streams))


# ---------------------------------------------------------------- events


def _events_log(with_motor_zero=True):
    phase = _phase([(0, 3), (1, 4), (2, 5), (8, 6), (10, 0)])     # LANDING (6) from 8 s to 10 s
    tof = _tof([(1, 0.10, 0), (2, 0.30, 0), (3, 0.84, 0), (4, 0.60, 0), (5, 0.55, 0), (6, 0.50, 0),
                (7, 0.45, 0), (8.0, 0.50, 0), (8.5, 0.20, 0), (9.0, 0.04, 0), (9.2, 0.0, 254), (9.4, 0.0, 254)])
    times = [8.0 + 0.2 * k for k in range(11)]                   # 8.0 ... 10.0
    duty = [0.5 if (t < 9.5 or not with_motor_zero) else 0.0 for t in times]
    motor = _frame(["timestamp_us", "duty_FR", "duty_RR", "duty_RL", "duty_FL"],
                   [(round(t * 1e6), d, d, d, d) for t, d in zip(times, duty)])
    return _log(flight_phase=phase, tof_bottom=tof, motor=motor)


def test_events_peak_settling_and_motor_stop():
    result = stats.stats_events(_events_log())
    v = result.values
    # Peak 0.84 m at 3.0 s. After it, before LANDING (8.0 s): the readings outside 0.5 +- 0.07 are 0.60 (4 s);
    # 0.55 (|0.05|), 0.50, 0.45 (|0.05|) and the 8.0 s 0.50 are inside -> last outside = 4.0 s.
    assert v["peak_m"] == pytest.approx(0.84) and v["peak_s"] == pytest.approx(3.0)
    assert v["last_outside_s"] == pytest.approx(4.0)
    # LANDING starts at 8.0 s. The first all-zero motor row after it is 9.6 s (9.4 s is still 0.5; the rows are
    # 8.0, 8.2, ..., 9.4 -> 0.5, 9.6 -> 0): stop after 1.6 s. Valid ToF between 8.0 and 9.6 s: the last is 9.0 s,
    # 4.0 cm, 0.6 s before the stop. Invalid in the last 0.5 s (9.1 .. 9.6): 9.2 s and 9.4 s -> 2.
    assert v["landing_s"] == pytest.approx(8.0) and v["stop_after_s"] == pytest.approx(1.6)
    assert v["last_valid_cm"] == pytest.approx(4.0) and v["last_valid_before_stop_s"] == pytest.approx(0.6)
    assert v["invalid_last_half_s"] == 2
    assert "all motors stopped 1.600 s later" in "\n".join(result.lines)


def test_events_when_the_motor_stream_never_reaches_zero_says_so():
    result = stats.stats_events(_events_log(with_motor_zero=False))
    assert result.values["stop_after_s"] is None
    text = "\n".join(result.lines)
    assert "no motor row with all duties 0" in text and "LANDING state ended     : at 10.000 s, 2.000 s after" in text


def test_events_target_and_band_options_and_no_landing():
    # Valid ToF 0.30 (1 s), 1.00 (2 s), 1.05 (3 s, the peak); no LANDING in flight_phase.
    # Target 1.0 +- 0.06: from the peak (3 s) on, 1.05 is inside (|0.05| < 0.06) -> never outside.
    # Target 1.0 +- 0.02: 1.05 is outside (|0.05| > 0.02) -> the last time outside is the peak itself, 3.0 s.
    log = _log(flight_phase=_phase([(0, 3), (5, 5)]), tof_bottom=_tof([(1, 0.30, 0), (2, 1.0, 0), (3, 1.05, 0)]))
    wide = stats.stats_events(log, target_m=1.0, band_m=0.06)
    assert wide.values["peak_m"] == pytest.approx(1.05) and wide.values["last_outside_s"] is None
    assert "inside the band from the peak on" in "\n".join(wide.lines)
    narrow = stats.stats_events(log, target_m=1.0, band_m=0.02)
    assert narrow.values["last_outside_s"] == pytest.approx(3.0)
    assert "never started" in "\n".join(narrow.lines)


def test_events_missing_streams():
    with pytest.raises(StatsError, match="stream 'tof_bottom'"):
        stats.stats_events(_log(flight_phase=_phase([(0, 6)])))
    log = _events_log()
    del log.streams["motor"]
    with pytest.raises(StatsError, match="stream 'motor'"):
        stats.stats_events(log)
    log = _events_log()
    del log.streams["flight_phase"]
    with pytest.raises(StatsError, match="stream 'flight_phase'"):
        stats.stats_events(log)


# ---------------------------------------------------------------- turn


def _attitude(yaws_deg, step_s=1.0):
    rows = []
    for k, psi in enumerate(yaws_deg):
        half = math.radians(psi) / 2
        rows.append((round(k * step_s * 1e6), math.cos(half), 0.0, 0.0, math.sin(half)))
    return _frame(["timestamp_us", "quat_w", "quat_x", "quat_y", "quat_z"], rows)


def _posvel(points):
    return _frame(["timestamp_us", "pos_x", "pos_y"], [(round(k * 1e6), x, y) for k, (x, y) in enumerate(points)])


def test_turn_unwraps_the_heading_across_plus_minus_180():
    # Headings 170, 175, 185 (= -175), 190 (= -170) deg: the steps are +5, +10, +5 -> +20 deg (a naive
    # last - first would give -340). Min 0, max +20. ToF 0.45 .. 0.52 m. Position (0,0), (0.3,0.4), (0.3,0):
    # distances from the first point 0, 0.5, 0.3 -> max drift 0.5 m.
    log = _log(attitude=_attitude([170, 175, -175, -170]),
               tof_bottom=_tof([(0, 0.50, 0), (1, 0.45, 0), (2, 0.52, 0), (3, 0.0, 254)]),
               posvel=_posvel([(0, 0), (0.3, 0.4), (0.3, 0), (0.3, 0)]))
    v = stats.stats_turn(log, 0, 3).values
    assert v["yaw_change_deg"] == pytest.approx(20.0) and v["yaw_min_deg"] == pytest.approx(0.0)
    assert v["yaw_max_deg"] == pytest.approx(20.0)
    assert (v["tof_min_m"], v["tof_max_m"]) == (pytest.approx(0.45), pytest.approx(0.52))
    assert v["drift_m"] == pytest.approx(0.5)


def test_turn_counter_clockwise_is_negative():
    log = _log(attitude=_attitude([0, -45, -90]), tof_bottom=_tof([(0, 0.5, 0)]), posvel=_posvel([(0, 0), (0, 0), (0, 0)]))
    result = stats.stats_turn(log, 0, 2)
    assert result.values["yaw_change_deg"] == pytest.approx(-90.0) and result.values["yaw_max_deg"] == pytest.approx(0.0)
    assert "-90.0 deg" in "\n".join(result.lines)


def test_turn_missing_streams_and_short_window():
    good = dict(attitude=_attitude([0, 10, 20]), tof_bottom=_tof([(0, 0.5, 0)]), posvel=_posvel([(0, 0)]))
    for name in ("attitude", "tof_bottom", "posvel"):
        streams = dict(good)
        del streams[name]
        with pytest.raises(StatsError, match=f"stream '{name}'"):
            stats.stats_turn(_log(**streams), 0, 2)
    with pytest.raises(StatsError, match="fewer than 2 samples"):
        stats.stats_turn(_log(**good), 5, 6)


# ---------------------------------------------------------------- baro


def test_baro_difference_to_the_nearest_tof_and_the_learned_offset():
    # Baro rows at 10.0 (0.52 m, valid), 10.5 (0.55 m, valid), 11.0 (0.60 m, NOT valid -> skipped).
    # ToF 0.50 m at 10.0, 10.5, 11.0. Differences 0.02 and 0.05: rms = sqrt((0.0004 + 0.0025) / 2) = 0.038079,
    # max 0.05. Learned rows (learned = 1): 10.5 (offset 0.2, age 1.0) and 11.0 (0.3, age 2.5) -> min 0.2,
    # max 0.3, age max 2.5.
    baro = _frame(["timestamp_us", "height", "offset", "valid", "learned", "offset_age"],
                  [(10_000_000, 0.52, 0.0, 1, 0, 0.0), (10_500_000, 0.55, 0.2, 1, 1, 1.0), (11_000_000, 0.60, 0.3, 0, 1, 2.5)])
    log = _log(baro_height=baro, tof_bottom=_tof([(10.0, 0.5, 0), (10.5, 0.5, 0), (11.0, 0.5, 0)]))
    v = stats.stats_baro(log, 10, 11).values
    assert v["pairs"] == 2
    assert v["rms_m"] == pytest.approx(math.sqrt(0.00145)) and v["max_abs_m"] == pytest.approx(0.05)
    assert (v["offset_min_m"], v["offset_max_m"], v["age_max_s"]) == (0.2, 0.3, 2.5)


def test_baro_never_learned_and_missing_streams():
    baro = _frame(["timestamp_us", "height", "offset", "valid", "learned", "offset_age"], [(10_000_000, 0.01, 0.0, 1, 0, 0.0)])
    result = stats.stats_baro(_log(baro_height=baro, tof_bottom=_tof([(10.0, 0.0, 254)])), 9, 11)
    assert "never learned" in "\n".join(result.lines) and "no valid pair" in "\n".join(result.lines)
    with pytest.raises(StatsError, match="stream 'baro_height'"):
        stats.stats_baro(_log(tof_bottom=_tof([(10.0, 0.5, 0)])), 9, 11)
    with pytest.raises(StatsError, match="stream 'tof_bottom'"):
        stats.stats_baro(_log(baro_height=baro), 9, 11)


# ---------------------------------------------------------------- boot


def test_boot_names_the_reasons_and_flags_the_unexpected_ones():
    status = _frame(["timestamp_us", "voltage", "reset_reason"],
                    [(1_000_000, 4.2, 1), (2_000_000, 4.0, 1), (3_000_000, float("nan"), 3), (4_000_000, 3.9, 4)])
    result = stats.stats_boot(_log(status=status))
    v = result.values
    assert v["reasons"] == [1, 3, 4] and v["unexpected"] == [4]
    assert (v["voltage_min_v"], v["voltage_max_v"]) == (3.9, 4.2)
    text = "\n".join(result.lines)
    assert "1 POWERON, 3 SW, 4 PANIC" in text and "unexpected reasons      : 4 PANIC" in text
    assert "min 3.90 V, max 4.20 V" in text
    only_expected = stats.stats_boot(_log(status=status), until=3.5)
    assert only_expected.values["unexpected"] == [] and "none (only 1 POWERON and 3 SW)" in "\n".join(only_expected.lines)


def test_boot_missing_status():
    with pytest.raises(StatsError, match="stream 'status'"):
        stats.stats_boot(_log())


# ---------------------------------------------------------------- compute() rules


def test_windowed_modes_need_both_ends_and_a_forward_window():
    log = _log(tof_bottom=_tof([(1, 0.03, 0)]))
    for mode in ("spread", "turn", "baro"):
        with pytest.raises(StatsError, match="needs both --since and --until"):
            stats.compute(log, mode, since=1.0)
    with pytest.raises(StatsError, match="must be greater than"):
        stats.compute(log, "spread", since=5, until=5)
    with pytest.raises(StatsError, match="unknown mode"):
        stats.compute(log, "nope")
