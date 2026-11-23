# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
# Part of StampFly Ecosystem (SILS thrust-excitation bundle checks).
"""
test_thrust_excitation_bundles.py - checks on the flight-log bundles of the thrust-excitation scenarios
that a pass/fail line of an .expect file cannot express (comparison between two flights, a per-cycle change).

test_thrust_excitation_bundles.py - 推力励振のシナリオの飛行記録の一式に対する検査。.expect の合否の線では
書けないもの（2 つの飛行の比較、1 周期あたりの変化）。

Run `sf sils regression` (or `sf sils scenario simulator/sils/scenarios/<name>.scn ...` for the
sysid_thrust_* scenarios) first: the tests read the bundles that it leaves in simulator/sils/viz/out_scn_<name>/ and are
skipped when a bundle is missing. The pass lines were fixed from the design (detailed_design.md §3, "thrust excitation")
before the bundles were looked at; the basis is written at each test.
先に `sf sils regression`（または sysid_thrust_* の `sf sils scenario`）を実行する: 試験はそれが
simulator/sils/viz/out_scn_<name>/ に残した一式を読み、一式が無ければ skip する。合格の線は、一式を見る前に設計
（detailed_design.md §3「推力励振」）から決めた。根拠は各試験に書く。

@design detailed_design.md §3 — thrust excitation [OK]
"""

from __future__ import annotations

import csv
import io
import zipfile
from pathlib import Path

import numpy as np
import pytest

VIZ_DIR = Path(__file__).resolve().parents[1] / "viz"
SAMPLE_RATE_HZ = 400.0
GRAVITY = 9.80665

# Largest change of the thrust command between two control cycles that is not a designed step [N]. Basis: the design's
# criterion for the abort ramp ("a step of at most 0.01 N per cycle"); the loop's own correction limit is 0.15 N over seconds.
# 設計された段でない、制御周期 2 つの間の推力指令の変化の上限 [N]。根拠: 設計の打ち切りの立ち下げの基準
# （「段差は 1 周期 0.01 N 以下」）。ループ自身の補正の上限は数 s で 0.15 N。
MAX_THRUST_STEP_N = 0.01
# Margin around an abort ramp seen on the 50 Hz record (one record = 20 ms). / 50 Hz の記録で見た打ち切りの立ち下げの前後の余裕（記録 1 つ = 20 ms）。
RAMP_MARGIN_S = 0.02
# Window of the multisine scenario in which the signal is at full size (s). / 多重正弦波のシナリオで信号が全振幅の窓 [s]。
EXCITED_WINDOW_S = (30.0, 44.0)
# Largest deviation of the excitation-to-acceleration gain with the DOB from the gain without it (design: the same gain within +-20 %), and the lowest tone compared [Hz].
# DOB のあるときの、励振から加速度への利得の、ないときの利得からのずれの許容（設計: 利得が ±20 % 以内で同じ）と、比べる最低の音 [Hz]。
DOB_RATIO_TOLERANCE = 0.20
DOB_MIN_TONE_HZ = 2.5
# Windows of the lock-in (s): the hover before the excitation, and 10 s inside the run (whole periods of every tone). / ロックインの窓 [s]: 励振の前のホバーと、実行の中の 10 s（どの音の周期の整数倍でもある）。
HOVER_WINDOW_S = (24.0, 26.0)
LOCK_IN_WINDOW_S = (30.0, 40.0)
EXCITATION_RATIO = 0.05   # the 5 % of the scenarios / シナリオの 5 %
# The thrust after the excitation must equal the thrust of the reference flight (no residual offset): 1 % of the hover thrust, a fifth of
# the 5 % amplitude. / 励振の後の推力は基準の飛行の推力と等しい（残るずれなし）: ホバー推力の 1 %。5 % の振幅の 5 分の 1。
POST_THRUST_TOLERANCE = 0.01
POST_WINDOW_S = (48.0, 50.0)
# The signal must be seen in the vertical acceleration: its RMS in the excited window exceeds that of the reference flight by this factor.
# 信号が鉛直の加速度に見える: 励振の窓の実効値が基準の飛行のこの倍率を超える。
MIN_ACCELERATION_GAIN = 1.5


def load_stream(name: str, stream: str) -> tuple[list[str], np.ndarray] | None:
    """Read one CSV stream of the bundle of a scenario; None when there is no bundle. / シナリオの一式の CSV を 1 つ読む。一式が無ければ None。"""
    zips = sorted((VIZ_DIR / f"out_scn_{name}").glob("*.sflog.zip"))
    if not zips:
        return None
    with zipfile.ZipFile(zips[-1]) as archive:
        rows = list(csv.reader(io.StringIO(archive.read(stream + ".csv").decode())))
    data = np.array([[float(cell) if cell != "" else np.nan for cell in row] for row in rows[1:]])
    return rows[0], data


def require(name: str, stream: str) -> tuple[list[str], np.ndarray]:
    loaded = load_stream(name, stream)
    if loaded is None:
        pytest.skip(f"no bundle for {name} (run the scenario first)")
    return loaded


def column(header: list[str], data: np.ndarray, name: str) -> np.ndarray:
    return data[:, header.index(name)]


def window(times_s: np.ndarray, values: np.ndarray, start_s: float, end_s: float) -> np.ndarray:
    return values[(times_s >= start_s) & (times_s <= end_s)]


def vertical_acceleration_rms(name: str, start_s: float, end_s: float) -> float:
    """RMS of the body-z accelerometer reading (mean removed) in a window; the craft hovers level, so it is the vertical acceleration."""
    header, data = require(name, "imu")
    times = column(header, data, "timestamp_us") / 1e6
    accel_z = window(times, column(header, data, "accel_z"), start_s, end_s)
    return float(np.sqrt(np.mean((accel_z - accel_z.mean()) ** 2)))


def test_multisine_thrust_never_steps_by_more_than_the_limit():
    header, data = require("sysid_thrust_multisine", "ctrl_output")
    times = column(header, data, "timestamp_us") / 1e6
    thrust = window(times, column(header, data, "thrust"), 26.0, 50.0)
    assert np.max(np.abs(np.diff(thrust))) <= MAX_THRUST_STEP_N


def test_abort_ramps_do_not_step_the_thrust():
    """In every abort ramp (state RampDown of the abort chain) the thrust changes by at most MAX_THRUST_STEP_N per control cycle."""
    header, data = require("sysid_thrust_abort_chain", "ctrl_output")
    times = column(header, data, "timestamp_us") / 1e6
    thrust = column(header, data, "thrust")
    ref_header, ref_data = require("sysid_thrust_abort_chain", "ctrl_ref")
    ref_times = column(ref_header, ref_data, "timestamp_us") / 1e6
    ramp_rows = np.where((column(ref_header, ref_data, "thrust_excitation_state").astype(int) & 0x07) == 4)[0]
    assert len(ramp_rows) > 0
    groups: list[list[int]] = []
    for row in ramp_rows:
        if groups and row - groups[-1][-1] <= 2:
            groups[-1].append(row)
        else:
            groups.append([row])
    assert len(groups) == 4                                       # cancel, stick, battery, height error / 取り消し・スティック・電池・高さのずれ
    for group in groups:
        first_s, last_s = ref_times[group[0]] - RAMP_MARGIN_S, ref_times[group[-1]] + RAMP_MARGIN_S
        in_ramp = thrust[(times >= first_s) & (times <= last_s)]
        assert np.max(np.abs(np.diff(in_ramp))) <= MAX_THRUST_STEP_N


def test_the_excitation_is_seen_in_the_vertical_acceleration():
    """Basis of MIN_ACCELERATION_GAIN: a 5 % signal moves the acceleration by a fraction of a m/s^2, the unexcited hover by far less (noise is off in SILS). / MIN_ACCELERATION_GAIN の根拠: 5 % の信号は加速度を 1 m/s² の何分の一か動かし、励振なしのホバーはずっと小さい（SILS は雑音なし）。"""
    excited = vertical_acceleration_rms("sysid_thrust_multisine", *EXCITED_WINDOW_S)
    reference = vertical_acceleration_rms("sysid_thrust_reference", *EXCITED_WINDOW_S)
    assert excited > MIN_ACCELERATION_GAIN * reference


def test_the_excitation_leaves_no_offset_in_the_thrust_afterwards():
    header, data = require("sysid_thrust_multisine", "ctrl_output")
    reference_header, reference_data = require("sysid_thrust_reference", "ctrl_output")
    after = window(column(header, data, "timestamp_us") / 1e6, column(header, data, "thrust"), *POST_WINDOW_S).mean()
    base = window(column(reference_header, reference_data, "timestamp_us") / 1e6,
                  column(reference_header, reference_data, "thrust"), *POST_WINDOW_S).mean()
    assert abs(after - base) / base <= POST_THRUST_TOLERANCE


def tone_gains(name: str) -> np.ndarray:
    """|vertical acceleration| / |excitation| at each tone, from a lock-in over 10 s (whole periods of every tone). The excitation amplitude of a tone is
    amplitude x hover thrust x MULTISINE_SCALE, with the hover thrust taken before the run, so no alignment of the start is needed (magnitudes only).
    各音での |鉛直の加速度| / |励振|。10 s（どの音の周期の整数倍でもある）のロックイン。1 つの音の励振の振幅は 振幅 × ホバー推力 × MULTISINE_SCALE
    （ホバー推力は実行の前の値）なので、開始の位置合わせは要らない（大きさだけ）。"""
    from tools.sysid import thrust_excitation_table as table
    header, data = require(name, "imu")
    times = column(header, data, "timestamp_us") / 1e6
    accel = column(header, data, "accel_z")
    out_header, out_data = require(name, "ctrl_output")
    hover = window(column(out_header, out_data, "timestamp_us") / 1e6, column(out_header, out_data, "thrust"), *HOVER_WINDOW_S).mean()
    mask = (times >= LOCK_IN_WINDOW_S[0]) & (times < LOCK_IN_WINDOW_S[1])
    deviation, stamps = accel[mask] - accel[mask].mean(), times[mask]
    gains = []
    for frequency_index in table.TONE_FREQUENCY_INDEX:
        omega = 2.0 * np.pi * frequency_index / 10.0
        response = 2.0 * np.mean(deviation * np.exp(-1j * omega * stamps))
        gains.append(abs(response) / (EXCITATION_RATIO * hover * table.MULTISINE_SCALE))
    return np.array(gains)


def test_the_dob_does_not_cancel_the_excitation():
    """The gain from the excitation to the vertical acceleration with the DOB on is within +-20 % of the gain with it off at the tones above
    DOB_MIN_TONE_HZ. Basis: the DOB cutoff is 1.5 Hz; from 2.7 Hz (1.8 x the cutoff) the DOB no longer reshapes the closed loop, so a gain
    that differs there would mean that the DOB had read the excitation as a disturbance. Below that frequency the closed loop itself differs
    with the DOB (measured at the first run: 0.7 to 0.83 of the gain at 1.1 and 1.7 Hz).
    DOB を有効にしたときの、励振から鉛直の加速度への利得は、DOB_MIN_TONE_HZ 以上の音で、無効のときの ±20 % 以内。根拠: DOB の遮断は
    1.5 Hz。2.7 Hz（遮断の 1.8 倍）からは DOB が閉ループの形を変えないので、そこで利得が違えば DOB が励振を外乱と読んだことになる。それより
    低い周波数では、DOB で閉ループ自体が違う（最初の実行で 1.1・1.7 Hz の利得の 0.7〜0.83 倍）。"""
    from tools.sysid import thrust_excitation_table as table
    with_dob, without_dob = tone_gains("sysid_thrust_dob"), tone_gains("sysid_thrust_multisine")
    tones_hz = np.array(table.TONE_FREQUENCY_INDEX) / 10.0
    compared = tones_hz >= DOB_MIN_TONE_HZ
    assert compared.sum() >= 4
    assert np.all(np.abs(with_dob[compared] / without_dob[compared] - 1.0) <= DOB_RATIO_TOLERANCE)


def test_the_state_byte_follows_the_designed_sequence():
    header, data = require("sysid_thrust_multisine", "ctrl_ref")
    state = column(header, data, "thrust_excitation_state").astype(int)
    states = state & 0x07
    waveforms = state >> 3
    active = state[states > 0]
    assert set(waveforms[states > 0]) == {1}                      # multisine only / 多重正弦波だけ
    order = [s for i, s in enumerate(states) if s > 0 and (i == 0 or states[i - 1] != s)]
    assert order == [1, 2, 3, 4]                                  # standby, ramp-up, running, ramp-down / 待機・立ち上げ・実行・立ち下げ
    assert len(active) > 0
