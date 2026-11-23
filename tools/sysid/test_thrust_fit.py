#!/usr/bin/env python3
"""
test_thrust_fit.py - tests of the vertical thrust-excitation identification
(tools/sysid/thrust_fit.py and `sf sysid thrust-excite`'s argument handling).
鉛直の推力励振の同定（tools/sysid/thrust_fit.py と `sf sysid thrust-excite` の引数の組み立て）の試験。

The synthetic flight has a known plant G(s) = K exp(-L s) / (tau s + 1), an altitude-loop-like
feedback from the (noisy) measured acceleration back into the thrust, and a ToF that sees the
height with a known delay.
試験用の合成飛行は、既知のプラント G(s) = K exp(-L s) / (tau s + 1)、（雑音を含む）測定加速度から
推力へ戻る高さのループに似たフィードバック、既知の遅れで高さを見る ToF を持つ。
"""

from __future__ import annotations

import argparse
import sys
import types
from pathlib import Path

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_REPO_ROOT / "lib"))

from tools.sysid import thrust_fit as tf  # noqa: E402
from tools.sysid import thrust_excitation_table as table  # noqa: E402

# --- Known truth of the synthetic flight / 合成飛行の真値 --------------------------------
TRUE_GAIN = 27.0                     # K [(m/s^2)/N]
TRUE_TAU_S = 0.016                   # tau [s]
TRUE_DEAD_TIME_S = 0.010             # L [s] (4 control cycles)
TRUE_TOF_DELAY_S = 0.060             # ToF delay relative to the accelerometer [s]
HOVER_THRUST_N = 0.34
AMPLITUDE_RATIO = 0.05
FEEDBACK_GAIN = 0.02                 # [N / (m/s^2)] noisy acceleration -> thrust
ACCEL_NOISE_STD = 0.3                # [m/s^2] per sample (in-flight vibration is of this order)
PRE_HOVER_CYCLES = 8000
STANDBY_CYCLES = 400
RUN_CYCLES = 8000                    # 20 s
POST_CYCLES = 800
TOF_RATE_HZ = 28.5
TOF_NOISE_STD_M = 0.002
STATE_RATE_DIVIDER = 8               # 400 Hz / 50 Hz


def _delayed_first_order(drive: float, state: float, alpha: float) -> float:
    return alpha * state + (1.0 - alpha) * TRUE_GAIN * drive


def make_record(seed=1, run_cycles=RUN_CYCLES, edge_offset=-3, feedback_gain=FEEDBACK_GAIN,
                accel_noise_std=ACCEL_NOISE_STD, tof_delay_s=TRUE_TOF_DELAY_S) -> tuple:
    """Closed-loop synthetic flight. Returns (record, true_start_cycle).
    閉ループの合成飛行。(記録, 真の開始の周期) を返す。"""
    rng = np.random.default_rng(seed)
    total = PRE_HOVER_CYCLES + STANDBY_CYCLES + run_cycles + POST_CYCLES
    start = PRE_HOVER_CYCLES + STANDBY_CYCLES
    excitation = np.zeros(total)
    excitation[start:start + run_cycles] = (AMPLITUDE_RATIO * HOVER_THRUST_N *
                                            tf.regenerate_multisine(run_cycles))
    alpha = np.exp(-1.0 / (tf.CYCLE_RATE_HZ * TRUE_TAU_S))
    delay = int(round(TRUE_DEAD_TIME_S * tf.CYCLE_RATE_HZ))
    noise = accel_noise_std * rng.standard_normal(total)
    thrust, accel_true, accel_meas = np.zeros(total), np.zeros(total), np.zeros(total)
    lag_state = 0.0
    for k in range(total):
        feedback = -feedback_gain * accel_meas[k - 1] if k > 0 else 0.0
        thrust[k] = HOVER_THRUST_N + excitation[k] + feedback
        drive = thrust[k - delay] - HOVER_THRUST_N if k >= delay else 0.0
        lag_state = _delayed_first_order(drive, lag_state, alpha)
        accel_true[k] = lag_state
        accel_meas[k] = lag_state + noise[k]
    record = _assemble_record(rng, thrust, accel_true, accel_meas, start, run_cycles,
                              edge_offset, tof_delay_s)
    return record, start


def _state_codes(total, start, run_cycles, edge_offset) -> np.ndarray:
    """thrust_excitation_state sampled at 50 Hz and held; the edge sits at start + edge_offset.
    50 Hz で標本化して保持した励振の状態。変わり目は start + edge_offset の位置。"""
    code = np.zeros(total, dtype=int)
    waveform = tf.WAVEFORM_MULTISINE << tf.WAVEFORM_SHIFT
    shifted = start + edge_offset
    code[shifted - STANDBY_CYCLES:shifted] = 1 | waveform
    code[shifted:shifted + table.MULTISINE_RAMP_UP_CYCLES] = 2 | waveform
    code[shifted + table.MULTISINE_RAMP_UP_CYCLES:shifted + run_cycles - table.MULTISINE_RAMP_DOWN_CYCLES] = 3 | waveform
    code[shifted + run_cycles - table.MULTISINE_RAMP_DOWN_CYCLES:shifted + run_cycles] = 4 | waveform
    return code


def _assemble_record(rng, thrust, accel_true, accel_meas, start, run_cycles, edge_offset, tof_delay_s):
    total = thrust.size
    cycle = np.arange(total, dtype=float)
    velocity = np.cumsum(accel_true) / tf.CYCLE_RATE_HZ
    height = 0.7 + np.cumsum(velocity) / tf.CYCLE_RATE_HZ
    interval = tf.CYCLE_RATE_HZ / TOF_RATE_HZ
    tof_cycle = np.arange(0.0, total - 1.0, interval)                # irregular on the 400 Hz grid / 400 Hz の格子に乗らない
    tof_height = np.interp(tof_cycle - tof_delay_s * tf.CYCLE_RATE_HZ, cycle, height)
    tof_height = np.round(tof_height + TOF_NOISE_STD_M * rng.standard_normal(tof_cycle.size), 3)
    return tf.ThrustRecord(cycle=cycle, thrust=thrust, upward_accel=accel_meas, tilt_cos=np.ones(total),
                           state_code=_state_codes(total, start, run_cycles, edge_offset),
                           tof_cycle=tof_cycle, tof_height=tof_height, gap_count=0, source="synthetic")


# =============================================================================
# (1) K, tau, L and the closed-loop bias / K・tau・L の復元と閉ループの偏り
# =============================================================================

def test_plant_is_recovered_under_closed_loop_noise():
    record, _ = make_record(seed=3)
    result = tf.analyze_record(record)
    fit = result["fit"]
    assert fit["K"] == pytest.approx(TRUE_GAIN, rel=0.05)
    assert fit["tau_s"] == pytest.approx(TRUE_TAU_S, abs=0.004)
    assert fit["L_s"] == pytest.approx(TRUE_DEAD_TIME_S, abs=0.003)
    assert result["start"]["correlation"] > 0.5


def _direct_ratio_welch(record, start, run_cycles) -> np.ndarray:
    """The plain ratio S_ua / S_uu of Welch estimates at the tone bins (biased under feedback).
    Welch 推定の単純な比 S_ua / S_uu（フィードバックがあると偏る）。"""
    from scipy.signal import csd
    first = start + table.MULTISINE_RAMP_UP_CYCLES
    last = start + run_cycles - table.MULTISINE_RAMP_DOWN_CYCLES
    u = record.thrust[first:last] - record.thrust[first:last].mean()
    a = record.upward_accel[first:last] - record.upward_accel[first:last].mean()
    segment = 4000
    frequency, s_ua = csd(u, a, fs=tf.CYCLE_RATE_HZ, nperseg=segment, window="boxcar", detrend=False)
    _, s_uu = csd(u, u, fs=tf.CYCLE_RATE_HZ, nperseg=segment, window="boxcar", detrend=False)
    bins = [int(np.argmin(np.abs(frequency - f))) for f in tf.TONE_FREQUENCIES_HZ]
    return (s_ua / s_uu)[bins]


def _true_response() -> np.ndarray:
    omega = 2 * np.pi * np.array(tf.TONE_FREQUENCIES_HZ)
    return tf.model_response([TRUE_GAIN, TRUE_TAU_S, TRUE_DEAD_TIME_S], omega)


def test_excitation_reference_is_unbiased_but_input_reference_is_biased():
    """Long run (60 s = 6 periods), noise that comes back through the loop. Averaged over 16 noise
    seeds and the six tones up to 4.1 Hz, G_hat / G_true from the excitation reference is near 1;
    that of the plain u-referenced ratio is a clear loss of gain.
    長い実行（60 s = 6 周期）・ループを通って戻る雑音。16 通りの雑音と 4.1 Hz までの 6 本の音で平均すると、
    励振を基準にした G_hat / G_true は 1 の近く、u を基準にした単純な比は明らかなゲインの低下になる。"""
    run_cycles = 6 * table.PERIOD_CYCLES
    truth = _true_response()
    tones = np.array(tf.TONE_FREQUENCIES_HZ)
    reference_errors, direct_errors = [], []
    for seed in range(16):
        record, start = make_record(seed=seed, run_cycles=run_cycles, feedback_gain=0.05, accel_noise_std=1.0)
        rows, _, _, _ = tf._steady_window(record, start, run_cycles)
        times = (record.cycle[rows] - start) / tf.CYCLE_RATE_HZ
        thrust_tones = tf.tone_amplitudes_with_noise(times, record.thrust[rows], tones)
        accel_tones = tf.tone_amplitudes_with_noise(times, record.upward_accel[rows], tones)
        reference = accel_tones["amplitude"] / thrust_tones["amplitude"]
        reference_errors.append((reference / truth)[:6])
        direct_errors.append((_direct_ratio_welch(record, start, run_cycles) / truth)[:6])
    reference_bias = abs(complex(np.mean(reference_errors)) - 1.0)     # mean of G_hat / G_true: 1 = unbiased
    direct_bias = abs(complex(np.mean(direct_errors)) - 1.0)
    assert reference_bias < 0.07
    assert direct_bias > 0.15
    assert direct_bias > 3.0 * reference_bias


# =============================================================================
# (2) Start-cycle search / 開始位置の探索
# =============================================================================

@pytest.mark.parametrize("edge_offset", [-11, -4, 0, 6, 11])
def test_start_search_recovers_a_known_offset(edge_offset):
    """The feedback leads the thrust slightly, so the peak sits a few cycles early when the loop is strong;
    with a weak loop it is within one cycle. (G does not depend on it: the time origin cancels in A_a / A_u.)
    フィードバックが推力をわずかに進めるので、ループが強いと峰が数周期早くなる。弱いループでは 1 周期以内。
    （G には影響しない: 時間の原点は A_a / A_u で消える。）"""
    record, start = make_record(seed=5, edge_offset=edge_offset, feedback_gain=0.005)
    segment = tf.find_excitation_segments(record)[0]
    assert segment["edge_row"] == start + edge_offset
    found = tf.search_start_cycle(record, segment["edge_row"], RUN_CYCLES)
    assert abs(found["start_cycle"] - start) <= 1
    assert found["correlation"] > 0.5
    assert found["correlation"] > found["correlation_runner_up"] - 1e-12


def test_segment_waveform_and_abort_detection():
    record, start = make_record(seed=2)
    segment = tf.find_excitation_segments(record)[0]
    assert segment["waveform"] == tf.WAVEFORM_MULTISINE
    assert segment["ramp_down_cycles"] == table.MULTISINE_RAMP_DOWN_CYCLES
    assert tf.estimate_total_cycles(RUN_CYCLES + 7) == RUN_CYCLES


def test_regenerated_excitation_matches_table_and_envelope():
    reference = tf.regenerate_multisine(RUN_CYCLES)
    assert reference.shape == (RUN_CYCLES,)
    assert abs(reference[0]) < 1e-12 and abs(reference[-1]) < 1e-12
    steady = tf.multisine_waveform(RUN_CYCLES)
    assert np.max(np.abs(steady)) == pytest.approx(1.0, abs=1e-3)          # unit peak / ピーク 1


# =============================================================================
# (3) ToF delay / ToF の遅れ
# =============================================================================

def _zero_order_hold_delay(record, start, run_cycles) -> float:
    """The method NOT used: hold each ToF sample up to the next and regress at 400 Hz.
    使わない手法: ToF の各標本を次まで保持し、400 Hz で回帰する。"""
    rows, _, _, _ = tf._steady_window(record, start, run_cycles)
    index = np.searchsorted(record.tof_cycle, record.cycle, side="right") - 1
    held = record.tof_height[np.clip(index, 0, record.tof_height.size - 1)]
    times = (record.cycle[rows] - start) / tf.CYCLE_RATE_HZ
    tones = np.array(tf.TONE_FREQUENCIES_HZ)
    accel = tf.tone_amplitudes_with_noise(times, record.upward_accel[rows], tones)
    tof = tf.tone_amplitudes_with_noise(times, held[rows], tones)
    return tf.estimate_tof_delay(tones, accel["amplitude"], tof)["tof_delay_relative_to_accelerometer_s"]


def test_tof_delay_is_recovered_and_zero_order_hold_is_biased():
    delays, zero_order = [], []
    for seed in range(8):
        record, start = make_record(seed=seed, accel_noise_std=0.3, feedback_gain=0.01)
        result = tf.analyze_record(record)
        delays.append(result["tof"]["tof_delay_relative_to_accelerometer_s"])
        zero_order.append(_zero_order_hold_delay(record, start, RUN_CYCLES))
    hold_extra_s = 0.5 / TOF_RATE_HZ                     # mean extra delay of a zero-order hold / 零次保持が足す平均の遅れ
    assert abs(np.mean(delays) - TRUE_TOF_DELAY_S) < 0.006
    assert np.mean(zero_order) - TRUE_TOF_DELAY_S > 0.6 * hold_extra_s


def test_tof_tones_with_low_snr_are_excluded():
    record, _ = make_record(seed=1)
    tof = tf.analyze_record(record)["tof"]
    assert 8.9 not in [round(f, 1) for f in tof["used_tones_hz"]]
    assert tof["std_error_s"] is not None


# =============================================================================
# (4) Accelerometer noise / 加速度計の雑音
# =============================================================================

def test_noise_spectrum_of_white_noise():
    density = 0.05                                        # [(m/s^2)/sqrt(Hz)]
    sigma = density * np.sqrt(tf.CYCLE_RATE_HZ / 2.0)
    values = sigma * np.random.default_rng(0).standard_normal(4000 * 6)
    summary = tf.noise_summary(values)
    assert summary["floor_mps2_per_sqrt_hz"] == pytest.approx(density, rel=0.12)
    assert summary["band_rms_mps2"]["3.0-6.0Hz"] == pytest.approx(density * np.sqrt(3.0), rel=0.15)
    assert summary["total_rms_mps2"] == pytest.approx(sigma, rel=0.03)


def test_hover_noise_window_is_found_before_the_standby():
    record, _ = make_record(seed=4, accel_noise_std=0.5, feedback_gain=0.0)
    noise = tf.analyze_record(record)["accelerometer_noise"]
    hover = noise["hover_before_excitation"]
    expected_floor = 0.5 / np.sqrt(tf.CYCLE_RATE_HZ / 2.0)
    assert hover["floor_mps2_per_sqrt_hz"] == pytest.approx(expected_floor, rel=0.2)
    assert "during_excitation" in noise


def test_upward_acceleration_is_gravity_free_at_rest_and_tilt_corrected():
    quaternion = np.array([[1.0, 0.0, 0.0, 0.0]])
    rest_force = np.array([[0.0, 0.0, -tf.GRAVITY_MPS2]])
    upward, tilt_cos = tf.upward_acceleration(quaternion, rest_force)
    assert upward[0] == pytest.approx(0.0, abs=1e-9) and tilt_cos[0] == pytest.approx(1.0)
    angle = np.radians(30.0)                              # tilted about x with a_up = 1 m/s^2
    tilted = np.array([[np.cos(angle / 2), np.sin(angle / 2), 0.0, 0.0]])
    magnitude = tf.GRAVITY_MPS2 + 1.0
    body_force = np.array([[0.0, -magnitude * np.sin(angle), -magnitude * np.cos(angle)]])
    upward, tilt_cos = tf.upward_acceleration(tilted, body_force)
    assert upward[0] == pytest.approx(1.0, abs=1e-6) and tilt_cos[0] == pytest.approx(np.cos(angle))


# =============================================================================
# (5) thrust-excite arguments / thrust-excite の引数の組み立て
# =============================================================================

@pytest.fixture
def sysid_module():
    from sfcli.commands import sysid
    return sysid


def _parse(sysid_module, argv):
    parser = argparse.ArgumentParser()
    sysid_module.register(parser.add_subparsers())
    return parser.parse_args(["sysid", "thrust-excite", *argv])


def test_thrust_excite_multisine_plan(sysid_module):
    args = _parse(sysid_module, ["--takeoff", "--land", "--amp", "5", "--dur", "20", "--height", "0.7"])
    commands = [command for _, command, _ in sysid_module.build_thrust_excite_plan(args)]
    assert commands == ["takeoff", "up 20", "sysid thrust multisine 5 20", "land"]
    timeouts = [timeout for _, _, timeout in sysid_module.build_thrust_excite_plan(args)]
    assert timeouts[2] > 20.0


def test_thrust_excite_steps_plan_without_takeoff(sysid_module):
    args = _parse(sysid_module, ["--waveform", "steps", "--amp", "4", "--half-period", "0.8", "--count", "4"])
    plan = sysid_module.build_thrust_excite_plan(args)
    assert [command for _, command, _ in plan] == ["sysid thrust step 4 0.8 4"]
    assert plan[0][2] > 2 * 4 * 0.8                      # waits for the whole run / 全体が終わるまで待つ


@pytest.mark.parametrize("argv", [["--amp", "0"], ["--dur", "-1"], ["--takeoff", "--height", "0.55"],
                                  ["--waveform", "steps", "--count", "0"]])
def test_thrust_excite_rejects_bad_arguments(sysid_module, argv):
    args = _parse(sysid_module, argv)
    with pytest.raises(ValueError):
        sysid_module.build_thrust_excite_plan(args)


class _FakeStampFlyError(RuntimeError):
    pass


def _install_fake_stampfly(monkeypatch, fail_on=None):
    sent = []

    class FakeStampFly:
        def __init__(self, ip):
            self.ip = ip

        def connect(self):
            sent.append(("command", None))

        def send(self, command, timeout=10.0):
            sent.append((command, timeout))
            if fail_on and command.startswith(fail_on):
                raise _FakeStampFlyError(f"'{command}' -> error excitation rejected: height")
            return "ok"

        def close(self):
            sent.append(("closed", None))

    module = types.ModuleType("stampfly")
    module.StampFly, module.StampFlyError = FakeStampFly, _FakeStampFlyError
    monkeypatch.setitem(sys.modules, "stampfly", module)
    return sent


def test_thrust_excite_sends_the_plan_to_a_mock_vehicle(sysid_module, monkeypatch):
    sent = _install_fake_stampfly(monkeypatch)
    args = _parse(sysid_module, ["--takeoff", "--land"])
    assert sysid_module.run_thrust_excite(args) == 0
    assert [c for c, _ in sent] == ["command", "takeoff", "up 20", "sysid thrust multisine 5 20", "land", "closed"]


def test_thrust_excite_lands_after_a_rejected_excitation(sysid_module, monkeypatch):
    sent = _install_fake_stampfly(monkeypatch, fail_on="sysid thrust")
    args = _parse(sysid_module, ["--takeoff", "--land"])
    assert sysid_module.run_thrust_excite(args) == 1
    assert [c for c, _ in sent][-2:] == ["land", "closed"]
