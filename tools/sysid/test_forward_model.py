#!/usr/bin/env python3
"""
test_forward_model.py - tests for tools/sysid/forward_model.py.
test_forward_model.py - tools/sysid/forward_model.py の試験。

Covers:
  - plant-independent properties (symmetric duty -> no torque; yaw sign pattern;
    a held duty settles at the plant's steady-state speed);
  - self-consistency against the SILS plant: run sysid_gate.scn (roll/pitch/yaw
    chirps in hover) and api_flip_roll.scn (roll flip) through the real SILS,
    then check that the module's angular acceleration predicted from the LOGGED
    duty matches the plant's own truth.csv body-rate derivative, airborne rows only.
    Skipped when the SILS binary is not built (`sf sils build`).
対象:
  - プラント非依存の性質 (対称な duty ならトルク 0、ヨー符号、保持した duty が
    プラントの定常回転数に落ち着く);
  - SILS プラントとの自己無矛盾性: 実際の SILS で sysid_gate.scn (ホバー中の
    ロール・ピッチ・ヨーのチャープ) と api_flip_roll.scn (ロール宙返り) を回し、
    ログされた duty から予測した角加速度がプラント自身の truth.csv の角速度微分に
    一致すること (空中の行のみ)。SILS 未ビルドなら skip。
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_REPO_ROOT / "lib"))

from tools.sysid import forward_model as fm  # noqa: E402
from tools.sysid import _generated_params as gp  # noqa: E402

_SILS_BINARY = _REPO_ROOT / "simulator" / "sils" / "build" / "emu_vehicle"
_SCENARIO_DIR = _REPO_ROOT / "simulator" / "sils" / "scenarios"
CONTROL_PERIOD_S = 0.0025                  # 400 Hz
# Tolerances: measured on 2026-11-01 (0.00-0.01 % roll/pitch/yaw against truth, 0.28 %
# worst case); the margin is ~10x. / 2026-11-01 実測 (真値に対し最大 0.28%) の約 10 倍。
RESIDUAL_TOLERANCE = 0.03
SCALE_TOLERANCE = 0.01
EXCITED_MIN_RMS = 1.0                      # rad/s^2: axes below this are not excited
SETTLE_S = 1.0
HOVER_DUTY = 0.55


def test_symmetric_duty_gives_no_torque():
    duty = np.full((400, 4), HOVER_DUTY)
    trace = fm.simulate_motors(duty, CONTROL_PERIOD_S, voltage=np.full(400, 3.7))
    tau = fm.row_mean(fm.body_torque(trace), trace)
    # Roll/pitch cancel exactly; yaw cancels (CW and CCW pairs) once the speeds settle.
    # ロール/ピッチは厳密に打ち消し、ヨーも定常なら CW/CCW 対で打ち消す。
    assert np.allclose(tau[-50:], 0.0, atol=1e-12)


def test_held_duty_settles_at_steady_state_speed():
    duty = np.full((800, 4), HOVER_DUTY)
    voltage = 3.7
    trace = fm.simulate_motors(duty, CONTROL_PERIOD_S, voltage=np.full(800, voltage),
                               pwm_bits=None)
    const = fm.DEFAULT_CONSTANTS
    rout = fm.rout_ohm(HOVER_DUTY, const)
    a = const.cq + const.kt * const.i0b
    b = const.kt * const.ke / rout
    c = const.kt * const.i0a - const.kt * voltage / rout
    omega_ss = (-b + np.sqrt(b * b - 4 * a * c)) / (2 * a)   # plant.cpp steadyStateOmega
    assert trace.omega[-1] == pytest.approx(omega_ss, rel=1e-4)


def test_unit_duty_difference_signs():
    # More duty on the front-right motor (index 0): the right side and the nose go up,
    # i.e. roll torque about FRD x is NEGATIVE (x positive = right wing down) and pitch
    # torque about y is POSITIVE (nose up).
    # 右前モータ (添字 0) を強くすると FRD の x まわりは負 (右が上がる)、y まわりは正 (機首上げ)。
    duty = np.full((400, 4), HOVER_DUTY)
    duty[:, 0] += 0.05
    trace = fm.simulate_motors(duty, CONTROL_PERIOD_S, voltage=np.full(400, 3.7))
    tau = fm.row_mean(fm.body_torque(trace), trace)[-1]
    assert tau[0] < 0.0 and tau[1] > 0.0


def test_spread_row_times_places_repeated_stamps_over_the_stall():
    # A 5 ms stall followed by two rows with the same stamp: the two rows are spread
    # over the stall (one control period apart), the stamp itself is kept for the last.
    # 5 ms の停滞のあと同一時刻の 2 行: 2 行を停滞区間に 1 制御周期間隔で並べ、
    # 最後の行は元の時刻を保つ。
    stamp_s = np.array([0.0, 0.0025, 0.0075, 0.0075, 0.0100])
    row_time = fm._spread_row_times(stamp_s)
    assert np.allclose(row_time, [0.0, 0.0025, 0.0050, 0.0075, 0.0100])


def _run_scenario(name: str, duration_us: int, extra: list, tmp_path: Path) -> Path:
    if not _SILS_BINARY.exists():
        pytest.skip("SILS not built (run: sf sils build)")
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(
        [str(_REPO_ROOT / "lib"), str(_REPO_ROOT)]))
    cmd = [sys.executable, "-m", "sfcli.cli", "sils", "scenario",
           str(_SCENARIO_DIR / f"{name}.scn"), "--target", "vehicle",
           "--duration", str(duration_us)] + extra
    subprocess.run(cmd, cwd=_REPO_ROOT, env=env, check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    bundles = sorted((_REPO_ROOT / "simulator" / "sils" / "viz" / f"out_scn_{name}").glob("*.sflog.zip"))
    if not bundles:
        pytest.skip(f"scenario {name} produced no bundle")
    return bundles[-1]


def _check_against_truth(bundle: Path, axes: tuple):
    motion = fm.load_logged_motion(bundle)
    t_truth, rate_truth, altitude = fm.load_truth(bundle)
    truth = fm.accel_from_rate(motion.t_s, rate_truth, motion.dt, t_src=t_truth)
    pred = fm.predict_angular_accel(motion, delay_s=fm.SILS_LOG_ALIGNMENT_DELAY_S)
    airborne = np.interp(motion.t_s, t_truth, altitude) > fm.AIRBORNE_MIN_ALT_M
    airborne &= motion.t_s > motion.t_s[0] + SETTLE_S
    assert airborne.sum() > 1000
    residual = fm.relative_residual(truth, pred, airborne)
    for axis in axes:
        rms = np.sqrt(np.nanmean(truth[airborne, axis] ** 2))
        assert rms > EXCITED_MIN_RMS, f"axis {axis} not excited"
        assert residual[axis] < RESIDUAL_TOLERANCE, (axis, residual)
        ok = airborne & np.isfinite(truth[:, axis])
        scale = np.dot(truth[ok, axis], pred[ok, axis]) / np.dot(pred[ok, axis], pred[ok, axis])
        assert abs(scale - 1.0) < SCALE_TOLERANCE, (axis, scale)


@pytest.fixture(scope="module")
def sysid_gate_bundle(tmp_path_factory):
    return _run_scenario("sysid_gate", 44_000_000, [], tmp_path_factory.mktemp("sg"))


@pytest.fixture(scope="module")
def flip_roll_bundle(tmp_path_factory):
    return _run_scenario("api_flip_roll", 39_000_000, ["--param", "flip.enable=1"],
                         tmp_path_factory.mktemp("fr"))


def test_matches_sils_truth_in_hover_chirps(sysid_gate_bundle):
    _check_against_truth(sysid_gate_bundle, axes=(0, 1, 2))


def test_matches_sils_truth_in_roll_flip(flip_roll_bundle):
    _check_against_truth(flip_roll_bundle, axes=(0,))


def test_rejects_unquantised_duty_assumption(sysid_gate_bundle):
    # Falsification of the PWM-quantisation term: feeding the unquantised duty must be
    # clearly WORSE (it was 3-4x worse when measured), so the term is doing real work.
    # PWM 量子化項の反証: 無量子化 duty を入れると明らかに悪化する (実測 3〜4 倍)。
    motion = fm.load_logged_motion(sysid_gate_bundle)
    t_truth, rate_truth, altitude = fm.load_truth(sysid_gate_bundle)
    truth = fm.accel_from_rate(motion.t_s, rate_truth, motion.dt, t_src=t_truth)
    airborne = np.interp(motion.t_s, t_truth, altitude) > fm.AIRBORNE_MIN_ALT_M
    results = []
    for bits in (fm.PWM_RESOLUTION_BITS, None):
        trace, torque = fm.predict_torque_trace(motion, pwm_bits=bits)
        pred = fm.accel_from_trace(motion, trace, torque, fm.SILS_LOG_ALIGNMENT_DELAY_S)
        results.append(fm.relative_residual(truth, pred, airborne))
    assert np.all(results[1] > 10 * results[0])
