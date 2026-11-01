#!/usr/bin/env python3
"""
test_forward_check.py - falsification test of the forward-model pass criteria.
test_forward_check.py - 順モデル照合の合格基準が実際に判別できることの反証試験。

The criteria of `sf sils sysid-gate` (analysis/datasets/flightlog/forward_check.yaml) are
only worth anything if a wrong propulsion model FAILS them.  This test runs the two
published real flip logs through
  - the SILS plant's R_out model: must PASS, with the values of
    analysis/notes/forward_model_check_20261101.md;
  - the old D*V model (the plant before 2026-10; written out below, self-contained):
    must FAIL, on every one of the four roll criteria in both logs.
`sf sils sysid-gate` の基準は、誤った推進系モデルが不合格になって初めて意味を持つ。公開した
2 本の実機宙返りログを (a) R_out モデル (合格し、報告書の値に一致すること)、(b) 旧 D×V
モデル (2026-10 以前のプラント。下に自己完結で記述。ロールの 4 基準すべて・両ログで不合格に
なること) に通す。

Runtime / 実行時間: about 10 s measured (the module-scoped fixtures run R_out and the old model once, 2 logs each).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_REPO_ROOT / "lib"))

from tools.sysid import _generated_params as gp  # noqa: E402
from tools.sysid import forward_check as fc  # noqa: E402
from tools.sysid import forward_model as fm  # noqa: E402

SPEC_PATH = fc.default_criteria_path(_REPO_ROOT)
LOG_DIR = SPEC_PATH.parent
INERTIA = np.array([gp.EXPECTED_IXX, gp.EXPECTED_IYY, gp.EXPECTED_IZZ])

# Expected R_out values (report chapter 5) and the tolerance of this test (the numbers
# are deterministic; the margin only absorbs library-version noise).
# R_out の期待値 (報告書 5 章) とこの試験の許容 (値は決定論的。許容はライブラリ差のみ)。
EXPECTED_ROUT = {
    # log index: (nofit residual, best delay ms, hover gain, hover delay ms)
    0: (0.311, 4.0, 0.775, 4.4),
    1: (0.285, 3.0, 0.837, 4.2),
}
TOL_RESIDUAL, TOL_DELAY_MS, TOL_GAIN, TOL_HOVER_DELAY_MS = 0.01, 1.0, 0.02, 0.3

# Old D*V motor model (analysis/scripts/ct_cq_bench_20261015/crosscheck/flipcheck/flipcheck.py,
# "M_now"): J dw/dt = Km (D V - Km w)/Rm - Cq w^2 - Qf, thrust = eta Ct w^2.
# 旧 D×V モータモデル (flipcheck.py の "M_now")。
_OLD = dict(J=1.375e-8, Km=5.682e-4, Rm=0.593, Cq=4.10e-11, Qf=9.507e-6, Ct=1.00e-8, eta=0.7133)
_OLD_SUBSTEPS = 5                       # RK4 steps per control period / 制御周期あたりの RK4 刻み数
_ROLL_SIGN = np.array([-1, -1, 1, 1.0])   # [FR, RR, RL, FL]
_PITCH_SIGN = np.array([1, -1, -1, 1.0])


def _old_omega_dot(w, duty, volt):
    p = _OLD
    return (p["Km"] * (duty * volt - p["Km"] * w) / p["Rm"] - p["Cq"] * w * w - p["Qf"]) / p["J"]


def _old_rk4(w, duty, volt, h):
    k1 = _old_omega_dot(w, duty, volt)
    k2 = _old_omega_dot(w + 0.5 * h * k1, duty, volt)
    k3 = _old_omega_dot(w + 0.5 * h * k2, duty, volt)
    k4 = _old_omega_dot(w + h * k3, duty, volt)
    return np.maximum(w + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4), 0)


def _old_model(motion):
    """Old D*V model as a ModelFactory. Yaw is not modelled (0). / 旧モデル。ヨーは 0。"""
    h = motion.dt / _OLD_SUBSTEPS
    duty = np.clip(motion.duty, 0, 1)
    w = np.zeros(4)
    for _ in range(int(1.0 / h)):              # 1 s pre-run = steady state at the first row
        w = _old_rk4(w, duty[0], motion.voltage[0], h)
    mean_w2 = np.empty_like(duty)
    for n in range(len(duty)):
        before = w
        for _ in range(_OLD_SUBSTEPS):
            w = _old_rk4(w, duty[n], motion.voltage[n], h)
        mean_w2[n] = (0.5 * (before + w)) ** 2
    thrust = _OLD["eta"] * _OLD["Ct"] * mean_w2
    tau = np.zeros((len(duty), 3))
    tau[:, 0] = gp.EXPECTED_ARM * (thrust @ _ROLL_SIGN)
    tau[:, 1] = gp.EXPECTED_ARM * (thrust @ _PITCH_SIGN)
    alpha = tau / INERTIA
    grid = np.arange(len(alpha)) * motion.dt

    def alpha_fn(delay_s):
        return np.stack([np.interp(grid - delay_s, grid, alpha[:, a]) for a in range(3)], axis=1)

    return alpha_fn


def _by_log(rows):
    """{log index: {metric name: Metric}} for the roll axis. / ロール軸の指標を引く。"""
    names = sorted({r.log for r in rows})
    return {i: {r.name: r for r in rows if r.log == name and r.axis == "roll"}
            for i, name in enumerate(names)}


@pytest.fixture(scope="module")
def rout_rows():
    return fc.evaluate_reference_logs(SPEC_PATH)


@pytest.fixture(scope="module")
def old_rows():
    return fc.evaluate_reference_logs(SPEC_PATH, model=_old_model)


def test_rout_model_passes_with_reported_values(rout_rows):
    assert fc.overall_pass(rout_rows)
    for index, (residual, delay, gain, hover_delay) in EXPECTED_ROUT.items():
        m = _by_log(rout_rows)[index]
        assert m["flip_nofit_residual"].value == pytest.approx(residual, abs=TOL_RESIDUAL)
        assert m["flip_best_delay_ms"].value == pytest.approx(delay, abs=TOL_DELAY_MS)
        assert m["hover_gain"].value == pytest.approx(gain, abs=TOL_GAIN)
        assert m["hover_delay_ms"].value == pytest.approx(hover_delay, abs=TOL_HOVER_DELAY_MS)


def test_pitch_rows_are_reference_only(rout_rows):
    pitch = [r for r in rout_rows if r.axis == "pitch"]
    assert pitch and all(r.status == fc.STATUS_REFERENCE for r in pitch)


def test_old_dxv_model_fails_every_roll_criterion_in_both_logs(old_rows):
    # Falsification: if any of these passed, that criterion could not tell the old model
    # from the R_out model.  Reported old values: residual 1.04 / 0.97, delay 14 / 17 ms,
    # hover gain 0.38 / 0.36, hover delay 14.4 ms.
    # 反証: どれか 1 つでも PASS なら、その基準は旧モデルと R_out を区別できない。
    assert not fc.overall_pass(old_rows)
    for index, metrics in _by_log(old_rows).items():
        for name in ("flip_nofit_residual", "flip_best_delay_ms", "hover_gain", "hover_delay_ms"):
            assert metrics[name].status == fc.STATUS_FAIL, (index, name, metrics[name].value)
