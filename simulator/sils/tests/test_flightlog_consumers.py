# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
# Part of StampFly Ecosystem (SILS flight-log v1 bundle consumer tests).
"""
test_flightlog_consumers.py - flight-log v1 bundle consumer tests for the SILS GUI
backend (simulator/sils/gui/server.py), the review-video renderer
(simulator/sils/viz/render_video.py), and the milestone gate
(simulator/sils/tools/sils_gate.py).

test_flightlog_consumers.py - フライトログ v1 一式の消費側テスト。対象は SILS GUI
バックエンド（simulator/sils/gui/server.py）、レビュー動画レンダラ
（simulator/sils/viz/render_video.py）、マイルストーン合否判定
（simulator/sils/tools/sils_gate.py）。

Context: docs/plans/flight-log-format-plan.md section 3.3 -- the SILS emulator no
longer writes `trajectory.csv`; each scenario run's bundle directory instead holds one
flight-log v1 bundle (`sils_<scenario>_<timestamp>.sflog.zip`, read via `lib/sflog`).
This file builds synthetic bundles in memory (no real emulator run needed) and checks
that the three consumers above read them correctly, including when optional streams
(`attitude`/`posvel`/`motor`/`rate_ref`) are absent -- the "workshop target" shape.

背景: 計画書 3.3節 -- SILS エミュレータはもう `trajectory.csv` を書かない。各シナリオ
実行のバンドルディレクトリには、代わりに1つのフライトログ v1 一式
（`sils_<scenario>_<timestamp>.sflog.zip`、`lib/sflog` 経由で読む）が入る。本ファイルは
（実エミュレータ実行なしで）合成一式をメモリ上に組み立て、上記3つの消費側が正しく
読めることを検査する。任意ストリーム（`attitude`/`posvel`/`motor`/`rate_ref`）が無い
場合（"workshop ターゲット" の形）も検査する。

@design docs/plans/flight-log-format-plan.md section 3.3 (Phase 3)
"""

from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
_LIB_DIR = _REPO_ROOT / "lib"
if str(_LIB_DIR) not in sys.path:
    sys.path.insert(0, str(_LIB_DIR))
import sflog  # noqa: E402

SILS_DIR = Path(__file__).resolve().parents[1]     # simulator/sils
GUI_DIR = SILS_DIR / "gui"
VIZ_DIR = SILS_DIR / "viz"
TOOLS_DIR = SILS_DIR / "tools"

# =============================================================================
# Synthetic flight-log v1 bundle builder
# 合成フライトログ v1 一式ビルダー
# =============================================================================
# A short synthetic hover with a small, monotonic roll rotation -- enough to give
# roll/roll_est/gyro-derived-rate a non-trivial (non-all-zero) value to assert on,
# without needing a real SILS/emulator run.
# 短い合成ホバー飛行に、小さく単調なロール回転を持たせる -- 実エミュレータ実行なしで
# roll/roll_est/ジャイロ由来レートに非自明な（全ゼロでない）値を持たせて検証できるように。

N_SAMPLES = 40
DT_US = 2500                 # virtual-clock step, ~400 Hz-ish cadence
FIRST_TIMESTAMP_US = 1_000_000
HOVER_ALT_M = 0.5
GYRO_X_RAD_S = 0.05          # constant body-FRD roll rate the synthetic flight holds
MOTOR_DUTY = 0.5


def _build_synthetic_streams(with_estimates: bool, with_motor: bool, with_rate_ref: bool) -> dict:
    """Build the DataFrames one SILS run's flight-log bundle carries.

    `truth`/`imu` are always included (schema.py's REQUIRED_STREAMS + the SILS
    per-source required list both always carry imu+truth -- protocol/spec/
    flight_log.yaml `required_streams.sils`). `attitude`/`posvel`/`motor`/`rate_ref`
    are each independently toggle-able to exercise the "some optional streams are
    absent" (workshop-target) code paths in the three consumers under test.

    1つの SILS 実行のフライトログ一式が持つ DataFrame 群を作る。`truth`/`imu` は
    常に含む（schema.py の REQUIRED_STREAMS も、SILS 向け必須ストリーム一覧も
    imu+truth を常に含む -- protocol/spec/flight_log.yaml の
    `required_streams.sils`）。`attitude`/`posvel`/`motor`/`rate_ref` は個別に
    on/off できる -- 検査対象3消費側の「一部の任意ストリームが無い」
    （workshop ターゲット）経路を検査するため。
    """
    ts = (np.arange(N_SAMPLES, dtype="int64") * DT_US + FIRST_TIMESTAMP_US)
    theta = np.linspace(0.0, 0.05, N_SAMPLES)  # small roll rotation, radians

    truth = pd.DataFrame({
        "timestamp_us": ts,
        "pos_x": 0.0, "pos_y": 0.0, "pos_z": -HOVER_ALT_M,
        "quat_w": np.cos(theta / 2), "quat_x": np.sin(theta / 2),
        "quat_y": 0.0, "quat_z": 0.0,
        "vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0,
        "rate_x": GYRO_X_RAD_S, "rate_y": 0.0, "rate_z": 0.0,
    })
    imu = pd.DataFrame({
        "timestamp_us": ts, "seq": range(N_SAMPLES),
        "gyro_x": GYRO_X_RAD_S, "gyro_y": 0.0, "gyro_z": 0.0,
        "accel_x": 0.0, "accel_y": 0.0, "accel_z": -9.81,
        "gyro_raw_x": GYRO_X_RAD_S, "gyro_raw_y": 0.0, "gyro_raw_z": 0.0,
        "accel_raw_x": 0.0, "accel_raw_y": 0.0, "accel_raw_z": -9.81,
    })
    streams = {"imu": imu, "truth": truth}

    if with_estimates:
        streams["attitude"] = pd.DataFrame({
            "timestamp_us": ts, "seq": range(N_SAMPLES),
            "quat_w": np.cos(theta / 2), "quat_x": np.sin(theta / 2),
            "quat_y": 0.0, "quat_z": 0.0,
            "gyro_bias_x": 0.0, "gyro_bias_y": 0.0, "gyro_bias_z": 0.0,
            "accel_bias_x": 0.0, "accel_bias_y": 0.0, "accel_bias_z": 0.0,
        })
        streams["posvel"] = pd.DataFrame({
            "timestamp_us": ts, "seq": range(N_SAMPLES),
            "pos_x": 0.0, "pos_y": 0.0, "pos_z": -(HOVER_ALT_M - 0.001),
            "vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0,
        })
    if with_motor:
        streams["motor"] = pd.DataFrame({
            "timestamp_us": ts, "seq": range(N_SAMPLES),
            "duty_FR": MOTOR_DUTY, "duty_RR": MOTOR_DUTY,
            "duty_RL": MOTOR_DUTY, "duty_FL": MOTOR_DUTY,
        })
    if with_rate_ref:
        streams["rate_ref"] = pd.DataFrame({
            "timestamp_us": ts, "seq": range(N_SAMPLES),
            "rate_ref_roll": GYRO_X_RAD_S, "rate_ref_pitch": 0.0, "rate_ref_yaw": 0.0,
        })
    return streams


def _write_bundle(bundle_dir: Path, scenario: str, *, with_estimates=True,
                   with_motor=True, with_rate_ref=True) -> Path:
    """Write a synthetic `sils_<scenario>_<timestamp>.sflog.zip` under `bundle_dir`
    and return its path.
    合成の `sils_<scenario>_<timestamp>.sflog.zip` を `bundle_dir` の下に書き、
    そのパスを返す。
    """
    streams = _build_synthetic_streams(with_estimates, with_motor, with_rate_ref)
    meta = sflog.make_meta(source="sils", tool_name="test_flightlog_consumers",
                            tool_version="0.0.0", streams=streams)
    log = sflog.FlightLog(meta=meta, schema=sflog.schema.schema_for(streams.keys()),
                           streams=streams)
    bundle_dir.mkdir(parents=True, exist_ok=True)
    zip_path = bundle_dir / f"sils_{scenario}_20260911T000000.sflog.zip"
    log.save(zip_path)
    return zip_path


@pytest.fixture
def full_bundle_dir(tmp_path) -> Path:
    """A run bundle dir with every stream present (imu/truth/attitude/posvel/motor/
    rate_ref) -- the "vehicle target" shape.
    全ストリームがある実行バンドル（imu/truth/attitude/posvel/motor/rate_ref）--
    "vehicle ターゲット" の形。
    """
    d = tmp_path / "out_scn_full"
    _write_bundle(d, "full")
    return d


@pytest.fixture
def minimal_bundle_dir(tmp_path) -> Path:
    """A run bundle dir with only the always-required streams (imu, truth) -- the
    "workshop target" shape (no attitude/posvel/motor/rate_ref).
    常に必須のストリーム（imu, truth）だけの実行バンドル -- "workshop ターゲット"
    の形（attitude/posvel/motor/rate_ref 無し）。
    """
    d = tmp_path / "out_scn_minimal"
    _write_bundle(d, "minimal", with_estimates=False, with_motor=False, with_rate_ref=False)
    return d


@pytest.fixture
def empty_bundle_dir(tmp_path) -> Path:
    """A run directory that exists but has not produced a flight-log bundle yet.
    ディレクトリ自体はあるが、まだフライトログ一式を書いていない実行。
    """
    d = tmp_path / "out_scn_empty"
    d.mkdir()
    return d


LEGACY_COLUMNS = [
    "t", "px", "py", "pz", "qw", "qx", "qy", "qz", "alt", "roll", "pitch",
    "yawrate", "yawcmd", "alt_est", "roll_est", "pitch_est", "m0", "m1", "m2", "m3",
]


# =============================================================================
# server.read_flightlog() (simulator/sils/gui/server.py)
# =============================================================================

def _import_server():
    if str(GUI_DIR) not in sys.path:
        sys.path.insert(0, str(GUI_DIR))
    import server  # noqa: PLC0415
    return server


def test_server_read_flightlog_legacy_shape(full_bundle_dir):
    """The 20 legacy columns come back, all the same length, with alt/roll in the
    units app.js expects (metres / degrees).
    旧20列が全て同じ長さで返り、alt/roll は app.js が期待する単位（メートル／度）。
    """
    server = _import_server()
    traj = server.read_flightlog(full_bundle_dir)

    assert traj["columns"] == LEGACY_COLUMNS
    assert set(traj["data"].keys()) == set(LEGACY_COLUMNS)
    assert {len(v) for v in traj["data"].values()} == {N_SAMPLES}

    assert traj["data"]["alt"] == pytest.approx([HOVER_ALT_M] * N_SAMPLES, abs=1e-9)
    # roll is in DEGREES (a display-only conversion -- app.js labels it "[deg]"):
    # the truth quaternion sweeps 0..0.05 rad, i.e. 0..~2.86 deg.
    roll_deg = traj["data"]["roll"]
    assert max(roll_deg) == pytest.approx(np.degrees(0.05), abs=1e-2)
    assert min(roll_deg) == pytest.approx(0.0, abs=1e-6)
    assert all(v == 0.0 for v in traj["data"]["yawcmd"])
    # motor stream present -> m0..m3 are real numbers, not null
    assert traj["data"]["m0"] == pytest.approx([MOTOR_DUTY] * N_SAMPLES)
    assert all(v is not None for v in traj["data"]["alt_est"])

    # A bare NaN is not valid JSON -- confirm the payload actually serializes clean.
    assert "NaN" not in json.dumps(traj)


def test_server_read_flightlog_optional_streams_absent_are_null(minimal_bundle_dir):
    """When `motor`/`attitude`/`posvel` are absent from the bundle (workshop
    target), the corresponding legacy columns are JSON null, not 0 or NaN.
    `motor`/`attitude`/`posvel` が一式に無い（workshop ターゲット）場合、
    対応する旧列は 0 でも NaN でもなく JSON の null になる。
    """
    server = _import_server()
    traj = server.read_flightlog(minimal_bundle_dir)

    assert traj["data"]["m0"] == [None] * N_SAMPLES
    assert traj["data"]["m1"] == [None] * N_SAMPLES
    assert traj["data"]["alt_est"] == [None] * N_SAMPLES
    assert traj["data"]["roll_est"] == [None] * N_SAMPLES
    # truth-derived columns are unaffected by the missing optional streams
    assert traj["data"]["alt"] == pytest.approx([HOVER_ALT_M] * N_SAMPLES, abs=1e-9)
    assert "NaN" not in json.dumps(traj)


def test_server_read_flightlog_no_bundle_returns_empty(empty_bundle_dir):
    """No `sils_*.sflog.zip` yet -> {} (mirrors the old read_trajectory()'s
    "file missing" tolerance, so app.js's "no trajectory" message still triggers).
    まだ `sils_*.sflog.zip` が無い -> {}（旧 read_trajectory() の「ファイル無し」
    許容に合わせる。app.js の「軌跡なし」表示がそのまま働く）。
    """
    server = _import_server()
    assert server.read_flightlog(empty_bundle_dir) == {}


# =============================================================================
# render_video.load_flightlog() (simulator/sils/viz/render_video.py)
# =============================================================================
# render_video.py normally runs under the dedicated SILS viz venv (mujoco +
# matplotlib + imageio, no pandas by default -- see the module's own comment on
# this) which this repo's `.venv` does not replicate (no mujoco/imageio here). Since
# `mujoco`/`imageio` are only imported at module load time and never called by
# load_flightlog() itself, stub them out in sys.modules before import so the module
# loads cleanly and only its pure numpy/pandas logic is exercised.
# render_video.py は通常、専用の SILS viz venv（mujoco + matplotlib + imageio。
# 既定では pandas 無し -- モジュール自身のコメント参照）で動く。このリポジトリの
# `.venv` にはそれが無い（mujoco/imageio 無し）。`mujoco`/`imageio` はモジュール
# 読み込み時に import されるだけで load_flightlog() 自体は呼ばないため、import 前に
# sys.modules へダミーを差し込んでおけば、素の numpy/pandas ロジックだけを検査できる。

def _import_render_video(monkeypatch):
    monkeypatch.setitem(sys.modules, "mujoco", types.ModuleType("mujoco"))
    imageio_stub = types.ModuleType("imageio")
    imageio_stub.v2 = types.ModuleType("imageio.v2")
    monkeypatch.setitem(sys.modules, "imageio", imageio_stub)
    monkeypatch.setitem(sys.modules, "imageio.v2", imageio_stub.v2)
    if str(VIZ_DIR) not in sys.path:
        sys.path.insert(0, str(VIZ_DIR))
    sys.modules.pop("render_video", None)  # force a fresh import against the stubs above
    import render_video  # noqa: PLC0415
    return render_video


def test_render_video_load_flightlog_shape(full_bundle_dir, monkeypatch):
    render_video = _import_render_video(monkeypatch)
    traj = render_video.load_flightlog(full_bundle_dir)

    expected_keys = {
        "t", "px", "py", "pz", "qw", "qx", "qy", "qz", "alt", "alt_est",
        "roll", "pitch", "roll_est", "pitch_est", "yawrate", "yawcmd",
        "m0", "m1", "m2", "m3",
    }
    assert set(traj.keys()) == expected_keys
    assert {len(v) for v in traj.values()} == {N_SAMPLES}
    assert np.allclose(traj["alt"], HOVER_ALT_M)
    assert np.allclose(traj["yawcmd"], 0.0)
    assert np.allclose(traj["m0"], MOTOR_DUTY)
    # px/py/pz and qw..qz must be finite (the MuJoCo/ENU frame the 3D renderer needs)
    for key in ("px", "py", "pz", "qw", "qx", "qy", "qz"):
        assert np.all(np.isfinite(traj[key]))


def test_render_video_load_flightlog_missing_motor_is_nan(minimal_bundle_dir, monkeypatch):
    """No `motor`/`attitude`/`posvel` stream -> NaN (a plotted gap), not an error."""
    render_video = _import_render_video(monkeypatch)
    traj = render_video.load_flightlog(minimal_bundle_dir)

    assert np.all(np.isnan(traj["m0"]))
    assert np.all(np.isnan(traj["roll_est"]))
    assert np.all(np.isnan(traj["alt_est"]))
    # truth-derived arrays are unaffected
    assert np.allclose(traj["alt"], HOVER_ALT_M)


def test_render_video_load_flightlog_no_bundle_raises(empty_bundle_dir, monkeypatch):
    render_video = _import_render_video(monkeypatch)
    with pytest.raises(ValueError):
        render_video.load_flightlog(empty_bundle_dir)


# =============================================================================
# sils_gate._has_flightlog_bundle() (simulator/sils/tools/sils_gate.py)
# =============================================================================

def _import_sils_gate():
    if str(TOOLS_DIR) not in sys.path:
        sys.path.insert(0, str(TOOLS_DIR))
    sys.modules.pop("sils_gate", None)
    import sils_gate  # noqa: PLC0415
    return sils_gate


def test_sils_gate_has_flightlog_bundle_true_when_zip_present(full_bundle_dir):
    sils_gate = _import_sils_gate()
    assert sils_gate._has_flightlog_bundle(str(full_bundle_dir)) is True


def test_sils_gate_has_flightlog_bundle_false_when_absent(empty_bundle_dir):
    sils_gate = _import_sils_gate()
    assert sils_gate._has_flightlog_bundle(str(empty_bundle_dir)) is False


def test_sils_gate_has_flightlog_bundle_false_for_nonexistent_dir(tmp_path):
    sils_gate = _import_sils_gate()
    assert sils_gate._has_flightlog_bundle(str(tmp_path / "does_not_exist")) is False
