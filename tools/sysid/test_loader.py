#!/usr/bin/env python3
"""
test_loader.py - tests for tools/sysid/loader.py's load_aligned(), the
single bundle-based replacement for the old "stream"/"convert" CSV format
detectors (docs/plans/flight-log-format-plan.md section 3.2).
test_loader.py - tools/sysid/loader.py の load_aligned()（旧
"stream"/"convert" CSV 形式判別の単一バンドルベース置き換え）の試験。

Covers:
  - the aligned-DataFrame column contract documented in loader.py's
    docstring (imu/attitude/posvel/rate_ref/motor/ctrl_output/status all
    present -> all their columns show up on one 400Hz-aligned table);
  - the "motor stream absent" fallback: when only `ctrl_ref` supplies
    duty_FR/RR/RL/FL (no `motor` stream in the bundle), aligned() keeps the
    BARE `duty_FR` name (no collision) and `df.attrs["bundle_streams"]`
    is the exact, non-heuristic signal a caller uses to tell this apart
    from genuine 400Hz motor-stream duty (see plant_fit.py's
    _load_axis_data() and rate_sysid.py's fit_from_df(), both of which
    build a synthetic duty_rate_hz array from this attrs flag instead of
    guessing from repeated-row patterns in the data).

対象:
  - loader.py の docstring が定める整列済み DataFrame の列契約
    （imu/attitude/posvel/rate_ref/motor/ctrl_output/status が全て
    あれば、1枚の400Hz整列表に全ての列が現れる）;
  - "motor ストリーム不在" のフォールバック: `ctrl_ref` だけが
    duty_FR/RR/RL/FL を供給する場合（一式に `motor` ストリームが無い）、
    aligned() は素の `duty_FR` 名を保つ（衝突なし）こと、および
    `df.attrs["bundle_streams"]` が、本物の400Hz motorストリーム由来の
    dutyと区別する厳密な（ヒューリスティックでない）シグナルであること
    （plant_fit.py の _load_axis_data()、rate_sysid.py の fit_from_df()
    はどちらもこの attrs フラグから合成の duty_rate_hz 配列を作る --
    データ中の重複行パターンから推測しない）。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # tools/
from sysid.loader import load_aligned  # noqa: E402

import sflog  # noqa: E402
from sflog.bundle import FlightLog, make_meta  # noqa: E402


def _lockstep(n: int, dt: float) -> tuple:
    """timestamp_us + seq for a lockstep stream, n rows starting at t=0.
    ロックステップ系ストリーム用の timestamp_us + seq（t=0開始、n行）。"""
    t = np.arange(n) * dt
    ts_us = np.round(t * 1e6).astype(np.int64)
    return ts_us, np.arange(n)


def _full_bundle_streams() -> dict:
    """imu/attitude/posvel/rate_ref/motor/ctrl_output at 400Hz (lockstep,
    shared seq) + ctrl_ref/status held at 50Hz/1Hz -- exercises every
    column family load_aligned()'s docstring documents.
    imu/attitude/posvel/rate_ref/motor/ctrl_output を400Hz（ロックステップ、
    seq共有）+ ctrl_ref/status を50Hz/1Hzで保持 -- load_aligned() の
    docstring が記す全ての列ファミリを網羅する。
    """
    n = 40  # 0.1s @ 400Hz
    dt = 1.0 / 400.0
    ts_us, seq = _lockstep(n, dt)

    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": np.full(n, 0.1), "gyro_y": np.full(n, 0.2), "gyro_z": np.full(n, 0.3),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.full(n, -9.81),
        "gyro_raw_x": np.full(n, 0.1), "gyro_raw_y": np.full(n, 0.2), "gyro_raw_z": np.full(n, 0.3),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.full(n, -9.81),
    })
    attitude = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "quat_w": np.ones(n), "quat_x": np.zeros(n), "quat_y": np.zeros(n), "quat_z": np.zeros(n),
        "gyro_bias_x": np.zeros(n), "gyro_bias_y": np.zeros(n), "gyro_bias_z": np.zeros(n),
        "accel_bias_x": np.zeros(n), "accel_bias_y": np.zeros(n), "accel_bias_z": np.zeros(n),
    })
    posvel = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "pos_x": np.zeros(n), "pos_y": np.zeros(n), "pos_z": np.full(n, -0.5),
        "vel_x": np.zeros(n), "vel_y": np.zeros(n), "vel_z": np.zeros(n),
    })
    rate_ref = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "rate_ref_roll": np.full(n, 0.5), "rate_ref_pitch": np.zeros(n), "rate_ref_yaw": np.zeros(n),
    })
    motor = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "duty_FR": np.full(n, 0.55), "duty_RR": np.full(n, 0.56),
        "duty_RL": np.full(n, 0.57), "duty_FL": np.full(n, 0.58),
    })
    ctrl_output = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "thrust": np.full(n, 0.36), "torque_roll": np.full(n, 1e-3),
        "torque_pitch": np.zeros(n), "torque_yaw": np.zeros(n),
    })

    # ctrl_ref: 50Hz, held -- its own duty_FR/RR/RL/FL COLLIDE with motor's
    # (motor merges first in schema.STREAMS order) and get renamed
    # ctrl_ref_duty_FR etc.
    # ctrl_ref: 50Hz、保持 -- 自身の duty_FR/RR/RL/FL は motor のものと
    # 衝突し（motor が schema.STREAMS の順で先に結合される）、
    # ctrl_ref_duty_FR 等へ改名される。
    n_ref = 5  # 0.1s @ 50Hz
    t_ref = np.arange(n_ref) / 50.0
    ctrl_ref = pd.DataFrame({
        "timestamp_us": np.round(t_ref * 1e6).astype(np.int64),
        "flight_mode": np.zeros(n_ref, dtype=int),
        "angle_ref_roll": np.zeros(n_ref), "angle_ref_pitch": np.zeros(n_ref),
        "total_thrust": np.full(n_ref, 0.36),
        "duty_FR": np.full(n_ref, 0.50), "duty_RR": np.full(n_ref, 0.50),
        "duty_RL": np.full(n_ref, 0.50), "duty_FL": np.full(n_ref, 0.50),
        "alt_setpoint": np.zeros(n_ref), "alt_vel_target": np.zeros(n_ref),
        "climb_rate_cmd": np.zeros(n_ref), "pos_setpoint_x": np.zeros(n_ref),
        "pos_setpoint_y": np.zeros(n_ref),
    })

    status = pd.DataFrame({
        "timestamp_us": np.array([0], dtype=np.int64),
        "uptime_ms": np.array([0], dtype=np.int64),
        "voltage": np.array([3.75]),
        "current_ma": np.array([500.0]),
        "flight_state": np.array([1], dtype=int),
        "sensor_health": np.array([0], dtype=int),
        "eskf_status": np.array([1], dtype=int),
        "reset_reason": np.array([1], dtype=int),
        "pid_roll_kp": np.array([1.0e-3]), "pid_roll_ti": np.array([0.7]), "pid_roll_td": np.array([0.002]),
        "pid_pitch_kp": np.array([1.4e-3]), "pid_pitch_ti": np.array([0.7]), "pid_pitch_td": np.array([0.025]),
        "pid_yaw_kp": np.array([0.8e-3]), "pid_yaw_ti": np.array([0.8]), "pid_yaw_td": np.array([0.01]),
    })

    return {
        "imu": imu, "attitude": attitude, "posvel": posvel, "rate_ref": rate_ref,
        "motor": motor, "ctrl_output": ctrl_output, "ctrl_ref": ctrl_ref, "status": status,
    }


def _save_bundle(tmp_path: Path, streams: dict, name: str = "test.sflog.zip") -> Path:
    meta = make_meta(source="sils", tool_name="test_loader.py", tool_version="0", streams=streams)
    log = FlightLog(meta=meta, schema={}, streams=streams)
    path = tmp_path / name
    log.save(path)
    return path


# =============================================================================
# Column contract
# =============================================================================

def test_load_aligned_full_bundle_column_contract(tmp_path):
    """Every documented column family shows up once all 8 streams are
    present, at the imu-base 400Hz row count.
    8ストリーム全てが揃えば、imu基準400Hzの行数で、文書化された全ての
    列ファミリが現れる。
    """
    path = _save_bundle(tmp_path, _full_bundle_streams())
    df = load_aligned(path)

    assert len(df) == 40  # imu's row count / imu の行数
    for col in ("timestamp_us", "seq", "gyro_x", "gyro_y", "gyro_z",
                "accel_x", "accel_y", "accel_z"):
        assert col in df.columns, col

    # attitude/posvel/rate_ref/ctrl_output: lockstep, merged bare (no collision)
    # attitude/posvel/rate_ref/ctrl_output: ロックステップ系、素の名前で結合（衝突なし）
    for col in ("quat_w", "quat_x", "quat_y", "quat_z",
                "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z",
                "rate_ref_roll", "rate_ref_pitch", "rate_ref_yaw",
                "thrust", "torque_roll", "torque_pitch", "torque_yaw"):
        assert col in df.columns, col

    # motor: genuine 400Hz duty, bare name (merges before ctrl_ref)
    # motor: 本物の400Hz duty、素の名前（ctrl_refより先に結合）
    for col in ("duty_FR", "duty_RR", "duty_RL", "duty_FL"):
        assert col in df.columns, col
    assert np.allclose(df["duty_FR"].to_numpy(), 0.55)

    # ctrl_ref: held (50Hz), its duty_* renamed due to the collision with motor
    # ctrl_ref: 保持（50Hz）、motorとの衝突により duty_* は改名される
    for col in ("ctrl_ref_duty_FR", "ctrl_ref_duty_RR", "ctrl_ref_duty_RL", "ctrl_ref_duty_FL",
                "total_thrust", "flight_mode", "angle_ref_roll", "angle_ref_pitch",
                "ctrl_ref_timestamp_us"):
        assert col in df.columns, col
    assert np.allclose(df["ctrl_ref_duty_FR"].to_numpy(), 0.50)

    # status: held (1Hz)
    # status: 保持（1Hz）
    for col in ("voltage", "status_timestamp_us"):
        assert col in df.columns, col
    assert np.allclose(df["voltage"].to_numpy(), 3.75)

    # provenance metadata attached by load_aligned() itself
    # load_aligned() 自身が付与する来歴メタデータ
    assert df.attrs["bundle_streams"] == set(_full_bundle_streams().keys())
    assert df.attrs["bundle_path"] == str(path)


def test_load_aligned_defaults_method_hold_backward(tmp_path):
    """method="hold" (the default) never looks into the future -- a held
    column's value at a base row equals the most recent OLDER (or exactly
    concurrent) observation, never a later one.
    method="hold"（既定）は将来を見ない -- 保持列の値は基準行の時刻以前
    （または同時刻）の直近観測であり、未来の値ではない。
    """
    path = _save_bundle(tmp_path, _full_bundle_streams())
    df = load_aligned(path)
    # ctrl_ref's first row is at t=0 -- every held value up to the next
    # ctrl_ref update (20ms later) must equal that first row's value, and
    # never anticipate the second ctrl_ref row.
    # ctrl_ref の最初の行は t=0 -- 次の ctrl_ref 更新（20ms後）までの保持値
    # は全て最初の行の値と一致し、2番目の ctrl_ref 行を先取りしない。
    first_period = df["timestamp_us"] < 20_000
    assert np.allclose(df.loc[first_period, "total_thrust"].to_numpy(), 0.36)


# =============================================================================
# "motor stream absent" -- non-heuristic detection via df.attrs
# =============================================================================

def test_load_aligned_motor_absent_uses_ctrl_ref_duty_bare_name(tmp_path):
    """When `motor` is NOT in the bundle, ctrl_ref's duty_FR/RR/RL/FL keep
    their BARE names (no collision partner merged first) -- this is the
    50Hz-held value, not genuine 400Hz duty, and df.attrs["bundle_streams"]
    is how a caller (plant_fit.py, rate_sysid.py) tells the difference
    without guessing from the data.
    `motor` が一式に無いとき、ctrl_ref の duty_FR/RR/RL/FL は素の名前を
    保つ（衝突相手が先に結合されないため）-- これは50Hz保持値であり本物の
    400Hz dutyではない。呼び出し側（plant_fit.py・rate_sysid.py）は
    データから推測せず df.attrs["bundle_streams"] でこれを判別する。
    """
    streams = _full_bundle_streams()
    del streams["motor"]
    path = _save_bundle(tmp_path, streams, name="no_motor.sflog.zip")
    df = load_aligned(path)

    assert "motor" not in df.attrs["bundle_streams"]
    # bare name present (from ctrl_ref), no ctrl_ref_duty_FR this time
    # 素の名前が存在する（ctrl_ref 由来）、今回は ctrl_ref_duty_FR は無い
    assert "duty_FR" in df.columns
    assert "ctrl_ref_duty_FR" not in df.columns
    # the value is ctrl_ref's (0.50), not motor's (0.55) -- proves it came
    # from the 50Hz stream, held onto the 400Hz base.
    # 値は ctrl_ref のもの（0.50）であり motor のもの（0.55）ではない --
    # 50Hzストリームから400Hz基準へ保持されたことの証拠。
    assert np.allclose(df["duty_FR"].to_numpy(), 0.50)
    # its provenance column is present, proving this is a HELD value
    # 随伴列が存在し、これが保持値であることを示す
    assert "ctrl_ref_timestamp_us" in df.columns


def test_load_aligned_motor_present_beats_ctrl_ref_duty(tmp_path):
    """The mirror case: when `motor` IS present, "duty_FR" is motor's
    (genuine 400Hz) value, and ctrl_ref's own copy is pushed to
    "ctrl_ref_duty_FR" instead -- restated here as its own test (rather
    than only asserted inside the "full bundle" test above) so a future
    change to that larger test cannot silently stop covering this case.
    対称のケース: `motor` があるとき "duty_FR" は motor の（本物の400Hz）
    値であり、ctrl_ref 自身のコピーは "ctrl_ref_duty_FR" へ追いやられる --
    上の「full bundle」試験の中だけで確認するのではなく独立した試験として
    再掲し、将来あちらの試験が変わってもこのケースの確認が黙って抜け
    落ちないようにする。
    """
    path = _save_bundle(tmp_path, _full_bundle_streams(), name="with_motor.sflog.zip")
    df = load_aligned(path)
    assert "motor" in df.attrs["bundle_streams"]
    assert np.allclose(df["duty_FR"].to_numpy(), 0.55)
    assert np.allclose(df["ctrl_ref_duty_FR"].to_numpy(), 0.50)


def test_load_aligned_minimal_bundle_imu_only(tmp_path):
    """A bundle with only the required `imu` stream still loads fine --
    every optional column family is simply absent.
    必須の `imu` ストリームだけの一式でも問題なく読み込める --
    任意の列ファミリは単に存在しないだけ。
    """
    n = 10
    ts_us, seq = _lockstep(n, 1.0 / 400.0)
    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": np.zeros(n), "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.full(n, -9.81),
        "gyro_raw_x": np.zeros(n), "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.full(n, -9.81),
    })
    path = _save_bundle(tmp_path, {"imu": imu}, name="minimal.sflog.zip")
    df = load_aligned(path)
    assert len(df) == n
    assert df.attrs["bundle_streams"] == {"imu"}
    assert "duty_FR" not in df.columns
    assert "rate_ref_roll" not in df.columns


def test_load_aligned_rejects_unknown_base(tmp_path):
    """base="mag" on a bundle without a mag stream raises ValueError,
    propagated unchanged from sflog.aligned() -- load_aligned() adds no
    swallowing/rewrapping of this error.
    mag ストリームの無い一式に base="mag" を指定すると ValueError --
    sflog.aligned() からそのまま伝播する。load_aligned() はこのエラーを
    握りつぶしたり包み直したりしない。
    """
    n = 5
    ts_us, seq = _lockstep(n, 1.0 / 400.0)
    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": np.zeros(n), "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.zeros(n),
        "gyro_raw_x": np.zeros(n), "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.zeros(n),
    })
    path = _save_bundle(tmp_path, {"imu": imu}, name="no_mag.sflog.zip")
    with pytest.raises(ValueError):
        load_aligned(path, base="mag")


def test_load_aligned_accepts_extracted_directory(tmp_path):
    """load_aligned() works on an extracted-directory bundle, not just a
    `.sflog.zip` file (both are valid per lib/sflog/bundle.py).
    load_aligned() は `.sflog.zip` だけでなく展開済みフォルダの一式でも
    動作する（どちらも lib/sflog/bundle.py 上有効）。
    """
    n = 5
    ts_us, seq = _lockstep(n, 1.0 / 400.0)
    imu = pd.DataFrame({
        "timestamp_us": ts_us, "seq": seq,
        "gyro_x": np.zeros(n), "gyro_y": np.zeros(n), "gyro_z": np.zeros(n),
        "accel_x": np.zeros(n), "accel_y": np.zeros(n), "accel_z": np.zeros(n),
        "gyro_raw_x": np.zeros(n), "gyro_raw_y": np.zeros(n), "gyro_raw_z": np.zeros(n),
        "accel_raw_x": np.zeros(n), "accel_raw_y": np.zeros(n), "accel_raw_z": np.zeros(n),
    })
    directory = tmp_path / "bundle_dir"
    meta = make_meta(source="sils", tool_name="test_loader.py", tool_version="0",
                     streams={"imu": imu})
    FlightLog(meta=meta, schema={}, streams={"imu": imu}).save(directory)
    assert sflog.is_bundle(directory)

    df = load_aligned(directory)
    assert len(df) == n
