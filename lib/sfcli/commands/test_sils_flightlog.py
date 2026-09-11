#!/usr/bin/env python3
"""
test_sils_flightlog.py - tests for lib/sfcli/commands/sils.py's flight-log
bundle helpers: `_finalize_flightlog()` (assembles the `*.sflog.zip` a SILS
run writes -- docs/plans/flight-log-format-plan.md section 3.3, Phase 3) and
`_bundle_metric()` (the `.expect` `metric` DSL's numerical gate, reading a
StampFly flight-log v1 bundle instead of the retired trajectory.csv).
test_sils_flightlog.py - lib/sfcli/commands/sils.py のフライトログ一式
ヘルパーの試験: `_finalize_flightlog()`（SILS 実行が書く `*.sflog.zip` を
組み立てる -- 計画書 3.3節 Phase 3）と `_bundle_metric()`（`.expect` の
`metric` DSL の数値ゲート。廃止した trajectory.csv の代わりに StampFly
フライトログ v1 一式を読む）。

Usage:
    .venv/bin/python3 -m pytest lib/sfcli/commands/test_sils_flightlog.py -q
"""

from __future__ import annotations

import json
import sys
import zipfile
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))  # lib/
from sfcli.commands.sils import _bundle_metric, _finalize_flightlog  # noqa: E402

import sflog  # noqa: E402

# tools/log_analyzer/conftest.py's build_synthetic_log() -- reused here (not
# a pytest fixture, a plain function) so _finalize_flightlog()'s test does
# not have to hand-build a full 17-stream bundle from scratch.
# tools/log_analyzer/conftest.py の build_synthetic_log()（フィクスチャでは
# なく素の関数）を再利用し、_finalize_flightlog() の試験が17ストリームの
# 一式をゼロから手組みしなくて済むようにする。
_LOG_ANALYZER_DIR = Path(__file__).resolve().parent.parent.parent.parent / "tools" / "log_analyzer"
sys.path.insert(0, str(_LOG_ANALYZER_DIR))
import conftest as log_analyzer_conftest  # noqa: E402


# =============================================================================
# _finalize_flightlog()
# =============================================================================


def test_finalize_flightlog_builds_zip(tmp_path):
    """A flightlog dir (synthetic bundle CSVs + a hand-written gains.json,
    matching what emu_flightlog_vehicle.cpp writes) plus a hand-written
    events.jsonl (matching emu_record.cpp's line format) are assembled into
    one *.sflog.zip: meta.source == "sils", the events stream has the right
    columns/values, gains.json rides along as an extra zip member, and the
    loose directory is removed.
    合成一式の CSV 群＋手書き gains.json（emu_flightlog_vehicle.cpp が書く
    形）と手書き events.jsonl（emu_record.cpp の行形式）を1個の *.sflog.zip
    に組み立てる: meta.source=="sils"、events ストリームの列/値が正しい、
    gains.json が zip の追加メンバとして乗る、元ディレクトリが消える。
    """
    flightlog_dir = tmp_path / "flightlog"
    log_analyzer_conftest.build_synthetic_log().save(flightlog_dir)

    gains = {
        "roll": {"kp": 1.0e-3, "ti": 0.5, "td": 0.005, "limit": 0.02},
        "pitch": {"kp": 1.0e-3, "ti": 0.5, "td": 0.005, "limit": 0.02},
        "yaw": {"kp": 5.0e-4, "ti": 1.0, "td": 0.0, "limit": 0.01},
    }
    (flightlog_dir / "gains.json").write_text(json.dumps(gains), encoding="utf-8")

    events_jsonl = tmp_path / "events.jsonl"
    events_jsonl.write_text(
        '{"t_us":1000,"ch":"wind","note":"gust start"}\n'
        '{"t_us":2000,"ch":"espnow","n":3,"bytes":"010203"}\n',
        encoding="utf-8",
    )

    zip_path = tmp_path / "sils_test_20260101T000000.sflog.zip"
    result = _finalize_flightlog(flightlog_dir, zip_path, notes="unit test",
                                 events_jsonl=events_jsonl)

    assert result == zip_path
    assert zip_path.exists()
    assert not flightlog_dir.exists(), "flightlog_dir must be removed after a successful save"

    log = sflog.FlightLog.load(zip_path)
    assert log.meta["source"] == "sils"
    assert log.meta["tool"]["name"] == "sf sils"
    assert log.meta["notes"] == "unit test"

    assert "events" in log.streams
    events_df = log.streams["events"]
    assert list(events_df.columns) == ["timestamp_us", "event", "value"]
    assert len(events_df) == 2
    assert events_df.iloc[0]["timestamp_us"] == 1000
    assert events_df.iloc[0]["event"] == "wind"
    assert events_df.iloc[0]["value"] == "gust start"
    assert events_df.iloc[1]["timestamp_us"] == 2000
    assert events_df.iloc[1]["event"] == "espnow"
    assert events_df.iloc[1]["value"] == "n=3 bytes=010203"

    with zipfile.ZipFile(zip_path) as zf:
        assert "gains.json" in zf.namelist()
        assert json.loads(zf.read("gains.json").decode("utf-8")) == gains


def test_finalize_flightlog_no_csv_returns_none(tmp_path):
    """An empty (or missing) flightlog dir -- e.g. a target not wired for
    SILS_EMU_FLIGHTLOG, or a crash before the first CSV write -- yields no
    bundle: _finalize_flightlog() returns None and writes no zip.
    空（または未作成）の flightlog ディレクトリ -- SILS_EMU_FLIGHTLOG 未配線の
    ターゲットや、最初の CSV 書き込み前のクラッシュ -- は一式を生成しない:
    _finalize_flightlog() は None を返し zip も書かない。
    """
    empty_dir = tmp_path / "empty_flightlog"
    empty_dir.mkdir()
    zip_path = tmp_path / "sils_test_20260101T000000.sflog.zip"

    assert _finalize_flightlog(empty_dir, zip_path, notes="x") is None
    assert not zip_path.exists()

    missing_dir = tmp_path / "does_not_exist"
    assert _finalize_flightlog(missing_dir, zip_path, notes="x") is None
    assert not zip_path.exists()


def test_finalize_flightlog_removes_stale_zip(tmp_path):
    """Exactly one bundle remains per run directory: a stale *.sflog.zip from
    a previous run is removed before the new one is written.
    実行ディレクトリごとに一式は1個だけ残る: 前回実行の残骸 *.sflog.zip は
    新しい zip を書く前に消される。
    """
    bundle_dir = tmp_path / "bundle"
    bundle_dir.mkdir()
    stale_zip = bundle_dir / "sils_old_20200101T000000.sflog.zip"
    stale_zip.write_bytes(b"stale bundle from a previous run")

    flightlog_dir = bundle_dir / "flightlog"
    log_analyzer_conftest.build_synthetic_log().save(flightlog_dir)

    new_zip_path = bundle_dir / "sils_new_20260101T000000.sflog.zip"
    result = _finalize_flightlog(flightlog_dir, new_zip_path, notes="x")

    assert result == new_zip_path
    assert new_zip_path.exists()
    assert not stale_zip.exists()
    assert sorted(bundle_dir.glob("*.sflog.zip")) == [new_zip_path]


# =============================================================================
# _bundle_metric()
# =============================================================================

# 400 Hz virtual clock, 200 samples (0.5 s): the first half holds a constant
# true tilt of TILT_PHASE1_RAD, the second half TILT_PHASE2_RAD -- so a
# `metric ... in <t0> <t1>` window over only the first half must read back
# TILT_PHASE1_RAD, proving the window actually filters by timestamp rather
# than silently seeing the whole bundle.
# 400Hz仮想クロック、200サンプル（0.5秒）: 前半は真の傾き
# TILT_PHASE1_RAD、後半は TILT_PHASE2_RAD で一定 -- 前半だけを指す
# `metric ... in <t0> <t1>` 窓は TILT_PHASE1_RAD を返すはずで、窓が
# タイムスタンプで実際に絞り込んでいる（一式全体を見ているのではない）ことの
# 証明になる。
_DT_US = 2500
_N_SAMPLES = 200
_HALF = _N_SAMPLES // 2
_TILT_PHASE1_RAD = 0.1
_TILT_PHASE2_RAD = 0.5
_EST_OFFSET_RAD = 0.02   # attitude estimate's constant roll bias vs truth
_ALT_M = 0.5
_DUTY_HEAVY = 0.6
_DUTY_OTHERS = 0.5


def _roll_quat(roll_rad):
    """Pure-roll body->NED quaternion (pitch=yaw=0): qw=cos(r/2), qx=sin(r/2).
    純ロール回転のクォータニオン（pitch=yaw=0）。"""
    return np.cos(roll_rad / 2.0), np.sin(roll_rad / 2.0), np.zeros_like(roll_rad), np.zeros_like(roll_rad)


def _build_metric_bundle(tmp_path, name="bundle", include_motor=True):
    """A minimal synthetic StampFly flight-log v1 bundle exercising every
    `_bundle_metric()` metric: truth (two-phase roll, per the module-level
    comment above), attitude (truth + a known constant offset), posvel
    (matches truth exactly -> alt_rmse == 0), and motor (one heavy duty
    channel so duty_max is unambiguous).
    `_bundle_metric()` の全メトリクスを検証できる最小限の合成一式:
    truth（上のコメント通り二段階ロール）、attitude（truth+既知の一定
    オフセット）、posvel（truth と完全一致=alt_rmse==0）、motor（duty_max が
    一意に決まるよう1チャンネルだけ重い duty）。
    """
    ts = np.arange(_N_SAMPLES, dtype=np.int64) * _DT_US
    roll_true = np.where(np.arange(_N_SAMPLES) < _HALF, _TILT_PHASE1_RAD, _TILT_PHASE2_RAD)
    qw, qx, qy, qz = _roll_quat(roll_true)

    truth = pd.DataFrame({
        "timestamp_us": ts,
        "pos_x": np.zeros(_N_SAMPLES), "pos_y": np.zeros(_N_SAMPLES),
        "pos_z": np.full(_N_SAMPLES, -_ALT_M),
        "quat_w": qw, "quat_x": qx, "quat_y": qy, "quat_z": qz,
        "vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0,
        "rate_x": 0.0, "rate_y": 0.0, "rate_z": 0.0,
    })

    roll_est = roll_true + _EST_OFFSET_RAD
    qw_e, qx_e, qy_e, qz_e = _roll_quat(roll_est)
    attitude = pd.DataFrame({
        "timestamp_us": ts, "seq": np.arange(_N_SAMPLES),
        "quat_w": qw_e, "quat_x": qx_e, "quat_y": qy_e, "quat_z": qz_e,
        "gyro_bias_x": 0.0, "gyro_bias_y": 0.0, "gyro_bias_z": 0.0,
        "accel_bias_x": 0.0, "accel_bias_y": 0.0, "accel_bias_z": 0.0,
    })

    posvel = pd.DataFrame({
        "timestamp_us": ts, "seq": np.arange(_N_SAMPLES),
        "pos_x": 0.0, "pos_y": 0.0, "pos_z": np.full(_N_SAMPLES, -_ALT_M),
        "vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0,
    })

    streams = {"truth": truth, "attitude": attitude, "posvel": posvel}
    if include_motor:
        streams["motor"] = pd.DataFrame({
            "timestamp_us": ts, "seq": np.arange(_N_SAMPLES),
            "duty_FR": _DUTY_HEAVY, "duty_RR": _DUTY_OTHERS,
            "duty_RL": _DUTY_OTHERS, "duty_FL": _DUTY_OTHERS,
        })

    log = sflog.FlightLog(streams=streams)
    bundle_path = tmp_path / name
    log.save(bundle_path)
    return bundle_path


def test_bundle_metric_tilt_max_full_window(tmp_path):
    bundle_path = _build_metric_bundle(tmp_path)
    m = _bundle_metric(bundle_path, "tilt_max")
    # abs=1e-6, not 1e-9: the bundle round-trips through CSV with `%.7g`
    # floats (bundle.py's csv_rules), so a quaternion component derived from
    # cos()/sin() loses precision below ~1e-7 relative on the way through disk.
    # abs=1e-6（1e-9 でない）: 一式は CSV の `%.7g` 浮動小数点（bundle.py の
    # csv_rules）を経由するため、cos()/sin() 由来のクォータニオン成分はディスク
    # 往復で相対 1e-7 程度未満の精度を失う。
    assert m == pytest.approx(_TILT_PHASE2_RAD, abs=1e-6)


def test_bundle_metric_window_filters_by_timestamp(tmp_path):
    """A window covering only the first half of the run must see ONLY
    TILT_PHASE1_RAD -- proof the [t0, t1] window genuinely filters rows by
    timestamp_us rather than reading the whole bundle regardless.
    前半だけを覆う窓は TILT_PHASE1_RAD だけを見るはず -- [t0, t1] 窓が
    timestamp_us で実際に行を絞り込んでいる（一式全体を無視して読んで
    いるわけではない）ことの証明。
    """
    bundle_path = _build_metric_bundle(tmp_path)
    t0 = 0.0
    t1 = (_HALF - 1) * _DT_US / 1e6   # last first-half sample, seconds
    m = _bundle_metric(bundle_path, "tilt_max", t0, t1)
    assert m == pytest.approx(_TILT_PHASE1_RAD, abs=1e-6)

    # Sanity: the unrestricted (whole-bundle) value differs from the windowed
    # one -- otherwise this test could pass by accident if windowing were a
    # silent no-op.
    # 確認: 無制限（一式全体）の値は窓ありと異なる -- そうでなければ窓処理が
    # 黙って no-op でもこのテストが偶然通ってしまう。
    m_full = _bundle_metric(bundle_path, "tilt_max")
    assert m_full != pytest.approx(m, abs=1e-9)


def test_bundle_metric_empty_window_returns_none(tmp_path):
    bundle_path = _build_metric_bundle(tmp_path)
    assert _bundle_metric(bundle_path, "tilt_max", 100.0, 200.0) is None


def test_bundle_metric_att_rmse(tmp_path):
    bundle_path = _build_metric_bundle(tmp_path)
    m = _bundle_metric(bundle_path, "att_rmse")
    assert m == pytest.approx(_EST_OFFSET_RAD, abs=1e-6)


def test_bundle_metric_roll_pitch_rmse(tmp_path):
    bundle_path = _build_metric_bundle(tmp_path)
    assert _bundle_metric(bundle_path, "roll_rmse") == pytest.approx(_EST_OFFSET_RAD, abs=1e-6)
    assert _bundle_metric(bundle_path, "pitch_rmse") == pytest.approx(0.0, abs=1e-6)


def test_bundle_metric_alt_rmse_zero(tmp_path):
    """posvel.pos_z matches truth.pos_z exactly -> zero estimation error."""
    bundle_path = _build_metric_bundle(tmp_path)
    assert _bundle_metric(bundle_path, "alt_rmse") == pytest.approx(0.0, abs=1e-9)


def test_bundle_metric_alt_stats(tmp_path):
    bundle_path = _build_metric_bundle(tmp_path)
    assert _bundle_metric(bundle_path, "alt_max") == pytest.approx(_ALT_M, abs=1e-9)
    assert _bundle_metric(bundle_path, "alt_min") == pytest.approx(_ALT_M, abs=1e-9)
    assert _bundle_metric(bundle_path, "alt_mean") == pytest.approx(_ALT_M, abs=1e-9)
    assert _bundle_metric(bundle_path, "alt_band") == pytest.approx(0.0, abs=1e-9)


def test_bundle_metric_duty_max(tmp_path):
    bundle_path = _build_metric_bundle(tmp_path)
    assert _bundle_metric(bundle_path, "duty_max") == pytest.approx(_DUTY_HEAVY, abs=1e-9)


def test_bundle_metric_duty_max_zero_when_motor_stream_has_no_rows_in_window(tmp_path):
    """motor.csv holds one row per control cycle that actually ran (the armed
    window) -- a window with truth rows but zero motor rows (e.g. before the
    vehicle was ever armed, as in pairing.expect's crosstalk-rejection
    assertion) means "no control cycle ran here", i.e. duty_max == 0.0, NOT
    "unknown" (None). Regression for a real `sf sils regression` failure
    found on pairing.expect's `metric duty_max < 0.05 in 5.8 7.8` window.
    motor.csv は実際に走った制御周期だけ1行を持つ（armed window）-- truth の
    行はあるがモータ行が0件の窓（pairing.expect の混信拒否窓のように、機体が
    一度も arm されていない場合等）は「制御周期が一度も走らなかった」
    =duty_max==0.0 を意味し、"unknown"（None）ではない。`sf sils regression`
    の実失敗（pairing.expect の `metric duty_max < 0.05 in 5.8 7.8`）の
    再発防止テスト。
    """
    n = 20
    dt_us = 2500
    ts = np.arange(n, dtype=np.int64) * dt_us
    truth = pd.DataFrame({
        "timestamp_us": ts,
        "pos_x": 0.0, "pos_y": 0.0, "pos_z": -0.5,
        "quat_w": 1.0, "quat_x": 0.0, "quat_y": 0.0, "quat_z": 0.0,
        "vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0,
        "rate_x": 0.0, "rate_y": 0.0, "rate_z": 0.0,
    })
    # motor.csv has rows only from the SECOND half onward -- mirrors
    # emu_flightlog_vehicle.cpp only recording a control cycle once armed.
    # motor.csv は後半からのみ行がある -- emu_flightlog_vehicle.cpp が
    # arm 後の制御周期しか記録しないことを模している。
    armed_from = n // 2
    motor_ts = ts[armed_from:]
    motor = pd.DataFrame({
        "timestamp_us": motor_ts, "seq": np.arange(len(motor_ts)),
        "duty_FR": 0.5, "duty_RR": 0.5, "duty_RL": 0.5, "duty_FL": 0.5,
    })
    log = sflog.FlightLog(streams={"truth": truth, "motor": motor})
    bundle_path = tmp_path / "pre_arm_gap"
    log.save(bundle_path)

    # A window entirely BEFORE the armed window: truth has rows here, motor
    # does not.
    # 窓は armed 前の期間全体: truth には行があるがモータには無い。
    t0 = 0.0
    t1 = (armed_from - 1) * dt_us / 1e6
    m = _bundle_metric(bundle_path, "duty_max", t0, t1)
    assert m == pytest.approx(0.0, abs=1e-9)


def test_bundle_metric_duty_max_none_without_motor_stream(tmp_path):
    bundle_path = _build_metric_bundle(tmp_path, name="no_motor", include_motor=False)
    assert _bundle_metric(bundle_path, "duty_max") is None


def test_bundle_metric_yaw_band_zero(tmp_path):
    """Pure roll motion (yaw component of the quaternion always zero) ->
    yaw_band ~ 0.
    純ロール運動（クォータニオンのヨー成分が常にゼロ）-> yaw_band ~ 0。
    """
    bundle_path = _build_metric_bundle(tmp_path)
    assert _bundle_metric(bundle_path, "yaw_band") == pytest.approx(0.0, abs=1e-9)


def test_bundle_metric_horizontal_drift_max_zero(tmp_path):
    bundle_path = _build_metric_bundle(tmp_path)
    assert _bundle_metric(bundle_path, "horizontal_drift_max") == pytest.approx(0.0, abs=1e-9)


def test_bundle_metric_unknown_name_returns_none(tmp_path):
    bundle_path = _build_metric_bundle(tmp_path)
    assert _bundle_metric(bundle_path, "not_a_real_metric") is None


def test_bundle_metric_none_path_returns_none():
    assert _bundle_metric(None, "tilt_max") is None


def test_bundle_metric_missing_truth_stream_returns_none(tmp_path):
    log = sflog.FlightLog(streams={"imu": pd.DataFrame({
        "timestamp_us": [0, 2500], "seq": [0, 1],
        "gyro_x": [0.0, 0.0], "gyro_y": [0.0, 0.0], "gyro_z": [0.0, 0.0],
        "accel_x": [0.0, 0.0], "accel_y": [0.0, 0.0], "accel_z": [-9.81, -9.81],
        "gyro_raw_x": [0.0, 0.0], "gyro_raw_y": [0.0, 0.0], "gyro_raw_z": [0.0, 0.0],
        "accel_raw_x": [0.0, 0.0], "accel_raw_y": [0.0, 0.0], "accel_raw_z": [-9.81, -9.81],
    })})
    bundle_path = tmp_path / "no_truth"
    log.save(bundle_path)
    assert _bundle_metric(bundle_path, "tilt_max") is None
