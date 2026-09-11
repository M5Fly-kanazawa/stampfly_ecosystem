"""
test_check.py - check.check_bundle(): clean bundle passes, and each of the
required error/warning classes is actually detected.
test_check.py - check.check_bundle(): 正常な一式は無指摘、かつ各種
error/warning が実際に検出されること。
"""

from __future__ import annotations

import pandas as pd

from sflog.bundle import FlightLog, make_meta
from sflog.check import check_bundle, is_ok


def _clean_imu(n=10, dt_us=2500, seq_start=0):
    """n rows at exactly the nominal 400 Hz (dt=2500us) rate, with a
    strictly increasing `seq` (protocol/spec/flight_log.yaml `seq` column).
    公称通り400Hz（dt=2500us）でn行。`seq`（protocol/spec/flight_log.yaml
    の `seq` 列）は厳密に増加する。
    """
    ts = [i * dt_us for i in range(n)]
    seq = [seq_start + i for i in range(n)]
    return pd.DataFrame(
        {
            "timestamp_us": ts,
            "seq": seq,
            "gyro_x": [0.01] * n,
            "gyro_y": [0.0] * n,
            "gyro_z": [-0.01] * n,
            "accel_x": [0.0] * n,
            "accel_y": [0.0] * n,
            "accel_z": [-9.81] * n,
            "gyro_raw_x": [0.01] * n,
            "gyro_raw_y": [0.0] * n,
            "gyro_raw_z": [-0.01] * n,
            "accel_raw_x": [0.0] * n,
            "accel_raw_y": [0.0] * n,
            "accel_raw_z": [-9.81] * n,
        }
    )


def _clean_motor(n=10, dt_us=2500, seq_start=0):
    """n rows of the "motor" lockstep stream (protocol/spec/flight_log.yaml
    `motor` stream), with `seq` starting at `seq_start` -- used to build a
    stream whose `seq` set is NOT a subset of imu's.
    "motor" ロックステップ系ストリーム（protocol/spec/flight_log.yaml の
    `motor` ストリーム）の n 行。`seq` は `seq_start` から始まる --
    imu の `seq` 集合の部分集合にならないストリームを作るのに使う。
    """
    ts = [i * dt_us for i in range(n)]
    seq = [seq_start + i for i in range(n)]
    return pd.DataFrame(
        {
            "timestamp_us": ts,
            "seq": seq,
            "duty_FR": [0.5] * n,
            "duty_RR": [0.5] * n,
            "duty_RL": [0.5] * n,
            "duty_FL": [0.5] * n,
        }
    )


def _clean_log(**extra_streams) -> FlightLog:
    streams = {"imu": _clean_imu(), **extra_streams}
    meta = make_meta(source="vehicle", tool_name="t", tool_version="0", streams=streams)
    from sflog import schema

    return FlightLog(meta=meta, schema=schema.schema_for(streams.keys()), streams=streams)


def test_clean_bundle_has_no_errors():
    findings = check_bundle(_clean_log())
    assert is_ok(findings), [str(f) for f in findings]


def test_non_monotonic_timestamp_is_an_error():
    imu = _clean_imu()
    # Force row 3's timestamp backward, violating monotonicity (seq is left
    # untouched -- still strictly increasing -- so only the timestamp_us
    # check should fire).
    # 行3の時刻を逆行させ、単調性の制約に違反させる（seq は触らないので
    # 厳密増加のまま -- timestamp_us の検査だけが反応するはず）。
    imu.loc[3, "timestamp_us"] = imu.loc[2, "timestamp_us"] - 1
    log = _clean_log()
    log.streams["imu"] = imu

    findings = check_bundle(log)
    errors = [f for f in findings if f.level == "error" and "timestamp_us" in f.message]
    assert errors, [str(f) for f in findings]
    assert not is_ok(findings)


def test_repeated_timestamp_is_a_warning_not_an_error():
    """A control cycle that reused a stale IMU sample repeats the previous
    row's timestamp_us -- plan section 2.2 "重複と欠落の扱い" requires this
    to be a warning (with the repeat count), never an error.
    制御周期が古い IMU 標本を再利用すると前行の timestamp_us を繰り返す --
    計画書 2.2節「重複と欠落の扱い」により、これは warning（重複件数付き）
    であって error であってはならない。
    """
    imu = _clean_imu()
    # Row 3 reuses row 2's timestamp (a legitimate repeat, NOT a decrease).
    # 行3が行2の時刻を再利用する（正当な重複であって逆行ではない）。
    imu.loc[3, "timestamp_us"] = imu.loc[2, "timestamp_us"]
    log = _clean_log()
    log.streams["imu"] = imu

    findings = check_bundle(log)
    assert is_ok(findings), [str(f) for f in findings]  # must NOT be an error
    warnings_found = [f for f in findings if f.level == "warning" and "repeated timestamp" in f.message]
    assert warnings_found, [str(f) for f in findings]
    assert "1 repeated" in warnings_found[0].message


def test_seq_gap_is_a_warning():
    """A gap in `seq` (packet/cycle loss) is expected and must be a
    warning naming the number of missing values, never an error.
    `seq` の飛び（パケット/周期の欠落）は正当に起こり得るため、欠落した
    件数を名指しした warning であって error であってはならない。
    """
    imu = _clean_imu()
    # seq jumps from 4 to 7 (2 cycles lost) at row 5 onward.
    # 行5以降、seq が 4 から 7 へ飛ぶ（2周期分の欠落）。
    imu.loc[5:, "seq"] = imu.loc[5:, "seq"] + 2
    log = _clean_log()
    log.streams["imu"] = imu

    findings = check_bundle(log)
    assert is_ok(findings), [str(f) for f in findings]
    warnings_found = [f for f in findings if f.level == "warning" and "seq" in f.message and "gap" in f.message]
    assert warnings_found, [str(f) for f in findings]
    assert "2 missing" in warnings_found[0].message


def test_seq_non_increasing_is_an_error():
    """Unlike timestamp_us, `seq` uniquely identifies a control cycle --
    a repeat or decrease is always a structural error.
    timestamp_us と異なり `seq` は制御周期を一意に識別する -- 重複や逆行は
    常に構造的な誤り。
    """
    imu = _clean_imu()
    imu.loc[3, "seq"] = imu.loc[2, "seq"]  # duplicate seq, not a legitimate repeat
    log = _clean_log()
    log.streams["imu"] = imu

    findings = check_bundle(log)
    assert not is_ok(findings)
    errors = [f for f in findings if f.level == "error" and "seq" in f.message]
    assert errors, [str(f) for f in findings]


def test_lockstep_seq_not_subset_of_imu_is_an_error():
    """A lockstep stream (schema.LOCKSTEP_STREAMS) whose `seq` values
    include some not present in imu's `seq` set is structurally
    inconsistent -- imu is the reference clock every control-cycle row is
    defined against (plan section 2.2).
    ロックステップ系ストリーム（schema.LOCKSTEP_STREAMS）の `seq` 値に
    imu の `seq` 集合に無い値が混じっているのは構造的な不整合 -- imu は
    全ての制御周期行の定義の基準になる時計だから（計画書 2.2節）。
    """
    imu = _clean_imu(n=10)  # seq 0..9
    motor = _clean_motor(n=10, seq_start=5)  # seq 5..14 -- 10..14 not in imu
    log = _clean_log(motor=motor)

    findings = check_bundle(log)
    assert not is_ok(findings)
    errors = [f for f in findings if f.level == "error" and f.stream == "motor" and "seq" in f.message]
    assert errors, [str(f) for f in findings]
    assert "5 seq values" in errors[0].message


def test_missing_required_stream_is_an_error():
    # Build a log with no "imu" stream at all (imu is REQUIRED per schema).
    # "imu" を全く持たないログを作る（schema 上 imu は必須）。
    mag = pd.DataFrame({"timestamp_us": [0, 1000], "x": [1.0, 2.0], "y": [0.0, 0.0], "z": [0.0, 0.0]})
    meta = make_meta(source="vehicle", tool_name="t", tool_version="0", streams={"mag": mag})
    from sflog import schema

    log = FlightLog(meta=meta, schema=schema.schema_for(["mag"]), streams={"mag": mag})

    findings = check_bundle(log)
    assert not is_ok(findings)
    assert any(f.level == "error" and f.stream == "imu" for f in findings)


def test_extra_column_is_a_warning_not_an_error():
    imu = _clean_imu()
    imu["totally_unexpected_column"] = 1.0
    log = _clean_log()
    log.streams["imu"] = imu

    findings = check_bundle(log)
    assert is_ok(findings)  # extra column must NOT be an error
    warnings = [f for f in findings if f.level == "warning" and "totally_unexpected_column" in f.message]
    assert warnings, [str(f) for f in findings]


def test_missing_expected_column_is_an_error():
    imu = _clean_imu().drop(columns=["accel_z"])
    log = _clean_log()
    log.streams["imu"] = imu

    findings = check_bundle(log)
    assert not is_ok(findings)
    assert any(f.level == "error" and "accel_z" in f.message for f in findings)


def test_rate_deviation_beyond_tolerance_is_a_warning():
    # 10 rows spaced at 10ms (100 Hz) instead of the nominal 400 Hz -- a
    # 75% deviation, well beyond the +-30% tolerance.
    # 公称400Hzに対し10ms間隔(100Hz)の10行 -- 許容+-30%を大きく超える75%乖離。
    imu = _clean_imu(dt_us=10_000)
    log = _clean_log()
    log.streams["imu"] = imu

    findings = check_bundle(log)
    assert any(f.level == "warning" and "rate" in f.message for f in findings)


def test_unit_sanity_gyro_out_of_range_is_a_warning():
    imu = _clean_imu()
    imu.loc[0, "gyro_x"] = 999.0  # way beyond +-50 rad/s
    log = _clean_log()
    log.streams["imu"] = imu

    findings = check_bundle(log)
    assert is_ok(findings)  # sanity range violations are warnings, not errors
    assert any(f.level == "warning" and "gyro_x" in f.message for f in findings)


def test_missing_meta_or_schema_is_an_error():
    streams = {"imu": _clean_imu()}
    log = FlightLog(meta={}, schema={}, streams=streams)

    findings = check_bundle(log)
    assert not is_ok(findings)
    assert any(f.stream == "meta.json" for f in findings)
    assert any(f.stream == "schema.json" for f in findings)
