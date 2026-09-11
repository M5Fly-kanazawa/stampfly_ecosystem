"""
test_align.py - align.aligned(): seq-based joins for lockstep streams,
hold/nearest merge_asof semantics for non-lockstep streams, deterministic
collision renaming.
test_align.py - align.aligned(): ロックステップ系ストリームの seq 結合、
非ロックステップ系ストリームの保持/最近傍 merge_asof 意味論、決定論的な
衝突時改名。
"""

from __future__ import annotations

import math

import pandas as pd
import pytest

from sflog.align import aligned
from sflog.bundle import FlightLog


def _log_with(streams: dict) -> FlightLog:
    return FlightLog(meta={}, schema={}, streams=streams)


# =============================================================================
# Non-lockstep streams: merge_asof hold/nearest
# 非ロックステップ系ストリーム: merge_asof の保持/最近傍
# =============================================================================


def test_hold_carries_earlier_observation_and_its_own_timestamp():
    """A sparser NON-lockstep stream's value (baro has no `seq` column --
    schema.LOCKSTEP_STREAMS) is held forward onto each base row, and the
    companion "<stream>_timestamp_us" column records exactly when that held
    value was actually observed (not the base row's time).
    疎な非ロックステップ系ストリーム（baro は `seq` 列を持たない --
    schema.LOCKSTEP_STREAMS）の値は基準行へ前方保持され、随伴列
    "<stream>_timestamp_us" にはその保持値が実際に観測された時刻
    （基準行の時刻ではない）が入る。
    """
    imu = pd.DataFrame({"timestamp_us": [0, 100, 200, 300, 400], "seq": [0, 1, 2, 3, 4], "gyro_x": [0.0] * 5})
    # baro observed at t=150 (altitude 1.0) and t=350 (altitude 2.0) only.
    # baro は t=150（altitude 1.0）と t=350（altitude 2.0）でしか観測されない。
    baro = pd.DataFrame({"timestamp_us": [150, 350], "altitude": [1.0, 2.0], "pressure": [101300.0, 101250.0]})

    table = aligned(_log_with({"imu": imu, "baro": baro}), base="imu", method="hold")

    assert list(table["timestamp_us"]) == [0, 100, 200, 300, 400]
    # Before the first baro observation (t=150): held value is NaN, and
    # so is the observation-timestamp column.
    # 最初の baro 観測（t=150）より前: 保持値は NaN、観測時刻列も NaN。
    assert math.isnan(table.loc[0, "altitude"])
    assert math.isnan(table.loc[0, "baro_timestamp_us"])
    assert math.isnan(table.loc[1, "altitude"])

    # At/after t=200: holds the t=150 observation (altitude 1.0), and
    # baro_timestamp_us == 150, NOT the base row's own timestamp (200).
    # t=200以降: t=150の観測（altitude 1.0）を保持し、baro_timestamp_us は
    # 基準行自身の時刻（200）ではなく150になる。
    assert table.loc[2, "altitude"] == pytest.approx(1.0)
    assert table.loc[2, "baro_timestamp_us"] == 150

    # At/after t=400: holds the t=350 observation (altitude 2.0).
    assert table.loc[4, "altitude"] == pytest.approx(2.0)
    assert table.loc[4, "baro_timestamp_us"] == 350


def test_nearest_picks_the_closer_observation():
    imu = pd.DataFrame({"timestamp_us": [0, 100, 200], "seq": [0, 1, 2], "gyro_x": [0.0] * 3})
    # Observations at 40 and 180.
    baro = pd.DataFrame({"timestamp_us": [40, 180], "altitude": [1.0, 2.0], "pressure": [1.0, 2.0]})

    table = aligned(_log_with({"imu": imu, "baro": baro}), base="imu", method="nearest")

    assert table.loc[0, "altitude"] == pytest.approx(1.0)  # |0-40|=40 < |0-180|=180
    assert table.loc[1, "altitude"] == pytest.approx(1.0)  # |100-40|=60 < |100-180|=80
    assert table.loc[2, "altitude"] == pytest.approx(2.0)  # |200-40|=160 > |200-180|=20


def test_base_with_repeated_timestamps_still_gets_a_merge_asof_match_on_every_row():
    """A base whose `timestamp_us` repeats (a control cycle that reused a
    stale IMU sample -- observed on 13% of real control cycles, plan
    section 7) must not lose the merge_asof match on any row against a
    non-lockstep stream: each base row is asof-matched independently.
    基準の `timestamp_us` が重複していても（制御周期が古い IMU 標本を
    再利用した場合 -- 実機で13%発生、計画書7節）、非ロックステップ系
    ストリームに対する merge_asof の対応をどの行も失わない: 基準の各行は
    個別に asof 照合される。
    """
    imu = pd.DataFrame({"timestamp_us": [0, 100, 100, 200], "seq": [0, 1, 2, 3], "gyro_x": [0.0] * 4})
    baro = pd.DataFrame({"timestamp_us": [50], "altitude": [9.0], "pressure": [1.0]})

    table = aligned(_log_with({"imu": imu, "baro": baro}), base="imu", method="hold")

    assert len(table) == 4
    assert math.isnan(table.loc[0, "altitude"])  # before baro's only observation
    assert table.loc[1, "altitude"] == pytest.approx(9.0)
    assert table.loc[2, "altitude"] == pytest.approx(9.0)  # repeated timestamp_us, still matched
    assert table.loc[3, "altitude"] == pytest.approx(9.0)


# =============================================================================
# Lockstep streams: exact join on `seq`
# ロックステップ系ストリーム: `seq` の完全一致結合
# =============================================================================


def test_lockstep_join_uses_seq_not_timestamp_when_timestamps_repeat():
    """Two lockstep rows sharing the same timestamp_us (a control cycle
    that reused a stale IMU sample) must still map 1:1 by `seq` -- they are
    NOT collapsed onto each other the way a timestamp_us-keyed join would.
    同じ timestamp_us を共有する2つのロックステップ系の行（制御周期が
    古い IMU 標本を再利用した場合）でも `seq` で 1:1 に対応付けられる --
    timestamp_us をキーにした結合のように互いに潰れたりしない。
    """
    imu = pd.DataFrame({
        "timestamp_us": [1000, 1000, 2000],
        "seq": [0, 1, 2],
        "gyro_x": [0.1, 0.2, 0.3],
    })
    rate_ref = pd.DataFrame({
        "timestamp_us": [1000, 1000, 2000],
        "seq": [0, 1, 2],
        "rate_ref_roll": [10.0, 20.0, 30.0],
    })

    table = aligned(_log_with({"imu": imu, "rate_ref": rate_ref}), base="imu")

    assert table["rate_ref_roll"].tolist() == [10.0, 20.0, 30.0]
    # A lockstep merge never adds a merge_asof provenance column.
    # ロックステップ系の結合は merge_asof の随伴列を追加しない。
    assert "rate_ref_timestamp_us" not in table.columns


def test_lockstep_join_is_keyed_by_seq_not_by_the_other_streams_row_order():
    """`rate_ref`'s rows are deliberately out of seq order here -- if the
    join were accidentally positional (row i of imu <-> row i of rate_ref)
    instead of a true `seq` key match, this would misalign the values.
    ここでは `rate_ref` の行順をわざと seq 順から崩している -- もし結合が
    真の `seq` キー一致ではなく誤って位置的（imu の行i <-> rate_refの行i）
    だったら、値がズレて検出される。
    """
    imu = pd.DataFrame({"timestamp_us": [1000, 1000, 2000], "seq": [0, 1, 2], "gyro_x": [0.1, 0.2, 0.3]})
    rate_ref = pd.DataFrame({
        "timestamp_us": [2000, 1000, 1000],
        "seq": [2, 0, 1],
        "rate_ref_roll": [30.0, 10.0, 20.0],
    })

    table = aligned(_log_with({"imu": imu, "rate_ref": rate_ref}), base="imu")

    assert table["rate_ref_roll"].tolist() == [10.0, 20.0, 30.0]


def test_lockstep_stream_without_seq_falls_back_to_row_index_join_with_warning():
    """A lockstep stream missing its `seq` column (should not happen for a
    v1 bundle) still merges via a row-index fallback, but `aligned()` must
    warn so a caller can notice the degraded, position-only join.
    `seq` 列を欠くロックステップ系ストリーム（v1 バンドルでは起こらない
    はず）でも行番号ベースのフォールバックで結合はできるが、`aligned()`
    は劣化した位置のみの結合であることに気づけるよう警告しなければ
    ならない。
    """
    imu = pd.DataFrame({"timestamp_us": [0, 100, 200], "gyro_x": [0.0, 0.0, 0.0]})  # no seq
    attitude = pd.DataFrame({"timestamp_us": [0, 100, 200], "quat_w": [1.0, 1.0, 1.0]})  # no seq

    with pytest.warns(UserWarning, match="seq"):
        table = aligned(_log_with({"imu": imu, "attitude": attitude}), base="imu")

    assert table["quat_w"].tolist() == [1.0, 1.0, 1.0]


def test_seq_is_kept_in_the_output():
    imu = pd.DataFrame({"timestamp_us": [0, 100], "seq": [0, 1], "gyro_x": [0.0, 0.0]})
    table = aligned(_log_with({"imu": imu}), base="imu")
    assert table["seq"].tolist() == [0, 1]


# =============================================================================
# Collision renaming
# 衝突時改名
# =============================================================================


def test_collision_renaming_prefers_earlier_stream_bare_name():
    """Deterministic collision rule: the stream merged FIRST keeps the bare
    column name; a later stream with the same column name gets prefixed.
    Streams are merged in `schema.STREAMS` order by default, where "motor"
    (lockstep, joined on seq) precedes "ctrl_ref" (not lockstep, joined by
    merge_asof) -- so motor.duty_FR keeps `duty_FR` and ctrl_ref.duty_FR
    becomes `ctrl_ref_duty_FR`.
    決定論的な衝突規則: 先に結合されたストリームが素の列名を保持する。
    既定の結合順は schema.STREAMS の記載順で、"motor"（ロックステップ系、
    seq 結合）は "ctrl_ref"（非ロックステップ系、merge_asof 結合）より
    先 -- よって motor.duty_FR は `duty_FR` のまま、ctrl_ref.duty_FR は
    `ctrl_ref_duty_FR` になる。
    """
    imu = pd.DataFrame({"timestamp_us": [0, 100], "seq": [0, 1], "gyro_x": [0.0, 0.0]})
    motor = pd.DataFrame({"timestamp_us": [0, 100], "seq": [0, 1], "duty_FR": [0.1, 0.2]})
    ctrl_ref = pd.DataFrame({"timestamp_us": [0, 100], "duty_FR": [0.5, 0.6]})

    table = aligned(
        _log_with({"imu": imu, "motor": motor, "ctrl_ref": ctrl_ref}),
        base="imu",
        # explicit order mirrors schema.STREAMS: motor before ctrl_ref
        streams=["motor", "ctrl_ref"],
    )

    assert table["duty_FR"].tolist() == [0.1, 0.2]
    assert table["ctrl_ref_duty_FR"].tolist() == [0.5, 0.6]


def test_default_stream_order_matches_schema_streams_order():
    """With `streams=None` (the default), motor is merged before ctrl_ref
    because that is schema.STREAMS' insertion order -- so the SAME
    collision result as the explicit-order test above must hold.
    `streams=None`（既定）でも、schema.STREAMS の記載順で motor が
    ctrl_ref より先に結合される -- 上の明示順テストと同じ衝突結果になる
    はず。
    """
    imu = pd.DataFrame({"timestamp_us": [0, 100], "seq": [0, 1], "gyro_x": [0.0, 0.0]})
    motor = pd.DataFrame({"timestamp_us": [0, 100], "seq": [0, 1], "duty_FR": [0.1, 0.2]})
    ctrl_ref = pd.DataFrame({"timestamp_us": [0, 100], "duty_FR": [0.5, 0.6]})

    table = aligned(_log_with({"imu": imu, "motor": motor, "ctrl_ref": ctrl_ref}), base="imu")

    assert table["duty_FR"].tolist() == [0.1, 0.2]
    assert table["ctrl_ref_duty_FR"].tolist() == [0.5, 0.6]


# =============================================================================
# Error handling
# エラー処理
# =============================================================================


def test_missing_base_raises():
    imu_only_log = _log_with({"mag": pd.DataFrame({"timestamp_us": [0], "x": [1.0]})})
    with pytest.raises(ValueError):
        aligned(imu_only_log, base="imu")


def test_unknown_method_raises():
    log = _log_with({"imu": pd.DataFrame({"timestamp_us": [0], "seq": [0], "gyro_x": [0.0]})})
    with pytest.raises(ValueError):
        aligned(log, base="imu", method="linear")
