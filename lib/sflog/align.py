"""
align.py - Build a single aligned table from a FlightLog's multi-rate
streams (a read-time, in-memory operation only -- see
docs/plans/flight-log-format-plan.md section 2.4/2.5).
align.py - FlightLog の多レートストリームから1枚の整列表を作る（読み込み時
のメモリ上操作のみ -- 計画書 2.4/2.5節参照）。

The primary CSV record never holds a forward-filled/interpolated value --
alignment always happens here, on demand, and is explicitly labelled
"derived" wherever it is written to disk (see convert.aligned_to_csv).
一次記録の CSV は前方補完・補間した値を持たない -- 整列は常にここで
必要なときだけ行い、ディスクに書くときは必ず「派生物」と明記する
（convert.aligned_to_csv 参照）。
"""

from __future__ import annotations

import warnings

import pandas as pd

from . import schema

_VALID_METHODS = ("hold", "nearest")

# Internal-only column used to restore `result`'s row order after a `pandas
# .merge()` on "seq" -- merge's row-order guarantee for a left join is an
# implementation detail we do not want to depend on, so we pin the order
# explicitly instead (see _merge_lockstep()).
# `pandas.merge()` を "seq" で行った後に `result` の行順を復元するための
# 内部専用列 -- 左結合の行順保持は実装依存の性質であり依存したくないため、
# 明示的に固定する（_merge_lockstep() 参照）。
_ORDER_COL = "__sflog_align_order__"


def aligned(log, base: str = "imu", streams=None, method: str = "hold") -> pd.DataFrame:
    """Build one table indexed by `base`'s `timestamp_us`, with every other
    requested stream's columns merged onto it.

    Column-naming rule (deterministic -- documented here because it is easy
    to get subtly wrong): streams are merged in the given order (default:
    `schema.STREAMS` insertion order, excluding `base`). For each merged
    stream, a column keeps its bare name UNLESS that name already exists in
    the result so far, in which case it is renamed "<stream>_<column>". For
    example, motor.csv is merged before ctrl_ref.csv in schema.py's stream
    order, so `motor.duty_FR` keeps the bare name `duty_FR` and
    `ctrl_ref.duty_FR` (same physical quantity, 50 Hz instead of motor's
    400 Hz) becomes `ctrl_ref_duty_FR`.

    Merge method, per non-base stream:
      * A LOCKSTEP stream (`schema.LOCKSTEP_STREAMS`: imu, attitude,
        posvel, rate_ref, motor, ctrl_output -- every stream whose schema
        declares a `seq` column) is joined by an EXACT match on `seq`,
        never on `timestamp_us`: two lockstep streams can legitimately
        share a repeated `timestamp_us` (a control cycle that reused a
        stale IMU sample) while still being distinct control cycles with
        their own freshly computed values (protocol/spec/flight_log.yaml
        `seq` column; plan section 2.2) -- only `seq` tells those cycles
        apart. If either side lacks `seq` (a pre-v1 bundle; should not
        happen for a v1 bundle), this falls back to a row-index join and
        emits a `warnings.warn`.
      * Any other stream (no `seq` column) is merged with
        `pandas.merge_asof` on `timestamp_us`, `direction="backward"`
        (method="hold": the most recently observed value at or before the
        base time) or `direction="nearest"` (method="nearest"). This holds
        even when the base stream's own `timestamp_us` repeats -- each
        base row is asof-matched independently, so every base row gets a
        match. A companion column "<stream>_timestamp_us" records the
        ACTUAL timestamp of the value that got carried in, so a
        held/matched value is always distinguishable from a real
        same-instant observation; it is NaN before that stream's first
        observation.

    `base`, `streams`から1枚の表を作る。`base` の `timestamp_us` を基準に、
    要求した他ストリームの列をそこへ結合する。

    列名の規則（決定論的。誤りやすいためここに明記する）: ストリームは
    指定順（既定は `base` を除く `schema.STREAMS` の記載順）で結合する。
    結合する列は、結果側に既に同名の列が無ければそのままの名前を使い、
    あれば "<stream>_<列名>" に改名する。例えば motor.csv は schema.py の
    並びで ctrl_ref.csv より先に来るため、`motor.duty_FR` は `duty_FR` の
    ままになり、`ctrl_ref.duty_FR`（同じ物理量、motor の400Hzに対し
    ctrl_ref は50Hz）は `ctrl_ref_duty_FR` に改名される。

    非 base ストリームごとの結合方式:
      * ロックステップ系ストリーム（`schema.LOCKSTEP_STREAMS`: imu,
        attitude, posvel, rate_ref, motor, ctrl_output -- スキーマが
        `seq` 列を持つ全ストリーム）は `seq` の完全一致で結合する。
        `timestamp_us` では絶対に結合しない: 2つのロックステップ系
        ストリームは、同じ `timestamp_us` を正当に共有しながら
        （制御周期が古い IMU 標本を再利用した場合）、それぞれ独立に
        新しく計算された値を持つ別個の制御周期であり得る
        （protocol/spec/flight_log.yaml の `seq` 列、計画書 2.2節）--
        そうした周期を区別できるのは `seq` だけ。どちらかに `seq` が
        無ければ（v1 以前のバンドル。v1 バンドルでは起こらないはず）
        行番号での結合にフォールバックし `warnings.warn` を発する。
      * それ以外（`seq` 列を持たない）のストリームは `timestamp_us` で
        `pandas.merge_asof` を使う -- `direction="backward"`
        （method="hold": その時刻以前で直近に観測された値）または
        `direction="nearest"`（method="nearest"）。基準ストリーム自身の
        `timestamp_us` が重複していても成立する -- 基準の各行は個別に
        asof 照合されるため、全ての基準行が対応を得る。随伴列
        "<stream>_timestamp_us" にその値が実際に観測された時刻を記録する
        ので、保持値と真の同時刻観測を常に区別できる（そのストリームの
        最初の観測より前は NaN）。

    Args:
        log: a FlightLog (see bundle.py).
        base: name of the stream whose timestamp_us becomes the result's
            index/first column. Must be present in `log.streams`.
        streams: iterable of stream names to merge in, in order. Defaults
            to every other stream present in `log.streams`, in
            `schema.STREAMS` order.
        method: "hold" (merge_asof backward) or "nearest" (merge_asof
            nearest).

    Returns:
        pandas.DataFrame with `base`'s timestamp_us as the first column.

    Raises:
        ValueError: `base` is not present in `log.streams`, or `method` is
            not one of "hold"/"nearest".
    """
    if method not in _VALID_METHODS:
        raise ValueError(f"unknown method {method!r}; expected one of {_VALID_METHODS}")
    if base not in log.streams:
        raise ValueError(f"base stream '{base}' is not present in this bundle")

    result = log.streams[base].sort_values("timestamp_us", kind="stable").reset_index(drop=True).copy()

    if streams is None:
        streams = [name for name in schema.STREAMS if name != base and name in log.streams]

    direction = "backward" if method == "hold" else "nearest"

    for name in streams:
        if name == base or name not in log.streams:
            continue
        other = log.streams[name]

        if name in schema.LOCKSTEP_STREAMS:
            result = _merge_lockstep(result, other, name)
        else:
            result = _merge_asof_stream(result, other, name, direction)

    return result


def _merge_lockstep(result: pd.DataFrame, other: pd.DataFrame, name: str) -> pd.DataFrame:
    """Merge a lockstep stream (`other`) onto `result` by an exact match on
    `seq` -- see `aligned()`'s docstring for why this must be `seq`, never
    `timestamp_us`.
    ロックステップ系ストリーム（`other`）を `seq` の完全一致で `result` へ
    結合する -- なぜ `timestamp_us` ではなく `seq` でなければならないかは
    `aligned()` のドキュメント参照。
    """
    if "seq" not in result.columns or "seq" not in other.columns:
        warnings.warn(
            f"aligned(): lockstep stream '{name}' (or the base stream) has no "
            "'seq' column -- falling back to a row-index join, which assumes "
            "row i of both streams is the same control cycle (only expected "
            "for a pre-v1 bundle)",
            stacklevel=3,
        )
        return _merge_by_row_index(result, other, name)

    # Defensive de-duplication: `seq` is defined to be unique per lockstep
    # stream (one row per control cycle), but guard against a malformed
    # bundle fanning this merge out into duplicate rows.
    # 防御的な重複除去: `seq` は各ロックステップ系ストリームで一意
    # （制御周期ごとに1行）と定義されているが、不正な一式がこの結合を
    # 重複行へ膨らませないよう保険を掛ける。
    other_dedup = other.drop_duplicates(subset="seq", keep="first")
    cols = [c for c in other_dedup.columns if c not in ("timestamp_us", "seq")]
    rename_map = _collision_rename(cols, name, result.columns)
    other_small = other_dedup[["seq"] + cols].rename(columns=rename_map)

    result = result.copy()
    result[_ORDER_COL] = range(len(result))
    merged = result.merge(other_small, on="seq", how="left")
    merged = merged.sort_values(_ORDER_COL).drop(columns=[_ORDER_COL]).reset_index(drop=True)
    return merged


def _merge_by_row_index(result: pd.DataFrame, other: pd.DataFrame, name: str) -> pd.DataFrame:
    """Positional fallback for `_merge_lockstep()` when `seq` is unavailable
    -- lines row i of `other` up with row i of `result` (after sorting
    `other` by `timestamp_us` when present). Never raises on a length
    mismatch: extra `result` rows get NaN, extra `other` rows are dropped.
    `_merge_lockstep()` の位置ベースの代替 -- `other` の行 i を `result` の
    行 i に合わせる（`other` に `timestamp_us` があれば先にそれで並べ替え
    る）。長さが食い違っても例外にはしない: `result` 側の余剰行は NaN に、
    `other` 側の余剰行は捨てる。
    """
    other_sorted = (
        other.sort_values("timestamp_us", kind="stable").reset_index(drop=True)
        if "timestamp_us" in other.columns
        else other.reset_index(drop=True)
    )
    cols = [c for c in other_sorted.columns if c not in ("timestamp_us", "seq")]
    rename_map = _collision_rename(cols, name, result.columns)
    to_add = other_sorted[cols].rename(columns=rename_map).reindex(result.index)
    return pd.concat([result, to_add], axis=1)


def _merge_asof_stream(result: pd.DataFrame, other: pd.DataFrame, name: str, direction: str) -> pd.DataFrame:
    """Merge a non-lockstep stream (`other`) onto `result` with
    `pandas.merge_asof` on `timestamp_us`, always adding the
    "<name>_timestamp_us" provenance column -- see `aligned()`'s docstring.
    非ロックステップ系ストリーム（`other`）を `timestamp_us` での
    `pandas.merge_asof` で `result` へ結合し、常に随伴列
    "<name>_timestamp_us" を追加する -- 詳細は `aligned()` のドキュメント
    参照。
    """
    other_sorted = other.sort_values("timestamp_us", kind="stable").reset_index(drop=True)
    other_ts_col = f"{name}_timestamp_us"
    other_renamed = other_sorted.rename(columns={"timestamp_us": other_ts_col})
    rename_map = _collision_rename(
        [c for c in other_renamed.columns if c != other_ts_col], name, result.columns
    )
    other_renamed = other_renamed.rename(columns=rename_map)

    # merge_asof requires both "on" columns sorted -- `result` already is
    # (built with a stable sort_values("timestamp_us") in aligned()/here),
    # and it tolerates a repeated key on either side, matching each row
    # independently -- exactly what lets a base with repeated timestamp_us
    # still get a match on every row. The sort MUST be stable (kind=
    # "stable", not the pandas default quicksort) so a tie in
    # timestamp_us -- legitimate for a lockstep stream reusing a stale IMU
    # sample -- never silently swaps two rows out of their true `seq`
    # order.
    # merge_asof は両方の "on" 列が整列済みであることを要求する -- `result`
    # は既に整列済み（aligned()/本関数内の安定ソート sort_values(
    # "timestamp_us") 由来）で、どちら側のキーが重複していても各行独立に
    # 照合するため、基準側の timestamp_us が重複していても全行が対応を
    # 得られる。ソートは安定でなければならない（既定の quicksort ではなく
    # kind="stable"）-- そうしないと、timestamp_us の同値
    # （ロックステップ系ストリームが古い IMU 標本を再利用した場合に
    # 正当に起こる）が、2行を真の `seq` 順から黙って入れ替えてしまう。
    return pd.merge_asof(
        result,
        other_renamed,
        left_on="timestamp_us",
        right_on=other_ts_col,
        direction=direction,
    )


def _collision_rename(candidate_columns, stream_name: str, existing_columns) -> dict:
    """name -> name unless it collides with `existing_columns`, in which
    case name -> "<stream_name>_<name>". See aligned()'s docstring for the
    full rule and a worked example (motor.duty_FR vs ctrl_ref.duty_FR).
    列名を、既存列と衝突しなければそのまま、衝突すれば
    "<stream_name>_<name>" に変える。詳細と具体例（motor.duty_FR と
    ctrl_ref.duty_FR）は aligned() のドキュメント参照。
    """
    existing = set(existing_columns)
    return {col: (f"{stream_name}_{col}" if col in existing else col) for col in candidate_columns}
