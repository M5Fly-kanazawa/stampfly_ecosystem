"""
convert.py - JSONL <-> flight-log v1 bundle conversion, and bundle -> aligned
CSV (the escape hatches the plan keeps for old data and one-off scripts).
convert.py - JSONL <-> フライトログ v1 一式の変換、および 一式 -> 整列 CSV
（計画書が旧データ・使い捨てスクリプト向けに残す退避路）。

`jsonl_to_bundle` reads the legacy per-sample JSONL written by
`tools/log_analyzer/udp_capture.py`'s `save_jsonl()` (one JSON object per
line, keyed by sensor "id"). `bundle_to_jsonl` is its exact inverse, so
research scripts under `analysis/scripts/` that were never migrated to
`lib/sflog` keep working against a re-exported JSONL file.
`jsonl_to_bundle` は `tools/log_analyzer/udp_capture.py` の `save_jsonl()`
が書くレガシーな1サンプル1行 JSONL（センサ種別 "id" をキーに持つ JSON
オブジェクトが1行に1件）を読む。`bundle_to_jsonl` はその厳密な逆で、
`lib/sflog` へ未移行の `analysis/scripts/` 配下の研究用スクリプトは、
再書き出しした JSONL に対して動き続けられる。

@design docs/plans/flight-log-format-plan.md section 2.4/2.5
"""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path
from typing import Optional

import pandas as pd

from . import schema
from .align import aligned as _aligned
from .bundle import FlightLog, _dataframe_to_csv_text, make_meta

_TOOL_VERSION = "0.1.0"


# =============================================================================
# JSONL id -> v1 stream row builders
# JSONL id -> v1 ストリーム行への変換
# =============================================================================
# Each builder takes one parsed JSONL object (dict) and returns
# {stream_name: row_dict, ...} -- most ids feed exactly one stream, but the
# "imu" id's single packet feeds both imu.csv (gyro/accel) and attitude.csv
# (quat/bias), since both come from the same 0x40 wire sample (see
# data_stream_wire.hpp WireImuEskf and protocol/spec/flight_log.yaml).
# 各ビルダーはパース済み JSONL オブジェクト（dict）を1件受け取り、
# {ストリーム名: 行dict, ...} を返す -- 大半の id は1ストリームだけに
# 供給するが、"imu" id の1パケットは imu.csv（gyro/accel）と
# attitude.csv（quat/bias）の両方に供給する。どちらも同じ 0x40 電文サンプル
# 由来のため（data_stream_wire.hpp の WireImuEskf、
# protocol/spec/flight_log.yaml 参照）。


def _row_from_imu(obj: dict) -> dict:
    ts = obj["ts"]
    gyro, accel = obj["gyro"], obj["accel"]
    gyro_raw, accel_raw = obj["gyro_raw"], obj["accel_raw"]
    quat, gyro_bias, accel_bias = obj["quat"], obj["gyro_bias"], obj["accel_bias"]
    return {
        "imu": {
            "timestamp_us": ts,
            "gyro_x": gyro[0], "gyro_y": gyro[1], "gyro_z": gyro[2],
            "accel_x": accel[0], "accel_y": accel[1], "accel_z": accel[2],
            "gyro_raw_x": gyro_raw[0], "gyro_raw_y": gyro_raw[1], "gyro_raw_z": gyro_raw[2],
            "accel_raw_x": accel_raw[0], "accel_raw_y": accel_raw[1], "accel_raw_z": accel_raw[2],
        },
        "attitude": {
            "timestamp_us": ts,
            "quat_w": quat[0], "quat_x": quat[1], "quat_y": quat[2], "quat_z": quat[3],
            "gyro_bias_x": gyro_bias[0], "gyro_bias_y": gyro_bias[1], "gyro_bias_z": gyro_bias[2],
            "accel_bias_x": accel_bias[0], "accel_bias_y": accel_bias[1], "accel_bias_z": accel_bias[2],
        },
    }


def _row_from_posvel(obj: dict) -> dict:
    pos, vel = obj["pos"], obj["vel"]
    return {
        "posvel": {
            "timestamp_us": obj["ts"],
            "pos_x": pos[0], "pos_y": pos[1], "pos_z": pos[2],
            "vel_x": vel[0], "vel_y": vel[1], "vel_z": vel[2],
        }
    }


def _row_from_ctrl(obj: dict) -> dict:
    # JSONL id "ctrl" (Control packet 0x42) -> v1 stream "pilot"
    # JSONL の id "ctrl"（Control パケット 0x42）-> v1 ストリーム "pilot"
    return {
        "pilot": {
            "timestamp_us": obj["ts"],
            "throttle": obj["throttle"],
            "roll": obj["roll"],
            "pitch": obj["pitch"],
            "yaw": obj["yaw"],
        }
    }


def _row_from_flow(obj: dict) -> dict:
    return {
        "flow": {
            "timestamp_us": obj["ts"],
            "dx": obj["dx"],
            "dy": obj["dy"],
            "quality": obj["quality"],
        }
    }


def _row_from_tof_b(obj: dict) -> dict:
    return {"tof_bottom": {"timestamp_us": obj["ts"], "distance": obj["distance"], "status": obj["status"]}}


def _row_from_tof_f(obj: dict) -> dict:
    return {"tof_front": {"timestamp_us": obj["ts"], "distance": obj["distance"], "status": obj["status"]}}


# hPa -> Pa: the wire (and this legacy JSONL field) carries pressure as raw
# hPa (see data_stream_wire.hpp WireBaro: "hPa on the wire, Pa inside the
# firmware"), but v1's baro.csv unifies on this document's Pa convention
# (protocol/spec/flight_log.yaml `units`/baro.pressure).
# hPa -> Pa: 電文（およびこのレガシー JSONL のフィールド）は気圧を生の
# hPa のまま運ぶ（data_stream_wire.hpp の WireBaro 参照: "hPa on the
# wire, Pa inside the firmware"）が、v1 の baro.csv は本書の Pa 規約
# （protocol/spec/flight_log.yaml の units/baro.pressure）に統一する。
_HPA_TO_PA = 100.0


def _row_from_baro(obj: dict) -> dict:
    return {
        "baro": {
            "timestamp_us": obj["ts"],
            "altitude": obj["altitude"],
            "pressure": obj["pressure"] * _HPA_TO_PA,
        }
    }


def _row_from_mag(obj: dict) -> dict:
    return {"mag": {"timestamp_us": obj["ts"], "x": obj["x"], "y": obj["y"], "z": obj["z"]}}


def _row_from_ctrl_ref(obj: dict) -> dict:
    angle_ref, duty, pos_sp = obj["angle_ref"], obj["motor_duty"], obj["pos_sp"]
    return {
        "ctrl_ref": {
            "timestamp_us": obj["ts"],
            "flight_mode": obj["mode"],
            "angle_ref_roll": angle_ref[0],
            "angle_ref_pitch": angle_ref[1],
            "total_thrust": obj.get("total_thrust", 0.0),
            "duty_FR": duty[0], "duty_RR": duty[1], "duty_RL": duty[2], "duty_FL": duty[3],
            "alt_setpoint": obj.get("alt_sp", 0.0),
            "alt_vel_target": obj.get("alt_vel_target", 0.0),
            "climb_rate_cmd": obj.get("climb_cmd", 0.0),
            "pos_setpoint_x": pos_sp[0], "pos_setpoint_y": pos_sp[1],
        }
    }


def _row_from_p_diag(obj: dict) -> dict:
    pos, vel, att, bg, ba = obj["pos"], obj["vel"], obj["att"], obj["bg"], obj["ba"]
    return {
        "eskf_cov": {
            "timestamp_us": obj["ts"],
            "p_pos_x": pos[0], "p_pos_y": pos[1], "p_pos_z": pos[2],
            "p_vel_x": vel[0], "p_vel_y": vel[1], "p_vel_z": vel[2],
            "p_att_x": att[0], "p_att_y": att[1], "p_att_z": att[2],
            "p_bg_x": bg[0], "p_bg_y": bg[1], "p_bg_z": bg[2],
            "p_ba_x": ba[0], "p_ba_y": ba[1], "p_ba_z": ba[2],
        }
    }


def _row_from_rate_ref(obj: dict) -> dict:
    rr = obj["rate_ref"]
    return {"rate_ref": {"timestamp_us": obj["ts"], "rate_ref_roll": rr[0], "rate_ref_pitch": rr[1], "rate_ref_yaw": rr[2]}}


def _row_from_duty400(obj: dict) -> dict:
    # JSONL id "duty400" (Duty400 packet 0x4A) -> v1 stream "motor"
    # JSONL の id "duty400"（Duty400 パケット 0x4A）-> v1 ストリーム "motor"
    d = obj["duty"]
    return {"motor": {"timestamp_us": obj["ts"], "duty_FR": d[0], "duty_RR": d[1], "duty_RL": d[2], "duty_FL": d[3]}}


# Columns present in status.csv's schema (protocol/spec/flight_log.yaml)
# that the legacy save_jsonl() lambda for PKT_STATUS never wrote at all --
# not a firmware-version difference, a tooling gap. jsonl_to_bundle leaves
# them as None (empty cell) rather than inventing values; see this
# module's docstring in the report handed back to the caller.
# status.csv のスキーマ（protocol/spec/flight_log.yaml）にはあるが、
# レガシー save_jsonl() の PKT_STATUS 用ラムダが一度も書いていなかった列
# -- ファーム版の違いではなくツール側の欠落。jsonl_to_bundle は値を
# 捏造せず None（空欄）のままにする。
_STATUS_UNSUPPORTED_BY_LEGACY_JSONL = ("sensor_health", "reset_reason")

_PID_AXES = ("roll", "pitch", "yaw")
_PID_TERMS = ("kp", "ti", "td")


def _row_from_status(obj: dict) -> dict:
    row = {
        "timestamp_us": obj["ts"],
        "uptime_ms": obj["uptime_ms"],
        "voltage": obj["voltage"],
        "current_ma": obj.get("current_ma"),
        "flight_state": obj["flight_state"],
        "eskf_status": obj["eskf_status"],
    }
    for col in _STATUS_UNSUPPORTED_BY_LEGACY_JSONL:
        row[col] = None
    for axis in _PID_AXES:
        key = f"pid_{axis}"
        values = obj.get(key)
        for i, term in enumerate(_PID_TERMS):
            row[f"pid_{axis}_{term}"] = values[i] if values is not None else None
    return {"status": row}


_ID_TO_ROW_BUILDER = {
    "imu": _row_from_imu,
    "posvel": _row_from_posvel,
    "ctrl": _row_from_ctrl,
    "flow": _row_from_flow,
    "tof_b": _row_from_tof_b,
    "tof_f": _row_from_tof_f,
    "baro": _row_from_baro,
    "mag": _row_from_mag,
    "ctrl_ref": _row_from_ctrl_ref,
    "p_diag": _row_from_p_diag,
    "rate_ref": _row_from_rate_ref,
    "duty400": _row_from_duty400,
    "status": _row_from_status,
}


def _sort_rows_stable(rows: list) -> list:
    """Stable sort by timestamp_us: ties keep their original relative
    (capture) order, which is exactly what a legacy JSONL's per-id
    subsequence already has (`save_jsonl()` globally sorts by timestamp
    before writing, and a stable sort preserves same-id relative order
    when filtering back out by id).
    timestamp_us で安定ソートする: 同値は元の相対順（捕捉順）を保つ。
    レガシー JSONL の id 別部分列は既にこの性質を持つ
    （`save_jsonl()` が書き出し前に全体をタイムスタンプで安定ソートして
    おり、id で絞り込んでも同一 id 内の相対順は保たれる）。
    """
    return sorted(rows, key=lambda r: r["timestamp_us"])


def _assign_imu_seq(imu_rows_sorted: list) -> tuple:
    """Assign `seq` = capture-order row index (0-based) to the imu stream
    -- imu is the reference clock every other lockstep stream's `seq` is
    matched against (protocol/spec/flight_log.yaml `seq` column).

    Returns:
        (rows, seq_lookup) where `seq_lookup` maps
        (timestamp_us, occurrence_index_within_that_timestamp) -> seq, for
        `_assign_seq_by_occurrence()` to look up the other lockstep
        streams' rows against.
    imu ストリームに `seq` = 捕捉順の行番号（0始まり）を付与する -- imu は
    他の全ロックステップ系ストリームの `seq` が対応付けの基準にする時計
    （protocol/spec/flight_log.yaml の `seq` 列）。

    戻り値: (rows, seq_lookup)。seq_lookup は
    (timestamp_us, その時刻内での出現順) -> seq の辞書で、
    `_assign_seq_by_occurrence()` が他のロックステップ系ストリームの行を
    引くのに使う。
    """
    seq_lookup: dict[tuple, int] = {}
    occurrence: dict[int, int] = defaultdict(int)
    for i, row in enumerate(imu_rows_sorted):
        row["seq"] = i
        ts = row["timestamp_us"]
        k = occurrence[ts]
        seq_lookup[(ts, k)] = i
        occurrence[ts] += 1
    return imu_rows_sorted, seq_lookup


def _assign_seq_by_occurrence(rows_sorted: list, seq_lookup: dict) -> tuple:
    """Assign `seq` to a non-imu lockstep stream's rows by matching this
    stream's own (timestamp_us, occurrence-index-within-that-timestamp)
    against `seq_lookup` (built from the imu stream by
    `_assign_imu_seq()`). A row with no match gets `seq = None` (an empty
    cell once written to CSV).

    This -- rather than assuming the two streams' raw row order lines up
    positionally -- is what lets this handle a stream that is only
    PARTIALLY present per control cycle (e.g. the 400Hz duty/ctrl_output
    entries are optional per unified packet) without misaligning anything.

    Returns:
        (rows, unmapped_count).
    imu 以外のロックステップ系ストリームの行に、このストリーム自身の
    (timestamp_us, その時刻内での出現順) を `_assign_imu_seq()` が作った
    `seq_lookup` に引き当てて `seq` を付与する。一致しない行は
    `seq = None`（CSV に書くと空欄になる）。

    2つのストリームの生の行順が位置的に一致していると仮定するのではなく
    こう照合するのは、制御周期ごとに「部分的にしか存在しない」ストリーム
    （400Hz の duty/ctrl_output エントリは統合パケットへの追加が任意）
    でもズレなく扱うため。

    戻り値: (rows, 対応が取れなかった件数)。
    """
    occurrence: dict[int, int] = defaultdict(int)
    unmapped = 0
    for row in rows_sorted:
        ts = row["timestamp_us"]
        k = occurrence[ts]
        occurrence[ts] += 1
        seq = seq_lookup.get((ts, k))
        row["seq"] = seq
        if seq is None:
            unmapped += 1
    return rows_sorted, unmapped


def _reorder_columns_to_schema(df: pd.DataFrame, stream_name: str) -> pd.DataFrame:
    """Reorder `df`'s columns to match protocol/spec/flight_log.yaml's
    declared column order for `stream_name` (e.g. `seq` right after
    `timestamp_us`), with any column the schema doesn't know about kept at
    the end. Purely cosmetic (readers key by name, not position) but keeps
    written CSVs matching the documented column order.
    `df` の列順を、protocol/spec/flight_log.yaml が宣言する
    `stream_name` の列順（例: `timestamp_us` の直後に `seq`）に合わせる。
    スキーマが知らない列は末尾に残す。見た目だけの整形（読み込み側は
    列名で引き、位置には依らない）だが、書き出す CSV をドキュメント上の
    列順と一致させておく。
    """
    expected = schema.COLUMN_NAMES.get(stream_name)
    if not expected:
        return df
    ordered = [c for c in expected if c in df.columns]
    extra = [c for c in df.columns if c not in expected]
    return df[ordered + extra]


def jsonl_to_bundle(
    jsonl_path,
    out_path,
    source: str = "vehicle",
    tool_name: str = "sf log convert",
    notes: Optional[str] = None,
) -> FlightLog:
    """Convert a legacy per-sample JSONL file into a v1 flight-log bundle.

    Keeps only the ids actually present in `jsonl_path`; any id not in
    `_ID_TO_ROW_BUILDER` (i.e. not one of the ids `udp_capture.py`'s
    `save_jsonl()` ever writes) is counted and reported via a printed
    warning rather than silently dropped.

    Every observation is kept -- NO deduplication by timestamp_us. A
    control cycle that did not receive a new IMU sample reuses the
    previous cycle's `timestamp_us`, but is still a real, distinct
    observation (rate_ref/duty/ctrl_output are freshly computed by the
    control law/mixer every cycle even when the IMU-derived fields are
    stale) -- see the plan's "primary record never drops an observation"
    principle. Rows are stable-sorted by `timestamp_us` per stream.

    For the 6 lockstep streams (`schema.LOCKSTEP_STREAMS`: imu, attitude,
    posvel, rate_ref, motor, ctrl_output), a `seq` column is assigned so
    rows remain uniquely identifiable and pairable across streams even
    when `timestamp_us` repeats: imu gets `seq` = its capture-order row
    index; every other lockstep stream's rows are matched to an imu row by
    (timestamp_us, occurrence order within that timestamp) -- see
    `_assign_imu_seq()`/`_assign_seq_by_occurrence()`. A row that cannot be
    matched gets an empty `seq`, and the count is recorded in `notes`.
    `meta.json`'s `notes` also records how many imu rows had a repeated
    timestamp ("repeated timestamps: N (imu)").

    Args:
        jsonl_path: path to the legacy `.jsonl` file.
        out_path: where to write the new bundle (`.sflog.zip` or a
            directory -- see FlightLog.save()).
        source: meta.json `source` field ("vehicle" for a real capture).
        tool_name: meta.json `tool.name`.
        notes: optional caller-supplied note, combined with the automatic
            repeated-timestamp/unmapped-seq notes this function generates.

    Returns:
        The FlightLog that was written (in-memory streams, not reloaded
        from disk -- so values are exactly what was parsed from the
        JSONL, not rounded by the `%.7g` CSV serialization used on disk).

    レガシーな1サンプル1行 JSONL を v1 フライトログ一式へ変換する。

    `jsonl_path` に実在する id だけを保持する。`_ID_TO_ROW_BUILDER` に無い
    id（`udp_capture.py` の `save_jsonl()` が書くどの id でもない）は件数を
    数えて警告表示する（無言で捨てない）。

    観測は全て保持する -- timestamp_us による重複除去はしない。新しい
    IMU 標本を得られなかった制御周期は前周期の `timestamp_us` を再利用
    するが、それでも実在する別個の観測である（IMU 由来の項目が古い値の
    ままでも、rate_ref/duty/ctrl_output は制御則/ミキサーがその周期に
    新しく計算する）-- 計画書の「一次記録は観測を捨てない」原則に従う。
    各ストリームの行は `timestamp_us` で安定ソートする。

    6つのロックステップ系ストリーム（`schema.LOCKSTEP_STREAMS`: imu,
    attitude, posvel, rate_ref, motor, ctrl_output）には `seq` 列を
    付与し、`timestamp_us` が重複していても行を一意に識別・ストリーム間
    で対応付けできるようにする: imu には捕捉順の行番号を `seq` として
    与え、他のロックステップ系ストリームの各行は (timestamp_us,
    その時刻内での出現順) で imu の行に対応付ける（
    `_assign_imu_seq()`/`_assign_seq_by_occurrence()` 参照）。対応が
    取れなかった行は `seq` が空欄になり、件数を `notes` に記録する。
    `meta.json` の `notes` には、imu で時刻が重複した行数
    （"repeated timestamps: N (imu)"）も記録する。

    戻り値は書き出した FlightLog（メモリ上のストリームそのもの -- ディスク
    上で使う `%.7g` CSV シリアライズによる丸めは受けていない。つまり
    JSONL からパースした値そのまま）。
    """
    rows_by_stream: dict[str, list[dict]] = defaultdict(list)
    unknown_id_counts: dict[str, int] = defaultdict(int)

    with open(jsonl_path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            builder = _ID_TO_ROW_BUILDER.get(obj.get("id"))
            if builder is None:
                unknown_id_counts[obj.get("id")] += 1
                continue
            for stream_name, row in builder(obj).items():
                rows_by_stream[stream_name].append(row)

    if unknown_id_counts:
        print(f"  jsonl_to_bundle: skipped unsupported ids: {dict(unknown_id_counts)}")

    notes_parts = [notes] if notes else []

    # imu first -- it is the seq reference every other lockstep stream is
    # matched against, so it must be assigned before them.
    # imu を先に処理する -- 他の全ロックステップ系ストリームが対応付けの
    # 基準にする seq の出所なので、他より先に割り当てる必要がある。
    seq_lookup: dict[tuple, int] = {}
    streams: dict[str, pd.DataFrame] = {}
    imu_rows = rows_by_stream.get("imu")
    if imu_rows is not None:
        imu_sorted = _sort_rows_stable(imu_rows)
        imu_sorted, seq_lookup = _assign_imu_seq(imu_sorted)
        imu_df = pd.DataFrame(imu_sorted)
        n_repeated = int(imu_df["timestamp_us"].duplicated().sum())
        if n_repeated > 0:
            notes_parts.append(f"repeated timestamps: {n_repeated} (imu)")
        streams["imu"] = _reorder_columns_to_schema(imu_df, "imu")

    for stream_name, rows in rows_by_stream.items():
        if stream_name == "imu":
            continue
        rows_sorted = _sort_rows_stable(rows)
        if stream_name in schema.LOCKSTEP_STREAMS:
            if imu_rows is None:
                for row in rows_sorted:
                    row["seq"] = None
                unmapped = len(rows_sorted)
            else:
                rows_sorted, unmapped = _assign_seq_by_occurrence(rows_sorted, seq_lookup)
            if unmapped > 0:
                notes_parts.append(f"{stream_name}: {unmapped} rows without a matching imu seq")
        df = pd.DataFrame(rows_sorted)
        streams[stream_name] = _reorder_columns_to_schema(df, stream_name)

    combined_notes = "; ".join(notes_parts) if notes_parts else None

    meta = make_meta(
        source=source,
        tool_name=tool_name,
        tool_version=_TOOL_VERSION,
        notes=combined_notes,
        streams=streams,
    )
    schema_json = schema.schema_for(streams.keys())
    log = FlightLog(meta=meta, schema=schema_json, streams=streams)
    log.save(out_path)
    return log


# =============================================================================
# bundle_to_jsonl()
# =============================================================================


def _nz(value):
    """None/NaN -> None, else the value unchanged (for optional legacy
    JSONL fields that must be OMITTED, not written as null).
    None/NaN -> None、それ以外はそのまま（省略すべき任意フィールド用。
    null として書くのではなく、キー自体を省く判定に使う）。
    """
    if value is None:
        return None
    if isinstance(value, float) and pd.isna(value):
        return None
    return value


def bundle_to_jsonl(log: FlightLog, out_path) -> int:
    """Write `log` back out as legacy per-sample JSONL, byte-for-byte
    compatible in id/field-name/list-shape with `udp_capture.py`'s
    `save_jsonl()` -- the escape hatch for `analysis/scripts/` and other
    one-off research scripts not migrated to `lib/sflog`.

    Only streams present in `log.streams` contribute lines (a stream
    absent from the bundle writes nothing, matching every other "absent,
    not invented" rule in this package). Numeric rounding mirrors
    `save_jsonl()`'s own lambdas exactly (e.g. `ctrl_ref`'s 4-decimal
    rounding) so a bundle converted FROM a legacy JSONL round-trips back
    to identical values.

    Returns:
        Number of JSON lines written.

    `log` をレガシーな1サンプル1行 JSONL へ書き戻す。id・フィールド名・
    リスト形状は `udp_capture.py` の `save_jsonl()` とバイト単位で互換 --
    `lib/sflog` へ未移行の `analysis/scripts/` 等のための退避路。

    `log.streams` に存在するストリームだけが行を出す（無いストリームは
    何も書かない -- 本パッケージ全体の「無いものは無い」規約に合わせる）。
    数値の丸めは `save_jsonl()` 自身のラムダに正確に合わせる（例:
    `ctrl_ref` の小数点以下4桁丸め）ので、レガシー JSONL から変換した
    バンドルは元の値へそのまま往復する。

    戻り値: 書き込んだ JSON 行数。
    """
    entries: list[tuple[int, dict]] = []

    imu = log.streams.get("imu")
    attitude = log.streams.get("attitude")
    if imu is not None:
        # Pair by `seq`, not timestamp_us: timestamp_us legitimately repeats
        # (a control cycle that reused a stale IMU sample), so a
        # timestamp-keyed dict would collapse distinct rows onto the same
        # key and silently mispair them. Fall back to timestamp_us only for
        # a bundle predating the `seq` column (rare; the fallback is exact
        # as long as timestamps happen to be unique in that bundle).
        # timestamp_us ではなく `seq` で対応付ける: timestamp_us は
        # 正当に重複し得る（制御周期が古い IMU 標本を再利用した場合）ため、
        # 時刻をキーにした辞書では別々の行が同じキーに潰れて黙って
        # 誤対応する。`seq` 列が無い（seq 導入前の）バンドルに限り
        # timestamp_us へフォールバックする（その場合はそのバンドル内で
        # 時刻がたまたま一意である限り正確）。
        use_seq = (
            attitude is not None and "seq" in imu.columns and "seq" in attitude.columns
        )
        if attitude is None:
            att_by_key = {}
        elif use_seq:
            att_by_key = {int(r["seq"]): r for _, r in attitude.iterrows() if pd.notna(r["seq"])}
        else:
            att_by_key = {int(r["timestamp_us"]): r for _, r in attitude.iterrows()}

        for _, r in imu.iterrows():
            ts = int(r["timestamp_us"])
            obj = {
                "id": "imu",
                "ts": ts,
                "gyro": [r["gyro_x"], r["gyro_y"], r["gyro_z"]],
                "accel": [r["accel_x"], r["accel_y"], r["accel_z"]],
                "gyro_raw": [r["gyro_raw_x"], r["gyro_raw_y"], r["gyro_raw_z"]],
                "accel_raw": [r["accel_raw_x"], r["accel_raw_y"], r["accel_raw_z"]],
            }
            key = int(r["seq"]) if use_seq and pd.notna(r["seq"]) else ts
            a = att_by_key.get(key)
            if a is not None:
                obj["quat"] = [a["quat_w"], a["quat_x"], a["quat_y"], a["quat_z"]]
                obj["gyro_bias"] = [a["gyro_bias_x"], a["gyro_bias_y"], a["gyro_bias_z"]]
                obj["accel_bias"] = [a["accel_bias_x"], a["accel_bias_y"], a["accel_bias_z"]]
            entries.append((ts, obj))

    posvel = log.streams.get("posvel")
    if posvel is not None:
        for _, r in posvel.iterrows():
            ts = int(r["timestamp_us"])
            entries.append((ts, {
                "id": "posvel", "ts": ts,
                "pos": [r["pos_x"], r["pos_y"], r["pos_z"]],
                "vel": [r["vel_x"], r["vel_y"], r["vel_z"]],
            }))

    pilot = log.streams.get("pilot")
    if pilot is not None:
        for _, r in pilot.iterrows():
            ts = int(r["timestamp_us"])
            entries.append((ts, {
                "id": "ctrl", "ts": ts,
                "throttle": r["throttle"], "roll": r["roll"], "pitch": r["pitch"], "yaw": r["yaw"],
            }))

    flow = log.streams.get("flow")
    if flow is not None:
        for _, r in flow.iterrows():
            ts = int(r["timestamp_us"])
            entries.append((ts, {
                "id": "flow", "ts": ts,
                "dx": int(r["dx"]), "dy": int(r["dy"]), "quality": int(r["quality"]),
            }))

    tof_bottom = log.streams.get("tof_bottom")
    if tof_bottom is not None:
        for _, r in tof_bottom.iterrows():
            ts = int(r["timestamp_us"])
            entries.append((ts, {"id": "tof_b", "ts": ts, "distance": r["distance"], "status": int(r["status"])}))

    tof_front = log.streams.get("tof_front")
    if tof_front is not None:
        for _, r in tof_front.iterrows():
            ts = int(r["timestamp_us"])
            entries.append((ts, {"id": "tof_f", "ts": ts, "distance": r["distance"], "status": int(r["status"])}))

    baro = log.streams.get("baro")
    if baro is not None:
        for _, r in baro.iterrows():
            ts = int(r["timestamp_us"])
            # Inverse of _row_from_baro()'s hPa -> Pa conversion: the legacy
            # JSONL field carries raw hPa, but the bundle's baro.csv is
            # unified to Pa (protocol/spec/flight_log.yaml baro.pressure) --
            # divide back by 100 so a bundle converted FROM a legacy JSONL
            # round-trips to the original value.
            # _row_from_baro() の hPa -> Pa 変換の逆: レガシー JSONL の
            # フィールドは生の hPa を運ぶが、バンドルの baro.csv は Pa に
            # 統一されている（protocol/spec/flight_log.yaml の
            # baro.pressure）-- 100 で割り戻すことで、レガシー JSONL から
            # 変換したバンドルが元の値へそのまま往復する。
            entries.append((ts, {
                "id": "baro", "ts": ts,
                "altitude": r["altitude"],
                "pressure": r["pressure"] / _HPA_TO_PA,
            }))

    mag = log.streams.get("mag")
    if mag is not None:
        for _, r in mag.iterrows():
            ts = int(r["timestamp_us"])
            entries.append((ts, {"id": "mag", "ts": ts, "x": r["x"], "y": r["y"], "z": r["z"]}))

    ctrl_ref = log.streams.get("ctrl_ref")
    if ctrl_ref is not None:
        for _, r in ctrl_ref.iterrows():
            ts = int(r["timestamp_us"])
            entries.append((ts, {
                "id": "ctrl_ref", "ts": ts,
                "mode": int(r["flight_mode"]),
                "angle_ref": [r["angle_ref_roll"], r["angle_ref_pitch"]],
                "total_thrust": round(float(r["total_thrust"]), 4),
                "motor_duty": [round(float(r[f"duty_{m}"]), 4) for m in ("FR", "RR", "RL", "FL")],
                "alt_sp": round(float(r["alt_setpoint"]), 4),
                "alt_vel_target": round(float(r["alt_vel_target"]), 4),
                "climb_cmd": round(float(r["climb_rate_cmd"]), 4),
                "pos_sp": [round(float(r["pos_setpoint_x"]), 4), round(float(r["pos_setpoint_y"]), 4)],
            }))

    eskf_cov = log.streams.get("eskf_cov")
    if eskf_cov is not None:
        for _, r in eskf_cov.iterrows():
            ts = int(r["timestamp_us"])
            entries.append((ts, {
                "id": "p_diag", "ts": ts,
                "pos": [r["p_pos_x"], r["p_pos_y"], r["p_pos_z"]],
                "vel": [r["p_vel_x"], r["p_vel_y"], r["p_vel_z"]],
                "att": [r["p_att_x"], r["p_att_y"], r["p_att_z"]],
                "bg": [r["p_bg_x"], r["p_bg_y"], r["p_bg_z"]],
                "ba": [r["p_ba_x"], r["p_ba_y"], r["p_ba_z"]],
            }))

    rate_ref = log.streams.get("rate_ref")
    if rate_ref is not None:
        for _, r in rate_ref.iterrows():
            ts = int(r["timestamp_us"])
            entries.append((ts, {
                "id": "rate_ref", "ts": ts,
                "rate_ref": [r["rate_ref_roll"], r["rate_ref_pitch"], r["rate_ref_yaw"]],
            }))

    motor = log.streams.get("motor")
    if motor is not None:
        for _, r in motor.iterrows():
            ts = int(r["timestamp_us"])
            entries.append((ts, {
                "id": "duty400", "ts": ts,
                "duty": [round(float(r[f"duty_{m}"]), 4) for m in ("FR", "RR", "RL", "FL")],
            }))

    status = log.streams.get("status")
    if status is not None:
        for _, r in status.iterrows():
            ts = int(r["timestamp_us"])
            obj = {
                "id": "status", "ts": ts,
                "uptime_ms": int(r["uptime_ms"]),
                "voltage": round(float(r["voltage"]), 3),
                "flight_state": int(r["flight_state"]),
                "eskf_status": int(r["eskf_status"]),
            }
            if _nz(r.get("pid_roll_kp")) is not None:
                for axis in _PID_AXES:
                    obj[f"pid_{axis}"] = [float(r[f"pid_{axis}_{term}"]) for term in _PID_TERMS]
            current_ma = _nz(r.get("current_ma"))
            if current_ma is not None:
                obj["current_ma"] = round(float(current_ma), 1)
            entries.append((ts, obj))

    entries.sort(key=lambda e: e[0])

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        for _, obj in entries:
            f.write(json.dumps(obj, separators=(",", ":")) + "\n")

    return len(entries)


# =============================================================================
# aligned_to_csv()
# =============================================================================


def aligned_to_csv(
    log: FlightLog,
    out_path,
    rate_note: Optional[str] = None,
    base: str = "imu",
    method: str = "hold",
    streams=None,
) -> Path:
    """Write one aligned (multi-rate, held/nearest) table as a DERIVED CSV,
    with a sidecar "<out>.meta.json" that marks it `derived: true` (plan
    section 2.4 -- alignment is a read-time operation and is never a
    primary record on disk without saying so).

    Args:
        log: the source FlightLog.
        out_path: where to write the aligned CSV.
        rate_note: optional human-readable note about the target rate
            (e.g. "400Hz", matching `sf log convert --aligned --rate 400`);
            stored in the sidecar only, has no effect on the computation.
        base, method, streams: forwarded to `align.aligned()`.

    Returns:
        The Path written (the CSV, not the sidecar).

    整列表（多レート、保持/最近傍）を派生 CSV として書き出し、
    "<out>.meta.json" に `derived: true` を明記したサイドカーを添える
    （計画書 2.4節 -- 整列は読み込み時の操作であり、そう明記せずに
    ディスク上の一次記録にはしない）。

    引数 rate_note は目標レートの説明（例: "400Hz"、
    `sf log convert --aligned --rate 400` に対応）。サイドカーに記録する
    だけで計算そのものには影響しない。他の引数は align.aligned() へ
    そのまま渡す。

    戻り値: 書き込んだ Path（CSV 本体。サイドカーではない）。
    """
    out_path = Path(out_path)
    table = _aligned(log, base=base, streams=streams, method=method)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(_dataframe_to_csv_text(table), encoding="utf-8")

    sidecar = {
        "derived": True,
        "derived_from": "aligned",
        "base": base,
        "method": method,
        "rate_note": rate_note,
        "source_meta": log.meta,
    }
    sidecar_path = out_path.with_name(out_path.name + ".meta.json")
    sidecar_path.write_text(json.dumps(sidecar, ensure_ascii=False, indent=2), encoding="utf-8")

    return out_path
