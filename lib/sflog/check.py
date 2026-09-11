"""
check.py - Structural/physical sanity checks for a flight-log v1 bundle.
check.py - フライトログ v1 一式の構造・物理妥当性検査。

This is the implementation behind the future `sf log check` command
(Phase 1). Read-only: never mutates the bundle it inspects.
将来の `sf log check` コマンド（Phase 1）の実体。読み取り専用で、検査対象の
一式を変更しない。

@design docs/plans/flight-log-format-plan.md section 2.5 / 3.5
"""

from __future__ import annotations

from dataclasses import dataclass

import pandas as pd

from . import schema

# =============================================================================
# Physical sanity thresholds
# 物理妥当性の閾値
# =============================================================================
# A value outside these ranges is almost always a units/decoding bug (e.g.
# reading a quantized int16 without dividing by its scale), not real flight
# data -- see data_stream_wire.hpp's quantize()/kBiasScale etc. for the
# encodings these guard against.
# 範囲外の値はほぼ確実に単位・デコードのバグ（量子化された int16 をスケール
# で割り戻し忘れる等）であり、実際の飛行データではない --
# data_stream_wire.hpp の quantize()/kBiasScale 等、想定するエンコードの
# 参考。

GYRO_ABS_MAX_RAD_S = 50.0
ACCEL_ABS_MAX_M_S2 = 200.0
QUAT_NORM_LO = 0.9
QUAT_NORM_HI = 1.1
DUTY_LO = 0.0
DUTY_HI = 1.05
VOLTAGE_LO = 2.5
VOLTAGE_HI = 5.0
RATE_TOLERANCE_FRACTION = 0.30  # +-30% of nominal_rate_hz


@dataclass
class Finding:
    """One check result.
    検査結果1件。

    Attributes:
        level: "error" (the bundle is not v1-conformant / unusable) or
            "warning" (unusual but not necessarily wrong).
        stream: stream name the finding is about, or "meta.json"/
            "schema.json" for bundle-level findings.
        message: human-readable detail (English; this is a developer/CI
            facing diagnostic, not end-user text requiring translation).
        level: "error"（v1 として不適合/使用不可）または "warning"
            （異常だが誤りとは限らない）。
        stream: 対象ストリーム名。バンドル全体に関わる指摘は
            "meta.json"/"schema.json"。
        message: 人間可読な詳細（英語。開発者/CI 向け診断であり、
            エンドユーザー向けの翻訳が要る文言ではない）。
    """

    level: str
    stream: str
    message: str

    def __str__(self) -> str:
        return f"[{self.level}] {self.stream}: {self.message}"


def is_ok(findings) -> bool:
    """True when `findings` has no "error"-level entries (warnings are
    fine).
    `findings` に "error" レベルが無ければ True（warning は許容）。
    """
    return not any(f.level == "error" for f in findings)


def check_bundle(path_or_log) -> list:
    """Run every structural/physical check and return the findings.

    Args:
        path_or_log: a path (str/Path, `.sflog.zip` or extracted
            directory) or an already-loaded `FlightLog`.

    Returns:
        list[Finding], possibly empty.

    全ての構造・物理検査を行い、指摘一覧を返す。引数はパスか、既に読み込み
    済みの FlightLog。
    """
    from .bundle import FlightLog  # local import: avoid bundle<->check import cycle

    log = path_or_log if isinstance(path_or_log, FlightLog) else FlightLog.load(path_or_log)

    findings: list[Finding] = []
    findings.extend(_check_meta(log))
    findings.extend(_check_schema_json(log))
    findings.extend(_check_required_streams(log))
    findings.extend(_check_lockstep_seq_subset(log))

    for name, df in log.streams.items():
        findings.extend(_check_columns(name, df))
        findings.extend(_check_timestamp(name, df))
        findings.extend(_check_seq(name, df))
        findings.extend(_check_rate(name, df))
        findings.extend(_check_numeric_dtype(name, df))
        findings.extend(_check_unit_sanity(name, df))

    return findings


# =============================================================================
# Bundle-level checks
# バンドル全体の検査
# =============================================================================


def _check_meta(log) -> list:
    if not log.meta:
        return [Finding("error", "meta.json", "meta.json is missing or empty")]
    findings = []
    for key in ("format", "version"):
        if key not in log.meta:
            findings.append(Finding("error", "meta.json", f"missing required key '{key}'"))
    return findings


def _check_schema_json(log) -> list:
    if not log.schema:
        return [Finding("error", "schema.json", "schema.json is missing or empty")]
    return []


def _check_required_streams(log) -> list:
    return [
        Finding("error", name, f"required stream '{name}' is missing")
        for name in schema.REQUIRED_STREAMS
        if name not in log.streams
    ]


def _check_lockstep_seq_subset(log) -> list:
    """Every non-imu lockstep stream's `seq` values must be a subset of
    imu's `seq` values -- imu is the reference clock every control-cycle
    row is defined against (protocol/spec/flight_log.yaml `seq` column;
    plan section 2.2). A `seq` value with no matching imu row means the
    bundle was built inconsistently (e.g. hand-edited, or a future writer
    bug), which is a structural error, not a warning.

    imu 以外の全ロックステップ系ストリームの `seq` 値は、imu の `seq` 値の
    部分集合でなければならない -- imu は全ての制御周期行の定義の基準になる
    時計だから（protocol/spec/flight_log.yaml の `seq` 列、計画書 2.2節）。
    対応する imu 行が無い `seq` 値が存在するのは、一式の構築が一貫していない
    （手編集、または将来の書き出しコードのバグ）ことを意味する構造的な
    誤りであり、warning ではなく error にする。
    """
    imu = log.streams.get("imu")
    if imu is None or "seq" not in imu.columns:
        return []
    imu_seq_ints = _to_integer_series(imu["seq"].dropna())
    if imu_seq_ints is None:
        return []  # already reported by _check_seq("imu", ...)
    imu_seq = set(imu_seq_ints.tolist())

    findings = []
    for name in schema.LOCKSTEP_STREAMS:
        if name == "imu":
            continue
        df = log.streams.get(name)
        if df is None or "seq" not in df.columns:
            continue
        other_ints = _to_integer_series(df["seq"].dropna())
        if other_ints is None:
            continue  # already reported by _check_seq(name, ...)
        extra = sorted(set(other_ints.tolist()) - imu_seq)
        if extra:
            findings.append(
                Finding(
                    "error",
                    name,
                    f"{len(extra)} seq values not present in imu's seq set "
                    f"(e.g. {extra[:5]})",
                )
            )
    return findings


# =============================================================================
# Per-stream checks
# ストリームごとの検査
# =============================================================================


def _to_integer_series(s: pd.Series):
    """Return `s` losslessly cast to int64, or None if any value is not
    integer-valued.

    A CSV read can upcast an all-integer column to float64 when the file's
    dtype inference sees a NaN elsewhere (e.g. an unmapped `seq` cell) --
    accept that as long as every remaining value is still integer-VALUED.
    `s` を int64 へロス無く変換して返す。整数値でない値が1つでもあれば
    None。

    CSV 読み込み時、他の行に NaN（対応の取れなかった `seq` 空欄等）がある
    と全整数列でも float64 へ格上げされることがある -- 残る値が全て整数値
    である限りは許容する。
    """
    if pd.api.types.is_integer_dtype(s):
        return s
    try:
        as_int = s.astype("int64")
    except (ValueError, TypeError):
        return None
    if not (as_int == s).all():
        return None
    return as_int


def _check_columns(name: str, df: pd.DataFrame) -> list:
    expected = schema.COLUMN_NAMES.get(name)
    if expected is None:
        return []  # unknown stream name -- nothing in schema.py to check against
    expected_set, actual_set = set(expected), set(df.columns)
    findings = [
        Finding("error", name, f"missing expected column '{col}'")
        for col in sorted(expected_set - actual_set)
    ]
    findings += [
        Finding("warning", name, f"unexpected extra column '{col}'")
        for col in sorted(actual_set - expected_set)
    ]
    return findings


def _check_timestamp(name: str, df: pd.DataFrame) -> list:
    """timestamp_us must be integer and NON-DECREASING -- equal consecutive
    values are allowed (a control cycle that reused a stale IMU sample
    repeats the previous cycle's timestamp; plan section 2.2 "重複と欠落の
    扱い"). A genuine decrease is a structural error; a run of repeats is
    reported as a warning with the repeat count, never silently dropped.
    timestamp_us は整数かつ非減少でなければならない -- 同値の連続は許容する
    （制御周期が古い IMU 標本を再利用すると前周期の時刻を繰り返す。計画書
    2.2節「重複と欠落の扱い」）。実際の逆行は構造的な誤り。連続した重複は
    件数を添えた warning として報告し、無言では捨てない。
    """
    if "timestamp_us" not in df.columns:
        return []  # already reported by _check_columns

    ts = _to_integer_series(df["timestamp_us"])
    if ts is None:
        return [Finding("error", name, "timestamp_us is not integer-valued")]

    diffs = ts.diff().iloc[1:]  # diff() drops row 0 (no previous row to compare)
    decreasing = diffs < 0
    if decreasing.any():
        first_bad_pos = decreasing.to_numpy().argmax() + 1  # +1: offset dropped row 0
        return [
            Finding(
                "error",
                name,
                f"timestamp_us decreases at row {first_bad_pos} "
                f"({ts.iloc[first_bad_pos - 1]} -> {ts.iloc[first_bad_pos]})",
            )
        ]

    n_repeated = int((diffs == 0).sum())
    if n_repeated > 0:
        # The IMU-reuse explanation only applies to the 400 Hz lockstep
        # streams (plan section 7); other streams just report the count.
        # 「IMU 標本の再利用」の説明は 400 Hz ロックステップ系だけに当てはまる
        # （計画書 §7）。他のストリームは件数だけ報告する。
        reason = (
            " (control cycle reused an IMU sample)"
            if name in schema.LOCKSTEP_STREAMS else ""
        )
        return [
            Finding(
                "warning",
                name,
                f"{n_repeated} repeated timestamps{reason}",
            )
        ]
    return []


def _check_seq(name: str, df: pd.DataFrame) -> list:
    """`seq` (present only on the lockstep streams, protocol/spec/
    flight_log.yaml `seq` column) must be integer and STRICTLY increasing
    among its non-empty values -- unlike timestamp_us it uniquely
    identifies a control cycle, so a repeat or decrease is always a
    structural error. A gap (seq skipping ahead by more than 1) is expected
    whenever a packet/cycle was lost and is reported as a warning with the
    number of missing values, never an error.

    Empty `seq` cells (a legacy-JSONL row `convert.py` could not match to
    an imu row; see the `seq` column's description) are excluded from this
    analysis entirely -- they are already counted in meta.json's `notes`.

    `seq`（protocol/spec/flight_log.yaml の `seq` 列。ロックステップ系
    ストリームにのみ存在）は整数かつ、空欄でない値の中で厳密に増加しな
    ければならない -- timestamp_us と異なり制御周期を一意に識別するため、
    重複・逆行は常に構造的な誤り。飛び（seq が1を超えて進む）はパケット/
    周期の欠落時に正当に起こり、欠落した値の件数を添えた warning として
    報告する（error にはしない）。

    `seq` が空欄の行（`convert.py` が imu 行に対応付けられなかったレガシー
    JSONL の行。`seq` 列の説明を参照）はこの検査から完全に除外する --
    件数は既に meta.json の `notes` に記録されている。
    """
    if "seq" not in df.columns:
        return []

    seq_notna = df["seq"].dropna()
    if len(seq_notna) < 2:
        return []

    seq = _to_integer_series(seq_notna)
    if seq is None:
        return [Finding("error", name, "seq is not integer-valued")]

    diffs = seq.diff().iloc[1:]
    non_increasing = diffs <= 0
    if non_increasing.any():
        first_bad_pos = non_increasing.to_numpy().argmax() + 1
        bad_row = seq.index[first_bad_pos]
        return [
            Finding(
                "error",
                name,
                f"seq is not strictly increasing at row {bad_row} "
                f"({seq.iloc[first_bad_pos - 1]} -> {seq.iloc[first_bad_pos]})",
            )
        ]

    missing = int((diffs - 1).sum())  # sum of (gap size - 1) over every step
    if missing > 0:
        return [
            Finding(
                "warning",
                name,
                f"seq has gaps totalling {missing} missing values (lost packets/cycles)",
            )
        ]
    return []


def _check_rate(name: str, df: pd.DataFrame) -> list:
    info = schema.STREAMS.get(name)
    if info is None or info["nominal_rate_hz"] is None:
        return []  # event-driven / variable-rate stream (truth, events, eskf_cov)
    if "timestamp_us" not in df.columns or len(df) < 2:
        return []

    ts = df["timestamp_us"]
    duration_s = (ts.iloc[-1] - ts.iloc[0]) / 1e6
    if duration_s <= 0:
        return []

    measured_hz = (len(df) - 1) / duration_s
    nominal_hz = info["nominal_rate_hz"]
    if abs(measured_hz - nominal_hz) > RATE_TOLERANCE_FRACTION * nominal_hz:
        return [
            Finding(
                "warning",
                name,
                f"measured rate {measured_hz:.1f} Hz deviates from nominal "
                f"{nominal_hz} Hz by more than {RATE_TOLERANCE_FRACTION * 100:.0f}%",
            )
        ]
    return []


def _check_numeric_dtype(name: str, df: pd.DataFrame) -> list:
    info = schema.STREAMS.get(name)
    if info is None:
        return []
    findings = []
    for col_name, col_type, _unit in info["columns"]:
        if col_type != "float" or col_name not in df.columns:
            continue
        if not pd.api.types.is_numeric_dtype(df[col_name]):
            findings.append(
                Finding("error", name, f"column '{col_name}' is declared float but is not numeric")
            )
    return findings


def _check_unit_sanity(name: str, df: pd.DataFrame) -> list:
    findings = []

    if name == "imu":
        findings += _abs_max_findings(df, ("gyro_x", "gyro_y", "gyro_z"), GYRO_ABS_MAX_RAD_S, name, "rad/s")
        findings += _abs_max_findings(
            df, ("accel_x", "accel_y", "accel_z"), ACCEL_ABS_MAX_M_S2, name, "m/s^2"
        )

    if name == "attitude" and {"quat_w", "quat_x", "quat_y", "quat_z"} <= set(df.columns):
        norm = (
            df["quat_w"] ** 2 + df["quat_x"] ** 2 + df["quat_y"] ** 2 + df["quat_z"] ** 2
        ) ** 0.5
        bad = int(((norm < QUAT_NORM_LO) | (norm > QUAT_NORM_HI)).sum())
        if bad > 0:
            findings.append(
                Finding(
                    "warning",
                    name,
                    f"quaternion norm outside [{QUAT_NORM_LO}, {QUAT_NORM_HI}] in {bad} rows",
                )
            )

    if name in ("motor", "ctrl_ref"):
        for col in ("duty_FR", "duty_RR", "duty_RL", "duty_FL"):
            findings += _range_findings(df, col, DUTY_LO, DUTY_HI, name)

    if name == "status" and "voltage" in df.columns:
        findings += _range_findings(df, "voltage", VOLTAGE_LO, VOLTAGE_HI, name, unit="V")

    return findings


def _abs_max_findings(df: pd.DataFrame, cols, max_abs: float, stream_name: str, unit: str) -> list:
    findings = []
    for col in cols:
        if col not in df.columns:
            continue
        bad = int((df[col].abs() > max_abs).sum())
        if bad > 0:
            findings.append(Finding("warning", stream_name, f"|{col}| > {max_abs} {unit} in {bad} rows"))
    return findings


def _range_findings(df: pd.DataFrame, col: str, lo: float, hi: float, stream_name: str, unit: str = "") -> list:
    if col not in df.columns:
        return []
    bad = int(((df[col] < lo) | (df[col] > hi)).sum())
    if bad == 0:
        return []
    suffix = f" {unit}" if unit else ""
    return [Finding("warning", stream_name, f"column '{col}' outside [{lo}, {hi}]{suffix} in {bad} rows")]
