"""
test_align_equivalence.py - Cross-check lib/sflog's aligned() output against
the CURRENT production code path (`tools/log_analyzer/udp_capture.py`'s
`save_stream_csv()`, the 400Hz merged CSV `sf sysid fit` reads today), using
a real local capture if one is available.
test_align_equivalence.py - lib/sflog の aligned() の出力を、現行の本番
コード経路（`tools/log_analyzer/udp_capture.py` の `save_stream_csv()`、
現在 `sf sysid fit` が読む400Hzマージ済みCSV）と、ローカルの実キャプチャが
あればそれを使って突き合わせる。

Strategy: udp_capture.py has no function that parses a JSONL file back into
a `UDPTelemetryCapture` object (JSONL is normally produced FROM a live
capture, not read back), so this test reconstructs the minimal subset of
`UDPTelemetryCapture.samples` that `save_stream_csv()` actually reads --
the exact inverse of `save_jsonl()`'s `JSONL_FORMAT` lambdas for the ids
that feed that method (imu, rate_ref, ctrl_ref, duty400, status). This is
read-only use of `udp_capture.py` (imported, never modified) per the task's
strict rules.
方針: udp_capture.py には JSONL を `UDPTelemetryCapture` オブジェクトへ
戻す関数が無い（JSONL は通常ライブキャプチャから作るものであり、
読み戻す用途は無い）ため、本テストは `save_stream_csv()` が実際に読む
`UDPTelemetryCapture.samples` の最小部分集合を、`save_jsonl()` の
`JSONL_FORMAT` ラムダのうち同メソッドへ供給する id（imu, rate_ref,
ctrl_ref, duty400, status）分だけ厳密な逆変換で再構築する。
udp_capture.py は読み取り専用で使う（import のみ、一切変更しない） --
タスクの厳格な規則に従う。
"""

from __future__ import annotations

import glob
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

import pandas as pd
import pytest

from sflog.align import aligned
from sflog.convert import jsonl_to_bundle

REPO_ROOT = Path(__file__).resolve().parents[3]
LOG_ANALYZER_DIR = REPO_ROOT / "tools" / "log_analyzer"

TOLERANCE = 1e-6

# (save_stream_csv() column name, aligned()'s column name for the same
# physical quantity). Both empty/absent gracefully -- a dataset without,
# say, ctrl_output samples simply contributes zero comparisons for those
# four pairs, it does not fail the test.
# (save_stream_csv() の列名, aligned() の同じ物理量に対応する列名)。
# どちらか欠けても許容する -- 例えば ctrl_output サンプルが無い
# データセットでは、その4組の比較件数が単に0になるだけで、テストは
# 失敗しない。
COLUMN_PAIRS = [
    ("gyro_x", "gyro_x"), ("gyro_y", "gyro_y"), ("gyro_z", "gyro_z"),
    ("accel_x", "accel_x"), ("accel_y", "accel_y"), ("accel_z", "accel_z"),
    ("quat_w", "quat_w"), ("quat_x", "quat_x"), ("quat_y", "quat_y"), ("quat_z", "quat_z"),
    ("gyro_bias_x", "gyro_bias_x"), ("gyro_bias_y", "gyro_bias_y"), ("gyro_bias_z", "gyro_bias_z"),
    ("accel_bias_x", "accel_bias_x"), ("accel_bias_y", "accel_bias_y"), ("accel_bias_z", "accel_bias_z"),
    ("rate_ref_roll", "rate_ref_roll"), ("rate_ref_pitch", "rate_ref_pitch"), ("rate_ref_yaw", "rate_ref_yaw"),
    ("angle_ref_roll", "angle_ref_roll"), ("angle_ref_pitch", "angle_ref_pitch"),
    ("total_thrust", "total_thrust"),
    ("motor_duty_FR", "duty_FR"), ("motor_duty_RR", "duty_RR"),
    ("motor_duty_RL", "duty_RL"), ("motor_duty_FL", "duty_FL"),
    ("flight_mode", "flight_mode"),
    ("vbat", "voltage"),
    ("ctrl_output_thrust", "thrust"),
    ("ctrl_output_torque_roll", "torque_roll"),
    ("ctrl_output_torque_pitch", "torque_pitch"),
    ("ctrl_output_torque_yaw", "torque_yaw"),
]


def _newest_local_jsonl():
    candidates = sorted(glob.glob(str(REPO_ROOT / "logs" / "stampfly_udp_*.jsonl")))
    if not candidates:
        return None
    return Path(max(candidates, key=os.path.getmtime))


def _reconstruct_samples(jsonl_path: Path, uc) -> dict:
    """Rebuild the subset of `UDPTelemetryCapture.samples` that
    `save_stream_csv()` reads, from a legacy JSONL file -- the exact
    inverse of `save_jsonl()`'s `JSONL_FORMAT` lambdas for the relevant ids.
    `save_stream_csv()` が読む `UDPTelemetryCapture.samples` の部分集合を、
    レガシー JSONL から再構築する -- 関係する id についての
    `save_jsonl()` の `JSONL_FORMAT` ラムダの厳密な逆変換。
    """
    samples = defaultdict(list)
    with open(jsonl_path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            oid, ts = obj["id"], obj["ts"]

            if oid == "imu":
                q, gb, ab = obj["quat"], obj["gyro_bias"], obj["accel_bias"]
                g, a = obj["gyro"], obj["accel"]
                samples[uc.PKT_IMU_ESKF].append({
                    "timestamp_us": ts,
                    "gyro_x": g[0], "gyro_y": g[1], "gyro_z": g[2],
                    "accel_x": a[0], "accel_y": a[1], "accel_z": a[2],
                    "quat_w": q[0], "quat_x": q[1], "quat_y": q[2], "quat_z": q[3],
                    "gyro_bias_x": gb[0], "gyro_bias_y": gb[1], "gyro_bias_z": gb[2],
                    "accel_bias_x": ab[0], "accel_bias_y": ab[1], "accel_bias_z": ab[2],
                })
            elif oid == "rate_ref":
                # JSONL stores rate_ref already divided by kRateRefScale
                # (1000.0); save_stream_csv() expects the pre-division raw
                # value and divides again itself -- multiply back.
                # JSONL は kRateRefScale（1000.0）で割った後の値を持つ；
                # save_stream_csv() は除算前の生値を期待し自身で割るため、
                # 掛け戻す。
                rr = obj["rate_ref"]
                samples[uc.PKT_RATE_REF].append({
                    "timestamp_us": ts,
                    "rate_ref_roll": rr[0] * 1000.0,
                    "rate_ref_pitch": rr[1] * 1000.0,
                    "rate_ref_yaw": rr[2] * 1000.0,
                })
            elif oid == "ctrl_ref":
                angle_ref, duty = obj["angle_ref"], obj["motor_duty"]
                samples[uc.PKT_CTRL_REF].append({
                    "timestamp_us": ts,
                    "flight_mode": obj["mode"],
                    # JSONL divided by kAngleRefScale (10000.0); multiply back
                    # for the same reason as rate_ref above.
                    "angle_ref_roll": angle_ref[0] * 10000.0,
                    "angle_ref_pitch": angle_ref[1] * 10000.0,
                    "total_thrust": obj.get("total_thrust", 0.0),
                    "motor_duty_FR": duty[0], "motor_duty_RR": duty[1],
                    "motor_duty_RL": duty[2], "motor_duty_FL": duty[3],
                })
            elif oid == "duty400":
                d = obj["duty"]
                samples[uc.PKT_DUTY400].append({
                    "timestamp_us": ts,
                    "duty_FR": d[0], "duty_RR": d[1], "duty_RL": d[2], "duty_FL": d[3],
                })
            elif oid == "status":
                samples[uc.PKT_STATUS].append({"timestamp_us": ts, "voltage": obj["voltage"]})
            # every other id is irrelevant to save_stream_csv() -- ignored.

    return samples


def test_bundle_imu_row_count_matches_save_stream_csv_row_count(tmp_path):
    """No rows lost: `jsonl_to_bundle()`'s imu stream must have EXACTLY as
    many rows as the current production writer (`save_stream_csv()`) emits
    for the same capture -- plan section 2.2 "重複と欠落の扱い" requires
    every observation to survive, with NO deduplication by timestamp_us.
    For `logs/stampfly_udp_20260908T121243.jsonl` this is 11736 (plan
    section 7's measured row count for that file).
    行の欠落が無いこと: `jsonl_to_bundle()` の imu ストリームの行数は、
    同じキャプチャに対する現行の本番書き出しコード（`save_stream_csv()`）
    の行数と厳密に一致しなければならない -- 計画書 2.2節「重複と欠落の
    扱い」により、timestamp_us による重複除去なしに全ての観測が残る
    べき。`logs/stampfly_udp_20260908T121243.jsonl` ではこれは 11736
    （計画書7節がこのファイルについて実測した行数）。
    """
    jsonl_path = _newest_local_jsonl()
    if jsonl_path is None:
        pytest.skip("no logs/stampfly_udp_*.jsonl available locally")

    sys.path.insert(0, str(LOG_ANALYZER_DIR))
    import udp_capture as uc  # noqa: E402  (read-only import, never modified)

    samples = _reconstruct_samples(jsonl_path, uc)
    if not samples.get(uc.PKT_IMU_ESKF):
        pytest.skip(f"{jsonl_path.name} has no 'imu' samples to compare")

    capture = uc.UDPTelemetryCapture(vehicle_ip="127.0.0.1")
    capture.samples = samples
    legacy_csv_path = tmp_path / "legacy_stream.csv"
    capture.save_stream_csv(str(legacy_csv_path))
    legacy_row_count = len(pd.read_csv(legacy_csv_path))

    bundle_path = tmp_path / "row_count.sflog.zip"
    bundle = jsonl_to_bundle(jsonl_path, bundle_path, notes="test_align_equivalence.py row count")

    assert len(bundle.streams["imu"]) == legacy_row_count
    if jsonl_path.name == "stampfly_udp_20260908T121243.jsonl":
        assert legacy_row_count == 11736


def test_aligned_matches_save_stream_csv_on_a_real_capture(tmp_path):
    jsonl_path = _newest_local_jsonl()
    if jsonl_path is None:
        pytest.skip("no logs/stampfly_udp_*.jsonl available locally")

    sys.path.insert(0, str(LOG_ANALYZER_DIR))
    import udp_capture as uc  # noqa: E402  (read-only import, never modified)

    samples = _reconstruct_samples(jsonl_path, uc)
    if not samples.get(uc.PKT_IMU_ESKF):
        pytest.skip(f"{jsonl_path.name} has no 'imu' samples to compare")

    capture = uc.UDPTelemetryCapture(vehicle_ip="127.0.0.1")
    capture.samples = samples
    legacy_csv_path = tmp_path / "legacy_stream.csv"
    capture.save_stream_csv(str(legacy_csv_path))
    legacy = pd.read_csv(legacy_csv_path).reset_index(drop=True)

    bundle_path = tmp_path / "equivalence.sflog.zip"
    bundle = jsonl_to_bundle(jsonl_path, bundle_path, notes="test_align_equivalence.py")
    table = aligned(bundle, base="imu", method="hold").reset_index(drop=True)

    # save_stream_csv() writes ONE merged row per control cycle, pairing
    # imu[i]/rate_ref[i]/duty400[i] POSITIONALLY (tools/log_analyzer/
    # udp_capture.py save_stream_csv(): `n = len(imu)`, then every block is
    # indexed by i, with no timestamp-based re-sorting of imu itself) --
    # that positional pairing is exactly what imu's `seq` = capture-order
    # row index means (protocol/spec/flight_log.yaml `seq` column; plan
    # section 2.2). So row i of `legacy` corresponds to row i of `table`
    # (base="imu"). This must NOT be joined on a shared timestamp_us:
    # timestamp_us legitimately repeats across distinct control cycles
    # (plan section 2.2/7 -- ~1561 repeats in this real capture) and a
    # join keyed on it would silently fan out into a many-to-many
    # cross-product instead of the true row-for-row correspondence.
    # save_stream_csv() は「制御周期1件=CSV1行」でマージ保存し、
    # imu[i]/rate_ref[i]/duty400[i] を位置で対応付ける（
    # tools/log_analyzer/udp_capture.py の save_stream_csv(): `n =
    # len(imu)` として各ブロックを i で添字アクセスし、imu 自体を
    # タイムスタンプで並べ替え直すことはしない）-- その位置的対応こそが
    # imu の `seq` = 捕捉順行番号（protocol/spec/flight_log.yaml の
    # `seq` 列、計画書 2.2節）の意味そのもの。よって `legacy` の行 i は
    # `table`（base="imu"）の行 i に対応する。これは共有された
    # timestamp_us で結合してはならない: timestamp_us は別個の制御周期間
    # で正当に重複し得（計画書 2.2/7節 -- この実キャプチャで約1561件）、
    # それをキーにした結合は本来の1行対1行の対応ではなく多対多の直積へ
    # 黙って膨らんでしまう。
    assert len(legacy) == len(table), (
        f"row count mismatch: legacy(save_stream_csv)={len(legacy)} vs "
        f"aligned(bundle, base='imu')={len(table)} -- lib/sflog must keep "
        "exactly as many imu rows as the current production writer, no dedup"
    )

    total_compared = 0
    mismatches = []
    for legacy_col, aligned_col in COLUMN_PAIRS:
        if legacy_col not in legacy.columns or aligned_col not in table.columns:
            continue  # this dataset doesn't carry this signal -- not an error

        a = pd.to_numeric(legacy[legacy_col], errors="coerce")
        b = pd.to_numeric(table[aligned_col], errors="coerce")
        # NaN on either side means "no data yet" in one of the two
        # representations (aligned() is NaN before the first observation;
        # legacy uses a 0/''-default before the first CtrlRef/Status
        # packet) -- comparing only where BOTH have real data is exactly
        # the "on the common rows" the task asks for.
        # どちらかが NaN は「まだデータが無い」を意味する（aligned() は
        # 最初の観測前が NaN、legacy は最初の CtrlRef/Status パケット前が
        # 0/''既定値）-- 両方に実データがある行だけを比較するのが、まさに
        # タスクが求める「共通する行で」の意味。
        valid = a.notna() & b.notna()
        if valid.sum() == 0:
            continue

        diff = (a[valid] - b[valid]).abs()
        max_diff = float(diff.max())
        total_compared += int(valid.sum())
        if max_diff >= TOLERANCE:
            mismatches.append(f"{legacy_col} vs {aligned_col}: max |diff|={max_diff:.3g} over {valid.sum()} rows")

    assert not mismatches, "\n".join(mismatches)
    assert total_compared > 0, "no column pair had any comparable (non-NaN on both sides) rows"
    print(f"\n  test_align_equivalence: {total_compared} value comparisons, all within {TOLERANCE}")
