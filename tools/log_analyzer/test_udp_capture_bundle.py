#!/usr/bin/env python3
"""
test_udp_capture_bundle.py - End-to-end test: synthetic UDP datagrams ->
UDPTelemetryCapture -> save_bundle() -> StampFly flight-log v1 bundle,
verified by loading it back with lib/sflog and running its own
check_bundle() (docs/plans/flight-log-format-plan.md).

Feeds a handful of synthetic packets (reusing test_udp_capture_duty400.py's
own wire-encoding helpers, plus a small standalone-baro-packet helper
defined here) through UDPTelemetryCapture._process_datagram() -- the same
entry point capture()'s real receive loop uses -- covering every stream a
single capture session can produce (imu/attitude/posvel/rate_ref/motor/
ctrl_output from 3 unified packets, plus a standalone baro packet), then
asserts:
  - every expected v1 stream/column is present in the loaded bundle;
  - the `seq` column's values match the unified packet's (unwrapped header
    sequence x 8 + in-packet index);
  - baro.csv's pressure is the wire's hPa converted to v1's SI Pa;
  - lib/sflog.check_bundle() finds no errors.

合成 UDP データグラム -> UDPTelemetryCapture -> save_bundle() -> StampFly
フライトログ v1 一式、という経路をエンドツーエンドで検証する。
lib/sflog で読み戻し、その check_bundle() を走らせて確認する（計画書参照）。

test_udp_capture_duty400.py 自身の電文エンコードヘルパーと、本ファイルに
定義した単独 baro パケット用の小さなヘルパーを再利用し、少数の合成
パケットを UDPTelemetryCapture._process_datagram()（capture() の実受信
ループと同じ入口）へ投入する --
1回のキャプチャセッションで得られる全ストリーム（3個の統合パケットからの
imu/attitude/posvel/rate_ref/motor/ctrl_output、および単独の baro パケット）
を網羅し、以下を確認する:
  - 読み込んだ一式に期待する v1 ストリーム/列が全て存在すること
  - `seq` 列の値が統合パケットの値（展開済みヘッダ sequence × 8 +
    パケット内インデックス）と一致すること
  - baro.csv の pressure が電文の hPa から v1 の SI Pa へ変換されていること
  - lib/sflog.check_bundle() がエラーを検出しないこと

Usage:
    python3 test_udp_capture_bundle.py
    pytest test_udp_capture_bundle.py
"""

import struct
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_capture  # noqa: E402
from test_udp_capture_duty400 import (  # noqa: E402
    build_unified, _duty400_entry, _ctrl_output400_entry, N,
)

import sflog  # noqa: E402

_BARO_HEADER = '<BHB'


def _baro_packet(seq, timestamp_us, altitude_m, pressure_hpa):
    """Build a raw standalone 0x45 Baro datagram (count=1) -- mirrors the
    non-unified SAMPLE_INFO framing parse_packet() also supports (older
    firmware / low-rate sensors sent standalone, not as a unified-packet
    entry).
    単独の 0x45 Baro データグラム（count=1）を組み立てる -- parse_packet()
    が対応するもう一方の枠組み（統合パケットのエントリではなく単独送信、
    旧ファーム/低レートセンサ向け）を模す。
    """
    header = struct.pack(_BARO_HEADER, udp_capture.PKT_BARO, seq, 1)
    payload = struct.pack(udp_capture.FMT_BARO, timestamp_us, altitude_m, pressure_hpa)
    body = header + payload
    checksum = 0
    for b in body:
        checksum ^= b
    return body + bytes([checksum])


def _build_capture_with_three_unified_packets() -> udp_capture.UDPTelemetryCapture:
    """Feed 3 consecutive unified packets (each with a duty400 AND a
    control_output400 entry) plus one standalone baro packet through
    _process_datagram(), covering every stream a real capture session can
    produce except pilot/ctrl_ref/tof/flow/mag/status/eskf_cov (already
    covered individually by other tests; adding them here would duplicate
    without adding coverage).
    統合パケット3個（それぞれ duty400 と control_output400 エントリ付き）と
    単独の baro パケット1個を _process_datagram() へ投入する。
    pilot/ctrl_ref/tof/flow/mag/status/eskf_cov は他のテストで個別に
    カバー済みのためここでは省く（重複するだけでカバレッジは増えない）。
    """
    cap = udp_capture.UDPTelemetryCapture()
    duties = [(0.5, 0.5, 0.5, 0.5)] * N
    co_samples = [(0.3, 0.0, 0.0, 0.0)] * N

    for i, seq in enumerate((1, 2, 3)):
        pkt, _ = build_unified(
            seq, base_ts_us=1_000_000 + i * N * 2500,
            entries=_duty400_entry(duties) + _ctrl_output400_entry(co_samples),
            entry_count=2,
        )
        cap._process_datagram(pkt)

    cap._process_datagram(_baro_packet(1, 1_000_500, altitude_m=12.5, pressure_hpa=1013.25))

    cap.start_time, cap.end_time = 0.0, 1.0
    return cap


def test_bundle_has_every_expected_stream_and_column():
    """Every stream this capture produced must load back with exactly the
    schema-declared columns present."""
    cap = _build_capture_with_three_unified_packets()

    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "flight_test.sflog.zip"
        cap.save_bundle(str(bundle_path), source="vehicle", tool_name="test")
        log = sflog.load(bundle_path)

        expected_streams = {"imu", "attitude", "posvel", "rate_ref", "motor",
                             "ctrl_output", "baro"}
        assert expected_streams <= set(log.streams.keys())

        for name in ("imu", "attitude", "posvel", "rate_ref", "motor", "ctrl_output"):
            expected_cols = set(sflog.schema.COLUMN_NAMES[name])
            assert expected_cols <= set(log.streams[name].columns)
            assert len(log.streams[name]) == 3 * N  # 3 unified packets x 8 sub-samples

        # Streams this capture never produced a sample for must be absent
        # entirely (plan section 2.2), not present as empty files.
        # このキャプチャが1件もサンプルを出さなかったストリームは
        # （空ファイルではなく）そもそも存在しないこと（計画書2.2節）。
        for absent in ("pilot", "ctrl_ref", "tof_bottom", "tof_front", "flow", "mag",
                       "status", "eskf_cov"):
            assert absent not in log.streams


def test_bundle_seq_values_match_unwrapped_header_times_8_plus_index():
    """`seq` on every lockstep stream must equal (unwrapped unified-packet
    header sequence) * 8 + in-packet index, for all 3 packets (headers
    1, 2, 3 -- no wraparound in this test; see
    test_udp_capture_duty400.test_seq_unwraps_across_65535_to_0_wrap for
    the wraparound case)."""
    cap = _build_capture_with_three_unified_packets()

    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "flight_test.sflog.zip"
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)

        expected_seq = [header * 8 + i for header in (1, 2, 3) for i in range(N)]
        for name in ("imu", "attitude", "posvel", "rate_ref", "motor", "ctrl_output"):
            assert list(log.streams[name]["seq"]) == expected_seq


def test_bundle_baro_pressure_converted_hpa_to_pa():
    """baro.csv's `pressure` must be the wire's hPa x 100 (v1's SI Pa
    convention, docs/plans/flight-log-format-plan.md section 2.2 "単位の
    例外"), and `altitude` must pass through unchanged (already meters on
    the wire)."""
    cap = _build_capture_with_three_unified_packets()

    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "flight_test.sflog.zip"
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)

        baro = log.streams["baro"]
        assert len(baro) == 1
        row = baro.iloc[0]
        assert abs(row["pressure"] - 1013.25 * 100.0) < 1e-3
        assert abs(row["altitude"] - 12.5) < 1e-6


def test_bundle_passes_check_bundle_with_no_errors():
    """The bundle this module builds must be fully v1-conformant -- zero
    `error`-level findings from lib/sflog.check_bundle() (warnings, if any,
    are fine; see sflog.check.is_ok())."""
    cap = _build_capture_with_three_unified_packets()

    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "flight_test.sflog.zip"
        cap.save_bundle(str(bundle_path))

        findings = sflog.check_bundle(bundle_path)
        errors = [f for f in findings if f.level == "error"]
        assert errors == [], f"unexpected check_bundle() errors: {errors}"
        assert sflog.is_ok(findings)


def _run_all():
    tests = [
        test_bundle_has_every_expected_stream_and_column,
        test_bundle_seq_values_match_unwrapped_header_times_8_plus_index,
        test_bundle_baro_pressure_converted_hpa_to_pa,
        test_bundle_passes_check_bundle_with_no_errors,
    ]
    failures = 0
    for t in tests:
        try:
            t()
            print(f"  [TEST] {t.__name__:<60} PASS")
        except AssertionError as e:
            failures += 1
            print(f"  [TEST] {t.__name__:<60} FAIL: {e}")
    total = len(tests)
    print(f"\n=== Results: {total - failures}/{total} passed, {failures} failed ===")
    return failures


if __name__ == '__main__':
    sys.exit(1 if _run_all() > 0 else 0)
