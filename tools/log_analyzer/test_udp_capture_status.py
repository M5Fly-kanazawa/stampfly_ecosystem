#!/usr/bin/env python3
"""
test_udp_capture_status.py - Host-side decode test for the 0x4F status packet

Builds legacy(17B) / v2(53B, +PID gains) / v3(57B, +current_ma) status
datagrams by hand (mirrors buildStatusPacket() in data_stream_wire.hpp) and
feeds them straight to udp_capture.parse_packet(), so a firmware-side wire
layout change is caught here BEFORE it reaches real hardware. Also proves
save_bundle()'s status.csv (docs/plans/flight-log-format-plan.md section
2.2) carries every field a packet actually sent, with an optional field
(current_ma/PID gains) absent on a legacy packet left empty rather than
invented.
0x4F ステータスパケットのホスト側デコードテスト。

legacy(17B)/v2(53B, +PIDゲイン)/v3(57B, +current_ma) のステータス電文を手組みし
（data_stream_wire.hpp の buildStatusPacket() を模す）、udp_capture.parse_packet()
に直接投入する。ファーム側の電文レイアウト変更を実機の前にここで検出する。
save_bundle() の status.csv（計画書2.2節）が、パケットが実際に送った
フィールドを全て運び、旧パケットに無い任意フィールド（current_ma/PID
ゲイン）は捏造せず空欄になることも検証する。

Usage:
    python3 test_udp_capture_status.py
    pytest test_udp_capture_status.py
"""

import struct
import sys
import tempfile
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_capture  # noqa: E402

import sflog  # noqa: E402

PKT_STATUS = udp_capture.PKT_STATUS

# Field layout shared by all versions — mirrors WireStatusPayload's fixed head.
# 全バージョン共通の先頭フィールド — WireStatusPayload の固定部を写す。
_HEAD_FMT = '<IfBBBB'   # uptime_ms, voltage, flight_state, sensor_health, eskf_status, reset_reason
_GAINS_FMT = '<9f'
_CURRENT_FMT = '<f'


def _xor_checksum(data: bytes) -> int:
    """Mirror xorChecksum() in data_stream_wire.hpp."""
    x = 0
    for b in data:
        x ^= b
    return x


def build_status_packet(uptime_ms, voltage, flight_state, sensor_health,
                         eskf_status, reset_reason, gains=None, current_ma=None,
                         sequence=1):
    """Build a raw 0x4F datagram; mirrors buildStatusPacket()'s byte layout.

    `gains=None` reproduces the legacy 17B packet (no PID gains, no current);
    `current_ma=None` (with gains set) reproduces the 53B v2 packet; both set
    reproduces the 57B v3 packet (current_ma appended AFTER pid_gains).
    生の 0x4F 電文を組み立てる。buildStatusPacket() のバイトレイアウトを写す。
    """
    header = struct.pack('<BHB', PKT_STATUS, sequence, 1)
    payload = struct.pack(_HEAD_FMT, uptime_ms, voltage, flight_state,
                           sensor_health, eskf_status, reset_reason)
    if gains is not None:
        payload += struct.pack(_GAINS_FMT, *gains)
    if current_ma is not None:
        payload += struct.pack(_CURRENT_FMT, current_ma)
    body = header + payload
    return body + bytes([_xor_checksum(body)])


def test_legacy_17b_decodes_without_current():
    """Old 17B firmware images (no gains, no current) must still decode."""
    pkt = build_status_packet(5000, 4.05, 2, 0x3F, 1, 1)
    assert len(pkt) == 17
    results = udp_capture.parse_packet(pkt)
    assert len(results) == 1
    pkt_id, sample = results[0]
    assert pkt_id == PKT_STATUS
    assert sample['uptime_ms'] == 5000
    assert abs(sample['voltage'] - 4.05) < 1e-4
    assert 'pid_roll_kp' not in sample
    assert 'current_ma' not in sample


def test_v2_53b_decodes_gains_without_current():
    """53B firmware (PID gains, no current) must still decode — the field this
    change appends after, not inserts before."""
    gains = [float(i) * 0.01 for i in range(9)]
    pkt = build_status_packet(6000, 3.95, 1, 0x3F, 1, 0, gains=gains)
    assert len(pkt) == 53
    results = udp_capture.parse_packet(pkt)
    pkt_id, sample = results[0]
    assert pkt_id == PKT_STATUS
    assert abs(sample['pid_roll_kp'] - gains[0]) < 1e-6
    assert abs(sample['pid_yaw_td'] - gains[8]) < 1e-6
    assert 'current_ma' not in sample


def test_v3_57b_decodes_gains_and_current():
    """New 57B firmware (PID gains + battery current) decodes both blocks."""
    gains = [float(i) * 0.01 for i in range(9)]
    pkt = build_status_packet(7000, 4.10, 3, 0x3F, 1, 0,
                               gains=gains, current_ma=612.5)
    assert len(pkt) == 57
    results = udp_capture.parse_packet(pkt)
    pkt_id, sample = results[0]
    assert pkt_id == PKT_STATUS
    assert abs(sample['pid_roll_kp'] - gains[0]) < 1e-6
    assert abs(sample['current_ma'] - 612.5) < 1e-3


def test_bad_checksum_is_rejected():
    """A corrupted datagram must be dropped, not mis-decoded."""
    pkt = bytearray(build_status_packet(1000, 4.0, 0, 0, 0, 0, current_ma=100.0,
                                         gains=[0.0] * 9))
    pkt[-1] ^= 0xFF   # flip the checksum byte / チェックサムを破壊
    assert udp_capture.parse_packet(bytes(pkt)) == []


# =============================================================================
# save_bundle() tests: status.csv fields (docs/plans/flight-log-format-plan.md
# section 2.2 -- status.csv, 1Hz). Feeds a real wire-encoded status packet
# through _process_datagram() (not a hand-seeded sample dict), so a bundle
# writer bug and a wire-decode bug are caught the same way as the
# parse_packet() tests above.
# save_bundle() のテスト: status.csv の各列（計画書2.2節 -- status.csv、
# 1Hz）。実際に電文エンコードした status パケットを _process_datagram() へ
# 投入する（手組みのサンプル dict ではない）ため、一式書き出し側のバグと
# 電文デコード側のバグの両方を、上の parse_packet() テストと同じ経路で
# 検出できる。
# =============================================================================

def test_save_bundle_status_csv_fields():
    """status.csv must carry every v3 (57B) field this packet actually
    sent: uptime_ms/voltage/current_ma/flight_state/sensor_health/
    eskf_status/reset_reason and all 9 PID gains -- none forward-filled
    or merged onto another rate (status.csv is its own 1Hz stream, plan
    section 2.2)."""
    gains = [float(i) * 0.01 for i in range(9)]
    pkt = build_status_packet(7000, 4.10, 3, 0x3F, 1, 2, gains=gains, current_ma=612.5)

    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt)

    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "test.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)

        status = log.streams['status']
        assert len(status) == 1
        row = status.iloc[0]
        assert row['timestamp_us'] == 7000 * 1000  # uptime_ms * 1000, see parse_packet()
        assert row['uptime_ms'] == 7000
        assert abs(row['voltage'] - 4.10) < 1e-4
        assert abs(row['current_ma'] - 612.5) < 1e-3
        assert row['flight_state'] == 3
        assert row['sensor_health'] == 0x3F
        assert row['eskf_status'] == 1
        assert row['reset_reason'] == 2
        assert abs(row['pid_roll_kp'] - gains[0]) < 1e-6
        assert abs(row['pid_yaw_td'] - gains[8]) < 1e-6


def test_save_bundle_status_csv_omits_optional_fields_when_legacy_packet():
    """A legacy 17B status packet (no PID gains, no current) must still
    produce a status.csv row -- with current_ma/pid_* left empty (NaN), not
    invented (plan section 2.2 "無いものは無い"), while the fields every
    version sends (sensor_health/reset_reason included) are still present."""
    pkt = build_status_packet(5000, 4.05, 2, 0x3F, 1, 1)  # no gains, no current

    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt)

    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "test.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)

        status = log.streams['status']
        assert len(status) == 1
        row = status.iloc[0]
        assert row['uptime_ms'] == 5000
        assert row['sensor_health'] == 0x3F
        assert row['reset_reason'] == 1
        assert pd.isna(row['current_ma'])
        assert pd.isna(row['pid_roll_kp'])


def _run_all():
    tests = [
        test_legacy_17b_decodes_without_current,
        test_v2_53b_decodes_gains_without_current,
        test_v3_57b_decodes_gains_and_current,
        test_bad_checksum_is_rejected,
        test_save_bundle_status_csv_fields,
        test_save_bundle_status_csv_omits_optional_fields_when_legacy_packet,
    ]
    failures = 0
    for t in tests:
        try:
            t()
            print(f"  [TEST] {t.__name__:<45} PASS")
        except AssertionError as e:
            failures += 1
            print(f"  [TEST] {t.__name__:<45} FAIL: {e}")
    total = len(tests)
    print(f"\n=== Results: {total - failures}/{total} passed, {failures} failed ===")
    return failures


if __name__ == '__main__':
    sys.exit(1 if _run_all() > 0 else 0)
