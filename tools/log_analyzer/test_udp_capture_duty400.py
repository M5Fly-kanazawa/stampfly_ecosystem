#!/usr/bin/env python3
"""
test_udp_capture_duty400.py - Host-side decode test for the 0x4A motor-duty
entry (kPktDuty400), the 400Hz plant-input record for `sf sysid fit`.

Builds unified packets (0x50) by hand -- mirrors UnifiedPacketBuilder in
data_stream_wire.hpp -- WITH and WITHOUT the 0x4A duty entry, and feeds them
to udp_capture.parse_packet(), so a firmware-side wire layout change is
caught here BEFORE it reaches real hardware. Also proves the forward/
backward compatibility contract (an unknown entry id is skipped via the
[id][size] framing alone) and save_bundle()'s motor.csv/ctrl_output.csv
construction: present only when the corresponding entry was actually
received (docs/plans/flight-log-format-plan.md section 2.2 -- a stream
with no samples gets no file), with the v1 `seq` column correctly unwrapped
from the unified packet's 16-bit header sequence across a 65535->0 wrap,
and lost (skipped) unified packets counted.
0x4A モータduty エントリ（kPktDuty400、`sf sysid fit` の400Hzプラント入力
記録）のホスト側デコードテスト。

統合パケット（0x50）を手組みし（data_stream_wire.hpp の
UnifiedPacketBuilder を模す）、0x4A エントリの有無それぞれで
udp_capture.parse_packet() に投入し、ファーム側の電文レイアウト変更を
実機の前にここで検出する。前方/後方互換の契約（未知のエントリIDは
[id][size] 枠組みだけでスキップされる）と、save_bundle() の
motor.csv/ctrl_output.csv 構築（対応するエントリを実際に受信した場合のみ
存在する -- 計画書2.2節「パケットが無いストリームのファイルは作らない」）
も検証する。v1 の `seq` 列が統合パケットの16bitヘッダ sequence から
65535->0 の巻き戻りをまたいで正しく展開されること、欠落（飛び番）した
統合パケットが数えられることも確認する。

Usage:
    python3 test_udp_capture_duty400.py
    pytest test_udp_capture_duty400.py
"""

import struct
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_capture  # noqa: E402

import sflog  # noqa: E402

PKT_UNIFIED = 0x50
N = 8  # kSamplesPerPacket

FMT_IMU_ESKF = udp_capture.FMT_IMU_ESKF
FMT_POS_VEL = udp_capture.FMT_POS_VEL
FMT_RATE_REF = udp_capture.FMT_RATE_REF


def _xor_checksum(data: bytes) -> int:
    """Mirror xorChecksum() in data_stream_wire.hpp."""
    x = 0
    for b in data:
        x ^= b
    return x


def _imu_bytes(ts):
    """80B WireImuEskf; only the timestamp varies, rest is filler."""
    gyro = accel = gyro_raw = accel_raw = (0.0, 0.0, 0.0)
    quat = (1.0, 0.0, 0.0, 0.0)
    gyro_bias = accel_bias = (0, 0, 0)
    return struct.pack(FMT_IMU_ESKF, ts, *gyro, *accel, *gyro_raw, *accel_raw,
                        *quat, *gyro_bias, *accel_bias)


def _posvel_bytes(ts):
    """28B WirePosVel; all-zero pos/vel filler."""
    return struct.pack(FMT_POS_VEL, ts, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _rateref_bytes():
    """6B WireRateRef; zero rate_ref filler."""
    return struct.pack(FMT_RATE_REF, 0, 0, 0)


def build_unified(seq, base_ts_us=1_000_000, dt_us=2500, entries=b'', entry_count=0):
    """Build a raw 0x50 datagram: header + 8xImuEskf + 8xPosVel + 8xRateRef +
    entry_count + entries + checksum -- mirrors UnifiedPacketBuilder::begin()/
    addEntry()/finish() in data_stream_wire.hpp. Returns (datagram, imu_timestamps).
    """
    header = struct.pack('<BHB', PKT_UNIFIED, seq, N)
    imu_ts = [base_ts_us + i * dt_us for i in range(N)]
    body = header
    for ts in imu_ts:
        body += _imu_bytes(ts)
    for ts in imu_ts:
        body += _posvel_bytes(ts)
    for _ in range(N):
        body += _rateref_bytes()
    body += bytes([entry_count]) + entries
    return body + bytes([_xor_checksum(body)]), imu_ts


def _duty400_entry(duties):
    """[id][size][payload] for kPktDuty400. `duties` is 8 (fr,rr,rl,fl)
    tuples in duty units [0,1]."""
    payload = b''.join(
        struct.pack('<4H', *(round(v * 65535) for v in d)) for d in duties
    )
    assert len(payload) == 64
    return bytes([udp_capture.PKT_DUTY400, 64]) + payload


# =============================================================================
# parse_packet() decode tests
# =============================================================================

def test_duty400_entry_decodes_8_samples_paired_with_imu_timestamps():
    """8 sub-samples come back, each timestamped with the SAME-INDEX IMU
    sample's timestamp (like PKT_RATE_REF) -- the pairing `sf sysid fit`
    relies on to build a 400Hz-aligned CSV row."""
    duties = [(0.1 + 0.01 * i, 0.2, 0.3, 0.4) for i in range(N)]
    pkt, imu_ts = build_unified(1, entries=_duty400_entry(duties), entry_count=1)

    results = udp_capture.parse_packet(pkt)
    duty_samples = [s for pid, s in results if pid == udp_capture.PKT_DUTY400]
    assert len(duty_samples) == N

    for i, s in enumerate(duty_samples):
        assert s['timestamp_us'] == imu_ts[i]
        assert abs(s['duty_FR'] - duties[i][0]) < 1e-3
        assert abs(s['duty_RR'] - duties[i][1]) < 1e-3
        assert abs(s['duty_RL'] - duties[i][2]) < 1e-3
        assert abs(s['duty_FL'] - duties[i][3]) < 1e-3


def test_packet_without_duty400_entry_parses_fine_no_duty_samples():
    """A unified packet with NO 0x4A entry (old firmware) must still parse
    cleanly, with zero PKT_DUTY400 samples -- the trigger for save_bundle()
    to omit motor.csv entirely (see test_no_motor_stream_without_duty400_entries)."""
    pkt, _ = build_unified(2, entries=b'', entry_count=0)
    results = udp_capture.parse_packet(pkt)
    assert results  # IMU/PosVel/RateRef fixed blocks still present
    duty_samples = [s for pid, s in results if pid == udp_capture.PKT_DUTY400]
    assert duty_samples == []


def test_duty400_entry_does_not_corrupt_a_following_entry():
    """duty400, added FIRST by DataStream::appendEntries(), must not disturb
    offset tracking for entries appended after it."""
    control_entry = bytes([udp_capture.PKT_CONTROL, 20]) + \
        struct.pack('<I4f', 42, 0.5, 0.0, 0.0, 0.0)
    entries = _duty400_entry([(0.5, 0.5, 0.5, 0.5)] * N) + control_entry
    pkt, _ = build_unified(3, entries=entries, entry_count=2)

    results = udp_capture.parse_packet(pkt)
    ids = [pid for pid, _ in results]
    assert udp_capture.PKT_DUTY400 in ids
    assert udp_capture.PKT_CONTROL in ids
    ctrl = next(s for pid, s in results if pid == udp_capture.PKT_CONTROL)
    assert ctrl['timestamp_us'] == 42
    assert abs(ctrl['ctrl_throttle'] - 0.5) < 1e-6


def test_unknown_entry_id_is_skipped_without_corrupting_later_entries():
    """Compatibility note (a): a future/unknown entry id (not the 0x4A this
    parser knows, and not in SAMPLE_INFO) must be skipped by the [id][size]
    framing alone, without breaking a REAL entry that follows -- this is
    exactly the mechanism an OLD udp_capture.py relies on to safely ignore
    a NEW firmware's 0x4A duty entry."""
    fake_future_entry = bytes([0xEE, 4]) + b'\x01\x02\x03\x04'
    control_entry = bytes([udp_capture.PKT_CONTROL, 20]) + \
        struct.pack('<I4f', 99, 0.25, 0.0, 0.0, 0.0)
    entries = fake_future_entry + control_entry
    pkt, _ = build_unified(4, entries=entries, entry_count=2)

    results = udp_capture.parse_packet(pkt)
    ctrl = next(s for pid, s in results if pid == udp_capture.PKT_CONTROL)
    assert ctrl['timestamp_us'] == 99
    assert abs(ctrl['ctrl_throttle'] - 0.25) < 1e-6


# =============================================================================
# save_bundle() tests: motor.csv (from duty400) -- presence, values, seq
# =============================================================================

def test_save_bundle_motor_csv_from_duty400_entries():
    """motor.csv must hold exactly the 400Hz duty400 values (not a 50Hz
    CtrlRef of any kind -- v1 keeps them as two independent streams, see
    plan section 2.2), with a `seq` column derived from the unified
    packet's header sequence (here 1) times 8 plus the in-packet index."""
    duties = [(0.1 + 0.01 * i, 0.2, 0.3, 0.4) for i in range(N)]
    pkt, imu_ts = build_unified(1, entries=_duty400_entry(duties), entry_count=1)

    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt)

    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "test.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)

        motor = log.streams['motor']
        assert len(motor) == N
        assert list(motor['seq']) == [1 * 8 + i for i in range(N)]
        for i in range(N):
            row = motor.iloc[i]
            assert row['timestamp_us'] == imu_ts[i]
            assert abs(row['duty_FR'] - duties[i][0]) < 1e-3
            assert abs(row['duty_RR'] - duties[i][1]) < 1e-3
            assert abs(row['duty_RL'] - duties[i][2]) < 1e-3
            assert abs(row['duty_FL'] - duties[i][3]) < 1e-3


def test_no_motor_stream_without_duty400_entries():
    """No duty400 entry ever received (old firmware) -> motor.csv is simply
    ABSENT from the bundle, never an empty/zero-filled file (plan section
    2.2: "パケットが無いストリームのファイルは作らない")."""
    pkt, _ = build_unified(1, entries=b'', entry_count=0)

    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt)

    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "test.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)
        assert 'motor' not in log.streams


# =============================================================================
# `seq` unwrapping and lost-packet-count tests (UDPTelemetryCapture.
# _unwrap_unified_seq() / _process_datagram()'s sequence-gap detection)
# `seq` 展開とパケット欠落数のテスト（UDPTelemetryCapture.
# _unwrap_unified_seq() / _process_datagram() のシーケンスギャップ検出）
# =============================================================================

def test_seq_unwraps_across_65535_to_0_wrap():
    """The unified packet's 16-bit header sequence wraps 65535 -> 0, but
    the v1 `seq` column must keep climbing (never reset to 0) -- proven by
    feeding a packet at raw seq=65535 then one at raw seq=0 and checking
    the second packet's `seq` values are 8 higher than the first's, exactly
    as if there had been no wrap at all."""
    duties = [(0.5, 0.5, 0.5, 0.5)] * N
    pkt_a, _ = build_unified(65535, base_ts_us=1_000_000,
                              entries=_duty400_entry(duties), entry_count=1)
    pkt_b, _ = build_unified(0, base_ts_us=1_020_000,
                              entries=_duty400_entry(duties), entry_count=1)

    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt_a)
    cap._process_datagram(pkt_b)

    seqs = [s['seq'] for s in cap.samples[udp_capture.PKT_DUTY400]]
    assert seqs[:N] == [65535 * 8 + i for i in range(N)]
    assert seqs[N:] == [65536 * 8 + i for i in range(N)]  # continues climbing, no reset


def test_lost_packet_count_from_seq_gap():
    """A skipped unified-packet sequence number (1 -> 3, i.e. 2 was never
    received) must be counted as 1 lost packet in seq_gaps -- the same
    number save_bundle() reports as meta.json's `capture.packets_lost`
    (see test_udp_capture_bundle.py)."""
    pkt_a, _ = build_unified(1, base_ts_us=1_000_000)
    pkt_b, _ = build_unified(3, base_ts_us=1_020_000)  # seq=2 skipped

    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt_a)
    cap._process_datagram(pkt_b)

    assert cap.seq_gaps[udp_capture.PKT_UNIFIED] == 1


def _ctrl_output400_entry(samples):
    """[id][size][payload] for kPktCtrlOutput400. `samples` is 8
    (thrust, torque_roll, torque_pitch, torque_yaw) tuples."""
    payload = b''.join(struct.pack('<4f', *s) for s in samples)
    assert len(payload) == 128
    return bytes([udp_capture.PKT_CTRL_OUTPUT400, 128]) + payload


# =============================================================================
# kPktCtrlOutput400 (0x4B) -- the PRE-MIXER commanded thrust+torque, the
# mixer-agnostic plant input for `sf sysid fit`/`rate-fit` (see
# plant_fit.py's module docstring and docs/events/sci_tutorial_2026 rate-
# sysid design memo, 2026-09-09). Mirrors the duty400 tests above.
# kPktCtrlOutput400（0x4B）-- ミキサー手前の指令推力+トルク、
# `sf sysid fit`/`rate-fit` のミキサー非依存なプラント入力。上の duty400
# テスト群を模す。
# =============================================================================

def test_ctrl_output400_entry_decodes_8_samples_paired_with_imu_timestamps():
    samples = [(0.3 + 0.01 * i, 0.01 * i, -0.02, 0.03) for i in range(N)]
    pkt, imu_ts = build_unified(1, entries=_ctrl_output400_entry(samples), entry_count=1)

    results = udp_capture.parse_packet(pkt)
    co_samples = [s for pid, s in results if pid == udp_capture.PKT_CTRL_OUTPUT400]
    assert len(co_samples) == N

    for i, s in enumerate(co_samples):
        assert s['timestamp_us'] == imu_ts[i]
        assert abs(s['ctrl_output_thrust'] - samples[i][0]) < 1e-5
        assert abs(s['ctrl_output_torque_roll'] - samples[i][1]) < 1e-5
        assert abs(s['ctrl_output_torque_pitch'] - samples[i][2]) < 1e-5
        assert abs(s['ctrl_output_torque_yaw'] - samples[i][3]) < 1e-5


def test_packet_without_ctrl_output400_entry_parses_fine_no_samples():
    """Old/other firmware without the 0x4B entry must still parse cleanly --
    the trigger for save_bundle() to omit ctrl_output.csv entirely (see
    test_no_ctrl_output_stream_without_entries)."""
    pkt, _ = build_unified(2, entries=b'', entry_count=0)
    results = udp_capture.parse_packet(pkt)
    co_samples = [s for pid, s in results if pid == udp_capture.PKT_CTRL_OUTPUT400]
    assert co_samples == []


def test_save_bundle_ctrl_output_csv_from_entries():
    """ctrl_output.csv must hold the PRE-MIXER thrust/torque values with a
    `seq` column, same shape as motor.csv's duty400 test above."""
    samples = [(0.3 + 0.01 * i, 0.01 * i, -0.02, 0.03) for i in range(N)]
    pkt, imu_ts = build_unified(1, entries=_ctrl_output400_entry(samples), entry_count=1)

    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt)

    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "test.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)

        ctrl_output = log.streams['ctrl_output']
        assert len(ctrl_output) == N
        assert list(ctrl_output['seq']) == [1 * 8 + i for i in range(N)]
        for i in range(N):
            row = ctrl_output.iloc[i]
            assert row['timestamp_us'] == imu_ts[i]
            assert abs(row['thrust'] - samples[i][0]) < 1e-5
            assert abs(row['torque_roll'] - samples[i][1]) < 1e-5
            assert abs(row['torque_pitch'] - samples[i][2]) < 1e-5
            assert abs(row['torque_yaw'] - samples[i][3]) < 1e-5


def test_no_ctrl_output_stream_without_entries():
    """No control_output entry ever received -> ctrl_output.csv is simply
    ABSENT from the bundle (plan section 2.2), not an empty/zero-filled
    file -- unlike the old save_stream_csv()'s '' placeholder columns."""
    pkt, _ = build_unified(1, entries=b'', entry_count=0)

    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt)

    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "test.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)
        assert 'ctrl_output' not in log.streams


def _run_all():
    tests = [
        test_duty400_entry_decodes_8_samples_paired_with_imu_timestamps,
        test_packet_without_duty400_entry_parses_fine_no_duty_samples,
        test_duty400_entry_does_not_corrupt_a_following_entry,
        test_unknown_entry_id_is_skipped_without_corrupting_later_entries,
        test_save_bundle_motor_csv_from_duty400_entries,
        test_no_motor_stream_without_duty400_entries,
        test_seq_unwraps_across_65535_to_0_wrap,
        test_lost_packet_count_from_seq_gap,
        test_ctrl_output400_entry_decodes_8_samples_paired_with_imu_timestamps,
        test_packet_without_ctrl_output400_entry_parses_fine_no_samples,
        test_save_bundle_ctrl_output_csv_from_entries,
        test_no_ctrl_output_stream_without_entries,
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
