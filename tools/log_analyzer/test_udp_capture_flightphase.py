#!/usr/bin/env python3
"""
test_udp_capture_flightphase.py - Host-side decode test for the 0x4C
flight-phase entry (kPktFlightPhase400, 400Hz) and the 0x4D flight-flags entry
(kPktFlightFlags, 50Hz) added for flip analysis.
フリップ解析用に追加した 0x4C 飛行フェーズエントリ（kPktFlightPhase400、400Hz）
と 0x4D 飛行フラグエントリ（kPktFlightFlags、50Hz）のホスト側デコードテスト。

Hand-assembles datagrams with the same byte layout as data_stream_wire.hpp
(WireFlightPhase400 / WireFlightFlags) and checks parse_packet() and
save_bundle() (flight_phase.csv / flight_flags.csv), plus backward
compatibility: a packet WITHOUT these entries (older firmware) still parses and
yields no such streams.
data_stream_wire.hpp（WireFlightPhase400 / WireFlightFlags）と同じバイト配置で
データグラムを手組みし、parse_packet() と save_bundle()（flight_phase.csv /
flight_flags.csv）を検査する。後方互換も確認: これらのエントリが「無い」パケット
（旧ファーム）も従来どおり読め、該当ストリームは作られない。

Usage / 使い方:
    pytest test_udp_capture_flightphase.py
"""

import struct
import sys
import tempfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_capture  # noqa: E402
from test_udp_capture_duty400 import build_unified  # noqa: E402

import sflog  # noqa: E402

N = 8  # kSamplesPerPacket

# FlipPhase / FlipResult values used below (data_types.hpp)
PHASE_SPIN = 2
RESULT_GYRO_LIMIT = 3


def _phase_entry(samples):
    """[id][size][payload] for kPktFlightPhase400. `samples` is 8
    (flight_state, flip_phase, flip_result, flip_phi_rad) tuples."""
    payload = b''.join(
        struct.pack('<3Bh', state, phase, result, round(phi * 1000))
        for state, phase, result, phi in samples
    )
    assert len(payload) == 40
    return bytes([udp_capture.PKT_FLIGHT_PHASE400, 40]) + payload


def _flags_entry(ts, buttons, flags, block_reason, arm_block):
    """[id][size][payload] for kPktFlightFlags."""
    payload = struct.pack('<I4B', ts, buttons, flags, block_reason, arm_block)
    assert len(payload) == 8
    return bytes([udp_capture.PKT_FLIGHT_FLAGS, 8]) + payload


def _tof_silence_entry(ts, velocity_up, rise_m, thrust_ratio, tof_flags, reason, mode_block):
    """[id][size][payload] for kPktToFSilence (0x4E, 19 bytes)."""
    payload = struct.pack('<I3f3B', ts, velocity_up, rise_m, thrust_ratio,
                          tof_flags, reason, mode_block)
    assert len(payload) == 19
    return bytes([udp_capture.PKT_TOF_SILENCE, 19]) + payload


def test_flight_phase_entry_decodes_8_samples_paired_with_imu_timestamps():
    samples = [(7, PHASE_SPIN, RESULT_GYRO_LIMIT, 0.5 * i) for i in range(N)]
    pkt, imu_ts = build_unified(1, entries=_phase_entry(samples), entry_count=1)

    results = udp_capture.parse_packet(pkt)
    got = [s for pid, s in results if pid == udp_capture.PKT_FLIGHT_PHASE400]
    assert len(got) == N
    for i, s in enumerate(got):
        assert s['timestamp_us'] == imu_ts[i]
        assert s['flight_state'] == 7
        assert s['flip_phase'] == PHASE_SPIN
        assert s['flip_result'] == RESULT_GYRO_LIMIT
        assert abs(s['flip_phi'] - 0.5 * i) < 1e-3


def test_flight_flags_entry_decodes_bits():
    buttons = udp_capture.PILOT_BUTTON_ARM | udp_capture.PILOT_BUTTON_FLIP
    flags = udp_capture.FLAG_FLIP_READY | udp_capture.FLAG_ATTITUDE_VERIFIED
    pkt, _ = build_unified(1, entries=_flags_entry(123456, buttons, flags, 9, 6),
                           entry_count=1)

    results = udp_capture.parse_packet(pkt)
    got = [s for pid, s in results if pid == udp_capture.PKT_FLIGHT_FLAGS]
    assert len(got) == 1
    row = got[0]
    assert row['timestamp_us'] == 123456
    assert (row['pilot_arm'], row['pilot_flip']) == (1, 1)
    assert row['flip_ready'] == 1
    assert row['flip_block_reason'] == 9
    assert row['arm_block'] == 6
    assert (row['attitude_mismatch'], row['attitude_verified']) == (0, 1)


def test_new_entries_do_not_corrupt_following_entry():
    control_entry = bytes([udp_capture.PKT_CONTROL, 20]) + \
        struct.pack('<I4f', 42, 0.5, 0.0, 0.0, 0.0)
    entries = (_phase_entry([(5, 0, 0, 0.0)] * N)
               + _flags_entry(1, 0, 0, 0, 0) + control_entry)
    pkt, _ = build_unified(1, entries=entries, entry_count=3)

    results = udp_capture.parse_packet(pkt)
    ctrl = next(s for pid, s in results if pid == udp_capture.PKT_CONTROL)
    assert ctrl['timestamp_us'] == 42


def test_save_bundle_writes_flight_phase_and_flags_streams():
    samples = [(7, PHASE_SPIN, 0, 0.1 * i) for i in range(N)]
    entries = _phase_entry(samples) + _flags_entry(2_000_000, 2, 1, 3, 0)
    pkt, imu_ts = build_unified(1, entries=entries, entry_count=2)

    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt)
    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "test.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)

        phase = log.streams['flight_phase']
        assert list(phase['seq']) == [1 * 8 + i for i in range(N)]
        assert list(phase['timestamp_us']) == imu_ts
        assert set(phase['flip_phase']) == {PHASE_SPIN}

        flags = log.streams['flight_flags']
        assert len(flags) == 1
        assert flags.iloc[0]['pilot_flip'] == 1
        assert flags.iloc[0]['flip_block_reason'] == 3


def test_old_firmware_packet_has_no_new_streams():
    """No 0x4C/0x4D entry (older firmware) -> the streams are simply absent."""
    pkt, _ = build_unified(1, entries=b'', entry_count=0)
    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt)
    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "test.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)
        assert 'flight_phase' not in log.streams
        assert 'flight_flags' not in log.streams


def test_tof_silence_entry_decodes_values_and_flag_bits():
    flags = udp_capture.TOF_FLAG_SILENT | udp_capture.TOF_FLAG_INVALID_STUCK
    entry = _tof_silence_entry(777000, -0.5, 0.25, 0.9375, flags, 2, 1)
    control_entry = bytes([udp_capture.PKT_CONTROL, 20]) + \
        struct.pack('<I4f', 42, 0.5, 0.0, 0.0, 0.0)
    pkt, _ = build_unified(1, entries=entry + control_entry, entry_count=2)

    results = udp_capture.parse_packet(pkt)
    got = [s for pid, s in results if pid == udp_capture.PKT_TOF_SILENCE]
    assert len(got) == 1
    row = got[0]
    assert row['timestamp_us'] == 777000
    assert (row['vertical_velocity_up'], row['vertical_rise_m'],
            row['descent_thrust_ratio']) == (-0.5, 0.25, 0.9375)
    assert (row['tof_silent'], row['tof_altitude_reliable'],
            row['tof_invalid_stuck'], row['tof_status_stale']) == (1, 0, 1, 0)
    assert (row['tof_reason'], row['mode_block']) == (2, 1)
    # the entry after it is not corrupted / 後ろのエントリは壊れない
    ctrl = next(s for pid, s in results if pid == udp_capture.PKT_CONTROL)
    assert ctrl['timestamp_us'] == 42


def test_save_bundle_writes_tof_silence_stream_and_old_firmware_has_none():
    entry = _tof_silence_entry(3_000_000, 0.1, 0.0, 0.0, udp_capture.TOF_FLAG_ALTITUDE_RELIABLE, 0, 0)
    pkt, _ = build_unified(1, entries=entry, entry_count=1)
    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt)
    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "test.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)
        silence = log.streams['tof_silence']
        assert len(silence) == 1
        assert silence.iloc[0]['tof_altitude_reliable'] == 1
        assert silence.iloc[0]['vertical_velocity_up'] == pytest.approx(0.1, abs=1e-6)
        # the structural check accepts the new optional stream / 構造検査は新しい任意ストリームを受け付ける
        assert not [f for f in sflog.check_bundle(log) if f.level == 'error' and 'tof_silence' in str(f)]

    old_pkt, _ = build_unified(1, entries=b'', entry_count=0)
    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(old_pkt)
    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "old.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        assert 'tof_silence' not in sflog.load(bundle_path).streams



def _baro_height_entry(ts, height_m, offset_m, age_decis, flags):
    """[id][size][payload] for kPktBaroHeight (0x52, 15 bytes)."""
    payload = struct.pack('<I2fHB', ts, height_m, offset_m, age_decis, flags)
    assert len(payload) == 15
    return bytes([udp_capture.PKT_BARO_HEIGHT, 15]) + payload


def test_baro_height_entry_decodes_and_neighbours_are_intact():
    baro = bytes([udp_capture.PKT_BARO, 12]) + struct.pack('<I2f', 5, 3.0, 1013.0)
    entry = _baro_height_entry(888000, 0.75, -0.5, 123, 0x03)
    control = bytes([udp_capture.PKT_CONTROL, 20]) + struct.pack('<I4f', 42, 0.5, 0.0, 0.0, 0.0)
    pkt, _ = build_unified(1, entries=baro + entry + control, entry_count=3)
    results = udp_capture.parse_packet(pkt)
    got = [s for pid, s in results if pid == udp_capture.PKT_BARO_HEIGHT]
    assert len(got) == 1
    assert got[0]['timestamp_us'] == 888000
    assert (got[0]['height_m'], got[0]['offset_m']) == (0.75, -0.5)
    assert (got[0]['offset_age_decis'], got[0]['flags']) == (123, 0x03)
    assert [s for pid, s in results if pid == udp_capture.PKT_BARO][0]['baro_pressure'] == 1013.0
    assert [s for pid, s in results if pid == udp_capture.PKT_CONTROL][0]['timestamp_us'] == 42


def test_unknown_entry_id_is_skipped_by_size():
    """A reader that does not know an id skips it by [id][size] (new captures stay readable by old readers)."""
    unknown = bytes([0x7E, 15]) + bytes(15)
    control = bytes([udp_capture.PKT_CONTROL, 20]) + struct.pack('<I4f', 42, 0.5, 0.0, 0.0, 0.0)
    pkt, _ = build_unified(1, entries=unknown + control, entry_count=2)
    results = udp_capture.parse_packet(pkt)
    assert [s['timestamp_us'] for pid, s in results if pid == udp_capture.PKT_CONTROL] == [42]


def test_save_bundle_writes_baro_height_stream_and_old_firmware_has_none():
    entries = (_baro_height_entry(3_000_000, 0.5, 0.1, 65535, 0x01)
               + _baro_height_entry(3_020_000, 0.6, 0.2, 25, 0x03))
    pkt, _ = build_unified(1, entries=entries, entry_count=2)
    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(pkt)
    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "test.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        log = sflog.load(bundle_path)
        rows = log.streams['baro_height']
        assert len(rows) == 2
        assert (rows.iloc[0]['valid'], rows.iloc[0]['learned']) == (1, 0)
        assert rows.iloc[0]['offset_age'] == -1.0
        assert (rows.iloc[1]['valid'], rows.iloc[1]['learned']) == (1, 1)
        assert rows.iloc[1]['offset_age'] == pytest.approx(2.5)
        assert rows.iloc[1]['height'] == pytest.approx(0.6, abs=1e-6)
        assert not [f for f in sflog.check_bundle(log) if f.level == 'error' and 'baro_height' in str(f)]

    old_pkt, _ = build_unified(1, entries=b'', entry_count=0)
    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(old_pkt)
    with tempfile.TemporaryDirectory() as td:
        bundle_path = Path(td) / "old.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(bundle_path))
        assert 'baro_height' not in sflog.load(bundle_path).streams
