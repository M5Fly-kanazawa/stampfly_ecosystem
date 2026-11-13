#!/usr/bin/env python3
"""
test_udp_capture_text_log.py - Host-side tests for the status packet v4 (0x4F, 224 B) and the text-log
packet (0x51): decoding, loss, out-of-order delivery, the bundle written by save_bundle(), a loopback capture
over 127.0.0.1, and the golden wire vectors shared with the firmware's host test.

The two packets are built here independently of the firmware (a mirror of data_stream_wire.hpp's
buildStatusPacketV4() / TextLogPacketBuilder), and the firmware's host test (firmware/vehicle/test/test_main.cpp,
wire_golden_vectors_match_the_python_side) builds the same packets with the real C++ builders and compares them
byte for byte with the files under firmware/vehicle/test/golden/. A drift on either side fails one of the two tests.
ステータスパケット v4（0x4F、224 B）とテキストログのパケット（0x51）のホスト側の試験: デコード、欠落、順序の
入れ替わり、save_bundle() が書く一式、127.0.0.1 上のループバック取得、ファームのホスト試験と共有する電文の
基準ベクトル。

2 つのパケットはファームとは独立にここで組み立て（data_stream_wire.hpp の buildStatusPacketV4() /
TextLogPacketBuilder の写し）、ファームのホスト試験（firmware/vehicle/test/test_main.cpp の
wire_golden_vectors_match_the_python_side）は本物の C++ ビルダーで同じパケットを組み、
firmware/vehicle/test/golden/ のファイルとバイト単位で比較する。どちらか片側がずれれば、2 つの試験の一方が落ちる。

Usage:
    pytest test_udp_capture_text_log.py
"""

import socket
import struct
import sys
import tempfile
import threading
import time
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_capture  # noqa: E402

import sflog  # noqa: E402

GOLDEN_DIR = Path(__file__).resolve().parents[2] / "firmware" / "vehicle" / "test" / "golden"


def _xor(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def _finish(body: bytes) -> bytes:
    return body + bytes([_xor(body)])


# ---- builders (mirror of data_stream_wire.hpp) / ビルダー（data_stream_wire.hpp の写し） -------------------

def golden_status_v4_values():
    """The values of the golden status v4 packet (the C++ test sets the same ones)."""
    return {
        'sequence': 42, 'uptime_ms': 123456, 'voltage': 3.75, 'flight_state': 5, 'sensor_health': 0x7F,
        'eskf_status': 1, 'reset_reason': 3, 'gains': [0.5 * (i + 1) for i in range(9)], 'current_ma': 1500.5,
        'bottom': (0, 2.5, 0.125, 1.75), 'front': (255, 0.0, 0.0, 0.0),
        'imu_loop': [100 + i for i in range(11)], 'control_loop': [200 + i for i in range(11)],
        'nonfinite': [400, 3, 0, 0, 3, 3, 3, 3, 1234567], 'first_stage': 2,
        'link': [11, 12, 13, 14],
    }


def build_status_v4(values=None) -> bytes:
    v = values or golden_status_v4_values()
    body = struct.pack('<BHB', udp_capture.PKT_STATUS, v['sequence'], 1)
    body += struct.pack('<IfBBBB', v['uptime_ms'], v['voltage'], v['flight_state'], v['sensor_health'],
                        v['eskf_status'], v['reset_reason'])
    body += struct.pack('<9f', *v['gains']) + struct.pack('<f', v['current_ma'])
    body += struct.pack('<B3f', *v['bottom']) + struct.pack('<B3f', *v['front'])
    body += struct.pack('<11I', *v['imu_loop']) + struct.pack('<11I', *v['control_loop'])
    body += struct.pack('<9IB', *v['nonfinite'], v['first_stage']) + struct.pack('<4I', *v['link'])
    return _finish(body)


GOLDEN_TEXT_LINES = [
    (1000000, 0, 0, "I (1000) Test: hello\n"),
    (1000500, 1, 7, "W (1000) Test: cut line\n"),
    (4294967295, 0, 0, "E (4294967) Test: non-ASCII é\n"),
]


def build_text_packet(sequence: int, lines) -> bytes:
    """lines: [(timestamp_us, flags, dropped_before, text), ...]"""
    body = struct.pack('<BHB', udp_capture.PKT_TEXT_LOG, sequence, len(lines))
    for timestamp_us, flags, dropped_before, text in lines:
        raw = text.encode('utf-8')
        body += struct.pack('<IBBH', timestamp_us, flags, dropped_before, len(raw)) + raw
    return _finish(body)


def _golden(name: str) -> bytes:
    return bytes.fromhex((GOLDEN_DIR / name).read_text().strip())


# ---- golden vectors shared with the firmware / ファームと共有する基準ベクトル --------------------------

def test_golden_status_v4_matches_the_builder_and_decodes():
    packet = build_status_v4()
    assert packet == _golden('status_v4.golden'), "Python mirror and golden file drifted"
    assert len(packet) == udp_capture.STATUS_V4_LENGTH == 224
    (pkt_id, sample), = udp_capture.parse_packet(packet)
    assert pkt_id == udp_capture.PKT_STATUS
    assert sample['uptime_ms'] == 123456 and sample['flight_state'] == 5 and sample['reset_reason'] == 3
    assert abs(sample['current_ma'] - 1500.5) < 1e-4 and abs(sample['pid_yaw_td'] - 4.5) < 1e-6
    assert sample['bottom_range_status'] == 0 and sample['bottom_signal_rate'] == 2.5
    assert sample['bottom_ambient_rate'] == 0.125 and sample['bottom_sigma_mm'] == 1.75
    assert sample['front_range_status'] == 255 and sample['front_signal_rate'] == 0.0
    assert [sample['imu_' + f] for f in udp_capture.LOOP_TIMING_FIELDS] == [100 + i for i in range(11)]
    assert sample['control_cycles'] == 200 and sample['control_armed_over_budget_count'] == 210
    assert sample['checked_cycles'] == 400 and sample['detected_cycles'] == 3
    assert sample['control_reference'] == 3 and sample['motor_command'] == 3 and sample['first_us'] == 1234567
    assert sample['first_stage'] == 2
    assert (sample['log_stream_overflow'], sample['send_drops'], sample['entry_drops'],
            sample['text_dropped_lines']) == (11, 12, 13, 14)


def test_golden_text_packet_matches_the_builder_and_decodes():
    packet = build_text_packet(513, GOLDEN_TEXT_LINES)
    assert packet == _golden('text_log.golden'), "Python mirror and golden file drifted"
    results = udp_capture.parse_packet(packet)
    assert [pid for pid, _ in results] == [udp_capture.PKT_TEXT_LOG] * 3
    assert [s['_idx'] for _, s in results] == [0, 1, 2]
    assert results[0][1]['text'] == "I (1000) Test: hello\n" and results[0][1]['timestamp_us'] == 1000000
    assert results[1][1]['truncated'] == 1 and results[1][1]['dropped_before'] == 7
    assert results[2][1]['timestamp_us'] == 4294967295 and results[2][1]['text'].endswith("é\n")


# ---- status decoding across versions / バージョンをまたぐステータスのデコード --------------------------

def test_status_v4_prefix_is_identical_to_v3():
    """An older reader that only knows 57 B reads the same first 56 bytes (v3 prefix layout kept)."""
    v4 = build_status_v4()
    v3_body = v4[:56]
    v3 = v3_body + bytes([_xor(v3_body)])
    assert len(v3) == 57
    (pkt_id, old_sample), = udp_capture.parse_packet(v3)
    (pkt_id, new_sample), = udp_capture.parse_packet(v4)
    assert 'imu_cycles' not in old_sample and 'imu_cycles' in new_sample
    for key, value in old_sample.items():
        assert new_sample[key] == value, key


def test_status_v4_with_a_wrong_length_is_rejected():
    packet = build_status_v4()
    body = packet[:-1] + b'\x00\x00'
    assert udp_capture.parse_packet(body + bytes([_xor(body)])) == []   # 225 B: not a known version


def test_save_bundle_status_v4_streams_carry_every_field():
    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(build_status_v4())
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "v4.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(path))
        log = sflog.load(path)
        assert {'status', 'tof_diagnostics', 'loop_timing', 'nonfinite', 'link_health'} <= set(log.streams)
        tof = log.streams['tof_diagnostics'].iloc[0]
        assert tof['timestamp_us'] == 123456 * 1000 and tof['bottom_signal_rate'] == 2.5
        assert tof['front_range_status'] == 255
        loop = log.streams['loop_timing'].iloc[0]
        assert loop['imu_cycles'] == 100 and loop['control_late_count'] == 202 and loop['control_armed_busy_max_us'] == 209
        nonfinite = log.streams['nonfinite'].iloc[0]
        assert nonfinite['checked_cycles'] == 400 and nonfinite['first_us'] == 1234567 and nonfinite['first_stage'] == 2
        link = log.streams['link_health'].iloc[0]
        assert link['text_dropped_lines'] == 14 and link['log_stream_overflow'] == 11
        assert _no_errors(sflog.check_bundle(path))


def test_save_bundle_from_a_v3_vehicle_has_no_v4_streams():
    """A bundle from a vehicle that sends status up to v3 simply lacks the new streams (absent, not invented)."""
    v4 = build_status_v4()
    body = v4[:56]
    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(body + bytes([_xor(body)]))
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "v3.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(path))
        log = sflog.load(path)
        assert set(log.streams) == {'status'}
        assert _no_errors(sflog.check_bundle(path))


# ---- text log: decoding, loss, order / テキストログ: デコード・欠落・順序 -----------------------------

def _no_errors(findings):
    """No error except the missing IMU stream: these test captures carry no 400 Hz unified packet on purpose.
    IMU ストリームの欠落以外にエラーがないこと: この試験の取得は 400 Hz の統合パケットを意図して持たない。"""
    return [f for f in findings if f.level == "error" and "required stream 'imu'" not in f.message] == []


def _capture_with(packets):
    cap = udp_capture.UDPTelemetryCapture()
    for packet in packets:
        cap._process_datagram(packet)
    return cap


def _saved_text(cap):
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "text.sflog.zip"
        cap.start_time, cap.end_time = 0, 1.0
        cap.save_bundle(str(path))
        log = sflog.load(path)
        findings = sflog.check_bundle(path)
        return log, findings


def _line(n, dropped_before=0):
    return (1000 * n, 0, dropped_before, f"I ({n}) T: line {n}\n")


def test_text_packet_with_a_bad_structure_is_rejected_whole():
    packet = build_text_packet(0, [_line(1), _line(2)])
    # count says 3 records but only 2 are there / 件数は 3 だがレコードは 2 つ
    body = bytearray(packet[:-1])
    body[3] = 3
    assert udp_capture.parse_packet(bytes(body) + bytes([_xor(bytes(body))])) == []
    # a record whose length runs past the end / 長さが末尾を越えるレコード
    body = bytearray(packet[:-1])
    struct.pack_into('<H', body, 4 + 6, 5000)
    assert udp_capture.parse_packet(bytes(body) + bytes([_xor(bytes(body))])) == []
    # a flipped bit is caught by the checksum / 1 ビット反転はチェックサムで検出
    broken = bytearray(packet)
    broken[10] ^= 0x01
    assert udp_capture.parse_packet(bytes(broken)) == []


def test_text_lines_are_saved_in_vehicle_order_with_all_fields():
    cap = _capture_with([build_text_packet(0, [_line(1), _line(2)]), build_text_packet(1, [_line(3)])])
    log, findings = _saved_text(cap)
    text = log.streams['text_log']
    assert list(text.columns) == ['timestamp_us', 'truncated', 'dropped_before', 'text']
    assert list(text['text']) == ["I (1) T: line 1", "I (2) T: line 2", "I (3) T: line 3"]   # trailing newline dropped
    assert list(text['timestamp_us']) == [1000, 2000, 3000]
    assert _no_errors(findings), findings


def test_out_of_order_and_duplicate_packets_are_restored_and_counted():
    packets = [build_text_packet(n, [_line(2 * n + 1), _line(2 * n + 2)]) for n in range(4)]
    cap = _capture_with([packets[0], packets[2], packets[1], packets[3], packets[2]])   # 1 late, 2 duplicated
    log, findings = _saved_text(cap)
    assert list(log.streams['text_log']['text']) == [f"I ({n}) T: line {n}" for n in range(1, 9)]
    assert cap.text_duplicate_packets == 1
    assert cap.text_packets_lost() == 0
    assert _no_errors(findings), findings


def test_a_lost_packet_is_counted_and_the_carried_loss_is_kept():
    # packet 1 never arrives; the vehicle dropped 3 lines before line 5 (the first line after its loss).
    # パケット 1 は届かない。機体は行 5（欠落の後の最初の行）の直前に 3 行を落とした。
    packets = [build_text_packet(0, [_line(1), _line(2)]), build_text_packet(2, [_line(5, dropped_before=3)])]
    status_values = golden_status_v4_values()
    status_values['link'] = [0, 0, 0, 3]
    cap = _capture_with(packets + [build_status_v4(status_values)])
    assert cap.text_packets_lost() == 1
    assert cap.text_lines_dropped_by_vehicle() == 3          # the exact total of the status v4 / ステータス v4 の正確な合計
    log, _ = _saved_text(cap)
    text = log.streams['text_log']
    assert list(text['dropped_before']) == [0, 0, 3]          # the loss sits where the lines went missing / 欠落は行が抜けた位置にある
    capture_meta = log.meta['capture']
    assert capture_meta['text_packets_lost'] == 1 and capture_meta['text_lines_dropped_by_vehicle'] == 3


def test_sequence_wrap_keeps_the_order():
    start = 65534
    packets = [build_text_packet((start + n) & 0xFFFF, [_line(n + 1)]) for n in range(4)]   # 65534, 65535, 0, 1
    cap = _capture_with([packets[0], packets[1], packets[3], packets[2]])   # the last two swapped / 最後の 2 つが入れ替わり
    log, _ = _saved_text(cap)
    assert list(log.streams['text_log']['text']) == [f"I ({n}) T: line {n}" for n in range(1, 5)]


def test_truncated_flag_and_awkward_text_survive_the_csv():
    lines = [(10, 1, 0, "I (1) T: " + "x" * 370 + "\n"),
             (20, 0, 0, "NA\n"), (30, 0, 0, "\n"), (40, 0, 0, 'W (4) T: "quoted", comma, 日本語\n')]
    log, findings = _saved_text(_capture_with([build_text_packet(0, lines)]))
    text = log.streams['text_log']
    assert list(text['truncated']) == [1, 0, 0, 0]
    assert list(text['text']) == ["I (1) T: " + "x" * 370, "NA", "", 'W (4) T: "quoted", comma, 日本語']
    assert _no_errors(findings), findings


def test_aligned_tables_leave_the_text_log_out():
    packets = [build_text_packet(0, [_line(1)]), build_status_v4()]
    log, _ = _saved_text(_capture_with(packets))
    aligned = sflog.aligned(log, base='status')
    assert 'text' not in aligned.columns
    assert 'bottom_signal_rate' in aligned.columns   # the 1 Hz diagnostics do take part / 1 Hz の診断値は参加する


def test_bundle_without_v4_and_text_still_checks():
    cap = udp_capture.UDPTelemetryCapture()
    cap._process_datagram(build_status_v4()[:56] + bytes([_xor(build_status_v4()[:56])]))
    log, findings = _saved_text(cap)
    assert 'text_log' not in log.streams and _no_errors(findings)


# ---- loopback capture: capture() -> save_bundle() -> reader / ループバック取得 ----------------------------

def test_loopback_capture_to_bundle_to_reader():
    """A fake vehicle on 127.0.0.1 answers START with a status v4, text packets (one lost, one late) and a
    second status; the real capture() loop receives them, save_bundle() writes the bundle, sflog reads it."""
    vehicle = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vehicle.bind(('127.0.0.1', 0))
    vehicle.settimeout(5.0)
    port = vehicle.getsockname()[1]

    def fake_vehicle():
        try:
            data, client = vehicle.recvfrom(16)
        except socket.timeout:
            return
        assert data[0] == udp_capture.CMD_START_LOG
        sends = [build_status_v4(),
                 build_text_packet(0, [_line(1), _line(2)]),
                 build_text_packet(2, [_line(5, dropped_before=2)]),   # packet 1 lost / パケット 1 は欠落
                 build_text_packet(3, [_line(6)]),
                 build_text_packet(4, [_line(8)]),
                 build_text_packet(3, [_line(6)])]                    # duplicate / 重複
        for packet in sends:
            vehicle.sendto(packet, client)
            time.sleep(0.01)

    thread = threading.Thread(target=fake_vehicle, daemon=True)
    thread.start()
    capture = udp_capture.UDPTelemetryCapture('127.0.0.1', port)
    assert capture.capture(0.6)
    thread.join(timeout=2)
    vehicle.close()

    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "loopback.sflog.zip"
        capture.save_bundle(str(path), tool_name="test")
        log = sflog.load(path)
        assert list(log.streams['text_log']['text']) == ["I (1) T: line 1", "I (2) T: line 2", "I (5) T: line 5",
                                                         "I (6) T: line 6", "I (8) T: line 8"]
        assert log.meta['capture']['text_packets_lost'] == 1
        assert 'loop_timing' in log.streams and 'link_health' in log.streams
        assert _no_errors(sflog.check_bundle(path))
        assert capture.text_duplicate_packets == 1


def test_print_stats_reports_diagnostics_and_the_text_loss(capsys):
    cap = _capture_with([build_status_v4(), build_text_packet(0, [_line(1)]),
                         build_text_packet(2, [_line(3, dropped_before=2)])])
    cap.start_time, cap.end_time = time.time() - 5, time.time()
    cap.print_stats()
    out = capsys.readouterr().out
    assert 'imu     loop: cycles=100' in out and 'nonfinite: detected=3 of 400' in out
    assert 'signal=2.50Mcps' in out and 'text_dropped_lines=14' in out
    assert 'Text log (0x51): 2 lines in 2 packets, 1 packets lost on the way, 14 lines dropped by the vehicle' in out


def test_print_stats_says_so_when_the_vehicle_sends_neither():
    body = build_status_v4()[:56]
    cap = _capture_with([body + bytes([_xor(body)])])
    cap.start_time, cap.end_time = time.time() - 5, time.time()
    cap.print_stats()   # must not raise on an older vehicle / 旧い機体でも例外を出さない
