#!/usr/bin/env python3
"""
test_udp_capture_ctrl_ref.py - Host-side decode test for the 0x48 control-reference entry (kPktCtrlRef, 50 Hz):
the 51-byte v6 layout of the current firmware (altitude and horizontal targets, thrust-excitation state and amplitude),
and the older 30-byte (v3) and 50-byte (v5) layouts.
0x48 制御目標エントリ（kPktCtrlRef、50Hz）のホスト側デコードの試験: 現行ファームの 51 バイトの v6（高さと水平の目標、
推力励振の状態と振幅）と、旧い 30 バイト（v3）・50 バイト（v5）の並び。

Hand-assembles datagrams with the byte layout of data_stream_wire.hpp (WireCtrlRef) and checks parse_packet() and the
ctrl_ref.csv row. A v3 packet (older firmware) still parses and yields 0 for the new cells.
data_stream_wire.hpp（WireCtrlRef）と同じバイト配置でデータグラムを手組みし、parse_packet() と ctrl_ref.csv の行を検査する。
v3 のパケット（旧ファーム）も読め、新しいセルは 0 になる。

Usage / 使い方:
    pytest test_udp_capture_ctrl_ref.py
"""

import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import udp_capture  # noqa: E402
from test_udp_capture_duty400 import build_unified  # noqa: E402

ANGLE_SCALE = 10000
EXCITATION_STATE_BYTE = (1 << 3) | 3      # multisine (1) in bits 3..4, Running (3) in bits 0..2 / 波形 1（多重正弦波）はビット 3〜4、Running（3）は下位 3 ビット
EXCITATION_AMPLITUDE_PERMILLE = 50         # 5.0 % of the hover thrust / ホバー推力の 5.0 %


def _ctrl_ref_entry(version):
    """[id][size][payload] of a CtrlRef entry of wire version 3, 5 or 6. / 電文版 3・5・6 の CtrlRef エントリ。"""
    head = struct.pack('<I2B2h', 123456, 2, EXCITATION_STATE_BYTE if version == 6 else 0, 100, -200)
    body = struct.pack('<5f', 0.38, 0.1, 0.2, 0.3, 0.4)           # total_thrust, duty x 4
    payload = head + body
    if version >= 5:
        payload += struct.pack('<5f', 0.7, 0.05, 0.0, 1.5, -2.5)   # alt_setpoint .. pos_setpoint_y
    if version == 6:
        payload += struct.pack('<B', EXCITATION_AMPLITUDE_PERMILLE)
    assert len(payload) == {3: 30, 5: 50, 6: 51}[version]
    return bytes([udp_capture.PKT_CTRL_REF, len(payload)]) + payload


def _decode(version):
    pkt, _ = build_unified(1, entries=_ctrl_ref_entry(version), entry_count=1)
    rows = [s for pid, s in udp_capture.parse_packet(pkt) if pid == udp_capture.PKT_CTRL_REF]
    assert len(rows) == 1
    return rows[0]


def test_v6_carries_targets_and_excitation_state():
    sample = _decode(6)
    assert sample['alt_setpoint'] == pytest.approx(0.7)
    assert sample['alt_vel_target'] == pytest.approx(0.05)
    assert sample['pos_setpoint_x'] == pytest.approx(1.5)
    assert sample['pos_setpoint_y'] == pytest.approx(-2.5)
    assert sample['thrust_excitation_state'] == EXCITATION_STATE_BYTE
    assert sample['thrust_excitation_amplitude'] == EXCITATION_AMPLITUDE_PERMILLE
    row = udp_capture._ctrl_ref_row(sample)
    assert row['thrust_excitation_state'] == EXCITATION_STATE_BYTE
    assert row['thrust_excitation_amplitude'] == EXCITATION_AMPLITUDE_PERMILLE


def test_v5_has_the_targets_and_no_excitation_amplitude():
    row = udp_capture._ctrl_ref_row(_decode(5))
    assert row['alt_setpoint'] == pytest.approx(0.7)
    assert row['thrust_excitation_amplitude'] == 0


def test_v3_older_firmware_still_parses_with_zero_cells():
    row = udp_capture._ctrl_ref_row(_decode(3))
    assert row['total_thrust'] == pytest.approx(0.38)
    assert row['alt_setpoint'] == 0.0 and row['pos_setpoint_x'] == 0.0
    assert row['thrust_excitation_state'] == 0 and row['thrust_excitation_amplitude'] == 0
