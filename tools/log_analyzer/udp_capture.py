#!/usr/bin/env python3
"""
udp_capture.py - UDP Telemetry Capture Tool for StampFly

Captures full-rate sensor data from StampFly via UDP.
Each sensor type arrives as independent packets at its native rate.
StampFly からフルレートセンサデータを UDP でキャプチャ。
各センサは固有レートの独立パケットとして到着する。

Packet IDs:
    0x40  IMU + ESKF (400Hz, 8 samples/unified packet)
    0x41  Position + Velocity (400Hz, 8 samples/unified packet)
    0x42  Control Input (50Hz)
    0x43  Optical Flow (100Hz)
    0x44  ToF (30Hz)
    0x45  Barometer (50Hz)
    0x46  Magnetometer (25Hz)
    0x49  ESKF P-diagonal (reserved, not sent by any current firmware)
    0x4A  Motor duty (400Hz, unified-packet entry, 8 samples/entry --
          plant input for `sf sysid fit`, see data_stream_wire.hpp kPktDuty400)
    0x4B  Control output (400Hz, unified-packet entry, 8 samples/entry --
          PRE-MIXER commanded thrust+torque, mixer-agnostic plant input for
          `sf sysid fit`/`rate-fit`, see data_stream_wire.hpp kPktCtrlOutput400)
    0x4C  Flight phase (400Hz, unified-packet entry, 8 samples/entry --
          flight_state + flip phase/result/angle, see data_stream_wire.hpp
          kPktFlightPhase400)
    0x4D  Flight flags (50Hz, unified-packet entry -- pilot ARM/FLIP buttons,
          flip readiness, ArmBlock, attitude verdicts, see kPktFlightFlags)
    0x4E  ToF silence facts (50Hz, unified-packet entry -- accelerometer-based
          vertical velocity estimate, estimated rise, thrust ratio of the descent
          without a ToF, failsafe verdict + reason, ToF health, ModeBlock; see
          data_stream_wire.hpp kPktToFSilence)
    0x4F  Status / Heartbeat (1Hz)
    0x50  Unified packet: header carries a 16-bit `sequence`, unwrapped here
          and combined with each sub-sample's in-packet index to form the
          v1 flight-log `seq` column (protocol/spec/flight_log.yaml; see
          UDPTelemetryCapture._unwrap_unified_seq()).

Output is a StampFly flight-log v1 bundle (docs/plans/
flight-log-format-plan.md section 2): one `.sflog.zip` with a CSV per
signal, written via lib/sflog (`save_bundle()`).
出力は StampFly フライトログ v1 一式（計画書 2節）: 信号ごとの CSV を
まとめた `.sflog.zip` 1個。lib/sflog 経由で書く（`save_bundle()`）。

Usage:
    python udp_capture.py [options]
    sf log wifi [options]

Examples:
    python udp_capture.py -d 30              # 30 seconds capture
    python udp_capture.py -d 60 -o flight_20260911T120000.sflog.zip
"""

import argparse
import socket
import struct
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from collections import defaultdict

import pandas as pd

import sflog

# save_bundle()'s meta.json `tool.version` -- this script's own version, not
# lib/sflog's (sflog.__version__, the on-disk format's implementation).
# save_bundle() が meta.json の `tool.version` に書く値 -- 本スクリプト
# 自身の版であり、lib/sflog 側の版（sflog.__version__、形式実装の版）とは
# 別。
_TOOL_VERSION = '1.0.0'

# =============================================================================
# Packet definitions (must match udp_telemetry.hpp)
# パケット定義（udp_telemetry.hpp と一致させること）
# =============================================================================

PKT_IMU_ESKF  = 0x40
PKT_POS_VEL   = 0x41
PKT_CONTROL   = 0x42
PKT_FLOW      = 0x43
PKT_TOF_BOTTOM = 0x44
PKT_TOF_FRONT  = 0x47
PKT_BARO      = 0x45
PKT_MAG       = 0x46
PKT_CTRL_REF  = 0x48
PKT_ESKF_PDIAG = 0x49
PKT_DUTY400   = 0x4A  # 400Hz motor duty (unified-packet entry, 8 samples/entry)
PKT_CTRL_OUTPUT400 = 0x4B  # 400Hz pre-mixer commanded thrust+torque (8 samples/entry)
PKT_FLIGHT_PHASE400 = 0x4C  # 400Hz flight_state + flip phase/result/angle (8 samples/entry)
PKT_FLIGHT_FLAGS    = 0x4D  # 50Hz pilot buttons + flip/arm readiness flags and block reasons
PKT_TOF_SILENCE     = 0x4E  # 50Hz downward-ToF silence facts (vertical velocity estimate, ToF-less descent, failsafe verdict, mode block)
PKT_RATE_REF  = 0x99  # virtual ID for 400Hz rate_ref (fixed part of unified packet)
PKT_STATUS    = 0x4F
PKT_UNIFIED   = 0x50  # 8x IMU+ESKF + 8x PosVel + 8x RateRef + variable entries
PKT_TEXT_LOG  = 0x51  # ESP_LOG text lines of the vehicle: [header 4B][record 8B + text]...[checksum 1B]

# Packet types whose 8 sub-samples per PKT_UNIFIED datagram get a v1
# flight-log `seq` (protocol/spec/flight_log.yaml): unwrapped unified-packet
# sequence x 8 + in-packet index. Exactly the schema's LOCKSTEP_STREAMS
# sources (imu/attitude share PKT_IMU_ESKF; posvel/rate_ref/motor/
# ctrl_output are one each) -- see UDPTelemetryCapture._process_datagram().
# PKT_UNIFIED データグラム1個につき8個のサブサンプルへ v1 フライトログの
# `seq`（protocol/spec/flight_log.yaml）を付与するパケット種別: 展開済み
# 統合パケット sequence × 8 + パケット内インデックス。スキーマの
# LOCKSTEP_STREAMS の由来そのもの（imu/attitude は PKT_IMU_ESKF を共有、
# posvel/rate_ref/motor/ctrl_output は1対1）--
# UDPTelemetryCapture._process_datagram() 参照。
UNIFIED_SEQ_PACKET_TYPES = (PKT_IMU_ESKF, PKT_POS_VEL, PKT_RATE_REF, PKT_DUTY400,
                            PKT_CTRL_OUTPUT400, PKT_FLIGHT_PHASE400)

CMD_START_LOG  = 0xF0
CMD_STOP_LOG   = 0xF1
CMD_HEARTBEAT  = 0xF2

UDP_LOG_PORT = 8890

# Wire quantization scales -- the firmware packs these two fields as
# int16 x SCALE to save bandwidth (data_stream_wire.hpp WireRateRef /
# WireCtrlRef); dividing back by SCALE recovers the physical unit
# (rad/s, rad) save_bundle() writes to imu-log v1's CSV columns.
# 電文の量子化スケール -- ファームはこの2フィールドを int16 x SCALE で
# 詰めて帯域を節約する（data_stream_wire.hpp の WireRateRef /
# WireCtrlRef）。SCALE で割り戻すと save_bundle() が v1 の CSV 列に書く
# 物理単位（rad/s, rad）に戻る。
RATE_REF_WIRE_SCALE = 1000.0
ANGLE_REF_WIRE_SCALE = 10000.0
FLIP_PHI_WIRE_SCALE = 1000.0   # int16 rad x 1000 (WireFlightPhase400.flip_phi)

# WireFlightFlags bit layout (data_stream_wire.hpp kPilotButton* / kFlag*)
# WireFlightFlags のビット配置（data_stream_wire.hpp の kPilotButton* / kFlag*）
PILOT_BUTTON_ARM = 0x01
PILOT_BUTTON_FLIP = 0x02
FLAG_FLIP_READY = 0x01
FLAG_ATTITUDE_MISMATCH = 0x02
FLAG_ATTITUDE_VERIFIED = 0x04

# WireToFSilence::tof_flags bit layout (data_stream_wire.hpp kToFFlag*)
# WireToFSilence::tof_flags のビット配置（data_stream_wire.hpp の kToFFlag*）
TOF_FLAG_SILENT = 0x01
TOF_FLAG_ALTITUDE_RELIABLE = 0x02
TOF_FLAG_INVALID_STUCK = 0x04
TOF_FLAG_STATUS_STALE = 0x08

# hPa -> Pa: the wire carries barometric pressure as raw hPa
# (data_stream_wire.hpp WireBaro), but v1's baro.csv unifies on SI Pa
# (protocol/spec/flight_log.yaml units/baro.pressure).
# hPa -> Pa: 電文は気圧を生の hPa のまま運ぶ（data_stream_wire.hpp の
# WireBaro）が、v1 の baro.csv は SI の Pa に統一する
# （protocol/spec/flight_log.yaml の units/baro.pressure）。
HPA_TO_PA = 100.0

# struct format strings for each sample type
# 各サンプル型の struct フォーマット
# '<' = little-endian

# ImuEskfSample: 80 bytes
#   timestamp(I) + gyro(3f) + accel(3f) + gyro_raw(3f) + accel_raw(3f) + quat(4f) + bias(6h)
FMT_IMU_ESKF = '<I 3f 3f 3f 3f 4f 3h 3h'
assert struct.calcsize(FMT_IMU_ESKF) == 80

# PosVelSample: 28 bytes
FMT_POS_VEL = '<I 3f 3f'
assert struct.calcsize(FMT_POS_VEL) == 28

# ControlSample: 20 bytes
FMT_CONTROL = '<I 4f'
assert struct.calcsize(FMT_CONTROL) == 20

# FlowSample: 9 bytes
FMT_FLOW = '<I 2h B'
assert struct.calcsize(FMT_FLOW) == 9

# ToFSingleSample: 9 bytes
FMT_TOF = '<I f B'
assert struct.calcsize(FMT_TOF) == 9

# BaroSample: 12 bytes
FMT_BARO = '<I 2f'
assert struct.calcsize(FMT_BARO) == 12

# MagSample: 16 bytes
FMT_MAG = '<I 3f'
assert struct.calcsize(FMT_MAG) == 16

# CtrlRefSample: 38 bytes (v4) or 30 bytes (v3) or 14 bytes (v2) or 10 bytes (v1)
# v1: timestamp(I) + flight_mode(B) + reserved(B) + angle_ref(2h) = 10B
# v2: v1 + total_thrust(f) = 14B
# v3: v2 + motor_duty(4f) = 30B
# v4: v3 + alt_setpoint(f) + alt_vel_target(f) = 38B
FMT_CTRL_REF_V1 = '<I 2B 2h'
FMT_CTRL_REF_V2 = '<I 2B 2h f'
FMT_CTRL_REF_V3 = '<I 2B 2h 5f'
FMT_CTRL_REF_V4 = '<I 2B 2h 7f'
FMT_CTRL_REF_V5 = '<I 2B 2h 10f'
FMT_CTRL_REF = FMT_CTRL_REF_V5  # default for new logs
assert struct.calcsize(FMT_CTRL_REF_V5) == 50

# EskfPDiagSample: 64 bytes (10Hz)
#   timestamp(I) + p_diag(15f) = 64B
FMT_ESKF_PDIAG = '<I 15f'
assert struct.calcsize(FMT_ESKF_PDIAG) == 64

# RateRefFixed: 6 bytes (rate ref, 400Hz, fixed part of unified packet)
#   rate_ref_roll(h) + rate_ref_pitch(h) + rate_ref_yaw(h)
FMT_RATE_REF = '<3h'
assert struct.calcsize(FMT_RATE_REF) == 6

# Duty400Sample: 8 bytes (one of 8 packed into a 64B kPktDuty400 entry) --
# mirrors WireDuty400 in data_stream_wire.hpp. Each value = duty(0..1) x 65535.
#   duty_FR(H) + duty_RR(H) + duty_RL(H) + duty_FL(H)
FMT_DUTY400 = '<4H'
assert struct.calcsize(FMT_DUTY400) == 8

# CtrlOutput400Sample: 16 bytes (one of 8 packed into a 128B kPktCtrlOutput400
# entry) -- mirrors WireControlOutput400 in data_stream_wire.hpp. The
# PRE-MIXER commanded thrust[N] + body torque[Nm] R,P,Y -- not quantized
# (unlike duty's fixed [0,1] range, thrust/torque have no natural fixed scale
# to quantize against without risking silent clipping).
#   thrust(f) + torque_roll(f) + torque_pitch(f) + torque_yaw(f)
FMT_CTRL_OUTPUT400 = '<4f'
assert struct.calcsize(FMT_CTRL_OUTPUT400) == 16

# FlightPhase400Sample: 5 bytes (one of 8 packed into a 40B kPktFlightPhase400
# entry) -- mirrors WireFlightPhase400 in data_stream_wire.hpp.
#   flight_state(B) + flip_phase(B) + flip_result(B) + flip_phi(h, rad x 1000)
FMT_FLIGHT_PHASE400 = '<3Bh'
assert struct.calcsize(FMT_FLIGHT_PHASE400) == 5

# FlightFlags: 8 bytes (one kPktFlightFlags entry) -- mirrors WireFlightFlags.
#   timestamp_us(I) + pilot_buttons(B) + status_flags(B) + flip_block_reason(B)
#   + arm_block(B)
FMT_FLIGHT_FLAGS = '<I4B'
assert struct.calcsize(FMT_FLIGHT_FLAGS) == 8

# ToFSilence: 19 bytes (one kPktToFSilence entry) -- mirrors WireToFSilence.
#   timestamp_us(I) + vertical_velocity_up(f) + vertical_rise_m(f)
#   + descent_thrust_ratio(f) + tof_flags(B) + tof_reason(B) + mode_block(B)
FMT_TOF_SILENCE = '<I3f3B'
assert struct.calcsize(FMT_TOF_SILENCE) == 19

# Text-log packet 0x51 (data_stream_wire.hpp WireTextRecord): one record header per line, then `length` bytes of text.
# テキストログのパケット 0x51（data_stream_wire.hpp の WireTextRecord）: 行ごとに 1 つのレコードヘッダ、続いて本文 length バイト。
TEXT_ORDER_STRIDE = 1000   # > the most records one text packet can hold (1029 B / 9 B), so (sequence, index) sorts as one number / 1 パケットが持てるレコード数（517 B / 9 B）より大きいので、（通し番号, 位置）が 1 つの数として並ぶ
FMT_TEXT_RECORD = '<I B B H'   # timestamp_us, flags, dropped_before, length
assert struct.calcsize(FMT_TEXT_RECORD) == 8
TEXT_FLAG_TRUNCATED = 0x01

# Status packet v4 extension (data_stream_wire.hpp WireStatusV4Extension), appended at buffer offset 56 (after the
# v3 payload); the version is chosen by the TOTAL datagram length: 17 (v1), 53 (v2), 57 (v3), 224 (v4).
# ステータスパケット v4 の拡張部（data_stream_wire.hpp の WireStatusV4Extension）。v3 のペイロードの後ろ、バッファ
# offset 56 から追記する。バージョンはデータグラムの全長で選ぶ: 17（v1）、53（v2）、57（v3）、224（v4）。
STATUS_V3_LENGTH = 57
STATUS_V4_LENGTH = 224
STATUS_V4_OFFSET = 56
FMT_TOF_DIAGNOSTIC = '<B 3f'    # range_status, signal_rate [Mcps], ambient_rate [Mcps], sigma_mm [mm]  (13 B)
FMT_LOOP_TIMING = '<11I'        # 11 x uint32 (44 B), the order of LOOP_TIMING_FIELDS
FMT_NONFINITE = '<9I B'         # checked, detected, 6 stages, first_us, first_stage (37 B)
FMT_LINK_HEALTH = '<4I'         # log_stream_overflow, send_drops, entry_drops, text_dropped_lines (16 B)
assert struct.calcsize(FMT_TOF_DIAGNOSTIC) == 13
assert struct.calcsize(FMT_LOOP_TIMING) == 44
assert struct.calcsize(FMT_NONFINITE) == 37
assert struct.calcsize(FMT_LINK_HEALTH) == 16
assert STATUS_V4_OFFSET + 2 * 13 + 2 * 44 + 37 + 16 + 1 == STATUS_V4_LENGTH
LOOP_TIMING_FIELDS = ('cycles', 'period_max_us', 'late_count', 'busy_max_us', 'over_budget_count', 'last_late_us',
                      'armed_cycles', 'armed_period_max_us', 'armed_late_count', 'armed_busy_max_us',
                      'armed_over_budget_count')
NONFINITE_FIELDS = ('checked_cycles', 'detected_cycles', 'estimate_input', 'setpoint_input', 'control_reference',
                    'control_output', 'mixer_output', 'motor_command', 'first_us')
LINK_HEALTH_FIELDS = ('log_stream_overflow', 'send_drops', 'entry_drops', 'text_dropped_lines')
TOF_DIAGNOSTIC_FIELDS = ('range_status', 'signal_rate', 'ambient_rate', 'sigma_mm')

# Header: 4 bytes
FMT_HEADER = '<B H B'
assert struct.calcsize(FMT_HEADER) == 4

SAMPLE_INFO = {
    PKT_IMU_ESKF:  ('IMU+ESKF',  FMT_IMU_ESKF, 80),
    PKT_POS_VEL:   ('Pos+Vel',   FMT_POS_VEL,  28),
    PKT_CONTROL:   ('Control',   FMT_CONTROL,   20),
    PKT_FLOW:      ('Flow',      FMT_FLOW,       9),
    PKT_TOF_BOTTOM:('ToF_Bot',   FMT_TOF,        9),
    PKT_BARO:      ('Baro',      FMT_BARO,      12),
    PKT_MAG:       ('Mag',       FMT_MAG,       16),
    PKT_TOF_FRONT: ('ToF_Frt',   FMT_TOF,        9),
    PKT_CTRL_REF:  ('CtrlRef',   FMT_CTRL_REF,  50),
    PKT_ESKF_PDIAG:('P_diag',   FMT_ESKF_PDIAG, 64),
}

# CSV column names per packet type
# パケット型ごとの CSV 列名
CSV_COLUMNS = {
    PKT_IMU_ESKF: [
        'timestamp_us',
        'gyro_x', 'gyro_y', 'gyro_z',
        'accel_x', 'accel_y', 'accel_z',
        'gyro_raw_x', 'gyro_raw_y', 'gyro_raw_z',
        'accel_raw_x', 'accel_raw_y', 'accel_raw_z',
        'quat_w', 'quat_x', 'quat_y', 'quat_z',
        'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z',
        'accel_bias_x', 'accel_bias_y', 'accel_bias_z',
    ],
    PKT_POS_VEL: [
        'timestamp_us',
        'pos_x', 'pos_y', 'pos_z',
        'vel_x', 'vel_y', 'vel_z',
    ],
    PKT_CONTROL: [
        'timestamp_us',
        'ctrl_throttle', 'ctrl_roll', 'ctrl_pitch', 'ctrl_yaw',
    ],
    PKT_FLOW: [
        'timestamp_us',
        'flow_x', 'flow_y', 'flow_quality',
    ],
    PKT_TOF_BOTTOM: [
        'timestamp_us',
        'tof_distance', 'tof_status',
    ],
    PKT_TOF_FRONT: [
        'timestamp_us',
        'tof_distance', 'tof_status',
    ],
    PKT_BARO: [
        'timestamp_us',
        'baro_altitude', 'baro_pressure',
    ],
    PKT_MAG: [
        'timestamp_us',
        'mag_x', 'mag_y', 'mag_z',
    ],
    PKT_CTRL_REF: [
        'timestamp_us',
        'flight_mode', 'reserved',
        'angle_ref_roll', 'angle_ref_pitch',
        'total_thrust',
        'motor_duty_FR', 'motor_duty_RR', 'motor_duty_RL', 'motor_duty_FL',
        'alt_setpoint', 'alt_vel_target',
        'climb_rate_cmd', 'pos_setpoint_x', 'pos_setpoint_y',
    ],
    PKT_ESKF_PDIAG: [
        'timestamp_us',
        'p_pos_x', 'p_pos_y', 'p_pos_z',
        'p_vel_x', 'p_vel_y', 'p_vel_z',
        'p_att_x', 'p_att_y', 'p_att_z',
        'p_bg_x', 'p_bg_y', 'p_bg_z',
        'p_ba_x', 'p_ba_y', 'p_ba_z',
    ],
}


# =============================================================================
# Packet parser
# パケットパーサー
# =============================================================================

def verify_checksum(data: bytes) -> bool:
    """Verify XOR checksum (last byte = XOR of all preceding bytes)"""
    xor_val = 0
    for b in data[:-1]:
        xor_val ^= b
    return xor_val == data[-1]


def _parse_text_log_packet(data: bytes, count: int) -> list:
    """Parse a text-log packet (0x51) into one (PKT_TEXT_LOG, sample) per line, in the packet's order.
    The records must account for every byte of the datagram and `count` must match: anything else is
    rejected as a whole (a damaged packet that passed the XOR checksum must not yield half a line).
    `_idx` is the record's index in the packet (the order key); `text` still ends with its newline.
    テキストログのパケット（0x51）を、パケット内の順序で行ごとの (PKT_TEXT_LOG, sample) にパースする。レコードは
    データグラムの全バイトを使い切り、`count` と一致しなければならない。そうでなければパケット全体を拒否する（XOR
    チェックサムを通り抜けた壊れたパケットが、行の半分を出してはならない）。`_idx` はパケット内の位置（順序のキー）、
    `text` は改行を含んだまま。
    """
    header_size = struct.calcsize(FMT_HEADER)
    record_size = struct.calcsize(FMT_TEXT_RECORD)
    end = len(data) - 1   # the last byte is the checksum / 最後の 1 バイトはチェックサム
    offset = header_size
    results = []
    for index in range(count):
        if offset + record_size > end:
            return []
        timestamp_us, flags, dropped_before, length = struct.unpack_from(FMT_TEXT_RECORD, data, offset)
        offset += record_size
        if offset + length > end:
            return []
        text = data[offset:offset + length].decode('utf-8', errors='replace')
        offset += length
        results.append((PKT_TEXT_LOG, {
            'timestamp_us': timestamp_us,
            'truncated': 1 if flags & TEXT_FLAG_TRUNCATED else 0,
            'dropped_before': dropped_before,
            'text': text,
            '_idx': index,
        }))
    if offset != end or not results:
        return []
    return results


def _parse_status_v4_extension(data: bytes) -> dict:
    """Decode the v4 part of a 224-byte status packet into sample keys named like the bundle columns
    (tof_diagnostics, loop_timing, nonfinite, link_health streams of protocol/spec/flight_log.yaml).
    224 バイトのステータスパケットの v4 部分を、一式の列名と同じ名前のサンプルのキーへデコードする
    （protocol/spec/flight_log.yaml の tof_diagnostics・loop_timing・nonfinite・link_health ストリーム）。
    """
    sample = {}
    offset = STATUS_V4_OFFSET
    for side in ('bottom', 'front'):
        values = struct.unpack_from(FMT_TOF_DIAGNOSTIC, data, offset)
        offset += struct.calcsize(FMT_TOF_DIAGNOSTIC)
        for field, value in zip(TOF_DIAGNOSTIC_FIELDS, values):
            sample[f'{side}_{field}'] = value
    for task in ('imu', 'control'):
        values = struct.unpack_from(FMT_LOOP_TIMING, data, offset)
        offset += struct.calcsize(FMT_LOOP_TIMING)
        for field, value in zip(LOOP_TIMING_FIELDS, values):
            sample[f'{task}_{field}'] = value
    values = struct.unpack_from(FMT_NONFINITE, data, offset)
    offset += struct.calcsize(FMT_NONFINITE)
    for field, value in zip(NONFINITE_FIELDS + ('first_stage',), values):
        sample[field] = value
    values = struct.unpack_from(FMT_LINK_HEALTH, data, offset)
    for field, value in zip(LINK_HEALTH_FIELDS, values):
        sample[field] = value
    return sample


def parse_packet(data: bytes) -> list:
    """Parse a UDP telemetry packet into list of (packet_id, sample_dict) tuples.
    Returns empty list on error."""

    if len(data) < 5:  # minimum: header(4) + checksum(1)
        return []

    if not verify_checksum(data):
        return []

    # Parse header
    pkt_id, seq, count = struct.unpack_from(FMT_HEADER, data, 0)

    # Unified packet (0x50): 8× IMU+ESKF + 8× PosVel + 8× RateRef + sensor entries
    # 統合パケット: 8× IMU+ESKF + 8× PosVel + 8× RateRef + センサエントリ
    #
    # Each of the 5 lockstep sub-sample kinds below (IMU+ESKF, PosVel,
    # RateRef, and -- further down -- Duty400/CtrlOutput400) gets an `_idx`
    # (0..7, its position within THIS datagram). UDPTelemetryCapture.
    # _process_datagram() turns `_idx` into the v1 flight-log `seq` column
    # (unwrapped unified-packet sequence x 8 + `_idx`) and pops it off --
    # parse_packet() itself is stateless and does not know the running
    # sequence, only the position within one packet.
    # 以下5種のロックステップ系サブサンプル（IMU+ESKF、PosVel、RateRef、
    # さらに下の Duty400/CtrlOutput400）にはそれぞれ `_idx`（0..7、この
    # データグラム内での位置）を付与する。UDPTelemetryCapture.
    # _process_datagram() が `_idx` を v1 フライトログの `seq` 列（展開済み
    # 統合パケット sequence × 8 + `_idx`）へ変換して取り除く --
    # parse_packet() 自体は状態を持たず、通し番号ではなく1パケット内の
    # 位置しか分からない。
    if pkt_id == PKT_UNIFIED:
        results = []
        imu_timestamps = []
        offset = 4  # after header

        # 8× ImuEskfSample (80B each)
        for i in range(8):
            values = struct.unpack_from(FMT_IMU_ESKF, data, offset)
            sample = dict(zip(CSV_COLUMNS[PKT_IMU_ESKF], values))
            for key in ['gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z',
                        'accel_bias_x', 'accel_bias_y', 'accel_bias_z']:
                sample[key] = sample[key] / 10000.0
            sample['_idx'] = i
            imu_timestamps.append(sample['timestamp_us'])
            results.append((PKT_IMU_ESKF, sample))
            offset += 80

        # 8× PosVelSample (28B each)
        for i in range(8):
            values = struct.unpack_from(FMT_POS_VEL, data, offset)
            sample = dict(zip(CSV_COLUMNS[PKT_POS_VEL], values))
            sample['_idx'] = i
            results.append((PKT_POS_VEL, sample))
            offset += 28

        # 8× RateRefFixed (6B each) — shares IMU timestamp
        for i in range(8):
            values = struct.unpack_from(FMT_RATE_REF, data, offset)
            sample = {
                'timestamp_us': imu_timestamps[i],
                'rate_ref_roll': values[0],
                'rate_ref_pitch': values[1],
                'rate_ref_yaw': values[2],
                '_idx': i,
            }
            results.append((PKT_RATE_REF, sample))
            offset += 6

        # Entry count (1B)
        entry_count = data[offset]
        offset += 1

        # Sensor entries (variable)
        for i in range(entry_count):
            if offset + 2 > len(data) - 1:  # -1 for checksum
                break
            sensor_id = data[offset]
            data_size = data[offset + 1]
            offset += 2

            # Motor duty (0x4A): NOT in SAMPLE_INFO -- one entry packs 8
            # sub-samples (like the RateRef fixed block), each paired by
            # index with the same unified packet's IMU timestamp.
            # モータduty（0x4A）: SAMPLE_INFO には無い特殊形式 -- 1エントリに
            # サブサンプル8件（RateRef固定ブロックと同様）、同一統合パケットの
            # IMU タイムスタンプと index で対応させる。
            if (sensor_id == PKT_DUTY400 and data_size == 64
                    and offset + data_size <= len(data) - 1):
                for j in range(8):
                    fr, rr, rl, fl = struct.unpack_from(FMT_DUTY400, data, offset + j * 8)
                    results.append((PKT_DUTY400, {
                        'timestamp_us': imu_timestamps[j],
                        'duty_FR': fr / 65535.0, 'duty_RR': rr / 65535.0,
                        'duty_RL': rl / 65535.0, 'duty_FL': fl / 65535.0,
                        '_idx': j,
                    }))
            # Control output (0x4B): same 8-sub-samples-per-entry convention
            # as duty400 above, PRE-MIXER thrust[N]+torque[Nm] instead of
            # post-mixer duty. Mixer-agnostic plant input for `sf sysid fit`/
            # `rate-fit` -- see data_stream_wire.hpp kPktCtrlOutput400.
            # 制御出力（0x4B）: 上の duty400 と同じ8サブサンプル/エントリの
            # 規約。ミキサー後ろの duty ではなく、ミキサー手前の
            # 推力[N]+トルク[Nm]。`sf sysid fit`/`rate-fit` のミキサー非依存な
            # プラント入力 -- data_stream_wire.hpp kPktCtrlOutput400 参照。
            elif (sensor_id == PKT_CTRL_OUTPUT400 and data_size == 128
                    and offset + data_size <= len(data) - 1):
                for j in range(8):
                    thrust, tq_roll, tq_pitch, tq_yaw = struct.unpack_from(
                        FMT_CTRL_OUTPUT400, data, offset + j * 16)
                    results.append((PKT_CTRL_OUTPUT400, {
                        'timestamp_us': imu_timestamps[j],
                        'ctrl_output_thrust': thrust,
                        'ctrl_output_torque_roll': tq_roll,
                        'ctrl_output_torque_pitch': tq_pitch,
                        'ctrl_output_torque_yaw': tq_yaw,
                        '_idx': j,
                    }))
            # Flight phase (0x4C): 8 sub-samples per entry, index-paired with the
            # packet's IMU timestamps (same convention as duty400).
            # 飛行フェーズ（0x4C）: 1エントリ8サブサンプル。パケットの IMU
            # タイムスタンプと index で対応（duty400 と同じ規約）。
            elif (sensor_id == PKT_FLIGHT_PHASE400 and data_size == 40
                    and offset + data_size <= len(data) - 1):
                for j in range(8):
                    state, phase, result, phi = struct.unpack_from(
                        FMT_FLIGHT_PHASE400, data, offset + j * 5)
                    results.append((PKT_FLIGHT_PHASE400, {
                        'timestamp_us': imu_timestamps[j],
                        'flight_state': state,
                        'flip_phase': phase,
                        'flip_result': result,
                        'flip_phi': phi / FLIP_PHI_WIRE_SCALE,
                        '_idx': j,
                    }))
            # Flight flags (0x4D): one 50Hz record per packet.
            # 飛行フラグ（0x4D）: パケットあたり 50Hz で1件。
            elif (sensor_id == PKT_FLIGHT_FLAGS and data_size == 8
                    and offset + data_size <= len(data) - 1):
                ts, buttons, flags, block, arm_block = struct.unpack_from(
                    FMT_FLIGHT_FLAGS, data, offset)
                results.append((PKT_FLIGHT_FLAGS, {
                    'timestamp_us': ts,
                    'pilot_arm': int(bool(buttons & PILOT_BUTTON_ARM)),
                    'pilot_flip': int(bool(buttons & PILOT_BUTTON_FLIP)),
                    'flip_ready': int(bool(flags & FLAG_FLIP_READY)),
                    'flip_block_reason': block,
                    'arm_block': arm_block,
                    'attitude_mismatch': int(bool(flags & FLAG_ATTITUDE_MISMATCH)),
                    'attitude_verified': int(bool(flags & FLAG_ATTITUDE_VERIFIED)),
                }))
            # ToF silence facts (0x4E): one 50Hz record per packet.
            # ToF 出力停止の事実（0x4E）: パケットあたり 50Hz で1件。
            elif (sensor_id == PKT_TOF_SILENCE and data_size == 19
                    and offset + data_size <= len(data) - 1):
                (ts, velocity_up, rise_m, thrust_ratio,
                 tof_flags, reason, mode_block) = struct.unpack_from(
                    FMT_TOF_SILENCE, data, offset)
                results.append((PKT_TOF_SILENCE, {
                    'timestamp_us': ts,
                    'vertical_velocity_up': velocity_up,
                    'vertical_rise_m': rise_m,
                    'descent_thrust_ratio': thrust_ratio,
                    'tof_silent': int(bool(tof_flags & TOF_FLAG_SILENT)),
                    'tof_reason': reason,
                    'tof_altitude_reliable': int(bool(tof_flags & TOF_FLAG_ALTITUDE_RELIABLE)),
                    'tof_invalid_stuck': int(bool(tof_flags & TOF_FLAG_INVALID_STUCK)),
                    'tof_status_stale': int(bool(tof_flags & TOF_FLAG_STATUS_STALE)),
                    'mode_block': mode_block,
                }))
            elif sensor_id in SAMPLE_INFO and offset + data_size <= len(data) - 1:
                _, fmt, sample_size = SAMPLE_INFO[sensor_id]
                if data_size == sample_size:
                    columns = CSV_COLUMNS[sensor_id]
                    values = struct.unpack_from(fmt, data, offset)
                    sample = dict(zip(columns, values))
                    results.append((sensor_id, sample))
                elif sensor_id == PKT_CTRL_REF:
                    # Backward compatible: accept v1(10B), v2(14B), v3(30B)
                    # 後方互換: v1(10B), v2(14B), v3(30B), v4(38B) を受け入れ
                    if data_size == 10:
                        cols = CSV_COLUMNS[PKT_CTRL_REF][:5]
                        vals = struct.unpack_from(FMT_CTRL_REF_V1, data, offset)
                        results.append((sensor_id, dict(zip(cols, vals))))
                    elif data_size == 14:
                        cols = CSV_COLUMNS[PKT_CTRL_REF][:6]
                        vals = struct.unpack_from(FMT_CTRL_REF_V2, data, offset)
                        results.append((sensor_id, dict(zip(cols, vals))))
                    elif data_size == 30:
                        cols = CSV_COLUMNS[PKT_CTRL_REF][:10]
                        vals = struct.unpack_from(FMT_CTRL_REF_V3, data, offset)
                        results.append((sensor_id, dict(zip(cols, vals))))
                    elif data_size == 38:
                        cols = CSV_COLUMNS[PKT_CTRL_REF][:12]
                        vals = struct.unpack_from(FMT_CTRL_REF_V4, data, offset)
                        results.append((sensor_id, dict(zip(cols, vals))))
            offset += data_size

        return results

    if pkt_id == PKT_TEXT_LOG:
        return _parse_text_log_packet(data, count)

    if pkt_id not in SAMPLE_INFO:
        # Status packet (0x4F) — parse fields
        # StatusPacket v2: [header 4B][uptime_ms 4B][voltage 4B][flight_state 1B]
        #   [sensor_health 1B][eskf_status 1B][padding 1B]
        #   [pid_roll Kp/Ti/Td 12B][pid_pitch Kp/Ti/Td 12B][pid_yaw Kp/Ti/Td 12B]
        #   [checksum 1B] = 53B
        # StatusPacket v3: v2 + [current_ma 4B][checksum 1B] = 57B (battery current,
        #   appended AFTER pid_gains — see data_stream_wire.hpp WireStatusPayload)
        # StatusPacket v4: v3 + the 167B WireStatusV4Extension = 224B (see _parse_status_v4_extension())
        # Legacy (17B) also supported for backward compatibility
        if pkt_id == PKT_STATUS and len(data) in (17, 53, 57, STATUS_V4_LENGTH):
            uptime_ms, voltage, flight_state, sensor_health, eskf_status, reset_reason = \
                struct.unpack_from('<IfBBBB', data, 4)
            # esp_reset_reason() names — read a crash cause over WiFi (no serial).
            _RST = {0: 'UNKNOWN', 1: 'POWERON', 2: 'EXT', 3: 'SW', 4: 'PANIC',
                    5: 'INT_WDT', 6: 'TASK_WDT', 7: 'WDT', 8: 'DEEPSLEEP',
                    9: 'BROWNOUT', 10: 'SDIO', 11: 'USB', 12: 'JTAG'}
            sample = {
                'timestamp_us': uptime_ms * 1000,
                'uptime_ms': uptime_ms,
                'voltage': voltage,
                'flight_state': flight_state,
                'sensor_health': sensor_health,
                'eskf_status': eskf_status,
                'reset_reason': reset_reason,
                'reset_reason_name': _RST.get(reset_reason, str(reset_reason)),
            }
            # Parse PID gains if present (v2/v3/v4, 53B, 57B or 224B)
            if len(data) in (53, 57, STATUS_V4_LENGTH):
                pid_vals = struct.unpack_from('<9f', data, 16)
                sample['pid_roll_kp']  = pid_vals[0]
                sample['pid_roll_ti']  = pid_vals[1]
                sample['pid_roll_td']  = pid_vals[2]
                sample['pid_pitch_kp'] = pid_vals[3]
                sample['pid_pitch_ti'] = pid_vals[4]
                sample['pid_pitch_td'] = pid_vals[5]
                sample['pid_yaw_kp']   = pid_vals[6]
                sample['pid_yaw_ti']   = pid_vals[7]
                sample['pid_yaw_td']   = pid_vals[8]
            # Parse battery current if present (v3, 57B) — appended after pid_gains
            # at payload offset 48 (buffer offset 4 + 48 = 52).
            if len(data) in (57, STATUS_V4_LENGTH):
                sample['current_ma'] = struct.unpack_from('<f', data, 52)[0]
            # v4 (224B): ToF diagnostics, loop timing of both 400 Hz tasks, non-finite counters, log-path losses
            if len(data) == STATUS_V4_LENGTH:
                sample.update(_parse_status_v4_extension(data))
            return [(PKT_STATUS, sample)]
        return []

    name, fmt, sample_size = SAMPLE_INFO[pkt_id]
    columns = CSV_COLUMNS[pkt_id]

    expected_size = 4 + sample_size * count + 1
    if len(data) != expected_size:
        return []

    results = []
    offset = 4  # after header
    for i in range(count):
        values = struct.unpack_from(fmt, data, offset)
        sample = dict(zip(columns, values))

        # Scale bias values back from int16 × 10000 to float
        # バイアス値を int16 × 10000 から float に戻す
        if pkt_id == PKT_IMU_ESKF:
            for key in ['gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z',
                        'accel_bias_x', 'accel_bias_y', 'accel_bias_z']:
                sample[key] = sample[key] / 10000.0

        results.append((pkt_id, sample))
        offset += sample_size

    return results


# =============================================================================
# v1 flight-log bundle row builders (self.samples[pkt_id] entry -> a v1
# stream's column dict, protocol/spec/flight_log.yaml)
# v1 フライトログ一式の行ビルダー（self.samples[pkt_id] の1件 -> v1
# ストリームの列名dict、protocol/spec/flight_log.yaml）
# =============================================================================
# Mirrors lib/sflog/convert.py's _row_from_*() builders, but the SOURCE here
# is this module's own already-decoded sample dict (parse_packet()'s
# CSV_COLUMNS names) rather than a legacy JSONL object -- so most of these
# are a plain subset-and-rename. Three keep a wire quantization/unit
# conversion parse_packet() does not itself undo: rate_ref and ctrl_ref's
# angle_ref (int16 x SCALE, see RATE_REF_WIRE_SCALE/ANGLE_REF_WIRE_SCALE)
# and baro's pressure (hPa -> Pa, see HPA_TO_PA).
# lib/sflog/convert.py の _row_from_*() を模すが、ここでの入力はこの
# モジュール自身が既にデコード済みのサンプル dict（parse_packet() の
# CSV_COLUMNS の名前）であり、レガシー JSONL オブジェクトではない --
# そのため大半は単純な部分集合＋改名で済む。parse_packet() 自身が戻して
# いない電文量子化/単位変換を残す3つだけ例外: rate_ref と ctrl_ref の
# angle_ref（int16 x SCALE、RATE_REF_WIRE_SCALE/ANGLE_REF_WIRE_SCALE 参照）、
# baro の pressure（hPa -> Pa、HPA_TO_PA 参照）。

_IMU_KEYS = (
    'timestamp_us', 'seq', 'gyro_x', 'gyro_y', 'gyro_z',
    'accel_x', 'accel_y', 'accel_z',
    'gyro_raw_x', 'gyro_raw_y', 'gyro_raw_z',
    'accel_raw_x', 'accel_raw_y', 'accel_raw_z',
)
_ATTITUDE_KEYS = (
    'timestamp_us', 'seq', 'quat_w', 'quat_x', 'quat_y', 'quat_z',
    'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z',
    'accel_bias_x', 'accel_bias_y', 'accel_bias_z',
)
_POSVEL_KEYS = ('timestamp_us', 'seq', 'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z')
_MOTOR_KEYS = ('timestamp_us', 'seq', 'duty_FR', 'duty_RR', 'duty_RL', 'duty_FL')
_FLIGHT_PHASE_KEYS = ('timestamp_us', 'seq', 'flight_state', 'flip_phase',
                      'flip_result', 'flip_phi')
_FLIGHT_FLAGS_KEYS = ('timestamp_us', 'pilot_arm', 'pilot_flip', 'flip_ready',
                      'flip_block_reason', 'arm_block', 'attitude_mismatch',
                      'attitude_verified')
_TOF_SILENCE_KEYS = ('timestamp_us', 'vertical_velocity_up', 'vertical_rise_m',
                     'descent_thrust_ratio', 'tof_silent', 'tof_reason',
                     'tof_altitude_reliable', 'tof_invalid_stuck',
                     'tof_status_stale', 'mode_block')
_ESKF_COV_KEYS = (
    'timestamp_us',
    'p_pos_x', 'p_pos_y', 'p_pos_z',
    'p_vel_x', 'p_vel_y', 'p_vel_z',
    'p_att_x', 'p_att_y', 'p_att_z',
    'p_bg_x', 'p_bg_y', 'p_bg_z',
    'p_ba_x', 'p_ba_y', 'p_ba_z',
)

_STATUS_PID_AXES = ('roll', 'pitch', 'yaw')
_STATUS_PID_TERMS = ('kp', 'ti', 'td')


def _select(sample: dict, keys) -> dict:
    """Subset `sample` to exactly `keys`, identity-named (the v1 column
    name equals parse_packet()'s CSV_COLUMNS name already). A key missing
    from `sample` (e.g. `seq` on a sample that never went through a unified
    packet) becomes None -- an empty CSV cell, per the "absent, not
    invented" rule (docs/plans/flight-log-format-plan.md section 2.2).
    `sample` を `keys` だけへ絞り込む（v1 の列名は parse_packet() の
    CSV_COLUMNS の名前と既に同じ）。`sample` に無いキー（統合パケットを
    一度も経由しなかったサンプルの `seq` 等）は None（CSV上は空欄）になる
    -- 「無いものは無い」規約（計画書 2.2節）。
    """
    return {k: sample.get(k) for k in keys}


def _rate_ref_row(sample: dict) -> dict:
    return {
        'timestamp_us': sample['timestamp_us'],
        'seq': sample.get('seq'),
        'rate_ref_roll': sample['rate_ref_roll'] / RATE_REF_WIRE_SCALE,
        'rate_ref_pitch': sample['rate_ref_pitch'] / RATE_REF_WIRE_SCALE,
        'rate_ref_yaw': sample['rate_ref_yaw'] / RATE_REF_WIRE_SCALE,
    }


def _ctrl_output_row(sample: dict) -> dict:
    return {
        'timestamp_us': sample['timestamp_us'],
        'seq': sample.get('seq'),
        'thrust': sample['ctrl_output_thrust'],
        'torque_roll': sample['ctrl_output_torque_roll'],
        'torque_pitch': sample['ctrl_output_torque_pitch'],
        'torque_yaw': sample['ctrl_output_torque_yaw'],
    }


def _pilot_row(sample: dict) -> dict:
    return {
        'timestamp_us': sample['timestamp_us'],
        'throttle': sample['ctrl_throttle'],
        'roll': sample['ctrl_roll'],
        'pitch': sample['ctrl_pitch'],
        'yaw': sample['ctrl_yaw'],
    }


def _ctrl_ref_row(sample: dict) -> dict:
    """CtrlRef (0x48) -> v1 ctrl_ref.csv row. Older wire versions (v1..v4,
    see FMT_CTRL_REF_V1..V4) omit every field after `angle_ref_pitch` --
    `.get()` with a schema-consistent 0.0 default matches this module's
    (now-removed) JSONL writer's own fallback for the same struct.
    CtrlRef (0x48) -> v1 ctrl_ref.csv の行。旧電文版（v1..v4、
    FMT_CTRL_REF_V1..V4 参照）は `angle_ref_pitch` 以降の全フィールドを
    持たない -- `.get()` とスキーマに整合する既定値 0.0 は、本モジュールの
    （削除済みの）JSONL 書き出しが同じ構造体に使っていたのと同じ既定値。
    """
    return {
        'timestamp_us': sample['timestamp_us'],
        'flight_mode': sample['flight_mode'],
        'angle_ref_roll': sample['angle_ref_roll'] / ANGLE_REF_WIRE_SCALE,
        'angle_ref_pitch': sample['angle_ref_pitch'] / ANGLE_REF_WIRE_SCALE,
        'total_thrust': sample.get('total_thrust', 0.0),
        'duty_FR': sample.get('motor_duty_FR', 0.0),
        'duty_RR': sample.get('motor_duty_RR', 0.0),
        'duty_RL': sample.get('motor_duty_RL', 0.0),
        'duty_FL': sample.get('motor_duty_FL', 0.0),
        'alt_setpoint': sample.get('alt_setpoint', 0.0),
        'alt_vel_target': sample.get('alt_vel_target', 0.0),
        'climb_rate_cmd': sample.get('climb_rate_cmd', 0.0),
        'pos_setpoint_x': sample.get('pos_setpoint_x', 0.0),
        'pos_setpoint_y': sample.get('pos_setpoint_y', 0.0),
    }


def _baro_row(sample: dict) -> dict:
    return {
        'timestamp_us': sample['timestamp_us'],
        'altitude': sample['baro_altitude'],
        'pressure': sample['baro_pressure'] * HPA_TO_PA,
    }


def _tof_row(sample: dict) -> dict:
    return {
        'timestamp_us': sample['timestamp_us'],
        'distance': sample['tof_distance'],
        'status': sample['tof_status'],
    }


def _flow_row(sample: dict) -> dict:
    return {
        'timestamp_us': sample['timestamp_us'],
        'dx': sample['flow_x'],
        'dy': sample['flow_y'],
        'quality': sample['flow_quality'],
    }


def _mag_row(sample: dict) -> dict:
    return {
        'timestamp_us': sample['timestamp_us'],
        'x': sample['mag_x'], 'y': sample['mag_y'], 'z': sample['mag_z'],
    }


def _status_row(sample: dict) -> dict:
    """Status (0x4F) -> v1 status.csv row. `sensor_health`/`reset_reason`
    are decoded for every wire length (17/53/57B, see parse_packet()) so
    are read directly; `current_ma` (57B only) and the 9 PID gains (53/57B
    only) fall back to None -- absent, not invented (plan section 2.2).
    Status (0x4F) -> v1 status.csv の行。`sensor_health`/`reset_reason` は
    電文長(17/53/57B、parse_packet() 参照)に関わらず常にデコードされるため
    直接読む。`current_ma`（57Bのみ）とPIDゲイン9個（53/57Bのみ）は
    無ければ None（無いものは無い、計画書2.2節）。
    """
    row = {
        'timestamp_us': sample['timestamp_us'],
        'uptime_ms': sample['uptime_ms'],
        'voltage': sample['voltage'],
        'current_ma': sample.get('current_ma'),
        'flight_state': sample['flight_state'],
        'sensor_health': sample['sensor_health'],
        'eskf_status': sample['eskf_status'],
        'reset_reason': sample['reset_reason'],
    }
    for axis in _STATUS_PID_AXES:
        for term in _STATUS_PID_TERMS:
            row[f'pid_{axis}_{term}'] = sample.get(f'pid_{axis}_{term}')
    return row


def _tof_diagnostics_row(sample: dict) -> dict:
    """Status v4 sample -> v1 tof_diagnostics.csv row. / ステータス v4 のサンプル -> tof_diagnostics.csv の行。"""
    row = {'timestamp_us': sample['timestamp_us']}
    for side in ('bottom', 'front'):
        for field in TOF_DIAGNOSTIC_FIELDS:
            row[f'{side}_{field}'] = sample[f'{side}_{field}']
    return row


def _loop_timing_row(sample: dict) -> dict:
    """Status v4 sample -> v1 loop_timing.csv row (both tasks side by side). / ステータス v4 のサンプル -> loop_timing.csv の行（2 タスクを横に並べる）。"""
    row = {'timestamp_us': sample['timestamp_us']}
    for task in ('imu', 'control'):
        for field in LOOP_TIMING_FIELDS:
            row[f'{task}_{field}'] = sample[f'{task}_{field}']
    return row


def _nonfinite_row(sample: dict) -> dict:
    """Status v4 sample -> v1 nonfinite.csv row. / ステータス v4 のサンプル -> nonfinite.csv の行。"""
    row = {'timestamp_us': sample['timestamp_us']}
    for field in NONFINITE_FIELDS + ('first_stage',):
        row[field] = sample[field]
    return row


def _link_health_row(sample: dict) -> dict:
    """Status v4 sample -> v1 link_health.csv row. / ステータス v4 のサンプル -> link_health.csv の行。"""
    row = {'timestamp_us': sample['timestamp_us']}
    for field in LINK_HEALTH_FIELDS:
        row[field] = sample[field]
    return row


def _text_log_row(sample: dict) -> dict:
    """Text-log sample -> v1 text_log.csv row. The line's trailing newline is dropped (one cell = one line).
    テキストログのサンプル -> text_log.csv の行。行末の改行は落とす（1 セル = 1 行）。
    """
    text = sample['text']
    if text.endswith('\n'):
        text = text[:-1]
    return {
        'timestamp_us': sample['timestamp_us'],
        'truncated': sample['truncated'],
        'dropped_before': sample['dropped_before'],
        'text': text,
    }



# =============================================================================
# UDP Capture class
# UDP キャプチャクラス
# =============================================================================

class UDPTelemetryCapture:
    """Captures UDP telemetry from StampFly and saves it as a StampFly
    flight-log v1 bundle (save_bundle(); docs/plans/flight-log-format-plan.md).
    StampFly から UDP テレメトリをキャプチャし、StampFly フライトログ v1
    一式として保存する（save_bundle()；計画書参照）。
    """

    def __init__(self, vehicle_ip: str = '192.168.10.1', port: int = UDP_LOG_PORT):
        self.vehicle_ip = vehicle_ip
        self.port = port
        self.sock = None
        self.running = False

        # Per-sensor sample storage
        # センサごとのサンプル格納
        self.samples = defaultdict(list)  # {pkt_id: [sample_dict, ...]}

        # Statistics
        self.packet_count = defaultdict(int)
        self.sample_count = defaultdict(int)
        self.bytes_received = 0
        self.checksum_errors = 0
        self.seq_gaps = defaultdict(int)
        self.last_seq = {}
        self.start_time = 0
        self.end_time = 0

        # Unified-packet (0x50) 16-bit header sequence, unwrapped into an
        # ever-increasing integer -- see _unwrap_unified_seq(). None until
        # the first unified packet arrives.
        # 統合パケット(0x50)の16bitヘッダ sequence を単調増加の整数へ展開した
        # 状態 -- _unwrap_unified_seq() 参照。最初の統合パケット受信までは None。
        self._unified_seq_last_raw = None
        self._unified_seq_wrap_offset = 0

        # Text-log packets (0x51) carry their own 16-bit sequence. Unlike the unified packet it is unwrapped
        # relative to the HIGHEST sequence seen (a signed 16-bit difference), so a packet delivered late (out of
        # order) is placed before its neighbours instead of being mistaken for a wrap. Duplicated datagrams are
        # dropped; the packets the vehicle sent but we never saw are counted by text_packets_lost().
        # テキストログのパケット（0x51）は専用の 16 ビットの通し番号を持つ。統合パケットと違い、見た「最大」の通し番号を
        # 基準に（16 ビットの符号付き差で）展開するので、遅れて届いたパケット（順序の入れ替わり）は巻き戻りと取り違えず
        # 近傍の手前に置く。重複して届いたデータグラムは捨てる。機体が送ったのに見ていないパケットは text_packets_lost()
        # で数える。
        self._text_seq_highest = None
        self._text_seq_seen = set()
        self.text_duplicate_packets = 0

    def start(self):
        """Create socket, send start command, begin capture."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)

        # Bind to receive responses
        # 応答を受信するためにバインド
        self.sock.bind(('', 0))  # OS picks a free port

        # Send start command
        # 開始コマンド送信
        self.sock.sendto(bytes([CMD_START_LOG]), (self.vehicle_ip, self.port))
        self.running = True
        self.start_time = time.time()

    def stop(self):
        """Send stop command and close socket."""
        if self.sock and self.running:
            try:
                self.sock.sendto(bytes([CMD_STOP_LOG]), (self.vehicle_ip, self.port))
            except Exception:
                pass
            self.running = False
            self.end_time = time.time()

    def _heartbeat_thread(self):
        """Send heartbeat every 2 seconds."""
        while self.running:
            try:
                self.sock.sendto(bytes([CMD_HEARTBEAT]), (self.vehicle_ip, self.port))
            except Exception:
                pass
            time.sleep(2.0)

    def capture(self, duration: float, progress_cb=None) -> bool:
        """Run capture for specified duration.
        Returns True on success."""

        self.start()

        # Start heartbeat thread
        # ハートビートスレッド開始
        hb_thread = threading.Thread(target=self._heartbeat_thread, daemon=True)
        hb_thread.start()

        deadline = time.time() + duration

        try:
            while time.time() < deadline and self.running:
                try:
                    data, addr = self.sock.recvfrom(2048)
                except socket.timeout:
                    continue

                self._process_datagram(data)

                # Progress callback
                if progress_cb:
                    elapsed = time.time() - self.start_time
                    progress_cb(elapsed, duration, sum(self.sample_count.values()))

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

        return sum(self.sample_count.values()) > 0

    def _unwrap_unified_seq(self, raw_seq: int) -> int:
        """Unwrap the unified packet's 16-bit header `sequence` into an
        ever-increasing integer, by adding 65536 each time the wire value
        wraps back down (raw_seq < the previous raw_seq). This is the basis
        of the v1 flight-log `seq` column (protocol/spec/flight_log.yaml):
        `seq = unwrapped_sequence * 8 + index_in_packet`. A lost packet
        still shows up as a forward JUMP in the unwrapped value (never
        masked by this unwrapping) -- `sf log check` reports that as a
        `seq` gap.
        統合パケットの16bitヘッダ `sequence` を、巻き戻る（raw_seq が直前より
        小さくなる）たびに65536を足すことで単調増加の整数へ展開する。これが
        v1 フライトログの `seq` 列（protocol/spec/flight_log.yaml）の元:
        `seq = 展開後のsequence * 8 + パケット内インデックス`。パケット欠落は
        展開後の値の前方への飛びとして残る（この展開で隠れない）--
        `sf log check` はそれを `seq` の飛びとして報告する。
        """
        if self._unified_seq_last_raw is not None and raw_seq < self._unified_seq_last_raw:
            self._unified_seq_wrap_offset += 0x10000
        self._unified_seq_last_raw = raw_seq
        return self._unified_seq_wrap_offset + raw_seq

    def _unwrap_text_seq(self, raw_seq: int) -> int:
        """Unwrap a text-log packet's 16-bit sequence relative to the highest sequence seen so far
        (see __init__): the signed 16-bit difference to it, so late packets get a smaller number than
        the highest one and a real wrap gets a larger one. The vehicle starts at 0 with every capture.
        テキストログのパケットの 16 ビットの通し番号を、ここまでに見た最大の通し番号を基準に展開する
        （__init__ 参照）: それとの符号付き 16 ビット差を使うので、遅れたパケットは最大より小さく、本当の巻き戻りは
        大きい番号になる。機体は取得のたびに 0 から始める。
        """
        if self._text_seq_highest is None:
            self._text_seq_highest = raw_seq
            return raw_seq
        difference = ((raw_seq - self._text_seq_highest + 0x8000) & 0xFFFF) - 0x8000
        unwrapped = self._text_seq_highest + difference
        if unwrapped > self._text_seq_highest:
            self._text_seq_highest = unwrapped
        return unwrapped

    def text_packets_lost(self) -> int:
        """Text-log packets the vehicle sent that never arrived: the sequence range 0..highest (the
        vehicle starts every capture at 0) minus the distinct sequences seen.
        機体が送ったのに届かなかったテキストログのパケット数: 通し番号の範囲 0〜最大（機体は取得のたびに 0 から始める）から、
        届いた異なる通し番号の数を引いたもの。
        """
        if not self._text_seq_seen:
            return 0
        return max(self._text_seq_seen) + 1 - len(self._text_seq_seen)

    def text_lines_dropped_by_vehicle(self) -> int:
        """Lines the vehicle's log sink refused (ring full), the exact total from the latest status v4
        packet; falls back to the sum of the per-line `dropped_before` (saturating at 255) when no v4
        status arrived.
        機体のログシンクが断った（リングが満杯）行数。最新のステータス v4 パケットの正確な合計。v4 のステータスが届いて
        いなければ、行ごとの `dropped_before`（255 で飽和）の総和で代える。
        """
        for sample in reversed(self.samples.get(PKT_STATUS, [])):
            if 'text_dropped_lines' in sample:
                return sample['text_dropped_lines']
        return sum(s['dropped_before'] for s in self.samples.get(PKT_TEXT_LOG, []))

    def _process_datagram(self, data: bytes) -> None:
        """Handle one received UDP datagram: parse it, store its samples,
        and update packet-level statistics (bytes, sample/packet counts,
        sequence-gap detection, unified-packet `seq` unwrapping). Split out
        of capture()'s receive loop so tests can feed synthetic datagrams
        without a real socket (see test_udp_capture_bundle.py).
        受信した UDP データグラム1個を処理する: パースし、サンプルを格納し、
        パケット単位の統計（バイト数・サンプル/パケット数・シーケンスギャップ
        検出・統合パケットの `seq` 展開）を更新する。capture() の受信ループ
        から分離し、テストが実ソケット無しで合成データグラムを投入できる
        ようにした（test_udp_capture_bundle.py 参照）。
        """
        self.bytes_received += len(data)

        results = parse_packet(data)
        if not results:
            self.checksum_errors += 1
            return

        # Header sequence (16-bit, wraps at 65536) -- read once and reused
        # below both to unwrap the unified packet's `seq` and for the
        # generic per-packet-type gap (lost-packet) detection.
        # ヘッダの sequence（16bit、65536で巻き戻る）-- 1回だけ読み、下の
        # 統合パケット `seq` 展開とパケット種別ごとの汎用ギャップ（欠落）
        # 検出の両方に使い回す。
        pkt_id, header_seq, _count = struct.unpack_from(FMT_HEADER, data, 0)

        # Unified packet (0x50): unwrap its header sequence once, then turn
        # every lockstep sub-sample's `_idx` (0..7, set by parse_packet())
        # into the v1 `seq` column -- see UNIFIED_SEQ_PACKET_TYPES and
        # _unwrap_unified_seq()'s docstring.
        # 統合パケット(0x50): ヘッダ sequence を1回展開し、ロックステップ系
        # サブサンプルの `_idx`（0..7、parse_packet() が設定）を v1 の `seq`
        # 列へ変換する -- UNIFIED_SEQ_PACKET_TYPES と
        # _unwrap_unified_seq() のドキュメント参照。
        unwrapped_seq = None
        if pkt_id == PKT_UNIFIED:
            unwrapped_seq = self._unwrap_unified_seq(header_seq)

        # Text-log packet (0x51): its own sequence. A duplicate datagram is dropped; every line gets an
        # order key (unwrapped sequence, index in the packet) so save_bundle() restores the vehicle's order
        # even when packets arrived out of order.
        # テキストログのパケット（0x51）: 専用の通し番号。重複したデータグラムは捨てる。各行には順序のキー
        # （展開した通し番号、パケット内の位置）を付け、パケットが順序を崩して届いても save_bundle() が機体の
        # 順序を復元できるようにする。
        if pkt_id == PKT_TEXT_LOG:
            text_seq = self._unwrap_text_seq(header_seq)
            if text_seq in self._text_seq_seen:
                self.text_duplicate_packets += 1
                return
            self._text_seq_seen.add(text_seq)
            for _sub_pkt_id, sample in results:
                sample['_order'] = text_seq * TEXT_ORDER_STRIDE + sample.pop('_idx')

        for sub_pkt_id, sample in results:
            if unwrapped_seq is not None and sub_pkt_id in UNIFIED_SEQ_PACKET_TYPES:
                sample['seq'] = unwrapped_seq * 8 + sample.pop('_idx')
            self.sample_count[sub_pkt_id] += 1
            self.samples[sub_pkt_id].append(sample)

        # Track packet-level stats
        self.packet_count[pkt_id] += 1

        # Sequence gap detection (packets lost). Also applied to the unified
        # packet (0x50): vehicle implements its sequence counter, so a lost
        # unified packet (= 8 lost 400Hz samples) is now detectable.
        # Guard: the legacy vehicle_old firmware sends seq=0 on every unified
        # packet (unimplemented TODO) — skip while seq stays 0 so old
        # captures do not report bogus gaps.
        # シーケンスギャップ検出（パケット欠落）。統合パケット(0x50)にも適用:
        # vehicle はシーケンスカウンタを実装済みで、統合パケット1個の欠落
        # (=400Hzサンプル8個の欠落)を検出できる。ガード: 旧 vehicle_old
        # ファームは統合パケットの seq が常に0(未実装TODO)のため、seq が0の
        # ままの間は検出をスキップし旧キャプチャで偽ギャップを報告しない。
        if pkt_id in SAMPLE_INFO or pkt_id == PKT_UNIFIED:
            if pkt_id in self.last_seq:
                last = self.last_seq[pkt_id]
                if not (header_seq == 0 and last == 0):   # legacy seq=0 guard
                    expected = (last + 1) & 0xFFFF
                    if header_seq != expected:
                        gap = (header_seq - expected) & 0xFFFF
                        self.seq_gaps[pkt_id] += gap
            self.last_seq[pkt_id] = header_seq

    def print_stats(self):
        """Print capture statistics with detailed timing analysis.
        詳細タイミング分析付きキャプチャ統計を表示。
        """
        import math

        duration = (self.end_time or time.time()) - self.start_time
        total_samples = sum(self.sample_count.values())

        print(f"\n{'='*70}")
        print(f"  UDP Telemetry Capture Statistics")
        print(f"{'='*70}")
        print(f"  Duration:     {duration:.1f}s")
        print(f"  Total bytes:  {self.bytes_received:,}")
        print(f"  Bandwidth:    {self.bytes_received / duration / 1024:.1f} KB/s")
        print(f"  Checksum err: {self.checksum_errors}")
        # Unified-packet (0x50) losses -- each one is 8 lost 400Hz samples
        # and shows up as a gap in the v1 flight-log `seq` column
        # (protocol/spec/flight_log.yaml; sf log check reports it as a
        # warning). Already folded into the "TOTAL ... Gaps" row further
        # down, but that row mixes it with any other packet type's gaps --
        # called out here on its own since `seq` is entirely derived from it.
        # 統合パケット(0x50)の欠落 -- 1個につき400Hzサンプル8個の欠落で、
        # v1フライトログの `seq` 列の飛びとして現れる
        # （protocol/spec/flight_log.yaml；sf log check が warning で報告）。
        # 下の「TOTAL ... Gaps」行にも合算済みだが、そちらは他パケット種別の
        # ギャップと混ざるため、`seq` が丸ごとこれ由来であることをここで
        # 明示する。
        print(f"  Pkts lost:    {self.seq_gaps.get(PKT_UNIFIED, 0)} (unified packet 0x50, seq gaps)")
        print()

        # Per-sensor statistics
        # センサごとの統計
        # Column widths: name=10, samples=7, hz=7, hz=7, hz=7, hz=7, std=6, loss=6, gaps=5
        W = '  {:<10s} {:>7s} {:>7s} {:>7s} {:>7s} {:>7s} {:>6s} {:>6s} {:>5s}'
        D = '  {:<10s} {:>7d} {:>7s} {:>7s} {:>7s} {:>7s} {:>6s} {:>6s} {:>5d}'

        print(W.format('Type', 'Samples', 'MedHz', 'AvgHz', 'MinHz', 'MaxHz',
                        'StdMs', 'Loss', 'Gaps'))
        print(W.format('─'*10, '─'*7, '─'*7, '─'*7, '─'*7, '─'*7,
                        '─'*6, '─'*6, '─'*5))

        for pkt_id, (name, _, _) in sorted(SAMPLE_INFO.items()):
            samps = self.sample_count.get(pkt_id, 0)
            gaps = self.seq_gaps.get(pkt_id, 0)
            samples = self.samples.get(pkt_id, [])

            if len(samples) < 2:
                if samps > 0:
                    print(D.format(name, samps, '-', '-', '-', '-', '-', '-', gaps))
                continue

            # Compute intervals from timestamps
            # タイムスタンプからインターバルを計算
            timestamps = [s['timestamp_us'] for s in samples]
            all_intervals = [timestamps[i+1] - timestamps[i]
                             for i in range(len(timestamps)-1)
                             if timestamps[i+1] > timestamps[i]]

            if not all_intervals:
                print(D.format(name, samps, '-', '-', '-', '-', '-', '-', gaps))
                continue

            # Filter out packet-loss gaps: intervals > 3× median are likely
            # caused by dropped packets, not actual sensor timing variation.
            # パケットロスによるギャップを除外: 中央値の3倍を超える間隔は
            # センサのタイミング変動ではなくパケット欠損と判断。
            all_intervals.sort()
            median_us = all_intervals[len(all_intervals) // 2]
            threshold = median_us * 3
            intervals = [x for x in all_intervals if x <= threshold]

            if not intervals:
                intervals = all_intervals  # fallback

            # Statistics on filtered intervals (true sensor timing)
            # フィルタ後の間隔で統計（真のセンサタイミング）
            avg_us = sum(intervals) / len(intervals)
            min_us = min(intervals)
            max_us = max(intervals)
            variance = sum((x - avg_us) ** 2 for x in intervals) / len(intervals)
            std_ms = math.sqrt(variance) / 1000.0

            med_hz = 1e6 / median_us if median_us > 0 else 0
            avg_hz = 1e6 / avg_us if avg_us > 0 else 0
            min_hz = 1e6 / max_us if max_us > 0 else 0  # min freq = max period
            max_hz = 1e6 / min_us if min_us > 0 else 0  # max freq = min period

            # Packet loss rate
            # パケットロス率
            expected = samps + gaps
            loss_pct = (gaps / expected * 100) if expected > 0 else 0

            print(D.format(name, samps,
                           f'{med_hz:.1f}', f'{avg_hz:.1f}',
                           f'{min_hz:.1f}', f'{max_hz:.1f}',
                           f'{std_ms:.2f}', f'{loss_pct:.1f}%', gaps))

        print(W.format('─'*10, '─'*7, '─'*7, '─'*7, '─'*7, '─'*7,
                        '─'*6, '─'*6, '─'*5))
        total_gaps = sum(self.seq_gaps.values())
        total_expected = total_samples + total_gaps
        total_loss = (total_gaps / total_expected * 100) if total_expected > 0 else 0
        print(D.format('TOTAL', total_samples, '', '', '', '',
                        '', f'{total_loss:.1f}%', total_gaps))
        print()

        # 400Hz motor duty entry (0x4A) presence -- plant input for
        # `sf sysid fit`, saved to the bundle's motor.csv. Old firmware
        # without it leaves motor.csv absent (schema.STREAMS: optional) and
        # needs --kp for the fit.
        # 400Hz モータduty エントリ（0x4A）の有無 -- `sf sysid fit` のプラント
        # 入力で、一式の motor.csv に保存される。無い旧ファームは motor.csv が
        # 存在せず（schema.STREAMS: 任意）、フィットには --kp が必要になる。
        duty_samples = self.sample_count.get(PKT_DUTY400, 0)
        if duty_samples > 0:
            print(f"  400Hz motor duty (0x4A): present ({duty_samples} samples) "
                  f"-- `sf sysid fit` can use --input duty")
        else:
            print("  400Hz motor duty (0x4A): NOT present -- "
                  "`sf sysid fit` falls back to --kp reconstruction")

        # 400Hz control_output entry (0x4B) presence -- the PRE-MIXER
        # commanded thrust+torque, mixer-agnostic plant input for
        # `sf sysid fit`/`rate-fit`. Absent is not an error: falls back to
        # the duty-based (mixer-specific) reconstruction above.
        # 400Hz control_output エントリ（0x4B）の有無 -- ミキサー手前の
        # 指令推力+トルク、`sf sysid fit`/`rate-fit` のミキサー非依存な
        # プラント入力。無くてもエラーではなく、上の duty ベース
        # （ミキサー依存）の復元にフォールバックする。
        ctrl_output_samples = self.sample_count.get(PKT_CTRL_OUTPUT400, 0)
        if ctrl_output_samples > 0:
            print(f"  400Hz control_output (0x4B): present ({ctrl_output_samples} samples) "
                  f"-- `sf sysid fit` can read u(t) directly, no --mixer needed")
        else:
            print("  400Hz control_output (0x4B): NOT present -- "
                  "`sf sysid fit` falls back to duty-based (--mixer) reconstruction")

        # 400Hz flight phase (0x4C) / 50Hz flight flags (0x4D): flip analysis
        # aids. Absent on firmware older than 2026-10-02 (informational only).
        # 400Hz 飛行フェーズ（0x4C）/ 50Hz 飛行フラグ（0x4D）: フリップ解析用。
        # 2026-10-02 より古いファームには無い（情報表示のみ）。
        for label, pkt in (('flight phase', PKT_FLIGHT_PHASE400),
                           ('flight flags', PKT_FLIGHT_FLAGS),
                           ('ToF silence', PKT_TOF_SILENCE)):
            n = self.sample_count.get(pkt, 0)
            state = f'present ({n} samples)' if n else 'NOT present (older firmware)'
            print(f"  {label} (0x{pkt:02X}): {state}")

        # 1Hz Status packet (0x4F) presence -- source of status.csv's
        # `voltage` column, needed only by `sf sysid fit --mixer vehicle`
        # (actuator.cpp's nonlinear thrust-to-duty motor curve is
        # voltage-dependent). Absent logs still work with --mixer vehicle --
        # plant_fit.py falls back to the nominal 1S LiPo voltage -- so this
        # is informational, not an error.
        # 1Hz Status パケット（0x4F）の有無 -- status.csv の `voltage` 列の元。
        # `sf sysid fit --mixer vehicle`（actuator.cpp の電圧依存の非線形
        # thrust→duty モータ曲線）だけが必要とする。無くても --mixer vehicle は
        # 動く（plant_fit.py が公称1S LiPo電圧にフォールバックする）ので、
        # これはエラーではなく情報表示。
        status_samples = self.sample_count.get(PKT_STATUS, 0)
        if status_samples > 0:
            print(f"  1Hz Status (0x4F): present ({status_samples} samples) "
                  f"-- voltage column available for `sf sysid fit --mixer vehicle`")
        else:
            print("  1Hz Status (0x4F): NOT present -- "
                  "`sf sysid fit --mixer vehicle` falls back to nominal battery voltage")
        self._print_diagnostics_stats()
        print()

    def _print_diagnostics_stats(self):
        """Print the status v4 diagnostics (the last 1 Hz values) and the text-log summary: the loop-timing
        counters of both 400 Hz tasks, the non-finite counters, the loss counters of the log paths, and how many
        text lines arrived and were lost. Prints a one-line hint instead when the vehicle sends no v4 status.
        ステータス v4 の診断値（最後の 1 Hz の値）とテキストログの要約を表示する: 2 つの 400 Hz タスクのループ周期の数え上げ、
        有限でない数の数え上げ、ログ経路の欠落の計数、届いたテキストの行数と失われた行数。機体が v4 のステータスを送らない
        ときは 1 行の案内を出す。
        """
        v4_samples = [s for s in self.samples.get(PKT_STATUS, []) if 'imu_cycles' in s]
        if not v4_samples:
            print("  Diagnostics (status v4, 0x4F 224B): NOT present (older firmware)")
        else:
            last = v4_samples[-1]
            for task in ('imu', 'control'):
                print(f"  {task:<7s} loop: cycles={last[task + '_cycles']} late={last[task + '_late_count']} "
                      f"period_max={last[task + '_period_max_us']}us busy_max={last[task + '_busy_max_us']}us "
                      f"over_budget={last[task + '_over_budget_count']} | armed: late={last[task + '_armed_late_count']} "
                      f"period_max={last[task + '_armed_period_max_us']}us "
                      f"busy_max={last[task + '_armed_busy_max_us']}us")
            print(f"  nonfinite: detected={last['detected_cycles']} of {last['checked_cycles']} armed cycles "
                  f"(first stage {last['first_stage']}, 255 = none)")
            print(f"  log paths: log_stream_overflow={last['log_stream_overflow']} send_drops={last['send_drops']} "
                  f"entry_drops={last['entry_drops']} text_dropped_lines={last['text_dropped_lines']}")
            for side in ('bottom', 'front'):
                print(f"  ToF {side:<6s}: range_status={last[side + '_range_status']} "
                      f"signal={last[side + '_signal_rate']:.2f}Mcps ambient={last[side + '_ambient_rate']:.2f}Mcps "
                      f"sigma={last[side + '_sigma_mm']:.1f}mm")
        lines = self.sample_count.get(PKT_TEXT_LOG, 0)
        if lines or self.packet_count.get(PKT_TEXT_LOG):
            print(f"  Text log (0x51): {lines} lines in {self.packet_count.get(PKT_TEXT_LOG, 0)} packets, "
                  f"{self.text_packets_lost()} packets lost on the way, "
                  f"{self.text_lines_dropped_by_vehicle()} lines dropped by the vehicle, "
                  f"{self.text_duplicate_packets} duplicate packets")
        else:
            print("  Text log (0x51): NOT present (no line logged during the capture, or older firmware)")

    def save_bundle(
        self,
        path,
        *,
        source: str = 'vehicle',
        tool_name: str = 'udp_capture.py',
        tool_version: str = _TOOL_VERSION,
        capture_info: dict = None,
        notes: str = None,
    ):
        """Save the captured samples as a StampFly flight-log v1 bundle
        (docs/plans/flight-log-format-plan.md section 2): one CSV per
        stream written into `path` (a `.sflog.zip` file, or a directory --
        see sflog.FlightLog.save()).

        Builds exactly the v1 streams that received at least one sample
        (schema.STREAMS name -> source packet id): imu/attitude share
        PKT_IMU_ESKF; posvel/rate_ref/motor/ctrl_output/pilot/ctrl_ref/
        baro/tof_bottom/tof_front/flow/mag/status/eskf_cov each come from
        their own packet id. A stream with zero samples is simply omitted,
        never written as an empty file (plan section 2.2: "パケットが無い
        ストリームのファイルは作らない"). Row values are converted to v1's
        physical units/column names by this module's `_*_row()` builders
        (mostly a subset-and-rename of parse_packet()'s already-decoded
        fields; three still apply a wire scale/unit conversion -- see the
        row-builders' section docstring above).

        Rows are ordered by capture (arrival) order, stable-sorted: by
        `seq` for the 6 lockstep streams (schema.LOCKSTEP_STREAMS) --
        `seq` uniquely and monotonically identifies a control cycle, unlike
        `timestamp_us` which legitimately repeats (plan section 2.2) -- and
        by `timestamp_us` for every other stream.

        Args:
            path: bundle path (`.sflog.zip` file or directory).
            source: meta.json `source` ("vehicle" for a real capture).
            tool_name, tool_version: meta.json `tool.name`/`tool.version`.
            capture_info: extra meta.json `capture` fields (e.g.
                `{"requested_duration_s": args.duration}`), MERGED on top
                of this method's own capture stats (`ip`, `port`,
                `actual_duration_s`, `packets_received`, `packets_lost`) --
                the caller only needs to supply what it alone knows (this
                method has no idea what duration the caller originally
                asked for).
            notes: optional free-form string, appended to this method's own
                default note about the `seq` column's definition.

        Returns:
            The sflog.FlightLog that was written.

        キャプチャしたサンプルを StampFly フライトログ v1 一式として保存する
        （計画書 2節）: `path`（`.sflog.zip` かディレクトリ）にストリーム
        ごとの CSV を1個ずつ書く。

        引数・戻り値の詳細は英語側を参照。capture_info はこのメソッド自身が
        持つキャプチャ統計（ip・port・実測秒数・受信/欠落パケット数）の上に
        マージする（呼び出し側だけが持つ情報 -- 例えば要求秒数 -- だけを
        渡せばよい。このメソッド自身は呼び出し側が何秒を要求したか判別できない）。
        """
        streams = {}

        if self.samples.get(PKT_IMU_ESKF):
            imu_samples = self.samples[PKT_IMU_ESKF]
            streams['imu'] = self._build_stream(
                [_select(s, _IMU_KEYS) for s in imu_samples], 'seq')
            streams['attitude'] = self._build_stream(
                [_select(s, _ATTITUDE_KEYS) for s in imu_samples], 'seq')
        if self.samples.get(PKT_POS_VEL):
            streams['posvel'] = self._build_stream(
                [_select(s, _POSVEL_KEYS) for s in self.samples[PKT_POS_VEL]], 'seq')
        if self.samples.get(PKT_RATE_REF):
            streams['rate_ref'] = self._build_stream(
                [_rate_ref_row(s) for s in self.samples[PKT_RATE_REF]], 'seq')
        if self.samples.get(PKT_DUTY400):
            streams['motor'] = self._build_stream(
                [_select(s, _MOTOR_KEYS) for s in self.samples[PKT_DUTY400]], 'seq')
        if self.samples.get(PKT_CTRL_OUTPUT400):
            streams['ctrl_output'] = self._build_stream(
                [_ctrl_output_row(s) for s in self.samples[PKT_CTRL_OUTPUT400]], 'seq')
        if self.samples.get(PKT_FLIGHT_PHASE400):
            streams['flight_phase'] = self._build_stream(
                [_select(s, _FLIGHT_PHASE_KEYS) for s in self.samples[PKT_FLIGHT_PHASE400]], 'seq')
        if self.samples.get(PKT_FLIGHT_FLAGS):
            streams['flight_flags'] = self._build_stream(
                [_select(s, _FLIGHT_FLAGS_KEYS) for s in self.samples[PKT_FLIGHT_FLAGS]], 'timestamp_us')
        if self.samples.get(PKT_TOF_SILENCE):
            streams['tof_silence'] = self._build_stream(
                [_select(s, _TOF_SILENCE_KEYS) for s in self.samples[PKT_TOF_SILENCE]], 'timestamp_us')
        if self.samples.get(PKT_CONTROL):
            streams['pilot'] = self._build_stream(
                [_pilot_row(s) for s in self.samples[PKT_CONTROL]], 'timestamp_us')
        if self.samples.get(PKT_CTRL_REF):
            streams['ctrl_ref'] = self._build_stream(
                [_ctrl_ref_row(s) for s in self.samples[PKT_CTRL_REF]], 'timestamp_us')
        if self.samples.get(PKT_BARO):
            streams['baro'] = self._build_stream(
                [_baro_row(s) for s in self.samples[PKT_BARO]], 'timestamp_us')
        if self.samples.get(PKT_TOF_BOTTOM):
            streams['tof_bottom'] = self._build_stream(
                [_tof_row(s) for s in self.samples[PKT_TOF_BOTTOM]], 'timestamp_us')
        if self.samples.get(PKT_TOF_FRONT):
            streams['tof_front'] = self._build_stream(
                [_tof_row(s) for s in self.samples[PKT_TOF_FRONT]], 'timestamp_us')
        if self.samples.get(PKT_FLOW):
            streams['flow'] = self._build_stream(
                [_flow_row(s) for s in self.samples[PKT_FLOW]], 'timestamp_us')
        if self.samples.get(PKT_MAG):
            streams['mag'] = self._build_stream(
                [_mag_row(s) for s in self.samples[PKT_MAG]], 'timestamp_us')
        if self.samples.get(PKT_STATUS):
            streams['status'] = self._build_stream(
                [_status_row(s) for s in self.samples[PKT_STATUS]], 'timestamp_us')
            # The status packet v4 carries four more streams; a bundle from a v1-v3 vehicle simply lacks them.
            # ステータスパケット v4 は 4 つのストリームを運ぶ。v1〜v3 の機体の一式にはそれらが単に無い。
            v4_samples = [s for s in self.samples[PKT_STATUS] if 'imu_cycles' in s]
            if v4_samples:
                streams['tof_diagnostics'] = self._build_stream(
                    [_tof_diagnostics_row(s) for s in v4_samples], 'timestamp_us')
                streams['loop_timing'] = self._build_stream(
                    [_loop_timing_row(s) for s in v4_samples], 'timestamp_us')
                streams['nonfinite'] = self._build_stream(
                    [_nonfinite_row(s) for s in v4_samples], 'timestamp_us')
                streams['link_health'] = self._build_stream(
                    [_link_health_row(s) for s in v4_samples], 'timestamp_us')
        if self.samples.get(PKT_TEXT_LOG):
            # Vehicle order: (unwrapped packet sequence, index in the packet), not arrival order, not time.
            # 機体の順序: （展開した通し番号, パケット内の位置）。到着順でも時刻順でもない。
            text_rows = [dict(_text_log_row(s), _order=s['_order']) for s in self.samples[PKT_TEXT_LOG]]
            streams['text_log'] = self._build_stream(text_rows, '_order').drop(columns='_order')
        if self.samples.get(PKT_ESKF_PDIAG):
            streams['eskf_cov'] = self._build_stream(
                [_select(s, _ESKF_COV_KEYS) for s in self.samples[PKT_ESKF_PDIAG]], 'timestamp_us')

        merged_capture = {
            'ip': self.vehicle_ip,
            'port': self.port,
            'actual_duration_s': (self.end_time or time.time()) - self.start_time,
            'packets_received': sum(self.packet_count.values()),
            'packets_lost': self.seq_gaps.get(PKT_UNIFIED, 0),
        }
        if self.packet_count.get(PKT_TEXT_LOG):
            # What the text log lost on the way and at the vehicle (see print_stats()).
            # テキストログが途中と機体で失った分（print_stats() 参照）。
            merged_capture['text_packets_lost'] = self.text_packets_lost()
            merged_capture['text_lines_dropped_by_vehicle'] = self.text_lines_dropped_by_vehicle()
        if capture_info:
            merged_capture.update(capture_info)

        seq_note = 'seq = unified packet sequence x 8 + index (unwrapped)'
        combined_notes = f'{seq_note}; {notes}' if notes else seq_note

        meta = sflog.make_meta(
            source=source,
            tool_name=tool_name,
            tool_version=tool_version,
            capture=merged_capture,
            notes=combined_notes,
            streams=streams,
        )
        schema_json = sflog.schema.schema_for(streams.keys())
        log = sflog.FlightLog(meta=meta, schema=schema_json, streams=streams)
        log.save(path)

        print(f"  Saved: {path} ({len(streams)} streams)")
        return log

    @staticmethod
    def _build_stream(rows: list, sort_key: str):
        """Build one stream's DataFrame from a list of v1-column row dicts,
        stable-sorted by `sort_key` to satisfy the "capture order" rule
        (docs/plans/flight-log-format-plan.md section 2.2) even if UDP
        delivered datagrams slightly out of order: `seq` for the 6 lockstep
        streams (uniquely identifies a control cycle), `timestamp_us` for
        every other stream.
        v1列名の行dictのリストから1ストリームのDataFrameを作る。「捕捉順」
        規約（計画書2.2節）を満たすため `sort_key` で安定ソートする -- UDPの
        データグラムがわずかに順序を崩して届いても対応できる: 6つの
        ロックステップ系ストリームは `seq`（制御周期を一意に識別）、他の
        ストリームは `timestamp_us`。
        """
        return pd.DataFrame(rows).sort_values(sort_key, kind='stable').reset_index(drop=True)


# =============================================================================
# Progress bar
# プログレスバー
# =============================================================================

def progress_bar(elapsed: float, duration: float, total_samples: int):
    """Print progress bar to stderr."""
    pct = min(elapsed / duration * 100, 100) if duration > 0 else 0
    bar_len = 30
    filled = int(bar_len * pct / 100)
    bar = '█' * filled + '░' * (bar_len - filled)
    sys.stderr.write(f"\r  [{bar}] {pct:5.1f}%  {total_samples:>6} samples  {elapsed:.1f}s/{duration:.0f}s")
    sys.stderr.flush()


# =============================================================================
# CLI entry point
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="UDP telemetry capture for StampFly",
    )
    parser.add_argument(
        '-o', '--output',
        help="Output bundle path (.sflog.zip; auto-generated if not specified)",
    )
    parser.add_argument('-d', '--duration', type=float, default=30.0, help="Capture duration in seconds (default: 30)")
    parser.add_argument('-i', '--ip', default='192.168.10.1', help="StampFly IP address (default: 192.168.10.1)")
    parser.add_argument('-p', '--port', type=int, default=UDP_LOG_PORT, help=f"UDP port (default: {UDP_LOG_PORT})")
    parser.add_argument('--no-save', action='store_true', help="Don't save to file, just display stats")
    args = parser.parse_args()

    # Generate output filename -- the "flight_<capture-start-time>" naming
    # this tool's caller (`sf log wifi`, lib/sfcli/commands/log.py) also
    # uses, timestamped at the moment capture BEGINS, not when it finishes.
    # 出力ファイル名を生成する -- 呼び出し元（`sf log wifi`、lib/sfcli/
    # commands/log.py）と同じ「flight_<取得開始時刻>」の命名。タイムスタンプは
    # 取得完了時ではなく開始時点。
    output = args.output
    if not output and not args.no_save:
        timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
        output = f"flight_{timestamp}.sflog.zip"

    print(f"Capturing UDP telemetry from {args.ip}:{args.port}")
    print(f"  Duration: {args.duration}s")
    if output:
        print(f"  Output: {output}")
    print()

    capture = UDPTelemetryCapture(args.ip, args.port)
    success = capture.capture(args.duration, progress_bar)
    print()  # Newline after progress bar

    if not success:
        print("No data received. Check WiFi connection and StampFly power.")
        return 1

    capture.print_stats()

    if not args.no_save and output:
        capture.save_bundle(output, capture_info={'requested_duration_s': args.duration})

    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
