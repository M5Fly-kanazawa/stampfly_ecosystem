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
PKT_RATE_REF  = 0x99  # virtual ID for 400Hz rate_ref (fixed part of unified packet)
PKT_STATUS    = 0x4F
PKT_UNIFIED   = 0x50  # 8x IMU+ESKF + 8x PosVel + 8x RateRef + variable entries

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
UNIFIED_SEQ_PACKET_TYPES = (PKT_IMU_ESKF, PKT_POS_VEL, PKT_RATE_REF, PKT_DUTY400, PKT_CTRL_OUTPUT400)

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

    if pkt_id not in SAMPLE_INFO:
        # Status packet (0x4F) — parse fields
        # StatusPacket v2: [header 4B][uptime_ms 4B][voltage 4B][flight_state 1B]
        #   [sensor_health 1B][eskf_status 1B][padding 1B]
        #   [pid_roll Kp/Ti/Td 12B][pid_pitch Kp/Ti/Td 12B][pid_yaw Kp/Ti/Td 12B]
        #   [checksum 1B] = 53B
        # StatusPacket v3: v2 + [current_ma 4B][checksum 1B] = 57B (battery current,
        #   appended AFTER pid_gains — see data_stream_wire.hpp WireStatusPayload)
        # Legacy (17B) also supported for backward compatibility
        if pkt_id == PKT_STATUS and len(data) in (17, 53, 57):
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
            # Parse PID gains if present (v2/v3, 53B or 57B)
            if len(data) in (53, 57):
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
            if len(data) == 57:
                sample['current_ma'] = struct.unpack_from('<f', data, 52)[0]
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
        print()

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
        マージする（呼び出し側だけが知る情報 -- 例えば要求秒数 -- だけを
        渡せばよい。このメソッド自身は呼び出し側が何秒を要求したか知らない）。
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
