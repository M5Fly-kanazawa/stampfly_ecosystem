#!/usr/bin/env python3
"""
udp_capture.py - UDP Telemetry Capture Tool for StampFly

Captures full-rate sensor data from StampFly via UDP.
Each sensor type arrives as independent packets at its native rate.
StampFly からフルレートセンサデータを UDP でキャプチャ。
各センサは固有レートの独立パケットとして到着する。

Packet IDs:
    0x40  IMU + ESKF (400Hz, batch 4)
    0x41  Position + Velocity (400Hz, batch 4)
    0x42  Control Input (50Hz, batch 4)
    0x43  Optical Flow (100Hz, batch 4)
    0x44  ToF (30Hz, batch 4)
    0x45  Barometer (50Hz, batch 4)
    0x46  Magnetometer (25Hz, batch 4)
    0x4F  Status / Heartbeat (1Hz)

Usage:
    python udp_capture.py [options]
    sf log wifi [options]

Examples:
    python udp_capture.py -d 30              # 30 seconds capture
    python udp_capture.py -d 60 -o flight.csv
"""

import argparse
import csv
import socket
import struct
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from collections import defaultdict

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
PKT_RATE_REF  = 0x99  # virtual ID for 400Hz rate_ref (fixed part of unified packet)
PKT_STATUS    = 0x4F

CMD_START_LOG  = 0xF0
CMD_STOP_LOG   = 0xF1
CMD_HEARTBEAT  = 0xF2

UDP_LOG_PORT = 8890

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
    PKT_UNIFIED = 0x50
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
            imu_timestamps.append(sample['timestamp_us'])
            results.append((PKT_IMU_ESKF, sample))
            offset += 80

        # 8× PosVelSample (28B each)
        for i in range(8):
            values = struct.unpack_from(FMT_POS_VEL, data, offset)
            sample = dict(zip(CSV_COLUMNS[PKT_POS_VEL], values))
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

            if sensor_id in SAMPLE_INFO and offset + data_size <= len(data) - 1:
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
        # Legacy (17B) also supported for backward compatibility
        if pkt_id == PKT_STATUS and len(data) in (17, 53):
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
            # Parse PID gains if present (v2, 53B)
            if len(data) == 53:
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
# UDP Capture class
# UDP キャプチャクラス
# =============================================================================

class UDPTelemetryCapture:
    """Captures UDP telemetry from StampFly and saves to CSV."""

    def __init__(self, vehicle_ip: str = '192.168.4.1', port: int = UDP_LOG_PORT):
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

                self.bytes_received += len(data)

                # Parse packet
                results = parse_packet(data)
                if not results:
                    self.checksum_errors += 1
                    continue

                for pkt_id, sample in results:
                    self.sample_count[pkt_id] += 1
                    self.samples[pkt_id].append(sample)

                # Track packet-level stats
                pkt_id = data[0]
                self.packet_count[pkt_id] += 1

                # Sequence gap detection. Also applied to the unified packet
                # (0x50): vehicle_new implements its sequence counter, so a lost
                # unified packet (= 8 lost 400Hz samples) is now detectable.
                # Guard: the legacy vehicle firmware sends seq=0 on every unified
                # packet (unimplemented TODO) — skip while seq stays 0 so old
                # captures do not report bogus gaps.
                # シーケンスギャップ検出。統合パケット(0x50)にも適用: vehicle_new は
                # シーケンスカウンタを実装済みで、統合パケット1個の欠落(=400Hzサンプル
                # 8個の欠落)を検出できる。ガード: 旧 vehicle ファームは統合パケットの
                # seq が常に0(未実装TODO)のため、seq が0のままの間は検出をスキップし
                # 旧キャプチャで偽ギャップを報告しない。
                if pkt_id in SAMPLE_INFO or pkt_id == 0x50:
                    _, seq, _ = struct.unpack_from(FMT_HEADER, data, 0)
                    if pkt_id in self.last_seq:
                        last = self.last_seq[pkt_id]
                        if not (seq == 0 and last == 0):   # legacy seq=0 guard
                            expected = (last + 1) & 0xFFFF
                            if seq != expected:
                                gap = (seq - expected) & 0xFFFF
                                self.seq_gaps[pkt_id] += gap
                    self.last_seq[pkt_id] = seq

                # Progress callback
                if progress_cb:
                    elapsed = time.time() - self.start_time
                    progress_cb(elapsed, duration, sum(self.sample_count.values()))

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

        return sum(self.sample_count.values()) > 0

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

    def save_jsonl(self, filepath: str):
        """Save captured data as JSONLines (1 line = 1 sensor sample).
        JSONLines形式で保存（1行 = 1センササンプル）。

        Each line is a self-describing JSON object with sensor type as 'id'.
        各行はセンサ種別を 'id' フィールドに持つ自己記述的 JSON オブジェクト。

        Example:
          {"id":"imu","ts":75834074,"gyro":[0.01,0.02,-0.003],"accel":[0.12,-0.08,-9.81],...}
          {"id":"posvel","ts":75834074,"pos":[1.2,0.5,-0.03],"vel":[0.01,-0.02,0.001]}
          {"id":"flow","ts":75836000,"dx":31,"dy":23,"quality":35}
        """
        import json

        # Sensor ID to human-readable name and field structure
        # センサIDから人間が読める名前とフィールド構造へのマッピング
        JSONL_FORMAT = {
            PKT_IMU_ESKF: lambda s: {
                'id': 'imu',
                'ts': s['timestamp_us'],
                'gyro': [s['gyro_x'], s['gyro_y'], s['gyro_z']],
                'accel': [s['accel_x'], s['accel_y'], s['accel_z']],
                'gyro_raw': [s['gyro_raw_x'], s['gyro_raw_y'], s['gyro_raw_z']],
                'accel_raw': [s['accel_raw_x'], s['accel_raw_y'], s['accel_raw_z']],
                'quat': [s['quat_w'], s['quat_x'], s['quat_y'], s['quat_z']],
                'gyro_bias': [s['gyro_bias_x'], s['gyro_bias_y'], s['gyro_bias_z']],
                'accel_bias': [s['accel_bias_x'], s['accel_bias_y'], s['accel_bias_z']],
            },
            PKT_POS_VEL: lambda s: {
                'id': 'posvel',
                'ts': s['timestamp_us'],
                'pos': [s['pos_x'], s['pos_y'], s['pos_z']],
                'vel': [s['vel_x'], s['vel_y'], s['vel_z']],
            },
            PKT_CONTROL: lambda s: {
                'id': 'ctrl',
                'ts': s['timestamp_us'],
                'throttle': s['ctrl_throttle'],
                'roll': s['ctrl_roll'],
                'pitch': s['ctrl_pitch'],
                'yaw': s['ctrl_yaw'],
            },
            PKT_FLOW: lambda s: {
                'id': 'flow',
                'ts': s['timestamp_us'],
                'dx': s['flow_x'],
                'dy': s['flow_y'],
                'quality': s['flow_quality'],
            },
            PKT_TOF_BOTTOM: lambda s: {
                'id': 'tof_b',
                'ts': s['timestamp_us'],
                'distance': s['tof_distance'],
                'status': s['tof_status'],
            },
            PKT_TOF_FRONT: lambda s: {
                'id': 'tof_f',
                'ts': s['timestamp_us'],
                'distance': s['tof_distance'],
                'status': s['tof_status'],
            },
            PKT_BARO: lambda s: {
                'id': 'baro',
                'ts': s['timestamp_us'],
                'altitude': s['baro_altitude'],
                'pressure': s['baro_pressure'],
            },
            PKT_MAG: lambda s: {
                'id': 'mag',
                'ts': s['timestamp_us'],
                'x': s['mag_x'],
                'y': s['mag_y'],
                'z': s['mag_z'],
            },
            PKT_CTRL_REF: lambda s: {
                'id': 'ctrl_ref',
                'ts': s['timestamp_us'],
                'mode': s['flight_mode'],
                'angle_ref': [s['angle_ref_roll'] / 10000.0, s['angle_ref_pitch'] / 10000.0],
                'total_thrust': round(s.get('total_thrust', 0.0), 4),
                'motor_duty': [round(s.get(f'motor_duty_{m}', 0.0), 4) for m in ('FR','RR','RL','FL')],
                'alt_sp': round(s.get('alt_setpoint', 0.0), 4),
                'alt_vel_target': round(s.get('alt_vel_target', 0.0), 4),
                'climb_cmd': round(s.get('climb_rate_cmd', 0.0), 4),
                'pos_sp': [round(s.get('pos_setpoint_x', 0.0), 4),
                           round(s.get('pos_setpoint_y', 0.0), 4)],
            },
            PKT_ESKF_PDIAG: lambda s: {
                'id': 'p_diag',
                'ts': s['timestamp_us'],
                'pos': [s['p_pos_x'], s['p_pos_y'], s['p_pos_z']],
                'vel': [s['p_vel_x'], s['p_vel_y'], s['p_vel_z']],
                'att': [s['p_att_x'], s['p_att_y'], s['p_att_z']],
                'bg': [s['p_bg_x'], s['p_bg_y'], s['p_bg_z']],
                'ba': [s['p_ba_x'], s['p_ba_y'], s['p_ba_z']],
            },
            PKT_RATE_REF: lambda s: {
                'id': 'rate_ref',
                'ts': s['timestamp_us'],
                'rate_ref': [s['rate_ref_roll'] / 1000.0, s['rate_ref_pitch'] / 1000.0, s['rate_ref_yaw'] / 1000.0],
            },
            PKT_STATUS: lambda s: {
                'id': 'status',
                'ts': s['timestamp_us'],
                'uptime_ms': s['uptime_ms'],
                'voltage': round(s['voltage'], 3),
                'flight_state': s['flight_state'],
                'eskf_status': s['eskf_status'],
                **({
                    'pid_roll':  [s['pid_roll_kp'],  s['pid_roll_ti'],  s['pid_roll_td']],
                    'pid_pitch': [s['pid_pitch_kp'], s['pid_pitch_ti'], s['pid_pitch_td']],
                    'pid_yaw':   [s['pid_yaw_kp'],   s['pid_yaw_ti'],  s['pid_yaw_td']],
                } if 'pid_roll_kp' in s else {}),
            },
        }

        # Collect all samples with their packet ID, sort by timestamp
        # 全サンプルをパケットID付きで収集し、タイムスタンプ順にソート
        all_entries = []
        for pkt_id, samples in self.samples.items():
            fmt_fn = JSONL_FORMAT.get(pkt_id)
            if not fmt_fn:
                continue
            for sample in samples:
                all_entries.append((sample['timestamp_us'], pkt_id, sample))

        all_entries.sort(key=lambda e: e[0])

        # Trim startup gap: if there's a gap > 1 second in the first 10 entries,
        # discard everything before the gap (partial batch from before capture start)
        # 起動時ギャップ除去: 最初の10エントリ内に1秒以上のギャップがあれば
        # ギャップ前のデータを破棄（キャプチャ開始前の不完全バッチ）
        if len(all_entries) > 10:
            for i in range(1, min(10, len(all_entries))):
                gap_us = all_entries[i][0] - all_entries[i-1][0]
                if gap_us > 1_000_000:  # > 1 second
                    all_entries = all_entries[i:]
                    print(f"  Trimmed {i} startup samples (gap: {gap_us/1e6:.1f}s)")
                    break

        # Write JSONLines
        with open(filepath, 'w') as f:
            for ts, pkt_id, sample in all_entries:
                obj = JSONL_FORMAT[pkt_id](sample)
                f.write(json.dumps(obj, separators=(',', ':')) + '\n')

        print(f"  Saved: {filepath} ({len(all_entries)} lines)")


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
    parser.add_argument('-o', '--output', help="Output CSV filename (auto-generated if not specified)")
    parser.add_argument('-d', '--duration', type=float, default=30.0, help="Capture duration in seconds (default: 30)")
    parser.add_argument('-i', '--ip', default='192.168.4.1', help="StampFly IP address (default: 192.168.4.1)")
    parser.add_argument('-p', '--port', type=int, default=UDP_LOG_PORT, help=f"UDP port (default: {UDP_LOG_PORT})")
    parser.add_argument('--no-save', action='store_true', help="Don't save to file, just display stats")
    args = parser.parse_args()

    # Generate output filename
    output = args.output
    if not output and not args.no_save:
        timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
        output = f"stampfly_udp_{timestamp}.jsonl"

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
        capture.save_jsonl(output)

    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
