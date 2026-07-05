"""
packet_parser.py - Binary packet parser for StampFly telemetry

Parses TelemetryExtendedBatchPacket (552 bytes) from WebSocket.
バイナリテレメトリパケットの解析

Packet structure:
- Header: 4 bytes (0xBD, 0x32, sample_count=4, reserved)
- Samples: 4 × 136 bytes (ExtendedSample)
- Footer: 4 bytes (checksum + padding)

NOTE (post vehicle_new->vehicle promotion): this WebSocket-based
ExtendedBatchPacket format is implemented only by the legacy firmware,
firmware/vehicle_old/components/sf_svc_telemetry/include/telemetry.hpp.
The current primary firmware (firmware/vehicle/) deliberately dropped
WebSocket telemetry in favor of a UDP-only unified packet (0x50, variable
length, see firmware/vehicle/components/sf_telemetry/include/data_stream_wire.hpp
and docs/telemetry/UDP_TELEMETRY_DESIGN.md) — this parser (and the
WebSocket client that uses it) is therefore only compatible with
firmware/vehicle_old, not the promoted firmware/vehicle.
注記(vehicle_new→vehicle昇格後): このWebSocketベースのExtendedBatchPacket
形式はレガシーファーム firmware/vehicle_old/components/sf_svc_telemetry/
include/telemetry.hpp のみが実装する。現行の主力ファーム
(firmware/vehicle/)はWebSocketテレメトリを設計上廃止し、UDP専用の統一
パケット(0x50、可変長)に一本化済み。本パーサ(およびこれを使う
WebSocketクライアント)は firmware/vehicle_old 専用であり、昇格後の
firmware/vehicle には対応していない。

References:
- firmware/vehicle_old/components/sf_svc_telemetry/include/telemetry.hpp
- tools/log_analyzer/wifi_capture.py
"""

import struct
from dataclasses import dataclass
from typing import List, Optional


# Packet constants
# パケット定数
EXTENDED_SAMPLE_SIZE = 136
EXTENDED_BATCH_SIZE = 4
EXTENDED_BATCH_HEADER_SIZE = 4
EXTENDED_BATCH_FOOTER_SIZE = 4
EXTENDED_BATCH_PACKET_SIZE = 552  # 4 + 136*4 + 4

HEADER_EXTENDED_BATCH = 0xBD
PACKET_TYPE_EXTENDED = 0x32


@dataclass
class ExtendedSample:
    """Single telemetry sample (136 bytes parsed)

    Contains raw sensor data, ESKF estimates, and additional sensors.
    生センサデータ、ESKF推定値、追加センサを含む
    """
    # Timestamp
    timestamp_us: int  # [μs] since boot

    # Raw gyro (LPF filtered, no bias correction)
    # 生ジャイロ（LPFフィルタ済み、バイアス補正なし）
    gyro_x: float  # [rad/s]
    gyro_y: float
    gyro_z: float

    # Raw accel (LPF filtered, no bias correction)
    # 生加速度（LPFフィルタ済み、バイアス補正なし）
    accel_x: float  # [m/s²]
    accel_y: float
    accel_z: float

    # Bias-corrected gyro (what control loop sees)
    # バイアス補正済みジャイロ
    gyro_corrected_x: float  # [rad/s]
    gyro_corrected_y: float
    gyro_corrected_z: float

    # Controller inputs (normalized)
    # 正規化制御入力
    ctrl_throttle: float  # [0-1]
    ctrl_roll: float      # [-1, 1]
    ctrl_pitch: float
    ctrl_yaw: float

    # ESKF attitude quaternion [w, x, y, z]
    # ESKF姿勢クォータニオン
    quat_w: float
    quat_x: float
    quat_y: float
    quat_z: float

    # ESKF position [m] NED frame
    # ESKF位置（NED座標系）
    pos_x: float  # North
    pos_y: float  # East
    pos_z: float  # Down

    # ESKF velocity [m/s] NED frame
    # ESKF速度（NED座標系）
    vel_x: float
    vel_y: float
    vel_z: float

    # Gyro bias estimate [rad/s]
    # ジャイロバイアス推定値
    gyro_bias_x: float
    gyro_bias_y: float
    gyro_bias_z: float

    # Accel bias estimate [m/s²]
    # 加速度バイアス推定値
    accel_bias_x: float
    accel_bias_y: float
    accel_bias_z: float

    # ESKF status
    eskf_status: int

    # Barometric altitude [m]
    # 気圧高度
    baro_altitude: float

    # ToF distances [m]
    # ToF距離
    tof_bottom: float
    tof_front: float

    # Optical flow
    # 光学フロー
    flow_x: int
    flow_y: int
    flow_quality: int


# struct format for ExtendedSample (136 bytes)
# Little-endian, matching C struct layout
EXTENDED_SAMPLE_FORMAT = '<'
EXTENDED_SAMPLE_FORMAT += 'I'       # timestamp_us (4 bytes)
EXTENDED_SAMPLE_FORMAT += 'fff'     # gyro_x/y/z raw (12 bytes)
EXTENDED_SAMPLE_FORMAT += 'fff'     # accel_x/y/z raw (12 bytes)
EXTENDED_SAMPLE_FORMAT += 'fff'     # gyro_corrected_x/y/z (12 bytes)
EXTENDED_SAMPLE_FORMAT += 'ffff'    # ctrl_throttle/roll/pitch/yaw (16 bytes)
EXTENDED_SAMPLE_FORMAT += 'ffff'    # quat_w/x/y/z (16 bytes)
EXTENDED_SAMPLE_FORMAT += 'fff'     # pos_x/y/z (12 bytes)
EXTENDED_SAMPLE_FORMAT += 'fff'     # vel_x/y/z (12 bytes)
EXTENDED_SAMPLE_FORMAT += 'hhh'     # gyro_bias_x/y/z (int16, 6 bytes)
EXTENDED_SAMPLE_FORMAT += 'hhh'     # accel_bias_x/y/z (int16, 6 bytes)
EXTENDED_SAMPLE_FORMAT += 'B'       # eskf_status (1 byte)
EXTENDED_SAMPLE_FORMAT += '7B'      # padding1 (7 bytes)
EXTENDED_SAMPLE_FORMAT += 'f'       # baro_altitude (4 bytes)
EXTENDED_SAMPLE_FORMAT += 'ff'      # tof_bottom/front (8 bytes)
EXTENDED_SAMPLE_FORMAT += 'hh'      # flow_x/y (int16, 4 bytes)
EXTENDED_SAMPLE_FORMAT += 'B'       # flow_quality (1 byte)
EXTENDED_SAMPLE_FORMAT += '3B'      # padding2 (3 bytes)

# Verify struct size
_EXTENDED_SAMPLE_CALCSIZE = struct.calcsize(EXTENDED_SAMPLE_FORMAT)
assert _EXTENDED_SAMPLE_CALCSIZE == 136, f"Extended sample size mismatch: {_EXTENDED_SAMPLE_CALCSIZE}"


def verify_checksum(data: bytes) -> bool:
    """Verify XOR checksum of packet.

    Checksum is at offset 548 (4 header + 136*4 samples).
    XOR of all bytes 0-547.

    Args:
        data: Raw packet bytes (552 bytes)

    Returns:
        True if checksum valid
    """
    if len(data) != EXTENDED_BATCH_PACKET_SIZE:
        return False

    checksum_offset = EXTENDED_BATCH_HEADER_SIZE + EXTENDED_BATCH_SIZE * EXTENDED_SAMPLE_SIZE
    calculated = 0
    for i in range(checksum_offset):
        calculated ^= data[i]

    return calculated == data[checksum_offset]


def parse_extended_sample(data: bytes) -> ExtendedSample:
    """Parse a single ExtendedSample from bytes.

    Args:
        data: 136 bytes of sample data

    Returns:
        ExtendedSample dataclass
    """
    values = struct.unpack(EXTENDED_SAMPLE_FORMAT, data)

    idx = 0
    timestamp_us = values[idx]
    idx += 1

    # Raw gyro
    gyro_x, gyro_y, gyro_z = values[idx:idx+3]
    idx += 3

    # Raw accel
    accel_x, accel_y, accel_z = values[idx:idx+3]
    idx += 3

    # Corrected gyro
    gyro_corrected_x, gyro_corrected_y, gyro_corrected_z = values[idx:idx+3]
    idx += 3

    # Control inputs
    ctrl_throttle, ctrl_roll, ctrl_pitch, ctrl_yaw = values[idx:idx+4]
    idx += 4

    # Quaternion
    quat_w, quat_x, quat_y, quat_z = values[idx:idx+4]
    idx += 4

    # Position
    pos_x, pos_y, pos_z = values[idx:idx+3]
    idx += 3

    # Velocity
    vel_x, vel_y, vel_z = values[idx:idx+3]
    idx += 3

    # Gyro bias (scaled by 10000)
    gyro_bias_x = values[idx] / 10000.0
    gyro_bias_y = values[idx+1] / 10000.0
    gyro_bias_z = values[idx+2] / 10000.0
    idx += 3

    # Accel bias (scaled by 10000)
    accel_bias_x = values[idx] / 10000.0
    accel_bias_y = values[idx+1] / 10000.0
    accel_bias_z = values[idx+2] / 10000.0
    idx += 3

    # ESKF status + padding (8 bytes total)
    eskf_status = values[idx]
    idx += 8  # skip status + 7 padding bytes

    # Baro altitude
    baro_altitude = values[idx]
    idx += 1

    # ToF
    tof_bottom, tof_front = values[idx:idx+2]
    idx += 2

    # Optical flow
    flow_x, flow_y = values[idx:idx+2]
    flow_quality = values[idx+2]

    return ExtendedSample(
        timestamp_us=timestamp_us,
        gyro_x=gyro_x, gyro_y=gyro_y, gyro_z=gyro_z,
        accel_x=accel_x, accel_y=accel_y, accel_z=accel_z,
        gyro_corrected_x=gyro_corrected_x,
        gyro_corrected_y=gyro_corrected_y,
        gyro_corrected_z=gyro_corrected_z,
        ctrl_throttle=ctrl_throttle,
        ctrl_roll=ctrl_roll, ctrl_pitch=ctrl_pitch, ctrl_yaw=ctrl_yaw,
        quat_w=quat_w, quat_x=quat_x, quat_y=quat_y, quat_z=quat_z,
        pos_x=pos_x, pos_y=pos_y, pos_z=pos_z,
        vel_x=vel_x, vel_y=vel_y, vel_z=vel_z,
        gyro_bias_x=gyro_bias_x, gyro_bias_y=gyro_bias_y, gyro_bias_z=gyro_bias_z,
        accel_bias_x=accel_bias_x, accel_bias_y=accel_bias_y, accel_bias_z=accel_bias_z,
        eskf_status=eskf_status,
        baro_altitude=baro_altitude,
        tof_bottom=tof_bottom, tof_front=tof_front,
        flow_x=flow_x, flow_y=flow_y, flow_quality=flow_quality,
    )


def parse_extended_batch_packet(data: bytes) -> Optional[List[ExtendedSample]]:
    """Parse TelemetryExtendedBatchPacket (552 bytes).

    Args:
        data: Raw packet bytes

    Returns:
        List of 4 ExtendedSample, or None if invalid
    """
    # Verify size
    if len(data) != EXTENDED_BATCH_PACKET_SIZE:
        return None

    # Verify header
    if data[0] != HEADER_EXTENDED_BATCH:
        return None

    if data[1] != PACKET_TYPE_EXTENDED:
        return None

    # Verify checksum
    if not verify_checksum(data):
        return None

    # Parse samples
    samples = []
    for i in range(EXTENDED_BATCH_SIZE):
        offset = EXTENDED_BATCH_HEADER_SIZE + i * EXTENDED_SAMPLE_SIZE
        sample_data = data[offset:offset + EXTENDED_SAMPLE_SIZE]
        sample = parse_extended_sample(sample_data)
        samples.append(sample)

    return samples
