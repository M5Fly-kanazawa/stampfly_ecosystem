# MIT License
#
# Copyright (c) 2025 Kouhei Ito
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
StampFly Protocol Message Classes
StampFly プロトコルメッセージクラス

Python implementation of protocol/spec/messages.yaml
protocol/spec/messages.yaml の Python 実装

Message Types:
- ControlPacket (14 bytes): Controller -> Vehicle
- TelemetryPacket (22 bytes): Vehicle -> Controller (ESP-NOW)
- TelemetryWSPacket (108 bytes): Vehicle -> GCS (WebSocket)
- PairingPacket (11 bytes): Pairing request/response
- TDMABeacon (2 bytes): TDMA synchronization
"""

import struct
from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
from typing import Optional, Tuple
import numpy as np


# =============================================================================
# Enums and Flags
# =============================================================================

class FlightState(IntEnum):
    """Flight state enumeration / 飛行状態列挙型"""
    INIT = 0
    IDLE = 1
    ARMED = 2
    FLYING = 3
    LANDING = 4
    ERROR = 5


class ControlFlags(IntFlag):
    """Control packet flags / コントロールパケットフラグ"""
    NONE = 0
    ARM = 1 << 0      # Arm/disarm motors
    FLIP = 1 << 1     # Trigger flip maneuver
    MODE = 1 << 2     # Flight mode toggle
    ALT_MODE = 1 << 3 # Altitude hold mode


class WarningFlags(IntFlag):
    """Telemetry warning flags / テレメトリ警告フラグ"""
    NONE = 0
    LOW_BATTERY = 1 << 0
    SENSOR_ERROR = 1 << 1
    COMM_LOST = 1 << 2
    CALIBRATING = 1 << 3


class SensorStatus(IntFlag):
    """Sensor health flags / センサ健全性フラグ"""
    NONE = 0
    IMU_OK = 1 << 0
    MAG_OK = 1 << 1
    BARO_OK = 1 << 2
    TOF_OK = 1 << 3
    FLOW_OK = 1 << 4


# =============================================================================
# Checksum Functions
# =============================================================================

def checksum_sum(data: bytes) -> int:
    """
    Calculate simple sum checksum.
    単純和チェックサムを計算

    Args:
        data: Bytes to checksum

    Returns:
        Checksum value (0-255)
    """
    return sum(data) & 0xFF


def checksum_xor(data: bytes) -> int:
    """
    Calculate XOR checksum.
    XORチェックサムを計算

    Args:
        data: Bytes to checksum

    Returns:
        Checksum value (0-255)
    """
    result = 0
    for b in data:
        result ^= b
    return result


# =============================================================================
# Control Packet (14 bytes)
# =============================================================================

@dataclass
class ControlPacket:
    """
    Control commands from controller to vehicle.
    コントローラからビークルへの制御コマンド

    Size: 14 bytes
    Transport: ESP-NOW
    Rate: 50Hz

    Attributes:
        drone_mac: Destination MAC (lower 3 bytes)
        throttle: Throttle value (0-1000)
        roll: Roll value (500=center, <500=left, >500=right)
        pitch: Pitch value (500=center, <500=forward, >500=backward)
        yaw: Yaw value (500=center, <500=CCW, >500=CW)
        flags: Control flags (ControlFlags)
    """
    SIZE = 14

    drone_mac: bytes = field(default_factory=lambda: bytes(3))
    throttle: int = 0
    roll: int = 500
    pitch: int = 500
    yaw: int = 500
    flags: ControlFlags = ControlFlags.NONE
    reserved: int = 0

    def pack(self) -> bytes:
        """
        Serialize to bytes.
        バイト列にシリアライズ

        Returns:
            14-byte packet
        """
        # Validate values
        throttle = max(0, min(1000, self.throttle))
        roll = max(0, min(1000, self.roll))
        pitch = max(0, min(1000, self.pitch))
        yaw = max(0, min(1000, self.yaw))

        # Pack fields (little-endian)
        data = bytearray(13)
        data[0:3] = self.drone_mac[:3]
        struct.pack_into('<H', data, 3, throttle)
        struct.pack_into('<H', data, 5, roll)
        struct.pack_into('<H', data, 7, pitch)
        struct.pack_into('<H', data, 9, yaw)
        data[11] = int(self.flags) & 0xFF
        data[12] = self.reserved & 0xFF

        # Calculate checksum
        cs = checksum_sum(data)

        return bytes(data) + bytes([cs])

    @classmethod
    def unpack(cls, data: bytes) -> Optional['ControlPacket']:
        """
        Deserialize from bytes.
        バイト列からデシリアライズ

        Args:
            data: 14-byte packet

        Returns:
            ControlPacket or None if invalid
        """
        if len(data) < cls.SIZE:
            return None

        # Verify checksum
        if checksum_sum(data[:13]) != data[13]:
            return None

        return cls(
            drone_mac=bytes(data[0:3]),
            throttle=struct.unpack_from('<H', data, 3)[0],
            roll=struct.unpack_from('<H', data, 5)[0],
            pitch=struct.unpack_from('<H', data, 7)[0],
            yaw=struct.unpack_from('<H', data, 9)[0],
            flags=ControlFlags(data[11]),
            reserved=data[12],
        )

    def to_normalized(self) -> Tuple[float, float, float, float]:
        """
        Convert to normalized values (-1 to 1, throttle 0 to 1).
        正規化値に変換（-1〜1、スロットルは0〜1）

        Returns:
            (throttle, roll, pitch, yaw) as floats
        """
        return (
            self.throttle / 1000.0,
            (self.roll - 500) / 500.0,
            (self.pitch - 500) / 500.0,
            (self.yaw - 500) / 500.0,
        )

    @classmethod
    def from_normalized(
        cls,
        throttle: float,
        roll: float,
        pitch: float,
        yaw: float,
        flags: ControlFlags = ControlFlags.NONE,
        drone_mac: bytes = bytes(3),
    ) -> 'ControlPacket':
        """
        Create from normalized values.
        正規化値から作成

        Args:
            throttle: 0.0 to 1.0
            roll: -1.0 to 1.0
            pitch: -1.0 to 1.0
            yaw: -1.0 to 1.0
            flags: Control flags
            drone_mac: Destination MAC

        Returns:
            ControlPacket
        """
        return cls(
            drone_mac=drone_mac,
            throttle=int(np.clip(throttle, 0, 1) * 1000),
            roll=int((np.clip(roll, -1, 1) + 1) * 500),
            pitch=int((np.clip(pitch, -1, 1) + 1) * 500),
            yaw=int((np.clip(yaw, -1, 1) + 1) * 500),
            flags=flags,
        )


# =============================================================================
# Telemetry Packet (22 bytes)
# =============================================================================

@dataclass
class TelemetryPacket:
    """
    Basic telemetry from vehicle to controller via ESP-NOW.
    ESP-NOW経由でビークルからコントローラへの基本テレメトリ

    Size: 22 bytes
    Transport: ESP-NOW
    Rate: 50Hz

    Attributes:
        sequence: Packet sequence number (0-255)
        battery_mv: Battery voltage in millivolts
        altitude_cm: Altitude in centimeters
        velocity: (vx, vy, vz) in mm/s (body frame)
        attitude_deg10: (roll, pitch, yaw) in 0.1 degree units
        state: Flight state
        flags: Warning flags
    """
    SIZE = 22
    HEADER = 0xAA
    PACKET_TYPE = 0x01

    sequence: int = 0
    battery_mv: int = 0
    altitude_cm: int = 0
    velocity_x: int = 0  # mm/s
    velocity_y: int = 0  # mm/s
    velocity_z: int = 0  # mm/s
    roll_deg10: int = 0  # 0.1 degree
    pitch_deg10: int = 0
    yaw_deg10: int = 0
    state: FlightState = FlightState.INIT
    flags: WarningFlags = WarningFlags.NONE

    def pack(self) -> bytes:
        """
        Serialize to bytes.
        バイト列にシリアライズ

        Returns:
            22-byte packet
        """
        data = bytearray(21)
        data[0] = self.HEADER
        data[1] = self.PACKET_TYPE
        data[2] = self.sequence & 0xFF
        struct.pack_into('<H', data, 3, self.battery_mv)
        struct.pack_into('<h', data, 5, self.altitude_cm)
        struct.pack_into('<h', data, 7, self.velocity_x)
        struct.pack_into('<h', data, 9, self.velocity_y)
        struct.pack_into('<h', data, 11, self.velocity_z)
        struct.pack_into('<h', data, 13, self.roll_deg10)
        struct.pack_into('<h', data, 15, self.pitch_deg10)
        struct.pack_into('<h', data, 17, self.yaw_deg10)
        data[19] = int(self.state)
        data[20] = int(self.flags)

        cs = checksum_sum(data)
        return bytes(data) + bytes([cs])

    @classmethod
    def unpack(cls, data: bytes) -> Optional['TelemetryPacket']:
        """
        Deserialize from bytes.
        バイト列からデシリアライズ

        Args:
            data: 22-byte packet

        Returns:
            TelemetryPacket or None if invalid
        """
        if len(data) < cls.SIZE:
            return None

        # Verify header
        if data[0] != cls.HEADER or data[1] != cls.PACKET_TYPE:
            return None

        # Verify checksum
        if checksum_sum(data[:21]) != data[21]:
            return None

        return cls(
            sequence=data[2],
            battery_mv=struct.unpack_from('<H', data, 3)[0],
            altitude_cm=struct.unpack_from('<h', data, 5)[0],
            velocity_x=struct.unpack_from('<h', data, 7)[0],
            velocity_y=struct.unpack_from('<h', data, 9)[0],
            velocity_z=struct.unpack_from('<h', data, 11)[0],
            roll_deg10=struct.unpack_from('<h', data, 13)[0],
            pitch_deg10=struct.unpack_from('<h', data, 15)[0],
            yaw_deg10=struct.unpack_from('<h', data, 17)[0],
            state=FlightState(data[19]),
            flags=WarningFlags(data[20]),
        )

    @classmethod
    def from_state(
        cls,
        sequence: int,
        battery_v: float,
        altitude_m: float,
        velocity_mps: np.ndarray,
        attitude_rad: np.ndarray,
        state: FlightState,
        flags: WarningFlags = WarningFlags.NONE,
    ) -> 'TelemetryPacket':
        """
        Create from physical units.
        物理単位から作成

        Args:
            sequence: Sequence number
            battery_v: Battery voltage (V)
            altitude_m: Altitude (m)
            velocity_mps: Velocity [vx, vy, vz] (m/s)
            attitude_rad: Attitude [roll, pitch, yaw] (rad)
            state: Flight state
            flags: Warning flags

        Returns:
            TelemetryPacket
        """
        return cls(
            sequence=sequence & 0xFF,
            battery_mv=int(battery_v * 1000),
            altitude_cm=int(altitude_m * 100),
            velocity_x=int(velocity_mps[0] * 1000),
            velocity_y=int(velocity_mps[1] * 1000),
            velocity_z=int(velocity_mps[2] * 1000),
            roll_deg10=int(np.rad2deg(attitude_rad[0]) * 10),
            pitch_deg10=int(np.rad2deg(attitude_rad[1]) * 10),
            yaw_deg10=int(np.rad2deg(attitude_rad[2]) * 10),
            state=state,
            flags=flags,
        )


# =============================================================================
# Extended Telemetry Packet (108 bytes)
# =============================================================================

@dataclass
class TelemetryWSPacket:
    """
    Extended telemetry from vehicle to GCS via WebSocket.
    WebSocket経由でビークルからGCSへの拡張テレメトリ

    Size: 108 bytes
    Transport: WebSocket (binary, little-endian)
    Rate: 50Hz

    Contains full state estimation, sensor data, and control inputs.
    完全な状態推定、センサデータ、制御入力を含む。
    """
    SIZE = 108
    HEADER = 0xAA
    PACKET_TYPE = 0x20

    # Header
    timestamp_ms: int = 0

    # Attitude (rad)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Position (m, NED frame)
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0

    # Velocity (m/s)
    vel_x: float = 0.0
    vel_y: float = 0.0
    vel_z: float = 0.0

    # Gyro (rad/s, bias corrected)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0

    # Accel (m/s², bias corrected)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0

    # Control inputs (normalized)
    ctrl_throttle: float = 0.0
    ctrl_roll: float = 0.0
    ctrl_pitch: float = 0.0
    ctrl_yaw: float = 0.0

    # Magnetometer (µT)
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0

    # Battery
    voltage: float = 0.0

    # Status
    flight_state: FlightState = FlightState.INIT
    sensor_status: SensorStatus = SensorStatus.NONE

    # Heartbeat
    heartbeat: int = 0

    def pack(self) -> bytes:
        """
        Serialize to bytes.
        バイト列にシリアライズ

        Returns:
            108-byte packet
        """
        data = bytearray(108)

        # Header
        data[0] = self.HEADER
        data[1] = self.PACKET_TYPE
        struct.pack_into('<I', data, 2, self.timestamp_ms)

        # Attitude
        struct.pack_into('<f', data, 6, self.roll)
        struct.pack_into('<f', data, 10, self.pitch)
        struct.pack_into('<f', data, 14, self.yaw)

        # Position
        struct.pack_into('<f', data, 18, self.pos_x)
        struct.pack_into('<f', data, 22, self.pos_y)
        struct.pack_into('<f', data, 26, self.pos_z)

        # Velocity
        struct.pack_into('<f', data, 30, self.vel_x)
        struct.pack_into('<f', data, 34, self.vel_y)
        struct.pack_into('<f', data, 38, self.vel_z)

        # Gyro
        struct.pack_into('<f', data, 42, self.gyro_x)
        struct.pack_into('<f', data, 46, self.gyro_y)
        struct.pack_into('<f', data, 50, self.gyro_z)

        # Accel
        struct.pack_into('<f', data, 54, self.accel_x)
        struct.pack_into('<f', data, 58, self.accel_y)
        struct.pack_into('<f', data, 62, self.accel_z)

        # Control inputs
        struct.pack_into('<f', data, 66, self.ctrl_throttle)
        struct.pack_into('<f', data, 70, self.ctrl_roll)
        struct.pack_into('<f', data, 74, self.ctrl_pitch)
        struct.pack_into('<f', data, 78, self.ctrl_yaw)

        # Magnetometer
        struct.pack_into('<f', data, 82, self.mag_x)
        struct.pack_into('<f', data, 86, self.mag_y)
        struct.pack_into('<f', data, 90, self.mag_z)

        # Battery
        struct.pack_into('<f', data, 94, self.voltage)

        # Status
        data[98] = int(self.flight_state)
        data[99] = int(self.sensor_status)

        # Heartbeat
        struct.pack_into('<I', data, 100, self.heartbeat)

        # Checksum (XOR of bytes 0-103)
        data[104] = checksum_xor(data[:104])

        # Padding (3 bytes)
        data[105:108] = b'\x00\x00\x00'

        return bytes(data)

    @classmethod
    def unpack(cls, data: bytes) -> Optional['TelemetryWSPacket']:
        """
        Deserialize from bytes.
        バイト列からデシリアライズ

        Args:
            data: 108-byte packet

        Returns:
            TelemetryWSPacket or None if invalid
        """
        if len(data) < cls.SIZE:
            return None

        # Verify header
        if data[0] != cls.HEADER or data[1] != cls.PACKET_TYPE:
            return None

        # Verify checksum
        if checksum_xor(data[:104]) != data[104]:
            return None

        return cls(
            timestamp_ms=struct.unpack_from('<I', data, 2)[0],
            roll=struct.unpack_from('<f', data, 6)[0],
            pitch=struct.unpack_from('<f', data, 10)[0],
            yaw=struct.unpack_from('<f', data, 14)[0],
            pos_x=struct.unpack_from('<f', data, 18)[0],
            pos_y=struct.unpack_from('<f', data, 22)[0],
            pos_z=struct.unpack_from('<f', data, 26)[0],
            vel_x=struct.unpack_from('<f', data, 30)[0],
            vel_y=struct.unpack_from('<f', data, 34)[0],
            vel_z=struct.unpack_from('<f', data, 38)[0],
            gyro_x=struct.unpack_from('<f', data, 42)[0],
            gyro_y=struct.unpack_from('<f', data, 46)[0],
            gyro_z=struct.unpack_from('<f', data, 50)[0],
            accel_x=struct.unpack_from('<f', data, 54)[0],
            accel_y=struct.unpack_from('<f', data, 58)[0],
            accel_z=struct.unpack_from('<f', data, 62)[0],
            ctrl_throttle=struct.unpack_from('<f', data, 66)[0],
            ctrl_roll=struct.unpack_from('<f', data, 70)[0],
            ctrl_pitch=struct.unpack_from('<f', data, 74)[0],
            ctrl_yaw=struct.unpack_from('<f', data, 78)[0],
            mag_x=struct.unpack_from('<f', data, 82)[0],
            mag_y=struct.unpack_from('<f', data, 86)[0],
            mag_z=struct.unpack_from('<f', data, 90)[0],
            voltage=struct.unpack_from('<f', data, 94)[0],
            flight_state=FlightState(data[98]),
            sensor_status=SensorStatus(data[99]),
            heartbeat=struct.unpack_from('<I', data, 100)[0],
        )

    @classmethod
    def from_simulation_state(
        cls,
        timestamp_ms: int,
        attitude_rad: np.ndarray,
        position_m: np.ndarray,
        velocity_mps: np.ndarray,
        gyro_rps: np.ndarray,
        accel_mps2: np.ndarray,
        control: Tuple[float, float, float, float],
        mag_ut: np.ndarray,
        voltage: float,
        flight_state: FlightState,
        sensor_status: SensorStatus,
        heartbeat: int,
    ) -> 'TelemetryWSPacket':
        """
        Create from simulation state.
        シミュレーション状態から作成

        Args:
            timestamp_ms: Timestamp in milliseconds
            attitude_rad: [roll, pitch, yaw] in radians
            position_m: [x, y, z] in meters (NED frame)
            velocity_mps: [vx, vy, vz] in m/s
            gyro_rps: [gx, gy, gz] in rad/s
            accel_mps2: [ax, ay, az] in m/s²
            control: (throttle, roll, pitch, yaw) normalized
            mag_ut: [mx, my, mz] in µT
            voltage: Battery voltage in V
            flight_state: Flight state
            sensor_status: Sensor status flags
            heartbeat: Heartbeat counter

        Returns:
            TelemetryWSPacket
        """
        return cls(
            timestamp_ms=timestamp_ms,
            roll=float(attitude_rad[0]),
            pitch=float(attitude_rad[1]),
            yaw=float(attitude_rad[2]),
            pos_x=float(position_m[0]),
            pos_y=float(position_m[1]),
            pos_z=float(position_m[2]),
            vel_x=float(velocity_mps[0]),
            vel_y=float(velocity_mps[1]),
            vel_z=float(velocity_mps[2]),
            gyro_x=float(gyro_rps[0]),
            gyro_y=float(gyro_rps[1]),
            gyro_z=float(gyro_rps[2]),
            accel_x=float(accel_mps2[0]),
            accel_y=float(accel_mps2[1]),
            accel_z=float(accel_mps2[2]),
            ctrl_throttle=float(control[0]),
            ctrl_roll=float(control[1]),
            ctrl_pitch=float(control[2]),
            ctrl_yaw=float(control[3]),
            mag_x=float(mag_ut[0]),
            mag_y=float(mag_ut[1]),
            mag_z=float(mag_ut[2]),
            voltage=float(voltage),
            flight_state=flight_state,
            sensor_status=sensor_status,
            heartbeat=heartbeat,
        )


# =============================================================================
# Pairing Packet (11 bytes)
# =============================================================================

@dataclass
class PairingPacket:
    """
    Pairing request/response packet.
    ペアリング要求/応答パケット

    Size: 11 bytes
    Transport: ESP-NOW

    Attributes:
        channel: WiFi channel (1-13)
        drone_mac: Full 6-byte drone MAC address
    """
    SIZE = 11
    SIGNATURE = bytes([0xAA, 0x55, 0x16, 0x88])

    channel: int = 1
    drone_mac: bytes = field(default_factory=lambda: bytes(6))

    def pack(self) -> bytes:
        """
        Serialize to bytes.
        バイト列にシリアライズ

        Returns:
            11-byte packet
        """
        data = bytearray(11)
        data[0] = self.channel & 0xFF
        data[1:7] = self.drone_mac[:6]
        data[7:11] = self.SIGNATURE
        return bytes(data)

    @classmethod
    def unpack(cls, data: bytes) -> Optional['PairingPacket']:
        """
        Deserialize from bytes.
        バイト列からデシリアライズ

        Args:
            data: 11-byte packet

        Returns:
            PairingPacket or None if invalid
        """
        if len(data) < cls.SIZE:
            return None

        # Verify signature
        if data[7:11] != cls.SIGNATURE:
            return None

        return cls(
            channel=data[0],
            drone_mac=bytes(data[1:7]),
        )


# =============================================================================
# TDMA Beacon (2 bytes)
# =============================================================================

@dataclass
class TDMABeacon:
    """
    TDMA synchronization beacon.
    TDMA同期ビーコン

    Size: 2 bytes
    Transport: ESP-NOW

    Used for frame synchronization in TDMA protocol.
    TDMAプロトコルのフレーム同期に使用。
    """
    SIZE = 2
    MARKER = bytes([0xBE, 0xAC])

    def pack(self) -> bytes:
        """
        Serialize to bytes.
        バイト列にシリアライズ

        Returns:
            2-byte packet
        """
        return self.MARKER

    @classmethod
    def unpack(cls, data: bytes) -> Optional['TDMABeacon']:
        """
        Deserialize from bytes.
        バイト列からデシリアライズ

        Args:
            data: 2-byte packet

        Returns:
            TDMABeacon or None if invalid
        """
        if len(data) < cls.SIZE:
            return None

        if data[:2] != cls.MARKER:
            return None

        return cls()


# =============================================================================
# Utility Functions
# =============================================================================

def identify_packet(data: bytes) -> Optional[str]:
    """
    Identify packet type from raw data.
    生データからパケットタイプを識別

    Args:
        data: Raw packet data

    Returns:
        Packet type name or None if unknown
    """
    if len(data) < 2:
        return None

    # Check for TDMA Beacon
    if data[:2] == TDMABeacon.MARKER:
        return 'TDMABeacon'

    # Check for Pairing Packet (by signature at offset 7)
    if len(data) >= 11 and data[7:11] == PairingPacket.SIGNATURE:
        return 'PairingPacket'

    # Check for telemetry packets (header 0xAA)
    if data[0] == 0xAA:
        if data[1] == TelemetryPacket.PACKET_TYPE:
            return 'TelemetryPacket'
        elif data[1] == TelemetryWSPacket.PACKET_TYPE:
            return 'TelemetryWSPacket'

    # Could be ControlPacket (no header)
    if len(data) >= ControlPacket.SIZE:
        # Try to validate checksum
        if checksum_sum(data[:13]) == data[13]:
            return 'ControlPacket'

    return None
