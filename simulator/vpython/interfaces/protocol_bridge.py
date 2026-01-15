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
Protocol Bridge - Simulator to Protocol Interface
プロトコルブリッジ - シミュレータとプロトコルのインターフェース

Converts simulator state to protocol messages and vice versa.
シミュレータ状態をプロトコルメッセージに変換（双方向）。

Provides:
- SimulatorState: Aggregated simulator state
- ProtocolBridge: Conversion between simulator and protocol
- BinaryLogger: Log telemetry to binary file
"""

import struct
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, List, BinaryIO, Callable
import numpy as np

from .messages import (
    ControlPacket,
    TelemetryPacket,
    TelemetryWSPacket,
    FlightState,
    WarningFlags,
    SensorStatus,
    ControlFlags,
)


# =============================================================================
# Simulator State Container
# =============================================================================

@dataclass
class SimulatorState:
    """
    Aggregated simulator state for protocol conversion.
    プロトコル変換用の集約されたシミュレータ状態

    Contains all data needed to generate telemetry packets.
    テレメトリパケット生成に必要な全データを含む。
    """
    # Time
    timestamp_ms: int = 0

    # Attitude (rad)
    attitude: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Position (m, NED frame)
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Velocity (m/s)
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # IMU data (bias corrected)
    gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))  # rad/s
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m/s²

    # Magnetometer (µT)
    mag: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Control inputs (normalized)
    control_throttle: float = 0.0
    control_roll: float = 0.0
    control_pitch: float = 0.0
    control_yaw: float = 0.0

    # Battery
    battery_voltage: float = 4.2

    # Status
    flight_state: FlightState = FlightState.INIT
    sensor_status: SensorStatus = SensorStatus.NONE
    warning_flags: WarningFlags = WarningFlags.NONE

    # Sequence counters
    telemetry_sequence: int = 0
    heartbeat: int = 0

    def get_altitude(self) -> float:
        """Get altitude (positive up) from NED position / NED位置から高度を取得"""
        return -self.position[2]


# =============================================================================
# Protocol Bridge
# =============================================================================

class ProtocolBridge:
    """
    Bridge between simulator state and protocol messages.
    シミュレータ状態とプロトコルメッセージ間のブリッジ

    Responsibilities:
    - Convert simulator state to TelemetryPacket / TelemetryWSPacket
    - Parse ControlPacket to normalized control inputs
    - Track sequence numbers and heartbeat

    役割：
    - シミュレータ状態を TelemetryPacket / TelemetryWSPacket に変換
    - ControlPacket を正規化制御入力にパース
    - シーケンス番号とハートビートを追跡
    """

    def __init__(self, start_time_ms: int = 0):
        """
        Initialize protocol bridge.
        プロトコルブリッジを初期化

        Args:
            start_time_ms: Initial timestamp
        """
        self._start_time_ms = start_time_ms
        self._telemetry_seq = 0
        self._heartbeat = 0

    def state_to_telemetry(self, state: SimulatorState) -> TelemetryPacket:
        """
        Convert simulator state to basic telemetry packet.
        シミュレータ状態を基本テレメトリパケットに変換

        Args:
            state: Current simulator state

        Returns:
            TelemetryPacket (22 bytes)
        """
        packet = TelemetryPacket.from_state(
            sequence=self._telemetry_seq,
            battery_v=state.battery_voltage,
            altitude_m=state.get_altitude(),
            velocity_mps=state.velocity,
            attitude_rad=state.attitude,
            state=state.flight_state,
            flags=state.warning_flags,
        )

        self._telemetry_seq = (self._telemetry_seq + 1) & 0xFF
        return packet

    def state_to_telemetry_ws(self, state: SimulatorState) -> TelemetryWSPacket:
        """
        Convert simulator state to extended WebSocket telemetry packet.
        シミュレータ状態を拡張WebSocketテレメトリパケットに変換

        Args:
            state: Current simulator state

        Returns:
            TelemetryWSPacket (108 bytes)
        """
        packet = TelemetryWSPacket.from_simulation_state(
            timestamp_ms=state.timestamp_ms,
            attitude_rad=state.attitude,
            position_m=state.position,
            velocity_mps=state.velocity,
            gyro_rps=state.gyro,
            accel_mps2=state.accel,
            control=(
                state.control_throttle,
                state.control_roll,
                state.control_pitch,
                state.control_yaw,
            ),
            mag_ut=state.mag,
            voltage=state.battery_voltage,
            flight_state=state.flight_state,
            sensor_status=state.sensor_status,
            heartbeat=self._heartbeat,
        )

        self._heartbeat = (self._heartbeat + 1) & 0xFFFFFFFF
        return packet

    def control_to_normalized(
        self,
        packet: ControlPacket,
    ) -> tuple[float, float, float, float, ControlFlags]:
        """
        Convert control packet to normalized values.
        コントロールパケットを正規化値に変換

        Args:
            packet: Control packet

        Returns:
            (throttle, roll, pitch, yaw, flags)
            throttle: 0.0 to 1.0
            roll/pitch/yaw: -1.0 to 1.0
        """
        throttle, roll, pitch, yaw = packet.to_normalized()
        return throttle, roll, pitch, yaw, packet.flags

    def apply_control(
        self,
        state: SimulatorState,
        packet: ControlPacket,
    ) -> SimulatorState:
        """
        Apply control packet to simulator state.
        コントロールパケットをシミュレータ状態に適用

        Args:
            state: Current simulator state
            packet: Control packet

        Returns:
            Updated simulator state
        """
        throttle, roll, pitch, yaw, flags = self.control_to_normalized(packet)

        state.control_throttle = throttle
        state.control_roll = roll
        state.control_pitch = pitch
        state.control_yaw = yaw

        # Update flight state based on arm flag
        # アームフラグに基づいて飛行状態を更新
        if flags & ControlFlags.ARM:
            if state.flight_state == FlightState.IDLE:
                state.flight_state = FlightState.ARMED
            elif state.flight_state == FlightState.ARMED and throttle > 0.1:
                state.flight_state = FlightState.FLYING
        else:
            if state.flight_state in (FlightState.ARMED, FlightState.FLYING):
                state.flight_state = FlightState.IDLE

        return state


# =============================================================================
# Binary Logger
# =============================================================================

class BinaryLogger:
    """
    Log telemetry packets to binary file.
    テレメトリパケットをバイナリファイルに記録

    File format (V2):
    - Header: 16 bytes
      - Magic: 4 bytes ('SFBL')
      - Version: 2 bytes
      - Header size: 2 bytes
      - Packet size: 2 bytes
      - Rate Hz: 2 bytes
      - Reserved: 4 bytes
    - Data: N x packet_size bytes

    ファイル形式（V2）：
    - ヘッダ: 16バイト
    - データ: N x パケットサイズ バイト
    """

    MAGIC = b'SFBL'  # StampFly Binary Log
    VERSION = 2
    HEADER_SIZE = 16

    def __init__(
        self,
        filepath: Path,
        packet_size: int = TelemetryWSPacket.SIZE,
        rate_hz: int = 50,
    ):
        """
        Initialize binary logger.
        バイナリロガーを初期化

        Args:
            filepath: Output file path
            packet_size: Size of each packet
            rate_hz: Logging rate
        """
        self.filepath = Path(filepath)
        self.packet_size = packet_size
        self.rate_hz = rate_hz
        self._file: Optional[BinaryIO] = None
        self._packet_count = 0

    def open(self):
        """Open log file and write header / ログファイルを開いてヘッダを書き込み"""
        self._file = open(self.filepath, 'wb')
        self._write_header()
        self._packet_count = 0

    def _write_header(self):
        """Write file header / ファイルヘッダを書き込み"""
        header = bytearray(self.HEADER_SIZE)
        header[0:4] = self.MAGIC
        struct.pack_into('<H', header, 4, self.VERSION)
        struct.pack_into('<H', header, 6, self.HEADER_SIZE)
        struct.pack_into('<H', header, 8, self.packet_size)
        struct.pack_into('<H', header, 10, self.rate_hz)
        # Reserved: 4 bytes (already zero)
        self._file.write(header)

    def write(self, packet: bytes):
        """
        Write a packet to the log.
        パケットをログに書き込み

        Args:
            packet: Raw packet bytes
        """
        if self._file is None:
            raise RuntimeError("Logger not opened")

        if len(packet) != self.packet_size:
            raise ValueError(f"Packet size mismatch: {len(packet)} != {self.packet_size}")

        self._file.write(packet)
        self._packet_count += 1

    def write_telemetry(self, packet: TelemetryWSPacket):
        """
        Write a TelemetryWSPacket to the log.
        TelemetryWSPacketをログに書き込み

        Args:
            packet: Telemetry packet
        """
        self.write(packet.pack())

    def close(self):
        """Close log file / ログファイルを閉じる"""
        if self._file is not None:
            self._file.close()
            self._file = None

    @property
    def packet_count(self) -> int:
        """Number of packets written / 書き込まれたパケット数"""
        return self._packet_count

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


# =============================================================================
# Binary Log Reader
# =============================================================================

class BinaryLogReader:
    """
    Read telemetry packets from binary log file.
    バイナリログファイルからテレメトリパケットを読み取り

    Compatible with tools/log_analyzer/ format.
    tools/log_analyzer/ 形式と互換。
    """

    def __init__(self, filepath: Path):
        """
        Initialize binary log reader.
        バイナリログリーダーを初期化

        Args:
            filepath: Log file path
        """
        self.filepath = Path(filepath)
        self._file: Optional[BinaryIO] = None
        self.version = 0
        self.header_size = 0
        self.packet_size = 0
        self.rate_hz = 0

    def open(self):
        """Open log file and read header / ログファイルを開いてヘッダを読み取り"""
        self._file = open(self.filepath, 'rb')
        self._read_header()

    def _read_header(self):
        """Read and validate file header / ファイルヘッダを読み取り検証"""
        header = self._file.read(BinaryLogger.HEADER_SIZE)

        if len(header) < BinaryLogger.HEADER_SIZE:
            raise ValueError("File too small for header")

        if header[0:4] != BinaryLogger.MAGIC:
            raise ValueError("Invalid file magic")

        self.version = struct.unpack_from('<H', header, 4)[0]
        self.header_size = struct.unpack_from('<H', header, 6)[0]
        self.packet_size = struct.unpack_from('<H', header, 8)[0]
        self.rate_hz = struct.unpack_from('<H', header, 10)[0]

        # Skip any additional header bytes
        if self.header_size > BinaryLogger.HEADER_SIZE:
            self._file.read(self.header_size - BinaryLogger.HEADER_SIZE)

    def read_packet(self) -> Optional[bytes]:
        """
        Read next packet from log.
        ログから次のパケットを読み取り

        Returns:
            Raw packet bytes or None if EOF
        """
        if self._file is None:
            raise RuntimeError("Reader not opened")

        data = self._file.read(self.packet_size)
        if len(data) < self.packet_size:
            return None

        return data

    def read_telemetry_ws(self) -> Optional[TelemetryWSPacket]:
        """
        Read next TelemetryWSPacket from log.
        ログから次のTelemetryWSPacketを読み取り

        Returns:
            TelemetryWSPacket or None if EOF/invalid
        """
        data = self.read_packet()
        if data is None:
            return None

        return TelemetryWSPacket.unpack(data)

    def read_all(self) -> List[TelemetryWSPacket]:
        """
        Read all packets from log.
        ログから全パケットを読み取り

        Returns:
            List of TelemetryWSPacket
        """
        packets = []
        while True:
            packet = self.read_telemetry_ws()
            if packet is None:
                break
            packets.append(packet)
        return packets

    def close(self):
        """Close log file / ログファイルを閉じる"""
        if self._file is not None:
            self._file.close()
            self._file = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    @property
    def packet_count(self) -> int:
        """
        Estimate total packet count.
        総パケット数を推定

        Returns:
            Estimated number of packets
        """
        if self._file is None:
            return 0

        current_pos = self._file.tell()
        self._file.seek(0, 2)  # End of file
        file_size = self._file.tell()
        self._file.seek(current_pos)

        data_size = file_size - self.header_size
        return data_size // self.packet_size


# =============================================================================
# Utility Functions
# =============================================================================

def log_to_numpy(packets: List[TelemetryWSPacket]) -> dict:
    """
    Convert list of telemetry packets to numpy arrays.
    テレメトリパケットのリストをnumpy配列に変換

    Args:
        packets: List of TelemetryWSPacket

    Returns:
        Dictionary of numpy arrays for each field
    """
    n = len(packets)
    if n == 0:
        return {}

    return {
        'timestamp_ms': np.array([p.timestamp_ms for p in packets]),
        'roll': np.array([p.roll for p in packets]),
        'pitch': np.array([p.pitch for p in packets]),
        'yaw': np.array([p.yaw for p in packets]),
        'pos_x': np.array([p.pos_x for p in packets]),
        'pos_y': np.array([p.pos_y for p in packets]),
        'pos_z': np.array([p.pos_z for p in packets]),
        'vel_x': np.array([p.vel_x for p in packets]),
        'vel_y': np.array([p.vel_y for p in packets]),
        'vel_z': np.array([p.vel_z for p in packets]),
        'gyro_x': np.array([p.gyro_x for p in packets]),
        'gyro_y': np.array([p.gyro_y for p in packets]),
        'gyro_z': np.array([p.gyro_z for p in packets]),
        'accel_x': np.array([p.accel_x for p in packets]),
        'accel_y': np.array([p.accel_y for p in packets]),
        'accel_z': np.array([p.accel_z for p in packets]),
        'ctrl_throttle': np.array([p.ctrl_throttle for p in packets]),
        'ctrl_roll': np.array([p.ctrl_roll for p in packets]),
        'ctrl_pitch': np.array([p.ctrl_pitch for p in packets]),
        'ctrl_yaw': np.array([p.ctrl_yaw for p in packets]),
        'mag_x': np.array([p.mag_x for p in packets]),
        'mag_y': np.array([p.mag_y for p in packets]),
        'mag_z': np.array([p.mag_z for p in packets]),
        'voltage': np.array([p.voltage for p in packets]),
        'flight_state': np.array([p.flight_state for p in packets]),
        'sensor_status': np.array([p.sensor_status for p in packets]),
        'heartbeat': np.array([p.heartbeat for p in packets]),
    }


def numpy_to_log(
    data: dict,
    filepath: Path,
    rate_hz: int = 50,
):
    """
    Convert numpy arrays back to binary log file.
    numpy配列をバイナリログファイルに変換

    Args:
        data: Dictionary of numpy arrays
        filepath: Output file path
        rate_hz: Logging rate
    """
    n = len(data.get('timestamp_ms', []))
    if n == 0:
        return

    with BinaryLogger(filepath, rate_hz=rate_hz) as logger:
        for i in range(n):
            packet = TelemetryWSPacket(
                timestamp_ms=int(data['timestamp_ms'][i]),
                roll=float(data['roll'][i]),
                pitch=float(data['pitch'][i]),
                yaw=float(data['yaw'][i]),
                pos_x=float(data['pos_x'][i]),
                pos_y=float(data['pos_y'][i]),
                pos_z=float(data['pos_z'][i]),
                vel_x=float(data['vel_x'][i]),
                vel_y=float(data['vel_y'][i]),
                vel_z=float(data['vel_z'][i]),
                gyro_x=float(data['gyro_x'][i]),
                gyro_y=float(data['gyro_y'][i]),
                gyro_z=float(data['gyro_z'][i]),
                accel_x=float(data['accel_x'][i]),
                accel_y=float(data['accel_y'][i]),
                accel_z=float(data['accel_z'][i]),
                ctrl_throttle=float(data['ctrl_throttle'][i]),
                ctrl_roll=float(data['ctrl_roll'][i]),
                ctrl_pitch=float(data['ctrl_pitch'][i]),
                ctrl_yaw=float(data['ctrl_yaw'][i]),
                mag_x=float(data['mag_x'][i]),
                mag_y=float(data['mag_y'][i]),
                mag_z=float(data['mag_z'][i]),
                voltage=float(data['voltage'][i]),
                flight_state=FlightState(int(data['flight_state'][i])),
                sensor_status=SensorStatus(int(data['sensor_status'][i])),
                heartbeat=int(data['heartbeat'][i]),
            )
            logger.write_telemetry(packet)
