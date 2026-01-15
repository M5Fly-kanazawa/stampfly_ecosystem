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
Hardware-in-the-Loop (HIL) Interface
ハードウェアインザループ（HIL）インターフェース

Provides interface for connecting simulator to real firmware via serial.
シリアル経由で実機ファームウェアとシミュレータを接続するインターフェース。

HIL Architecture:
HILアーキテクチャ：

┌─────────────────┐         Serial          ┌─────────────────┐
│   Simulator     │ ←────────────────────→  │   Firmware      │
│   (Python)      │   Sensor Injection      │   (ESP32)       │
│                 │   Actuator Readback     │                 │
│  - Physics      │                         │  - Control      │
│  - Sensors      │                         │  - ESKF         │
│  - Visualization│                         │  - State Mgmt   │
└─────────────────┘                         └─────────────────┘

Communication Protocol:
- Sensor data: Simulator → Firmware (at sensor rates)
- Actuator data: Firmware → Simulator (at control rate)
- Sync: Firmware provides timing, Simulator follows
"""

import struct
import threading
import queue
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Callable, List, Tuple
from enum import IntEnum
import numpy as np

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from .messages import (
    TelemetryWSPacket,
    FlightState,
    SensorStatus,
    checksum_sum,
    checksum_xor,
)


# =============================================================================
# HIL Message Definitions
# =============================================================================

class HILMessageType(IntEnum):
    """HIL message types / HILメッセージタイプ"""
    # Simulator → Firmware (Sensor Injection)
    IMU_DATA = 0x10
    MAG_DATA = 0x11
    BARO_DATA = 0x12
    TOF_DATA = 0x13
    FLOW_DATA = 0x14

    # Firmware → Simulator (Actuator Readback)
    MOTOR_OUTPUT = 0x20
    STATE_UPDATE = 0x21

    # Control
    SYNC_REQUEST = 0x30
    SYNC_RESPONSE = 0x31
    HIL_ENABLE = 0x40
    HIL_DISABLE = 0x41


@dataclass
class HILIMUData:
    """
    IMU sensor data for HIL injection.
    HIL注入用IMUセンサデータ

    Size: 28 bytes
    """
    SIZE = 28
    MSG_TYPE = HILMessageType.IMU_DATA

    timestamp_us: int = 0
    gyro_x: float = 0.0      # rad/s
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    accel_x: float = 0.0     # m/s²
    accel_y: float = 0.0
    accel_z: float = 9.81

    def pack(self) -> bytes:
        """Serialize to bytes / バイト列にシリアライズ"""
        data = struct.pack(
            '<BIffffffB',
            self.MSG_TYPE,
            self.timestamp_us,
            self.gyro_x, self.gyro_y, self.gyro_z,
            self.accel_x, self.accel_y, self.accel_z,
            0  # Placeholder for checksum
        )
        cs = checksum_sum(data[:-1])
        return data[:-1] + bytes([cs])

    @classmethod
    def unpack(cls, data: bytes) -> Optional['HILIMUData']:
        """Deserialize from bytes / バイト列からデシリアライズ"""
        if len(data) < cls.SIZE or data[0] != cls.MSG_TYPE:
            return None
        if checksum_sum(data[:-1]) != data[-1]:
            return None

        values = struct.unpack('<BIffffff', data[:27])
        return cls(
            timestamp_us=values[1],
            gyro_x=values[2], gyro_y=values[3], gyro_z=values[4],
            accel_x=values[5], accel_y=values[6], accel_z=values[7],
        )


@dataclass
class HILMagData:
    """
    Magnetometer data for HIL injection.
    HIL注入用地磁気センサデータ

    Size: 16 bytes
    """
    SIZE = 16
    MSG_TYPE = HILMessageType.MAG_DATA

    timestamp_us: int = 0
    mag_x: float = 0.0       # µT
    mag_y: float = 0.0
    mag_z: float = 0.0

    def pack(self) -> bytes:
        data = struct.pack(
            '<BIfffB',
            self.MSG_TYPE,
            self.timestamp_us,
            self.mag_x, self.mag_y, self.mag_z,
            0
        )
        cs = checksum_sum(data[:-1])
        return data[:-1] + bytes([cs])

    @classmethod
    def unpack(cls, data: bytes) -> Optional['HILMagData']:
        if len(data) < cls.SIZE or data[0] != cls.MSG_TYPE:
            return None
        if checksum_sum(data[:-1]) != data[-1]:
            return None

        values = struct.unpack('<BIfff', data[:15])
        return cls(
            timestamp_us=values[1],
            mag_x=values[2], mag_y=values[3], mag_z=values[4],
        )


@dataclass
class HILBaroData:
    """
    Barometer data for HIL injection.
    HIL注入用気圧センサデータ

    Size: 14 bytes
    """
    SIZE = 14
    MSG_TYPE = HILMessageType.BARO_DATA

    timestamp_us: int = 0
    pressure_pa: float = 101325.0
    temperature_c: float = 25.0

    def pack(self) -> bytes:
        data = struct.pack(
            '<BIffB',
            self.MSG_TYPE,
            self.timestamp_us,
            self.pressure_pa, self.temperature_c,
            0
        )
        cs = checksum_sum(data[:-1])
        return data[:-1] + bytes([cs])

    @classmethod
    def unpack(cls, data: bytes) -> Optional['HILBaroData']:
        if len(data) < cls.SIZE or data[0] != cls.MSG_TYPE:
            return None
        if checksum_sum(data[:-1]) != data[-1]:
            return None

        values = struct.unpack('<BIff', data[:13])
        return cls(
            timestamp_us=values[1],
            pressure_pa=values[2], temperature_c=values[3],
        )


@dataclass
class HILToFData:
    """
    ToF sensor data for HIL injection.
    HIL注入用ToFセンサデータ

    Size: 12 bytes
    """
    SIZE = 12
    MSG_TYPE = HILMessageType.TOF_DATA

    timestamp_us: int = 0
    distance_mm: int = 0
    valid: bool = True

    def pack(self) -> bytes:
        data = struct.pack(
            '<BIHBB',
            self.MSG_TYPE,
            self.timestamp_us,
            self.distance_mm,
            1 if self.valid else 0,
            0
        )
        cs = checksum_sum(data[:-1])
        return data[:-1] + bytes([cs])

    @classmethod
    def unpack(cls, data: bytes) -> Optional['HILToFData']:
        if len(data) < cls.SIZE or data[0] != cls.MSG_TYPE:
            return None
        if checksum_sum(data[:-1]) != data[-1]:
            return None

        values = struct.unpack('<BIHB', data[:11])
        return cls(
            timestamp_us=values[1],
            distance_mm=values[2],
            valid=values[3] != 0,
        )


@dataclass
class HILFlowData:
    """
    Optical flow data for HIL injection.
    HIL注入用オプティカルフローデータ

    Size: 14 bytes
    """
    SIZE = 14
    MSG_TYPE = HILMessageType.FLOW_DATA

    timestamp_us: int = 0
    delta_x: int = 0         # pixels
    delta_y: int = 0
    quality: int = 0         # SQUAL (0-255)

    def pack(self) -> bytes:
        data = struct.pack(
            '<BIhhBB',
            self.MSG_TYPE,
            self.timestamp_us,
            self.delta_x, self.delta_y,
            self.quality,
            0
        )
        cs = checksum_sum(data[:-1])
        return data[:-1] + bytes([cs])

    @classmethod
    def unpack(cls, data: bytes) -> Optional['HILFlowData']:
        if len(data) < cls.SIZE or data[0] != cls.MSG_TYPE:
            return None
        if checksum_sum(data[:-1]) != data[-1]:
            return None

        values = struct.unpack('<BIhhB', data[:13])
        return cls(
            timestamp_us=values[1],
            delta_x=values[2], delta_y=values[3],
            quality=values[4],
        )


@dataclass
class HILMotorOutput:
    """
    Motor output data from firmware.
    ファームウェアからのモーター出力データ

    Size: 22 bytes
    """
    SIZE = 22
    MSG_TYPE = HILMessageType.MOTOR_OUTPUT

    timestamp_us: int = 0
    motor1: float = 0.0      # 0-1 normalized
    motor2: float = 0.0
    motor3: float = 0.0
    motor4: float = 0.0

    def pack(self) -> bytes:
        data = struct.pack(
            '<BIffffB',
            self.MSG_TYPE,
            self.timestamp_us,
            self.motor1, self.motor2, self.motor3, self.motor4,
            0
        )
        cs = checksum_sum(data[:-1])
        return data[:-1] + bytes([cs])

    @classmethod
    def unpack(cls, data: bytes) -> Optional['HILMotorOutput']:
        if len(data) < cls.SIZE or data[0] != cls.MSG_TYPE:
            return None
        if checksum_sum(data[:-1]) != data[-1]:
            return None

        values = struct.unpack('<BIffff', data[:21])
        return cls(
            timestamp_us=values[1],
            motor1=values[2], motor2=values[3],
            motor3=values[4], motor4=values[5],
        )

    @property
    def motors(self) -> np.ndarray:
        """Get motor outputs as array / モーター出力を配列で取得"""
        return np.array([self.motor1, self.motor2, self.motor3, self.motor4])


@dataclass
class HILStateUpdate:
    """
    State update from firmware.
    ファームウェアからの状態更新

    Size: 10 bytes
    """
    SIZE = 10
    MSG_TYPE = HILMessageType.STATE_UPDATE

    timestamp_us: int = 0
    flight_state: FlightState = FlightState.INIT
    sensor_status: SensorStatus = SensorStatus.NONE
    armed: bool = False

    def pack(self) -> bytes:
        data = struct.pack(
            '<BIBBB',
            self.MSG_TYPE,
            self.timestamp_us,
            int(self.flight_state),
            int(self.sensor_status),
            1 if self.armed else 0,
        )
        cs = checksum_sum(data)
        return data + bytes([cs])

    @classmethod
    def unpack(cls, data: bytes) -> Optional['HILStateUpdate']:
        if len(data) < cls.SIZE or data[0] != cls.MSG_TYPE:
            return None
        if checksum_sum(data[:-1]) != data[-1]:
            return None

        values = struct.unpack('<BIBBB', data[:9])
        return cls(
            timestamp_us=values[1],
            flight_state=FlightState(values[2]),
            sensor_status=SensorStatus(values[3]),
            armed=values[4] != 0,
        )


# =============================================================================
# HIL Interface
# =============================================================================

class HILInterface:
    """
    Hardware-in-the-Loop Interface.
    ハードウェアインザループインターフェース

    Connects simulator to real firmware via serial port.
    シリアルポート経由でシミュレータを実機ファームウェアに接続。

    Usage:
        hil = HILInterface()
        hil.connect('/dev/ttyUSB0')
        hil.enable_hil()

        while running:
            # Inject sensor data
            hil.inject_imu(timestamp, gyro, accel)
            hil.inject_baro(timestamp, pressure, temp)

            # Get motor outputs
            motors = hil.get_motor_output()

            # Apply to physics simulation
            ...

        hil.disable_hil()
        hil.disconnect()
    """

    DEFAULT_BAUDRATE = 921600
    READ_TIMEOUT = 0.001  # 1ms

    def __init__(self):
        """Initialize HIL interface / HILインターフェースを初期化"""
        if not SERIAL_AVAILABLE:
            raise ImportError("pyserial is required for HIL interface. Install with: pip install pyserial")

        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._read_thread: Optional[threading.Thread] = None

        # Message queues
        self._motor_queue: queue.Queue[HILMotorOutput] = queue.Queue(maxsize=10)
        self._state_queue: queue.Queue[HILStateUpdate] = queue.Queue(maxsize=10)

        # Callbacks
        self._motor_callback: Optional[Callable[[HILMotorOutput], None]] = None
        self._state_callback: Optional[Callable[[HILStateUpdate], None]] = None

        # Statistics
        self._tx_count = 0
        self._rx_count = 0
        self._error_count = 0

        # Timing
        self._last_sync_time = 0
        self._firmware_time_offset = 0

    @staticmethod
    def list_ports() -> List[str]:
        """
        List available serial ports.
        利用可能なシリアルポートを一覧表示

        Returns:
            List of port names
        """
        if not SERIAL_AVAILABLE:
            return []
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
    ) -> bool:
        """
        Connect to firmware via serial port.
        シリアルポート経由でファームウェアに接続

        Args:
            port: Serial port name (e.g., '/dev/ttyUSB0', 'COM3')
            baudrate: Baud rate (default 921600)

        Returns:
            True if connected successfully
        """
        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=self.READ_TIMEOUT,
                write_timeout=0.1,
            )

            # Start read thread
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()

            return True

        except serial.SerialException as e:
            print(f"HIL: Failed to connect: {e}")
            return False

    def disconnect(self):
        """Disconnect from firmware / ファームウェアから切断"""
        self._running = False

        if self._read_thread:
            self._read_thread.join(timeout=1.0)
            self._read_thread = None

        if self._serial:
            self._serial.close()
            self._serial = None

    def is_connected(self) -> bool:
        """Check if connected / 接続しているかチェック"""
        return self._serial is not None and self._serial.is_open

    def enable_hil(self) -> bool:
        """
        Enable HIL mode on firmware.
        ファームウェアでHILモードを有効化

        Returns:
            True if successful
        """
        if not self.is_connected():
            return False

        data = struct.pack('<BB', HILMessageType.HIL_ENABLE, 0)
        cs = checksum_sum(data)
        self._send(data + bytes([cs]))
        return True

    def disable_hil(self) -> bool:
        """
        Disable HIL mode on firmware.
        ファームウェアでHILモードを無効化

        Returns:
            True if successful
        """
        if not self.is_connected():
            return False

        data = struct.pack('<BB', HILMessageType.HIL_DISABLE, 0)
        cs = checksum_sum(data)
        self._send(data + bytes([cs]))
        return True

    def inject_imu(
        self,
        timestamp_us: int,
        gyro: np.ndarray,
        accel: np.ndarray,
    ):
        """
        Inject IMU sensor data.
        IMUセンサデータを注入

        Args:
            timestamp_us: Timestamp in microseconds
            gyro: Angular rates [gx, gy, gz] (rad/s)
            accel: Accelerations [ax, ay, az] (m/s²)
        """
        msg = HILIMUData(
            timestamp_us=timestamp_us,
            gyro_x=float(gyro[0]),
            gyro_y=float(gyro[1]),
            gyro_z=float(gyro[2]),
            accel_x=float(accel[0]),
            accel_y=float(accel[1]),
            accel_z=float(accel[2]),
        )
        self._send(msg.pack())

    def inject_mag(
        self,
        timestamp_us: int,
        mag: np.ndarray,
    ):
        """
        Inject magnetometer data.
        地磁気センサデータを注入

        Args:
            timestamp_us: Timestamp in microseconds
            mag: Magnetic field [mx, my, mz] (µT)
        """
        msg = HILMagData(
            timestamp_us=timestamp_us,
            mag_x=float(mag[0]),
            mag_y=float(mag[1]),
            mag_z=float(mag[2]),
        )
        self._send(msg.pack())

    def inject_baro(
        self,
        timestamp_us: int,
        pressure_pa: float,
        temperature_c: float,
    ):
        """
        Inject barometer data.
        気圧センサデータを注入

        Args:
            timestamp_us: Timestamp in microseconds
            pressure_pa: Pressure (Pa)
            temperature_c: Temperature (°C)
        """
        msg = HILBaroData(
            timestamp_us=timestamp_us,
            pressure_pa=pressure_pa,
            temperature_c=temperature_c,
        )
        self._send(msg.pack())

    def inject_tof(
        self,
        timestamp_us: int,
        distance_mm: int,
        valid: bool = True,
    ):
        """
        Inject ToF sensor data.
        ToFセンサデータを注入

        Args:
            timestamp_us: Timestamp in microseconds
            distance_mm: Distance in millimeters
            valid: Whether measurement is valid
        """
        msg = HILToFData(
            timestamp_us=timestamp_us,
            distance_mm=distance_mm,
            valid=valid,
        )
        self._send(msg.pack())

    def inject_flow(
        self,
        timestamp_us: int,
        delta_x: int,
        delta_y: int,
        quality: int,
    ):
        """
        Inject optical flow data.
        オプティカルフローデータを注入

        Args:
            timestamp_us: Timestamp in microseconds
            delta_x: X displacement (pixels)
            delta_y: Y displacement (pixels)
            quality: Surface quality (0-255)
        """
        msg = HILFlowData(
            timestamp_us=timestamp_us,
            delta_x=delta_x,
            delta_y=delta_y,
            quality=quality,
        )
        self._send(msg.pack())

    def get_motor_output(self, timeout: float = 0.0) -> Optional[HILMotorOutput]:
        """
        Get motor output from firmware.
        ファームウェアからモーター出力を取得

        Args:
            timeout: Timeout in seconds (0 = non-blocking)

        Returns:
            HILMotorOutput or None
        """
        try:
            if timeout > 0:
                return self._motor_queue.get(timeout=timeout)
            else:
                return self._motor_queue.get_nowait()
        except queue.Empty:
            return None

    def get_state_update(self, timeout: float = 0.0) -> Optional[HILStateUpdate]:
        """
        Get state update from firmware.
        ファームウェアから状態更新を取得

        Args:
            timeout: Timeout in seconds (0 = non-blocking)

        Returns:
            HILStateUpdate or None
        """
        try:
            if timeout > 0:
                return self._state_queue.get(timeout=timeout)
            else:
                return self._state_queue.get_nowait()
        except queue.Empty:
            return None

    def set_motor_callback(self, callback: Callable[[HILMotorOutput], None]):
        """Set callback for motor output / モーター出力のコールバックを設定"""
        self._motor_callback = callback

    def set_state_callback(self, callback: Callable[[HILStateUpdate], None]):
        """Set callback for state update / 状態更新のコールバックを設定"""
        self._state_callback = callback

    def sync_time(self) -> int:
        """
        Synchronize time with firmware.
        ファームウェアと時刻同期

        Returns:
            Firmware timestamp (µs)
        """
        if not self.is_connected():
            return 0

        # Send sync request
        data = struct.pack('<BB', HILMessageType.SYNC_REQUEST, 0)
        cs = checksum_sum(data)
        self._send(data + bytes([cs]))

        # Wait for response (handled in read thread)
        self._last_sync_time = int(time.time() * 1e6)
        return self._last_sync_time

    def _send(self, data: bytes):
        """Send data to firmware / ファームウェアにデータを送信"""
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(data)
                self._tx_count += 1
            except serial.SerialException:
                self._error_count += 1

    def _read_loop(self):
        """Background thread for reading serial data / シリアルデータ読み取りの背景スレッド"""
        buffer = bytearray()

        while self._running and self._serial:
            try:
                # Read available data
                if self._serial.in_waiting > 0:
                    data = self._serial.read(self._serial.in_waiting)
                    buffer.extend(data)

                # Process complete messages
                while len(buffer) >= 2:
                    msg_type = buffer[0]

                    # Determine message size based on type
                    if msg_type == HILMessageType.MOTOR_OUTPUT:
                        msg_size = HILMotorOutput.SIZE
                    elif msg_type == HILMessageType.STATE_UPDATE:
                        msg_size = HILStateUpdate.SIZE
                    elif msg_type == HILMessageType.SYNC_RESPONSE:
                        msg_size = 6  # type + timestamp + checksum
                    else:
                        # Unknown message, skip byte
                        buffer.pop(0)
                        continue

                    if len(buffer) < msg_size:
                        break  # Wait for more data

                    # Extract and process message
                    msg_data = bytes(buffer[:msg_size])
                    buffer = buffer[msg_size:]

                    self._process_message(msg_type, msg_data)

            except serial.SerialException:
                self._error_count += 1
                time.sleep(0.01)

    def _process_message(self, msg_type: int, data: bytes):
        """Process received message / 受信メッセージを処理"""
        self._rx_count += 1

        if msg_type == HILMessageType.MOTOR_OUTPUT:
            msg = HILMotorOutput.unpack(data)
            if msg:
                try:
                    self._motor_queue.put_nowait(msg)
                except queue.Full:
                    pass
                if self._motor_callback:
                    self._motor_callback(msg)

        elif msg_type == HILMessageType.STATE_UPDATE:
            msg = HILStateUpdate.unpack(data)
            if msg:
                try:
                    self._state_queue.put_nowait(msg)
                except queue.Full:
                    pass
                if self._state_callback:
                    self._state_callback(msg)

        elif msg_type == HILMessageType.SYNC_RESPONSE:
            # Extract firmware timestamp
            if len(data) >= 5:
                self._firmware_time_offset = struct.unpack('<I', data[1:5])[0]

    @property
    def stats(self) -> dict:
        """Get communication statistics / 通信統計を取得"""
        return {
            'tx_count': self._tx_count,
            'rx_count': self._rx_count,
            'error_count': self._error_count,
        }


# =============================================================================
# HIL Simulation Runner
# =============================================================================

class HILSimulationRunner:
    """
    HIL simulation runner that coordinates physics, sensors, and firmware.
    物理・センサ・ファームウェアを調整するHILシミュレーション実行器

    Manages the real-time loop for HIL testing.
    HILテスト用のリアルタイムループを管理。
    """

    def __init__(
        self,
        hil: HILInterface,
        physics_callback: Callable[[np.ndarray, float], dict],
        sensor_rates: dict = None,
    ):
        """
        Initialize HIL simulation runner.
        HILシミュレーション実行器を初期化

        Args:
            hil: HIL interface instance
            physics_callback: Function that takes (motor_outputs, dt) and returns
                             sensor data dict
            sensor_rates: Dict of sensor rates in Hz (default: firmware rates)
        """
        self.hil = hil
        self.physics_callback = physics_callback

        # Default sensor rates (matching firmware)
        self.sensor_rates = sensor_rates or {
            'imu': 400,    # Hz
            'mag': 100,
            'baro': 50,
            'tof': 30,
            'flow': 100,
        }

        self._running = False
        self._sim_time_us = 0
        self._last_sensor_time = {k: 0 for k in self.sensor_rates}

    def run(self, duration_s: float = None, realtime: bool = True):
        """
        Run HIL simulation.
        HILシミュレーションを実行

        Args:
            duration_s: Simulation duration (None = run until stopped)
            realtime: Run in real-time (True) or as fast as possible (False)
        """
        self._running = True
        self._sim_time_us = 0
        start_time = time.time()

        # Enable HIL mode
        self.hil.enable_hil()

        try:
            while self._running:
                loop_start = time.time()

                # Get motor outputs from firmware
                motor_output = self.hil.get_motor_output(timeout=0.001)

                if motor_output:
                    # Calculate dt
                    dt = 0.0025  # 400Hz default

                    # Run physics simulation
                    sensor_data = self.physics_callback(motor_output.motors, dt)

                    # Inject sensors at their respective rates
                    self._inject_sensors(sensor_data)

                    # Update simulation time
                    self._sim_time_us += int(dt * 1e6)

                # Check duration
                if duration_s and (time.time() - start_time) >= duration_s:
                    break

                # Real-time pacing
                if realtime:
                    elapsed = time.time() - loop_start
                    sleep_time = 0.0025 - elapsed  # 400Hz
                    if sleep_time > 0:
                        time.sleep(sleep_time)

        finally:
            self.hil.disable_hil()

    def stop(self):
        """Stop simulation / シミュレーションを停止"""
        self._running = False

    def _inject_sensors(self, sensor_data: dict):
        """Inject sensor data at appropriate rates / 適切なレートでセンサデータを注入"""
        current_time = self._sim_time_us

        # IMU (400Hz)
        if self._should_inject('imu', current_time):
            if 'gyro' in sensor_data and 'accel' in sensor_data:
                self.hil.inject_imu(
                    current_time,
                    sensor_data['gyro'],
                    sensor_data['accel'],
                )

        # Magnetometer (100Hz)
        if self._should_inject('mag', current_time):
            if 'mag' in sensor_data:
                self.hil.inject_mag(current_time, sensor_data['mag'])

        # Barometer (50Hz)
        if self._should_inject('baro', current_time):
            if 'pressure' in sensor_data:
                self.hil.inject_baro(
                    current_time,
                    sensor_data['pressure'],
                    sensor_data.get('temperature', 25.0),
                )

        # ToF (30Hz)
        if self._should_inject('tof', current_time):
            if 'tof_distance' in sensor_data:
                self.hil.inject_tof(
                    current_time,
                    int(sensor_data['tof_distance'] * 1000),
                    sensor_data.get('tof_valid', True),
                )

        # Optical Flow (100Hz)
        if self._should_inject('flow', current_time):
            if 'flow_x' in sensor_data:
                self.hil.inject_flow(
                    current_time,
                    int(sensor_data['flow_x']),
                    int(sensor_data['flow_y']),
                    sensor_data.get('flow_quality', 128),
                )

    def _should_inject(self, sensor: str, current_time: int) -> bool:
        """Check if sensor should be injected / センサを注入すべきかチェック"""
        period_us = int(1e6 / self.sensor_rates[sensor])
        if current_time - self._last_sensor_time[sensor] >= period_us:
            self._last_sensor_time[sensor] = current_time
            return True
        return False
