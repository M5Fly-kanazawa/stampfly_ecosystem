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
Software-in-the-Loop (SIL) Interface
ソフトウェアインザループ（SIL）インターフェース

Provides interface for running control algorithms in software
while simulating sensor inputs and actuator outputs.
センサ入力とアクチュエータ出力をシミュレートしながら
制御アルゴリズムをソフトウェアで実行するためのインターフェース。

SIL Modes:
1. Standalone: Simulator runs control algorithms internally
2. External: Control algorithms run in separate process/thread
3. Replay: Replay recorded sensor data through control algorithms
"""

import threading
import queue
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Callable, Dict, Any
import numpy as np

from .messages import (
    ControlPacket,
    TelemetryPacket,
    TelemetryWSPacket,
    FlightState,
    SensorStatus,
    WarningFlags,
)
from .protocol_bridge import (
    SimulatorState,
    ProtocolBridge,
    BinaryLogger,
)


# =============================================================================
# Sensor Data Container
# =============================================================================

@dataclass
class SensorData:
    """
    Aggregated sensor data for SIL interface.
    SILインターフェース用の集約されたセンサデータ

    Matches firmware sensor data structure.
    ファームウェアのセンサデータ構造と一致。
    """
    timestamp_us: int = 0

    # IMU (BMI270)
    gyro: np.ndarray = None  # rad/s
    accel: np.ndarray = None  # m/s²

    # Magnetometer (BMM150)
    mag: np.ndarray = None  # µT

    # Barometer (BMP280)
    pressure_pa: float = 101325.0
    temperature_c: float = 25.0

    # ToF (VL53L3CX)
    tof_distance_m: float = 0.0
    tof_valid: bool = False

    # Optical Flow (PMW3901)
    flow_delta_x: int = 0
    flow_delta_y: int = 0
    flow_quality: int = 0

    # Battery (INA3221)
    battery_voltage: float = 4.2
    battery_current: float = 0.0

    def __post_init__(self):
        if self.gyro is None:
            self.gyro = np.zeros(3)
        if self.accel is None:
            self.accel = np.array([0.0, 0.0, 9.81])
        if self.mag is None:
            self.mag = np.zeros(3)


@dataclass
class ActuatorCommand:
    """
    Actuator commands from control algorithm.
    制御アルゴリズムからのアクチュエータコマンド

    Motor outputs are normalized [0, 1].
    モーター出力は正規化 [0, 1]。
    """
    timestamp_us: int = 0
    motor_outputs: np.ndarray = None  # [FL, FR, RL, RR] normalized

    def __post_init__(self):
        if self.motor_outputs is None:
            self.motor_outputs = np.zeros(4)


# =============================================================================
# SIL Interface
# =============================================================================

class SILInterface:
    """
    Software-in-the-Loop Interface.
    ソフトウェアインザループインターフェース

    Bridges simulator physics with control algorithms.
    シミュレータ物理と制御アルゴリズムを橋渡し。

    Usage:
        sil = SILInterface()
        sil.start()

        while running:
            # Provide sensor data
            sil.push_sensor_data(sensor_data)

            # Get actuator commands
            commands = sil.pop_actuator_commands()

            # Apply to physics simulation
            ...

        sil.stop()
    """

    def __init__(
        self,
        control_rate_hz: float = 400.0,
        telemetry_rate_hz: float = 50.0,
        enable_logging: bool = False,
        log_path: Optional[Path] = None,
    ):
        """
        Initialize SIL interface.
        SILインターフェースを初期化

        Args:
            control_rate_hz: Control loop rate
            telemetry_rate_hz: Telemetry output rate
            enable_logging: Enable binary logging
            log_path: Path for log files
        """
        self.control_rate_hz = control_rate_hz
        self.telemetry_rate_hz = telemetry_rate_hz
        self.control_period_us = int(1e6 / control_rate_hz)
        self.telemetry_period_us = int(1e6 / telemetry_rate_hz)

        # Queues for data exchange
        # データ交換用キュー
        self._sensor_queue: queue.Queue[SensorData] = queue.Queue(maxsize=10)
        self._actuator_queue: queue.Queue[ActuatorCommand] = queue.Queue(maxsize=10)
        self._telemetry_queue: queue.Queue[TelemetryWSPacket] = queue.Queue(maxsize=10)

        # Control algorithm callback
        # 制御アルゴリズムコールバック
        self._control_callback: Optional[Callable[[SensorData], ActuatorCommand]] = None

        # State
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._sim_time_us = 0
        self._last_telemetry_us = 0

        # Logging
        self._enable_logging = enable_logging
        self._log_path = log_path
        self._logger: Optional[BinaryLogger] = None
        self._protocol_bridge = ProtocolBridge()

        # Simulator state
        self._state = SimulatorState()

    def set_control_callback(
        self,
        callback: Callable[[SensorData], ActuatorCommand],
    ):
        """
        Set control algorithm callback.
        制御アルゴリズムコールバックを設定

        Args:
            callback: Function that takes SensorData and returns ActuatorCommand
        """
        self._control_callback = callback

    def start(self):
        """Start SIL interface / SILインターフェースを開始"""
        if self._running:
            return

        self._running = True
        self._sim_time_us = 0
        self._last_telemetry_us = 0

        # Open logger if enabled
        if self._enable_logging and self._log_path:
            self._logger = BinaryLogger(
                self._log_path,
                rate_hz=int(self.telemetry_rate_hz),
            )
            self._logger.open()

    def stop(self):
        """Stop SIL interface / SILインターフェースを停止"""
        self._running = False

        if self._logger:
            self._logger.close()
            self._logger = None

    def push_sensor_data(self, data: SensorData):
        """
        Push sensor data to SIL interface.
        センサデータをSILインターフェースにプッシュ

        Args:
            data: Sensor data from simulator
        """
        try:
            self._sensor_queue.put_nowait(data)
        except queue.Full:
            # Drop oldest data
            try:
                self._sensor_queue.get_nowait()
                self._sensor_queue.put_nowait(data)
            except queue.Empty:
                pass

    def pop_actuator_commands(self, timeout: float = 0.0) -> Optional[ActuatorCommand]:
        """
        Pop actuator commands from SIL interface.
        SILインターフェースからアクチュエータコマンドをポップ

        Args:
            timeout: Timeout in seconds (0 = non-blocking)

        Returns:
            ActuatorCommand or None
        """
        try:
            if timeout > 0:
                return self._actuator_queue.get(timeout=timeout)
            else:
                return self._actuator_queue.get_nowait()
        except queue.Empty:
            return None

    def pop_telemetry(self, timeout: float = 0.0) -> Optional[TelemetryWSPacket]:
        """
        Pop telemetry packet from SIL interface.
        SILインターフェースからテレメトリパケットをポップ

        Args:
            timeout: Timeout in seconds (0 = non-blocking)

        Returns:
            TelemetryWSPacket or None
        """
        try:
            if timeout > 0:
                return self._telemetry_queue.get(timeout=timeout)
            else:
                return self._telemetry_queue.get_nowait()
        except queue.Empty:
            return None

    def step(self, dt_us: int) -> Optional[ActuatorCommand]:
        """
        Step SIL interface by given time.
        指定時間でSILインターフェースをステップ

        This is the main synchronous update method.
        これがメイン同期更新メソッド。

        Args:
            dt_us: Time step in microseconds

        Returns:
            ActuatorCommand if control ran, else None
        """
        self._sim_time_us += dt_us
        commands = None

        # Get latest sensor data
        sensor_data = None
        try:
            while True:
                sensor_data = self._sensor_queue.get_nowait()
        except queue.Empty:
            pass

        if sensor_data is None:
            return None

        # Run control callback
        if self._control_callback:
            commands = self._control_callback(sensor_data)
            if commands:
                try:
                    self._actuator_queue.put_nowait(commands)
                except queue.Full:
                    pass

        # Update state for telemetry
        self._update_state(sensor_data, commands)

        # Generate telemetry at lower rate
        if self._sim_time_us - self._last_telemetry_us >= self.telemetry_period_us:
            self._last_telemetry_us = self._sim_time_us
            self._generate_telemetry()

        return commands

    def _update_state(
        self,
        sensor_data: SensorData,
        commands: Optional[ActuatorCommand],
    ):
        """Update internal state from sensor data / センサデータから内部状態を更新"""
        self._state.timestamp_ms = sensor_data.timestamp_us // 1000
        self._state.gyro = sensor_data.gyro
        self._state.accel = sensor_data.accel
        self._state.mag = sensor_data.mag
        self._state.battery_voltage = sensor_data.battery_voltage

        # Update sensor status
        status = SensorStatus.NONE
        if np.linalg.norm(sensor_data.accel) > 0.1:
            status |= SensorStatus.IMU_OK
        if np.linalg.norm(sensor_data.mag) > 0.1:
            status |= SensorStatus.MAG_OK
        if sensor_data.pressure_pa > 0:
            status |= SensorStatus.BARO_OK
        if sensor_data.tof_valid:
            status |= SensorStatus.TOF_OK
        if sensor_data.flow_quality > 50:
            status |= SensorStatus.FLOW_OK
        self._state.sensor_status = status

        # Update control inputs from commands
        if commands:
            avg_output = np.mean(commands.motor_outputs)
            self._state.control_throttle = avg_output

    def _generate_telemetry(self):
        """Generate and queue telemetry packet / テレメトリパケットを生成してキューに入れる"""
        packet = self._protocol_bridge.state_to_telemetry_ws(self._state)

        try:
            self._telemetry_queue.put_nowait(packet)
        except queue.Full:
            pass

        # Log if enabled
        if self._logger:
            self._logger.write_telemetry(packet)

    @property
    def is_running(self) -> bool:
        """Check if interface is running / インターフェースが実行中かチェック"""
        return self._running

    @property
    def sim_time_s(self) -> float:
        """Get simulation time in seconds / シミュレーション時間（秒）を取得"""
        return self._sim_time_us / 1e6


# =============================================================================
# Simple Control Algorithm for Testing
# =============================================================================

class SimpleRateController:
    """
    Simple rate (angular velocity) controller for testing.
    テスト用のシンプルなレートコントローラ

    Implements basic PID control for angular rates.
    角速度のベーシックなPID制御を実装。
    """

    def __init__(
        self,
        kp: float = 0.5,
        ki: float = 0.1,
        kd: float = 0.01,
    ):
        """
        Initialize rate controller.
        レートコントローラを初期化

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self._integral = np.zeros(3)
        self._last_error = np.zeros(3)
        self._last_time_us = 0

        # Rate setpoints (rad/s)
        self.rate_setpoint = np.zeros(3)

        # Throttle setpoint (0-1)
        self.throttle_setpoint = 0.0

    def set_setpoints(
        self,
        throttle: float,
        roll_rate: float,
        pitch_rate: float,
        yaw_rate: float,
    ):
        """
        Set control setpoints.
        制御セットポイントを設定

        Args:
            throttle: Throttle (0-1)
            roll_rate: Roll rate setpoint (rad/s)
            pitch_rate: Pitch rate setpoint (rad/s)
            yaw_rate: Yaw rate setpoint (rad/s)
        """
        self.throttle_setpoint = np.clip(throttle, 0, 1)
        self.rate_setpoint = np.array([roll_rate, pitch_rate, yaw_rate])

    def update(self, sensor_data: SensorData) -> ActuatorCommand:
        """
        Update controller and get motor commands.
        コントローラを更新してモーターコマンドを取得

        Args:
            sensor_data: Current sensor data

        Returns:
            Actuator commands
        """
        # Calculate dt
        if self._last_time_us == 0:
            dt = 0.0025  # 400Hz default
        else:
            dt = (sensor_data.timestamp_us - self._last_time_us) / 1e6
            dt = np.clip(dt, 0.0001, 0.1)
        self._last_time_us = sensor_data.timestamp_us

        # Rate error
        error = self.rate_setpoint - sensor_data.gyro

        # PID calculation
        self._integral += error * dt
        self._integral = np.clip(self._integral, -1.0, 1.0)

        derivative = (error - self._last_error) / dt if dt > 0 else np.zeros(3)
        self._last_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # Motor mixing (X configuration)
        #   FL (M4)   FR (M1)
        #      \  /
        #       \/
        #       /\
        #      /  \
        #   RL (M3)  RR (M2)
        #
        # Roll: FL+RL down, FR+RR up
        # Pitch: FL+FR down, RL+RR up
        # Yaw: FL+RR CW, FR+RL CCW
        roll_out = output[0]
        pitch_out = output[1]
        yaw_out = output[2]

        motors = np.array([
            self.throttle_setpoint - roll_out + pitch_out + yaw_out,  # M1 (FR)
            self.throttle_setpoint - roll_out - pitch_out - yaw_out,  # M2 (RR)
            self.throttle_setpoint + roll_out - pitch_out + yaw_out,  # M3 (RL)
            self.throttle_setpoint + roll_out + pitch_out - yaw_out,  # M4 (FL)
        ])

        # Clamp to [0, 1]
        motors = np.clip(motors, 0, 1)

        return ActuatorCommand(
            timestamp_us=sensor_data.timestamp_us,
            motor_outputs=motors,
        )

    def reset(self):
        """Reset controller state / コントローラ状態をリセット"""
        self._integral = np.zeros(3)
        self._last_error = np.zeros(3)
        self._last_time_us = 0
