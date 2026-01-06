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
Attitude Controller (Angle Mode)
姿勢コントローラ（アングルモード）

Cascaded attitude control: attitude → rate → motors
カスケード姿勢制御：姿勢 → レート → モーター

This mode controls attitude angles (roll, pitch, yaw) rather than
angular rates. It uses an outer loop that generates rate setpoints
for the inner rate controller.

このモードは角速度ではなく姿勢角（roll, pitch, yaw）を制御します。
外側ループで内側のレートコントローラ用のレートセットポイントを生成します。
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional

from .pid import PID
from .rate_controller import RateController, RateControlConfig


@dataclass
class AttitudeControlConfig:
    """
    Attitude control configuration.
    姿勢制御設定

    Defines maximum angles and attitude PID gains.
    最大角度と姿勢PIDゲインを定義。
    """
    # Maximum angles (rad) for full stick deflection
    # スティック最大倒しでの最大角度（rad）
    max_roll_angle: float = np.deg2rad(30)   # 30 degrees
    max_pitch_angle: float = np.deg2rad(30)  # 30 degrees

    # Attitude (outer loop) PID gains
    # 姿勢（外側ループ）PIDゲイン
    # These output rate setpoints (rad/s)
    roll_angle_kp: float = 5.0
    roll_angle_ti: float = 0.0   # No integral for attitude
    roll_angle_td: float = 0.0   # No derivative (rate loop handles it)

    pitch_angle_kp: float = 5.0
    pitch_angle_ti: float = 0.0
    pitch_angle_td: float = 0.0

    # Yaw uses rate control directly (heading hold is separate)
    # Yawはレート制御を直接使用（ヘディングホールドは別）

    # Output limits (max rate setpoint in rad/s)
    max_rate_setpoint: float = 3.0  # rad/s


DEFAULT_ATTITUDE_CONFIG = AttitudeControlConfig()


class AttitudeController:
    """
    Attitude (Angle) Controller with Cascaded Rate Control.
    カスケードレート制御付き姿勢（アングル）コントローラ

    Architecture:
    アーキテクチャ：

    stick_input → [Attitude PID] → rate_setpoint → [Rate PID] → motor_output
    スティック入力 → [姿勢PID] → レートセットポイント → [レートPID] → モーター出力

    Features:
    - Cascaded control (attitude outer loop, rate inner loop)
    - Self-leveling when sticks are centered
    - Configurable maximum tilt angles
    - Uses existing RateController for inner loop

    機能：
    - カスケード制御（姿勢外側ループ、レート内側ループ）
    - スティック中立時のセルフレベリング
    - 設定可能な最大傾斜角
    - 内側ループに既存のRateControllerを使用
    """

    def __init__(
        self,
        attitude_config: AttitudeControlConfig = None,
        rate_config: RateControlConfig = None,
    ):
        """
        Initialize attitude controller.
        姿勢コントローラを初期化

        Args:
            attitude_config: Attitude control configuration
            rate_config: Rate control configuration for inner loop
        """
        self.attitude_config = attitude_config or DEFAULT_ATTITUDE_CONFIG
        self.rate_config = rate_config or RateControlConfig()

        # Outer loop: attitude PIDs
        # 外側ループ：姿勢PID
        self.roll_angle_pid = PID(
            Kp=self.attitude_config.roll_angle_kp,
            Ti=self.attitude_config.roll_angle_ti,
            Td=self.attitude_config.roll_angle_td,
            output_min=-self.attitude_config.max_rate_setpoint,
            output_max=self.attitude_config.max_rate_setpoint,
        )

        self.pitch_angle_pid = PID(
            Kp=self.attitude_config.pitch_angle_kp,
            Ti=self.attitude_config.pitch_angle_ti,
            Td=self.attitude_config.pitch_angle_td,
            output_min=-self.attitude_config.max_rate_setpoint,
            output_max=self.attitude_config.max_rate_setpoint,
        )

        # Inner loop: rate controller
        # 内側ループ：レートコントローラ
        self.rate_controller = RateController(self.rate_config)

    def update(
        self,
        stick_roll: float,
        stick_pitch: float,
        stick_yaw: float,
        attitude: np.ndarray,
        gyro: np.ndarray,
        dt: float,
    ) -> Tuple[float, float, float]:
        """
        Update attitude controller.
        姿勢コントローラを更新

        Args:
            stick_roll: Roll stick input (-1 to 1)
            stick_pitch: Pitch stick input (-1 to 1)
            stick_yaw: Yaw stick input (-1 to 1, rate control)
            attitude: Current attitude [roll, pitch, yaw] (rad)
                     現在の姿勢 [roll, pitch, yaw]（rad）
            gyro: Current angular rates [gx, gy, gz] (rad/s)
                 現在の角速度 [gx, gy, gz]（rad/s）
            dt: Time step (s)

        Returns:
            (roll_output, pitch_output, yaw_output) for motor mixer
        """
        # Convert stick to angle setpoints
        # スティックを角度セットポイントに変換
        roll_angle_setpoint = stick_roll * self.attitude_config.max_roll_angle
        pitch_angle_setpoint = stick_pitch * self.attitude_config.max_pitch_angle

        # Outer loop: attitude → rate setpoints
        # 外側ループ：姿勢 → レートセットポイント
        roll_rate_setpoint = self.roll_angle_pid.update(
            roll_angle_setpoint, attitude[0], dt
        )
        pitch_rate_setpoint = self.pitch_angle_pid.update(
            pitch_angle_setpoint, attitude[1], dt
        )

        # Yaw uses direct rate control
        # Yawは直接レート制御を使用
        yaw_rate_setpoint = stick_yaw * self.rate_config.yaw_rate_max

        # Inner loop: rate control
        # 内側ループ：レート制御
        rate_setpoint = np.array([
            roll_rate_setpoint,
            pitch_rate_setpoint,
            yaw_rate_setpoint,
        ])

        return self.rate_controller.update(rate_setpoint, gyro, dt)

    def reset(self):
        """Reset all controllers / 全コントローラをリセット"""
        self.roll_angle_pid.reset()
        self.pitch_angle_pid.reset()
        self.rate_controller.reset()


class AltitudeController:
    """
    Altitude Hold Controller.
    高度ホールドコントローラ

    Controls vertical position/velocity using throttle adjustment.
    スロットル調整で垂直位置/速度を制御。

    Architecture:
    アーキテクチャ：

    altitude_setpoint → [Position PID] → velocity_setpoint → [Velocity PID] → throttle_adjustment
    高度セットポイント → [位置PID] → 速度セットポイント → [速度PID] → スロットル調整
    """

    def __init__(
        self,
        position_kp: float = 1.5,
        position_ti: float = 0.0,
        position_td: float = 0.0,
        velocity_kp: float = 0.8,
        velocity_ti: float = 2.0,
        velocity_td: float = 0.05,
        max_velocity: float = 1.0,       # m/s
        max_throttle_adj: float = 0.3,   # +/- 30%
        hover_throttle: float = 0.5,     # Base throttle for hover
    ):
        """
        Initialize altitude controller.
        高度コントローラを初期化

        Args:
            position_kp: Position loop proportional gain
            position_ti: Position loop integral time
            position_td: Position loop derivative time
            velocity_kp: Velocity loop proportional gain
            velocity_ti: Velocity loop integral time
            velocity_td: Velocity loop derivative time
            max_velocity: Maximum vertical velocity (m/s)
            max_throttle_adj: Maximum throttle adjustment (+/-)
            hover_throttle: Base throttle for hover
        """
        self.max_velocity = max_velocity
        self.max_throttle_adj = max_throttle_adj
        self.hover_throttle = hover_throttle

        # Outer loop: position → velocity setpoint
        self.position_pid = PID(
            Kp=position_kp,
            Ti=position_ti,
            Td=position_td,
            output_min=-max_velocity,
            output_max=max_velocity,
        )

        # Inner loop: velocity → throttle adjustment
        self.velocity_pid = PID(
            Kp=velocity_kp,
            Ti=velocity_ti,
            Td=velocity_td,
            output_min=-max_throttle_adj,
            output_max=max_throttle_adj,
        )

        self._altitude_setpoint = 0.0
        self._enabled = False

    def set_altitude(self, altitude: float):
        """
        Set altitude setpoint.
        高度セットポイントを設定

        Args:
            altitude: Target altitude (m, positive up)
        """
        self._altitude_setpoint = altitude
        self._enabled = True

    def update(
        self,
        current_altitude: float,
        vertical_velocity: float,
        stick_throttle: float,
        dt: float,
    ) -> float:
        """
        Update altitude controller.
        高度コントローラを更新

        Args:
            current_altitude: Current altitude (m, positive up)
            vertical_velocity: Current vertical velocity (m/s, positive up)
            stick_throttle: Throttle stick input (0-1)
                           Used for manual override or setpoint adjustment
            dt: Time step (s)

        Returns:
            Throttle command (0-1)
        """
        if not self._enabled:
            return stick_throttle

        # Adjust setpoint based on stick position
        # スティック位置に基づいてセットポイントを調整
        # Center stick (0.5) = hold altitude
        # Above center = climb, below center = descend
        deadband = 0.1
        if abs(stick_throttle - 0.5) > deadband:
            climb_rate = (stick_throttle - 0.5) * 2.0 * self.max_velocity
            self._altitude_setpoint += climb_rate * dt

        # Outer loop: position → velocity setpoint
        velocity_setpoint = self.position_pid.update(
            self._altitude_setpoint, current_altitude, dt
        )

        # Inner loop: velocity → throttle adjustment
        throttle_adj = self.velocity_pid.update(
            velocity_setpoint, vertical_velocity, dt
        )

        # Add to hover throttle
        throttle = self.hover_throttle + throttle_adj

        return np.clip(throttle, 0.0, 1.0)

    def disable(self):
        """Disable altitude hold / 高度ホールドを無効化"""
        self._enabled = False

    def reset(self):
        """Reset controller / コントローラをリセット"""
        self.position_pid.reset()
        self.velocity_pid.reset()
        self._enabled = False

    @property
    def altitude_setpoint(self) -> float:
        """Get current altitude setpoint / 現在の高度セットポイントを取得"""
        return self._altitude_setpoint

    @property
    def is_enabled(self) -> bool:
        """Check if altitude hold is enabled / 高度ホールドが有効かチェック"""
        return self._enabled
