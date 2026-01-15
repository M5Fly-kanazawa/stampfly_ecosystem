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
Rate Controller (Angular Velocity Control)
レートコントローラ（角速度制御）

Firmware-compatible rate controller matching firmware/vehicle/main/rate_controller.hpp
ファームウェア firmware/vehicle/main/rate_controller.hpp と互換のレートコントローラ

Features:
- Roll/Pitch/Yaw rate PID control
- Firmware-matched gains
- Output limiting per axis
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple

from .pid import PID, PIDGains


# =============================================================================
# Firmware-matched configuration
# ファームウェア一致の設定
# =============================================================================

@dataclass
class RateControlConfig:
    """
    Rate control configuration matching firmware config.hpp
    ファームウェア config.hpp と一致するレート制御設定

    All gains from firmware/vehicle/main/config.hpp
    """
    # Rate sensitivity (max rate in rad/s for full stick deflection)
    # レート感度（スティック最大倒しでの最大角速度）
    roll_rate_max: float = 1.0    # rad/s (≈57.3 deg/s)
    pitch_rate_max: float = 1.0   # rad/s (≈57.3 deg/s)
    yaw_rate_max: float = 5.0     # rad/s (≈286.5 deg/s)

    # Roll PID gains (from config.hpp)
    roll_kp: float = 0.65
    roll_ti: float = 0.7    # s
    roll_td: float = 0.01   # s

    # Pitch PID gains (from config.hpp)
    pitch_kp: float = 0.95
    pitch_ti: float = 0.7   # s
    pitch_td: float = 0.025 # s

    # Yaw PID gains (from config.hpp)
    yaw_kp: float = 3.0
    yaw_ti: float = 0.8     # s
    yaw_td: float = 0.01    # s

    # Common PID parameters
    eta: float = 0.125          # Incomplete derivative filter
    output_limit: float = 3.7   # V (voltage scale, as in firmware)

    # Control frequency
    control_rate_hz: float = 400.0


# Default configuration instance
# デフォルト設定インスタンス
DEFAULT_RATE_CONFIG = RateControlConfig()


class RateController:
    """
    Rate (Angular Velocity) Controller.
    レート（角速度）コントローラ

    Firmware-compatible implementation of rate_controller.hpp
    ファームウェア rate_controller.hpp の互換実装

    Controls angular velocity (gyro rate) to track setpoints.
    角速度（ジャイロレート）をセットポイントに追従させる制御。

    Usage:
        ctrl = RateController()
        roll_out, pitch_out, yaw_out = ctrl.update(
            rate_setpoint,  # [roll_rate, pitch_rate, yaw_rate] rad/s
            gyro,           # [gx, gy, gz] rad/s
            dt              # time step
        )
    """

    def __init__(self, config: RateControlConfig = None):
        """
        Initialize rate controller.
        レートコントローラを初期化

        Args:
            config: Rate control configuration (uses defaults if None)
        """
        self.config = config or DEFAULT_RATE_CONFIG

        # Create PID controllers for each axis
        # 各軸のPIDコントローラを作成
        self.roll_pid = PID(
            Kp=self.config.roll_kp,
            Ti=self.config.roll_ti,
            Td=self.config.roll_td,
            eta=self.config.eta,
            output_min=-self.config.output_limit,
            output_max=self.config.output_limit,
        )

        self.pitch_pid = PID(
            Kp=self.config.pitch_kp,
            Ti=self.config.pitch_ti,
            Td=self.config.pitch_td,
            eta=self.config.eta,
            output_min=-self.config.output_limit,
            output_max=self.config.output_limit,
        )

        self.yaw_pid = PID(
            Kp=self.config.yaw_kp,
            Ti=self.config.yaw_ti,
            Td=self.config.yaw_td,
            eta=self.config.eta,
            output_min=-self.config.output_limit,
            output_max=self.config.output_limit,
        )

    def update(
        self,
        rate_setpoint: np.ndarray,
        gyro: np.ndarray,
        dt: float,
    ) -> Tuple[float, float, float]:
        """
        Update rate controller.
        レートコントローラを更新

        Args:
            rate_setpoint: Target angular rates [roll, pitch, yaw] (rad/s)
                          目標角速度 [roll, pitch, yaw]（rad/s）
            gyro: Current angular rates [gx, gy, gz] (rad/s)
                 現在の角速度 [gx, gy, gz]（rad/s）
            dt: Time step (s)
                時間ステップ（秒）

        Returns:
            (roll_output, pitch_output, yaw_output) in voltage scale
            電圧スケールでの (roll出力, pitch出力, yaw出力)
        """
        roll_out = self.roll_pid.update(rate_setpoint[0], gyro[0], dt)
        pitch_out = self.pitch_pid.update(rate_setpoint[1], gyro[1], dt)
        yaw_out = self.yaw_pid.update(rate_setpoint[2], gyro[2], dt)

        return roll_out, pitch_out, yaw_out

    def update_from_stick(
        self,
        stick_roll: float,
        stick_pitch: float,
        stick_yaw: float,
        gyro: np.ndarray,
        dt: float,
    ) -> Tuple[float, float, float]:
        """
        Update rate controller from normalized stick inputs.
        正規化スティック入力からレートコントローラを更新

        Args:
            stick_roll: Roll stick (-1 to 1)
            stick_pitch: Pitch stick (-1 to 1)
            stick_yaw: Yaw stick (-1 to 1)
            gyro: Current angular rates [gx, gy, gz] (rad/s)
            dt: Time step (s)

        Returns:
            (roll_output, pitch_output, yaw_output)
        """
        # Convert stick to rate setpoint
        rate_setpoint = np.array([
            stick_roll * self.config.roll_rate_max,
            stick_pitch * self.config.pitch_rate_max,
            stick_yaw * self.config.yaw_rate_max,
        ])

        return self.update(rate_setpoint, gyro, dt)

    def reset(self):
        """Reset all PID controllers / 全PIDコントローラをリセット"""
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()

    def get_pid_terms(self) -> dict:
        """
        Get all PID terms for debugging.
        デバッグ用に全PID項を取得

        Returns:
            Dictionary with P, I, D terms for each axis
        """
        roll_p, roll_i, roll_d = self.roll_pid.get_terms()
        pitch_p, pitch_i, pitch_d = self.pitch_pid.get_terms()
        yaw_p, yaw_i, yaw_d = self.yaw_pid.get_terms()

        return {
            'roll': {'P': roll_p, 'I': roll_i, 'D': roll_d},
            'pitch': {'P': pitch_p, 'I': pitch_i, 'D': pitch_d},
            'yaw': {'P': yaw_p, 'I': yaw_i, 'D': yaw_d},
        }
