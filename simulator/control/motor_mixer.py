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
Motor Mixer for X-Configuration Quadcopter
X字型クアッドコプター用モーターミキサー

Firmware-compatible motor mixer matching firmware/vehicle/components/sf_hal_motor/
ファームウェア firmware/vehicle/components/sf_hal_motor/ と互換のモーターミキサー

Motor Layout (X-Configuration):
モーターレイアウト（X字型）:

                 Front
            FL(M4)   FR(M1)
              ╲  ▲  ╱
               ╲ │ ╱
                ╲│╱
                 ╳         ← Center
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲
           RL(M3)  RR(M2)
                 Rear

Motor Rotation:
- M1 (FR): CCW
- M2 (RR): CW
- M3 (RL): CCW
- M4 (FL): CW
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional
from enum import IntEnum


class MotorIndex(IntEnum):
    """Motor indices / モーターインデックス"""
    M1_FR = 0  # Front-Right (CCW)
    M2_RR = 1  # Rear-Right (CW)
    M3_RL = 2  # Rear-Left (CCW)
    M4_FL = 3  # Front-Left (CW)


@dataclass
class MixerConfig:
    """
    Motor mixer configuration.
    モーターミキサー設定

    Attributes:
        output_scale: Scale factor for PID output (default 0.25/3.7 from firmware)
                     PID出力のスケールファクター（ファームウェアから0.25/3.7）
        min_output: Minimum motor output (0-1)
        max_output: Maximum motor output (0-1)
        idle_output: Motor output when armed but no throttle
                    ARM時でスロットル0の時のモーター出力
    """
    output_scale: float = 0.25 / 3.7  # ≈0.0676
    min_output: float = 0.0
    max_output: float = 1.0
    idle_output: float = 0.05  # 5% idle when armed


# Default mixer configuration
DEFAULT_MIXER_CONFIG = MixerConfig()


class MotorMixer:
    """
    X-Configuration Quadcopter Motor Mixer.
    X字型クアッドコプターモーターミキサー

    Converts throttle + PID outputs to individual motor commands.
    スロットル + PID出力を個別モーターコマンドに変換。

    Firmware-compatible mixing matrix:
    ファームウェア互換のミキシング行列：

    m1 = throttle + scale*(-roll + pitch + yaw)   # FR
    m2 = throttle + scale*(-roll - pitch - yaw)   # RR
    m3 = throttle + scale*(+roll - pitch + yaw)   # RL
    m4 = throttle + scale*(+roll + pitch - yaw)   # FL

    Usage:
        mixer = MotorMixer()
        motors = mixer.mix(throttle, roll, pitch, yaw)
    """

    # Mixing matrix coefficients [roll, pitch, yaw] for each motor
    # 各モーターの [roll, pitch, yaw] 係数
    MIXING_MATRIX = np.array([
        [-1, +1, +1],  # M1 (FR)
        [-1, -1, -1],  # M2 (RR)
        [+1, -1, +1],  # M3 (RL)
        [+1, +1, -1],  # M4 (FL)
    ], dtype=float)

    def __init__(self, config: MixerConfig = None):
        """
        Initialize motor mixer.
        モーターミキサーを初期化

        Args:
            config: Mixer configuration (uses defaults if None)
        """
        self.config = config or DEFAULT_MIXER_CONFIG

    def mix(
        self,
        throttle: float,
        roll: float,
        pitch: float,
        yaw: float,
        armed: bool = True,
    ) -> np.ndarray:
        """
        Mix throttle and control outputs to motor commands.
        スロットルと制御出力をモーターコマンドにミキシング

        Args:
            throttle: Throttle command (0-1)
                     スロットルコマンド（0-1）
            roll: Roll PID output (voltage scale from rate controller)
                 Roll PID出力（レートコントローラからの電圧スケール）
            pitch: Pitch PID output
            yaw: Yaw PID output
            armed: Whether motors are armed
                  モーターがARMかどうか

        Returns:
            Motor outputs [m1, m2, m3, m4] normalized (0-1)
            モーター出力 [m1, m2, m3, m4] 正規化（0-1）
        """
        if not armed:
            return np.zeros(4)

        # Apply scale factor to control outputs
        scale = self.config.output_scale
        control = np.array([roll, pitch, yaw]) * scale

        # Mix: throttle + mixing_matrix @ control
        motors = throttle + self.MIXING_MATRIX @ control

        # Apply idle output when throttle is low
        if throttle < self.config.idle_output:
            motors = np.maximum(motors, self.config.idle_output)

        # Clamp to valid range
        motors = np.clip(motors, self.config.min_output, self.config.max_output)

        return motors

    def mix_normalized(
        self,
        throttle: float,
        roll: float,
        pitch: float,
        yaw: float,
        armed: bool = True,
    ) -> np.ndarray:
        """
        Mix with normalized control inputs (-1 to 1).
        正規化制御入力（-1から1）でミキシング

        This is a simplified mixer for direct control without PID.
        これはPIDなしの直接制御用の簡略化ミキサー。

        Args:
            throttle: Throttle (0-1)
            roll: Roll command (-1 to 1)
            pitch: Pitch command (-1 to 1)
            yaw: Yaw command (-1 to 1)
            armed: Whether motors are armed

        Returns:
            Motor outputs [m1, m2, m3, m4] normalized (0-1)
        """
        if not armed:
            return np.zeros(4)

        # Scale normalized inputs to motor effect
        # Typical value: 0.25 for reasonable control authority
        control_scale = 0.25

        control = np.array([roll, pitch, yaw]) * control_scale
        motors = throttle + self.MIXING_MATRIX @ control

        # Apply idle and clamp
        if throttle < self.config.idle_output:
            motors = np.maximum(motors, self.config.idle_output)

        motors = np.clip(motors, self.config.min_output, self.config.max_output)

        return motors

    def inverse_mix(
        self,
        motors: np.ndarray,
    ) -> Tuple[float, float, float, float]:
        """
        Inverse mixing: motor outputs to throttle + control.
        逆ミキシング：モーター出力からスロットル+制御へ

        Useful for analysis and debugging.
        分析・デバッグに有用。

        Args:
            motors: Motor outputs [m1, m2, m3, m4]

        Returns:
            (throttle, roll, pitch, yaw)
        """
        # Throttle is average of all motors
        throttle = np.mean(motors)

        # Solve for control: (motors - throttle) = MIXING_MATRIX @ (control * scale)
        residual = motors - throttle
        control_scaled = np.linalg.lstsq(self.MIXING_MATRIX, residual, rcond=None)[0]
        control = control_scaled / self.config.output_scale

        return throttle, control[0], control[1], control[2]

    @staticmethod
    def motor_to_thrust(motor_output: float, max_thrust_n: float = 0.15) -> float:
        """
        Convert motor output to thrust force.
        モーター出力をスラスト力に変換

        Assumes quadratic relationship: thrust ∝ output^2
        2次関係を仮定：スラスト ∝ 出力^2

        Args:
            motor_output: Motor output (0-1)
            max_thrust_n: Maximum thrust per motor (N)

        Returns:
            Thrust force (N)
        """
        return max_thrust_n * motor_output ** 2

    @staticmethod
    def motor_to_torque(
        motors: np.ndarray,
        arm_length_m: float = 0.065,
        max_thrust_n: float = 0.15,
        torque_coeff: float = 0.01,
    ) -> np.ndarray:
        """
        Calculate torques from motor outputs.
        モーター出力からトルクを計算

        Args:
            motors: Motor outputs [m1, m2, m3, m4]
            arm_length_m: Distance from center to motor (m)
            max_thrust_n: Maximum thrust per motor (N)
            torque_coeff: Torque to thrust ratio (Nm/N)

        Returns:
            Torques [roll, pitch, yaw] (Nm)
        """
        # Convert to thrust
        thrust = np.array([
            MotorMixer.motor_to_thrust(m, max_thrust_n) for m in motors
        ])

        # Arm length factor (X-config: sqrt(2)/2 * arm_length)
        arm = arm_length_m * np.sqrt(2) / 2

        # Roll torque: right motors down, left motors up
        # τ_roll = arm * (T3 + T4 - T1 - T2)
        roll_torque = arm * (thrust[2] + thrust[3] - thrust[0] - thrust[1])

        # Pitch torque: front motors down, rear motors up
        # τ_pitch = arm * (T1 + T4 - T2 - T3)
        pitch_torque = arm * (thrust[0] + thrust[3] - thrust[1] - thrust[2])

        # Yaw torque: CW motors - CCW motors (reaction torque)
        # M2(CW) + M4(CW) - M1(CCW) - M3(CCW)
        yaw_torque = torque_coeff * (thrust[1] + thrust[3] - thrust[0] - thrust[2])

        return np.array([roll_torque, pitch_torque, yaw_torque])


# =============================================================================
# Convenience functions
# =============================================================================

def mix_motors(
    throttle: float,
    roll: float,
    pitch: float,
    yaw: float,
    armed: bool = True,
) -> np.ndarray:
    """
    Convenience function for motor mixing.
    モーターミキシングの便利関数

    Args:
        throttle: Throttle (0-1)
        roll: Roll PID output
        pitch: Pitch PID output
        yaw: Yaw PID output
        armed: Whether motors are armed

    Returns:
        Motor outputs [m1, m2, m3, m4]
    """
    mixer = MotorMixer()
    return mixer.mix(throttle, roll, pitch, yaw, armed)
