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
BMI270 IMU Sensor Model
BMI270 IMUセンサモデル

Simulates BMI270 6-axis IMU (accelerometer + gyroscope) with realistic
noise characteristics based on Allan variance analysis.
BMI270 6軸IMU（加速度計＋ジャイロ）をAllan分散解析に基づく
リアリスティックなノイズ特性でシミュレート。

Reference: BMI270 Datasheet, Bosch Sensortec
"""

import numpy as np
from typing import Optional
from .noise_models import IMUNoiseGenerator, AllanVarianceParams


class IMU:
    """
    BMI270 IMU Sensor Simulation Model
    BMI270 IMUセンサシミュレーションモデル

    Simulates both accelerometer and gyroscope with:
    - White noise (from datasheet noise density)
    - Bias instability (random walk)
    - Temperature effects (optional)
    - Motor vibration effects (optional)

    加速度計とジャイロを以下でシミュレート：
    - 白色ノイズ（データシートのノイズ密度より）
    - バイアス不安定性（ランダムウォーク）
    - 温度効果（オプション）
    - モーター振動効果（オプション）

    Attributes:
        sample_rate_hz: Sensor sample rate (Hz)
                       センササンプルレート（Hz）
        gyro_range_dps: Gyroscope full-scale range (deg/s)
                       ジャイロフルスケールレンジ（deg/s）
        accel_range_g: Accelerometer full-scale range (g)
                      加速度計フルスケールレンジ（g）
    """

    # BMI270 specifications (from datasheet)
    # BMI270仕様（データシートより）
    GYRO_RANGES_DPS = [125, 250, 500, 1000, 2000]
    ACCEL_RANGES_G = [2, 4, 8, 16]

    # Resolution
    GYRO_RESOLUTION_BITS = 16
    ACCEL_RESOLUTION_BITS = 16

    # Gravity constant
    GRAVITY = 9.80665  # m/s^2

    def __init__(
        self,
        sample_rate_hz: float = 400.0,
        gyro_range_dps: int = 2000,
        accel_range_g: int = 8,
        gyro_noise_density: float = 0.007,  # deg/s/√Hz
        accel_noise_density: float = 120.0,  # µg/√Hz
        enable_bias_drift: bool = True,
        enable_vibration: bool = False,
        vibration_amplitude: float = 0.01,  # rad/s
    ):
        """
        Initialize IMU sensor model.
        IMUセンサモデルを初期化

        Args:
            sample_rate_hz: Sensor sample rate (Hz)
                           センササンプルレート（Hz）
            gyro_range_dps: Gyroscope full-scale range (deg/s)
                           ジャイロフルスケールレンジ（deg/s）
            accel_range_g: Accelerometer full-scale range (g)
                          加速度計フルスケールレンジ（g）
            gyro_noise_density: Gyro noise density (deg/s/√Hz)
                               ジャイロノイズ密度（deg/s/√Hz）
            accel_noise_density: Accel noise density (µg/√Hz)
                                加速度ノイズ密度（µg/√Hz）
            enable_bias_drift: Enable bias random walk
                              バイアスランダムウォークを有効化
            enable_vibration: Enable motor vibration simulation
                             モーター振動シミュレーションを有効化
            vibration_amplitude: Vibration amplitude (rad/s)
                                振動振幅（rad/s）
        """
        self.sample_rate_hz = sample_rate_hz
        self.gyro_range_dps = gyro_range_dps
        self.accel_range_g = accel_range_g
        self.enable_bias_drift = enable_bias_drift
        self.enable_vibration = enable_vibration
        self.vibration_amplitude = vibration_amplitude

        self.dt = 1.0 / sample_rate_hz

        # Compute LSB values
        # LSB値を計算
        self.gyro_lsb = gyro_range_dps / (2 ** (self.GYRO_RESOLUTION_BITS - 1))
        self.accel_lsb = accel_range_g / (2 ** (self.ACCEL_RESOLUTION_BITS - 1))

        # Create Allan variance parameters from noise density
        # ノイズ密度からAllan分散パラメータを作成
        gyro_arw = np.deg2rad(gyro_noise_density)  # Convert to rad/s/√Hz
        accel_vrw = (accel_noise_density * 1e-6) * self.GRAVITY  # Convert to m/s²/√Hz

        gyro_params = AllanVarianceParams(
            random_walk=gyro_arw,
            bias_instability=np.deg2rad(0.1) if enable_bias_drift else 0.0,
            rate_random_walk=gyro_arw * 0.01 if enable_bias_drift else 0.0,
        )
        accel_params = AllanVarianceParams(
            random_walk=accel_vrw,
            bias_instability=0.002 * self.GRAVITY if enable_bias_drift else 0.0,
            rate_random_walk=accel_vrw * 0.01 if enable_bias_drift else 0.0,
        )

        self._noise_gen = IMUNoiseGenerator(
            gyro_params=gyro_params,
            accel_params=accel_params,
            sample_rate_hz=sample_rate_hz,
        )

        # Motor state for vibration simulation
        # 振動シミュレーション用モーター状態
        self._motor_angles = np.zeros(4)

    def read(
        self,
        true_angular_velocity: np.ndarray,
        true_acceleration: np.ndarray,
        motor_speeds: Optional[np.ndarray] = None,
    ) -> dict:
        """
        Simulate sensor reading with noise.
        ノイズを含むセンサ読み取りをシミュレート

        Args:
            true_angular_velocity: True angular velocity [p, q, r] (rad/s)
                                  真の角速度 [p, q, r] (rad/s)
            true_acceleration: True specific force [ax, ay, az] (m/s²)
                              真の比力 [ax, ay, az] (m/s²)
                              Note: Should include gravity in body frame
                              注意：機体座標系での重力を含むこと
            motor_speeds: Motor angular speeds [ω1, ω2, ω3, ω4] (rad/s)
                         モーター角速度 [ω1, ω2, ω3, ω4] (rad/s)
                         Optional, used for vibration simulation

        Returns:
            dict with gyro (rad/s), accel (m/s²), gyro_raw, accel_raw
            gyro (rad/s), accel (m/s²), gyro_raw, accel_raw を含む辞書
        """
        # Get noise from Allan variance model
        # Allan分散モデルからノイズを取得
        gyro_noise = self._noise_gen.generate_gyro_noise()
        accel_noise = self._noise_gen.generate_accel_noise()

        # Add vibration if enabled
        # 振動が有効な場合追加
        if self.enable_vibration and motor_speeds is not None:
            vibration = self._compute_vibration(motor_speeds)
            gyro_noise += vibration

        # Apply noise to true values
        # 真値にノイズを適用
        measured_gyro = true_angular_velocity + gyro_noise
        measured_accel = true_acceleration + accel_noise

        # Quantize to simulate ADC
        # ADCをシミュレートするために量子化
        gyro_raw = self._quantize_gyro(measured_gyro)
        accel_raw = self._quantize_accel(measured_accel)

        # Convert back from quantized (what firmware would see)
        # 量子化から戻す（ファームウェアが見る値）
        gyro_from_raw = gyro_raw * np.deg2rad(self.gyro_lsb)
        accel_from_raw = accel_raw * (self.accel_lsb * self.GRAVITY)

        return {
            'gyro': measured_gyro,           # Ideal measured (rad/s)
            'accel': measured_accel,         # Ideal measured (m/s²)
            'gyro_raw': gyro_raw,            # Raw ADC counts
            'accel_raw': accel_raw,          # Raw ADC counts
            'gyro_dps': np.rad2deg(measured_gyro),  # deg/s for convenience
            'accel_g': measured_accel / self.GRAVITY,  # g for convenience
        }

    def _compute_vibration(self, motor_speeds: np.ndarray) -> np.ndarray:
        """
        Compute motor-induced vibration.
        モーター起因の振動を計算

        Args:
            motor_speeds: Motor angular speeds [ω1, ω2, ω3, ω4] (rad/s)

        Returns:
            Vibration contribution to gyro noise (rad/s)
        """
        # Update motor angles
        # モーター角度を更新
        self._motor_angles += motor_speeds * self.dt
        self._motor_angles = np.mod(self._motor_angles, 2 * np.pi)

        # Vibration is sum of sinusoidal components from each motor
        # 振動は各モーターからの正弦波成分の和
        vibration = np.zeros(3)
        for i, angle in enumerate(self._motor_angles):
            # Each motor contributes to all axes with phase offset
            # 各モーターは位相オフセットを持って全軸に寄与
            phase = i * np.pi / 2
            amp = self.vibration_amplitude * (motor_speeds[i] / 1000.0)  # Scale with speed
            vibration[0] += amp * np.sin(angle + phase)
            vibration[1] += amp * np.sin(angle + phase + np.pi/3)
            vibration[2] += amp * np.sin(angle + phase + 2*np.pi/3)

        return vibration

    def _quantize_gyro(self, gyro_rad_s: np.ndarray) -> np.ndarray:
        """Quantize gyro reading to ADC counts / ジャイロ読み取りをADCカウントに量子化"""
        gyro_dps = np.rad2deg(gyro_rad_s)
        counts = gyro_dps / self.gyro_lsb
        counts = np.clip(counts, -32768, 32767)
        return np.round(counts).astype(np.int16)

    def _quantize_accel(self, accel_m_s2: np.ndarray) -> np.ndarray:
        """Quantize accel reading to ADC counts / 加速度読み取りをADCカウントに量子化"""
        accel_g = accel_m_s2 / self.GRAVITY
        counts = accel_g / self.accel_lsb
        counts = np.clip(counts, -32768, 32767)
        return np.round(counts).astype(np.int16)

    def reset(self):
        """Reset sensor state (bias) / センサ状態（バイアス）をリセット"""
        self._noise_gen.reset_bias()
        self._motor_angles = np.zeros(4)

    @property
    def gyro_bias(self) -> np.ndarray:
        """Current gyroscope bias (rad/s) / 現在のジャイロバイアス"""
        return self._noise_gen.gyro_bias

    @property
    def accel_bias(self) -> np.ndarray:
        """Current accelerometer bias (m/s²) / 現在の加速度バイアス"""
        return self._noise_gen.accel_bias


# Legacy function for backward compatibility
# 後方互換性のためのレガシー関数
def imu(pqr, mot_angle, dist):
    """
    Legacy IMU function for backward compatibility.
    後方互換性のためのレガシーIMU関数

    DEPRECATED: Use IMU class instead.
    非推奨：代わりにIMUクラスを使用してください。
    """
    p = pqr[0][0]
    q = pqr[1][0]
    r = pqr[2][0]

    # Simple noise model (original implementation)
    # シンプルなノイズモデル（元の実装）
    noise = np.random.normal(0, dist, 3)
    p += noise[0]
    q += noise[1]
    r += noise[2]

    return np.array([[p], [q], [r]])
