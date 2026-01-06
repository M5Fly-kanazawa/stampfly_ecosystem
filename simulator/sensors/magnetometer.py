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
BMM150 Magnetometer Sensor Model
BMM150 地磁気センサモデル

Simulates BMM150 3-axis magnetometer with realistic noise and
hard/soft iron distortion effects.
BMM150 3軸地磁気センサをリアリスティックなノイズと
ハード/ソフトアイアン歪み効果でシミュレート。

Reference: BMM150 Datasheet, Bosch Sensortec
"""

import numpy as np
from typing import Optional


class Magnetometer:
    """
    BMM150 Magnetometer Sensor Simulation Model
    BMM150 地磁気センサシミュレーションモデル

    Simulates 3-axis magnetometer with:
    - White noise
    - Hard iron distortion (constant offset)
    - Soft iron distortion (scale/axis misalignment)
    - Motor magnetic interference (optional)

    3軸地磁気センサを以下でシミュレート：
    - 白色ノイズ
    - ハードアイアン歪み（定数オフセット）
    - ソフトアイアン歪み（スケール/軸ずれ）
    - モーター磁気干渉（オプション）

    Attributes:
        noise_std: Measurement noise standard deviation (µT)
                  測定ノイズ標準偏差（µT）
        hard_iron: Hard iron offset vector [x, y, z] (µT)
                  ハードアイアンオフセットベクトル [x, y, z]（µT）
        soft_iron: Soft iron matrix (3x3)
                  ソフトアイアン行列（3x3）
    """

    # BMM150 specifications (from datasheet)
    # BMM150仕様（データシートより）
    RANGE_UT = 1300  # ±1300 µT for X/Y, ±2500 µT for Z
    RESOLUTION_UT = 0.3  # µT (typical)
    NOISE_DENSITY_UT = 0.3  # µT RMS (typical at high accuracy preset)

    # Earth's magnetic field typical values (Japan/East Asia)
    # 地球磁場の代表値（日本/東アジア）
    EARTH_FIELD_MAGNITUDE_UT = 46.0  # µT (typical)
    EARTH_FIELD_INCLINATION_DEG = 49.0  # degrees (typical for Japan)
    EARTH_FIELD_DECLINATION_DEG = -7.0  # degrees (typical for Japan)

    def __init__(
        self,
        noise_std: float = 0.3,
        hard_iron: Optional[np.ndarray] = None,
        soft_iron: Optional[np.ndarray] = None,
        earth_field_ned: Optional[np.ndarray] = None,
        enable_motor_interference: bool = False,
        motor_interference_scale: float = 0.5,
    ):
        """
        Initialize magnetometer sensor model.
        地磁気センサモデルを初期化

        Args:
            noise_std: Measurement noise std dev (µT)
                      測定ノイズ標準偏差（µT）
            hard_iron: Hard iron offset [x, y, z] (µT), default random
                      ハードアイアンオフセット [x, y, z]（µT）、デフォルトはランダム
            soft_iron: Soft iron matrix (3x3), default identity with slight error
                      ソフトアイアン行列（3x3）、デフォルトは微小誤差付き単位行列
            earth_field_ned: Earth's magnetic field in NED frame (µT)
                            NED座標系での地球磁場（µT）
            enable_motor_interference: Enable motor current interference
                                      モーター電流干渉を有効化
            motor_interference_scale: Scale factor for motor interference (µT/A)
                                     モーター干渉スケール係数（µT/A）
        """
        self.noise_std = noise_std
        self.enable_motor_interference = enable_motor_interference
        self.motor_interference_scale = motor_interference_scale

        # Hard iron offset (typical calibration error or PCB interference)
        # ハードアイアンオフセット（典型的なキャリブレーション誤差またはPCB干渉）
        if hard_iron is None:
            # Random offset simulating uncalibrated sensor
            # 未キャリブレーションセンサをシミュレートするランダムオフセット
            self.hard_iron = np.random.uniform(-10, 10, 3)
        else:
            self.hard_iron = np.array(hard_iron)

        # Soft iron matrix (scale factors and axis misalignment)
        # ソフトアイアン行列（スケール係数と軸ずれ）
        if soft_iron is None:
            # Slight scale error and cross-axis coupling
            # 微小なスケール誤差と軸間結合
            scale_error = np.diag(1.0 + np.random.uniform(-0.05, 0.05, 3))
            cross_axis = np.eye(3) + np.random.uniform(-0.02, 0.02, (3, 3))
            np.fill_diagonal(cross_axis, 1.0)
            self.soft_iron = scale_error @ cross_axis
        else:
            self.soft_iron = np.array(soft_iron)

        # Earth's magnetic field in NED frame
        # NED座標系での地球磁場
        if earth_field_ned is None:
            self.earth_field_ned = self._compute_earth_field()
        else:
            self.earth_field_ned = np.array(earth_field_ned)

    def _compute_earth_field(self) -> np.ndarray:
        """
        Compute Earth's magnetic field vector in NED frame.
        NED座標系での地球磁場ベクトルを計算

        Returns:
            Magnetic field [Bx, By, Bz] in NED frame (µT)
            NED座標系での磁場 [Bx, By, Bz]（µT）
        """
        # Convert inclination and declination to NED components
        # 伏角と偏角をNED成分に変換
        incl = np.deg2rad(self.EARTH_FIELD_INCLINATION_DEG)
        decl = np.deg2rad(self.EARTH_FIELD_DECLINATION_DEG)
        mag = self.EARTH_FIELD_MAGNITUDE_UT

        # Horizontal component
        # 水平成分
        h = mag * np.cos(incl)

        # NED components
        # NED成分
        bn = h * np.cos(decl)  # North
        be = h * np.sin(decl)  # East
        bd = mag * np.sin(incl)  # Down (positive into earth)

        return np.array([bn, be, bd])

    def read(
        self,
        rotation_matrix: np.ndarray,
        motor_currents: Optional[np.ndarray] = None,
    ) -> dict:
        """
        Simulate sensor reading with noise and distortions.
        ノイズと歪みを含むセンサ読み取りをシミュレート

        Args:
            rotation_matrix: Body-to-NED rotation matrix (3x3)
                            機体からNEDへの回転行列（3x3）
                            Transforms vectors from body to NED: v_ned = R @ v_body
            motor_currents: Motor currents [I1, I2, I3, I4] (A)
                           モーター電流 [I1, I2, I3, I4]（A）
                           Optional, used for interference simulation

        Returns:
            dict with mag_body (µT), mag_raw, heading_deg
            mag_body (µT), mag_raw, heading_deg を含む辞書
        """
        # Transform Earth field from NED to body frame
        # 地球磁場をNEDから機体座標系に変換
        # v_body = R^T @ v_ned
        R_ned_to_body = rotation_matrix.T
        mag_body_true = R_ned_to_body @ self.earth_field_ned

        # Apply soft iron distortion
        # ソフトアイアン歪みを適用
        mag_distorted = self.soft_iron @ mag_body_true

        # Add hard iron offset
        # ハードアイアンオフセットを追加
        mag_distorted = mag_distorted + self.hard_iron

        # Add motor interference if enabled
        # モーター干渉が有効な場合追加
        if self.enable_motor_interference and motor_currents is not None:
            interference = self._compute_motor_interference(motor_currents)
            mag_distorted = mag_distorted + interference

        # Add measurement noise
        # 測定ノイズを追加
        noise = np.random.normal(0, self.noise_std, 3)
        mag_measured = mag_distorted + noise

        # Quantize to sensor resolution
        # センサ分解能に量子化
        mag_raw = np.round(mag_measured / self.RESOLUTION_UT).astype(np.int16)
        mag_quantized = mag_raw * self.RESOLUTION_UT

        # Compute heading (yaw angle from magnetic north)
        # ヘディングを計算（磁北からのヨー角）
        heading_rad = np.arctan2(mag_measured[1], mag_measured[0])
        heading_deg = np.rad2deg(heading_rad)

        return {
            'mag_body': mag_measured,      # Measured field in body frame (µT)
            'mag_raw': mag_raw,            # Raw ADC counts
            'mag_true': mag_body_true,     # True field (no distortion)
            'heading_deg': heading_deg,    # Magnetic heading (degrees)
            'magnitude': np.linalg.norm(mag_measured),
        }

    def _compute_motor_interference(self, motor_currents: np.ndarray) -> np.ndarray:
        """
        Compute magnetic interference from motor currents.
        モーター電流からの磁気干渉を計算

        Simple model: each motor creates a field proportional to its current.
        シンプルモデル：各モーターはその電流に比例した磁場を生成。

        Args:
            motor_currents: Motor currents [I1, I2, I3, I4] (A)

        Returns:
            Interference field in body frame (µT)
        """
        # Motor positions relative to magnetometer (approximate, in body frame)
        # 地磁気センサに対するモーター位置（近似、機体座標系）
        # Assuming X-config quadrotor with motors at ±arm_length on x/y
        motor_positions = np.array([
            [0.05, -0.05, 0],   # Front-right
            [-0.05, -0.05, 0],  # Rear-right
            [-0.05, 0.05, 0],   # Rear-left
            [0.05, 0.05, 0],    # Front-left
        ])

        # Simple dipole-like interference model
        # シンプルな双極子ライクな干渉モデル
        interference = np.zeros(3)
        for i, (pos, current) in enumerate(zip(motor_positions, motor_currents)):
            # Distance from motor to magnetometer (assumed at origin)
            r = np.linalg.norm(pos)
            if r > 0.01:  # Avoid division by zero
                # Simplified: field proportional to I/r²
                field_strength = self.motor_interference_scale * current / (r * r)
                # Direction: primarily Z-axis (vertical motor shafts)
                interference[2] += field_strength * (1 if i % 2 == 0 else -1)
                # Small XY components based on position
                interference[0] += field_strength * 0.1 * np.sign(pos[0])
                interference[1] += field_strength * 0.1 * np.sign(pos[1])

        return interference

    def calibrate(
        self,
        measurements: np.ndarray,
    ) -> tuple:
        """
        Perform simple hard/soft iron calibration from measurements.
        測定値からシンプルなハード/ソフトアイアンキャリブレーションを実行

        Uses ellipsoid fitting to estimate distortion parameters.
        楕円体フィッティングを使用して歪みパラメータを推定。

        Args:
            measurements: Array of magnetometer readings (N x 3)
                         地磁気センサ読み取りの配列（N x 3）

        Returns:
            Tuple of (hard_iron_offset, soft_iron_matrix)
            (ハードアイアンオフセット, ソフトアイアン行列) のタプル

        Note:
            For accurate calibration, measurements should cover all orientations.
            正確なキャリブレーションには、全方位をカバーする測定が必要。
        """
        # Simple sphere fit for hard iron estimation
        # ハードアイアン推定のためのシンプルな球フィット
        center = np.mean(measurements, axis=0)

        # Center the data
        centered = measurements - center

        # Estimate soft iron from covariance (simplified)
        # 共分散からソフトアイアンを推定（簡略化）
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # Normalize to sphere
        scale = np.sqrt(np.mean(eigenvalues) / eigenvalues)
        soft_iron_inv = eigenvectors @ np.diag(scale) @ eigenvectors.T

        return center, soft_iron_inv

    def set_calibration(
        self,
        hard_iron: np.ndarray,
        soft_iron: np.ndarray,
    ):
        """
        Set calibration parameters.
        キャリブレーションパラメータを設定

        Args:
            hard_iron: Hard iron offset to subtract
                      減算するハードアイアンオフセット
            soft_iron: Soft iron correction matrix
                      ソフトアイアン補正行列
        """
        # Store inverse of distortion for correction
        # 補正用に歪みの逆を保存
        self.hard_iron = -hard_iron
        self.soft_iron = np.linalg.inv(soft_iron)

    def reset_distortion(self):
        """
        Reset to ideal sensor (no distortion).
        理想センサにリセット（歪みなし）
        """
        self.hard_iron = np.zeros(3)
        self.soft_iron = np.eye(3)
