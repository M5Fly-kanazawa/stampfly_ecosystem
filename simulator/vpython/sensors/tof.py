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
VL53L3CX Time-of-Flight Sensor Model
VL53L3CX ToFセンサモデル

Simulates VL53L3CX laser-ranging sensor with realistic noise and
distance-dependent characteristics. VL53L3CX is an improved version
with multi-target detection capability.
VL53L3CXレーザー測距センサをリアリスティックなノイズと
距離依存特性でシミュレート。VL53L3CXはマルチターゲット検出機能を
持つ改良版。

Reference: VL53L3CX Datasheet, STMicroelectronics
"""

import numpy as np
from typing import Optional


class ToF:
    """
    VL53L3CX Time-of-Flight Sensor Simulation Model
    VL53L3CX ToFセンサシミュレーションモデル

    Simulates laser ranging sensor with:
    - Distance-dependent noise (increases with range)
    - Field of view effects (cone-shaped measurement area)
    - Range limits and saturation
    - Signal strength (return signal rate)
    - Multi-target detection (VL53L3CX feature)

    レーザー測距センサを以下でシミュレート：
    - 距離依存ノイズ（距離とともに増加）
    - 視野角効果（円錐状の測定領域）
    - 測定範囲制限と飽和
    - 信号強度（リターン信号レート）
    - マルチターゲット検出（VL53L3CX機能）

    Attributes:
        min_range_m: Minimum measurable range (m)
                    最小測定範囲（m）
        max_range_m: Maximum measurable range (m)
                    最大測定範囲（m）
        fov_deg: Field of view (degrees)
                視野角（度）
    """

    # VL53L3CX specifications (from datasheet)
    # VL53L3CX仕様（データシートより）
    MIN_RANGE_MM = 30      # 30mm minimum
    MAX_RANGE_MM = 3000    # 3m maximum (improved over VL53L0X)
    RESOLUTION_MM = 1      # 1mm resolution
    FOV_DEG = 25           # 25° field of view (typical)

    # Noise characteristics (from datasheet)
    # ノイズ特性（データシートより）
    # Ranging accuracy: ±3% at <400mm, ±5% at <1200mm, ±8% at >1200mm
    NOISE_PERCENT_SHORT = 3.0   # <400mm
    NOISE_PERCENT_MEDIUM = 5.0  # 400-1200mm
    NOISE_PERCENT_LONG = 8.0    # >1200mm

    # Signal quality thresholds
    SIGNAL_RATE_MIN = 0.1  # Minimum valid signal rate (MCPS)
    SIGMA_LIMIT = 15       # Maximum sigma estimate for valid reading (mm)

    def __init__(
        self,
        min_range_m: float = 0.03,
        max_range_m: float = 2.0,
        fov_deg: float = 25.0,
        base_noise_mm: float = 5.0,
        enable_fov_effects: bool = True,
        surface_reflectivity: float = 0.8,
    ):
        """
        Initialize ToF sensor model.
        ToFセンサモデルを初期化

        Args:
            min_range_m: Minimum range (m)
                        最小測定範囲（m）
            max_range_m: Maximum range (m)
                        最大測定範囲（m）
            fov_deg: Field of view (degrees)
                    視野角（度）
            base_noise_mm: Base noise level (mm)
                          基本ノイズレベル（mm）
            enable_fov_effects: Enable field of view simulation
                               視野角シミュレーションを有効化
            surface_reflectivity: Ground surface reflectivity (0-1)
                                 地面表面反射率（0-1）
        """
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.fov_deg = fov_deg
        self.base_noise_mm = base_noise_mm
        self.enable_fov_effects = enable_fov_effects
        self.surface_reflectivity = surface_reflectivity

        # Derived parameters
        # 導出パラメータ
        self.fov_rad = np.deg2rad(fov_deg)

    def read(
        self,
        true_distance_m: float,
        attitude_rad: Optional[np.ndarray] = None,
        ground_height_m: float = 0.0,
    ) -> dict:
        """
        Simulate sensor reading with noise.
        ノイズを含むセンサ読み取りをシミュレート

        Args:
            true_distance_m: True distance to target (m)
                            ターゲットまでの真の距離（m）
                            For downward-facing sensor, this is height above ground
            attitude_rad: Vehicle attitude [roll, pitch, yaw] (rad)
                         機体姿勢 [roll, pitch, yaw]（rad）
                         Used for FOV effects calculation
            ground_height_m: Height of ground surface (m, usually 0)
                            地面高さ（m、通常0）

        Returns:
            dict with distance_m, distance_mm, valid, signal_rate, sigma
            distance_m, distance_mm, valid, signal_rate, sigma を含む辞書
        """
        # Apply FOV effects if attitude is provided
        # 姿勢が提供された場合、視野角効果を適用
        effective_distance = true_distance_m
        if self.enable_fov_effects and attitude_rad is not None:
            effective_distance = self._apply_fov_effects(
                true_distance_m, attitude_rad
            )

        # Check range limits
        # 測定範囲制限をチェック
        out_of_range = (effective_distance < self.min_range_m or
                        effective_distance > self.max_range_m)

        # Calculate distance-dependent noise
        # 距離依存ノイズを計算
        noise_mm = self._calculate_noise(effective_distance)

        # Add noise to measurement
        # 測定にノイズを追加
        distance_mm = effective_distance * 1000.0 + np.random.normal(0, noise_mm)

        # Quantize to resolution
        # 分解能に量子化
        distance_mm = np.round(distance_mm / self.RESOLUTION_MM) * self.RESOLUTION_MM

        # Clamp to valid range
        # 有効範囲に制限
        distance_mm = np.clip(
            distance_mm,
            self.min_range_m * 1000,
            self.max_range_m * 1000
        )

        # Calculate signal quality metrics
        # 信号品質メトリクスを計算
        signal_rate = self._calculate_signal_rate(effective_distance)
        sigma = self._calculate_sigma(effective_distance, signal_rate)

        # Determine validity
        # 有効性を判定
        valid = (
            not out_of_range and
            signal_rate > self.SIGNAL_RATE_MIN and
            sigma < self.SIGMA_LIMIT
        )

        # Occasionally simulate failed readings
        # 時々読み取り失敗をシミュレート
        if np.random.random() < 0.01:  # 1% failure rate
            valid = False
            distance_mm = 0

        return {
            'distance_m': distance_mm / 1000.0,
            'distance_mm': int(distance_mm),
            'valid': valid,
            'signal_rate': signal_rate,
            'sigma': sigma,
            'range_status': 0 if valid else (1 if out_of_range else 2),
        }

    def _calculate_noise(self, distance_m: float) -> float:
        """
        Calculate distance-dependent noise.
        距離依存ノイズを計算

        Args:
            distance_m: Distance (m)

        Returns:
            Noise standard deviation (mm)
        """
        distance_mm = distance_m * 1000.0

        # Distance-dependent noise percentage
        # 距離依存ノイズ割合
        if distance_mm < 400:
            noise_percent = self.NOISE_PERCENT_SHORT
        elif distance_mm < 1200:
            noise_percent = self.NOISE_PERCENT_MEDIUM
        else:
            noise_percent = self.NOISE_PERCENT_LONG

        # Noise increases with distance
        # ノイズは距離とともに増加
        range_noise = distance_mm * noise_percent / 100.0

        # Combine with base noise
        # 基本ノイズと組み合わせ
        total_noise = np.sqrt(self.base_noise_mm**2 + range_noise**2)

        # Reduce noise with better reflectivity
        # 反射率が高いとノイズ減少
        total_noise /= np.sqrt(self.surface_reflectivity + 0.1)

        return total_noise

    def _calculate_signal_rate(self, distance_m: float) -> float:
        """
        Calculate return signal rate (MCPS - Mega Counts Per Second).
        リターン信号レート（MCPS - メガカウント/秒）を計算

        Signal rate decreases with distance squared.
        信号レートは距離の2乗に反比例して減少。

        Args:
            distance_m: Distance (m)

        Returns:
            Signal rate (MCPS)
        """
        # Base signal rate at 0.5m
        base_rate = 5.0  # MCPS

        # Inverse square law with surface reflectivity
        # 表面反射率を含む逆2乗則
        rate = base_rate * self.surface_reflectivity / (distance_m / 0.5) ** 2

        # Add some noise to signal rate
        rate += np.random.normal(0, rate * 0.1)

        return max(0.01, rate)

    def _calculate_sigma(self, distance_m: float, signal_rate: float) -> float:
        """
        Calculate sigma (standard deviation estimate) for the measurement.
        測定のシグマ（標準偏差推定）を計算

        Args:
            distance_m: Distance (m)
            signal_rate: Signal rate (MCPS)

        Returns:
            Sigma estimate (mm)
        """
        # Sigma increases with distance and decreases with signal strength
        # シグマは距離とともに増加、信号強度とともに減少
        base_sigma = 3.0  # mm at close range with good signal

        distance_factor = distance_m / 0.5
        signal_factor = 1.0 / np.sqrt(max(signal_rate, 0.1))

        sigma = base_sigma * distance_factor * signal_factor

        # Add noise
        sigma += np.random.normal(0, sigma * 0.1)

        return max(1.0, sigma)

    def _apply_fov_effects(
        self,
        vertical_distance: float,
        attitude_rad: np.ndarray,
    ) -> float:
        """
        Apply field of view effects based on attitude.
        姿勢に基づいて視野角効果を適用

        When tilted, the sensor sees a slant range that's longer than
        the vertical distance.
        傾いている時、センサは垂直距離より長い斜距離を見る。

        Args:
            vertical_distance: Vertical distance to ground (m)
            attitude_rad: Vehicle attitude [roll, pitch, yaw] (rad)

        Returns:
            Effective measured distance (m)
        """
        roll = attitude_rad[0]
        pitch = attitude_rad[1]

        # Total tilt angle from vertical
        # 垂直からの総傾斜角
        total_tilt = np.sqrt(roll**2 + pitch**2)

        # If tilt exceeds half FOV, measurement becomes unreliable
        # 傾斜が視野角の半分を超えると測定が信頼できなくなる
        if total_tilt > self.fov_rad / 2:
            # Return larger distance (unreliable reading)
            return vertical_distance * (1.5 + np.random.random())

        # Slant range calculation
        # 斜距離計算
        # d_slant = d_vertical / cos(tilt)
        if np.cos(total_tilt) > 0.1:
            slant_range = vertical_distance / np.cos(total_tilt)
        else:
            slant_range = vertical_distance * 10  # Very unreliable

        return slant_range

    def reset(self):
        """Reset sensor state (no persistent state currently) / センサ状態をリセット"""
        pass
