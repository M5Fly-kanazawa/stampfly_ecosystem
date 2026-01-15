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
BMP280 Barometric Pressure Sensor Model
BMP280 気圧センサモデル

Simulates BMP280 barometric pressure sensor with realistic noise characteristics.
BMP280気圧センサを現実的なノイズ特性でシミュレート

Reference: BMP280 Datasheet, Bosch Sensortec
"""

import numpy as np


class Barometer:
    """
    BMP280 Barometric Pressure Sensor Simulation Model
    BMP280 気圧センサシミュレーションモデル

    Attributes:
        sea_level_pressure_pa: Reference sea level pressure (Pa)
                              基準海面気圧（Pa）
        pressure_noise_std: Pressure measurement noise std dev (Pa)
                           気圧測定ノイズ標準偏差（Pa）
        temperature_noise_std: Temperature measurement noise std dev (C)
                              温度測定ノイズ標準偏差（C）
        altitude_noise_std: Altitude calculation noise std dev (m)
                           高度計算ノイズ標準偏差（m）
    """

    # Physical constants
    # 物理定数
    GRAVITY = 9.80665  # m/s^2
    MOLAR_MASS_AIR = 0.0289644  # kg/mol
    GAS_CONSTANT = 8.31447  # J/(mol·K)
    LAPSE_RATE = 0.0065  # K/m (temperature lapse rate in troposphere)

    # BMP280 specifications (from datasheet)
    # BMP280仕様（データシートより）
    PRESSURE_RESOLUTION = 0.16  # Pa (typical at highest resolution)
    TEMPERATURE_RESOLUTION = 0.01  # °C
    ALTITUDE_RESOLUTION = 0.01  # m (derived)

    # Noise characteristics (typical values)
    # ノイズ特性（代表値）
    PRESSURE_NOISE_RMS = 1.3  # Pa RMS at X16 oversampling
    TEMPERATURE_NOISE_RMS = 0.005  # °C

    def __init__(
        self,
        sea_level_pressure_pa: float = 101325.0,
        temperature_c: float = 25.0,
        pressure_noise_std: float = 1.3,
        temperature_noise_std: float = 0.01,
        drift_rate: float = 0.0,
    ):
        """
        Initialize barometer model.
        気圧計モデルを初期化

        Args:
            sea_level_pressure_pa: Reference sea level pressure (Pa)
                                  基準海面気圧（Pa）
            temperature_c: Ambient temperature (°C)
                          周囲温度（°C）
            pressure_noise_std: Pressure noise std dev (Pa)
                               気圧ノイズ標準偏差（Pa）
            temperature_noise_std: Temperature noise std dev (°C)
                                  温度ノイズ標準偏差（°C）
            drift_rate: Pressure drift rate (Pa/s)
                       気圧ドリフト率（Pa/s）
        """
        self.sea_level_pressure_pa = sea_level_pressure_pa
        self.ambient_temperature_c = temperature_c
        self.pressure_noise_std = pressure_noise_std
        self.temperature_noise_std = temperature_noise_std
        self.drift_rate = drift_rate

        # Internal state
        # 内部状態
        self._accumulated_drift = 0.0
        self._time = 0.0

    def pressure_from_altitude(self, altitude_m: float) -> float:
        """
        Calculate pressure from altitude using barometric formula.
        気圧公式を用いて高度から気圧を計算

        Uses the international barometric formula for troposphere.
        対流圏の国際気圧公式を使用

        Args:
            altitude_m: Altitude above sea level (m)
                       海面上高度（m）

        Returns:
            Pressure in Pascal
            気圧（Pascal）
        """
        # International barometric formula (troposphere, h < 11000m)
        # P = P0 * (1 - L*h/T0)^(g*M/(R*L))
        T0 = self.ambient_temperature_c + 273.15  # Convert to Kelvin
        exponent = (self.GRAVITY * self.MOLAR_MASS_AIR) / (self.GAS_CONSTANT * self.LAPSE_RATE)

        pressure = self.sea_level_pressure_pa * (
            (1 - self.LAPSE_RATE * altitude_m / T0) ** exponent
        )
        return pressure

    def altitude_from_pressure(self, pressure_pa: float) -> float:
        """
        Calculate altitude from pressure using barometric formula.
        気圧公式を用いて気圧から高度を計算

        Args:
            pressure_pa: Pressure in Pascal
                        気圧（Pascal）

        Returns:
            Altitude in meters
            高度（m）
        """
        T0 = self.ambient_temperature_c + 273.15  # Convert to Kelvin
        exponent = (self.GAS_CONSTANT * self.LAPSE_RATE) / (self.GRAVITY * self.MOLAR_MASS_AIR)

        altitude = (T0 / self.LAPSE_RATE) * (
            1 - (pressure_pa / self.sea_level_pressure_pa) ** exponent
        )
        return altitude

    def read(self, true_altitude_m: float, dt: float = 0.02) -> dict:
        """
        Simulate sensor reading with noise.
        ノイズを含むセンサ読み取りをシミュレート

        Args:
            true_altitude_m: True altitude above sea level (m)
                            真の海面上高度（m）
                            Note: In NED frame, altitude is typically negative (down)
                            注意：NED座標系では高度は通常負（下向き）
            dt: Time step (s) for drift simulation
                ドリフトシミュレーション用時間ステップ（s）

        Returns:
            dict with pressure_pa, temperature_c, altitude_m
            pressure_pa, temperature_c, altitude_m を含む辞書
        """
        # Update drift
        # ドリフト更新
        self._accumulated_drift += self.drift_rate * dt
        self._time += dt

        # Calculate true pressure at altitude
        # 高度における真の気圧を計算
        # Note: Convert from NED (down positive) to altitude (up positive)
        # 注意：NED（下向き正）から高度（上向き正）に変換
        altitude_up = -true_altitude_m  # NED z is down, altitude is up
        true_pressure = self.pressure_from_altitude(altitude_up)

        # Add noise and drift
        # ノイズとドリフトを追加
        measured_pressure = (
            true_pressure
            + np.random.normal(0, self.pressure_noise_std)
            + self._accumulated_drift
        )

        # Temperature with noise
        # ノイズ付き温度
        measured_temperature = (
            self.ambient_temperature_c
            + np.random.normal(0, self.temperature_noise_std)
        )

        # Calculate altitude from measured pressure
        # 測定気圧から高度を計算
        measured_altitude = self.altitude_from_pressure(measured_pressure)

        return {
            'pressure_pa': measured_pressure,
            'temperature_c': measured_temperature,
            'altitude_m': measured_altitude,  # Up positive
        }

    def reset_drift(self):
        """
        Reset accumulated drift.
        累積ドリフトをリセット
        """
        self._accumulated_drift = 0.0

    def set_sea_level_pressure(self, pressure_pa: float):
        """
        Set reference sea level pressure (for calibration).
        基準海面気圧を設定（キャリブレーション用）

        Args:
            pressure_pa: Sea level pressure (Pa)
                        海面気圧（Pa）
        """
        self.sea_level_pressure_pa = pressure_pa
