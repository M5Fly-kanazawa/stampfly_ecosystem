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
INA3221 Power Monitor Sensor Model
INA3221 電源モニタセンサモデル

Simulates INA3221 3-channel current/voltage monitor with battery
discharge model and low voltage warning.
INA3221 3チャンネル電流/電圧モニタをバッテリー放電モデルと
低電圧警告でシミュレート。

Reference: INA3221 Datasheet, Texas Instruments
"""

import numpy as np
from typing import Optional


class PowerMonitor:
    """
    INA3221 Power Monitor Sensor Simulation Model
    INA3221 電源モニタセンサシミュレーションモデル

    Simulates battery voltage/current monitoring with:
    - LiPo battery discharge curve
    - Current measurement based on motor power
    - Low battery warning
    - Measurement noise

    バッテリー電圧/電流監視を以下でシミュレート：
    - LiPoバッテリー放電曲線
    - モーター出力に基づく電流測定
    - 低バッテリー警告
    - 測定ノイズ

    Attributes:
        full_voltage: Fully charged battery voltage (V)
                     満充電バッテリー電圧（V）
        empty_voltage: Empty battery voltage (V)
                      空バッテリー電圧（V）
        capacity_mah: Battery capacity (mAh)
                     バッテリー容量（mAh）
    """

    # INA3221 specifications (from datasheet)
    # INA3221仕様（データシートより）
    VOLTAGE_LSB_MV = 8.0       # 8mV per LSB for bus voltage
    CURRENT_LSB_UA = 40.0      # 40µA per LSB for shunt voltage (with 0.1Ω)
    NUM_CHANNELS = 3

    # LiPo battery characteristics (1S)
    # LiPoバッテリー特性（1S）
    FULL_VOLTAGE = 4.2      # V (fully charged)
    NOMINAL_VOLTAGE = 3.7   # V (nominal)
    EMPTY_VOLTAGE = 3.3     # V (empty, should not discharge below)
    LOW_VOLTAGE_THRESHOLD = 3.4  # V (low battery warning)

    # Noise characteristics
    # ノイズ特性
    VOLTAGE_NOISE_STD = 0.005   # V
    CURRENT_NOISE_STD = 5.0     # mA

    def __init__(
        self,
        initial_voltage: float = 4.2,
        capacity_mah: float = 300.0,
        internal_resistance_ohm: float = 0.1,
        shunt_resistor_ohm: float = 0.1,
        voltage_noise_std: float = 0.005,
        current_noise_std: float = 5.0,
    ):
        """
        Initialize power monitor sensor model.
        電源モニタセンサモデルを初期化

        Args:
            initial_voltage: Initial battery voltage (V)
                            初期バッテリー電圧（V）
            capacity_mah: Battery capacity (mAh)
                         バッテリー容量（mAh）
            internal_resistance_ohm: Battery internal resistance (Ω)
                                    バッテリー内部抵抗（Ω）
            shunt_resistor_ohm: Shunt resistor value (Ω)
                               シャント抵抗値（Ω）
            voltage_noise_std: Voltage measurement noise std dev (V)
                              電圧測定ノイズ標準偏差（V）
            current_noise_std: Current measurement noise std dev (mA)
                              電流測定ノイズ標準偏差（mA）
        """
        self.capacity_mah = capacity_mah
        self.internal_resistance = internal_resistance_ohm
        self.shunt_resistor = shunt_resistor_ohm
        self.voltage_noise_std = voltage_noise_std
        self.current_noise_std = current_noise_std

        # Battery state
        # バッテリー状態
        self._open_circuit_voltage = initial_voltage
        self._remaining_capacity_mah = self._voltage_to_capacity(initial_voltage)
        self._current_ma = 0.0
        self._time_s = 0.0

    def _voltage_to_capacity(self, voltage: float) -> float:
        """
        Convert open-circuit voltage to remaining capacity.
        開回路電圧を残容量に変換

        Uses a simplified LiPo discharge curve.
        簡略化したLiPo放電曲線を使用。

        Args:
            voltage: Open-circuit voltage (V)

        Returns:
            Remaining capacity (mAh)
        """
        # Clamp voltage to valid range
        v = np.clip(voltage, self.EMPTY_VOLTAGE, self.FULL_VOLTAGE)

        # Simplified discharge curve (mostly linear in middle range)
        # 簡略化した放電曲線（中間領域はほぼ線形）
        percent = (v - self.EMPTY_VOLTAGE) / (self.FULL_VOLTAGE - self.EMPTY_VOLTAGE)

        # Apply slight non-linearity (steep at ends)
        # 若干の非線形性を適用（両端で急峻）
        if percent > 0.9:
            # Steep drop at full charge
            percent = 0.9 + (percent - 0.9) * 0.5
        elif percent < 0.1:
            # Steep drop at empty
            percent = percent * 0.5

        return percent * self.capacity_mah

    def _capacity_to_voltage(self, capacity_mah: float) -> float:
        """
        Convert remaining capacity to open-circuit voltage.
        残容量を開回路電圧に変換

        Args:
            capacity_mah: Remaining capacity (mAh)

        Returns:
            Open-circuit voltage (V)
        """
        percent = np.clip(capacity_mah / self.capacity_mah, 0.0, 1.0)

        # Inverse of the non-linear curve
        # 非線形曲線の逆
        if percent > 0.95:
            adj_percent = 0.9 + (percent - 0.9) * 2
        elif percent < 0.05:
            adj_percent = percent * 2
        else:
            adj_percent = percent

        voltage = self.EMPTY_VOLTAGE + adj_percent * (self.FULL_VOLTAGE - self.EMPTY_VOLTAGE)
        return np.clip(voltage, self.EMPTY_VOLTAGE, self.FULL_VOLTAGE)

    def update(self, current_ma: float, dt: float):
        """
        Update battery state based on current draw.
        電流消費に基づいてバッテリー状態を更新

        Args:
            current_ma: Current draw (mA)
                       電流消費（mA）
            dt: Time step (s)
                時間ステップ（s）
        """
        self._current_ma = current_ma
        self._time_s += dt

        # Update remaining capacity (Coulomb counting)
        # 残容量を更新（クーロンカウンティング）
        consumed_mah = current_ma * dt / 3600.0  # Convert s to h
        self._remaining_capacity_mah -= consumed_mah
        self._remaining_capacity_mah = max(0.0, self._remaining_capacity_mah)

        # Update open-circuit voltage based on remaining capacity
        # 残容量に基づいて開回路電圧を更新
        self._open_circuit_voltage = self._capacity_to_voltage(self._remaining_capacity_mah)

    def read(self, current_ma: Optional[float] = None) -> dict:
        """
        Simulate sensor reading with noise.
        ノイズを含むセンサ読み取りをシミュレート

        Args:
            current_ma: Current draw (mA), uses last value if None
                       電流消費（mA）、Noneの場合は前回値を使用

        Returns:
            dict with voltage_v, current_ma, power_mw, battery_percent, low_battery
            voltage_v, current_ma, power_mw, battery_percent, low_battery を含む辞書
        """
        if current_ma is not None:
            self._current_ma = current_ma

        # Calculate terminal voltage (OCV - IR drop)
        # 端子電圧を計算（OCV - IR降下）
        ir_drop = self._current_ma * self.internal_resistance / 1000.0  # V
        terminal_voltage = self._open_circuit_voltage - ir_drop

        # Add measurement noise
        # 測定ノイズを追加
        measured_voltage = terminal_voltage + np.random.normal(0, self.voltage_noise_std)
        measured_current = self._current_ma + np.random.normal(0, self.current_noise_std)

        # Clamp to valid ranges
        # 有効範囲に制限
        measured_voltage = max(0.0, measured_voltage)
        measured_current = max(0.0, measured_current)

        # Calculate power
        # 電力を計算
        power_mw = measured_voltage * measured_current

        # Calculate battery percentage
        # バッテリー残量を計算
        battery_percent = (self._remaining_capacity_mah / self.capacity_mah) * 100.0

        # Check low battery
        # 低バッテリーをチェック
        low_battery = measured_voltage < self.LOW_VOLTAGE_THRESHOLD

        return {
            'voltage_v': measured_voltage,
            'current_ma': measured_current,
            'power_mw': power_mw,
            'battery_percent': battery_percent,
            'low_battery': low_battery,
            'remaining_capacity_mah': self._remaining_capacity_mah,
            'open_circuit_voltage_v': self._open_circuit_voltage,
        }

    def estimate_current_from_motors(
        self,
        motor_outputs: np.ndarray,
        hover_current_ma: float = 800.0,
        idle_current_ma: float = 100.0,
    ) -> float:
        """
        Estimate battery current from motor outputs.
        モーター出力からバッテリー電流を推定

        Args:
            motor_outputs: Motor output values [0-1] for each motor
                          各モーターの出力値 [0-1]
            hover_current_ma: Current at hover (all motors ~50%)
                             ホバー時の電流（全モーター約50%）
            idle_current_ma: Idle current (electronics only)
                            アイドル電流（電子回路のみ）

        Returns:
            Estimated total current (mA)
            推定総電流（mA）
        """
        # Motor current is roughly proportional to thrust (output^1.5)
        # モーター電流はスラストにほぼ比例（出力^1.5）
        total_motor_output = np.sum(np.power(np.clip(motor_outputs, 0, 1), 1.5))

        # Scale to hover current (4 motors at 50% ≈ 4 * 0.35 = 1.4)
        motor_current = (total_motor_output / 1.4) * (hover_current_ma - idle_current_ma)

        return idle_current_ma + motor_current

    def reset(self, voltage: float = 4.2):
        """
        Reset battery to specified voltage.
        バッテリーを指定電圧にリセット

        Args:
            voltage: Reset voltage (V)
        """
        self._open_circuit_voltage = voltage
        self._remaining_capacity_mah = self._voltage_to_capacity(voltage)
        self._current_ma = 0.0
        self._time_s = 0.0

    @property
    def is_empty(self) -> bool:
        """Check if battery is empty / バッテリーが空かチェック"""
        return self._open_circuit_voltage <= self.EMPTY_VOLTAGE

    @property
    def is_low(self) -> bool:
        """Check if battery is low / バッテリーが低いかチェック"""
        return self._open_circuit_voltage < self.LOW_VOLTAGE_THRESHOLD

    @property
    def flight_time_remaining_s(self) -> float:
        """
        Estimate remaining flight time based on current draw.
        現在の電流消費に基づいて残り飛行時間を推定

        Returns:
            Remaining flight time (s), inf if current is 0
        """
        if self._current_ma <= 0:
            return float('inf')

        remaining_hours = self._remaining_capacity_mah / self._current_ma
        return remaining_hours * 3600.0
