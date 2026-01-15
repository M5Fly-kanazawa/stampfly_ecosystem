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
PID Controller with Incomplete Derivative Filter
不完全微分フィルタ付きPIDコントローラ

Implements PID control matching firmware sf_algo_pid:
ファームウェア sf_algo_pid と一致するPID制御を実装：

- Bilinear transform (Tustin) for discretization
- Incomplete derivative filter (eta coefficient)
- Back-calculation anti-windup
- Derivative-on-measurement option
- Output limiting

Transfer function:
C(s) = Kp(1 + 1/(Ti·s) + Td·s/(η·Td·s + 1))
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class PIDGains:
    """
    PID gains in engineering form.
    工学形式のPIDゲイン

    Uses Ti (integral time) and Td (derivative time) instead of Ki and Kd.
    Ki と Kd の代わりに Ti（積分時間）と Td（微分時間）を使用。

    Attributes:
        Kp: Proportional gain / 比例ゲイン
        Ti: Integral time (s), <=0 disables integral / 積分時間（秒）、0以下で無効
        Td: Derivative time (s), <=0 disables derivative / 微分時間（秒）、0以下で無効
        eta: Incomplete derivative filter coefficient / 不完全微分フィルタ係数
    """
    Kp: float = 1.0
    Ti: float = 0.0   # <=0 disables integral
    Td: float = 0.0   # <=0 disables derivative
    eta: float = 0.125  # Incomplete derivative filter coefficient


class PID:
    """
    PID Controller with Incomplete Derivative Filter.
    不完全微分フィルタ付きPIDコントローラ

    Firmware-compatible implementation with:
    ファームウェア互換の実装：

    - Bilinear (Tustin) transform for discrete-time implementation
    - Incomplete derivative filter: Td·s / (η·Td·s + 1)
    - Back-calculation anti-windup
    - Derivative-on-measurement to avoid setpoint kicks
    - Output limiting

    Usage:
        pid = PID(Kp=0.65, Ti=0.7, Td=0.01)
        output = pid.update(setpoint, measurement, dt)
    """

    def __init__(
        self,
        Kp: float = 1.0,
        Ti: float = 0.0,
        Td: float = 0.0,
        eta: float = 0.125,
        output_min: float = -float('inf'),
        output_max: float = float('inf'),
        derivative_on_measurement: bool = True,
    ):
        """
        Initialize PID controller.
        PIDコントローラを初期化

        Args:
            Kp: Proportional gain / 比例ゲイン
            Ti: Integral time (s), <=0 disables / 積分時間、0以下で無効
            Td: Derivative time (s), <=0 disables / 微分時間、0以下で無効
            eta: Incomplete derivative filter coefficient (0.1-0.2 typical)
                 不完全微分フィルタ係数（典型的には0.1-0.2）
            output_min: Minimum output value / 最小出力値
            output_max: Maximum output value / 最大出力値
            derivative_on_measurement: Use measurement for derivative (avoids kick)
                                      微分に測定値を使用（キック回避）
        """
        self.Kp = Kp
        self.Ti = Ti
        self.Td = Td
        self.eta = eta
        self.output_min = output_min
        self.output_max = output_max
        self.derivative_on_measurement = derivative_on_measurement

        # Calculate tracking time constant for anti-windup
        # アンチワインドアップ用の追跡時定数を計算
        if Ti > 0 and Td > 0:
            self.Tt = np.sqrt(Ti * Td)
        elif Ti > 0:
            self.Tt = Ti
        else:
            self.Tt = 1.0  # Default

        # State variables
        self._integral = 0.0
        self._deriv_filtered = 0.0
        self._prev_error = 0.0
        self._prev_measurement = 0.0
        self._prev_deriv_input = 0.0
        self._initialized = False

    @classmethod
    def from_gains(cls, gains: PIDGains, **kwargs) -> 'PID':
        """
        Create PID from PIDGains dataclass.
        PIDGainsデータクラスからPIDを作成
        """
        return cls(
            Kp=gains.Kp,
            Ti=gains.Ti,
            Td=gains.Td,
            eta=gains.eta,
            **kwargs
        )

    def update(
        self,
        setpoint: float,
        measurement: float,
        dt: float,
    ) -> float:
        """
        Update PID controller and get output.
        PIDコントローラを更新して出力を取得

        Args:
            setpoint: Desired value / 目標値
            measurement: Current value / 現在値
            dt: Time step (s) / 時間ステップ（秒）

        Returns:
            Control output (limited) / 制御出力（制限付き）
        """
        if dt <= 0:
            return 0.0

        # Initialize on first call
        if not self._initialized:
            self._prev_error = setpoint - measurement
            self._prev_measurement = measurement
            self._prev_deriv_input = -measurement if self.derivative_on_measurement else self._prev_error
            self._initialized = True

        error = setpoint - measurement

        # ==========================================
        # Proportional term
        # 比例項
        # ==========================================
        P = self.Kp * error

        # ==========================================
        # Integral term (Tustin/Bilinear transform)
        # 積分項（Tustin/双一次変換）
        # ==========================================
        I = 0.0
        if self.Ti > 0:
            # Trapezoidal integration: ∫e dt ≈ (dt/2)(e[k] + e[k-1])
            self._integral += (dt / (2.0 * self.Ti)) * (error + self._prev_error)
            I = self.Kp * self._integral

        # ==========================================
        # Derivative term (Incomplete derivative filter)
        # 微分項（不完全微分フィルタ）
        # ==========================================
        D = 0.0
        if self.Td > 0:
            # Derivative input: error or -measurement (D-on-M)
            if self.derivative_on_measurement:
                deriv_input = -measurement
            else:
                deriv_input = error

            # Incomplete derivative filter coefficients (Bilinear transform)
            # D(s) = Td·s / (η·Td·s + 1)
            # Using bilinear: s = (2/dt)(z-1)/(z+1)
            alpha = 2.0 * self.eta * self.Td / dt
            deriv_a = (alpha - 1.0) / (alpha + 1.0)
            deriv_b = 2.0 * self.Td / ((alpha + 1.0) * dt)

            # Filter update
            self._deriv_filtered = (
                deriv_a * self._deriv_filtered +
                deriv_b * (deriv_input - self._prev_deriv_input)
            )
            D = self.Kp * self._deriv_filtered

            self._prev_deriv_input = deriv_input

        # ==========================================
        # Total output
        # 合計出力
        # ==========================================
        output_unlimited = P + I + D

        # Apply output limits
        output_limited = np.clip(output_unlimited, self.output_min, self.output_max)

        # ==========================================
        # Anti-windup (Back-calculation)
        # アンチワインドアップ（バックカルキュレーション）
        # ==========================================
        if self.Ti > 0:
            saturation = output_limited - output_unlimited
            if saturation != 0.0 and self.Kp != 0.0:
                self._integral += saturation * (dt / self.Tt) / self.Kp

        # Store previous values
        self._prev_error = error
        self._prev_measurement = measurement

        return output_limited

    def reset(self):
        """Reset controller state / コントローラ状態をリセット"""
        self._integral = 0.0
        self._deriv_filtered = 0.0
        self._prev_error = 0.0
        self._prev_measurement = 0.0
        self._prev_deriv_input = 0.0
        self._initialized = False

    def set_integral(self, value: float):
        """Set integral term directly / 積分項を直接設定"""
        self._integral = value

    @property
    def integral(self) -> float:
        """Get current integral value / 現在の積分値を取得"""
        return self._integral

    def get_terms(self) -> tuple:
        """
        Get individual PID terms (for debugging).
        個別のPID項を取得（デバッグ用）

        Returns:
            (P, I, D) terms
        """
        P = self.Kp * self._prev_error
        I = self.Kp * self._integral if self.Ti > 0 else 0.0
        D = self.Kp * self._deriv_filtered if self.Td > 0 else 0.0
        return P, I, D


# =============================================================================
# Legacy compatibility wrapper
# レガシー互換ラッパー
# =============================================================================

class LegacyPID:
    """
    Simple PID for backward compatibility with original simulator.
    元のシミュレータとの後方互換性のためのシンプルなPID

    This is the original implementation from stampfly_sim.
    これは stampfly_sim からの元の実装です。
    """

    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, ref: float, measured_value: float, step_time: float) -> float:
        error = ref - measured_value
        self.integral += error * step_time
        derivative = (error - self.prev_error) / step_time if step_time > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
