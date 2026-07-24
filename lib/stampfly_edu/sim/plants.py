"""
Plant models for control system simulation
制御システムシミュレーション用のプラントモデル

Provides simple plant models that students can use to understand
feedback control before working with the real drone.
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional

# Import StampFly physical parameters
# StampFly 物理パラメータをインポート
import sys
from pathlib import Path
_tools_dir = Path(__file__).parent.parent.parent.parent / "tools" / "sysid"
if str(_tools_dir) not in sys.path:
    sys.path.insert(0, str(_tools_dir))

try:
    from defaults import get_flat_defaults
    _DEFAULTS = get_flat_defaults()
except ImportError:
    # Fallback values must be kept in sync with tools/sysid/defaults.py
    # get_flat_defaults() (checked by `sf params check`) — this dict is only
    # used when tools/sysid is not importable (e.g. standalone script use),
    # so it cannot pull values from defaults.py automatically and has to be
    # updated by hand whenever the measured parameters change.
    # このフォールバック値は tools/sysid/defaults.py の get_flat_defaults() と
    # 同期必須（`sf params check` が検査）— tools/sysid をインポートできない
    # 場合（スタンドアロン実行時等）のみ使われるため defaults.py から自動取得
    # できず、実測パラメータが更新されるたびに手動更新が必要。
    _DEFAULTS = {
        "mass": 0.037,  # 実測36.8g、2026-07-24統一（旧0.035は更新漏れ）
        "Ixx": 9.16e-6, "Iyy": 13.3e-6, "Izz": 20.4e-6,
        "arm_length": 0.023, "Ct": 6.7e-9,  # 2026-07-15実測（現行プロペラ）
        "tau_m": 0.02,
        "max_thrust": 0.15, "g": 9.80665,
    }


class FirstOrderPlant:
    """First-order plant: G(s) = K / (tau*s + 1)

    1次系プラント: G(s) = K / (tau*s + 1)

    Args:
        K: DC gain / DC ゲイン
        tau: Time constant (s) / 時定数（秒）

    Usage:
        >>> plant = FirstOrderPlant(K=2.0, tau=0.5)
        >>> plant.step(0.01)  # Apply step, advance 10ms
        0.039...
    """

    def __init__(self, K: float = 1.0, tau: float = 1.0):
        self.K = K
        self.tau = tau
        self.y = 0.0

    def step(self, u: float, dt: float = 0.001) -> float:
        """Advance one time step.

        1タイムステップ進める。

        Args:
            u: Input / 入力
            dt: Time step (s) / 時間ステップ（秒）

        Returns:
            Output / 出力
        """
        # dy/dt = (K*u - y) / tau
        self.y += (self.K * u - self.y) / self.tau * dt
        return self.y

    def reset(self):
        self.y = 0.0

    def __repr__(self):
        return f"FirstOrderPlant(K={self.K}, tau={self.tau})"


class SecondOrderPlant:
    """Second-order plant: G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)

    2次系プラント: G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)

    Args:
        wn: Natural frequency (rad/s) / 固有振動数
        zeta: Damping ratio / 減衰比
        gain: DC gain / DC ゲイン

    Usage:
        >>> plant = SecondOrderPlant(wn=10.0, zeta=0.7)
        >>> y = 0.0
        >>> for _ in range(100):
        ...     y = plant.step(1.0, 0.01)
        >>> abs(y - 1.0) < 0.1  # Should converge near 1.0
        True
    """

    def __init__(self, wn: float = 10.0, zeta: float = 0.7, gain: float = 1.0):
        self.wn = wn
        self.zeta = zeta
        self.gain = gain
        self.y = 0.0
        self.dy = 0.0

    def step(self, u: float, dt: float = 0.001) -> float:
        """Advance one time step.

        1タイムステップ進める。
        """
        # d²y/dt² = wn²(gain*u - y) - 2*zeta*wn*dy/dt
        ddy = self.wn**2 * (self.gain * u - self.y) - 2 * self.zeta * self.wn * self.dy
        self.dy += ddy * dt
        self.y += self.dy * dt
        return self.y

    def reset(self):
        self.y = 0.0
        self.dy = 0.0

    def __repr__(self):
        return f"SecondOrderPlant(wn={self.wn}, zeta={self.zeta})"


class DronePlant:
    """Simplified drone angular rate dynamics.

    簡略化したドローン角速度ダイナミクス。

    Models the roll/pitch axis as:
    ロール/ピッチ軸をモデル化：

        J * d(omega)/dt = tau_motor - D * omega
        tau_motor = Ct * L * (delta_rpm^2)  (simplified)

    With motor dynamics as first-order lag.
    モータダイナミクスを1次遅れでモデル化。

    Args:
        axis: "roll", "pitch", or "yaw" / 軸
        params: Override default parameters / パラメータの上書き

    Usage:
        >>> plant = DronePlant(axis="roll")
        >>> y = plant.step(0.5)  # 50% differential thrust
    """

    def __init__(self, axis: str = "roll", params: Optional[dict] = None):
        p = dict(_DEFAULTS)
        if params:
            p.update(params)

        self.axis = axis

        if axis == "roll":
            self.J = p["Ixx"]
        elif axis == "pitch":
            self.J = p["Iyy"]
        elif axis == "yaw":
            self.J = p["Izz"]
        else:
            raise ValueError(f"Unknown axis: {axis}. Use 'roll', 'pitch', or 'yaw'.")

        self.arm_length = p["arm_length"]
        self.max_thrust = p["max_thrust"]
        self.tau_m = p["tau_m"]
        self.damping = 1e-6  # Aerodynamic damping

        # State: angular rate, motor output
        # 状態: 角速度、モータ出力
        self.omega = 0.0
        self.motor_output = 0.0

    def step(self, u: float, dt: float = 0.0025) -> float:
        """Advance one time step.

        1タイムステップ進める。

        Args:
            u: Normalized differential thrust command (-1 to 1)
               正規化差動推力コマンド (-1 to 1)
            dt: Time step (s) / 時間ステップ（秒）

        Returns:
            Angular rate (rad/s) / 角速度
        """
        u = np.clip(u, -1.0, 1.0)

        # Motor dynamics (first-order lag)
        # モータダイナミクス（1次遅れ）
        self.motor_output += (u - self.motor_output) / self.tau_m * dt

        # Torque from differential thrust
        # 差動推力からのトルク
        torque = self.motor_output * self.max_thrust * self.arm_length * 2

        # Angular dynamics: J * d(omega)/dt = torque - D * omega
        # 角運動方程式
        domega = (torque - self.damping * self.omega) / self.J
        self.omega += domega * dt

        return self.omega

    def reset(self):
        self.omega = 0.0
        self.motor_output = 0.0

    def __repr__(self):
        return f"DronePlant(axis={self.axis!r}, J={self.J:.2e})"
