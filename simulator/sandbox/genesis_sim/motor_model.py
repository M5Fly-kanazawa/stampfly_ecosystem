#!/usr/bin/env python3
# MIT License
# Copyright (c) 2025 Kouhei Ito
"""
motor_model.py - StampFly Motor-Propeller Model for Genesis
StampFly用モータ・プロペラモデル（Genesis版）

Simulates the electrical-mechanical dynamics of brushed DC motors
with propellers using RK4 integration.
ブラシDCモータとプロペラの電気機械動特性をRK4積分でシミュレート。

Motor dynamics equation:
モータダイナミクス方程式:

  dω/dt = [-(Dm + Km²/Rm)·ω - Cq·ω² - Qf + Km·V/Rm] / Jmp

Where:
  ω: Angular velocity (rad/s) / 角速度
  V: Applied voltage (V) / 印加電圧
  Dm: Damping coefficient / 減衰係数
  Km: Motor constant / モータ定数
  Rm: Motor resistance / 巻線抵抗
  Cq: Torque coefficient / トルク係数
  Qf: Friction torque / 摩擦トルク
  Jmp: Rotor inertia / ローター慣性

Thrust and torque:
推力とトルク:
  T = Ct · ω²
  τ = Cq · ω²
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class MotorParams:
    """
    Motor-propeller parameters measured from StampFly hardware.
    StampFly実機から測定したモータ・プロペラパラメータ
    """
    # Electrical parameters (LCR meter measurement)
    # 電気パラメータ（LCRメータ測定）
    Rm: float = 0.34      # Resistance (Ohm) / 巻線抵抗
    Lm: float = 1.0e-6    # Inductance (H) / インダクタンス

    # RPM-voltage relationship coefficients
    # 回転数-電圧関係係数
    Am: float = 5.39e-8
    Bm: float = 6.33e-4
    Cm: float = 1.53e-2

    # Aerodynamic coefficients (thrust/torque vs RPM measurement)
    # 空力係数（推力・トルク vs 回転数測定）
    Ct: float = 1.00e-8   # Thrust coefficient (N·s²/rad²) / 推力係数
    Cq: float = 9.71e-11  # Torque coefficient (N·m·s²/rad²) / トルク係数

    # Rotor inertia (estimated from shape and weight)
    # ローター慣性（形状と重量から推定）
    Jmp: float = 2.01e-8  # kg·m²

    # Battery voltage / バッテリー電圧
    Vbat: float = 3.7     # V

    @property
    def Km(self) -> float:
        """Motor constant / モータ定数"""
        return self.Cq * self.Rm / self.Am

    @property
    def Dm(self) -> float:
        """Damping coefficient / 減衰係数"""
        return (self.Bm - self.Cq * self.Rm / self.Am) * (self.Cq / self.Am)

    @property
    def Qf(self) -> float:
        """Friction torque / 摩擦トルク"""
        return self.Cm * self.Cq / self.Am

    @property
    def kappa(self) -> float:
        """Torque to thrust ratio / トルク推力比"""
        return self.Cq / self.Ct


# Default parameters / デフォルトパラメータ
DEFAULT_MOTOR_PARAMS = MotorParams()


class Motor:
    """
    Single motor-propeller unit model.
    単一モータ・プロペラユニットモデル
    """

    def __init__(
        self,
        position: np.ndarray,
        rotation_dir: int,
        params: MotorParams = DEFAULT_MOTOR_PARAMS,
    ):
        """
        Initialize motor model.
        モータモデルを初期化

        Args:
            position: Motor position in body frame [x, y, z] (m)
                      機体座標系でのモータ位置
            rotation_dir: Rotation direction (1=CW, -1=CCW)
                          回転方向
            params: Motor parameters / モータパラメータ
        """
        self.position = np.array(position)
        self.rotation_dir = rotation_dir
        self.params = params

        # State variables / 状態変数
        self.omega = 0.0    # Angular velocity (rad/s)
        self.thrust = 0.0   # Current thrust (N)
        self.torque = 0.0   # Current torque (Nm)

    def omega_dot(self, omega: float, voltage: float) -> float:
        """
        Compute angular acceleration.
        角加速度を計算

        Args:
            omega: Current angular velocity (rad/s)
            voltage: Applied voltage (V)

        Returns:
            Angular acceleration (rad/s²)
        """
        p = self.params
        return (
            -(p.Dm + p.Km**2 / p.Rm) * omega
            - p.Cq * omega**2
            - p.Qf
            + p.Km * voltage / p.Rm
        ) / p.Jmp

    def step(self, voltage: float, dt: float) -> Tuple[float, float]:
        """
        Update motor state using RK4 integration.
        RK4積分でモータ状態を更新

        Args:
            voltage: Applied voltage (V)
            dt: Time step (s)

        Returns:
            (thrust, torque) tuple
        """
        # Clamp voltage to valid range
        voltage = max(0.0, min(voltage, self.params.Vbat))

        # Runge-Kutta 4th order integration
        k1 = self.omega_dot(self.omega, voltage)
        k2 = self.omega_dot(self.omega + k1 * dt / 2.0, voltage)
        k3 = self.omega_dot(self.omega + k2 * dt / 2.0, voltage)
        k4 = self.omega_dot(self.omega + k3 * dt, voltage)
        self.omega += (k1 + 2*k2 + 2*k3 + k4) * dt / 6.0

        # Ensure non-negative angular velocity
        self.omega = max(0.0, self.omega)

        # Compute thrust and torque
        self.thrust = self.params.Ct * self.omega**2
        self.torque = self.params.Cq * self.omega**2

        return self.thrust, self.torque

    def reset(self):
        """Reset motor state / モータ状態をリセット"""
        self.omega = 0.0
        self.thrust = 0.0
        self.torque = 0.0


class QuadMotorSystem:
    """
    Four motor system for X-quad configuration.
    X-quad配置用4モータシステム

    Motor layout (NED body frame):
    モータ配置（NED機体座標系）:

        Front (NED +X)
        FL(M4)     FR(M1)
          CW  ▲    CCW
            ╲ │ ╱
             ╲│╱
             ╱│╲
            ╱ │ ╲
          CCW │  CW
        RL(M3)     RR(M2)
        Rear

    NED coordinates:
      X: Forward (前方)
      Y: Right (右方)
      Z: Down (下方)

    Thrust direction: -Z (up in NED) / 推力方向: -Z（NEDで上向き）
    """

    # Motor configuration (NED body frame)
    # モータ設定（NED機体座標系）
    ARM_LENGTH = 0.023   # m (23mm from center to motor)
    MOTOR_HEIGHT = -0.005  # m (-5mm, above CG in NED)

    MOTOR_POSITIONS = [
        # [x, y, z] in NED body frame
        [ARM_LENGTH, ARM_LENGTH, MOTOR_HEIGHT],    # M1: FR
        [-ARM_LENGTH, ARM_LENGTH, MOTOR_HEIGHT],   # M2: RR
        [-ARM_LENGTH, -ARM_LENGTH, MOTOR_HEIGHT],  # M3: RL
        [ARM_LENGTH, -ARM_LENGTH, MOTOR_HEIGHT],   # M4: FL
    ]

    MOTOR_DIRECTIONS = [-1, 1, -1, 1]  # M1:CCW, M2:CW, M3:CCW, M4:CW

    def __init__(self, params: MotorParams = DEFAULT_MOTOR_PARAMS):
        """
        Initialize quad motor system.
        クアッドモータシステムを初期化

        Args:
            params: Motor parameters / モータパラメータ
        """
        self.params = params
        self.motors = [
            Motor(
                position=self.MOTOR_POSITIONS[i],
                rotation_dir=self.MOTOR_DIRECTIONS[i],
                params=params,
            )
            for i in range(4)
        ]

    def step(self, voltages: List[float], dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update all motors and compute total force/moment in NED body frame.
        全モータを更新し、NED機体座標系での合計力/モーメントを計算

        Args:
            voltages: List of voltages [V1, V2, V3, V4] for motors M1-M4
            dt: Time step (s)

        Returns:
            (force, moment) tuple in NED body frame
            force: [Fx, Fy, Fz] (N)
            moment: [Mx, My, Mz] (Nm)
        """
        total_force = np.zeros(3)
        total_moment = np.zeros(3)

        for i, motor in enumerate(self.motors):
            motor.step(voltages[i], dt)

            # Thrust force in NED: [0, 0, -T] (upward)
            # NED推力: [0, 0, -T]（上向き）
            force = np.array([0.0, 0.0, -motor.thrust])
            total_force += force

            # Reaction torque (yaw) / 反トルク（ヨー）
            # CCW motor (-1) produces +Z torque (nose right)
            # CW motor (+1) produces -Z torque (nose left)
            moment = np.array([0.0, 0.0, -motor.rotation_dir * motor.torque])

            # Moment from thrust at motor position
            # モータ位置での推力によるモーメント
            moment += np.cross(motor.position, force)

            total_moment += moment

        return total_force, total_moment

    def step_with_duty(self, duties: List[float], dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update motors with duty cycle inputs.
        デューティサイクル入力でモータを更新

        Args:
            duties: List of duty cycles [d1, d2, d3, d4] (0.0 to 1.0)
            dt: Time step (s)

        Returns:
            (force, moment) tuple in NED body frame
        """
        voltages = [d * self.params.Vbat for d in duties]
        return self.step(voltages, dt)

    def reset(self):
        """Reset all motors / 全モータをリセット"""
        for motor in self.motors:
            motor.reset()

    @property
    def thrusts(self) -> List[float]:
        """Get all motor thrusts / 全モータ推力を取得"""
        return [m.thrust for m in self.motors]

    @property
    def omegas(self) -> List[float]:
        """Get all motor angular velocities / 全モータ角速度を取得"""
        return [m.omega for m in self.motors]

    @property
    def total_thrust(self) -> float:
        """Get total thrust / 合計推力を取得"""
        return sum(self.thrusts)


def compute_hover_conditions(params: MotorParams = DEFAULT_MOTOR_PARAMS, mass: float = 0.035):
    """
    Compute hover conditions for reference.
    参考用にホバリング条件を計算

    Args:
        params: Motor parameters
        mass: Vehicle mass (kg)

    Returns:
        dict with hover conditions
    """
    weight = mass * 9.81
    thrust_per_motor = weight / 4.0
    omega_hover = np.sqrt(thrust_per_motor / params.Ct)
    rpm_hover = omega_hover * 60 / (2 * np.pi)

    # Equilibrium voltage
    voltage_hover = params.Rm * (
        (params.Dm + params.Km**2 / params.Rm) * omega_hover +
        params.Cq * omega_hover**2 +
        params.Qf
    ) / params.Km

    duty_hover = voltage_hover / params.Vbat

    return {
        "thrust_per_motor": thrust_per_motor,
        "omega_hover": omega_hover,
        "rpm_hover": rpm_hover,
        "voltage_hover": voltage_hover,
        "duty_hover": duty_hover,
    }


if __name__ == "__main__":
    # Test and display parameters
    print("=" * 60)
    print("StampFly Motor Model Parameters")
    print("=" * 60)

    p = DEFAULT_MOTOR_PARAMS
    print(f"\nMeasured parameters:")
    print(f"  Rm = {p.Rm} Ω")
    print(f"  Lm = {p.Lm} H")
    print(f"  Ct = {p.Ct} N·s²/rad²")
    print(f"  Cq = {p.Cq} N·m·s²/rad²")
    print(f"  Jmp = {p.Jmp} kg·m²")
    print(f"  Vbat = {p.Vbat} V")

    print(f"\nDerived parameters:")
    print(f"  Km = {p.Km:.4e} V·s/rad")
    print(f"  Dm = {p.Dm:.4e} N·m·s/rad")
    print(f"  Qf = {p.Qf:.4e} N·m")
    print(f"  κ  = {p.kappa:.4e} m")

    hover = compute_hover_conditions()
    print(f"\nHover conditions (mass=35g):")
    print(f"  Thrust/motor = {hover['thrust_per_motor']*1000:.2f} mN")
    print(f"  ω_hover = {hover['omega_hover']:.0f} rad/s")
    print(f"  RPM_hover = {hover['rpm_hover']:.0f}")
    print(f"  V_hover = {hover['voltage_hover']:.2f} V")
    print(f"  Duty_hover = {hover['duty_hover']*100:.1f}%")

    # Quick simulation test
    print(f"\nStep response test (0→100% duty):")
    motor_sys = QuadMotorSystem()
    dt = 0.001  # 1ms
    duties = [1.0, 1.0, 1.0, 1.0]  # 100% duty

    for step in range(100):
        force, moment = motor_sys.step_with_duty(duties, dt)
        if step % 20 == 0:
            t = step * dt * 1000
            print(f"  t={t:3.0f}ms: thrust={motor_sys.total_thrust*1000:.1f}mN, "
                  f"ω={motor_sys.omegas[0]:.0f} rad/s")
