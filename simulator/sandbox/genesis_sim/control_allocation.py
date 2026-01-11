#!/usr/bin/env python3
# MIT License
# Copyright (c) 2025 Kouhei Ito
"""
control_allocation.py - Control Allocation for X-Quad
X-Quadのコントロールアロケーション

Allocation Matrix B (forward):
アロケーション行列 B（順方向）:
  [uₜ, u_φ, u_θ, u_ψ]ᵀ = B × [T₁, T₂, T₃, T₄]ᵀ

Mixing Matrix B⁻¹ (inverse):
ミキサー行列 B⁻¹（逆方向）:
  [T₁, T₂, T₃, T₄]ᵀ = B⁻¹ × [uₜ, u_φ, u_θ, u_ψ]ᵀ

Physical units:
物理単位:
  uₜ: Total thrust (N) / 総推力
  u_φ: Roll torque (Nm) / ロールトルク
  u_θ: Pitch torque (Nm) / ピッチトルク
  u_ψ: Yaw torque (Nm) / ヨートルク
  T₁~T₄: Motor thrusts (N) / モータ推力

Motor layout (NED body frame):
モータ配置（NED機体座標系）:

        Front (NED +X)
    FL(M4)     FR(M1)
      CW   ▲    CCW
        ╲  │  ╱
         ╲ │ ╱
          ╲│╱
          ╱│╲
         ╱ │ ╲
        ╱  │  ╲
      CCW  │   CW
    RL(M3)     RR(M2)
        Rear
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple


@dataclass
class QuadConfig:
    """
    X-Quad configuration parameters.
    X-Quad設定パラメータ
    """
    # Moment arm (m) / モーメントアーム
    d: float = 0.023

    # Torque/Thrust ratio κ = Cq/Ct (m)
    # トルク推力比
    kappa: float = 9.71e-3

    # Motor positions [x, y] in NED body frame (m)
    # モータ位置（NED機体座標系）
    # M1:FR, M2:RR, M3:RL, M4:FL
    motor_x: Tuple[float, float, float, float] = (0.023, -0.023, -0.023, 0.023)
    motor_y: Tuple[float, float, float, float] = (0.023, 0.023, -0.023, -0.023)

    # Motor rotation directions (1=CW, -1=CCW)
    # モータ回転方向
    motor_dir: Tuple[int, int, int, int] = (-1, 1, -1, 1)  # CCW, CW, CCW, CW


DEFAULT_CONFIG = QuadConfig()


class ControlAllocator:
    """
    Control allocation for X-Quad.
    X-Quadのコントロールアロケーション

    Converts between:
    変換:
      - Control inputs [uₜ, u_φ, u_θ, u_ψ] (N, Nm, Nm, Nm)
      - Motor thrusts [T₁, T₂, T₃, T₄] (N)
    """

    def __init__(self, config: QuadConfig = DEFAULT_CONFIG):
        """
        Initialize control allocator.
        コントロールアロケータを初期化

        Args:
            config: Quad configuration / クワッド設定
        """
        self.config = config
        self._build_matrices()

    def _build_matrices(self):
        """
        Build allocation and mixing matrices.
        アロケーション行列とミキサー行列を構築
        """
        c = self.config

        # Allocation matrix B: u = B @ T
        # Row 0: Total thrust (sum of all thrusts)
        # Row 1: Roll torque = -y × T (moment around X-axis from thrust at y position)
        # Row 2: Pitch torque = +x × T (moment around Y-axis from thrust at x position)
        # Row 3: Yaw torque (reaction torques)
        self.B = np.array([
            [1.0, 1.0, 1.0, 1.0],
            [-c.motor_y[0], -c.motor_y[1], -c.motor_y[2], -c.motor_y[3]],
            [c.motor_x[0], c.motor_x[1], c.motor_x[2], c.motor_x[3]],
            [-c.motor_dir[0] * c.kappa,
             -c.motor_dir[1] * c.kappa,
             -c.motor_dir[2] * c.kappa,
             -c.motor_dir[3] * c.kappa],
        ])

        # Mixing matrix B_inv: T = B_inv @ u
        self.B_inv = np.linalg.inv(self.B)

    def allocate(self, thrusts: np.ndarray) -> np.ndarray:
        """
        Forward allocation: motor thrusts → control inputs.
        順方向アロケーション: モータ推力 → 制御入力

        Args:
            thrusts: [T₁, T₂, T₃, T₄] motor thrusts (N)

        Returns:
            [uₜ, u_φ, u_θ, u_ψ] control inputs (N, Nm, Nm, Nm)
        """
        return self.B @ thrusts

    def mix(self, control: np.ndarray) -> np.ndarray:
        """
        Inverse allocation (mixing): control inputs → motor thrusts.
        逆アロケーション（ミキシング）: 制御入力 → モータ推力

        Args:
            control: [uₜ, u_φ, u_θ, u_ψ] control inputs (N, Nm, Nm, Nm)

        Returns:
            [T₁, T₂, T₃, T₄] motor thrusts (N)
        """
        thrusts = self.B_inv @ control
        # Clamp to non-negative (motors can only push)
        return np.maximum(thrusts, 0.0)

    def mix_with_saturation(
        self,
        control: np.ndarray,
        max_thrust_per_motor: float = 0.15
    ) -> Tuple[np.ndarray, bool]:
        """
        Mix with saturation handling.
        飽和処理付きミキシング

        Args:
            control: [uₜ, u_φ, u_θ, u_ψ] control inputs
            max_thrust_per_motor: Maximum thrust per motor (N)

        Returns:
            (thrusts, saturated) tuple
        """
        thrusts = self.mix(control)
        saturated = np.any(thrusts > max_thrust_per_motor) or np.any(thrusts < 0)
        thrusts = np.clip(thrusts, 0.0, max_thrust_per_motor)
        return thrusts, saturated


def thrust_to_duty(thrust: float, Ct: float = 1.0e-8, Vbat: float = 3.7,
                   Km: float = 6.125e-4, Rm: float = 0.34,
                   Dm: float = 3.69e-8, Cq: float = 9.71e-11,
                   Qf: float = 2.76e-5) -> float:
    """
    Convert thrust to motor duty cycle (steady-state approximation).
    推力からモータDutyサイクルへの変換（定常状態近似）

    Args:
        thrust: Desired thrust (N)
        Other args: Motor parameters

    Returns:
        Duty cycle (0.0 to 1.0)
    """
    if thrust <= 0:
        return 0.0

    # Equilibrium angular velocity for desired thrust
    omega = np.sqrt(thrust / Ct)

    # Equilibrium voltage
    voltage = Rm * ((Dm + Km**2 / Rm) * omega + Cq * omega**2 + Qf) / Km

    # Duty cycle
    duty = voltage / Vbat
    return np.clip(duty, 0.0, 1.0)


def thrusts_to_duties(thrusts: np.ndarray) -> np.ndarray:
    """
    Convert motor thrusts to duty cycles.
    モータ推力をDutyサイクルに変換

    Args:
        thrusts: [T₁, T₂, T₃, T₄] motor thrusts (N)

    Returns:
        [d₁, d₂, d₃, d₄] duty cycles (0.0 to 1.0)
    """
    return np.array([thrust_to_duty(t) for t in thrusts])


if __name__ == "__main__":
    # Test and display matrices
    print("=" * 60)
    print("Control Allocation Matrices")
    print("=" * 60)

    alloc = ControlAllocator()
    c = alloc.config

    print(f"\nConfiguration:")
    print(f"  Moment arm d = {c.d * 1000:.1f} mm")
    print(f"  Kappa κ = {c.kappa:.4e} m")

    print(f"\nAllocation Matrix B:")
    print(f"  [uₜ ]   [ {alloc.B[0,0]:+.0f}  {alloc.B[0,1]:+.0f}  {alloc.B[0,2]:+.0f}  {alloc.B[0,3]:+.0f} ]   [T₁]")
    print(f"  [u_φ] = [{alloc.B[1,0]:+.3f} {alloc.B[1,1]:+.3f} {alloc.B[1,2]:+.3f} {alloc.B[1,3]:+.3f}] × [T₂]")
    print(f"  [u_θ]   [{alloc.B[2,0]:+.3f} {alloc.B[2,1]:+.3f} {alloc.B[2,2]:+.3f} {alloc.B[2,3]:+.3f}]   [T₃]")
    print(f"  [u_ψ]   [{alloc.B[3,0]:+.4f} {alloc.B[3,1]:+.4f} {alloc.B[3,2]:+.4f} {alloc.B[3,3]:+.4f}]   [T₄]")

    print(f"\nMixing Matrix B⁻¹:")
    print(f"  [T₁]   [{alloc.B_inv[0,0]:+.2f} {alloc.B_inv[0,1]:+.1f} {alloc.B_inv[0,2]:+.1f} {alloc.B_inv[0,3]:+.1f}]   [uₜ ]")
    print(f"  [T₂] = [{alloc.B_inv[1,0]:+.2f} {alloc.B_inv[1,1]:+.1f} {alloc.B_inv[1,2]:+.1f} {alloc.B_inv[1,3]:+.1f}] × [u_φ]")
    print(f"  [T₃]   [{alloc.B_inv[2,0]:+.2f} {alloc.B_inv[2,1]:+.1f} {alloc.B_inv[2,2]:+.1f} {alloc.B_inv[2,3]:+.1f}]   [u_θ]")
    print(f"  [T₄]   [{alloc.B_inv[3,0]:+.2f} {alloc.B_inv[3,1]:+.1f} {alloc.B_inv[3,2]:+.1f} {alloc.B_inv[3,3]:+.1f}]   [u_ψ]")

    # Test: hover condition
    print(f"\n=== Test: Hover (0.035kg) ===")
    weight = 0.035 * 9.81  # N
    control_hover = np.array([weight, 0.0, 0.0, 0.0])
    thrusts_hover = alloc.mix(control_hover)
    duties_hover = thrusts_to_duties(thrusts_hover)
    print(f"  Control: uₜ={weight:.3f}N, u_φ=0, u_θ=0, u_ψ=0")
    print(f"  Thrusts: {[f'{t*1000:.1f}mN' for t in thrusts_hover]}")
    print(f"  Duties:  {[f'{d*100:.1f}%' for d in duties_hover]}")

    # Test: roll torque
    print(f"\n=== Test: Roll torque (1 mNm) ===")
    control_roll = np.array([weight, 1e-3, 0.0, 0.0])
    thrusts_roll = alloc.mix(control_roll)
    duties_roll = thrusts_to_duties(thrusts_roll)
    print(f"  Control: uₜ={weight:.3f}N, u_φ=1mNm, u_θ=0, u_ψ=0")
    print(f"  Thrusts: {[f'{t*1000:.1f}mN' for t in thrusts_roll]}")
    print(f"  Duties:  {[f'{d*100:.1f}%' for d in duties_roll]}")

    # Verify: forward allocation recovers control
    print(f"\n=== Verify: B @ B⁻¹ = I ===")
    recovered = alloc.allocate(thrusts_roll)
    print(f"  Recovered control: uₜ={recovered[0]:.3f}N, "
          f"u_φ={recovered[1]*1000:.3f}mNm, "
          f"u_θ={recovered[2]*1000:.3f}mNm, "
          f"u_ψ={recovered[3]*1000:.3f}mNm")
