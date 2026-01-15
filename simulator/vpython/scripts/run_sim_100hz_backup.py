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
Physical Units Mode Flight Simulation (Joystick Control)
物理単位モードフライトシミュレーション（ジョイスティック操作）

This script implements firmware-compatible physical units based control:
ファームウェア互換の物理単位ベース制御を実装:
- PID output in torque [Nm]
- Control allocation via B⁻¹ matrix
- Thrust to voltage conversion via motor model

Based on run_sim.py with same joystick interface.
run_sim.py と同じジョイスティックインターフェースに基づく。

Controls (same as run_sim.py):
操作（run_sim.py と同じ）:
- Throttle (Axis 0): Hover ± delta thrust
- Roll (Axis 1): Roll rate/angle command
- Pitch (Axis 2): Pitch rate/angle command
- Yaw (Axis 3): Yaw rate command
- Mode button (Button 2 / bit 2): Toggle ACRO/STABILIZE
"""

import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt

# Add vpython package to path
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_VPYTHON_DIR = os.path.dirname(_SCRIPT_DIR)
if _VPYTHON_DIR not in sys.path:
    sys.path.insert(0, _VPYTHON_DIR)

from core import dynamics as mc
from visualization.vpython_backend import render
from vpython import *
from control.pid import PID
from interfaces.joystick import Joystick


# =============================================================================
# Motor Model Parameters (matching firmware motor_model.hpp)
# =============================================================================
class MotorParams:
    """Motor parameters for thrust-to-voltage conversion"""
    Ct = 1.0e-8       # Thrust coefficient [N/(rad/s)²]
    Cq = 9.71e-11     # Torque coefficient [Nm/(rad/s)²]
    Rm = 0.34         # Motor resistance [Ω]
    Km = 6.125e-4     # Motor constant [V·s/rad]
    Dm = 3.69e-8      # Viscous damping [Nm·s/rad]
    Qf = 2.76e-5      # Friction torque [Nm]
    Vbat = 3.7        # Battery voltage [V]


def thrust_to_omega(thrust: float) -> float:
    """Convert thrust to angular velocity (steady-state)"""
    if thrust <= 0:
        return 0.0
    return math.sqrt(thrust / MotorParams.Ct)


def omega_to_voltage(omega: float) -> float:
    """Convert angular velocity to required voltage (steady-state)"""
    if omega <= 0:
        return 0.0
    p = MotorParams
    viscous_term = (p.Dm + p.Km * p.Km / p.Rm) * omega
    aero_term = p.Cq * omega * omega
    friction_term = p.Qf
    voltage = p.Rm * (viscous_term + aero_term + friction_term) / p.Km
    return voltage


def thrust_to_voltage(thrust: float) -> float:
    """Convert thrust to voltage (steady-state approximation)"""
    if thrust <= 0:
        return 0.0
    omega = thrust_to_omega(thrust)
    voltage = omega_to_voltage(omega)
    return min(max(voltage, 0.0), MotorParams.Vbat)


# =============================================================================
# Control Allocation (matching firmware control_allocation.cpp)
# =============================================================================
class ControlAllocator:
    """
    Control allocation for X-Quad using B⁻¹ matrix
    X-Quad用のB⁻¹行列による制御配分
    """
    def __init__(self):
        # Quad parameters
        self.d = 0.023          # Moment arm [m]
        self.kappa = 9.71e-3    # Cq/Ct ratio [m]
        self.max_thrust = 0.15  # Max thrust per motor [N]

        # Build B⁻¹ matrix
        inv_d = 1.0 / self.d
        inv_kappa = 1.0 / self.kappa

        # B⁻¹ matrix: T = B⁻¹ × [total_thrust, roll_torque, pitch_torque, yaw_torque]
        # Motor order: M1(FR), M2(RR), M3(RL), M4(FL)
        self.B_inv = np.array([
            [0.25, -0.25 * inv_d,  0.25 * inv_d,  0.25 * inv_kappa],  # M1 (FR)
            [0.25, -0.25 * inv_d, -0.25 * inv_d, -0.25 * inv_kappa],  # M2 (RR)
            [0.25,  0.25 * inv_d, -0.25 * inv_d,  0.25 * inv_kappa],  # M3 (RL)
            [0.25,  0.25 * inv_d,  0.25 * inv_d, -0.25 * inv_kappa],  # M4 (FL)
        ])

    def mix(self, total_thrust: float, roll_torque: float,
            pitch_torque: float, yaw_torque: float) -> np.ndarray:
        """
        Convert control inputs to motor thrusts
        制御入力をモータ推力に変換

        Args:
            total_thrust: Total thrust [N]
            roll_torque: Roll torque [Nm]
            pitch_torque: Pitch torque [Nm]
            yaw_torque: Yaw torque [Nm]

        Returns:
            Motor thrusts [T1, T2, T3, T4] in [N]
        """
        control = np.array([total_thrust, roll_torque, pitch_torque, yaw_torque])
        thrusts = self.B_inv @ control
        # Clamp to valid range
        thrusts = np.clip(thrusts, 0.0, self.max_thrust)
        return thrusts

    def thrusts_to_voltages(self, thrusts: np.ndarray) -> np.ndarray:
        """
        Convert motor thrusts to voltages
        モータ推力を電圧に変換
        """
        return np.array([thrust_to_voltage(t) for t in thrusts])


# =============================================================================
# Physical Units PID Configuration (matching corrected firmware config.hpp)
# =============================================================================
class PhysicalUnitsPIDConfig:
    """
    PID configuration with corrected k_τ conversion
    修正されたk_τ変換係数を使用したPID設定

    k_τ_roll/pitch = 4 × (0.25/3.7) × 0.23 × 0.023 ≈ 1.4×10⁻³ Nm/V
    k_τ_yaw = 4 × (0.25/3.7) × 0.23 × 0.00971 ≈ 5.9×10⁻⁴ Nm/V
    """
    # Rate sensitivity (matching firmware)
    roll_rate_max = 1.0    # rad/s (~57 deg/s)
    pitch_rate_max = 1.0   # rad/s (~57 deg/s)
    yaw_rate_max = 5.0     # rad/s (~286 deg/s)

    # Roll Rate PID [Nm/(rad/s)]
    roll_kp = 9.1e-4
    roll_ti = 0.7
    roll_td = 0.01
    roll_limit = 5.2e-3  # Nm

    # Pitch Rate PID [Nm/(rad/s)]
    pitch_kp = 1.33e-3
    pitch_ti = 0.7
    pitch_td = 0.025
    pitch_limit = 5.2e-3  # Nm

    # Yaw Rate PID [Nm/(rad/s)]
    yaw_kp = 1.77e-3
    yaw_ti = 0.8
    yaw_td = 0.01
    yaw_limit = 2.2e-3  # Nm

    # Common
    eta = 0.125


# =============================================================================
# Main Simulation
# =============================================================================
def flight_sim_physical_units(world_type='voxel', seed=None, control_mode='rate'):
    """
    Flight simulation with physical units based control
    物理単位ベース制御によるフライトシミュレーション

    Parameters:
        world_type: 'ringworld' or 'voxel'
        seed: Terrain seed (None = random)
        control_mode: 'rate' (ACRO) or 'angle' (STABILIZE)
    """
    mass = 0.035
    weight = mass * 9.81
    hover_thrust = weight  # Total thrust for hover [N]

    stampfly = mc.multicopter(
        mass=mass,
        inersia=[[9.16e-6, 0.0, 0.0], [0.0, 13.3e-6, 0.0], [0.0, 0.0, 20.4e-6]]
    )

    # Initialize renderer
    Render = render(60, world_type=world_type, seed=seed)

    # Get safe spawn position
    spawn_x, spawn_y, spawn_z = Render.get_safe_spawn_position(x=0.0, y=0.0, clearance=1.0)
    stampfly.body.set_position([[spawn_x], [spawn_y], [spawn_z]])
    print(f"Spawn position: ({spawn_x:.2f}, {spawn_y:.2f}, {spawn_z:.2f})")

    # Initialize joystick
    joystick = Joystick()
    joystick.open()

    # Simulation parameters
    t = 0.0
    h = 0.001  # Physics timestep [s]
    control_interval = 1e-2  # 100Hz control (matching run_sim.py)
    control_time = 0.0

    # Initialize drone state
    stampfly.set_pqr([[0.0], [0.0], [0.0]])
    stampfly.set_uvw([[0.0], [0.0], [0.0]])
    stampfly.set_euler([[0], [0], [0]])

    # Initialize motors at hover
    nominal_omega = stampfly.motor_prop[0].equilibrium_anguler_velocity(weight / 4)
    for mp in stampfly.motor_prop:
        mp.omega = nominal_omega

    stampfly.set_disturbance(moment=[0, 0, 0], force=[0, 0, 0])

    # Initialize control allocator
    allocator = ControlAllocator()

    # Initialize PID controllers with physical units gains
    cfg = PhysicalUnitsPIDConfig

    # Rate PID (inner loop) - output is torque [Nm]
    roll_rate_pid = PID(cfg.roll_kp, cfg.roll_ti, cfg.roll_td,
                        output_min=-cfg.roll_limit, output_max=cfg.roll_limit)
    pitch_rate_pid = PID(cfg.pitch_kp, cfg.pitch_ti, cfg.pitch_td,
                         output_min=-cfg.pitch_limit, output_max=cfg.pitch_limit)
    yaw_rate_pid = PID(cfg.yaw_kp, cfg.yaw_ti, cfg.yaw_td,
                       output_min=-cfg.yaw_limit, output_max=cfg.yaw_limit)

    # Attitude PID (outer loop) - output is angular rate [rad/s]
    roll_pid = PID(5.0, 1.0, 0.0)
    pitch_pid = PID(5.0, 1.0, 0.0)

    # Control mode: 'rate' (ACRO) or 'angle' (STABILIZE)
    use_rate_mode = (control_mode == 'rate')
    prev_mode_button = False
    print(f"Control mode: {'ACRO (Rate)' if use_rate_mode else 'STABILIZE (Angle)'}")

    # Joystick calibration (same as run_sim.py)
    print("Calibrating joystick...")
    thrust_offset = 0.0
    roll_offset = 0.0
    pitch_offset = 0.0
    yaw_offset = 0.0

    i = 0
    num = 100
    while i < num:
        joydata = joystick.read()
        if joydata is not None:
            thrust_offset += (joydata[0] - 127) / 127.0
            roll_offset += (joydata[1] - 127) / 127.0
            pitch_offset += (joydata[2] - 127) / 127.0
            yaw_offset += (joydata[3] - 127) / 127.0
            i += 1
            print(i)

    thrust_offset /= num
    roll_offset /= num
    pitch_offset /= num
    yaw_offset /= num
    print(f"Calibration done: thrust={thrust_offset:.4f}, roll={roll_offset:.4f}, "
          f"pitch={pitch_offset:.4f}, yaw={yaw_offset:.4f}")

    # Data logging
    T = [t]
    PQR = [stampfly.body.pqr.copy()]
    PQR_REF = [np.array([[0.0], [0.0], [0.0]])]
    EULER = [stampfly.body.euler.copy()]
    POS = [stampfly.body.position.copy()]

    # Control state variables
    roll_ref = 0.0
    pitch_ref = 0.0
    yaw_ref = 0.0
    delta_thrust = 0.0  # Thrust delta from hover

    # Control outputs (torque)
    roll_torque = 0.0
    pitch_torque = 0.0
    yaw_torque = 0.0

    print("=" * 60)
    print("Physical Units Mode Simulation")
    print("物理単位モードシミュレーション")
    print("=" * 60)
    print(f"Roll Rate PID: Kp={cfg.roll_kp:.2e} Nm/(rad/s)")
    print(f"Pitch Rate PID: Kp={cfg.pitch_kp:.2e} Nm/(rad/s)")
    print(f"Yaw Rate PID: Kp={cfg.yaw_kp:.2e} Nm/(rad/s)")
    print(f"Hover thrust: {hover_thrust*1000:.1f} mN")
    print("=" * 60)

    # Main simulation loop
    while t < 6000.0:
        # Get current state
        rate_p = stampfly.body.pqr[0][0]
        rate_q = stampfly.body.pqr[1][0]
        rate_r = stampfly.body.pqr[2][0]
        phi = stampfly.body.euler[0][0]
        theta = stampfly.body.euler[1][0]
        psi = stampfly.body.euler[2][0]

        # Read joystick (same as run_sim.py)
        joydata = joystick.read()
        if joydata is not None:
            # Raw values with offset correction
            thrust_raw = (joydata[0] - 127) / 127.0 - thrust_offset
            roll_raw = (joydata[1] - 127) / 127.0 - roll_offset
            pitch_raw = (joydata[2] - 127) / 127.0 - pitch_offset
            yaw_raw = (joydata[3] - 127) / 127.0 - yaw_offset

            # Mode button (bit 2 of buttons byte)
            buttons = joydata[4] if len(joydata) > 4 else 0
            mode_button = bool(buttons & 0x04)

            # Toggle mode on rising edge
            if mode_button and not prev_mode_button:
                use_rate_mode = not use_rate_mode
                mode_name = 'ACRO (Rate)' if use_rate_mode else 'STABILIZE (Angle)'
                print(f"Mode changed: {mode_name}")
            prev_mode_button = mode_button

            # Thrust: hover + delta (same scale as run_sim.py)
            delta_thrust = 0.5 * thrust_raw * hover_thrust  # ±50% of hover

            if use_rate_mode:
                # ACRO mode: stick -> target angular rate
                roll_ref = cfg.roll_rate_max * roll_raw
                pitch_ref = cfg.pitch_rate_max * pitch_raw
                yaw_ref = cfg.yaw_rate_max * yaw_raw
            else:
                # STABILIZE mode: stick -> target angle
                roll_ref = 0.25 * roll_raw * np.pi    # max ±45°
                pitch_ref = 0.25 * pitch_raw * np.pi  # max ±45°
                yaw_ref = 0.3 * yaw_raw * np.pi       # yaw rate

        # Control loop
        control_on = True
        if t >= control_time and control_on:
            control_time += control_interval

            if use_rate_mode:
                # ACRO mode: stick input is directly target angular rate
                roll_rate_ref = roll_ref
                pitch_rate_ref = pitch_ref
                yaw_rate_ref = yaw_ref
            else:
                # STABILIZE mode: attitude error -> target angular rate
                roll_rate_ref = roll_pid.update(roll_ref, phi, control_interval)
                pitch_rate_ref = pitch_pid.update(pitch_ref, theta, control_interval)
                yaw_rate_ref = yaw_ref

            # Rate control (inner loop) - output is torque [Nm]
            roll_torque = roll_rate_pid.update(roll_rate_ref, rate_p, control_interval)
            pitch_torque = pitch_rate_pid.update(pitch_rate_ref, rate_q, control_interval)
            yaw_torque = yaw_rate_pid.update(yaw_rate_ref, rate_r, control_interval)

        # Total thrust = hover + delta
        total_thrust = hover_thrust + delta_thrust
        total_thrust = max(0.0, min(total_thrust, 4 * 0.15))  # Clamp to valid range

        # Control allocation: [thrust, τ_roll, τ_pitch, τ_yaw] -> motor thrusts
        motor_thrusts = allocator.mix(total_thrust, roll_torque, pitch_torque, yaw_torque)

        # Convert thrusts to voltages
        motor_voltages = allocator.thrusts_to_voltages(motor_thrusts)

        # Apply to simulation (motor order: FR, RR, RL, FL)
        voltage = [motor_voltages[0], motor_voltages[1], motor_voltages[2], motor_voltages[3]]
        stampfly.step(voltage, h)

        # Collision detection
        pos = stampfly.body.position
        if Render.check_collision(pos[0][0], pos[1][0], pos[2][0]):
            print(f"COLLISION at t={t:.2f}s, pos=({pos[0][0]:.2f}, {pos[1][0]:.2f}, {pos[2][0]:.2f})")
            Render.show_collision(pos[0][0], pos[1][0], pos[2][0])
            break

        # Render
        key = Render.rendering(t, stampfly)

        t += h
        T.append(t)
        PQR.append(stampfly.body.pqr.copy())
        PQR_REF.append(np.array([[roll_ref], [pitch_ref], [yaw_ref]]))
        EULER.append(stampfly.body.euler.copy())
        POS.append(stampfly.body.position.copy())

    # Convert to numpy arrays
    T = np.array(T)
    PQR = np.array(PQR)
    PQR_REF = np.array(PQR_REF)
    EULER = np.array(EULER)
    POS = np.array(POS)

    # Plot results
    if True:
        plt.subplot(4, 1, 1)
        plt.plot(T, PQR[:, 0, 0], label='P')
        plt.plot(T, PQR[:, 1, 0], label='Q')
        plt.plot(T, PQR[:, 2, 0], label='R')
        plt.plot(T, PQR_REF[:, 0, 0], label='P_ref')
        plt.plot(T, PQR_REF[:, 1, 0], label='Q_ref')
        plt.plot(T, PQR_REF[:, 2, 0], label='R_ref')
        plt.legend()
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('PQR (rad/s)')

        plt.subplot(4, 1, 2)
        plt.plot(T, np.rad2deg(EULER[:, 0, 0]), label='φ (roll)')
        plt.plot(T, np.rad2deg(EULER[:, 1, 0]), label='θ (pitch)')
        plt.plot(T, np.rad2deg(EULER[:, 2, 0]), label='ψ (yaw)')
        plt.legend()
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('Euler Angle (deg)')

        plt.subplot(4, 1, 3)
        plt.plot(T, POS[:, 0, 0], label='X')
        plt.plot(T, POS[:, 1, 0], label='Y')
        plt.plot(T, POS[:, 2, 0], label='Z')
        plt.legend()
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')

        plt.subplot(4, 1, 4)
        # UVW not logged in this version, show position derivative approximation
        if len(T) > 1:
            dT = np.diff(T)
            dX = np.diff(POS[:, 0, 0]) / dT
            dY = np.diff(POS[:, 1, 0]) / dT
            dZ = np.diff(POS[:, 2, 0]) / dT
            plt.plot(T[:-1], dX, label='dX/dt')
            plt.plot(T[:-1], dY, label='dY/dt')
            plt.plot(T[:-1], dZ, label='dZ/dt')
        plt.legend()
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Physical Units Mode Flight Simulator')
    parser.add_argument('--world', '-w', type=str, default='voxel',
                        choices=['ringworld', 'voxel'],
                        help='World type (default: voxel)')
    parser.add_argument('--seed', '-s', type=int, default=None,
                        help='Terrain seed (random if not specified)')
    parser.add_argument('--mode', '-m', type=str, default='rate',
                        choices=['rate', 'angle'],
                        help='Control mode: rate=ACRO, angle=STABILIZE (default: rate)')
    args = parser.parse_args()

    print(f"Starting physical units mode simulation: world={args.world}, mode={args.mode}")
    flight_sim_physical_units(world_type=args.world, seed=args.seed, control_mode=args.mode)
