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
High-Frequency Physical Units Mode Flight Simulation (Experimental)
高周波物理単位モードフライトシミュレーション（実験版）

Timing configuration:
タイミング設定:
- Physics: 2000Hz (h = 0.5ms)
- Control: 400Hz (5 physics steps per control)
- Visualization: 60Hz

Real-time synchronization using wall-clock time.
実時間同期のためウォールクロック時間を使用。
"""

import sys
import os
import math
import time
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
        """
        control = np.array([total_thrust, roll_torque, pitch_torque, yaw_torque])
        thrusts = self.B_inv @ control
        thrusts = np.clip(thrusts, 0.0, self.max_thrust)
        return thrusts

    def thrusts_to_voltages(self, thrusts: np.ndarray) -> np.ndarray:
        """Convert motor thrusts to voltages"""
        return np.array([thrust_to_voltage(t) for t in thrusts])


# =============================================================================
# Physical Units PID Configuration (matching corrected firmware config.hpp)
# =============================================================================
class PhysicalUnitsPIDConfig:
    """
    PID configuration with corrected k_τ conversion
    修正されたk_τ変換係数を使用したPID設定
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
def flight_sim_2000hz(world_type='voxel', seed=None, control_mode='rate'):
    """
    Flight simulation with high-frequency physics and control
    高周波物理・制御によるフライトシミュレーション

    Timing:
    - Physics: 2000Hz
    - Control: 400Hz
    - Visualization: 60Hz
    """
    # ===========================================
    # Timing Configuration
    # ===========================================
    PHYSICS_HZ = 2000
    PHYSICS_DT = 1.0 / PHYSICS_HZ  # 0.0005s = 0.5ms

    CONTROL_HZ = 400
    CONTROL_DT = 1.0 / CONTROL_HZ  # 0.0025s = 2.5ms
    PHYSICS_PER_CONTROL = PHYSICS_HZ // CONTROL_HZ  # 5 physics steps per control

    RENDER_FPS = 60
    RENDER_DT = 1.0 / RENDER_FPS  # ~16.67ms

    print("=" * 60)
    print("High-Frequency Simulation (Experimental)")
    print("高周波シミュレーション（実験版）")
    print("=" * 60)
    print(f"Physics:       {PHYSICS_HZ} Hz (dt = {PHYSICS_DT*1000:.2f} ms)")
    print(f"Control:       {CONTROL_HZ} Hz (dt = {CONTROL_DT*1000:.2f} ms)")
    print(f"Visualization: {RENDER_FPS} FPS")
    print(f"Physics steps per control: {PHYSICS_PER_CONTROL}")
    print("=" * 60)

    # ===========================================
    # Drone Setup
    # ===========================================
    mass = 0.035
    weight = mass * 9.81
    hover_thrust = weight

    stampfly = mc.multicopter(
        mass=mass,
        inersia=[[9.16e-6, 0.0, 0.0], [0.0, 13.3e-6, 0.0], [0.0, 0.0, 20.4e-6]]
    )

    # Initialize renderer (60 FPS target)
    Render = render(RENDER_FPS, world_type=world_type, seed=seed)

    # Get safe spawn position
    spawn_x, spawn_y, spawn_z = Render.get_safe_spawn_position(x=0.0, y=0.0, clearance=1.0)
    stampfly.body.set_position([[spawn_x], [spawn_y], [spawn_z]])
    print(f"Spawn position: ({spawn_x:.2f}, {spawn_y:.2f}, {spawn_z:.2f})")

    # Initialize joystick
    joystick = Joystick()
    joystick.open()

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

    # ===========================================
    # PID Controllers
    # ===========================================
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

    # Control mode
    use_rate_mode = (control_mode == 'rate')
    prev_mode_button = False
    print(f"Control mode: {'ACRO (Rate)' if use_rate_mode else 'STABILIZE (Angle)'}")

    # ===========================================
    # Joystick Calibration
    # ===========================================
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

    # ===========================================
    # Simulation State
    # ===========================================
    sim_time = 0.0  # Simulation time [s]
    physics_steps = 0
    control_steps = 0
    render_steps = 0

    next_control_time = 0.0
    next_render_time = 0.0

    # Control state
    roll_ref = 0.0
    pitch_ref = 0.0
    yaw_ref = 0.0
    delta_thrust = 0.0

    # Control outputs (torque)
    roll_torque = 0.0
    pitch_torque = 0.0
    yaw_torque = 0.0

    # Current motor voltages
    voltage = [0.0, 0.0, 0.0, 0.0]

    # Data logging (reduced frequency to save memory)
    LOG_INTERVAL = 10  # Log every 10 physics steps (200Hz logging)
    T_log = []
    PQR_log = []
    EULER_log = []
    POS_log = []

    # Performance monitoring
    perf_physics_time = 0.0
    perf_control_time = 0.0
    perf_render_time = 0.0
    perf_last_report = 0.0
    perf_physics_count = 0
    perf_control_count = 0
    perf_render_count = 0

    print("\n" + "=" * 60)
    print(f"Roll Rate PID: Kp={cfg.roll_kp:.2e} Nm/(rad/s)")
    print(f"Pitch Rate PID: Kp={cfg.pitch_kp:.2e} Nm/(rad/s)")
    print(f"Yaw Rate PID: Kp={cfg.yaw_kp:.2e} Nm/(rad/s)")
    print(f"Hover thrust: {hover_thrust*1000:.1f} mN")
    print("=" * 60)
    print("\nStarting simulation...")

    # ===========================================
    # Main Simulation Loop (Real-time synchronized)
    # ===========================================
    wall_start = time.perf_counter()
    last_print_time = -1

    try:
        while sim_time < 6000.0:
            wall_now = time.perf_counter() - wall_start

            # ===========================================
            # Read Joystick (every loop iteration)
            # ===========================================
            joydata = joystick.read()
            if joydata is not None:
                thrust_raw = (joydata[0] - 127) / 127.0 - thrust_offset
                roll_raw = (joydata[1] - 127) / 127.0 - roll_offset
                pitch_raw = (joydata[2] - 127) / 127.0 - pitch_offset
                yaw_raw = (joydata[3] - 127) / 127.0 - yaw_offset

                # Mode button
                buttons = joydata[4] if len(joydata) > 4 else 0
                mode_button = bool(buttons & 0x04)

                if mode_button and not prev_mode_button:
                    use_rate_mode = not use_rate_mode
                    mode_name = 'ACRO (Rate)' if use_rate_mode else 'STABILIZE (Angle)'
                    print(f"Mode changed: {mode_name}")
                prev_mode_button = mode_button

                delta_thrust = 0.5 * thrust_raw * hover_thrust

                if use_rate_mode:
                    roll_ref = cfg.roll_rate_max * roll_raw
                    pitch_ref = cfg.pitch_rate_max * pitch_raw
                    yaw_ref = cfg.yaw_rate_max * yaw_raw
                else:
                    roll_ref = 0.25 * roll_raw * np.pi
                    pitch_ref = 0.25 * pitch_raw * np.pi
                    yaw_ref = 0.3 * yaw_raw * np.pi

            # ===========================================
            # Physics Loop (catch up to real time)
            # ===========================================
            while sim_time <= wall_now:
                t0 = time.perf_counter()

                # Control update at 400Hz
                if sim_time >= next_control_time:
                    rate_p = stampfly.body.pqr[0][0]
                    rate_q = stampfly.body.pqr[1][0]
                    rate_r = stampfly.body.pqr[2][0]
                    phi = stampfly.body.euler[0][0]
                    theta = stampfly.body.euler[1][0]

                    if use_rate_mode:
                        roll_rate_ref = roll_ref
                        pitch_rate_ref = pitch_ref
                        yaw_rate_ref = yaw_ref
                    else:
                        roll_rate_ref = roll_pid.update(roll_ref, phi, CONTROL_DT)
                        pitch_rate_ref = pitch_pid.update(pitch_ref, theta, CONTROL_DT)
                        yaw_rate_ref = yaw_ref

                    roll_torque = roll_rate_pid.update(roll_rate_ref, rate_p, CONTROL_DT)
                    pitch_torque = pitch_rate_pid.update(pitch_rate_ref, rate_q, CONTROL_DT)
                    yaw_torque = yaw_rate_pid.update(yaw_rate_ref, rate_r, CONTROL_DT)

                    # Control allocation
                    total_thrust = hover_thrust + delta_thrust
                    total_thrust = max(0.0, min(total_thrust, 4 * 0.15))
                    motor_thrusts = allocator.mix(total_thrust, roll_torque, pitch_torque, yaw_torque)
                    motor_voltages = allocator.thrusts_to_voltages(motor_thrusts)
                    voltage = [motor_voltages[0], motor_voltages[1], motor_voltages[2], motor_voltages[3]]

                    control_steps += 1
                    next_control_time = control_steps * CONTROL_DT
                    perf_control_count += 1
                    perf_control_time += time.perf_counter() - t0
                    t0 = time.perf_counter()

                # Physics step
                stampfly.step(voltage, PHYSICS_DT)
                physics_steps += 1
                sim_time = physics_steps * PHYSICS_DT
                perf_physics_count += 1
                perf_physics_time += time.perf_counter() - t0

                # Data logging (reduced frequency)
                if physics_steps % LOG_INTERVAL == 0:
                    T_log.append(sim_time)
                    PQR_log.append(stampfly.body.pqr.copy())
                    EULER_log.append(stampfly.body.euler.copy())
                    POS_log.append(stampfly.body.position.copy())

                # Collision detection
                pos = stampfly.body.position
                if Render.check_collision(pos[0][0], pos[1][0], pos[2][0]):
                    print(f"COLLISION at t={sim_time:.2f}s")
                    Render.show_collision(pos[0][0], pos[1][0], pos[2][0])
                    raise KeyboardInterrupt  # Exit cleanly

            # ===========================================
            # Rendering at target FPS (without rate() blocking)
            # ===========================================
            if wall_now >= next_render_time:
                t0 = time.perf_counter()
                # Direct VPython update without rate() blocking
                # rate()を使わず直接VPythonオブジェクトを更新
                Render.copter.pos = vector(*stampfly.body.position)
                axis_x = vector(stampfly.body.DCM[0,0], stampfly.body.DCM[1,0], stampfly.body.DCM[2,0])
                axis_z = vector(stampfly.body.DCM[0,2], stampfly.body.DCM[1,2], stampfly.body.DCM[2,2])
                Render.copter.axis = axis_x
                Render.copter.up = axis_z
                Render.follow_camera_setting(stampfly, t=sim_time)
                Render.timer_text.text = f"Elapsed Time: {sim_time:.1f} s"
                render_steps += 1
                next_render_time = render_steps * RENDER_DT
                perf_render_count += 1
                perf_render_time += time.perf_counter() - t0

            # ===========================================
            # Performance Report (every second)
            # ===========================================
            current_second = int(sim_time)
            if current_second > last_print_time:
                last_print_time = current_second

                # Calculate real-time ratio
                rt_ratio = sim_time / wall_now if wall_now > 0 else 1.0

                # Get state for display
                euler = stampfly.body.euler
                pqr = stampfly.body.pqr
                pos = stampfly.body.position

                mode_str = "ACRO" if use_rate_mode else "STAB"
                print(f"[{mode_str}] t={sim_time:.1f}s RT={rt_ratio:.2f}x | "
                      f"pos=({pos[0][0]:+.1f},{pos[1][0]:+.1f},{pos[2][0]:.1f}) | "
                      f"RPY=({np.degrees(euler[0][0]):+.0f},{np.degrees(euler[1][0]):+.0f},{np.degrees(euler[2][0]):+.0f})")

                # Detailed performance every 5 seconds
                if current_second % 5 == 0 and current_second > 0:
                    elapsed = wall_now - perf_last_report
                    if elapsed > 0:
                        actual_physics_hz = perf_physics_count / elapsed
                        actual_control_hz = perf_control_count / elapsed
                        actual_render_fps = perf_render_count / elapsed
                        print(f"  Performance: Physics={actual_physics_hz:.0f}Hz "
                              f"Control={actual_control_hz:.0f}Hz "
                              f"Render={actual_render_fps:.0f}FPS")

                    perf_last_report = wall_now
                    perf_physics_count = 0
                    perf_control_count = 0
                    perf_render_count = 0

            # Small sleep to prevent CPU spinning when ahead of real-time
            sleep_time = sim_time - wall_now
            if sleep_time > 0.001:
                time.sleep(sleep_time * 0.5)  # Sleep for half the available time

    except KeyboardInterrupt:
        pass

    print("\nSimulation ended.")
    print(f"Total physics steps: {physics_steps}")
    print(f"Total control steps: {control_steps}")
    print(f"Total render frames: {render_steps}")

    # ===========================================
    # Plot Results
    # ===========================================
    if len(T_log) > 0:
        T_log = np.array(T_log)
        PQR_log = np.array(PQR_log)
        EULER_log = np.array(EULER_log)
        POS_log = np.array(POS_log)

        plt.figure(figsize=(12, 10))

        plt.subplot(3, 1, 1)
        plt.plot(T_log, PQR_log[:, 0, 0], label='P (roll rate)')
        plt.plot(T_log, PQR_log[:, 1, 0], label='Q (pitch rate)')
        plt.plot(T_log, PQR_log[:, 2, 0], label='R (yaw rate)')
        plt.legend()
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Rate (rad/s)')
        plt.title('Angular Rates (2000Hz Physics, 400Hz Control)')

        plt.subplot(3, 1, 2)
        plt.plot(T_log, np.rad2deg(EULER_log[:, 0, 0]), label='φ (roll)')
        plt.plot(T_log, np.rad2deg(EULER_log[:, 1, 0]), label='θ (pitch)')
        plt.plot(T_log, np.rad2deg(EULER_log[:, 2, 0]), label='ψ (yaw)')
        plt.legend()
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('Euler Angle (deg)')
        plt.title('Euler Angles')

        plt.subplot(3, 1, 3)
        plt.plot(T_log, POS_log[:, 0, 0], label='X')
        plt.plot(T_log, POS_log[:, 1, 0], label='Y')
        plt.plot(T_log, POS_log[:, 2, 0], label='Z')
        plt.legend()
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Position')

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='High-Frequency Flight Simulator (2000Hz Physics)')
    parser.add_argument('--world', '-w', type=str, default='voxel',
                        choices=['ringworld', 'voxel'],
                        help='World type (default: voxel)')
    parser.add_argument('--seed', '-s', type=int, default=None,
                        help='Terrain seed (random if not specified)')
    parser.add_argument('--mode', '-m', type=str, default='rate',
                        choices=['rate', 'angle'],
                        help='Control mode: rate=ACRO, angle=STABILIZE (default: rate)')
    args = parser.parse_args()

    print(f"Starting 2000Hz simulation: world={args.world}, mode={args.mode}")
    flight_sim_2000hz(world_type=args.world, seed=args.seed, control_mode=args.mode)
