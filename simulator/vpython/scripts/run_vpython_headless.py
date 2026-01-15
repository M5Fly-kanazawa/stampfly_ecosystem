#!/usr/bin/env python3
"""
run_vpython_headless.py - VPython Headless Simulation for Comparison Testing
VPython ヘッドレスシミュレーション（比較テスト用）

Runs VPython physics without visualization, with file-based input/output.
VPython物理をビジュアライゼーションなしで実行、ファイルベースの入出力

Usage:
  python run_vpython_headless.py --input input.csv --output output.csv --duration 10
"""

import sys
import os
import math
import time
import numpy as np
import argparse

# Add paths
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_VPYTHON_DIR = os.path.dirname(_SCRIPT_DIR)
_SIMULATOR_DIR = os.path.dirname(_VPYTHON_DIR)
_TOOLS_DIR = os.path.join(_SIMULATOR_DIR, "tools", "compare_simulators")

if _VPYTHON_DIR not in sys.path:
    sys.path.insert(0, _VPYTHON_DIR)
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

from core import dynamics as mc
from control.pid import PID
from sim_io import load_input_csv, save_output_csv, StateLog, get_input_at_time


# =============================================================================
# Motor Model Parameters (matching firmware motor_model.hpp)
# =============================================================================
class MotorParams:
    Ct = 1.0e-8
    Cq = 9.71e-11
    Rm = 0.34
    Km = 6.125e-4
    Dm = 3.69e-8
    Qf = 2.76e-5
    Vbat = 3.7


def thrust_to_omega(thrust):
    if thrust <= 0:
        return 0.0
    return math.sqrt(thrust / MotorParams.Ct)


def omega_to_voltage(omega):
    if omega <= 0:
        return 0.0
    p = MotorParams
    viscous_term = (p.Dm + p.Km * p.Km / p.Rm) * omega
    aero_term = p.Cq * omega * omega
    friction_term = p.Qf
    voltage = p.Rm * (viscous_term + aero_term + friction_term) / p.Km
    return voltage


def thrust_to_voltage(thrust):
    if thrust <= 0:
        return 0.0
    omega = thrust_to_omega(thrust)
    voltage = omega_to_voltage(omega)
    return min(max(voltage, 0.0), MotorParams.Vbat)


# =============================================================================
# Control Allocation
# =============================================================================
class ControlAllocator:
    def __init__(self):
        self.d = 0.023
        self.kappa = 9.71e-3
        self.max_thrust = 0.15

        inv_d = 1.0 / self.d
        inv_kappa = 1.0 / self.kappa

        self.B_inv = np.array([
            [0.25, -0.25 * inv_d,  0.25 * inv_d,  0.25 * inv_kappa],
            [0.25, -0.25 * inv_d, -0.25 * inv_d, -0.25 * inv_kappa],
            [0.25,  0.25 * inv_d, -0.25 * inv_d,  0.25 * inv_kappa],
            [0.25,  0.25 * inv_d,  0.25 * inv_d, -0.25 * inv_kappa],
        ])

    def mix(self, total_thrust, roll_torque, pitch_torque, yaw_torque):
        control = np.array([total_thrust, roll_torque, pitch_torque, yaw_torque])
        thrusts = self.B_inv @ control
        thrusts = np.clip(thrusts, 0.0, self.max_thrust)
        return thrusts

    def thrusts_to_voltages(self, thrusts):
        return np.array([thrust_to_voltage(t) for t in thrusts])


# =============================================================================
# PID Configuration
# =============================================================================
class PIDConfig:
    roll_rate_max = 1.0
    pitch_rate_max = 1.0
    yaw_rate_max = 5.0

    roll_kp = 9.1e-4
    roll_ti = 0.7
    roll_td = 0.01
    roll_limit = 5.2e-3

    pitch_kp = 1.33e-3
    pitch_ti = 0.7
    pitch_td = 0.025
    pitch_limit = 5.2e-3

    yaw_kp = 1.77e-3
    yaw_ti = 0.8
    yaw_td = 0.01
    yaw_limit = 2.2e-3


# =============================================================================
# Main Headless Simulation
# =============================================================================

def run_headless(input_file, output_file, duration=10.0):
    """
    Run headless VPython simulation.
    ヘッドレスVPythonシミュレーションを実行
    """
    print("=" * 60)
    print("VPython Headless Simulation")
    print("VPython ヘッドレスシミュレーション")
    print("=" * 60)

    # Load input
    input_sequence = load_input_csv(input_file)
    print(f"Input: {input_file} ({len(input_sequence)} samples)")
    print(f"Output: {output_file}")
    print(f"Duration: {duration}s")

    # Timing
    PHYSICS_HZ = 2000
    PHYSICS_DT = 1.0 / PHYSICS_HZ
    CONTROL_HZ = 400
    CONTROL_DT = 1.0 / CONTROL_HZ
    PHYSICS_PER_CONTROL = PHYSICS_HZ // CONTROL_HZ
    LOG_HZ = 100
    LOG_INTERVAL = PHYSICS_HZ // LOG_HZ

    print(f"\nPhysics: {PHYSICS_HZ}Hz, Control: {CONTROL_HZ}Hz, Log: {LOG_HZ}Hz")

    # Create drone
    mass = 0.035
    weight = mass * 9.81
    hover_thrust = weight

    stampfly = mc.multicopter(
        mass=mass,
        inersia=[[9.16e-6, 0.0, 0.0], [0.0, 13.3e-6, 0.0], [0.0, 0.0, 20.4e-6]]
    )

    # Initial state (start at origin for comparison)
    # 比較用に原点から開始
    stampfly.body.set_position([[0.0], [0.0], [0.0]])
    stampfly.set_pqr([[0.0], [0.0], [0.0]])
    stampfly.set_uvw([[0.0], [0.0], [0.0]])
    stampfly.set_euler([[0], [0], [0]])

    # Initialize motors at hover
    nominal_omega = stampfly.motor_prop[0].equilibrium_anguler_velocity(weight / 4)
    for mp in stampfly.motor_prop:
        mp.omega = nominal_omega

    stampfly.set_disturbance(moment=[0, 0, 0], force=[0, 0, 0])

    # Control systems
    allocator = ControlAllocator()
    cfg = PIDConfig

    roll_rate_pid = PID(cfg.roll_kp, cfg.roll_ti, cfg.roll_td,
                       output_min=-cfg.roll_limit, output_max=cfg.roll_limit)
    pitch_rate_pid = PID(cfg.pitch_kp, cfg.pitch_ti, cfg.pitch_td,
                        output_min=-cfg.pitch_limit, output_max=cfg.pitch_limit)
    yaw_rate_pid = PID(cfg.yaw_kp, cfg.yaw_ti, cfg.yaw_td,
                      output_min=-cfg.yaw_limit, output_max=cfg.yaw_limit)

    # Simulation state
    physics_steps = 0
    control_steps = 0
    next_control_time = 0.0
    voltage = [0.0, 0.0, 0.0, 0.0]

    # Output log
    state_logs = []

    print(f"\n[1] Running simulation for {duration}s...")
    start_time = time.perf_counter()

    while physics_steps * PHYSICS_DT < duration:
        sim_time = physics_steps * PHYSICS_DT

        # Control loop
        if sim_time >= next_control_time:
            # Get input
            inp = get_input_at_time(input_sequence, sim_time, CONTROL_DT)

            # Get state
            rate_p = stampfly.body.pqr[0][0]
            rate_q = stampfly.body.pqr[1][0]
            rate_r = stampfly.body.pqr[2][0]

            # Rate control
            roll_rate_ref = inp.roll * cfg.roll_rate_max
            pitch_rate_ref = inp.pitch * cfg.pitch_rate_max
            yaw_rate_ref = inp.yaw * cfg.yaw_rate_max

            roll_torque = roll_rate_pid.update(roll_rate_ref, rate_p, CONTROL_DT)
            pitch_torque = pitch_rate_pid.update(pitch_rate_ref, rate_q, CONTROL_DT)
            yaw_torque = yaw_rate_pid.update(yaw_rate_ref, rate_r, CONTROL_DT)

            # Thrust
            delta_thrust = inp.throttle * 0.5 * hover_thrust
            total_thrust = hover_thrust + delta_thrust
            total_thrust = max(0.0, min(total_thrust, 4 * 0.15))

            # Control allocation
            motor_thrusts = allocator.mix(total_thrust, roll_torque, pitch_torque, yaw_torque)
            motor_voltages = allocator.thrusts_to_voltages(motor_thrusts)
            voltage = list(motor_voltages)

            control_steps += 1
            next_control_time = control_steps * CONTROL_DT

        # Physics step
        stampfly.step(voltage, PHYSICS_DT)
        physics_steps += 1

        # Logging
        if physics_steps % LOG_INTERVAL == 0:
            pos = stampfly.body.position
            euler = stampfly.body.euler
            pqr = stampfly.body.pqr

            state_logs.append(StateLog(
                time=sim_time,
                x=pos[0][0], y=pos[1][0], z=pos[2][0],
                roll=euler[0][0], pitch=euler[1][0], yaw=euler[2][0],
                p=pqr[0][0], q=pqr[1][0], r=pqr[2][0],
            ))

        # Progress
        if physics_steps % (PHYSICS_HZ * 1) == 0:
            elapsed = time.perf_counter() - start_time
            speed = sim_time / elapsed if elapsed > 0 else 0
            print(f"  t={sim_time:.1f}s ({speed:.1f}x realtime)")

    # Save output
    print(f"\n[2] Saving output to {output_file}...")
    metadata = {
        'simulator': 'vpython',
        'physics_hz': PHYSICS_HZ,
        'control_hz': CONTROL_HZ,
        'input_file': input_file,
    }
    save_output_csv(output_file, state_logs, metadata)

    elapsed = time.perf_counter() - start_time
    print(f"\nDone! Simulated {duration}s in {elapsed:.1f}s ({duration/elapsed:.1f}x realtime)")
    print(f"Output: {len(state_logs)} samples -> {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='VPython Headless Simulation')
    parser.add_argument('--input', '-i', type=str, required=True,
                       help='Input CSV file')
    parser.add_argument('--output', '-o', type=str, required=True,
                       help='Output CSV file')
    parser.add_argument('--duration', '-d', type=float, default=10.0,
                       help='Simulation duration [s]')
    args = parser.parse_args()

    run_headless(args.input, args.output, args.duration)
