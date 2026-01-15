#!/usr/bin/env python3
"""
run_genesis_headless.py - Genesis Headless Simulation for Comparison Testing
Genesis ヘッドレスシミュレーション（比較テスト用）

Runs simulation without viewer, with file-based input/output.
ビューアなしでシミュレーションを実行、ファイルベースの入出力

Usage:
  python run_genesis_headless.py --input input.csv --output output.csv --duration 10
"""

import sys
from pathlib import Path

script_dir = Path(__file__).parent
genesis_dir = script_dir.parent
simulator_dir = genesis_dir.parent
vpython_dir = simulator_dir / "vpython"
tools_dir = simulator_dir / "tools" / "compare_simulators"

sys.path.insert(0, str(genesis_dir))  # for motor_model, control_allocation
sys.path.insert(0, str(vpython_dir))  # for control.pid
sys.path.insert(0, str(tools_dir))    # for sim_io

import genesis as gs
import numpy as np
import time
import argparse

from motor_model import QuadMotorSystem, compute_hover_conditions
from control_allocation import ControlAllocator, thrusts_to_duties
from control.pid import PID
from sim_io import load_input_csv, save_output_csv, StateLog, get_input_at_time, CONTROL_DT


# =============================================================================
# Physical Constants
# =============================================================================
GRAVITY = 9.81
MASS = 0.035
WEIGHT = MASS * GRAVITY


# =============================================================================
# Coordinate Transforms (same as run_genesis_sim.py)
# =============================================================================

def quat_to_rotation_matrix(quat):
    w, x, y, z = [float(v) for v in quat]
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


def quat_to_euler(quat):
    w, x, y, z = [float(v) for v in quat]
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


def ned_to_genesis_force(force_ned):
    return np.array([force_ned[1], force_ned[0], -force_ned[2]])


def ned_to_genesis_moment(moment_ned):
    return np.array([moment_ned[1], moment_ned[0], -moment_ned[2]])


def genesis_gyro_to_ned(gyro_genesis_body):
    return np.array([gyro_genesis_body[1], gyro_genesis_body[0], -gyro_genesis_body[2]])


def genesis_euler_to_ned(euler_genesis):
    return np.array([euler_genesis[1], euler_genesis[0], -euler_genesis[2]])


# =============================================================================
# Rate Controller (simplified from run_genesis_sim.py)
# =============================================================================

class RateController:
    def __init__(self):
        self.roll_pid = PID(Kp=9.1e-4, Ti=0.7, Td=0.01, eta=0.125,
                           output_min=-5.2e-3, output_max=5.2e-3)
        self.pitch_pid = PID(Kp=1.33e-3, Ti=0.7, Td=0.025, eta=0.125,
                            output_min=-5.2e-3, output_max=5.2e-3)
        self.yaw_pid = PID(Kp=1.77e-3, Ti=0.8, Td=0.01, eta=0.125,
                          output_min=-2.2e-3, output_max=2.2e-3)
        self.roll_rate_max = 1.0
        self.pitch_rate_max = 1.0
        self.yaw_rate_max = 5.0

    def update_from_stick(self, roll, pitch, yaw, gyro, dt):
        rate_ref = np.array([
            roll * self.roll_rate_max,
            pitch * self.pitch_rate_max,
            yaw * self.yaw_rate_max,
        ])
        roll_torque = self.roll_pid.update(rate_ref[0], gyro[0], dt)
        pitch_torque = self.pitch_pid.update(rate_ref[1], gyro[1], dt)
        yaw_torque = self.yaw_pid.update(rate_ref[2], gyro[2], dt)
        return roll_torque, pitch_torque, yaw_torque

    def reset(self):
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()


# =============================================================================
# Main Headless Simulation
# =============================================================================

def run_headless(input_file, output_file, duration=10.0):
    """
    Run headless simulation with file I/O.
    ファイル入出力でヘッドレスシミュレーションを実行
    """
    print("=" * 60)
    print("Genesis Headless Simulation")
    print("Genesis ヘッドレスシミュレーション")
    print("=" * 60)

    # Load input sequence
    input_sequence = load_input_csv(input_file)
    print(f"Input: {input_file} ({len(input_sequence)} samples)")
    print(f"Output: {output_file}")
    print(f"Duration: {duration}s")

    # Timing
    PHYSICS_HZ = 2000
    PHYSICS_DT = 1 / PHYSICS_HZ
    CONTROL_HZ = 400
    CONTROL_DT = 1 / CONTROL_HZ
    LOG_HZ = 100  # Log at 100Hz
    LOG_INTERVAL = int(PHYSICS_HZ / LOG_HZ)

    print(f"\nPhysics: {PHYSICS_HZ}Hz, Control: {CONTROL_HZ}Hz, Log: {LOG_HZ}Hz")

    # URDF path
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly_fixed.urdf"
    if not urdf_file.exists():
        print(f"ERROR: URDF not found: {urdf_file}")
        return

    # Initialize Genesis (headless)
    print("\n[1] Initializing Genesis (headless)...")
    gs.init(backend=gs.cpu)

    scene = gs.Scene(
        show_viewer=False,
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -GRAVITY),
            dt=PHYSICS_DT,
        ),
    )

    # Add drone only (no ground for headless comparison)
    # 比較用ヘッドレスでは床なし
    drone = scene.add_entity(
        gs.morphs.URDF(
            file=str(urdf_file),
            pos=(0, 0, 0.0),  # Start at origin
            euler=(0, 0, 0),
            fixed=False,
        ),
    )
    scene.build()

    # Initialize control systems
    print("[2] Initializing control systems...")
    allocator = ControlAllocator()
    motor_system = QuadMotorSystem()
    rate_controller = RateController()

    hover = compute_hover_conditions()
    for motor in motor_system.motors:
        motor.omega = hover['omega_hover']

    # Simulation state
    physics_steps = 0
    control_steps = 0
    next_control_time = 0
    current_torque = np.array([0.0, 0.0, 0.0])

    # Output log
    state_logs = []

    print(f"\n[3] Running simulation for {duration}s...")
    start_time = time.perf_counter()

    while physics_steps * PHYSICS_DT < duration:
        sim_time = physics_steps * PHYSICS_DT

        # Control loop (400Hz)
        if sim_time >= next_control_time:
            # Get input from sequence
            inp = get_input_at_time(input_sequence, sim_time, CONTROL_DT)

            # Get current state
            dofs_vel = drone.get_dofs_velocity()
            gyro_genesis = np.array([float(dofs_vel[3]), float(dofs_vel[4]), float(dofs_vel[5])])
            gyro_ned = genesis_gyro_to_ned(gyro_genesis)

            # Rate control
            roll_torque, pitch_torque, yaw_torque = rate_controller.update_from_stick(
                inp.roll, inp.pitch, inp.yaw, gyro_ned, CONTROL_DT
            )
            current_torque = np.array([roll_torque, pitch_torque, yaw_torque])

            # Thrust command
            u_thrust = WEIGHT + inp.throttle * 0.5 * WEIGHT

            # Control allocation
            control = np.array([u_thrust, current_torque[0], current_torque[1], current_torque[2]])
            target_thrusts = allocator.mix(control)
            target_duties = thrusts_to_duties(target_thrusts)

            control_steps += 1
            next_control_time = control_steps * CONTROL_DT

        # Motor dynamics
        force_ned, moment_ned = motor_system.step_with_duty(target_duties.tolist(), PHYSICS_DT)
        force_genesis = ned_to_genesis_force(force_ned)
        moment_genesis = ned_to_genesis_moment(moment_ned)

        # Apply to drone
        quat = drone.get_quat()
        R = quat_to_rotation_matrix(quat)
        force_world = R @ force_genesis

        dof_force = np.zeros(drone.n_dofs)
        dof_force[0:3] = force_world
        dof_force[3:6] = moment_genesis
        drone.control_dofs_force(dof_force)

        # Physics step
        scene.step()
        physics_steps += 1

        # Logging (at LOG_HZ)
        if physics_steps % LOG_INTERVAL == 0:
            pos = drone.get_pos()
            quat = drone.get_quat()
            euler_genesis = quat_to_euler(quat)
            euler_ned = genesis_euler_to_ned(euler_genesis)

            dofs_vel = drone.get_dofs_velocity()
            gyro_genesis = np.array([float(dofs_vel[3]), float(dofs_vel[4]), float(dofs_vel[5])])
            gyro_ned = genesis_gyro_to_ned(gyro_genesis)

            state_logs.append(StateLog(
                time=sim_time,
                x=float(pos[0]), y=float(pos[1]), z=float(pos[2]),
                roll=euler_ned[0], pitch=euler_ned[1], yaw=euler_ned[2],
                p=gyro_ned[0], q=gyro_ned[1], r=gyro_ned[2],
            ))

        # Progress report
        if physics_steps % (PHYSICS_HZ * 1) == 0:  # Every second
            elapsed = time.perf_counter() - start_time
            speed = sim_time / elapsed if elapsed > 0 else 0
            print(f"  t={sim_time:.1f}s ({speed:.1f}x realtime)")

    # Save output
    print(f"\n[4] Saving output to {output_file}...")
    metadata = {
        'simulator': 'genesis',
        'physics_hz': PHYSICS_HZ,
        'control_hz': CONTROL_HZ,
        'input_file': input_file,
    }
    save_output_csv(output_file, state_logs, metadata)

    elapsed = time.perf_counter() - start_time
    print(f"\nDone! Simulated {duration}s in {elapsed:.1f}s ({duration/elapsed:.1f}x realtime)")
    print(f"Output: {len(state_logs)} samples -> {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Genesis Headless Simulation')
    parser.add_argument('--input', '-i', type=str, required=True,
                       help='Input CSV file')
    parser.add_argument('--output', '-o', type=str, required=True,
                       help='Output CSV file')
    parser.add_argument('--duration', '-d', type=float, default=10.0,
                       help='Simulation duration [s]')
    args = parser.parse_args()

    run_headless(args.input, args.output, args.duration)
