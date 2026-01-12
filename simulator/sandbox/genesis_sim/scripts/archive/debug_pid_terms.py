#!/usr/bin/env python3
"""
debug_pid_terms.py - Test I and D terms separately
I項とD項を別々にテスト

Usage:
  python debug_pid_terms.py p      # P only
  python debug_pid_terms.py pi     # P + I
  python debug_pid_terms.py pd     # P + D
  python debug_pid_terms.py pid    # Full PID
"""

import sys
from pathlib import Path

script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir.parent))
sys.path.insert(0, str(script_dir.parent.parent.parent))

import genesis as gs
import numpy as np
import time

from motor_model import QuadMotorSystem, compute_hover_conditions
from control_allocation import ControlAllocator, thrusts_to_duties
from control.pid import PID

GRAVITY = 9.81
MASS = 0.035
WEIGHT = MASS * GRAVITY

PHYSICS_HZ = 2000
PHYSICS_DT = 1 / PHYSICS_HZ
CONTROL_HZ = 400
CONTROL_DT = 1 / CONTROL_HZ


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


def genesis_gyro_to_ned(gyro_genesis_body):
    return np.array([gyro_genesis_body[1], gyro_genesis_body[0], -gyro_genesis_body[2]])


def ned_to_genesis_force(force_ned):
    return np.array([force_ned[1], force_ned[0], -force_ned[2]])


def ned_to_genesis_moment(moment_ned):
    return np.array([moment_ned[1], moment_ned[0], -moment_ned[2]])


def genesis_euler_to_ned(euler_genesis):
    return np.array([euler_genesis[1], euler_genesis[0], -euler_genesis[2]])


def main():
    # Parse command line
    mode = sys.argv[1] if len(sys.argv) > 1 else "p"
    mode = mode.lower()

    use_i = "i" in mode
    use_d = "d" in mode

    print("=" * 70)
    print(f"PID Terms Test: mode={mode.upper()}")
    print(f"P: ENABLED")
    print(f"I: {'ENABLED' if use_i else 'disabled'}")
    print(f"D: {'ENABLED' if use_d else 'disabled'}")
    print("=" * 70)

    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly_fixed.urdf"

    gs.init(backend=gs.cpu)

    scene = gs.Scene(
        show_viewer=False,  # Headless
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -GRAVITY),
            dt=PHYSICS_DT,
        ),
    )

    scene.add_entity(gs.morphs.Plane(collision=True))

    drone = scene.add_entity(
        gs.morphs.URDF(
            file=str(urdf_file),
            pos=(0, 0, 1.0),
            euler=(0, 0, 0),
            fixed=False,
        ),
    )

    scene.build()

    # Motor system and control allocation
    motor_system = QuadMotorSystem()
    allocator = ControlAllocator()

    # Initialize motors at hover
    hover = compute_hover_conditions()
    for motor in motor_system.motors:
        motor.omega = hover['omega_hover']

    # Configure PID based on mode
    Ti_roll = 0.7 if use_i else 0
    Ti_pitch = 0.7 if use_i else 0
    Ti_yaw = 0.8 if use_i else 0
    Td_roll = 0.01 if use_d else 0
    Td_pitch = 0.025 if use_d else 0
    Td_yaw = 0.01 if use_d else 0

    roll_pid = PID(Kp=9.1e-4, Ti=Ti_roll, Td=Td_roll, eta=0.125,
                   output_min=-5.2e-3, output_max=5.2e-3)
    pitch_pid = PID(Kp=1.33e-3, Ti=Ti_pitch, Td=Td_pitch, eta=0.125,
                    output_min=-5.2e-3, output_max=5.2e-3)
    yaw_pid = PID(Kp=1.77e-3, Ti=Ti_yaw, Td=Td_yaw, eta=0.125,
                  output_min=-2.2e-3, output_max=2.2e-3)

    print(f"Roll PID:  Kp={9.1e-4:.2e}, Ti={Ti_roll}, Td={Td_roll}")
    print(f"Pitch PID: Kp={1.33e-3:.2e}, Ti={Ti_pitch}, Td={Td_pitch}")
    print(f"Yaw PID:   Kp={1.77e-3:.2e}, Ti={Ti_yaw}, Td={Td_yaw}")
    print()

    physics_steps = 0
    control_steps = 0
    next_control_time = 0
    last_print = -1
    duration = 10.0

    current_torque = np.array([0.0, 0.0, 0.0])

    while physics_steps * PHYSICS_DT < duration:
        sim_time = physics_steps * PHYSICS_DT

        # Control loop
        if sim_time >= next_control_time:
            dofs_vel = drone.get_dofs_velocity()
            gyro_genesis = np.array([float(dofs_vel[3]), float(dofs_vel[4]), float(dofs_vel[5])])
            gyro_ned = genesis_gyro_to_ned(gyro_genesis)

            roll_torque = roll_pid.update(0.0, gyro_ned[0], CONTROL_DT)
            pitch_torque = pitch_pid.update(0.0, gyro_ned[1], CONTROL_DT)
            yaw_torque = yaw_pid.update(0.0, gyro_ned[2], CONTROL_DT)
            current_torque = np.array([roll_torque, pitch_torque, yaw_torque])

            # Debug: Print PID terms for first 10 steps
            if control_steps < 10:
                P_r, I_r, D_r = roll_pid.get_terms()
                P_p, I_p, D_p = pitch_pid.get_terms()
                print(f"[{control_steps:3d}] gyro=({np.degrees(gyro_ned[0]):+6.2f},{np.degrees(gyro_ned[1]):+6.2f},{np.degrees(gyro_ned[2]):+6.2f})°/s "
                      f"| Roll P={P_r*1e6:+8.1f} I={I_r*1e6:+8.1f} D={D_r*1e6:+8.1f} µNm "
                      f"| Pitch P={P_p*1e6:+8.1f} I={I_p*1e6:+8.1f} D={D_p*1e6:+8.1f} µNm")

            control_steps += 1
            next_control_time = control_steps * CONTROL_DT

        # Control allocation
        control = np.array([WEIGHT, current_torque[0], current_torque[1], current_torque[2]])
        target_thrusts = allocator.mix(control)
        target_duties = thrusts_to_duties(target_thrusts)

        # Physics step
        force_ned, moment_ned = motor_system.step_with_duty(target_duties.tolist(), PHYSICS_DT)
        force_genesis_body = ned_to_genesis_force(force_ned)
        moment_genesis_body = ned_to_genesis_moment(moment_ned)

        quat = drone.get_quat()
        R = quat_to_rotation_matrix(quat)
        force_world = R @ force_genesis_body

        dof_force = np.zeros(drone.n_dofs)
        dof_force[0:3] = force_world
        dof_force[3:6] = moment_genesis_body
        drone.control_dofs_force(dof_force)

        scene.step()
        physics_steps += 1

        # Status every second
        current_sec = int(sim_time)
        if current_sec > last_print:
            last_print = current_sec
            pos = drone.get_pos()
            euler = genesis_euler_to_ned(quat_to_euler(drone.get_quat()))
            dofs_vel_now = drone.get_dofs_velocity()
            gyro_disp = genesis_gyro_to_ned(np.array([float(dofs_vel_now[3]), float(dofs_vel_now[4]), float(dofs_vel_now[5])]))

            print(f"t={sim_time:.0f}s | z={float(pos[2]):.2f}m | "
                  f"RPY=({np.degrees(euler[0]):+.0f},{np.degrees(euler[1]):+.0f},{np.degrees(euler[2]):+.0f})° | "
                  f"gyro=({np.degrees(gyro_disp[0]):+.0f},{np.degrees(gyro_disp[1]):+.0f},{np.degrees(gyro_disp[2]):+.0f})°/s | "
                  f"τ=({current_torque[0]*1e6:+.0f},{current_torque[1]*1e6:+.0f},{current_torque[2]*1e6:+.0f})µNm")

            if max(abs(euler[0]), abs(euler[1])) > np.radians(30):
                print("\n!!! DIVERGENCE DETECTED !!!")
                break

    final_time = physics_steps * PHYSICS_DT
    diverged = final_time < duration - 0.1
    result = "FAILED (diverged)" if diverged else "PASSED"
    print(f"\nResult: {result} at t={final_time:.1f}s")


if __name__ == "__main__":
    main()
