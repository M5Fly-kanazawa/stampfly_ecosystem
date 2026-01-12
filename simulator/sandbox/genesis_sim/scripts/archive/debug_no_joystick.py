#!/usr/bin/env python3
"""
debug_no_joystick.py - Test without joystick input
ジョイスティック入力なしのテスト

Test if the yaw drift is caused by joystick drift or something else.
Yawドリフトがジョイスティックのドリフトによるものかどうかをテスト。
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


class RateController:
    """Simple rate controller using PID with I/D enabled"""
    def __init__(self):
        # Full PID (same as firmware)
        self.roll_pid = PID(
            Kp=9.1e-4, Ti=0.7, Td=0.01, eta=0.125,
            output_min=-5.2e-3, output_max=5.2e-3
        )
        self.pitch_pid = PID(
            Kp=1.33e-3, Ti=0.7, Td=0.025, eta=0.125,
            output_min=-5.2e-3, output_max=5.2e-3
        )
        self.yaw_pid = PID(
            Kp=1.77e-3, Ti=0.8, Td=0.01, eta=0.125,
            output_min=-2.2e-3, output_max=2.2e-3
        )

    def update(self, gyro_ned, dt):
        """Update with zero setpoint (hover)"""
        roll_torque = self.roll_pid.update(0.0, gyro_ned[0], dt)
        pitch_torque = self.pitch_pid.update(0.0, gyro_ned[1], dt)
        yaw_torque = self.yaw_pid.update(0.0, gyro_ned[2], dt)
        return np.array([roll_torque, pitch_torque, yaw_torque])

    def reset(self):
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()


def main():
    print("=" * 70)
    print("Debug: No Joystick Test (Full PID with I/D enabled)")
    print("=" * 70)
    print()
    print("Testing without joystick input to isolate drift source.")
    print("Using FULL PID (same gains as VPython/firmware).")
    print()

    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly_fixed.urdf"

    gs.init(backend=gs.cpu)

    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0, -2, 1.5),
            camera_lookat=(0, 0, 1),
            camera_fov=60,
            max_FPS=30,
        ),
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

    # Rate controller with FULL PID
    rate_controller = RateController()

    print(f"Motors initialized at ω={hover['omega_hover']:.0f} rad/s")
    print(f"PID gains: Roll Kp={9.1e-4:.2e}, Ti=0.7, Td=0.01")
    print(f"          Pitch Kp={1.33e-3:.2e}, Ti=0.7, Td=0.025")
    print(f"          Yaw Kp={1.77e-3:.2e}, Ti=0.8, Td=0.01")
    print()

    physics_steps = 0
    control_steps = 0
    start_time = time.perf_counter()
    next_control_time = 0
    last_print_sec = -1

    current_torque = np.array([0.0, 0.0, 0.0])

    try:
        while scene.viewer.is_alive():
            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            # Control loop
            if sim_time >= next_control_time:
                dofs_vel = drone.get_dofs_velocity()
                gyro_genesis = np.array([float(dofs_vel[3]), float(dofs_vel[4]), float(dofs_vel[5])])
                gyro_ned = genesis_gyro_to_ned(gyro_genesis)

                current_torque = rate_controller.update(gyro_ned, CONTROL_DT)

                control_steps += 1
                next_control_time = control_steps * CONTROL_DT

            # Control allocation
            control = np.array([WEIGHT, current_torque[0], current_torque[1], current_torque[2]])
            target_thrusts = allocator.mix(control)
            target_duties = thrusts_to_duties(target_thrusts)

            # Physics loop
            while sim_time <= real_time and sim_time < 30.0:
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

                scene.step(update_visualizer=False, refresh_visualizer=False)
                physics_steps += 1
                sim_time = physics_steps * PHYSICS_DT

            scene.visualizer.update()

            # Status every second
            current_sec = int(sim_time)
            if current_sec > last_print_sec:
                last_print_sec = current_sec
                pos = drone.get_pos()
                euler = genesis_euler_to_ned(quat_to_euler(drone.get_quat()))
                dofs_vel_now = drone.get_dofs_velocity()
                gyro_disp = genesis_gyro_to_ned(np.array([float(dofs_vel_now[3]), float(dofs_vel_now[4]), float(dofs_vel_now[5])]))

                print(f"t={sim_time:.0f}s | z={float(pos[2]):.2f}m | "
                      f"RPY=({np.degrees(euler[0]):+.0f},{np.degrees(euler[1]):+.0f},{np.degrees(euler[2]):+.0f})° | "
                      f"gyro=({np.degrees(gyro_disp[0]):+.0f},{np.degrees(gyro_disp[1]):+.0f},{np.degrees(gyro_disp[2]):+.0f})°/s | "
                      f"τ=({current_torque[0]*1e6:+.0f},{current_torque[1]*1e6:+.0f},{current_torque[2]*1e6:+.0f})µNm")

                # Check for divergence
                if max(abs(euler[0]), abs(euler[1])) > np.radians(30):
                    print("\n!!! DIVERGENCE DETECTED !!!")
                    break

            if sim_time >= 30.0:
                print("\n>>> Test completed - 30 seconds stable")
                break

            time.sleep(max(0, sim_time - real_time))

    except KeyboardInterrupt:
        pass

    print("\nTest ended.")


if __name__ == "__main__":
    main()
