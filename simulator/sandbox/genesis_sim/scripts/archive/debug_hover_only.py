#!/usr/bin/env python3
"""
debug_hover_only.py - Hover Only Test (No Control)
ホバーのみテスト（制御なし）

Tests if the drone can hover with only thrust, no attitude control.
推力のみでホバリングできるかテスト。制御なし。
"""

import sys
from pathlib import Path

script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir.parent))

import genesis as gs
import numpy as np
import time

from motor_model import QuadMotorSystem, compute_hover_conditions

# =============================================================================
# Constants
# =============================================================================
GRAVITY = 9.81
MASS = 0.035
WEIGHT = MASS * GRAVITY

PHYSICS_HZ = 1000
PHYSICS_DT = 1 / PHYSICS_HZ
RENDER_FPS = 30


def quat_to_euler(quat):
    """Quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw) in rad"""
    w, x, y, z = [float(v) for v in quat]
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


def quat_to_rotation_matrix(quat):
    """Quaternion (w, x, y, z) to rotation matrix"""
    w, x, y, z = [float(v) for v in quat]
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


def main():
    print("=" * 60)
    print("Debug: Hover Only Test (No Control)")
    print("=" * 60)

    # Hover conditions
    hover = compute_hover_conditions()
    print(f"Hover thrust: {WEIGHT*1000:.1f} mN")
    print(f"Hover duty: {hover['duty_hover']*100:.1f}%")
    print(f"Hover omega: {hover['omega_hover']:.0f} rad/s")

    # Path setup
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly_fixed.urdf"

    if not urdf_file.exists():
        print(f"ERROR: URDF not found: {urdf_file}")
        return

    # Genesis init
    print("\nInitializing Genesis...")
    gs.init(backend=gs.cpu)

    DRONE_SPAWN_POS = (0, 0, 1.0)

    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0, -2, 1.5),
            camera_lookat=DRONE_SPAWN_POS,
            camera_fov=60,
            max_FPS=RENDER_FPS,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -GRAVITY),
            dt=PHYSICS_DT,
        ),
    )

    # Ground
    scene.add_entity(
        gs.morphs.Plane(collision=True),
        surface=gs.surfaces.Default(color=(0.3, 0.3, 0.3)),
    )

    # Drone
    drone = scene.add_entity(
        gs.morphs.URDF(
            file=str(urdf_file),
            pos=DRONE_SPAWN_POS,
            euler=(0, 0, 0),
            fixed=False,
        ),
    )

    scene.build()

    # Motor system
    motor_system = QuadMotorSystem()

    # Initialize motors at hover speed
    for motor in motor_system.motors:
        motor.omega = hover['omega_hover']

    print("\n" + "=" * 60)
    print("Test 1: Constant hover duty (no control)")
    print("If this is unstable, the issue is in physics/force application")
    print("=" * 60)

    physics_steps = 0
    start_time = time.perf_counter()
    last_print = -1

    # Constant hover duty for all motors
    hover_duty = hover['duty_hover']
    duties = [hover_duty, hover_duty, hover_duty, hover_duty]

    try:
        while scene.viewer.is_alive():
            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            # Physics loop
            while sim_time <= real_time:
                # Motor dynamics
                force_ned, moment_ned = motor_system.step_with_duty(duties, PHYSICS_DT)

                # NED to Genesis conversion
                # NED: X=forward, Y=right, Z=down
                # Genesis: X=right, Y=forward, Z=up
                force_genesis_body = np.array([force_ned[1], force_ned[0], -force_ned[2]])
                moment_genesis_body = np.array([moment_ned[1], moment_ned[0], -moment_ned[2]])

                # Body to world for force
                quat = drone.get_quat()
                R = quat_to_rotation_matrix(quat)
                force_world = R @ force_genesis_body

                # Apply force/torque
                dof_force = np.zeros(drone.n_dofs)
                dof_force[0:3] = force_world
                dof_force[3:6] = moment_genesis_body
                drone.control_dofs_force(dof_force)

                scene.step(update_visualizer=False, refresh_visualizer=False)
                physics_steps += 1
                sim_time = physics_steps * PHYSICS_DT

            # Render
            scene.visualizer.update()

            # Status
            current_sec = int(sim_time)
            if current_sec > last_print:
                last_print = current_sec
                pos = drone.get_pos()
                quat = drone.get_quat()
                euler = quat_to_euler(quat)

                # Get angular velocity
                dofs_vel = drone.get_dofs_velocity()
                gyro = np.array([float(dofs_vel[3]), float(dofs_vel[4]), float(dofs_vel[5])])

                print(f"t={sim_time:.0f}s | "
                      f"pos=({float(pos[0]):+.2f},{float(pos[1]):+.2f},{float(pos[2]):.2f}) | "
                      f"RPY=({np.degrees(euler[0]):+.1f},{np.degrees(euler[1]):+.1f},{np.degrees(euler[2]):+.1f})° | "
                      f"gyro=({np.degrees(gyro[0]):+.1f},{np.degrees(gyro[1]):+.1f},{np.degrees(gyro[2]):+.1f})°/s | "
                      f"thrust={motor_system.total_thrust*1000:.1f}mN")

                # Check for divergence
                if abs(euler[0]) > 0.5 or abs(euler[1]) > 0.5:
                    print("\n>>> DIVERGENCE DETECTED!")
                    print(f"    Force NED: {force_ned}")
                    print(f"    Moment NED: {moment_ned}")
                    print(f"    Force world: {force_world}")
                    print(f"    Moment genesis body: {moment_genesis_body}")

            time.sleep(max(0, sim_time - real_time))

    except KeyboardInterrupt:
        pass

    print("\nTest ended.")


if __name__ == "__main__":
    main()
