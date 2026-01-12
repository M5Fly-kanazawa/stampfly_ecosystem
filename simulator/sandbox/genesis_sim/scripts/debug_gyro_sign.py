#!/usr/bin/env python3
"""
debug_gyro_sign.py - Test gyro sign convention
角速度の符号を確認するテスト

Apply a known torque and check if gyro reading matches expected direction.
既知のトルクを加えて、角速度の読みが期待通りの方向か確認。
"""

import sys
from pathlib import Path

script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir.parent))

import genesis as gs
import numpy as np
import time

GRAVITY = 9.81
MASS = 0.035
WEIGHT = MASS * GRAVITY

PHYSICS_HZ = 1000
PHYSICS_DT = 1 / PHYSICS_HZ


def quat_to_euler(quat):
    w, x, y, z = [float(v) for v in quat]
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


def quat_to_rotation_matrix(quat):
    w, x, y, z = [float(v) for v in quat]
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


def genesis_gyro_to_ned(gyro_genesis_body):
    """Current conversion - to be verified"""
    return np.array([gyro_genesis_body[1], gyro_genesis_body[0], -gyro_genesis_body[2]])


def main():
    print("=" * 60)
    print("Debug: Gyro Sign Convention Test")
    print("=" * 60)

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

    print("\n" + "=" * 60)
    print("Test: Apply +roll torque (NED: +τx, expect +p)")
    print("NED convention: +roll = right wing down")
    print("=" * 60)
    print("\nApplying torque for 0.5s, then observing...")

    # Apply positive roll torque in NED
    # NED: τx = roll torque (positive = right wing down)
    # Genesis body: τy = roll torque (since Genesis Y = NED X)

    test_torque_ned = 0.001  # 1 mNm

    physics_steps = 0
    start_time = time.perf_counter()
    torque_duration = 0.5  # Apply torque for 0.5s

    print(f"\nApplying NED roll torque: +{test_torque_ned*1000:.1f} mNm")
    print("Expected: positive roll rate (p > 0), positive roll angle (φ > 0)")
    print()

    last_print_step = -1

    try:
        while scene.viewer.is_alive():
            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            while sim_time <= real_time and sim_time < 3.0:
                # Hover thrust in NED: -Z is UP
                thrust_ned = np.array([0, 0, -WEIGHT])  # NED: -Z = up

                # Apply roll torque for first 0.5s
                if sim_time < torque_duration:
                    torque_ned = np.array([test_torque_ned, 0, 0])  # +roll torque
                else:
                    torque_ned = np.array([0, 0, 0])

                # NED to Genesis body frame conversion
                # Force: (fx, fy, fz)_ned -> (fy, fx, -fz)_genesis
                # Torque: (τx, τy, τz)_ned -> (τy, τx, -τz)_genesis
                force_genesis_body = np.array([thrust_ned[1], thrust_ned[0], -thrust_ned[2]])
                torque_genesis_body = np.array([torque_ned[1], torque_ned[0], -torque_ned[2]])

                # Force: body to world
                quat = drone.get_quat()
                R = quat_to_rotation_matrix(quat)
                force_world = R @ force_genesis_body

                # Apply
                dof_force = np.zeros(drone.n_dofs)
                dof_force[0:3] = force_world
                dof_force[3:6] = torque_genesis_body
                drone.control_dofs_force(dof_force)

                scene.step(update_visualizer=False, refresh_visualizer=False)
                physics_steps += 1
                sim_time = physics_steps * PHYSICS_DT

                # Print every 50ms during TORQUE phase, every 500ms during FREE phase
                print_interval = 50 if sim_time < torque_duration else 500
                if physics_steps - last_print_step >= print_interval:
                    last_print_step = physics_steps
                    pos = drone.get_pos()
                    quat_now = drone.get_quat()
                    euler = quat_to_euler(quat_now)
                    dofs_vel = drone.get_dofs_velocity()
                    gyro_raw = np.array([float(dofs_vel[3]), float(dofs_vel[4]), float(dofs_vel[5])])
                    gyro_ned = genesis_gyro_to_ned(gyro_raw)
                    phase = "TORQUE" if sim_time < torque_duration else "FREE"
                    print(f"[{phase}] t={sim_time:.2f}s z={float(pos[2]):.2f} | "
                          f"φ={np.degrees(euler[0]):+.1f}° | "
                          f"gyro_raw=({np.degrees(gyro_raw[0]):+.0f},{np.degrees(gyro_raw[1]):+.0f},{np.degrees(gyro_raw[2]):+.0f}) | "
                          f"p(ned)={np.degrees(gyro_ned[0]):+.0f}°/s")

            scene.visualizer.update()

            # Print at key moments (backup, less frequent)
            if physics_steps % 500 == 0 and physics_steps != last_print_step:
                pos = drone.get_pos()
                quat = drone.get_quat()
                euler = quat_to_euler(quat)

                # Raw gyro from Genesis
                dofs_vel = drone.get_dofs_velocity()
                gyro_raw = np.array([float(dofs_vel[3]), float(dofs_vel[4]), float(dofs_vel[5])])
                gyro_ned = genesis_gyro_to_ned(gyro_raw)

                phase = "TORQUE" if sim_time < torque_duration else "FREE"
                print(f"[{phase}] t={sim_time:.2f}s | "
                      f"φ(roll)={np.degrees(euler[0]):+.1f}° | "
                      f"gyro_raw=({np.degrees(gyro_raw[0]):+.1f},{np.degrees(gyro_raw[1]):+.1f},{np.degrees(gyro_raw[2]):+.1f}) | "
                      f"gyro_ned(p,q,r)=({np.degrees(gyro_ned[0]):+.1f},{np.degrees(gyro_ned[1]):+.1f},{np.degrees(gyro_ned[2]):+.1f})°/s")

            if sim_time >= 3.0:
                break

            time.sleep(max(0, sim_time - real_time))

    except KeyboardInterrupt:
        pass

    print("\n" + "=" * 60)
    print("Analysis:")
    print("  - Applied +roll torque (NED τx > 0)")
    print("  - Expected: φ > 0 (roll right), p > 0 (positive roll rate)")
    print("  - If gyro_ned[0] (p) was NEGATIVE while φ increased,")
    print("    then the sign conversion is WRONG")
    print("=" * 60)


if __name__ == "__main__":
    main()
