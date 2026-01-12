#!/usr/bin/env python3
"""
debug_torque_response.py - Test Genesis torque-to-gyro response
Genesis トルク→ジャイロ応答テスト

Apply torque directly and observe angular velocity response.
直接トルクを適用して角速度の応答を観察。

This test helps verify:
1. Which DOF indices correspond to which axes
2. Sign conventions for torque application
3. Body vs world frame interpretation
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


def quat_to_rotation_matrix(quat):
    """Quaternion (w, x, y, z) to rotation matrix"""
    w, x, y, z = [float(v) for v in quat]
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


def main():
    print("=" * 70)
    print("Debug: Genesis Torque-to-Gyro Response Test")
    print("=" * 70)
    print()
    print("Test sequence:")
    print("  1. Apply hover thrust only (0-1s) - should be stable")
    print("  2. Apply +τx (dof[3]) torque (1-1.5s) - observe which gyro changes")
    print("  3. Apply +τy (dof[4]) torque (2-2.5s) - observe which gyro changes")
    print("  4. Apply +τz (dof[5]) torque (3-3.5s) - observe which gyro changes")
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

    print("\nStarting test...")
    print("Genesis coordinate system: X=Right, Y=Forward, Z=Up")
    print("Expected in Genesis body frame:")
    print("  +τx (torque around X/Right) → +wx (pitch rate increases)")
    print("  +τy (torque around Y/Forward) → +wy (roll rate increases)")
    print("  +τz (torque around Z/Up) → +wz (yaw rate increases)")
    print()

    physics_steps = 0
    start_time = time.perf_counter()

    test_torque_magnitude = 0.001  # 1 mNm

    # Test phases: (start_time, end_time, torque_index, torque_name)
    test_phases = [
        (0.0, 1.0, None, "HOVER ONLY"),
        (1.0, 1.5, 3, "+τx (dof[3])"),
        (1.5, 2.0, None, "FREE"),
        (2.0, 2.5, 4, "+τy (dof[4])"),
        (2.5, 3.0, None, "FREE"),
        (3.0, 3.5, 5, "+τz (dof[5])"),
        (3.5, 5.0, None, "FREE"),
    ]

    last_print_step = -50  # Print every 50ms

    try:
        while scene.viewer.is_alive():
            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            while sim_time <= real_time and sim_time < 5.0:
                # Determine current phase
                current_phase = "UNKNOWN"
                torque_idx = None
                for (t_start, t_end, idx, name) in test_phases:
                    if t_start <= sim_time < t_end:
                        current_phase = name
                        torque_idx = idx
                        break

                # Apply hover thrust (Genesis body frame +Z = up)
                quat = drone.get_quat()
                R = quat_to_rotation_matrix(quat)

                # Genesis body force: [0, 0, +thrust] (pointing up in body frame)
                force_body = np.array([0.0, 0.0, WEIGHT])
                force_world = R @ force_body

                dof_force = np.zeros(drone.n_dofs)
                dof_force[0:3] = force_world

                # Apply test torque if in a torque phase
                if torque_idx is not None:
                    dof_force[torque_idx] = test_torque_magnitude

                drone.control_dofs_force(dof_force)

                scene.step(update_visualizer=False, refresh_visualizer=False)
                physics_steps += 1
                sim_time = physics_steps * PHYSICS_DT

                # Print status
                if physics_steps - last_print_step >= 50:  # Every 50ms
                    last_print_step = physics_steps

                    pos = drone.get_pos()
                    dofs_vel = drone.get_dofs_velocity()
                    gyro_raw = np.array([float(dofs_vel[3]), float(dofs_vel[4]), float(dofs_vel[5])])

                    # Get world angular velocity for comparison
                    ang_world = drone.get_ang()

                    print(f"[{current_phase:15s}] t={sim_time:.2f}s | z={float(pos[2]):.2f}m | "
                          f"gyro_body(wx,wy,wz)=({np.degrees(gyro_raw[0]):+6.1f},{np.degrees(gyro_raw[1]):+6.1f},{np.degrees(gyro_raw[2]):+6.1f})°/s | "
                          f"ang_world=({np.degrees(float(ang_world[0])):+6.1f},{np.degrees(float(ang_world[1])):+6.1f},{np.degrees(float(ang_world[2])):+6.1f})°/s")

            scene.visualizer.update()

            if sim_time >= 5.0:
                break

            time.sleep(max(0, sim_time - real_time))

    except KeyboardInterrupt:
        pass

    print()
    print("=" * 70)
    print("Analysis:")
    print("=" * 70)
    print("""
Expected if dof_force[3:6] is BODY frame torque:
  - +τx (dof[3]) → wx increases → pitch rate increases
  - +τy (dof[4]) → wy increases → roll rate increases
  - +τz (dof[5]) → wz increases → yaw rate increases

If the gyro responds OPPOSITE to the applied torque index,
then there's a sign or frame convention issue.

For NED control:
  - NED +τx (roll) should map to Genesis +τy (dof[4])
  - NED +τy (pitch) should map to Genesis +τx (dof[3])
  - NED +τz (yaw) should map to Genesis -τz (dof[5])
""")


if __name__ == "__main__":
    main()
