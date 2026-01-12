#!/usr/bin/env python3
"""
24_pid_rate_control.py - PID角速度制御
PID angular velocity (rate) control

角速度制御ループを実装：
  スティック → レートセットポイント → PID → トルク → モータ

Rate control loop:
  Stick input → Rate setpoint → PID → Torque → Motor

操作 (StampFly Controller USB HID):
  - スロットル軸 (Axis 0): 推力増減 ±ΔT (N)
  - ロール軸 (Axis 1): ロールレート指令 (rad/s)
  - ピッチ軸 (Axis 2): ピッチレート指令 (rad/s)
  - ヨー軸 (Axis 3): ヨーレート指令 (rad/s)
  - Modeボタン (Button 2) / Rキー: リセット
  - Optionボタン (Button 3) / Qキー: 終了
"""

import sys
from pathlib import Path

script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir.parent))
sys.path.insert(0, str(script_dir.parent.parent.parent))  # simulator/ for control module

import genesis as gs
import numpy as np
import time
import pygame

from motor_model import QuadMotorSystem
from control_allocation import ControlAllocator, thrusts_to_duties

# Import rate controller from simulator/control
from control.rate_controller import RateController, RateControlConfig


# Physical constants
GRAVITY = 9.81  # m/s^2
MASS = 0.035    # kg (35g)
WEIGHT = MASS * GRAVITY  # N


# Voxel color palette
COLORS = {
    'red': (0.8, 0.2, 0.2),
    'orange': (0.9, 0.5, 0.1),
    'yellow': (0.9, 0.8, 0.1),
    'green': (0.2, 0.7, 0.3),
    'cyan': (0.2, 0.7, 0.8),
    'blue': (0.2, 0.3, 0.8),
    'purple': (0.6, 0.2, 0.8),
    'white': (0.9, 0.9, 0.9),
    'gray': (0.5, 0.5, 0.5),
}


def add_block(scene, pos, size, color):
    """Add single block / 単一ブロックを追加"""
    scene.add_entity(
        gs.morphs.Box(
            size=(size, size, size),
            pos=pos,
            fixed=True,
            collision=False,
        ),
        surface=gs.surfaces.Default(color=color),
    )


def add_ring(scene, center, radius, block_size, color, axis='z'):
    """Create ring with blocks / ブロックで輪を作成"""
    cx, cy, cz = center
    num_blocks = int(2 * np.pi * radius / block_size)
    blocks = 0
    for i in range(num_blocks):
        angle = 2 * np.pi * i / num_blocks
        if axis == 'z':
            x, y, z = cx + radius * np.cos(angle), cy + radius * np.sin(angle), cz
        elif axis == 'x':
            x, y, z = cx, cy + radius * np.cos(angle), cz + radius * np.sin(angle)
        else:
            x, y, z = cx + radius * np.cos(angle), cy, cz + radius * np.sin(angle)
        add_block(scene, (x, y, z), block_size, color)
        blocks += 1
    return blocks


def add_pillar(scene, base_pos, height_blocks, block_size, color):
    """Create pillar / 柱を作成"""
    bx, by, bz = base_pos
    for i in range(height_blocks):
        z = bz + i * block_size + block_size / 2
        add_block(scene, (bx, by, z), block_size, color)
    return height_blocks


def add_arch(scene, center, width_blocks, height_blocks, block_size, color, direction='x'):
    """Create arch gate / アーチ門を作成"""
    cx, cy, cz = center
    blocks = 0
    half_w = width_blocks // 2
    for i in range(height_blocks):
        z = cz + i * block_size + block_size / 2
        if direction == 'x':
            add_block(scene, (cx, cy - half_w * block_size, z), block_size, color)
            add_block(scene, (cx, cy + half_w * block_size, z), block_size, color)
        else:
            add_block(scene, (cx - half_w * block_size, cy, z), block_size, color)
            add_block(scene, (cx + half_w * block_size, cy, z), block_size, color)
        blocks += 2
    top_z = cz + height_blocks * block_size + block_size / 2
    for i in range(-half_w, half_w + 1):
        if direction == 'x':
            add_block(scene, (cx, cy + i * block_size, top_z), block_size, color)
        else:
            add_block(scene, (cx + i * block_size, cy, top_z), block_size, color)
        blocks += 1
    return blocks


def add_grid_floor(scene, size, tile_size=1.0, line_width=0.02):
    """Create grid floor / グリッド線で床タイルを表現"""
    half_size = size / 2
    num_lines = int(size / tile_size) + 1
    z = 0.001
    for i in range(num_lines):
        y = -half_size + i * tile_size
        scene.add_entity(
            gs.morphs.Box(size=(size, line_width, line_width), pos=(0, y, z),
                          fixed=True, collision=False),
            surface=gs.surfaces.Default(color=(0.3, 0.3, 0.3)),
        )
    for i in range(num_lines):
        x = -half_size + i * tile_size
        scene.add_entity(
            gs.morphs.Box(size=(line_width, size, line_width), pos=(x, 0, z),
                          fixed=True, collision=False),
            surface=gs.surfaces.Default(color=(0.3, 0.3, 0.3)),
        )
    return (num_lines - 1) * 2


def add_voxel_structures(scene, block_size=0.5):
    """Add voxel structures (center 4m x 4m clear)"""
    total = 0

    rings = [
        ((12, 8, 5), 3.0, COLORS['red'], 'y'),
        ((20, 20, 4), 2.5, COLORS['orange'], 'x'),
        ((-20, 16, 6), 2.0, COLORS['yellow'], 'y'),
        ((30, -25, 5), 3.0, COLORS['green'], 'x'),
        ((-30, -20, 4), 2.5, COLORS['cyan'], 'y'),
        ((0, 40, 7), 2.0, COLORS['blue'], 'x'),
        ((-40, 0, 5), 3.0, COLORS['purple'], 'y'),
    ]
    for center, radius, color, axis in rings:
        total += add_ring(scene, center, radius, block_size, color, axis)

    pillars = [
        ((10, -10, 0), 16, COLORS['gray']),
        ((-10, -10, 0), 12, COLORS['white']),
        ((10, 10, 0), 10, COLORS['gray']),
        ((-10, 10, 0), 14, COLORS['white']),
        ((35, 0, 0), 20, COLORS['red']),
        ((-35, 0, 0), 18, COLORS['blue']),
        ((0, 35, 0), 16, COLORS['green']),
        ((0, -35, 0), 14, COLORS['yellow']),
        ((25, 25, 0), 8, COLORS['cyan']),
        ((-25, 25, 0), 10, COLORS['orange']),
        ((25, -25, 0), 12, COLORS['purple']),
        ((-25, -25, 0), 6, COLORS['white']),
    ]
    for base_pos, height, color in pillars:
        total += add_pillar(scene, base_pos, height, block_size, color)

    arches = [
        ((15, 0, 0), 6, 8, COLORS['orange'], 'x'),
        ((-15, 0, 0), 8, 10, COLORS['cyan'], 'x'),
        ((0, 15, 0), 6, 6, COLORS['green'], 'y'),
        ((0, -15, 0), 8, 8, COLORS['purple'], 'y'),
        ((40, 30, 0), 6, 12, COLORS['red'], 'x'),
        ((-40, -30, 0), 8, 10, COLORS['blue'], 'y'),
    ]
    for center, width, height, color, direction in arches:
        total += add_arch(scene, center, width, height, block_size, color, direction)

    return total


def quat_to_rotation_matrix(quat):
    """Quaternion (w, x, y, z) to rotation matrix"""
    w, x, y, z = [float(v) for v in quat]
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


def ned_to_genesis_force(force_ned):
    """NED body force to Genesis body force"""
    return np.array([force_ned[1], force_ned[0], -force_ned[2]])


def ned_to_genesis_moment(moment_ned):
    """NED body moment to Genesis body moment"""
    return np.array([moment_ned[1], moment_ned[0], -moment_ned[2]])


def world_ang_vel_to_body(ang_vel_world, R):
    """
    Transform angular velocity from world frame to body frame
    世界座標系の角速度を機体座標系に変換

    Args:
        ang_vel_world: Angular velocity in world frame [wx, wy, wz]
        R: Rotation matrix (body to world)

    Returns:
        Angular velocity in body frame
    """
    return R.T @ ang_vel_world


def genesis_gyro_to_ned(gyro_genesis_body):
    """
    Convert Genesis body frame angular velocity to NED body frame
    Genesis機体座標系の角速度をNED機体座標系に変換

    Genesis body: (wx, wy, wz) - X right, Y forward, Z up
    NED body: (p, q, r) - roll rate, pitch rate, yaw rate
    """
    return np.array([gyro_genesis_body[1], gyro_genesis_body[0], -gyro_genesis_body[2]])


def main():
    print("=" * 60)
    print("PID Rate Control - Angular Velocity Control")
    print("Stick -> Rate Setpoint -> PID -> Torque -> Motor")
    print("=" * 60)

    # Path setup
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly_fixed.urdf"

    if not urdf_file.exists():
        print(f"ERROR: URDF not found: {urdf_file}")
        return

    # Timing settings
    PHYSICS_HZ = 1000
    PHYSICS_DT = 1 / PHYSICS_HZ
    CONTROL_HZ = 400      # Control loop rate (matches firmware)
    CONTROL_DT = 1 / CONTROL_HZ
    RENDER_FPS = 60
    RENDER_DT = 1 / RENDER_FPS

    # World settings
    BLOCK_SIZE = 0.5
    WORLD_SIZE = 100.0
    TILE_SIZE = 1.0

    # Control settings
    DEADZONE = 0.05
    MAX_THRUST_DELTA = 0.2  # N

    # PID output to torque scaling
    # PID出力（電圧スケール）からトルク（Nm）への変換
    # PID output_limit = 3.7V (from config)
    # Scale to reasonable torque range based on previous testing
    OUTPUT_LIMIT = 3.7
    MAX_ROLL_TORQUE = 0.2e-3    # Nm (increased for PID response)
    MAX_PITCH_TORQUE = 0.2e-3   # Nm
    MAX_YAW_TORQUE = 0.05e-3    # Nm

    ROLL_TORQUE_SCALE = MAX_ROLL_TORQUE / OUTPUT_LIMIT
    PITCH_TORQUE_SCALE = MAX_PITCH_TORQUE / OUTPUT_LIMIT
    YAW_TORQUE_SCALE = MAX_YAW_TORQUE / OUTPUT_LIMIT

    print(f"\nWorld: {WORLD_SIZE}m x {WORLD_SIZE}m")
    print(f"Control rate: {CONTROL_HZ}Hz, Physics: {PHYSICS_HZ}Hz")

    # pygame initialization
    print("\n[1] Initializing controller...")
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("ERROR: No controller found!")
        pygame.quit()
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"  Controller: {joystick.get_name()}")

    # Control system initialization
    print("\n[2] Initializing control system...")
    allocator = ControlAllocator()
    motor_system = QuadMotorSystem()

    # Rate controller with firmware-matched gains
    rate_config = RateControlConfig(
        roll_rate_max=1.0,   # rad/s (57.3 deg/s)
        pitch_rate_max=1.0,  # rad/s
        yaw_rate_max=5.0,    # rad/s (286.5 deg/s)
        # Use firmware-matched gains
        roll_kp=0.65, roll_ti=0.7, roll_td=0.01,
        pitch_kp=0.95, pitch_ti=0.7, pitch_td=0.025,
        yaw_kp=3.0, yaw_ti=0.8, yaw_td=0.01,
        eta=0.125,
        output_limit=OUTPUT_LIMIT,
    )
    rate_controller = RateController(rate_config)

    print(f"  Rate PID gains:")
    print(f"    Roll:  Kp={rate_config.roll_kp}, Ti={rate_config.roll_ti}, Td={rate_config.roll_td}")
    print(f"    Pitch: Kp={rate_config.pitch_kp}, Ti={rate_config.pitch_ti}, Td={rate_config.pitch_td}")
    print(f"    Yaw:   Kp={rate_config.yaw_kp}, Ti={rate_config.yaw_ti}, Td={rate_config.yaw_td}")
    print(f"  Rate limits: Roll/Pitch={rate_config.roll_rate_max:.1f} rad/s, Yaw={rate_config.yaw_rate_max:.1f} rad/s")

    # Genesis initialization
    print("\n[3] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # Scene creation
    print("\n[4] Creating scene...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(10.0, -10.0, 8.0),
            camera_lookat=(0, 0, 3.0),
            camera_fov=60,
            max_FPS=RENDER_FPS,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -GRAVITY),
            dt=PHYSICS_DT,
        ),
    )

    # Ground + grid
    print("\n[5] Adding ground with grid...")
    scene.add_entity(
        gs.morphs.Plane(collision=True),
        surface=gs.surfaces.Default(color=(0.12, 0.12, 0.12)),
    )
    grid_lines = add_grid_floor(scene, WORLD_SIZE, TILE_SIZE)
    print(f"    Grid lines: {grid_lines}")

    # Voxel structures
    print("\n[6] Adding voxel structures...")
    total_blocks = add_voxel_structures(scene, BLOCK_SIZE)
    print(f"    Total blocks: {total_blocks}")

    # Load StampFly
    print("\n[7] Loading StampFly...")
    drone = scene.add_entity(
        gs.morphs.URDF(
            file=str(urdf_file),
            pos=(0, 0, 3.0),
            euler=(0, 0, 0),
            fixed=False,
            prioritize_urdf_material=True,
        ),
    )

    # Build scene
    print("\n[8] Building scene...")
    build_start = time.perf_counter()
    scene.build()
    print(f"    Build time: {time.perf_counter() - build_start:.1f}s")

    # Info display
    print("\n" + "=" * 60)
    print("PID Rate Control")
    print("=" * 60)
    print(f"  Throttle: Hover ± {MAX_THRUST_DELTA*1000:.0f}mN")
    print(f"  Roll/Pitch rate: ±{np.degrees(rate_config.roll_rate_max):.0f} deg/s")
    print(f"  Yaw rate: ±{np.degrees(rate_config.yaw_rate_max):.0f} deg/s")
    print(f"  Max torque: Roll/Pitch={MAX_ROLL_TORQUE*1e6:.0f}uNm, Yaw={MAX_YAW_TORQUE*1e6:.0f}uNm")
    print(f"  Mode/R: Reset, Option/Q: Exit")
    print("=" * 60)

    # Simulation loop
    print("\n[9] Running simulation...")

    physics_steps = 0
    control_steps = 0
    next_render_time = 0
    next_control_time = 0
    start_time = time.perf_counter()
    last_print_time = -1

    # Current torque commands (updated at control rate)
    current_torque = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw] Nm

    def apply_deadzone(value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - deadzone) / (1 - deadzone)

    try:
        while scene.viewer.is_alive():
            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        print("\n>>> Exit (Q key)")
                        raise KeyboardInterrupt
                    if event.key == pygame.K_r:
                        scene.reset()
                        motor_system.reset()
                        rate_controller.reset()
                        physics_steps = 0
                        control_steps = 0
                        next_render_time = 0
                        next_control_time = 0
                        start_time = time.perf_counter()
                        current_torque = np.array([0.0, 0.0, 0.0])
                        print("\n>>> Reset")
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 2:
                        scene.reset()
                        motor_system.reset()
                        rate_controller.reset()
                        physics_steps = 0
                        control_steps = 0
                        next_render_time = 0
                        next_control_time = 0
                        start_time = time.perf_counter()
                        current_torque = np.array([0.0, 0.0, 0.0])
                        print("\n>>> Reset")
                    if event.button == 3:
                        print("\n>>> Exit")
                        raise KeyboardInterrupt

            # Controller input
            throttle_raw = apply_deadzone(joystick.get_axis(0), DEADZONE)
            roll_raw = apply_deadzone(joystick.get_axis(1), DEADZONE)
            pitch_raw = apply_deadzone(joystick.get_axis(2), DEADZONE)
            yaw_raw = apply_deadzone(joystick.get_axis(3), DEADZONE)

            # Thrust command (direct)
            u_thrust = WEIGHT + throttle_raw * MAX_THRUST_DELTA

            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            # Control loop (at CONTROL_HZ)
            if sim_time >= next_control_time:
                # Get angular velocity from DOFs (body frame directly)
                # Genesis DOF convention:
                #   get_dofs_velocity()[3:6] = body frame angular velocity (wx, wy, wz)
                #   get_ang() = world frame angular velocity
                dofs_vel = drone.get_dofs_velocity()
                gyro_genesis_body = np.array([float(dofs_vel[3]), float(dofs_vel[4]), float(dofs_vel[5])])

                # Convert Genesis body frame (wx, wy, wz) to NED body frame (p, q, r)
                # See docs/architecture/genesis-integration.md:
                #   (wx, wy, wz)_genesis → (wy, wx, -wz) = (p, q, r)_ned
                gyro_ned = genesis_gyro_to_ned(gyro_genesis_body)

                # PID rate control: stick -> rate setpoint -> PID -> voltage output
                roll_out, pitch_out, yaw_out = rate_controller.update_from_stick(
                    stick_roll=roll_raw,
                    stick_pitch=pitch_raw,
                    stick_yaw=yaw_raw,
                    gyro=gyro_ned,
                    dt=CONTROL_DT,
                )

                # Scale voltage output to torque (Nm)
                current_torque[0] = roll_out * ROLL_TORQUE_SCALE
                current_torque[1] = pitch_out * PITCH_TORQUE_SCALE
                current_torque[2] = yaw_out * YAW_TORQUE_SCALE

                control_steps += 1
                next_control_time = control_steps * CONTROL_DT

            # Control allocation (thrust + torque -> motor thrusts)
            control = np.array([u_thrust, current_torque[0], current_torque[1], current_torque[2]])
            target_thrusts = allocator.mix(control)
            target_duties = thrusts_to_duties(target_thrusts)

            # Physics loop
            while sim_time <= real_time:
                force_ned, moment_ned = motor_system.step_with_duty(
                    target_duties.tolist(), PHYSICS_DT
                )

                # Convert NED body frame to Genesis body frame
                # See docs/architecture/genesis-integration.md:
                #   (fx, fy, fz)_ned → (fy, fx, -fz)_genesis
                #   (τx, τy, τz)_ned → (τy, τx, -τz)_genesis
                force_genesis_body = ned_to_genesis_force(force_ned)
                moment_genesis_body = ned_to_genesis_moment(moment_ned)

                # Genesis DOF force convention:
                #   dof_force[0:3] = WORLD frame force
                #   dof_force[3:6] = BODY frame torque
                quat = drone.get_quat()
                R = quat_to_rotation_matrix(quat)

                force_world = R @ force_genesis_body  # Body → World (for force)
                # Torque stays in body frame (no transformation needed)

                dof_force = np.zeros(drone.n_dofs)
                dof_force[0:3] = force_world           # World frame force
                dof_force[3:6] = moment_genesis_body   # Body frame torque
                drone.control_dofs_force(dof_force)

                scene.step(update_visualizer=False, refresh_visualizer=False)
                physics_steps += 1
                sim_time = physics_steps * PHYSICS_DT

            # Rendering
            if real_time >= next_render_time:
                scene.visualizer.update()
                next_render_time += RENDER_DT

            # Wait
            sleep_time = sim_time - real_time
            if sleep_time > 0:
                time.sleep(sleep_time)

            # Status display (every second)
            current_second = int(sim_time)
            if current_second > last_print_time:
                last_print_time = current_second

                pos = drone.get_pos()
                quat = drone.get_quat()
                w, gx, gy, gz = [float(v) for v in quat]
                genesis_rx = np.degrees(np.arctan2(2*(w*gx + gy*gz), 1 - 2*(gx*gx + gy*gy)))
                genesis_ry = np.degrees(np.arcsin(np.clip(2*(w*gy - gz*gx), -1, 1)))
                genesis_rz = np.degrees(np.arctan2(2*(w*gz + gx*gy), 1 - 2*(gy*gy + gz*gz)))

                # Get gyro for display using get_dofs_velocity() (body frame directly)
                dofs_vel_disp = drone.get_dofs_velocity()
                gyro_genesis_body = np.array([float(dofs_vel_disp[3]), float(dofs_vel_disp[4]), float(dofs_vel_disp[5])])
                gyro_ned = genesis_gyro_to_ned(gyro_genesis_body)

                print(f"  t={sim_time:.0f}s | "
                      f"pos=({float(pos[0]):+.1f},{float(pos[1]):+.1f},{float(pos[2]):.1f}) | "
                      f"RPY=({genesis_ry:+.0f},{genesis_rx:+.0f},{-genesis_rz:+.0f}) | "
                      f"gyro=({np.degrees(gyro_ned[0]):+.0f},{np.degrees(gyro_ned[1]):+.0f},{np.degrees(gyro_ned[2]):+.0f})deg/s")

    except KeyboardInterrupt:
        pass

    print("\nSimulation ended.")
    pygame.quit()


if __name__ == "__main__":
    main()
