#!/usr/bin/env python3
"""
20_controller_motor_test.py - コントローラでモータモデルテスト
Controller-based motor model test for StampFly

物理単位ベースの制御アロケーション:
  スティック入力 → 制御入力 [uₜ, u_φ, u_θ, u_ψ] (N, Nm)
  ミキサー行列 → モータ推力 [T₁, T₂, T₃, T₄] (N)
  逆モータモデル → Duty (0~1)
  モータダイナミクス → 実推力 (N)

操作 (StampFly Controller USB HID):
  - スロットル軸 (Axis 0): 推力増減 ±ΔT (N)
  - ロール軸 (Axis 1): ロールトルク u_φ (Nm)
  - ピッチ軸 (Axis 2): ピッチトルク u_θ (Nm)
  - ヨー軸 (Axis 3): ヨートルク u_ψ (Nm)
  - Modeボタン (Button 2): リセット
  - Optionボタン (Button 3): 終了

スロットル動作:
  - リセット後はホバリング推力がデフォルト
  - スティック中立: ホバリング維持
  - スティック上: 上昇 (推力増加)
  - スティック下: 下降 (推力減少)
"""

import sys
from pathlib import Path

# Add parent directory to path for imports
script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir.parent))

import genesis as gs
import numpy as np
import time
import pygame

from motor_model import QuadMotorSystem, MotorParams
from control_allocation import ControlAllocator, thrusts_to_duties


# Physical constants
GRAVITY = 9.81  # m/s²
MASS = 0.035    # kg (35g)
WEIGHT = MASS * GRAVITY  # N


def quat_to_rotation_matrix(quat):
    """
    クォータニオン (w, x, y, z) から回転行列を計算
    """
    w, x, y, z = [float(v) for v in quat]
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])
    return R


def ned_to_genesis_force(force_ned):
    """NED機体座標系の力をGenesis機体座標系に変換"""
    return np.array([force_ned[1], force_ned[0], -force_ned[2]])


def ned_to_genesis_moment(moment_ned):
    """NED機体座標系のモーメントをGenesis機体座標系に変換"""
    return np.array([moment_ned[1], moment_ned[0], -moment_ned[2]])


def main():
    print("=" * 60)
    print("Controller Motor Model Test - StampFly")
    print("Physical Units Based Control Allocation")
    print("=" * 60)

    # パス設定
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly_fixed.urdf"

    if not urdf_file.exists():
        print(f"ERROR: URDF not found: {urdf_file}")
        return

    # タイミング設定
    PHYSICS_HZ = 1000
    PHYSICS_DT = 1 / PHYSICS_HZ
    RENDER_FPS = 60
    RENDER_DT = 1 / RENDER_FPS

    # 制御設定（物理単位）
    DEADZONE = 0.05
    MAX_THRUST_DELTA = 0.2      # 最大推力変化 (N) ±0.2N
    MAX_ROLL_TORQUE = 2e-3      # 最大ロールトルク (Nm) ±2mNm
    MAX_PITCH_TORQUE = 2e-3     # 最大ピッチトルク (Nm) ±2mNm
    MAX_YAW_TORQUE = 0.5e-3     # 最大ヨートルク (Nm) ±0.5mNm

    print(f"\nPhysical Parameters:")
    print(f"  Mass: {MASS*1000:.0f}g")
    print(f"  Weight: {WEIGHT*1000:.1f}mN")
    print(f"  Hover thrust/motor: {WEIGHT/4*1000:.1f}mN")

    print(f"\nControl Ranges:")
    print(f"  Thrust: {WEIGHT*1000:.0f} ± {MAX_THRUST_DELTA*1000:.0f} mN")
    print(f"  Roll torque: ±{MAX_ROLL_TORQUE*1000:.1f} mNm")
    print(f"  Pitch torque: ±{MAX_PITCH_TORQUE*1000:.1f} mNm")
    print(f"  Yaw torque: ±{MAX_YAW_TORQUE*1000:.2f} mNm")

    # pygame初期化
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

    # コントロールアロケータ初期化
    print("\n[2] Initializing control allocator...")
    allocator = ControlAllocator()

    # モータモデル初期化
    print("\n[3] Initializing motor model...")
    motor_system = QuadMotorSystem()

    # Genesis初期化
    print("\n[4] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成
    print("\n[5] Creating scene...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0.5, -0.5, 0.3),
            camera_lookat=(0, 0, 0.15),
            camera_fov=60,
            max_FPS=RENDER_FPS,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -GRAVITY),
            dt=PHYSICS_DT,
        ),
    )

    # 地面
    scene.add_entity(
        gs.morphs.Plane(collision=True),
        surface=gs.surfaces.Default(color=(0.3, 0.3, 0.3)),
    )

    # 座標軸マーカー
    print("\n[6] Adding axis markers...")
    axis_origin = (-0.4, -0.4, 0.001)
    axis_len, axis_thick = 0.15, 0.003
    scene.add_entity(
        gs.morphs.Box(size=(axis_len, axis_thick, axis_thick),
                      pos=(axis_origin[0] + axis_len/2, axis_origin[1], axis_origin[2]),
                      fixed=True, collision=False),
        surface=gs.surfaces.Default(color=(1, 0.2, 0.2)),
    )
    scene.add_entity(
        gs.morphs.Box(size=(axis_thick, axis_len, axis_thick),
                      pos=(axis_origin[0], axis_origin[1] + axis_len/2, axis_origin[2]),
                      fixed=True, collision=False),
        surface=gs.surfaces.Default(color=(0.2, 1, 0.2)),
    )
    scene.add_entity(
        gs.morphs.Box(size=(axis_thick, axis_thick, axis_len),
                      pos=(axis_origin[0], axis_origin[1], axis_origin[2] + axis_len/2),
                      fixed=True, collision=False),
        surface=gs.surfaces.Default(color=(0.2, 0.5, 1)),
    )

    # StampFly読み込み
    print("\n[7] Loading StampFly URDF...")
    drone = scene.add_entity(
        gs.morphs.URDF(
            file=str(urdf_file),
            pos=(0, 0, 0.15),
            euler=(0, 0, 0),
            fixed=False,
            prioritize_urdf_material=True,
        ),
    )

    # シーンビルド
    print("\n[8] Building scene...")
    build_start = time.perf_counter()
    scene.build()
    print(f"    Build time: {time.perf_counter() - build_start:.1f}s")

    # 情報表示
    print("\n" + "=" * 60)
    print("Controls (Physical Units)")
    print("=" * 60)
    print(f"  Throttle: Hover({WEIGHT*1000:.0f}mN) ± {MAX_THRUST_DELTA*1000:.0f}mN")
    print(f"  Roll:     ±{MAX_ROLL_TORQUE*1000:.1f} mNm")
    print(f"  Pitch:    ±{MAX_PITCH_TORQUE*1000:.1f} mNm")
    print(f"  Yaw:      ±{MAX_YAW_TORQUE*1000:.2f} mNm")
    print(f"  Mode: Reset, Option: Exit")
    print("=" * 60)

    # シミュレーション実行
    print("\n[9] Running simulation...")

    physics_steps = 0
    next_render_time = 0
    start_time = time.perf_counter()
    last_print_time = -1

    def apply_deadzone(value, deadzone):
        if abs(value) < deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - deadzone) / (1 - deadzone)

    try:
        while scene.viewer.is_alive():
            # pygameイベント処理
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 2:  # Mode: Reset
                        scene.reset()
                        motor_system.reset()
                        physics_steps = 0
                        next_render_time = 0
                        start_time = time.perf_counter()
                        print("\n>>> Reset")
                    if event.button == 3:  # Option: Exit
                        print("\n>>> Exit")
                        raise KeyboardInterrupt

            # コントローラ入力取得
            throttle_raw = apply_deadzone(joystick.get_axis(0), DEADZONE)
            roll_raw = apply_deadzone(joystick.get_axis(1), DEADZONE)
            pitch_raw = apply_deadzone(joystick.get_axis(2), DEADZONE)
            yaw_raw = apply_deadzone(joystick.get_axis(3), DEADZONE)

            # 物理単位に変換
            # 制御入力 [uₜ, u_φ, u_θ, u_ψ] (N, Nm, Nm, Nm)
            u_thrust = WEIGHT + throttle_raw * MAX_THRUST_DELTA  # 総推力 (N)
            u_roll = roll_raw * MAX_ROLL_TORQUE                   # ロールトルク (Nm)
            u_pitch = pitch_raw * MAX_PITCH_TORQUE                # ピッチトルク (Nm)
            u_yaw = yaw_raw * MAX_YAW_TORQUE                      # ヨートルク (Nm)

            control = np.array([u_thrust, u_roll, u_pitch, u_yaw])

            # ミキサー: 制御入力 → モータ推力 (N)
            target_thrusts = allocator.mix(control)

            # 推力 → Duty変換（定常状態近似）
            target_duties = thrusts_to_duties(target_thrusts)

            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            # 物理ステップ
            while sim_time <= real_time:
                # モータモデル更新（Duty入力、NED機体座標系で出力）
                force_ned, moment_ned = motor_system.step_with_duty(
                    target_duties.tolist(), PHYSICS_DT
                )

                # NED → Genesis機体座標系に変換
                force_genesis_body = ned_to_genesis_force(force_ned)
                moment_genesis_body = ned_to_genesis_moment(moment_ned)

                # 現在の姿勢を取得
                quat = drone.get_quat()
                R = quat_to_rotation_matrix(quat)

                # 機体座標系 → 世界座標系に変換
                force_world = R @ force_genesis_body
                moment_world = R @ moment_genesis_body

                # DOF力として適用
                dof_force = np.zeros(drone.n_dofs)
                dof_force[0:3] = force_world
                dof_force[3:6] = moment_world
                drone.control_dofs_force(dof_force)

                scene.step(update_visualizer=False, refresh_visualizer=False)
                physics_steps += 1
                sim_time = physics_steps * PHYSICS_DT

            # レンダリング
            if real_time >= next_render_time:
                scene.visualizer.update()
                next_render_time += RENDER_DT

            # 待機
            sleep_time = sim_time - real_time
            if sleep_time > 0:
                time.sleep(sleep_time)

            # 1秒ごとに状態表示
            current_second = int(sim_time)
            if current_second > last_print_time:
                last_print_time = current_second

                # 姿勢取得
                quat = drone.get_quat()
                w, gx, gy, gz = [float(v) for v in quat]
                genesis_rx = np.degrees(np.arctan2(2*(w*gx + gy*gz), 1 - 2*(gx*gx + gy*gy)))
                genesis_ry = np.degrees(np.arcsin(np.clip(2*(w*gy - gz*gx), -1, 1)))
                genesis_rz = np.degrees(np.arctan2(2*(w*gz + gx*gy), 1 - 2*(gy*gy + gz*gz)))
                ned_roll, ned_pitch, ned_yaw = genesis_ry, genesis_rx, -genesis_rz

                # 位置取得
                pos = drone.get_pos()
                alt = float(pos[2])

                # モータ状態
                actual_thrust = motor_system.total_thrust * 1000  # mN

                print(f"  t={sim_time:.0f}s | "
                      f"cmd=({u_thrust*1000:.0f},{u_roll*1e6:.0f},{u_pitch*1e6:.0f},{u_yaw*1e6:.0f}) | "
                      f"T={actual_thrust:.0f}mN | "
                      f"alt={alt:.2f}m | "
                      f"RPY=({ned_roll:+.0f},{ned_pitch:+.0f},{ned_yaw:+.0f})")

    except KeyboardInterrupt:
        pass

    print("\nSimulation ended.")
    pygame.quit()


if __name__ == "__main__":
    main()
