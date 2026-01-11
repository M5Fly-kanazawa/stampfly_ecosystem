#!/usr/bin/env python3
"""
19_controller_torque_test.py - コントローラでトルク制御テスト
Controller-based torque control test for StampFly

操作 (StampFly Controller USB HID):
  - 右スティック X (Axis 1): Roll (X軸トルク)
  - 右スティック Y (Axis 2): Pitch (Y軸トルク)
  - 左スティック X (Axis 3): Yaw (Z軸トルク)
  - Modeボタン (Button 2, 右サイド): リセット
  - Optionボタン (Button 3, 左サイド): 終了

USB HID Axis構成:
  - Axis 0: Throttle (左Y)
  - Axis 1: Roll/Aileron (右X)
  - Axis 2: Pitch/Elevator (右Y)
  - Axis 3: Yaw/Rudder (左X)

USB HID Button構成:
  - Button 0: Arm (左スティック押込)
  - Button 1: Flip (右スティック押込)
  - Button 2: Mode (右サイドボタン)
  - Button 3: Option (左サイドボタン)

DOF構成 (stampfly_fixed.urdf):
  - DOF 0, 1, 2: 並進 (x, y, z)
  - DOF 3, 4, 5: 回転 (rx, ry, rz)
"""

import genesis as gs
from pathlib import Path
import numpy as np
import time
import pygame


def main():
    print("=" * 60)
    print("Controller Torque Test - StampFly")
    print("=" * 60)

    # パス設定
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly_fixed.urdf"

    if not urdf_file.exists():
        print(f"ERROR: URDF not found: {urdf_file}")
        return

    # タイミング設定
    PHYSICS_HZ = 240
    PHYSICS_DT = 1 / PHYSICS_HZ
    RENDER_FPS = 60
    RENDER_DT = 1 / RENDER_FPS

    # トルク設定
    MAX_TORQUE = 2e-4  # 最大トルク (Nm)
    DEADZONE = 0.1     # スティックのデッドゾーン

    # pygame初期化（コントローラ用）
    print("\n[1] Initializing controller...")
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("ERROR: No controller found!")
        print("Please connect a controller and try again.")
        pygame.quit()
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"  Controller: {joystick.get_name()}")
    print(f"  Axes: {joystick.get_numaxes()}")
    print(f"  Buttons: {joystick.get_numbuttons()}")

    # Genesis初期化
    print("\n[2] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成（無重力）
    print("\n[3] Creating scene (zero gravity)...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0.5, -0.5, 0.3),
            camera_lookat=(0, 0, 0.15),
            camera_fov=60,
            max_FPS=RENDER_FPS,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, 0),
            dt=PHYSICS_DT,
        ),
    )

    # 地面（視覚参照のみ）
    print("\n[4] Adding ground plane...")
    scene.add_entity(
        gs.morphs.Plane(collision=False),
        surface=gs.surfaces.Default(color=(0.3, 0.3, 0.3)),
    )

    # 座標軸マーカー
    print("\n[5] Adding axis markers...")
    axis_len, axis_thick = 0.1, 0.002
    scene.add_entity(
        gs.morphs.Box(size=(axis_len, axis_thick, axis_thick),
                      pos=(axis_len/2, 0, 0.001), fixed=True, collision=False),
        surface=gs.surfaces.Default(color=(1, 0.2, 0.2)),
    )
    scene.add_entity(
        gs.morphs.Box(size=(axis_thick, axis_len, axis_thick),
                      pos=(0, axis_len/2, 0.001), fixed=True, collision=False),
        surface=gs.surfaces.Default(color=(0.2, 1, 0.2)),
    )
    scene.add_entity(
        gs.morphs.Box(size=(axis_thick, axis_thick, axis_len),
                      pos=(0, 0, axis_len/2), fixed=True, collision=False),
        surface=gs.surfaces.Default(color=(0.2, 0.5, 1)),
    )

    # StampFly読み込み
    print("\n[6] Loading StampFly URDF...")
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
    print("\n[7] Building scene...")
    build_start = time.perf_counter()
    scene.build()
    build_elapsed = time.perf_counter() - build_start
    print(f"    Build time: {build_elapsed:.1f}s")

    # 情報表示
    print("\n" + "=" * 60)
    print("Controls (StampFly Controller USB HID)")
    print("=" * 60)
    print("  Right Stick X (Axis 1): Roll  (X-axis torque)")
    print("  Right Stick Y (Axis 2): Pitch (Y-axis torque)")
    print("  Left Stick X  (Axis 3): Yaw   (Z-axis torque)")
    print("  Mode button (Button 2): Reset")
    print("  Option button (Button 3): Exit")
    print()
    print(f"  Max torque: {MAX_TORQUE} Nm")
    print(f"  Deadzone: {DEADZONE}")
    print("=" * 60)

    # シミュレーション実行
    print("\n[8] Running simulation...")

    physics_steps = 0
    next_render_time = 0
    start_time = time.perf_counter()
    last_print_time = -1

    def apply_deadzone(value, deadzone):
        """デッドゾーン適用"""
        if abs(value) < deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - deadzone) / (1 - deadzone)

    try:
        while scene.viewer.is_alive():
            # pygameイベント処理
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    # Mode button (Button 2, 右サイドボタン): リセット
                    if event.button == 2:
                        scene.reset()
                        physics_steps = 0
                        next_render_time = 0
                        start_time = time.perf_counter()
                        print("\n>>> Reset")
                    # Option button (Button 3, 左サイドボタン): 終了
                    if event.button == 3:
                        print("\n>>> Exit")
                        raise KeyboardInterrupt

            # コントローラ入力取得 (USB HID axis mapping)
            # Axis 0: Throttle (左Y) - 未使用
            # Axis 1: Roll/Aileron (右X)
            # Axis 2: Pitch/Elevator (右Y)
            # Axis 3: Yaw/Rudder (左X)
            roll_input = apply_deadzone(joystick.get_axis(1), DEADZONE)   # 右スティックX
            pitch_input = apply_deadzone(joystick.get_axis(2), DEADZONE)  # 右スティックY
            yaw_input = apply_deadzone(joystick.get_axis(3), DEADZONE)    # 左スティックX

            # トルク計算
            torque_x = roll_input * MAX_TORQUE    # Roll (X軸トルク)
            torque_y = -pitch_input * MAX_TORQUE  # Pitch (Y軸トルク, 反転)
            torque_z = yaw_input * MAX_TORQUE     # Yaw (Z軸トルク)

            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            # 物理ステップ
            while sim_time <= real_time:
                force = np.zeros(drone.n_dofs)
                force[3] = torque_x  # Roll
                force[4] = torque_y  # Pitch
                force[5] = torque_z  # Yaw
                drone.control_dofs_force(force)

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
                dofs_vel = drone.get_dofs_velocity()
                ang_vel = dofs_vel[3:6]

                # クォータニオン → オイラー角
                w, x, y, z = [float(v) for v in quat]
                roll = np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
                pitch = np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1, 1)))
                yaw = np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))

                print(f"  t={sim_time:.0f}s | "
                      f"stick=({roll_input:+.2f},{pitch_input:+.2f},{yaw_input:+.2f}) | "
                      f"angle=({roll:+.0f},{pitch:+.0f},{yaw:+.0f})deg | "
                      f"omega=({float(ang_vel[0]):+.1f},{float(ang_vel[1]):+.1f},{float(ang_vel[2]):+.1f})")

    except KeyboardInterrupt:
        pass

    print("\nSimulation ended.")
    pygame.quit()


if __name__ == "__main__":
    main()
