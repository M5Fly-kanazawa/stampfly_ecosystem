#!/usr/bin/env python3
"""
20_controller_motor_test.py - コントローラでモータモデルテスト
Controller-based motor model test for StampFly

モータ・プロペラダイナミクスをシミュレート:
  - DCモータ電気系 (R, L, Km)
  - プロペラ空力 (Ct, Cq)
  - RK4積分による角速度計算

操作 (StampFly Controller USB HID):
  - スロットル軸 (Axis 0): ホバリングDutyからの増減 (±調整)
  - ロール軸 (Axis 1): Roll制御
  - ピッチ軸 (Axis 2): Pitch制御
  - ヨー軸 (Axis 3): Yaw制御
  - Modeボタン (Button 2): リセット
  - Optionボタン (Button 3): 終了

スロットル動作:
  - リセット後はホバリングDuty (約63%) がデフォルト
  - スティック中立: ホバリング維持
  - スティック上: 上昇 (Duty増加)
  - スティック下: 下降 (Duty減少)

モータミキシング (X-quad):
  M1 (FR): throttle - roll + pitch + yaw  CCW
  M2 (RR): throttle - roll - pitch - yaw  CW
  M3 (RL): throttle + roll - pitch + yaw  CCW
  M4 (FL): throttle + roll + pitch - yaw  CW

座標系:
  - モータモデル出力: NED機体座標系
  - Genesis入力: Genesis世界座標系
  - 変換: クォータニオン回転行列使用
"""

import sys
from pathlib import Path

# Add parent directory to path for motor_model import
script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir.parent))

import genesis as gs
import numpy as np
import time
import pygame

from motor_model import QuadMotorSystem, compute_hover_conditions


def quat_to_rotation_matrix(quat):
    """
    クォータニオン (w, x, y, z) から回転行列を計算
    Compute rotation matrix from quaternion (w, x, y, z)

    Returns: 3x3 rotation matrix (body to world)
    """
    w, x, y, z = [float(v) for v in quat]

    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])
    return R


def ned_to_genesis_force(force_ned):
    """
    NED機体座標系の力をGenesis機体座標系に変換
    NED: X=Forward, Y=Right, Z=Down
    Genesis: X=Right, Y=Forward, Z=Up
    """
    return np.array([force_ned[1], force_ned[0], -force_ned[2]])


def ned_to_genesis_moment(moment_ned):
    """
    NED機体座標系のモーメントをGenesis機体座標系に変換
    NED: roll=X, pitch=Y, yaw=Z
    Genesis: rx=pitch, ry=roll, rz=-yaw
    """
    return np.array([moment_ned[1], moment_ned[0], -moment_ned[2]])


def mix_motors(throttle, roll, pitch, yaw, mix_scale=0.3):
    """
    X-quad motor mixing
    スティック入力から各モータのDuty比を計算

    Args:
        throttle: 0.0 to 1.0
        roll, pitch, yaw: -1.0 to 1.0
        mix_scale: Control authority scaling

    Returns:
        [duty1, duty2, duty3, duty4] for motors M1-M4
    """
    # Scale control inputs
    r = roll * mix_scale
    p = pitch * mix_scale
    y = yaw * mix_scale

    # X-quad mixing matrix
    #        thrust  roll  pitch  yaw
    # M1 FR:   +      -      +     +   CCW
    # M2 RR:   +      -      -     -   CW
    # M3 RL:   +      +      -     +   CCW
    # M4 FL:   +      +      +     -   CW
    d1 = throttle - r + p + y
    d2 = throttle - r - p - y
    d3 = throttle + r - p + y
    d4 = throttle + r + p - y

    # Clamp to valid range
    duties = [max(0.0, min(1.0, d)) for d in [d1, d2, d3, d4]]
    return duties


def main():
    print("=" * 60)
    print("Controller Motor Model Test - StampFly")
    print("=" * 60)

    # パス設定
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly_fixed.urdf"

    if not urdf_file.exists():
        print(f"ERROR: URDF not found: {urdf_file}")
        return

    # タイミング設定
    PHYSICS_HZ = 1000    # モータモデル用に高周波数
    PHYSICS_DT = 1 / PHYSICS_HZ
    RENDER_FPS = 60
    RENDER_DT = 1 / RENDER_FPS

    # 制御設定
    DEADZONE = 0.05      # スティックのデッドゾーン
    MIX_SCALE = 0.3      # ミキシングスケール
    THROTTLE_RANGE = 0.4 # スロットル調整範囲 (±40%)

    # ホバリング条件を計算
    hover = compute_hover_conditions()
    HOVER_DUTY = hover['duty_hover']

    print(f"\nHover conditions:")
    print(f"  Thrust/motor: {hover['thrust_per_motor']*1000:.1f} mN")
    print(f"  RPM: {hover['rpm_hover']:.0f}")
    print(f"  Voltage: {hover['voltage_hover']:.2f} V")
    print(f"  Duty: {HOVER_DUTY*100:.1f}%")

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

    # モータモデル初期化
    print("\n[2] Initializing motor model...")
    motor_system = QuadMotorSystem()

    # Genesis初期化
    print("\n[3] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成（重力あり）
    print("\n[4] Creating scene (with gravity)...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0.5, -0.5, 0.3),
            camera_lookat=(0, 0, 0.15),
            camera_fov=60,
            max_FPS=RENDER_FPS,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -9.81),
            dt=PHYSICS_DT,
        ),
    )

    # 地面
    print("\n[5] Adding ground plane...")
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
    build_elapsed = time.perf_counter() - build_start
    print(f"    Build time: {build_elapsed:.1f}s")

    # 情報表示
    print("\n" + "=" * 60)
    print("Controls (StampFly Controller USB HID)")
    print("=" * 60)
    print("  Throttle (Axis 0): Hover ± adjustment")
    print("  Roll     (Axis 1): Roll control")
    print("  Pitch    (Axis 2): Pitch control")
    print("  Yaw      (Axis 3): Yaw control")
    print("  Mode button (Button 2): Reset")
    print("  Option button (Button 3): Exit")
    print()
    print(f"  Base duty: {HOVER_DUTY*100:.0f}% (hover)")
    print(f"  Throttle range: ±{THROTTLE_RANGE*100:.0f}%")
    print(f"  Physics: {PHYSICS_HZ} Hz")
    print(f"  Deadzone: {DEADZONE}")
    print("=" * 60)

    # シミュレーション実行
    print("\n[9] Running simulation...")

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
            roll_input = apply_deadzone(joystick.get_axis(1), DEADZONE)
            pitch_input = apply_deadzone(joystick.get_axis(2), DEADZONE)
            yaw_input = apply_deadzone(joystick.get_axis(3), DEADZONE)

            # スロットル: ホバリングDutyを基準に±調整
            # スティック中立(0) → ホバリングDuty
            # スティック上(+1) → ホバリングDuty + THROTTLE_RANGE
            # スティック下(-1) → ホバリングDuty - THROTTLE_RANGE
            throttle_adjust = throttle_raw * THROTTLE_RANGE
            throttle = HOVER_DUTY + throttle_adjust

            # モータDuty計算
            duties = mix_motors(throttle, roll_input, pitch_input, yaw_input, MIX_SCALE)

            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            # 物理ステップ
            while sim_time <= real_time:
                # モータモデル更新 (NED機体座標系で力・モーメント計算)
                force_ned, moment_ned = motor_system.step_with_duty(duties, PHYSICS_DT)

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

                ned_roll = genesis_ry
                ned_pitch = genesis_rx
                ned_yaw = -genesis_rz

                # 位置取得
                pos = drone.get_pos()
                alt = float(pos[2])

                # モータ状態
                total_thrust = motor_system.total_thrust * 1000  # mN

                print(f"  t={sim_time:.0f}s | "
                      f"thr={throttle*100:.0f}%({throttle_adjust*100:+.0f}) | "
                      f"T={total_thrust:.0f}mN | "
                      f"alt={alt:.2f}m | "
                      f"RPY=({ned_roll:+.0f},{ned_pitch:+.0f},{ned_yaw:+.0f})deg")

    except KeyboardInterrupt:
        pass

    print("\nSimulation ended.")
    pygame.quit()


if __name__ == "__main__":
    main()
