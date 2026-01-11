#!/usr/bin/env python3
"""
19_controller_torque_test.py - コントローラでトルク制御テスト
Controller-based torque control test for StampFly

操作 (StampFly Controller USB HID):
  - ロール軸スティック (Axis 1): Roll  (NED X軸/前方軸 回りのトルク)
  - ピッチ軸スティック (Axis 2): Pitch (NED Y軸/右方軸 回りのトルク)
  - ヨー軸スティック (Axis 3): Yaw   (NED Z軸/下方軸 回りのトルク)
  - Modeボタン (Button 2): リセット
  - Optionボタン (Button 3): 終了

USB HID Axis構成 (モード非依存の論理軸):
  - Axis 0: Throttle (スロットル軸)
  - Axis 1: Roll/Aileron (ロール軸)
  - Axis 2: Pitch/Elevator (ピッチ軸)
  - Axis 3: Yaw/Rudder (ヨー軸)

USB HID Button構成:
  - Button 0: Arm (スロットル側スティック押込)
  - Button 1: Flip (エレベータ側スティック押込)
  - Button 2: Mode (サイドボタン)
  - Button 3: Option (サイドボタン)

座標系変換 (NED → Genesis):
  NED座標系: X=前, Y=右, Z=下 (航空工学標準)
  Genesis座標系: X=右, Y=前, Z=上

  位置/力:    NED (x,y,z) → Genesis (y, x, -z)
  トルク:     NED (τx,τy,τz) → Genesis body (τy, τx, -τz)
  オイラー角: NED (roll,pitch,yaw) → Genesis euler (ry, rx, -rz)
  角速度:     NED (p,q,r) → Genesis (ωy, ωx, -ωz)

機体座標系での力・トルク適用:
  - 推力は機体座標系で定義され、機体の傾きに追従して回転する
  - トルクも機体座標系で定義され、機体軸周りに作用する
  - クォータニオンを使って機体座標系→世界座標系に変換後、DOFに適用

Genesis DOF構成 (stampfly_fixed.urdf):
  - DOF 0, 1, 2: 並進 (gx, gy, gz) - 世界座標系
  - DOF 3, 4, 5: 回転 (rx, ry, rz) - 世界座標系
"""

import genesis as gs
from pathlib import Path
import numpy as np
import time
import pygame


def quat_to_rotation_matrix(quat):
    """
    クォータニオン (w, x, y, z) から回転行列を計算
    Compute rotation matrix from quaternion (w, x, y, z)

    Returns: 3x3 rotation matrix (body to world)
    """
    w, x, y, z = [float(v) for v in quat]

    # Rotation matrix from quaternion
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])
    return R


def body_to_world_force(force_body, quat):
    """
    機体座標系の力を世界座標系に変換
    Transform body-frame force to world-frame force

    force_body: 機体座標系での力ベクトル (Genesis座標系)
    quat: 機体の姿勢クォータニオン (w, x, y, z)

    Returns: 世界座標系での力ベクトル
    """
    R = quat_to_rotation_matrix(quat)
    force_world = R @ force_body
    return force_world


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

    # 制御設定
    MAX_TORQUE = 5e-5  # 最大トルク (Nm) - 小さめに設定
    MAX_THRUST = 1.0   # 最大推力 (N) - ホバー推力 ≈ 0.33N, 余裕を持たせる
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

    # シーン作成（重力あり）
    print("\n[3] Creating scene (with gravity)...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0.5, -0.5, 0.3),
            camera_lookat=(0, 0, 0.15),
            camera_fov=60,
            max_FPS=RENDER_FPS,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -9.81),  # Genesis Z-up: 重力は-Z方向
            dt=PHYSICS_DT,
        ),
    )

    # 地面（衝突あり）
    print("\n[4] Adding ground plane...")
    scene.add_entity(
        gs.morphs.Plane(collision=True),
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
    print("  Throttle stick (Axis 0): Thrust (機体上向き推力)")
    print("  Roll stick     (Axis 1): Roll   (機体X軸/前方軸)")
    print("  Pitch stick    (Axis 2): Pitch  (機体Y軸/右方軸)")
    print("  Yaw stick      (Axis 3): Yaw    (機体Z軸/下方軸)")
    print("  Mode button (Button 2): Reset")
    print("  Option button (Button 3): Exit")
    print()
    print("  Coordinate: NED body frame (X=Forward, Y=Right, Z=Down)")
    print("  Force/Torque: Applied in body frame, rotates with drone")
    print(f"  Max torque: {MAX_TORQUE} Nm")
    print(f"  Max thrust: {MAX_THRUST} N (hover ≈ 0.33N = 33%)")
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
            # Axis 0: Throttle (スロットル軸) - 推力制御
            # Axis 1: Roll/Aileron (ロール軸)
            # Axis 2: Pitch/Elevator (ピッチ軸)
            # Axis 3: Yaw/Rudder (ヨー軸)
            # 注: ファームウェアはスロットルに不感帯を適用していないため、
            #     シミュレータ側で適用する
            throttle_raw = apply_deadzone(joystick.get_axis(0), DEADZONE)  # 不感帯適用
            roll_input = apply_deadzone(joystick.get_axis(1), DEADZONE)    # ロール軸
            pitch_input = apply_deadzone(joystick.get_axis(2), DEADZONE)   # ピッチ軸
            yaw_input = apply_deadzone(joystick.get_axis(3), DEADZONE)     # ヨー軸

            # スロットル: 中立(0)より上で推力発生
            # スティック中立以下 = 0.0 (推力なし)
            # スティック上 = +1.0 → 最大推力
            if throttle_raw > 0:
                throttle_normalized = throttle_raw  # 0~1
            else:
                throttle_normalized = 0.0
            thrust_force = throttle_normalized * MAX_THRUST  # 上向き推力 (N)

            # NED座標系でのトルク計算
            # Roll: NED X軸(前方)周り, +で右翼下げ
            # Pitch: NED Y軸(右方)周り, +で機首上げ, -で機首下げ
            # Yaw: NED Z軸(下方)周り, +で右旋回
            #
            # パイロット操作規約:
            #   スティック前 = 負の信号 = 機首下げ (ダイブ) = -Pitch
            #   スティック後 = 正の信号 = 機首上げ (引き起こし) = +Pitch
            ned_torque_roll = roll_input * MAX_TORQUE
            ned_torque_pitch = pitch_input * MAX_TORQUE  # 符号反転不要：スティック前→負→機首下げ
            ned_torque_yaw = yaw_input * MAX_TORQUE

            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            # 物理ステップ
            while sim_time <= real_time:
                # 現在の姿勢を取得
                quat = drone.get_quat()  # (w, x, y, z) in Genesis

                # 機体座標系での推力 (Genesis座標系)
                # Genesis機体座標系: Z軸が上向き（ローター推力方向）
                thrust_body = np.array([0.0, 0.0, thrust_force])

                # 機体座標系 → 世界座標系に変換
                # 機体が傾くと推力方向も傾く（実際の物理現象を再現）
                thrust_world = body_to_world_force(thrust_body, quat)

                # 機体座標系でのトルク (Genesis座標系)
                # Genesis機体座標系でのトルク:
                #   NED Roll  (X/前方軸) → Genesis body ry (前方軸)
                #   NED Pitch (Y/右方軸) → Genesis body rx (右方軸)
                #   NED Yaw   (Z/下方軸) → Genesis body rz (上方軸) 符号反転
                torque_body = np.array([
                    ned_torque_pitch,   # Genesis rx = NED pitch
                    ned_torque_roll,    # Genesis ry = NED roll
                    -ned_torque_yaw     # Genesis rz = -NED yaw
                ])

                # 機体座標系 → 世界座標系に変換
                torque_world = body_to_world_force(torque_body, quat)

                # DOF力として適用 (世界座標系)
                force = np.zeros(drone.n_dofs)
                # 並進力 (DOF 0,1,2 = Genesis world x,y,z)
                force[0] = thrust_world[0]
                force[1] = thrust_world[1]
                force[2] = thrust_world[2]
                # 回転トルク (DOF 3,4,5 = Genesis world rx,ry,rz)
                force[3] = torque_world[0]
                force[4] = torque_world[1]
                force[5] = torque_world[2]
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

                # Genesis座標系での姿勢・角速度取得
                quat = drone.get_quat()
                dofs_vel = drone.get_dofs_velocity()
                genesis_ang_vel = dofs_vel[3:6]  # (ωx, ωy, ωz) in Genesis

                # Genesis クォータニオン → Genesis オイラー角
                # Genesis uses WXYZ quaternion format
                w, gx, gy, gz = [float(v) for v in quat]
                # Genesis euler angles (rx, ry, rz)
                genesis_rx = np.degrees(np.arctan2(2*(w*gx + gy*gz), 1 - 2*(gx*gx + gy*gy)))
                genesis_ry = np.degrees(np.arcsin(np.clip(2*(w*gy - gz*gx), -1, 1)))
                genesis_rz = np.degrees(np.arctan2(2*(w*gz + gx*gy), 1 - 2*(gy*gy + gz*gz)))

                # Genesis → NED 変換
                # オイラー角: Genesis (rx, ry, rz) → NED (roll=ry, pitch=rx, yaw=-rz)
                ned_roll = genesis_ry
                ned_pitch = genesis_rx
                ned_yaw = -genesis_rz

                # 角速度: Genesis (ωx, ωy, ωz) → NED (p=ωy, q=ωx, r=-ωz)
                ned_p = float(genesis_ang_vel[1])  # Genesis ωy → NED p
                ned_q = float(genesis_ang_vel[0])  # Genesis ωx → NED q
                ned_r = -float(genesis_ang_vel[2]) # Genesis ωz → NED r (符号反転)

                # 位置取得 (Genesis → NED)
                pos = drone.get_pos()
                ned_z = -float(pos[2])  # Genesis Z(上) → NED Z(下)

                print(f"  t={sim_time:.0f}s | "
                      f"thr={throttle_normalized:.2f} | "
                      f"stick=({roll_input:+.2f},{pitch_input:+.2f},{yaw_input:+.2f}) | "
                      f"alt={-ned_z:.2f}m | "
                      f"angle=({ned_roll:+.0f},{ned_pitch:+.0f},{ned_yaw:+.0f})deg")

    except KeyboardInterrupt:
        pass

    print("\nSimulation ended.")
    pygame.quit()


if __name__ == "__main__":
    main()
