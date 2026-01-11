#!/usr/bin/env python3
"""
17_realtime_torque_test.py - StampFly トルク応答テスト（正しいリアルタイム版）
StampFly torque response test in zero gravity (correct realtime implementation)

リアルタイム設計:
  - dt = 1/240秒 (物理刻み幅、望む精度で設定)
  - 描画FPS = 60 (ビューア設定)
  - 物理計算: scene.step(update_visualizer=False, refresh_visualizer=False)
  - レンダリング: scene.visualizer.update() (物理を進めない)
  - 同期: 実時間に対してシミュレーション時間を追従

原理:
  - シミュレーション時間 = step数 × dt
  - 実時間より遅れていたら物理ステップを実行
  - 実時間より先行していたら待機
  - レンダリング時刻に達したら描画

DOF構成 (fixed=False URDF):
  - DOF 0, 1, 2: 並進 (x, y, z)
  - DOF 3, 4, 5: 回転 (rx, ry, rz)
  - DOF 6-9: プロペラジョイント
"""

import genesis as gs
from pathlib import Path
import numpy as np
import time


def main():
    print("=" * 60)
    print("StampFly Torque Response Test (Zero Gravity, REALTIME)")
    print("=" * 60)

    # パス設定
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly.urdf"

    if not urdf_file.exists():
        print(f"ERROR: URDF not found: {urdf_file}")
        return

    # タイミング設定
    PHYSICS_HZ = 240           # 物理シミュレーション周波数
    PHYSICS_DT = 1 / PHYSICS_HZ  # 物理時間刻み
    RENDER_FPS = 60            # 描画FPS
    RENDER_DT = 1 / RENDER_FPS  # 描画間隔

    print(f"\nRealtime Configuration:")
    print(f"  Physics: {PHYSICS_HZ} Hz (dt = {PHYSICS_DT*1000:.2f} ms)")
    print(f"  Render: {RENDER_FPS} FPS (interval = {RENDER_DT*1000:.1f} ms)")
    print(f"  Sync method: realtime wall-clock")

    # Genesis初期化
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成（無重力）
    print("\n[2] Creating scene (zero gravity)...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0.4, 0.4, 0.3),
            camera_lookat=(0, 0, 0.15),
            camera_fov=45,
            max_FPS=RENDER_FPS,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, 0),
            dt=PHYSICS_DT,
        ),
    )

    # 地面（視覚参照のみ）
    print("\n[3] Adding ground plane (visual only)...")
    scene.add_entity(
        gs.morphs.Plane(collision=False),
        surface=gs.surfaces.Default(color=(0.3, 0.3, 0.3)),
    )

    # 座標軸マーカー（コーナー、コリジョンなし）
    print("\n[4] Adding axis markers...")
    axis_origin = (-0.3, -0.3, 0.001)
    axis_len, axis_thick = 0.1, 0.002
    scene.add_entity(
        gs.morphs.Box(size=(axis_len, axis_thick, axis_thick),
                      pos=(axis_origin[0]+axis_len/2, axis_origin[1], axis_origin[2]),
                      fixed=True, collision=False),
        surface=gs.surfaces.Default(color=(1, 0.2, 0.2)),
    )
    scene.add_entity(
        gs.morphs.Box(size=(axis_thick, axis_len, axis_thick),
                      pos=(axis_origin[0], axis_origin[1]+axis_len/2, axis_origin[2]),
                      fixed=True, collision=False),
        surface=gs.surfaces.Default(color=(0.2, 1, 0.2)),
    )
    scene.add_entity(
        gs.morphs.Box(size=(axis_thick, axis_thick, axis_len),
                      pos=(axis_origin[0], axis_origin[1], axis_origin[2]+axis_len/2),
                      fixed=True, collision=False),
        surface=gs.surfaces.Default(color=(0.2, 0.5, 1)),
    )

    # StampFly URDF
    print("\n[5] Loading StampFly URDF...")
    stampfly = scene.add_entity(
        gs.morphs.URDF(
            file=str(urdf_file),
            pos=(0, 0, 0.15),
            euler=(0, 0, 0),
            fixed=False,
            prioritize_urdf_material=True,
        ),
    )

    # シーンビルド
    print("\n[6] Building scene...")
    scene.build()

    # トルク設定
    TORQUE = 1e-4  # Nm
    PHASE_DURATION = 5.0  # 各軸5秒（リアルタイム）

    phases = [
        {"name": "X-axis (Red)", "dof": 3, "duration": PHASE_DURATION, "reset": True},
        {"name": "Rest", "dof": None, "duration": 1.0, "reset": False},
        {"name": "Y-axis (Green)", "dof": 4, "duration": PHASE_DURATION, "reset": True},
        {"name": "Rest", "dof": None, "duration": 1.0, "reset": False},
        {"name": "Z-axis (Blue)", "dof": 5, "duration": PHASE_DURATION, "reset": True},
        {"name": "Rest", "dof": None, "duration": 1.0, "reset": False},
    ]

    # 情報表示
    print("\n" + "=" * 60)
    print("Test Configuration")
    print("=" * 60)
    print(f"  Torque: {TORQUE} Nm")
    print(f"  Phase duration: {PHASE_DURATION} s (REALTIME)")
    print()
    print("  Phases:")
    for i, p in enumerate(phases):
        print(f"    {i+1}. {p['name']}: {p['duration']}s")
    print()
    print("  Controls: Q/ESC=Exit, Mouse=Rotate, Scroll=Zoom")
    print("=" * 60)

    # シミュレーション実行（正しいリアルタイムループ）
    print("\n[7] Running simulation (REALTIME)...")

    physics_steps = 0
    phase_idx = 0
    phase_start_step = 0
    next_render_time = 0
    last_print_time = -1  # 1秒ごとに表示
    start_time = time.perf_counter()

    try:
        while phase_idx < len(phases) and scene.viewer.is_alive():
            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            phase = phases[phase_idx]
            phase_sim_time = (physics_steps - phase_start_step) * PHYSICS_DT

            # フェーズ切り替え
            if phase_sim_time >= phase["duration"]:
                phase_idx += 1
                phase_start_step = physics_steps
                if phase_idx < len(phases):
                    print(f"\n>>> Phase {phase_idx+1}: {phases[phase_idx]['name']}")
                continue

            # フェーズ開始
            if physics_steps == phase_start_step:
                print(f"\n>>> Phase {phase_idx+1}: {phase['name']}")
                if phase.get("reset"):
                    scene.reset()
                    physics_steps = phase_start_step = 0
                    next_render_time = 0
                    last_print_time = -1
                    start_time = time.perf_counter()
                    real_time = 0
                    sim_time = 0
                    print("    (Reset)")

            # シミュレーションが実時間より遅れている間は物理を進める
            while sim_time <= real_time:
                # トルク適用
                force = np.zeros(stampfly.n_dofs)
                if phase["dof"] is not None:
                    force[phase["dof"]] = TORQUE
                stampfly.control_dofs_force(force)

                # 物理ステップ（レンダリングなし）
                scene.step(update_visualizer=False, refresh_visualizer=False)
                physics_steps += 1
                sim_time = physics_steps * PHYSICS_DT

            # レンダリング時刻に達したら描画（物理は進めない）
            if real_time >= next_render_time:
                scene.visualizer.update()
                next_render_time += RENDER_DT

            # シミュレーションが実時間より先に進んだら待機
            sleep_time = sim_time - real_time
            if sleep_time > 0:
                time.sleep(sleep_time)

            # 1秒ごとに状態表示
            current_second = int(sim_time)
            if current_second > last_print_time:
                last_print_time = current_second
                real_elapsed = time.perf_counter() - start_time

                quat = stampfly.get_quat()
                dofs_vel = stampfly.get_dofs_velocity()
                ang_vel = dofs_vel[3:6]

                # クォータニオン → オイラー角
                w, x, y, z = [float(v) for v in quat]
                roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
                pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
                yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

                # リアルタイム比率
                ratio = sim_time / real_elapsed if real_elapsed > 0 else 0

                print(f"  t={sim_time:.1f}s (real:{real_elapsed:.1f}s, ratio:{ratio:.2f}) | "
                      f"euler=({np.degrees(roll):.0f}°,{np.degrees(pitch):.0f}°,{np.degrees(yaw):.0f}°) | "
                      f"ω=({float(ang_vel[0]):.1f},{float(ang_vel[1]):.1f},{float(ang_vel[2]):.1f}) rad/s")

        print("\n  All phases completed!")
        print("  Close viewer to exit.")

        # 終了後もビューア維持
        while scene.viewer.is_alive():
            scene.visualizer.update()
            time.sleep(RENDER_DT)

    except KeyboardInterrupt:
        pass

    print("\nSimulation ended.")


if __name__ == "__main__":
    main()
