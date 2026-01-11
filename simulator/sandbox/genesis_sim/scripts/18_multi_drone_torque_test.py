#!/usr/bin/env python3
"""
18_multi_drone_torque_test.py - 36機 StampFly 同時トルク応答テスト
36 StampFly drones torque response test in zero gravity (6x6 grid)

構成:
  - 6×6 グリッド配置（36機）
  - 各機体に同じトルクを適用
  - リアルタイム同期

制限事項:
  - Genesis (Taichi) の snode 数制限により 100機は不可
  - 49機 (7×7): ratio ≈ 0.89 (やや遅い)
  - 36機 (6×6): ratio ≈ 1.00 (リアルタイム達成)

グリッド設計:
  - 間隔: 0.15m (機体幅の約2倍)
  - 総サイズ: 0.75m × 0.75m
  - 中心: 原点 (0, 0)

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
    # グリッド設定（先に定義）
    GRID_SIZE = 6              # 6×6 = 36機

    print("=" * 60)
    print(f"Multi StampFly Drones Torque Test ({GRID_SIZE}x{GRID_SIZE} Grid, REALTIME)")
    print("=" * 60)

    # パス設定
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly.urdf"

    if not urdf_file.exists():
        print(f"ERROR: URDF not found: {urdf_file}")
        return

    SPACING = 0.15             # 機体間隔 (m)
    DRONE_HEIGHT = 0.15        # 浮遊高度 (m)

    # タイミング設定
    PHYSICS_HZ = 240           # 物理シミュレーション周波数
    PHYSICS_DT = 1 / PHYSICS_HZ
    RENDER_FPS = 60            # 描画FPS
    RENDER_DT = 1 / RENDER_FPS

    print(f"\nConfiguration:")
    print(f"  Drones: {GRID_SIZE}x{GRID_SIZE} = {GRID_SIZE**2}")
    print(f"  Grid spacing: {SPACING}m")
    print(f"  Grid size: {SPACING*(GRID_SIZE-1):.2f}m x {SPACING*(GRID_SIZE-1):.2f}m")
    print(f"  Physics: {PHYSICS_HZ} Hz")
    print(f"  Render: {RENDER_FPS} FPS")

    # Genesis初期化
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成（無重力）
    print("\n[2] Creating scene (zero gravity)...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0, -1.5, 1.5),    # 斜め上から俯瞰
            camera_lookat=(0, 0, DRONE_HEIGHT),
            camera_fov=60,
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

    # 座標軸マーカー（コーナー）
    print("\n[4] Adding axis markers...")
    grid_edge = SPACING * GRID_SIZE / 2 + 0.1
    axis_origin = (-grid_edge, -grid_edge, 0.001)
    axis_len, axis_thick = 0.15, 0.003
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

    # StampFly 100機を読み込み
    print(f"\n[5] Loading {GRID_SIZE**2} StampFly URDFs...")
    drones = []
    load_start = time.perf_counter()

    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            # グリッド中心を原点に
            x = (col - GRID_SIZE/2 + 0.5) * SPACING
            y = (row - GRID_SIZE/2 + 0.5) * SPACING
            z = DRONE_HEIGHT

            drone = scene.add_entity(
                gs.morphs.URDF(
                    file=str(urdf_file),
                    pos=(x, y, z),
                    euler=(0, 0, 0),
                    fixed=False,
                    prioritize_urdf_material=True,
                ),
            )
            drones.append(drone)

        # 進捗表示
        loaded = (row + 1) * GRID_SIZE
        print(f"    {loaded}/{GRID_SIZE**2} loaded...")

    load_elapsed = time.perf_counter() - load_start
    print(f"    Loading time: {load_elapsed:.1f}s")

    # シーンビルド
    print("\n[6] Building scene...")
    build_start = time.perf_counter()
    scene.build()
    build_elapsed = time.perf_counter() - build_start
    print(f"    Build time: {build_elapsed:.1f}s")

    # トルク設定
    TORQUE = 1e-4  # Nm
    PHASE_DURATION = 5.0  # 各軸5秒

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
    print(f"  Drones: {len(drones)}")
    print(f"  Total DOFs: {len(drones) * drones[0].n_dofs}")
    print(f"  Torque: {TORQUE} Nm")
    print(f"  Phase duration: {PHASE_DURATION} s (REALTIME)")
    print()
    print("  Phases:")
    for i, p in enumerate(phases):
        print(f"    {i+1}. {p['name']}: {p['duration']}s")
    print()
    print("  Controls: Q/ESC=Exit, Mouse=Rotate, Scroll=Zoom")
    print("=" * 60)

    # シミュレーション実行
    print("\n[7] Running simulation (REALTIME, infinite loop)...")

    physics_steps = 0
    phase_idx = 0
    phase_start_step = 0
    next_render_time = 0
    last_print_time = -1
    loop_count = 0
    start_time = time.perf_counter()

    # パフォーマンス計測用
    physics_time_total = 0
    physics_count = 0

    try:
        while scene.viewer.is_alive():
            real_time = time.perf_counter() - start_time
            sim_time = physics_steps * PHYSICS_DT

            phase = phases[phase_idx]
            phase_sim_time = (physics_steps - phase_start_step) * PHYSICS_DT

            # フェーズ切り替え
            if phase_sim_time >= phase["duration"]:
                phase_idx += 1
                phase_start_step = physics_steps
                if phase_idx >= len(phases):
                    loop_count += 1
                    phase_idx = 0
                    print(f"\n{'='*60}")
                    print(f"Loop {loop_count + 1} starting...")
                    print(f"{'='*60}")
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
                    physics_time_total = 0
                    physics_count = 0
                    print("    (Reset)")

            # シミュレーションが実時間より遅れている間は物理を進める
            while sim_time <= real_time:
                physics_start = time.perf_counter()

                # 全機にトルク適用
                for drone in drones:
                    force = np.zeros(drone.n_dofs)
                    if phase["dof"] is not None:
                        force[phase["dof"]] = TORQUE
                    drone.control_dofs_force(force)

                # 物理ステップ（レンダリングなし）
                scene.step(update_visualizer=False, refresh_visualizer=False)
                physics_steps += 1
                sim_time = physics_steps * PHYSICS_DT

                physics_time_total += time.perf_counter() - physics_start
                physics_count += 1

            # レンダリング時刻に達したら描画
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
                ratio = sim_time / real_elapsed if real_elapsed > 0 else 0

                # 代表機（中央）の状態
                center_drone = drones[GRID_SIZE * GRID_SIZE // 2]
                quat = center_drone.get_quat()
                dofs_vel = center_drone.get_dofs_velocity()
                ang_vel = dofs_vel[3:6]

                # クォータニオン → オイラー角
                w, x, y, z = [float(v) for v in quat]
                roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
                pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
                yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

                # 平均物理ステップ時間
                avg_physics_ms = (physics_time_total / physics_count * 1000) if physics_count > 0 else 0

                print(f"  t={sim_time:.1f}s (real:{real_elapsed:.1f}s, ratio:{ratio:.2f}) | "
                      f"physics:{avg_physics_ms:.2f}ms/step | "
                      f"ω=({float(ang_vel[0]):.1f},{float(ang_vel[1]):.1f},{float(ang_vel[2]):.1f})")

    except KeyboardInterrupt:
        pass

    print(f"\nSimulation ended after {loop_count + 1} loops.")


if __name__ == "__main__":
    main()
