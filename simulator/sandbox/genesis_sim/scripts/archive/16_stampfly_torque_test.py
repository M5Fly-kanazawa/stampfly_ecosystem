#!/usr/bin/env python3
"""
16_stampfly_torque_test.py - StampFly トルク応答テスト（無重力）
StampFly torque response test in zero gravity

タイミング設計:
  - 物理シミュレーション: 240 Hz (dt = 1/240秒)
  - ビューア更新: 約57 FPS (max_FPS=60設定、実測約57)
  - 注意: scene.step() は毎回描画するため、実時間はシミュレーション時間の約4倍
    → 1フレーム実時間 ≈ 17.6ms (vsync待機)
    → 1フレームでシミュレーション時間 = 1/240 = 4.17ms
    → 実時間/シミュレーション時間 ≈ 4.2倍

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
    print("StampFly Torque Response Test (Zero Gravity, Realtime)")
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
    VIEWER_FPS = 57            # ビューア実測FPS（max_FPS=60で約57）
    # 注意: scene.step() は毎回描画待機するため、1step = ~17.6ms (実時間)
    # シミュレーション時間 5秒 = 1200 steps = 約21秒 (実時間)

    print(f"\nTiming Configuration:")
    print(f"  Physics: {PHYSICS_HZ} Hz (dt = {PHYSICS_DT*1000:.2f} ms)")
    print(f"  Viewer: ~{VIEWER_FPS} FPS (vsync)")
    print(f"  Sim/Real ratio: {PHYSICS_HZ / VIEWER_FPS:.2f}x slower")
    print(f"  5s sim time = ~{5.0 * VIEWER_FPS / PHYSICS_HZ * (1000/VIEWER_FPS):.0f}s real time")

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
            max_FPS=60,
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
    print(f"  Torque: {TORQUE} Nm")
    print(f"  Phase duration: {PHASE_DURATION} s (realtime)")
    print()
    print("  Phases:")
    for i, p in enumerate(phases):
        print(f"    {i+1}. {p['name']}: {p['duration']}s")
    print()
    print("  Controls: Q/ESC=Exit, Mouse=Rotate, Scroll=Zoom")
    print("=" * 60)

    # シミュレーション実行
    print("\n[7] Running simulation...")
    print("    (Note: Simulation runs ~4x slower than realtime due to vsync)")

    sim_step = 0
    phase_idx = 0
    phase_start_step = 0
    real_start_time = time.perf_counter()
    last_print_step = -PHYSICS_HZ  # 1秒ごとに表示

    try:
        while phase_idx < len(phases) and scene.viewer.is_alive():
            phase = phases[phase_idx]
            sim_time = sim_step * PHYSICS_DT
            phase_sim_time = (sim_step - phase_start_step) * PHYSICS_DT

            # フェーズ切り替え
            if phase_sim_time >= phase["duration"]:
                phase_idx += 1
                phase_start_step = sim_step
                if phase_idx < len(phases):
                    print(f"\n>>> Phase {phase_idx+1}: {phases[phase_idx]['name']}")
                continue

            # フェーズ開始
            if sim_step == phase_start_step:
                print(f"\n>>> Phase {phase_idx+1}: {phase['name']}")
                if phase.get("reset"):
                    scene.reset()
                    sim_step = phase_start_step = 0
                    real_start_time = time.perf_counter()
                    last_print_step = -PHYSICS_HZ
                    print("    (Reset)")

            # 1ステップ実行
            force = np.zeros(stampfly.n_dofs)
            if phase["dof"] is not None:
                force[phase["dof"]] = TORQUE
            stampfly.control_dofs_force(force)
            scene.step()
            sim_step += 1

            # シミュレーション時間1秒ごとに状態表示
            if sim_step - last_print_step >= PHYSICS_HZ:
                last_print_step = sim_step
                real_elapsed = time.perf_counter() - real_start_time
                sim_elapsed = sim_step * PHYSICS_DT

                quat = stampfly.get_quat()
                dofs_vel = stampfly.get_dofs_velocity()
                ang_vel = dofs_vel[3:6]

                # クォータニオン → オイラー角
                w, x, y, z = [float(v) for v in quat]
                roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
                pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
                yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

                print(f"  t={sim_elapsed:.1f}s (real:{real_elapsed:.1f}s) | "
                      f"euler=({np.degrees(roll):.0f}°,{np.degrees(pitch):.0f}°,{np.degrees(yaw):.0f}°) | "
                      f"ω=({float(ang_vel[0]):.1f},{float(ang_vel[1]):.1f},{float(ang_vel[2]):.1f}) rad/s")

        print("\n  All phases completed!")
        print("  Close viewer to exit.")

        while scene.viewer.is_alive():
            scene.step()

    except KeyboardInterrupt:
        pass

    print("\nSimulation ended.")


if __name__ == "__main__":
    main()
