#!/usr/bin/env python3
"""
15_stampfly_freefall.py - StampFly自由落下シミュレーション
StampFly free fall simulation

目的/Purpose:
- URDFモデルの自由落下を確認
- Verify URDF model free fall behavior
- 重力による運動を観察
- Observe gravity-driven motion
"""

import genesis as gs
from pathlib import Path


def main():
    print("=" * 60)
    print("StampFly Free Fall Simulation")
    print("=" * 60)

    # パス設定
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly.urdf"

    if not urdf_file.exists():
        print(f"ERROR: URDF not found: {urdf_file}")
        print("Run generate_stampfly_urdf.py first.")
        return

    # Genesis初期化
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成（重力あり）
    print("\n[2] Creating scene with gravity...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0.3, 0.3, 0.4),
            camera_lookat=(0, 0, 0.2),
            camera_fov=45,
            max_FPS=60,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -9.81),  # Genesis: Z-up, gravity is -Z
            dt=1/240,  # 240Hz physics
        ),
    )

    # 地面追加
    print("\n[3] Adding ground plane...")
    scene.add_entity(gs.morphs.Plane())

    # 座標軸マーカー
    print("\n[4] Adding axis markers...")
    scene.add_entity(
        gs.morphs.Box(size=(0.2, 0.005, 0.005), pos=(0.1, 0, 0.0025), fixed=True),
        surface=gs.surfaces.Default(color=(1, 0.2, 0.2)),
    )
    scene.add_entity(
        gs.morphs.Box(size=(0.005, 0.2, 0.005), pos=(0, 0.1, 0.0025), fixed=True),
        surface=gs.surfaces.Default(color=(0.2, 1, 0.2)),
    )
    scene.add_entity(
        gs.morphs.Box(size=(0.005, 0.005, 0.2), pos=(0, 0, 0.1), fixed=True),
        surface=gs.surfaces.Default(color=(0.2, 0.5, 1)),
    )

    # StampFly URDF読み込み
    print("\n[5] Loading StampFly URDF...")
    spawn_height = 0.5  # 50cm上空からスタート

    stampfly = scene.add_entity(
        gs.morphs.URDF(
            file=str(urdf_file),
            pos=(0, 0, spawn_height),
            euler=(0, 0, 0),  # 変換はURDF内で済み
            fixed=False,  # 自由落下させる
            prioritize_urdf_material=True,
        ),
    )
    print(f"    -> Spawned at height {spawn_height}m")

    # シーンビルド
    print("\n[6] Building scene...")
    scene.build()

    # 情報表示
    print("\n" + "=" * 60)
    print("Free Fall Simulation")
    print("=" * 60)
    print()
    print(f"  Initial height: {spawn_height}m")
    print(f"  Gravity: -9.81 m/s² (Genesis -Z direction)")
    print(f"  Expected fall time: ~{(2*spawn_height/9.81)**0.5:.2f}s")
    print()
    print("  Controls:")
    print("    Q/ESC - Exit")
    print("    Mouse drag - Rotate view")
    print("    Scroll - Zoom")
    print()
    print("=" * 60)

    # シミュレーション実行
    print("\n[7] Running simulation...")

    try:
        step = 0
        while True:
            scene.step()

            # 状態表示（1秒ごと）
            if step % 240 == 0:
                pos = stampfly.get_pos()
                vel = stampfly.get_vel()
                t = step / 240.0
                # Genesis座標で表示
                print(f"  t={t:.1f}s: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})m, "
                      f"vel_z={vel[2]:.2f}m/s")

                # 地面に到達したらメッセージ
                if pos[2] < 0.02 and step > 10:
                    print(f"\n  Ground contact at t={t:.2f}s!")

            step += 1

            if not scene.viewer.is_alive():
                break

    except KeyboardInterrupt:
        pass

    print("\nSimulation ended.")


if __name__ == "__main__":
    main()
