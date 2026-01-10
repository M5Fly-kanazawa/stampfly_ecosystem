#!/usr/bin/env python3
"""
06_coordinate_test.py - 座標系確認テスト
Coordinate system verification test

目的/Purpose:
- Genesisの座標系を確認
- Verify Genesis coordinate system
- 重力方向、軸の向きを可視化
- Visualize gravity direction and axis orientation
"""

import genesis as gs


def main():
    print("=" * 50)
    print("Genesis Coordinate System Test")
    print("=" * 50)

    # Genesis初期化
    # Initialize Genesis
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成（重力をZ負方向に設定）
    # Create scene with gravity in -Z direction
    print("\n[2] Creating scene...")
    print("    Gravity: (0, 0, -9.81) - Z is UP")
    scene = gs.Scene(
        show_viewer=True,
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -9.81),
            dt=1/60,
        ),
    )

    # 地面追加
    # Add ground plane
    print("\n[3] Adding ground plane at Z=0...")
    scene.add_entity(gs.morphs.Plane())

    # 座標軸マーカーとして色付き立方体を配置
    # Place colored cubes as axis markers
    print("\n[4] Adding axis markers (cubes)...")

    # 原点（白）
    # Origin (white) - will fall
    print("    Origin (0,0,0.5): White cube")
    origin_cube = scene.add_entity(
        gs.morphs.Box(
            size=(0.1, 0.1, 0.1),
            pos=(0, 0, 0.5),
        ),
    )

    # X軸正方向（赤）- 1m先
    # X-axis positive (red) - 1m away
    print("    X+ (1,0,0.5): Red marker")
    x_cube = scene.add_entity(
        gs.morphs.Box(
            size=(0.2, 0.1, 0.1),  # X方向に長い
            pos=(1, 0, 0.5),
        ),
    )

    # Y軸正方向（緑）- 1m先
    # Y-axis positive (green) - 1m away
    print("    Y+ (0,1,0.5): Green marker")
    y_cube = scene.add_entity(
        gs.morphs.Box(
            size=(0.1, 0.2, 0.1),  # Y方向に長い
            pos=(0, 1, 0.5),
        ),
    )

    # Z軸正方向（青）- 1m上
    # Z-axis positive (blue) - 1m up
    print("    Z+ (0,0,1.5): Blue marker (tall)")
    z_cube = scene.add_entity(
        gs.morphs.Box(
            size=(0.1, 0.1, 0.3),  # Z方向に長い
            pos=(0, 0, 1.5),
        ),
    )

    # シーンビルド
    # Build scene
    print("\n[5] Building scene...")
    scene.build()

    print("\n" + "=" * 50)
    print("Coordinate System Summary:")
    print("=" * 50)
    print("  Genesis uses RIGHT-HANDED coordinate system")
    print("  - X-axis: Horizontal (long cube at X+)")
    print("  - Y-axis: Horizontal (long cube at Y+)")
    print("  - Z-axis: UP (tall cube at Z+)")
    print("  - Gravity: -Z direction (down)")
    print("")
    print("  Comparison with other systems:")
    print("  ┌─────────────┬───────┬───────┬───────┐")
    print("  │ System      │ Right │  Up   │Forward│")
    print("  ├─────────────┼───────┼───────┼───────┤")
    print("  │ Genesis     │  +X?  │  +Z   │  +Y?  │")
    print("  │ NED (drone) │  +Y   │  -Z   │  +X   │")
    print("  │ WebGL       │  -X   │  +Y   │  +Z   │")
    print("  └─────────────┴───────┴───────┴───────┘")
    print("")
    print("  Watch the cubes fall and observe the axes!")
    print("=" * 50)

    # シミュレーション実行
    # Run simulation
    print("\n[6] Running simulation for 5 seconds...")
    print("    All cubes will fall due to gravity (-Z)")

    try:
        for i in range(300):
            scene.step()
            if i % 60 == 0:
                # 各物体の位置を表示
                print(f"\n    Time {i/60:.1f}s:")
    except KeyboardInterrupt:
        print("\n    Interrupted by user")

    print("\n" + "=" * 50)
    print("Test completed!")
    print("=" * 50)


if __name__ == "__main__":
    main()
