#!/usr/bin/env python3
"""
03_falling_cube.py - 立方体の自由落下
Falling cube simulation

目的/Purpose:
- 剛体の自由落下シミュレーション
- Rigid body free fall simulation
- 物理演算の動作確認
- Verify physics simulation
"""

import genesis as gs


def main():
    print("=" * 50)
    print("Genesis Falling Cube Test")
    print("=" * 50)

    # Genesis初期化
    # Initialize Genesis
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成（重力あり）
    # Create scene with gravity
    print("\n[2] Creating scene with gravity...")
    scene = gs.Scene(
        show_viewer=True,
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -9.81),  # Z-down gravity
            dt=1/60,  # 60 Hz simulation
        ),
    )

    # 地面追加
    # Add ground plane
    print("\n[3] Adding ground plane...")
    plane = scene.add_entity(
        gs.morphs.Plane(),
    )

    # 立方体追加（高さ2mからスタート）
    # Add cube (starting at 2m height)
    print("\n[4] Adding falling cube at 2m height...")
    cube = scene.add_entity(
        gs.morphs.Box(
            size=(0.1, 0.1, 0.1),  # 10cm cube
            pos=(0, 0, 2.0),  # 2m height
        ),
    )
    print(f"    -> Cube added: {cube}")

    # シーンビルド
    # Build scene
    print("\n[5] Building scene...")
    scene.build()
    print("    -> Scene built!")

    # シミュレーション実行
    # Run simulation
    print("\n[6] Running simulation for 5 seconds...")
    print("    Watch the cube fall and bounce!")

    try:
        for i in range(300):
            scene.step()
            if i % 60 == 0:
                print(f"    Step {i}/300 ({i/60:.1f}s)")
    except KeyboardInterrupt:
        print("\n    Interrupted by user")

    print("\n" + "=" * 50)
    print("Test completed successfully!")
    print("=" * 50)


if __name__ == "__main__":
    main()
