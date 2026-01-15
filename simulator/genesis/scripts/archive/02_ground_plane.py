#!/usr/bin/env python3
"""
02_ground_plane.py - 地面追加テスト
Ground plane test

目的/Purpose:
- 地面（Plane）をシーンに追加
- Add ground plane to scene
- 物理シミュレーションの基本確認
- Verify basic physics simulation
"""

import genesis as gs


def main():
    print("=" * 50)
    print("Genesis Ground Plane Test")
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
        ),
    )

    # 地面追加
    # Add ground plane
    print("\n[3] Adding ground plane...")
    plane = scene.add_entity(
        gs.morphs.Plane(),
    )
    print(f"    -> Ground plane added: {plane}")

    # シーンビルド
    # Build scene
    print("\n[4] Building scene...")
    scene.build()
    print("    -> Scene built!")

    # シミュレーション実行
    # Run simulation
    print("\n[5] Running simulation for 5 seconds...")
    print("    (Close the viewer window or wait to exit)")

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
