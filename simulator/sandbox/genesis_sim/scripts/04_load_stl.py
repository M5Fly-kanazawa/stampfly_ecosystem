#!/usr/bin/env python3
"""
04_load_stl.py - STLファイル読み込み
STL file loading test

目的/Purpose:
- STLメッシュファイルの読み込み
- Load STL mesh file
- メッシュの表示確認
- Verify mesh display
"""

import genesis as gs
from pathlib import Path


def main():
    print("=" * 50)
    print("Genesis STL Loading Test")
    print("=" * 50)

    # STLファイルパス
    # STL file path
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    stl_file = assets_dir / "frame.stl"

    print(f"\n[0] STL file: {stl_file}")
    if not stl_file.exists():
        print(f"    ERROR: STL file not found!")
        print(f"    Make sure the assets symlink exists:")
        print(f"    cd genesis_sim && ln -s ../../assets assets")
        return

    # Genesis初期化
    # Initialize Genesis
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成
    # Create scene
    print("\n[2] Creating scene...")
    scene = gs.Scene(
        show_viewer=True,
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -9.81),
        ),
    )

    # 地面追加
    # Add ground plane
    print("\n[3] Adding ground plane...")
    scene.add_entity(gs.morphs.Plane())

    # STLメッシュ追加
    # Add STL mesh
    # STLはmm単位なので0.001倍してmに変換
    # STL is in mm, so scale by 0.001 to convert to m
    print("\n[4] Loading STL mesh...")
    mesh = scene.add_entity(
        gs.morphs.Mesh(
            file=str(stl_file),
            scale=0.001,  # mm -> m
            pos=(0, 0, 0.1),  # 10cm above ground
        ),
    )
    print(f"    -> Mesh loaded: {mesh}")

    # シーンビルド
    # Build scene
    print("\n[5] Building scene...")
    scene.build()
    print("    -> Scene built!")

    # シミュレーション実行
    # Run simulation
    print("\n[6] Running simulation for 5 seconds...")
    print("    You should see the StampFly frame mesh!")

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
