#!/usr/bin/env python3
"""
07_stl_orientation_test.py - STL座標系確認テスト
STL coordinate system verification test

目的/Purpose:
- STLファイル（WebGL座標）がGenesisでどう表示されるか確認
- Verify how STL files (WebGL coordinates) appear in Genesis
- 必要な回転調整を特定
- Identify required rotation adjustments
"""

import genesis as gs
from pathlib import Path
import math


def main():
    print("=" * 50)
    print("Genesis STL Orientation Test")
    print("=" * 50)

    # STLファイルパス
    # STL file path
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    stl_file = assets_dir / "frame.stl"

    print(f"\n[0] STL file: {stl_file}")
    if not stl_file.exists():
        print(f"    ERROR: STL file not found!")
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
            dt=1/60,
        ),
    )

    # 地面追加
    # Add ground plane
    print("\n[3] Adding ground plane...")
    scene.add_entity(gs.morphs.Plane())

    # 座標軸マーカー追加
    # Add axis markers
    print("\n[4] Adding axis markers...")

    # X軸（赤方向）
    scene.add_entity(
        gs.morphs.Box(
            size=(0.5, 0.02, 0.02),
            pos=(0.25, 0, 0.01),
            fixed=True,
        ),
    )

    # Y軸（緑方向）
    scene.add_entity(
        gs.morphs.Box(
            size=(0.02, 0.5, 0.02),
            pos=(0, 0.25, 0.01),
            fixed=True,
        ),
    )

    # Z軸（青方向 - 上向き）
    scene.add_entity(
        gs.morphs.Box(
            size=(0.02, 0.02, 0.5),
            pos=(0, 0, 0.25),
            fixed=True,
        ),
    )

    print("    X-axis: Long box along X (should be horizontal)")
    print("    Y-axis: Long box along Y (should be horizontal)")
    print("    Z-axis: Long box along Z (should be vertical/up)")

    # STL読み込み（変換なし）
    # Load STL without transformation
    print("\n[5] Loading StampFly STL (no rotation)...")
    print("    STL is in WebGL coordinates (Y-up, Z-forward)")
    print("    Genesis uses Z-up coordinates")
    print("    Expected: Model may appear rotated")

    mesh_original = scene.add_entity(
        gs.morphs.Mesh(
            file=str(stl_file),
            scale=0.001,  # mm → m
            pos=(0, 0, 0.5),  # 50cm height
            fixed=True,
        ),
    )
    print(f"    -> Original mesh loaded at (0, 0, 0.5)")

    # STL読み込み（座標変換あり）
    # Load STL with coordinate transformation
    # WebGL (Y-up, Z-forward) → Genesis (Z-up, Y-forward)
    # Rotation: -90° around X-axis
    print("\n[6] Loading StampFly STL (with -90° X rotation)...")
    print("    WebGL→Genesis: Rotate -90° around X-axis")
    print("    This should align Y-up to Z-up")

    mesh_rotated = scene.add_entity(
        gs.morphs.Mesh(
            file=str(stl_file),
            scale=0.001,
            pos=(0.3, 0, 0.5),  # Offset to the right
            euler=(-90, 0, 0),  # -90° around X-axis (degrees)
            fixed=True,
        ),
    )
    print(f"    -> Rotated mesh loaded at (0.3, 0, 0.5)")

    # シーンビルド
    # Build scene
    print("\n[7] Building scene...")
    scene.build()

    print("\n" + "=" * 50)
    print("Coordinate System Check:")
    print("=" * 50)
    print("")
    print("  Genesis coordinate system:")
    print("    X-axis: Horizontal (long box)")
    print("    Y-axis: Horizontal (long box)")
    print("    Z-axis: UP (vertical box)")
    print("")
    print("  Two StampFly models loaded:")
    print("    LEFT  (0,0,0.5): Original - no rotation")
    print("    RIGHT (0.3,0,0.5): Rotated -90° around X")
    print("")
    print("  Expected result:")
    print("    - Original may appear with wrong orientation")
    print("    - Rotated should have correct orientation")
    print("      (top facing Z+, front facing Y+)")
    print("")
    print("  STL Coordinate: WebGL (X=left, Y=up, Z=forward)")
    print("  Genesis:        Z-up (X=right?, Y=forward?, Z=up)")
    print("=" * 50)

    # シミュレーション実行
    # Run simulation
    print("\n[8] Running simulation for 10 seconds...")
    print("    Compare the two models to verify orientation")

    try:
        for i in range(600):
            scene.step()
            if i % 120 == 0:
                print(f"    Time {i/60:.1f}s")
    except KeyboardInterrupt:
        print("\n    Interrupted by user")

    print("\n" + "=" * 50)
    print("Test completed!")
    print("=" * 50)


if __name__ == "__main__":
    main()
