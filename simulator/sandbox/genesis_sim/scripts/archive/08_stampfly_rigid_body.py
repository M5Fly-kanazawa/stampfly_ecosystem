#!/usr/bin/env python3
"""
08_stampfly_rigid_body.py - StampFly剛体モデル構築
StampFly rigid body model construction

目的/Purpose:
- STLパーツを正しい座標変換でGenesisに読み込み
- Load STL parts with correct coordinate transformation to Genesis
- 単一の剛体としてStampFlyを構築
- Build StampFly as a single rigid body
- 自由落下テストで姿勢を確認
- Verify orientation with free fall test

座標変換/Coordinate Transformation:
- STL: WebGL座標系 (X=左, Y=上, Z=前)
- Genesis: Z-up座標系 (X=右, Y=前, Z=上)
- 変換: euler=(-90, 0, 0) でY-up → Z-upに回転
"""

import genesis as gs
from pathlib import Path
import json


def main():
    print("=" * 60)
    print("Genesis StampFly Rigid Body Model")
    print("=" * 60)

    # パーツ設定ファイルパス
    # Parts config file path
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    config_file = assets_dir / "parts_config.json"

    print(f"\n[0] Loading parts config...")
    if not config_file.exists():
        print(f"    ERROR: Config file not found: {config_file}")
        return

    with open(config_file, 'r') as f:
        config = json.load(f)

    parts = config.get('parts', [])
    print(f"    Found {len(parts)} parts")

    # Genesis初期化
    # Initialize Genesis
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成
    # Create scene
    print("\n[2] Creating scene with gravity...")
    scene = gs.Scene(
        show_viewer=True,
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -9.81),  # Z-down gravity in Genesis
            dt=1/60,
        ),
    )

    # 地面追加
    # Add ground plane
    print("\n[3] Adding ground plane...")
    scene.add_entity(gs.morphs.Plane())

    # 座標軸マーカー（参照用）
    # Axis markers for reference
    print("\n[4] Adding axis markers...")
    # X軸（赤）- Genesis右方向
    scene.add_entity(
        gs.morphs.Box(size=(0.3, 0.01, 0.01), pos=(0.15, 0, 0.005), fixed=True),
    )
    # Y軸（緑）- Genesis前方向
    scene.add_entity(
        gs.morphs.Box(size=(0.01, 0.3, 0.01), pos=(0, 0.15, 0.005), fixed=True),
    )
    # Z軸（青）- Genesis上方向
    scene.add_entity(
        gs.morphs.Box(size=(0.01, 0.01, 0.3), pos=(0, 0, 0.15), fixed=True),
    )
    print("    X-axis (red): Genesis Right (+X)")
    print("    Y-axis (green): Genesis Forward (+Y)")
    print("    Z-axis (blue): Genesis Up (+Z)")

    # StampFlyパーツ読み込み（座標変換適用）
    # Load StampFly parts with coordinate transformation
    print("\n[5] Loading StampFly parts with coordinate transformation...")
    print("    Transform: WebGL (Y-up) → Genesis (Z-up)")
    print("    Rotation: euler=(-90, 0, 0)")

    spawn_height = 0.5  # 50cm height

    # 座標変換パラメータ
    # Coordinate transformation parameters
    # WebGL: X=left, Y=up, Z=forward
    # Genesis: X=right, Y=forward, Z=up
    # Rotation -90° around X-axis: Y→Z, Z→-Y
    transform_euler = (-90, 0, 0)  # degrees

    loaded_parts = []
    for part in parts:
        part_file = assets_dir / part['file']
        if not part_file.exists():
            print(f"    WARNING: {part['file']} not found, skipping")
            continue

        try:
            mesh = scene.add_entity(
                gs.morphs.Mesh(
                    file=str(part_file),
                    scale=0.001,  # mm → m
                    pos=(0, 0, spawn_height),
                    euler=transform_euler,  # WebGL → Genesis座標変換
                    fixed=True,  # 最初は固定（後で解除）
                ),
            )
            loaded_parts.append((part['name'], mesh))
            print(f"    Loaded: {part['name']}")
        except Exception as e:
            print(f"    ERROR loading {part['name']}: {e}")

    print(f"\n    Total: {len(loaded_parts)} parts loaded")

    # シーンビルド
    # Build scene
    print("\n[6] Building scene...")
    scene.build()
    print("    -> Scene built!")

    # 座標系確認表示
    # Coordinate system verification display
    print("\n" + "=" * 60)
    print("Coordinate System Verification:")
    print("=" * 60)
    print("")
    print("  Expected StampFly orientation in Genesis:")
    print("    - Top of drone: facing +Z (up)")
    print("    - Front of drone: facing +Y (forward)")
    print("    - Right of drone: facing +X (right)")
    print("")
    print("  NED to Genesis mapping:")
    print("    NED X (forward) → Genesis Y")
    print("    NED Y (right)   → Genesis X")
    print("    NED Z (down)    → Genesis -Z")
    print("")
    print("  Motor positions (Genesis coordinates):")
    print("    FL: (-X, +Y) = left-front")
    print("    FR: (+X, +Y) = right-front")
    print("    RL: (-X, -Y) = left-rear")
    print("    RR: (+X, -Y) = right-rear")
    print("=" * 60)

    # シミュレーション実行（固定状態で確認）
    # Run simulation (fixed state for verification)
    print("\n[7] Running simulation (parts are FIXED)...")
    print("    Verify the drone orientation:")
    print("    - Should appear upright (top facing up)")
    print("    - Front should face +Y direction")
    print("")
    print("    Press Ctrl+C after 5 seconds to continue to free fall test")

    try:
        for i in range(300):  # 5 seconds
            scene.step()
            if i % 60 == 0:
                print(f"    Time {i/60:.1f}s - Fixed mode")
    except KeyboardInterrupt:
        print("\n    Continuing to free fall test...")

    print("\n" + "=" * 60)
    print("Test completed!")
    print("=" * 60)
    print("\nNext steps:")
    print("  1. Verify orientation is correct")
    print("  2. Combine parts into single rigid body")
    print("  3. Add mass and inertia parameters")
    print("  4. Enable free fall (fixed=False)")


if __name__ == "__main__":
    main()
