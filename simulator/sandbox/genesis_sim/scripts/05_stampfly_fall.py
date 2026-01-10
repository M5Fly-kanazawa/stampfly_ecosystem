#!/usr/bin/env python3
"""
05_stampfly_fall.py - StampFly自由落下
StampFly free fall simulation

目的/Purpose:
- StampFlyの全パーツを読み込み
- Load all StampFly parts
- 空中からの自由落下シミュレーション
- Free fall simulation from mid-air
"""

import genesis as gs
from pathlib import Path
import json


def main():
    print("=" * 50)
    print("Genesis StampFly Free Fall Test")
    print("=" * 50)

    # パーツ設定ファイルパス
    # Parts config file path
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    config_file = assets_dir / "parts_config.json"

    print(f"\n[0] Loading parts config from: {config_file}")
    if not config_file.exists():
        print(f"    ERROR: Config file not found!")
        print(f"    Make sure the assets symlink exists:")
        print(f"    cd genesis_sim && ln -s ../../assets assets")
        return

    # パーツ設定読み込み
    # Load parts config
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
            gravity=(0, 0, -9.81),
            dt=1/60,
        ),
    )

    # 地面追加
    # Add ground plane
    print("\n[3] Adding ground plane...")
    scene.add_entity(gs.morphs.Plane())

    # StampFlyパーツ読み込み
    # Load StampFly parts
    print("\n[4] Loading StampFly parts...")
    spawn_height = 1.0  # 1m height

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
                    scale=0.001,  # mm -> m
                    pos=(0, 0, spawn_height),
                    fixed=True,  # Fixed initially (no physics)
                ),
            )
            loaded_parts.append((part['name'], mesh))
            print(f"    Loaded: {part['name']}")
        except Exception as e:
            print(f"    ERROR loading {part['name']}: {e}")

    print(f"    -> {len(loaded_parts)} parts loaded")

    # シーンビルド
    # Build scene
    print("\n[5] Building scene...")
    scene.build()
    print("    -> Scene built!")

    # シミュレーション実行
    # Run simulation
    print("\n[6] Running simulation for 5 seconds...")
    print("    You should see the StampFly floating at 1m height!")
    print("    (Parts are fixed, so they won't fall)")

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
