#!/usr/bin/env python3
"""
09_stampfly_freefall.py - StampFly自由落下テスト
StampFly free fall test

目的/Purpose:
- 物理パラメータを設定したStampFlyの自由落下
- Free fall of StampFly with physical parameters
- 姿勢を維持したまま落下するか確認
- Verify it falls while maintaining orientation

物理パラメータ/Physical Parameters:
- 総質量: 約50g (バッテリー込み)
- Total mass: ~50g (including battery)
- サイズ: 約100mm x 100mm (アーム間)
- Size: ~100mm x 100mm (arm to arm)
"""

import genesis as gs
from pathlib import Path
import json


def main():
    print("=" * 60)
    print("Genesis StampFly Free Fall Test")
    print("=" * 60)

    # パーツ設定ファイルパス
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
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成
    print("\n[2] Creating scene with gravity...")
    scene = gs.Scene(
        show_viewer=True,
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -9.81),
            dt=1/60,
        ),
    )

    # 地面追加
    print("\n[3] Adding ground plane...")
    scene.add_entity(gs.morphs.Plane())

    # 座標軸マーカー
    print("\n[4] Adding axis markers...")
    scene.add_entity(
        gs.morphs.Box(size=(0.3, 0.01, 0.01), pos=(0.15, 0, 0.005), fixed=True),
    )
    scene.add_entity(
        gs.morphs.Box(size=(0.01, 0.3, 0.01), pos=(0, 0.15, 0.005), fixed=True),
    )
    scene.add_entity(
        gs.morphs.Box(size=(0.01, 0.01, 0.3), pos=(0, 0, 0.15), fixed=True),
    )

    # StampFlyパーツ読み込み（自由落下用）
    print("\n[5] Loading StampFly parts for free fall...")
    print("    Coordinate transform: euler=(-90, 0, 0)")
    print("    Physical: fixed=False (will fall)")

    spawn_height = 1.0  # 1m height
    transform_euler = (-90, 0, 0)

    # StampFly物理パラメータ
    # StampFly physical parameters
    # 総質量約50g、各パーツに分配
    # Total mass ~50g, distributed to each part
    part_masses = {
        'frame': 0.015,        # 15g - メインフレーム
        'pcb': 0.008,          # 8g - PCB基板
        'battery': 0.012,      # 12g - バッテリー
        'battery_adapter': 0.003,  # 3g
        'm5stamps3': 0.004,    # 4g - M5StampS3
        'motor_fl': 0.002,     # 2g each motor
        'motor_fr': 0.002,
        'motor_rl': 0.002,
        'motor_rr': 0.002,
        'propeller_fl': 0.0005,  # 0.5g each propeller
        'propeller_fr': 0.0005,
        'propeller_rl': 0.0005,
        'propeller_rr': 0.0005,
    }

    loaded_parts = []
    total_mass = 0

    for part in parts:
        part_file = assets_dir / part['file']
        if not part_file.exists():
            print(f"    WARNING: {part['file']} not found, skipping")
            continue

        part_name = part['name']
        mass = part_masses.get(part_name, 0.001)  # デフォルト1g
        total_mass += mass

        try:
            mesh = scene.add_entity(
                gs.morphs.Mesh(
                    file=str(part_file),
                    scale=0.001,
                    pos=(0, 0, spawn_height),
                    euler=transform_euler,
                    fixed=False,  # 自由落下を許可
                ),
                material=gs.materials.Rigid(
                    rho=1000,  # 密度 (kg/m³) - メッシュから質量計算
                ),
            )
            loaded_parts.append((part_name, mesh, mass))
            print(f"    Loaded: {part_name} (mass: {mass*1000:.1f}g)")
        except Exception as e:
            print(f"    ERROR loading {part_name}: {e}")

    print(f"\n    Total: {len(loaded_parts)} parts")
    print(f"    Total mass: {total_mass*1000:.1f}g")

    # シーンビルド
    print("\n[6] Building scene...")
    scene.build()
    print("    -> Scene built!")

    # 自由落下情報
    print("\n" + "=" * 60)
    print("Free Fall Test:")
    print("=" * 60)
    print(f"  Spawn height: {spawn_height}m")
    print(f"  Expected fall time: {(2*spawn_height/9.81)**0.5:.2f}s")
    print("")
    print("  Expected behavior:")
    print("    - All parts should fall together")
    print("    - Drone should maintain upright orientation")
    print("    - Parts will separate on ground impact")
    print("      (because they are separate rigid bodies)")
    print("")
    print("  Note: In a real drone simulation, parts should")
    print("  be combined into a single rigid body.")
    print("=" * 60)

    # シミュレーション実行
    print("\n[7] Running free fall simulation for 5 seconds...")

    try:
        for i in range(300):
            scene.step()
            if i % 60 == 0:
                print(f"    Time {i/60:.1f}s")
    except KeyboardInterrupt:
        print("\n    Interrupted by user")

    print("\n" + "=" * 60)
    print("Test completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
