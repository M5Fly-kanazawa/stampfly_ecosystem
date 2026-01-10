#!/usr/bin/env python3
"""
10_single_body_freefall.py - 単一剛体自由落下テスト
Single rigid body free fall test

目的/Purpose:
- フレームのみを単一剛体として自由落下
- Free fall of frame only as single rigid body
- シンプルな構成で物理シミュレーションを確認
- Verify physics simulation with simple configuration

注意/Note:
- 複数パーツの衝突は複雑なので、まず単一メッシュでテスト
- Multiple parts collision is complex, so test with single mesh first
"""

import genesis as gs
from pathlib import Path


def main():
    print("=" * 60)
    print("Genesis Single Body Free Fall Test")
    print("=" * 60)

    # STLファイルパス
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    frame_file = assets_dir / "frame.stl"

    if not frame_file.exists():
        print(f"ERROR: Frame STL not found: {frame_file}")
        return

    # Genesis初期化
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成（小さいタイムステップで安定性向上）
    print("\n[2] Creating scene...")
    scene = gs.Scene(
        show_viewer=True,
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -9.81),
            dt=1/120,  # 120Hz for stability
        ),
    )

    # 地面追加
    print("\n[3] Adding ground plane...")
    scene.add_entity(gs.morphs.Plane())

    # 座標軸マーカー
    print("\n[4] Adding axis markers...")
    # X軸（赤）
    scene.add_entity(
        gs.morphs.Box(size=(0.3, 0.01, 0.01), pos=(0.15, 0, 0.005), fixed=True),
    )
    # Y軸（緑）
    scene.add_entity(
        gs.morphs.Box(size=(0.01, 0.3, 0.01), pos=(0, 0.15, 0.005), fixed=True),
    )
    # Z軸（青）
    scene.add_entity(
        gs.morphs.Box(size=(0.01, 0.01, 0.3), pos=(0, 0, 0.15), fixed=True),
    )

    # フレーム読み込み（単一剛体）
    print("\n[5] Loading StampFly frame as single rigid body...")
    print("    Coordinate transform: euler=(-90, 0, 0)")
    print("    Mass: 50g (0.05kg)")

    spawn_height = 0.5  # 50cm

    frame = scene.add_entity(
        gs.morphs.Mesh(
            file=str(frame_file),
            scale=0.001,  # mm → m
            pos=(0, 0, spawn_height),
            euler=(-90, 0, 0),  # WebGL → Genesis
            fixed=False,
        ),
        material=gs.materials.Rigid(
            rho=500,  # 密度調整（軽い素材）
        ),
    )
    print(f"    -> Frame loaded at height {spawn_height}m")

    # シーンビルド
    print("\n[6] Building scene...")
    scene.build()
    print("    -> Scene built!")

    # 自由落下情報
    print("\n" + "=" * 60)
    print("Free Fall Test:")
    print("=" * 60)
    print(f"  Spawn height: {spawn_height}m")
    print(f"  Simulation dt: 1/120s (120Hz)")
    print(f"  Expected fall time: {(2*spawn_height/9.81)**0.5:.2f}s")
    print("")
    print("  Expected orientation in Genesis:")
    print("    - Top of frame: +Z (up)")
    print("    - Front of frame: +Y (forward)")
    print("    - Right of frame: +X (right)")
    print("=" * 60)

    # シミュレーション実行
    print("\n[7] Running free fall simulation for 5 seconds...")
    print("    (120Hz simulation, 60FPS display)")

    try:
        for i in range(600):  # 5 seconds at 120Hz
            scene.step()
            if i % 120 == 0:
                print(f"    Time {i/120:.1f}s")
    except KeyboardInterrupt:
        print("\n    Interrupted by user")

    print("\n" + "=" * 60)
    print("Test completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
