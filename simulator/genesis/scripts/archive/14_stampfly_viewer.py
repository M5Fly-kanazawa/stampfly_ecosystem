#!/usr/bin/env python3
"""
14_stampfly_viewer.py - StampFly全体ビューア
StampFly complete model viewer

目的/Purpose:
- StampFly全パーツを読み込んで表示
- Load and display all StampFly parts
- インタラクティブに回転・ズーム可能
- Interactive rotation and zoom

操作/Controls:
- マウスドラッグ: 回転 / Mouse drag: Rotate
- スクロール: ズーム / Scroll: Zoom
- Q/ESC: 終了 / Exit
"""

import genesis as gs
from pathlib import Path
import json


def main():
    print("=" * 60)
    print("StampFly Complete Model Viewer")
    print("=" * 60)

    # パーツ設定ファイルパス
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    config_file = assets_dir / "parts_config.json"

    # パーツ設定読み込み
    print("\n[1] Loading parts config...")
    with open(config_file, 'r') as f:
        config = json.load(f)
    parts = config.get('parts', [])
    print(f"    Found {len(parts)} parts")

    # Genesis初期化
    print("\n[2] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成（物理なし、ビューアのみ）
    print("\n[3] Creating scene...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0.08, 0.08, 0.06),  # カメラを近く / Close camera
            camera_lookat=(0, 0, 0.015),    # 機体中心を見る / Look at center
            camera_fov=45,
            max_FPS=60,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, 0),  # 重力なし / No gravity
            dt=1/60,
        ),
    )

    # 座標軸ヘルパー（小さいボックスで軸を表現）
    print("\n[4] Adding axis helpers...")
    axis_size = 0.002
    axis_length = 0.03

    # X軸（赤）
    scene.add_entity(
        gs.morphs.Box(
            size=(axis_length, axis_size, axis_size),
            pos=(axis_length/2, 0, 0),
            fixed=True,
        ),
        surface=gs.surfaces.Default(color=(1, 0.2, 0.2)),
    )
    # Y軸（緑）
    scene.add_entity(
        gs.morphs.Box(
            size=(axis_size, axis_length, axis_size),
            pos=(0, axis_length/2, 0),
            fixed=True,
        ),
        surface=gs.surfaces.Default(color=(0.2, 1, 0.2)),
    )
    # Z軸（青）
    scene.add_entity(
        gs.morphs.Box(
            size=(axis_size, axis_size, axis_length),
            pos=(0, 0, axis_length/2),
            fixed=True,
        ),
        surface=gs.surfaces.Default(color=(0.2, 0.5, 1)),
    )

    # 全パーツ読み込み
    print("\n[5] Loading all StampFly parts...")
    loaded_count = 0
    for part in parts:
        part_file = assets_dir / part['file']
        if not part_file.exists():
            print(f"    WARNING: {part['file']} not found")
            continue

        # 色を取得
        color = part.get('color', [0.5, 0.5, 0.5])
        opacity = part.get('opacity', 1.0)

        try:
            # 座標系変換: WebGL(STL) → Genesis
            # WebGL: X=左, Y=上, Z=前
            # Genesis: X=右, Y=前, Z=上
            # 変換行列: Genesis = [-x, z, y] (WebGLベース)
            # euler=(-90, 180, 0): X軸-90度 → Y軸180度
            mesh = scene.add_entity(
                gs.morphs.Mesh(
                    file=str(part_file),
                    scale=(0.001, 0.001, 0.001),  # mm -> m
                    euler=(-90, 180, 0),  # WebGL→Genesis座標変換
                    pos=(0, 0, 0),
                    fixed=True,
                    convexify=False,
                ),
                surface=gs.surfaces.Default(
                    color=(color[0], color[1], color[2]),
                    opacity=opacity,  # 透明度を適用
                ),
            )
            loaded_count += 1
            print(f"    Loaded: {part['name']}")
        except Exception as e:
            print(f"    ERROR loading {part['name']}: {e}")

    print(f"\n    -> {loaded_count}/{len(parts)} parts loaded")

    # シーンビルド
    print("\n[6] Building scene...")
    scene.build()

    # ビューア表示
    print("\n" + "=" * 60)
    print("StampFly Viewer Ready!")
    print("=" * 60)
    print()
    print("Controls:")
    print("  - Mouse drag: Rotate view")
    print("  - Scroll: Zoom in/out")
    print("  - Q / ESC: Exit")
    print()
    print("=" * 60)

    # シミュレーションループ（ビューア用）
    try:
        while True:
            scene.step()
            if not scene.viewer.is_alive():
                break
    except (KeyboardInterrupt, Exception):
        pass

    print("\nViewer closed.")


if __name__ == "__main__":
    main()
