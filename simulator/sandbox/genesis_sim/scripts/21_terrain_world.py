#!/usr/bin/env python3
"""
21_terrain_world.py - プロシージャル地形テスト
Procedural terrain test using Genesis Terrain

Genesis Terrainの機能:
  - ハイトマップベースの地形生成
  - 複数のサブ地形タイプ（山、階段、波など）
  - horizontal_scale: 水平方向の解像度
  - vertical_scale: 高さスケール

操作:
  - Q: 終了
  - R: リセット
"""

import sys
from pathlib import Path

script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir.parent))

import genesis as gs
import numpy as np
import time
import pygame


def main():
    print("=" * 60)
    print("Procedural Terrain World Test")
    print("=" * 60)

    # pygame初期化（キーボード入力用）
    pygame.init()
    screen = pygame.display.set_mode((200, 100))
    pygame.display.set_caption("Terrain Test - Press Q to quit")

    # Genesis初期化
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # シーン作成
    print("\n[2] Creating scene...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(3.0, -3.0, 2.0),
            camera_lookat=(0, 0, 0),
            camera_fov=60,
            max_FPS=60,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -9.81),
            dt=1/240,
        ),
    )

    # プロシージャル地形を作成
    print("\n[3] Creating procedural terrain...")
    try:
        terrain = scene.add_entity(
            gs.morphs.Terrain(
                n_subterrains=(3, 3),
                subterrain_size=(4.0, 4.0),  # 各サブ地形のサイズ (m)
                subterrain_types=[
                    ['flat_terrain', 'random_uniform_terrain', 'pyramid_sloped_terrain'],
                    ['wave_terrain', 'discrete_obstacles_terrain', 'stairs_terrain'],
                    ['fractal_terrain', 'stepping_stones_terrain', 'flat_terrain'],
                ],
                horizontal_scale=0.1,  # 水平解像度 (m)
                vertical_scale=0.5,    # 高さスケール
                randomize=True,
            ),
        )
        print("  Terrain created successfully!")
    except Exception as e:
        print(f"  Terrain creation failed: {e}")
        print("  Falling back to simple height map...")

        # フォールバック: カスタムハイトマップ
        size = 64
        x = np.linspace(-1, 1, size)
        y = np.linspace(-1, 1, size)
        X, Y = np.meshgrid(x, y)
        height_map = 0.3 * np.sin(3 * X) * np.cos(3 * Y) + 0.1 * np.random.rand(size, size)

        terrain = scene.add_entity(
            gs.morphs.Terrain(
                height_field=height_map.astype(np.float32),
                horizontal_scale=0.2,
                vertical_scale=1.0,
            ),
        )

    # ドローンの代わりにテスト用の球体
    print("\n[4] Adding test sphere...")
    sphere = scene.add_entity(
        gs.morphs.Sphere(
            pos=(0, 0, 3.0),
            radius=0.1,
            fixed=False,
        ),
        surface=gs.surfaces.Default(color=(1, 0.5, 0)),
    )

    # シーンビルド
    print("\n[5] Building scene...")
    build_start = time.perf_counter()
    scene.build()
    print(f"    Build time: {time.perf_counter() - build_start:.1f}s")

    # シミュレーション実行
    print("\n[6] Running simulation...")
    print("    Press Q to quit, R to reset")

    running = True
    try:
        while running and scene.viewer.is_alive():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        print("\n>>> Exit (Q key)")
                        running = False
                    if event.key == pygame.K_r:
                        scene.reset()
                        print("\n>>> Reset")

            scene.step()

    except KeyboardInterrupt:
        pass

    print("\nSimulation ended.")
    pygame.quit()


if __name__ == "__main__":
    main()
