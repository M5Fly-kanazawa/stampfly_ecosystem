#!/usr/bin/env python3
"""
22_voxel_world.py - Voxel風ブロックワールドテスト
Voxel-style block world using Box primitives

Box primitiveを積み上げてMinecraft風の背景を作成。
地形データから自動生成も可能。

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


# Voxelカラーパレット
COLORS = {
    'grass': (0.3, 0.7, 0.2),
    'dirt': (0.5, 0.35, 0.2),
    'stone': (0.5, 0.5, 0.5),
    'water': (0.2, 0.4, 0.8),
    'sand': (0.9, 0.85, 0.6),
    'wood': (0.6, 0.4, 0.2),
    'leaves': (0.2, 0.6, 0.1),
}


def generate_terrain_heightmap(width, depth, scale=1.0):
    """
    簡易地形ハイトマップを生成
    """
    x = np.linspace(0, 4 * np.pi, width)
    z = np.linspace(0, 4 * np.pi, depth)
    X, Z = np.meshgrid(x, z)

    # パーリンノイズ風の地形
    height = (
        np.sin(X * 0.5) * np.cos(Z * 0.5) * 2 +
        np.sin(X * 1.5 + 1) * np.cos(Z * 1.2) * 1 +
        np.random.rand(depth, width) * 0.5
    )
    height = (height - height.min()) / (height.max() - height.min())
    height = (height * scale).astype(int)

    return height


def add_voxel_terrain(scene, width=20, depth=20, max_height=5, block_size=0.5, offset=(0, 0, 0)):
    """
    Voxel地形をシーンに追加

    Args:
        width: X方向のブロック数
        depth: Y方向のブロック数
        max_height: 最大高さ（ブロック数）
        block_size: 1ブロックの1辺の長さ (m)
        offset: オフセット位置
    """
    print(f"    Generating {width}x{depth} voxel terrain (block size: {block_size}m)...")
    print(f"    World size: {width * block_size}m x {depth * block_size}m")

    # ハイトマップ生成
    heightmap = generate_terrain_heightmap(width, depth, scale=max_height)

    blocks_added = 0
    ox, oy, oz = offset

    for ix in range(width):
        for iz in range(depth):
            h = heightmap[iz, ix]

            # 各列にブロックを積む
            for iy in range(h + 1):
                # ブロック位置（隙間なく敷き詰め）
                x = ox + (ix - width / 2) * block_size + block_size / 2
                y = oy + (iz - depth / 2) * block_size + block_size / 2
                z = oz + iy * block_size + block_size / 2

                # ブロックタイプ決定
                if iy == h:
                    color = COLORS['grass']
                elif iy >= h - 1:
                    color = COLORS['dirt']
                else:
                    color = COLORS['stone']

                # ブロック追加（隙間なし）
                scene.add_entity(
                    gs.morphs.Box(
                        size=(block_size, block_size, block_size),
                        pos=(x, y, z),
                        fixed=True,
                        collision=False,  # 衝突無効（描画のみ）
                    ),
                    surface=gs.surfaces.Default(color=color),
                )
                blocks_added += 1

    print(f"    Added {blocks_added} blocks")
    return blocks_added


def add_simple_tree(scene, pos, block_size=0.5):
    """
    簡単な木を追加

    Args:
        pos: 木の根元位置 (x, y, z)
        block_size: ブロックサイズ (m)
    """
    x, y, z = pos

    # 幹（4ブロック）
    for i in range(4):
        scene.add_entity(
            gs.morphs.Box(
                size=(block_size * 0.5, block_size * 0.5, block_size),
                pos=(x, y, z + i * block_size + block_size / 2),
                fixed=True,
                collision=False,
            ),
            surface=gs.surfaces.Default(color=COLORS['wood']),
        )

    # 葉（簡易的な十字形 + 上）
    leaf_positions = [
        (0, 0, 4), (1, 0, 4), (-1, 0, 4), (0, 1, 4), (0, -1, 4),
        (0, 0, 5), (1, 0, 5), (-1, 0, 5), (0, 1, 5), (0, -1, 5),
        (0, 0, 6),
    ]
    for dx, dy, dz in leaf_positions:
        scene.add_entity(
            gs.morphs.Box(
                size=(block_size, block_size, block_size),
                pos=(x + dx * block_size, y + dy * block_size, z + dz * block_size + block_size / 2),
                fixed=True,
                collision=False,
            ),
            surface=gs.surfaces.Default(color=COLORS['leaves']),
        )


def main():
    print("=" * 60)
    print("Voxel Block World Test")
    print("=" * 60)

    # pygame初期化
    pygame.init()
    screen = pygame.display.set_mode((200, 100))
    pygame.display.set_caption("Voxel World - Press Q to quit")

    # Genesis初期化
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # ワールド設定
    BLOCK_SIZE = 0.5      # 1ブロック = 50cm
    WORLD_SIZE = 50.0     # 50m四方
    GRID_SIZE = int(WORLD_SIZE / BLOCK_SIZE)  # 100x100グリッド

    # シーン作成
    print("\n[2] Creating scene...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(60.0, -60.0, 40.0),
            camera_lookat=(0, 0, 2.0),
            camera_fov=60,
            max_FPS=60,
        ),
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, -9.81),
            dt=1/240,
        ),
    )

    # 地面
    print("\n[3] Adding ground plane...")
    scene.add_entity(
        gs.morphs.Plane(collision=True),
        surface=gs.surfaces.Default(color=(0.2, 0.2, 0.2)),
    )

    # Voxel地形を追加
    print("\n[4] Creating voxel terrain...")
    print(f"    Block size: {BLOCK_SIZE}m, Grid: {GRID_SIZE}x{GRID_SIZE}")
    add_voxel_terrain(
        scene,
        width=GRID_SIZE,
        depth=GRID_SIZE,
        max_height=4,
        block_size=BLOCK_SIZE,
        offset=(0, 0, 0)
    )

    # 木を数本追加
    print("\n[5] Adding trees...")
    tree_positions = [
        (-15.0, -15.0, 1.5),
        (12.0, 18.0, 1.0),
        (-8.0, 12.0, 1.25),
        (15.0, -10.0, 1.0),
        (-20.0, 5.0, 1.5),
        (8.0, -18.0, 1.0),
        (0.0, 20.0, 1.25),
        (-5.0, -20.0, 1.0),
    ]
    for pos in tree_positions:
        add_simple_tree(scene, pos, BLOCK_SIZE)

    # テスト用の球体（ドローン代わり）
    print("\n[6] Adding test sphere...")
    sphere = scene.add_entity(
        gs.morphs.Sphere(
            pos=(0, 0, 10.0),
            radius=0.5,
            fixed=False,
        ),
        surface=gs.surfaces.Default(color=(1, 0.3, 0)),
    )

    # シーンビルド
    print("\n[7] Building scene...")
    build_start = time.perf_counter()
    scene.build()
    print(f"    Build time: {time.perf_counter() - build_start:.1f}s")

    # シミュレーション実行
    print("\n[8] Running simulation...")
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
