#!/usr/bin/env python3
"""
22_voxel_world.py - Voxel構造物ワールド
Voxel structure world with rings, tunnels, and pillars

50m四方の空間にブロックでできた構造物（輪、トンネル、柱）を配置。
ドローン飛行用の障害物コースとして使用可能。

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
    'red': (0.8, 0.2, 0.2),
    'orange': (0.9, 0.5, 0.1),
    'yellow': (0.9, 0.8, 0.1),
    'green': (0.2, 0.7, 0.3),
    'cyan': (0.2, 0.7, 0.8),
    'blue': (0.2, 0.3, 0.8),
    'purple': (0.6, 0.2, 0.8),
    'white': (0.9, 0.9, 0.9),
    'gray': (0.5, 0.5, 0.5),
}


def add_block(scene, pos, size, color):
    """単一ブロックを追加"""
    scene.add_entity(
        gs.morphs.Box(
            size=(size, size, size),
            pos=pos,
            fixed=True,
            collision=False,
        ),
        surface=gs.surfaces.Default(color=color),
    )


def add_ring(scene, center, radius, block_size, color, axis='z'):
    """
    ブロックで輪を作成

    Args:
        center: 中心位置 (x, y, z)
        radius: 輪の半径 (m)
        block_size: ブロックサイズ (m)
        color: ブロック色
        axis: 輪の軸 ('x', 'y', 'z')
    """
    cx, cy, cz = center
    num_blocks = int(2 * np.pi * radius / block_size)
    blocks = 0

    for i in range(num_blocks):
        angle = 2 * np.pi * i / num_blocks

        if axis == 'z':
            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            z = cz
        elif axis == 'x':
            x = cx
            y = cy + radius * np.cos(angle)
            z = cz + radius * np.sin(angle)
        else:  # axis == 'y'
            x = cx + radius * np.cos(angle)
            y = cy
            z = cz + radius * np.sin(angle)

        add_block(scene, (x, y, z), block_size, color)
        blocks += 1

    return blocks


def add_pillar(scene, base_pos, height_blocks, block_size, color):
    """
    柱を作成

    Args:
        base_pos: 底面中心位置 (x, y, z)
        height_blocks: 高さ（ブロック数）
        block_size: ブロックサイズ (m)
        color: ブロック色
    """
    bx, by, bz = base_pos
    blocks = 0

    for i in range(height_blocks):
        z = bz + i * block_size + block_size / 2
        add_block(scene, (bx, by, z), block_size, color)
        blocks += 1

    return blocks


def add_tunnel(scene, start_pos, length_blocks, block_size, color, direction='x'):
    """
    トンネル（アーチ）を作成

    Args:
        start_pos: 開始位置 (x, y, z)
        length_blocks: 長さ（ブロック数）
        block_size: ブロックサイズ (m)
        color: ブロック色
        direction: トンネル方向 ('x' or 'y')
    """
    sx, sy, sz = start_pos
    blocks = 0

    # アーチの断面（半円 + 柱）
    arch_positions = []
    radius = 2  # ブロック単位
    for angle in np.linspace(0, np.pi, 8):
        dx = radius * np.cos(angle)
        dz = radius * np.sin(angle) + radius
        arch_positions.append((dx, dz))

    # 左右の柱
    for i in range(radius):
        arch_positions.append((-radius, i))
        arch_positions.append((radius, i))

    for i in range(length_blocks):
        if direction == 'x':
            px = sx + i * block_size
            for dx, dz in arch_positions:
                add_block(scene, (px, sy + dx * block_size, sz + dz * block_size), block_size, color)
                blocks += 1
        else:  # direction == 'y'
            py = sy + i * block_size
            for dx, dz in arch_positions:
                add_block(scene, (sx + dx * block_size, py, sz + dz * block_size), block_size, color)
                blocks += 1

    return blocks


def add_arch(scene, center, width_blocks, height_blocks, block_size, color, direction='x'):
    """
    アーチ門を作成

    Args:
        center: 中心位置 (x, y, z)
        width_blocks: 幅（ブロック数）
        height_blocks: 高さ（ブロック数）
        block_size: ブロックサイズ (m)
        color: ブロック色
        direction: 門の向き ('x' or 'y')
    """
    cx, cy, cz = center
    blocks = 0
    half_w = width_blocks // 2

    # 左右の柱
    for i in range(height_blocks):
        z = cz + i * block_size + block_size / 2
        if direction == 'x':
            add_block(scene, (cx, cy - half_w * block_size, z), block_size, color)
            add_block(scene, (cx, cy + half_w * block_size, z), block_size, color)
        else:
            add_block(scene, (cx - half_w * block_size, cy, z), block_size, color)
            add_block(scene, (cx + half_w * block_size, cy, z), block_size, color)
        blocks += 2

    # 上部のアーチ
    top_z = cz + height_blocks * block_size + block_size / 2
    for i in range(-half_w, half_w + 1):
        if direction == 'x':
            add_block(scene, (cx, cy + i * block_size, top_z), block_size, color)
        else:
            add_block(scene, (cx + i * block_size, cy, top_z), block_size, color)
        blocks += 1

    return blocks


def main():
    print("=" * 60)
    print("Voxel Structure World")
    print("50m x 50m with rings, tunnels, and pillars")
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

    # シーン作成
    print("\n[2] Creating scene...")
    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(40.0, -40.0, 30.0),
            camera_lookat=(0, 0, 5.0),
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
        surface=gs.surfaces.Default(color=(0.15, 0.15, 0.15)),
    )

    total_blocks = 0

    # 構造物を配置
    print("\n[4] Creating voxel structures...")

    # 輪（リング）- ドローンがくぐれる
    print("    Adding rings...")
    rings = [
        # (center, radius, color, axis)
        ((0, 0, 5), 3.0, COLORS['red'], 'y'),
        ((10, 10, 4), 2.5, COLORS['orange'], 'x'),
        ((-10, 8, 6), 2.0, COLORS['yellow'], 'y'),
        ((15, -12, 5), 3.0, COLORS['green'], 'x'),
        ((-15, -10, 4), 2.5, COLORS['cyan'], 'y'),
        ((0, 20, 7), 2.0, COLORS['blue'], 'x'),
        ((-20, 0, 5), 3.0, COLORS['purple'], 'y'),
    ]
    for center, radius, color, axis in rings:
        total_blocks += add_ring(scene, center, radius, BLOCK_SIZE, color, axis)

    # 柱
    print("    Adding pillars...")
    pillars = [
        # (base_pos, height_blocks, color)
        ((5, -5, 0), 16, COLORS['gray']),
        ((-5, -5, 0), 12, COLORS['white']),
        ((5, 5, 0), 10, COLORS['gray']),
        ((-5, 5, 0), 14, COLORS['white']),
        ((18, 0, 0), 20, COLORS['red']),
        ((-18, 0, 0), 18, COLORS['blue']),
        ((0, 18, 0), 16, COLORS['green']),
        ((0, -18, 0), 14, COLORS['yellow']),
        ((12, 12, 0), 8, COLORS['cyan']),
        ((-12, 12, 0), 10, COLORS['orange']),
        ((12, -12, 0), 12, COLORS['purple']),
        ((-12, -12, 0), 6, COLORS['white']),
    ]
    for base_pos, height, color in pillars:
        total_blocks += add_pillar(scene, base_pos, height, BLOCK_SIZE, color)

    # アーチ門
    print("    Adding arches...")
    arches = [
        # (center, width_blocks, height_blocks, color, direction)
        ((8, 0, 0), 6, 8, COLORS['orange'], 'x'),
        ((-8, 0, 0), 8, 10, COLORS['cyan'], 'x'),
        ((0, 8, 0), 6, 6, COLORS['green'], 'y'),
        ((0, -8, 0), 8, 8, COLORS['purple'], 'y'),
        ((20, 15, 0), 6, 12, COLORS['red'], 'x'),
        ((-20, -15, 0), 8, 10, COLORS['blue'], 'y'),
    ]
    for center, width, height, color, direction in arches:
        total_blocks += add_arch(scene, center, width, height, BLOCK_SIZE, color, direction)

    # トンネル
    print("    Adding tunnels...")
    tunnels = [
        # (start_pos, length_blocks, color, direction)
        ((-15, 15, 0), 8, COLORS['gray'], 'x'),
        ((15, -15, 0), 10, COLORS['white'], 'y'),
    ]
    for start_pos, length, color, direction in tunnels:
        total_blocks += add_tunnel(scene, start_pos, length, BLOCK_SIZE, color, direction)

    print(f"    Total blocks: {total_blocks}")

    # テスト用の球体（ドローン代わり）
    print("\n[5] Adding test sphere...")
    sphere = scene.add_entity(
        gs.morphs.Sphere(
            pos=(0, 0, 10.0),
            radius=0.3,
            fixed=False,
        ),
        surface=gs.surfaces.Default(color=(1, 0.3, 0)),
    )

    # シーンビルド
    print("\n[6] Building scene...")
    build_start = time.perf_counter()
    scene.build()
    print(f"    Build time: {time.perf_counter() - build_start:.1f}s")

    # シミュレーション実行
    print("\n[7] Running simulation...")
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
