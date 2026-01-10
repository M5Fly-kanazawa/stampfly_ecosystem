#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STL Splitter Tool
STL分割ツール

Splits a single STL file into multiple parts using:
1. Automatic method: Connected component analysis
2. Manual method: Index-based splitting (fallback)

単一のSTLファイルを複数パーツに分割:
1. 自動方式: 連結成分分析
2. 手動方式: インデックスベース分割（フォールバック）
"""

import os
import sys
import numpy as np
from stl import mesh
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components
from collections import defaultdict

# Paths
# パス設定
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIMULATOR_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
ASSETS_DIR = os.path.join(SIMULATOR_DIR, 'assets', 'meshes')
OUTPUT_DIR = os.path.join(SCRIPT_DIR, 'parts')

# Source STL file
# 元STLファイル
SOURCE_STL = os.path.join(ASSETS_DIR, 'stampfly_v1.stl')

# Manual split configuration (from vpython_backend.py)
# 手動分割設定（vpython_backend.pyより）
MANUAL_PARTS = [
    {'name': 'frame',      'start': 0,    'end': 4520,  'color': (0.9, 0.9, 0.8)},
    {'name': 'motor1',     'start': 4520, 'end': 4730,  'color': (0.8, 1.0, 1.0)},
    {'name': 'propeller1', 'start': 4730, 'end': 5450,  'color': (1.0, 0.2, 0.2)},
    {'name': 'motor2',     'start': 5450, 'end': 5660,  'color': (0.8, 1.0, 1.0)},
    {'name': 'motor3_4',   'start': 5660, 'end': 6050,  'color': (0.8, 1.0, 1.0)},
    {'name': 'propeller2', 'start': 6050, 'end': 8120,  'color': (1.0, 0.2, 0.2)},
    {'name': 'm5stamps3',  'start': 8120, 'end': 8411,  'color': (0.9, 0.45, 0.0)},
    {'name': 'other',      'start': 8411, 'end': None,  'color': (0.2, 0.2, 0.2)},
]


def load_stl(filepath):
    """
    Load STL file.
    STLファイルを読み込む
    """
    print(f"Loading STL: {filepath}")
    stl_mesh = mesh.Mesh.from_file(filepath)
    print(f"  Triangles: {len(stl_mesh.vectors)}")
    return stl_mesh


def save_part_stl(vectors, normals, filepath):
    """
    Save triangles as a new STL file.
    三角形群を新しいSTLファイルとして保存
    """
    if len(vectors) == 0:
        print(f"  Warning: No triangles to save for {filepath}")
        return False

    # Create new mesh
    new_mesh = mesh.Mesh(np.zeros(len(vectors), dtype=mesh.Mesh.dtype))
    for i, (v, n) in enumerate(zip(vectors, normals)):
        new_mesh.vectors[i] = v
        new_mesh.normals[i] = n

    new_mesh.save(filepath)
    print(f"  Saved: {filepath} ({len(vectors)} triangles)")
    return True


def split_manual(stl_mesh, output_dir):
    """
    Split STL using manual index-based configuration.
    手動インデックスベースで分割
    """
    print("\n=== Manual Split (Index-based) ===")
    print("=== 手動分割（インデックスベース） ===\n")

    total = len(stl_mesh.vectors)
    parts_info = []

    for part in MANUAL_PARTS:
        name = part['name']
        start = part['start']
        end = part['end'] if part['end'] is not None else total
        color = part['color']

        # Extract triangles for this part
        vectors = stl_mesh.vectors[start:end]
        normals = stl_mesh.normals[start:end]

        filepath = os.path.join(output_dir, f"{name}.stl")
        if save_part_stl(vectors, normals, filepath):
            parts_info.append({
                'name': name,
                'file': f"{name}.stl",
                'triangles': len(vectors),
                'color': color
            })

    return parts_info


def split_automatic(stl_mesh, output_dir, tolerance=1e-6):
    """
    Split STL using connected component analysis.
    連結成分分析で自動分割

    Triangles sharing vertices (within tolerance) are grouped together.
    頂点を共有する三角形（許容誤差内）をグループ化
    """
    print("\n=== Automatic Split (Connected Components) ===")
    print("=== 自動分割（連結成分分析） ===\n")

    vectors = stl_mesh.vectors
    n_triangles = len(vectors)

    print(f"  Analyzing {n_triangles} triangles...")

    # Build vertex-to-triangle mapping
    # 頂点→三角形のマッピングを構築
    vertex_to_triangles = defaultdict(list)

    for tri_idx, tri in enumerate(vectors):
        for vertex in tri:
            # Round vertex coordinates to handle floating point precision
            # 浮動小数点精度のため座標を丸める
            key = tuple(np.round(vertex / tolerance).astype(int))
            vertex_to_triangles[key].append(tri_idx)

    print(f"  Unique vertices: {len(vertex_to_triangles)}")

    # Build adjacency matrix
    # 隣接行列を構築
    row_indices = []
    col_indices = []

    for tri_list in vertex_to_triangles.values():
        # All triangles sharing this vertex are connected
        for i, t1 in enumerate(tri_list):
            for t2 in tri_list[i+1:]:
                row_indices.extend([t1, t2])
                col_indices.extend([t2, t1])

    if len(row_indices) == 0:
        print("  Warning: No connected triangles found!")
        return []

    data = np.ones(len(row_indices), dtype=np.int8)
    adjacency = csr_matrix((data, (row_indices, col_indices)),
                          shape=(n_triangles, n_triangles))

    # Find connected components
    # 連結成分を検出
    n_components, labels = connected_components(adjacency, directed=False)
    print(f"  Found {n_components} connected components (parts)")

    # Group triangles by component
    # コンポーネントごとに三角形をグループ化
    parts_info = []

    # Generate colors for each part
    colors = generate_colors(n_components)

    for comp_id in range(n_components):
        mask = labels == comp_id
        part_vectors = vectors[mask]
        part_normals = stl_mesh.normals[mask]

        name = f"part_{comp_id:02d}"
        filepath = os.path.join(output_dir, f"{name}.stl")

        if save_part_stl(part_vectors, part_normals, filepath):
            parts_info.append({
                'name': name,
                'file': f"{name}.stl",
                'triangles': len(part_vectors),
                'color': colors[comp_id],
                'indices': np.where(mask)[0].tolist()
            })

    # Sort by first triangle index (to match original order)
    # 元の順序に合わせてソート
    parts_info.sort(key=lambda x: x['indices'][0] if x['indices'] else 0)

    return parts_info


def generate_colors(n):
    """
    Generate n distinct colors.
    n個の異なる色を生成
    """
    colors = []
    for i in range(n):
        hue = i / n
        # HSV to RGB (simplified, saturation=0.7, value=0.9)
        h = hue * 6
        c = 0.9 * 0.7
        x = c * (1 - abs(h % 2 - 1))
        m = 0.9 - c

        if h < 1:
            r, g, b = c, x, 0
        elif h < 2:
            r, g, b = x, c, 0
        elif h < 3:
            r, g, b = 0, c, x
        elif h < 4:
            r, g, b = 0, x, c
        elif h < 5:
            r, g, b = x, 0, c
        else:
            r, g, b = c, 0, x

        colors.append((round(r + m, 2), round(g + m, 2), round(b + m, 2)))

    return colors


def save_parts_config(parts_info, output_dir, method):
    """
    Save parts configuration as JSON.
    パーツ設定をJSONとして保存
    """
    import json

    config = {
        'method': method,
        'parts': [{
            'name': p['name'],
            'file': p['file'],
            'triangles': p['triangles'],
            'color': p['color']
        } for p in parts_info]
    }

    config_path = os.path.join(output_dir, 'parts_config.json')
    with open(config_path, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)

    print(f"\nConfiguration saved: {config_path}")
    return config_path


def main():
    """Main function / メイン関数"""
    import argparse

    parser = argparse.ArgumentParser(
        description='Split STL file into parts / STLファイルをパーツに分割'
    )
    parser.add_argument(
        '--method', '-m',
        choices=['auto', 'manual', 'both'],
        default='both',
        help='Split method: auto (connected components), manual (index-based), or both'
    )
    parser.add_argument(
        '--input', '-i',
        default=SOURCE_STL,
        help='Input STL file path'
    )
    parser.add_argument(
        '--output', '-o',
        default=OUTPUT_DIR,
        help='Output directory for split parts'
    )

    args = parser.parse_args()

    # Ensure output directory exists
    os.makedirs(args.output, exist_ok=True)

    # Load STL
    stl_mesh = load_stl(args.input)

    results = {}

    # Run splitting
    if args.method in ['auto', 'both']:
        auto_dir = os.path.join(args.output, 'auto') if args.method == 'both' else args.output
        os.makedirs(auto_dir, exist_ok=True)
        auto_parts = split_automatic(stl_mesh, auto_dir)
        save_parts_config(auto_parts, auto_dir, 'automatic')
        results['auto'] = auto_parts

    if args.method in ['manual', 'both']:
        manual_dir = os.path.join(args.output, 'manual') if args.method == 'both' else args.output
        os.makedirs(manual_dir, exist_ok=True)
        manual_parts = split_manual(stl_mesh, manual_dir)
        save_parts_config(manual_parts, manual_dir, 'manual')
        results['manual'] = manual_parts

    # Summary
    print("\n" + "="*50)
    print("Summary / サマリー")
    print("="*50)

    for method, parts in results.items():
        print(f"\n{method.upper()}:")
        for p in parts:
            print(f"  {p['name']}: {p['triangles']} triangles")

    print("\nDone! / 完了!")
    return results


if __name__ == '__main__':
    main()
