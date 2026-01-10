#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analyze XYZ distribution comparing auto and manual parts
自動・手動パーツのXYZ分布を比較分析
"""

import os
import numpy as np
from stl import mesh

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARTS_DIR = os.path.join(SCRIPT_DIR, 'parts')


def get_centroids(stl_mesh):
    """Get centroids of all triangles"""
    centroids = []
    for tri in stl_mesh.vectors:
        centroids.append(np.mean(tri, axis=0))
    return np.array(centroids)


def print_stats(name, centroids):
    """Print statistics for a part"""
    print(f"\n{name}:")
    print(f"  Triangles: {len(centroids)}")
    print(f"  X: {centroids[:,0].min():7.2f} to {centroids[:,0].max():7.2f} mm")
    print(f"  Y: {centroids[:,1].min():7.2f} to {centroids[:,1].max():7.2f} mm")
    print(f"  Z: {centroids[:,2].min():7.2f} to {centroids[:,2].max():7.2f} mm")
    print(f"  X mean: {centroids[:,0].mean():7.2f} mm")
    print(f"  Y mean: {centroids[:,1].mean():7.2f} mm")


def main():
    print("=== XYZ Distribution Analysis ===")
    print("=== XYZ分布分析 ===\n")

    # Load auto part_00
    auto_path = os.path.join(PARTS_DIR, 'auto', 'part_00.stl')
    auto_mesh = mesh.Mesh.from_file(auto_path)
    auto_centroids = get_centroids(auto_mesh)
    print_stats("Auto part_00 (combined)", auto_centroids)

    # Load manual parts
    manual_parts = ['frame', 'm5stamps3', 'other']
    manual_data = {}

    for name in manual_parts:
        path = os.path.join(PARTS_DIR, 'manual', f'{name}.stl')
        if os.path.exists(path):
            m = mesh.Mesh.from_file(path)
            centroids = get_centroids(m)
            manual_data[name] = centroids
            print_stats(f"Manual {name}", centroids)

    # Find distinguishing features
    print("\n" + "="*50)
    print("Key Observations / 重要な観察点:")
    print("="*50)

    if 'm5stamps3' in manual_data and 'frame' in manual_data:
        m5_x_mean = manual_data['m5stamps3'][:,0].mean()
        frame_x_mean = manual_data['frame'][:,0].mean()
        print(f"\nm5stamps3 X mean: {m5_x_mean:.2f} mm")
        print(f"frame X mean: {frame_x_mean:.2f} mm")

        if m5_x_mean < frame_x_mean - 10:
            print(f"→ m5stamps3 is on the NEGATIVE X side (rear)")
        elif m5_x_mean > frame_x_mean + 10:
            print(f"→ m5stamps3 is on the POSITIVE X side (front)")

    if 'other' in manual_data:
        other_x = manual_data['other'][:,0]
        other_y = manual_data['other'][:,1]
        print(f"\n'other' (PCB) distribution:")
        print(f"  X: {other_x.min():.2f} to {other_x.max():.2f}, mean={other_x.mean():.2f}")
        print(f"  Y: {other_y.min():.2f} to {other_y.max():.2f}, mean={other_y.mean():.2f}")

    # Suggest split strategy
    print("\n" + "="*50)
    print("Recommended Split Strategy / 推奨分割戦略:")
    print("="*50)

    if 'm5stamps3' in manual_data:
        m5_x_min = manual_data['m5stamps3'][:,0].min()
        m5_x_max = manual_data['m5stamps3'][:,0].max()
        print(f"\n1. m5stamps3 can be isolated by X range:")
        print(f"   X ∈ [{m5_x_min:.1f}, {m5_x_max:.1f}]")

    if 'other' in manual_data and 'frame' in manual_data:
        # Check Y distribution
        frame_y_range = manual_data['frame'][:,1].max() - manual_data['frame'][:,1].min()
        other_y_range = manual_data['other'][:,1].max() - manual_data['other'][:,1].min()
        print(f"\n2. frame Y span: {frame_y_range:.1f} mm")
        print(f"   other Y span: {other_y_range:.1f} mm")


if __name__ == '__main__':
    main()
