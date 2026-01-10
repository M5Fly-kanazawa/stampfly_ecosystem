#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Normal-based STL Analysis
法線ベースのSTL分析

Analyze triangle normals to find natural groupings.
三角形の法線を分析して自然なグループを検出。
"""

import os
import numpy as np
from stl import mesh
from sklearn.cluster import KMeans
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARTS_DIR = os.path.join(SCRIPT_DIR, 'parts')


def analyze_normals(stl_mesh):
    """
    Analyze surface normal distribution.
    表面法線の分布を分析
    """
    print("\n=== Normal Analysis ===")

    normals = stl_mesh.normals
    # Normalize (in case they're not unit vectors)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms[norms == 0] = 1  # Avoid division by zero
    normals_unit = normals / norms

    # Check dominant directions
    print("\nNormal direction statistics:")
    print(f"  X component: mean={normals_unit[:,0].mean():.3f}, std={normals_unit[:,0].std():.3f}")
    print(f"  Y component: mean={normals_unit[:,1].mean():.3f}, std={normals_unit[:,1].std():.3f}")
    print(f"  Z component: mean={normals_unit[:,2].mean():.3f}, std={normals_unit[:,2].std():.3f}")

    # Count triangles facing each direction
    x_pos = (normals_unit[:,0] > 0.5).sum()
    x_neg = (normals_unit[:,0] < -0.5).sum()
    y_pos = (normals_unit[:,1] > 0.5).sum()
    y_neg = (normals_unit[:,1] < -0.5).sum()
    z_pos = (normals_unit[:,2] > 0.5).sum()
    z_neg = (normals_unit[:,2] < -0.5).sum()

    print(f"\nTriangles facing each direction (threshold > 0.5):")
    print(f"  +X: {x_pos}, -X: {x_neg}")
    print(f"  +Y: {y_pos}, -Y: {y_neg}")
    print(f"  +Z: {z_pos}, -Z: {z_neg}")

    return normals_unit


def combined_feature_analysis(stl_mesh):
    """
    Analyze using combined position + normal features.
    位置＋法線の組み合わせ特徴で分析
    """
    print("\n=== Combined Feature Analysis (Position + Normal) ===")

    # Get centroids
    centroids = np.array([np.mean(tri, axis=0) for tri in stl_mesh.vectors])

    # Get normals
    normals = stl_mesh.normals
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms[norms == 0] = 1
    normals_unit = normals / norms

    # Combine features with different weights
    # Position scaled to similar magnitude as normals
    pos_scale = 0.02  # 1/50 to bring mm scale to ~normal scale
    features = np.hstack([
        centroids * pos_scale,  # x, y, z (scaled)
        normals_unit            # nx, ny, nz
    ])

    print(f"Feature shape: {features.shape}")

    # Try K-means with different k
    for k in [3, 4, 5]:
        kmeans = KMeans(n_clusters=k, random_state=42, n_init=10)
        labels = kmeans.fit_predict(features)

        print(f"\nK-means (k={k}) clusters:")
        for i in range(k):
            mask = labels == i
            c = centroids[mask]
            n = normals_unit[mask]
            print(f"  Cluster {i}: {mask.sum()} triangles")
            print(f"    Position - X: [{c[:,0].min():.1f}, {c[:,0].max():.1f}], Y: [{c[:,1].min():.1f}, {c[:,1].max():.1f}]")
            print(f"    Normal   - X: {n[:,0].mean():.2f}, Y: {n[:,1].mean():.2f}, Z: {n[:,2].mean():.2f}")


def analyze_bounding_boxes(stl_mesh):
    """
    Check for distinct bounding boxes in different regions.
    異なる領域の独立したバウンディングボックスを確認
    """
    print("\n=== Bounding Box Analysis ===")

    centroids = np.array([np.mean(tri, axis=0) for tri in stl_mesh.vectors])

    # Split by Y coordinate to check m5stamps3 region
    y_threshold = 7.0
    high_y_mask = centroids[:, 1] > y_threshold

    print(f"\nRegion: Y > {y_threshold} (potential m5stamps3)")
    if high_y_mask.sum() > 0:
        region = centroids[high_y_mask]
        print(f"  Triangles: {high_y_mask.sum()}")
        print(f"  X: [{region[:,0].min():.1f}, {region[:,0].max():.1f}]")
        print(f"  Y: [{region[:,1].min():.1f}, {region[:,1].max():.1f}]")
        print(f"  Z: [{region[:,2].min():.1f}, {region[:,2].max():.1f}]")

    # Check if we can identify m5stamps3 by X position too
    # m5stamps3 manual data: X mean = -13.08, X range = [-34.7, 13.4]
    x_range_mask = (centroids[:, 0] > -35) & (centroids[:, 0] < 14) & (centroids[:, 1] > 2.8)
    print(f"\nRegion: -35 < X < 14 and Y > 2.8 (refined m5stamps3 region)")
    if x_range_mask.sum() > 0:
        region = centroids[x_range_mask]
        print(f"  Triangles: {x_range_mask.sum()}")
        print(f"  X: [{region[:,0].min():.1f}, {region[:,0].max():.1f}]")
        print(f"  Y: [{region[:,1].min():.1f}, {region[:,1].max():.1f}]")
        print(f"  Z: [{region[:,2].min():.1f}, {region[:,2].max():.1f}]")

    # Check PCB region (positive X, middle Y)
    pcb_mask = (centroids[:, 0] > 0) & (centroids[:, 1] > -9) & (centroids[:, 1] < 14) & (np.abs(centroids[:, 2]) < 21)
    print(f"\nRegion: X > 0 and -9 < Y < 14 and |Z| < 21 (potential PCB)")
    if pcb_mask.sum() > 0:
        region = centroids[pcb_mask]
        print(f"  Triangles: {pcb_mask.sum()}")
        print(f"  X: [{region[:,0].min():.1f}, {region[:,0].max():.1f}]")
        print(f"  Y: [{region[:,1].min():.1f}, {region[:,1].max():.1f}]")
        print(f"  Z: [{region[:,2].min():.1f}, {region[:,2].max():.1f}]")


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Normal-based STL analysis')
    parser.add_argument('--input', '-i', default=os.path.join(PARTS_DIR, 'auto', 'part_00.stl'),
                       help='Input STL file')

    args = parser.parse_args()

    print(f"Loading: {args.input}")
    stl_mesh = mesh.Mesh.from_file(args.input)
    print(f"Triangles: {len(stl_mesh.vectors)}")

    analyze_normals(stl_mesh)
    analyze_bounding_boxes(stl_mesh)
    combined_feature_analysis(stl_mesh)


if __name__ == '__main__':
    main()
