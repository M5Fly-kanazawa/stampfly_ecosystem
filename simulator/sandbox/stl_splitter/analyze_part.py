#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analyze STL part for potential sub-division
STLパーツの細分化可能性を分析

Analyzes Z-height distribution and spatial clustering to find
natural separation points.
Z高さ分布と空間クラスタリングで自然な分割点を探す
"""

import os
import numpy as np
from stl import mesh
from collections import Counter

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARTS_DIR = os.path.join(SCRIPT_DIR, 'parts')


def analyze_z_distribution(stl_mesh, name="part"):
    """
    Analyze Z-height distribution of triangles.
    三角形のZ高さ分布を分析
    """
    print(f"\n=== Z-Height Analysis for {name} ===")
    print(f"=== {name}のZ高さ分析 ===\n")

    # Get Z coordinates of all vertices
    all_z = []
    centroids_z = []

    for tri in stl_mesh.vectors:
        for vertex in tri:
            all_z.append(vertex[2])  # Z coordinate
        # Centroid Z
        centroid_z = np.mean([v[2] for v in tri])
        centroids_z.append(centroid_z)

    all_z = np.array(all_z)
    centroids_z = np.array(centroids_z)

    print(f"Vertex Z range: {all_z.min():.3f} to {all_z.max():.3f} mm")
    print(f"Centroid Z range: {centroids_z.min():.3f} to {centroids_z.max():.3f} mm")

    # Find Z histogram
    print("\nZ-height histogram (centroid):")
    hist, bin_edges = np.histogram(centroids_z, bins=20)
    for i, count in enumerate(hist):
        z_low = bin_edges[i]
        z_high = bin_edges[i+1]
        bar = '#' * (count // 20 + 1) if count > 0 else ''
        print(f"  {z_low:7.2f} - {z_high:7.2f}: {count:5d} {bar}")

    # Find gaps in Z distribution (potential split points)
    print("\nLooking for gaps in Z distribution...")
    sorted_z = np.sort(centroids_z)
    diffs = np.diff(sorted_z)
    threshold = np.mean(diffs) + 2 * np.std(diffs)

    gaps = []
    for i, diff in enumerate(diffs):
        if diff > threshold:
            gap_z = (sorted_z[i] + sorted_z[i+1]) / 2
            gaps.append((gap_z, diff))

    if gaps:
        print(f"Found {len(gaps)} potential gap(s):")
        for gap_z, gap_size in sorted(gaps):
            print(f"  Z = {gap_z:.2f} mm (gap size: {gap_size:.3f} mm)")
    else:
        print("No significant gaps found in Z distribution.")

    return centroids_z, gaps


def analyze_spatial_clusters(stl_mesh, name="part"):
    """
    Analyze spatial distribution using simple clustering.
    単純なクラスタリングで空間分布を分析
    """
    print(f"\n=== Spatial Cluster Analysis for {name} ===")
    print(f"=== {name}の空間クラスタ分析 ===\n")

    # Get centroids
    centroids = []
    for tri in stl_mesh.vectors:
        centroid = np.mean(tri, axis=0)
        centroids.append(centroid)
    centroids = np.array(centroids)

    print(f"X range: {centroids[:,0].min():.2f} to {centroids[:,0].max():.2f} mm")
    print(f"Y range: {centroids[:,1].min():.2f} to {centroids[:,1].max():.2f} mm")
    print(f"Z range: {centroids[:,2].min():.2f} to {centroids[:,2].max():.2f} mm")

    return centroids


def split_by_z_threshold(stl_mesh, z_thresholds, output_dir, base_name="part"):
    """
    Split mesh by Z-height thresholds.
    Z高さ閾値でメッシュを分割
    """
    print(f"\n=== Splitting by Z thresholds: {z_thresholds} ===")

    # Sort thresholds
    thresholds = sorted(z_thresholds)

    # Calculate centroid Z for each triangle
    centroids_z = []
    for tri in stl_mesh.vectors:
        centroid_z = np.mean([v[2] for v in tri])
        centroids_z.append(centroid_z)
    centroids_z = np.array(centroids_z)

    # Assign each triangle to a bin
    bins = np.digitize(centroids_z, thresholds)

    # Group triangles
    groups = {}
    for i, bin_idx in enumerate(bins):
        if bin_idx not in groups:
            groups[bin_idx] = []
        groups[bin_idx].append(i)

    # Save each group
    results = []
    for bin_idx in sorted(groups.keys()):
        indices = groups[bin_idx]
        vectors = stl_mesh.vectors[indices]
        normals = stl_mesh.normals[indices]

        # Create new mesh
        new_mesh = mesh.Mesh(np.zeros(len(vectors), dtype=mesh.Mesh.dtype))
        for i, (v, n) in enumerate(zip(vectors, normals)):
            new_mesh.vectors[i] = v
            new_mesh.normals[i] = n

        filename = f"{base_name}_z{bin_idx}.stl"
        filepath = os.path.join(output_dir, filename)
        new_mesh.save(filepath)

        z_min = centroids_z[indices].min()
        z_max = centroids_z[indices].max()
        print(f"  {filename}: {len(indices)} triangles (Z: {z_min:.2f} - {z_max:.2f})")

        results.append({
            'file': filename,
            'triangles': len(indices),
            'z_range': (z_min, z_max)
        })

    return results


def compare_with_manual(auto_mesh, manual_dir):
    """
    Compare auto part with manual parts to understand overlap.
    自動パーツと手動パーツを比較して重複を理解
    """
    print("\n=== Comparison with Manual Split ===")
    print("=== 手動分割との比較 ===\n")

    # Load manual parts that might be in the auto part
    manual_parts = ['frame', 'm5stamps3', 'other']

    auto_centroids = []
    for tri in auto_mesh.vectors:
        auto_centroids.append(np.mean(tri, axis=0))
    auto_centroids = np.array(auto_centroids)

    for part_name in manual_parts:
        part_path = os.path.join(manual_dir, f"{part_name}.stl")
        if not os.path.exists(part_path):
            continue

        manual_mesh = mesh.Mesh.from_file(part_path)
        manual_centroids = []
        for tri in manual_mesh.vectors:
            manual_centroids.append(np.mean(tri, axis=0))
        manual_centroids = np.array(manual_centroids)

        print(f"{part_name}:")
        print(f"  Triangles: {len(manual_mesh.vectors)}")
        print(f"  Z range: {manual_centroids[:,2].min():.2f} to {manual_centroids[:,2].max():.2f} mm")


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Analyze STL part for sub-division')
    parser.add_argument('--part', '-p', default='auto/part_00.stl',
                       help='Part file to analyze (relative to parts/)')
    parser.add_argument('--split-z', '-z', type=float, nargs='+',
                       help='Z thresholds for splitting')

    args = parser.parse_args()

    # Load part
    part_path = os.path.join(PARTS_DIR, args.part)
    print(f"Loading: {part_path}")
    stl_mesh = mesh.Mesh.from_file(part_path)
    print(f"Triangles: {len(stl_mesh.vectors)}")

    # Analyze
    centroids_z, gaps = analyze_z_distribution(stl_mesh, args.part)
    centroids = analyze_spatial_clusters(stl_mesh, args.part)

    # Compare with manual
    manual_dir = os.path.join(PARTS_DIR, 'manual')
    compare_with_manual(stl_mesh, manual_dir)

    # Split if thresholds provided
    if args.split_z:
        output_dir = os.path.join(PARTS_DIR, 'auto_subdivided')
        os.makedirs(output_dir, exist_ok=True)
        split_by_z_threshold(stl_mesh, args.split_z, output_dir, 'frame')


if __name__ == '__main__':
    main()
