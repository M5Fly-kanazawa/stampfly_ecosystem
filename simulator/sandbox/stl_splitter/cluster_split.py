#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cluster-based STL Splitting
クラスタリングベースのSTL分割

Uses K-means and DBSCAN to find natural groupings in the mesh.
K-meansとDBSCANを使用してメッシュの自然なグループを検出。
"""

import os
import numpy as np
from stl import mesh
from sklearn.cluster import KMeans, DBSCAN
from sklearn.preprocessing import StandardScaler
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARTS_DIR = os.path.join(SCRIPT_DIR, 'parts')


def get_triangle_features(stl_mesh):
    """
    Extract features for each triangle.
    各三角形の特徴を抽出
    """
    features = []
    for tri in stl_mesh.vectors:
        # Centroid (x, y, z)
        centroid = np.mean(tri, axis=0)
        features.append(centroid)
    return np.array(features)


def split_by_kmeans(stl_mesh, n_clusters=3, output_dir=None):
    """
    Split mesh using K-means clustering.
    K-meansクラスタリングでメッシュを分割
    """
    print(f"\n=== K-means Clustering (k={n_clusters}) ===")

    features = get_triangle_features(stl_mesh)

    # Normalize features
    scaler = StandardScaler()
    features_scaled = scaler.fit_transform(features)

    # K-means
    kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init=10)
    labels = kmeans.fit_predict(features_scaled)

    # Analyze clusters
    print("\nCluster statistics:")
    for i in range(n_clusters):
        mask = labels == i
        cluster_features = features[mask]
        print(f"  Cluster {i}: {mask.sum()} triangles")
        print(f"    X: {cluster_features[:,0].min():.1f} to {cluster_features[:,0].max():.1f}, mean={cluster_features[:,0].mean():.1f}")
        print(f"    Y: {cluster_features[:,1].min():.1f} to {cluster_features[:,1].max():.1f}, mean={cluster_features[:,1].mean():.1f}")
        print(f"    Z: {cluster_features[:,2].min():.1f} to {cluster_features[:,2].max():.1f}, mean={cluster_features[:,2].mean():.1f}")

    if output_dir:
        save_clusters(stl_mesh, labels, n_clusters, output_dir, 'kmeans')

    return labels


def split_by_dbscan(stl_mesh, eps=5.0, min_samples=10, output_dir=None):
    """
    Split mesh using DBSCAN clustering.
    DBSCANクラスタリングでメッシュを分割

    DBSCAN finds clusters of arbitrary shape based on density.
    DBSCANは密度に基づいて任意形状のクラスタを検出。
    """
    print(f"\n=== DBSCAN Clustering (eps={eps}, min_samples={min_samples}) ===")

    features = get_triangle_features(stl_mesh)

    # DBSCAN (no need to scale for DBSCAN with eps in original units)
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    labels = dbscan.fit_predict(features)

    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise = (labels == -1).sum()

    print(f"\nFound {n_clusters} clusters, {n_noise} noise points")

    # Analyze clusters
    for i in range(n_clusters):
        mask = labels == i
        cluster_features = features[mask]
        print(f"  Cluster {i}: {mask.sum()} triangles")
        print(f"    X: {cluster_features[:,0].min():.1f} to {cluster_features[:,0].max():.1f}, mean={cluster_features[:,0].mean():.1f}")
        print(f"    Y: {cluster_features[:,1].min():.1f} to {cluster_features[:,1].max():.1f}, mean={cluster_features[:,1].mean():.1f}")
        print(f"    Z: {cluster_features[:,2].min():.1f} to {cluster_features[:,2].max():.1f}, mean={cluster_features[:,2].mean():.1f}")

    if n_noise > 0:
        print(f"  Noise: {n_noise} triangles")

    if output_dir:
        save_clusters(stl_mesh, labels, n_clusters, output_dir, 'dbscan', include_noise=True)

    return labels


def save_clusters(stl_mesh, labels, n_clusters, output_dir, method, include_noise=False):
    """
    Save clustered parts as STL files.
    クラスタ化されたパーツをSTLファイルとして保存
    """
    os.makedirs(output_dir, exist_ok=True)

    colors = generate_colors(n_clusters + (1 if include_noise else 0))
    parts_info = []

    unique_labels = sorted(set(labels))

    for label in unique_labels:
        if label == -1 and not include_noise:
            continue

        mask = labels == label
        indices = np.where(mask)[0]

        part_vectors = stl_mesh.vectors[indices]
        part_normals = stl_mesh.normals[indices]

        # Create new mesh
        new_mesh = mesh.Mesh(np.zeros(len(part_vectors), dtype=mesh.Mesh.dtype))
        for i, (v, n) in enumerate(zip(part_vectors, part_normals)):
            new_mesh.vectors[i] = v
            new_mesh.normals[i] = n

        name = 'noise' if label == -1 else f'cluster_{label:02d}'
        filename = f'{name}.stl'
        filepath = os.path.join(output_dir, filename)
        new_mesh.save(filepath)
        print(f"  Saved: {filename} ({len(indices)} triangles)")

        color_idx = label if label >= 0 else n_clusters
        parts_info.append({
            'name': name,
            'file': filename,
            'triangles': len(indices),
            'color': colors[color_idx] if color_idx < len(colors) else (0.5, 0.5, 0.5)
        })

    # Save config
    config = {'method': method, 'parts': parts_info}
    config_path = os.path.join(output_dir, 'parts_config.json')
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)
    print(f"  Config: {config_path}")

    return parts_info


def generate_colors(n):
    """Generate n distinct colors"""
    colors = []
    for i in range(n):
        hue = i / max(n, 1)
        h = hue * 6
        c = 0.9 * 0.7
        x = c * (1 - abs(h % 2 - 1))
        m = 0.9 - c

        if h < 1: r, g, b = c, x, 0
        elif h < 2: r, g, b = x, c, 0
        elif h < 3: r, g, b = 0, c, x
        elif h < 4: r, g, b = 0, x, c
        elif h < 5: r, g, b = x, 0, c
        else: r, g, b = c, 0, x

        colors.append((round(r + m, 2), round(g + m, 2), round(b + m, 2)))
    return colors


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Cluster-based STL splitting')
    parser.add_argument('--input', '-i', default=os.path.join(PARTS_DIR, 'auto', 'part_00.stl'),
                       help='Input STL file')
    parser.add_argument('--method', '-m', choices=['kmeans', 'dbscan', 'both'], default='both',
                       help='Clustering method')
    parser.add_argument('--k', type=int, default=3, help='Number of clusters for K-means')
    parser.add_argument('--eps', type=float, default=5.0, help='DBSCAN eps parameter (mm)')
    parser.add_argument('--min-samples', type=int, default=10, help='DBSCAN min_samples')
    parser.add_argument('--output', '-o', default=os.path.join(PARTS_DIR, 'cluster'),
                       help='Output directory')

    args = parser.parse_args()

    print(f"Loading: {args.input}")
    stl_mesh = mesh.Mesh.from_file(args.input)
    print(f"Triangles: {len(stl_mesh.vectors)}")

    if args.method in ['kmeans', 'both']:
        kmeans_dir = os.path.join(args.output, 'kmeans') if args.method == 'both' else args.output
        split_by_kmeans(stl_mesh, n_clusters=args.k, output_dir=kmeans_dir)

    if args.method in ['dbscan', 'both']:
        dbscan_dir = os.path.join(args.output, 'dbscan') if args.method == 'both' else args.output
        split_by_dbscan(stl_mesh, eps=args.eps, min_samples=args.min_samples, output_dir=dbscan_dir)

    print("\nDone!")


if __name__ == '__main__':
    main()
