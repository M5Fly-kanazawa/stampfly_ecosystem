#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Index-based STL Split
インデックスベースのSTL分割

Split part_00 (frame+m5stamps3+pcb) into separate parts using:
1. Exact index matching for first 4520 triangles
2. Spatial classification for remaining triangles
Also includes other auto parts (propellers, motors).

part_00を分割（フレーム+m5stamps3+pcb）:
1. 最初の4520三角形は正確なインデックスマッチング
2. 残りの三角形は空間的分類
その他の自動分割パーツ（プロペラ、モーター）も含む。
"""

import os
import shutil
import numpy as np
from stl import mesh
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARTS_DIR = os.path.join(SCRIPT_DIR, 'parts')


def split_part00_by_index():
    """
    Split part_00 into frame, m5stamps3, and pcb.
    """
    input_path = os.path.join(PARTS_DIR, 'auto', 'part_00.stl')
    output_dir = os.path.join(PARTS_DIR, 'index_split')
    os.makedirs(output_dir, exist_ok=True)

    print(f"Loading: {input_path}")
    part00 = mesh.Mesh.from_file(input_path)
    print(f"Total triangles: {len(part00.vectors)}")

    # Calculate centroids for spatial classification
    centroids = np.array([np.mean(tri, axis=0) for tri in part00.vectors])

    # Classification based on index and spatial position
    # First 4520 triangles match original file exactly
    labels = np.zeros(len(part00.vectors), dtype=int)

    # 0: Frame, 1: M5StampS3, 2: PCB
    # Indices 0-1523: Frame1
    labels[0:1524] = 0
    # Indices 1524-2151: M5StampS3
    labels[1524:2152] = 1
    # Indices 2152-4519: PCB
    labels[2152:4520] = 2

    # For remaining triangles (4520+), use spatial classification
    for i in range(4520, len(part00.vectors)):
        x, y, z = centroids[i]
        if y < 0.5:
            # Back side of frame (low Y)
            labels[i] = 0  # Frame
        elif abs(z) < 21:
            # Middle Z range = PCB
            labels[i] = 2  # PCB
        else:
            # High |Z| = Frame arms
            labels[i] = 0  # Frame

    # Count each part
    print(f"\nClassification results:")
    print(f"  Frame: {(labels == 0).sum()} triangles")
    print(f"  M5StampS3: {(labels == 1).sum()} triangles")
    print(f"  PCB: {(labels == 2).sum()} triangles")

    # Save each part
    parts_config = {
        "method": "index_split",
        "source": "part_00.stl",
        "parts": []
    }

    part_names = ['frame', 'm5stamps3', 'pcb']
    part_colors = [
        [0.9, 0.9, 0.9],  # Frame: light gray
        [0.3, 0.8, 0.3],  # M5StampS3: green
        [0.2, 0.5, 0.8]   # PCB: blue
    ]

    for label, (name, color) in enumerate(zip(part_names, part_colors)):
        mask = labels == label
        indices = np.where(mask)[0]

        if len(indices) > 0:
            # Create new mesh for this part
            part_vectors = part00.vectors[indices]
            part_mesh = mesh.Mesh(np.zeros(len(part_vectors), dtype=mesh.Mesh.dtype))
            part_mesh.vectors = part_vectors

            # Save
            output_path = os.path.join(output_dir, f"{name}.stl")
            part_mesh.save(output_path)
            print(f"Saved: {output_path} ({len(indices)} triangles)")

            parts_config["parts"].append({
                "name": name,
                "file": f"{name}.stl",
                "triangles": int(len(indices)),
                "color": color
            })

    # Save config
    config_path = os.path.join(output_dir, "parts_config.json")
    with open(config_path, 'w') as f:
        json.dump(parts_config, f, indent=2)
    print(f"\nConfig saved: {config_path}")

    return output_dir


def copy_other_auto_parts(output_dir):
    """
    Copy other auto parts (propellers, motors) to output directory.
    他の自動分割パーツ（プロペラ、モーター）をコピー
    """
    auto_dir = os.path.join(PARTS_DIR, 'auto')
    auto_config_path = os.path.join(auto_dir, 'parts_config.json')

    with open(auto_config_path, 'r') as f:
        auto_config = json.load(f)

    # Map auto parts to meaningful names
    # Based on analysis: 690 triangles = propeller, 208 = motor, 52 = unknown small part
    part_mapping = {
        'part_01': {'name': 'motor_fr', 'color': [0.4, 0.4, 0.4]},  # 208 tri
        'part_02': {'name': 'propeller_fr', 'color': [0.2, 0.2, 0.2]},  # 690 tri
        'part_03': {'name': 'motor_rr', 'color': [0.4, 0.4, 0.4]},  # 208 tri
        'part_04': {'name': 'motor_rl', 'color': [0.4, 0.4, 0.4]},  # 208 tri
        'part_05': {'name': 'motor_fl', 'color': [0.4, 0.4, 0.4]},  # 208 tri
        'part_06': {'name': 'propeller_rr', 'color': [0.2, 0.2, 0.2]},  # 690 tri
        'part_07': {'name': 'propeller_rl', 'color': [0.2, 0.2, 0.2]},  # 690 tri
        'part_08': {'name': 'propeller_fl', 'color': [0.2, 0.2, 0.2]},  # 690 tri
        'part_09': {'name': 'battery_holder', 'color': [0.6, 0.3, 0.1]},  # 52 tri
    }

    copied_parts = []
    for auto_part in auto_config['parts']:
        old_name = auto_part['name']
        if old_name in part_mapping:
            mapping = part_mapping[old_name]
            new_name = mapping['name']

            # Copy STL file with new name
            src_path = os.path.join(auto_dir, auto_part['file'])
            dst_path = os.path.join(output_dir, f"{new_name}.stl")
            shutil.copy2(src_path, dst_path)
            print(f"Copied: {old_name} -> {new_name}.stl")

            copied_parts.append({
                "name": new_name,
                "file": f"{new_name}.stl",
                "triangles": auto_part['triangles'],
                "color": mapping['color']
            })

    return copied_parts


def main():
    output_dir = split_part00_by_index()

    # Copy other auto parts
    print("\n=== Copying other auto parts ===")
    other_parts = copy_other_auto_parts(output_dir)

    # Update config with all parts
    config_path = os.path.join(output_dir, "parts_config.json")
    with open(config_path, 'r') as f:
        config = json.load(f)

    config['parts'].extend(other_parts)

    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)

    print(f"\nUpdated config with {len(other_parts)} additional parts")
    print(f"\n=== Done ===")
    print(f"Output directory: {output_dir}")
    print(f"Total parts: {len(config['parts'])}")


if __name__ == '__main__':
    main()
