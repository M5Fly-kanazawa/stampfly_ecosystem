#!/usr/bin/env python3
"""
12_parts_viewer.py - Individual Parts Viewer
各パーツを個別に表示して確認

Purpose:
- Display each STL part individually
- Verify normals and mesh quality
- Navigate through parts with keyboard
"""

import genesis as gs
from pathlib import Path
import json
import sys


def main():
    print("=" * 60)
    print("Genesis Individual Parts Viewer")
    print("=" * 60)

    # Paths
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    config_file = assets_dir / "parts_config.json"

    # Load configuration
    with open(config_file, 'r') as f:
        config = json.load(f)

    parts = config.get('parts', [])
    print(f"\nFound {len(parts)} parts:")
    for i, part in enumerate(parts):
        print(f"  {i}: {part['name']}")

    # Get part index from command line or default to 0
    part_idx = 0
    if len(sys.argv) > 1:
        try:
            part_idx = int(sys.argv[1])
        except ValueError:
            # Search by name
            for i, p in enumerate(parts):
                if sys.argv[1].lower() in p['name'].lower():
                    part_idx = i
                    break

    if part_idx < 0 or part_idx >= len(parts):
        print(f"Invalid part index: {part_idx}")
        return

    part = parts[part_idx]
    print(f"\n[Selected] Part {part_idx}: {part['name']}")

    # Initialize Genesis
    print("\n[1] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # Create scene
    print("\n[2] Creating scene...")
    scene = gs.Scene(
        show_viewer=True,
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, 0),
            dt=1/60,
        ),
    )

    # Add ground plane
    scene.add_entity(gs.morphs.Plane())

    # Add axis markers
    # X-axis (red)
    scene.add_entity(
        gs.morphs.Box(size=(0.3, 0.01, 0.01), pos=(0.15, 0, 0.005), fixed=True),
        surface=gs.surfaces.Default(color=(1.0, 0.2, 0.2)),
    )
    # Y-axis (green)
    scene.add_entity(
        gs.morphs.Box(size=(0.01, 0.3, 0.01), pos=(0, 0.15, 0.005), fixed=True),
        surface=gs.surfaces.Default(color=(0.2, 1.0, 0.2)),
    )
    # Z-axis (blue)
    scene.add_entity(
        gs.morphs.Box(size=(0.01, 0.01, 0.3), pos=(0, 0, 0.15), fixed=True),
        surface=gs.surfaces.Default(color=(0.2, 0.2, 1.0)),
    )

    # Load part
    print(f"\n[3] Loading part: {part['name']}")
    stl_file = assets_dir / part['file']
    color = part['color']
    opacity = part.get('opacity', 1.0)

    print(f"    File: {stl_file}")
    print(f"    Color: RGB({color[0]:.2f}, {color[1]:.2f}, {color[2]:.2f})")
    print(f"    Opacity: {opacity}")

    # Coordinate transformation
    transform_euler = (-90, 0, 0)
    spawn_height = 0.5  # Camera level

    try:
        mesh_entity = scene.add_entity(
            gs.morphs.Mesh(
                file=str(stl_file),
                pos=(0, 0, spawn_height),
                euler=transform_euler,
                scale=(0.001, 0.001, 0.001),  # mm to meters
                fixed=True,
                convexify=False,
            ),
            surface=gs.surfaces.Default(
                color=(color[0], color[1], color[2]),
            ),
        )
        print(f"    -> Loaded successfully")
        print(f"    -> Transform: euler={transform_euler}")
    except Exception as e:
        print(f"    ERROR: {e}")
        return

    # Build scene
    print("\n[4] Building scene...")
    scene.build()

    # Display info
    print("\n" + "=" * 60)
    print(f"Viewing: {part['name']}")
    print("=" * 60)
    print("")
    print("  Controls:")
    print("    Q/ESC - Exit")
    print("")
    print(f"  To view other parts, run:")
    print(f"    python {Path(__file__).name} <part_index>")
    print(f"    python {Path(__file__).name} <part_name>")
    print("")
    print("  Available parts:")
    for i, p in enumerate(parts):
        marker = " -> " if i == part_idx else "    "
        print(f"  {marker}{i}: {p['name']}")
    print("=" * 60)

    # Run viewer
    try:
        while True:
            scene.step()
            if not scene.viewer.is_alive():
                break
    except KeyboardInterrupt:
        pass

    print("\nViewer closed.")


if __name__ == "__main__":
    main()
