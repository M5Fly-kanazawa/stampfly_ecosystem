#!/usr/bin/env python3
"""
11_urdf_test.py - StampFly URDF Verification Test
URDFモデルの読み込みと表示確認

Purpose:
- Load StampFly URDF generated from STL + JSON
- Verify coordinate transformation (correct orientation)
- Verify per-part coloring
- Test with gravity OFF for static visualization

Expected Results:
- Drone should appear upright (top facing +Z)
- Front should face +Y direction
- Each part should have its own color from JSON
- 4 propeller links attached to base_link
"""

import genesis as gs
from pathlib import Path
import sys


def main():
    print("=" * 60)
    print("Genesis StampFly URDF Verification Test")
    print("=" * 60)

    # Paths
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    urdf_file = assets_dir / "stampfly.urdf"

    # Check if URDF exists, generate if not
    if not urdf_file.exists():
        print(f"\n[0] URDF not found, generating...")
        sys.path.insert(0, str(script_dir))
        from generate_stampfly_urdf import generate_urdf
        config_file = assets_dir / "parts_config.json"
        generate_urdf(config_file, urdf_file)

    print(f"\n[1] Loading URDF: {urdf_file}")

    # Initialize Genesis
    print("\n[2] Initializing Genesis...")
    gs.init(backend=gs.cpu)

    # Create scene with NO gravity (for static verification)
    print("\n[3] Creating scene (gravity OFF)...")
    scene = gs.Scene(
        show_viewer=True,
        sim_options=gs.options.SimOptions(
            gravity=(0, 0, 0),  # No gravity for static viewing
            dt=1/60,
        ),
    )

    # Add ground plane for reference
    print("\n[4] Adding ground plane...")
    scene.add_entity(gs.morphs.Plane())

    # Add axis markers for reference
    print("\n[5] Adding axis markers...")
    # X-axis (red) - Genesis Right
    scene.add_entity(
        gs.morphs.Box(size=(0.3, 0.01, 0.01), pos=(0.15, 0, 0.005), fixed=True),
        surface=gs.surfaces.Default(color=(1.0, 0.2, 0.2)),
    )
    # Y-axis (green) - Genesis Forward
    scene.add_entity(
        gs.morphs.Box(size=(0.01, 0.3, 0.01), pos=(0, 0.15, 0.005), fixed=True),
        surface=gs.surfaces.Default(color=(0.2, 1.0, 0.2)),
    )
    # Z-axis (blue) - Genesis Up
    scene.add_entity(
        gs.morphs.Box(size=(0.01, 0.01, 0.3), pos=(0, 0, 0.15), fixed=True),
        surface=gs.surfaces.Default(color=(0.2, 0.2, 1.0)),
    )
    print("    X-axis (red): Right (+X)")
    print("    Y-axis (green): Forward (+Y)")
    print("    Z-axis (blue): Up (+Z)")

    # Load StampFly URDF
    print("\n[6] Loading StampFly URDF...")
    spawn_height = 0.3  # 30cm above ground

    # Coordinate transformation is baked into URDF (rpy in each visual element)
    # WebGL→Genesis: rpy="-1.5708 3.14159 0" (X:-90°, Y:180°)
    # No additional transformation needed at load time
    transform_euler = (0, 0, 0)

    try:
        stampfly = scene.add_entity(
            gs.morphs.URDF(
                file=str(urdf_file),
                pos=(0, 0, spawn_height),
                euler=transform_euler,  # Apply coordinate transform here
                fixed=True,  # Fixed for static viewing
                prioritize_urdf_material=True,  # Use URDF colors
                decimate=False,  # Don't simplify mesh - preserve all faces
                convexify=False,  # Don't convert to convex hull
            ),
        )
        print(f"    -> URDF loaded at height {spawn_height}m")
        print(f"    -> Transform: euler={transform_euler}")
        print(f"    -> Mesh processing: decimate=False, convexify=False")
    except Exception as e:
        print(f"    ERROR loading URDF: {e}")
        print("\n    Trying with gs.morphs.Drone instead...")
        try:
            stampfly = scene.add_entity(
                gs.morphs.Drone(
                    file=str(urdf_file),
                    pos=(0, 0, spawn_height),
                    prioritize_urdf_material=True,
                ),
            )
            print(f"    -> Drone loaded at height {spawn_height}m")
        except Exception as e2:
            print(f"    ERROR loading as Drone: {e2}")
            return

    # Build scene
    print("\n[7] Building scene...")
    scene.build()
    print("    -> Scene built!")

    # Display verification info
    print("\n" + "=" * 60)
    print("Verification Checklist:")
    print("=" * 60)
    print("")
    print("  1. ORIENTATION:")
    print("     [  ] Top of drone facing UP (+Z, blue axis)")
    print("     [  ] Front of drone facing FORWARD (+Y, green axis)")
    print("     [  ] Right of drone facing RIGHT (+X, red axis)")
    print("")
    print("  2. COLORS (from parts_config.json):")
    print("     [  ] Frame: light gray (0.9, 0.9, 0.9)")
    print("     [  ] Propellers: red (1.0, 0.0, 0.0)")
    print("     [  ] Motors: silver (0.78, 0.83, 0.84)")
    print("     [  ] PCB: dark gray (0.1, 0.1, 0.1)")
    print("     [  ] M5StampS3: orange (1.0, 0.4, 0.0)")
    print("     [  ] Battery: dark brown (0.16, 0.07, 0.0)")
    print("")
    print("  3. STRUCTURE:")
    print("     [  ] All parts visible and connected")
    print("     [  ] 4 propellers at corners")
    print("     [  ] Motors under propellers")
    print("")
    print("  Controls: Press Q or ESC to exit, or close the viewer window.")
    print("  Note: Gravity is OFF for static viewing.")
    print("=" * 60)

    # Run simulation (static viewing)
    print("\n[8] Running static visualization...")
    print("    Press Q or ESC to exit.")

    try:
        while True:
            scene.step()
            # Check if viewer is still open
            if not scene.viewer.is_alive():
                print("\n    Viewer closed.")
                break
    except KeyboardInterrupt:
        print("\n    Interrupted by user")

    print("\n" + "=" * 60)
    print("Test completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
