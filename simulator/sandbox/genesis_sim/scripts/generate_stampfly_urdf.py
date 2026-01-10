#!/usr/bin/env python3
"""
generate_stampfly_urdf.py - StampFly URDF Generator
STL + JSONからURDFを自動生成

Purpose:
- Generate URDF from STL parts and parts_config.json
- Apply coordinate transformation (WebGL → Genesis)
- Set colors from JSON configuration
- Structure for gs.morphs.Drone compatibility

Coordinate Transformation:
- WebGL (STL): X=left, Y=up, Z=forward
- Genesis: X=right, Y=forward, Z=up
- Transform matrix: Genesis = [-WebGL.x, WebGL.z, WebGL.y]
- URDF rpy: "-1.5708 3.14159 0" (X:-90°, Y:180°, Z:0°)
"""

import json
from pathlib import Path
import math


def generate_urdf(config_path: Path, output_path: Path):
    """Generate URDF from parts config."""

    # Load configuration
    with open(config_path, 'r') as f:
        config = json.load(f)

    parts = config.get('parts', [])

    # Coordinate transformation: WebGL → Genesis
    # WebGL (STL): X=left, Y=up, Z=forward
    # Genesis: X=right, Y=forward, Z=up
    # Transform: euler=(-90, 180, 0) degrees = rpy(-π/2, π, 0) radians
    transform_rpy = "-1.5708 3.14159 0"  # X:-90°, Y:180°, Z:0°

    # Scale: STL is in mm, URDF expects meters
    scale = "0.001 0.001 0.001"

    # Categorize parts
    body_parts = []
    propeller_parts = []

    for part in parts:
        name = part['name']
        if name.startswith('propeller_'):
            propeller_parts.append(part)
        else:
            body_parts.append(part)

    # Map propeller names to Genesis drone convention
    # Genesis expects: prop0_link, prop1_link, prop2_link, prop3_link
    # StampFly: FL=M4, FR=M1, RL=M3, RR=M2
    # Genesis CF2X order: prop0=FR, prop1=RL, prop2=FL, prop3=RR
    propeller_mapping = {
        'propeller_fr': ('prop0_link', 'prop0_joint'),  # FR = prop0
        'propeller_rl': ('prop1_link', 'prop1_joint'),  # RL = prop1
        'propeller_fl': ('prop2_link', 'prop2_joint'),  # FL = prop2
        'propeller_rr': ('prop3_link', 'prop3_joint'),  # RR = prop3
    }

    # Start URDF
    urdf_lines = [
        '<?xml version="1.0"?>',
        '<robot name="stampfly">',
        '',
        '  <!-- Materials -->',
    ]

    # Generate material definitions
    materials_defined = set()
    for part in parts:
        name = part['name']
        color = part['color']
        opacity = part.get('opacity', 1.0)
        material_name = f"{name}_material"

        if material_name not in materials_defined:
            rgba = f"{color[0]:.4f} {color[1]:.4f} {color[2]:.4f} {opacity}"
            urdf_lines.append(f'  <material name="{material_name}">')
            urdf_lines.append(f'    <color rgba="{rgba}"/>')
            urdf_lines.append(f'  </material>')
            materials_defined.add(material_name)

    urdf_lines.append('')
    urdf_lines.append('  <!-- Base Link (main body) -->')
    urdf_lines.append('  <link name="base_link">')

    # Add body parts as visual elements of base_link
    for part in body_parts:
        name = part['name']
        filename = part['file']
        material_name = f"{name}_material"

        urdf_lines.append(f'    <!-- {name} -->')
        urdf_lines.append(f'    <visual name="{name}_visual">')
        urdf_lines.append(f'      <origin xyz="0 0 0" rpy="{transform_rpy}"/>')
        urdf_lines.append(f'      <geometry>')
        urdf_lines.append(f'        <mesh filename="{filename}" scale="{scale}"/>')
        urdf_lines.append(f'      </geometry>')
        urdf_lines.append(f'      <material name="{material_name}"/>')
        urdf_lines.append(f'    </visual>')

    # Add collision for base_link (simplified - just frame)
    frame_part = next((p for p in body_parts if p['name'] == 'frame'), None)
    if frame_part:
        urdf_lines.append(f'    <collision name="base_collision">')
        urdf_lines.append(f'      <origin xyz="0 0 0" rpy="{transform_rpy}"/>')
        urdf_lines.append(f'      <geometry>')
        urdf_lines.append(f'        <mesh filename="{frame_part["file"]}" scale="{scale}"/>')
        urdf_lines.append(f'      </geometry>')
        urdf_lines.append(f'    </collision>')

    # Inertial properties for base_link
    # Mass: 35g total, body ~30g
    urdf_lines.append('    <inertial>')
    urdf_lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
    urdf_lines.append('      <mass value="0.030"/>')
    urdf_lines.append('      <inertia ixx="9.16e-6" ixy="0" ixz="0" iyy="13.3e-6" iyz="0" izz="20.4e-6"/>')
    urdf_lines.append('    </inertial>')
    urdf_lines.append('  </link>')
    urdf_lines.append('')

    # Add propeller links and joints
    # Note: STL files already contain propeller positions relative to origin
    # Joint origin is (0,0,0) - position comes from the mesh geometry

    urdf_lines.append('  <!-- Propeller Links -->')

    for part in propeller_parts:
        name = part['name']
        filename = part['file']
        material_name = f"{name}_material"

        if name not in propeller_mapping:
            print(f"Warning: Unknown propeller {name}, skipping")
            continue

        link_name, joint_name = propeller_mapping[name]

        urdf_lines.append(f'  <!-- {name} -> {link_name} -->')
        urdf_lines.append(f'  <link name="{link_name}">')
        urdf_lines.append(f'    <visual name="{name}_visual">')
        urdf_lines.append(f'      <origin xyz="0 0 0" rpy="{transform_rpy}"/>')
        urdf_lines.append(f'      <geometry>')
        urdf_lines.append(f'        <mesh filename="{filename}" scale="{scale}"/>')
        urdf_lines.append(f'      </geometry>')
        urdf_lines.append(f'      <material name="{material_name}"/>')
        urdf_lines.append(f'    </visual>')
        urdf_lines.append(f'    <inertial>')
        urdf_lines.append(f'      <origin xyz="0 0 0" rpy="0 0 0"/>')
        urdf_lines.append(f'      <mass value="0.001"/>')  # 1g per propeller
        urdf_lines.append(f'      <inertia ixx="1e-9" ixy="0" ixz="0" iyy="1e-9" iyz="0" izz="1e-9"/>')
        urdf_lines.append(f'    </inertial>')
        urdf_lines.append(f'  </link>')
        urdf_lines.append('')

        # Continuous joint for propeller rotation
        # Joint origin is (0,0,0) - STL already contains correct position
        urdf_lines.append(f'  <joint name="{joint_name}" type="continuous">')
        urdf_lines.append(f'    <parent link="base_link"/>')
        urdf_lines.append(f'    <child link="{link_name}"/>')
        urdf_lines.append(f'    <origin xyz="0 0 0" rpy="0 0 0"/>')
        urdf_lines.append(f'    <axis xyz="0 0 1"/>')  # Rotate around Z-axis
        urdf_lines.append(f'    <dynamics damping="0.0001"/>')
        urdf_lines.append(f'  </joint>')
        urdf_lines.append('')

    urdf_lines.append('</robot>')

    # Write URDF
    urdf_content = '\n'.join(urdf_lines)
    with open(output_path, 'w') as f:
        f.write(urdf_content)

    print(f"Generated URDF: {output_path}")
    print(f"  - Body parts: {len(body_parts)}")
    print(f"  - Propeller links: {len(propeller_parts)}")
    print(f"  - Coordinate transform: rpy={transform_rpy}")

    return urdf_content


def main():
    print("=" * 60)
    print("StampFly URDF Generator")
    print("=" * 60)

    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    config_file = assets_dir / "parts_config.json"
    output_file = assets_dir / "stampfly.urdf"

    if not config_file.exists():
        print(f"ERROR: Config file not found: {config_file}")
        return

    print(f"\nInput: {config_file}")
    print(f"Output: {output_file}")
    print()

    generate_urdf(config_file, output_file)

    print("\n" + "=" * 60)
    print("URDF generation complete!")
    print("=" * 60)
    print("\nNext: Run 11_urdf_test.py to verify in Genesis")


if __name__ == "__main__":
    main()
