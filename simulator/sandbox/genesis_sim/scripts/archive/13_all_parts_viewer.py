#!/usr/bin/env python3
"""
13_all_parts_viewer.py - All Parts Sequential Viewer
全パーツを順番に表示して確認

Purpose:
- Display all STL parts one by one
- Each part runs in a separate process to avoid Genesis limitations
- Press Q/ESC or close viewer to move to next part
"""

import subprocess
import sys
from pathlib import Path
import json


def main():
    print("=" * 60)
    print("Genesis All Parts Viewer")
    print("=" * 60)

    # Paths
    script_dir = Path(__file__).parent
    assets_dir = script_dir.parent / "assets" / "meshes" / "parts"
    config_file = assets_dir / "parts_config.json"
    viewer_script = script_dir / "12_parts_viewer.py"

    # Load configuration
    with open(config_file, 'r') as f:
        config = json.load(f)

    parts = config.get('parts', [])
    print(f"\nWill display {len(parts)} parts:")
    for i, part in enumerate(parts):
        print(f"  {i}: {part['name']}")

    print("\n" + "-" * 60)
    print("Each part will be shown in a separate process.")
    print("Close each viewer window to proceed to the next part.")
    print("Press Ctrl+C to stop.")
    print("-" * 60)

    # Get Python executable from current environment
    python_exe = sys.executable

    # View each part
    for i, part in enumerate(parts):
        print(f"\n{'='*60}")
        print(f"Part {i + 1}/{len(parts)}: {part['name']}")
        print(f"{'='*60}")

        try:
            result = subprocess.run(
                [python_exe, str(viewer_script), str(i)],
                cwd=str(script_dir),
            )
            if result.returncode != 0:
                print(f"  Warning: viewer exited with code {result.returncode}")
        except KeyboardInterrupt:
            print("\n\nStopped by user.")
            break

    print("\n" + "=" * 60)
    print("All parts viewed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
