"""
sf sim - Simulator commands

Launch and manage StampFly flight simulators.
StampFlyフライトシミュレータの起動と管理を行います。

Subcommands:
    list      - List available simulators
    run       - Run interactive simulator
    headless  - Run headless simulation
"""

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Optional

from ..utils import console, paths

COMMAND_NAME = "sim"
COMMAND_HELP = "Run flight simulator"

# Simulator backends
BACKENDS = {
    "vpython": {
        "name": "VPython",
        "description": "VPython-based 3D visualization (2000Hz physics, 400Hz control)",
        "script": "simulator/vpython/scripts/run_sim.py",
        "headless_script": "simulator/vpython/scripts/run_vpython_headless.py",
        "requires_venv": False,
    },
    "genesis": {
        "name": "Genesis",
        "description": "Genesis physics engine (2000Hz physics, 400Hz control, 30Hz render)",
        "script": "simulator/genesis/scripts/run_genesis_sim.py",
        "headless_script": "simulator/genesis/scripts/run_genesis_headless.py",
        "requires_venv": True,
        "venv_path": "simulator/genesis/venv",
    },
}


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Create sub-subparsers
    sim_subparsers = parser.add_subparsers(
        dest="sim_command",
        title="subcommands",
        metavar="<subcommand>",
    )

    # --- list ---
    list_parser = sim_subparsers.add_parser(
        "list",
        help="List available simulators",
        description="Show all available simulator backends.",
    )
    list_parser.set_defaults(func=run_list)

    # --- run ---
    run_parser = sim_subparsers.add_parser(
        "run",
        help="Run interactive simulator",
        description="Launch interactive flight simulator with visualization.",
    )
    run_parser.add_argument(
        "backend",
        nargs="?",
        default="vpython",
        choices=list(BACKENDS.keys()),
        help="Simulator backend (default: vpython)",
    )
    run_parser.add_argument(
        "--no-joystick",
        action="store_true",
        help="Disable joystick input",
    )
    run_parser.set_defaults(func=run_sim)

    # --- headless ---
    headless_parser = sim_subparsers.add_parser(
        "headless",
        help="Run headless simulation",
        description="Run simulation without visualization for automated testing.",
    )
    headless_parser.add_argument(
        "backend",
        nargs="?",
        default="vpython",
        choices=list(BACKENDS.keys()),
        help="Simulator backend (default: vpython)",
    )
    headless_parser.add_argument(
        "-d", "--duration",
        type=float,
        default=10.0,
        help="Simulation duration in seconds (default: 10)",
    )
    headless_parser.add_argument(
        "-o", "--output",
        help="Output log file path",
    )
    headless_parser.set_defaults(func=run_headless)

    parser.set_defaults(func=run_help)


def run_help(args: argparse.Namespace) -> int:
    """Show help when no subcommand specified"""
    console.print("Usage: sf sim <subcommand> [options]")
    console.print()
    console.print("Subcommands:")
    console.print("  list      List available simulator backends")
    console.print("  run       Run interactive simulator")
    console.print("  headless  Run headless simulation")
    console.print()
    console.print("Examples:")
    console.print("  sf sim run              # Run VPython simulator")
    console.print("  sf sim run genesis      # Run Genesis simulator")
    console.print("  sf sim headless -d 30   # 30s headless simulation")
    console.print()
    console.print("Run 'sf sim <subcommand> --help' for details.")
    return 0


def run_list(args: argparse.Namespace) -> int:
    """List available simulators"""
    console.info("Available simulator backends:")
    console.print()

    for backend_id, backend in BACKENDS.items():
        script_path = paths.root() / backend["script"]
        available = script_path.exists()

        status = "[OK]" if available else "[NOT FOUND]"
        status_color = "green" if available else "red"

        console.print(f"  {backend_id:12s} - {backend['name']}")
        console.print(f"               {backend['description']}")
        console.print(f"               Script: {backend['script']}")

        if backend.get("requires_venv"):
            venv_path = paths.root() / backend["venv_path"]
            venv_exists = venv_path.exists()
            venv_status = "exists" if venv_exists else "not found"
            console.print(f"               Venv: {backend['venv_path']} ({venv_status})")

        console.print(f"               Status: {status}")
        console.print()

    console.print("Usage:")
    console.print("  sf sim run [backend]      # Interactive mode")
    console.print("  sf sim headless [backend] # Headless mode")

    return 0


def run_sim(args: argparse.Namespace) -> int:
    """Run interactive simulator"""
    backend_id = args.backend
    backend = BACKENDS.get(backend_id)

    if not backend:
        console.error(f"Unknown backend: {backend_id}")
        return 1

    script_path = paths.root() / backend["script"]

    if not script_path.exists():
        console.error(f"Simulator script not found: {script_path}")
        console.print("  Run 'sf sim list' to check available backends")
        return 1

    # Prepare Python command (check dependencies first)
    python_cmd = _get_python_cmd(backend)
    if not python_cmd:
        return 1

    console.info(f"Starting {backend['name']} simulator...")
    console.print(f"  Backend: {backend_id}")
    console.print(f"  Script: {script_path}")
    console.print()

    # Build command
    cmd = [python_cmd, str(script_path)]

    if hasattr(args, 'no_joystick') and args.no_joystick:
        cmd.append("--no-joystick")

    console.print("Controls:")
    console.print("  - Throttle (Axis 0): Total thrust")
    console.print("  - Roll (Axis 1): Roll command")
    console.print("  - Pitch (Axis 2): Pitch command")
    console.print("  - Yaw (Axis 3): Yaw command")
    console.print("  - Arm button: Reset simulation")
    console.print("  - Mode button: Toggle ACRO/STABILIZE")
    console.print("  - Q key: Exit")
    console.print()

    try:
        # Run simulator
        result = subprocess.run(
            cmd,
            cwd=script_path.parent,
        )
        return result.returncode

    except KeyboardInterrupt:
        console.print("\nSimulator interrupted")
        return 0
    except Exception as e:
        console.error(f"Failed to start simulator: {e}")
        return 1


def run_headless(args: argparse.Namespace) -> int:
    """Run headless simulation"""
    backend_id = args.backend
    backend = BACKENDS.get(backend_id)

    if not backend:
        console.error(f"Unknown backend: {backend_id}")
        return 1

    headless_script = backend.get("headless_script")
    if not headless_script:
        console.error(f"Headless mode not supported for {backend_id}")
        return 1

    script_path = paths.root() / headless_script

    if not script_path.exists():
        console.error(f"Headless script not found: {script_path}")
        return 1

    # Prepare Python command (check dependencies first)
    python_cmd = _get_python_cmd(backend)
    if not python_cmd:
        return 1

    console.info(f"Starting headless {backend['name']} simulation...")
    console.print(f"  Backend: {backend_id}")
    console.print(f"  Duration: {args.duration}s")
    if args.output:
        console.print(f"  Output: {args.output}")
    console.print()

    # Build command
    cmd = [python_cmd, str(script_path)]

    if args.duration:
        cmd.extend(["--duration", str(args.duration)])

    if args.output:
        cmd.extend(["--output", args.output])

    try:
        result = subprocess.run(
            cmd,
            cwd=script_path.parent,
        )
        return result.returncode

    except KeyboardInterrupt:
        console.print("\nSimulation interrupted")
        return 0
    except Exception as e:
        console.error(f"Failed to run simulation: {e}")
        return 1


def _check_vpython_available(python_cmd: str) -> bool:
    """Check if vpython is available"""
    try:
        result = subprocess.run(
            [python_cmd, "-c", "import vpython"],
            capture_output=True,
            timeout=10,
        )
        return result.returncode == 0
    except Exception:
        return False


def _get_python_cmd(backend: dict) -> Optional[str]:
    """Get Python command for backend"""
    if backend.get("requires_venv"):
        # Use venv Python
        venv_path = paths.root() / backend["venv_path"]
        python_path = venv_path / "bin" / "python"

        if not python_path.exists():
            console.error(f"Venv not found: {venv_path}")
            console.print("  To create the venv:")
            console.print(f"    cd {venv_path.parent}")
            console.print(f"    python3 -m venv venv")
            console.print(f"    source venv/bin/activate")
            console.print(f"    pip install genesis-world pygame")
            return None

        return str(python_path)
    else:
        # For vpython, check if it's available
        backend_id = backend.get("script", "")
        if "vpython" in backend_id.lower() or backend.get("name") == "VPython":
            # Try system Python first (more likely to have vpython)
            system_pythons = ["/usr/bin/python3", "/usr/local/bin/python3"]

            # Check current Python
            if _check_vpython_available(sys.executable):
                return sys.executable

            # Check system Pythons
            for py in system_pythons:
                if Path(py).exists() and _check_vpython_available(py):
                    return py

            # Not found
            console.error("vpython module not found")
            console.print()
            console.print("  Install vpython:")
            console.print("    pip3 install vpython pygame numpy")
            console.print()
            console.print("  Or create a dedicated venv:")
            console.print("    cd simulator")
            console.print("    python3 -m venv venv")
            console.print("    source venv/bin/activate")
            console.print("    pip install vpython pygame numpy")
            return None

        # Use current Python
        return sys.executable
