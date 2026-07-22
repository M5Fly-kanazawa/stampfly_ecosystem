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
        "-w", "--world",
        default="voxel",
        choices=["ringworld", "voxel", "minimal"],
        help="World type (default: voxel, minimal for debugging)",
    )
    run_parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for world generation",
    )
    run_parser.add_argument(
        "--mode",
        default="rate",
        choices=["rate", "angle"],
        help="Control mode: rate=ACRO, angle=STABILIZE (default: rate)",
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
    console.print("  sf sim run                      # Run VPython simulator (voxel)")
    console.print("  sf sim run -w ringworld         # Run with ring world (lighter)")
    console.print("  sf sim run --mode angle         # STABILIZE mode")
    console.print("  sf sim run genesis              # Run Genesis simulator")
    console.print("  sf sim headless -d 30           # 30s headless simulation")
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

    # Check hidapi native library for joystick support
    # ジョイスティック用のhidapiネイティブライブラリを確認
    no_joystick = hasattr(args, 'no_joystick') and args.no_joystick
    if not no_joystick and not _check_hidapi_available(python_cmd):
        console.print()
        console.print("  Continuing without joystick (--no-joystick)...")
        console.print()
        no_joystick = True

    console.info(f"Starting {backend['name']} simulator...")
    console.print(f"  Backend: {backend_id}")
    console.print(f"  Script: {script_path}")
    console.print()

    # Build command
    # コマンドを構築
    cmd = [python_cmd, str(script_path)]

    if hasattr(args, 'world') and args.world:
        cmd.extend(["--world", args.world])
    if hasattr(args, 'seed') and args.seed is not None:
        cmd.extend(["--seed", str(args.seed)])
    if hasattr(args, 'mode') and args.mode:
        cmd.extend(["--mode", args.mode])
    if no_joystick:
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
            text=True,
            timeout=10,
        )
        if result.returncode == 0:
            return True

        # Detect pkg_resources issue (setuptools 82+ removed it)
        # pkg_resources問題を検出（setuptools 82+で削除された）
        if "pkg_resources" in result.stderr:
            console.warning(
                "vpython failed: setuptools 82+ removed pkg_resources"
            )
            console.print("  Attempting auto-fix: pip install 'setuptools>=68,<81'...")
            fix = subprocess.run(
                [python_cmd, "-m", "pip", "install", "setuptools>=68,<81"],
                capture_output=True,
                timeout=60,
            )
            if fix.returncode == 0:
                # Retry import after fix
                # 修復後にインポートを再試行
                retry = subprocess.run(
                    [python_cmd, "-c", "import vpython"],
                    capture_output=True,
                    timeout=10,
                )
                if retry.returncode == 0:
                    console.info("setuptools fixed, vpython is now available")
                    return True
            console.error("Auto-fix failed. Run manually:")
            console.print("    pip install 'setuptools>=68,<81'")

        return False
    except Exception:
        return False


def _prompt_yes_no(message: str, default: bool = True) -> bool:
    """Ask a y/n question on stdin and return the answer.

    EOF-safe: if stdin is closed (redirected from /dev/null, a CI runner,
    etc.), input() raises EOFError, which is caught here and treated as
    accepting `default` rather than propagating or hanging. Callers must
    still gate on sys.stdin.isatty() first -- this function only makes a
    *closed* stdin safe, it does not detect a non-interactive terminal.

    stdin上でy/n形式の質問をし、回答を返す。

    EOFセーフ: stdinが閉じている(/dev/nullへのリダイレクト、CI環境等)場合、
    input()はEOFErrorを送出するが、ここで捕捉して例外を伝播・ハングさせず
    `default` を受理したものとして扱う。呼び出し側はそれでも先に
    sys.stdin.isatty() で対話端末かを確認すること -- この関数が保証するのは
    「閉じた stdin」への安全性のみで、非対話端末の検出は行わない。
    """
    try:
        response = input(message).strip().lower()
    except (EOFError, KeyboardInterrupt):
        console.print()
        return default
    if not response:
        return default
    return response in ("y", "yes")


def _install_vpython_into(python_exe: str) -> bool:
    """Install vpython into `python_exe`'s own environment and verify it
    actually imports afterward (not just that pip reported success).

    Used to self-heal `sf sim run` when vpython is missing from sf's own
    environment (see _get_python_cmd()): installing into `sys.executable`
    -- the SAME interpreter that `sf` itself is running under, since `sf`
    is a console_scripts entry point inside the ESP-IDF venv (see
    pyproject.toml's `[project.scripts] sf = "sfcli.cli:main"`) -- keeps
    vpython inside the sf environment instead of landing in an unrelated
    system Python via a bare `pip3 install vpython` (the exact drift that
    prompted this function to exist, see the module-level guidance text
    below).

    `python_exe` 自身の環境にvpythonを導入し、実際にimportできることまで
    確認する(pipが成功と報告しただけで終わらせない)。

    sf自身の環境にvpythonが無い場合の `sf sim run` 自己修復に使う
    (_get_python_cmd() 参照): `sys.executable` -- `sf` 自身が動いている
    のと同じインタプリタ(`sf` はESP-IDF venv内のconsole_scripts
    エントリポイントのため。pyproject.tomlの
    `[project.scripts] sf = "sfcli.cli:main"` 参照) -- に導入することで、
    素の `pip3 install vpython` が無関係なシステムPythonに着地してしまう
    (この関数が存在する契機となったずれそのもの。下のガイド文言参照)の
    ではなく、vpythonをsf環境の中に留める。
    """
    console.print(f"  Installing vpython into {python_exe} ...")
    try:
        result = subprocess.run(
            [python_exe, "-m", "pip", "install", "vpython"],
            timeout=300,
        )
    except subprocess.TimeoutExpired:
        console.error("pip install timed out")
        return False
    except Exception as e:
        console.error(f"Failed to run pip: {e}")
        return False
    if result.returncode != 0:
        return False
    return _check_vpython_available(python_exe)


def _check_hidapi_available(python_cmd: str) -> bool:
    """Check if hidapi native library is available for the hid package.
    hidパッケージのネイティブライブラリが利用可能か確認"""
    try:
        result = subprocess.run(
            [python_cmd, "-c", "import hid; hid.enumerate()"],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode == 0:
            return True

        # Check if this is a missing native library issue
        # ネイティブライブラリの欠如を検出
        if "libhidapi" in result.stderr or "Unable to load" in result.stderr:
            if sys.platform == "darwin":
                console.warning(
                    "hid package requires libhidapi native library"
                )
                console.print("  Install with Homebrew:")
                console.print("    brew install hidapi")
                console.print()
                console.print("  Or use --no-joystick to run without controller")
            elif sys.platform == "linux":
                console.warning(
                    "hid package requires libhidapi native library"
                )
                console.print("  Install with package manager:")
                console.print("    sudo apt install libhidapi-dev  # Debian/Ubuntu")
                console.print("    sudo dnf install hidapi-devel   # Fedora")
                console.print()
                console.print("  Or use --no-joystick to run without controller")
            return False

        return False
    except Exception:
        return False


def _get_python_cmd(backend: dict) -> Optional[str]:
    """Get Python command for backend"""
    if backend.get("requires_venv"):
        # Use venv Python
        # Windows uses Scripts/python.exe, Unix uses bin/python
        # WindowsはScripts/python.exe、Unixはbin/pythonを使用
        venv_path = paths.root() / backend["venv_path"]
        if sys.platform == "win32":
            python_path = venv_path / "Scripts" / "python.exe"
        else:
            python_path = venv_path / "bin" / "python"

        if not python_path.exists():
            console.error(f"Venv not found: {venv_path}")
            console.print("  To create the venv:")
            console.print(f"    cd {venv_path.parent}")
            if sys.platform == "win32":
                console.print(f"    python -m venv venv")
                console.print(f"    venv\\Scripts\\activate")
            else:
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

            # Check current Python. This is normally the sf/ESP-IDF venv's
            # OWN interpreter -- `sf` is a console_scripts entry point
            # installed inside that venv (see pyproject.toml's
            # `[project.scripts] sf = "sfcli.cli:main"`), so sys.executable
            # here already IS the environment `sf sim run` should be using.
            # 現在のPythonを確認する。これは通常sf/ESP-IDF venv自身の
            # インタプリタである -- `sf` はそのvenv内にインストールされた
            # console_scriptsエントリポイントのため(pyproject.tomlの
            # `[project.scripts] sf = "sfcli.cli:main"` 参照)、ここでの
            # sys.executableは既に`sf sim run`が使うべき環境そのもの。
            if _check_vpython_available(sys.executable):
                return sys.executable

            # Missing from sf's own environment. Self-heal by offering to
            # install it there directly when attached to an interactive
            # terminal, instead of silently falling back to an unrelated
            # system Python (below) that would leave the sf environment
            # itself still missing vpython. Installing to sys.executable
            # here (not a bare `pip3 install`) is the fix for the exact
            # drift this function used to cause: a user following the old
            # "pip3 install vpython" guidance landed vpython in pyenv's
            # system Python instead of the sf/ESP-IDF venv, so `sf sim run`
            # kept working only by accident, via the system-Python fallback
            # below, while the sf environment itself stayed out of sync.
            # sf自身の環境に無い場合、対話端末に接続されていればそこへ
            # 直接インストールするか提案して自己修復する。無関係な
            # システムPython(下記)への代替探索に無言で頼ると、sf環境
            # 自体はvpythonが無いままになってしまう。ここで(素の
            # `pip3 install`ではなく)sys.executableへインストールするのが、
            # この関数がかつて引き起こしていたずれそのものへの対処:
            # 旧来の「pip3 install vpython」案内に従ったユーザーはvpythonを
            # pyenvのシステムPythonに導入してしまい、sf/ESP-IDF venvとは
            # 別物になっていた。`sf sim run`は下の代替Python探索が拾って
            # 偶然動いていただけで、sf環境自体はずれたままだった。
            if sys.stdin.isatty() and sys.stdout.isatty():
                console.warning(f"vpython not found in the sf environment ({sys.executable})")
                if _prompt_yes_no("  Install it there now? [Y/n]: ", default=True):
                    if _install_vpython_into(sys.executable):
                        console.info("vpython installed in the sf environment")
                        return sys.executable
                    console.error("Auto-install failed.")

            # Check system Pythons as a last-resort fallback so the
            # simulator can still run today, but flag it clearly: this is
            # NOT the sf environment, and running from here again risks
            # the exact drift described above.
            # 代替として最後にシステムPythonを確認し、今すぐシミュレータを
            # 動かせるようにする。ただしこれはsf環境ではないことを明示する
            # -- ここから動かし続けると上述のずれを再び招く。
            for py in system_pythons:
                if Path(py).exists() and _check_vpython_available(py):
                    console.warning(
                        f"Using {py} (NOT the sf environment: {sys.executable}). "
                        "Install vpython into the sf environment (see below) to "
                        "avoid drift."
                    )
                    return py

            # Not found anywhere
            console.error("vpython module not found")
            console.print()
            console.print("  Install it into the sf environment (not a system pip3):")
            console.print(f"    {sys.executable} -m pip install vpython")
            return None

        # Use current Python
        return sys.executable
