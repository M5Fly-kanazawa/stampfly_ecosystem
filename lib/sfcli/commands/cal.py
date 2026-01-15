"""
sf cal - Sensor calibration commands

Calibrate sensors via serial connection to StampFly.
シリアル接続経由でStampFlyのセンサキャリブレーションを行います。

Subcommands:
    list    - List available calibrations
    gyro    - Gyroscope bias calibration
    accel   - Accelerometer calibration
    mag     - Magnetometer calibration (interactive)
    level   - Level calibration (attitude reference)
    status  - Show calibration status
    plot    - Plot magnetometer data from log
"""

import argparse
import glob
import sys
import time
from pathlib import Path
from typing import Optional

from ..utils import console, paths

COMMAND_NAME = "cal"
COMMAND_HELP = "Sensor calibration"

# Default serial settings
DEFAULT_BAUDRATE = 115200


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Create sub-subparsers
    cal_subparsers = parser.add_subparsers(
        dest="cal_command",
        title="subcommands",
        metavar="<subcommand>",
    )

    # --- list ---
    list_parser = cal_subparsers.add_parser(
        "list",
        help="List available calibrations",
        description="Show all available calibration types.",
    )
    list_parser.set_defaults(func=run_list)

    # --- gyro ---
    gyro_parser = cal_subparsers.add_parser(
        "gyro",
        help="Gyroscope bias calibration",
        description="Calibrate gyroscope bias. Keep the drone still during calibration.",
    )
    gyro_parser.add_argument(
        "-p", "--port",
        help="Serial port (auto-detect if not specified)",
    )
    gyro_parser.set_defaults(func=run_gyro)

    # --- accel ---
    accel_parser = cal_subparsers.add_parser(
        "accel",
        help="Accelerometer calibration",
        description="Calibrate accelerometer. Keep the drone level and still.",
    )
    accel_parser.add_argument(
        "-p", "--port",
        help="Serial port (auto-detect if not specified)",
    )
    accel_parser.set_defaults(func=run_accel)

    # --- mag ---
    mag_parser = cal_subparsers.add_parser(
        "mag",
        help="Magnetometer calibration (interactive)",
        description="Interactive magnetometer calibration. Rotate drone in all directions.",
    )
    mag_parser.add_argument(
        "-p", "--port",
        help="Serial port (auto-detect if not specified)",
    )
    mag_parser.add_argument(
        "action",
        nargs="?",
        choices=["start", "stop", "status", "save", "clear"],
        default="start",
        help="Calibration action (default: start)",
    )
    mag_parser.set_defaults(func=run_mag)

    # --- level ---
    level_parser = cal_subparsers.add_parser(
        "level",
        help="Level calibration (attitude reference)",
        description="Calibrate attitude reference on level surface.",
    )
    level_parser.add_argument(
        "-p", "--port",
        help="Serial port (auto-detect if not specified)",
    )
    level_parser.set_defaults(func=run_level)

    # --- status ---
    status_parser = cal_subparsers.add_parser(
        "status",
        help="Show calibration status",
        description="Show current calibration status from device.",
    )
    status_parser.add_argument(
        "-p", "--port",
        help="Serial port (auto-detect if not specified)",
    )
    status_parser.set_defaults(func=run_status)

    # --- plot ---
    plot_parser = cal_subparsers.add_parser(
        "plot",
        help="Plot magnetometer data from log",
        description="Plot magnetometer XY data to verify calibration.",
    )
    plot_parser.add_argument(
        "file",
        nargs="?",
        help="Binary log file (default: latest)",
    )
    plot_parser.add_argument(
        "-o", "--output",
        help="Output image file",
    )
    plot_parser.set_defaults(func=run_plot)

    parser.set_defaults(func=run_help)


def run_help(args: argparse.Namespace) -> int:
    """Show help when no subcommand specified"""
    console.print("Usage: sf cal <subcommand> [options]")
    console.print()
    console.print("Subcommands:")
    console.print("  list    List available calibration types")
    console.print("  gyro    Calibrate gyroscope bias")
    console.print("  accel   Calibrate accelerometer")
    console.print("  mag     Magnetometer calibration (interactive)")
    console.print("  level   Level calibration (attitude reference)")
    console.print("  status  Show calibration status")
    console.print("  plot    Plot magnetometer data from log")
    console.print()
    console.print("Examples:")
    console.print("  sf cal gyro           # Calibrate gyro (keep still)")
    console.print("  sf cal mag start      # Start mag calibration")
    console.print("  sf cal mag save       # Save mag calibration")
    console.print("  sf cal plot log.bin   # Plot mag XY from log")
    console.print()
    console.print("Run 'sf cal <subcommand> --help' for details.")
    return 0


def run_list(args: argparse.Namespace) -> int:
    """List available calibrations"""
    console.info("Available calibration types:")
    console.print()

    calibrations = [
        ("gyro", "Gyroscope bias", "Keep drone still for 2-3 seconds"),
        ("accel", "Accelerometer", "Place drone on level surface"),
        ("mag", "Magnetometer", "Rotate drone in all directions (figure-8)"),
        ("level", "Level reference", "Place on level surface, then land"),
    ]

    for cal_id, name, description in calibrations:
        console.print(f"  {cal_id:8s} - {name}")
        console.print(f"             {description}")
        console.print()

    console.print("Usage:")
    console.print("  sf cal <type>        # Run calibration via serial")
    console.print("  sf cal status        # Check calibration status")

    return 0


def run_gyro(args: argparse.Namespace) -> int:
    """Run gyroscope calibration"""
    port = args.port or _find_serial_port()
    if not port:
        console.error("No serial port found. Please specify with --port")
        return 1

    console.info("Gyroscope Bias Calibration")
    console.print(f"  Port: {port}")
    console.print()
    console.warning("Keep the drone COMPLETELY STILL during calibration!")
    console.print()

    return _send_calibration_command(port, "calib gyro")


def run_accel(args: argparse.Namespace) -> int:
    """Run accelerometer calibration"""
    port = args.port or _find_serial_port()
    if not port:
        console.error("No serial port found. Please specify with --port")
        return 1

    console.info("Accelerometer Calibration")
    console.print(f"  Port: {port}")
    console.print()
    console.warning("Place the drone on a LEVEL surface and keep still!")
    console.print()

    return _send_calibration_command(port, "calib accel")


def run_mag(args: argparse.Namespace) -> int:
    """Run magnetometer calibration"""
    port = args.port or _find_serial_port()
    if not port:
        console.error("No serial port found. Please specify with --port")
        return 1

    action = args.action

    console.info(f"Magnetometer Calibration - {action}")
    console.print(f"  Port: {port}")
    console.print()

    if action == "start":
        console.print("Instructions:")
        console.print("  1. After starting, rotate the drone in all directions")
        console.print("  2. Draw figure-8 patterns in the air")
        console.print("  3. Cover all orientations (takes ~30 seconds)")
        console.print("  4. Run 'sf cal mag stop' when done")
        console.print("  5. Run 'sf cal mag save' to save calibration")
        console.print()

    return _send_calibration_command(port, f"magcal {action}")


def run_level(args: argparse.Namespace) -> int:
    """Run level calibration"""
    port = args.port or _find_serial_port()
    if not port:
        console.error("No serial port found. Please specify with --port")
        return 1

    console.info("Level Calibration (Attitude Reference)")
    console.print(f"  Port: {port}")
    console.print()
    console.print("This calibration uses the landing detection to set attitude reference.")
    console.print("Place the drone on a level surface and it will auto-calibrate on landing.")
    console.print()

    # Level calibration uses the landing handler, not a direct command
    # Just show status for now
    return _send_calibration_command(port, "status")


def run_status(args: argparse.Namespace) -> int:
    """Show calibration status"""
    port = args.port or _find_serial_port()
    if not port:
        console.error("No serial port found. Please specify with --port")
        return 1

    console.info("Calibration Status")
    console.print(f"  Port: {port}")
    console.print()

    # Get sensor status
    return _send_calibration_command(port, "sensor all")


def run_plot(args: argparse.Namespace) -> int:
    """Plot magnetometer data"""
    file_path = args.file

    # Find latest binary log if not specified
    if not file_path:
        file_path = _find_latest_binlog()
        if not file_path:
            console.error("No binary log files found.")
            return 1
        console.info(f"Using latest log: {file_path}")

    path = Path(file_path)
    if not path.exists():
        console.error(f"File not found: {path}")
        return 1

    console.info(f"Plotting magnetometer data from: {path.name}")

    # Try to import and run plot_mag_xy
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "calibration"))
        import plot_mag_xy

        data = plot_mag_xy.load_binlog(str(path))
        if not data:
            console.error("No valid magnetometer data found in file")
            return 1

        console.print(f"  Samples: {len(data)}")
        plot_mag_xy.plot_mag_xy(data, args.output)
        return 0

    except ImportError as e:
        console.error(f"Failed to import plot_mag_xy: {e}")
        console.print("  Required: numpy matplotlib")
        return 1
    except Exception as e:
        console.error(f"Plot failed: {e}")
        return 1
    finally:
        if str(paths.root() / "tools" / "calibration") in sys.path:
            sys.path.remove(str(paths.root() / "tools" / "calibration"))


# --- Helper functions ---

def _find_serial_port() -> Optional[str]:
    """Find StampFly serial port"""
    patterns = [
        "/dev/tty.usbmodem*",
        "/dev/tty.usbserial*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    ]

    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]

    return None


def _find_latest_binlog() -> Optional[str]:
    """Find most recent binary log file"""
    search_dirs = [
        paths.root() / "logs",
        paths.root() / "tools" / "log_analyzer",
        paths.root() / "firmware" / "vehicle" / "logs",
    ]

    files = []
    for search_dir in search_dirs:
        if search_dir.exists():
            files.extend(search_dir.glob("*.bin"))

    if not files:
        return None

    files.sort(key=lambda f: f.stat().st_mtime, reverse=True)
    return str(files[0])


def _send_calibration_command(port: str, command: str) -> int:
    """Send command to device via serial and show response"""
    try:
        import serial
    except ImportError:
        console.error("pyserial required: pip install pyserial")
        return 1

    try:
        console.print(f"Sending: {command}")
        console.print()

        ser = serial.Serial(port, DEFAULT_BAUDRATE, timeout=1.0)

        # Clear buffers
        ser.reset_input_buffer()
        time.sleep(0.1)

        # Send command
        ser.write(f"{command}\r\n".encode())
        ser.flush()

        # Read response
        time.sleep(0.5)
        response_lines = []
        timeout = time.time() + 3.0

        while time.time() < timeout:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    response_lines.append(line)
                    console.print(f"  {line}")
            else:
                time.sleep(0.05)
                # Break if we got some response and no more data
                if response_lines and ser.in_waiting == 0:
                    time.sleep(0.2)
                    if ser.in_waiting == 0:
                        break

        ser.close()

        if not response_lines:
            console.warning("No response from device")

        console.print()
        console.success("Command sent successfully")
        return 0

    except serial.SerialException as e:
        console.error(f"Serial error: {e}")
        return 1
    except Exception as e:
        console.error(f"Error: {e}")
        return 1
