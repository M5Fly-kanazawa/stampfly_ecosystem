"""
sf log - Log capture and analysis commands

Captures telemetry logs and provides analysis tools.
テレメトリログをキャプチャし、解析ツールを提供します。

Subcommands:
    list     - List captured log files
    capture  - Capture binary log via USB serial
    wifi     - Capture telemetry via WiFi WebSocket
    convert  - Convert binary log to CSV
    info     - Show log file information
    analyze  - Analyze flight log data
    viz      - Visualize log data
"""

import argparse
import asyncio
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional, List

from ..utils import console, paths

COMMAND_NAME = "log"
COMMAND_HELP = "Log capture and analysis"

# Default log directory
DEFAULT_LOG_DIR = "logs"


def get_log_dir() -> Path:
    """Get log directory path, create if needed"""
    log_dir = paths.root() / DEFAULT_LOG_DIR
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Create sub-subparsers for log subcommands
    log_subparsers = parser.add_subparsers(
        dest="log_command",
        title="subcommands",
        metavar="<subcommand>",
    )

    # --- list ---
    list_parser = log_subparsers.add_parser(
        "list",
        help="List captured log files",
        description="List all log files in the logs directory.",
    )
    list_parser.add_argument(
        "-n", "--limit",
        type=int,
        default=20,
        help="Number of recent files to show (default: 20)",
    )
    list_parser.add_argument(
        "--all",
        action="store_true",
        help="Show all files (ignore limit)",
    )
    list_parser.set_defaults(func=run_list)

    # --- capture ---
    capture_parser = log_subparsers.add_parser(
        "capture",
        help="Capture binary log via USB serial",
        description="Capture binary sensor log from StampFly via USB serial port.",
    )
    capture_parser.add_argument(
        "-p", "--port",
        help="Serial port (auto-detect if not specified)",
    )
    capture_parser.add_argument(
        "-o", "--output",
        help="Output filename (auto-generated if not specified)",
    )
    capture_parser.add_argument(
        "-d", "--duration",
        type=float,
        default=60.0,
        help="Capture duration in seconds (default: 60)",
    )
    capture_parser.add_argument(
        "-b", "--baudrate",
        type=int,
        default=115200,
        help="Baudrate (default: 115200)",
    )
    capture_parser.add_argument(
        "--live",
        action="store_true",
        help="Show live packet data",
    )
    capture_parser.add_argument(
        "--no-auto",
        action="store_true",
        help="Do not auto-send binlog on/off commands",
    )
    capture_parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug output",
    )
    capture_parser.set_defaults(func=run_capture)

    # --- wifi ---
    wifi_parser = log_subparsers.add_parser(
        "wifi",
        help="Capture telemetry via WiFi WebSocket",
        description="Capture high-rate telemetry from StampFly via WiFi WebSocket.",
    )
    wifi_parser.add_argument(
        "-o", "--output",
        help="Output CSV filename (auto-generated if not specified)",
    )
    wifi_parser.add_argument(
        "-d", "--duration",
        type=float,
        default=30.0,
        help="Capture duration in seconds (default: 30)",
    )
    wifi_parser.add_argument(
        "-i", "--ip",
        default="192.168.4.1",
        help="StampFly IP address (default: 192.168.4.1)",
    )
    wifi_parser.add_argument(
        "--port",
        type=int,
        default=80,
        help="WebSocket port (default: 80)",
    )
    wifi_parser.add_argument(
        "--fft",
        action="store_true",
        help="Run FFT analysis after capture",
    )
    wifi_parser.add_argument(
        "--no-save",
        action="store_true",
        help="Don't save to file, just display stats",
    )
    wifi_parser.set_defaults(func=run_wifi)

    # --- convert ---
    convert_parser = log_subparsers.add_parser(
        "convert",
        help="Convert binary log to CSV",
        description="Convert binary log file (.bin) to CSV format.",
    )
    convert_parser.add_argument(
        "input",
        help="Input binary log file (.bin)",
    )
    convert_parser.add_argument(
        "-o", "--output",
        help="Output CSV file (default: same name with .csv)",
    )
    convert_parser.set_defaults(func=run_convert)

    # --- info ---
    info_parser = log_subparsers.add_parser(
        "info",
        help="Show log file information",
        description="Display information about a log file.",
    )
    info_parser.add_argument(
        "file",
        nargs="?",
        help="Log file path (default: latest)",
    )
    info_parser.set_defaults(func=run_info)

    # --- analyze ---
    analyze_parser = log_subparsers.add_parser(
        "analyze",
        help="Analyze flight log data",
        description="Analyze flight log for stability, oscillation, and tuning insights.",
    )
    analyze_parser.add_argument(
        "file",
        nargs="?",
        help="Log file path (default: latest CSV)",
    )
    analyze_parser.add_argument(
        "--fft",
        action="store_true",
        help="Run FFT analysis",
    )
    analyze_parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip visualization",
    )
    analyze_parser.set_defaults(func=run_analyze)

    # --- viz ---
    viz_parser = log_subparsers.add_parser(
        "viz",
        help="Visualize log data",
        description="Visualize telemetry log data with comprehensive plots.",
    )
    viz_parser.add_argument(
        "file",
        nargs="?",
        help="Log file path (default: latest CSV)",
    )
    viz_parser.add_argument(
        "--mode",
        choices=["all", "sensors", "attitude", "position", "eskf"],
        default="all",
        help="Visualization mode (default: all)",
    )
    viz_parser.add_argument(
        "--save",
        metavar="FILE",
        help="Save plot to file instead of displaying",
    )
    viz_parser.add_argument(
        "--time-range",
        nargs=2,
        type=float,
        metavar=("START", "END"),
        help="Time range to plot (seconds)",
    )
    viz_parser.add_argument(
        "--no-eskf",
        action="store_true",
        help="Hide ESKF panels",
    )
    viz_parser.add_argument(
        "--no-sensors",
        action="store_true",
        help="Hide additional sensor panels (baro, tof, flow)",
    )
    viz_parser.set_defaults(func=run_viz)

    parser.set_defaults(func=run_help)


def run_help(args: argparse.Namespace) -> int:
    """Show help when no subcommand specified"""
    console.print("Usage: sf log <subcommand> [options]")
    console.print()
    console.print("Subcommands:")
    console.print("  list      List captured log files")
    console.print("  capture   Capture binary log via USB serial")
    console.print("  wifi      Capture telemetry via WiFi WebSocket")
    console.print("  convert   Convert binary log to CSV")
    console.print("  info      Show log file information")
    console.print("  analyze   Analyze flight log data")
    console.print("  viz       Visualize log data")
    console.print()
    console.print("Run 'sf log <subcommand> --help' for details.")
    return 0


def run_list(args: argparse.Namespace) -> int:
    """List log files"""
    log_dir = get_log_dir()
    analyzer_dir = paths.root() / "tools" / "log_analyzer"

    # Collect all log files
    files = []

    # From logs/
    for pattern in ["*.bin", "*.csv"]:
        files.extend(log_dir.glob(pattern))

    # From tools/log_analyzer/
    if analyzer_dir.exists():
        for pattern in ["*.bin", "*.csv"]:
            files.extend(analyzer_dir.glob(pattern))

    if not files:
        console.info("No log files found.")
        console.print(f"  Directories searched:")
        console.print(f"    - {log_dir}")
        console.print(f"    - {analyzer_dir}")
        return 0

    # Sort by modification time (newest first)
    files.sort(key=lambda f: f.stat().st_mtime, reverse=True)

    # Apply limit
    if not args.all:
        files = files[:args.limit]

    console.info(f"Log files (showing {len(files)} most recent):")
    console.print()

    for f in files:
        stat = f.stat()
        size_kb = stat.st_size / 1024
        mtime = datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
        ext = f.suffix

        # Color based on type
        if ext == ".bin":
            type_str = "[BIN]"
        else:
            type_str = "[CSV]"

        console.print(f"  {type_str} {f.name:45s} {size_kb:8.1f} KB  {mtime}")

    console.print()
    console.print(f"Log directories:")
    console.print(f"  - {log_dir}")
    console.print(f"  - {analyzer_dir}")

    return 0


def run_capture(args: argparse.Namespace) -> int:
    """Capture binary log via USB serial"""
    # Import the capture module
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_capture"))
        import log_capture
    except ImportError as e:
        console.error(f"Failed to import log_capture module: {e}")
        return 1
    finally:
        sys.path.pop(0)

    # Auto-detect port if not specified
    port = args.port
    if not port:
        port = _find_serial_port()
        if not port:
            console.error("No serial port found. Please specify with --port")
            return 1
        console.info(f"Auto-detected port: {port}")

    # Generate output filename if not specified
    output = args.output
    if not output:
        timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
        output = str(get_log_dir() / f"stampfly_{timestamp}.bin")

    console.info(f"Capturing binary log from {port}")
    console.print(f"  Duration: {args.duration}s")
    console.print(f"  Output: {output}")
    console.print()

    try:
        log_capture.capture_log(
            port=port,
            output=output,
            duration=args.duration,
            baudrate=args.baudrate,
            show_live=args.live,
            auto_control=not args.no_auto,
            debug=args.debug,
        )
        return 0
    except Exception as e:
        console.error(f"Capture failed: {e}")
        return 1


def run_wifi(args: argparse.Namespace) -> int:
    """Capture telemetry via WiFi WebSocket"""
    # Import the wifi capture module
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))
        import wifi_capture
    except ImportError as e:
        console.error(f"Failed to import wifi_capture module: {e}")
        console.print("  Make sure 'websockets' is installed: pip install websockets")
        return 1
    finally:
        sys.path.pop(0)

    # Generate output filename if not specified
    output = args.output
    if not output and not args.no_save:
        timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
        output = str(get_log_dir() / f"stampfly_wifi_{timestamp}.csv")

    console.info(f"Capturing WiFi telemetry from {args.ip}:{args.port}")
    console.print(f"  Duration: {args.duration}s")
    if output:
        console.print(f"  Output: {output}")
    console.print()

    try:
        # Create capture instance
        capture = wifi_capture.TelemetryCapture(args.ip, args.port)

        # Run async capture
        success = asyncio.run(capture.capture(args.duration, wifi_capture.progress_bar))
        print()  # Newline after progress bar

        if not success:
            return 1

        # Print stats
        capture.print_stats()

        # Save to file
        if not args.no_save and output:
            capture.save_csv(output)

        # Run FFT if requested
        if args.fft:
            capture.run_fft_analysis()

        return 0

    except Exception as e:
        console.error(f"WiFi capture failed: {e}")
        return 1


def run_convert(args: argparse.Namespace) -> int:
    """Convert binary log to CSV"""
    # Import the capture module for conversion
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_capture"))
        import log_capture
    except ImportError as e:
        console.error(f"Failed to import log_capture module: {e}")
        return 1
    finally:
        sys.path.pop(0)

    input_path = Path(args.input)
    if not input_path.exists():
        console.error(f"Input file not found: {input_path}")
        return 1

    # Generate output filename if not specified
    output = args.output
    if not output:
        output = str(input_path.with_suffix(".csv"))

    console.info(f"Converting {input_path.name} to CSV...")

    try:
        log_capture.convert_to_csv(str(input_path), output)
        console.success(f"Converted to: {output}")
        return 0
    except Exception as e:
        console.error(f"Conversion failed: {e}")
        return 1


def run_info(args: argparse.Namespace) -> int:
    """Show log file information"""
    file_path = args.file

    # Find latest file if not specified
    if not file_path:
        file_path = _find_latest_log()
        if not file_path:
            console.error("No log files found.")
            return 1
        console.info(f"Using latest log: {file_path}")

    path = Path(file_path)
    if not path.exists():
        console.error(f"File not found: {path}")
        return 1

    if path.suffix == ".bin":
        return _show_bin_info(path)
    elif path.suffix == ".csv":
        return _show_csv_info(path)
    else:
        console.error(f"Unsupported file type: {path.suffix}")
        return 1


def run_analyze(args: argparse.Namespace) -> int:
    """Analyze flight log"""
    file_path = args.file

    # Find latest CSV if not specified
    if not file_path:
        file_path = _find_latest_log(extension=".csv")
        if not file_path:
            console.error("No CSV log files found.")
            return 1
        console.info(f"Using latest CSV: {file_path}")

    path = Path(file_path)
    if not path.exists():
        console.error(f"File not found: {path}")
        return 1

    if path.suffix != ".csv":
        console.error("Analysis requires CSV file. Use 'sf log convert' first.")
        return 1

    console.info(f"Analyzing: {path.name}")

    try:
        # Import analysis module
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))
        import flight_analysis
        sys.path.pop(0)

        # Run analysis
        flight_analysis.analyze_flight(str(path))
        return 0

    except ImportError as e:
        console.error(f"Failed to import analysis module: {e}")
        console.print("  Required: pandas, matplotlib, scipy")
        return 1
    except Exception as e:
        console.error(f"Analysis failed: {e}")
        return 1


def run_viz(args: argparse.Namespace) -> int:
    """Visualize log data"""
    file_path = args.file

    # Find latest CSV if not specified
    if not file_path:
        file_path = _find_latest_log(extension=".csv")
        if not file_path:
            console.error("No CSV log files found.")
            return 1
        console.info(f"Using latest CSV: {file_path}")

    path = Path(file_path)
    if not path.exists():
        console.error(f"File not found: {path}")
        return 1

    if path.suffix != ".csv":
        console.error("Visualization requires CSV file. Use 'sf log convert' first.")
        return 1

    console.info(f"Visualizing: {path.name}")

    try:
        # Import visualization module
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))

        # Detect CSV format to choose appropriate visualizer
        import csv
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            columns = reader.fieldnames

        # Extended format (400Hz with ESKF) - has timestamp_us and quat_w
        if 'timestamp_us' in columns and 'quat_w' in columns:
            import visualize_extended
            console.info("Detected: Extended telemetry (400Hz with ESKF)")

            data, fmt = visualize_extended.load_csv(str(path))
            visualize_extended.plot_extended(
                data,
                output_file=args.save,
                time_range=args.time_range,
                show_eskf=not args.no_eskf,
                show_sensors=not args.no_sensors,
            )
        # FFT batch format - has timestamp_ms and gyro_corrected_x
        elif 'timestamp_ms' in columns and 'gyro_corrected_x' in columns:
            import visualize_extended
            console.info("Detected: FFT batch telemetry")

            data, fmt = visualize_extended.load_csv(str(path))
            visualize_extended.plot_legacy(data, fmt, output_file=args.save)
        # Normal WiFi telemetry - has timestamp_ms and roll_deg
        elif 'timestamp_ms' in columns and 'roll_deg' in columns:
            import visualize_telemetry
            console.info("Detected: Normal WiFi telemetry")

            # Load data and call appropriate function
            df = visualize_telemetry.load_telemetry_csv(str(path))
            show = args.save is None

            if args.mode == "sensors":
                visualize_telemetry.visualize_sensors_only(df, str(path), args.save, show)
            elif args.mode == "attitude":
                visualize_telemetry.visualize_attitude_only(df, str(path), args.save, show)
            elif args.mode == "position":
                visualize_telemetry.visualize_position_only(df, str(path), args.save, show)
            else:
                visualize_telemetry.visualize_all(df, str(path), args.save, show)
        else:
            console.error("Unknown CSV format. Cannot determine visualizer.")
            return 1

        sys.path.pop(0)
        return 0

    except ImportError as e:
        console.error(f"Failed to import visualization module: {e}")
        console.print("  Required: matplotlib, numpy")
        return 1
    except Exception as e:
        console.error(f"Visualization failed: {e}")
        return 1


# --- Helper functions ---

def _find_serial_port() -> Optional[str]:
    """Find StampFly serial port"""
    import glob

    # Common patterns for ESP32
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


def _find_latest_log(extension: Optional[str] = None) -> Optional[str]:
    """Find most recent log file"""
    log_dir = get_log_dir()
    analyzer_dir = paths.root() / "tools" / "log_analyzer"

    files = []

    if extension:
        patterns = [f"*{extension}"]
    else:
        patterns = ["*.bin", "*.csv"]

    for pattern in patterns:
        files.extend(log_dir.glob(pattern))
        if analyzer_dir.exists():
            files.extend(analyzer_dir.glob(pattern))

    if not files:
        return None

    # Return newest
    files.sort(key=lambda f: f.stat().st_mtime, reverse=True)
    return str(files[0])


def _show_bin_info(path: Path) -> int:
    """Show binary log info"""
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_capture"))
        import log_capture
        sys.path.pop(0)

        packets = log_capture.parse_log_file(str(path))
        if not packets:
            console.error("No valid packets found in file")
            return 1

        import math

        console.print(f"File: {path.name}")
        console.print(f"Size: {path.stat().st_size / 1024:.1f} KB")
        console.print(f"Packets: {len(packets)}")

        duration = (packets[-1].timestamp_ms - packets[0].timestamp_ms) / 1000.0
        console.print(f"Duration: {duration:.2f} seconds")
        console.print(f"Rate: {len(packets) / duration:.1f} Hz")

        console.print()
        console.print("First packet:")
        console.print(f"  {packets[0]}")
        console.print("Last packet:")
        console.print(f"  {packets[-1]}")

        return 0

    except Exception as e:
        console.error(f"Failed to parse binary log: {e}")
        return 1


def _show_csv_info(path: Path) -> int:
    """Show CSV log info"""
    try:
        import pandas as pd
    except ImportError:
        console.error("pandas required for CSV analysis: pip install pandas")
        return 1

    try:
        df = pd.read_csv(path)

        console.print(f"File: {path.name}")
        console.print(f"Size: {path.stat().st_size / 1024:.1f} KB")
        console.print(f"Samples: {len(df)}")
        console.print(f"Columns: {len(df.columns)}")

        if 'timestamp_ms' in df.columns:
            duration = (df['timestamp_ms'].iloc[-1] - df['timestamp_ms'].iloc[0]) / 1000.0
            console.print(f"Duration: {duration:.2f} seconds")
            console.print(f"Rate: {len(df) / duration:.1f} Hz")

        console.print()
        console.print("Columns:")
        for col in df.columns:
            console.print(f"  - {col}")

        return 0

    except Exception as e:
        console.error(f"Failed to read CSV: {e}")
        return 1
