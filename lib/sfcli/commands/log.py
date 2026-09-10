"""
sf log - Log capture and analysis commands

Captures telemetry logs and provides analysis tools.
テレメトリログをキャプチャし、解析ツールを提供します。

Subcommands:
    list     - List captured log files
    capture  - Capture binary log via USB serial
    wifi     - Capture telemetry via WiFi UDP
    convert  - Convert binary log to CSV
    info     - Show log file information
    analyze  - Analyze flight log data (--health: motor-fault report)
    viz      - Visualize log data
"""

import argparse
import asyncio
import sys
from datetime import datetime
from pathlib import Path
from typing import Callable, List, Optional

from ..utils import console, paths, plotting

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
        help="Capture telemetry via WiFi UDP",
        description="Capture full-rate telemetry from StampFly via WiFi UDP.",
    )
    wifi_parser.add_argument(
        "-o", "--output",
        help="Output filename (auto-generated .jsonl if not specified). "
             "A .csv extension saves a merged Data Stream CSV (one row per "
             "400Hz cycle, the format `sf sysid fit` reads) instead of the "
             "default per-sample .jsonl",
    )
    wifi_parser.add_argument(
        "-d", "--duration",
        type=float,
        default=30.0,
        help="Capture duration in seconds (default: 30)",
    )
    wifi_parser.add_argument(
        "-i", "--ip",
        default="192.168.10.1",
        help="StampFly IP address (default: 192.168.10.1)",
    )
    wifi_parser.add_argument(
        "--port",
        type=int,
        default=8890,
        help="UDP telemetry port (default: 8890)",
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
        "--health",
        action="store_true",
        help="Motor health report: detect a degraded rotor from hover trim (JSONL)",
    )
    analyze_parser.add_argument(
        "--batch",
        action="store_true",
        help="With --health: analyze all JSONL logs for the CG-removed corner test",
    )
    analyze_parser.add_argument(
        "--json",
        action="store_true",
        help="With --health: emit a machine-readable JSON verdict",
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
    viz_parser.add_argument(
        "--show-invalid",
        action="store_true",
        help="Show invalid sensor data (default: hidden as gaps)",
    )
    viz_parser.add_argument(
        "-i", "--interactive",
        action="store_true",
        help="Interactive mode (Plotly, opens in browser)",
    )
    viz_parser.add_argument(
        "--layout",
        metavar="RxC",
        help="Tile layout as ROWSxCOLS for interactive mode (e.g., 3x2)",
    )
    viz_parser.add_argument(
        "--groups",
        nargs="+",
        help="Signal groups for interactive mode (e.g., attitude bias_gyro)",
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
    console.print("  wifi      Capture telemetry via WiFi UDP")
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
    """Capture telemetry via WiFi (UDP full-rate or legacy WebSocket)"""
    # Import the UDP capture module (primary)
    # UDP キャプチャモジュールをインポート（主要）
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))
        import udp_capture
    except ImportError as e:
        console.error(f"Failed to import udp_capture module: {e}")
        return 1
    finally:
        sys.path.pop(0)

    # Generate output filename if not specified
    output = args.output
    if not output and not args.no_save:
        timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
        output = str(get_log_dir() / f"stampfly_udp_{timestamp}.jsonl")

    port = getattr(args, 'port', 8890)
    console.info(f"Capturing UDP telemetry from {args.ip}:{port}")
    console.print(f"  Duration: {args.duration}s")
    if output:
        console.print(f"  Output: {output}")
    console.print()

    try:
        capture = udp_capture.UDPTelemetryCapture(args.ip, port)
        success = capture.capture(args.duration, udp_capture.progress_bar)
        print()  # Newline after progress bar

        if not success:
            console.error("No data received. Check WiFi connection and StampFly power.")
            return 1

        capture.print_stats()

        if not args.no_save and output:
            # .csv -> merged Data Stream CSV (one row per 400Hz cycle -- the
            # format `sf sysid fit` auto-detects); anything else (default
            # .jsonl) -> per-sample JSON Lines.
            # .csv -> マージ済み Data Stream CSV（400Hz周期1件=1行 —
            # `sf sysid fit` が自動判別する形式）; それ以外（既定 .jsonl）->
            # 1サンプル1行の JSON Lines。
            if output.lower().endswith(".csv"):
                capture.save_stream_csv(output)
            else:
                capture.save_jsonl(output)

        return 0

    except Exception as e:
        console.error(f"UDP capture failed: {e}")
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
    # Motor health report path (JSONL-based, detects a degraded rotor).
    # モータ健全性レポート（JSONL ベース、劣化ロータを検出）。
    if getattr(args, "health", False):
        return _run_motor_health(args)

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


def _run_motor_health(args: argparse.Namespace) -> int:
    """Run the motor health report (sf log analyze --health).
    モータ健全性レポートを実行する。

    Detects a degraded rotor from the steady hover trim. One log identifies
    the spin-direction group; --batch adds the CG-removed cross-log corner test.
    1ログで回転グループを判定、--batch でCG除去のクロスログ隅特定を追加。"""
    log_dir = get_log_dir()

    # The cross-log corner test assumes ONE airframe (constant CG). Default the
    # batch to the most-recent logs so an old/other airframe is not mixed in;
    # pass a glob to scope a specific session/airframe explicitly.
    # クロスログ隅特定は同一機体（CG一定）が前提。既定は最新ログに限定し、
    # 別機体の混入を避ける。特定セッションはグロブで明示する。
    MAX_BATCH = 12
    if args.batch:
        if args.file:
            # Explicit glob (scope to one airframe/session).
            jsonl_paths = sorted(str(p) for p in log_dir.glob(Path(args.file).name))
            if not jsonl_paths:
                from glob import glob as _glob
                jsonl_paths = sorted(_glob(args.file))
            if not jsonl_paths:
                console.error(f"No JSONL logs match: {args.file}")
                return 1
            console.info(f"Health report over {len(jsonl_paths)} JSONL logs (glob)")
        else:
            # Default: the most-recent JSONL logs (by mtime), capped.
            all_jsonl = sorted(log_dir.glob("*.jsonl"),
                               key=lambda f: f.stat().st_mtime, reverse=True)
            if not all_jsonl:
                console.error("No JSONL logs found for --batch.")
                return 1
            recent = all_jsonl[:MAX_BATCH]
            jsonl_paths = sorted(str(p) for p in recent)
            console.info(f"Health report over the {len(jsonl_paths)} most recent "
                         f"JSONL logs (assumes one airframe; pass a glob to scope)")
    else:
        file_path = args.file or _find_latest_log(extension=".jsonl")
        if not file_path:
            console.error("No JSONL log found. Capture one with 'sf log wifi'.")
            return 1
        if Path(file_path).suffix != ".jsonl":
            console.error("Health report requires a .jsonl log (per-motor duty). "
                          "Use 'sf log wifi' to capture, or --batch over the log dir.")
            return 1
        jsonl_paths = [str(file_path)]
        console.info(f"Health report: {Path(file_path).name}")

    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))
        import motor_health
        sys.path.pop(0)
        result = motor_health.analyze_health(jsonl_paths, json_out=args.json)
        return 0 if "error" not in result else 1
    except Exception as e:  # noqa: BLE001
        console.error(f"Health report failed: {e}")
        return 1


def run_viz(args: argparse.Namespace) -> int:
    """Visualize log data: resolve the file, then dispatch to the
    interactive (Plotly), JSONL, or CSV renderer.
    ログデータを可視化する: ファイルを解決し、インタラクティブ (Plotly)・
    JSONL・CSV いずれかの描画処理へ振り分ける。"""
    file_path = args.file

    # Find latest log file if not specified
    # 指定がなければ最新のログファイルを探す
    if not file_path:
        # Try JSONL first (new UDP format), then CSV (legacy)
        file_path = _find_latest_log(extension=".jsonl")
        if not file_path:
            file_path = _find_latest_log(extension=".csv")
        if not file_path:
            console.error("No log files found (.jsonl or .csv)")
            return 1
        console.info(f"Using latest log: {file_path}")

    path = Path(file_path)
    if not path.exists():
        console.error(f"File not found: {path}")
        return 1

    if path.suffix not in ('.csv', '.jsonl'):
        console.error("Visualization requires .csv or .jsonl file.")
        return 1

    console.info(f"Visualizing: {path.name}")

    # JSONL default: static overview. JSONL + -i: interactive Plotly.
    # JSONL デフォルト: 静的一覧。JSONL + -i: インタラクティブ Plotly。
    is_interactive = getattr(args, 'interactive', False)

    # JSONL static overview (default for .jsonl files)
    # JSONL 静的一覧表示（.jsonl ファイルのデフォルト）
    if path.suffix == '.jsonl' and not is_interactive:
        return _viz_jsonl(path, args)

    # Interactive mode (Plotly) -- works for both .csv and .jsonl, and
    # never touches matplotlib, so it needs no backend handling.
    # インタラクティブモード (Plotly) -- .csv と .jsonl の両方に対応し、
    # matplotlib には一切触れないためバックエンド対応は不要。
    if is_interactive:
        return _viz_interactive(path, args)

    return _viz_csv(path, args)


def _viz_interactive(path: Path, args: argparse.Namespace) -> int:
    """Interactive Plotly visualization. Unchanged by the matplotlib
    backend fallback work -- Plotly opens in a browser tab, not a
    matplotlib window, so there is no headless case to handle here.
    インタラクティブな Plotly 可視化。matplotlib バックエンドのフォール
    バック対応による変更なし -- Plotly はブラウザタブで開くため
    matplotlib のウィンドウではなく、ここで扱うべきヘッドレスの場合はない。
    """
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))
        import visualize_interactive

        layout = None
        if args.layout:
            try:
                parts = args.layout.lower().split('x')
                layout = (int(parts[0]), int(parts[1]))
            except (ValueError, IndexError):
                console.error(f"Invalid layout '{args.layout}'. Use ROWSxCOLS (e.g., 3x2)")
                return 1

        visualize_interactive.visualize(
            str(path),
            groups=args.groups,
            layout=layout,
            output=args.save,
        )
        return 0
    except ImportError as e:
        console.error(f"Failed to import interactive visualizer: {e}")
        console.print("  Required: pip install plotly")
        return 1
    except Exception as e:
        console.error(f"Interactive visualization failed: {e}")
        return 1
    finally:
        sys.path.pop(0)


def _viz_jsonl(path: Path, args: argparse.Namespace) -> int:
    """Static JSONL overview (matplotlib), with the headless PNG fallback.
    静的な JSONL 一覧表示（matplotlib）。GUIバックエンドが無ければPNGへ
    フォールバックする。"""
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))

        def render(save_path: Optional[str], show: bool) -> None:
            # visualize_jsonl imports matplotlib.pyplot at module load, so
            # it must not be imported before _render_with_fallback() has
            # picked a backend -- this closure only runs after that.
            # visualize_jsonl はモジュール読み込み時に matplotlib.pyplot を
            # import するため、_render_with_fallback() がバックエンドを
            # 選ぶより前に import してはならない -- このクロージャは
            # それより後にしか呼ばれない。
            import visualize_jsonl

            show_invalid = getattr(args, 'show_invalid', False)
            data = visualize_jsonl.load_jsonl(str(path), hide_invalid=not show_invalid)
            tr = None
            if hasattr(args, 'time_range') and args.time_range:
                tr = tuple(args.time_range)
            # plot_overview has no `show` argument: it shows when save is
            # None and saves otherwise, so `show` is unused here.
            # plot_overview に show 引数は無い: save が None なら表示、
            # そうでなければ保存するため、ここでは show を使わない。
            visualize_jsonl.plot_overview(
                data,
                title=path.name,
                save=save_path,
                time_range=tr,
            )

        return _render_with_fallback(path, args, render)
    except ImportError as e:
        console.error(f"Failed to import visualizer: {e}")
        console.print("  Required: pip install matplotlib numpy")
        return 1
    finally:
        sys.path.pop(0)


def _viz_csv(path: Path, args: argparse.Namespace) -> int:
    """CSV visualization: detect the format, then draw it with the
    matching visualize_* module (falls back to a saved PNG when no GUI
    backend is usable).
    CSV可視化: 書式を判定し、対応する visualize_* モジュールで描画する
    （GUIバックエンドが使えない場合は保存したPNGへフォールバックする）。"""
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))

        # Every visualize_* module below imports matplotlib.pyplot at
        # module load time, which locks in whatever backend is active at
        # that moment. Format detection itself needs visualize_stream (for
        # is_stream_csv()), so the backend must be chosen here, before that
        # first import. The result is handed to _render_with_fallback() so
        # the (Tk window-creating) probe runs only once.
        # 以下の visualize_* モジュールはいずれもモジュール読み込み時に
        # matplotlib.pyplot を import し、その時点で有効なバックエンドを
        # 固定してしまう。書式判定自体が visualize_stream（is_stream_csv()
        # 用）を必要とするため、その最初の import より前にここでバックエンドを
        # 選んでおく。結果は _render_with_fallback() に渡し、（Tk ウィンドウを
        # 作る）プローブが一度しか走らないようにする。
        backend = plotting.select_backend(want_window=args.save is None)

        # Detect CSV format to choose appropriate visualizer
        import csv
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            columns = reader.fieldnames

        # Data Stream CSV (sf log wifi -o *.csv) - 400Hz IMU+ESKF merged with
        # rate_ref and the 50Hz CtrlRef. Must be checked before the extended
        # format because it also carries timestamp_us + quat_w.
        # Data Stream CSV（sf log wifi -o *.csv）- 400Hz IMU+ESKF に rate_ref と
        # 50Hz CtrlRef をマージした形式。timestamp_us + quat_w も持つため、
        # extended 形式より先に判定する。
        import visualize_stream
        if visualize_stream.is_stream_csv(columns):
            console.info("Detected: Data Stream CSV (sf log wifi -o *.csv, 400Hz)")
            df = visualize_stream.load_stream_csv(str(path))

            def render(save_path: Optional[str], show: bool) -> None:
                visualize_stream.visualize_all(
                    df, str(path), save_path=save_path, show=show,
                    time_range=args.time_range, mode=args.mode,
                )
        # Extended format (400Hz with ESKF) - has timestamp_us and quat_w
        elif 'timestamp_us' in columns and 'quat_w' in columns:
            import visualize_extended
            console.info("Detected: Extended telemetry (400Hz with ESKF)")
            data, fmt = visualize_extended.load_csv(str(path))

            def render(save_path: Optional[str], show: bool) -> None:
                # plot_extended has no `show` argument: passing a save path
                # makes it save instead of show -- same behaviour as before
                # this refactor, so `show` is unused here.
                # plot_extended に show 引数は無い: 保存パスを渡すと表示の
                # 代わりに保存する -- この改修前と同じ挙動のため、ここでは
                # show を使わない。
                visualize_extended.plot_extended(
                    data,
                    output_file=save_path,
                    time_range=args.time_range,
                    show_eskf=not args.no_eskf,
                    show_sensors=not args.no_sensors,
                )
        # FFT batch format - has timestamp_ms and gyro_corrected_x
        elif 'timestamp_ms' in columns and 'gyro_corrected_x' in columns:
            import visualize_extended
            console.info("Detected: FFT batch telemetry")
            data, fmt = visualize_extended.load_csv(str(path))

            def render(save_path: Optional[str], show: bool) -> None:
                visualize_extended.plot_legacy(data, fmt, output_file=save_path)
        # Normal WiFi telemetry - has timestamp_ms and roll_deg
        elif 'timestamp_ms' in columns and 'roll_deg' in columns:
            import visualize_telemetry
            console.info("Detected: Normal WiFi telemetry")
            df = visualize_telemetry.load_telemetry_csv(str(path))

            def render(save_path: Optional[str], show: bool) -> None:
                if args.mode == "sensors":
                    visualize_telemetry.visualize_sensors_only(df, str(path), save_path, show)
                elif args.mode == "attitude":
                    visualize_telemetry.visualize_attitude_only(df, str(path), save_path, show)
                elif args.mode == "position":
                    visualize_telemetry.visualize_position_only(df, str(path), save_path, show)
                else:
                    visualize_telemetry.visualize_all(df, str(path), save_path, show)
        # SILS trajectory.csv (sf sils scenario output) - has t, px, alt, roll,
        # yawrate, yawcmd, alt_est, m0-m3. Checked as a set (column order is not
        # guaranteed) against a combination distinctive enough not to collide
        # with the WiFi/extended/FFT formats above.
        # SILS trajectory.csv（sf sils scenario の出力）- t, px, alt, roll, yawrate,
        # yawcmd, alt_est, m0-m3 を持つ。列順は保証されないため集合として判定し、
        # 上の WiFi/extended/FFT 各書式と衝突しない組み合わせを使う。
        elif {
            't', 'px', 'py', 'pz', 'qw', 'alt', 'roll', 'pitch',
            'yawrate', 'yawcmd', 'alt_est', 'm0', 'm1', 'm2', 'm3',
        }.issubset(set(columns)):
            import visualize_sils_trajectory
            console.info("Detected: SILS trajectory (sf sils scenario)")
            df = visualize_sils_trajectory.load_trajectory_csv(str(path))

            def render(save_path: Optional[str], show: bool) -> None:
                visualize_sils_trajectory.visualize_all(
                    df, str(path), save_path=save_path, show=show,
                )
        else:
            # Unknown format: bail out before _render_with_fallback() so
            # this case never triggers backend-selection or PNG fallback
            # logic -- there is nothing renderable to fall back to.
            # 未知の書式: _render_with_fallback() を呼ぶ前に打ち切ることで、
            # このケースがバックエンド選択やPNGフォールバックの経路に
            # 入らないようにする -- フォールバックできる描画対象がそもそもない。
            console.error("Unknown CSV format. Cannot determine visualizer.")
            return 1

        return _render_with_fallback(path, args, render, backend)

    except ImportError as e:
        console.error(f"Failed to import visualization module: {e}")
        console.print("  Required: matplotlib, numpy")
        return 1
    except Exception as e:
        # Covers format-detection failures (bad/unreadable CSV) -- errors
        # from render() itself are already handled inside
        # _render_with_fallback() and never reach this far.
        # 書式判定自体の失敗（壊れた/読めないCSV）を捕捉する -- render()
        # 自体のエラーは _render_with_fallback() 内で既に処理済みで、
        # ここまでは届かない。
        console.error(f"Visualization failed: {e}")
        return 1
    finally:
        sys.path.pop(0)


def _render_with_fallback(
    path: Path,
    args: argparse.Namespace,
    render: Callable[[Optional[str], bool], None],
    backend: Optional[plotting.BackendInfo] = None,
) -> int:
    """Draw one figure set via `render(save_path, show)`, choosing the
    matplotlib backend first. Falls back to a PNG saved next to the log
    (opened with the OS default viewer) when no window can be shown, and
    retries headlessly once if a GUI backend passes its import-time probe
    but still fails while actually drawing/showing.
    `render(save_path, show)` で1つの図を描く。まず matplotlib バックエンドを
    選ぶ。ウィンドウを表示できない場合はログの隣に PNG を保存して
    （OS標準の画像ビューアで開く）フォールバックし、GUIバックエンドが
    import時のプローブは通過したのに実際の描画/表示で失敗した場合は
    一度だけヘッドレスで再試行する。

    `render` must not import any matplotlib.pyplot-importing module until
    it is actually called -- the backend must be selected first, either by
    the caller (passed as `backend`) or here.
    `render` は実際に呼ばれるまで matplotlib.pyplot を import するモジュール
    を import してはならない -- バックエンドの選択は、呼び出し側（`backend`
    で渡す）かこの関数が先に行う。
    """
    want_window = args.save is None
    info = backend or plotting.select_backend(want_window=want_window)

    save_path, show = args.save, want_window
    opened_fallback = False
    if want_window and info.interactive:
        # One line so a user can see which GUI backend the window uses
        # (macosx / tkagg / qtagg) without any extra flag.
        # 追加のフラグ無しで、ウィンドウがどの GUI バックエンド
        # （macosx / tkagg / qtagg）で開くかを 1 行で示す。
        console.info(f"Plot window backend: {info.name}")
    if want_window and not info.interactive:
        save_path = str(plotting.default_png_path(path))
        show = False
        opened_fallback = True
        plotting.report_headless(console, info, Path(save_path))

    try:
        render(save_path, show)
    except Exception as first_error:  # noqa: BLE001 - draw-time errors must not crash sf
        if not show:
            console.error(f"Visualization failed: {first_error}")
            return 1
        # The GUI backend passed the probe but failed while drawing/showing
        # (e.g. a Tk/Qt runtime error): retry the same render headlessly once.
        # GUIバックエンドはプローブを通過したが描画/表示時に失敗した
        # （Tk/Qtの実行時エラー等）: 同じ描画を一度だけヘッドレスで再試行する。
        plotting.force_headless()
        save_path = str(plotting.default_png_path(path))
        opened_fallback = True
        try:
            render(save_path, False)
        except Exception:  # noqa: BLE001 - report the ORIGINAL error, not the retry's
            console.error(f"Visualization failed: {first_error}")
            return 1
        console.warning(f"Plot window failed ({first_error}); saved the plot to {save_path} instead.")

    if opened_fallback:
        plotting.open_with_default_viewer(Path(save_path))
    return 0


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
