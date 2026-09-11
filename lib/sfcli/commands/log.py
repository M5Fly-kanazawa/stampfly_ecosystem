"""
sf log - Log capture and analysis commands

Captures telemetry logs and provides analysis tools. The primary record is
a StampFly flight-log v1 bundle (`.sflog.zip`; docs/plans/
flight-log-format-plan.md): one zip with a CSV per signal, plus meta.json/
schema.json. Aligned/JSONL files are derived products made on demand by
`sf log convert`, never written as the primary capture.
テレメトリログをキャプチャし、解析ツールを提供します。一次記録は StampFly
フライトログ v1 一式（`.sflog.zip`；計画書参照）: 信号ごとの CSV と
meta.json/schema.json をまとめた zip 1個。整列表/JSONL は `sf log convert`
が必要なときだけ作る派生物であり、一次記録として書くことはない。

Subcommands:
    list     - List captured flight-log bundles
    wifi     - Capture telemetry via WiFi UDP, saved as a bundle
    check    - Validate a bundle's structure/units (lib/sflog.check_bundle)
    convert  - Convert JSONL<->bundle, or a bundle -> aligned/JSONL CSV
    info     - Show a bundle's summary (meta.json + per-stream stats)
    analyze  - Analyze flight log data (--health: motor-fault report)
    viz      - Visualize log data
"""

import argparse
import asyncio
import sys
from datetime import datetime
from pathlib import Path
from typing import Callable, List, Optional

import sflog

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

    # --- wifi ---
    wifi_parser = log_subparsers.add_parser(
        "wifi",
        help="Capture telemetry via WiFi UDP",
        description="Capture full-rate telemetry from StampFly via WiFi UDP, "
                     "saved as a StampFly flight-log v1 bundle (.sflog.zip).",
    )
    wifi_parser.add_argument(
        "-o", "--output",
        help="Output path (auto-generated logs/flight_<timestamp>.sflog.zip "
             "if not specified). A path ending in .sflog.zip writes that "
             "zip; an existing directory (or a path ending in a path "
             "separator) gets a same-named bundle written inside it. "
             "Other extensions (.csv/.jsonl/.bin) are rejected -- the "
             "bundle is the only capture format; use `sf log convert "
             "--aligned` or `--jsonl` afterwards for a derived file.",
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

    # --- check ---
    check_parser = log_subparsers.add_parser(
        "check",
        help="Validate a flight-log bundle",
        description="Check a StampFly flight-log v1 bundle's structure, "
                     "units, and timing (lib/sflog.check_bundle).",
    )
    check_parser.add_argument(
        "bundle",
        nargs="?",
        help="Bundle path, .sflog.zip or directory (default: newest in logs/)",
    )
    check_parser.set_defaults(func=run_check)

    # --- convert ---
    convert_parser = log_subparsers.add_parser(
        "convert",
        help="Convert between flight-log formats",
        description="Convert a legacy .jsonl log into a v1 bundle, or a "
                     "bundle into a derived aligned/JSONL CSV.",
    )
    convert_parser.add_argument(
        "input",
        help="Input file: a legacy .jsonl log, or a bundle (.sflog.zip/directory)",
    )
    convert_parser.add_argument(
        "-o", "--output",
        help="Output path (default: derived from the input name)",
    )
    convert_parser.add_argument(
        "--aligned",
        action="store_true",
        help="Bundle input -> a derived <stem>_aligned<rate>.csv "
             "(one row per --base sample, other streams held/nearest-matched)",
    )
    convert_parser.add_argument(
        "--base",
        default="imu",
        help="With --aligned: base stream whose timestamps become the "
             "aligned table's rows (default: imu)",
    )
    convert_parser.add_argument(
        "--method",
        choices=["hold", "nearest"],
        default="hold",
        help="With --aligned: how non-lockstep streams are matched to the "
             "base timestamps (default: hold)",
    )
    convert_parser.add_argument(
        "--jsonl",
        action="store_true",
        help="Bundle input -> a derived legacy-format .jsonl "
             "(for analysis/scripts/ research scripts not yet on lib/sflog)",
    )
    convert_parser.set_defaults(func=run_convert)

    # --- info ---
    info_parser = log_subparsers.add_parser(
        "info",
        help="Show flight-log bundle information",
        description="Display a bundle's meta.json summary and per-stream stats.",
    )
    info_parser.add_argument(
        "bundle",
        nargs="?",
        help="Bundle path, .sflog.zip or directory (default: newest in logs/)",
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
    console.print("  list      List captured flight-log bundles")
    console.print("  wifi      Capture telemetry via WiFi UDP (-> bundle)")
    console.print("  check     Validate a flight-log bundle")
    console.print("  convert   Convert JSONL<->bundle, or bundle->aligned/JSONL CSV")
    console.print("  info      Show flight-log bundle information")
    console.print("  analyze   Analyze flight log data")
    console.print("  viz       Visualize log data")
    console.print()
    console.print("Run 'sf log <subcommand> --help' for details.")
    return 0


def run_list(args: argparse.Namespace) -> int:
    """List flight-log bundles (`*.sflog.zip` files and directory bundles)
    under logs/, newest first.
    logs/ 配下のフライトログ一式（`*.sflog.zip` ファイルおよびディレクトリ
    一式）を新しい順に一覧表示する。
    """
    log_dir = get_log_dir()
    bundles = list(_iter_bundles(log_dir))

    if not bundles:
        console.info("No flight-log bundles found.")
        console.print(f"  Directory searched: {log_dir}")
        return 0

    bundles.sort(key=lambda item: item[0].stat().st_mtime, reverse=True)
    if not args.all:
        bundles = bundles[:args.limit]

    console.info(f"Flight-log bundles (showing {len(bundles)} most recent):")
    console.print()
    console.print(f"  {'Name':<40s} {'Size':>9s}  {'Source':<8s} {'Created':<19s} {'Dur(s)':>7s} {'Streams':>7s}")

    for path, meta in bundles:
        size_kb = _bundle_size_kb(path)
        source = meta.get("source") or "?"
        created_at = _format_created_at(meta.get("created_at"))
        duration_s = _bundle_duration_s(meta)
        n_streams = len(meta.get("streams") or {})
        console.print(
            f"  {path.name:<40s} {size_kb:8.1f}K  {source:<8s} {created_at:<19s} "
            f"{duration_s:7.1f} {n_streams:7d}"
        )

    console.print()
    console.print(f"Log directory: {log_dir}")
    return 0


def run_wifi(args: argparse.Namespace) -> int:
    """Capture telemetry via WiFi UDP, saved as a StampFly flight-log v1
    bundle (`.sflog.zip`; docs/plans/flight-log-format-plan.md).
    WiFi UDP でテレメトリを取得し、StampFly フライトログ v1 一式
    （`.sflog.zip`；計画書参照）として保存する。
    """
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

    port = getattr(args, "port", 8890)

    output = None
    if not args.no_save:
        try:
            output = _resolve_wifi_output(args.output, get_log_dir())
        except ValueError as e:
            console.error(str(e))
            return 1

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

        if output is None:
            return 0

        capture.save_bundle(
            str(output),
            tool_name="sf log wifi",
            capture_info={"requested_duration_s": args.duration},
        )
        console.success(f"Saved: {output}")

        # Validate the freshly written bundle -- catches a writer bug (or a
        # firmware wire-format drift parse_packet() silently tolerated)
        # before the user carries a bad capture into analysis.
        # 書いたばかりの一式を検証する -- 解析に持ち込む前に、書き出し側の
        # バグ（または parse_packet() が黙って許容したファーム側の電文
        # 形式のずれ）を検出する。
        findings = sflog.check_bundle(output)
        for finding in findings:
            console.print(str(finding))
        if not sflog.is_ok(findings):
            console.error("Bundle failed validation (sf log check) -- see errors above.")
            return 1

        return 0

    except Exception as e:
        console.error(f"UDP capture failed: {e}")
        return 1


def run_check(args: argparse.Namespace) -> int:
    """Validate a flight-log bundle's structure, units, and timing
    (`lib/sflog.check_bundle`).
    フライトログ一式の構造・単位・時刻整合性を検査する
    （`lib/sflog.check_bundle`）。
    """
    bundle_path = _resolve_bundle_arg(args.bundle)
    if bundle_path is None:
        return 1

    findings = sflog.check_bundle(bundle_path)
    for finding in findings:
        console.print(str(finding))

    n_errors = sum(1 for f in findings if f.level == "error")
    n_warnings = sum(1 for f in findings if f.level == "warning")
    console.print()
    if not findings:
        console.success(f"{bundle_path.name}: no issues found")
        return 0

    console.print(f"{n_errors} error(s), {n_warnings} warning(s)")
    return 1 if n_errors else 0


def run_convert(args: argparse.Namespace) -> int:
    """Convert between flight-log formats: legacy `.jsonl` -> bundle, or
    bundle -> a derived aligned/JSONL CSV (`--aligned`/`--jsonl`).
    フライトログ形式間を変換する: レガシー `.jsonl` -> 一式、または
    一式 -> 派生の整列/JSONL CSV（`--aligned`/`--jsonl`）。
    """
    input_path = Path(args.input)
    if not input_path.exists():
        console.error(f"Input file not found: {input_path}")
        return 1

    if args.aligned or args.jsonl:
        if not sflog.is_bundle(input_path):
            console.error(
                f"--aligned/--jsonl require a flight-log bundle input, not: {input_path}"
            )
            return 1
        return _convert_bundle(input_path, args)

    if input_path.suffix.lower() == ".jsonl":
        return _convert_jsonl_to_bundle(input_path, args)

    if sflog.is_bundle(input_path):
        console.error(
            "A bundle input needs --aligned or --jsonl to say what derived "
            "file to build from it -- the bundle itself is already the "
            "primary record."
        )
        return 1

    console.error(
        f"Unrecognized input: {input_path}. Expected a legacy .jsonl log "
        "or a flight-log bundle (.sflog.zip or a directory bundle)."
    )
    return 1


def _convert_jsonl_to_bundle(input_path: Path, args: argparse.Namespace) -> int:
    """convert mode (a): legacy `.jsonl` -> bundle.
    変換モード(a): レガシー `.jsonl` -> 一式。
    """
    output = Path(args.output) if args.output else input_path.with_suffix(".sflog.zip")
    console.info(f"Converting {input_path.name} -> bundle...")
    try:
        sflog.jsonl_to_bundle(input_path, output)
    except Exception as e:  # noqa: BLE001
        console.error(f"Conversion failed: {e}")
        return 1
    console.success(f"Converted to: {output}")
    return 0


def _convert_bundle(input_path: Path, args: argparse.Namespace) -> int:
    """convert modes (b)/(c): bundle -> aligned CSV (`--aligned`) or bundle
    -> legacy JSONL (`--jsonl`).
    変換モード(b)/(c): 一式 -> 整列CSV（`--aligned`）または
    一式 -> レガシーJSONL（`--jsonl`）。
    """
    try:
        log = sflog.load(input_path)
    except Exception as e:  # noqa: BLE001
        console.error(f"Failed to load bundle: {e}")
        return 1

    stem = _bundle_stem(input_path)

    if args.aligned:
        if args.base not in log.streams:
            console.error(f"Base stream '{args.base}' is not present in this bundle.")
            return 1
        nominal_hz = sflog.schema.STREAMS.get(args.base, {}).get("nominal_rate_hz")
        rate_suffix = str(int(nominal_hz)) if nominal_hz else ""
        default_output = input_path.with_name(f"{stem}_aligned{rate_suffix}.csv")
        output = Path(args.output) if args.output else default_output

        console.info(f"Building aligned table (base={args.base}, method={args.method})...")
        try:
            sflog.aligned_to_csv(
                log, output,
                rate_note=f"{nominal_hz}Hz" if nominal_hz else None,
                base=args.base, method=args.method,
            )
        except Exception as e:  # noqa: BLE001
            console.error(f"Alignment failed: {e}")
            return 1
        console.success(f"Converted to: {output} (derived; see {output.name}.meta.json)")
        return 0

    # args.jsonl (the only other mode _convert_bundle() is called for --
    # run_convert() requires args.aligned or args.jsonl before dispatching here)
    default_output = input_path.with_name(f"{stem}.jsonl")
    output = Path(args.output) if args.output else default_output
    console.info("Converting bundle -> legacy JSONL...")
    try:
        n_lines = sflog.bundle_to_jsonl(log, output)
    except Exception as e:  # noqa: BLE001
        console.error(f"Conversion failed: {e}")
        return 1
    console.success(f"Converted to: {output} ({n_lines} lines)")
    return 0


def run_info(args: argparse.Namespace) -> int:
    """Show a bundle's meta.json summary and per-stream stats.
    一式の meta.json 要約とストリームごとの統計を表示する。
    """
    bundle_path = _resolve_bundle_arg(args.bundle)
    if bundle_path is None:
        return 1

    meta = _read_bundle_meta(bundle_path)
    if meta is None:
        console.error(f"Failed to read meta.json from: {bundle_path}")
        return 1

    tool = meta.get("tool") or {}
    capture = meta.get("capture") or {}

    console.print(f"Bundle: {bundle_path.name}")
    console.print(f"  Size:    {_bundle_size_kb(bundle_path):.1f} KB")
    console.print(f"  Source:  {meta.get('source', '?')}")
    console.print(f"  Created: {_format_created_at(meta.get('created_at'))}")
    console.print(f"  Tool:    {tool.get('name', '?')} {tool.get('version', '')}".rstrip())
    if capture:
        console.print(
            f"  Capture: {capture.get('ip', '?')}:{capture.get('port', '?')} "
            f"(requested {capture.get('requested_duration_s', '?')}s, "
            f"actual {capture.get('actual_duration_s', 0):.1f}s, "
            f"{capture.get('packets_lost', 0)} packets lost)"
        )
    notes = meta.get("notes")
    if notes:
        console.print(f"  Notes:   {notes}")

    streams = meta.get("streams") or {}
    if not streams:
        console.print()
        console.print("  (no streams)")
        return 0

    console.print()
    console.print(
        f"  {'Stream':<14s} {'Rows':>8s} {'NomHz':>7s} {'MeasHz':>7s} "
        f"{'First(us)':>14s} {'Last(us)':>14s} {'Dur(s)':>7s}"
    )
    for name in sorted(streams):
        console.print(_stream_info_row(name, streams[name]))

    return 0


def _stream_info_row(name: str, stats: dict) -> str:
    """Format one `sf log info` stream-table row from meta.json's
    per-stream stats block (bundle.py's `_stream_stats()`).
    meta.json のストリームごとの統計ブロック（bundle.py の
    `_stream_stats()`）から `sf log info` の表の1行を組み立てる。
    """
    rows = stats.get("rows", 0)
    nominal = stats.get("nominal_rate_hz")
    measured = stats.get("measured_rate_hz")
    first_ts = stats.get("first_timestamp_us")
    last_ts = stats.get("last_timestamp_us")
    duration_s = (last_ts - first_ts) / 1e6 if (first_ts is not None and last_ts is not None) else 0.0

    nominal_str = str(nominal) if nominal is not None else "-"
    measured_str = f"{measured:.1f}" if measured is not None else "-"
    first_str = str(first_ts) if first_ts is not None else "-"
    last_str = str(last_ts) if last_ts is not None else "-"

    return (
        f"  {name:<14s} {rows:>8d} {nominal_str:>7s} {measured_str:>7s} "
        f"{first_str:>14s} {last_str:>14s} {duration_s:>7.1f}"
    )



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


# --- Flight-log v1 bundle helpers (list/info/check; not used by the
# untouched viz/analyze paths above, which still resolve via
# _find_latest_log() against legacy .jsonl/.csv files -- Phase 2)
# --- フライトログ v1 一式のヘルパー（list/info/check 用。上の未変更の
# viz/analyze 経路は引き続き _find_latest_log() でレガシー .jsonl/.csv を
# 解決する -- Phase 2）
# =============================================================================


def _iter_bundles(log_dir: Path):
    """Yield (path, meta) for every v1 flight-log bundle directly under
    `log_dir`: `*.sflog.zip` files and directory bundles, identified by
    `sflog.is_bundle()` (checks meta.json's `format` field, not just the
    file name) so a stray `.zip`/directory is never mistaken for one.
    `log_dir` 直下にある v1 フライトログ一式（`*.sflog.zip` ファイルおよび
    ディレクトリ一式）ごとに (path, meta) を返す。判定は `sflog.is_bundle()`
    （ファイル名でなく meta.json の `format` フィールドを見る）なので、
    無関係な `.zip`/ディレクトリを一式と誤認しない。
    """
    if not log_dir.exists():
        return
    for path in sorted(log_dir.iterdir()):
        if not sflog.is_bundle(path):
            continue
        meta = _read_bundle_meta(path)
        if meta is not None:
            yield path, meta


def _read_bundle_meta(path: Path) -> Optional[dict]:
    """Read just a bundle's meta.json (zip or directory), without loading
    every stream CSV via sflog.FlightLog.load() -- much cheaper for a
    listing. Returns None if unreadable.
    一式の meta.json だけを読む（zip・ディレクトリ両対応）。
    sflog.FlightLog.load() のように全ストリーム CSV を読まないため
    一覧表示にはこちらの方がずっと軽い。読めなければ None。
    """
    import json
    import zipfile

    try:
        if path.is_dir():
            meta_path = path / "meta.json"
            if not meta_path.exists():
                return None
            return json.loads(meta_path.read_text(encoding="utf-8"))
        with zipfile.ZipFile(path) as zf:
            if "meta.json" not in zf.namelist():
                return None
            return json.loads(zf.read("meta.json").decode("utf-8"))
    except (OSError, ValueError) as e:
        console.debug(f"Failed to read {path}/meta.json: {e}")
        return None


def _bundle_size_kb(path: Path) -> float:
    """Bundle size in KB: the zip file's own size, or the sum of every
    file inside a directory bundle.
    一式のサイズ[KB]: zip ファイルそのもののサイズ、またはディレクトリ
    一式内の全ファイルサイズの合計。
    """
    if path.is_file():
        return path.stat().st_size / 1024
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file()) / 1024


def _bundle_duration_s(meta: dict) -> float:
    """Best-available capture duration for `sf log list`'s summary row:
    the actual measured capture span if meta.json recorded one, else the
    required `imu` stream's own first/last timestamp span.
    `sf log list` の要約行に出す取得時間の最良推定: meta.json に実測の
    キャプチャ時間が記録されていればそれ、無ければ必須ストリーム `imu`
    自身の最初/最後の時刻の差。
    """
    capture = meta.get("capture") or {}
    if capture.get("actual_duration_s") is not None:
        return float(capture["actual_duration_s"])
    imu = (meta.get("streams") or {}).get("imu") or {}
    first_ts, last_ts = imu.get("first_timestamp_us"), imu.get("last_timestamp_us")
    if first_ts is not None and last_ts is not None:
        return (last_ts - first_ts) / 1e6
    return 0.0


def _format_created_at(created_at: Optional[str]) -> str:
    """meta.json's ISO 8601 `created_at` -> a fixed-width display string.
    meta.json の ISO 8601 形式 `created_at` -> 表示用の固定幅文字列。
    """
    if not created_at:
        return "?"
    try:
        return datetime.fromisoformat(created_at).strftime("%Y-%m-%d %H:%M:%S")
    except ValueError:
        return str(created_at)[:19]


def _bundle_stem(bundle_path: Path) -> str:
    """Base name for a derived file built from `bundle_path` (`sf log
    convert --aligned`/`--jsonl`): "flight_x.sflog.zip" -> "flight_x", so
    the derived name reads "flight_x_aligned400.csv", not
    "flight_x.sflog_aligned400.csv" (Path.stem strips only the LAST
    suffix, leaving ".sflog" behind for a `.sflog.zip` file). A directory
    bundle has no such double suffix, so `.stem` already gives the
    intended name.
    `bundle_path` から作る派生ファイル名の基幹部（`sf log convert
    --aligned`/`--jsonl`）: "flight_x.sflog.zip" -> "flight_x"（派生名を
    "flight_x_aligned400.csv" にするため。Path.stem は最後の拡張子だけを
    外すので、`.sflog.zip` では ".sflog" が残ってしまう）。ディレクトリ
    一式にはこの二重拡張子が無いため、`.stem` がそのまま意図した名前になる。
    """
    stem = bundle_path.stem
    if stem.endswith(".sflog"):
        stem = stem[: -len(".sflog")]
    return stem


def _find_latest_bundle() -> Optional[Path]:
    """Most recently modified flight-log bundle under logs/ -- the
    default `sf log list/info/check` fall back to when no bundle is named
    explicitly. Distinct from _find_latest_log() above, which viz/analyze
    (Phase 2, untouched by this change) still use for legacy .jsonl/.csv.
    logs/ 配下で最も新しく更新されたフライトログ一式 -- `sf log
    list/info/check` がバンドル未指定時に使う既定値。上の
    _find_latest_log()（viz/analyze が引き続きレガシー .jsonl/.csv に
    使う。Phase 2、本変更では未変更）とは別物。
    """
    bundles = list(_iter_bundles(get_log_dir()))
    if not bundles:
        return None
    bundles.sort(key=lambda item: item[0].stat().st_mtime, reverse=True)
    return bundles[0][0]


def _resolve_bundle_arg(bundle_arg: Optional[str]) -> Optional[Path]:
    """Resolve `sf log info/check`'s optional bundle argument: the given
    path if valid, else the newest bundle in logs/. Prints its own error
    (via `console.error`) and returns None on any failure, so callers can
    just `if bundle_path is None: return 1`.
    `sf log info/check` の任意のバンドル引数を解決する: 指定があれば
    そのパス、無ければ logs/ 内の最新の一式。失敗時は自身で
    `console.error` を出し None を返すため、呼び出し側は
    `if bundle_path is None: return 1` するだけでよい。
    """
    if not bundle_arg:
        latest = _find_latest_bundle()
        if not latest:
            console.error("No flight-log bundles found in logs/. Capture one with 'sf log wifi'.")
            return None
        console.info(f"Using latest bundle: {latest}")
        return latest

    path = Path(bundle_arg)
    if not path.exists():
        console.error(f"Bundle not found: {path}")
        return None
    if not sflog.is_bundle(path):
        console.error(f"Not a StampFly flight-log bundle (no valid meta.json): {path}")
        return None
    return path


def _resolve_wifi_output(output_arg: Optional[str], log_dir: Path) -> Path:
    """Resolve `sf log wifi -o`'s output path (see wifi_parser's --help
    and this module's docstring for the 3 cases):
      (a) not given -> `logs/flight_<timestamp>.sflog.zip`.
      (b) ends in `.zip` (covers the `.sflog.zip` convention -- Path.suffix
          reads only the last extension, same rule sflog.FlightLog.save()
          itself dispatches on) -> that exact path, a zip bundle.
      (c) an existing directory, or ends in a path separator -> a
          same-named (`flight_<timestamp>`, no `.sflog.zip` suffix -- that
          suffix would force zip mode) DIRECTORY bundle written INSIDE it.
      (d) any other extension (.csv/.jsonl/.bin) -> rejected: the bundle
          is the only capture format now (derive a CSV/JSONL afterwards
          with `sf log convert --aligned`/`--jsonl`).
      (e) no extension and not an existing directory -> `path` itself
          becomes the bundle's own directory (sflog.FlightLog.save()'s
          plain "not a .zip" branch populates the given path directly).

    Raises:
        ValueError: case (d), with a message meant to be shown as-is via
            `console.error()`.

    `sf log wifi -o` の出力パスを解決する（3ケースの詳細は英語側 /
    wifi_parser の --help 参照）。case (d) は ValueError を送出し、
    メッセージはそのまま `console.error()` で表示する想定。
    """
    default_stem = f"flight_{datetime.now().strftime('%Y%m%dT%H%M%S')}"

    if not output_arg:
        return log_dir / f"{default_stem}.sflog.zip"

    path = Path(output_arg)
    if path.suffix.lower() == ".zip":
        return path

    looks_like_dir = str(output_arg).endswith(("/", "\\")) or path.is_dir()
    if looks_like_dir:
        return path / default_stem

    if path.suffix.lower() in (".csv", ".jsonl", ".bin"):
        raise ValueError(
            f"'{output_arg}': the flight-log bundle (.sflog.zip) is the only "
            "capture format now -- pass a .sflog.zip path (or a directory), "
            "then use `sf log convert --aligned` or `--jsonl` for a derived "
            f"{path.suffix} file."
        )

    return path
