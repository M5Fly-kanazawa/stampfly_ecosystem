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
    analyze  - Analyze a bundle (gyro PSD, hover stats; --health: motor-fault report)
    viz      - Visualize a bundle (every stream at its native rate; -i: Plotly)

Every reader accepts a `.sflog.zip` or an extracted bundle directory and
defaults to the newest bundle in logs/ (`paths.latest_bundle()`).
読み込み側は全て `.sflog.zip` または展開済みフォルダを受け付け、未指定なら
logs/ 内の最新の一式（`paths.latest_bundle()`）を使う。
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
             "if not specified). A bare name gets .sflog.zip appended "
             "(`-o flight1` -> flight1.sflog.zip); a path ending in "
             ".sflog.zip writes that zip; an existing directory (or a path "
             "ending in a path separator) gets the default-named .sflog.zip "
             "written inside it. Other extensions (.csv/.jsonl/.bin) "
             "are rejected -- the bundle is the only capture format; use "
             "`sf log convert --aligned` or `--jsonl` afterwards for a "
             "derived file.",
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
        help="Bundle path, .sflog.zip or directory (default: newest in logs/; "
             "extension may be omitted; also searched in logs/)",
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
        help="Input file: a legacy .jsonl log, or a bundle (.sflog.zip/directory; "
             "extension may be omitted; also searched in logs/)",
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
        help="Bundle path, .sflog.zip or directory (default: newest in logs/; "
             "extension may be omitted; also searched in logs/)",
    )
    info_parser.set_defaults(func=run_info)

    # --- analyze ---
    analyze_parser = log_subparsers.add_parser(
        "analyze",
        help="Analyze a flight-log bundle",
        description="Analyze a flight-log bundle for stability, oscillation "
                     "(gyro PSD from imu.csv at its true rate), hover statistics "
                     "and tuning insights; --health gives the motor-health report.",
    )
    analyze_parser.add_argument(
        "bundle",
        nargs="?",
        help="Bundle path, .sflog.zip or directory (default: newest in logs/; "
             "extension may be omitted; also searched in logs/). "
             "With --health --batch: a glob such as 'logs/flight_202609*.sflog.zip'",
    )
    analyze_parser.add_argument(
        "--save",
        metavar="FILE",
        help="PNG path for the analysis figure "
             "(default: <bundle stem>_analysis.png next to the bundle)",
    )
    analyze_parser.add_argument(
        "--health",
        action="store_true",
        help="Motor health report: detect a degraded rotor from the hover trim "
             "(duties from motor.csv, else ctrl_ref.csv; gyro_z from imu.csv; "
             "voltage from status.csv)",
    )
    analyze_parser.add_argument(
        "--batch",
        action="store_true",
        help="With --health: analyze several bundles (the newest in logs/, or "
             "the glob given as `bundle`) for the CG-removed corner test",
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
        help="Visualize a flight-log bundle",
        description="Plot a flight-log bundle: every stream at its own native "
                     "rate (gyro + rate_ref, accel, attitude, position/velocity, "
                     "motor duty, control output, pilot sticks, baro/ToF/flow/mag, "
                     "battery). -i opens an interactive Plotly dashboard instead.",
    )
    viz_parser.add_argument(
        "bundle",
        nargs="?",
        help="Bundle path, .sflog.zip or directory (default: newest in logs/; "
             "extension may be omitted; also searched in logs/)",
    )
    viz_parser.add_argument(
        "--mode",
        choices=["all", "attitude", "sensors", "position", "eskf"],
        default="all",
        help="Panel group (default: all)",
    )
    viz_parser.add_argument(
        "--cols",
        type=int,
        default=3,
        choices=[1, 2, 3, 4],
        help="Panel columns of the overview grid (default: 3)",
    )
    viz_parser.add_argument(
        "--save",
        metavar="FILE",
        help="Save the figure (PNG) -- or the dashboard HTML with -i -- "
             "instead of opening a window/browser",
    )
    viz_parser.add_argument(
        "--time-range",
        nargs=2,
        type=float,
        metavar=("START", "END"),
        help="Time range to plot (seconds from the first sample)",
    )
    viz_parser.add_argument(
        "-i", "--interactive",
        action="store_true",
        help="Interactive mode (Plotly dashboard, opens in the browser)",
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
    console.print("  analyze   Analyze a bundle (--health: motor-health report)")
    console.print("  viz       Visualize a bundle (-i: interactive Plotly)")
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

    The legacy-JSONL check comes FIRST, before bundle-name resolution --
    a `.jsonl` name is never extension-omitted (the bundle resolver would
    never guess to append `.jsonl`), so it must be recognized on its own.
    Anything else is resolved as a bundle argument (extension may be
    omitted; also searched in logs/, see `sflog.resolve_bundle_path()`).
    フライトログ形式間を変換する: レガシー `.jsonl` -> 一式、または
    一式 -> 派生の整列/JSONL CSV（`--aligned`/`--jsonl`）。

    レガシーJSONLの判定を、バンドル名解決より先に行う -- `.jsonl` という
    名前は拡張子省略ではあり得ない（バンドル解決側が `.jsonl` を補うことは
    ない）ため、単独で認識する必要がある。それ以外はバンドル引数として
    解決する（拡張子省略可、logs/ も検索対象 --
    `sflog.resolve_bundle_path()` 参照）。
    """
    if args.input.endswith(".jsonl"):
        input_path = Path(args.input)
        if not input_path.exists():
            console.error(f"Input file not found: {input_path}")
            return 1
        return _convert_jsonl_to_bundle(input_path, args)

    try:
        input_path = sflog.resolve_bundle_path(
            args.input, search_dirs=(get_log_dir(),), notify=console.info
        )
    except FileNotFoundError as e:
        console.error(str(e))
        return 1

    if args.aligned or args.jsonl:
        return _convert_bundle(input_path, args)

    console.error(
        "A bundle input needs --aligned or --jsonl to say what derived "
        "file to build from it -- the bundle itself is already the "
        "primary record."
    )
    return 1


def _convert_jsonl_to_bundle(input_path: Path, args: argparse.Namespace) -> int:
    """convert mode (a): legacy `.jsonl` -> bundle.
    変換モード(a): レガシー `.jsonl` -> 一式。
    """
    try:
        output = _bundle_output_path(args.output, input_path.with_suffix(".sflog.zip"))
    except ValueError as e:
        console.error(str(e))
        return 1
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
    """Analyze a flight-log bundle (tools/log_analyzer/flight_analysis.py):
    gyro statistics and PSD from imu.csv at its true sample rate, attitude/
    position/pilot statistics, hover-window statistics, a 5 s segment table
    and tuning observations, plus a PNG figure. `--health` switches to the
    motor-health report instead.
    フライトログ一式を解析する（tools/log_analyzer/flight_analysis.py）:
    imu.csv の真の標本化レートでのジャイロ統計と PSD、姿勢・位置・操縦入力の
    統計、ホバー区間の統計、5 秒区切りの表、チューニング所見、PNG 図。
    `--health` はモータ健全性レポートに切り替える。
    """
    if getattr(args, "health", False):
        return _run_motor_health(args)

    bundle_path = _resolve_bundle_arg(args.bundle)
    if bundle_path is None:
        return 1

    png_path = Path(args.save) if args.save else _derived_path(bundle_path, "_analysis.png")

    console.info(f"Analyzing: {bundle_path.name}")
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))
        import flight_analysis
    except ImportError as e:
        console.error(f"Failed to import analysis module: {e}")
        console.print("  Required: pandas, matplotlib, scipy")
        return 1
    finally:
        sys.path.pop(0)

    try:
        log = sflog.load(bundle_path)
        flight_analysis.analyze_flight(log, bundle_path.name, png_path=str(png_path))
        return 0
    except Exception as e:  # noqa: BLE001 - report, never crash sf
        console.error(f"Analysis failed: {e}")
        return 1


def _run_motor_health(args: argparse.Namespace) -> int:
    """Run the motor health report (sf log analyze --health) over one or
    several bundles (tools/log_analyzer/motor_health.py).
    モータ健全性レポート（sf log analyze --health）を 1 つまたは複数の
    一式に対して実行する（tools/log_analyzer/motor_health.py）。

    Detects a degraded rotor from the steady hover trim. One bundle
    identifies the spin-direction group; --batch adds the CG-removed
    cross-log corner test.
    1 本の一式で回転グループを判定し、--batch で CG 除去のクロスログ隅特定を
    加える。"""
    if args.batch:
        bundle_paths = _batch_bundle_paths(args.bundle)
        if not bundle_paths:
            return 1
    else:
        bundle_path = _resolve_bundle_arg(args.bundle)
        if bundle_path is None:
            return 1
        bundle_paths = [bundle_path]
        console.info(f"Health report: {bundle_path.name}")

    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))
        import motor_health
        sys.path.pop(0)
        result = motor_health.analyze_health([str(p) for p in bundle_paths], json_out=args.json)
        return 0 if "error" not in result else 1
    except Exception as e:  # noqa: BLE001
        console.error(f"Health report failed: {e}")
        return 1


# The cross-log corner test assumes ONE airframe (constant CG). The default
# batch is capped to the most recent bundles so an old/other airframe is not
# mixed in; pass a glob to scope a specific session/airframe explicitly.
# クロスログ隅特定は同一機体（CG 一定）が前提。既定のバッチは最新の一式に
# 限定して別機体の混入を避ける。特定セッションはグロブで明示する。
HEALTH_BATCH_MAX_BUNDLES = 12


def _batch_bundle_paths(glob_arg: Optional[str]) -> List[Path]:
    """Bundles for `--health --batch`: those matching `glob_arg` (tried
    relative to logs/ first, then as given), or the newest
    HEALTH_BATCH_MAX_BUNDLES `*.sflog.zip` in logs/. Prints its own error
    and returns [] when nothing matches.
    `--health --batch` の対象一式: `glob_arg` に一致するもの（まず logs/
    相対、次に指定そのまま）、または logs/ 内の最新
    HEALTH_BATCH_MAX_BUNDLES 本の `*.sflog.zip`。該当なしなら自身でエラーを
    出し [] を返す。
    """
    from glob import glob as _glob

    log_dir = get_log_dir()
    if glob_arg:
        matched = sorted(log_dir.glob(Path(glob_arg).name))
        if not matched:
            matched = sorted(Path(p) for p in _glob(glob_arg))
        if not matched:
            console.error(f"No flight-log bundles match: {glob_arg}")
            return []
        console.info(f"Health report over {len(matched)} bundles (glob)")
        return matched

    all_bundles = sorted((path for path, _meta in _iter_bundles(log_dir)),
                         key=lambda f: f.stat().st_mtime, reverse=True)
    if not all_bundles:
        console.error("No flight-log bundles found in logs/ for --batch.")
        return []
    recent = sorted(all_bundles[:HEALTH_BATCH_MAX_BUNDLES])
    console.info(f"Health report over the {len(recent)} most recent bundles "
                 "(assumes one airframe; pass a glob to scope)")
    return recent


def run_viz(args: argparse.Namespace) -> int:
    """Visualize a flight-log bundle: resolve the bundle, then dispatch to
    the interactive (Plotly) dashboard or the matplotlib renderer
    (tools/log_analyzer/visualize_stream.py) with the headless PNG fallback.
    フライトログ一式を可視化する: 一式を解決し、インタラクティブ（Plotly）
    ダッシュボードか、ヘッドレス PNG フォールバック付きの matplotlib 描画
    （tools/log_analyzer/visualize_stream.py）へ振り分ける。"""
    bundle_path = _resolve_bundle_arg(args.bundle)
    if bundle_path is None:
        return 1

    console.info(f"Visualizing: {bundle_path.name}")

    if getattr(args, "interactive", False):
        return _viz_interactive(bundle_path, args)
    return _viz_bundle(bundle_path, args)


def _viz_interactive(bundle_path: Path, args: argparse.Namespace) -> int:
    """Interactive Plotly dashboard (tools/log_analyzer/visualize_interactive.py).
    Plotly opens in a browser tab, not a matplotlib window, so there is no
    headless case to handle here; `--save` writes the HTML instead.
    インタラクティブな Plotly ダッシュボード。ブラウザタブで開くため
    matplotlib のヘッドレス対応は不要。`--save` は HTML を書き出す。
    """
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))
        import visualize_interactive

        visualize_interactive.visualize(str(bundle_path), output=args.save)
        return 0
    except ImportError as e:
        console.error(f"Failed to import interactive visualizer: {e}")
        return 1
    except Exception as e:  # noqa: BLE001
        console.error(f"Interactive visualization failed: {e}")
        return 1
    finally:
        sys.path.pop(0)


def _viz_bundle(bundle_path: Path, args: argparse.Namespace) -> int:
    """matplotlib rendering of a bundle (falls back to a saved PNG when no
    GUI backend is usable).
    一式の matplotlib 描画（GUI バックエンドが使えなければ保存した PNG へ
    フォールバックする）。"""
    try:
        sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))

        # visualize_stream imports matplotlib.pyplot at module load time,
        # which locks in whatever backend is active at that moment -- so the
        # backend is chosen here, before that import, and handed to
        # _render_with_fallback() so the (Tk window-creating) probe runs once.
        # visualize_stream はモジュール読み込み時に matplotlib.pyplot を
        # import し、その時点で有効なバックエンドを固定してしまう。そのため
        # import より前にここでバックエンドを選び、_render_with_fallback() へ
        # 渡して（Tk ウィンドウを作る）プローブが一度しか走らないようにする。
        backend = plotting.select_backend(want_window=args.save is None)

        import visualize_stream
        log = visualize_stream.load_bundle(bundle_path)

        def render(save_path: Optional[str], show: bool) -> None:
            visualize_stream.render(
                log, bundle_path.name, save_path=save_path, show=show,
                time_range=tuple(args.time_range) if args.time_range else None,
                mode=args.mode,
                cols=getattr(args, "cols", visualize_stream.DEFAULT_COLUMNS),
            )

        return _render_with_fallback(bundle_path, args, render, backend)

    except ImportError as e:
        console.error(f"Failed to import visualization module: {e}")
        console.print("  Required: matplotlib, numpy, pandas")
        return 1
    except Exception as e:  # noqa: BLE001
        # Covers bundle-loading failures -- errors from render() itself are
        # handled inside _render_with_fallback() and never reach this far.
        # 一式の読み込み失敗を捕捉する -- render() 自体のエラーは
        # _render_with_fallback() 内で処理済みで、ここまでは届かない。
        console.error(f"Visualization failed: {e}")
        return 1
    finally:
        sys.path.pop(0)


def _render_with_fallback(
    bundle_path: Path,
    args: argparse.Namespace,
    render: Callable[[Optional[str], bool], None],
    backend: Optional[plotting.BackendInfo] = None,
) -> int:
    """Draw one figure set via `render(save_path, show)`, choosing the
    matplotlib backend first. Falls back to a PNG saved next to the bundle
    (opened with the OS default viewer) when no window can be shown, and
    retries headlessly once if a GUI backend passes its import-time probe
    but still fails while actually drawing/showing.
    `render(save_path, show)` で1つの図を描く。まず matplotlib バックエンドを
    選ぶ。ウィンドウを表示できない場合は一式の隣に PNG を保存して
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
    fallback_png = _derived_path(bundle_path, ".png")

    save_path, show = args.save, want_window
    opened_fallback = False
    if want_window and info.interactive:
        # One line so a user can see which GUI backend the window uses
        # (macosx / tkagg / qtagg) without any extra flag.
        # 追加のフラグ無しで、ウィンドウがどの GUI バックエンド
        # （macosx / tkagg / qtagg）で開くかを 1 行で示す。
        console.info(f"Plot window backend: {info.name}")
    if want_window and not info.interactive:
        save_path = str(fallback_png)
        show = False
        opened_fallback = True
        plotting.report_headless(console, info, fallback_png)

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
        save_path = str(fallback_png)
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


# =============================================================================
# --- Flight-log v1 bundle helpers (shared by list/info/check/analyze/viz)
# --- フライトログ v1 一式のヘルパー（list/info/check/analyze/viz で共有）
# =============================================================================


def _derived_path(bundle_path: Path, suffix: str) -> Path:
    """Path of a file derived from `bundle_path`, next to it:
    "logs/flight_x.sflog.zip" + "_analysis.png" -> "logs/flight_x_analysis.png"
    (see _bundle_stem() for why Path.stem alone is not enough).
    `bundle_path` から派生するファイルのパス（一式の隣）:
    "logs/flight_x.sflog.zip" + "_analysis.png" -> "logs/flight_x_analysis.png"
    （Path.stem だけでは足りない理由は _bundle_stem() 参照）。
    """
    return bundle_path.parent / f"{_bundle_stem(bundle_path)}{suffix}"


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
    """Most recently modified flight-log bundle under logs/ -- the default
    every `sf log` subcommand falls back to when no bundle is named.
    `paths.latest_bundle()` is the project-wide "newest log" lookup shared
    with `sf trim`/`sf cal`/`sf sysid`; it detects bundles by content (zip
    or directory with a v1 meta.json), whatever the file name.
    logs/ 配下で最も新しく更新されたフライトログ一式 -- 一式を指定しない
    `sf log` 各サブコマンドの既定値。`paths.latest_bundle()`（`sf trim`/
    `sf cal`/`sf sysid` と共有するプロジェクト共通の「最新ログ」探索）は
    ファイル名ではなく中身（v1 の meta.json を持つ zip かフォルダ）で判定する。
    """
    return paths.latest_bundle()


def _resolve_bundle_arg(bundle_arg: Optional[str]) -> Optional[Path]:
    """Resolve a subcommand's optional bundle argument (`sf log info/check/
    analyze/viz`): the given path if valid, else the newest bundle in
    logs/. The extension may be omitted, and a bare name is also looked up
    in logs/ (`sflog.resolve_bundle_path()`), so `sf log viz
    flight_20260912T093015` works from any directory. Prints its own error
    (via `console.error`) and returns None on any failure, so callers can
    just `if bundle_path is None: return 1`.
    サブコマンド（`sf log info/check/analyze/viz`）の任意のバンドル引数を
    解決する: 指定があればそのパス、無ければ logs/ 内の最新の一式。拡張子は
    省略でき、裸の名前は logs/ 内も探す（`sflog.resolve_bundle_path()`）ため、
    `sf log viz flight_20260912T093015` はどのディレクトリからでも動く。
    失敗時は自身で `console.error` を出し None を返すため、呼び出し側は
    `if bundle_path is None: return 1` するだけでよい。
    """
    if not bundle_arg:
        latest = _find_latest_bundle()
        if not latest:
            console.error("No flight-log bundles found in logs/. Capture one with 'sf log wifi'.")
            return None
        console.info(f"Using latest bundle: {latest}")
        return latest

    try:
        return sflog.resolve_bundle_path(
            bundle_arg, search_dirs=(get_log_dir(),), notify=console.info
        )
    except FileNotFoundError as e:
        console.error(str(e))
        return None


def _bundle_output_path(output_arg: Optional[str], default: Path) -> Path:
    """Resolve a user-given output path for a flight-log bundle:
      (a) not given -> `default`;
      (b) ends in `.zip` (the `.sflog.zip` convention) -> that path;
      (c) an existing directory, or a path ending in a separator -> the
          default-named `.sflog.zip` written INSIDE that directory;
      (d) `.csv`/`.jsonl`/`.bin` -> ValueError (derived files come from
          `sf log convert --aligned`/`--jsonl`, never from a capture);
      (e) a bare name (or any other extension) -> `.sflog.zip` appended,
          so `-o flight1` writes `flight1.sflog.zip`.
    利用者が与えた一式の出力パスを解決する: 未指定なら `default`、`.zip`
    ならそのまま、既存フォルダ（またはパス区切りで終わる）ならその中に既定名の
    `.sflog.zip`、`.csv`/`.jsonl`/`.bin` は拒否、拡張子なし（または他の拡張子）
    なら `.sflog.zip` を補う（`-o flight1` -> `flight1.sflog.zip`）。
    """
    if not output_arg:
        return default

    path = Path(output_arg)
    if path.suffix.lower() == ".zip":
        return path

    looks_like_dir = str(output_arg).endswith(("/", "\\")) or path.is_dir()
    if looks_like_dir:
        return path / default.name

    if path.suffix.lower() in (".csv", ".jsonl", ".bin"):
        raise ValueError(
            f"'{output_arg}': the flight-log bundle (.sflog.zip) is the only "
            "capture format -- pass a .sflog.zip path, a bare name, or a "
            "directory, then use `sf log convert --aligned` or `--jsonl` for "
            f"a derived {path.suffix} file."
        )

    return path.with_name(path.name + ".sflog.zip")


def _resolve_wifi_output(output_arg: Optional[str], log_dir: Path) -> Path:
    """`sf log wifi -o` output path: `logs/flight_<timestamp>.sflog.zip`
    by default, otherwise the rules of _bundle_output_path().
    `sf log wifi -o` の出力パス: 既定は `logs/flight_<日時>.sflog.zip`、
    指定時は _bundle_output_path() の規則に従う。
    """
    default_name = f"flight_{datetime.now().strftime('%Y%m%dT%H%M%S')}.sflog.zip"
    return _bundle_output_path(output_arg, log_dir / default_name)
