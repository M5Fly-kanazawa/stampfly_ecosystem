"""
sf sysid - System identification commands
sf sysid - システム同定コマンド

Unix philosophy: Small tools that do one thing well and work together via pipes.

Subcommands:
    noise      - Sensor noise characterization (Allan variance)
    inertia    - Moment of inertia estimation (step response)
    motor      - Motor dynamics identification (Ct, Cq, τm)
    drag       - Aerodynamic drag coefficient estimation
    params     - Parameter management (show, diff, export)
    validate   - Validation and consistency checks
    fit        - Rate-loop transfer function fit from flight log
    plan       - Flight test plan generation
    rate-fit   - Rate controller closed-loop identification (ETFE + fit)
    rate-tune  - PID gain design from an identified rate-loop model
    rate-excite - Rate-loop excitation signal generation (chirp/doublet)
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

import yaml

from ..utils import console, paths, plotting

COMMAND_NAME = "sysid"
COMMAND_HELP = "System identification from flight logs"


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Create sub-subparsers for sysid subcommands
    sysid_subparsers = parser.add_subparsers(
        dest="sysid_command",
        title="subcommands",
        metavar="<subcommand>",
    )

    # --- noise ---
    _register_noise(sysid_subparsers)

    # --- inertia ---
    _register_inertia(sysid_subparsers)

    # --- motor ---
    _register_motor(sysid_subparsers)

    # --- drag ---
    _register_drag(sysid_subparsers)

    # --- params ---
    _register_params(sysid_subparsers)

    # --- validate ---
    _register_validate(sysid_subparsers)

    # --- fit ---
    _register_fit(sysid_subparsers)

    # --- plan ---
    _register_plan(sysid_subparsers)

    # --- rate-loop identification + auto-tuning (rate_sysid.py backend) ---
    _register_rate_fit(sysid_subparsers)
    _register_rate_tune(sysid_subparsers)
    _register_rate_excite(sysid_subparsers)

    parser.set_defaults(func=run_help)


def _register_noise(subparsers):
    """Register noise subcommand"""
    parser = subparsers.add_parser(
        "noise",
        help="Sensor noise characterization (Allan variance)",
        description="Estimate sensor noise parameters using Allan variance analysis.",
    )
    parser.add_argument(
        "input",
        help="Input flight-log bundle (.sflog.zip or extracted directory, "
             "static sensor data)",
    )
    parser.add_argument(
        "-o", "--output",
        help="Output file (YAML/JSON)",
    )
    parser.add_argument(
        "--sensor",
        choices=["gyro", "accel", "baro", "tof", "all"],
        default="all",
        help="Sensor to analyze (default: all)",
    )
    parser.add_argument(
        "--min-duration",
        type=float,
        default=10.0,
        help="Minimum data duration in seconds (default: 10)",
    )
    parser.add_argument(
        "--static-only",
        action="store_true",
        help="Only use static (stationary) segments",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show Allan deviation plots",
    )
    parser.add_argument(
        "--plot-output",
        help="Save plot to file (PNG/PDF)",
    )
    parser.set_defaults(func=run_noise)


def _register_inertia(subparsers):
    """Register inertia subcommand"""
    parser = subparsers.add_parser(
        "inertia",
        help="Moment of inertia estimation (step response)",
        description="Estimate Ixx, Iyy, Izz from angular rate step responses.",
    )
    parser.add_argument(
        "input",
        help="Input flight-log bundle (.sflog.zip or extracted directory, "
             "step response data)",
    )
    parser.add_argument(
        "-o", "--output",
        help="Output file (YAML/JSON)",
    )
    parser.add_argument(
        "--axis",
        choices=["roll", "pitch", "yaw", "all"],
        default="all",
        help="Axis to analyze (default: all)",
    )
    parser.add_argument(
        "--time-range",
        nargs=2,
        type=float,
        metavar=("START", "END"),
        help="Time range to analyze [seconds]",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show identification plots",
    )
    parser.set_defaults(func=run_inertia)


def _register_motor(subparsers):
    """Register motor subcommand"""
    parser = subparsers.add_parser(
        "motor",
        help="Motor dynamics identification",
        description="Identify thrust coefficient (Ct), torque coefficient (Cq), and time constant (τm).",
    )
    parser.add_argument(
        "input",
        help="Input flight-log bundle (.sflog.zip or extracted directory)",
    )
    parser.add_argument(
        "-o", "--output",
        help="Output file (YAML/JSON)",
    )
    parser.add_argument(
        "--param",
        choices=["Ct", "Cq", "tau", "all"],
        default="all",
        help="Parameter to identify (default: all)",
    )
    parser.add_argument(
        "--mass",
        type=float,
        default=0.037,
        help="Vehicle mass in kg (default: 0.037)",
    )
    parser.add_argument(
        "--hover-only",
        action="store_true",
        help="Only use hover segments for Ct estimation",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show identification plots",
    )
    parser.set_defaults(func=run_motor)


def _register_drag(subparsers):
    """Register drag subcommand"""
    parser = subparsers.add_parser(
        "drag",
        help="Aerodynamic drag coefficient estimation",
        description="Estimate drag coefficients from coastdown/decay data.",
    )
    parser.add_argument(
        "input",
        help="Input flight-log bundle (.sflog.zip or extracted directory, "
             "coastdown data)",
    )
    parser.add_argument(
        "-o", "--output",
        help="Output file (YAML/JSON)",
    )
    parser.add_argument(
        "--type",
        choices=["trans", "rot", "all"],
        default="all",
        help="Drag type: trans (translational), rot (rotational), all",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show decay plots",
    )
    parser.set_defaults(func=run_drag)


def _register_params(subparsers):
    """Register params subcommand"""
    parser = subparsers.add_parser(
        "params",
        help="Parameter management",
        description="Show, compare, and export parameters.",
    )
    params_subparsers = parser.add_subparsers(
        dest="params_command",
        title="params subcommands",
        metavar="<action>",
    )

    # params show
    show_parser = params_subparsers.add_parser(
        "show",
        help="Show default or loaded parameters",
    )
    show_parser.add_argument(
        "file",
        nargs="?",
        help="Parameter file to show (default: show defaults)",
    )
    show_parser.add_argument(
        "--format",
        choices=["yaml", "json"],
        default="yaml",
        help="Output format (default: yaml)",
    )
    show_parser.set_defaults(func=run_params_show)

    # params diff
    diff_parser = params_subparsers.add_parser(
        "diff",
        help="Compare two parameter files",
    )
    diff_parser.add_argument(
        "file1",
        help="First parameter file",
    )
    diff_parser.add_argument(
        "file2",
        help="Second parameter file",
    )
    diff_parser.set_defaults(func=run_params_diff)

    # params export
    export_parser = params_subparsers.add_parser(
        "export",
        help="Export parameters to C header",
    )
    export_parser.add_argument(
        "file",
        help="Parameter file to export",
    )
    export_parser.add_argument(
        "-o", "--output",
        required=True,
        help="Output .h file",
    )
    export_parser.set_defaults(func=run_params_export)

    parser.set_defaults(func=run_params_help)


def _register_validate(subparsers):
    """Register validate subcommand"""
    parser = subparsers.add_parser(
        "validate",
        help="Validate identified parameters",
        description="Check physical consistency of identified parameters.",
    )
    parser.add_argument(
        "file",
        help="Parameter file to validate",
    )
    parser.add_argument(
        "--ref",
        help="Reference parameter file for comparison",
    )
    parser.set_defaults(func=run_validate)


def _register_fit(subparsers):
    """Register fit subcommand"""
    parser = subparsers.add_parser(
        "fit",
        help="Fit plant model to flight data",
        description=(
            "Identify open-loop plant parameters G_p(s) = K/(s*(tau_m*s+1)) "
            "from closed-loop P-control flight data. Default (--input auto): "
            "fits the CLOSED-LOOP target->gyro transfer function directly "
            "('indirect' -- far more robust on real, modestly-excited "
            "human-piloted flight than any direct u->y fit, 2026-09-10) "
            "using a Kp that needs NO manual --kp for a typical log -- it is "
            "auto-estimated from the log's own duty via a short FIR "
            "regression of the reconstructed control effort against lags of "
            "target-gyro (no PID-structure assumption). Only when neither "
            "--kp nor a usable duty/control_output signal is available does "
            "'auto' fall back to a direct fit ('control_output' reads the "
            "PRE-MIXER commanded thrust+torque, 400Hz kPktCtrlOutput400/0x4B, "
            "no --mixer needed; 'duty' inverts the actual motor duty, 400Hz "
            "kPktDuty400, via --mixer below) or to the OLD 'kp' "
            "reconstruction (--kp required). When both control_output and "
            "genuine 400Hz duty are present, a mixer-gain diagnostic is also "
            "reported (how far the mixer that actually flew is from the "
            "physical model -- see the rate-sysid design memo, "
            "docs/events/sci_tutorial_2026, 2026-09-09). The input is a "
            "StampFly flight-log v1 bundle (`.sflog.zip` file or extracted "
            "directory, produced by `sf log wifi`; see lib/sflog) -- its "
            "rate_ref stream (rate_ref_roll/pitch/yaw, already rad/s -- "
            "--rate-max is ignored) and imu stream (gyro_x/y/z) supply the "
            "target/gyro pair, aligned at the 400Hz control-cycle rate. "
            "--mixer (default legacy) picks WHICH duty inversion the "
            "FIR Kp auto-estimate and 'duty' fallback use, and the bundle alone "
            "cannot say which is right -- you "
            "must know which firmware produced the log: 'legacy' inverts "
            "the simple linear X-quad mixer (ws_internal.hpp / vehicle_old) "
            "-- correct for `sf lesson` (firmware/workshop) and "
            "firmware/vehicle_old logs; 'vehicle' inverts firmware/vehicle's "
            "actual mixer (physical B^-1 allocation through a nonlinear "
            "motor curve, sf_actuator/actuator.cpp) -- REQUIRED for `sf "
            "app` (firmware/vehicle-based custom controller) logs, or the "
            "fit silently reconstructs the wrong signal (looks like a fit "
            "failure or gives a physically-impossible K/tau_m). "
            "Run --selftest to verify the whole pipeline (all input/mixer "
            "combinations) against a synthetic known plant."
        ),
        epilog=(
            "Examples:\n"
            "  sf sysid fit flight.sflog.zip --plot\n"
            "      Fully automatic: no --kp, no --mixer override needed for a\n"
            "      `sf lesson` (firmware/workshop) log. Auto-estimates the\n"
            "      effective Kp from flight.sflog.zip's own duty via a short FIR\n"
            "      regression, then runs the robust indirect closed-loop fit.\n"
            "      完全自動: `sf lesson`（firmware/workshop）ログなら --kp も\n"
            "      --mixer 指定も不要。flight.sflog.zip 自身の duty から短いFIR回帰で\n"
            "      実効Kpを自動推定し、頑健な間接閉ループフィットを実行する。\n"
            "\n"
            "  sf sysid fit flight.sflog.zip --mixer vehicle --plot\n"
            "      For a log from `sf app` (a firmware/vehicle-based custom\n"
            "      controller): inverts firmware/vehicle's ACTUAL mixer\n"
            "      (physical B^-1 allocation + nonlinear motor curve) instead\n"
            "      of the legacy linear one. --mixer legacy on this data\n"
            "      silently fits the WRONG signal.\n"
            "      `sf app`（firmware/vehicle ベースの自作コントローラ）の\n"
            "      ログ向け: legacy の線形ミキサーではなく firmware/vehicle の\n"
            "      実際のミキサー（物理的なB^-1配分＋非線形モータ曲線）を\n"
            "      逆算する。このデータに --mixer legacy を使うと誤った信号を\n"
            "      静かにフィットしてしまう。\n"
            "\n"
            "  sf sysid fit flight.sflog.zip --kp 0.5 --plot\n"
            "      Fallback for OLD logs without the 400Hz duty columns:\n"
            "      --kp is the roll/pitch rate P gain written in user_code.cpp\n"
            "      for the tutorial (実習 7). Must match what actually flew.\n"
            "      400Hzduty列の無い旧ログ向けフォールバック: --kp には実習7の\n"
            "      user_code.cppに書いたロール/ピッチのレートP制御ゲインを指定\n"
            "      する（実際に飛行させた値と一致させること）。\n"
            "\n"
            "  sf sysid fit flight.sflog.zip --axis roll -o fit.yaml\n"
            "      Identify roll only and save the result to a YAML file.\n"
            "      roll軸のみ同定し、結果をYAMLファイルに保存する。\n"
            "\n"
            "  sf sysid fit --selftest\n"
            "      Verify the whole pipeline (duty AND kp input modes, both\n"
            "      --mixer legacy AND --mixer vehicle) against a synthetic\n"
            "      known plant.\n"
            "      既知の合成プラントに対してパイプライン全体（duty/kp両方の\n"
            "      入力モード、--mixer legacy/vehicle 両方）を自己検証する。\n"
            "\n"
            "Output:\n"
            "  Prints K and tau_m [s] per axis, each compared against a\n"
            "  reference: K vs. REFERENCE_PLANT_GAINS ('legacy' mixer,\n"
            "  [rad/s^2/duty]) or REFERENCE_PLANT_GAINS_VEHICLE ('vehicle'\n"
            "  mixer, [rad/s^2/Nm] = 1/body-inertia) depending on --mixer, and\n"
            "  tau_m vs. the firmware's default tau_m, with the percent error\n"
            "  for both, plus which input mode was actually used and, for\n"
            "  'indirect', whether Kp was given (--kp) or auto-estimated via\n"
            "  FIR regression (with that estimate's own R^2).\n"
            "  軸ごとにKとtau_m[s]を表示し、--mixer に応じた理論値\n"
            "  （'legacy'ならREFERENCE_PLANT_GAINS [rad/s^2/duty]、'vehicle'なら\n"
            "  REFERENCE_PLANT_GAINS_VEHICLE [rad/s^2/Nm]=1/機体慣性）および\n"
            "  ファーム既定のtau_mとの誤差[%]を併記する。実際に使った入力\n"
            "  モードと、'indirect'ならKpが--kp指定か FIR回帰による自動推定かも\n"
            "  （自動推定ならそのR^2も）表示する。\n"
            "\n"
            "Capture the input data with:\n"
            "  sf log wifi -d 30\n"
            "  sf sysid fit logs/flight_<timestamp>.sflog.zip --plot\n"
            "  入力データは上記コマンドで取得する（-dは秒数。書き出し先は\n"
            "  logs/ 配下の一式ファイルで固定、-o は指定しない）。\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "input",
        nargs="?",
        help="StampFly flight-log bundle (.sflog.zip or extracted directory)",
    )
    parser.add_argument(
        "--axis",
        choices=["roll", "pitch", "yaw", "all"],
        default="all",
        help="Axis to identify (default: all)",
    )
    parser.add_argument(
        "--input",
        dest="input_mode",
        choices=["auto", "control_output", "duty", "indirect", "kp"],
        default="auto",
        help="Plant-input reconstruction mode (default: auto). 'auto' now "
             "needs NO --kp for a typical firmware/workshop log: when --kp "
             "is not given, it auto-estimates the effective proportional "
             "gain from the log's own duty via a short FIR regression "
             "(see --kp below), then runs 'indirect' with that estimate -- "
             "'indirect' fits the CLOSED-LOOP target->gyro transfer "
             "function directly and backs out K/tau_m from Kp algebraically, "
             "instead of fitting a reconstructed u(t) directly (ANY direct "
             "fit -- 'duty', 'control_output', or the old 'kp' mode -- is a "
             "textbook closed-loop identifiability trap on real, modestly- "
             "excited human-piloted flight: a human cannot safely produce "
             "the ~8Hz persistent stick motion a direct fit would need. "
             "'indirect' recovered K within a few %% to ~25%% of theory on "
             "real lesson_07 test flights where direct fits gave R^2<0 and "
             "K off by 1-3 orders of magnitude on the SAME data, "
             "2026-09-10). 'control_output' = the PRE-MIXER commanded "
             "thrust+torque (the bundle's 400Hz ctrl_output.csv, "
             "kPktCtrlOutput400/0x4B) fit directly -- no --mixer needed, "
             "but still closed-loop-biased; only used by 'auto' as a "
             "fallback when no Kp (given or auto-estimated) is available. "
             "'duty' = mixer-inverse of the bundle's 400Hz motor.csv "
             "duty_FR/RR/RL/FL fit directly (same fallback role, --mixer must match "
             "the firmware). 'kp' = the OLD direct Kp*(target-gyro) "
             "reconstruction (--kp required) -- kept for comparison/"
             "debugging, not recommended for real flight data.",
    )
    parser.add_argument(
        "--mixer",
        choices=["legacy", "vehicle"],
        default="legacy",
        help="Which mixer the 'duty'/'auto' input modes invert (default: "
             "legacy). 'legacy' = the simple linear X-quad mixer "
             "(ws_internal.hpp / vehicle_old) -- correct for `sf lesson` "
             "(firmware/workshop) and firmware/vehicle_old logs. 'vehicle' "
             "= firmware/vehicle's ACTUAL mixer (physical B^-1 allocation "
             "through a nonlinear motor curve, sf_actuator/actuator.cpp) -- "
             "required for `sf app` (firmware/vehicle-based custom "
             "controller) logs. The bundle cannot say which firmware produced "
             "it, so this is never auto-detected -- pick wrong and the fit "
             "silently reconstructs the wrong signal. Ignored when --input "
             "resolves to 'kp'.",
    )
    parser.add_argument(
        "--kp",
        type=float,
        help="P gain used during flight (must match firmware value). "
             "OPTIONAL in --input auto/indirect (default): when omitted, "
             "it is auto-estimated from the log's own duty via a short FIR "
             "regression against target-gyro -- pass this only to override "
             "that estimate (e.g. to sanity-check it against the value you "
             "configured) or when the log has no usable duty at all. "
             "Required for --input kp.",
    )
    parser.add_argument(
        "--rate-max",
        type=float,
        default=1.0,
        help="Maximum angular rate [rad/s] (default: 1.0, yaw typically 5.0). "
             "Kept for backward compatibility of scripts that pass it: a "
             "bundle's rate_ref stream is already in rad/s, so this no longer "
             "scales the target and only sets the --min-target-std-frac "
             "reference scale.",
    )
    parser.add_argument(
        "--min-target-std-frac",
        type=float,
        default=0.1,
        help="Drop a segment when std(target/rate_ref) over it is below "
             "this fraction of --rate-max (default: 0.1). A near-constant "
             "stick reference makes u=Kp*(target-y) collapse onto -Kp*y, "
             "which fits a strongly negative/implausible K that looks like "
             "a sign-inverted plant but is a closed-loop identifiability "
             "artifact -- fly with larger, more continuous stick motion "
             "instead of loosening this check. Pass 0 to disable.",
    )
    parser.add_argument(
        "--time-range",
        nargs=2,
        type=float,
        metavar=("START", "END"),
        help="Time range to analyze [seconds]",
    )
    parser.add_argument(
        "-o", "--output",
        help="Output file (YAML/JSON)",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show fit plots",
    )
    parser.add_argument(
        "--plot-output",
        help="Save plot to file (PNG/PDF)",
    )
    parser.add_argument(
        "--selftest",
        action="store_true",
        help="Run the synthetic-plant pipeline self-test and exit",
    )
    parser.set_defaults(func=run_fit)


def _register_plan(subparsers):
    """Register plan subcommand"""
    parser = subparsers.add_parser(
        "plan",
        help="Generate flight test plan",
        description="Generate a test plan for system identification experiments.",
    )
    parser.add_argument(
        "--type",
        choices=["noise", "inertia", "motor", "drag", "all"],
        default="all",
        help="Test type (default: all)",
    )
    parser.add_argument(
        "-o", "--output",
        help="Output file (Markdown)",
    )
    parser.set_defaults(func=run_plan)


# =============================================================================
# Command implementations
# =============================================================================

def run_help(args: argparse.Namespace) -> int:
    """Show help when no subcommand specified"""
    console.print("Usage: sf sysid <subcommand> [options]")
    console.print()
    console.print("Subcommands:")
    console.print("  noise      Sensor noise characterization (Allan variance)")
    console.print("  inertia    Moment of inertia estimation (step response)")
    console.print("  motor      Motor dynamics identification")
    console.print("  fit        Fit plant model G_p(s) = K/(s*(tau_m*s+1))")
    console.print("  drag       Aerodynamic drag coefficient estimation")
    console.print("  params     Parameter management")
    console.print("  validate   Validation and consistency checks")
    console.print("  plan       Flight test plan generation")
    console.print("  rate-fit   Identify rate-loop plant G(s)=b*e^(-Ls)/(s(Ts+1)) (ETFE + fit)")
    console.print("  rate-tune  Auto-tune rate PID for a gain-crossover + phase-margin spec")
    console.print("  rate-excite Fly rate-loop excitation (chirp/doublet) via the API")
    console.print()
    console.print("Run 'sf sysid <subcommand> --help' for details.")
    console.print()
    console.print("Examples:")
    console.print("  sf sysid noise static.sflog.zip --sensor all --plot")
    console.print("  sf sysid fit flight.sflog.zip --plot")
    console.print("  sf sysid inertia roll_step.sflog.zip --axis roll -o result.yaml")
    console.print("  sf sysid params show")
    console.print("  sf sysid validate identified.yaml --ref defaults.yaml")
    console.print("  sf sysid rate-fit flight.sflog.zip --axis roll --plot   # duty-based, no --kp needed")
    console.print("  sf sysid rate-tune --fit fit.json --wc 25 --pm 60")
    console.print("  sf sysid rate-excite --axis roll --takeoff --land")
    return 0


def _resolve_plot_target(
    args: argparse.Namespace, info: 'plotting.BackendInfo', suffix: str,
) -> tuple:
    """Decide where a sysid plot goes: a live window, the user's explicit
    --plot-output path, or (when no GUI backend works) a PNG saved next to
    the input file.
    sysid のプロットの行き先を決める: ライブウィンドウか、ユーザー指定の
    --plot-output パスか、（GUIバックエンドが使えない場合は）入力ファイルの
    隣に保存する PNG か。

    Args:
        args: parsed CLI args (reads .plot, .plot_output, .input)
        info: backend chosen by plotting.select_backend(), called by the
            caller BEFORE importing sysid.visualizer
        suffix: filename suffix for the headless fallback PNG (e.g. "_fit")

    Returns (plot_output_base, show, headless):
        plot_output_base: None (let the module show without saving) or a
            Path base to save to -- same base --plot-output already used,
            so per-axis filenames are derived from it the same way.
        show: whether to open a live window.
        headless: True when this function chose the PNG fallback path --
            the caller must open the produced file(s) with the default
            viewer afterwards.
    """
    if args.plot_output:
        # Explicit file: save there; also show a window only when one is
        # possible (never call plt.show() on a headless backend).
        # 明示的なファイル指定: そこへ保存し、ウィンドウは表示可能なときだけ
        # 開く（ヘッドレスなバックエンドで plt.show() は呼ばない）。
        if args.plot and not info.interactive:
            console.warning("No GUI backend for matplotlib is usable; plot window skipped (file saved).")
        return Path(args.plot_output), args.plot and info.interactive, False

    if not args.plot:
        return None, False, False

    if info.interactive:
        console.info(f"Plot window backend: {info.name}")
        return None, True, False

    fallback_base = plotting.default_png_path(Path(args.input), suffix)
    plotting.report_headless(console, info, fallback_base)
    return fallback_base, False, True


def run_fit(args: argparse.Namespace) -> int:
    """Run plant model fitting"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.loader import load_aligned
        from sysid.plant_fit import (
            fit_plant, compute_fit_timeseries, REFERENCE_PLANT_GAINS,
            REFERENCE_PLANT_GAINS_VEHICLE, selftest,
        )
        from sysid.defaults import get_flat_defaults
    except ImportError as e:
        console.error(f"Failed to import sysid.plant_fit: {e}")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    if args.selftest:
        return 0 if selftest() else 1

    if not args.input:
        console.error("input bundle required (or --selftest)")
        return 1
    # NOTE: --kp is NOT required here unconditionally -- with --input auto
    # (default) or --input duty, fit_plant() reads the plant input from the
    # 400Hz motor-duty columns instead. fit_plant() raises a clear ValueError
    # (caught per-axis below) when the resolved mode is 'kp' and --kp is
    # missing, or when 'duty' is requested/resolved but the bundle lacks a
    # motor stream (genuine 400Hz duty).
    # 注意: --kp はここで無条件必須にしない -- --input auto（既定）や
    # --input duty では fit_plant() が 400Hz モータduty列からプラント入力を
    # 読む。解決したモードが 'kp' で --kp が無い場合、または 'duty' が
    # 指定/解決されたのに一式に motor ストリーム（本物の400Hz duty）が無い場合は、fit_plant() が
    # 明確な ValueError を出す（下の軸ごとの try/except で捕捉）。

    # Check input bundle
    # 入力一式の確認
    if not Path(args.input).exists():
        console.error(f"Input bundle not found: {args.input}")
        return 1

    # Determine axes to process
    axes = ["roll", "pitch", "yaw"] if args.axis == "all" else [args.axis]
    # Axis-specific rate_max defaults (yaw is typically higher)
    rate_max_defaults = {"roll": 1.0, "pitch": 1.0, "yaw": 5.0}

    # Load the bundle ONCE into the aligned 400Hz table every axis (and the
    # plot below) reads from -- see tools/sysid/loader.py for the column
    # contract.
    # 一式を1回だけ読み、全軸（と下のプロット）が使う整列済み400Hz表を作る
    # -- 列契約は tools/sysid/loader.py 参照。
    console.info(f"Loading bundle: {args.input}")
    try:
        df = load_aligned(args.input)
    except (ValueError, OSError) as e:
        console.error(f"Failed to load bundle: {e}")
        return 1

    results = {}
    defaults = get_flat_defaults()
    all_ok = True

    for axis in axes:
        # Use user-provided rate_max, or axis default when --axis all
        if args.axis == "all":
            rate_max = rate_max_defaults[axis]
        else:
            rate_max = args.rate_max

        try:
            result = fit_plant(
                df=df,
                axis=axis,
                kp=args.kp,
                rate_max=rate_max,
                time_range=tuple(args.time_range) if args.time_range else None,
                input_mode=args.input_mode,
                mixer=args.mixer,
                min_target_std_frac=args.min_target_std_frac,
            )
            results[axis] = result
        except ValueError as e:
            console.warning(f"  {axis}: {e}")
            all_ok = False
            continue

    if not results:
        console.error("Fitting failed for all axes.")
        return 1

    # Print summary table
    console.print()
    console.print("=== Plant Model Identification ===")
    console.print(f"  Model: G_p(s) = K / (s * (tau_m * s + 1))")
    console.print()

    for axis, r in results.items():
        # K's reference/units depend on which mixer produced it -- see
        # PlantFitResult.to_dict() in plant_fit.py for the same logic.
        # K の参照値・単位は、どちらのミキサーが生成したかで異なる --
        # plant_fit.py の PlantFitResult.to_dict() と同じロジック。
        ref_gains = REFERENCE_PLANT_GAINS_VEHICLE if r.mixer == 'vehicle' else REFERENCE_PLANT_GAINS
        K_unit = "rad/s^2 per Nm" if r.mixer == 'vehicle' else "rad/s^2 per differential duty"
        ref_K = ref_gains.get(axis, 0.0)
        ref_tau = defaults['tau_m']
        K_err = abs(r.K - ref_K) / ref_K * 100 if ref_K > 0 else 0
        tau_err = abs(r.tau_m - ref_tau) / ref_tau * 100 if ref_tau > 0 else 0

        line = (
            f"  {axis.capitalize():6s} "
            f"K = {r.K:10.1f} (ref: {ref_K:10.1f}, err: {K_err:4.1f}%)  "
            f"tau_m = {r.tau_m:.3f} (ref: {ref_tau:.3f}, err: {tau_err:4.1f}%)  "
            f"R2 = {r.r_squared:.2f}  "
            f"[{r.n_segments} segs]"
        )
        console.print(line)
        if r.input_mode == 'control_output':
            mode_desc = "control_output (pre-mixer commanded thrust+torque, 400Hz, no --mixer needed)"
        elif r.input_mode == 'duty':
            mode_desc = f"motor duty (--mixer {r.mixer} inverse of motor.csv duty_FR/RR/RL/FL, 400Hz)"
        elif r.input_mode == 'indirect':
            if r.kp_source == 'fir_auto':
                kp_desc = (f"Kp={r.kp_used:.4g} auto-estimated via FIR "
                           f"regression, R2={r.kp_auto_r_squared:.3f}")
            else:
                kp_desc = f"Kp={r.kp_used} given"
            mode_desc = f"indirect closed-loop fit (target->gyro, {kp_desc})"
        else:
            mode_desc = f"Kp reconstruction (Kp={r.kp_used})"
        console.print(f"         input: {mode_desc}  units: K [{K_unit}]")
        if r.crash_truncated_at is not None:
            console.warning(
                f"         crash/anomaly detected: gyro exceeded the sanity "
                f"bound at t={r.crash_truncated_at:.2f}s -- everything from "
                "there on was automatically excluded from fitting"
            )
        if r.duty_reason:
            console.print(f"         duty check: {r.duty_reason}")
        # Mixer-gain diagnostic (rate-sysid design memo, 2026-09-09, §07/§08):
        # populated whenever the log has genuine 400Hz duty alongside the
        # resolved input mode -- tells the caller how far the mixer that
        # actually flew is from the physical model, straight from the log.
        # ミキサーゲイン診断（2026-09-09 レート同定設計メモ §07/§08）: 本物の
        # 400Hz dutyが解決済み入力モードと揃っていれば計算される -- 実際に
        # 飛んだミキサーが物理モデルからどれだけ乖離しているかを、ログだけ
        # から呼び出し側に伝える。
        if r.mixer_gain is not None:
            console.info(
                f"         mixer gain: {r.mixer_gain:.4g}  (R^2={r.mixer_gain_r_squared:.3f})  "
                f"{r.mixer_gain_label}"
            )
        if r.target_excitation_note:
            console.warning(f"         excitation: {r.target_excitation_note}")

    # Design Kp (zeta=0.7). For --mixer vehicle fits, this is directly in the
    # SAME units (Nm/(rad/s)) as firmware/vehicle's rate.roll/pitch/yaw.kp
    # params (K there is a torque gain -- see plant_fit.py's
    # REFERENCE_PLANT_GAINS_VEHICLE docstring); for 'legacy'/'kp' fits it is
    # the legacy duty-differential-scale Kp, NOT directly usable as a
    # firmware/vehicle param.
    # 設計Kp（zeta=0.7）。--mixer vehicle のフィットでは、これは
    # firmware/vehicle の rate.roll/pitch/yaw.kp パラメータと**同じ単位**
    # （Nm/(rad/s)）になる（K がトルクゲインのため -- plant_fit.py の
    # REFERENCE_PLANT_GAINS_VEHICLE docstring参照）。'legacy'/'kp' の
    # フィットでは legacy の duty差動スケールのKpであり、firmware/vehicle の
    # パラメータにそのまま使えるものではない。
    console.print()
    console.print("  Design Kp (zeta=0.7):")
    for axis, r in results.items():
        if r.K > 0 and r.tau_m > 0:
            Kp_design = 1.0 / (4.0 * 0.7**2 * r.K * r.tau_m)
            ref_gains = REFERENCE_PLANT_GAINS_VEHICLE if r.mixer == 'vehicle' else REFERENCE_PLANT_GAINS
            ref_K = ref_gains.get(axis, 0.0)
            Kp_ref = 1.0 / (4.0 * 0.7**2 * ref_K * defaults['tau_m']) if ref_K > 0 else 0
            Kp_unit = "Nm/(rad/s)" if r.mixer == 'vehicle' else "duty/(rad/s)"
            console.print(f"    {axis.capitalize():6s} Kp = {Kp_design:.6f} (ref: {Kp_ref:.6f}) [{Kp_unit}]")

    # Save output
    if args.output:
        output_path = Path(args.output)
        data = {
            'method': 'plant_fit',
            'source': str(args.input),
            'kp': args.kp,
            'mixer': args.mixer,
            'axes': {axis: r.to_dict() for axis, r in results.items()},
        }

        with open(output_path, 'w') as f:
            if output_path.suffix == '.json':
                json.dump(data, f, indent=2)
            else:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        console.success(f"Saved: {args.output}")

    # Plot
    if args.plot or args.plot_output:
        try:
            sys.path.insert(0, str(paths.root() / "tools"))
            # Backend must be chosen before this import -- sysid.visualizer
            # imports matplotlib.pyplot at module load time.
            # このimportより前にバックエンドを選ぶこと -- sysid.visualizer は
            # モジュール読み込み時に matplotlib.pyplot を import する。
            info = plotting.select_backend(want_window=bool(args.plot))
            from sysid.visualizer import plot_plant_fit
        except ImportError:
            console.warning("matplotlib not available, skipping plot")
        else:
            plot_output_base, show, headless = _resolve_plot_target(args, info, "_fit")
            saved_paths = []
            for axis, r in results.items():
                try:
                    # Use axis-specific rate_max
                    if args.axis == "all":
                        rate_max = rate_max_defaults[axis]
                    else:
                        rate_max = args.rate_max

                    ts = compute_fit_timeseries(
                        df=df,
                        result=r,
                        rate_max=rate_max,
                        time_range=tuple(args.time_range) if args.time_range else None,
                    )

                    plot_out = None
                    if plot_output_base:
                        plot_out = str(plot_output_base.with_stem(f"{plot_output_base.stem}_{axis}"))

                    if r.input_mode == 'duty':
                        u_plant_unit = 'Nm' if r.mixer == 'vehicle' else 'duty'
                    else:
                        u_plant_unit = 'duty (Kp*err)'

                    plot_plant_fit(
                        time=ts['time'],
                        u_plant=ts['u_plant'],
                        y_measured=ts['y_measured'],
                        y_simulated=ts['y_simulated'],
                        residual=ts['residual'],
                        axis=axis,
                        K=r.K,
                        tau_m=r.tau_m,
                        r_squared=r.r_squared,
                        output_path=plot_out,
                        show=show,
                        u_plant_unit=u_plant_unit,
                    )
                    if headless and plot_out:
                        saved_paths.append(plot_out)
                except Exception as e:
                    console.warning(f"Plot failed for {axis}: {e}")

            for saved_path in saved_paths:
                plotting.open_with_default_viewer(Path(saved_path))
        finally:
            if str(paths.root() / "tools") in sys.path:
                sys.path.remove(str(paths.root() / "tools"))

    return 0 if all_ok else 1


def run_noise(args: argparse.Namespace) -> int:
    """Run noise characterization"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        # sysid.visualizer (imported later, only if a plot is requested)
        # imports matplotlib.pyplot at module load time, so it is NOT
        # imported here -- plotting.select_backend() must run first.
        # sysid.visualizer（プロットが要求された場合のみ後で import する）は
        # モジュール読み込み時に matplotlib.pyplot を import するため、
        # ここでは import しない -- plotting.select_backend() を先に
        # 実行する必要がある。
        from sysid.loader import load_aligned
        from sysid.noise import load_and_estimate
    except ImportError as e:
        console.error(f"Failed to import sysid module: {e}")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    # Check input bundle
    # 入力一式の確認
    if not Path(args.input).exists():
        console.error(f"Input bundle not found: {args.input}")
        return 1

    console.info(f"Loading bundle: {args.input}")

    try:
        df = load_aligned(args.input)
        result = load_and_estimate(
            df,
            sensor=args.sensor,
            static_only=args.static_only,
            min_duration=args.min_duration,
        )
    except Exception as e:
        console.error(f"Analysis failed: {e}")
        return 1

    console.info(f"Samples: {result.samples}, Duration: {result.duration_s:.1f}s")

    # Print summary
    console.print()
    console.print("=== Estimated Parameters ===")
    console.print()
    console.print("Process Noise (Q):")
    console.print(f"  gyro_noise:       {float(result.gyro_arw.mean()):.6f} rad/s/√Hz")
    console.print(f"  accel_noise:      {float(result.accel_vrw.mean()):.6f} m/s²/√Hz")
    console.print(f"  gyro_bias_noise:  {float(result.gyro_bias_inst.mean()):.8f}")
    console.print(f"  accel_bias_noise: {float(result.accel_bias_inst.mean()):.8f}")
    console.print()
    console.print("Measurement Noise (R):")
    console.print(f"  baro_noise: {result.baro_std:.4f} m")
    console.print(f"  tof_noise:  {result.tof_std:.4f} m")
    console.print(f"  flow_noise: {result.flow_std:.2f}")

    # Save output
    if args.output:
        output_path = Path(args.output)
        data = result.to_dict()
        data['_metadata'] = {
            'method': 'allan_variance',
            'source': str(args.input),
            'sensor': args.sensor,
        }

        with open(output_path, 'w') as f:
            if output_path.suffix == '.json':
                json.dump(data, f, indent=2)
            else:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)

        console.success(f"Saved: {args.output}")

    # Plot
    if args.plot or args.plot_output:
        try:
            sys.path.insert(0, str(paths.root() / "tools"))
            # Backend must be chosen before this import -- sysid.visualizer
            # imports matplotlib.pyplot at module load time.
            # このimportより前にバックエンドを選ぶこと -- sysid.visualizer は
            # モジュール読み込み時に matplotlib.pyplot を import する。
            info = plotting.select_backend(want_window=bool(args.plot))
            from sysid.visualizer import plot_noise_analysis
        except ImportError:
            console.warning("matplotlib not available, skipping plot")
        else:
            plot_output_base, show, headless = _resolve_plot_target(args, info, "_noise")
            output_path = str(plot_output_base) if plot_output_base else None
            try:
                plot_noise_analysis(
                    result,
                    output_path=output_path,
                    show=show,
                )
                if headless and output_path:
                    plotting.open_with_default_viewer(Path(output_path))
            except Exception as e:
                console.warning(f"Plot failed: {e}")
        finally:
            if str(paths.root() / "tools") in sys.path:
                sys.path.remove(str(paths.root() / "tools"))

    return 0


def run_inertia(args: argparse.Namespace) -> int:
    """Run inertia estimation"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.loader import load_aligned
        from sysid.inertia import estimate_inertia, load_step_response
    except ImportError as e:
        console.error(f"Failed to import sysid.inertia: {e}")
        console.print("Note: This module may not be implemented yet.")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    # Check input bundle
    # 入力一式の確認
    if not Path(args.input).exists():
        console.error(f"Input bundle not found: {args.input}")
        return 1

    console.info(f"Loading bundle: {args.input}")

    try:
        df = load_aligned(args.input)
        result = estimate_inertia(
            df,
            axis=args.axis,
            time_range=args.time_range,
        )
    except Exception as e:
        console.error(f"Estimation failed: {e}")
        return 1

    # Print results
    console.print()
    console.print("=== Estimated Inertia ===")
    for key, val in result.get('estimated', {}).items():
        console.print(f"  {key}: {val:.4e} kg·m²")

    # Save output
    if args.output:
        output_path = Path(args.output)
        with open(output_path, 'w') as f:
            if output_path.suffix == '.json':
                json.dump(result, f, indent=2)
            else:
                yaml.dump(result, f, default_flow_style=False, sort_keys=False)
        console.success(f"Saved: {args.output}")

    return 0


def run_motor(args: argparse.Namespace) -> int:
    """Run motor dynamics identification"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.loader import load_aligned
        from sysid.motor import estimate_motor_params
    except ImportError as e:
        console.error(f"Failed to import sysid.motor: {e}")
        console.print("Note: This module may not be implemented yet.")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    # Check input bundle
    # 入力一式の確認
    if not Path(args.input).exists():
        console.error(f"Input bundle not found: {args.input}")
        return 1

    console.info(f"Loading bundle: {args.input}")

    try:
        df = load_aligned(args.input)
        result = estimate_motor_params(
            df,
            param=args.param,
            mass=args.mass,
            hover_only=args.hover_only,
        )
    except Exception as e:
        console.error(f"Estimation failed: {e}")
        return 1

    # Print results
    console.print()
    console.print("=== Estimated Motor Parameters ===")
    for key, val in result.get('estimated', {}).items():
        console.print(f"  {key}: {val:.4e}")

    # Save output
    if args.output:
        output_path = Path(args.output)
        with open(output_path, 'w') as f:
            if output_path.suffix == '.json':
                json.dump(result, f, indent=2)
            else:
                yaml.dump(result, f, default_flow_style=False, sort_keys=False)
        console.success(f"Saved: {args.output}")

    return 0


def run_drag(args: argparse.Namespace) -> int:
    """Run drag coefficient estimation"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.loader import load_aligned
        from sysid.drag import estimate_drag
    except ImportError as e:
        console.error(f"Failed to import sysid.drag: {e}")
        console.print("Note: This module may not be implemented yet.")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    # Check input bundle
    # 入力一式の確認
    if not Path(args.input).exists():
        console.error(f"Input bundle not found: {args.input}")
        return 1

    console.info(f"Loading bundle: {args.input}")

    try:
        df = load_aligned(args.input)
        result = estimate_drag(
            df,
            drag_type=args.type,
        )
    except Exception as e:
        console.error(f"Estimation failed: {e}")
        return 1

    # Print results
    console.print()
    console.print("=== Estimated Drag Coefficients ===")
    for key, val in result.get('estimated', {}).items():
        console.print(f"  {key}: {val:.4e}")

    # Save output
    if args.output:
        output_path = Path(args.output)
        with open(output_path, 'w') as f:
            if output_path.suffix == '.json':
                json.dump(result, f, indent=2)
            else:
                yaml.dump(result, f, default_flow_style=False, sort_keys=False)
        console.success(f"Saved: {args.output}")

    return 0


def run_params_help(args: argparse.Namespace) -> int:
    """Show params help"""
    console.print("Usage: sf sysid params <action> [options]")
    console.print()
    console.print("Actions:")
    console.print("  show      Show default or loaded parameters")
    console.print("  diff      Compare two parameter files")
    console.print("  export    Export parameters to C header")
    return 0


def run_params_show(args: argparse.Namespace) -> int:
    """Show parameters"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.defaults import get_flat_defaults
        from sysid.params import load_params, flatten_params
    except ImportError as e:
        console.error(f"Failed to import sysid module: {e}")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    if args.file:
        if not Path(args.file).exists():
            console.error(f"File not found: {args.file}")
            return 1
        params = load_params(args.file)
    else:
        params = get_flat_defaults()

    if args.format == 'yaml':
        print(yaml.dump(params, default_flow_style=False, sort_keys=False))
    else:
        print(json.dumps(params, indent=2))

    return 0


def run_params_diff(args: argparse.Namespace) -> int:
    """Compare two parameter files"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.params import load_params, diff_params, flatten_params
    except ImportError as e:
        console.error(f"Failed to import sysid module: {e}")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    for f in [args.file1, args.file2]:
        if not Path(f).exists():
            console.error(f"File not found: {f}")
            return 1

    params1 = load_params(args.file1)
    params2 = load_params(args.file2)

    diffs = diff_params(params1, params2)

    console.print(f"Comparing: {args.file1} vs {args.file2}")
    console.print()

    if not diffs:
        console.print("  No differences found.")
    else:
        for key, v1, v2 in diffs:
            if v1 is None:
                console.print(f"  + {key}: {v2}")
            elif v2 is None:
                console.print(f"  - {key}: {v1}")
            else:
                console.print(f"  ~ {key}: {v1} -> {v2}")

    return 0


def run_params_export(args: argparse.Namespace) -> int:
    """Export parameters to C header"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.params import load_params, export_to_c_header
    except ImportError as e:
        console.error(f"Failed to import sysid module: {e}")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    if not Path(args.file).exists():
        console.error(f"File not found: {args.file}")
        return 1

    params = load_params(args.file)

    export_to_c_header(params, args.output)
    console.success(f"Exported to: {args.output}")

    return 0


def run_validate(args: argparse.Namespace) -> int:
    """Validate identified parameters"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.params import load_params, validate_params, diff_params
        from sysid.defaults import get_flat_defaults
    except ImportError as e:
        console.error(f"Failed to import sysid module: {e}")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    if not Path(args.file).exists():
        console.error(f"File not found: {args.file}")
        return 1

    params = load_params(args.file)

    # Validate
    warnings = validate_params(params)

    console.print(f"Validating: {args.file}")
    console.print()

    if warnings:
        console.print("Warnings:")
        for w in warnings:
            console.warning(f"  {w}")
    else:
        console.success("All consistency checks passed.")

    # Compare with reference
    if args.ref:
        if not Path(args.ref).exists():
            console.error(f"Reference file not found: {args.ref}")
            return 1

        ref_params = load_params(args.ref)
        diffs = diff_params(params, ref_params)

        console.print()
        console.print(f"Comparison with reference: {args.ref}")

        if diffs:
            for key, v1, v2 in diffs:
                if v1 is not None and v2 is not None:
                    try:
                        error_pct = abs(float(v1) - float(v2)) / abs(float(v2)) * 100
                        status = "OK" if error_pct < 20 else "CHECK"
                        console.print(f"  {key}: {v1} vs {v2} ({error_pct:.1f}% diff) [{status}]")
                    except (ValueError, TypeError):
                        console.print(f"  {key}: {v1} vs {v2}")

    return 0 if not warnings else 1


def run_plan(args: argparse.Namespace) -> int:
    """Generate flight test plan"""
    plans = {
        "noise": """
## Sensor Noise Characterization Test Plan

### Purpose
Characterize sensor noise parameters for ESKF tuning.

### Equipment
- StampFly with telemetry enabled
- Stable surface (no vibrations)
- Room-temperature environment

### Procedure
1. Place StampFly on stable, level surface
2. Power on and wait 10 seconds for sensor warm-up
3. Start data capture: `sf log wifi -d 60` (writes logs/flight_<timestamp>.sflog.zip)
4. Ensure vehicle is completely stationary during capture
5. Run analysis: `sf sysid noise logs/flight_<timestamp>.sflog.zip --sensor all --plot`

### Expected Duration
- Data capture: 60 seconds minimum
- Analysis: < 1 minute

### Output Parameters
- Gyroscope ARW (Angle Random Walk)
- Gyroscope bias instability
- Accelerometer VRW (Velocity Random Walk)
- Accelerometer bias instability
- Barometer altitude noise
- ToF range noise
""",
        "inertia": """
## Moment of Inertia Estimation Test Plan

### Purpose
Estimate roll, pitch, and yaw moments of inertia from step responses.

### Equipment
- StampFly in ACRO mode
- Clear flight area (minimum 2m x 2m)
- High-rate telemetry (400Hz)

### Safety
- Battery below 80% charge (reduced thrust margin)
- Props guards recommended
- Experienced pilot required

### Procedure (Roll)
1. Take off and hover at ~0.5m altitude
2. Start data capture: `sf log wifi -d 20` (writes logs/flight_<timestamp>.sflog.zip)
3. Apply quick roll stick input (±50%) and release
4. Wait for oscillation to settle
5. Repeat 3-5 times
6. Land and analyze: `sf sysid inertia logs/flight_<timestamp>.sflog.zip --axis roll --plot`

### Procedure (Pitch)
- Same as roll, using pitch stick

### Procedure (Yaw)
- Same as roll, using yaw stick
- Note: Yaw response is typically slower

### Expected Duration
- Per axis: 2-3 minutes flight time
- Total: ~10 minutes including setup

### Output Parameters
- Ixx (roll moment of inertia)
- Iyy (pitch moment of inertia)
- Izz (yaw moment of inertia)
""",
        "motor": """
## Motor Dynamics Identification Test Plan

### Purpose
Identify thrust coefficient (Ct), torque coefficient (Cq), and motor time constant (τm).

### Equipment
- StampFly in hover mode
- Clear flight area
- High-rate telemetry (400Hz)
- Known vehicle mass (default: 35g)

### Procedure (Hover - for Ct)
1. Take off and achieve stable hover
2. Start data capture: `sf log wifi -d 30` (writes logs/flight_<timestamp>.sflog.zip)
3. Maintain hover for at least 20 seconds
4. Land and analyze: `sf sysid motor logs/flight_<timestamp>.sflog.zip --param Ct`

### Procedure (Throttle Step - for τm)
1. Hover at ~0.3m altitude
2. Start data capture: `sf log wifi -d 20` (writes logs/flight_<timestamp>.sflog.zip)
3. Apply quick throttle increase (50% → 70%)
4. Hold for 2 seconds, then return
5. Repeat 3-5 times
6. Analyze: `sf sysid motor logs/flight_<timestamp>.sflog.zip --param tau --plot`

### Expected Duration
- Hover test: 2-3 minutes
- Throttle step test: 5 minutes
- Total: ~10 minutes

### Output Parameters
- Ct (thrust coefficient)
- Cq (torque coefficient)
- κ (torque/thrust ratio)
- τm (motor time constant)
""",
        "drag": """
## Aerodynamic Drag Estimation Test Plan

### Purpose
Estimate translational and rotational drag coefficients.

### Equipment
- StampFly with velocity estimation enabled
- Open flight area (minimum 3m x 3m)
- High-rate telemetry (400Hz)

### Procedure (Translational Drag)
1. Take off and hover at ~1m altitude
2. Start data capture: `sf log wifi -d 30` (writes logs/flight_<timestamp>.sflog.zip)
3. Apply forward velocity (pitch forward)
4. Cut throttle momentarily and observe deceleration
5. Repeat for backward, left, right
6. Analyze: `sf sysid drag logs/flight_<timestamp>.sflog.zip --type trans --plot`

### Procedure (Rotational Drag)
1. Hover at ~0.5m altitude
2. Start data capture: `sf log wifi -d 20` (writes logs/flight_<timestamp>.sflog.zip)
3. Apply yaw rate input
4. Release and observe yaw rate decay
5. Repeat 3-5 times
6. Analyze: `sf sysid drag logs/flight_<timestamp>.sflog.zip --type rot --plot`

### Expected Duration
- Translational: 5-10 minutes
- Rotational: 5 minutes
- Total: ~15 minutes

### Output Parameters
- Cd_trans (translational drag coefficient)
- Cd_rot (rotational drag coefficient)
""",
    }

    console.print("# StampFly System Identification Test Plans")
    console.print()

    if args.type == "all":
        for name, plan in plans.items():
            console.print(plan)
            console.print()
    elif args.type in plans:
        console.print(plans[args.type])
    else:
        console.error(f"Unknown test type: {args.type}")
        return 1

    if args.output:
        content = ""
        if args.type == "all":
            content = "# StampFly System Identification Test Plans\n\n"
            for name, plan in plans.items():
                content += plan + "\n\n"
        else:
            content = f"# StampFly System Identification Test Plans\n{plans[args.type]}"

        Path(args.output).write_text(content, encoding="utf-8")
        console.success(f"Saved: {args.output}")

    return 0


# =============================================================================
# Rate-loop identification + PID auto-tuning (backend: tools/log_analyzer/
# rate_sysid.py). Workflow on hardware:
#   1. sf log wifi -d 30                (start the 400Hz capture -> a
#                                         logs/flight_<timestamp>.sflog.zip
#                                         bundle -- see lib/sflog)
#   2. sf sysid rate-excite --axis roll (API: takeoff -> chirp -> land)
#   3. sf sysid rate-fit logs/flight_<timestamp>.sflog.zip --axis roll
#                                        (-> plant b, T, L)
#   4. sf sysid rate-tune --fit fit.json --wc 25 --pm 60   (-> param set lines)
# レートループ同定＋PID自動チューニング（バックエンド: rate_sysid.py）。
# 実機手順: 400Hzキャプチャ（一式を書く） → API励振飛行 → rate-fit → rate-tune。
# =============================================================================

def _rate_backend():
    import sys as _sys
    backend = paths.root() / "tools" / "log_analyzer"
    if str(backend) not in _sys.path:
        _sys.path.insert(0, str(backend))
    import rate_sysid
    return rate_sysid


def _register_rate_fit(subparsers):
    parser = subparsers.add_parser(
        "rate-fit",
        help="Identify the rate-loop plant G(s)=b·e^(-Ls)/(s(Ts+1)) from a flight-log bundle",
        description=(
            "Recovers the rate-PID output u(t). The PRIMARY path (--input duty, "
            "default when the log has it) reads the 400Hz motor-duty entry "
            "directly and inverts firmware/vehicle's real mixer -- exact, and "
            "correct even if the gains that flew are unknown or changed "
            "mid-flight (same implementation `sf sysid fit --mixer vehicle` "
            "uses). Only logs without genuine 400Hz duty (older firmware/"
            "capture) fall back to --input kp (exact firmware PID replay on "
            "rate_ref/gyro, needs --kp/--ti/--td). Either way, computes the "
            "ETFE over the excited band and fits b (1/inertia), T (motor lag), "
            "L (dead time). Run --selftest to verify both input paths against "
            "a synthetic known plant."),
        epilog=(
            "Capture the input data with:\n"
            "  sf log wifi -d 30\n"
            "  sf sysid rate-fit logs/flight_<timestamp>.sflog.zip --axis roll --plot\n"
            "入力データは上記コマンドで取得する（-dは秒数）。\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "input", nargs="?",
        help="StampFly flight-log bundle (.sflog.zip or extracted directory)",
    )
    parser.add_argument("--axis", choices=["roll", "pitch", "yaw"], default="roll",
                        help="axis to identify (default: roll)")
    parser.add_argument("--input-mode", dest="input_mode",
                        choices=["auto", "duty", "kp"], default="auto",
                        help="how to recover u(t) (default: auto -- prefers the "
                             "400Hz motor-duty reconstruction, falls back to "
                             "--kp replay only for logs without genuine 400Hz "
                             "duty). 'duty' forces the duty path (error if "
                             "unavailable); 'kp' forces the legacy PID-replay "
                             "path for old logs (requires --kp)")
    parser.add_argument("--kp", type=float,
                        help="rate Kp that flew -- only used by the --input-mode "
                             "kp/auto-fallback replay path (default: firmware "
                             "default); not needed for the duty path")
    parser.add_argument("--ti", type=float, help="rate Ti that flew (kp-replay path only)")
    parser.add_argument("--td", type=float, help="rate Td that flew (kp-replay path only)")
    parser.add_argument("--f-lo", type=float, default=0.8, help="fit band low [Hz]")
    parser.add_argument("--f-hi", type=float, default=30.0, help="fit band high [Hz]")
    parser.add_argument("-o", "--output", help="write the fit result JSON here")
    parser.add_argument("--selftest", action="store_true",
                        help="run the synthetic-plant pipeline self-test and exit")
    parser.add_argument("--plot", action="store_true",
                        help="save a Bode (measured vs fit) + coherence figure next to the input bundle")
    parser.add_argument("--plot-output", help="path for the Bode+coherence PNG (implies --plot)")
    parser.set_defaults(func=run_rate_fit)


def run_rate_fit(args) -> int:
    rs = _rate_backend()
    if args.selftest:
        return 0 if rs.selftest() else 1
    if not args.input:
        console.error("input bundle required (or --selftest)")
        return 1
    if not Path(args.input).exists():
        console.error(f"Input bundle not found: {args.input}")
        return 1
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.loader import load_aligned
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))
    console.info(f"Loading bundle: {args.input}")
    df = load_aligned(args.input)
    gains = {k: v for k, v in
             (("kp", args.kp), ("ti", args.ti), ("td", args.td)) if v is not None}
    plot_path = None
    if args.plot_output:
        plot_path = args.plot_output
    elif args.plot:
        # Path.with_suffix("") on a "*.sflog.zip" bundle only strips the
        # trailing ".zip" (leaving "*.sflog") -- fine as a filename base, it
        # just keeps the ".sflog" fragment in the plot's file name.
        # "*.sflog.zip" 一式に Path.with_suffix("") を使うと末尾の ".zip" だけ
        # 剥がれる（"*.sflog" が残る）-- ファイル名の元としては問題なく、
        # プロットのファイル名に ".sflog" の断片が残るだけ。
        plot_path = str(Path(args.input).with_suffix("")) + f"_bode_{args.axis}.png"
    result = rs.fit_from_df(df, args.axis, gains=gains,
                            f_lo=args.f_lo, f_hi=args.f_hi, plot_path=plot_path,
                            input_mode=args.input_mode)
    input_desc = ("400Hz motor duty (firmware/vehicle mixer inverted)"
                  if result["input_mode"] == "duty" else "PID replay (--kp)")
    console.info(f"axis {result['axis']}: input={input_desc}")
    if result["input_mode"] == "kp":
        console.info(f"  ({result['duty_reason']})")
    console.info(f"b={result['b']:.0f} 1/(kg m^2)  (J_eff={result['inertia_eff']:.3e})")
    console.info(f"T={result['T'] * 1e3:.1f} ms (motor lag)   "
                 f"L={result['L'] * 1e3:.2f} ms (dead time)   "
                 f"coherence={result['coherence_mean']:.2f}")
    if result["coherence_mean"] < 0.6:
        console.warning("coherence < 0.6 — weak excitation or noisy data; "
                        "re-fly with larger amplitude / longer chirp")
    if result.get("plot_path"):
        console.success(f"Bode + coherence figure: {result['plot_path']}")
    if args.output:
        import json as _json
        Path(args.output).write_text(_json.dumps(result, indent=2))
        console.info(f"fit JSON: {args.output}")
    return 0


def _register_rate_tune(subparsers):
    parser = subparsers.add_parser(
        "rate-tune",
        help="Auto-tune the firmware rate PID for a gain-crossover + phase-margin spec",
        description=(
            "Solves the firmware's exact PID form C(s)=Kp(1+1/(Ti s)+"
            "Td s/(0.125 Td s+1)) so that |C G(jwc)|=1 and PM is met, then "
            "verifies the margins numerically. Plant from --fit JSON, "
            "explicit --plant b,T,L, or --from-specs (mechanical parameters)."),
    )
    parser.add_argument("--fit", help="fit JSON from rate-fit")
    parser.add_argument("--plant", help="explicit b,T,L (e.g. 109000,0.02,0.005)")
    parser.add_argument("--from-specs", choices=["roll", "pitch", "yaw"],
                        help="use the mechanical-spec plant for this axis")
    parser.add_argument("--wc", type=float, default=25.0,
                        help="gain-crossover target [rad/s] (default 25)")
    parser.add_argument("--pm", type=float, default=60.0,
                        help="phase-margin target [deg] (default 60)")
    parser.add_argument("--ti-factor", type=float, default=10.0,
                        help="Ti = ti_factor/wc (default 10 — integral well below crossover)")
    parser.set_defaults(func=run_rate_tune)


def run_rate_tune(args) -> int:
    rs = _rate_backend()
    axis = None
    if args.fit:
        import json as _json
        fit = _json.loads(Path(args.fit).read_text())
        b, T, L = fit["b"], fit["T"], fit["L"]
        axis = fit.get("axis")
        console.info(f"plant from fit ({axis}): b={b:.0f} T={T * 1e3:.1f}ms L={L * 1e3:.2f}ms")
    elif args.plant:
        b, T, L = (float(x) for x in args.plant.split(","))
    elif args.from_specs:
        axis = args.from_specs
        b = 1.0 / rs.SPEC_INERTIA[axis]
        T, L = rs.SPEC_MOTOR_T, rs.SPEC_DELAY_L
        console.info(f"mechanical-spec plant ({axis}): b={b:.0f} "
                     f"T={T * 1e3:.0f}ms L={L * 1e3:.1f}ms")
    else:
        console.error("need --fit, --plant or --from-specs")
        return 1

    try:
        tune = rs.tune_pid(b, T, L, wc=args.wc, pm_deg=args.pm,
                           ti_factor=args.ti_factor)
    except ValueError as exc:
        console.error(str(exc))
        return 1

    ach = tune["achieved"]
    console.info(f"PID: kp={tune['kp']:.4e}  ti={tune['ti']:.4f}  td={tune['td']:.5f}")
    console.info(f"verified: wc={ach['wc']:.1f} rad/s  PM={ach['pm_deg']:.1f} deg  "
                 f"GM={ach['gm_db']:.1f} dB (w180={ach['w180']:.0f} rad/s)")
    if axis:
        print()
        print("# apply on the vehicle (CLI / TCP) — live, then persist when landed:")
        print(f"param set rate.{axis}.kp {tune['kp']:.6e}")
        print(f"param set rate.{axis}.ti {tune['ti']:.4f}")
        print(f"param set rate.{axis}.td {tune['td']:.5f}")
        print("param save")
    return 0


def _register_rate_excite(subparsers):
    parser = subparsers.add_parser(
        "rate-excite",
        help="Fly the identification excitation via the Tello-style API",
        description=(
            "Connects to the vehicle API (UDP :8889), optionally takes off "
            "(POS_HOLD hover), runs the rate-loop chirp on one axis, and "
            "optionally lands. Start `sf log wifi` in another terminal FIRST "
            "to capture the 400Hz rate_ref/gyro data the fit needs."),
    )
    parser.add_argument("--ip", default="192.168.10.1", help="vehicle IP")
    parser.add_argument("--axis", choices=["roll", "pitch", "yaw"], default="roll",
                        help="axis to excite (default: roll)")
    parser.add_argument("--waveform", choices=["chirp", "doublet"], default="chirp",
                        help="excitation waveform (default: chirp)")
    parser.add_argument("--amp", type=float, default=25.0, help="amplitude [deg/s] (default 25)")
    parser.add_argument("--dur", type=float, default=5.0, help="duration [s] (default 5)")
    parser.add_argument("--takeoff", action="store_true", help="take off first (POS_HOLD)")
    parser.add_argument("--land", action="store_true", help="land afterwards")
    parser.set_defaults(func=run_rate_excite)


def run_rate_excite(args) -> int:
    import sys as _sys
    sdk_dir = paths.root() / "tools" / "stampfly_py"
    if str(sdk_dir) not in _sys.path:
        _sys.path.insert(0, str(sdk_dir))
    from stampfly import StampFly, StampFlyError

    console.info(f"connecting to {args.ip} ... (keep the RC neutral; "
                 "start `sf log wifi` first to capture)")
    fly = StampFly(args.ip)
    try:
        fly.connect()
        if args.takeoff:
            console.info("takeoff (POS_HOLD, 0.8 m) ...")
            fly.takeoff()
        console.info(f"excitation: {args.axis} {args.waveform} "
                     f"±{args.amp:.0f} deg/s for {args.dur:.0f}s ...")
        fly.send(f"sysid {args.axis} {args.waveform} {args.amp} {args.dur}",
                 timeout=args.dur + 10.0)
        console.info("excitation done")
        if args.land:
            console.info("landing ...")
            fly.land()
        return 0
    except StampFlyError as exc:
        console.error(str(exc))
        return 1
    finally:
        fly.close()
