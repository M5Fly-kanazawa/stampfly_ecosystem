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

from ..utils import console, paths

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
        help="Input CSV file (static sensor data)",
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
        help="Input CSV file (step response data)",
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
        help="Input CSV file",
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
        help="Input CSV file (coastdown data)",
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
            "from closed-loop P-control flight data. Requires --kp (the P "
            "gain used during flight). The input CSV format is auto-detected: "
            "the current 400Hz Data Stream (`sf log wifi -o *.csv`, columns "
            "rate_ref_roll/pitch/yaw + gyro_x/y/z -- shared by vehicle and "
            "workshop, --rate-max is ignored since rate_ref is already rad/s) "
            "or the legacy analysis CSV (ctrl_roll/pitch/yaw + "
            "gyro_corrected_x/y/z, --rate-max required). "
            "Run --selftest to verify the whole pipeline against a synthetic "
            "known plant."
        ),
    )
    parser.add_argument(
        "input",
        nargs="?",
        help="Input CSV file (Data Stream or legacy format, auto-detected)",
    )
    parser.add_argument(
        "--axis",
        choices=["roll", "pitch", "yaw", "all"],
        default="all",
        help="Axis to identify (default: all)",
    )
    parser.add_argument(
        "--kp",
        type=float,
        help="P gain used during flight (must match firmware value; "
             "required unless --selftest)",
    )
    parser.add_argument(
        "--rate-max",
        type=float,
        default=1.0,
        help="Maximum angular rate [rad/s] (default: 1.0, yaw typically 5.0). "
             "Ignored for Data Stream CSVs -- rate_ref is already an absolute "
             "rad/s target, only the legacy ctrl*rate_max path uses this.",
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
    console.print()
    console.print("Run 'sf sysid <subcommand> --help' for details.")
    console.print()
    console.print("Examples:")
    console.print("  sf sysid noise static.csv --sensor all --plot")
    console.print("  sf sysid fit flight.csv --kp 0.5 --plot")
    console.print("  sf sysid inertia roll_step.csv --axis roll -o result.yaml")
    console.print("  sf sysid params show")
    console.print("  sf sysid validate identified.yaml --ref defaults.yaml")
    return 0


def run_fit(args: argparse.Namespace) -> int:
    """Run plant model fitting"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.plant_fit import (
            fit_plant, compute_fit_timeseries, REFERENCE_PLANT_GAINS, selftest,
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
        console.error("input CSV required (or --selftest)")
        return 1
    if args.kp is None:
        console.error("--kp is required (or --selftest)")
        return 1

    # Check input file
    if not Path(args.input).exists():
        console.error(f"Input file not found: {args.input}")
        return 1

    # Determine axes to process
    axes = ["roll", "pitch", "yaw"] if args.axis == "all" else [args.axis]
    # Axis-specific rate_max defaults (yaw is typically higher)
    rate_max_defaults = {"roll": 1.0, "pitch": 1.0, "yaw": 5.0}

    console.info(f"Loading: {args.input}")

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
                filepath=args.input,
                axis=axis,
                kp=args.kp,
                rate_max=rate_max,
                time_range=tuple(args.time_range) if args.time_range else None,
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
        ref_K = REFERENCE_PLANT_GAINS.get(axis, 0.0)
        ref_tau = defaults['tau_m']
        K_err = abs(r.K - ref_K) / ref_K * 100 if ref_K > 0 else 0
        tau_err = abs(r.tau_m - ref_tau) / ref_tau * 100 if ref_tau > 0 else 0

        line = (
            f"  {axis.capitalize():6s} "
            f"K = {r.K:6.1f} (ref: {ref_K:5.1f}, err: {K_err:4.1f}%)  "
            f"tau_m = {r.tau_m:.3f} (ref: {ref_tau:.3f}, err: {tau_err:4.1f}%)  "
            f"R2 = {r.r_squared:.2f}  "
            f"[{r.n_segments} segs]"
        )
        console.print(line)

    # Design Kp (zeta=0.7)
    console.print()
    console.print("  Design Kp (zeta=0.7):")
    for axis, r in results.items():
        if r.K > 0 and r.tau_m > 0:
            Kp_design = 1.0 / (4.0 * 0.7**2 * r.K * r.tau_m)
            ref_K = REFERENCE_PLANT_GAINS.get(axis, 0.0)
            Kp_ref = 1.0 / (4.0 * 0.7**2 * ref_K * defaults['tau_m']) if ref_K > 0 else 0
            console.print(f"    {axis.capitalize():6s} Kp = {Kp_design:.4f} (ref: {Kp_ref:.4f})")

    # Save output
    if args.output:
        output_path = Path(args.output)
        data = {
            'method': 'plant_fit',
            'source': str(args.input),
            'kp': args.kp,
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
            from sysid.visualizer import plot_plant_fit
        except ImportError:
            console.warning("matplotlib not available, skipping plot")
        else:
            for axis, r in results.items():
                try:
                    # Use axis-specific rate_max
                    if args.axis == "all":
                        rate_max = rate_max_defaults[axis]
                    else:
                        rate_max = args.rate_max

                    ts = compute_fit_timeseries(
                        filepath=args.input,
                        result=r,
                        rate_max=rate_max,
                        time_range=tuple(args.time_range) if args.time_range else None,
                    )

                    plot_out = None
                    if args.plot_output:
                        base = Path(args.plot_output)
                        plot_out = str(base.with_stem(f"{base.stem}_{axis}"))

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
                        show=args.plot,
                    )
                except Exception as e:
                    console.warning(f"Plot failed for {axis}: {e}")
        finally:
            if str(paths.root() / "tools") in sys.path:
                sys.path.remove(str(paths.root() / "tools"))

    return 0 if all_ok else 1


def run_noise(args: argparse.Namespace) -> int:
    """Run noise characterization"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.noise import load_and_estimate
        from sysid.visualizer import plot_noise_analysis
    except ImportError as e:
        console.error(f"Failed to import sysid module: {e}")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    # Check input file
    if not Path(args.input).exists():
        console.error(f"Input file not found: {args.input}")
        return 1

    console.info(f"Loading: {args.input}")

    try:
        result = load_and_estimate(
            filepath=args.input,
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
            plot_noise_analysis(
                result,
                output_path=args.plot_output,
                show=args.plot,
            )
        except ImportError:
            console.warning("matplotlib not available, skipping plot")

    return 0


def run_inertia(args: argparse.Namespace) -> int:
    """Run inertia estimation"""
    try:
        sys.path.insert(0, str(paths.root() / "tools"))
        from sysid.inertia import estimate_inertia, load_step_response
    except ImportError as e:
        console.error(f"Failed to import sysid.inertia: {e}")
        console.print("Note: This module may not be implemented yet.")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    # Check input file
    if not Path(args.input).exists():
        console.error(f"Input file not found: {args.input}")
        return 1

    console.info(f"Loading: {args.input}")

    try:
        result = estimate_inertia(
            filepath=args.input,
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
        from sysid.motor import estimate_motor_params
    except ImportError as e:
        console.error(f"Failed to import sysid.motor: {e}")
        console.print("Note: This module may not be implemented yet.")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    # Check input file
    if not Path(args.input).exists():
        console.error(f"Input file not found: {args.input}")
        return 1

    console.info(f"Loading: {args.input}")

    try:
        result = estimate_motor_params(
            filepath=args.input,
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
        from sysid.drag import estimate_drag
    except ImportError as e:
        console.error(f"Failed to import sysid.drag: {e}")
        console.print("Note: This module may not be implemented yet.")
        return 1
    finally:
        if str(paths.root() / "tools") in sys.path:
            sys.path.remove(str(paths.root() / "tools"))

    # Check input file
    if not Path(args.input).exists():
        console.error(f"Input file not found: {args.input}")
        return 1

    console.info(f"Loading: {args.input}")

    try:
        result = estimate_drag(
            filepath=args.input,
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
3. Start data capture: `sf log wifi -d 60 -o static_noise.csv`
4. Ensure vehicle is completely stationary during capture
5. Run analysis: `sf sysid noise static_noise.csv --sensor all --plot`

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
2. Start data capture: `sf log wifi -d 20 -o roll_step.csv`
3. Apply quick roll stick input (±50%) and release
4. Wait for oscillation to settle
5. Repeat 3-5 times
6. Land and analyze: `sf sysid inertia roll_step.csv --axis roll --plot`

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
2. Start data capture: `sf log wifi -d 30 -o hover.csv`
3. Maintain hover for at least 20 seconds
4. Land and analyze: `sf sysid motor hover.csv --param Ct`

### Procedure (Throttle Step - for τm)
1. Hover at ~0.3m altitude
2. Start data capture: `sf log wifi -d 20 -o throttle_step.csv`
3. Apply quick throttle increase (50% → 70%)
4. Hold for 2 seconds, then return
5. Repeat 3-5 times
6. Analyze: `sf sysid motor throttle_step.csv --param tau --plot`

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
2. Start data capture: `sf log wifi -d 30 -o coastdown.csv`
3. Apply forward velocity (pitch forward)
4. Cut throttle momentarily and observe deceleration
5. Repeat for backward, left, right
6. Analyze: `sf sysid drag coastdown.csv --type trans --plot`

### Procedure (Rotational Drag)
1. Hover at ~0.5m altitude
2. Start data capture: `sf log wifi -d 20 -o yaw_decay.csv`
3. Apply yaw rate input
4. Release and observe yaw rate decay
5. Repeat 3-5 times
6. Analyze: `sf sysid drag yaw_decay.csv --type rot --plot`

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

        Path(args.output).write_text(content)
        console.success(f"Saved: {args.output}")

    return 0


# =============================================================================
# Rate-loop identification + PID auto-tuning (backend: tools/log_analyzer/
# rate_sysid.py). Workflow on hardware:
#   1. sf log wifi -o sysid01           (start the 400Hz capture)
#   2. sf sysid rate-excite --axis roll (API: takeoff -> chirp -> land)
#   3. sf log convert sysid01 ...       (-> CSV with rate_ref/gyro)
#   4. sf sysid rate-fit sysid01.csv --axis roll      (-> plant b, T, L)
#   5. sf sysid rate-tune --fit fit.json --wc 25 --pm 60   (-> param set lines)
# レートループ同定＋PID自動チューニング（バックエンド: rate_sysid.py）。
# 実機手順: 400Hzキャプチャ → API励振飛行 → CSV変換 → rate-fit → rate-tune。
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
        help="Identify the rate-loop plant G(s)=b·e^(-Ls)/(s(Ts+1)) from a Data Stream CSV",
        description=(
            "Reconstructs the rate-PID output (exact firmware replay on "
            "rate_ref/gyro), computes the ETFE over the excited band and fits "
            "b (1/inertia), T (motor lag), L (dead time). Run --selftest to "
            "verify the whole pipeline against a synthetic known plant."),
    )
    parser.add_argument("input", nargs="?", help="Data Stream CSV (sf log convert output)")
    parser.add_argument("--axis", choices=["roll", "pitch", "yaw"], default="roll")
    parser.add_argument("--kp", type=float, help="rate Kp that flew (default: firmware default)")
    parser.add_argument("--ti", type=float, help="rate Ti that flew")
    parser.add_argument("--td", type=float, help="rate Td that flew")
    parser.add_argument("--f-lo", type=float, default=0.8, help="fit band low [Hz]")
    parser.add_argument("--f-hi", type=float, default=30.0, help="fit band high [Hz]")
    parser.add_argument("-o", "--output", help="write the fit result JSON here")
    parser.add_argument("--selftest", action="store_true",
                        help="run the synthetic-plant pipeline self-test and exit")
    parser.add_argument("--plot", action="store_true",
                        help="save a Bode (measured vs fit) + coherence figure next to the CSV")
    parser.add_argument("--plot-output", help="path for the Bode+coherence PNG (implies --plot)")
    parser.set_defaults(func=run_rate_fit)


def run_rate_fit(args) -> int:
    rs = _rate_backend()
    if args.selftest:
        return 0 if rs.selftest() else 1
    if not args.input:
        console.error("input CSV required (or --selftest)")
        return 1
    gains = {k: v for k, v in
             (("kp", args.kp), ("ti", args.ti), ("td", args.td)) if v is not None}
    plot_path = None
    if args.plot_output:
        plot_path = args.plot_output
    elif args.plot:
        plot_path = str(Path(args.input).with_suffix("")) + f"_bode_{args.axis}.png"
    result = rs.fit_from_csv(args.input, args.axis, gains=gains,
                             f_lo=args.f_lo, f_hi=args.f_hi, plot_path=plot_path)
    console.info(f"axis {result['axis']}: "
                 f"b={result['b']:.0f} 1/(kg m^2)  (J_eff={result['inertia_eff']:.3e})")
    console.info(f"T={result['T'] * 1e3:.1f} ms (motor lag)   "
                 f"L={result['L'] * 1e3:.2f} ms (dead time)   "
                 f"coherence={result['coherence_mean']:.2f}")
    if result["coherence_mean"] < 0.6:
        console.warn("coherence < 0.6 — weak excitation or noisy data; "
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
    parser.add_argument("--axis", choices=["roll", "pitch", "yaw"], default="roll")
    parser.add_argument("--waveform", choices=["chirp", "doublet"], default="chirp")
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
