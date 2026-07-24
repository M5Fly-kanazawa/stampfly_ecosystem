"""
System Identification Result Validation
システム同定結果の検証

Physical consistency checks:
- Hover thrust balance: m×g ≈ 4×Ct×ω²
- Inertia ordering: Izz > Iyy > Ixx (typical X-quad)
- κ consistency: κ = Cq/Ct
- Reasonable parameter ranges

Cross-validation between parameters.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .defaults import get_flat_defaults


@dataclass
class ValidationResult:
    """Validation result"""
    passed: bool
    warnings: List[str]
    errors: List[str]
    checks: Dict[str, Dict[str, Any]]

    def to_dict(self) -> Dict[str, Any]:
        return {
            'passed': self.passed,
            'warnings': self.warnings,
            'errors': self.errors,
            'checks': self.checks,
        }

    def print_summary(self):
        """Print validation summary to console"""
        status = "PASSED" if self.passed else "FAILED"
        print(f"Validation: {status}")
        print()

        if self.errors:
            print("Errors:")
            for e in self.errors:
                print(f"  ✗ {e}")

        if self.warnings:
            print("Warnings:")
            for w in self.warnings:
                print(f"  ⚠ {w}")

        print("\nChecks:")
        for name, check in self.checks.items():
            status = "✓" if check.get('passed', False) else "✗"
            print(f"  {status} {name}: {check.get('message', '')}")


def validate_params_comprehensive(
    params: Dict[str, Any],
    reference: Optional[Dict[str, Any]] = None,
    tolerance_pct: float = 30.0,
) -> ValidationResult:
    """
    Comprehensive parameter validation

    Args:
        params: Identified parameters (flat or nested)
        reference: Reference parameters for comparison
        tolerance_pct: Percentage tolerance for comparison

    Returns:
        ValidationResult with all checks
    """
    warnings = []
    errors = []
    checks = {}

    defaults = get_flat_defaults()

    # Helper to extract value
    def get_val(d: Dict, key: str) -> Optional[float]:
        # Try flat key first
        if key in d:
            val = d[key]
            if isinstance(val, dict) and 'value' in val:
                return val['value']
            return val if isinstance(val, (int, float)) else None

        # Try nested
        parts = key.split('.')
        current = d
        for part in parts:
            if isinstance(current, dict) and part in current:
                current = current[part]
            else:
                return None
        if isinstance(current, dict) and 'value' in current:
            return current['value']
        return current if isinstance(current, (int, float)) else None

    # Get key parameters
    mass = get_val(params, 'mass') or defaults['mass']
    Ixx = get_val(params, 'Ixx')
    Iyy = get_val(params, 'Iyy')
    Izz = get_val(params, 'Izz')
    Ct = get_val(params, 'Ct')
    Cq = get_val(params, 'Cq')
    kappa = get_val(params, 'kappa')
    tau_m = get_val(params, 'tau_m')
    Cd_trans = get_val(params, 'Cd_trans')
    Cd_rot = get_val(params, 'Cd_rot')

    g = 9.80665

    # =========================================================================
    # Check 1: Hover thrust balance
    # =========================================================================
    if Ct:
        hover_thrust_per_motor = mass * g / 4
        hover_omega = np.sqrt(hover_thrust_per_motor / Ct)

        checks['hover_balance'] = {
            'expected_omega': hover_omega,
            'message': f"Hover ω = {hover_omega:.0f} rad/s (~{hover_omega*60/(2*np.pi):.0f} RPM)",
        }

        # Reasonable range: 1500-5000 rad/s (14000-48000 RPM)
        if 1500 <= hover_omega <= 5000:
            checks['hover_balance']['passed'] = True
        else:
            checks['hover_balance']['passed'] = False
            warnings.append(
                f"Hover angular velocity ({hover_omega:.0f} rad/s) outside typical range. "
                f"Check mass ({mass} kg) and Ct ({Ct:.2e})."
            )

    # =========================================================================
    # Check 2: Inertia ordering (typical X-quad: Izz > Iyy >= Ixx)
    # =========================================================================
    if Ixx and Iyy and Izz:
        checks['inertia_order'] = {
            'Ixx': Ixx,
            'Iyy': Iyy,
            'Izz': Izz,
        }

        if Izz > Iyy and Izz > Ixx:
            checks['inertia_order']['passed'] = True
            checks['inertia_order']['message'] = "Izz > Iyy, Izz > Ixx (OK for X-quad)"
        else:
            checks['inertia_order']['passed'] = False
            warnings.append(
                f"Unusual inertia ordering: Ixx={Ixx:.2e}, Iyy={Iyy:.2e}, Izz={Izz:.2e}. "
                f"For X-quads, typically Izz > Iyy > Ixx."
            )
            checks['inertia_order']['message'] = "Unusual ordering"

        # Check ratios
        if Iyy / Ixx > 3 or Izz / Ixx > 5:
            warnings.append(
                f"Large inertia ratios: Iyy/Ixx={Iyy/Ixx:.1f}, Izz/Ixx={Izz/Ixx:.1f}. "
                f"Verify measurements."
            )

    # =========================================================================
    # Check 3: κ consistency (κ = Cq/Ct)
    # =========================================================================
    if Ct and Cq:
        expected_kappa = Cq / Ct
        checks['kappa_consistency'] = {
            'Ct': Ct,
            'Cq': Cq,
            'computed_kappa': expected_kappa,
        }

        if kappa:
            error_pct = abs(kappa - expected_kappa) / expected_kappa * 100
            checks['kappa_consistency']['given_kappa'] = kappa
            checks['kappa_consistency']['error_pct'] = error_pct

            if error_pct < 10:
                checks['kappa_consistency']['passed'] = True
                checks['kappa_consistency']['message'] = f"κ consistent ({error_pct:.1f}% error)"
            else:
                checks['kappa_consistency']['passed'] = False
                warnings.append(
                    f"κ inconsistency: κ={kappa:.4e} but Cq/Ct={expected_kappa:.4e} "
                    f"({error_pct:.1f}% error)"
                )
                checks['kappa_consistency']['message'] = f"κ inconsistent ({error_pct:.1f}% error)"
        else:
            checks['kappa_consistency']['passed'] = True
            checks['kappa_consistency']['message'] = f"κ = Cq/Ct = {expected_kappa:.4e}"

        # κ should be small (typically 5-15 mm for small quads)
        if not (0.005 <= expected_kappa <= 0.020):
            warnings.append(
                f"κ = {expected_kappa:.4e} outside typical range [0.005, 0.020] for small quads"
            )

    # =========================================================================
    # Check 4: Motor time constant
    # =========================================================================
    if tau_m:
        checks['motor_time_constant'] = {
            'tau_m': tau_m,
        }

        # Typical range: 10-100 ms
        if 0.01 <= tau_m <= 0.1:
            checks['motor_time_constant']['passed'] = True
            checks['motor_time_constant']['message'] = f"τm = {tau_m*1000:.1f} ms (typical)"
        else:
            checks['motor_time_constant']['passed'] = False
            if tau_m < 0.01:
                warnings.append(f"Motor time constant ({tau_m*1000:.1f} ms) unusually fast")
            else:
                warnings.append(f"Motor time constant ({tau_m*1000:.1f} ms) unusually slow")
            checks['motor_time_constant']['message'] = f"τm = {tau_m*1000:.1f} ms (unusual)"

    # =========================================================================
    # Check 5: Drag coefficients
    # =========================================================================
    if Cd_trans:
        checks['trans_drag'] = {
            'Cd_trans': Cd_trans,
        }

        # Typical range for small quads
        if 0.01 <= Cd_trans <= 0.5:
            checks['trans_drag']['passed'] = True
            checks['trans_drag']['message'] = f"Cd_trans = {Cd_trans:.3f} (reasonable)"
        else:
            checks['trans_drag']['passed'] = False
            warnings.append(f"Translational drag coefficient ({Cd_trans:.3f}) outside typical range")
            checks['trans_drag']['message'] = f"Cd_trans = {Cd_trans:.3f} (unusual)"

    if Cd_rot:
        checks['rot_drag'] = {
            'Cd_rot': Cd_rot,
        }

        # Typical range
        if 1e-7 <= Cd_rot <= 1e-4:
            checks['rot_drag']['passed'] = True
            checks['rot_drag']['message'] = f"Cd_rot = {Cd_rot:.2e} (reasonable)"
        else:
            checks['rot_drag']['passed'] = False
            warnings.append(f"Rotational drag coefficient ({Cd_rot:.2e}) outside typical range")
            checks['rot_drag']['message'] = f"Cd_rot = {Cd_rot:.2e} (unusual)"

    # =========================================================================
    # Check 6: Positive values
    # =========================================================================
    positive_params = [
        ('mass', mass),
        ('Ixx', Ixx),
        ('Iyy', Iyy),
        ('Izz', Izz),
        ('Ct', Ct),
        ('Cq', Cq),
    ]

    for name, val in positive_params:
        if val is not None and val <= 0:
            errors.append(f"{name} must be positive (got {val})")

    # =========================================================================
    # Check 7: Comparison with reference
    # =========================================================================
    if reference:
        ref = reference
    else:
        ref = defaults

    comparison_params = ['mass', 'Ixx', 'Iyy', 'Izz', 'Ct', 'Cq', 'kappa', 'tau_m']
    for param_name in comparison_params:
        est_val = get_val(params, param_name)
        ref_val = get_val(ref, param_name) if isinstance(ref, dict) else ref.get(param_name)

        if est_val is not None and ref_val is not None and ref_val != 0:
            error_pct = abs(est_val - ref_val) / abs(ref_val) * 100

            checks[f'ref_{param_name}'] = {
                'estimated': est_val,
                'reference': ref_val,
                'error_pct': error_pct,
                'passed': error_pct < tolerance_pct,
                'message': f"{param_name}: {est_val:.3e} vs ref {ref_val:.3e} ({error_pct:.1f}%)",
            }

            if error_pct >= tolerance_pct:
                warnings.append(
                    f"{param_name} differs from reference by {error_pct:.1f}% "
                    f"(>{tolerance_pct}% threshold)"
                )

    # Overall result
    passed = len(errors) == 0 and all(c.get('passed', True) for c in checks.values())

    return ValidationResult(
        passed=passed,
        warnings=warnings,
        errors=errors,
        checks=checks,
    )


def cross_validate_inertia_thrust(
    Ixx: float,
    Iyy: float,
    Izz: float,
    Ct: float,
    arm_length: float = 0.023,
    mass: float = 0.037,
) -> Dict[str, Any]:
    """
    Cross-validate inertia and thrust coefficient estimates

    At hover, the control authority depends on both inertia and Ct.
    Maximum angular acceleration = max_torque / I

    Args:
        Ixx, Iyy, Izz: Moments of inertia [kg·m²]
        Ct: Thrust coefficient [N/(rad/s)²]
        arm_length: Motor arm length [m]
        mass: Vehicle mass [kg]

    Returns:
        Dictionary with cross-validation results
    """
    g = 9.80665

    # Hover omega
    hover_thrust_per_motor = mass * g / 4
    hover_omega = np.sqrt(hover_thrust_per_motor / Ct)

    # Maximum differential thrust (assuming 50% throttle margin)
    max_diff_thrust = hover_thrust_per_motor * 0.5

    # Maximum torque
    max_roll_torque = 2 * max_diff_thrust * arm_length  # Two motors contribute
    max_pitch_torque = 2 * max_diff_thrust * arm_length
    max_yaw_torque = 4 * max_diff_thrust * 0.01  # κ ≈ 0.01

    # Maximum angular acceleration
    max_roll_accel = max_roll_torque / Ixx
    max_pitch_accel = max_pitch_torque / Iyy
    max_yaw_accel = max_yaw_torque / Izz

    return {
        'hover_omega': hover_omega,
        'max_roll_accel_rad_s2': max_roll_accel,
        'max_pitch_accel_rad_s2': max_pitch_accel,
        'max_yaw_accel_rad_s2': max_yaw_accel,
        'roll_bandwidth_estimate_hz': np.sqrt(max_roll_accel / Ixx) / (2 * np.pi),
        'pitch_bandwidth_estimate_hz': np.sqrt(max_pitch_accel / Iyy) / (2 * np.pi),
    }
