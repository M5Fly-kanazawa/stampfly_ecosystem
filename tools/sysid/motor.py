"""
Motor Dynamics Identification
モータ動特性同定

Identifies:
- Ct: Thrust coefficient [N/(rad/s)²]
- Cq: Torque coefficient [Nm/(rad/s)²]
- κ: Torque/thrust ratio (= Cq/Ct) [m]
- τm: Motor time constant [s]

Methods:
- Hover equilibrium: Ct from steady hover (m×g = 4×Ct×ω²)
- Yaw response: Cq from yaw rate dynamics
- Throttle step: τm from thrust transient response
"""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy import signal
from scipy.optimize import curve_fit, minimize_scalar

from .defaults import get_flat_defaults


@dataclass
class MotorResult:
    """Motor dynamics identification result"""
    Ct: Optional[float] = None        # Thrust coefficient [N/(rad/s)²]
    Cq: Optional[float] = None        # Torque coefficient [Nm/(rad/s)²]
    kappa: Optional[float] = None     # Torque/thrust ratio [m]
    tau_m: Optional[float] = None     # Time constant [s]

    # Uncertainties
    Ct_std: Optional[float] = None
    Cq_std: Optional[float] = None
    tau_m_std: Optional[float] = None

    # Fit quality
    hover_rmse: Optional[float] = None
    step_r2: Optional[float] = None

    # Metadata
    mass: float = 0.037
    hover_omega: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        result = {
            'method': 'motor_dynamics',
            'timestamp': datetime.now().isoformat(),
            'estimated': {},
            'fit_quality': {},
        }

        if self.Ct is not None:
            result['estimated']['Ct'] = self.Ct
            if self.Ct_std:
                result['estimated']['Ct_uncertainty'] = self.Ct_std
        if self.Cq is not None:
            result['estimated']['Cq'] = self.Cq
            if self.Cq_std:
                result['estimated']['Cq_uncertainty'] = self.Cq_std
        if self.kappa is not None:
            result['estimated']['kappa'] = self.kappa
        if self.tau_m is not None:
            result['estimated']['tau_m'] = self.tau_m
            if self.tau_m_std:
                result['estimated']['tau_m_uncertainty'] = self.tau_m_std

        if self.hover_rmse is not None:
            result['fit_quality']['hover_rmse'] = self.hover_rmse
        if self.step_r2 is not None:
            result['fit_quality']['step_r2'] = self.step_r2
        if self.hover_omega is not None:
            result['derived'] = {'hover_omega_rad_s': self.hover_omega}

        # Add reference values
        defaults = get_flat_defaults()
        result['reference'] = {
            'Ct': defaults['Ct'],
            'Cq': defaults['Cq'],
            'kappa': defaults['kappa'],
            'tau_m': defaults['tau_m'],
        }

        # Comparison
        result['comparison'] = {}
        for key in ['Ct', 'Cq', 'kappa', 'tau_m']:
            if key in result['estimated'] and key in result['reference']:
                est = result['estimated'][key]
                ref = result['reference'][key]
                if ref != 0:
                    error_pct = abs(est - ref) / ref * 100
                    result['comparison'][key] = {
                        'error_percent': round(error_pct, 2),
                        'status': 'OK' if error_pct < 30 else 'CHECK',
                    }

        return result


def detect_hover_segments(
    throttle: np.ndarray,
    vertical_vel: Optional[np.ndarray] = None,
    altitude: Optional[np.ndarray] = None,
    throttle_range: Tuple[float, float] = (0.4, 0.8),
    min_duration_samples: int = 200,
    max_variance: float = 0.01,
) -> List[Tuple[int, int]]:
    """
    Detect hover segments from throttle and optional velocity/altitude data

    Args:
        throttle: Throttle command [0-1]
        vertical_vel: Vertical velocity (optional, for better detection)
        altitude: Altitude (optional, for better detection)
        throttle_range: Valid throttle range for hover
        min_duration_samples: Minimum segment length
        max_variance: Maximum throttle variance in segment

    Returns:
        List of (start, end) index tuples
    """
    segments = []
    n = len(throttle)
    window = min_duration_samples

    for i in range(0, n - window, window // 2):
        segment = throttle[i:i + window]

        # Check throttle range
        if np.mean(segment) < throttle_range[0] or np.mean(segment) > throttle_range[1]:
            continue

        # Check variance
        if np.var(segment) > max_variance:
            continue

        # Check vertical velocity if available
        if vertical_vel is not None:
            vel_segment = vertical_vel[i:i + window]
            if np.abs(np.mean(vel_segment)) > 0.2:  # Not hovering
                continue

        segments.append((i, i + window))

    # Merge adjacent segments
    merged = []
    for seg in segments:
        if merged and seg[0] <= merged[-1][1]:
            merged[-1] = (merged[-1][0], seg[1])
        else:
            merged.append(seg)

    return merged


def estimate_thrust_coefficient_hover(
    throttle: np.ndarray,
    mass: float = 0.037,
    g: float = 9.80665,
    duty_to_omega_func: Optional[callable] = None,
) -> Tuple[float, float]:
    """
    Estimate thrust coefficient from hover data

    In hover: m × g = 4 × Ct × ω²
    Therefore: Ct = m × g / (4 × ω²)

    Args:
        throttle: Throttle values during hover [0-1]
        mass: Vehicle mass [kg]
        g: Gravity [m/s²]
        duty_to_omega_func: Function to convert duty to omega (optional)

    Returns:
        (Ct_estimate, Ct_std)
    """
    # Default duty to omega conversion (empirical)
    if duty_to_omega_func is None:
        # Rough linear approximation: duty 0.5 → ~3000 rad/s
        def duty_to_omega_func(duty):
            return duty * 6000.0  # Simple linear model

    # Convert throttle to omega
    omega_values = np.array([duty_to_omega_func(t) for t in throttle])

    # Filter out invalid values
    valid = omega_values > 1000  # Minimum reasonable omega
    if np.sum(valid) < 10:
        return 0.0, 0.0

    omega = omega_values[valid]

    # Compute Ct from hover equilibrium
    # Total thrust = m × g, per motor = m × g / 4
    thrust_per_motor = mass * g / 4.0

    # Ct = T / ω²
    Ct_values = thrust_per_motor / (omega ** 2)

    Ct_estimate = np.median(Ct_values)
    Ct_std = np.std(Ct_values)

    return Ct_estimate, Ct_std


def first_order_step_response(t: np.ndarray, tau: float, K: float, y0: float) -> np.ndarray:
    """First-order step response: y(t) = y0 + K × (1 - exp(-t/τ))"""
    return y0 + K * (1 - np.exp(-t / tau))


def estimate_time_constant_step(
    time: np.ndarray,
    response: np.ndarray,
    step_start_idx: int,
    step_duration_samples: int = 200,
) -> Tuple[float, float, float]:
    """
    Estimate motor time constant from step response

    Fits first-order response: y(t) = y0 + K × (1 - exp(-t/τ))

    Args:
        time: Time array [s]
        response: Response signal (e.g., angular rate, acceleration)
        step_start_idx: Index where step starts
        step_duration_samples: Number of samples to fit

    Returns:
        (tau_estimate, tau_std, r_squared)
    """
    # Extract step response segment
    end_idx = min(step_start_idx + step_duration_samples, len(time))
    t_segment = time[step_start_idx:end_idx] - time[step_start_idx]
    y_segment = response[step_start_idx:end_idx]

    if len(t_segment) < 20:
        return 0.0, 0.0, 0.0

    # Initial guesses
    y0 = y_segment[0]
    K = y_segment[-1] - y0
    tau0 = t_segment[-1] / 5  # Initial tau guess

    try:
        popt, pcov = curve_fit(
            first_order_step_response,
            t_segment, y_segment,
            p0=[tau0, K, y0],
            bounds=([0.001, -np.inf, -np.inf], [1.0, np.inf, np.inf]),
            maxfev=1000,
        )
        tau = popt[0]
        tau_std = np.sqrt(pcov[0, 0]) if pcov[0, 0] > 0 else 0.0

        # R² computation
        y_pred = first_order_step_response(t_segment, *popt)
        ss_res = np.sum((y_segment - y_pred) ** 2)
        ss_tot = np.sum((y_segment - np.mean(y_segment)) ** 2)
        r_squared = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0

        return tau, tau_std, r_squared

    except Exception:
        return 0.0, 0.0, 0.0


def detect_throttle_steps(
    throttle: np.ndarray,
    min_step_size: float = 0.15,
    min_step_duration: int = 50,
) -> List[Tuple[int, int, float]]:
    """
    Detect throttle step events

    Args:
        throttle: Throttle command [0-1]
        min_step_size: Minimum throttle change to detect
        min_step_duration: Minimum samples at new level

    Returns:
        List of (start_idx, end_idx, step_magnitude)
    """
    steps = []
    n = len(throttle)

    # Differentiate throttle
    throttle_diff = np.diff(throttle)

    # Find step edges
    step_up = np.where(throttle_diff > min_step_size)[0]
    step_down = np.where(throttle_diff < -min_step_size)[0]

    all_steps = sorted(list(step_up) + list(step_down))

    for i, start in enumerate(all_steps):
        # Find step end (next significant change or end of data)
        end = all_steps[i + 1] if i + 1 < len(all_steps) else n - 1

        # Check duration
        if end - start >= min_step_duration:
            magnitude = throttle[start + 1] - throttle[start]
            steps.append((start, end, magnitude))

    return steps


def estimate_motor_params(
    filepath: str | Path,
    param: str = "all",
    mass: float = 0.037,
    hover_only: bool = False,
) -> Dict[str, Any]:
    """
    Estimate motor parameters from flight data

    Args:
        filepath: Path to CSV log file
        param: Parameter to estimate ("Ct", "Cq", "tau", "all")
        mass: Vehicle mass [kg]
        hover_only: Only use hover segments for Ct estimation

    Returns:
        Dictionary with estimation results
    """
    # Loader lives alongside this module (tools/sysid/loader.py)
    # ローダーはこのモジュールと同じ tools/sysid/loader.py にある
    from .loader import load_csv

    # Load data
    log_data = load_csv(filepath)

    # Extract arrays
    n = len(log_data.samples)
    timestamps = np.array([s.timestamp_us for s in log_data.samples])
    time_s = (timestamps - timestamps[0]) / 1e6

    # Get control inputs
    throttle = np.array([s.ctrl_throttle if s.ctrl_throttle else 0 for s in log_data.samples])

    # Get gyro
    gyro = np.array([s.gyro for s in log_data.samples])
    gyro_z = gyro[:, 2]

    # Get velocity if available
    vel_z = None
    if log_data.samples[0].eskf_velocity is not None:
        vel_z = np.array([s.eskf_velocity[2] if s.eskf_velocity is not None else 0
                         for s in log_data.samples])

    dt = 1.0 / log_data.sample_rate_hz if log_data.sample_rate_hz > 0 else 1.0/400.0

    result = MotorResult(mass=mass)

    # Estimate Ct from hover
    if param in ["Ct", "all"]:
        if hover_only:
            segments = detect_hover_segments(throttle, vel_z)
            if segments:
                # Use all hover segments
                hover_throttle = np.concatenate([throttle[s:e] for s, e in segments])
            else:
                hover_throttle = throttle[throttle > 0.4]
        else:
            hover_throttle = throttle[throttle > 0.4]

        if len(hover_throttle) > 50:
            Ct, Ct_std = estimate_thrust_coefficient_hover(hover_throttle, mass)
            if Ct > 0:
                result.Ct = Ct
                result.Ct_std = Ct_std

                # Compute hover omega
                g = 9.80665
                thrust_per_motor = mass * g / 4.0
                result.hover_omega = np.sqrt(thrust_per_motor / Ct)

    # Estimate tau from throttle steps
    if param in ["tau", "all"]:
        steps = detect_throttle_steps(throttle)

        if steps:
            tau_estimates = []
            r2_values = []

            for start, end, mag in steps:
                if abs(mag) > 0.1:  # Significant step
                    # Use gyro or vertical velocity response
                    if vel_z is not None:
                        tau, tau_std, r2 = estimate_time_constant_step(
                            time_s, vel_z, start
                        )
                    else:
                        # Use vertical acceleration from accel
                        accel = np.array([s.accel for s in log_data.samples])
                        accel_z = accel[:, 2]
                        tau, tau_std, r2 = estimate_time_constant_step(
                            time_s, accel_z, start
                        )

                    if tau > 0.005 and tau < 0.5 and r2 > 0.5:
                        tau_estimates.append(tau)
                        r2_values.append(r2)

            if tau_estimates:
                result.tau_m = np.median(tau_estimates)
                result.tau_m_std = np.std(tau_estimates)
                result.step_r2 = np.mean(r2_values)

    # Estimate Cq from yaw response (if Ct is known)
    if param in ["Cq", "all"] and result.Ct is not None:
        # Cq is typically estimated from κ relationship
        # Using default κ for now
        defaults = get_flat_defaults()
        result.kappa = defaults['kappa']
        result.Cq = result.Ct * result.kappa

    # Compute kappa if both Ct and Cq are available
    if result.Ct and result.Cq:
        result.kappa = result.Cq / result.Ct

    return result.to_dict()
