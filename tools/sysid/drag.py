"""
Aerodynamic Drag Coefficient Estimation
空気抵抗係数推定

Estimates:
- Cd_trans: Translational drag coefficient (F_drag = Cd × v²)
- Cd_rot: Rotational drag coefficient (τ_drag = Cd × ω²)

Methods:
- Coastdown analysis: Fit exponential decay of velocity after throttle cut
- Yaw decay analysis: Fit exponential decay of yaw rate
"""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy.optimize import curve_fit

from .defaults import get_flat_defaults


@dataclass
class DragResult:
    """Drag coefficient estimation result"""
    Cd_trans: Optional[float] = None      # Translational drag coefficient
    Cd_rot: Optional[float] = None        # Rotational drag coefficient

    # Uncertainties
    Cd_trans_std: Optional[float] = None
    Cd_rot_std: Optional[float] = None

    # Fit quality
    trans_r2: Optional[float] = None
    rot_r2: Optional[float] = None

    # Raw data for plotting
    coastdown_time: Optional[np.ndarray] = None
    coastdown_vel: Optional[np.ndarray] = None
    coastdown_fitted: Optional[np.ndarray] = None
    yaw_decay_time: Optional[np.ndarray] = None
    yaw_decay_rate: Optional[np.ndarray] = None
    yaw_decay_fitted: Optional[np.ndarray] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        result = {
            'method': 'decay_analysis',
            'timestamp': datetime.now().isoformat(),
            'estimated': {},
            'fit_quality': {},
        }

        if self.Cd_trans is not None:
            result['estimated']['Cd_trans'] = self.Cd_trans
            if self.Cd_trans_std:
                result['estimated']['Cd_trans_uncertainty'] = self.Cd_trans_std
        if self.Cd_rot is not None:
            result['estimated']['Cd_rot'] = self.Cd_rot
            if self.Cd_rot_std:
                result['estimated']['Cd_rot_uncertainty'] = self.Cd_rot_std

        if self.trans_r2 is not None:
            result['fit_quality']['trans_r2'] = self.trans_r2
        if self.rot_r2 is not None:
            result['fit_quality']['rot_r2'] = self.rot_r2

        # Add reference values
        defaults = get_flat_defaults()
        result['reference'] = {
            'Cd_trans': defaults['Cd_trans'],
            'Cd_rot': defaults['Cd_rot'],
        }

        # Comparison
        result['comparison'] = {}
        for key in ['Cd_trans', 'Cd_rot']:
            if key in result['estimated'] and key in result['reference']:
                est = result['estimated'][key]
                ref = result['reference'][key]
                if ref != 0:
                    error_pct = abs(est - ref) / ref * 100
                    result['comparison'][key] = {
                        'error_percent': round(error_pct, 2),
                        'status': 'OK' if error_pct < 50 else 'CHECK',
                    }

        return result


def detect_decay_segments(
    signal: np.ndarray,
    threshold: float = 0.1,
    min_initial: float = 0.3,
    min_duration_samples: int = 50,
) -> List[Tuple[int, int]]:
    """
    Detect decay (coastdown) segments in a signal

    Looks for regions where:
    1. Signal starts above min_initial
    2. Signal monotonically decreases
    3. Duration is at least min_duration_samples

    Args:
        signal: Velocity or angular rate signal
        threshold: Noise threshold
        min_initial: Minimum initial value
        min_duration_samples: Minimum decay duration

    Returns:
        List of (start, end) index tuples
    """
    segments = []
    n = len(signal)
    abs_signal = np.abs(signal)

    # Find peaks above minimum
    from scipy.signal import find_peaks
    peaks, _ = find_peaks(abs_signal, height=min_initial, distance=min_duration_samples)

    for peak in peaks:
        # Look for decay after peak
        start = peak
        end = peak

        # Find where signal drops below threshold
        for i in range(peak + 1, min(peak + 500, n)):
            if abs_signal[i] < threshold:
                end = i
                break
            # Check for monotonic decrease (with some tolerance)
            if abs_signal[i] > abs_signal[i - 1] + 0.05:
                end = i
                break
            end = i

        if end - start >= min_duration_samples:
            segments.append((start, end))

    return segments


def exp_decay(t: np.ndarray, A: float, tau: float) -> np.ndarray:
    """Exponential decay: y(t) = A × exp(-t/τ)"""
    return A * np.exp(-t / tau)


def quadratic_decay(t: np.ndarray, v0: float, k: float) -> np.ndarray:
    """
    Quadratic drag decay solution
    dv/dt = -k × v²
    Solution: v(t) = v0 / (1 + k × v0 × t)
    """
    return v0 / (1 + k * v0 * t)


def fit_translational_decay(
    time: np.ndarray,
    velocity: np.ndarray,
    mass: float = 0.037,
) -> Tuple[float, float, float]:
    """
    Fit translational decay to estimate drag coefficient

    Model: m × dv/dt = -Cd × v²
    Solution: v(t) = v0 / (1 + (Cd/m) × v0 × t)

    Args:
        time: Time array [s]
        velocity: Velocity magnitude [m/s]
        mass: Vehicle mass [kg]

    Returns:
        (Cd_estimate, Cd_std, r_squared)
    """
    # Normalize time
    t = time - time[0]

    # Initial velocity
    v0 = velocity[0]
    if v0 < 0.1:
        return 0.0, 0.0, 0.0

    try:
        # Fit quadratic decay: v(t) = v0 / (1 + k×v0×t)
        # where k = Cd/m
        popt, pcov = curve_fit(
            lambda t, k: quadratic_decay(t, v0, k),
            t, velocity,
            p0=[1.0],
            bounds=([0.01], [100.0]),
            maxfev=1000,
        )

        k = popt[0]
        k_std = np.sqrt(pcov[0, 0]) if pcov[0, 0] > 0 else 0.0

        Cd = k * mass
        Cd_std = k_std * mass

        # R² computation
        v_pred = quadratic_decay(t, v0, k)
        ss_res = np.sum((velocity - v_pred) ** 2)
        ss_tot = np.sum((velocity - np.mean(velocity)) ** 2)
        r_squared = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0

        return Cd, Cd_std, r_squared

    except Exception:
        return 0.0, 0.0, 0.0


def fit_rotational_decay(
    time: np.ndarray,
    omega: np.ndarray,
    Izz: float = 20.4e-6,
) -> Tuple[float, float, float]:
    """
    Fit rotational decay to estimate rotational drag coefficient

    Model: Izz × dω/dt = -Cd_rot × ω²
    Solution: ω(t) = ω0 / (1 + (Cd_rot/Izz) × ω0 × t)

    Args:
        time: Time array [s]
        omega: Angular rate [rad/s]
        Izz: Yaw moment of inertia [kg·m²]

    Returns:
        (Cd_rot_estimate, Cd_rot_std, r_squared)
    """
    # Normalize time
    t = time - time[0]

    # Initial angular rate
    omega0 = abs(omega[0])
    if omega0 < 0.1:
        return 0.0, 0.0, 0.0

    abs_omega = np.abs(omega)

    try:
        # Fit quadratic decay
        popt, pcov = curve_fit(
            lambda t, k: quadratic_decay(t, omega0, k),
            t, abs_omega,
            p0=[0.001],
            bounds=([1e-6], [1.0]),
            maxfev=1000,
        )

        k = popt[0]
        k_std = np.sqrt(pcov[0, 0]) if pcov[0, 0] > 0 else 0.0

        Cd_rot = k * Izz
        Cd_rot_std = k_std * Izz

        # R² computation
        omega_pred = quadratic_decay(t, omega0, k)
        ss_res = np.sum((abs_omega - omega_pred) ** 2)
        ss_tot = np.sum((abs_omega - np.mean(abs_omega)) ** 2)
        r_squared = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0

        return Cd_rot, Cd_rot_std, r_squared

    except Exception:
        return 0.0, 0.0, 0.0


def estimate_drag(
    filepath: str | Path,
    drag_type: str = "all",
    mass: float = 0.037,
    Izz: float = 20.4e-6,
) -> Dict[str, Any]:
    """
    Estimate drag coefficients from coastdown/decay data

    Args:
        filepath: Path to CSV log file
        drag_type: Type to estimate ("trans", "rot", "all")
        mass: Vehicle mass [kg]
        Izz: Yaw moment of inertia [kg·m²]

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

    # Get gyro for yaw rate
    gyro = np.array([s.gyro for s in log_data.samples])
    gyro_z = gyro[:, 2]

    # Get velocity if available
    vel = None
    if log_data.samples[0].eskf_velocity is not None:
        vel = np.array([s.eskf_velocity if s.eskf_velocity is not None else [0, 0, 0]
                        for s in log_data.samples])
        vel_xy = np.sqrt(vel[:, 0]**2 + vel[:, 1]**2)  # Horizontal velocity magnitude
    else:
        vel_xy = None

    result = DragResult()

    # Estimate translational drag
    if drag_type in ["trans", "all"] and vel_xy is not None:
        segments = detect_decay_segments(vel_xy, threshold=0.1, min_initial=0.3)

        if segments:
            Cd_estimates = []
            r2_values = []

            for start, end in segments:
                Cd, Cd_std, r2 = fit_translational_decay(
                    time_s[start:end],
                    vel_xy[start:end],
                    mass,
                )
                if Cd > 0 and r2 > 0.5:
                    Cd_estimates.append(Cd)
                    r2_values.append(r2)

            if Cd_estimates:
                result.Cd_trans = np.median(Cd_estimates)
                result.Cd_trans_std = np.std(Cd_estimates)
                result.trans_r2 = np.mean(r2_values)

                # Store best segment for plotting
                best_idx = np.argmax(r2_values)
                start, end = segments[best_idx]
                result.coastdown_time = time_s[start:end] - time_s[start]
                result.coastdown_vel = vel_xy[start:end]
                # Compute fitted curve
                v0 = result.coastdown_vel[0]
                k = result.Cd_trans / mass
                result.coastdown_fitted = quadratic_decay(result.coastdown_time, v0, k)

    # Estimate rotational drag
    if drag_type in ["rot", "all"]:
        segments = detect_decay_segments(gyro_z, threshold=0.1, min_initial=0.5)

        if segments:
            Cd_rot_estimates = []
            r2_values = []

            for start, end in segments:
                Cd_rot, Cd_rot_std, r2 = fit_rotational_decay(
                    time_s[start:end],
                    gyro_z[start:end],
                    Izz,
                )
                if Cd_rot > 0 and r2 > 0.5:
                    Cd_rot_estimates.append(Cd_rot)
                    r2_values.append(r2)

            if Cd_rot_estimates:
                result.Cd_rot = np.median(Cd_rot_estimates)
                result.Cd_rot_std = np.std(Cd_rot_estimates)
                result.rot_r2 = np.mean(r2_values)

                # Store best segment for plotting
                best_idx = np.argmax(r2_values)
                start, end = segments[best_idx]
                result.yaw_decay_time = time_s[start:end] - time_s[start]
                result.yaw_decay_rate = np.abs(gyro_z[start:end])
                # Compute fitted curve
                omega0 = result.yaw_decay_rate[0]
                k = result.Cd_rot / Izz
                result.yaw_decay_fitted = quadratic_decay(result.yaw_decay_time, omega0, k)

    return result.to_dict()
