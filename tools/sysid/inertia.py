"""
Moment of Inertia Estimation from Step Response
ステップ応答からの慣性モーメント推定

Method: τ = I × α (Torque = Moment of Inertia × Angular Acceleration)

Using reconstructed torques from motor commands and measured angular rates,
we estimate Ixx, Iyy, Izz by fitting τ/α.
"""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy import signal
from scipy.optimize import least_squares

from .defaults import get_flat_defaults


# =============================================================================
# Motor Model and Mixing Parameters (from reconstruct_duties.py)
# =============================================================================

# Motor/propeller coefficients. Ct is PROVISIONAL as of 2026-08-03 (the
# 2026-07-15 thrust-stand value was retracted for lack of a valid
# simultaneous voltage/RPM/thrust measurement on the new propeller); Cq is
# coast-down MEASURED 2026-07-15 and still trusted. See
# control/models/stampfly_physical.yaml measured_2026_07.Ct for the full
# provenance of the provisional value.
CT = 1.0e-8            # Thrust coefficient [N/(rad/s)²] (2026-08-03暫定採用)
CQ = 4.10e-11          # Torque coefficient [Nm/(rad/s)²] (2026-07-15実測)
KAPPA = 4.10e-3        # Torque/thrust ratio [m] (Cq/Ct, 2026-08-03改定)

# Geometry
D = 0.023              # Moment arm [m]

# Motor model
# Rm/Km sourced from tools/sysid/defaults.py (single source of truth) instead of
# duplicating the literals here, so this file cannot re-diverge from the rest of
# the sysid tooling. NOTE: RM/KM/DM/QF/VBAT/B_INV below are currently UNUSED by
# reconstruct_torques() (torque is reconstructed from the firmware PID gains
# directly, not the motor electrical model) — kept for a possible future
# duty->torque reconstruction path; updated here for correctness/consistency.
# Rm/Km は tools/sysid/defaults.py（唯一の正）から取得し、ここでの数値の二重管理を
# 避ける。なお RM/KM/DM/QF/VBAT/B_INV は reconstruct_torques() では現在未使用
# （トルクはファームPIDゲインから再構成しており、モータ電気モデル経由ではない）—
# 将来 duty→トルク再構成が必要になった場合のために残し、整合のため値のみ更新する。
_defaults = get_flat_defaults()
RM = _defaults['Rm']   # Winding resistance [Ω] (0.593, paper LCR measurement, 2026-07-15 decision)
KM = _defaults['Km']   # Back-EMF constant [V/(rad/s)] (5.682e-4, direct measurement, 2026-07-16)
DM = 3.69e-8           # Viscous friction [Nm*s/rad] (matches defaults.py DEFAULT_PARAMS['motor']['Dm'])
QF = 2.76e-5           # Static friction [Nm] (matches defaults.py DEFAULT_PARAMS['motor']['Qf'])
VBAT = 3.7             # Battery voltage [V]

# PID gains (from firmware)
ROLL_KP = 9.1e-4       # Nm/(rad/s)
ROLL_TD = 0.01         # s
PITCH_KP = 1.33e-3     # Nm/(rad/s)
PITCH_TD = 0.025       # s
YAW_KP = 1.77e-3       # Nm/(rad/s)
YAW_TD = 0.01          # s
PID_ETA = 0.125

# Rate limits
ROLL_RATE_MAX = 1.0    # rad/s
PITCH_RATE_MAX = 1.0   # rad/s
YAW_RATE_MAX = 5.0     # rad/s

# Output limits [Nm]
ROLL_OUTPUT_LIMIT = 5.2e-3
PITCH_OUTPUT_LIMIT = 5.2e-3
YAW_OUTPUT_LIMIT = 2.2e-3

MAX_THRUST_PER_MOTOR = 0.15  # N

# Mixing matrix B_inv (control to thrust)
inv_d = 1.0 / D
inv_kappa = 1.0 / KAPPA

B_INV = np.array([
    [0.25, -0.25*inv_d,  0.25*inv_d,  0.25*inv_kappa],  # M1 (FR)
    [0.25, -0.25*inv_d, -0.25*inv_d, -0.25*inv_kappa],  # M2 (RR)
    [0.25,  0.25*inv_d, -0.25*inv_d,  0.25*inv_kappa],  # M3 (RL)
    [0.25,  0.25*inv_d,  0.25*inv_d, -0.25*inv_kappa],  # M4 (FL)
])


@dataclass
class InertiaResult:
    """Inertia estimation result"""
    Ixx: Optional[float] = None
    Iyy: Optional[float] = None
    Izz: Optional[float] = None
    Ixx_std: Optional[float] = None
    Iyy_std: Optional[float] = None
    Izz_std: Optional[float] = None

    # Fit quality
    roll_r2: Optional[float] = None
    pitch_r2: Optional[float] = None
    yaw_r2: Optional[float] = None

    # Raw data for plotting
    time: Optional[np.ndarray] = None
    torque_roll: Optional[np.ndarray] = None
    torque_pitch: Optional[np.ndarray] = None
    torque_yaw: Optional[np.ndarray] = None
    alpha_roll: Optional[np.ndarray] = None
    alpha_pitch: Optional[np.ndarray] = None
    alpha_yaw: Optional[np.ndarray] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        result = {
            'method': 'step_response',
            'timestamp': datetime.now().isoformat(),
            'estimated': {},
            'fit_quality': {},
        }

        if self.Ixx is not None:
            result['estimated']['Ixx'] = self.Ixx
            if self.Ixx_std:
                result['estimated']['Ixx_uncertainty'] = self.Ixx_std
        if self.Iyy is not None:
            result['estimated']['Iyy'] = self.Iyy
            if self.Iyy_std:
                result['estimated']['Iyy_uncertainty'] = self.Iyy_std
        if self.Izz is not None:
            result['estimated']['Izz'] = self.Izz
            if self.Izz_std:
                result['estimated']['Izz_uncertainty'] = self.Izz_std

        if self.roll_r2 is not None:
            result['fit_quality']['roll_r2'] = self.roll_r2
        if self.pitch_r2 is not None:
            result['fit_quality']['pitch_r2'] = self.pitch_r2
        if self.yaw_r2 is not None:
            result['fit_quality']['yaw_r2'] = self.yaw_r2

        # Add reference values
        defaults = get_flat_defaults()
        result['reference'] = {
            'Ixx': defaults['Ixx'],
            'Iyy': defaults['Iyy'],
            'Izz': defaults['Izz'],
        }

        # Comparison
        result['comparison'] = {}
        for key in ['Ixx', 'Iyy', 'Izz']:
            if key in result['estimated'] and key in result['reference']:
                est = result['estimated'][key]
                ref = result['reference'][key]
                error_pct = abs(est - ref) / ref * 100
                result['comparison'][key] = {
                    'error_percent': round(error_pct, 2),
                    'status': 'OK' if error_pct < 20 else 'CHECK',
                }

        return result


class PIDController:
    """Simple P+D controller for torque reconstruction"""
    def __init__(self, Kp: float, Td: float, eta: float, output_limit: float):
        self.Kp = Kp
        self.Td = Td
        self.eta = eta
        self.output_limit = output_limit
        self.deriv_filtered = 0.0
        self.prev_deriv_input = 0.0
        self.first_run = True

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        error = setpoint - measurement

        # P term
        P = self.Kp * error

        # D term (derivative on measurement)
        if self.Td > 0 and dt > 0 and not self.first_run:
            deriv_input = -measurement
            alpha = 2.0 * self.eta * self.Td / dt
            deriv_a = (alpha - 1.0) / (alpha + 1.0)
            deriv_b = 2.0 * self.Td / ((alpha + 1.0) * dt)
            deriv_diff = deriv_input - self.prev_deriv_input
            self.deriv_filtered = deriv_a * self.deriv_filtered + deriv_b * deriv_diff
            D = self.Kp * self.deriv_filtered
            self.prev_deriv_input = deriv_input
        else:
            D = 0.0
            self.prev_deriv_input = -measurement

        self.first_run = False

        output = P + D
        return np.clip(output, -self.output_limit, self.output_limit)

    def reset(self):
        self.deriv_filtered = 0.0
        self.prev_deriv_input = 0.0
        self.first_run = True


def reconstruct_torques(
    ctrl_roll: np.ndarray,
    ctrl_pitch: np.ndarray,
    ctrl_yaw: np.ndarray,
    gyro_x: np.ndarray,
    gyro_y: np.ndarray,
    gyro_z: np.ndarray,
    dt: float = 1.0/400.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Reconstruct control torques from stick inputs and gyro measurements

    Uses the same PID equations as firmware to compute torque commands.

    Args:
        ctrl_roll: Roll stick input [-1, 1]
        ctrl_pitch: Pitch stick input [-1, 1]
        ctrl_yaw: Yaw stick input [-1, 1]
        gyro_x: Roll rate [rad/s]
        gyro_y: Pitch rate [rad/s]
        gyro_z: Yaw rate [rad/s]
        dt: Sample period [s]

    Returns:
        (torque_roll, torque_pitch, torque_yaw) in [Nm]
    """
    n = len(ctrl_roll)

    roll_pid = PIDController(ROLL_KP, ROLL_TD, PID_ETA, ROLL_OUTPUT_LIMIT)
    pitch_pid = PIDController(PITCH_KP, PITCH_TD, PID_ETA, PITCH_OUTPUT_LIMIT)
    yaw_pid = PIDController(YAW_KP, YAW_TD, PID_ETA, YAW_OUTPUT_LIMIT)

    torque_roll = np.zeros(n)
    torque_pitch = np.zeros(n)
    torque_yaw = np.zeros(n)

    for i in range(n):
        # Rate setpoints (ACRO mode)
        roll_rate_sp = ctrl_roll[i] * ROLL_RATE_MAX
        pitch_rate_sp = ctrl_pitch[i] * PITCH_RATE_MAX
        yaw_rate_sp = ctrl_yaw[i] * YAW_RATE_MAX

        # PID outputs (torque commands)
        torque_roll[i] = roll_pid.update(roll_rate_sp, gyro_x[i], dt)
        torque_pitch[i] = pitch_pid.update(pitch_rate_sp, gyro_y[i], dt)
        torque_yaw[i] = yaw_pid.update(yaw_rate_sp, gyro_z[i], dt)

    return torque_roll, torque_pitch, torque_yaw


def compute_angular_acceleration(
    gyro: np.ndarray,
    dt: float,
    filter_cutoff: float = 20.0,
) -> np.ndarray:
    """
    Compute angular acceleration from gyro measurements

    Args:
        gyro: Angular rate [rad/s]
        dt: Sample period [s]
        filter_cutoff: Low-pass filter cutoff frequency [Hz]

    Returns:
        Angular acceleration [rad/s²]
    """
    fs = 1.0 / dt

    # Design low-pass filter
    if filter_cutoff < fs / 2:
        b, a = signal.butter(2, filter_cutoff / (fs / 2), 'low')
        gyro_filtered = signal.filtfilt(b, a, gyro)
    else:
        gyro_filtered = gyro

    # Differentiate
    alpha = np.gradient(gyro_filtered, dt)

    # Additional filtering on acceleration
    if filter_cutoff < fs / 2:
        alpha = signal.filtfilt(b, a, alpha)

    return alpha


def fit_inertia_linear(
    torque: np.ndarray,
    alpha: np.ndarray,
    min_alpha: float = 0.5,
) -> Tuple[float, float, float]:
    """
    Estimate moment of inertia using linear regression

    I = τ / α, fitted using least squares

    Args:
        torque: Applied torque [Nm]
        alpha: Angular acceleration [rad/s²]
        min_alpha: Minimum |α| to include (to avoid division issues)

    Returns:
        (I_estimate, I_std, r_squared)
    """
    # Filter out low acceleration points
    mask = np.abs(alpha) > min_alpha

    if np.sum(mask) < 10:
        return 0.0, 0.0, 0.0

    tau = torque[mask]
    a = alpha[mask]

    # Robust linear fit: τ = I × α
    # Using simple ratio estimation with outlier rejection
    ratios = tau / a

    # Remove outliers (IQR method)
    q1, q3 = np.percentile(ratios, [25, 75])
    iqr = q3 - q1
    valid = (ratios > q1 - 1.5*iqr) & (ratios < q3 + 1.5*iqr)

    if np.sum(valid) < 5:
        return 0.0, 0.0, 0.0

    I_estimate = np.median(ratios[valid])
    I_std = np.std(ratios[valid])

    # R² computation
    predicted = I_estimate * a[valid]
    ss_res = np.sum((tau[valid] - predicted) ** 2)
    ss_tot = np.sum((tau[valid] - np.mean(tau[valid])) ** 2)
    r_squared = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0

    return abs(I_estimate), I_std, r_squared


def detect_step_regions(
    ctrl: np.ndarray,
    threshold: float = 0.1,
    min_duration_samples: int = 40,
) -> List[Tuple[int, int]]:
    """
    Detect step input regions in control signal

    Args:
        ctrl: Control input signal
        threshold: Minimum absolute value to consider as step
        min_duration_samples: Minimum step duration in samples

    Returns:
        List of (start, end) index tuples
    """
    regions = []
    in_step = False
    start = 0

    for i, val in enumerate(ctrl):
        if abs(val) > threshold and not in_step:
            start = i
            in_step = True
        elif abs(val) <= threshold and in_step:
            if i - start >= min_duration_samples:
                regions.append((start, i))
            in_step = False

    if in_step and len(ctrl) - start >= min_duration_samples:
        regions.append((start, len(ctrl)))

    return regions


def estimate_inertia(
    filepath: str | Path,
    axis: str = "all",
    time_range: Optional[Tuple[float, float]] = None,
) -> Dict[str, Any]:
    """
    Estimate moments of inertia from step response data

    Args:
        filepath: Path to CSV log file
        axis: Which axis to analyze ("roll", "pitch", "yaw", "all")
        time_range: Optional time range [start, end] in seconds

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

    # Get gyro (corrected if available)
    gyro = np.array([s.gyro for s in log_data.samples])
    if log_data.samples[0].gyro_corrected is not None:
        gyro = np.array([s.gyro_corrected for s in log_data.samples])

    gyro_x = gyro[:, 0]
    gyro_y = gyro[:, 1]
    gyro_z = gyro[:, 2]

    # Get control inputs
    ctrl_roll = np.array([s.ctrl_roll if s.ctrl_roll else 0 for s in log_data.samples])
    ctrl_pitch = np.array([s.ctrl_pitch if s.ctrl_pitch else 0 for s in log_data.samples])
    ctrl_yaw = np.array([s.ctrl_yaw if s.ctrl_yaw else 0 for s in log_data.samples])

    # Compute dt
    dt = 1.0 / log_data.sample_rate_hz if log_data.sample_rate_hz > 0 else 1.0/400.0

    # Apply time range filter
    if time_range:
        mask = (time_s >= time_range[0]) & (time_s <= time_range[1])
        time_s = time_s[mask]
        gyro_x = gyro_x[mask]
        gyro_y = gyro_y[mask]
        gyro_z = gyro_z[mask]
        ctrl_roll = ctrl_roll[mask]
        ctrl_pitch = ctrl_pitch[mask]
        ctrl_yaw = ctrl_yaw[mask]

    # Reconstruct torques
    torque_roll, torque_pitch, torque_yaw = reconstruct_torques(
        ctrl_roll, ctrl_pitch, ctrl_yaw,
        gyro_x, gyro_y, gyro_z,
        dt,
    )

    # Compute angular accelerations
    alpha_roll = compute_angular_acceleration(gyro_x, dt)
    alpha_pitch = compute_angular_acceleration(gyro_y, dt)
    alpha_yaw = compute_angular_acceleration(gyro_z, dt)

    result = InertiaResult(
        time=time_s,
        torque_roll=torque_roll,
        torque_pitch=torque_pitch,
        torque_yaw=torque_yaw,
        alpha_roll=alpha_roll,
        alpha_pitch=alpha_pitch,
        alpha_yaw=alpha_yaw,
    )

    # Estimate inertias
    if axis in ["roll", "all"]:
        Ixx, Ixx_std, r2 = fit_inertia_linear(torque_roll, alpha_roll)
        if Ixx > 0:
            result.Ixx = Ixx
            result.Ixx_std = Ixx_std
            result.roll_r2 = r2

    if axis in ["pitch", "all"]:
        Iyy, Iyy_std, r2 = fit_inertia_linear(torque_pitch, alpha_pitch)
        if Iyy > 0:
            result.Iyy = Iyy
            result.Iyy_std = Iyy_std
            result.pitch_r2 = r2

    if axis in ["yaw", "all"]:
        Izz, Izz_std, r2 = fit_inertia_linear(torque_yaw, alpha_yaw)
        if Izz > 0:
            result.Izz = Izz
            result.Izz_std = Izz_std
            result.yaw_r2 = r2

    return result.to_dict()


def load_step_response(filepath: str | Path) -> Dict[str, np.ndarray]:
    """
    Load step response data from CSV

    Returns dict with: time, gyro_x/y/z, ctrl_roll/pitch/yaw, etc.
    """
    # Loader lives alongside this module (tools/sysid/loader.py)
    # ローダーはこのモジュールと同じ tools/sysid/loader.py にある
    from .loader import load_csv

    log_data = load_csv(filepath)

    timestamps = np.array([s.timestamp_us for s in log_data.samples])

    data = {
        'time': (timestamps - timestamps[0]) / 1e6,
        'gyro_x': np.array([s.gyro[0] for s in log_data.samples]),
        'gyro_y': np.array([s.gyro[1] for s in log_data.samples]),
        'gyro_z': np.array([s.gyro[2] for s in log_data.samples]),
        'ctrl_roll': np.array([s.ctrl_roll or 0 for s in log_data.samples]),
        'ctrl_pitch': np.array([s.ctrl_pitch or 0 for s in log_data.samples]),
        'ctrl_yaw': np.array([s.ctrl_yaw or 0 for s in log_data.samples]),
        'sample_rate': log_data.sample_rate_hz,
    }

    return data
