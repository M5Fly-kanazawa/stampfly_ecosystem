"""
Sensor Noise Characterization using Allan Variance
Allan分散によるセンサノイズ特性化

Estimates:
- Gyroscope: Angle Random Walk (ARW), Bias Instability
- Accelerometer: Velocity Random Walk (VRW), Bias Instability
- Barometer: Altitude noise
- ToF: Range noise

Reference: IEEE Std 952-1997 (IEEE Standard Specification Format Guide
           and Test Procedure for Single-Axis Interferometric Fiber Optic Gyros)
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


@dataclass
class AllanResult:
    """Allan variance analysis result
    Allan分散解析結果
    """
    tau: np.ndarray           # Cluster times [s]
    avar: np.ndarray          # Allan variance
    adev: np.ndarray          # Allan deviation (sqrt of variance)

    # Extracted parameters
    arw: float = 0.0          # Angle/Velocity Random Walk at τ=1s
    bias_instability: float = 0.0  # Minimum of Allan deviation
    bi_tau: float = 0.0       # τ at bias instability

    # For plotting
    unit: str = ""
    label: str = ""


@dataclass
class NoiseEstimate:
    """Complete sensor noise estimate
    センサノイズ推定結果
    """
    # Gyroscope (rad/s units)
    gyro_arw: np.ndarray = field(default_factory=lambda: np.zeros(3))  # ARW [rad/s/√Hz]
    gyro_bias_inst: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [rad/s]
    gyro_std: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [rad/s]
    gyro_mean: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [rad/s]

    # Accelerometer (m/s² units)
    accel_vrw: np.ndarray = field(default_factory=lambda: np.zeros(3))  # VRW [m/s²/√Hz]
    accel_bias_inst: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [m/s²]
    accel_std: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [m/s²]
    accel_mean: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [m/s²]

    # Barometer
    baro_std: float = 0.0     # [m]
    baro_mean: float = 0.0    # [m]

    # ToF
    tof_std: float = 0.0      # [m]
    tof_mean: float = 0.0     # [m]

    # Flow
    flow_std: float = 0.0     # [counts]
    flow_mean: float = 0.0    # [counts]

    # Metadata
    sample_rate_hz: float = 400.0
    duration_s: float = 0.0
    samples: int = 0

    # Allan variance results (for plotting)
    gyro_allan: List[AllanResult] = field(default_factory=list)
    accel_allan: List[AllanResult] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization"""
        return {
            "process_noise": {
                "gyro_noise": float(np.mean(self.gyro_arw)),
                "accel_noise": float(np.mean(self.accel_vrw)),
                "gyro_bias_noise": float(np.mean(self.gyro_bias_inst)),
                "accel_bias_noise": float(np.mean(self.accel_bias_inst)),
            },
            "measurement_noise": {
                "gyro_std": [float(x) for x in self.gyro_std],
                "accel_std": [float(x) for x in self.accel_std],
                "baro_noise": float(self.baro_std),
                "tof_noise": float(self.tof_std),
                "flow_noise": float(self.flow_std),
            },
            "statistics": {
                "gyro_mean": [float(x) for x in self.gyro_mean],
                "accel_mean": [float(x) for x in self.accel_mean],
                "gyro_bias_estimate": [float(x) for x in self.gyro_mean],
                "accel_bias_estimate": [float(x) for x in (self.accel_mean - np.array([0, 0, 9.81]))],
            },
            "analysis": {
                "sample_rate_hz": self.sample_rate_hz,
                "samples": self.samples,
                "duration_sec": self.duration_s,
            },
        }


def compute_allan_variance(
    data: np.ndarray,
    dt: float,
    max_clusters: int = 1000,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute Allan variance for time series data

    Args:
        data: 1D array of sensor measurements
        dt: Sample period in seconds
        max_clusters: Maximum number of cluster sizes to compute

    Returns:
        (tau, avar): Arrays of cluster times and Allan variances
    """
    n = len(data)
    if n < 4:
        return np.array([]), np.array([])

    taus = []
    avars = []

    # Cluster sizes from 1 to n/2, logarithmically spaced
    max_m = min(n // 2, max_clusters)
    m_values = np.unique(np.logspace(0, np.log10(max_m), 100).astype(int))

    for m in m_values:
        if m == 0:
            continue

        tau = m * dt

        # Number of complete clusters
        n_clusters = n // m
        if n_clusters < 2:
            continue

        # Truncate data and reshape into clusters
        truncated = data[:n_clusters * m]
        clusters = truncated.reshape(n_clusters, m).mean(axis=1)

        # Allan variance: 0.5 * E[(θ_{k+1} - θ_k)²]
        diff = np.diff(clusters)
        avar = 0.5 * np.mean(diff ** 2)

        taus.append(tau)
        avars.append(avar)

    return np.array(taus), np.array(avars)


def extract_noise_params(
    tau: np.ndarray,
    adev: np.ndarray,
) -> Tuple[float, float, float]:
    """
    Extract noise parameters from Allan deviation curve

    Args:
        tau: Cluster times [s]
        adev: Allan deviation

    Returns:
        (arw, bias_instability, bi_tau):
        - arw: Angle/Velocity Random Walk at τ=1s
        - bias_instability: Minimum of Allan deviation
        - bi_tau: τ at bias instability
    """
    if len(tau) == 0 or len(adev) == 0:
        return 0.0, 0.0, 0.0

    # ARW/VRW: value at τ=1s (or extrapolated)
    # On log-log plot, ARW region has slope -0.5
    idx_1s = np.argmin(np.abs(tau - 1.0))
    arw = adev[idx_1s]

    # Bias instability: minimum of Allan deviation
    bi_idx = np.argmin(adev)
    bias_instability = adev[bi_idx]
    bi_tau = tau[bi_idx]

    return arw, bias_instability, bi_tau


def analyze_allan(
    data: np.ndarray,
    dt: float,
    label: str = "",
    unit: str = "",
) -> AllanResult:
    """
    Complete Allan variance analysis for a single axis

    Args:
        data: 1D array of sensor measurements
        dt: Sample period in seconds
        label: Axis label (e.g., "Gyro X")
        unit: Unit string (e.g., "rad/s")

    Returns:
        AllanResult with all analysis data
    """
    tau, avar = compute_allan_variance(data, dt)

    if len(tau) == 0:
        return AllanResult(
            tau=tau, avar=avar, adev=np.array([]),
            unit=unit, label=label
        )

    adev = np.sqrt(avar)
    arw, bi, bi_tau = extract_noise_params(tau, adev)

    return AllanResult(
        tau=tau,
        avar=avar,
        adev=adev,
        arw=arw,
        bias_instability=bi,
        bi_tau=bi_tau,
        unit=unit,
        label=label,
    )


def estimate_sensor_noise(
    gyro: np.ndarray,
    accel: np.ndarray,
    sample_rate: float = 400.0,
    baro: Optional[np.ndarray] = None,
    tof: Optional[np.ndarray] = None,
    flow: Optional[np.ndarray] = None,
    run_allan: bool = True,
) -> NoiseEstimate:
    """
    Estimate sensor noise parameters from static data

    Args:
        gyro: Nx3 array of gyroscope data [rad/s]
        accel: Nx3 array of accelerometer data [m/s²]
        sample_rate: Sample rate in Hz
        baro: Optional barometer altitude data [m]
        tof: Optional ToF distance data [m]
        flow: Optional optical flow data [counts] (Nx2 or 1D)
        run_allan: If True, run Allan variance analysis

    Returns:
        NoiseEstimate with all noise parameters
    """
    dt = 1.0 / sample_rate
    n = len(gyro)

    result = NoiseEstimate(
        sample_rate_hz=sample_rate,
        samples=n,
        duration_s=n * dt,
    )

    # Gyroscope analysis
    result.gyro_std = np.std(gyro, axis=0)
    result.gyro_mean = np.mean(gyro, axis=0)

    if run_allan:
        for axis in range(3):
            ar = analyze_allan(
                gyro[:, axis], dt,
                label=f"Gyro {'XYZ'[axis]}",
                unit="rad/s"
            )
            result.gyro_allan.append(ar)
            result.gyro_arw[axis] = ar.arw
            result.gyro_bias_inst[axis] = ar.bias_instability
    else:
        # Fallback: use standard deviation
        result.gyro_arw = result.gyro_std
        result.gyro_bias_inst = result.gyro_std * 0.01

    # Accelerometer analysis
    result.accel_std = np.std(accel, axis=0)
    result.accel_mean = np.mean(accel, axis=0)

    if run_allan:
        for axis in range(3):
            ar = analyze_allan(
                accel[:, axis], dt,
                label=f"Accel {'XYZ'[axis]}",
                unit="m/s²"
            )
            result.accel_allan.append(ar)
            result.accel_vrw[axis] = ar.arw
            result.accel_bias_inst[axis] = ar.bias_instability
    else:
        result.accel_vrw = result.accel_std
        result.accel_bias_inst = result.accel_std * 0.01

    # Barometer
    if baro is not None and len(baro) > 0:
        result.baro_std = float(np.std(baro))
        result.baro_mean = float(np.mean(baro))

    # ToF (filter invalid readings)
    if tof is not None and len(tof) > 0:
        valid = tof[(tof > 0.01) & (tof < 4.0)]
        if len(valid) > 10:
            result.tof_std = float(np.std(valid))
            result.tof_mean = float(np.mean(valid))
        else:
            result.tof_std = 0.01  # Default
            result.tof_mean = 0.0

    # Optical flow
    if flow is not None and len(flow) > 0:
        if flow.ndim == 2:
            flow_var = (np.var(flow[:, 0]) + np.var(flow[:, 1])) / 2
        else:
            flow_var = np.var(flow)
        result.flow_std = float(np.sqrt(flow_var))
        result.flow_mean = float(np.mean(flow))

    return result


def detect_static_segments(
    accel: np.ndarray,
    gyro: np.ndarray,
    window_s: float = 0.5,
    sample_rate: float = 400.0,
    accel_threshold: float = 0.5,  # m/s²
    gyro_threshold: float = 0.1,   # rad/s
) -> List[Tuple[int, int]]:
    """
    Detect static (stationary) segments in sensor data

    Args:
        accel: Nx3 accelerometer data [m/s²]
        gyro: Nx3 gyroscope data [rad/s]
        window_s: Window size for variance computation [s]
        sample_rate: Sample rate [Hz]
        accel_threshold: Accel variance threshold for static detection
        gyro_threshold: Gyro variance threshold for static detection

    Returns:
        List of (start_idx, end_idx) tuples for static segments
    """
    window = int(window_s * sample_rate)
    n = len(accel)

    if n < window * 2:
        return [(0, n)]

    static_mask = np.zeros(n, dtype=bool)

    for i in range(0, n - window, window // 2):
        end = min(i + window, n)

        # Compute variance in window
        accel_var = np.var(accel[i:end], axis=0).mean()
        gyro_var = np.var(gyro[i:end], axis=0).mean()

        # Check if static
        if accel_var < accel_threshold ** 2 and gyro_var < gyro_threshold ** 2:
            static_mask[i:end] = True

    # Find continuous segments
    segments = []
    in_segment = False
    start_idx = 0

    for i, is_static in enumerate(static_mask):
        if is_static and not in_segment:
            start_idx = i
            in_segment = True
        elif not is_static and in_segment:
            if i - start_idx > window:  # Minimum segment length
                segments.append((start_idx, i))
            in_segment = False

    if in_segment:
        segments.append((start_idx, n))

    return segments


def load_and_estimate(
    filepath: str | Path,
    sample_rate: Optional[float] = None,
    sensor: str = "all",
    static_only: bool = False,
    min_duration: float = 10.0,
) -> NoiseEstimate:
    """
    Load CSV file and estimate sensor noise

    Args:
        filepath: Path to CSV file
        sample_rate: Sample rate (auto-detected if None)
        sensor: Which sensors to analyze ("gyro", "accel", "baro", "tof", "all")
        static_only: If True, only use static segments
        min_duration: Minimum data duration required [s]

    Returns:
        NoiseEstimate with analysis results

    Raises:
        ValueError: If file format is unsupported or data is too short
    """
    # Loader lives alongside this module (tools/sysid/loader.py)
    # ローダーはこのモジュールと同じ tools/sysid/loader.py にある
    from .loader import load_csv

    # Load data
    log_data = load_csv(filepath)

    # Extract arrays
    n = len(log_data.samples)
    gyro = np.array([s.gyro for s in log_data.samples])
    accel = np.array([s.accel for s in log_data.samples])

    # Determine sample rate
    if sample_rate is None:
        sample_rate = log_data.sample_rate_hz
        if sample_rate == 0:
            sample_rate = 400.0  # Default

    duration = n / sample_rate

    if duration < min_duration:
        raise ValueError(
            f"Data duration ({duration:.1f}s) is less than minimum ({min_duration}s)"
        )

    # Static segment detection
    if static_only:
        segments = detect_static_segments(accel, gyro, sample_rate=sample_rate)
        if segments:
            # Use longest static segment
            longest = max(segments, key=lambda x: x[1] - x[0])
            gyro = gyro[longest[0]:longest[1]]
            accel = accel[longest[0]:longest[1]]
            n = len(gyro)

    # Extract optional sensors
    baro = None
    tof = None
    flow = None

    if sensor in ["baro", "all"]:
        baro_list = [s.baro_altitude for s in log_data.samples if s.baro_altitude is not None]
        if baro_list:
            baro = np.array(baro_list[:n])

    if sensor in ["tof", "all"]:
        tof_list = [s.tof_distance for s in log_data.samples if s.tof_distance is not None]
        if tof_list:
            tof = np.array(tof_list[:n])

    if sensor in ["flow", "all"]:
        flow_list = [
            [s.flow_dx or 0, s.flow_dy or 0]
            for s in log_data.samples if s.flow_dx is not None
        ]
        if flow_list:
            flow = np.array(flow_list[:n])

    # Run estimation
    run_allan = sensor in ["gyro", "accel", "all"]

    return estimate_sensor_noise(
        gyro=gyro if sensor in ["gyro", "all"] else np.zeros((n, 3)),
        accel=accel if sensor in ["accel", "all"] else np.zeros((n, 3)),
        sample_rate=sample_rate,
        baro=baro,
        tof=tof,
        flow=flow,
        run_allan=run_allan,
    )
