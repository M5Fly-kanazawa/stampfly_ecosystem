#!/usr/bin/env python3
"""
estimate_qr.py - ESKF Q/R Parameter Estimation Tool

Estimates process noise (Q) and measurement noise (R) parameters
from static sensor data logs.

Usage:
    python estimate_qr.py --input static_calibration.bin --output eskf_params.json [--plot noise_analysis.png]
"""

import argparse
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime

from log_capture import parse_log_file


def load_static_data(filepath: str) -> dict:
    """Load binary log and extract sensor arrays"""
    packets = parse_log_file(filepath)

    if not packets:
        raise ValueError("No valid packets found in file")

    data = {
        'timestamp_ms': np.array([p.timestamp_ms for p in packets]),
        'accel': np.array([[p.accel_x, p.accel_y, p.accel_z] for p in packets]),
        'gyro': np.array([[p.gyro_x, p.gyro_y, p.gyro_z] for p in packets]),
        'mag': np.array([[p.mag_x, p.mag_y, p.mag_z] for p in packets]),
        'baro_alt': np.array([p.baro_alt for p in packets]),
        'pressure': np.array([p.pressure for p in packets]),
        'tof_bottom': np.array([p.tof_bottom for p in packets]),
        'tof_front': np.array([p.tof_front for p in packets]),
        'flow_dx': np.array([p.flow_dx for p in packets]),
        'flow_dy': np.array([p.flow_dy for p in packets]),
        'flow_squal': np.array([p.flow_squal for p in packets]),
    }

    return data


def compute_allan_variance(data: np.ndarray, dt: float, max_clusters: int = 1000) -> tuple:
    """
    Compute Allan variance for IMU noise characterization

    Args:
        data: 1D array of sensor measurements
        dt: Sample period in seconds
        max_clusters: Maximum number of cluster sizes to compute

    Returns:
        (tau, avar): Arrays of cluster times and Allan variances
    """
    n = len(data)
    taus = []
    avars = []

    # Cluster sizes from 1 to n/2
    max_m = min(n // 2, max_clusters)
    m_values = np.unique(np.logspace(0, np.log10(max_m), 100).astype(int))

    for m in m_values:
        if m == 0:
            continue

        tau = m * dt

        # Average clusters
        n_clusters = n // m
        if n_clusters < 2:
            continue

        truncated = data[:n_clusters * m]
        clusters = truncated.reshape(n_clusters, m).mean(axis=1)

        # Allan variance
        diff = np.diff(clusters)
        avar = 0.5 * np.mean(diff ** 2)

        taus.append(tau)
        avars.append(avar)

    return np.array(taus), np.array(avars)


def extract_arw_bias_instability(tau: np.ndarray, adev: np.ndarray) -> tuple:
    """
    Extract Angle Random Walk (ARW) and Bias Instability from Allan deviation

    Args:
        tau: Cluster times
        adev: Allan deviation (sqrt of variance)

    Returns:
        (arw, bias_instability): Estimated noise parameters
    """
    # ARW: slope = -0.5 region, value at tau=1
    # Find tau closest to 1 second
    idx_1s = np.argmin(np.abs(tau - 1.0))
    arw = adev[idx_1s]  # ARW in units/sqrt(Hz)

    # Bias instability: minimum of Allan deviation
    bias_instability = np.min(adev)

    return arw, bias_instability


def estimate_noise_parameters(data: dict, sample_rate: float = 100.0) -> dict:
    """
    Estimate Q and R parameters from static sensor data

    Args:
        data: Dictionary of sensor arrays from load_static_data
        sample_rate: Sample rate in Hz

    Returns:
        Dictionary of estimated parameters
    """
    dt = 1.0 / sample_rate

    # =========================================================================
    # Measurement Noise R (simple variance method)
    # =========================================================================

    # Accelerometer noise (variance of each axis)
    accel_var = np.var(data['accel'], axis=0)
    accel_std = np.sqrt(accel_var)

    # Gyroscope noise
    gyro_var = np.var(data['gyro'], axis=0)
    gyro_std = np.sqrt(gyro_var)

    # Magnetometer noise
    mag_var = np.var(data['mag'], axis=0)
    mag_std = np.sqrt(mag_var)

    # Barometer noise
    baro_var = np.var(data['baro_alt'])
    baro_std = np.sqrt(baro_var)

    # ToF noise (filter out invalid readings)
    tof_valid = data['tof_bottom'][(data['tof_bottom'] > 0.01) & (data['tof_bottom'] < 4.0)]
    if len(tof_valid) > 10:
        tof_var = np.var(tof_valid)
        tof_std = np.sqrt(tof_var)
    else:
        tof_var = 0.01  # Default
        tof_std = 0.1

    # Optical flow noise (when squal > 30)
    flow_valid_mask = data['flow_squal'] >= 30
    if np.sum(flow_valid_mask) > 10:
        flow_dx_valid = data['flow_dx'][flow_valid_mask]
        flow_dy_valid = data['flow_dy'][flow_valid_mask]
        flow_var = (np.var(flow_dx_valid) + np.var(flow_dy_valid)) / 2
        flow_std = np.sqrt(flow_var)
    else:
        flow_var = 1.0  # Default
        flow_std = 1.0

    # =========================================================================
    # Process Noise Q (Allan variance method for IMU)
    # =========================================================================

    # Gyro Allan variance analysis
    gyro_arw = []
    gyro_bias_inst = []
    for axis in range(3):
        tau, avar = compute_allan_variance(data['gyro'][:, axis], dt)
        if len(tau) > 0:
            adev = np.sqrt(avar)
            arw, bi = extract_arw_bias_instability(tau, adev)
            gyro_arw.append(arw)
            gyro_bias_inst.append(bi)
        else:
            gyro_arw.append(gyro_std[axis])
            gyro_bias_inst.append(gyro_std[axis] * 0.01)

    # Accel Allan variance analysis
    accel_vrw = []
    accel_bias_inst = []
    for axis in range(3):
        tau, avar = compute_allan_variance(data['accel'][:, axis], dt)
        if len(tau) > 0:
            adev = np.sqrt(avar)
            arw, bi = extract_arw_bias_instability(tau, adev)
            accel_vrw.append(arw)
            accel_bias_inst.append(bi)
        else:
            accel_vrw.append(accel_std[axis])
            accel_bias_inst.append(accel_std[axis] * 0.01)

    # =========================================================================
    # Compile results
    # =========================================================================

    # Average across axes for process noise
    gyro_noise = np.mean(gyro_arw)
    gyro_bias_noise = np.mean(gyro_bias_inst)
    accel_noise = np.mean(accel_vrw)
    accel_bias_noise = np.mean(accel_bias_inst)

    params = {
        "process_noise": {
            "gyro_noise": float(gyro_noise),
            "accel_noise": float(accel_noise),
            "gyro_bias_noise": float(gyro_bias_noise),
            "accel_bias_noise": float(accel_bias_noise)
        },
        "measurement_noise": {
            "accel_std": [float(x) for x in accel_std],
            "gyro_std": [float(x) for x in gyro_std],
            "mag_std": [float(x) for x in mag_std],
            "baro_noise": float(baro_std),
            "tof_noise": float(tof_std),
            "flow_noise": float(flow_std)
        },
        "statistics": {
            "accel_mean": [float(x) for x in np.mean(data['accel'], axis=0)],
            "gyro_mean": [float(x) for x in np.mean(data['gyro'], axis=0)],
            "gyro_bias_estimate": [float(x) for x in np.mean(data['gyro'], axis=0)],
            "accel_bias_estimate": [float(x) for x in (np.mean(data['accel'], axis=0) - [0, 0, 9.81])],
        },
        "analysis": {
            "sample_rate_hz": sample_rate,
            "samples": len(data['timestamp_ms']),
            "duration_sec": float((data['timestamp_ms'][-1] - data['timestamp_ms'][0]) / 1000.0),
            "timestamp": datetime.now().isoformat()
        }
    }

    return params


def plot_noise_analysis(data: dict, params: dict, output_path: Path = None):
    """Generate noise analysis plots"""
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    dt = 1.0 / params['analysis']['sample_rate_hz']

    # Gyro Allan deviation
    for axis, color, label in [(0, 'r', 'X'), (1, 'g', 'Y'), (2, 'b', 'Z')]:
        tau, avar = compute_allan_variance(data['gyro'][:, axis], dt)
        if len(tau) > 0:
            adev = np.sqrt(avar)
            axes[0, 0].loglog(tau, adev, color + '-', alpha=0.7, label=label)

    axes[0, 0].set_xlabel('Cluster Time τ [s]')
    axes[0, 0].set_ylabel('Allan Deviation [rad/s]')
    axes[0, 0].set_title('Gyroscope Allan Deviation')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3, which='both')

    # Accel Allan deviation
    for axis, color, label in [(0, 'r', 'X'), (1, 'g', 'Y'), (2, 'b', 'Z')]:
        tau, avar = compute_allan_variance(data['accel'][:, axis], dt)
        if len(tau) > 0:
            adev = np.sqrt(avar)
            axes[0, 1].loglog(tau, adev, color + '-', alpha=0.7, label=label)

    axes[0, 1].set_xlabel('Cluster Time τ [s]')
    axes[0, 1].set_ylabel('Allan Deviation [m/s²]')
    axes[0, 1].set_title('Accelerometer Allan Deviation')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3, which='both')

    # Gyro histogram
    for axis, color in [(0, 'r'), (1, 'g'), (2, 'b')]:
        axes[1, 0].hist(data['gyro'][:, axis], bins=50, alpha=0.5, color=color,
                        label=f"{'XYZ'[axis]} σ={params['measurement_noise']['gyro_std'][axis]:.5f}")
    axes[1, 0].set_xlabel('Gyro [rad/s]')
    axes[1, 0].set_ylabel('Count')
    axes[1, 0].set_title('Gyroscope Distribution')
    axes[1, 0].legend()

    # Accel histogram
    for axis, color in [(0, 'r'), (1, 'g'), (2, 'b')]:
        axes[1, 1].hist(data['accel'][:, axis], bins=50, alpha=0.5, color=color,
                        label=f"{'XYZ'[axis]} σ={params['measurement_noise']['accel_std'][axis]:.4f}")
    axes[1, 1].set_xlabel('Accel [m/s²]')
    axes[1, 1].set_ylabel('Count')
    axes[1, 1].set_title('Accelerometer Distribution')
    axes[1, 1].legend()

    # Baro / ToF histogram
    axes[2, 0].hist(data['baro_alt'], bins=50, alpha=0.7, color='blue',
                    label=f"Baro σ={params['measurement_noise']['baro_noise']:.4f}")
    tof_valid = data['tof_bottom'][(data['tof_bottom'] > 0.01) & (data['tof_bottom'] < 4.0)]
    if len(tof_valid) > 0:
        axes[2, 0].hist(tof_valid, bins=50, alpha=0.7, color='red',
                        label=f"ToF σ={params['measurement_noise']['tof_noise']:.4f}")
    axes[2, 0].set_xlabel('Altitude [m]')
    axes[2, 0].set_ylabel('Count')
    axes[2, 0].set_title('Altitude Sensors Distribution')
    axes[2, 0].legend()

    # Summary text
    summary = (
        f"Process Noise (Q):\n"
        f"  gyro_noise: {params['process_noise']['gyro_noise']:.6f} rad/s/√Hz\n"
        f"  accel_noise: {params['process_noise']['accel_noise']:.6f} m/s²/√Hz\n"
        f"  gyro_bias_noise: {params['process_noise']['gyro_bias_noise']:.8f}\n"
        f"  accel_bias_noise: {params['process_noise']['accel_bias_noise']:.8f}\n\n"
        f"Measurement Noise (R):\n"
        f"  baro: {params['measurement_noise']['baro_noise']:.4f} m\n"
        f"  tof: {params['measurement_noise']['tof_noise']:.4f} m\n"
        f"  flow: {params['measurement_noise']['flow_noise']:.2f}\n\n"
        f"Initial Bias Estimates:\n"
        f"  gyro: {params['statistics']['gyro_bias_estimate']}\n"
        f"  accel: {params['statistics']['accel_bias_estimate']}"
    )

    axes[2, 1].text(0.1, 0.9, summary, transform=axes[2, 1].transAxes,
                    fontsize=10, verticalalignment='top', fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    axes[2, 1].axis('off')
    axes[2, 1].set_title('Estimated Parameters')

    fig.suptitle(f"Noise Analysis - {params['analysis']['samples']} samples, "
                 f"{params['analysis']['duration_sec']:.1f}s")
    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150)
        print(f"Saved plot: {output_path}")

    return fig


def main():
    parser = argparse.ArgumentParser(description='ESKF Q/R Parameter Estimation Tool')
    parser.add_argument('--input', '-i', required=True, help='Static calibration log file (.bin)')
    parser.add_argument('--output', '-o', required=True, help='Output parameter file (.json)')
    parser.add_argument('--plot', '-p', help='Output plot file (.png)')
    parser.add_argument('--rate', '-r', type=float, default=100.0, help='Sample rate in Hz')
    parser.add_argument('--show', '-s', action='store_true', help='Show plots interactively')

    args = parser.parse_args()

    print(f"=== ESKF Q/R Parameter Estimation ===")
    print(f"Input: {args.input}")
    print(f"Sample rate: {args.rate} Hz")

    # Load data
    print("\nLoading static calibration data...")
    data = load_static_data(args.input)
    print(f"  Loaded {len(data['timestamp_ms'])} samples")
    print(f"  Duration: {(data['timestamp_ms'][-1] - data['timestamp_ms'][0]) / 1000:.2f} seconds")

    # Estimate parameters
    print("\nEstimating noise parameters...")
    params = estimate_noise_parameters(data, args.rate)

    # Print summary
    print("\n=== Estimated Parameters ===")
    print("\nProcess Noise (Q):")
    print(f"  gyro_noise:       {params['process_noise']['gyro_noise']:.6f} rad/s/√Hz")
    print(f"  accel_noise:      {params['process_noise']['accel_noise']:.6f} m/s²/√Hz")
    print(f"  gyro_bias_noise:  {params['process_noise']['gyro_bias_noise']:.8f}")
    print(f"  accel_bias_noise: {params['process_noise']['accel_bias_noise']:.8f}")

    print("\nMeasurement Noise (R):")
    print(f"  baro_noise: {params['measurement_noise']['baro_noise']:.4f} m")
    print(f"  tof_noise:  {params['measurement_noise']['tof_noise']:.4f} m")
    print(f"  mag_noise:  {params['measurement_noise']['mag_std']} uT")
    print(f"  flow_noise: {params['measurement_noise']['flow_noise']:.2f}")

    print("\nInitial Bias Estimates:")
    print(f"  gyro:  {params['statistics']['gyro_bias_estimate']} rad/s")
    print(f"  accel: {params['statistics']['accel_bias_estimate']} m/s²")

    # Save parameters
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w') as f:
        json.dump(params, f, indent=2)
    print(f"\nSaved parameters to: {output_path}")

    # Generate plot
    if args.plot:
        plot_path = Path(args.plot)
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        plot_noise_analysis(data, params, plot_path)

    if args.show:
        plot_noise_analysis(data, params)
        plt.show()


if __name__ == '__main__':
    main()
