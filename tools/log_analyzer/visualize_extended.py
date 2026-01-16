#!/usr/bin/env python3
"""
visualize_extended.py - Extended Telemetry Visualization Tool

Visualizes all state data from 400Hz extended telemetry logs captured by wifi_capture.py.
Displays IMU, ESKF estimates, sensors, and controller inputs in a comprehensive dashboard.

Usage:
    python visualize_extended.py <csv_file> [options]

Options:
    --save PNG          Save plot to PNG file instead of displaying
    --time-range START END  Plot only specified time range (seconds)
    --no-eskf           Hide ESKF panels
    --no-sensors        Hide sensor panels

Examples:
    python visualize_extended.py stampfly_fft_20260115T120000.csv
    python visualize_extended.py flight.csv --save flight_analysis.png
    python visualize_extended.py flight.csv --time-range 5 15
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
except ImportError:
    print("Error: matplotlib required")
    print("Install with: pip install matplotlib")
    sys.exit(1)


def quat_to_euler(qw, qx, qy, qz):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def load_csv(filename):
    """Load CSV file and detect format"""
    data = {}
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        columns = reader.fieldnames

        # Detect format from columns
        if 'timestamp_us' in columns and 'quat_w' in columns:
            fmt = 'extended'
        elif 'timestamp_ms' in columns and 'gyro_corrected_x' in columns:
            fmt = 'fft_batch'
        elif 'timestamp_ms' in columns and 'roll_deg' in columns:
            fmt = 'normal'
        else:
            fmt = 'unknown'

        # Initialize arrays
        for col in columns:
            data[col] = []

        # Read data
        for row in reader:
            for col in columns:
                try:
                    data[col].append(float(row[col]))
                except (ValueError, KeyError):
                    data[col].append(0.0)

    # Convert to numpy arrays
    for col in data:
        data[col] = np.array(data[col])

    return data, fmt


def plot_extended(data, output_file=None, time_range=None, show_eskf=True, show_sensors=True):
    """Create comprehensive visualization for extended telemetry data"""

    # Time axis
    t_us = data['timestamp_us']
    t = (t_us - t_us[0]) / 1e6  # seconds from start

    # Apply time range filter
    if time_range:
        mask = (t >= time_range[0]) & (t <= time_range[1])
        t = t[mask]
        for key in data:
            data[key] = data[key][mask]

    n_samples = len(t)
    duration = t[-1] - t[0]
    sample_rate = n_samples / duration if duration > 0 else 0

    print(f"Samples: {n_samples}")
    print(f"Duration: {duration:.2f}s")
    print(f"Sample rate: {sample_rate:.1f} Hz")

    # Determine layout - all full-width rows
    n_rows = 4  # Gyro, Accel, Gyro Corrected, Control
    if show_eskf:
        n_rows += 4  # Euler, Position, Velocity, Biases
    if show_sensors:
        n_rows += 2  # Height, Flow

    # Create figure with shared x-axis
    row_height = 1.8
    fig, axes = plt.subplots(n_rows, 1, figsize=(14, row_height * n_rows),
                              sharex=True, squeeze=False)
    axes = axes.flatten()

    # Common legend style
    legend_style = dict(
        fontsize=8,
        framealpha=0.9,
        edgecolor='gray',
        fancybox=True,
        handlelength=1.5,
        loc='upper left',
        ncol=6,
    )

    ax_idx = 0

    # === Raw Gyro ===
    ax = axes[ax_idx]
    ax.plot(t, np.rad2deg(data['gyro_x']), 'r-', label='X', linewidth=0.5, alpha=0.8)
    ax.plot(t, np.rad2deg(data['gyro_y']), 'g-', label='Y', linewidth=0.5, alpha=0.8)
    ax.plot(t, np.rad2deg(data['gyro_z']), 'b-', label='Z', linewidth=0.5, alpha=0.8)
    ax.set_ylabel('Gyro Raw\n[deg/s]', fontsize=8)
    ax.legend(**legend_style)
    ax.grid(True, alpha=0.3)
    ax_idx += 1

    # === Raw Accel ===
    ax = axes[ax_idx]
    ax.plot(t, data['accel_x'], 'r-', label='X', linewidth=0.5, alpha=0.8)
    ax.plot(t, data['accel_y'], 'g-', label='Y', linewidth=0.5, alpha=0.8)
    ax.plot(t, data['accel_z'], 'b-', label='Z', linewidth=0.5, alpha=0.8)
    ax.set_ylabel('Accel Raw\n[m/s²]', fontsize=8)
    ax.legend(**legend_style)
    ax.grid(True, alpha=0.3)
    ax_idx += 1

    # === Bias-corrected Gyro ===
    ax = axes[ax_idx]
    ax.plot(t, np.rad2deg(data['gyro_corrected_x']), 'r-', label='X', linewidth=0.5, alpha=0.8)
    ax.plot(t, np.rad2deg(data['gyro_corrected_y']), 'g-', label='Y', linewidth=0.5, alpha=0.8)
    ax.plot(t, np.rad2deg(data['gyro_corrected_z']), 'b-', label='Z', linewidth=0.5, alpha=0.8)
    ax.set_ylabel('Gyro Corr\n[deg/s]', fontsize=8)
    ax.legend(**legend_style)
    ax.grid(True, alpha=0.3)
    ax_idx += 1

    # === Controller Inputs ===
    ax = axes[ax_idx]
    ax.plot(t, data['ctrl_throttle'], 'k-', label='Thr', linewidth=0.8)
    ax.plot(t, data['ctrl_roll'], 'r-', label='Roll', linewidth=0.8, alpha=0.8)
    ax.plot(t, data['ctrl_pitch'], 'g-', label='Pitch', linewidth=0.8, alpha=0.8)
    ax.plot(t, data['ctrl_yaw'], 'b-', label='Yaw', linewidth=0.8, alpha=0.8)
    ax.set_ylabel('Control\n[-1,1]', fontsize=8)
    ax.legend(**legend_style)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-1.1, 1.1)
    ax_idx += 1

    # === ESKF Panels ===
    if show_eskf:
        # Convert quaternion to Euler angles
        roll, pitch, yaw = quat_to_euler(
            data['quat_w'], data['quat_x'], data['quat_y'], data['quat_z']
        )

        # Euler angles
        ax = axes[ax_idx]
        ax.plot(t, np.rad2deg(roll), 'r-', label='Roll', linewidth=0.8)
        ax.plot(t, np.rad2deg(pitch), 'g-', label='Pitch', linewidth=0.8)
        ax.plot(t, np.rad2deg(yaw), 'b-', label='Yaw', linewidth=0.8)
        ax.set_ylabel('Attitude\n[deg]', fontsize=8)
        ax.legend(**legend_style)
        ax.grid(True, alpha=0.3)
        ax_idx += 1

        # Position
        ax = axes[ax_idx]
        ax.plot(t, data['pos_x'], 'r-', label='X', linewidth=0.8)
        ax.plot(t, data['pos_y'], 'g-', label='Y', linewidth=0.8)
        ax.plot(t, data['pos_z'], 'b-', label='Z', linewidth=0.8)
        ax.set_ylabel('Position\n[m]', fontsize=8)
        ax.legend(**legend_style)
        ax.grid(True, alpha=0.3)
        ax_idx += 1

        # Velocity
        ax = axes[ax_idx]
        ax.plot(t, data['vel_x'], 'r-', label='Vx', linewidth=0.8)
        ax.plot(t, data['vel_y'], 'g-', label='Vy', linewidth=0.8)
        ax.plot(t, data['vel_z'], 'b-', label='Vz', linewidth=0.8)
        ax.set_ylabel('Velocity\n[m/s]', fontsize=8)
        ax.legend(**legend_style)
        ax.grid(True, alpha=0.3)
        ax_idx += 1

        # Biases (combined gyro and accel)
        ax = axes[ax_idx]
        ax.plot(t, np.rad2deg(data['gyro_bias_x']), 'r-', label='Gyro X', linewidth=0.8)
        ax.plot(t, np.rad2deg(data['gyro_bias_y']), 'g-', label='Gyro Y', linewidth=0.8)
        ax.plot(t, np.rad2deg(data['gyro_bias_z']), 'b-', label='Gyro Z', linewidth=0.8)
        ax.plot(t, data['accel_bias_x'] * 10, 'r--', label='Acc X×10', linewidth=0.8, alpha=0.6)
        ax.plot(t, data['accel_bias_y'] * 10, 'g--', label='Acc Y×10', linewidth=0.8, alpha=0.6)
        ax.plot(t, data['accel_bias_z'] * 10, 'b--', label='Acc Z×10', linewidth=0.8, alpha=0.6)
        ax.set_ylabel('Bias\n[deg/s]', fontsize=8)
        ax.legend(**legend_style)
        ax.grid(True, alpha=0.3)
        ax_idx += 1

    # === Sensor Panels ===
    if show_sensors:
        # Height sensors
        ax = axes[ax_idx]
        ax.plot(t, data['baro_altitude'], 'purple', label='Baro', linewidth=0.8)
        ax.plot(t, data['tof_bottom'], 'orange', label='ToF Bot', linewidth=0.8)
        ax.plot(t, data['tof_front'], 'cyan', label='ToF Fwd', linewidth=0.8)
        ax.set_ylabel('Height\n[m]', fontsize=8)
        ax.legend(**legend_style)
        ax.grid(True, alpha=0.3)
        ax_idx += 1

        # Optical flow (combined with quality)
        ax = axes[ax_idx]
        ax.plot(t, data['flow_x'], 'r-', label='Flow X', linewidth=0.5, alpha=0.8)
        ax.plot(t, data['flow_y'], 'g-', label='Flow Y', linewidth=0.5, alpha=0.8)
        ax2 = ax.twinx()
        ax2.plot(t, data['flow_quality'], 'k-', label='SQUAL', linewidth=0.5, alpha=0.5)
        ax2.set_ylabel('SQUAL', fontsize=8)
        ax2.set_ylim(0, 255)
        ax.set_ylabel('Flow\n[counts]', fontsize=8)
        ax.legend(**legend_style)
        ax.grid(True, alpha=0.3)
        ax_idx += 1

    # Set x-axis label only on bottom plot
    axes[-1].set_xlabel('Time [s]', fontsize=9)

    # Main title
    fig.suptitle(f'Extended Telemetry ({n_samples} samples @ {sample_rate:.0f}Hz, {duration:.1f}s)',
                 fontsize=11, fontweight='bold')

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved plot to: {output_file}")
    else:
        plt.show()


def plot_legacy(data, fmt, output_file=None):
    """Simple plot for legacy formats (fft_batch, normal)"""

    if fmt == 'fft_batch':
        t_ms = data['timestamp_ms']
        t = (t_ms - t_ms[0]) / 1000.0
        title = 'FFT Batch Telemetry'
    else:
        t_ms = data['timestamp_ms']
        t = (t_ms - t_ms[0]) / 1000.0
        title = 'Normal Telemetry'

    n_samples = len(t)
    duration = t[-1] - t[0]
    sample_rate = n_samples / duration if duration > 0 else 0

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    # Gyro
    axes[0].plot(t, np.rad2deg(data['gyro_x']), 'r-', label='X', linewidth=0.5)
    axes[0].plot(t, np.rad2deg(data['gyro_y']), 'g-', label='Y', linewidth=0.5)
    axes[0].plot(t, np.rad2deg(data['gyro_z']), 'b-', label='Z', linewidth=0.5)
    axes[0].set_ylabel('Gyro [deg/s]')
    axes[0].legend(loc='upper right', ncol=3)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Gyroscope')

    # Accel
    axes[1].plot(t, data['accel_x'], 'r-', label='X', linewidth=0.5)
    axes[1].plot(t, data['accel_y'], 'g-', label='Y', linewidth=0.5)
    axes[1].plot(t, data['accel_z'], 'b-', label='Z', linewidth=0.5)
    axes[1].set_ylabel('Accel [m/s²]')
    axes[1].legend(loc='upper right', ncol=3)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('Accelerometer')

    # Controller or other data
    if 'ctrl_throttle' in data:
        axes[2].plot(t, data['ctrl_throttle'], 'k-', label='Throttle', linewidth=0.8)
        axes[2].plot(t, data['ctrl_roll'], 'r-', label='Roll', linewidth=0.8)
        axes[2].plot(t, data['ctrl_pitch'], 'g-', label='Pitch', linewidth=0.8)
        axes[2].plot(t, data['ctrl_yaw'], 'b-', label='Yaw', linewidth=0.8)
        axes[2].set_ylabel('Control [-1,1]')
        axes[2].legend(loc='upper right', ncol=4)
        axes[2].set_title('Controller Inputs')
    else:
        axes[2].plot(t, data.get('roll_deg', np.zeros_like(t)), 'r-', label='Roll')
        axes[2].plot(t, data.get('pitch_deg', np.zeros_like(t)), 'g-', label='Pitch')
        axes[2].plot(t, data.get('yaw_deg', np.zeros_like(t)), 'b-', label='Yaw')
        axes[2].set_ylabel('Angle [deg]')
        axes[2].legend(loc='upper right', ncol=3)
        axes[2].set_title('Attitude')

    axes[2].set_xlabel('Time [s]')
    axes[2].grid(True, alpha=0.3)

    fig.suptitle(f'{title} ({n_samples} samples @ {sample_rate:.0f}Hz)', fontsize=12, fontweight='bold')
    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved plot to: {output_file}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize extended telemetry data from StampFly',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('csv_file', help='Input CSV file from wifi_capture.py')
    parser.add_argument('--save', metavar='PNG', help='Save plot to PNG file')
    parser.add_argument('--time-range', nargs=2, type=float, metavar=('START', 'END'),
                        help='Time range to plot (seconds)')
    parser.add_argument('--no-eskf', action='store_true', help='Hide ESKF panels')
    parser.add_argument('--no-sensors', action='store_true', help='Hide sensor panels')

    args = parser.parse_args()

    # Check file exists
    if not Path(args.csv_file).exists():
        print(f"Error: File not found: {args.csv_file}")
        return 1

    # Load data
    print(f"Loading: {args.csv_file}")
    data, fmt = load_csv(args.csv_file)

    print(f"Detected format: {fmt}")

    if fmt == 'extended':
        plot_extended(
            data,
            output_file=args.save,
            time_range=args.time_range,
            show_eskf=not args.no_eskf,
            show_sensors=not args.no_sensors
        )
    elif fmt in ('fft_batch', 'normal'):
        plot_legacy(data, fmt, output_file=args.save)
    else:
        print(f"Error: Unknown CSV format")
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
