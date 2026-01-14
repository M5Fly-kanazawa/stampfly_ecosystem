#!/usr/bin/env python3
"""
IMU Time Series Visualization
IMU時系列可視化スクリプト
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_timeseries(csv_path):
    # Load data
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} samples from {csv_path}")
    print(f"Columns: {list(df.columns)}")

    # Calculate time from timestamp
    if 'timestamp_ms' in df.columns:
        df['time_s'] = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    else:
        df['time_s'] = np.arange(len(df)) / 400.0  # Assume 400Hz

    # Create figure with subplots
    fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

    # Plot gyroscope data
    ax1 = axes[0]
    if 'gyro_x' in df.columns:
        ax1.plot(df['time_s'], df['gyro_x'], label='Gyro X (Roll)', alpha=0.8)
        ax1.plot(df['time_s'], df['gyro_y'], label='Gyro Y (Pitch)', alpha=0.8)
        ax1.plot(df['time_s'], df['gyro_z'], label='Gyro Z (Yaw)', alpha=0.8)
    ax1.set_ylabel('Angular Rate [rad/s]')
    ax1.set_title('Gyroscope Time Series')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

    # Plot accelerometer data
    ax2 = axes[1]
    if 'accel_x' in df.columns:
        ax2.plot(df['time_s'], df['accel_x'], label='Accel X', alpha=0.8)
        ax2.plot(df['time_s'], df['accel_y'], label='Accel Y', alpha=0.8)
        ax2.plot(df['time_s'], df['accel_z'], label='Accel Z', alpha=0.8)
    ax2.set_ylabel('Acceleration [m/s²]')
    ax2.set_xlabel('Time [s]')
    ax2.set_title('Accelerometer Time Series')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=-9.81, color='k', linestyle='--', linewidth=0.5, label='Gravity')

    plt.tight_layout()

    # Save figure
    output_path = csv_path.replace('.csv', '_timeseries.png')
    plt.savefig(output_path, dpi=150)
    print(f"Saved: {output_path}")

    # Print statistics
    print("\n=== Statistics ===")
    if 'gyro_x' in df.columns:
        print(f"Gyro X: mean={df['gyro_x'].mean():.4f}, std={df['gyro_x'].std():.4f}, range=[{df['gyro_x'].min():.3f}, {df['gyro_x'].max():.3f}]")
        print(f"Gyro Y: mean={df['gyro_y'].mean():.4f}, std={df['gyro_y'].std():.4f}, range=[{df['gyro_y'].min():.3f}, {df['gyro_y'].max():.3f}]")
        print(f"Gyro Z: mean={df['gyro_z'].mean():.4f}, std={df['gyro_z'].std():.4f}, range=[{df['gyro_z'].min():.3f}, {df['gyro_z'].max():.3f}]")
    if 'accel_x' in df.columns:
        print(f"Accel X: mean={df['accel_x'].mean():.4f}, std={df['accel_x'].std():.4f}")
        print(f"Accel Y: mean={df['accel_y'].mean():.4f}, std={df['accel_y'].std():.4f}")
        print(f"Accel Z: mean={df['accel_z'].mean():.4f}, std={df['accel_z'].std():.4f}")

    # Check for drift indicators
    print("\n=== Drift Analysis ===")
    if 'gyro_x' in df.columns:
        # Check if mean is significantly non-zero (potential bias/drift source)
        gyro_means = [df['gyro_x'].mean(), df['gyro_y'].mean(), df['gyro_z'].mean()]
        for i, axis in enumerate(['X', 'Y', 'Z']):
            if abs(gyro_means[i]) > 0.01:  # 0.01 rad/s threshold
                print(f"  WARNING: Gyro {axis} has non-zero mean: {gyro_means[i]:.4f} rad/s")
                print(f"           This could cause I-term accumulation!")

    plt.show()
    return df

if __name__ == "__main__":
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        csv_path = "/Users/kouhei/Library/CloudStorage/Dropbox/01教育研究/20マルチコプタ/stampfly_ecosystem/tools/log_analyzer/stampfly_fft_20260113T231806.csv"

    plot_timeseries(csv_path)
