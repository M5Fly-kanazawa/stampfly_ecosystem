#!/usr/bin/env python3
"""
ACRO Flight Analysis - Hover Stability
ACROモード飛行解析 - ホバー安定性

Analyzes gyro response, controller inputs, and their correlation
during ACRO mode hovering attempt.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
import sys

def analyze_flight(csv_path):
    # Load data
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} samples from {csv_path}")
    print(f"Columns: {list(df.columns)}")

    # Calculate time from timestamp
    df['time_s'] = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    duration = df['time_s'].max()
    sample_rate = len(df) / duration
    print(f"Duration: {duration:.1f}s, Sample rate: {sample_rate:.1f}Hz")

    # Convert gyro to deg/s for readability
    RAD_TO_DEG = 180.0 / np.pi
    df['gyro_x_deg'] = df['gyro_x'] * RAD_TO_DEG
    df['gyro_y_deg'] = df['gyro_y'] * RAD_TO_DEG
    df['gyro_z_deg'] = df['gyro_z'] * RAD_TO_DEG
    df['gyro_corrected_x_deg'] = df['gyro_corrected_x'] * RAD_TO_DEG
    df['gyro_corrected_y_deg'] = df['gyro_corrected_y'] * RAD_TO_DEG
    df['gyro_corrected_z_deg'] = df['gyro_corrected_z'] * RAD_TO_DEG

    # === Statistics ===
    print("\n" + "="*60)
    print("=== Gyro Statistics (deg/s) ===")
    print("="*60)

    for axis, name in [('x', 'Roll'), ('y', 'Pitch'), ('z', 'Yaw')]:
        raw = df[f'gyro_{axis}_deg']
        corrected = df[f'gyro_corrected_{axis}_deg']
        print(f"\n{name}:")
        print(f"  Raw:       mean={raw.mean():+.3f}, std={raw.std():.3f}, range=[{raw.min():.2f}, {raw.max():.2f}]")
        print(f"  Corrected: mean={corrected.mean():+.3f}, std={corrected.std():.3f}, range=[{corrected.min():.2f}, {corrected.max():.2f}]")

    # === Controller Input Statistics ===
    print("\n" + "="*60)
    print("=== Controller Input Statistics ===")
    print("="*60)

    for col, name in [('ctrl_throttle', 'Throttle'), ('ctrl_roll', 'Roll'),
                      ('ctrl_pitch', 'Pitch'), ('ctrl_yaw', 'Yaw')]:
        data = df[col]
        print(f"{name:10s}: mean={data.mean():+.4f}, std={data.std():.4f}, range=[{data.min():.3f}, {data.max():.3f}]")

    # === Oscillation Analysis ===
    print("\n" + "="*60)
    print("=== Oscillation Analysis ===")
    print("="*60)

    # Find dominant oscillation frequency using FFT
    for axis, name in [('x', 'Roll'), ('y', 'Pitch'), ('z', 'Yaw')]:
        gyro_data = df[f'gyro_corrected_{axis}_deg'].values

        # Remove DC component
        gyro_ac = gyro_data - np.mean(gyro_data)

        # FFT
        n = len(gyro_ac)
        freqs = np.fft.fftfreq(n, d=1/sample_rate)
        fft_vals = np.abs(np.fft.fft(gyro_ac))

        # Find peaks in positive frequencies (0.5-50Hz range for oscillations)
        pos_mask = (freqs > 0.5) & (freqs < 50)
        pos_freqs = freqs[pos_mask]
        pos_fft = fft_vals[pos_mask]

        # Find top 3 peaks
        peak_indices = np.argsort(pos_fft)[-3:][::-1]
        print(f"\n{name} dominant frequencies:")
        for i, idx in enumerate(peak_indices):
            print(f"  {i+1}. {pos_freqs[idx]:.2f} Hz (amplitude: {pos_fft[idx]:.1f})")

    # === Cross-correlation: Input vs Gyro Response ===
    print("\n" + "="*60)
    print("=== Input-Response Correlation ===")
    print("="*60)

    # Downsample for correlation analysis (100Hz)
    downsample = int(sample_rate / 100)
    df_ds = df.iloc[::downsample].copy()

    for axis, ctrl_name in [('x', 'ctrl_roll'), ('y', 'ctrl_pitch'), ('z', 'ctrl_yaw')]:
        ctrl = df_ds[ctrl_name].values
        gyro = df_ds[f'gyro_corrected_{axis}_deg'].values

        # Only analyze if there's controller input
        if np.std(ctrl) > 0.01:
            correlation = np.corrcoef(ctrl, gyro)[0, 1]
            print(f"{ctrl_name:12s} vs gyro_{axis}: correlation = {correlation:+.3f}")

            # Calculate lag using cross-correlation
            corr = np.correlate(gyro - np.mean(gyro), ctrl - np.mean(ctrl), mode='full')
            lags = np.arange(-len(ctrl)+1, len(ctrl))
            lag_ms = lags / 100 * 1000  # Convert to ms

            # Find peak lag (within ±500ms)
            valid_mask = np.abs(lag_ms) < 500
            valid_corr = corr[valid_mask]
            valid_lags = lag_ms[valid_mask]
            peak_idx = np.argmax(np.abs(valid_corr))
            peak_lag = valid_lags[peak_idx]
            print(f"              Peak correlation at lag: {peak_lag:.0f} ms")
        else:
            print(f"{ctrl_name:12s}: No significant input detected")

    # === Create Visualization ===
    fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)

    # 1. Gyro Roll/Pitch (corrected)
    ax1 = axes[0]
    ax1.plot(df['time_s'], df['gyro_corrected_x_deg'], label='Roll', alpha=0.8, linewidth=0.5)
    ax1.plot(df['time_s'], df['gyro_corrected_y_deg'], label='Pitch', alpha=0.8, linewidth=0.5)
    ax1.set_ylabel('Angular Rate [deg/s]')
    ax1.set_title('Gyro Roll/Pitch (Bias Corrected)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax1.set_ylim([-60, 60])

    # 2. Gyro Yaw (corrected)
    ax2 = axes[1]
    ax2.plot(df['time_s'], df['gyro_corrected_z_deg'], label='Yaw', color='green', alpha=0.8, linewidth=0.5)
    ax2.set_ylabel('Angular Rate [deg/s]')
    ax2.set_title('Gyro Yaw (Bias Corrected)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax2.set_ylim([-60, 60])

    # 3. Controller Roll/Pitch
    ax3 = axes[2]
    ax3.plot(df['time_s'], df['ctrl_roll'], label='Roll Stick', alpha=0.8)
    ax3.plot(df['time_s'], df['ctrl_pitch'], label='Pitch Stick', alpha=0.8)
    ax3.set_ylabel('Stick Input [-1, 1]')
    ax3.set_title('Controller Roll/Pitch Input')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax3.set_ylim([-1.1, 1.1])

    # 4. Controller Throttle/Yaw
    ax4 = axes[3]
    ax4.plot(df['time_s'], df['ctrl_throttle'], label='Throttle', alpha=0.8)
    ax4.plot(df['time_s'], df['ctrl_yaw'], label='Yaw Stick', alpha=0.8)
    ax4.set_ylabel('Stick Input')
    ax4.set_title('Controller Throttle/Yaw Input')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

    # 5. Accelerometer Z (for thrust reference)
    ax5 = axes[4]
    ax5.plot(df['time_s'], df['accel_z'], label='Accel Z', color='purple', alpha=0.8, linewidth=0.5)
    ax5.set_ylabel('Acceleration [m/s²]')
    ax5.set_xlabel('Time [s]')
    ax5.set_title('Accelerometer Z (Vertical)')
    ax5.legend(loc='upper right')
    ax5.grid(True, alpha=0.3)
    ax5.axhline(y=-9.81, color='k', linestyle='--', linewidth=0.5, label='Gravity')

    plt.tight_layout()

    # Save figure
    output_path = csv_path.replace('.csv', '_flight_analysis.png')
    plt.savefig(output_path, dpi=150)
    print(f"\nSaved: {output_path}")

    # === Detailed time-segment analysis ===
    print("\n" + "="*60)
    print("=== Time Segment Analysis (5s windows) ===")
    print("="*60)

    window_size = 5.0  # seconds
    for start in np.arange(0, duration - window_size, window_size):
        end = start + window_size
        mask = (df['time_s'] >= start) & (df['time_s'] < end)
        segment = df[mask]

        roll_std = segment['gyro_corrected_x_deg'].std()
        pitch_std = segment['gyro_corrected_y_deg'].std()
        yaw_std = segment['gyro_corrected_z_deg'].std()
        throttle_mean = segment['ctrl_throttle'].mean()

        stability = "STABLE" if roll_std < 5 and pitch_std < 5 else "UNSTABLE"
        print(f"{start:5.1f}-{end:5.1f}s: Roll σ={roll_std:5.2f}°/s, Pitch σ={pitch_std:5.2f}°/s, Yaw σ={yaw_std:5.2f}°/s, Throttle={throttle_mean:.2f} [{stability}]")

    # === PID Tuning Recommendations ===
    print("\n" + "="*60)
    print("=== PID Tuning Observations ===")
    print("="*60)

    roll_std = df['gyro_corrected_x_deg'].std()
    pitch_std = df['gyro_corrected_y_deg'].std()
    yaw_std = df['gyro_corrected_z_deg'].std()

    # Check for oscillation signs
    if roll_std > 15 or pitch_std > 15:
        print("⚠ High gyro variance detected - possible oscillation")
        print("  Recommendations:")
        print("  - Reduce P gain if oscillations are fast (>5Hz)")
        print("  - Reduce D gain if oscillations are slow (<2Hz) with overshoot")
        print("  - Check for mechanical issues (loose props, motor vibration)")

    # Check controller activity
    roll_input_std = df['ctrl_roll'].std()
    pitch_input_std = df['ctrl_pitch'].std()

    if roll_input_std > 0.1 or pitch_input_std > 0.1:
        print(f"\n⚠ High pilot correction activity detected")
        print(f"  Roll input σ={roll_input_std:.3f}, Pitch input σ={pitch_input_std:.3f}")
        print("  This indicates the drone is not self-stabilizing well")

    # Check for I-term windup signs
    roll_mean = df['gyro_corrected_x_deg'].mean()
    pitch_mean = df['gyro_corrected_y_deg'].mean()
    if abs(roll_mean) > 2 or abs(pitch_mean) > 2:
        print(f"\n⚠ Non-zero gyro mean during flight")
        print(f"  Roll mean={roll_mean:.2f}°/s, Pitch mean={pitch_mean:.2f}°/s")
        print("  Possible I-term accumulation or persistent external force")

    plt.show()
    return df

if __name__ == "__main__":
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        csv_path = "/Users/kouhei/Library/CloudStorage/Dropbox/01教育研究/20マルチコプタ/stampfly_ecosystem/tools/log_analyzer/stampfly_fft_20260114T121831.csv"

    analyze_flight(csv_path)
