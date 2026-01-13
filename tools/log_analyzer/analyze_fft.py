#!/usr/bin/env python3
"""
FFT Analysis Script for StampFly sensor data
センサーデータのFFT分析スクリプト

Analyzes CSV data to identify vibration frequencies for notch filter design.
ノッチフィルタ設計のための振動周波数を特定します。
"""

import numpy as np
import pandas as pd
from scipy import signal
from scipy.fft import rfft, rfftfreq
import sys
import os

def analyze_csv(filepath: str, top_n: int = 10):
    """Analyze CSV file and find dominant frequencies."""

    print(f"\n{'='*60}")
    print(f"Analyzing: {os.path.basename(filepath)}")
    print(f"{'='*60}")

    # Load data
    df = pd.read_csv(filepath)
    print(f"\nData shape: {df.shape}")
    print(f"Columns: {list(df.columns)}")

    # Remove duplicate timestamps (keep first)
    df_unique = df.drop_duplicates(subset=['timestamp_ms'], keep='first')
    print(f"Unique timestamps: {len(df_unique)} ({len(df) - len(df_unique)} duplicates removed)")

    # Calculate actual sample rate
    timestamps = df_unique['timestamp_ms'].values
    duration_ms = timestamps[-1] - timestamps[0]
    duration_s = duration_ms / 1000.0
    actual_rate = len(df_unique) / duration_s

    print(f"\nDuration: {duration_s:.2f} seconds")
    print(f"Actual sample rate: {actual_rate:.1f} Hz")
    print(f"Nyquist frequency: {actual_rate/2:.1f} Hz")

    # Data statistics
    print(f"\n--- Data Statistics ---")
    for col in ['gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z']:
        data = df_unique[col].values
        print(f"{col:10s}: mean={np.mean(data):10.4f}, std={np.std(data):10.4f}, "
              f"min={np.min(data):10.4f}, max={np.max(data):10.4f}")

    # Check if this is static data (low variance in gyro)
    gyro_std = np.std(df_unique[['gyro_x', 'gyro_y', 'gyro_z']].values)
    if gyro_std < 0.01:
        print(f"\n⚠️  WARNING: Low gyro variance ({gyro_std:.6f} rad/s)")
        print("    This appears to be STATIC data (drone not flying)")
        print("    For notch filter design, capture data during FLIGHT with motors running")

    # Resample to uniform rate for FFT (interpolation)
    # Use nominal rate based on actual data
    nominal_rate = round(actual_rate)
    t_original = (timestamps - timestamps[0]) / 1000.0  # seconds from start
    t_uniform = np.arange(0, duration_s, 1.0/nominal_rate)

    print(f"\n--- FFT Analysis (resampled to {nominal_rate} Hz) ---")

    results = {}

    channels = [
        ('gyro_x', 'Gyro X', 'rad/s'),
        ('gyro_y', 'Gyro Y', 'rad/s'),
        ('gyro_z', 'Gyro Z', 'rad/s'),
        ('accel_x', 'Accel X', 'm/s²'),
        ('accel_y', 'Accel Y', 'm/s²'),
        ('accel_z', 'Accel Z', 'm/s²'),
    ]

    for col, label, unit in channels:
        # Interpolate to uniform sampling
        data_original = df_unique[col].values
        data_uniform = np.interp(t_uniform, t_original, data_original)

        # Remove DC component (mean)
        data_centered = data_uniform - np.mean(data_uniform)

        # Apply window function
        window = signal.windows.hann(len(data_centered))
        data_windowed = data_centered * window

        # FFT
        n = len(data_windowed)
        yf = rfft(data_windowed)
        xf = rfftfreq(n, 1.0/nominal_rate)

        # Magnitude spectrum (normalized)
        magnitude = np.abs(yf) * 2.0 / n

        # Find peaks
        # Minimum distance between peaks: 5 Hz
        min_distance = int(5 * n / nominal_rate)
        peak_indices, peak_props = signal.find_peaks(
            magnitude,
            height=np.max(magnitude) * 0.01,  # 1% of max
            distance=max(1, min_distance)
        )

        # Sort by magnitude
        if len(peak_indices) > 0:
            sorted_idx = np.argsort(magnitude[peak_indices])[::-1]
            top_peaks = peak_indices[sorted_idx[:top_n]]

            results[col] = {
                'frequencies': xf[top_peaks],
                'magnitudes': magnitude[top_peaks],
                'unit': unit
            }
        else:
            results[col] = {
                'frequencies': np.array([]),
                'magnitudes': np.array([]),
                'unit': unit
            }

    # Print results
    print(f"\nTop {top_n} frequency peaks by channel:")
    print("-" * 60)

    for col, label, unit in channels:
        print(f"\n{label} [{unit}]:")
        r = results[col]
        if len(r['frequencies']) == 0:
            print("  No significant peaks found")
        else:
            for i, (freq, mag) in enumerate(zip(r['frequencies'], r['magnitudes'])):
                print(f"  {i+1:2d}. {freq:7.1f} Hz  (magnitude: {mag:.6f})")

    # Summary: Find common frequencies across axes
    print(f"\n{'='*60}")
    print("SUMMARY: Potential Notch Filter Frequencies")
    print(f"{'='*60}")

    # Collect all significant frequencies
    all_freqs = []
    for col in ['gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z']:
        r = results[col]
        for freq, mag in zip(r['frequencies'][:5], r['magnitudes'][:5]):
            if freq > 5:  # Ignore very low frequencies
                all_freqs.append(freq)

    if len(all_freqs) == 0:
        print("\nNo significant vibration frequencies detected.")
        print("This is expected for static data.")
        print("\n→ To identify motor vibration frequencies:")
        print("  1. Capture data during flight with motors running")
        print("  2. Look for peaks in the 50-200 Hz range (typical motor frequencies)")
    else:
        # Cluster nearby frequencies
        all_freqs = np.array(sorted(all_freqs))
        clusters = []
        cluster_start = all_freqs[0]
        cluster_freqs = [all_freqs[0]]

        for freq in all_freqs[1:]:
            if freq - cluster_freqs[-1] < 10:  # Within 10 Hz
                cluster_freqs.append(freq)
            else:
                clusters.append((np.mean(cluster_freqs), len(cluster_freqs)))
                cluster_freqs = [freq]
        clusters.append((np.mean(cluster_freqs), len(cluster_freqs)))

        # Sort by occurrence count
        clusters.sort(key=lambda x: -x[1])

        print("\nDetected vibration frequencies (sorted by occurrence):")
        for freq, count in clusters[:5]:
            print(f"  • {freq:.1f} Hz  (detected in {count} channel(s))")

        print("\nRecommended notch filter settings:")
        for i, (freq, count) in enumerate(clusters[:3]):
            if count >= 2:  # Present in at least 2 channels
                print(f"  Notch {i+1}: center={freq:.0f} Hz, bandwidth=10 Hz")

    return results


def main():
    # Find latest CSV file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_files = [f for f in os.listdir(script_dir) if f.startswith('stampfly_fft_') and f.endswith('.csv')]

    if not csv_files:
        print("No stampfly_fft_*.csv files found!")
        sys.exit(1)

    csv_files.sort()
    latest = csv_files[-1]
    filepath = os.path.join(script_dir, latest)

    print(f"Found {len(csv_files)} CSV file(s)")
    print(f"Analyzing latest: {latest}")

    analyze_csv(filepath)


if __name__ == '__main__':
    main()
