#!/usr/bin/env python3
"""
Detailed FFT Analysis for Notch Filter Design
ノッチフィルタ設計のための詳細FFT分析
"""

import numpy as np
import pandas as pd
from scipy import signal
from scipy.fft import rfft, rfftfreq
import sys
import os

def analyze_for_notch_filter(filepath: str):
    """Detailed analysis for notch filter frequency identification."""

    print(f"\n{'='*70}")
    print(f"NOTCH FILTER FREQUENCY ANALYSIS")
    print(f"{'='*70}")
    print(f"File: {os.path.basename(filepath)}")

    # Load and preprocess data
    df = pd.read_csv(filepath)

    # Check for FFT format (minimal columns)
    expected_cols = ['timestamp_ms', 'gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z']
    if not all(col in df.columns for col in expected_cols):
        print(f"ERROR: Expected columns {expected_cols}")
        print(f"Found: {list(df.columns)}")
        return

    # Remove duplicates
    df_unique = df.drop_duplicates(subset=['timestamp_ms'], keep='first')

    timestamps = df_unique['timestamp_ms'].values
    duration_s = (timestamps[-1] - timestamps[0]) / 1000.0
    sample_rate = len(df_unique) / duration_s

    print(f"\nData: {len(df_unique)} samples over {duration_s:.1f}s")
    print(f"Effective sample rate: {sample_rate:.1f} Hz")
    print(f"Nyquist frequency: {sample_rate/2:.1f} Hz")

    # Resample to uniform rate
    nominal_rate = round(sample_rate)
    t_original = (timestamps - timestamps[0]) / 1000.0
    t_uniform = np.arange(0, duration_s, 1.0/nominal_rate)

    print(f"\n{'='*70}")
    print("FREQUENCY SPECTRUM ANALYSIS")
    print(f"{'='*70}")

    # Analyze each channel
    channels = {
        'gyro_x': ('Gyro X', 'rad/s'),
        'gyro_y': ('Gyro Y', 'rad/s'),
        'gyro_z': ('Gyro Z', 'rad/s'),
        'accel_x': ('Accel X', 'm/s²'),
        'accel_y': ('Accel Y', 'm/s²'),
        'accel_z': ('Accel Z', 'm/s²'),
    }

    # Frequency bins for accumulating results
    freq_bins = {}  # freq -> list of (channel, magnitude)

    for col, (label, unit) in channels.items():
        # Interpolate to uniform sampling
        data = np.interp(t_uniform, t_original, df_unique[col].values)

        # Remove DC and apply window
        data_centered = data - np.mean(data)
        window = signal.windows.hann(len(data_centered))
        data_windowed = data_centered * window

        # FFT
        n = len(data_windowed)
        yf = rfft(data_windowed)
        xf = rfftfreq(n, 1.0/nominal_rate)

        # Power spectrum (more intuitive than magnitude)
        power = np.abs(yf)**2 / n

        # Find significant peaks (above 10% of max in frequency range 5-80Hz)
        freq_mask = (xf >= 5) & (xf <= sample_rate/2 - 5)
        power_masked = np.where(freq_mask, power, 0)

        if np.max(power_masked) > 0:
            threshold = np.max(power_masked) * 0.05  # 5% of max
            min_dist = int(3 * n / nominal_rate)  # 3 Hz minimum distance

            peak_idx, _ = signal.find_peaks(power_masked, height=threshold, distance=max(1, min_dist))

            print(f"\n{label} [{unit}]:")
            print(f"  Signal std: {np.std(data):.4f}, max power freq range: 5-{sample_rate/2:.0f} Hz")

            if len(peak_idx) == 0:
                print("  No significant peaks detected (>5 Hz)")
            else:
                # Sort by power
                sorted_idx = np.argsort(power[peak_idx])[::-1]
                top_peaks = peak_idx[sorted_idx[:8]]

                for i, idx in enumerate(top_peaks):
                    freq = xf[idx]
                    pwr = power[idx]
                    print(f"  {i+1}. {freq:6.1f} Hz  (power: {pwr:.2e})")

                    # Accumulate for summary
                    freq_rounded = round(freq)
                    if freq_rounded not in freq_bins:
                        freq_bins[freq_rounded] = []
                    freq_bins[freq_rounded].append((col, pwr))

    # Summary
    print(f"\n{'='*70}")
    print("NOTCH FILTER RECOMMENDATIONS")
    print(f"{'='*70}")

    # Group nearby frequencies
    freq_groups = []
    sorted_freqs = sorted(freq_bins.keys())

    if len(sorted_freqs) == 0:
        print("\nNo significant vibration frequencies detected above 5 Hz")
        print("This data may be from static/ground operation")
        return

    current_group = [sorted_freqs[0]]
    for freq in sorted_freqs[1:]:
        if freq - current_group[-1] <= 5:  # Within 5 Hz
            current_group.append(freq)
        else:
            freq_groups.append(current_group)
            current_group = [freq]
    freq_groups.append(current_group)

    # Score each group by total power and channel coverage
    group_scores = []
    for group in freq_groups:
        total_power = 0
        channels_hit = set()
        for freq in group:
            for col, pwr in freq_bins[freq]:
                total_power += pwr
                channels_hit.add(col)
        center_freq = np.mean(group)
        group_scores.append((center_freq, total_power, len(channels_hit), group))

    # Sort by combined score (channels * power)
    group_scores.sort(key=lambda x: -x[1] * x[2])

    print("\nDetected vibration frequency bands:")
    print("-" * 50)

    for i, (center, power, n_channels, freqs) in enumerate(group_scores[:5]):
        freq_range = f"{min(freqs)}-{max(freqs)}" if len(freqs) > 1 else f"{freqs[0]}"
        print(f"  Band {i+1}: {center:5.1f} Hz  (range: {freq_range} Hz, "
              f"channels: {n_channels}, power: {power:.2e})")

    print("\n" + "="*50)
    print("RECOMMENDED NOTCH FILTER SETTINGS:")
    print("="*50)

    # Recommend top 3 notch filters
    recommended = []
    for center, power, n_channels, freqs in group_scores[:3]:
        if n_channels >= 2:  # Must appear in at least 2 channels
            bandwidth = max(10, max(freqs) - min(freqs) + 4)
            recommended.append((center, bandwidth))

    if len(recommended) == 0:
        print("\nNo clear recommendations. The peaks are not consistent across channels.")
        print("Consider capturing data during more aggressive flight maneuvers.")
    else:
        for i, (center, bw) in enumerate(recommended):
            print(f"\n  Notch Filter {i+1}:")
            print(f"    Center frequency: {center:.0f} Hz")
            print(f"    Bandwidth: {bw:.0f} Hz")
            print(f"    Q factor: {center/bw:.1f}")

    # Motor frequency estimation
    print(f"\n{'='*70}")
    print("MOTOR FREQUENCY ESTIMATION")
    print(f"{'='*70}")

    # StampFly uses 0720 coreless motors, typical RPM range 15000-30000
    # At full throttle, ~25000 RPM = 417 Hz (beyond Nyquist)
    # At hover (~50% throttle), ~12000 RPM = 200 Hz (beyond Nyquist)
    # But propeller blade pass frequency: 2 blades × RPM/60 = 2 × 200 = 400 Hz (beyond)

    print("\nNote: Direct motor frequencies (100-400 Hz) are above the Nyquist limit.")
    print("The detected frequencies are likely:")
    print("  - Frame resonance modes")
    print("  - Mechanical vibration harmonics")
    print("  - Aliased motor frequencies (if present)")

    if len(recommended) > 0:
        print(f"\nThe dominant frequency around {recommended[0][0]:.0f} Hz suggests:")
        freq = recommended[0][0]
        if freq < 20:
            print(f"  - This could be a frame/arm resonance mode")
            print(f"  - Or low-frequency control oscillation")
        elif freq < 50:
            print(f"  - This could be a structural resonance")
            print(f"  - Propeller imbalance at low RPM")
        else:
            print(f"  - This is likely mechanical resonance or aliased motor vibration")


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_files = sorted([f for f in os.listdir(script_dir)
                       if f.startswith('stampfly_fft_') and f.endswith('.csv')])

    if not csv_files:
        print("No FFT CSV files found")
        return

    latest = csv_files[-1]
    filepath = os.path.join(script_dir, latest)

    analyze_for_notch_filter(filepath)


if __name__ == '__main__':
    main()
