#!/usr/bin/env python3
"""
Flight Log Noise Analyzer v2 — Segment-based analysis
フライトログノイズ解析 v2 — 区間ベース解析

Automatically detects flight phases (static/flight/transition) by
analyzing time-series data, then extracts noise characteristics
from confirmed flight segments.

時系列データを解析して飛行フェーズ（静止/飛行/遷移）を自動検出し、
確認された飛行区間からノイズ特性を抽出する。

Usage:
    python3 analyze_logs.py --log-dir ../../../logs
    python3 analyze_logs.py --log-dir ../../../logs --plot-ambiguous
"""

import os
import sys
import glob
import argparse
import numpy as np
from collections import defaultdict

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def load_log(filepath):
    """Load a single CSV flight log"""
    try:
        data = np.genfromtxt(filepath, delimiter=',', names=True,
                             invalid_raise=False)
        if data is None or len(data) < 50:
            return None
        return data
    except Exception:
        return None


def detect_segments(data, window_size=200):
    """
    Detect flight phases using sliding window noise analysis.
    スライディングウィンドウノイズ解析で飛行フェーズを検出。

    Returns list of (start_idx, end_idx, phase) tuples.
    phase: 'static', 'flight', 'transition'
    """
    n = len(data)
    if n < window_size * 2:
        window_size = max(50, n // 4)

    # Compute windowed accel_z standard deviation
    # ウィンドウ加速度Zの標準偏差を計算
    if 'accel_z' not in data.dtype.names:
        return [{'start': 0, 'end': n, 'phase': 'unknown'}]

    accel_z = data['accel_z']
    accel_z = np.where(np.isnan(accel_z), 0, accel_z)

    # Sliding window RMS (deviation from window mean)
    # スライディングウィンドウRMS
    half_w = window_size // 2
    noise_profile = np.zeros(n)
    for i in range(n):
        lo = max(0, i - half_w)
        hi = min(n, i + half_w)
        segment = accel_z[lo:hi]
        noise_profile[i] = np.std(segment)

    # Also check ToF for altitude information
    # ToFで高度情報もチェック
    tof = np.zeros(n)
    if 'tof_bottom' in data.dtype.names:
        tof = data['tof_bottom']
        tof = np.where(np.isnan(tof), 0, tof)

    # Also check pos_z for ESKF altitude
    # ESKFの高度もチェック
    pos_z = np.zeros(n)
    if 'pos_z' in data.dtype.names:
        pos_z = data['pos_z']
        pos_z = np.where(np.isnan(pos_z), 0, pos_z)

    # Classify each point
    # 各ポイントを分類
    # Thresholds derived from data overview:
    # Static: accel_z_std < 0.05 m/s²
    # Flight: accel_z_std > 0.3 m/s² (vibration from motors)
    STATIC_THRESH = 0.05
    FLIGHT_THRESH = 0.3

    phases = np.zeros(n, dtype='U10')
    for i in range(n):
        if noise_profile[i] < STATIC_THRESH:
            phases[i] = 'static'
        elif noise_profile[i] > FLIGHT_THRESH:
            phases[i] = 'flight'
        else:
            phases[i] = 'trans'

    # Merge into segments (minimum segment length)
    # セグメントにマージ（最小セグメント長）
    MIN_SEGMENT = 100  # ~0.25s at 400Hz
    segments = []
    current_phase = phases[0]
    seg_start = 0

    for i in range(1, n):
        if phases[i] != current_phase:
            if i - seg_start >= MIN_SEGMENT:
                segments.append({
                    'start': seg_start,
                    'end': i,
                    'phase': current_phase,
                    'duration_s': (i - seg_start) * 0.0025,
                    'noise_mean': np.mean(noise_profile[seg_start:i]),
                    'tof_mean': np.mean(tof[seg_start:i]),
                    'pos_z_mean': np.mean(pos_z[seg_start:i]),
                })
            current_phase = phases[i]
            seg_start = i

    # Last segment
    if n - seg_start >= MIN_SEGMENT:
        segments.append({
            'start': seg_start,
            'end': n,
            'phase': current_phase,
            'duration_s': (n - seg_start) * 0.0025,
            'noise_mean': np.mean(noise_profile[seg_start:n]),
            'tof_mean': np.mean(tof[seg_start:n]),
            'pos_z_mean': np.mean(pos_z[seg_start:n]),
        })

    return segments, noise_profile


def analyze_segment(data, seg):
    """Extract noise statistics from a segment"""
    s, e = seg['start'], seg['end']
    result = {}

    for col in ['accel_x', 'accel_y', 'accel_z',
                 'gyro_x', 'gyro_y', 'gyro_z']:
        if col in data.dtype.names:
            vals = data[col][s:e]
            vals = vals[~np.isnan(vals)]
            if len(vals) > 10:
                result[f'{col}_std'] = float(np.std(vals))
                result[f'{col}_mean'] = float(np.mean(vals))

    if 'tof_bottom' in data.dtype.names:
        tof = data['tof_bottom'][s:e]
        tof = tof[~np.isnan(tof)]
        tof = tof[tof > 0.01]
        if len(tof) > 10:
            result['tof_std'] = float(np.std(tof))
            result['tof_mean'] = float(np.mean(tof))

    if 'ctrl_throttle' in data.dtype.names:
        thr = data['ctrl_throttle'][s:e]
        thr = thr[~np.isnan(thr)]
        if len(thr) > 10:
            result['throttle_mean'] = float(np.mean(thr))

    for col in ['accel_bias_x', 'accel_bias_y', 'accel_bias_z',
                 'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z']:
        if col in data.dtype.names:
            vals = data[col][s:e]
            vals = vals[~np.isnan(vals)]
            if len(vals) > 10:
                result[f'{col}_mean'] = float(np.mean(vals))

    return result


def plot_log_segments(data, segments, noise_profile, filename, output_dir):
    """Generate segment visualization for ambiguous logs"""
    if not HAS_MATPLOTLIB:
        return

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f'Segment Analysis: {os.path.basename(filename)}', fontsize=12)

    n = len(data)
    t = np.arange(n) * 0.0025  # Assuming 400Hz

    # Accel Z
    if 'accel_z' in data.dtype.names:
        axes[0].plot(t, data['accel_z'], 'b-', linewidth=0.3, alpha=0.5)
        axes[0].set_ylabel('Accel Z [m/s²]')
        axes[0].set_title('Accelerometer Z')

    # Noise profile with thresholds
    axes[1].plot(t, noise_profile, 'k-', linewidth=0.5)
    axes[1].axhline(y=0.05, color='g', linestyle='--', label='Static thresh')
    axes[1].axhline(y=0.3, color='r', linestyle='--', label='Flight thresh')
    axes[1].set_ylabel('Window σ [m/s²]')
    axes[1].set_title('Noise Profile')
    axes[1].legend(fontsize=8)

    # Segments colored
    colors = {'static': 'green', 'flight': 'red', 'trans': 'orange', 'unknown': 'gray'}
    for seg in segments:
        s_t = seg['start'] * 0.0025
        e_t = seg['end'] * 0.0025
        axes[1].axvspan(s_t, e_t, alpha=0.2, color=colors.get(seg['phase'], 'gray'))

    # ToF
    if 'tof_bottom' in data.dtype.names:
        axes[2].plot(t, data['tof_bottom'], 'b-', linewidth=0.5)
        axes[2].set_ylabel('ToF [m]')
        axes[2].set_title('ToF Bottom')

    # Pos Z
    if 'pos_z' in data.dtype.names:
        axes[3].plot(t, -data['pos_z'], 'b-', linewidth=0.5)
        axes[3].set_ylabel('Altitude [m]')
        axes[3].set_title('ESKF Altitude')

    axes[3].set_xlabel('Time [s]')

    os.makedirs(output_dir, exist_ok=True)
    out_file = os.path.join(output_dir, os.path.basename(filename).replace('.csv', '_segments.png'))
    plt.savefig(out_file, dpi=100, bbox_inches='tight')
    plt.close()
    return out_file


def main():
    parser = argparse.ArgumentParser(description='Flight Log Noise Analyzer v2')
    parser.add_argument('--log-dir', default='../../../logs')
    parser.add_argument('--output', default='noise_analysis_v2.txt')
    parser.add_argument('--plot-all', action='store_true',
                        help='Generate segment plots for all logs')
    parser.add_argument('--plot-ambiguous', action='store_true',
                        help='Generate segment plots for ambiguous logs only')
    parser.add_argument('--plot-dir', default='log_plots')
    args = parser.parse_args()

    if not os.path.isdir(args.log_dir):
        print(f"Error: {args.log_dir} not found")
        sys.exit(1)

    files = sorted(glob.glob(os.path.join(args.log_dir, '*.csv')))
    print(f"Found {len(files)} log files\n")

    # Collect all flight segments across all logs
    all_static = []
    all_flight = []
    all_trans = []
    log_summaries = []
    ambiguous_files = []

    for filepath in files:
        name = os.path.basename(filepath)
        data = load_log(filepath)
        if data is None:
            continue

        segments, noise_profile = detect_segments(data)

        has_flight = any(s['phase'] == 'flight' for s in segments)
        has_static = any(s['phase'] == 'static' for s in segments)
        total_flight_s = sum(s['duration_s'] for s in segments if s['phase'] == 'flight')
        total_static_s = sum(s['duration_s'] for s in segments if s['phase'] == 'static')
        total_trans_s = sum(s['duration_s'] for s in segments if s['phase'] == 'trans')

        # Analyze each segment
        for seg in segments:
            stats = analyze_segment(data, seg)
            stats['file'] = name
            stats['phase'] = seg['phase']
            stats['duration_s'] = seg['duration_s']

            if seg['phase'] == 'static':
                all_static.append(stats)
            elif seg['phase'] == 'flight':
                all_flight.append(stats)
            else:
                all_trans.append(stats)

        # Log summary
        summary = f"  {name:<50s} flight={total_flight_s:5.1f}s  static={total_static_s:5.1f}s  trans={total_trans_s:5.1f}s  segs={len(segments)}"
        log_summaries.append(summary)

        is_ambiguous = total_trans_s > 1.0 or (has_flight and has_static and total_flight_s < 2)
        if is_ambiguous:
            ambiguous_files.append(filepath)

        # Plot if requested
        if args.plot_all or (args.plot_ambiguous and is_ambiguous):
            plot_log_segments(data, segments, noise_profile, filepath, args.plot_dir)

    # Build report
    report = []
    report.append("=" * 80)
    report.append("FLIGHT LOG NOISE ANALYSIS v2 — Segment-based")
    report.append("=" * 80)
    report.append(f"\nLogs analyzed: {len(log_summaries)}")
    report.append(f"Flight segments: {len(all_flight)} ({sum(s['duration_s'] for s in all_flight):.1f}s total)")
    report.append(f"Static segments: {len(all_static)} ({sum(s['duration_s'] for s in all_static):.1f}s total)")
    report.append(f"Transition segments: {len(all_trans)} ({sum(s['duration_s'] for s in all_trans):.1f}s total)")
    report.append(f"Ambiguous logs: {len(ambiguous_files)}")

    report.append("\n--- Per-log summary ---")
    for s in log_summaries:
        report.append(s)

    # Static noise statistics
    report.append("\n" + "=" * 80)
    report.append("STATIC SEGMENTS — Sensor noise baseline (motors OFF)")
    report.append("=" * 80)
    if all_static:
        for key in ['accel_x_std', 'accel_y_std', 'accel_z_std',
                     'gyro_x_std', 'gyro_y_std', 'gyro_z_std']:
            vals = [s[key] for s in all_static if key in s]
            if vals:
                report.append(f"  {key:25s}: median={np.median(vals):.6f}  "
                              f"mean={np.mean(vals):.6f}  "
                              f"[{np.percentile(vals,5):.6f}, {np.percentile(vals,95):.6f}] "
                              f"(n={len(vals)})")

    # Flight noise statistics
    report.append("\n" + "=" * 80)
    report.append("FLIGHT SEGMENTS — In-flight noise (motors ON, vibration)")
    report.append("=" * 80)
    if all_flight:
        for key in ['accel_x_std', 'accel_y_std', 'accel_z_std',
                     'gyro_x_std', 'gyro_y_std', 'gyro_z_std',
                     'tof_std', 'tof_mean', 'throttle_mean']:
            vals = [s[key] for s in all_flight if key in s]
            if vals:
                report.append(f"  {key:25s}: median={np.median(vals):.4f}  "
                              f"mean={np.mean(vals):.4f}  "
                              f"[{np.percentile(vals,5):.4f}, {np.percentile(vals,95):.4f}] "
                              f"(n={len(vals)})")

    # Vibration ratio
    report.append("\n" + "=" * 80)
    report.append("VIBRATION RATIO — Flight / Static")
    report.append("=" * 80)
    for key in ['accel_x_std', 'accel_y_std', 'accel_z_std',
                 'gyro_x_std', 'gyro_y_std', 'gyro_z_std']:
        s_vals = [s[key] for s in all_static if key in s]
        f_vals = [s[key] for s in all_flight if key in s]
        if s_vals and f_vals:
            ratio = np.median(f_vals) / np.median(s_vals)
            report.append(f"  {key:25s}: static={np.median(s_vals):.6f}  "
                          f"flight={np.median(f_vals):.4f}  "
                          f"ratio={ratio:.0f}x")

    # Bias statistics from flight segments
    report.append("\n" + "=" * 80)
    report.append("BIAS VALUES — From flight segments")
    report.append("=" * 80)
    for key in ['gyro_bias_x_mean', 'gyro_bias_y_mean', 'gyro_bias_z_mean',
                'accel_bias_x_mean', 'accel_bias_y_mean', 'accel_bias_z_mean']:
        vals = [s[key] for s in all_flight if key in s]
        if vals:
            report.append(f"  {key:25s}: median={np.median(vals):.6f}  "
                          f"std={np.std(vals):.6f}  "
                          f"(n={len(vals)})")

    # SIL recommended parameters
    report.append("\n" + "=" * 80)
    report.append("RECOMMENDED SIL NOISE PARAMETERS")
    report.append("=" * 80)

    if all_static:
        az_static = [s['accel_z_std'] for s in all_static if 'accel_z_std' in s]
        gx_static = [s['gyro_x_std'] for s in all_static if 'gyro_x_std' in s]
        if az_static:
            nd = np.median(az_static) / np.sqrt(400)
            report.append(f"  accel_noise_density (static):  {nd:.6f} m/s²/√Hz  (from median σ={np.median(az_static):.6f})")
        if gx_static:
            nd = np.median(gx_static) / np.sqrt(400)
            report.append(f"  gyro_noise_density (static):   {nd:.6f} rad/s/√Hz  (from median σ={np.median(gx_static):.6f})")

    if all_flight:
        az_flight = [s['accel_z_std'] for s in all_flight if 'accel_z_std' in s]
        gx_flight = [s['gyro_x_std'] for s in all_flight if 'gyro_x_std' in s]
        if az_flight:
            report.append(f"  vib_accel_k (hover):           {np.median(az_flight):.2f} m/s² (median flight accel_z σ)")
        if gx_flight:
            report.append(f"  vib_gyro_k (hover):            {np.median(gx_flight):.2f} rad/s (median flight gyro_x σ)")
        tof_f = [s['tof_std'] for s in all_flight if 'tof_std' in s]
        if tof_f:
            report.append(f"  tof_noise (flight):            {np.median(tof_f):.4f} m (median flight ToF σ)")

    full_report = '\n'.join(report)
    print(full_report)

    with open(args.output, 'w') as f:
        f.write(full_report)
    print(f"\nReport saved to {args.output}")

    if ambiguous_files and not args.plot_ambiguous:
        print(f"\n{len(ambiguous_files)} ambiguous logs detected. Run with --plot-ambiguous to inspect.")


if __name__ == '__main__':
    main()
