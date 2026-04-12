#!/usr/bin/env python3
"""
StampFly SIL Flight Visualizer
StampFly SILフライト可視化ツール

Reads CSV output from sil_stampfly and generates flight analysis plots.
sil_stampflyのCSV出力を読み込み、フライト解析プロットを生成する。

Usage / 使い方:
    make plot                    # Build, run, and plot
    python3 plot_flight.py       # Plot from existing flight_log.csv
    python3 plot_flight.py -i custom.csv  # Custom input file

@design requirements.md §10 — SIL simulator visualization             [--]
"""

import sys
import argparse
import numpy as np

try:
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
except ImportError:
    print("Error: matplotlib is required. Install with: pip3 install matplotlib")
    sys.exit(1)


def load_csv(filename):
    """Load flight log CSV / フライトログCSVを読み込む"""
    # Read only lines that start with a digit (skip log messages)
    # 数字で始まる行のみ読む（ログメッセージをスキップ）
    import io
    with open(filename, 'r') as f:
        lines = f.readlines()

    # Find header line (starts with 'time')
    # ヘッダー行を探す（'time'で始まる）
    header_idx = None
    for i, line in enumerate(lines):
        if line.startswith('time'):
            header_idx = i
            break

    if header_idx is None:
        raise ValueError("CSV header not found")

    # Keep header + data lines (start with digit)
    # ヘッダー + データ行（数字で始まる）のみ保持
    clean_lines = [lines[header_idx]]
    for line in lines[header_idx + 1:]:
        if line and line[0].isdigit():
            clean_lines.append(line)

    data = np.genfromtxt(io.StringIO(''.join(clean_lines)),
                         delimiter=',', names=True)
    print(f"Loaded {len(data)} samples from {filename}")
    return data


def plot_flight(data, output_file=None):
    """Generate flight analysis plots / フライト解析プロットを生成"""

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('StampFly SIL Flight Analysis', fontsize=14, fontweight='bold')

    gs = gridspec.GridSpec(3, 2, hspace=0.35, wspace=0.3)
    t = data['time']

    # =====================================================================
    # Plot 1: Altitude (true vs ESKF estimate)
    # プロット1: 高度（真値 vs ESKF推定値）
    # =====================================================================
    ax1 = fig.add_subplot(gs[0, 0])
    true_alt = -data['true_z']  # NED→height
    eskf_alt = -data['eskf_z']
    ax1.plot(t, true_alt, 'b-', label='True altitude', linewidth=1.5)
    ax1.plot(t, eskf_alt, 'r--', label='ESKF estimate', linewidth=1.0)
    ax1.axhline(y=0.5, color='g', linestyle=':', alpha=0.5, label='Target (0.5m)')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Altitude [m]')
    ax1.set_title('Altitude')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([-0.1, 1.5])

    # =====================================================================
    # Plot 2: Vertical velocity
    # プロット2: 垂直速度
    # =====================================================================
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, -data['true_vz'], 'b-', label='True Vz', linewidth=1.0)
    ax2.plot(t, -data['eskf_vz'], 'r--', label='ESKF Vz', linewidth=1.0)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Climb rate [m/s]')
    ax2.set_title('Vertical Velocity')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # =====================================================================
    # Plot 3: Attitude (roll, pitch, yaw)
    # プロット3: 姿勢（ロール、ピッチ、ヨー）
    # =====================================================================
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t, data['roll'], 'r-', label='True Roll', alpha=0.7, linewidth=0.8)
    ax3.plot(t, data['pitch'], 'g-', label='True Pitch', alpha=0.7, linewidth=0.8)
    ax3.plot(t, data['yaw'], 'b-', label='True Yaw', alpha=0.7, linewidth=0.8)
    ax3.plot(t, data['eskf_roll'], 'r--', label='ESKF Roll', linewidth=0.8)
    ax3.plot(t, data['eskf_pitch'], 'g--', label='ESKF Pitch', linewidth=0.8)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Angle [deg]')
    ax3.set_title('Attitude')
    ax3.legend(fontsize=7, ncol=2)
    ax3.grid(True, alpha=0.3)

    # =====================================================================
    # Plot 4: Motor duty
    # プロット4: モーターduty
    # =====================================================================
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t, data['m1'], label='M1 (FR)', linewidth=0.8)
    ax4.plot(t, data['m2'], label='M2 (RR)', linewidth=0.8)
    ax4.plot(t, data['m3'], label='M3 (RL)', linewidth=0.8)
    ax4.plot(t, data['m4'], label='M4 (FL)', linewidth=0.8)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Duty [0-1]')
    ax4.set_title('Motor Commands')
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)
    ax4.set_ylim([-0.05, 1.0])

    # =====================================================================
    # Plot 5: Thrust command
    # プロット5: 推力コマンド
    # =====================================================================
    ax5 = fig.add_subplot(gs[2, 0])
    ax5.plot(t, data['thrust'], 'k-', linewidth=1.0)
    hover = 0.037 * 9.81
    ax5.axhline(y=hover, color='g', linestyle=':', alpha=0.5, label=f'Hover ({hover:.3f}N)')
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Thrust [N]')
    ax5.set_title('Total Thrust')
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)

    # =====================================================================
    # Plot 6: IMU sensor output (accel_z, gyro_x)
    # プロット6: IMUセンサ出力
    # =====================================================================
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.plot(t, data['accel_z'], 'b-', label='Accel Z', linewidth=0.5, alpha=0.7)
    ax6r = ax6.twinx()
    ax6r.plot(t, data['gyro_x'] * 57.3, 'r-', label='Gyro X', linewidth=0.5, alpha=0.7)
    ax6.set_xlabel('Time [s]')
    ax6.set_ylabel('Accel Z [m/s²]', color='b')
    ax6r.set_ylabel('Gyro X [°/s]', color='r')
    ax6.set_title('IMU Sensor Output (with noise)')
    ax6.grid(True, alpha=0.3)

    # Save or show / 保存または表示
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved plot to {output_file}")
    else:
        plt.savefig('flight_analysis.png', dpi=150, bbox_inches='tight')
        print("Saved plot to flight_analysis.png")

    try:
        plt.show()
    except Exception:
        pass  # Non-interactive backend


def main():
    parser = argparse.ArgumentParser(
        description='StampFly SIL Flight Visualizer')
    parser.add_argument('-i', '--input', default='flight_log.csv',
                        help='Input CSV file (default: flight_log.csv)')
    parser.add_argument('-o', '--output', default=None,
                        help='Output image file (default: flight_analysis.png)')
    args = parser.parse_args()

    data = load_csv(args.input)
    plot_flight(data, args.output)


if __name__ == '__main__':
    main()
