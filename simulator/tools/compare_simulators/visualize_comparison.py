#!/usr/bin/env python3
"""
visualize_comparison.py - Simulator Comparison Visualization
シミュレータ比較可視化ツール

Compares output from VPython and Genesis simulators.
VPythonとGenesisシミュレータの出力を比較

Usage:
  python visualize_comparison.py --vpython vpython_output.csv --genesis genesis_output.csv
  python visualize_comparison.py --vpython vpython_output.csv --genesis genesis_output.csv --save comparison.png
"""

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Add path
_SCRIPT_DIR = Path(__file__).parent
sys.path.insert(0, str(_SCRIPT_DIR))

from sim_io import load_output_csv, StateLog


def extract_arrays(logs):
    """
    Extract numpy arrays from state logs.
    状態ログからnumpy配列を抽出
    """
    n = len(logs)
    data = {
        'time': np.zeros(n),
        'x': np.zeros(n),
        'y': np.zeros(n),
        'z': np.zeros(n),
        'roll': np.zeros(n),
        'pitch': np.zeros(n),
        'yaw': np.zeros(n),
        'p': np.zeros(n),
        'q': np.zeros(n),
        'r': np.zeros(n),
    }

    for i, log in enumerate(logs):
        data['time'][i] = log.time
        data['x'][i] = log.x
        data['y'][i] = log.y
        data['z'][i] = log.z
        data['roll'][i] = log.roll
        data['pitch'][i] = log.pitch
        data['yaw'][i] = log.yaw
        data['p'][i] = log.p
        data['q'][i] = log.q
        data['r'][i] = log.r

    return data


def compute_error_metrics(vpython_data, genesis_data):
    """
    Compute error metrics between two datasets.
    2つのデータセット間の誤差メトリクスを計算
    """
    # Interpolate to common time base
    t_common = np.union1d(vpython_data['time'], genesis_data['time'])
    t_common = t_common[(t_common >= max(vpython_data['time'][0], genesis_data['time'][0])) &
                        (t_common <= min(vpython_data['time'][-1], genesis_data['time'][-1]))]

    metrics = {}
    variables = ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'p', 'q', 'r']

    for var in variables:
        vp = np.interp(t_common, vpython_data['time'], vpython_data[var])
        gs = np.interp(t_common, genesis_data['time'], genesis_data[var])

        diff = vp - gs
        metrics[var] = {
            'rmse': np.sqrt(np.mean(diff**2)),
            'max_error': np.max(np.abs(diff)),
            'mean_error': np.mean(diff),
        }

    return metrics


def plot_comparison(vpython_data, genesis_data, title="Simulator Comparison", save_path=None):
    """
    Plot comparison between VPython and Genesis outputs.
    VPythonとGenesisの出力を比較プロット
    """
    fig = plt.figure(figsize=(14, 12))
    fig.suptitle(title, fontsize=14, fontweight='bold')

    # Create subplots grid: 3 rows x 3 columns
    # Row 1: Position (x, y, z)
    # Row 2: Attitude (roll, pitch, yaw)
    # Row 3: Angular rates (p, q, r)

    t_vp = vpython_data['time']
    t_gs = genesis_data['time']

    # Position plots
    ax1 = fig.add_subplot(3, 3, 1)
    ax1.plot(t_vp, vpython_data['x'], 'b-', label='VPython', linewidth=1.5)
    ax1.plot(t_gs, genesis_data['x'], 'r--', label='Genesis', linewidth=1.5)
    ax1.set_ylabel('X [m]')
    ax1.set_title('Position X')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(3, 3, 2)
    ax2.plot(t_vp, vpython_data['y'], 'b-', label='VPython', linewidth=1.5)
    ax2.plot(t_gs, genesis_data['y'], 'r--', label='Genesis', linewidth=1.5)
    ax2.set_ylabel('Y [m]')
    ax2.set_title('Position Y')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(3, 3, 3)
    ax3.plot(t_vp, vpython_data['z'], 'b-', label='VPython', linewidth=1.5)
    ax3.plot(t_gs, genesis_data['z'], 'r--', label='Genesis', linewidth=1.5)
    ax3.set_ylabel('Z [m]')
    ax3.set_title('Position Z (Height)')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)

    # Attitude plots (convert to degrees for display)
    rad2deg = 180.0 / np.pi

    ax4 = fig.add_subplot(3, 3, 4)
    ax4.plot(t_vp, vpython_data['roll'] * rad2deg, 'b-', label='VPython', linewidth=1.5)
    ax4.plot(t_gs, genesis_data['roll'] * rad2deg, 'r--', label='Genesis', linewidth=1.5)
    ax4.set_ylabel('Roll [deg]')
    ax4.set_title('Roll Angle')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)

    ax5 = fig.add_subplot(3, 3, 5)
    ax5.plot(t_vp, vpython_data['pitch'] * rad2deg, 'b-', label='VPython', linewidth=1.5)
    ax5.plot(t_gs, genesis_data['pitch'] * rad2deg, 'r--', label='Genesis', linewidth=1.5)
    ax5.set_ylabel('Pitch [deg]')
    ax5.set_title('Pitch Angle')
    ax5.legend(loc='upper right')
    ax5.grid(True, alpha=0.3)

    ax6 = fig.add_subplot(3, 3, 6)
    ax6.plot(t_vp, vpython_data['yaw'] * rad2deg, 'b-', label='VPython', linewidth=1.5)
    ax6.plot(t_gs, genesis_data['yaw'] * rad2deg, 'r--', label='Genesis', linewidth=1.5)
    ax6.set_ylabel('Yaw [deg]')
    ax6.set_title('Yaw Angle')
    ax6.legend(loc='upper right')
    ax6.grid(True, alpha=0.3)

    # Angular rate plots (convert to deg/s for display)
    ax7 = fig.add_subplot(3, 3, 7)
    ax7.plot(t_vp, vpython_data['p'] * rad2deg, 'b-', label='VPython', linewidth=1.5)
    ax7.plot(t_gs, genesis_data['p'] * rad2deg, 'r--', label='Genesis', linewidth=1.5)
    ax7.set_xlabel('Time [s]')
    ax7.set_ylabel('p [deg/s]')
    ax7.set_title('Roll Rate')
    ax7.legend(loc='upper right')
    ax7.grid(True, alpha=0.3)

    ax8 = fig.add_subplot(3, 3, 8)
    ax8.plot(t_vp, vpython_data['q'] * rad2deg, 'b-', label='VPython', linewidth=1.5)
    ax8.plot(t_gs, genesis_data['q'] * rad2deg, 'r--', label='Genesis', linewidth=1.5)
    ax8.set_xlabel('Time [s]')
    ax8.set_ylabel('q [deg/s]')
    ax8.set_title('Pitch Rate')
    ax8.legend(loc='upper right')
    ax8.grid(True, alpha=0.3)

    ax9 = fig.add_subplot(3, 3, 9)
    ax9.plot(t_vp, vpython_data['r'] * rad2deg, 'b-', label='VPython', linewidth=1.5)
    ax9.plot(t_gs, genesis_data['r'] * rad2deg, 'r--', label='Genesis', linewidth=1.5)
    ax9.set_xlabel('Time [s]')
    ax9.set_ylabel('r [deg/s]')
    ax9.set_title('Yaw Rate')
    ax9.legend(loc='upper right')
    ax9.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")

    return fig


def plot_error(vpython_data, genesis_data, title="Error Analysis", save_path=None):
    """
    Plot error between VPython and Genesis outputs.
    VPythonとGenesisの出力間の誤差をプロット
    """
    # Interpolate to common time base
    t_vp = vpython_data['time']
    t_gs = genesis_data['time']
    t_common = np.linspace(
        max(t_vp[0], t_gs[0]),
        min(t_vp[-1], t_gs[-1]),
        min(len(t_vp), len(t_gs))
    )

    fig = plt.figure(figsize=(14, 8))
    fig.suptitle(title, fontsize=14, fontweight='bold')

    rad2deg = 180.0 / np.pi

    # Position error
    ax1 = fig.add_subplot(2, 3, 1)
    for var, label in [('x', 'X'), ('y', 'Y'), ('z', 'Z')]:
        vp = np.interp(t_common, t_vp, vpython_data[var])
        gs = np.interp(t_common, t_gs, genesis_data[var])
        ax1.plot(t_common, vp - gs, label=label, linewidth=1.5)
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Error [m]')
    ax1.set_title('Position Error (VPython - Genesis)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Attitude error
    ax2 = fig.add_subplot(2, 3, 2)
    for var, label in [('roll', 'Roll'), ('pitch', 'Pitch'), ('yaw', 'Yaw')]:
        vp = np.interp(t_common, t_vp, vpython_data[var])
        gs = np.interp(t_common, t_gs, genesis_data[var])
        ax2.plot(t_common, (vp - gs) * rad2deg, label=label, linewidth=1.5)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Error [deg]')
    ax2.set_title('Attitude Error (VPython - Genesis)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Angular rate error
    ax3 = fig.add_subplot(2, 3, 3)
    for var, label in [('p', 'p'), ('q', 'q'), ('r', 'r')]:
        vp = np.interp(t_common, t_vp, vpython_data[var])
        gs = np.interp(t_common, t_gs, genesis_data[var])
        ax3.plot(t_common, (vp - gs) * rad2deg, label=label, linewidth=1.5)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Error [deg/s]')
    ax3.set_title('Angular Rate Error (VPython - Genesis)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 3D trajectory comparison
    ax4 = fig.add_subplot(2, 3, 4, projection='3d')
    ax4.plot(vpython_data['x'], vpython_data['y'], vpython_data['z'],
             'b-', label='VPython', linewidth=1.5)
    ax4.plot(genesis_data['x'], genesis_data['y'], genesis_data['z'],
             'r--', label='Genesis', linewidth=1.5)
    ax4.set_xlabel('X [m]')
    ax4.set_ylabel('Y [m]')
    ax4.set_zlabel('Z [m]')
    ax4.set_title('3D Trajectory')
    ax4.legend()

    # XY plane trajectory
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(vpython_data['x'], vpython_data['y'], 'b-', label='VPython', linewidth=1.5)
    ax5.plot(genesis_data['x'], genesis_data['y'], 'r--', label='Genesis', linewidth=1.5)
    ax5.set_xlabel('X [m]')
    ax5.set_ylabel('Y [m]')
    ax5.set_title('XY Trajectory (Top View)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    ax5.axis('equal')

    # Error statistics text
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.axis('off')

    metrics = compute_error_metrics(vpython_data, genesis_data)

    text = "Error Statistics (VPython - Genesis)\n"
    text += "=" * 40 + "\n\n"
    text += f"{'Variable':<12} {'RMSE':>12} {'Max Error':>12}\n"
    text += "-" * 40 + "\n"

    for var in ['x', 'y', 'z']:
        m = metrics[var]
        text += f"{var:<12} {m['rmse']:>12.4f} m  {m['max_error']:>10.4f} m\n"

    text += "\n"
    for var in ['roll', 'pitch', 'yaw']:
        m = metrics[var]
        text += f"{var:<12} {m['rmse']*rad2deg:>12.4f} deg  {m['max_error']*rad2deg:>8.4f} deg\n"

    text += "\n"
    for var in ['p', 'q', 'r']:
        m = metrics[var]
        text += f"{var:<12} {m['rmse']*rad2deg:>12.4f} deg/s  {m['max_error']*rad2deg:>6.4f} deg/s\n"

    ax6.text(0.1, 0.9, text, transform=ax6.transAxes, fontsize=10,
             verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {save_path}")

    return fig


def main():
    parser = argparse.ArgumentParser(description='Simulator Comparison Visualization')
    parser.add_argument('--vpython', '-v', type=str, required=True,
                       help='VPython output CSV file')
    parser.add_argument('--genesis', '-g', type=str, required=True,
                       help='Genesis output CSV file')
    parser.add_argument('--save', '-s', type=str, default=None,
                       help='Save comparison plot to file')
    parser.add_argument('--save-error', '-e', type=str, default=None,
                       help='Save error analysis plot to file')
    parser.add_argument('--no-show', action='store_true',
                       help='Do not display plots (only save)')
    args = parser.parse_args()

    print("=" * 60)
    print("Simulator Comparison Visualization")
    print("シミュレータ比較可視化")
    print("=" * 60)

    # Load data
    print(f"\nLoading VPython: {args.vpython}")
    vpython_logs, vpython_meta = load_output_csv(args.vpython)
    print(f"  Samples: {len(vpython_logs)}")
    print(f"  Metadata: {vpython_meta}")

    print(f"\nLoading Genesis: {args.genesis}")
    genesis_logs, genesis_meta = load_output_csv(args.genesis)
    print(f"  Samples: {len(genesis_logs)}")
    print(f"  Metadata: {genesis_meta}")

    # Extract arrays
    vpython_data = extract_arrays(vpython_logs)
    genesis_data = extract_arrays(genesis_logs)

    # Compute metrics
    print("\n" + "=" * 60)
    print("Error Metrics")
    print("=" * 60)
    metrics = compute_error_metrics(vpython_data, genesis_data)

    rad2deg = 180.0 / np.pi
    print(f"\n{'Variable':<12} {'RMSE':>15} {'Max Error':>15}")
    print("-" * 45)
    for var in ['x', 'y', 'z']:
        m = metrics[var]
        print(f"{var:<12} {m['rmse']:>12.6f} m   {m['max_error']:>12.6f} m")
    for var in ['roll', 'pitch', 'yaw']:
        m = metrics[var]
        print(f"{var:<12} {m['rmse']*rad2deg:>12.6f} deg {m['max_error']*rad2deg:>12.6f} deg")
    for var in ['p', 'q', 'r']:
        m = metrics[var]
        print(f"{var:<12} {m['rmse']*rad2deg:>12.6f} deg/s {m['max_error']*rad2deg:>10.6f} deg/s")

    # Plot comparison
    print("\n" + "=" * 60)
    print("Generating plots...")
    print("=" * 60)

    plot_comparison(vpython_data, genesis_data,
                   title="VPython vs Genesis Comparison",
                   save_path=args.save)

    plot_error(vpython_data, genesis_data,
              title="VPython vs Genesis Error Analysis",
              save_path=args.save_error)

    if not args.no_show:
        print("\nShowing plots... (close windows to exit)")
        plt.show()

    print("\nDone!")


if __name__ == "__main__":
    main()
