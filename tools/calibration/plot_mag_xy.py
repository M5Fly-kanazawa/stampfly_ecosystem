#!/usr/bin/env python3
"""
Plot Magnetometer XY data from a StampFly flight-log bundle to verify
calibration.
StampFly フライトログ一式から地磁気 XY データを描画し、校正を検証する。

The plot should show points forming a circle centered at origin
if calibration is correct.
校正が正しければ、点群は原点を中心とした円を描くはず。

Usage:
    python plot_mag_xy.py <bundle> [output.png]

`<bundle>` is a StampFly flight-log v1 bundle: a `.sflog.zip` file or an
extracted directory (see lib/sflog). This reads its `mag` stream (x/y/z in
uT, see lib/sflog/schema.py) -- the old vehicle_old USB blackbox `.bin`
format (V2 packet, 128 bytes) is no longer read here; that format and
`sf log capture` were removed together (docs/plans/flight-log-format-plan.md
section 3.4).
`<bundle>` は StampFly フライトログ v1 一式（`.sflog.zip` または展開済み
フォルダ、lib/sflog 参照）。その `mag` ストリーム（x/y/z、単位 uT、
lib/sflog/schema.py 参照）を読む -- 旧 vehicle_old の USB ブラックボックス
`.bin` 形式（V2パケット、128バイト）はもう読まない。この形式と
`sf log capture` は一緒に削除された（計画書 3.4節）。
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

import sflog


def load_mag_bundle(path):
    """Load the `mag` stream (x/y/z in uT) from a flight-log bundle.
    フライトログ一式から `mag` ストリーム（x/y/z、単位 uT）を読み込む。

    Args:
        path: `.sflog.zip` file or extracted-directory bundle path.

    Returns:
        pandas.DataFrame with columns timestamp_us, x, y, z (see
        lib/sflog/schema.py's `mag` stream).

    Raises:
        ValueError: the bundle has no `mag` stream (the firmware/log did
            not record magnetometer data).
    """
    log = sflog.FlightLog.load(path)
    if "mag" not in log.streams:
        raise ValueError(
            f"no mag stream in this bundle ({path}) -- the firmware/log did "
            "not record magnetometer data"
        )
    return log.streams["mag"]


def plot_mag_xy(mag_df, output_file=None):
    """Plot magnetometer XY data.
    地磁気 XY データを描画する。

    Args:
        mag_df: pandas.DataFrame with columns timestamp_us, x, y, z (uT) --
            e.g. the return value of load_mag_bundle().
        output_file: optional PNG path to save to (in addition to showing
            the plot interactively).
    """
    mag_x = mag_df["x"].to_numpy()
    mag_y = mag_df["y"].to_numpy()
    mag_z = mag_df["z"].to_numpy()

    # Calculate statistics
    center_x = (np.max(mag_x) + np.min(mag_x)) / 2
    center_y = (np.max(mag_y) + np.min(mag_y)) / 2
    range_x = np.max(mag_x) - np.min(mag_x)
    range_y = np.max(mag_y) - np.min(mag_y)

    # Calculate norm
    norm = np.sqrt(mag_x**2 + mag_y**2 + mag_z**2)

    print(f"\n=== Magnetometer Statistics ===")
    print(f"Samples: {len(mag_df)}")
    print(f"X: min={np.min(mag_x):.1f}, max={np.max(mag_x):.1f}, center={center_x:.1f}, range={range_x:.1f}")
    print(f"Y: min={np.min(mag_y):.1f}, max={np.max(mag_y):.1f}, center={center_y:.1f}, range={range_y:.1f}")
    print(f"Z: min={np.min(mag_z):.1f}, max={np.max(mag_z):.1f}, mean={np.mean(mag_z):.1f}")
    print(f"Norm: min={np.min(norm):.1f}, max={np.max(norm):.1f}, mean={np.mean(norm):.1f}, std={np.std(norm):.1f}")
    print(f"\nEstimated Hard Iron Offset (from min/max):")
    print(f"  X: {center_x:.1f} uT")
    print(f"  Y: {center_y:.1f} uT")

    # Create figure
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Plot 1: XY scatter with color by time
    ax1 = axes[0]
    timestamps = mag_df["timestamp_us"].to_numpy()
    t_relative = (timestamps - timestamps[0]) / 1e6  # timestamp_us -> seconds

    scatter = ax1.scatter(mag_x, mag_y, c=t_relative, cmap='viridis', s=10, alpha=0.7)
    plt.colorbar(scatter, ax=ax1, label='Time [s]')

    # Mark center
    ax1.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax1.axvline(x=0, color='gray', linestyle='--', alpha=0.5)
    ax1.plot(center_x, center_y, 'r+', markersize=15, markeredgewidth=2, label=f'Center ({center_x:.1f}, {center_y:.1f})')
    ax1.plot(0, 0, 'ko', markersize=8, label='Origin')

    # Draw estimated circle (if calibrated, should be centered at origin)
    radius = (range_x + range_y) / 4  # average radius
    theta = np.linspace(0, 2*np.pi, 100)
    ax1.plot(center_x + radius * np.cos(theta), center_y + radius * np.sin(theta),
             'r--', alpha=0.5, label=f'Fitted circle (r={radius:.1f})')

    ax1.set_xlabel('Mag X [uT]')
    ax1.set_ylabel('Mag Y [uT]')
    ax1.set_title('Magnetometer XY (colored by time)')
    ax1.legend(loc='upper right')
    ax1.axis('equal')
    ax1.grid(True, alpha=0.3)

    # Plot 2: Norm over time
    ax2 = axes[1]
    ax2.plot(t_relative, norm, 'b-', alpha=0.7, linewidth=0.5)
    ax2.axhline(y=np.mean(norm), color='r', linestyle='--', label=f'Mean: {np.mean(norm):.1f} uT')
    ax2.axhline(y=45, color='g', linestyle=':', label='Expected: ~45 uT')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Mag Norm [uT]')
    ax2.set_title('Magnetometer Norm over Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    # Save-only when an output file is given (no blocking window); otherwise
    # open the interactive window.
    # 出力ファイル指定時は保存だけ（ウィンドウで止めない）。未指定なら
    # 対話ウィンドウを開く。
    if output_file:
        plt.savefig(output_file, dpi=150)
        print(f"\nPlot saved to: {output_file}")
        plt.close()
        return

    plt.show()


def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_mag_xy.py <bundle>")
        print("       python plot_mag_xy.py <bundle> <output.png>")
        print("<bundle> is a StampFly flight-log v1 bundle: a .sflog.zip file")
        print("or an extracted directory (see lib/sflog).")
        sys.exit(1)

    input_path = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    if not Path(input_path).exists():
        print(f"Error: bundle not found: {input_path}")
        sys.exit(1)

    print(f"Loading: {input_path}")
    try:
        mag_df = load_mag_bundle(input_path)
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    print(f"Loaded {len(mag_df)} samples")

    if len(mag_df) == 0:
        print("Error: No valid data found")
        sys.exit(1)

    plot_mag_xy(mag_df, output_file)


if __name__ == '__main__':
    main()
