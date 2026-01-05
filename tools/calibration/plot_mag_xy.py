#!/usr/bin/env python3
"""
Plot Magnetometer XY data from binary log to verify calibration.

Usage:
    python plot_mag_xy.py <binlog_file>

The plot should show points forming a circle centered at origin
if calibration is correct.
"""

import sys
import struct
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


# V2 Packet constants (128 bytes, header 0xAA 0x56)
PACKET_SIZE = 128
PACKET_HEADER = bytes([0xAA, 0x56])
PACKET_FORMAT = '<2sI6f3f2f2f2hB3f3f3f3fB15sB'


def load_binlog(filename):
    """Load V2 binary log (128 bytes, header 0xAA 0x56)"""
    data = []
    with open(filename, 'rb') as f:
        content = f.read()

    i = 0
    while i < len(content) - PACKET_SIZE + 1:
        # Find header
        if content[i] == 0xAA and content[i+1] == 0x56:
            packet = content[i:i+PACKET_SIZE]

            # Verify checksum
            checksum = 0
            for b in packet[2:127]:
                checksum ^= b
            if checksum != packet[127]:
                i += 1
                continue

            # Parse packet
            try:
                unpacked = struct.unpack(PACKET_FORMAT, packet)
                timestamp_ms = unpacked[1]
                mag_x = unpacked[8]
                mag_y = unpacked[9]
                mag_z = unpacked[10]

                data.append({
                    'timestamp_ms': timestamp_ms,
                    'mag_x': mag_x,
                    'mag_y': mag_y,
                    'mag_z': mag_z
                })
            except:
                pass
            i += PACKET_SIZE
        else:
            i += 1

    return data


def plot_mag_xy(data, output_file=None):
    """Plot magnetometer XY data"""
    mag_x = np.array([d['mag_x'] for d in data])
    mag_y = np.array([d['mag_y'] for d in data])
    mag_z = np.array([d['mag_z'] for d in data])

    # Calculate statistics
    center_x = (np.max(mag_x) + np.min(mag_x)) / 2
    center_y = (np.max(mag_y) + np.min(mag_y)) / 2
    range_x = np.max(mag_x) - np.min(mag_x)
    range_y = np.max(mag_y) - np.min(mag_y)

    # Calculate norm
    norm = np.sqrt(mag_x**2 + mag_y**2 + mag_z**2)

    print(f"\n=== Magnetometer Statistics ===")
    print(f"Samples: {len(data)}")
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
    timestamps = np.array([d['timestamp_ms'] for d in data])
    t_relative = (timestamps - timestamps[0]) / 1000.0  # seconds

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

    if output_file:
        plt.savefig(output_file, dpi=150)
        print(f"\nPlot saved to: {output_file}")

    plt.show()


def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_mag_xy.py <binlog_file>")
        print("       python plot_mag_xy.py <binlog_file> <output.png>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    if not Path(input_file).exists():
        print(f"Error: File not found: {input_file}")
        sys.exit(1)

    print(f"Loading: {input_file}")
    print(f"Packet format: V2 ({PACKET_SIZE} bytes)")
    data = load_binlog(input_file)
    print(f"Loaded {len(data)} samples")

    if len(data) == 0:
        print("Error: No valid data found")
        sys.exit(1)

    plot_mag_xy(data, output_file)


if __name__ == '__main__':
    main()
