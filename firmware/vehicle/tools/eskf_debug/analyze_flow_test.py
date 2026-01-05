#!/usr/bin/env python3
"""
Flow sensor calibration analysis tool
Analyzes raw flow data from binary logs to determine axis mapping and scale
"""

import struct
import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# V2 packet format (128 bytes) - matches replay.cpp BinaryLogPacketV2
V2_PACKET_SIZE = 128

def load_binary_log(filepath):
    """Load V2 binary log file"""
    packets = []
    with open(filepath, 'rb') as f:
        while True:
            data = f.read(V2_PACKET_SIZE)
            if len(data) < V2_PACKET_SIZE:
                break

            # Check header
            if data[0] != 0xAA or data[1] != 0x56:
                continue

            # Parse V2 packet (matching BinaryLogPacketV2 in replay.cpp)
            # header[2], timestamp_ms(4), accel(12), gyro(12), mag(12)
            # pressure(4), baro_alt(4), tof_bottom(4), tof_front(4)
            # flow_dx(2), flow_dy(2), flow_squal(1)
            # pos(12), vel(12), roll/pitch/yaw(12)
            # gyro_bias_z(4), accel_bias_x(4), accel_bias_y(4)
            # eskf_status(1), baro_ref_alt(4), reserved(11), checksum(1)

            offset = 2  # Skip header
            timestamp_ms = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            accel = struct.unpack('<fff', data[offset:offset+12])
            offset += 12

            gyro = struct.unpack('<fff', data[offset:offset+12])
            offset += 12

            mag = struct.unpack('<fff', data[offset:offset+12])
            offset += 12

            pressure = struct.unpack('<f', data[offset:offset+4])[0]
            offset += 4

            baro_alt = struct.unpack('<f', data[offset:offset+4])[0]
            offset += 4

            tof_bottom = struct.unpack('<f', data[offset:offset+4])[0]
            offset += 4

            tof_front = struct.unpack('<f', data[offset:offset+4])[0]
            offset += 4

            flow_dx, flow_dy = struct.unpack('<hh', data[offset:offset+4])
            offset += 4

            flow_squal = data[offset]
            offset += 1

            pos = struct.unpack('<fff', data[offset:offset+12])
            offset += 12

            vel = struct.unpack('<fff', data[offset:offset+12])
            offset += 12

            rpy = struct.unpack('<fff', data[offset:offset+12])
            offset += 12

            gyro_bias_z = struct.unpack('<f', data[offset:offset+4])[0]
            offset += 4

            accel_bias = struct.unpack('<ff', data[offset:offset+8])
            offset += 8

            packet = {
                'timestamp': timestamp_ms,
                'gyro_x': gyro[0], 'gyro_y': gyro[1], 'gyro_z': gyro[2],
                'accel_x': accel[0], 'accel_y': accel[1], 'accel_z': accel[2],
                'mag_x': mag[0], 'mag_y': mag[1], 'mag_z': mag[2],
                'baro_alt': baro_alt,
                'tof_dist': tof_bottom,
                'flow_dx': flow_dx,  # Raw counts
                'flow_dy': flow_dy,  # Raw counts
                'flow_squal': flow_squal,
                'pos_x': pos[0], 'pos_y': pos[1], 'pos_z': pos[2],
                'vel_x': vel[0], 'vel_y': vel[1], 'vel_z': vel[2],
                'roll': rpy[0], 'pitch': rpy[1], 'yaw': rpy[2],
                'gyro_bias_z': gyro_bias_z,
            }
            packets.append(packet)
    return packets

def analyze_flow_data(packets, test_name="Flow Test"):
    """Analyze flow sensor data"""

    if len(packets) == 0:
        print("No packets loaded!")
        return None

    # Extract time series
    t0 = packets[0]['timestamp']
    time = np.array([(p['timestamp'] - t0) / 1000.0 for p in packets])

    # Raw flow counts
    flow_dx = np.array([p['flow_dx'] for p in packets])
    flow_dy = np.array([p['flow_dy'] for p in packets])
    flow_squal = np.array([p['flow_squal'] for p in packets])

    gyro_x = np.array([p['gyro_x'] for p in packets])
    gyro_y = np.array([p['gyro_y'] for p in packets])
    gyro_z = np.array([p['gyro_z'] for p in packets])

    height = np.array([p['tof_dist'] for p in packets])

    pos_x = np.array([p['pos_x'] for p in packets])
    pos_y = np.array([p['pos_y'] for p in packets])

    roll = np.array([p['roll'] for p in packets])
    pitch = np.array([p['pitch'] for p in packets])
    yaw = np.array([p['yaw'] for p in packets])

    # Flow scale (counts to radians)
    FLOW_SCALE = 0.16  # rad/count (from ESKF code)
    flow_x_rad = flow_dx * FLOW_SCALE
    flow_y_rad = flow_dy * FLOW_SCALE

    # Average height (valid readings only)
    valid_height = height[(height > 0.05) & (height < 2.0)]
    avg_height = np.mean(valid_height) if len(valid_height) > 0 else 0.3

    # Integrate flow to position
    # velocity = flow_rad * height / dt, but flow is angle per sample
    # Simple approximation: position = cumsum(flow_rad) * height
    flow_x_cum = np.cumsum(flow_x_rad) * avg_height
    flow_y_cum = np.cumsum(flow_y_rad) * avg_height

    # Statistics
    print(f"\n{'='*60}")
    print(f"=== {test_name} Analysis ===")
    print(f"{'='*60}")
    print(f"Duration: {time[-1]:.2f} s")
    print(f"Samples: {len(packets)}")
    print(f"Sample rate: {len(packets)/time[-1]:.1f} Hz")
    print(f"Average height: {avg_height*100:.1f} cm")
    print(f"Height range: {np.min(valid_height)*100:.1f} - {np.max(valid_height)*100:.1f} cm")

    print(f"\n--- Raw Flow (counts) ---")
    print(f"  flow_dx: sum={np.sum(flow_dx):6d}, mean={np.mean(flow_dx):7.2f}, std={np.std(flow_dx):7.2f}")
    print(f"  flow_dy: sum={np.sum(flow_dy):6d}, mean={np.mean(flow_dy):7.2f}, std={np.std(flow_dy):7.2f}")
    print(f"  quality: mean={np.mean(flow_squal):.1f}, min={np.min(flow_squal)}, max={np.max(flow_squal)}")

    print(f"\n--- Integrated Position (flow × scale × height) ---")
    print(f"  From flow_dx: {flow_x_cum[-1]*100:+.1f} cm")
    print(f"  From flow_dy: {flow_y_cum[-1]*100:+.1f} cm")

    print(f"\n--- Device ESKF Position ---")
    print(f"  X: {pos_x[0]*100:.1f} → {pos_x[-1]*100:.1f} cm (Δ={( pos_x[-1]-pos_x[0])*100:+.1f} cm)")
    print(f"  Y: {pos_y[0]*100:.1f} → {pos_y[-1]*100:.1f} cm (Δ={( pos_y[-1]-pos_y[0])*100:+.1f} cm)")

    print(f"\n--- Gyro Statistics ---")
    print(f"  gyro_x: mean={np.mean(gyro_x)*180/np.pi:+.3f} deg/s")
    print(f"  gyro_y: mean={np.mean(gyro_y)*180/np.pi:+.3f} deg/s")
    print(f"  gyro_z: mean={np.mean(gyro_z)*180/np.pi:+.3f} deg/s")

    print(f"\n--- Attitude (Device) ---")
    print(f"  Roll:  {np.min(roll)*180/np.pi:.1f}° to {np.max(roll)*180/np.pi:.1f}°")
    print(f"  Pitch: {np.min(pitch)*180/np.pi:.1f}° to {np.max(pitch)*180/np.pi:.1f}°")
    print(f"  Yaw:   {np.min(yaw)*180/np.pi:.1f}° to {np.max(yaw)*180/np.pi:.1f}°")

    # Create visualization
    fig, axes = plt.subplots(4, 2, figsize=(14, 12))
    fig.suptitle(f'{test_name}\nHeight: {avg_height*100:.0f}cm, Duration: {time[-1]:.1f}s', fontsize=14)

    # Row 1: Raw flow counts
    axes[0, 0].plot(time, flow_dx, 'b-', label='flow_dx', alpha=0.7)
    axes[0, 0].plot(time, flow_dy, 'r-', label='flow_dy', alpha=0.7)
    axes[0, 0].set_ylabel('Flow [counts]')
    axes[0, 0].set_title('Raw Flow Data (counts)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # Row 1: Cumulative flow position
    axes[0, 1].plot(time, flow_x_cum * 100, 'b-', label=f'∫flow_dx → {flow_x_cum[-1]*100:+.1f}cm')
    axes[0, 1].plot(time, flow_y_cum * 100, 'r-', label=f'∫flow_dy → {flow_y_cum[-1]*100:+.1f}cm')
    axes[0, 1].set_ylabel('Position [cm]')
    axes[0, 1].set_title('Integrated Flow Position')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # Row 2: Gyro data
    axes[1, 0].plot(time, np.rad2deg(gyro_x), 'b-', label='gyro_x', alpha=0.7)
    axes[1, 0].plot(time, np.rad2deg(gyro_y), 'r-', label='gyro_y', alpha=0.7)
    axes[1, 0].plot(time, np.rad2deg(gyro_z), 'g-', label='gyro_z', alpha=0.7)
    axes[1, 0].set_ylabel('Angular rate [deg/s]')
    axes[1, 0].set_title('Gyroscope Data')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Row 2: Height and quality
    ax2r = axes[1, 1].twinx()
    axes[1, 1].plot(time, height * 100, 'k-', label='Height')
    ax2r.plot(time, flow_squal, 'g-', alpha=0.5, label='Quality')
    axes[1, 1].set_ylabel('Height [cm]')
    ax2r.set_ylabel('Flow Quality', color='g')
    axes[1, 1].set_title('ToF Height & Flow Quality')
    axes[1, 1].grid(True, alpha=0.3)

    # Row 3: Device ESKF position
    axes[2, 0].plot(time, pos_x * 100, 'b-', label=f'X → {pos_x[-1]*100:.1f}cm')
    axes[2, 0].plot(time, pos_y * 100, 'r-', label=f'Y → {pos_y[-1]*100:.1f}cm')
    axes[2, 0].set_ylabel('Position [cm]')
    axes[2, 0].set_title('Device ESKF Position')
    axes[2, 0].legend()
    axes[2, 0].grid(True, alpha=0.3)

    # Row 3: Attitude
    axes[2, 1].plot(time, np.rad2deg(roll), 'b-', label='Roll', alpha=0.7)
    axes[2, 1].plot(time, np.rad2deg(pitch), 'r-', label='Pitch', alpha=0.7)
    axes[2, 1].plot(time, np.rad2deg(yaw), 'g-', label='Yaw', alpha=0.7)
    axes[2, 1].set_ylabel('Angle [deg]')
    axes[2, 1].set_title('Device Attitude')
    axes[2, 1].legend()
    axes[2, 1].grid(True, alpha=0.3)

    # Row 4: Trajectory comparison
    axes[3, 0].plot(pos_y * 100, pos_x * 100, 'g-', linewidth=2, label='Device ESKF')
    axes[3, 0].plot(flow_y_cum * 100, flow_x_cum * 100, 'b--', linewidth=1.5, label='Flow integrated')
    axes[3, 0].plot(pos_y[0] * 100, pos_x[0] * 100, 'ko', markersize=10, label='Start')
    axes[3, 0].plot(pos_y[-1] * 100, pos_x[-1] * 100, 'kx', markersize=10, label='End')
    axes[3, 0].set_xlabel('Y [cm] - Right')
    axes[3, 0].set_ylabel('X [cm] - Forward')
    axes[3, 0].set_title('Trajectory (Top View)')
    axes[3, 0].legend()
    axes[3, 0].grid(True, alpha=0.3)
    axes[3, 0].axis('equal')

    # Row 4: Flow distribution
    axes[3, 1].hist(flow_dx, bins=50, alpha=0.5, label=f'flow_dx (sum={np.sum(flow_dx)})')
    axes[3, 1].hist(flow_dy, bins=50, alpha=0.5, label=f'flow_dy (sum={np.sum(flow_dy)})')
    axes[3, 1].set_xlabel('Flow [counts]')
    axes[3, 1].set_ylabel('Count')
    axes[3, 1].set_title('Flow Distribution')
    axes[3, 1].legend()
    axes[3, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    return fig

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_flow_test.py <log.bin> [output.png]")
        sys.exit(1)

    input_file = Path(sys.argv[1])
    output_file = sys.argv[2] if len(sys.argv) > 2 else input_file.stem + "_analysis.png"

    print(f"Loading {input_file}...")
    packets = load_binary_log(input_file)
    print(f"Loaded {len(packets)} packets")

    if len(packets) == 0:
        print("Error: No valid packets found!")
        sys.exit(1)

    fig = analyze_flow_data(packets, test_name=input_file.stem)

    if fig:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"\nSaved: {output_file}")
        plt.show()

if __name__ == "__main__":
    main()
