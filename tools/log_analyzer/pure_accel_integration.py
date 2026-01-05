#!/usr/bin/env python3
"""
Pure accelerometer integration for position estimation visualization.
Applies coordinate transformation to show drift without ESKF correction.
"""

import numpy as np
import matplotlib.pyplot as plt
import struct
import os

# V2 Packet constants (128 bytes, header 0xAA 0x56)
PACKET_SIZE = 128
PACKET_HEADER = bytes([0xAA, 0x56])
PACKET_FORMAT = '<2sI6f3f2f2f2hB3f3f3f3fB15sB'


def load_sensor_data(filename):
    """Load sensor data from V2 binary log file"""
    data = []

    with open(filename, 'rb') as f:
        content = f.read()

    print(f"Loading V2 format ({PACKET_SIZE} bytes/packet)")

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
                timestamp = unpacked[1]
                accel = np.array([unpacked[2], unpacked[3], unpacked[4]])
                gyro = np.array([unpacked[5], unpacked[6], unpacked[7]])

                data.append({
                    'timestamp': timestamp,
                    'accel': accel,
                    'gyro': gyro
                })
            except:
                pass
            i += PACKET_SIZE
        else:
            i += 1

    return data


def quaternion_multiply(q1, q2):
    """Quaternion multiplication"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def quaternion_to_rotation_matrix(q):
    """Compute rotation matrix from quaternion"""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])


def integrate_pure_accel(sensor_data):
    """Estimate position using pure accelerometer integration"""
    # Gravity vector (NED: Z-axis is down = gravity is -9.81)
    # Accelerometer measures force opposing gravity, so at rest measures +9.81
    # Correction: subtract -9.81
    gravity = np.array([0, 0, -9.81])

    # Initial state
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Unit quaternion
    position = np.array([0.0, 0.0, 0.0])
    velocity = np.array([0.0, 0.0, 0.0])

    # Results storage
    results = {
        'time': [],
        'position': [],
        'velocity': [],
        'attitude': [],  # roll, pitch, yaw
        'accel_world': [],
        'accel_body': []
    }

    prev_timestamp = None

    for sample in sensor_data:
        timestamp = sample['timestamp'] / 1000.0  # ms -> s
        accel = sample['accel']
        gyro = sample['gyro']

        if prev_timestamp is None:
            prev_timestamp = timestamp
            results['time'].append(timestamp)
            results['position'].append(position.copy())
            results['velocity'].append(velocity.copy())
            results['attitude'].append(np.array([0.0, 0.0, 0.0]))
            results['accel_world'].append(np.array([0.0, 0.0, 0.0]))
            results['accel_body'].append(accel.copy())
            continue

        dt = timestamp - prev_timestamp
        if dt <= 0 or dt > 0.1:  # Abnormal dt
            prev_timestamp = timestamp
            continue

        # Update attitude using gyro
        omega = np.linalg.norm(gyro)
        if omega > 1e-6:
            axis = gyro / omega
            angle = omega * dt
            dq = np.array([
                np.cos(angle / 2),
                axis[0] * np.sin(angle / 2),
                axis[1] * np.sin(angle / 2),
                axis[2] * np.sin(angle / 2)
            ])
            quaternion = quaternion_multiply(quaternion, dq)
            quaternion /= np.linalg.norm(quaternion)

        # Compute rotation matrix
        R = quaternion_to_rotation_matrix(quaternion)

        # Transform body acceleration to world frame
        accel_world = R @ accel

        # Remove gravity
        accel_world_nograv = accel_world - gravity

        # Integrate velocity and position
        velocity += accel_world_nograv * dt
        position += velocity * dt

        # Convert attitude to Euler angles
        # Roll
        roll = np.arctan2(2*(quaternion[0]*quaternion[1] + quaternion[2]*quaternion[3]),
                         1 - 2*(quaternion[1]**2 + quaternion[2]**2))
        # Pitch
        sinp = 2*(quaternion[0]*quaternion[2] - quaternion[3]*quaternion[1])
        pitch = np.arcsin(np.clip(sinp, -1, 1))
        # Yaw
        yaw = np.arctan2(2*(quaternion[0]*quaternion[3] + quaternion[1]*quaternion[2]),
                        1 - 2*(quaternion[2]**2 + quaternion[3]**2))

        # Store results
        results['time'].append(timestamp)
        results['position'].append(position.copy())
        results['velocity'].append(velocity.copy())
        results['attitude'].append(np.array([roll, pitch, yaw]))
        results['accel_world'].append(accel_world_nograv.copy())
        results['accel_body'].append(accel.copy())

        prev_timestamp = timestamp

    # Convert to numpy arrays
    for key in results:
        results[key] = np.array(results[key])

    return results


def main():
    # Data file path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_file = os.path.join(script_dir, 'flow01.bin')

    print(f"Loading sensor data from {data_file}...")
    sensor_data = load_sensor_data(data_file)
    print(f"Loaded {len(sensor_data)} samples")

    # Check timestamps
    if len(sensor_data) > 1:
        ts_diff = sensor_data[1]['timestamp'] - sensor_data[0]['timestamp']
        print(f"First timestamp diff: {ts_diff} ms")

    print("Integrating accelerometer data...")
    results = integrate_pure_accel(sensor_data)

    if len(results['time']) < 2:
        print("Error: Not enough valid samples")
        return

    # Results summary
    print("\n=== Pure Accelerometer Integration Results ===")
    print(f"Valid samples: {len(results['time'])}")
    print(f"Total time: {results['time'][-1] - results['time'][0]:.2f} s")
    print(f"Final position: X={results['position'][-1, 0]*100:.1f} cm, "
          f"Y={results['position'][-1, 1]*100:.1f} cm, "
          f"Z={results['position'][-1, 2]*100:.1f} cm")
    print(f"Final velocity: Vx={results['velocity'][-1, 0]:.2f} m/s, "
          f"Vy={results['velocity'][-1, 1]:.2f} m/s, "
          f"Vz={results['velocity'][-1, 2]:.2f} m/s")
    print(f"World acceleration mean: Ax={results['accel_world'][:, 0].mean():.4f}, "
          f"Ay={results['accel_world'][:, 1].mean():.4f}, "
          f"Az={results['accel_world'][:, 2].mean():.4f} m/s^2")

    # Visualization
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('Pure Accelerometer Integration Results\n(with coordinate transformation, without ESKF)', fontsize=14)

    time = results['time'] - results['time'][0]

    # 1. Position
    ax = axes[0, 0]
    ax.plot(time, results['position'][:, 0] * 100, 'r-', label='X', linewidth=1)
    ax.plot(time, results['position'][:, 1] * 100, 'g-', label='Y', linewidth=1)
    ax.plot(time, results['position'][:, 2] * 100, 'b-', label='Z', linewidth=1)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Position [cm]')
    ax.set_title('Position (massive drift due to accel bias)')
    ax.legend()
    ax.grid(True)

    # 2. Velocity
    ax = axes[0, 1]
    ax.plot(time, results['velocity'][:, 0], 'r-', label='Vx', linewidth=1)
    ax.plot(time, results['velocity'][:, 1], 'g-', label='Vy', linewidth=1)
    ax.plot(time, results['velocity'][:, 2], 'b-', label='Vz', linewidth=1)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Velocity [m/s]')
    ax.set_title('Velocity (accumulating due to accel bias)')
    ax.legend()
    ax.grid(True)

    # 3. Attitude
    ax = axes[1, 0]
    ax.plot(time, np.degrees(results['attitude'][:, 0]), 'r-', label='Roll', linewidth=1)
    ax.plot(time, np.degrees(results['attitude'][:, 1]), 'g-', label='Pitch', linewidth=1)
    ax.plot(time, np.degrees(results['attitude'][:, 2]), 'b-', label='Yaw', linewidth=1)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angle [deg]')
    ax.set_title('Attitude (from gyro integration only)')
    ax.legend()
    ax.grid(True)

    # 4. World-frame acceleration
    ax = axes[1, 1]
    ax.plot(time, results['accel_world'][:, 0], 'r-', alpha=0.5, label='Ax', linewidth=0.5)
    ax.plot(time, results['accel_world'][:, 1], 'g-', alpha=0.5, label='Ay', linewidth=0.5)
    ax.plot(time, results['accel_world'][:, 2], 'b-', alpha=0.5, label='Az', linewidth=0.5)
    ax.axhline(y=results['accel_world'][:, 0].mean(), color='r', linestyle='--',
               label=f'Ax mean={results["accel_world"][:, 0].mean():.3f}')
    ax.axhline(y=results['accel_world'][:, 1].mean(), color='g', linestyle='--',
               label=f'Ay mean={results["accel_world"][:, 1].mean():.3f}')
    ax.axhline(y=results['accel_world'][:, 2].mean(), color='b', linestyle='--',
               label=f'Az mean={results["accel_world"][:, 2].mean():.3f}')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Acceleration [m/s^2]')
    ax.set_title('World-frame Acceleration (gravity removed)')
    ax.legend(fontsize=8)
    ax.grid(True)

    # 5. XY trajectory
    ax = axes[2, 0]
    ax.plot(results['position'][:, 0] * 100, results['position'][:, 1] * 100, 'b-', linewidth=1)
    ax.plot(0, 0, 'go', markersize=10, label='Start')
    ax.plot(results['position'][-1, 0] * 100, results['position'][-1, 1] * 100, 'ro',
            markersize=10, label='End')
    ax.set_xlabel('X [cm]')
    ax.set_ylabel('Y [cm]')
    ax.set_title('XY Trajectory (top view)')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')

    # 6. Body-frame acceleration
    ax = axes[2, 1]
    ax.plot(time, results['accel_body'][:, 0], 'r-', alpha=0.5, label='Ax_body', linewidth=0.5)
    ax.plot(time, results['accel_body'][:, 1], 'g-', alpha=0.5, label='Ay_body', linewidth=0.5)
    ax.plot(time, results['accel_body'][:, 2], 'b-', alpha=0.5, label='Az_body', linewidth=0.5)
    ax.axhline(y=results['accel_body'][:, 0].mean(), color='r', linestyle='--',
               label=f'mean={results["accel_body"][:, 0].mean():.3f}')
    ax.axhline(y=results['accel_body'][:, 1].mean(), color='g', linestyle='--',
               label=f'mean={results["accel_body"][:, 1].mean():.3f}')
    ax.axhline(y=results['accel_body'][:, 2].mean(), color='b', linestyle='--',
               label=f'mean={results["accel_body"][:, 2].mean():.3f}')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Acceleration [m/s^2]')
    ax.set_title('Body-frame Acceleration (raw sensor)')
    ax.legend(fontsize=8)
    ax.grid(True)

    plt.tight_layout()

    # Save
    output_file = os.path.join(script_dir, 'pure_accel_integration.png')
    plt.savefig(output_file, dpi=150)
    print(f"\nSaved visualization to {output_file}")

    plt.show()

if __name__ == '__main__':
    main()
