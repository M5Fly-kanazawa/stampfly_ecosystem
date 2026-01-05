#!/usr/bin/env python3
"""
ESKF Parameter Tuning Script
Reads binary log, runs ESKF with different parameters, finds best match to 20cm square.
"""
import struct
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple
import sys

# Constants
GRAVITY = 9.81

@dataclass
class SensorData:
    timestamp_ms: int
    accel: np.ndarray  # [x, y, z]
    gyro: np.ndarray   # [x, y, z]
    mag: np.ndarray    # [x, y, z]
    baro_alt: float
    tof_bottom: float
    flow_dx: int
    flow_dy: int
    flow_squal: int
    # Device ESKF (V2 only)
    dev_pos: np.ndarray = None
    dev_vel: np.ndarray = None
    dev_rpy: np.ndarray = None
    gyro_bias_z: float = 0.0
    accel_bias_xy: np.ndarray = None

def load_binary_log(filename: str) -> Tuple[List[SensorData], int, float]:
    """Load binary log file, return packets, version, and baro_ref"""
    packets = []
    
    with open(filename, 'rb') as f:
        header = f.read(2)
        f.seek(0)
        
        if header == b'\xaa\x55':
            version = 1
            packet_size = 64
        elif header == b'\xaa\x56':
            version = 2
            packet_size = 128
        else:
            raise ValueError(f"Unknown log format: {header.hex()}")
        
        baro_ref = None
        
        while True:
            data = f.read(packet_size)
            if len(data) < packet_size:
                break
            
            if version == 1:
                # V1: 64 bytes
                fields = struct.unpack('<2sI9f2fhh2B', data[:62])
                pkt = SensorData(
                    timestamp_ms=fields[1],
                    accel=np.array([fields[2], fields[3], fields[4]]),
                    gyro=np.array([fields[5], fields[6], fields[7]]),
                    mag=np.array([fields[8], fields[9], fields[10]]),
                    baro_alt=fields[12],
                    tof_bottom=fields[13],
                    flow_dx=fields[15],
                    flow_dy=fields[16],
                    flow_squal=fields[17]
                )
            else:
                # V2: 128 bytes
                # 2s: header, I: timestamp,
                # 3f: accel, 3f: gyro, 3f: mag, f: pressure, f: baro_alt,
                # f: tof_bottom, f: tof_front, h: flow_dx, h: flow_dy, B: flow_squal,
                # 3f: pos, 3f: vel, 3f: rpy, f: gyro_bias_z, 2f: accel_bias_xy,
                # B: eskf_status, f: baro_ref_alt, 11s: reserved, B: checksum
                fmt = '<2s I 3f 3f 3f f f f f hh B 3f 3f 3f f 2f B f 11s B'
                fields = struct.unpack(fmt, data)
                # Indices after unpacking:
                # 0: header, 1: timestamp
                # 2,3,4: accel, 5,6,7: gyro, 8,9,10: mag
                # 11: pressure, 12: baro_alt, 13: tof_bottom, 14: tof_front
                # 15: flow_dx, 16: flow_dy, 17: flow_squal
                # 18,19,20: pos, 21,22,23: vel, 24,25,26: rpy
                # 27: gyro_bias_z, 28,29: accel_bias_xy
                # 30: eskf_status, 31: baro_ref_alt, 32: reserved, 33: checksum
                pkt = SensorData(
                    timestamp_ms=fields[1],
                    accel=np.array([fields[2], fields[3], fields[4]]),
                    gyro=np.array([fields[5], fields[6], fields[7]]),
                    mag=np.array([fields[8], fields[9], fields[10]]),
                    baro_alt=fields[12],
                    tof_bottom=fields[13],
                    flow_dx=fields[15],
                    flow_dy=fields[16],
                    flow_squal=fields[17],
                    dev_pos=np.array([fields[18], fields[19], fields[20]]),
                    dev_vel=np.array([fields[21], fields[22], fields[23]]),
                    dev_rpy=np.array([fields[24], fields[25], fields[26]]),
                    gyro_bias_z=fields[27],
                    accel_bias_xy=np.array([fields[28], fields[29]])
                )
                if baro_ref is None:
                    baro_ref = fields[31]  # baro_ref_alt
            
            packets.append(pkt)
    
    if baro_ref is None:
        baro_ref = packets[0].baro_alt if packets else 0.0
    
    return packets, version, baro_ref

class SimpleESKF:
    """Simplified ESKF for parameter tuning"""
    def __init__(self, flow_scale=0.16, flow_noise=0.1, gyro_bias_z=0.0, accel_bias_xy=None):
        self.flow_scale = flow_scale
        self.flow_noise = flow_noise
        
        # State
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        
        # Biases
        self.gyro_bias = np.array([0.0, 0.0, gyro_bias_z])
        self.accel_bias = np.array([accel_bias_xy[0] if accel_bias_xy is not None else 0.0,
                                     accel_bias_xy[1] if accel_bias_xy is not None else 0.0,
                                     0.0])
        
        # Gyro compensation coefficients
        self.k_xx = 1.35 * flow_scale
        self.k_xy = 9.30 * flow_scale
        self.k_yx = -2.65 * flow_scale
        self.k_yy = 0.0
        
        # Mag reference
        self.mag_ref = None
        self.mag_ref_samples = []
        
        # Simple velocity Kalman filter
        self.vel_cov = np.eye(2) * 0.1
        
    def predict(self, accel, gyro, dt):
        """IMU prediction step"""
        # Bias correction
        gyro_corr = gyro - self.gyro_bias
        accel_corr = accel - self.accel_bias
        
        # Update attitude (simple integration)
        self.roll += gyro_corr[0] * dt
        self.pitch += gyro_corr[1] * dt
        self.yaw += gyro_corr[2] * dt
        
        # Rotation matrix (yaw only for simplicity)
        c_yaw = np.cos(self.yaw)
        s_yaw = np.sin(self.yaw)
        c_roll = np.cos(self.roll)
        s_roll = np.sin(self.roll)
        c_pitch = np.cos(self.pitch)
        s_pitch = np.sin(self.pitch)
        
        # Full rotation matrix
        R = np.array([
            [c_yaw*c_pitch, c_yaw*s_pitch*s_roll - s_yaw*c_roll, c_yaw*s_pitch*c_roll + s_yaw*s_roll],
            [s_yaw*c_pitch, s_yaw*s_pitch*s_roll + c_yaw*c_roll, s_yaw*s_pitch*c_roll - c_yaw*s_roll],
            [-s_pitch, c_pitch*s_roll, c_pitch*c_roll]
        ])
        
        # World frame acceleration
        accel_world = R @ accel_corr
        accel_world[2] += GRAVITY  # Remove gravity
        
        # Velocity and position update
        self.vel += accel_world * dt
        self.pos += self.vel * dt
        
    def update_flow(self, flow_dx, flow_dy, height, gyro):
        """Flow measurement update"""
        if height < 0.02:
            return
            
        # Convert to radians
        flow_x = flow_dx * self.flow_scale
        flow_y = flow_dy * self.flow_scale
        
        # Gyro compensation
        flow_x_comp = flow_x - self.k_xx * gyro[0] - self.k_xy * gyro[1]
        flow_y_comp = flow_y - self.k_yx * gyro[0] - self.k_yy * gyro[1]
        
        # Body velocity
        vx_body = flow_x_comp * height
        vy_body = flow_y_comp * height
        
        # Body to NED
        c_yaw = np.cos(self.yaw)
        s_yaw = np.sin(self.yaw)
        vx_ned = c_yaw * vx_body - s_yaw * vy_body
        vy_ned = s_yaw * vx_body + c_yaw * vy_body
        
        # Simple Kalman update for velocity
        z = np.array([vx_ned, vy_ned])
        h = self.vel[:2]
        R = np.eye(2) * (self.flow_noise ** 2)
        
        S = self.vel_cov + R
        K = self.vel_cov @ np.linalg.inv(S)
        
        innovation = z - h
        self.vel[:2] += K @ innovation
        self.vel_cov = (np.eye(2) - K) @ self.vel_cov
        
        # Position update from velocity
        # (simplified: trust flow velocity directly for position)
        
    def update_mag(self, mag):
        """Mag update for yaw"""
        if self.mag_ref is None:
            self.mag_ref_samples.append(mag.copy())
            if len(self.mag_ref_samples) >= 10:
                self.mag_ref = np.mean(self.mag_ref_samples, axis=0)
            return
            
        # Expected mag based on current yaw
        c_yaw = np.cos(self.yaw)
        s_yaw = np.sin(self.yaw)
        mag_expected = np.array([
            c_yaw * self.mag_ref[0] - s_yaw * self.mag_ref[1],
            s_yaw * self.mag_ref[0] + c_yaw * self.mag_ref[1],
            self.mag_ref[2]
        ])
        
        yaw_meas = np.arctan2(mag[1], mag[0])
        yaw_pred = np.arctan2(mag_expected[1], mag_expected[0])
        
        yaw_err = yaw_meas - yaw_pred
        while yaw_err > np.pi: yaw_err -= 2*np.pi
        while yaw_err < -np.pi: yaw_err += 2*np.pi
        
        # Simple correction
        self.yaw += 0.1 * yaw_err

def run_eskf(packets, flow_scale=0.16, flow_noise=0.1, flow_scale_x=None, flow_scale_y=None):
    """Run ESKF with given parameters, return trajectory"""
    if not packets:
        return [], []
    
    # Use separate scales if provided
    scale_x = flow_scale_x if flow_scale_x is not None else flow_scale
    scale_y = flow_scale_y if flow_scale_y is not None else flow_scale
    
    eskf = SimpleESKF(
        flow_scale=flow_scale,
        flow_noise=flow_noise,
        gyro_bias_z=packets[0].gyro_bias_z,
        accel_bias_xy=packets[0].accel_bias_xy
    )
    
    # Override scale for asymmetric test
    eskf.flow_scale_x = scale_x
    eskf.flow_scale_y = scale_y
    
    trajectory = []
    last_t = packets[0].timestamp_ms
    
    for i, pkt in enumerate(packets):
        dt = (pkt.timestamp_ms - last_t) / 1000.0
        if dt <= 0 or dt > 0.1:
            dt = 0.01
        last_t = pkt.timestamp_ms
        
        # Accel scaling (if in g units)
        accel = pkt.accel.copy()
        if abs(accel[2]) < 2.0:
            accel *= GRAVITY
            
        eskf.predict(accel, pkt.gyro, dt)
        
        # Flow update with separate scales
        if pkt.flow_squal >= 30 and pkt.tof_bottom > 0.02:
            flow_x = pkt.flow_dx * scale_x
            flow_y = pkt.flow_dy * scale_y
            
            gyro = pkt.gyro
            k_xx = 1.35 * scale_x
            k_xy = 9.30 * scale_x
            k_yx = -2.65 * scale_y
            
            flow_x_comp = flow_x - k_xx * gyro[0] - k_xy * gyro[1]
            flow_y_comp = flow_y - k_yx * gyro[0]
            
            height = max(pkt.tof_bottom, 0.02)
            vx_body = flow_x_comp * height
            vy_body = flow_y_comp * height
            
            c_yaw = np.cos(eskf.yaw)
            s_yaw = np.sin(eskf.yaw)
            vx_ned = c_yaw * vx_body - s_yaw * vy_body
            vy_ned = s_yaw * vx_body + c_yaw * vy_body
            
            # Simple velocity filter
            alpha = 0.3  # Flow weight
            eskf.vel[0] = alpha * vx_ned + (1 - alpha) * eskf.vel[0]
            eskf.vel[1] = alpha * vy_ned + (1 - alpha) * eskf.vel[1]
        
        # Mag update
        if i % 10 == 0:
            mag_norm = np.linalg.norm(pkt.mag)
            if mag_norm > 10:
                eskf.update_mag(pkt.mag)
        
        trajectory.append(eskf.pos.copy())
    
    return np.array(trajectory)

def evaluate_trajectory(traj, target_size=0.20):
    """Evaluate how close trajectory is to target square size"""
    if len(traj) < 10:
        return float('inf'), 0, 0
    
    x_range = traj[:, 0].max() - traj[:, 0].min()
    y_range = traj[:, 1].max() - traj[:, 1].min()
    
    # Error from target
    x_err = abs(x_range - target_size)
    y_err = abs(y_range - target_size)
    total_err = x_err + y_err
    
    return total_err, x_range, y_range

def main():
    if len(sys.argv) < 2:
        print("Usage: python tune_params.py <input.bin>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    print(f"Loading {input_file}...")
    packets, version, baro_ref = load_binary_log(input_file)
    print(f"Loaded {len(packets)} packets (V{version})")
    
    # Current parameters
    print("\n=== Current Parameters (flow_scale=0.16) ===")
    traj = run_eskf(packets, flow_scale=0.16)
    err, x_range, y_range = evaluate_trajectory(traj)
    print(f"X range: {x_range*100:.1f} cm, Y range: {y_range*100:.1f} cm")
    print(f"Target: 20 cm x 20 cm")
    print(f"Error: {err*100:.1f} cm")
    
    # Calculate required scales
    current_x = x_range
    current_y = y_range
    target = 0.20
    
    required_scale_x = 0.16 * (target / current_x) if current_x > 0 else 0.16
    required_scale_y = 0.16 * (target / current_y) if current_y > 0 else 0.16
    
    print(f"\n=== Required Scales to Match 20cm ===")
    print(f"Scale X: {required_scale_x:.4f} (current 0.16, ratio {required_scale_x/0.16:.2f})")
    print(f"Scale Y: {required_scale_y:.4f} (current 0.16, ratio {required_scale_y/0.16:.2f})")
    
    # Try different flow_scale values
    print("\n=== Grid Search: Uniform Scale ===")
    scales = [0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.35, 0.40]
    best_err = float('inf')
    best_scale = 0.16
    
    results = []
    for scale in scales:
        traj = run_eskf(packets, flow_scale=scale)
        err, x_range, y_range = evaluate_trajectory(traj)
        results.append((scale, x_range, y_range, err))
        print(f"scale={scale:.2f}: X={x_range*100:.1f}cm, Y={y_range*100:.1f}cm, err={err*100:.1f}cm")
        if err < best_err:
            best_err = err
            best_scale = scale
    
    print(f"\nBest uniform scale: {best_scale:.2f} (error={best_err*100:.1f}cm)")
    
    # Try asymmetric scales
    print("\n=== Grid Search: Asymmetric Scales ===")
    scale_x_range = np.arange(0.16, 0.30, 0.02)
    scale_y_range = np.arange(0.30, 0.60, 0.05)
    
    best_err_asym = float('inf')
    best_scale_x = 0.16
    best_scale_y = 0.16
    
    for sx in scale_x_range:
        for sy in scale_y_range:
            traj = run_eskf(packets, flow_scale_x=sx, flow_scale_y=sy)
            err, x_range, y_range = evaluate_trajectory(traj)
            if err < best_err_asym:
                best_err_asym = err
                best_scale_x = sx
                best_scale_y = sy
    
    print(f"Best asymmetric: scale_x={best_scale_x:.2f}, scale_y={best_scale_y:.2f}")
    traj = run_eskf(packets, flow_scale_x=best_scale_x, flow_scale_y=best_scale_y)
    err, x_range, y_range = evaluate_trajectory(traj)
    print(f"  X={x_range*100:.1f}cm, Y={y_range*100:.1f}cm, err={err*100:.1f}cm")
    
    # Visualize best results
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    # Current
    traj_current = run_eskf(packets, flow_scale=0.16)
    ax = axes[0]
    ax.plot(traj_current[:, 1]*100, traj_current[:, 0]*100, 'b-', label='PC')
    ax.plot(0, 0, 'ko', markersize=10)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.axhline(y=20, color='gray', linestyle='--', alpha=0.3)
    ax.axvline(x=0, color='gray', linestyle='--', alpha=0.3)
    ax.axvline(x=20, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Y (cm)')
    ax.set_ylabel('X (cm)')
    ax.set_title(f'Current (scale=0.16)\nX={traj_current[:,0].ptp()*100:.1f}cm, Y={traj_current[:,1].ptp()*100:.1f}cm')
    ax.axis('equal')
    ax.grid(True)
    
    # Best uniform
    traj_best = run_eskf(packets, flow_scale=best_scale)
    ax = axes[1]
    ax.plot(traj_best[:, 1]*100, traj_best[:, 0]*100, 'g-', label='PC')
    ax.plot(0, 0, 'ko', markersize=10)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.axhline(y=20, color='gray', linestyle='--', alpha=0.3)
    ax.axvline(x=0, color='gray', linestyle='--', alpha=0.3)
    ax.axvline(x=20, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Y (cm)')
    ax.set_ylabel('X (cm)')
    ax.set_title(f'Best Uniform (scale={best_scale:.2f})\nX={traj_best[:,0].ptp()*100:.1f}cm, Y={traj_best[:,1].ptp()*100:.1f}cm')
    ax.axis('equal')
    ax.grid(True)
    
    # Best asymmetric
    traj_asym = run_eskf(packets, flow_scale_x=best_scale_x, flow_scale_y=best_scale_y)
    ax = axes[2]
    ax.plot(traj_asym[:, 1]*100, traj_asym[:, 0]*100, 'r-', label='PC')
    ax.plot(0, 0, 'ko', markersize=10)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.axhline(y=20, color='gray', linestyle='--', alpha=0.3)
    ax.axvline(x=0, color='gray', linestyle='--', alpha=0.3)
    ax.axvline(x=20, color='gray', linestyle='--', alpha=0.3)
    ax.set_xlabel('Y (cm)')
    ax.set_ylabel('X (cm)')
    ax.set_title(f'Best Asymmetric (sx={best_scale_x:.2f}, sy={best_scale_y:.2f})\nX={traj_asym[:,0].ptp()*100:.1f}cm, Y={traj_asym[:,1].ptp()*100:.1f}cm')
    ax.axis('equal')
    ax.grid(True)
    
    plt.tight_layout()
    plt.savefig('tune_results.png', dpi=150)
    print("\nSaved: tune_results.png")

if __name__ == '__main__':
    main()
