# MIT License
# Copyright (c) 2025 Kouhei Ito

"""
Sensor Model Visualization Script
センサモデル可視化スクリプト

Visualizes the Phase 2 sensor models behavior.
Phase 2センサモデルの動作を可視化。
"""

import sys
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SIMULATOR_DIR = os.path.dirname(_SCRIPT_DIR)
_ECOSYSTEM_DIR = os.path.dirname(_SIMULATOR_DIR)
if _ECOSYSTEM_DIR not in sys.path:
    sys.path.insert(0, _ECOSYSTEM_DIR)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Set font for Japanese support (if available)
plt.rcParams['font.family'] = ['Hiragino Sans', 'Arial', 'sans-serif']


def visualize_barometer():
    """Visualize barometer characteristics / 気圧計特性を可視化"""
    from simulator.sensors.barometer import Barometer

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('BMP280 Barometer Model / BMP280気圧計モデル', fontsize=14)

    # 1. Altitude vs Pressure (理論曲線)
    ax1 = axes[0, 0]
    baro = Barometer(pressure_noise_std=0)  # No noise for theoretical curve
    altitudes = np.linspace(0, 100, 200)
    pressures = [baro.pressure_from_altitude(alt) for alt in altitudes]
    ax1.plot(altitudes, pressures, 'b-', linewidth=2)
    ax1.set_xlabel('Altitude (m) / 高度')
    ax1.set_ylabel('Pressure (Pa) / 気圧')
    ax1.set_title('Altitude vs Pressure / 高度-気圧特性')
    ax1.grid(True, alpha=0.3)

    # 2. Noise characteristics (ノイズ特性)
    ax2 = axes[0, 1]
    baro_noisy = Barometer(pressure_noise_std=1.3)
    n_samples = 500
    true_alt = 5.0  # 5m altitude
    readings = [baro_noisy.read(-true_alt, dt=0.02) for _ in range(n_samples)]
    measured_alts = [r['altitude_m'] for r in readings]

    ax2.hist(measured_alts, bins=30, density=True, alpha=0.7, color='blue')
    ax2.axvline(true_alt, color='red', linestyle='--', linewidth=2, label=f'True: {true_alt}m')
    ax2.axvline(np.mean(measured_alts), color='green', linestyle='-', linewidth=2,
                label=f'Mean: {np.mean(measured_alts):.2f}m')
    ax2.set_xlabel('Measured Altitude (m) / 測定高度')
    ax2.set_ylabel('Density / 密度')
    ax2.set_title('Altitude Measurement Distribution / 高度測定分布')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. Time series with drift (ドリフト付き時系列)
    ax3 = axes[1, 0]
    baro_drift = Barometer(pressure_noise_std=1.3, drift_rate=0.5)  # 0.5 Pa/s drift
    time = np.arange(0, 60, 0.02)  # 60 seconds
    drift_readings = []
    for t in time:
        drift_readings.append(baro_drift.read(-5.0, dt=0.02))

    measured_alts_drift = [r['altitude_m'] for r in drift_readings]
    ax3.plot(time, measured_alts_drift, 'b-', alpha=0.5, linewidth=0.5)
    ax3.axhline(5.0, color='red', linestyle='--', linewidth=2, label='True altitude')
    ax3.set_xlabel('Time (s) / 時間')
    ax3.set_ylabel('Measured Altitude (m) / 測定高度')
    ax3.set_title('Altitude with Drift (0.5 Pa/s) / ドリフト付き高度')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. Temperature effect on pressure (温度影響)
    ax4 = axes[1, 1]
    temperatures = [15, 20, 25, 30, 35]
    for temp in temperatures:
        baro_temp = Barometer(temperature_c=temp, pressure_noise_std=0)
        pressures_temp = [baro_temp.pressure_from_altitude(alt) for alt in altitudes[:50]]
        ax4.plot(altitudes[:50], pressures_temp, label=f'{temp}°C')

    ax4.set_xlabel('Altitude (m) / 高度')
    ax4.set_ylabel('Pressure (Pa) / 気圧')
    ax4.set_title('Temperature Effect / 温度の影響')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def visualize_optical_flow():
    """Visualize optical flow characteristics / オプティカルフロー特性を可視化"""
    from simulator.sensors.opticalflow import OpticalFlow

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('PMW3901 Optical Flow Model / PMW3901オプティカルフローモデル', fontsize=14)

    of = OpticalFlow(flow_noise_std=0.5, frame_rate_hz=121.0)

    # 1. Flow vs Velocity at different heights (異なる高度での速度-フロー関係)
    ax1 = axes[0, 0]
    velocities = np.linspace(-1.0, 1.0, 50)
    heights = [0.3, 0.5, 1.0, 2.0]

    for h in heights:
        flows = []
        for v in velocities:
            delta_x, delta_y = of.velocity_to_flow(
                np.array([v, 0, 0]),
                np.array([0, 0, 0]),
                h
            )
            flows.append(delta_x)
        ax1.plot(velocities, flows, label=f'h={h}m')

    ax1.set_xlabel('Forward Velocity (m/s) / 前進速度')
    ax1.set_ylabel('Flow delta_x (pixels/frame)')
    ax1.set_title('Flow vs Velocity (Height Effect) / 速度-フロー（高度影響）')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2. Surface Quality vs Height (高度と表面品質)
    ax2 = axes[0, 1]
    heights_test = np.linspace(0.05, 3.0, 100)
    squals = []
    squals_std = []

    for h in heights_test:
        samples = [of.compute_squal(h, surface_texture=1.0) for _ in range(50)]
        squals.append(np.mean(samples))
        squals_std.append(np.std(samples))

    squals = np.array(squals)
    squals_std = np.array(squals_std)
    ax2.fill_between(heights_test, squals - squals_std, squals + squals_std, alpha=0.3)
    ax2.plot(heights_test, squals, 'b-', linewidth=2)
    ax2.axvline(of.min_height, color='red', linestyle='--', label='Min height')
    ax2.axvline(of.max_height, color='red', linestyle='--', label='Max height')
    ax2.set_xlabel('Height (m) / 高度')
    ax2.set_ylabel('Surface Quality (SQUAL)')
    ax2.set_title('Surface Quality vs Height / 高度と表面品質')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. Circular motion simulation (円運動シミュレーション)
    ax3 = axes[1, 0]
    t = np.linspace(0, 5, 500)  # 5 seconds
    omega = 2 * np.pi / 2  # 2秒で1周
    radius = 0.5  # 0.5m radius

    vx = -radius * omega * np.sin(omega * t)
    vy = radius * omega * np.cos(omega * t)

    delta_xs = []
    delta_ys = []

    of_test = OpticalFlow(flow_noise_std=0.3, frame_rate_hz=100.0)
    for i in range(len(t)):
        reading = of_test.read(
            np.array([vx[i], vy[i], 0]),
            np.array([0, 0, 0]),
            height_m=1.0
        )
        delta_xs.append(reading['delta_x'])
        delta_ys.append(reading['delta_y'])

    ax3.plot(delta_xs, delta_ys, 'b.', alpha=0.3, markersize=2)
    ax3.plot(delta_xs[0], delta_ys[0], 'go', markersize=10, label='Start')
    ax3.set_xlabel('delta_x (pixels)')
    ax3.set_ylabel('delta_y (pixels)')
    ax3.set_title('Circular Motion Flow / 円運動時のフロー')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')

    # 4. Rotation compensation demo (回転補償デモ)
    ax4 = axes[1, 1]
    # Same translation, different rotation rates
    v_body = np.array([0.5, 0, 0])
    rotation_rates = np.linspace(-1.0, 1.0, 50)  # rad/s pitch rate

    flows_with_rotation = []
    flows_compensated = []

    for q in rotation_rates:
        delta_x_raw, _ = of.velocity_to_flow(v_body, np.array([0, q, 0]), 1.0)
        flows_with_rotation.append(delta_x_raw)

        # What flow would be without rotation
        delta_x_trans, _ = of.velocity_to_flow(v_body, np.array([0, 0, 0]), 1.0)
        flows_compensated.append(delta_x_trans)

    ax4.plot(rotation_rates, flows_with_rotation, 'b-', linewidth=2, label='Raw flow')
    ax4.plot(rotation_rates, flows_compensated, 'r--', linewidth=2, label='Translation only')
    ax4.set_xlabel('Pitch Rate (rad/s) / ピッチレート')
    ax4.set_ylabel('delta_x (pixels/frame)')
    ax4.set_title('Rotation Effect on Flow / 回転の影響')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def visualize_noise_models():
    """Visualize IMU noise model characteristics / IMUノイズモデル特性を可視化"""
    from simulator.sensors.noise_models import (
        IMUNoiseGenerator,
        compute_allan_variance,
    )

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('IMU Noise Models (BMI270) / IMUノイズモデル', fontsize=14)

    noise_gen = IMUNoiseGenerator(sample_rate_hz=400.0)

    # Generate long time series for Allan variance
    n_samples = 40000  # 100 seconds at 400Hz
    gyro_data = np.zeros((n_samples, 3))
    accel_data = np.zeros((n_samples, 3))

    for i in range(n_samples):
        gyro_data[i] = noise_gen.generate_gyro_noise()
        accel_data[i] = noise_gen.generate_accel_noise()

    time = np.arange(n_samples) / 400.0

    # 1. Gyro noise time series (ジャイロノイズ時系列)
    ax1 = axes[0, 0]
    ax1.plot(time[:2000], np.rad2deg(gyro_data[:2000, 0]), 'b-', alpha=0.7, linewidth=0.5, label='X')
    ax1.plot(time[:2000], np.rad2deg(gyro_data[:2000, 1]), 'g-', alpha=0.7, linewidth=0.5, label='Y')
    ax1.plot(time[:2000], np.rad2deg(gyro_data[:2000, 2]), 'r-', alpha=0.7, linewidth=0.5, label='Z')
    ax1.set_xlabel('Time (s) / 時間')
    ax1.set_ylabel('Angular Rate (deg/s) / 角速度')
    ax1.set_title('Gyroscope Noise (5s) / ジャイロノイズ')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # 2. Accel noise time series (加速度ノイズ時系列)
    ax2 = axes[0, 1]
    ax2.plot(time[:2000], accel_data[:2000, 0], 'b-', alpha=0.7, linewidth=0.5, label='X')
    ax2.plot(time[:2000], accel_data[:2000, 1], 'g-', alpha=0.7, linewidth=0.5, label='Y')
    ax2.plot(time[:2000], accel_data[:2000, 2], 'r-', alpha=0.7, linewidth=0.5, label='Z')
    ax2.set_xlabel('Time (s) / 時間')
    ax2.set_ylabel('Acceleration (m/s²) / 加速度')
    ax2.set_title('Accelerometer Noise (5s) / 加速度ノイズ')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    # 3. Allan Deviation - Gyro (Allan偏差 - ジャイロ)
    ax3 = axes[1, 0]
    tau, avar, adev = compute_allan_variance(gyro_data[:, 0], 400.0)
    adev_deg = np.rad2deg(adev)  # Convert to deg/s

    ax3.loglog(tau, adev_deg, 'b-', linewidth=2, label='Measured')

    # Theoretical slopes
    tau_short = tau[tau < 0.1]
    tau_long = tau[tau > 1.0]
    if len(tau_short) > 0:
        arw_line = adev_deg[0] * np.sqrt(tau[0] / tau_short)
        ax3.loglog(tau_short, arw_line, 'g--', alpha=0.7, label='ARW slope (-1/2)')
    if len(tau_long) > 0:
        rrw_line = adev_deg[-1] * np.sqrt(tau_long / tau[-1])
        ax3.loglog(tau_long, rrw_line, 'r--', alpha=0.7, label='RRW slope (+1/2)')

    ax3.set_xlabel('Cluster Time τ (s)')
    ax3.set_ylabel('Allan Deviation (deg/s)')
    ax3.set_title('Gyro Allan Deviation / ジャイロAllan偏差')
    ax3.legend()
    ax3.grid(True, alpha=0.3, which='both')

    # 4. Allan Deviation - Accel (Allan偏差 - 加速度)
    ax4 = axes[1, 1]
    tau_a, avar_a, adev_a = compute_allan_variance(accel_data[:, 0], 400.0)

    ax4.loglog(tau_a, adev_a * 1000, 'b-', linewidth=2, label='Measured')  # Convert to mm/s²

    ax4.set_xlabel('Cluster Time τ (s)')
    ax4.set_ylabel('Allan Deviation (mm/s²)')
    ax4.set_title('Accel Allan Deviation / 加速度Allan偏差')
    ax4.legend()
    ax4.grid(True, alpha=0.3, which='both')

    plt.tight_layout()
    return fig


def main():
    """Generate all visualizations / 全可視化を生成"""
    print("Generating sensor visualizations...")
    print("センサ可視化を生成中...")

    # Create output directory
    output_dir = os.path.join(_SIMULATOR_DIR, 'output')
    os.makedirs(output_dir, exist_ok=True)

    # Generate figures
    fig1 = visualize_barometer()
    fig1.savefig(os.path.join(output_dir, 'barometer_test.png'), dpi=150)
    print(f"  Saved: {output_dir}/barometer_test.png")

    fig2 = visualize_optical_flow()
    fig2.savefig(os.path.join(output_dir, 'opticalflow_test.png'), dpi=150)
    print(f"  Saved: {output_dir}/opticalflow_test.png")

    fig3 = visualize_noise_models()
    fig3.savefig(os.path.join(output_dir, 'noise_models_test.png'), dpi=150)
    print(f"  Saved: {output_dir}/noise_models_test.png")

    print("\nDone! / 完了！")
    print(f"Output directory: {output_dir}")

    # Show plots
    plt.show()


if __name__ == "__main__":
    main()
