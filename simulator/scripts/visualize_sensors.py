# MIT License
# Copyright (c) 2025 Kouhei Ito

"""
Sensor Model Visualization Script
センサモデル可視化スクリプト

Visualizes all Phase 2 sensor models behavior.
Phase 2の全センサモデルの動作を可視化。
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

# Set font for Japanese support (if available)
plt.rcParams['font.family'] = ['Hiragino Sans', 'Arial', 'sans-serif']


def visualize_barometer():
    """Visualize barometer characteristics / 気圧計特性を可視化"""
    from simulator.sensors.barometer import Barometer

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('BMP280 Barometer Model / BMP280気圧計モデル', fontsize=14)

    # 1. Altitude vs Pressure
    ax1 = axes[0, 0]
    baro = Barometer(pressure_noise_std=0)
    altitudes = np.linspace(0, 100, 200)
    pressures = [baro.pressure_from_altitude(alt) for alt in altitudes]
    ax1.plot(altitudes, pressures, 'b-', linewidth=2)
    ax1.set_xlabel('Altitude (m) / 高度')
    ax1.set_ylabel('Pressure (Pa) / 気圧')
    ax1.set_title('Altitude vs Pressure / 高度-気圧特性')
    ax1.grid(True, alpha=0.3)

    # 2. Noise characteristics
    ax2 = axes[0, 1]
    baro_noisy = Barometer(pressure_noise_std=1.3)
    n_samples = 500
    true_alt = 5.0
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

    # 3. Time series with drift
    ax3 = axes[1, 0]
    baro_drift = Barometer(pressure_noise_std=1.3, drift_rate=0.5)
    time = np.arange(0, 60, 0.02)
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

    # 4. Temperature effect
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

    # 1. Flow vs Velocity at different heights
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

    # 2. Surface Quality vs Height
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

    # 3. Circular motion simulation
    ax3 = axes[1, 0]
    t = np.linspace(0, 5, 500)
    omega = 2 * np.pi / 2
    radius = 0.5

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

    # 4. Rotation compensation demo
    ax4 = axes[1, 1]
    v_body = np.array([0.5, 0, 0])
    rotation_rates = np.linspace(-1.0, 1.0, 50)

    flows_with_rotation = []
    flows_compensated = []

    for q in rotation_rates:
        delta_x_raw, _ = of.velocity_to_flow(v_body, np.array([0, q, 0]), 1.0)
        flows_with_rotation.append(delta_x_raw)
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


def visualize_imu():
    """Visualize IMU noise characteristics / IMUノイズ特性を可視化"""
    from simulator.sensors.imu import IMU
    from simulator.sensors.noise_models import compute_allan_variance

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('BMI270 IMU Model / BMI270 IMUモデル', fontsize=14)

    imu = IMU(sample_rate_hz=400.0, enable_bias_drift=True)

    # Generate data
    n_samples = 40000
    gyro_data = np.zeros((n_samples, 3))
    accel_data = np.zeros((n_samples, 3))

    true_gyro = np.zeros(3)
    true_accel = np.array([0, 0, 9.80665])  # Gravity in body frame (level)

    for i in range(n_samples):
        reading = imu.read(true_gyro, true_accel)
        gyro_data[i] = reading['gyro']
        accel_data[i] = reading['accel']

    time = np.arange(n_samples) / 400.0

    # 1. Gyro noise time series
    ax1 = axes[0, 0]
    ax1.plot(time[:2000], np.rad2deg(gyro_data[:2000, 0]), 'b-', alpha=0.7, linewidth=0.5, label='X')
    ax1.plot(time[:2000], np.rad2deg(gyro_data[:2000, 1]), 'g-', alpha=0.7, linewidth=0.5, label='Y')
    ax1.plot(time[:2000], np.rad2deg(gyro_data[:2000, 2]), 'r-', alpha=0.7, linewidth=0.5, label='Z')
    ax1.set_xlabel('Time (s) / 時間')
    ax1.set_ylabel('Angular Rate (deg/s) / 角速度')
    ax1.set_title('Gyroscope Noise (5s) / ジャイロノイズ')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # 2. Accel noise time series
    ax2 = axes[0, 1]
    accel_centered = accel_data - np.mean(accel_data, axis=0)
    ax2.plot(time[:2000], accel_centered[:2000, 0], 'b-', alpha=0.7, linewidth=0.5, label='X')
    ax2.plot(time[:2000], accel_centered[:2000, 1], 'g-', alpha=0.7, linewidth=0.5, label='Y')
    ax2.plot(time[:2000], accel_centered[:2000, 2], 'r-', alpha=0.7, linewidth=0.5, label='Z')
    ax2.set_xlabel('Time (s) / 時間')
    ax2.set_ylabel('Acceleration noise (m/s²) / 加速度ノイズ')
    ax2.set_title('Accelerometer Noise (5s) / 加速度ノイズ')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    # 3. Gyro Allan Deviation
    ax3 = axes[1, 0]
    tau, avar, adev = compute_allan_variance(gyro_data[:, 0], 400.0)
    adev_deg = np.rad2deg(adev)
    ax3.loglog(tau, adev_deg, 'b-', linewidth=2, label='Measured')
    ax3.set_xlabel('Cluster Time τ (s)')
    ax3.set_ylabel('Allan Deviation (deg/s)')
    ax3.set_title('Gyro Allan Deviation / ジャイロAllan偏差')
    ax3.legend()
    ax3.grid(True, alpha=0.3, which='both')

    # 4. Accel Allan Deviation
    ax4 = axes[1, 1]
    tau_a, avar_a, adev_a = compute_allan_variance(accel_centered[:, 2], 400.0)
    ax4.loglog(tau_a, adev_a * 1000, 'b-', linewidth=2, label='Measured')
    ax4.set_xlabel('Cluster Time τ (s)')
    ax4.set_ylabel('Allan Deviation (mm/s²)')
    ax4.set_title('Accel Allan Deviation / 加速度Allan偏差')
    ax4.legend()
    ax4.grid(True, alpha=0.3, which='both')

    plt.tight_layout()
    return fig


def visualize_magnetometer():
    """Visualize magnetometer characteristics / 地磁気センサ特性を可視化"""
    from simulator.sensors.magnetometer import Magnetometer

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('BMM150 Magnetometer Model / BMM150地磁気センサモデル', fontsize=14)

    mag = Magnetometer(noise_std=0.3)

    # 1. Magnetic field in different orientations
    ax1 = axes[0, 0]
    yaw_angles = np.linspace(0, 2*np.pi, 100)
    mag_x = []
    mag_y = []

    for yaw in yaw_angles:
        # Rotation matrix for yaw only (level flight)
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        reading = mag.read(R)
        mag_x.append(reading['mag_body'][0])
        mag_y.append(reading['mag_body'][1])

    ax1.plot(mag_x, mag_y, 'b-', linewidth=2)
    ax1.plot(mag_x[0], mag_y[0], 'go', markersize=10, label='North')
    ax1.set_xlabel('Mag X (µT)')
    ax1.set_ylabel('Mag Y (µT)')
    ax1.set_title('Magnetic Field vs Yaw (Level) / ヨー角と磁場（水平）')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # 2. Hard iron distortion visualization
    ax2 = axes[0, 1]
    mag_ideal = Magnetometer(noise_std=0.0, hard_iron=np.zeros(3), soft_iron=np.eye(3))
    mag_distorted = Magnetometer(noise_std=0.0, hard_iron=np.array([15, -10, 5]))

    mag_ideal_x, mag_ideal_y = [], []
    mag_dist_x, mag_dist_y = [], []

    for yaw in yaw_angles:
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])

        reading_ideal = mag_ideal.read(R)
        mag_ideal_x.append(reading_ideal['mag_body'][0])
        mag_ideal_y.append(reading_ideal['mag_body'][1])

        reading_dist = mag_distorted.read(R)
        mag_dist_x.append(reading_dist['mag_body'][0])
        mag_dist_y.append(reading_dist['mag_body'][1])

    ax2.plot(mag_ideal_x, mag_ideal_y, 'g-', linewidth=2, label='Ideal')
    ax2.plot(mag_dist_x, mag_dist_y, 'r-', linewidth=2, label='Hard Iron')
    ax2.set_xlabel('Mag X (µT)')
    ax2.set_ylabel('Mag Y (µT)')
    ax2.set_title('Hard Iron Distortion / ハードアイアン歪み')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')

    # 3. Heading calculation
    ax3 = axes[1, 0]
    headings_true = np.rad2deg(yaw_angles)
    headings_measured = []

    mag_noisy = Magnetometer(noise_std=0.5)
    for yaw in yaw_angles:
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        reading = mag_noisy.read(R)
        headings_measured.append(reading['heading_deg'])

    ax3.plot(headings_true, headings_true, 'g--', linewidth=2, label='True')
    ax3.plot(headings_true, headings_measured, 'b-', linewidth=1, alpha=0.7, label='Measured')
    ax3.set_xlabel('True Heading (deg) / 真方位')
    ax3.set_ylabel('Measured Heading (deg) / 測定方位')
    ax3.set_title('Heading Accuracy / 方位精度')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. Noise distribution
    ax4 = axes[1, 1]
    R_level = np.eye(3)
    mag_readings = []
    for _ in range(500):
        reading = mag_noisy.read(R_level)
        mag_readings.append(reading['magnitude'])

    ax4.hist(mag_readings, bins=30, density=True, alpha=0.7, color='blue')
    ax4.axvline(np.mean(mag_readings), color='red', linestyle='--', linewidth=2,
                label=f'Mean: {np.mean(mag_readings):.1f} µT')
    ax4.set_xlabel('Magnitude (µT) / 磁場強度')
    ax4.set_ylabel('Density / 密度')
    ax4.set_title('Magnetic Field Magnitude Distribution / 磁場強度分布')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def visualize_tof():
    """Visualize ToF sensor characteristics / ToFセンサ特性を可視化"""
    from simulator.sensors.tof import ToF

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('VL53L3CX ToF Sensor Model / VL53L3CX ToFセンサモデル', fontsize=14)

    tof = ToF(base_noise_mm=5.0)

    # 1. Distance measurement accuracy
    ax1 = axes[0, 0]
    true_distances = np.linspace(0.05, 2.5, 50)
    measured_means = []
    measured_stds = []

    for d in true_distances:
        measurements = [tof.read(d)['distance_m'] for _ in range(50)]
        measured_means.append(np.mean(measurements))
        measured_stds.append(np.std(measurements))

    measured_means = np.array(measured_means)
    measured_stds = np.array(measured_stds)

    ax1.fill_between(true_distances, measured_means - measured_stds,
                     measured_means + measured_stds, alpha=0.3)
    ax1.plot(true_distances, measured_means, 'b-', linewidth=2, label='Measured')
    ax1.plot(true_distances, true_distances, 'r--', linewidth=2, label='True')
    ax1.set_xlabel('True Distance (m) / 真の距離')
    ax1.set_ylabel('Measured Distance (m) / 測定距離')
    ax1.set_title('Distance Measurement Accuracy / 距離測定精度')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2. Noise vs Distance
    ax2 = axes[0, 1]
    ax2.plot(true_distances, measured_stds * 1000, 'b-', linewidth=2)
    ax2.set_xlabel('Distance (m) / 距離')
    ax2.set_ylabel('Noise Std Dev (mm) / ノイズ標準偏差')
    ax2.set_title('Noise vs Distance / 距離とノイズ')
    ax2.grid(True, alpha=0.3)

    # 3. Signal rate vs Distance
    ax3 = axes[1, 0]
    signal_rates = []
    for d in true_distances:
        readings = [tof.read(d)['signal_rate'] for _ in range(20)]
        signal_rates.append(np.mean(readings))

    ax3.semilogy(true_distances, signal_rates, 'b-', linewidth=2)
    ax3.axhline(tof.SIGNAL_RATE_MIN, color='red', linestyle='--', label='Min valid')
    ax3.set_xlabel('Distance (m) / 距離')
    ax3.set_ylabel('Signal Rate (MCPS)')
    ax3.set_title('Signal Rate vs Distance / 距離と信号強度')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. Tilt effect
    ax4 = axes[1, 1]
    tilt_angles = np.linspace(0, 30, 20)
    distance_at_tilts = []

    for tilt_deg in tilt_angles:
        tilt_rad = np.deg2rad(tilt_deg)
        attitude = np.array([tilt_rad, 0, 0])  # Roll tilt
        readings = [tof.read(1.0, attitude_rad=attitude)['distance_m'] for _ in range(30)]
        distance_at_tilts.append(np.mean(readings))

    ax4.plot(tilt_angles, distance_at_tilts, 'b-', linewidth=2)
    ax4.axhline(1.0, color='red', linestyle='--', label='True (1m)')
    ax4.axvline(tof.fov_deg / 2, color='orange', linestyle='--', label='FOV/2')
    ax4.set_xlabel('Tilt Angle (deg) / 傾斜角')
    ax4.set_ylabel('Measured Distance (m) / 測定距離')
    ax4.set_title('Tilt Effect on Measurement / 傾斜の影響')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def visualize_power_monitor():
    """Visualize power monitor characteristics / 電源モニタ特性を可視化"""
    from simulator.sensors.power_monitor import PowerMonitor

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('INA3221 Power Monitor Model / INA3221電源モニタモデル', fontsize=14)

    # 1. Battery discharge curve
    ax1 = axes[0, 0]
    pm = PowerMonitor(initial_voltage=4.2, capacity_mah=300)

    time_points = []
    voltages = []
    percentages = []

    dt = 1.0  # 1 second steps
    current_ma = 500  # 500mA draw

    t = 0
    while pm._remaining_capacity_mah > 0 and t < 3600:  # Max 1 hour
        reading = pm.read(current_ma)
        time_points.append(t / 60)  # Convert to minutes
        voltages.append(reading['voltage_v'])
        percentages.append(reading['battery_percent'])
        pm.update(current_ma, dt)
        t += dt

    ax1.plot(time_points, voltages, 'b-', linewidth=2)
    ax1.axhline(PowerMonitor.LOW_VOLTAGE_THRESHOLD, color='red', linestyle='--',
                label=f'Low battery ({PowerMonitor.LOW_VOLTAGE_THRESHOLD}V)')
    ax1.set_xlabel('Time (min) / 時間（分）')
    ax1.set_ylabel('Voltage (V) / 電圧')
    ax1.set_title(f'Battery Discharge @ {current_ma}mA / バッテリー放電')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2. Battery percentage vs voltage
    ax2 = axes[0, 1]
    ax2.plot(voltages, percentages, 'b-', linewidth=2)
    ax2.set_xlabel('Voltage (V) / 電圧')
    ax2.set_ylabel('Battery Percent (%) / バッテリー残量')
    ax2.set_title('Voltage vs Capacity / 電圧と残量')
    ax2.grid(True, alpha=0.3)

    # 3. Current estimation from motors
    ax3 = axes[1, 0]
    pm_fresh = PowerMonitor(initial_voltage=4.2, capacity_mah=300)

    motor_outputs = np.linspace(0, 1, 50)
    currents = []

    for output in motor_outputs:
        motor_array = np.array([output, output, output, output])
        current = pm_fresh.estimate_current_from_motors(motor_array)
        currents.append(current)

    ax3.plot(motor_outputs * 100, currents, 'b-', linewidth=2)
    ax3.set_xlabel('Motor Output (%) / モーター出力')
    ax3.set_ylabel('Estimated Current (mA) / 推定電流')
    ax3.set_title('Current vs Motor Output / モーター出力と電流')
    ax3.grid(True, alpha=0.3)

    # 4. Voltage noise
    ax4 = axes[1, 1]
    pm_test = PowerMonitor(initial_voltage=3.8, voltage_noise_std=0.01)

    voltage_readings = []
    for _ in range(500):
        reading = pm_test.read(300)
        voltage_readings.append(reading['voltage_v'])

    ax4.hist(voltage_readings, bins=30, density=True, alpha=0.7, color='blue')
    ax4.axvline(np.mean(voltage_readings), color='red', linestyle='--', linewidth=2,
                label=f'Mean: {np.mean(voltage_readings):.3f}V')
    ax4.set_xlabel('Voltage (V) / 電圧')
    ax4.set_ylabel('Density / 密度')
    ax4.set_title('Voltage Measurement Distribution / 電圧測定分布')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

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
    print("  1/5 Barometer (BMP280)...")
    fig1 = visualize_barometer()
    fig1.savefig(os.path.join(output_dir, 'barometer_test.png'), dpi=150)
    print(f"      Saved: barometer_test.png")

    print("  2/5 Optical Flow (PMW3901)...")
    fig2 = visualize_optical_flow()
    fig2.savefig(os.path.join(output_dir, 'opticalflow_test.png'), dpi=150)
    print(f"      Saved: opticalflow_test.png")

    print("  3/5 IMU (BMI270)...")
    fig3 = visualize_imu()
    fig3.savefig(os.path.join(output_dir, 'imu_test.png'), dpi=150)
    print(f"      Saved: imu_test.png")

    print("  4/5 Magnetometer (BMM150)...")
    fig4 = visualize_magnetometer()
    fig4.savefig(os.path.join(output_dir, 'magnetometer_test.png'), dpi=150)
    print(f"      Saved: magnetometer_test.png")

    print("  5/5 ToF (VL53L3CX)...")
    fig5 = visualize_tof()
    fig5.savefig(os.path.join(output_dir, 'tof_test.png'), dpi=150)
    print(f"      Saved: tof_test.png")

    print("  6/6 Power Monitor (INA3221)...")
    fig6 = visualize_power_monitor()
    fig6.savefig(os.path.join(output_dir, 'power_monitor_test.png'), dpi=150)
    print(f"      Saved: power_monitor_test.png")

    print("\nDone! / 完了！")
    print(f"Output directory: {output_dir}")

    plt.show()


if __name__ == "__main__":
    main()
