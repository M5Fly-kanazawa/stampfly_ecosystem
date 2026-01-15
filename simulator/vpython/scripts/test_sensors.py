# MIT License
# Copyright (c) 2025 Kouhei Ito

"""
Sensor Model Verification Script
センサモデル検証スクリプト

Tests the new Phase 2 sensor models: Barometer, OpticalFlow, NoiseModels
Phase 2で追加したセンサモデルをテスト：気圧計、オプティカルフロー、ノイズモデル
"""

import sys
import os

# Add vpython package to path
# vpythonパッケージをパスに追加
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_VPYTHON_DIR = os.path.dirname(_SCRIPT_DIR)
if _VPYTHON_DIR not in sys.path:
    sys.path.insert(0, _VPYTHON_DIR)

import numpy as np


def test_barometer():
    """Test BMP280 barometer model / BMP280気圧計モデルをテスト"""
    print("=" * 50)
    print("Barometer (BMP280) Test / 気圧計テスト")
    print("=" * 50)

    from sensors.barometer import Barometer

    baro = Barometer(
        sea_level_pressure_pa=101325.0,
        temperature_c=25.0,
        pressure_noise_std=1.3,
    )

    # Test at various altitudes (NED frame: down is positive)
    # 様々な高度でテスト（NED座標系：下が正）
    altitudes_ned = [0.0, -1.0, -5.0, -10.0]  # 0m, 1m, 5m, 10m above ground

    print("\nAltitude vs Pressure:")
    print("-" * 40)
    for alt_ned in altitudes_ned:
        reading = baro.read(true_altitude_m=alt_ned, dt=0.02)
        alt_up = -alt_ned  # Convert to up-positive
        print(f"  Height: {alt_up:5.1f}m -> "
              f"Pressure: {reading['pressure_pa']:.1f} Pa, "
              f"Measured Alt: {reading['altitude_m']:.2f} m")

    print("\n✓ Barometer test passed / 気圧計テスト完了")
    return True


def test_optical_flow():
    """Test PMW3901 optical flow model / PMW3901オプティカルフローモデルをテスト"""
    print("\n" + "=" * 50)
    print("Optical Flow (PMW3901) Test / オプティカルフローテスト")
    print("=" * 50)

    from sensors.opticalflow import OpticalFlow

    of = OpticalFlow(
        focal_length_px=35.0,
        flow_noise_std=0.5,
        frame_rate_hz=121.0,
    )

    # Test: forward velocity at 1m height
    # テスト：1m高度での前進速度
    velocity_body = np.array([0.5, 0.0, 0.0])  # 0.5 m/s forward
    angular_velocity = np.array([0.0, 0.0, 0.0])  # No rotation
    height = 1.0  # 1m above ground

    print("\nTest 1: Forward motion at 1m height")
    print(f"  Velocity: {velocity_body} m/s")
    reading = of.read(velocity_body, angular_velocity, height)
    print(f"  delta_x: {reading['delta_x']}, delta_y: {reading['delta_y']}")
    print(f"  squal: {reading['squal']}, motion: {reading['motion_detected']}")

    # Test: sideways velocity
    # テスト：横方向速度
    velocity_body = np.array([0.0, 0.3, 0.0])  # 0.3 m/s right
    print("\nTest 2: Sideways motion")
    print(f"  Velocity: {velocity_body} m/s")
    reading = of.read(velocity_body, angular_velocity, height)
    print(f"  delta_x: {reading['delta_x']}, delta_y: {reading['delta_y']}")

    # Test: rotation effect
    # テスト：回転の影響
    velocity_body = np.array([0.0, 0.0, 0.0])  # No translation
    angular_velocity = np.array([0.0, 0.5, 0.0])  # Pitch rate 0.5 rad/s
    print("\nTest 3: Pure rotation (pitch)")
    print(f"  Angular velocity: {angular_velocity} rad/s")
    reading = of.read(velocity_body, angular_velocity, height)
    print(f"  delta_x: {reading['delta_x']}, delta_y: {reading['delta_y']}")

    # Test: inverse conversion
    # テスト：逆変換
    print("\nTest 4: Flow to velocity conversion")
    velocity_body = np.array([0.5, 0.2, 0.0])
    angular_velocity = np.array([0.1, 0.1, 0.0])
    height = 0.5

    delta_x, delta_y = of.velocity_to_flow(velocity_body, angular_velocity, height)
    recovered_vel = of.flow_to_velocity(
        int(delta_x), int(delta_y), angular_velocity, height
    )
    print(f"  Original velocity: [{velocity_body[0]:.3f}, {velocity_body[1]:.3f}]")
    print(f"  Recovered velocity: [{recovered_vel[0]:.3f}, {recovered_vel[1]:.3f}]")

    print("\n✓ Optical flow test passed / オプティカルフローテスト完了")
    return True


def test_noise_models():
    """Test Allan variance noise models / Allan分散ノイズモデルをテスト"""
    print("\n" + "=" * 50)
    print("Noise Models (Allan Variance) Test / ノイズモデルテスト")
    print("=" * 50)

    from sensors.noise_models import (
        BMI270_GyroParams,
        BMI270_AccelParams,
        IMUNoiseGenerator,
    )

    # Test BMI270 parameters
    # BMI270パラメータをテスト
    gyro_params = BMI270_GyroParams()
    accel_params = BMI270_AccelParams()

    print("\nBMI270 Gyro Specs:")
    print(f"  Noise density: {gyro_params.noise_density_dps_sqrthz} deg/s/√Hz")
    print(f"  Zero-rate offset: ±{gyro_params.zero_rate_offset_dps} deg/s")

    print("\nBMI270 Accel Specs:")
    print(f"  Noise density: {accel_params.noise_density_ug_sqrthz} µg/√Hz")
    print(f"  Zero-g offset: ±{accel_params.zero_g_offset_mg} mg")

    # Test noise generator
    # ノイズ生成器をテスト
    noise_gen = IMUNoiseGenerator(sample_rate_hz=400.0)

    print("\nNoise Generation Test (100 samples):")
    gyro_noise_samples = np.array([noise_gen.generate_gyro_noise() for _ in range(100)])
    accel_noise_samples = np.array([noise_gen.generate_accel_noise() for _ in range(100)])

    print(f"  Gyro noise std:  {np.std(gyro_noise_samples, axis=0)} rad/s")
    print(f"  Accel noise std: {np.std(accel_noise_samples, axis=0)} m/s²")
    print(f"  Gyro bias:  {noise_gen.gyro_bias} rad/s")
    print(f"  Accel bias: {noise_gen.accel_bias} m/s²")

    print("\n✓ Noise models test passed / ノイズモデルテスト完了")
    return True


def main():
    """Run all sensor tests / 全センサテストを実行"""
    print("\n" + "#" * 60)
    print("# Phase 2 Sensor Models Verification")
    print("# Phase 2 センサモデル検証")
    print("#" * 60)

    all_passed = True

    try:
        all_passed &= test_barometer()
    except Exception as e:
        print(f"\n✗ Barometer test failed: {e}")
        all_passed = False

    try:
        all_passed &= test_optical_flow()
    except Exception as e:
        print(f"\n✗ Optical flow test failed: {e}")
        all_passed = False

    try:
        all_passed &= test_noise_models()
    except Exception as e:
        print(f"\n✗ Noise models test failed: {e}")
        all_passed = False

    print("\n" + "=" * 60)
    if all_passed:
        print("All tests passed! / 全テスト完了！")
    else:
        print("Some tests failed. / 一部のテストが失敗しました。")
    print("=" * 60 + "\n")

    return all_passed


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
