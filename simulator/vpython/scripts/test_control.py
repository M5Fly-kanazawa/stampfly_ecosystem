#!/usr/bin/env python3
"""
Control System Test Script
制御システムテストスクリプト

Verifies:
1. PID controller with incomplete derivative filter
2. Rate controller with firmware-matched gains
3. Motor mixer
4. Attitude controller
5. Altitude controller
"""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from control import (
    # PID
    PID,
    PIDGains,
    LegacyPID,
    # Rate Controller
    RateController,
    RateControlConfig,
    # Attitude Controller
    AttitudeController,
    AltitudeController,
    # Motor Mixer
    MotorMixer,
    mix_motors,
    MotorIndex,
)


def test_pid_basic():
    """Test basic PID functionality / 基本的なPID機能をテスト"""
    print("Testing PID basic functionality...")

    # Create PID with firmware-like gains
    pid = PID(Kp=0.65, Ti=0.7, Td=0.01, eta=0.125, output_min=-3.7, output_max=3.7)

    # Simulate step response
    setpoint = 1.0
    measurement = 0.0
    dt = 0.0025  # 400Hz

    outputs = []
    for i in range(200):  # 0.5 seconds
        output = pid.update(setpoint, measurement, dt)
        outputs.append(output)
        # Simple first-order system response
        measurement += output * 0.01

    # Check output is bounded
    assert all(-3.7 <= o <= 3.7 for o in outputs), "Output exceeds limits"

    # Check system approaches setpoint (measurement should increase)
    assert outputs[0] > 0, "Initial output should be positive for positive error"
    assert measurement > 0.5, "System should respond to setpoint"

    # Test reset
    pid.reset()
    assert pid.integral == 0.0, "Integral should reset to 0"

    print("  ✓ PID basic tests passed")


def test_pid_derivative_filter():
    """Test incomplete derivative filter / 不完全微分フィルタをテスト"""
    print("Testing PID derivative filter...")

    # PID with derivative
    pid = PID(Kp=1.0, Ti=0.0, Td=0.1, eta=0.125)

    dt = 0.0025
    outputs = []

    # Step change in setpoint
    for i in range(100):
        if i < 50:
            measurement = 0.0
        else:
            measurement = 1.0  # Step
        output = pid.update(0.0, measurement, dt)
        outputs.append(output)

    # Derivative should filter the step (not infinite spike)
    max_output = max(abs(o) for o in outputs)
    assert max_output < 100, f"Derivative filter should limit spikes: {max_output}"

    print("  ✓ PID derivative filter tests passed")


def test_pid_antiwindup():
    """Test anti-windup / アンチワインドアップをテスト"""
    print("Testing PID anti-windup...")

    # PID with integral and limiting
    pid = PID(Kp=1.0, Ti=0.5, Td=0.0, output_min=-1.0, output_max=1.0)

    dt = 0.0025
    setpoint = 10.0  # Large setpoint to cause saturation
    measurement = 0.0

    # Run for a while to saturate
    for _ in range(1000):
        pid.update(setpoint, measurement, dt)

    # Integral should not grow unbounded due to anti-windup
    # With back-calculation, integral should be limited
    integral_after_saturation = pid.integral
    assert abs(integral_after_saturation) < 100, f"Anti-windup should limit integral: {integral_after_saturation}"

    print("  ✓ PID anti-windup tests passed")


def test_rate_controller():
    """Test rate controller / レートコントローラをテスト"""
    print("Testing rate controller...")

    # Create rate controller with default (firmware) config
    ctrl = RateController()

    # Test with zero error (should output near zero)
    rate_setpoint = np.array([0.0, 0.0, 0.0])
    gyro = np.array([0.0, 0.0, 0.0])
    dt = 0.0025

    roll_out, pitch_out, yaw_out = ctrl.update(rate_setpoint, gyro, dt)
    assert abs(roll_out) < 0.1, f"Zero error should give near-zero output: {roll_out}"

    # Test with error
    rate_setpoint = np.array([1.0, 0.0, 0.0])  # 1 rad/s roll rate
    gyro = np.array([0.0, 0.0, 0.0])  # Zero measured rate

    roll_out, pitch_out, yaw_out = ctrl.update(rate_setpoint, gyro, dt)
    assert roll_out > 0, "Positive error should give positive output"

    # Test from stick input
    roll_out, pitch_out, yaw_out = ctrl.update_from_stick(
        stick_roll=0.5,    # Half stick right
        stick_pitch=0.0,
        stick_yaw=0.0,
        gyro=np.array([0.0, 0.0, 0.0]),
        dt=dt,
    )
    assert roll_out > 0, "Right stick should give positive roll output"

    # Test reset
    ctrl.reset()

    print("  ✓ Rate controller tests passed")


def test_motor_mixer():
    """Test motor mixer / モーターミキサーをテスト"""
    print("Testing motor mixer...")

    mixer = MotorMixer()

    # Test hover (throttle only, no control)
    motors = mixer.mix(throttle=0.5, roll=0.0, pitch=0.0, yaw=0.0)
    assert len(motors) == 4
    assert all(abs(m - 0.5) < 0.01 for m in motors), f"Hover should give equal motors: {motors}"

    # Test roll right (positive roll)
    motors = mixer.mix(throttle=0.5, roll=1.0, pitch=0.0, yaw=0.0)
    # Roll right: left motors up, right motors down
    # M1 (FR) and M2 (RR) should decrease, M3 (RL) and M4 (FL) should increase
    assert motors[MotorIndex.M4_FL] > motors[MotorIndex.M1_FR], "Roll right: FL > FR"
    assert motors[MotorIndex.M3_RL] > motors[MotorIndex.M2_RR], "Roll right: RL > RR"

    # Test pitch forward (positive pitch)
    motors = mixer.mix(throttle=0.5, roll=0.0, pitch=1.0, yaw=0.0)
    # Pitch forward: rear motors up, front motors down
    # M1 (FR) and M4 (FL) should increase, M2 (RR) and M3 (RL) should decrease
    assert motors[MotorIndex.M1_FR] > motors[MotorIndex.M2_RR], "Pitch fwd: FR > RR"
    assert motors[MotorIndex.M4_FL] > motors[MotorIndex.M3_RL], "Pitch fwd: FL > RL"

    # Test yaw right (positive yaw)
    motors = mixer.mix(throttle=0.5, roll=0.0, pitch=0.0, yaw=1.0)
    # Yaw right: CCW motors speed up, CW motors slow down
    # M1 (CCW) and M3 (CCW) should increase
    assert motors[MotorIndex.M1_FR] > motors[MotorIndex.M2_RR], "Yaw right: M1 > M2"
    assert motors[MotorIndex.M3_RL] > motors[MotorIndex.M4_FL], "Yaw right: M3 > M4"

    # Test disarmed
    motors = mixer.mix(throttle=0.5, roll=0.0, pitch=0.0, yaw=0.0, armed=False)
    assert all(m == 0 for m in motors), "Disarmed should give zero motors"

    # Test clamping
    motors = mixer.mix(throttle=1.0, roll=10.0, pitch=10.0, yaw=10.0)
    assert all(0 <= m <= 1 for m in motors), "Motors should be clamped to [0, 1]"

    # Test convenience function
    motors2 = mix_motors(0.5, 0.0, 0.0, 0.0)
    assert len(motors2) == 4

    print("  ✓ Motor mixer tests passed")


def test_attitude_controller():
    """Test attitude controller / 姿勢コントローラをテスト"""
    print("Testing attitude controller...")

    ctrl = AttitudeController()

    dt = 0.0025
    attitude = np.array([0.0, 0.0, 0.0])  # Level
    gyro = np.array([0.0, 0.0, 0.0])      # No rotation

    # Test self-leveling (sticks centered)
    roll_out, pitch_out, yaw_out = ctrl.update(
        stick_roll=0.0,
        stick_pitch=0.0,
        stick_yaw=0.0,
        attitude=attitude,
        gyro=gyro,
        dt=dt,
    )
    # Should output near zero when level with zero stick
    assert abs(roll_out) < 0.5, "Level attitude with zero stick should give low output"

    # Test with tilted attitude (should try to level)
    attitude = np.array([0.2, 0.0, 0.0])  # 0.2 rad roll (about 11 degrees)
    roll_out, pitch_out, yaw_out = ctrl.update(
        stick_roll=0.0,   # Want to be level
        stick_pitch=0.0,
        stick_yaw=0.0,
        attitude=attitude,
        gyro=gyro,
        dt=dt,
    )
    # Should output negative roll to correct
    assert roll_out < 0, "Should output negative roll to correct positive tilt"

    # Test reset
    ctrl.reset()

    print("  ✓ Attitude controller tests passed")


def test_altitude_controller():
    """Test altitude controller / 高度コントローラをテスト"""
    print("Testing altitude controller...")

    ctrl = AltitudeController(hover_throttle=0.5)

    # Should return stick throttle when disabled
    throttle = ctrl.update(
        current_altitude=1.0,
        vertical_velocity=0.0,
        stick_throttle=0.6,
        dt=0.02,
    )
    assert throttle == 0.6, "Should return stick throttle when disabled"

    # Enable and set altitude
    ctrl.set_altitude(1.5)
    assert ctrl.is_enabled
    assert ctrl.altitude_setpoint == 1.5

    # Below setpoint, should increase throttle
    throttle = ctrl.update(
        current_altitude=1.0,  # Below setpoint
        vertical_velocity=0.0,
        stick_throttle=0.5,   # Centered
        dt=0.02,
    )
    assert throttle > 0.5, "Should increase throttle when below setpoint"

    # Above setpoint, should decrease throttle
    ctrl.reset()
    ctrl.set_altitude(1.0)
    throttle = ctrl.update(
        current_altitude=1.5,  # Above setpoint
        vertical_velocity=0.0,
        stick_throttle=0.5,
        dt=0.02,
    )
    assert throttle < 0.5, "Should decrease throttle when above setpoint"

    # Test disable
    ctrl.disable()
    assert not ctrl.is_enabled

    print("  ✓ Altitude controller tests passed")


def test_integrated_control():
    """Test integrated control system / 統合制御システムをテスト"""
    print("Testing integrated control system...")

    # Create controllers
    rate_ctrl = RateController()
    mixer = MotorMixer()

    dt = 0.0025  # 400Hz

    # Simulate rate control loop
    gyro = np.array([0.0, 0.0, 0.0])
    throttle = 0.5

    # Apply stick input
    roll_out, pitch_out, yaw_out = rate_ctrl.update_from_stick(
        stick_roll=0.3,
        stick_pitch=-0.2,
        stick_yaw=0.1,
        gyro=gyro,
        dt=dt,
    )

    # Mix to motors
    motors = mixer.mix(throttle, roll_out, pitch_out, yaw_out)

    # All motors should be valid
    assert len(motors) == 4
    assert all(0 <= m <= 1 for m in motors)

    # With positive roll input and zero gyro, roll output should be positive
    assert roll_out > 0, "Positive roll stick should give positive roll output"

    print("  ✓ Integrated control tests passed")


def main():
    """Run all tests / すべてのテストを実行"""
    print("=" * 60)
    print("Control System Tests / 制御システムテスト")
    print("=" * 60)
    print()

    try:
        test_pid_basic()
        test_pid_derivative_filter()
        test_pid_antiwindup()
        test_rate_controller()
        test_motor_mixer()
        test_attitude_controller()
        test_altitude_controller()
        test_integrated_control()

        print()
        print("=" * 60)
        print("All tests passed! / すべてのテストに合格！")
        print("=" * 60)
        return 0

    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        return 1
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
