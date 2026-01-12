#!/usr/bin/env python3
"""
Control Allocation Verification Test
制御アロケーション検証テスト

Verifies that the firmware control allocation matches the simulator implementation.
ファームウェアの制御アロケーションがシミュレータの実装と一致することを検証。

This test compares:
1. Allocation matrix B and inverse B⁻¹
2. Thrust-to-duty conversion
3. Hover duty values

@see docs/architecture/control-allocation-migration.md
@see firmware/vehicle/components/sf_algo_control/
"""

import numpy as np
import sys
import os

# Add simulator path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.motors import motor_prop


# =============================================================================
# Firmware Parameters (from control_allocation.hpp)
# =============================================================================

class FirmwareParams:
    """Parameters matching firmware implementation"""
    # QuadConfig defaults
    d = 0.023  # Moment arm [m]
    kappa = 9.71e-3  # Cq/Ct [m]
    motor_x = [0.023, -0.023, -0.023, 0.023]  # M1:FR, M2:RR, M3:RL, M4:FL
    motor_y = [0.023, 0.023, -0.023, -0.023]
    motor_dir = [-1, 1, -1, 1]  # CCW, CW, CCW, CW
    max_thrust_per_motor = 0.15  # [N]

    # MotorParams defaults
    Ct = 1.0e-8  # [N/(rad/s)²]
    Cq = 9.71e-11  # [Nm/(rad/s)²]
    Rm = 0.34  # [Ω]
    Km = 6.125e-4  # [V·s/rad]
    Dm = 3.69e-8  # [Nm·s/rad]
    Qf = 2.76e-5  # [Nm]
    Vbat = 3.7  # [V]


def build_allocation_matrix_firmware():
    """
    Build allocation matrix B as in firmware control_allocation.cpp:buildMatrices()

    B maps [T1, T2, T3, T4] -> [u_thrust, u_roll, u_pitch, u_yaw]
    """
    p = FirmwareParams()
    B = np.zeros((4, 4))

    for i in range(4):
        B[0, i] = 1.0  # Total thrust
        B[1, i] = -p.motor_y[i]  # Roll torque (around X-axis)
        B[2, i] = p.motor_x[i]   # Pitch torque (around Y-axis)
        B[3, i] = -p.motor_dir[i] * p.kappa  # Yaw torque

    return B


def build_inverse_matrix_firmware():
    """
    Build inverse matrix B⁻¹ as in firmware control_allocation.cpp:buildMatrices()

    B⁻¹ maps [u_thrust, u_roll, u_pitch, u_yaw] -> [T1, T2, T3, T4]
    """
    p = FirmwareParams()
    d = p.d
    kappa = p.kappa
    inv_d = 1.0 / d
    inv_kappa = 1.0 / kappa

    B_inv = np.zeros((4, 4))

    # M1 (FR): +x, +y, CCW (σ=-1)
    B_inv[0, 0] = 0.25
    B_inv[0, 1] = -0.25 * inv_d
    B_inv[0, 2] = 0.25 * inv_d
    B_inv[0, 3] = 0.25 * inv_kappa

    # M2 (RR): -x, +y, CW (σ=+1)
    B_inv[1, 0] = 0.25
    B_inv[1, 1] = -0.25 * inv_d
    B_inv[1, 2] = -0.25 * inv_d
    B_inv[1, 3] = -0.25 * inv_kappa

    # M3 (RL): -x, -y, CCW (σ=-1)
    B_inv[2, 0] = 0.25
    B_inv[2, 1] = 0.25 * inv_d
    B_inv[2, 2] = -0.25 * inv_d
    B_inv[2, 3] = 0.25 * inv_kappa

    # M4 (FL): +x, -y, CW (σ=+1)
    B_inv[3, 0] = 0.25
    B_inv[3, 1] = 0.25 * inv_d
    B_inv[3, 2] = 0.25 * inv_d
    B_inv[3, 3] = -0.25 * inv_kappa

    return B_inv


def thrust_to_omega(thrust: float, Ct: float) -> float:
    """Convert thrust [N] to angular velocity [rad/s]"""
    if thrust <= 0:
        return 0.0
    return np.sqrt(thrust / Ct)


def omega_to_voltage(omega: float, params: FirmwareParams) -> float:
    """
    Convert angular velocity to voltage (steady-state approximation)
    From motor_model.hpp:omegaToVoltage()
    """
    p = params
    return p.Rm * ((p.Dm + p.Km**2 / p.Rm) * omega + p.Cq * omega**2 + p.Qf) / p.Km


def thrust_to_duty_firmware(thrust: float, params: FirmwareParams = None) -> float:
    """
    Convert thrust [N] to duty cycle [0-1]
    Matches firmware motor_model.hpp:thrustToDuty()
    """
    if params is None:
        params = FirmwareParams()

    if thrust <= 0:
        return 0.0

    omega = thrust_to_omega(thrust, params.Ct)
    voltage = omega_to_voltage(omega, params)
    duty = voltage / params.Vbat

    return np.clip(duty, 0.0, 1.0)


def thrust_to_duty_simulator(thrust: float) -> float:
    """
    Convert thrust to duty using simulator's motor_prop class
    """
    motor = motor_prop(1)
    if thrust <= 0:
        return 0.0
    voltage = motor.equilibrium_voltage(thrust)
    duty = voltage / FirmwareParams.Vbat
    return np.clip(duty, 0.0, 1.0)


# =============================================================================
# Tests
# =============================================================================

def test_allocation_matrix():
    """Test 1: Verify allocation matrix B"""
    print("\n" + "="*60)
    print("Test 1: Allocation Matrix B")
    print("="*60)

    B = build_allocation_matrix_firmware()

    print("\nFirmware B matrix:")
    print(B)

    # Verify B × B⁻¹ = I
    B_inv = build_inverse_matrix_firmware()
    identity = B @ B_inv

    print("\nB × B⁻¹ (should be identity):")
    print(identity)

    is_identity = np.allclose(identity, np.eye(4), atol=1e-6)
    print(f"\n✓ B × B⁻¹ = I: {is_identity}")

    return is_identity


def test_inverse_matrix():
    """Test 2: Verify inverse matrix B⁻¹"""
    print("\n" + "="*60)
    print("Test 2: Inverse Matrix B⁻¹")
    print("="*60)

    B_inv = build_inverse_matrix_firmware()

    print("\nFirmware B⁻¹ matrix:")
    print(B_inv)

    # Check against direct matrix inverse
    B = build_allocation_matrix_firmware()
    B_inv_numpy = np.linalg.inv(B)

    print("\nNumPy inv(B):")
    print(B_inv_numpy)

    is_close = np.allclose(B_inv, B_inv_numpy, atol=1e-6)
    print(f"\n✓ Firmware B⁻¹ matches numpy inv(B): {is_close}")

    return is_close


def test_hover_mixing():
    """Test 3: Verify hover mixing (equal thrust distribution)"""
    print("\n" + "="*60)
    print("Test 3: Hover Mixing")
    print("="*60)

    p = FirmwareParams()
    hover_thrust = 0.035 * 9.81  # 35g × g = 0.343 N

    print(f"\nHover thrust (total): {hover_thrust:.4f} N")
    print(f"Hover thrust (per motor): {hover_thrust/4:.4f} N")

    # Control input for hover: [total_thrust, 0, 0, 0]
    control = np.array([hover_thrust, 0.0, 0.0, 0.0])

    # Mix to get motor thrusts
    B_inv = build_inverse_matrix_firmware()
    thrusts = B_inv @ control

    print(f"\nMotor thrusts [T1, T2, T3, T4]: {thrusts}")

    # Verify equal distribution
    expected = hover_thrust / 4
    is_equal = np.allclose(thrusts, expected, atol=1e-6)
    print(f"\n✓ Equal thrust distribution: {is_equal}")

    return is_equal


def test_thrust_to_duty_comparison():
    """Test 4: Compare firmware and simulator thrust-to-duty conversion"""
    print("\n" + "="*60)
    print("Test 4: Thrust-to-Duty Comparison")
    print("="*60)

    test_thrusts = [0.0, 0.02, 0.05, 0.0857, 0.10, 0.15]

    print("\n  Thrust [N]   Firmware Duty   Simulator Duty   Difference")
    print("-" * 60)

    max_diff = 0.0
    for T in test_thrusts:
        duty_fw = thrust_to_duty_firmware(T)
        duty_sim = thrust_to_duty_simulator(T)
        diff = abs(duty_fw - duty_sim)
        max_diff = max(max_diff, diff)
        print(f"  {T:8.4f}      {duty_fw:8.4f}        {duty_sim:8.4f}        {diff:8.4f}")

    # Allow 5% difference due to parameter variations
    is_close = max_diff < 0.05
    print(f"\n✓ Max difference < 5%: {is_close} (max={max_diff:.4f})")

    return is_close


def test_hover_duty():
    """Test 5: Verify hover duty cycle"""
    print("\n" + "="*60)
    print("Test 5: Hover Duty Verification")
    print("="*60)

    p = FirmwareParams()
    hover_thrust_total = 0.035 * 9.81  # 0.343 N
    hover_thrust_per_motor = hover_thrust_total / 4  # 0.0857 N

    duty_fw = thrust_to_duty_firmware(hover_thrust_per_motor)
    duty_sim = thrust_to_duty_simulator(hover_thrust_per_motor)

    # Expected: ~63% (calculated with full motor model)
    # Note: Documentation may have older value of 75% using simplified model
    expected_duty = 0.63

    print(f"\nHover thrust per motor: {hover_thrust_per_motor:.4f} N")
    print(f"Firmware duty: {duty_fw:.4f} ({duty_fw*100:.1f}%)")
    print(f"Simulator duty: {duty_sim:.4f} ({duty_sim*100:.1f}%)")
    print(f"Expected: {expected_duty:.4f} ({expected_duty*100:.1f}%)")

    # Allow 5% tolerance
    is_close = abs(duty_fw - expected_duty) < 0.05
    print(f"\n✓ Hover duty ≈ 63%: {is_close}")

    return is_close


def test_roll_torque():
    """Test 6: Verify roll torque mixing"""
    print("\n" + "="*60)
    print("Test 6: Roll Torque Mixing")
    print("="*60)

    p = FirmwareParams()
    hover_thrust = 0.343  # N
    roll_torque = 1e-4  # 0.1 mNm

    control = np.array([hover_thrust, roll_torque, 0.0, 0.0])

    B_inv = build_inverse_matrix_firmware()
    thrusts = B_inv @ control

    print(f"\nControl input: thrust={hover_thrust:.3f}N, roll_torque={roll_torque*1000:.2f}mNm")
    print(f"Motor thrusts [T1, T2, T3, T4]: {thrusts}")

    # For positive roll torque (right wing down):
    # Left motors (M3, M4) should increase
    # Right motors (M1, M2) should decrease
    left_avg = (thrusts[2] + thrusts[3]) / 2  # M3, M4
    right_avg = (thrusts[0] + thrusts[1]) / 2  # M1, M2

    print(f"\nLeft motors avg: {left_avg:.4f} N")
    print(f"Right motors avg: {right_avg:.4f} N")
    print(f"Difference: {(left_avg - right_avg)*1000:.2f} mN")

    # Verify B × thrusts recovers original control
    B = build_allocation_matrix_firmware()
    recovered = B @ thrusts

    print(f"\nRecovered control: {recovered}")
    is_close = np.allclose(recovered, control, atol=1e-8)
    print(f"\n✓ B × T = u (round-trip): {is_close}")

    return is_close


def test_yaw_torque():
    """Test 7: Verify yaw torque mixing"""
    print("\n" + "="*60)
    print("Test 7: Yaw Torque Mixing")
    print("="*60)

    p = FirmwareParams()
    hover_thrust = 0.343  # N
    yaw_torque = 1e-5  # 0.01 mNm

    control = np.array([hover_thrust, 0.0, 0.0, yaw_torque])

    B_inv = build_inverse_matrix_firmware()
    thrusts = B_inv @ control

    print(f"\nControl input: thrust={hover_thrust:.3f}N, yaw_torque={yaw_torque*1000:.3f}mNm")
    print(f"Motor thrusts [T1, T2, T3, T4]: {thrusts}")

    # For positive yaw torque (nose right):
    # CCW motors (M1, M3) should increase (produce more CW reaction)
    # CW motors (M2, M4) should decrease
    ccw_avg = (thrusts[0] + thrusts[2]) / 2  # M1, M3
    cw_avg = (thrusts[1] + thrusts[3]) / 2   # M2, M4

    print(f"\nCCW motors (M1,M3) avg: {ccw_avg:.4f} N")
    print(f"CW motors (M2,M4) avg: {cw_avg:.4f} N")

    # Verify round-trip
    B = build_allocation_matrix_firmware()
    recovered = B @ thrusts

    is_close = np.allclose(recovered, control, atol=1e-8)
    print(f"\n✓ B × T = u (round-trip): {is_close}")

    return is_close


def run_all_tests():
    """Run all verification tests"""
    print("\n" + "#"*60)
    print("# Control Allocation Verification Tests")
    print("# 制御アロケーション検証テスト")
    print("#"*60)

    results = []

    results.append(("Allocation Matrix B", test_allocation_matrix()))
    results.append(("Inverse Matrix B⁻¹", test_inverse_matrix()))
    results.append(("Hover Mixing", test_hover_mixing()))
    results.append(("Thrust-to-Duty Comparison", test_thrust_to_duty_comparison()))
    results.append(("Hover Duty", test_hover_duty()))
    results.append(("Roll Torque Mixing", test_roll_torque()))
    results.append(("Yaw Torque Mixing", test_yaw_torque()))

    print("\n" + "="*60)
    print("Summary / サマリー")
    print("="*60)

    all_passed = True
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {status}: {name}")
        if not passed:
            all_passed = False

    print("\n" + "="*60)
    if all_passed:
        print("All tests passed! ファームウェアとシミュレータの実装が一致しています。")
    else:
        print("Some tests failed. Check implementation differences.")
    print("="*60)

    return all_passed


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
