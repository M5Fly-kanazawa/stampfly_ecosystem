#!/usr/bin/env python3
"""
debug_control_loop.py - Debug control loop sign conventions
制御ループの符号規約をデバッグ

Test the full control loop to find where the sign error is.
完全な制御ループをテストして符号エラーの場所を特定。
"""

import sys
from pathlib import Path

script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir.parent))
sys.path.insert(0, str(script_dir.parent.parent.parent))

import numpy as np

from motor_model import QuadMotorSystem
from control_allocation import ControlAllocator, thrusts_to_duties
from control.pid import PID

GRAVITY = 9.81
MASS = 0.035
WEIGHT = MASS * GRAVITY


def genesis_gyro_to_ned(gyro_genesis_body):
    """Genesis body (wx, wy, wz) -> NED body (p, q, r)"""
    return np.array([gyro_genesis_body[1], gyro_genesis_body[0], -gyro_genesis_body[2]])


def ned_to_genesis_moment(moment_ned):
    """NED body moment -> Genesis body moment"""
    return np.array([moment_ned[1], moment_ned[0], -moment_ned[2]])


def main():
    print("=" * 70)
    print("Debug: Control Loop Sign Convention Test")
    print("=" * 70)

    # Initialize components
    allocator = ControlAllocator()
    motor_system = QuadMotorSystem()

    # Initialize motors at hover
    from motor_model import compute_hover_conditions
    hover = compute_hover_conditions()
    for motor in motor_system.motors:
        motor.omega = hover['omega_hover']

    # Simple P controller (no I, no D)
    Kp_roll = 9.1e-4  # Nm/(rad/s)
    roll_pid = PID(Kp=Kp_roll, Ti=0, Td=0)

    print("\n" + "=" * 70)
    print("Test: Simulated positive roll rate disturbance")
    print("=" * 70)

    # Simulate: drone has positive roll rate (rolling right)
    # p = +1 rad/s (57 deg/s)
    gyro_ned = np.array([1.0, 0.0, 0.0])  # p=+1 rad/s, q=0, r=0

    print(f"\n1. Disturbance: p = +{np.degrees(gyro_ned[0]):.0f} deg/s (rolling right)")

    # PID update (setpoint = 0, stick neutral)
    setpoint = 0.0
    roll_torque = roll_pid.update(setpoint, gyro_ned[0], 0.0025)

    print(f"\n2. PID output:")
    print(f"   setpoint = {setpoint}")
    print(f"   measurement (p) = {gyro_ned[0]:.2f} rad/s")
    print(f"   error = setpoint - measurement = {setpoint - gyro_ned[0]:.2f}")
    print(f"   roll_torque = Kp * error = {Kp_roll:.2e} * {setpoint - gyro_ned[0]:.2f} = {roll_torque:.2e} Nm")
    print(f"   roll_torque = {roll_torque*1000:.3f} mNm")

    expected_torque_sign = "NEGATIVE (to counter positive roll rate)"
    actual_sign = "NEGATIVE" if roll_torque < 0 else "POSITIVE"
    print(f"\n   Expected: {expected_torque_sign}")
    print(f"   Actual: {actual_sign}")

    # Control allocation
    control = np.array([WEIGHT, roll_torque, 0.0, 0.0])
    target_thrusts = allocator.mix(control)

    print(f"\n3. Control allocation:")
    print(f"   control = [thrust={WEIGHT:.3f}N, τ_roll={roll_torque*1000:.3f}mNm, τ_pitch=0, τ_yaw=0]")
    print(f"   target_thrusts = {[f'{t*1000:.2f}mN' for t in target_thrusts]}")
    print(f"   M1(FR)={target_thrusts[0]*1000:.2f}mN, M2(RR)={target_thrusts[1]*1000:.2f}mN, "
          f"M3(RL)={target_thrusts[2]*1000:.2f}mN, M4(FL)={target_thrusts[3]*1000:.2f}mN")

    # Expected: negative roll torque -> increase M1,M2 (right side), decrease M3,M4 (left side)
    hover_thrust = WEIGHT / 4
    print(f"\n   Hover thrust per motor: {hover_thrust*1000:.2f}mN")
    print(f"   M1,M2 (right side) change: {(target_thrusts[0] - hover_thrust)*1000:+.2f}mN (expect INCREASE)")
    print(f"   M3,M4 (left side) change: {(target_thrusts[2] - hover_thrust)*1000:+.2f}mN (expect DECREASE)")

    # Convert to duties and run motor model
    target_duties = thrusts_to_duties(target_thrusts)
    force_ned, moment_ned = motor_system.step_with_duty(target_duties.tolist(), 0.001)

    print(f"\n4. Motor model output (NED):")
    print(f"   force_ned = {force_ned}")
    print(f"   moment_ned = {moment_ned}")
    print(f"   roll_moment (τx) = {moment_ned[0]*1000:.3f} mNm")

    expected_moment_sign = "NEGATIVE (same as PID output)"
    actual_moment_sign = "NEGATIVE" if moment_ned[0] < 0 else "POSITIVE"
    print(f"\n   Expected roll moment: {expected_moment_sign}")
    print(f"   Actual roll moment: {actual_moment_sign}")

    # Convert to Genesis
    moment_genesis = ned_to_genesis_moment(moment_ned)
    print(f"\n5. Genesis conversion:")
    print(f"   moment_genesis = {moment_genesis}")
    print(f"   Genesis τy (roll axis) = {moment_genesis[1]*1000:.3f} mNm")

    # Summary
    print("\n" + "=" * 70)
    print("Summary: Sign chain analysis")
    print("=" * 70)
    print(f"  Disturbance: p > 0 (rolling right)")
    print(f"  → PID error: {setpoint - gyro_ned[0]:+.2f} (negative)")
    print(f"  → PID output (τ_roll): {roll_torque*1000:+.3f} mNm")
    print(f"  → Motor thrusts: Right↑ Left↓" if roll_torque < 0 else "  → Motor thrusts: Right↓ Left↑")
    print(f"  → Motor moment (NED τx): {moment_ned[0]*1000:+.3f} mNm")
    print(f"  → Genesis moment (τy): {moment_genesis[1]*1000:+.3f} mNm")

    if roll_torque < 0 and moment_ned[0] < 0:
        print("\n✓ Control chain is CORRECT (negative feedback)")
        print("  Negative torque should slow down positive roll rate")
    else:
        print("\n✗ Control chain has SIGN ERROR (positive feedback)")
        print("  This would cause divergence!")

    # Test: What happens in Genesis when we apply this moment?
    print("\n" + "=" * 70)
    print("Expected Genesis response:")
    print("=" * 70)
    print(f"  Applied Genesis τy = {moment_genesis[1]*1000:+.3f} mNm")
    print(f"  Genesis: τy > 0 → wy increases (roll rate increases)")
    print(f"  Genesis: τy < 0 → wy decreases (roll rate decreases)")
    print(f"  Current: τy = {moment_genesis[1]*1000:+.3f} mNm → wy should {'INCREASE' if moment_genesis[1] > 0 else 'DECREASE'}")
    print(f"  We want: wy (roll rate) to DECREASE")

    if moment_genesis[1] < 0:
        print("\n✓ Genesis will receive negative τy, causing wy to decrease. CORRECT!")
    else:
        print("\n✗ Genesis will receive positive τy, causing wy to INCREASE. WRONG!")
        print("  This is the bug! The moment sign is inverted somewhere.")


if __name__ == "__main__":
    main()
