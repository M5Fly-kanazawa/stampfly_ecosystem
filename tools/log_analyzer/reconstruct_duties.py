#!/usr/bin/env python3
"""
Reconstruct Motor Duties from Flight Log
フライトログからモータDuty値を再現

Uses the same equations as firmware to reconstruct motor outputs.
ファームウェアと同じ式でモータ出力を再現。
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

# =============================================================================
# Firmware Parameters (from config.hpp, motor_model.hpp, control_allocation.hpp)
# =============================================================================

# Rate control parameters
ROLL_RATE_MAX = 1.0    # rad/s
PITCH_RATE_MAX = 1.0   # rad/s
YAW_RATE_MAX = 5.0     # rad/s

# PID gains (Physical units, Ti=0 test)
ROLL_KP = 9.1e-4       # Nm/(rad/s)
ROLL_TI = 0.0          # s (disabled)
ROLL_TD = 0.01         # s
PITCH_KP = 1.33e-3     # Nm/(rad/s)
PITCH_TI = 0.0         # s (disabled)
PITCH_TD = 0.025       # s
YAW_KP = 1.77e-3       # Nm/(rad/s)
YAW_TI = 0.0           # s (disabled)
YAW_TD = 0.01          # s
PID_ETA = 0.125

# Output limits [Nm]
ROLL_OUTPUT_LIMIT = 5.2e-3
PITCH_OUTPUT_LIMIT = 5.2e-3
YAW_OUTPUT_LIMIT = 2.2e-3

# Quad config
D = 0.023              # moment arm [m]
KAPPA = 9.71e-3        # torque/thrust ratio [m]
MAX_THRUST_PER_MOTOR = 0.15  # N
MAX_TOTAL_THRUST = 4 * MAX_THRUST_PER_MOTOR  # 0.6 N

# Motor params (for thrust to duty conversion)
CT = 1.0e-8            # N/(rad/s)^2
CQ = 9.71e-11          # Nm/(rad/s)^2
RM = 0.34              # Ohm
KM = 6.125e-4          # V*s/rad
DM = 3.69e-8           # Nm*s/rad
QF = 2.76e-5           # Nm
VBAT = 3.7             # V

# =============================================================================
# Mixing Matrix B_inv (from control_allocation.cpp)
# =============================================================================
# T = B_inv * [total_thrust, roll_torque, pitch_torque, yaw_torque]
# M1(FR), M2(RR), M3(RL), M4(FL)

inv_d = 1.0 / D
inv_kappa = 1.0 / KAPPA

B_INV = np.array([
    [0.25, -0.25*inv_d,  0.25*inv_d,  0.25*inv_kappa],  # M1 (FR)
    [0.25, -0.25*inv_d, -0.25*inv_d, -0.25*inv_kappa],  # M2 (RR)
    [0.25,  0.25*inv_d, -0.25*inv_d,  0.25*inv_kappa],  # M3 (RL)
    [0.25,  0.25*inv_d,  0.25*inv_d, -0.25*inv_kappa],  # M4 (FL)
])

# =============================================================================
# Helper Functions
# =============================================================================

def thrust_to_omega(thrust):
    """Thrust -> Angular velocity (steady-state)"""
    if thrust <= 0:
        return 0.0
    return np.sqrt(thrust / CT)

def omega_to_voltage(omega):
    """Angular velocity -> Required voltage (steady-state)"""
    if omega <= 0:
        return 0.0
    viscous_term = (DM + KM * KM / RM) * omega
    aero_term = CQ * omega * omega
    friction_term = QF
    voltage = RM * (viscous_term + aero_term + friction_term) / KM
    return voltage

def thrust_to_duty(thrust):
    """Thrust -> Duty cycle"""
    if thrust <= 0:
        return 0.0
    omega = thrust_to_omega(thrust)
    voltage = omega_to_voltage(omega)
    duty = voltage / VBAT
    return np.clip(duty, 0.0, 1.0)

def mix(control):
    """
    Control allocation (mixing)
    control = [total_thrust, roll_torque, pitch_torque, yaw_torque]
    Returns: thrusts[4], saturated flag
    """
    thrusts = B_INV @ control
    saturated = False

    for i in range(4):
        if thrusts[i] < 0:
            thrusts[i] = 0
            saturated = True
        elif thrusts[i] > MAX_THRUST_PER_MOTOR:
            thrusts[i] = MAX_THRUST_PER_MOTOR
            saturated = True

    return thrusts, saturated

class PIDController:
    """Simple P+D controller (Ti=0)"""
    def __init__(self, Kp, Td, eta, output_limit):
        self.Kp = Kp
        self.Td = Td
        self.eta = eta
        self.output_limit = output_limit
        self.prev_measurement = 0.0
        self.deriv_filtered = 0.0
        self.first_run = True

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement

        # P term
        P = self.Kp * error

        # D term (derivative on measurement, incomplete derivative)
        if self.Td > 0 and dt > 0 and not self.first_run:
            deriv_input = -measurement
            alpha = 2.0 * self.eta * self.Td / dt
            deriv_a = (alpha - 1.0) / (alpha + 1.0)
            deriv_b = 2.0 * self.Td / ((alpha + 1.0) * dt)
            deriv_diff = deriv_input - self.prev_deriv_input
            self.deriv_filtered = deriv_a * self.deriv_filtered + deriv_b * deriv_diff
            D = self.Kp * self.deriv_filtered
        else:
            D = 0.0
            self.prev_deriv_input = -measurement

        self.prev_deriv_input = -measurement
        self.first_run = False

        output = P + D
        return np.clip(output, -self.output_limit, self.output_limit)

    def reset(self):
        self.prev_measurement = 0.0
        self.deriv_filtered = 0.0
        self.first_run = True

# =============================================================================
# Main Analysis
# =============================================================================

def reconstruct_duties(csv_path):
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} samples from {csv_path}")

    # Time
    df['time_s'] = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    dt = 1.0 / 400.0  # 400 Hz

    # Initialize PIDs
    roll_pid = PIDController(ROLL_KP, ROLL_TD, PID_ETA, ROLL_OUTPUT_LIMIT)
    pitch_pid = PIDController(PITCH_KP, PITCH_TD, PID_ETA, PITCH_OUTPUT_LIMIT)
    yaw_pid = PIDController(YAW_KP, YAW_TD, PID_ETA, YAW_OUTPUT_LIMIT)

    # Output arrays
    n = len(df)
    duties = np.zeros((n, 4))
    thrusts = np.zeros((n, 4))
    saturated = np.zeros(n, dtype=bool)
    torques = np.zeros((n, 3))  # roll, pitch, yaw torques

    for i in range(n):
        # Get inputs
        throttle = df['ctrl_throttle'].iloc[i]
        roll_cmd = df['ctrl_roll'].iloc[i]
        pitch_cmd = df['ctrl_pitch'].iloc[i]
        yaw_cmd = df['ctrl_yaw'].iloc[i]

        # Get gyro (already bias-corrected)
        gyro_x = df['gyro_corrected_x'].iloc[i]  # rad/s
        gyro_y = df['gyro_corrected_y'].iloc[i]
        gyro_z = df['gyro_corrected_z'].iloc[i]

        # ACRO mode: stick -> rate setpoint
        roll_rate_target = roll_cmd * ROLL_RATE_MAX
        pitch_rate_target = pitch_cmd * PITCH_RATE_MAX
        yaw_rate_target = yaw_cmd * YAW_RATE_MAX

        # PID control
        roll_out = roll_pid.update(roll_rate_target, gyro_x, dt)
        pitch_out = pitch_pid.update(pitch_rate_target, gyro_y, dt)
        yaw_out = yaw_pid.update(yaw_rate_target, gyro_z, dt)

        torques[i] = [roll_out, pitch_out, yaw_out]

        # Total thrust
        total_thrust = throttle * MAX_TOTAL_THRUST

        # Control vector
        control = np.array([total_thrust, roll_out, pitch_out, yaw_out])

        # Mixing
        motor_thrusts, sat = mix(control)
        thrusts[i] = motor_thrusts
        saturated[i] = sat

        # Thrust to Duty
        for j in range(4):
            duties[i, j] = thrust_to_duty(motor_thrusts[j])

    # Add to dataframe
    df['duty_m1'] = duties[:, 0]
    df['duty_m2'] = duties[:, 1]
    df['duty_m3'] = duties[:, 2]
    df['duty_m4'] = duties[:, 3]
    df['thrust_m1'] = thrusts[:, 0]
    df['thrust_m2'] = thrusts[:, 1]
    df['thrust_m3'] = thrusts[:, 2]
    df['thrust_m4'] = thrusts[:, 3]
    df['saturated'] = saturated
    df['torque_roll'] = torques[:, 0]
    df['torque_pitch'] = torques[:, 1]
    df['torque_yaw'] = torques[:, 2]

    # Statistics
    sat_pct = 100 * saturated.sum() / n
    print(f"\n=== Saturation Analysis ===")
    print(f"Saturated samples: {saturated.sum()} / {n} ({sat_pct:.1f}%)")

    print(f"\n=== Duty Statistics ===")
    for j, name in enumerate(['M1(FR)', 'M2(RR)', 'M3(RL)', 'M4(FL)']):
        d = duties[:, j]
        print(f"{name}: mean={d.mean():.3f}, min={d.min():.3f}, max={d.max():.3f}")

    # Visualization
    fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)

    # 1. Motor Duties
    ax1 = axes[0]
    ax1.plot(df['time_s'], df['duty_m1'], label='M1 (FR)', alpha=0.8)
    ax1.plot(df['time_s'], df['duty_m2'], label='M2 (RR)', alpha=0.8)
    ax1.plot(df['time_s'], df['duty_m3'], label='M3 (RL)', alpha=0.8)
    ax1.plot(df['time_s'], df['duty_m4'], label='M4 (FL)', alpha=0.8)
    ax1.axhline(y=1.0, color='r', linestyle='--', linewidth=1, label='Saturation')
    ax1.axhline(y=0.0, color='k', linestyle='-', linewidth=0.5)
    ax1.set_ylabel('Duty [0-1]')
    ax1.set_title('Reconstructed Motor Duties')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([-0.1, 1.1])

    # 2. Saturation indicator
    ax2 = axes[1]
    ax2.fill_between(df['time_s'], 0, df['saturated'].astype(float),
                     alpha=0.5, color='red', label='Saturated')
    ax2.plot(df['time_s'], df['ctrl_throttle'], label='Throttle', color='blue')
    ax2.set_ylabel('Saturation / Throttle')
    ax2.set_title('Mixing Saturation and Throttle')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([-0.1, 1.1])

    # 3. Motor Thrusts
    ax3 = axes[2]
    ax3.plot(df['time_s'], df['thrust_m1']*1000, label='M1 (FR)', alpha=0.8)
    ax3.plot(df['time_s'], df['thrust_m2']*1000, label='M2 (RR)', alpha=0.8)
    ax3.plot(df['time_s'], df['thrust_m3']*1000, label='M3 (RL)', alpha=0.8)
    ax3.plot(df['time_s'], df['thrust_m4']*1000, label='M4 (FL)', alpha=0.8)
    ax3.axhline(y=MAX_THRUST_PER_MOTOR*1000, color='r', linestyle='--', linewidth=1, label='Max')
    ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax3.set_ylabel('Thrust [mN]')
    ax3.set_title('Motor Thrusts')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)

    # 4. PID Torque Outputs
    ax4 = axes[3]
    ax4.plot(df['time_s'], df['torque_roll']*1000, label='Roll τ', alpha=0.8)
    ax4.plot(df['time_s'], df['torque_pitch']*1000, label='Pitch τ', alpha=0.8)
    ax4.plot(df['time_s'], df['torque_yaw']*1000, label='Yaw τ', alpha=0.8)
    ax4.axhline(y=ROLL_OUTPUT_LIMIT*1000, color='r', linestyle='--', linewidth=0.5)
    ax4.axhline(y=-ROLL_OUTPUT_LIMIT*1000, color='r', linestyle='--', linewidth=0.5)
    ax4.set_ylabel('Torque [mNm]')
    ax4.set_title('PID Torque Outputs')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)

    # 5. Duty difference (diagonal motors)
    ax5 = axes[4]
    # M1-M3 (FR-RL diagonal) for roll
    # M2-M4 (RR-FL diagonal) for roll (opposite)
    duty_diff_13 = df['duty_m1'] - df['duty_m3']  # Should be similar for roll control
    duty_diff_24 = df['duty_m2'] - df['duty_m4']
    ax5.plot(df['time_s'], duty_diff_13, label='M1-M3 (FR-RL)', alpha=0.8)
    ax5.plot(df['time_s'], duty_diff_24, label='M2-M4 (RR-FL)', alpha=0.8)
    ax5.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax5.set_ylabel('Duty Difference')
    ax5.set_xlabel('Time [s]')
    ax5.set_title('Diagonal Motor Duty Differences (Control Authority)')
    ax5.legend(loc='upper right')
    ax5.grid(True, alpha=0.3)

    plt.tight_layout()

    output_path = csv_path.replace('.csv', '_duties.png')
    plt.savefig(output_path, dpi=150)
    print(f"\nSaved: {output_path}")

    # Time segment analysis
    print(f"\n=== Saturation by Time Window ===")
    for start in np.arange(0, df['time_s'].max() - 5, 5):
        end = start + 5
        mask = (df['time_s'] >= start) & (df['time_s'] < end)
        seg = df[mask]
        sat_count = seg['saturated'].sum()
        sat_pct = 100 * sat_count / len(seg) if len(seg) > 0 else 0
        throttle_mean = seg['ctrl_throttle'].mean()
        print(f"{start:5.0f}-{end:5.0f}s: Saturated={sat_pct:5.1f}%, Throttle={throttle_mean:.2f}")

    plt.show()
    return df

if __name__ == "__main__":
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        csv_path = "/Users/kouhei/Library/CloudStorage/Dropbox/01教育研究/20マルチコプタ/stampfly_ecosystem/tools/log_analyzer/stampfly_fft_20260114T124517.csv"

    reconstruct_duties(csv_path)
