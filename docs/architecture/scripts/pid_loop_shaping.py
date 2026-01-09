#!/usr/bin/env python3
"""
PID Controller Loop Shaping Design for StampFly Angular Velocity Control

This script designs a PID controller using loop shaping method
with specified phase margin and gain crossover frequency.

PID Controller Form (with incomplete differentiation):
    C(s) = Kp * (1 + 1/(Ti*s) + Td*s/(eta*Td*s + 1))

Plant (Roll axis):
    G(s) = (1/Ixx) / (s*(tau_m*s + 1))
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import fsolve
import os

# Plant parameters (Roll axis)
I_xx = 9.16e-6      # kg*m^2
tau_m = 0.02        # s (motor time constant)
K_plant = 1 / I_xx  # 1.09e5

# Create output directory
script_dir = os.path.dirname(os.path.abspath(__file__))
output_dir = os.path.join(script_dir, '..', 'images')
os.makedirs(output_dir, exist_ok=True)


def pid_controller(Kp, Ti, Td, eta=0.1):
    """
    Create PID controller transfer function
    C(s) = Kp * (1 + 1/(Ti*s) + Td*s/(eta*Td*s + 1))

    Returns numerator and denominator coefficients
    """
    # C(s) = Kp * [Ti*s*(eta*Td*s + 1) + (eta*Td*s + 1) + Ti*Td*s^2] / [Ti*s*(eta*Td*s + 1)]
    # Numerator: Kp * [eta*Ti*Td*s^2 + Ti*s + eta*Td*s + 1 + Ti*Td*s^2]
    #          = Kp * [(eta*Ti*Td + Ti*Td)*s^2 + (Ti + eta*Td)*s + 1]
    #          = Kp * [Ti*Td*(eta + 1)*s^2 + (Ti + eta*Td)*s + 1]
    # Denominator: Ti*s*(eta*Td*s + 1) = eta*Ti*Td*s^2 + Ti*s

    num = Kp * np.array([Ti*Td*(eta + 1), Ti + eta*Td, 1])
    den = np.array([eta*Ti*Td, Ti, 0])

    return num, den


def plant_tf():
    """
    Plant transfer function
    G(s) = K_plant / (s*(tau_m*s + 1))
         = K_plant / (tau_m*s^2 + s)
    """
    num = np.array([K_plant])
    den = np.array([tau_m, 1, 0])
    return num, den


def open_loop_tf(Kp, Ti, Td, eta=0.1):
    """
    Open loop transfer function L(s) = C(s) * G(s)
    """
    c_num, c_den = pid_controller(Kp, Ti, Td, eta)
    g_num, g_den = plant_tf()

    # Convolve to multiply transfer functions
    l_num = np.convolve(c_num, g_num)
    l_den = np.convolve(c_den, g_den)

    return l_num, l_den


def freq_response(num, den, omega):
    """Calculate frequency response at given frequencies"""
    s = 1j * omega
    H = np.polyval(num, s) / np.polyval(den, s)
    return H


def design_pid_loop_shaping(omega_gc, PM_deg, eta=0.1):
    """
    Design PID controller for specified gain crossover frequency and phase margin.

    Parameters:
    -----------
    omega_gc : float
        Desired gain crossover frequency [rad/s]
    PM_deg : float
        Desired phase margin [degrees]
    eta : float
        Derivative filter coefficient (typically 0.1)

    Returns:
    --------
    Kp, Ti, Td : PID parameters
    """
    PM_rad = np.deg2rad(PM_deg)

    # Plant frequency response at omega_gc
    g_num, g_den = plant_tf()
    G_gc = freq_response(g_num, g_den, omega_gc)
    G_mag = np.abs(G_gc)
    G_phase = np.angle(G_gc)

    print(f"\n=== Design for omega_gc = {omega_gc:.1f} rad/s, PM = {PM_deg}° ===")
    print(f"Plant at omega_gc: |G| = {20*np.log10(G_mag):.1f} dB, phase = {np.rad2deg(G_phase):.1f}°")

    # Required controller phase at omega_gc
    # L(jw) = C(jw) * G(jw)
    # Phase margin: PM = 180 + angle(L(jw_gc))
    # So: angle(C(jw_gc)) = PM - 180 - angle(G(jw_gc))
    required_C_phase = PM_rad - np.pi - G_phase
    print(f"Required controller phase: {np.rad2deg(required_C_phase):.1f}°")

    # PID controller phase contribution:
    # C(jw) = Kp * (1 + 1/(Ti*jw) + Td*jw/(eta*Td*jw + 1))
    #
    # Design approach:
    # 1. The derivative term with filter provides phase lead
    # 2. The integral term provides phase lag
    # 3. Balance these to achieve required phase at omega_gc

    def controller_phase(Ti, Td, omega, eta):
        s = 1j * omega
        C = 1 + 1/(Ti*s) + Td*s/(eta*Td*s + 1)
        return np.angle(C)

    # Optimization to find Ti, Td that give required phase
    from scipy.optimize import minimize

    def objective(params):
        Ti, Td = params
        if Ti <= 0.01 or Td <= 0.001:
            return 1e10
        phase = controller_phase(Ti, Td, omega_gc, eta)
        return (phase - required_C_phase)**2

    # Initial guess based on heuristics
    Ti_init = 1.0 / omega_gc  # Integral zero at omega_gc
    Td_init = 0.5 / omega_gc  # Derivative zero at 2*omega_gc

    result = minimize(objective, [Ti_init, Td_init],
                     bounds=[(0.01, 100), (0.001, 10)],
                     method='L-BFGS-B')
    Ti, Td = result.x

    # Calculate Kp for |L(jw_gc)| = 1
    c_num, c_den = pid_controller(1.0, Ti, Td, eta)  # Kp=1
    C_gc = freq_response(c_num, c_den, omega_gc)
    L_gc_unit = C_gc * G_gc
    Kp = 1.0 / np.abs(L_gc_unit)

    print(f"\nDesigned PID parameters:")
    print(f"  Kp = {Kp:.6f}")
    print(f"  Ti = {Ti:.4f} s")
    print(f"  Td = {Td:.4f} s")
    print(f"  eta = {eta}")

    # Verify
    l_num, l_den = open_loop_tf(Kp, Ti, Td, eta)
    L_gc = freq_response(l_num, l_den, omega_gc)
    actual_PM = 180 + np.rad2deg(np.angle(L_gc))
    print(f"\nVerification at omega_gc = {omega_gc:.1f} rad/s:")
    print(f"  |L(jw_gc)| = {np.abs(L_gc):.4f} ({20*np.log10(np.abs(L_gc)):.2f} dB)")
    print(f"  Phase margin = {actual_PM:.1f}°")

    return Kp, Ti, Td


def calculate_margins(Kp, Ti, Td, eta=0.1):
    """Calculate gain margin and phase margin"""
    l_num, l_den = open_loop_tf(Kp, Ti, Td, eta)

    omega = np.logspace(-1, 4, 10000)
    L = freq_response(l_num, l_den, omega)

    mag = np.abs(L)
    phase_deg = np.rad2deg(np.unwrap(np.angle(L)))

    # Gain crossover frequency (|L| = 1)
    idx_gc = np.argmin(np.abs(mag - 1))
    omega_gc = omega[idx_gc]
    PM = 180 + phase_deg[idx_gc]

    # Phase crossover frequency (phase = -180°)
    idx_pc = np.argmin(np.abs(phase_deg + 180))
    omega_pc = omega[idx_pc]
    GM_dB = -20 * np.log10(mag[idx_pc])

    return omega_gc, PM, omega_pc, GM_dB


def plot_bode(Kp, Ti, Td, eta, omega_gc_target, filename):
    """Generate Bode plot for open loop transfer function"""
    l_num, l_den = open_loop_tf(Kp, Ti, Td, eta)
    g_num, g_den = plant_tf()
    c_num, c_den = pid_controller(Kp, Ti, Td, eta)

    omega = np.logspace(-1, 4, 1000)

    L = freq_response(l_num, l_den, omega)
    G = freq_response(g_num, g_den, omega)
    C = freq_response(c_num, c_den, omega)

    # Calculate margins
    omega_gc, PM, omega_pc, GM_dB = calculate_margins(Kp, Ti, Td, eta)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Magnitude plot
    ax1.semilogx(omega, 20*np.log10(np.abs(L)), 'b-', linewidth=2, label='Open Loop L(s)')
    ax1.semilogx(omega, 20*np.log10(np.abs(G)), 'g--', linewidth=1.5, label='Plant G(s)')
    ax1.semilogx(omega, 20*np.log10(np.abs(C)), 'r-.', linewidth=1.5, label='Controller C(s)')
    ax1.axhline(y=0, color='k', linestyle=':', linewidth=0.5)
    ax1.axvline(x=omega_gc, color='b', linestyle=':', linewidth=1, alpha=0.7)
    ax1.axvline(x=omega_pc, color='m', linestyle=':', linewidth=1, alpha=0.7)
    ax1.set_ylabel('Magnitude [dB]', fontsize=12)
    ax1.set_title(f'Bode Plot: Loop Shaping Design\n'
                  f'$\\omega_{{gc}}$ = {omega_gc:.1f} rad/s, PM = {PM:.1f}°, GM = {GM_dB:.1f} dB',
                  fontsize=12)
    ax1.legend(loc='upper right')
    ax1.grid(True, which='both', linestyle='-', alpha=0.3)
    ax1.set_ylim([-80, 100])

    # Phase plot
    phase_L = np.rad2deg(np.unwrap(np.angle(L)))
    phase_G = np.rad2deg(np.unwrap(np.angle(G)))
    phase_C = np.rad2deg(np.unwrap(np.angle(C)))

    ax2.semilogx(omega, phase_L, 'b-', linewidth=2, label='Open Loop L(s)')
    ax2.semilogx(omega, phase_G, 'g--', linewidth=1.5, label='Plant G(s)')
    ax2.semilogx(omega, phase_C, 'r-.', linewidth=1.5, label='Controller C(s)')
    ax2.axhline(y=-180, color='k', linestyle=':', linewidth=0.5)
    ax2.axvline(x=omega_gc, color='b', linestyle=':', linewidth=1, alpha=0.7)

    # Mark phase margin
    idx_gc = np.argmin(np.abs(omega - omega_gc))
    ax2.annotate('', xy=(omega_gc, -180), xytext=(omega_gc, phase_L[idx_gc]),
                arrowprops=dict(arrowstyle='<->', color='blue', lw=1.5))
    ax2.text(omega_gc*1.2, (phase_L[idx_gc] - 180)/2 - 180, f'PM={PM:.0f}°',
            fontsize=10, color='blue')

    ax2.set_xlabel('Frequency [rad/s]', fontsize=12)
    ax2.set_ylabel('Phase [deg]', fontsize=12)
    ax2.legend(loc='upper right')
    ax2.grid(True, which='both', linestyle='-', alpha=0.3)
    ax2.set_ylim([-270, 90])

    plt.tight_layout()
    filepath = os.path.join(output_dir, filename)
    plt.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {filepath}")

    return omega_gc, PM, omega_pc, GM_dB


def plot_closed_loop_step(Kp, Ti, Td, eta, filename):
    """Generate step response of closed loop system"""
    l_num, l_den = open_loop_tf(Kp, Ti, Td, eta)

    # Closed loop: T(s) = L(s) / (1 + L(s))
    # T_num = L_num
    # T_den = L_den + L_num (after proper padding)

    # Pad arrays to same length
    max_len = max(len(l_num), len(l_den))
    l_num_pad = np.pad(l_num, (max_len - len(l_num), 0))
    l_den_pad = np.pad(l_den, (max_len - len(l_den), 0))

    t_num = l_num_pad
    t_den = l_den_pad + l_num_pad

    # Create transfer function and compute step response
    sys = signal.TransferFunction(t_num, t_den)
    t = np.linspace(0, 0.5, 1000)
    t_out, y = signal.step(sys, T=t)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t_out * 1000, y, 'b-', linewidth=2)
    ax.axhline(y=1, color='k', linestyle='--', linewidth=0.5)
    ax.axhline(y=1.05, color='r', linestyle=':', linewidth=0.5, label='±5%')
    ax.axhline(y=0.95, color='r', linestyle=':', linewidth=0.5)

    # Find settling time (5%)
    settled_idx = np.where(np.abs(y - 1) < 0.05)[0]
    if len(settled_idx) > 0:
        # Find first time it stays within 5%
        for i in range(len(settled_idx)):
            if np.all(np.abs(y[settled_idx[i]:] - 1) < 0.05):
                ts = t_out[settled_idx[i]] * 1000
                ax.axvline(x=ts, color='g', linestyle=':', linewidth=1)
                ax.text(ts + 5, 0.5, f'$t_s$ = {ts:.0f} ms', fontsize=10, color='g')
                break

    # Find overshoot
    overshoot = (np.max(y) - 1) * 100
    ax.text(t_out[np.argmax(y)] * 1000 + 5, np.max(y),
            f'Overshoot: {overshoot:.1f}%', fontsize=10, color='b')

    ax.set_xlabel('Time [ms]', fontsize=12)
    ax.set_ylabel('Angular Velocity Response', fontsize=12)
    ax.set_title('Closed-Loop Step Response', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 500])
    ax.legend()

    plt.tight_layout()
    filepath = os.path.join(output_dir, filename)
    plt.savefig(filepath, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {filepath}")


def main():
    print("=" * 60)
    print("PID Loop Shaping Design for StampFly Roll Axis")
    print("=" * 60)

    print(f"\nPlant parameters:")
    print(f"  I_xx = {I_xx:.2e} kg·m²")
    print(f"  tau_m = {tau_m} s (motor time constant)")
    print(f"  K_plant = 1/I_xx = {K_plant:.2e}")
    print(f"  Motor bandwidth = 1/tau_m = {1/tau_m:.0f} rad/s")

    # Design parameters
    eta = 0.1  # Derivative filter coefficient
    PM_target = 60  # degrees

    # Discussion of gain crossover frequency choice:
    # 1. Motor bandwidth is 50 rad/s (1/tau_m)
    # 2. Control loop should not exceed motor bandwidth
    # 3. Typical choice: omega_gc = 0.2 ~ 0.5 * motor_bandwidth
    # 4. For good disturbance rejection: higher omega_gc is better
    # 5. For robustness to unmodeled dynamics: lower omega_gc is better
    # 6. IMU runs at 400 Hz, so Nyquist is 1256 rad/s (not a constraint)

    # Choose omega_gc = 15 rad/s (about 30% of motor bandwidth)
    omega_gc_target = 15.0  # rad/s

    print(f"\n" + "=" * 60)
    print("Design Specifications:")
    print(f"  Target phase margin: {PM_target}°")
    print(f"  Target gain crossover frequency: {omega_gc_target} rad/s")
    print(f"  Derivative filter coefficient: eta = {eta}")
    print("=" * 60)

    # Design PID
    Kp, Ti, Td = design_pid_loop_shaping(omega_gc_target, PM_target, eta)

    # Generate Bode plot
    omega_gc, PM, omega_pc, GM_dB = plot_bode(Kp, Ti, Td, eta, omega_gc_target,
                                               'bode_loop_shaping.png')

    # Generate step response
    plot_closed_loop_step(Kp, Ti, Td, eta, 'step_response.png')

    # Summary
    print(f"\n" + "=" * 60)
    print("DESIGN SUMMARY")
    print("=" * 60)
    print(f"\nPID Controller: C(s) = Kp * (1 + 1/(Ti*s) + Td*s/(eta*Td*s + 1))")
    print(f"\nParameters:")
    print(f"  Kp  = {Kp:.6f}")
    print(f"  Ti  = {Ti:.4f} s")
    print(f"  Td  = {Td:.4f} s")
    print(f"  eta = {eta}")
    print(f"\nPerformance:")
    print(f"  Gain crossover frequency: {omega_gc:.1f} rad/s ({omega_gc/(2*np.pi):.2f} Hz)")
    print(f"  Phase margin: {PM:.1f}°")
    print(f"  Gain margin: {GM_dB:.1f} dB")
    print(f"  Phase crossover frequency: {omega_pc:.1f} rad/s")

    # Also design for pitch and yaw for comparison
    print(f"\n" + "=" * 60)
    print("Parameters for other axes (same design method):")
    print("=" * 60)

    # Pitch axis
    I_yy = 13.3e-6
    K_plant_pitch = 1 / I_yy
    ratio_pitch = I_xx / I_yy
    Kp_pitch = Kp * ratio_pitch
    print(f"\nPitch axis (I_yy = {I_yy:.2e} kg·m²):")
    print(f"  Kp = {Kp_pitch:.6f} (= Kp_roll * I_xx/I_yy)")
    print(f"  Ti = {Ti:.4f} s (same)")
    print(f"  Td = {Td:.4f} s (same)")

    # Yaw axis
    I_zz = 20.4e-6
    K_plant_yaw = 1 / I_zz
    ratio_yaw = I_xx / I_zz
    Kp_yaw = Kp * ratio_yaw
    print(f"\nYaw axis (I_zz = {I_zz:.2e} kg·m²):")
    print(f"  Kp = {Kp_yaw:.6f} (= Kp_roll * I_xx/I_zz)")
    print(f"  Ti = {Ti:.4f} s (same)")
    print(f"  Td = {Td:.4f} s (same)")


if __name__ == "__main__":
    main()
