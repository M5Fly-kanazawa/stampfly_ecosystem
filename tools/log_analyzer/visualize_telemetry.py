#!/usr/bin/env python3
"""
visualize_telemetry.py - WiFi Telemetry CSV Visualization Tool

Visualizes all state variables from WiFi telemetry CSV logs.
These logs are saved in firmware/vehicle/logs/ directory.

Usage:
    python visualize_telemetry.py <csv_file> [options]
    python visualize_telemetry.py  # opens file dialog if no file specified

Options:
    --all           Show all panels (default)
    --attitude      Show attitude (roll, pitch, yaw)
    --position      Show position and velocity
    --sensors       Show raw sensor data (accel, gyro, mag)
    --control       Show control inputs (throttle, roll, pitch, yaw)
    --tof           Show ToF sensors and altitude
    --save FILE     Save figure to file
    --no-show       Don't display window (use with --save)

Examples:
    python visualize_telemetry.py stampfly_log_20260113T070745.csv
    python visualize_telemetry.py stampfly_log_*.csv --attitude --position
    python visualize_telemetry.py log.csv --all --save output.png
"""

import argparse
import sys
import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Default log directory
DEFAULT_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
    'firmware', 'vehicle', 'logs'
)


def load_telemetry_csv(filename):
    """Load WiFi telemetry CSV file"""
    df = pd.read_csv(filename)

    # Convert timestamp to seconds from start
    df['time_s'] = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0

    print(f"Loaded {len(df)} samples from {os.path.basename(filename)}")
    print(f"Duration: {df['time_s'].iloc[-1]:.1f}s")
    print(f"Sample rate: {len(df) / df['time_s'].iloc[-1]:.1f} Hz")

    return df


def find_latest_csv(log_dir=DEFAULT_LOG_DIR):
    """Find the most recent CSV file in log directory"""
    pattern = os.path.join(log_dir, '*.csv')
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def plot_attitude(ax_list, df, title_prefix=''):
    """Plot attitude angles"""
    ax_roll, ax_pitch, ax_yaw = ax_list
    time = df['time_s']

    ax_roll.plot(time, df['roll_deg'], 'b-', linewidth=0.8)
    ax_roll.set_ylabel('Roll [deg]')
    ax_roll.set_title(f'{title_prefix}Roll')
    ax_roll.grid(True, alpha=0.3)
    ax_roll.axhline(y=0, color='k', linewidth=0.5)

    ax_pitch.plot(time, df['pitch_deg'], 'g-', linewidth=0.8)
    ax_pitch.set_ylabel('Pitch [deg]')
    ax_pitch.set_title(f'{title_prefix}Pitch')
    ax_pitch.grid(True, alpha=0.3)
    ax_pitch.axhline(y=0, color='k', linewidth=0.5)

    ax_yaw.plot(time, df['yaw_deg'], 'r-', linewidth=0.8)
    ax_yaw.set_ylabel('Yaw [deg]')
    ax_yaw.set_title(f'{title_prefix}Yaw')
    ax_yaw.set_xlabel('Time [s]')
    ax_yaw.grid(True, alpha=0.3)


def plot_position(ax_list, df, title_prefix=''):
    """Plot position"""
    ax_x, ax_y, ax_z = ax_list
    time = df['time_s']

    ax_x.plot(time, df['pos_x'], 'b-', linewidth=0.8)
    ax_x.set_ylabel('X [m]')
    ax_x.set_title(f'{title_prefix}Position X (North)')
    ax_x.grid(True, alpha=0.3)

    ax_y.plot(time, df['pos_y'], 'g-', linewidth=0.8)
    ax_y.set_ylabel('Y [m]')
    ax_y.set_title(f'{title_prefix}Position Y (East)')
    ax_y.grid(True, alpha=0.3)

    ax_z.plot(time, df['pos_z'], 'r-', linewidth=0.8)
    ax_z.set_ylabel('Z [m]')
    ax_z.set_title(f'{title_prefix}Position Z (Down)')
    ax_z.set_xlabel('Time [s]')
    ax_z.grid(True, alpha=0.3)
    ax_z.invert_yaxis()  # Z down is positive in NED


def plot_velocity(ax_list, df, title_prefix=''):
    """Plot velocity"""
    ax_vx, ax_vy, ax_vz = ax_list
    time = df['time_s']

    ax_vx.plot(time, df['vel_x'], 'b-', linewidth=0.8)
    ax_vx.set_ylabel('Vx [m/s]')
    ax_vx.set_title(f'{title_prefix}Velocity X')
    ax_vx.grid(True, alpha=0.3)
    ax_vx.axhline(y=0, color='k', linewidth=0.5)

    ax_vy.plot(time, df['vel_y'], 'g-', linewidth=0.8)
    ax_vy.set_ylabel('Vy [m/s]')
    ax_vy.set_title(f'{title_prefix}Velocity Y')
    ax_vy.grid(True, alpha=0.3)
    ax_vy.axhline(y=0, color='k', linewidth=0.5)

    ax_vz.plot(time, df['vel_z'], 'r-', linewidth=0.8)
    ax_vz.set_ylabel('Vz [m/s]')
    ax_vz.set_title(f'{title_prefix}Velocity Z')
    ax_vz.set_xlabel('Time [s]')
    ax_vz.grid(True, alpha=0.3)
    ax_vz.axhline(y=0, color='k', linewidth=0.5)


def plot_accel(ax, df, title_prefix=''):
    """Plot accelerometer data"""
    time = df['time_s']
    ax.plot(time, df['accel_x'], 'r-', linewidth=0.5, label='X', alpha=0.8)
    ax.plot(time, df['accel_y'], 'g-', linewidth=0.5, label='Y', alpha=0.8)
    ax.plot(time, df['accel_z'], 'b-', linewidth=0.5, label='Z', alpha=0.8)
    ax.set_ylabel('Accel [m/s²]')
    ax.set_title(f'{title_prefix}Accelerometer')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)


def plot_gyro(ax, df, title_prefix=''):
    """Plot gyroscope data"""
    time = df['time_s']
    ax.plot(time, df['gyro_x'], 'r-', linewidth=0.5, label='X', alpha=0.8)
    ax.plot(time, df['gyro_y'], 'g-', linewidth=0.5, label='Y', alpha=0.8)
    ax.plot(time, df['gyro_z'], 'b-', linewidth=0.5, label='Z', alpha=0.8)
    ax.set_ylabel('Gyro [rad/s]')
    ax.set_title(f'{title_prefix}Gyroscope')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)


def plot_mag(ax, df, title_prefix=''):
    """Plot magnetometer data"""
    time = df['time_s']
    ax.plot(time, df['mag_x'], 'r-', linewidth=0.5, label='X', alpha=0.8)
    ax.plot(time, df['mag_y'], 'g-', linewidth=0.5, label='Y', alpha=0.8)
    ax.plot(time, df['mag_z'], 'b-', linewidth=0.5, label='Z', alpha=0.8)
    ax.set_ylabel('Mag [uT]')
    ax.set_title(f'{title_prefix}Magnetometer')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)


def plot_control(ax_list, df, title_prefix=''):
    """Plot control inputs"""
    ax_throttle, ax_ctrl = ax_list
    time = df['time_s']

    ax_throttle.plot(time, df['throttle'], 'k-', linewidth=0.8)
    ax_throttle.set_ylabel('Throttle')
    ax_throttle.set_title(f'{title_prefix}Throttle')
    ax_throttle.set_ylim(-0.1, 1.1)
    ax_throttle.grid(True, alpha=0.3)

    ax_ctrl.plot(time, df['ctrl_roll'], 'r-', linewidth=0.5, label='Roll', alpha=0.8)
    ax_ctrl.plot(time, df['ctrl_pitch'], 'g-', linewidth=0.5, label='Pitch', alpha=0.8)
    ax_ctrl.plot(time, df['ctrl_yaw'], 'b-', linewidth=0.5, label='Yaw', alpha=0.8)
    ax_ctrl.set_ylabel('Control [-1, 1]')
    ax_ctrl.set_title(f'{title_prefix}Control Inputs')
    ax_ctrl.set_xlabel('Time [s]')
    ax_ctrl.legend(loc='upper right', fontsize=8)
    ax_ctrl.grid(True, alpha=0.3)
    ax_ctrl.axhline(y=0, color='k', linewidth=0.5)


def plot_tof(ax_list, df, title_prefix=''):
    """Plot ToF sensors"""
    ax_bottom, ax_front = ax_list
    time = df['time_s']

    ax_bottom.plot(time, df['tof_bottom'], 'b-', linewidth=0.8)
    ax_bottom.set_ylabel('Distance [m]')
    ax_bottom.set_title(f'{title_prefix}ToF Bottom (Altitude)')
    ax_bottom.grid(True, alpha=0.3)
    ax_bottom.set_ylim(bottom=0)

    ax_front.plot(time, df['tof_front'], 'r-', linewidth=0.8)
    ax_front.set_ylabel('Distance [m]')
    ax_front.set_title(f'{title_prefix}ToF Front')
    ax_front.set_xlabel('Time [s]')
    ax_front.grid(True, alpha=0.3)
    ax_front.set_ylim(bottom=0)


def plot_voltage(ax, df, title_prefix=''):
    """Plot battery voltage"""
    time = df['time_s']
    ax.plot(time, df['voltage'], 'orange', linewidth=0.8)
    ax.set_ylabel('Voltage [V]')
    ax.set_title(f'{title_prefix}Battery Voltage')
    ax.set_xlabel('Time [s]')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=3.3, color='r', linewidth=0.5, linestyle='--', label='Low (3.3V)')
    ax.axhline(y=4.2, color='g', linewidth=0.5, linestyle='--', label='Full (4.2V)')
    ax.legend(loc='upper right', fontsize=8)


def plot_trajectory_xy(ax, df, title_prefix=''):
    """Plot XY trajectory"""
    ax.plot(df['pos_y'], df['pos_x'], 'b-', linewidth=0.8)
    ax.plot(df['pos_y'].iloc[0], df['pos_x'].iloc[0], 'go', markersize=8, label='Start')
    ax.plot(df['pos_y'].iloc[-1], df['pos_x'].iloc[-1], 'ro', markersize=8, label='End')
    ax.set_xlabel('Y (East) [m]')
    ax.set_ylabel('X (North) [m]')
    ax.set_title(f'{title_prefix}XY Trajectory')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')


def plot_state(ax, df, title_prefix=''):
    """Plot flight state"""
    time = df['time_s']
    ax.plot(time, df['state'], 'k-', linewidth=0.8, drawstyle='steps-post')
    ax.set_ylabel('State')
    ax.set_title(f'{title_prefix}Flight State')
    ax.set_xlabel('Time [s]')
    ax.grid(True, alpha=0.3)
    ax.set_yticks([0, 1, 2, 3, 4, 5, 6])
    ax.set_yticklabels(['INIT', 'CALIB', 'IDLE', 'ARMED', 'FLYING', 'LAND', 'ERR'], fontsize=7)


def visualize_all(df, filename, save_path=None, show=True):
    """Create comprehensive visualization with all panels"""
    fig = plt.figure(figsize=(20, 16))
    fig.suptitle(f'StampFly Telemetry: {os.path.basename(filename)}', fontsize=14)

    gs = GridSpec(5, 4, figure=fig, hspace=0.35, wspace=0.3)

    # Row 1: Attitude + State
    ax_roll = fig.add_subplot(gs[0, 0])
    ax_pitch = fig.add_subplot(gs[0, 1])
    ax_yaw = fig.add_subplot(gs[0, 2])
    ax_state = fig.add_subplot(gs[0, 3])
    plot_attitude([ax_roll, ax_pitch, ax_yaw], df)
    plot_state(ax_state, df)

    # Row 2: Position + Trajectory
    ax_pos_x = fig.add_subplot(gs[1, 0])
    ax_pos_y = fig.add_subplot(gs[1, 1])
    ax_pos_z = fig.add_subplot(gs[1, 2])
    ax_traj = fig.add_subplot(gs[1, 3])
    plot_position([ax_pos_x, ax_pos_y, ax_pos_z], df)
    plot_trajectory_xy(ax_traj, df)

    # Row 3: Velocity + Voltage
    ax_vel_x = fig.add_subplot(gs[2, 0])
    ax_vel_y = fig.add_subplot(gs[2, 1])
    ax_vel_z = fig.add_subplot(gs[2, 2])
    ax_voltage = fig.add_subplot(gs[2, 3])
    plot_velocity([ax_vel_x, ax_vel_y, ax_vel_z], df)
    plot_voltage(ax_voltage, df)

    # Row 4: Sensors (Accel, Gyro, Mag, ToF Bottom)
    ax_accel = fig.add_subplot(gs[3, 0])
    ax_gyro = fig.add_subplot(gs[3, 1])
    ax_mag = fig.add_subplot(gs[3, 2])
    ax_tof_bottom = fig.add_subplot(gs[3, 3])
    plot_accel(ax_accel, df)
    plot_gyro(ax_gyro, df)
    plot_mag(ax_mag, df)
    ax_tof_bottom.plot(df['time_s'], df['tof_bottom'], 'b-', linewidth=0.8)
    ax_tof_bottom.set_ylabel('Distance [m]')
    ax_tof_bottom.set_title('ToF Bottom (Altitude)')
    ax_tof_bottom.grid(True, alpha=0.3)
    ax_tof_bottom.set_ylim(bottom=0)

    # Row 5: Control + ToF Front
    ax_throttle = fig.add_subplot(gs[4, 0])
    ax_ctrl = fig.add_subplot(gs[4, 1])
    ax_tof_front = fig.add_subplot(gs[4, 2])
    ax_empty = fig.add_subplot(gs[4, 3])
    plot_control([ax_throttle, ax_ctrl], df)
    ax_tof_front.plot(df['time_s'], df['tof_front'], 'r-', linewidth=0.8)
    ax_tof_front.set_ylabel('Distance [m]')
    ax_tof_front.set_title('ToF Front')
    ax_tof_front.set_xlabel('Time [s]')
    ax_tof_front.grid(True, alpha=0.3)
    ax_tof_front.set_ylim(bottom=0)
    ax_empty.axis('off')

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved figure to {save_path}")

    if show:
        plt.show()
    else:
        plt.close()


def visualize_attitude_only(df, filename, save_path=None, show=True):
    """Visualize attitude only"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f'Attitude: {os.path.basename(filename)}', fontsize=12)

    plot_attitude(axes, df)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    else:
        plt.close()


def visualize_position_only(df, filename, save_path=None, show=True):
    """Visualize position and velocity"""
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f'Position & Velocity: {os.path.basename(filename)}', fontsize=12)

    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)

    # Position
    ax_pos_x = fig.add_subplot(gs[0, 0])
    ax_pos_y = fig.add_subplot(gs[0, 1])
    ax_pos_z = fig.add_subplot(gs[0, 2])
    plot_position([ax_pos_x, ax_pos_y, ax_pos_z], df)

    # Velocity
    ax_vel_x = fig.add_subplot(gs[1, 0])
    ax_vel_y = fig.add_subplot(gs[1, 1])
    ax_vel_z = fig.add_subplot(gs[1, 2])
    plot_velocity([ax_vel_x, ax_vel_y, ax_vel_z], df)

    # Trajectory + ToF
    ax_traj = fig.add_subplot(gs[2, 0:2])
    ax_tof = fig.add_subplot(gs[2, 2])
    plot_trajectory_xy(ax_traj, df)
    ax_tof.plot(df['time_s'], df['tof_bottom'], 'b-', linewidth=0.8, label='Bottom')
    ax_tof.plot(df['time_s'], -df['pos_z'], 'r--', linewidth=0.8, label='-Pos_Z')
    ax_tof.set_ylabel('Altitude [m]')
    ax_tof.set_title('Altitude Comparison')
    ax_tof.set_xlabel('Time [s]')
    ax_tof.legend(fontsize=8)
    ax_tof.grid(True, alpha=0.3)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    else:
        plt.close()


def visualize_sensors_only(df, filename, save_path=None, show=True):
    """Visualize raw sensor data"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f'Raw Sensors: {os.path.basename(filename)}', fontsize=12)

    plot_accel(axes[0], df)
    plot_gyro(axes[1], df)
    plot_mag(axes[2], df)
    axes[2].set_xlabel('Time [s]')

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    else:
        plt.close()


def visualize_control_only(df, filename, save_path=None, show=True):
    """Visualize control inputs"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    fig.suptitle(f'Control Inputs: {os.path.basename(filename)}', fontsize=12)

    plot_control(axes, df)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    else:
        plt.close()


def visualize_tof_only(df, filename, save_path=None, show=True):
    """Visualize ToF sensors"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    fig.suptitle(f'ToF Sensors: {os.path.basename(filename)}', fontsize=12)

    plot_tof(axes, df)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    else:
        plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize WiFi telemetry CSV logs',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('file', nargs='?', help='CSV file to visualize')
    parser.add_argument('--all', action='store_true', help='Show all panels (default)')
    parser.add_argument('--attitude', action='store_true', help='Show attitude only')
    parser.add_argument('--position', action='store_true', help='Show position/velocity')
    parser.add_argument('--sensors', action='store_true', help='Show raw sensors')
    parser.add_argument('--control', action='store_true', help='Show control inputs')
    parser.add_argument('--tof', action='store_true', help='Show ToF sensors')
    parser.add_argument('--save', metavar='FILE', help='Save figure to file')
    parser.add_argument('--no-show', action='store_true', help="Don't display window")

    args = parser.parse_args()

    # Determine file to visualize
    filename = args.file
    if not filename:
        # Try to find latest CSV in default log directory
        filename = find_latest_csv()
        if filename:
            print(f"Using latest CSV: {filename}")
        else:
            # Try file dialog
            try:
                import tkinter as tk
                from tkinter import filedialog
                root = tk.Tk()
                root.withdraw()
                filename = filedialog.askopenfilename(
                    title='Select telemetry CSV file',
                    initialdir=DEFAULT_LOG_DIR,
                    filetypes=[('CSV files', '*.csv'), ('All files', '*.*')]
                )
                if not filename:
                    print("No file selected")
                    sys.exit(1)
            except ImportError:
                print("No file specified and no CSV files found in default directory")
                print(f"Usage: python {sys.argv[0]} <csv_file>")
                sys.exit(1)

    if not os.path.exists(filename):
        print(f"Error: File not found: {filename}")
        sys.exit(1)

    # Load data
    df = load_telemetry_csv(filename)

    # Determine what to show
    show = not args.no_show
    save_path = args.save

    # If no specific option, show all
    if not (args.attitude or args.position or args.sensors or args.control or args.tof):
        args.all = True

    if args.all:
        visualize_all(df, filename, save_path, show)
    else:
        if args.attitude:
            visualize_attitude_only(df, filename, save_path, show)
        if args.position:
            visualize_position_only(df, filename, save_path, show)
        if args.sensors:
            visualize_sensors_only(df, filename, save_path, show)
        if args.control:
            visualize_control_only(df, filename, save_path, show)
        if args.tof:
            visualize_tof_only(df, filename, save_path, show)


if __name__ == '__main__':
    main()
