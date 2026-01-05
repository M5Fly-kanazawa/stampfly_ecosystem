#!/usr/bin/env python3
"""
visualize_eskf.py - ESKF Comprehensive Visualization Tool

Unified visualization tool for ESKF state variables and raw sensor data.
Supports binary log files (.bin) and CSV files from eskf_replay.

Usage:
    python visualize_eskf.py data.bin [options]
    python visualize_eskf.py data.csv [options]

Options:
    --all           Show all panels (default if no option specified)
    --sensors       Show raw sensor data (accel, gyro, mag, baro, tof, flow)
    --attitude      Show attitude (roll, pitch, yaw)
    --position      Show position and velocity
    --biases        Show estimated biases (gyro, accel)
    --compare       Compare PC vs Device (requires V2 binary or CSV with dev_* columns)
    --save FILE     Save figure to file instead of displaying
    --no-show       Don't display (use with --save)

Examples:
    python visualize_eskf.py flow01.bin --all
    python visualize_eskf.py flow01.bin --sensors --attitude
    python visualize_eskf.py result.csv --compare --position
"""

import argparse
import sys
import os
import struct
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# V2 Packet constants (128 bytes, header 0xAA 0x56)
PACKET_SIZE = 128
PACKET_HEADER = bytes([0xAA, 0x56])

# Packet structure format (little-endian)
# 2B header + 4B timestamp + 24B IMU + 12B mag + 8B baro + 8B tof + 5B flow +
# 12B pos + 12B vel + 12B att + 12B bias + 1B status + 15B reserved + 1B checksum = 128B
PACKET_FORMAT = '<2sI6f3f2f2f2hB3f3f3f3fB15sB'


def parse_packet(packet):
    """Parse V2 packet (128 bytes)"""
    if len(packet) < PACKET_SIZE or packet[0:2] != PACKET_HEADER:
        return None

    try:
        unpacked = struct.unpack(PACKET_FORMAT, packet)

        # Extract fields
        timestamp_ms = unpacked[1]

        # IMU data
        accel = (unpacked[2], unpacked[3], unpacked[4])
        gyro = (unpacked[5], unpacked[6], unpacked[7])

        # Mag data
        mag = (unpacked[8], unpacked[9], unpacked[10])

        # Baro data
        pressure = unpacked[11]
        baro_alt = unpacked[12]

        # ToF data
        tof_bottom = unpacked[13]
        tof_front = unpacked[14]

        # Flow data
        flow_dx = unpacked[15]
        flow_dy = unpacked[16]
        flow_squal = unpacked[17]

        # ESKF estimates
        pos = (unpacked[18], unpacked[19], unpacked[20])
        vel = (unpacked[21], unpacked[22], unpacked[23])
        att = (unpacked[24], unpacked[25], unpacked[26])  # roll, pitch, yaw in radians
        bias = (unpacked[27], unpacked[28], unpacked[29])  # gyro_bias_z, accel_bias_x, accel_bias_y
        eskf_status = unpacked[30]

        return {
            'timestamp_ms': timestamp_ms,
            'accel': accel,
            'gyro': gyro,
            'mag': mag,
            'pressure': pressure,
            'baro_alt': baro_alt,
            'tof_bottom': tof_bottom,
            'tof_front': tof_front,
            'flow_dx': flow_dx,
            'flow_dy': flow_dy,
            'flow_squal': flow_squal,
            'pos': pos,
            'vel': vel,
            'att': att,  # radians
            'bias': bias,
            'eskf_status': eskf_status,
        }
    except Exception as e:
        return None


def load_binary_file(filename):
    """Load binary log file and return DataFrame"""
    with open(filename, 'rb') as f:
        data = f.read()

    print(f"Loading V2 format ({PACKET_SIZE} bytes per packet)")

    records = []
    offset = 0

    # Find first valid packet
    while offset + PACKET_SIZE <= len(data):
        if data[offset:offset+2] == PACKET_HEADER:
            break
        offset += 1

    # Parse packets
    while offset + PACKET_SIZE <= len(data):
        packet = data[offset:offset + PACKET_SIZE]
        if packet[0:2] != PACKET_HEADER:
            offset += 1
            continue

        # Verify checksum
        checksum = 0
        for i in range(2, 127):
            checksum ^= packet[i]
        if checksum != packet[127]:
            offset += 1
            continue

        parsed = parse_packet(packet)
        if parsed:
            records.append({
                'timestamp_ms': parsed['timestamp_ms'],
                'raw_accel_x': parsed['accel'][0],
                'raw_accel_y': parsed['accel'][1],
                'raw_accel_z': parsed['accel'][2],
                'raw_gyro_x': parsed['gyro'][0],
                'raw_gyro_y': parsed['gyro'][1],
                'raw_gyro_z': parsed['gyro'][2],
                'raw_mag_x': parsed['mag'][0],
                'raw_mag_y': parsed['mag'][1],
                'raw_mag_z': parsed['mag'][2],
                'raw_pressure': parsed['pressure'],
                'raw_baro_alt': parsed['baro_alt'],
                'raw_tof': parsed['tof_bottom'],
                'raw_tof_front': parsed['tof_front'],
                'raw_flow_dx': parsed['flow_dx'],
                'raw_flow_dy': parsed['flow_dy'],
                'raw_flow_squal': parsed['flow_squal'],
                # Device ESKF estimates
                'dev_pos_x': parsed['pos'][0],
                'dev_pos_y': parsed['pos'][1],
                'dev_pos_z': parsed['pos'][2],
                'dev_vel_x': parsed['vel'][0],
                'dev_vel_y': parsed['vel'][1],
                'dev_vel_z': parsed['vel'][2],
                'dev_roll_deg': np.degrees(parsed['att'][0]),
                'dev_pitch_deg': np.degrees(parsed['att'][1]),
                'dev_yaw_deg': np.degrees(parsed['att'][2]),
                'dev_gyro_bias_z': parsed['bias'][0],
                'dev_accel_bias_x': parsed['bias'][1],
                'dev_accel_bias_y': parsed['bias'][2],
                'eskf_status': parsed['eskf_status'],
            })

        offset += PACKET_SIZE

    df = pd.DataFrame(records)
    print(f"Loaded {len(df)} packets")
    return df


def load_csv_file(filename):
    """Load CSV file from eskf_replay"""
    df = pd.read_csv(filename)
    print(f"Loaded {len(df)} rows from CSV")

    # Check if it has device columns
    has_device = 'dev_pos_x' in df.columns
    return df, has_device


def get_time_axis(df):
    """Get time axis in seconds"""
    if 'timestamp_ms' in df.columns:
        return (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    else:
        return np.arange(len(df)) * 0.02  # Assume 50Hz


def plot_sensors(df, time):
    """Plot raw sensor data"""
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('Raw Sensor Data', fontsize=14, fontweight='bold')

    # Accelerometer
    ax = axes[0, 0]
    ax.plot(time, df['raw_accel_x'], 'r-', alpha=0.7, label='X')
    ax.plot(time, df['raw_accel_y'], 'g-', alpha=0.7, label='Y')
    ax.plot(time, df['raw_accel_z'], 'b-', alpha=0.7, label='Z')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Acceleration [m/s²]')
    ax.set_title('Accelerometer')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Gyroscope
    ax = axes[0, 1]
    ax.plot(time, np.degrees(df['raw_gyro_x']), 'r-', alpha=0.7, label='X')
    ax.plot(time, np.degrees(df['raw_gyro_y']), 'g-', alpha=0.7, label='Y')
    ax.plot(time, np.degrees(df['raw_gyro_z']), 'b-', alpha=0.7, label='Z')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Angular Rate [deg/s]')
    ax.set_title('Gyroscope')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Magnetometer
    ax = axes[1, 0]
    ax.plot(time, df['raw_mag_x'], 'r-', alpha=0.7, label='X')
    ax.plot(time, df['raw_mag_y'], 'g-', alpha=0.7, label='Y')
    ax.plot(time, df['raw_mag_z'], 'b-', alpha=0.7, label='Z')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Magnetic Field [uT]')
    ax.set_title('Magnetometer')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Barometer & ToF
    ax = axes[1, 1]
    ax2 = ax.twinx()
    l1 = ax.plot(time, df['raw_baro_alt'], 'b-', alpha=0.7, label='Baro Alt')
    l2 = ax2.plot(time, df['raw_tof'] * 100, 'r-', alpha=0.7, label='ToF')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Baro Altitude [m]', color='b')
    ax2.set_ylabel('ToF Distance [cm]', color='r')
    ax.set_title('Barometer & ToF')
    lines = l1 + l2
    ax.legend(lines, [l.get_label() for l in lines])
    ax.grid(True, alpha=0.3)

    # Optical Flow
    ax = axes[2, 0]
    ax.plot(time, df['raw_flow_dx'], 'r-', alpha=0.7, label='dx')
    ax.plot(time, df['raw_flow_dy'], 'b-', alpha=0.7, label='dy')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Flow [counts]')
    ax.set_title('Optical Flow')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Flow Quality
    ax = axes[2, 1]
    ax.plot(time, df['raw_flow_squal'], 'g-', alpha=0.7)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Surface Quality')
    ax.set_title('Flow Surface Quality')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_attitude(df, time, has_pc=True, has_device=False):
    """Plot attitude angles"""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    fig.suptitle('Attitude Estimation', fontsize=14, fontweight='bold')

    labels = ['Roll', 'Pitch', 'Yaw']
    pc_cols = ['roll_deg', 'pitch_deg', 'yaw_deg']
    dev_cols = ['dev_roll_deg', 'dev_pitch_deg', 'dev_yaw_deg']

    for i, (ax, label, pc_col, dev_col) in enumerate(zip(axes, labels, pc_cols, dev_cols)):
        if has_pc and pc_col in df.columns:
            ax.plot(time, df[pc_col], 'b-', alpha=0.7, label='PC ESKF')
        if has_device and dev_col in df.columns:
            ax.plot(time, df[dev_col], 'r--', alpha=0.7, label='Device')
        ax.axhline(0, color='k', linestyle=':', alpha=0.3)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(f'{label} [deg]')
        ax.set_title(label)
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_position(df, time, has_pc=True, has_device=False):
    """Plot position and velocity"""
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle('Position and Velocity Estimation', fontsize=14, fontweight='bold')

    # Position
    pos_labels = ['Position X', 'Position Y', 'Position Z']
    pc_pos_cols = ['pos_x', 'pos_y', 'pos_z']
    dev_pos_cols = ['dev_pos_x', 'dev_pos_y', 'dev_pos_z']

    for i, (label, pc_col, dev_col) in enumerate(zip(pos_labels, pc_pos_cols, dev_pos_cols)):
        ax = axes[0, i]
        if has_pc and pc_col in df.columns:
            ax.plot(time, df[pc_col] * 100, 'b-', alpha=0.7, label='PC ESKF')
        if has_device and dev_col in df.columns:
            ax.plot(time, df[dev_col] * 100, 'r--', alpha=0.7, label='Device')
        ax.axhline(0, color='k', linestyle=':', alpha=0.3)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(f'{label} [cm]')
        ax.set_title(label)
        ax.legend()
        ax.grid(True, alpha=0.3)

    # Velocity
    vel_labels = ['Velocity X', 'Velocity Y', 'Velocity Z']
    pc_vel_cols = ['vel_x', 'vel_y', 'vel_z']
    dev_vel_cols = ['dev_vel_x', 'dev_vel_y', 'dev_vel_z']

    for i, (label, pc_col, dev_col) in enumerate(zip(vel_labels, pc_vel_cols, dev_vel_cols)):
        ax = axes[1, i]
        if has_pc and pc_col in df.columns:
            ax.plot(time, df[pc_col], 'b-', alpha=0.7, label='PC ESKF')
        if has_device and dev_col in df.columns:
            ax.plot(time, df[dev_col], 'r--', alpha=0.7, label='Device')
        ax.axhline(0, color='k', linestyle=':', alpha=0.3)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(f'{label} [m/s]')
        ax.set_title(label)
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_biases(df, time, has_pc=True, has_device=False):
    """Plot estimated biases"""
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle('Bias Estimation', fontsize=14, fontweight='bold')

    # Gyro biases (PC ESKF has x, y, z; Device only has z)
    gyro_labels = ['Gyro Bias X', 'Gyro Bias Y', 'Gyro Bias Z']
    pc_gyro_cols = ['gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z']

    for i, (label, pc_col) in enumerate(zip(gyro_labels, pc_gyro_cols)):
        ax = axes[0, i]
        if has_pc and pc_col in df.columns:
            ax.plot(time, np.degrees(df[pc_col]), 'b-', alpha=0.7, label='PC ESKF')
        if has_device and i == 2 and 'dev_gyro_bias_z' in df.columns:
            ax.plot(time, np.degrees(df['dev_gyro_bias_z']), 'r--', alpha=0.7, label='Device')
        ax.axhline(0, color='k', linestyle=':', alpha=0.3)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(f'{label} [deg/s]')
        ax.set_title(label)
        ax.legend()
        ax.grid(True, alpha=0.3)

    # Accel biases (PC ESKF has x, y, z; Device has x, y)
    accel_labels = ['Accel Bias X', 'Accel Bias Y', 'Accel Bias Z']
    pc_accel_cols = ['accel_bias_x', 'accel_bias_y', 'accel_bias_z']
    dev_accel_cols = ['dev_accel_bias_x', 'dev_accel_bias_y', None]

    for i, (label, pc_col, dev_col) in enumerate(zip(accel_labels, pc_accel_cols, dev_accel_cols)):
        ax = axes[1, i]
        if has_pc and pc_col in df.columns:
            ax.plot(time, df[pc_col], 'b-', alpha=0.7, label='PC ESKF')
        if has_device and dev_col and dev_col in df.columns:
            ax.plot(time, df[dev_col], 'r--', alpha=0.7, label='Device')
        ax.axhline(0, color='k', linestyle=':', alpha=0.3)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(f'{label} [m/s²]')
        ax.set_title(label)
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_xy_trajectory(df, has_pc=True, has_device=False):
    """Plot XY trajectory"""
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    fig.suptitle('XY Trajectory', fontsize=14, fontweight='bold')

    if has_pc and 'pos_x' in df.columns:
        ax.plot(df['pos_x'] * 100, df['pos_y'] * 100, 'b-', alpha=0.7, label='PC ESKF', linewidth=1.5)
        ax.plot(df['pos_x'].iloc[-1] * 100, df['pos_y'].iloc[-1] * 100, 'b^', markersize=10)

    if has_device and 'dev_pos_x' in df.columns:
        ax.plot(df['dev_pos_x'] * 100, df['dev_pos_y'] * 100, 'r--', alpha=0.7, label='Device', linewidth=1.5)
        ax.plot(df['dev_pos_x'].iloc[-1] * 100, df['dev_pos_y'].iloc[-1] * 100, 'r^', markersize=10)

    ax.plot(0, 0, 'k+', markersize=15, mew=3, label='Origin')
    ax.set_xlabel('Position X [cm]')
    ax.set_ylabel('Position Y [cm]')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    plt.tight_layout()
    return fig


def plot_all(df, time, has_pc=True, has_device=False):
    """Plot all data in one comprehensive figure"""
    fig = plt.figure(figsize=(20, 16))
    fig.suptitle('ESKF Comprehensive View', fontsize=16, fontweight='bold')

    # Create grid
    gs = fig.add_gridspec(4, 4, hspace=0.3, wspace=0.3)

    # Row 1: Raw IMU
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(time, df['raw_accel_x'], 'r-', alpha=0.7, label='X')
    ax.plot(time, df['raw_accel_y'], 'g-', alpha=0.7, label='Y')
    ax.plot(time, df['raw_accel_z'], 'b-', alpha=0.7, label='Z')
    ax.set_ylabel('Accel [m/s²]')
    ax.set_title('Accelerometer')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(gs[0, 1])
    ax.plot(time, np.degrees(df['raw_gyro_x']), 'r-', alpha=0.7, label='X')
    ax.plot(time, np.degrees(df['raw_gyro_y']), 'g-', alpha=0.7, label='Y')
    ax.plot(time, np.degrees(df['raw_gyro_z']), 'b-', alpha=0.7, label='Z')
    ax.set_ylabel('Gyro [deg/s]')
    ax.set_title('Gyroscope')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(gs[0, 2])
    ax.plot(time, df['raw_mag_x'], 'r-', alpha=0.7, label='X')
    ax.plot(time, df['raw_mag_y'], 'g-', alpha=0.7, label='Y')
    ax.plot(time, df['raw_mag_z'], 'b-', alpha=0.7, label='Z')
    ax.set_ylabel('Mag [uT]')
    ax.set_title('Magnetometer')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(gs[0, 3])
    ax.plot(time, df['raw_tof'] * 100, 'b-', alpha=0.7)
    ax.set_ylabel('ToF [cm]')
    ax.set_title('ToF Distance')
    ax.grid(True, alpha=0.3)

    # Row 2: Flow & Attitude
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(time, df['raw_flow_dx'], 'r-', alpha=0.7, label='dx')
    ax.plot(time, df['raw_flow_dy'], 'b-', alpha=0.7, label='dy')
    ax.set_ylabel('Flow [counts]')
    ax.set_title('Optical Flow')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    for i, (col, dev_col, title) in enumerate([
        ('roll_deg', 'dev_roll_deg', 'Roll'),
        ('pitch_deg', 'dev_pitch_deg', 'Pitch'),
        ('yaw_deg', 'dev_yaw_deg', 'Yaw')
    ]):
        ax = fig.add_subplot(gs[1, i+1])
        if has_pc and col in df.columns:
            ax.plot(time, df[col], 'b-', alpha=0.7, label='PC')
        if has_device and dev_col in df.columns:
            ax.plot(time, df[dev_col], 'r--', alpha=0.7, label='Dev')
        ax.axhline(0, color='k', linestyle=':', alpha=0.3)
        ax.set_ylabel(f'{title} [deg]')
        ax.set_title(title)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # Row 3: Position
    for i, (col, dev_col, title) in enumerate([
        ('pos_x', 'dev_pos_x', 'Pos X'),
        ('pos_y', 'dev_pos_y', 'Pos Y'),
        ('pos_z', 'dev_pos_z', 'Pos Z')
    ]):
        ax = fig.add_subplot(gs[2, i])
        if has_pc and col in df.columns:
            ax.plot(time, df[col] * 100, 'b-', alpha=0.7, label='PC')
        if has_device and dev_col in df.columns:
            ax.plot(time, df[dev_col] * 100, 'r--', alpha=0.7, label='Dev')
        ax.axhline(0, color='k', linestyle=':', alpha=0.3)
        ax.set_ylabel(f'{title} [cm]')
        ax.set_title(title)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # XY Trajectory
    ax = fig.add_subplot(gs[2, 3])
    if has_pc and 'pos_x' in df.columns:
        ax.plot(df['pos_x'] * 100, df['pos_y'] * 100, 'b-', alpha=0.7, label='PC')
    if has_device and 'dev_pos_x' in df.columns:
        ax.plot(df['dev_pos_x'] * 100, df['dev_pos_y'] * 100, 'r--', alpha=0.7, label='Dev')
    ax.plot(0, 0, 'k+', markersize=10, mew=2)
    ax.set_xlabel('X [cm]')
    ax.set_ylabel('Y [cm]')
    ax.set_title('XY Trajectory')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    # Row 4: Velocity & Biases
    for i, (col, dev_col, title) in enumerate([
        ('vel_x', 'dev_vel_x', 'Vel X'),
        ('vel_y', 'dev_vel_y', 'Vel Y'),
        ('vel_z', 'dev_vel_z', 'Vel Z')
    ]):
        ax = fig.add_subplot(gs[3, i])
        if has_pc and col in df.columns:
            ax.plot(time, df[col], 'b-', alpha=0.7, label='PC')
        if has_device and dev_col in df.columns:
            ax.plot(time, df[dev_col], 'r--', alpha=0.7, label='Dev')
        ax.axhline(0, color='k', linestyle=':', alpha=0.3)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(f'{title} [m/s]')
        ax.set_title(title)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    # Gyro biases
    ax = fig.add_subplot(gs[3, 3])
    if has_pc and 'gyro_bias_x' in df.columns:
        ax.plot(time, np.degrees(df['gyro_bias_x']), 'r-', alpha=0.7, label='Gx')
        ax.plot(time, np.degrees(df['gyro_bias_y']), 'g-', alpha=0.7, label='Gy')
        ax.plot(time, np.degrees(df['gyro_bias_z']), 'b-', alpha=0.7, label='Gz')
    if has_device and 'dev_gyro_bias_z' in df.columns:
        ax.plot(time, np.degrees(df['dev_gyro_bias_z']), 'b--', alpha=0.7, label='Gz(Dev)')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Gyro Bias [deg/s]')
    ax.set_title('Gyro Bias')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    return fig


def main():
    parser = argparse.ArgumentParser(description='ESKF Comprehensive Visualization Tool')
    parser.add_argument('input', help='Input file (.bin or .csv)')
    parser.add_argument('--all', action='store_true', help='Show all panels')
    parser.add_argument('--sensors', action='store_true', help='Show raw sensor data')
    parser.add_argument('--attitude', action='store_true', help='Show attitude')
    parser.add_argument('--position', action='store_true', help='Show position/velocity')
    parser.add_argument('--biases', action='store_true', help='Show biases')
    parser.add_argument('--trajectory', action='store_true', help='Show XY trajectory')
    parser.add_argument('--compare', action='store_true', help='Compare PC vs Device')
    parser.add_argument('--pc', action='store_true', help='Show PC simulation results only (hide Device)')
    parser.add_argument('--save', type=str, help='Save to file')
    parser.add_argument('--no-show', action='store_true', help='Do not display')

    args = parser.parse_args()

    # Load data
    if args.input.endswith('.csv'):
        df, has_device = load_csv_file(args.input)
        has_pc = 'pos_x' in df.columns
    else:
        df = load_binary_file(args.input)
        has_device = 'dev_pos_x' in df.columns
        has_pc = False  # Binary only has device data, need to run eskf_replay for PC

    time = get_time_axis(df)

    # Default to --all if no option specified
    if not any([args.all, args.sensors, args.attitude, args.position, args.biases, args.trajectory]):
        args.all = True

    # Force compare mode if requested
    if args.compare:
        has_device = has_device or ('dev_pos_x' in df.columns)

    # PC-only mode: hide device data
    if args.pc:
        has_device = False

    figures = []

    if args.all:
        figures.append(('all', plot_all(df, time, has_pc, has_device)))
    else:
        if args.sensors:
            figures.append(('sensors', plot_sensors(df, time)))
        if args.attitude:
            figures.append(('attitude', plot_attitude(df, time, has_pc, has_device)))
        if args.position:
            figures.append(('position', plot_position(df, time, has_pc, has_device)))
        if args.biases:
            figures.append(('biases', plot_biases(df, time, has_pc, has_device)))
        if args.trajectory:
            figures.append(('trajectory', plot_xy_trajectory(df, has_pc, has_device)))

    # Save or show
    if args.save:
        base, ext = os.path.splitext(args.save)
        for name, fig in figures:
            if len(figures) > 1:
                filename = f"{base}_{name}{ext}"
            else:
                filename = args.save
            fig.savefig(filename, dpi=150, bbox_inches='tight')
            print(f"Saved: {filename}")

    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
