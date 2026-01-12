#!/usr/bin/env python3
"""
sim_io.py - Simulator Input/Output Format
シミュレータ入出力フォーマット

Common CSV format for simulator comparison.
シミュレータ比較用の共通CSVフォーマット

Input CSV format:
  time,throttle,roll,pitch,yaw
  0.000,0.0,0.0,0.0,0.0
  0.0025,0.0,0.3,0.0,0.0
  ...

Output CSV format:
  time,x,y,z,roll,pitch,yaw,p,q,r
  0.000,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0
  ...
"""

import csv
import numpy as np
from dataclasses import dataclass, asdict
from typing import List, Optional
from pathlib import Path


# =============================================================================
# Constants
# =============================================================================
CONTROL_DT = 0.0025  # Control loop timestep [s] (400Hz)


# =============================================================================
# Input Format
# =============================================================================

@dataclass
class ControlInput:
    """Control input at a given time / 指定時刻の制御入力"""
    time: float           # Time [s]
    throttle: float       # Throttle [-1, 1] (0 = hover)
    roll: float          # Roll stick [-1, 1]
    pitch: float         # Pitch stick [-1, 1]
    yaw: float           # Yaw stick [-1, 1]


def load_input_csv(filepath: str) -> List[ControlInput]:
    """Load input sequence from CSV / CSVから入力シーケンスを読み込み"""
    inputs = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            inputs.append(ControlInput(
                time=float(row['time']),
                throttle=float(row['throttle']),
                roll=float(row['roll']),
                pitch=float(row['pitch']),
                yaw=float(row['yaw']),
            ))
    return inputs


def save_input_csv(filepath: str, inputs: List[ControlInput]):
    """Save input sequence to CSV / 入力シーケンスをCSVに保存"""
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'throttle', 'roll', 'pitch', 'yaw'])
        for inp in inputs:
            writer.writerow([f"{inp.time:.6f}", f"{inp.throttle:.6f}",
                           f"{inp.roll:.6f}", f"{inp.pitch:.6f}", f"{inp.yaw:.6f}"])


def get_input_at_time(inputs: List[ControlInput], t: float, dt: float = 0.0025) -> ControlInput:
    """
    Get control input at given time (nearest neighbor).
    指定時刻の制御入力を取得（最近傍）
    """
    if not inputs:
        return ControlInput(time=t, throttle=0, roll=0, pitch=0, yaw=0)

    # Find nearest index
    idx = int(t / dt)
    idx = max(0, min(idx, len(inputs) - 1))
    return inputs[idx]


# =============================================================================
# Output Format
# =============================================================================

@dataclass
class StateLog:
    """State log entry / 状態ログエントリ"""
    time: float    # Time [s]
    x: float       # Position X [m]
    y: float       # Position Y [m]
    z: float       # Position Z [m]
    roll: float    # Roll angle [rad]
    pitch: float   # Pitch angle [rad]
    yaw: float     # Yaw angle [rad]
    p: float       # Roll rate [rad/s]
    q: float       # Pitch rate [rad/s]
    r: float       # Yaw rate [rad/s]


def save_output_csv(filepath: str, logs: List[StateLog], metadata: Optional[dict] = None):
    """
    Save state logs to CSV.
    状態ログをCSVに保存

    Args:
        filepath: Output file path
        logs: List of state logs
        metadata: Optional metadata (written as comments)
    """
    with open(filepath, 'w', newline='') as f:
        # Write metadata as comments
        if metadata:
            for key, value in metadata.items():
                f.write(f"# {key}: {value}\n")

        writer = csv.writer(f)
        writer.writerow(['time', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'p', 'q', 'r'])
        for log in logs:
            writer.writerow([
                f"{log.time:.6f}",
                f"{log.x:.6f}", f"{log.y:.6f}", f"{log.z:.6f}",
                f"{log.roll:.6f}", f"{log.pitch:.6f}", f"{log.yaw:.6f}",
                f"{log.p:.6f}", f"{log.q:.6f}", f"{log.r:.6f}",
            ])


def load_output_csv(filepath: str) -> tuple[List[StateLog], dict]:
    """
    Load state logs from CSV.
    CSVから状態ログを読み込み

    Returns:
        (logs, metadata)
    """
    logs = []
    metadata = {}

    with open(filepath, 'r') as f:
        # Read metadata from comments
        for line in f:
            if line.startswith('#'):
                parts = line[1:].strip().split(': ', 1)
                if len(parts) == 2:
                    metadata[parts[0].strip()] = parts[1].strip()
            else:
                break

    with open(filepath, 'r') as f:
        # Skip comments
        reader = csv.DictReader(line for line in f if not line.startswith('#'))
        for row in reader:
            logs.append(StateLog(
                time=float(row['time']),
                x=float(row['x']), y=float(row['y']), z=float(row['z']),
                roll=float(row['roll']), pitch=float(row['pitch']), yaw=float(row['yaw']),
                p=float(row['p']), q=float(row['q']), r=float(row['r']),
            ))

    return logs, metadata


# =============================================================================
# Input Sequence Generators
# =============================================================================

def generate_step_sequence(duration: float = 10.0, dt: float = 0.0025) -> List[ControlInput]:
    """
    Generate step input sequence for testing.
    ステップ入力シーケンスを生成

    Sequence:
    - 0-1s: Hover (no input)
    - 1-2s: Roll step (+0.3)
    - 2-3s: Hover
    - 3-4s: Pitch step (+0.3)
    - 4-5s: Hover
    - 5-6s: Yaw step (+0.3)
    - 6-7s: Hover
    - 7-8s: Throttle step (+0.2)
    - 8-10s: Hover
    """
    inputs = []
    t = 0.0

    while t < duration:
        throttle = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        if 1.0 <= t < 2.0:
            roll = 0.3
        elif 3.0 <= t < 4.0:
            pitch = 0.3
        elif 5.0 <= t < 6.0:
            yaw = 0.3
        elif 7.0 <= t < 8.0:
            throttle = 0.2

        inputs.append(ControlInput(time=t, throttle=throttle, roll=roll, pitch=pitch, yaw=yaw))
        t += dt

    return inputs


def generate_doublet_sequence(duration: float = 10.0, dt: float = 0.0025,
                               pulse_duration: float = 0.3, amplitude: float = 0.5) -> List[ControlInput]:
    """
    Generate doublet input sequence (system identification pattern).
    ダブレット入力シーケンスを生成（システム同定用パターン）
    """
    inputs = []
    t = 0.0

    while t < duration:
        throttle = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        # Roll doublet at t=1s
        if 1.0 <= t < 1.0 + pulse_duration:
            roll = amplitude
        elif 1.0 + pulse_duration <= t < 1.0 + 2 * pulse_duration:
            roll = -amplitude

        # Pitch doublet at t=3s
        if 3.0 <= t < 3.0 + pulse_duration:
            pitch = amplitude
        elif 3.0 + pulse_duration <= t < 3.0 + 2 * pulse_duration:
            pitch = -amplitude

        # Yaw doublet at t=5s
        if 5.0 <= t < 5.0 + pulse_duration:
            yaw = amplitude
        elif 5.0 + pulse_duration <= t < 5.0 + 2 * pulse_duration:
            yaw = -amplitude

        inputs.append(ControlInput(time=t, throttle=throttle, roll=roll, pitch=pitch, yaw=yaw))
        t += dt

    return inputs


# Sequence registry
SEQUENCES = {
    'step': generate_step_sequence,
    'doublet': generate_doublet_sequence,
}


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Generate test input sequence')
    parser.add_argument('--type', '-t', choices=list(SEQUENCES.keys()), default='step',
                       help='Sequence type')
    parser.add_argument('--duration', '-d', type=float, default=10.0,
                       help='Duration [s]')
    parser.add_argument('--output', '-o', type=str, default='input_sequence.csv',
                       help='Output file')
    args = parser.parse_args()

    inputs = SEQUENCES[args.type](duration=args.duration)
    save_input_csv(args.output, inputs)
    print(f"Generated {args.type} sequence ({len(inputs)} samples) -> {args.output}")
