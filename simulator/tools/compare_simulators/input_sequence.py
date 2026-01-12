#!/usr/bin/env python3
"""
input_sequence.py - Test Input Sequence Generator
テスト入力シーケンス生成

Generates deterministic input sequences for simulator comparison.
シミュレータ比較用の決定的入力シーケンスを生成
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class ControlInput:
    """Control input at a given time"""
    time: float           # Time [s]
    throttle: float       # Throttle [-1, 1] (0 = hover)
    roll: float          # Roll stick [-1, 1]
    pitch: float         # Pitch stick [-1, 1]
    yaw: float           # Yaw stick [-1, 1]


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


def generate_sine_sequence(duration: float = 10.0, dt: float = 0.0025,
                           frequency: float = 0.5, amplitude: float = 0.3) -> List[ControlInput]:
    """
    Generate sinusoidal input sequence.
    正弦波入力シーケンスを生成

    Sequence:
    - 0-3s: Roll sine wave
    - 3-6s: Pitch sine wave
    - 6-9s: Yaw sine wave
    - 9-10s: Hover
    """
    inputs = []
    t = 0.0

    while t < duration:
        throttle = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        if t < 3.0:
            roll = amplitude * np.sin(2 * np.pi * frequency * t)
        elif t < 6.0:
            pitch = amplitude * np.sin(2 * np.pi * frequency * (t - 3.0))
        elif t < 9.0:
            yaw = amplitude * np.sin(2 * np.pi * frequency * (t - 6.0))

        inputs.append(ControlInput(time=t, throttle=throttle, roll=roll, pitch=pitch, yaw=yaw))
        t += dt

    return inputs


def generate_doublet_sequence(duration: float = 10.0, dt: float = 0.0025,
                               pulse_duration: float = 0.5, amplitude: float = 0.5) -> List[ControlInput]:
    """
    Generate doublet input sequence (system identification pattern).
    ダブレット入力シーケンスを生成（システム同定用パターン）

    Doublet: +amplitude for pulse_duration, then -amplitude for pulse_duration
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


def get_input_at_time(inputs: List[ControlInput], t: float) -> ControlInput:
    """
    Get control input at given time (nearest neighbor).
    指定時刻の制御入力を取得（最近傍）
    """
    if not inputs:
        return ControlInput(time=t, throttle=0, roll=0, pitch=0, yaw=0)

    # Binary search for nearest time
    idx = int(t / (inputs[1].time - inputs[0].time)) if len(inputs) > 1 else 0
    idx = max(0, min(idx, len(inputs) - 1))

    return inputs[idx]


# Test sequences registry
SEQUENCES = {
    'step': generate_step_sequence,
    'sine': generate_sine_sequence,
    'doublet': generate_doublet_sequence,
}


if __name__ == "__main__":
    # Test: print first few inputs of each sequence
    for name, gen_func in SEQUENCES.items():
        inputs = gen_func(duration=2.0)
        print(f"\n{name} sequence (first 10):")
        for inp in inputs[:10]:
            print(f"  t={inp.time:.3f}s: throttle={inp.throttle:.2f}, "
                  f"roll={inp.roll:.2f}, pitch={inp.pitch:.2f}, yaw={inp.yaw:.2f}")
