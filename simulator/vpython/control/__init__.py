# Control modules
# 制御モジュール
"""
Control algorithms for StampFly simulator.
StampFlyシミュレータ用制御アルゴリズム

Modules:
- pid: PID controller with incomplete derivative filter
- rate_controller: Angular velocity (rate) control
- attitude_controller: Attitude (angle) control and altitude hold
- motor_mixer: X-quad motor mixing

Control Modes:
- Rate (Acro): Direct angular velocity control
- Angle (Stabilize): Attitude angle control with self-leveling
- Altitude Hold: Vertical position control
"""

# PID Controller
from .pid import PID, PIDGains, LegacyPID

# Rate Controller
from .rate_controller import (
    RateController,
    RateControlConfig,
    DEFAULT_RATE_CONFIG,
)

# Attitude Controller
from .attitude_controller import (
    AttitudeController,
    AttitudeControlConfig,
    DEFAULT_ATTITUDE_CONFIG,
    AltitudeController,
)

# Motor Mixer
from .motor_mixer import (
    MotorMixer,
    MixerConfig,
    MotorIndex,
    DEFAULT_MIXER_CONFIG,
    mix_motors,
)

__all__ = [
    # PID
    'PID',
    'PIDGains',
    'LegacyPID',
    # Rate Controller
    'RateController',
    'RateControlConfig',
    'DEFAULT_RATE_CONFIG',
    # Attitude Controller
    'AttitudeController',
    'AttitudeControlConfig',
    'DEFAULT_ATTITUDE_CONFIG',
    'AltitudeController',
    # Motor Mixer
    'MotorMixer',
    'MixerConfig',
    'MotorIndex',
    'DEFAULT_MIXER_CONFIG',
    'mix_motors',
]
