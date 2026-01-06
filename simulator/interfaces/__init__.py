# Interface modules
# インターフェースモジュール
"""
External interfaces: joystick, protocol bridge, SIL, etc.
外部インターフェース：ジョイスティック、プロトコルブリッジ、SILなど

Modules:
- joystick: HID joystick input (ATOM Joystick)
- messages: Protocol message classes (ControlPacket, TelemetryPacket, etc.)
- protocol_bridge: Simulator state <-> Protocol conversion
- sil_interface: Software-in-the-Loop interface
"""

# Joystick
from .joystick import Joystick

# Protocol Messages
from .messages import (
    # Enums
    FlightState,
    ControlFlags,
    WarningFlags,
    SensorStatus,
    # Checksum functions
    checksum_sum,
    checksum_xor,
    # Packet classes
    ControlPacket,
    TelemetryPacket,
    TelemetryWSPacket,
    PairingPacket,
    TDMABeacon,
    # Utility
    identify_packet,
)

# Protocol Bridge
from .protocol_bridge import (
    SimulatorState,
    ProtocolBridge,
    BinaryLogger,
    BinaryLogReader,
    log_to_numpy,
    numpy_to_log,
)

# SIL Interface
from .sil_interface import (
    SensorData,
    ActuatorCommand,
    SILInterface,
    SimpleRateController,
)

__all__ = [
    # Joystick
    'Joystick',
    # Enums
    'FlightState',
    'ControlFlags',
    'WarningFlags',
    'SensorStatus',
    # Checksum
    'checksum_sum',
    'checksum_xor',
    # Packets
    'ControlPacket',
    'TelemetryPacket',
    'TelemetryWSPacket',
    'PairingPacket',
    'TDMABeacon',
    'identify_packet',
    # Protocol Bridge
    'SimulatorState',
    'ProtocolBridge',
    'BinaryLogger',
    'BinaryLogReader',
    'log_to_numpy',
    'numpy_to_log',
    # SIL Interface
    'SensorData',
    'ActuatorCommand',
    'SILInterface',
    'SimpleRateController',
]
