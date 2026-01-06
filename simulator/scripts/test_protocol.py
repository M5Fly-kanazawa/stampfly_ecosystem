#!/usr/bin/env python3
"""
Protocol Message Test Script
プロトコルメッセージテストスクリプト

Verifies:
1. Message serialization/deserialization
2. Checksum calculation
3. Protocol bridge functionality
4. Binary logging
"""

import sys
import tempfile
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from interfaces import (
    # Messages
    ControlPacket,
    TelemetryPacket,
    TelemetryWSPacket,
    PairingPacket,
    TDMABeacon,
    FlightState,
    ControlFlags,
    WarningFlags,
    SensorStatus,
    checksum_sum,
    checksum_xor,
    identify_packet,
    # Protocol Bridge
    SimulatorState,
    ProtocolBridge,
    BinaryLogger,
    BinaryLogReader,
    log_to_numpy,
    # SIL
    SensorData,
    ActuatorCommand,
    SILInterface,
    SimpleRateController,
)


def test_checksum():
    """Test checksum functions / チェックサム関数をテスト"""
    print("Testing checksum functions...")

    # Test SUM checksum
    data = bytes([0x01, 0x02, 0x03, 0x04])
    assert checksum_sum(data) == 0x0A, "SUM checksum failed"

    # Test XOR checksum
    assert checksum_xor(data) == 0x04, "XOR checksum failed"

    # Test wraparound
    data = bytes([0xFF, 0x02])
    assert checksum_sum(data) == 0x01, "SUM checksum wraparound failed"

    print("  ✓ Checksum tests passed")


def test_control_packet():
    """Test ControlPacket / ControlPacketをテスト"""
    print("Testing ControlPacket...")

    # Create packet
    packet = ControlPacket(
        drone_mac=bytes([0x12, 0x34, 0x56]),
        throttle=500,
        roll=600,
        pitch=400,
        yaw=500,
        flags=ControlFlags.ARM | ControlFlags.MODE,
    )

    # Serialize
    data = packet.pack()
    assert len(data) == ControlPacket.SIZE, f"Size mismatch: {len(data)} != {ControlPacket.SIZE}"

    # Deserialize
    unpacked = ControlPacket.unpack(data)
    assert unpacked is not None, "Unpack failed"
    assert unpacked.throttle == 500, f"Throttle mismatch: {unpacked.throttle}"
    assert unpacked.roll == 600, f"Roll mismatch: {unpacked.roll}"
    assert unpacked.flags & ControlFlags.ARM, "ARM flag missing"

    # Test normalized conversion
    throttle, roll, pitch, yaw = packet.to_normalized()
    assert abs(throttle - 0.5) < 0.01, f"Normalized throttle: {throttle}"
    assert abs(roll - 0.2) < 0.01, f"Normalized roll: {roll}"
    assert abs(pitch + 0.2) < 0.01, f"Normalized pitch: {pitch}"

    # Test from_normalized
    packet2 = ControlPacket.from_normalized(0.5, 0.2, -0.2, 0.0)
    assert packet2.throttle == 500
    assert packet2.roll == 600
    assert packet2.pitch == 400

    print("  ✓ ControlPacket tests passed")


def test_telemetry_packet():
    """Test TelemetryPacket / TelemetryPacketをテスト"""
    print("Testing TelemetryPacket...")

    # Create from state
    packet = TelemetryPacket.from_state(
        sequence=42,
        battery_v=3.8,
        altitude_m=1.5,
        velocity_mps=np.array([0.1, 0.2, -0.05]),
        attitude_rad=np.array([0.1, 0.05, 1.57]),
        state=FlightState.FLYING,
        flags=WarningFlags.LOW_BATTERY,
    )

    # Serialize
    data = packet.pack()
    assert len(data) == TelemetryPacket.SIZE, f"Size mismatch: {len(data)}"

    # Verify header
    assert data[0] == TelemetryPacket.HEADER
    assert data[1] == TelemetryPacket.PACKET_TYPE

    # Deserialize
    unpacked = TelemetryPacket.unpack(data)
    assert unpacked is not None, "Unpack failed"
    assert unpacked.sequence == 42
    assert unpacked.battery_mv == 3800
    assert unpacked.altitude_cm == 150
    assert unpacked.state == FlightState.FLYING
    assert unpacked.flags & WarningFlags.LOW_BATTERY

    print("  ✓ TelemetryPacket tests passed")


def test_telemetry_ws_packet():
    """Test TelemetryWSPacket / TelemetryWSPacketをテスト"""
    print("Testing TelemetryWSPacket...")

    # Create from simulation state
    packet = TelemetryWSPacket.from_simulation_state(
        timestamp_ms=12345,
        attitude_rad=np.array([0.1, 0.2, 0.3]),
        position_m=np.array([1.0, 2.0, -1.5]),
        velocity_mps=np.array([0.5, 0.3, 0.1]),
        gyro_rps=np.array([0.01, 0.02, 0.03]),
        accel_mps2=np.array([0.1, 0.2, 9.8]),
        control=(0.5, 0.1, -0.1, 0.05),
        mag_ut=np.array([20.0, -5.0, 45.0]),
        voltage=3.9,
        flight_state=FlightState.FLYING,
        sensor_status=SensorStatus.IMU_OK | SensorStatus.MAG_OK | SensorStatus.BARO_OK,
        heartbeat=100,
    )

    # Serialize
    data = packet.pack()
    assert len(data) == TelemetryWSPacket.SIZE, f"Size mismatch: {len(data)}"

    # Verify header
    assert data[0] == TelemetryWSPacket.HEADER
    assert data[1] == TelemetryWSPacket.PACKET_TYPE

    # Deserialize
    unpacked = TelemetryWSPacket.unpack(data)
    assert unpacked is not None, "Unpack failed"
    assert unpacked.timestamp_ms == 12345
    assert abs(unpacked.roll - 0.1) < 0.001
    assert abs(unpacked.pos_x - 1.0) < 0.001
    assert abs(unpacked.voltage - 3.9) < 0.001
    assert unpacked.flight_state == FlightState.FLYING
    assert unpacked.sensor_status & SensorStatus.IMU_OK
    assert unpacked.heartbeat == 100

    print("  ✓ TelemetryWSPacket tests passed")


def test_pairing_and_beacon():
    """Test PairingPacket and TDMABeacon / PairingPacketとTDMABeaconをテスト"""
    print("Testing PairingPacket and TDMABeacon...")

    # PairingPacket
    pairing = PairingPacket(
        channel=6,
        drone_mac=bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66]),
    )
    data = pairing.pack()
    assert len(data) == PairingPacket.SIZE

    unpacked = PairingPacket.unpack(data)
    assert unpacked is not None
    assert unpacked.channel == 6
    assert unpacked.drone_mac == bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66])

    # TDMABeacon
    beacon = TDMABeacon()
    data = beacon.pack()
    assert len(data) == TDMABeacon.SIZE
    assert data == TDMABeacon.MARKER

    unpacked = TDMABeacon.unpack(data)
    assert unpacked is not None

    print("  ✓ PairingPacket and TDMABeacon tests passed")


def test_packet_identification():
    """Test packet identification / パケット識別をテスト"""
    print("Testing packet identification...")

    # Control packet
    ctrl = ControlPacket(throttle=500).pack()
    assert identify_packet(ctrl) == 'ControlPacket'

    # Telemetry packet
    telem = TelemetryPacket(sequence=1).pack()
    assert identify_packet(telem) == 'TelemetryPacket'

    # Telemetry WS packet
    telem_ws = TelemetryWSPacket(timestamp_ms=1000).pack()
    assert identify_packet(telem_ws) == 'TelemetryWSPacket'

    # Pairing packet
    pair = PairingPacket().pack()
    assert identify_packet(pair) == 'PairingPacket'

    # TDMA beacon
    beacon = TDMABeacon().pack()
    assert identify_packet(beacon) == 'TDMABeacon'

    print("  ✓ Packet identification tests passed")


def test_protocol_bridge():
    """Test ProtocolBridge / ProtocolBridgeをテスト"""
    print("Testing ProtocolBridge...")

    bridge = ProtocolBridge()

    # Create simulator state
    state = SimulatorState(
        timestamp_ms=5000,
        attitude=np.array([0.1, 0.05, 0.8]),
        position=np.array([1.0, 2.0, -1.5]),
        velocity=np.array([0.5, 0.3, 0.1]),
        gyro=np.array([0.01, 0.02, 0.03]),
        accel=np.array([0.1, 0.2, 9.8]),
        mag=np.array([20.0, -5.0, 45.0]),
        control_throttle=0.6,
        control_roll=0.1,
        battery_voltage=3.85,
        flight_state=FlightState.FLYING,
        sensor_status=SensorStatus.IMU_OK | SensorStatus.BARO_OK,
    )

    # Convert to telemetry
    telem = bridge.state_to_telemetry(state)
    assert telem.sequence == 0
    assert telem.battery_mv == 3850
    assert telem.altitude_cm == 150  # -(-1.5) * 100
    assert telem.state == FlightState.FLYING

    # Convert to WS telemetry
    telem_ws = bridge.state_to_telemetry_ws(state)
    assert telem_ws.timestamp_ms == 5000
    assert abs(telem_ws.roll - 0.1) < 0.001
    assert abs(telem_ws.voltage - 3.85) < 0.001
    assert telem_ws.heartbeat == 0

    # Test sequence increment
    telem2 = bridge.state_to_telemetry(state)
    assert telem2.sequence == 1

    print("  ✓ ProtocolBridge tests passed")


def test_binary_logging():
    """Test binary logging / バイナリロギングをテスト"""
    print("Testing binary logging...")

    with tempfile.NamedTemporaryFile(suffix='.bin', delete=False) as f:
        log_path = Path(f.name)

    try:
        # Write log
        with BinaryLogger(log_path, rate_hz=50) as logger:
            for i in range(100):
                packet = TelemetryWSPacket(
                    timestamp_ms=i * 20,
                    roll=0.1 * np.sin(i * 0.1),
                    pitch=0.05 * np.cos(i * 0.1),
                    voltage=4.2 - i * 0.005,
                    heartbeat=i,
                )
                logger.write_telemetry(packet)

        # Read log
        with BinaryLogReader(log_path) as reader:
            assert reader.version == BinaryLogger.VERSION
            assert reader.rate_hz == 50
            assert reader.packet_size == TelemetryWSPacket.SIZE

            packets = reader.read_all()
            assert len(packets) == 100
            assert packets[0].timestamp_ms == 0
            assert packets[99].heartbeat == 99

        # Convert to numpy
        with BinaryLogReader(log_path) as reader:
            packets = reader.read_all()

        data = log_to_numpy(packets)
        assert len(data['timestamp_ms']) == 100
        assert data['heartbeat'][50] == 50

        print("  ✓ Binary logging tests passed")

    finally:
        log_path.unlink()


def test_sil_interface():
    """Test SIL interface / SILインターフェースをテスト"""
    print("Testing SIL interface...")

    # Create controller
    controller = SimpleRateController(kp=0.5, ki=0.1, kd=0.01)
    controller.set_setpoints(throttle=0.5, roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0)

    # Create SIL interface
    sil = SILInterface(control_rate_hz=400, telemetry_rate_hz=50)
    sil.set_control_callback(controller.update)
    sil.start()

    # Simulate a few steps
    for i in range(100):
        # Create sensor data
        sensor_data = SensorData(
            timestamp_us=i * 2500,  # 400Hz
            gyro=np.array([0.01, 0.02, 0.0]),
            accel=np.array([0.0, 0.0, 9.81]),
            mag=np.array([20.0, -5.0, 45.0]),
        )

        sil.push_sensor_data(sensor_data)
        commands = sil.step(2500)

        if commands is not None:
            assert len(commands.motor_outputs) == 4
            assert all(0 <= m <= 1 for m in commands.motor_outputs)

    # Check telemetry was generated
    telemetry_count = 0
    while True:
        telem = sil.pop_telemetry()
        if telem is None:
            break
        telemetry_count += 1

    assert telemetry_count > 0, "No telemetry generated"

    sil.stop()

    print("  ✓ SIL interface tests passed")


def main():
    """Run all tests / すべてのテストを実行"""
    print("=" * 60)
    print("Protocol Message Tests / プロトコルメッセージテスト")
    print("=" * 60)
    print()

    try:
        test_checksum()
        test_control_packet()
        test_telemetry_packet()
        test_telemetry_ws_packet()
        test_pairing_and_beacon()
        test_packet_identification()
        test_protocol_bridge()
        test_binary_logging()
        test_sil_interface()

        print()
        print("=" * 60)
        print("All tests passed! / すべてのテストに合格！")
        print("=" * 60)
        return 0

    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        return 1
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
