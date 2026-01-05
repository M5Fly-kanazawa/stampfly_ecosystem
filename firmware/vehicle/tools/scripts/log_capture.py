#!/usr/bin/env python3
"""
log_capture.py - StampFly Binary Log Capture Tool

Captures binary sensor log packets from StampFly via USB serial and saves to file.
Automatically sends 'binlog on/off' commands to control the device.

Packet format (128 bytes): Header 0xAA 0x56
Contains: IMU, Mag, Baro, ToF, Flow, ESKF estimates

Usage:
    python log_capture.py capture --port /dev/tty.usbmodem* --output sensor_log.bin --duration 60
    python log_capture.py convert --input sensor_log.bin --output sensor_log.csv
    python log_capture.py info sensor_log.bin
"""

import argparse
import serial
import struct
import time
import sys
import signal
from pathlib import Path
from datetime import datetime


# Packet constants
PACKET_SIZE = 128
PACKET_HEADER = bytes([0xAA, 0x56])

# Packet structure format (little-endian)
# 2B header + 4B timestamp + 24B IMU + 12B mag + 8B baro + 8B tof + 5B flow +
# 12B pos + 12B vel + 12B att + 12B bias + 1B status + 15B reserved + 1B checksum = 128B
PACKET_FORMAT = '<2sI6f3f2f2f2hB3f3f3f3fB15sB'


class BinaryLogPacket:
    """Represents a binary log packet (sensor data + ESKF estimates)"""

    def __init__(self, data: bytes):
        if len(data) != PACKET_SIZE:
            raise ValueError(f"Invalid packet size: {len(data)} (expected {PACKET_SIZE})")

        unpacked = struct.unpack(PACKET_FORMAT, data)

        self.header = unpacked[0]
        self.timestamp_ms = unpacked[1]

        # IMU data
        self.accel_x = unpacked[2]
        self.accel_y = unpacked[3]
        self.accel_z = unpacked[4]
        self.gyro_x = unpacked[5]
        self.gyro_y = unpacked[6]
        self.gyro_z = unpacked[7]

        # Mag data
        self.mag_x = unpacked[8]
        self.mag_y = unpacked[9]
        self.mag_z = unpacked[10]

        # Baro data
        self.pressure = unpacked[11]
        self.baro_alt = unpacked[12]

        # ToF data
        self.tof_bottom = unpacked[13]
        self.tof_front = unpacked[14]

        # Flow data
        self.flow_dx = unpacked[15]
        self.flow_dy = unpacked[16]
        self.flow_squal = unpacked[17]

        # ESKF estimates
        self.pos_x = unpacked[18]
        self.pos_y = unpacked[19]
        self.pos_z = unpacked[20]
        self.vel_x = unpacked[21]
        self.vel_y = unpacked[22]
        self.vel_z = unpacked[23]
        self.roll = unpacked[24]   # radians
        self.pitch = unpacked[25]  # radians
        self.yaw = unpacked[26]    # radians
        self.gyro_bias_z = unpacked[27]
        self.accel_bias_x = unpacked[28]
        self.accel_bias_y = unpacked[29]
        self.eskf_status = unpacked[30]
        # unpacked[31] is reserved bytes (15 bytes, ignored)
        self.checksum = unpacked[32]

        self.raw_data = data

    def verify_checksum(self, debug: bool = False) -> bool:
        """Verify packet checksum (XOR of bytes 2-126)"""
        calculated = 0
        for i in range(2, 127):
            calculated ^= self.raw_data[i]
        if debug and calculated != self.checksum:
            print(f"\n[CHECKSUM] Expected: {self.checksum:02x}, Calculated: {calculated:02x}")
            print(f"[CHECKSUM] Header: {self.raw_data[0]:02x} {self.raw_data[1]:02x}")
            print(f"[CHECKSUM] First 16 bytes: {self.raw_data[:16].hex(' ')}")
            print(f"[CHECKSUM] Last 16 bytes: {self.raw_data[-16:].hex(' ')}")
        return calculated == self.checksum

    def __str__(self) -> str:
        import math
        roll_deg = math.degrees(self.roll)
        pitch_deg = math.degrees(self.pitch)
        yaw_deg = math.degrees(self.yaw)
        return (f"t={self.timestamp_ms:8d}ms "
                f"pos=[{self.pos_x:6.3f},{self.pos_y:6.3f},{self.pos_z:6.3f}] "
                f"vel=[{self.vel_x:5.2f},{self.vel_y:5.2f},{self.vel_z:5.2f}] "
                f"att=[{roll_deg:5.1f},{pitch_deg:5.1f},{yaw_deg:6.1f}]deg")


def send_command(ser: serial.Serial, command: str, wait_response: bool = True, debug: bool = False) -> str:
    """Send a command to StampFly CLI and optionally wait for response"""
    # Clear input buffer
    ser.reset_input_buffer()

    # Send Enter first to get a fresh prompt
    ser.write(b'\r\n')
    ser.flush()
    time.sleep(0.1)
    ser.reset_input_buffer()

    # Send command with CR
    cmd_bytes = (command + '\r').encode('utf-8')
    ser.write(cmd_bytes)
    ser.flush()

    if debug:
        print(f"[DEBUG] Sent: {repr(cmd_bytes)}")

    if not wait_response:
        return ""

    # Wait for response
    time.sleep(0.2)
    response = b""
    timeout = time.time() + 3.0

    while time.time() < timeout:
        if ser.in_waiting > 0:
            try:
                chunk = ser.read(ser.in_waiting)
                response += chunk
                if debug:
                    print(f"[DEBUG] Received raw: {chunk.hex(' ')} = {repr(chunk)}")
                # Check if we got a response indicating command was processed
                response_str = response.decode('utf-8', errors='ignore')
                if 'Binary logging' in response_str:
                    break
                if '>' in response_str and len(response_str) > 5:
                    break
            except Exception as e:
                if debug:
                    print(f"[DEBUG] Error: {e}")
        time.sleep(0.05)

    return response.decode('utf-8', errors='ignore')


def find_sync(ser: serial.Serial, timeout: float = 5.0, debug: bool = False) -> bool:
    """Find packet sync header (0xAA 0x56)"""
    start_time = time.time()
    buffer = bytearray()
    bytes_seen = 0

    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            buffer.append(byte[0])
            bytes_seen += 1

            if debug and bytes_seen <= 100:
                print(f"[DEBUG] Byte: {byte[0]:02x}", end=" ")
                if bytes_seen % 16 == 0:
                    print()

            # Keep only last 2 bytes
            if len(buffer) > 2:
                buffer.pop(0)

            # Check for header
            if bytes(buffer) == PACKET_HEADER:
                if debug:
                    print(f"\n[DEBUG] Found sync after {bytes_seen} bytes")
                return True

    if debug:
        print(f"\n[DEBUG] No sync found, saw {bytes_seen} bytes total")
    return False


# Global for signal handler
_capture_running = True
_serial_port = None


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    global _capture_running
    print("\n\nInterrupted! Stopping capture...")
    _capture_running = False


def capture_log(port: str, output: str, duration: float, baudrate: int = 115200,
                show_live: bool = False, auto_control: bool = True, debug: bool = False):
    """
    Capture binary log from serial port and save to file
    """
    global _capture_running, _serial_port
    _capture_running = True

    print(f"Opening serial port: {port} @ {baudrate} baud")
    print(f"Packet format: {PACKET_SIZE} bytes (sensor + ESKF)")

    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        _serial_port = ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    # Setup signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # Clear any pending data
    ser.reset_input_buffer()
    time.sleep(0.3)

    # Drain any existing data
    while ser.in_waiting > 0:
        ser.read(ser.in_waiting)
        time.sleep(0.1)

    if auto_control:
        cmd = "binlog on"
        print(f"Sending '{cmd}' command...")
        response = send_command(ser, cmd, wait_response=True, debug=debug)
        # Filter out binary data (non-printable chars) from response for display
        text_response = ''.join(c for c in response if c.isprintable() or c in '\r\n')
        # Only show up to the prompt
        if '>' in text_response:
            text_response = text_response[:text_response.index('>') + 1]
        print(f"Response: {text_response.strip()}")
        # Wait for ESP_LOG to be suppressed and binary stream to start
        time.sleep(0.3)

    print("Waiting for sync header (0xAA 0x56)...")

    found = find_sync(ser, timeout=10.0, debug=debug)
    if not found:
        print("Timeout waiting for sync header.")
        if auto_control:
            print("Sending 'binlog off' command...")
            send_command(ser, "binlog off", wait_response=False)
        ser.close()
        sys.exit(1)

    print(f"Sync found! Starting capture for {duration} seconds...")
    print(f"Output file: {output}")
    print("Press Ctrl+C to stop early\n")

    start_time = time.time()
    packet_count = 0
    error_count = 0
    last_timestamp = 0

    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        with open(output_path, 'wb') as f:
            # After find_sync, header (2 bytes) has been consumed
            need_header = False

            while _capture_running and time.time() - start_time < duration:
                data = bytearray()

                if need_header:
                    # Read header first
                    found = find_sync(ser, timeout=0.5, debug=False)
                    if not found:
                        continue
                    data.extend(PACKET_HEADER)
                else:
                    # First packet after initial sync - header already consumed
                    data.extend(PACKET_HEADER)
                    need_header = True

                # Read remaining bytes
                remaining = PACKET_SIZE - len(data)
                read_start = time.time()
                while len(data) < PACKET_SIZE and time.time() - read_start < 0.5:
                    chunk = ser.read(remaining)
                    if chunk:
                        data.extend(chunk)
                        remaining = PACKET_SIZE - len(data)

                if len(data) != PACKET_SIZE:
                    continue

                try:
                    pkt = BinaryLogPacket(bytes(data))
                except Exception:
                    continue

                if pkt.verify_checksum(debug=False):
                    # Valid packet
                    f.write(pkt.raw_data)
                    packet_count += 1

                    if show_live and packet_count % 10 == 0:
                        print(f"\r{pkt}", end='', flush=True)

                    if last_timestamp > 0 and pkt.timestamp_ms < last_timestamp:
                        print(f"\nWarning: Timestamp went backwards ({last_timestamp} -> {pkt.timestamp_ms})")

                    last_timestamp = pkt.timestamp_ms
                else:
                    error_count += 1
                    if debug and error_count <= 5:
                        pkt.verify_checksum(debug=True)
                    if error_count % 10 == 0:
                        print(f"\nChecksum errors: {error_count}")

                # Progress indicator
                elapsed = time.time() - start_time
                if packet_count % 100 == 0:
                    rate = packet_count / elapsed if elapsed > 0 else 0
                    print(f"\rPackets: {packet_count}, Rate: {rate:.1f} Hz, "
                          f"Errors: {error_count}, Time: {elapsed:.1f}s / {duration:.1f}s",
                          end='', flush=True)

    finally:
        # Always try to stop binary logging
        if auto_control:
            print("\n\nSending 'binlog off' command...")
            time.sleep(0.1)
            ser.reset_input_buffer()
            send_command(ser, "binlog off", wait_response=False)
            time.sleep(0.1)

        ser.close()

    actual_duration = time.time() - start_time
    print(f"\nCapture complete!")
    print(f"  Packet format: {PACKET_SIZE} bytes")
    print(f"  Total packets: {packet_count}")
    print(f"  Checksum errors: {error_count}")
    print(f"  Error rate: {error_count / (packet_count + error_count) * 100:.2f}%" if packet_count + error_count > 0 else "  Error rate: N/A")
    print(f"  Duration: {actual_duration:.1f} seconds")
    print(f"  Average rate: {packet_count / actual_duration:.1f} Hz" if actual_duration > 0 else "  Average rate: N/A")
    print(f"  File size: {output_path.stat().st_size} bytes")
    print(f"  Output: {output_path}")


def parse_log_file(filepath: str) -> list:
    """Parse a binary log file and return list of packets"""
    packets = []

    with open(filepath, 'rb') as f:
        while True:
            data = f.read(PACKET_SIZE)
            if len(data) < PACKET_SIZE:
                break

            # Verify header
            if data[0:2] != PACKET_HEADER:
                continue

            try:
                pkt = BinaryLogPacket(data)
                if pkt.verify_checksum():
                    packets.append(pkt)
            except Exception as e:
                print(f"Warning: Error parsing packet: {e}")

    return packets


def convert_to_csv(input_file: str, output_file: str):
    """Convert binary log file to CSV"""
    packets = parse_log_file(input_file)

    if not packets:
        print("No valid packets found in file")
        return

    with open(output_file, 'w') as f:
        f.write("timestamp_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,")
        f.write("mag_x,mag_y,mag_z,pressure,baro_alt,tof_bottom,tof_front,")
        f.write("flow_dx,flow_dy,flow_squal,")
        f.write("pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,")
        f.write("roll,pitch,yaw,gyro_bias_z,accel_bias_x,accel_bias_y,eskf_status\n")

        for pkt in packets:
            f.write(f"{pkt.timestamp_ms},{pkt.accel_x:.6f},{pkt.accel_y:.6f},{pkt.accel_z:.6f},")
            f.write(f"{pkt.gyro_x:.6f},{pkt.gyro_y:.6f},{pkt.gyro_z:.6f},")
            f.write(f"{pkt.mag_x:.2f},{pkt.mag_y:.2f},{pkt.mag_z:.2f},")
            f.write(f"{pkt.pressure:.1f},{pkt.baro_alt:.4f},")
            f.write(f"{pkt.tof_bottom:.4f},{pkt.tof_front:.4f},")
            f.write(f"{pkt.flow_dx},{pkt.flow_dy},{pkt.flow_squal},")
            f.write(f"{pkt.pos_x:.6f},{pkt.pos_y:.6f},{pkt.pos_z:.6f},")
            f.write(f"{pkt.vel_x:.6f},{pkt.vel_y:.6f},{pkt.vel_z:.6f},")
            f.write(f"{pkt.roll:.6f},{pkt.pitch:.6f},{pkt.yaw:.6f},")
            f.write(f"{pkt.gyro_bias_z:.6f},{pkt.accel_bias_x:.6f},{pkt.accel_bias_y:.6f},")
            f.write(f"{pkt.eskf_status}\n")

    print(f"Converted {len(packets)} packets to {output_file}")


def main():
    parser = argparse.ArgumentParser(description='StampFly Binary Log Capture Tool')
    subparsers = parser.add_subparsers(dest='command', help='Commands')

    # Capture command
    capture_parser = subparsers.add_parser('capture', help='Capture log from serial port')
    capture_parser.add_argument('--port', '-p', required=True, help='Serial port')
    capture_parser.add_argument('--output', '-o', required=True, help='Output file (.bin)')
    capture_parser.add_argument('--duration', '-d', type=float, default=60.0,
                                help='Capture duration in seconds (default: 60)')
    capture_parser.add_argument('--baudrate', '-b', type=int, default=115200,
                                help='Baudrate (default: 115200)')
    capture_parser.add_argument('--live', '-l', action='store_true',
                                help='Show live packet data')
    capture_parser.add_argument('--no-auto', action='store_true',
                                help='Do not auto-send binlog on/off commands')
    capture_parser.add_argument('--debug', action='store_true',
                                help='Enable debug output')

    # Convert command
    convert_parser = subparsers.add_parser('convert', help='Convert binary log to CSV')
    convert_parser.add_argument('--input', '-i', required=True, help='Input file (.bin)')
    convert_parser.add_argument('--output', '-o', required=True, help='Output file (.csv)')

    # Info command
    info_parser = subparsers.add_parser('info', help='Show info about log file')
    info_parser.add_argument('file', help='Log file (.bin)')

    args = parser.parse_args()

    if args.command == 'capture':
        capture_log(args.port, args.output, args.duration, args.baudrate,
                    args.live, auto_control=not args.no_auto, debug=args.debug)

    elif args.command == 'convert':
        convert_to_csv(args.input, args.output)

    elif args.command == 'info':
        packets = parse_log_file(args.file)
        if packets:
            import math
            print(f"File: {args.file}")
            print(f"Packet size: {PACKET_SIZE} bytes")
            print(f"Packets: {len(packets)}")
            print(f"Duration: {(packets[-1].timestamp_ms - packets[0].timestamp_ms) / 1000:.2f} seconds")
            print(f"Start timestamp: {packets[0].timestamp_ms} ms")
            print(f"End timestamp: {packets[-1].timestamp_ms} ms")
            print(f"\nFirst packet:")
            print(f"  {packets[0]}")
            print(f"\nLast packet:")
            print(f"  {packets[-1]}")

            # Show ESKF statistics
            rolls = [math.degrees(p.roll) for p in packets]
            pitches = [math.degrees(p.pitch) for p in packets]
            yaws = [math.degrees(p.yaw) for p in packets]
            print(f"\nESKF Statistics:")
            print(f"  Roll:  min={min(rolls):.1f}deg, max={max(rolls):.1f}deg")
            print(f"  Pitch: min={min(pitches):.1f}deg, max={max(pitches):.1f}deg")
            print(f"  Yaw:   min={min(yaws):.1f}deg, max={max(yaws):.1f}deg")
            print(f"  ToF:   min={min(p.tof_bottom for p in packets):.3f}m, max={max(p.tof_bottom for p in packets):.3f}m")
            pos_z = [p.pos_z for p in packets]
            print(f"  Pos Z: min={min(pos_z):.3f}m, max={max(pos_z):.3f}m")
        else:
            print("No valid packets found")

    else:
        parser.print_help()


if __name__ == '__main__':
    main()
