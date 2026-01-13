#!/usr/bin/env python3
"""
wifi_capture.py - WiFi Telemetry Capture Tool for FFT Analysis

Captures high-rate telemetry data from StampFly drone via WiFi WebSocket
for sensor FFT analysis and notch filter design.

Usage:
    python wifi_capture.py [options]

Options:
    -o, --output FILE   Output CSV filename (default: auto-generated)
    -d, --duration SEC  Capture duration in seconds (default: 30)
    -i, --ip IP         StampFly IP address (default: 192.168.4.1)
    -p, --port PORT     WebSocket port (default: 80)
    --fft               Run FFT analysis after capture
    --no-save           Don't save to file, just display stats

Workflow:
    1. Connect USB to StampFly, run 'fftmode on' in CLI (160Hz max)
    2. Disconnect USB, power on with battery
    3. Connect your PC to StampFly WiFi AP
    4. Run: python wifi_capture.py --duration 30 --fft

Note: WiFi bandwidth limits max rate to ~160Hz (Nyquist: 80Hz)

Examples:
    python wifi_capture.py                    # Capture 30s, auto-save
    python wifi_capture.py -d 60 -o flight.csv --fft
    python wifi_capture.py --no-save          # Just check connection
"""

import argparse
import asyncio
import csv
import math
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import websockets
except ImportError:
    print("Error: websockets library required")
    print("Install with: pip install websockets")
    sys.exit(1)

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


# Packet format (matches TelemetryWSPacket in firmware)
# Total size: 116 bytes
PACKET_FORMAT = '<'  # Little-endian
PACKET_FORMAT += 'B'    # header (0xAA)
PACKET_FORMAT += 'B'    # packet_type (0x20)
PACKET_FORMAT += 'I'    # timestamp_ms
PACKET_FORMAT += 'fff'  # roll, pitch, yaw
PACKET_FORMAT += 'fff'  # pos_x, pos_y, pos_z
PACKET_FORMAT += 'fff'  # vel_x, vel_y, vel_z
PACKET_FORMAT += 'fff'  # gyro_x, gyro_y, gyro_z
PACKET_FORMAT += 'fff'  # accel_x, accel_y, accel_z
PACKET_FORMAT += 'ffff' # ctrl_throttle, ctrl_roll, ctrl_pitch, ctrl_yaw
PACKET_FORMAT += 'fff'  # mag_x, mag_y, mag_z
PACKET_FORMAT += 'f'    # voltage
PACKET_FORMAT += 'ff'   # tof_bottom, tof_front
PACKET_FORMAT += 'BB'   # flight_state, sensor_status
PACKET_FORMAT += 'I'    # heartbeat
PACKET_FORMAT += 'B'    # checksum
PACKET_FORMAT += '3B'   # padding

PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
assert PACKET_SIZE == 116, f"Packet size mismatch: {PACKET_SIZE} != 116"

# CSV header columns
CSV_COLUMNS = [
    'timestamp_ms',
    'roll_deg', 'pitch_deg', 'yaw_deg',
    'pos_x', 'pos_y', 'pos_z',
    'vel_x', 'vel_y', 'vel_z',
    'gyro_x', 'gyro_y', 'gyro_z',
    'accel_x', 'accel_y', 'accel_z',
    'throttle', 'ctrl_roll', 'ctrl_pitch', 'ctrl_yaw',
    'mag_x', 'mag_y', 'mag_z',
    'voltage',
    'tof_bottom', 'tof_front',
    'state', 'sensor_status',
    'heartbeat'
]


def parse_packet(data: bytes) -> dict:
    """Parse binary telemetry packet into dict"""
    if len(data) != PACKET_SIZE:
        return None

    values = struct.unpack(PACKET_FORMAT, data)

    # Verify header and checksum
    header = values[0]
    if header != 0xAA:
        return None

    # Calculate checksum (XOR of bytes 0-111)
    checksum = 0
    for i in range(112):
        checksum ^= data[i]

    if checksum != values[-4]:  # checksum is at index -4 before padding
        return None

    return {
        'timestamp_ms': values[2],
        'roll_deg': math.degrees(values[3]),
        'pitch_deg': math.degrees(values[4]),
        'yaw_deg': math.degrees(values[5]),
        'pos_x': values[6],
        'pos_y': values[7],
        'pos_z': values[8],
        'vel_x': values[9],
        'vel_y': values[10],
        'vel_z': values[11],
        'gyro_x': values[12],
        'gyro_y': values[13],
        'gyro_z': values[14],
        'accel_x': values[15],
        'accel_y': values[16],
        'accel_z': values[17],
        'throttle': values[18],
        'ctrl_roll': values[19],
        'ctrl_pitch': values[20],
        'ctrl_yaw': values[21],
        'mag_x': values[22],
        'mag_y': values[23],
        'mag_z': values[24],
        'voltage': values[25],
        'tof_bottom': values[26],
        'tof_front': values[27],
        'state': values[28],
        'sensor_status': values[29],
        'heartbeat': values[30],
    }


class TelemetryCapture:
    """Capture telemetry data from WebSocket"""

    def __init__(self, ip: str = '192.168.4.1', port: int = 80):
        self.uri = f'ws://{ip}:{port}/ws'
        self.packets = []
        self.start_time = None
        self.errors = 0

    async def capture(self, duration: float, progress_callback=None):
        """Capture packets for specified duration"""
        print(f"Connecting to {self.uri}...")

        try:
            async with websockets.connect(self.uri, ping_interval=None) as ws:
                print(f"Connected! Capturing for {duration}s...")
                self.start_time = time.time()
                end_time = self.start_time + duration

                while time.time() < end_time:
                    try:
                        data = await asyncio.wait_for(ws.recv(), timeout=1.0)
                        if isinstance(data, bytes):
                            packet = parse_packet(data)
                            if packet:
                                self.packets.append(packet)
                            else:
                                self.errors += 1
                    except asyncio.TimeoutError:
                        print("Timeout waiting for packet")
                        continue

                    # Progress update
                    if progress_callback:
                        elapsed = time.time() - self.start_time
                        progress_callback(elapsed, duration, len(self.packets))

        except ConnectionRefusedError:
            print(f"Error: Connection refused to {self.uri}")
            print("Make sure StampFly WiFi AP is active and you're connected to it")
            return False
        except Exception as e:
            print(f"Error: {e}")
            return False

        return True

    def save_csv(self, filename: str):
        """Save captured packets to CSV"""
        if not self.packets:
            print("No packets to save")
            return False

        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            for pkt in self.packets:
                # Map packet fields to CSV columns
                row = {col: pkt.get(col, '') for col in CSV_COLUMNS}
                writer.writerow(row)

        print(f"Saved {len(self.packets)} packets to {filename}")
        return True

    def print_stats(self):
        """Print capture statistics"""
        if not self.packets:
            print("No packets captured")
            return

        n = len(self.packets)
        duration = (self.packets[-1]['timestamp_ms'] - self.packets[0]['timestamp_ms']) / 1000.0
        rate = n / duration if duration > 0 else 0

        print(f"\n=== Capture Statistics ===")
        print(f"Packets: {n}")
        print(f"Errors: {self.errors}")
        print(f"Duration: {duration:.2f}s")
        print(f"Rate: {rate:.1f} Hz")

        # Voltage stats
        voltages = [p['voltage'] for p in self.packets]
        print(f"Voltage: {min(voltages):.2f} - {max(voltages):.2f}V")

        # Gyro stats (for FFT preview)
        if HAS_NUMPY:
            gyro_x = np.array([p['gyro_x'] for p in self.packets])
            gyro_y = np.array([p['gyro_y'] for p in self.packets])
            gyro_z = np.array([p['gyro_z'] for p in self.packets])
            print(f"Gyro X: mean={np.mean(gyro_x):.4f}, std={np.std(gyro_x):.4f} rad/s")
            print(f"Gyro Y: mean={np.mean(gyro_y):.4f}, std={np.std(gyro_y):.4f} rad/s")
            print(f"Gyro Z: mean={np.mean(gyro_z):.4f}, std={np.std(gyro_z):.4f} rad/s")

    def run_fft_analysis(self):
        """Run FFT analysis on captured data"""
        if not HAS_NUMPY:
            print("FFT analysis requires numpy: pip install numpy")
            return

        if len(self.packets) < 64:
            print("Not enough data for FFT (need at least 64 samples)")
            return

        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("FFT visualization requires matplotlib: pip install matplotlib")
            return

        # Extract data
        n = len(self.packets)
        timestamps = np.array([p['timestamp_ms'] for p in self.packets])
        dt = np.mean(np.diff(timestamps)) / 1000.0  # seconds
        fs = 1.0 / dt  # sampling frequency

        gyro_x = np.array([p['gyro_x'] for p in self.packets])
        gyro_y = np.array([p['gyro_y'] for p in self.packets])
        gyro_z = np.array([p['gyro_z'] for p in self.packets])
        accel_x = np.array([p['accel_x'] for p in self.packets])
        accel_y = np.array([p['accel_y'] for p in self.packets])
        accel_z = np.array([p['accel_z'] for p in self.packets])

        print(f"\n=== FFT Analysis ===")
        print(f"Samples: {n}")
        print(f"Sample rate: {fs:.1f} Hz")
        print(f"Frequency resolution: {fs/n:.2f} Hz")
        print(f"Max frequency: {fs/2:.1f} Hz (Nyquist)")

        # Compute FFT
        freq = np.fft.rfftfreq(n, dt)

        def compute_fft_db(signal):
            """Compute FFT magnitude in dB"""
            fft = np.fft.rfft(signal - np.mean(signal))  # Remove DC
            magnitude = np.abs(fft) / n
            return 20 * np.log10(magnitude + 1e-10)

        # Plot
        fig, axes = plt.subplots(2, 3, figsize=(14, 8))
        fig.suptitle(f'FFT Analysis (Fs={fs:.1f}Hz, N={n})', fontsize=12)

        signals = [
            (gyro_x, 'Gyro X [rad/s]'),
            (gyro_y, 'Gyro Y [rad/s]'),
            (gyro_z, 'Gyro Z [rad/s]'),
            (accel_x, 'Accel X [m/s²]'),
            (accel_y, 'Accel Y [m/s²]'),
            (accel_z, 'Accel Z [m/s²]'),
        ]

        for ax, (signal, label) in zip(axes.flat, signals):
            fft_db = compute_fft_db(signal)
            ax.plot(freq, fft_db, 'b-', linewidth=0.5)
            ax.set_xlabel('Frequency [Hz]')
            ax.set_ylabel('Magnitude [dB]')
            ax.set_title(label)
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0, min(200, fs/2))  # Limit to 200Hz or Nyquist

            # Find peaks
            peak_idx = np.argmax(fft_db[1:]) + 1  # Skip DC
            peak_freq = freq[peak_idx]
            peak_db = fft_db[peak_idx]
            ax.axvline(peak_freq, color='r', linewidth=0.5, alpha=0.5)
            ax.annotate(f'{peak_freq:.1f}Hz', xy=(peak_freq, peak_db),
                       fontsize=8, color='r')

        plt.tight_layout()
        plt.show()


def progress_bar(elapsed, total, packets):
    """Print progress bar"""
    pct = elapsed / total * 100
    bar_len = 30
    filled = int(bar_len * elapsed / total)
    bar = '=' * filled + '-' * (bar_len - filled)
    rate = packets / elapsed if elapsed > 0 else 0
    print(f"\r[{bar}] {pct:.0f}% | {packets} packets | {rate:.0f} Hz", end='', flush=True)


async def main():
    parser = argparse.ArgumentParser(
        description='Capture WiFi telemetry for FFT analysis',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('-o', '--output', help='Output CSV filename')
    parser.add_argument('-d', '--duration', type=float, default=30,
                       help='Capture duration in seconds (default: 30)')
    parser.add_argument('-i', '--ip', default='192.168.4.1',
                       help='StampFly IP address (default: 192.168.4.1)')
    parser.add_argument('-p', '--port', type=int, default=80,
                       help='WebSocket port (default: 80)')
    parser.add_argument('--fft', action='store_true',
                       help='Run FFT analysis after capture')
    parser.add_argument('--no-save', action='store_true',
                       help="Don't save to file")

    args = parser.parse_args()

    # Generate output filename if not specified
    if not args.output and not args.no_save:
        timestamp = datetime.now().strftime('%Y%m%dT%H%M%S')
        args.output = f'stampfly_fft_{timestamp}.csv'

    # Create capture instance
    capture = TelemetryCapture(args.ip, args.port)

    # Run capture
    success = await capture.capture(args.duration, progress_bar)
    print()  # Newline after progress bar

    if not success:
        return 1

    # Print statistics
    capture.print_stats()

    # Save to CSV
    if not args.no_save and args.output:
        capture.save_csv(args.output)

    # Run FFT analysis
    if args.fft:
        capture.run_fft_analysis()

    return 0


if __name__ == '__main__':
    sys.exit(asyncio.run(main()))
