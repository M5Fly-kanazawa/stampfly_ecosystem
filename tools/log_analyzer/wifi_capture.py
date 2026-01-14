#!/usr/bin/env python3
"""
wifi_capture.py - WiFi Telemetry Capture Tool for FFT Analysis

Captures high-rate telemetry data from StampFly drone via WiFi WebSocket
for sensor FFT analysis and notch filter design.

Supports three packet formats:
- Normal mode (50Hz): 116-byte full packet (header 0xAA)
- FFT batch mode (>50Hz): 120-byte batch packet (header 0xBC, 4 samples)
- Legacy FFT mode: 32-byte single packet (header 0xBB, deprecated)

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
    1. Connect USB to StampFly, run 'fftmode on' in CLI
    2. Disconnect USB, power on with battery
    3. Connect your PC to StampFly WiFi AP
    4. Run: python wifi_capture.py --duration 30 --fft

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


# =============================================================================
# Packet Formats
# =============================================================================

# Normal packet format (116 bytes, header 0xAA)
NORMAL_PACKET_FORMAT = '<'  # Little-endian
NORMAL_PACKET_FORMAT += 'B'    # header (0xAA)
NORMAL_PACKET_FORMAT += 'B'    # packet_type (0x20)
NORMAL_PACKET_FORMAT += 'I'    # timestamp_ms
NORMAL_PACKET_FORMAT += 'fff'  # roll, pitch, yaw
NORMAL_PACKET_FORMAT += 'fff'  # pos_x, pos_y, pos_z
NORMAL_PACKET_FORMAT += 'fff'  # vel_x, vel_y, vel_z
NORMAL_PACKET_FORMAT += 'fff'  # gyro_x, gyro_y, gyro_z
NORMAL_PACKET_FORMAT += 'fff'  # accel_x, accel_y, accel_z
NORMAL_PACKET_FORMAT += 'ffff' # ctrl_throttle, ctrl_roll, ctrl_pitch, ctrl_yaw
NORMAL_PACKET_FORMAT += 'fff'  # mag_x, mag_y, mag_z
NORMAL_PACKET_FORMAT += 'f'    # voltage
NORMAL_PACKET_FORMAT += 'ff'   # tof_bottom, tof_front
NORMAL_PACKET_FORMAT += 'BB'   # flight_state, sensor_status
NORMAL_PACKET_FORMAT += 'I'    # heartbeat
NORMAL_PACKET_FORMAT += 'B'    # checksum
NORMAL_PACKET_FORMAT += '3B'   # padding

NORMAL_PACKET_SIZE = struct.calcsize(NORMAL_PACKET_FORMAT)
assert NORMAL_PACKET_SIZE == 116, f"Normal packet size mismatch: {NORMAL_PACKET_SIZE}"

# FFT Batch packet format (184 bytes, header 0xBC)
# Contains 4 samples of 44 bytes each (with controller inputs)
FFT_SAMPLE_FORMAT = '<Iffffff ffff'  # timestamp_ms, gyro_xyz, accel_xyz, ctrl_throttle/roll/pitch/yaw (44 bytes)
FFT_SAMPLE_SIZE = struct.calcsize(FFT_SAMPLE_FORMAT)
assert FFT_SAMPLE_SIZE == 44, f"FFT sample size mismatch: {FFT_SAMPLE_SIZE}"

FFT_BATCH_SIZE = 4
FFT_BATCH_PACKET_FORMAT = '<'   # Little-endian
FFT_BATCH_PACKET_FORMAT += 'B'  # header (0xBC)
FFT_BATCH_PACKET_FORMAT += 'B'  # packet_type (0x31)
FFT_BATCH_PACKET_FORMAT += 'B'  # sample_count (4)
FFT_BATCH_PACKET_FORMAT += 'B'  # reserved
# 4 samples × 44 bytes = 176 bytes (parsed separately)
FFT_BATCH_HEADER_SIZE = 4
FFT_BATCH_PACKET_SIZE = 184  # 4 + 176 + 4

# Legacy single FFT packet (32 bytes, header 0xBB) - deprecated
LEGACY_FFT_PACKET_FORMAT = '<'
LEGACY_FFT_PACKET_FORMAT += 'B'    # header (0xBB)
LEGACY_FFT_PACKET_FORMAT += 'B'    # packet_type (0x30)
LEGACY_FFT_PACKET_FORMAT += 'I'    # timestamp_ms
LEGACY_FFT_PACKET_FORMAT += 'fff'  # gyro_x, gyro_y, gyro_z
LEGACY_FFT_PACKET_FORMAT += 'fff'  # accel_x, accel_y, accel_z
LEGACY_FFT_PACKET_FORMAT += 'B'    # checksum
LEGACY_FFT_PACKET_FORMAT += 'B'    # padding
LEGACY_FFT_PACKET_SIZE = 32

# CSV columns for FFT mode (with controller inputs)
FFT_CSV_COLUMNS = [
    'timestamp_ms',
    'gyro_x', 'gyro_y', 'gyro_z',
    'accel_x', 'accel_y', 'accel_z',
    'ctrl_throttle', 'ctrl_roll', 'ctrl_pitch', 'ctrl_yaw',
]

# CSV columns for normal mode (full)
NORMAL_CSV_COLUMNS = [
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


def parse_fft_batch_packet(data: bytes) -> list:
    """Parse FFT batch packet (184 bytes, header 0xBC)
    Returns list of 4 sample dicts
    """
    if len(data) != FFT_BATCH_PACKET_SIZE:
        return None

    # Verify header
    if data[0] != 0xBC:
        return None

    # Calculate checksum (XOR of bytes 0 to checksum_offset-1)
    # checksum is at offset 180 (4 header + 176 samples)
    checksum_offset = 4 + FFT_BATCH_SIZE * FFT_SAMPLE_SIZE  # 180
    checksum = 0
    for i in range(checksum_offset):
        checksum ^= data[i]

    if checksum != data[checksum_offset]:
        return None

    # Parse header
    header = data[0]
    packet_type = data[1]
    sample_count = data[2]

    # Parse samples
    samples = []
    for i in range(FFT_BATCH_SIZE):
        offset = 4 + i * FFT_SAMPLE_SIZE  # 4-byte header + sample offset
        sample_data = data[offset:offset + FFT_SAMPLE_SIZE]
        values = struct.unpack(FFT_SAMPLE_FORMAT, sample_data)

        samples.append({
            'timestamp_ms': values[0],
            'gyro_x': values[1],
            'gyro_y': values[2],
            'gyro_z': values[3],
            'accel_x': values[4],
            'accel_y': values[5],
            'accel_z': values[6],
            'ctrl_throttle': values[7],
            'ctrl_roll': values[8],
            'ctrl_pitch': values[9],
            'ctrl_yaw': values[10],
        })

    return samples


def parse_legacy_fft_packet(data: bytes) -> dict:
    """Parse legacy single FFT packet (32 bytes, header 0xBB)"""
    if len(data) != LEGACY_FFT_PACKET_SIZE:
        return None

    values = struct.unpack(LEGACY_FFT_PACKET_FORMAT, data)

    # Verify header
    if values[0] != 0xBB:
        return None

    # Calculate checksum (XOR of bytes 0-29)
    checksum = 0
    for i in range(30):
        checksum ^= data[i]

    if checksum != values[-2]:
        return None

    return {
        'timestamp_ms': values[2],
        'gyro_x': values[3],
        'gyro_y': values[4],
        'gyro_z': values[5],
        'accel_x': values[6],
        'accel_y': values[7],
        'accel_z': values[8],
    }


def parse_normal_packet(data: bytes) -> dict:
    """Parse normal packet (116 bytes, header 0xAA)"""
    if len(data) != NORMAL_PACKET_SIZE:
        return None

    values = struct.unpack(NORMAL_PACKET_FORMAT, data)

    # Verify header
    if values[0] != 0xAA:
        return None

    # Calculate checksum (XOR of bytes 0-111)
    checksum = 0
    for i in range(112):
        checksum ^= data[i]

    if checksum != values[-4]:
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


def parse_packet(data: bytes) -> tuple:
    """
    Parse packet, auto-detecting format from header byte.
    Returns (list_of_samples, mode_string)
    - Normal: returns ([packet_dict], "normal")
    - FFT Batch: returns ([sample1, sample2, sample3, sample4], "fft_batch")
    - Legacy FFT: returns ([packet_dict], "fft_legacy")
    """
    if len(data) == 0:
        return None, None

    header = data[0]

    if header == 0xBC and len(data) == FFT_BATCH_PACKET_SIZE:
        samples = parse_fft_batch_packet(data)
        if samples:
            return samples, "fft_batch"
        return None, None

    elif header == 0xBB and len(data) == LEGACY_FFT_PACKET_SIZE:
        pkt = parse_legacy_fft_packet(data)
        if pkt:
            return [pkt], "fft_legacy"
        return None, None

    elif header == 0xAA and len(data) == NORMAL_PACKET_SIZE:
        pkt = parse_normal_packet(data)
        if pkt:
            return [pkt], "normal"
        return None, None

    else:
        return None, None


class TelemetryCapture:
    """Capture telemetry data from WebSocket"""

    def __init__(self, ip: str = '192.168.4.1', port: int = 80):
        self.uri = f'ws://{ip}:{port}/ws'
        self.packets = []
        self.start_time = None
        self.errors = 0
        self.mode = None  # Detected from first packet
        self.frame_count = 0

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
                            samples, mode = parse_packet(data)
                            if samples:
                                self.packets.extend(samples)
                                self.frame_count += 1
                                # Detect mode from first packet
                                if self.mode is None:
                                    self.mode = mode
                                    mode_str = {
                                        "normal": "Normal (116B)",
                                        "fft_batch": "FFT Batch (120B, 4 samples)",
                                        "fft_legacy": "FFT Legacy (32B)"
                                    }.get(mode, mode)
                                    print(f"Detected mode: {mode_str}")
                            else:
                                self.errors += 1
                    except asyncio.TimeoutError:
                        print("Timeout waiting for packet")
                        continue

                    # Progress update
                    if progress_callback:
                        elapsed = time.time() - self.start_time
                        progress_callback(elapsed, duration, len(self.packets), self.frame_count)

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

        # Choose columns based on mode
        is_fft = self.mode in ("fft_batch", "fft_legacy")
        columns = FFT_CSV_COLUMNS if is_fft else NORMAL_CSV_COLUMNS

        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
            writer.writeheader()
            for pkt in self.packets:
                writer.writerow(pkt)

        print(f"Saved {len(self.packets)} samples to {filename}")
        return True

    def print_stats(self):
        """Print capture statistics"""
        if not self.packets:
            print("No packets captured")
            return

        n = len(self.packets)
        duration = (self.packets[-1]['timestamp_ms'] - self.packets[0]['timestamp_ms']) / 1000.0
        sample_rate = n / duration if duration > 0 else 0
        frame_rate = self.frame_count / duration if duration > 0 else 0

        mode_str = {
            "normal": "Normal (116B)",
            "fft_batch": "FFT Batch (120B, 4 samples/frame)",
            "fft_legacy": "FFT Legacy (32B)"
        }.get(self.mode, self.mode)

        print(f"\n=== Capture Statistics ===")
        print(f"Mode: {mode_str}")
        print(f"Samples: {n}")
        print(f"Frames: {self.frame_count}")
        print(f"Errors: {self.errors}")
        print(f"Duration: {duration:.2f}s")
        print(f"Sample rate: {sample_rate:.1f} Hz")
        print(f"Frame rate: {frame_rate:.1f} Hz")
        print(f"Nyquist: {sample_rate/2:.1f} Hz")

        # Gyro/Accel stats
        if HAS_NUMPY:
            gyro_x = np.array([p['gyro_x'] for p in self.packets])
            gyro_y = np.array([p['gyro_y'] for p in self.packets])
            gyro_z = np.array([p['gyro_z'] for p in self.packets])
            accel_x = np.array([p['accel_x'] for p in self.packets])
            accel_y = np.array([p['accel_y'] for p in self.packets])
            accel_z = np.array([p['accel_z'] for p in self.packets])

            print(f"\nGyro [rad/s]:")
            print(f"  X: mean={np.mean(gyro_x):+.4f}, std={np.std(gyro_x):.4f}")
            print(f"  Y: mean={np.mean(gyro_y):+.4f}, std={np.std(gyro_y):.4f}")
            print(f"  Z: mean={np.mean(gyro_z):+.4f}, std={np.std(gyro_z):.4f}")
            print(f"Accel [m/s^2]:")
            print(f"  X: mean={np.mean(accel_x):+.2f}, std={np.std(accel_x):.2f}")
            print(f"  Y: mean={np.mean(accel_y):+.2f}, std={np.std(accel_y):.2f}")
            print(f"  Z: mean={np.mean(accel_z):+.2f}, std={np.std(accel_z):.2f}")

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
        print(f"Frequency resolution: {fs/n:.3f} Hz")
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
        fig.suptitle(f'FFT Analysis (Fs={fs:.1f}Hz, N={n}, Nyquist={fs/2:.1f}Hz)', fontsize=12)

        signals = [
            (gyro_x, 'Gyro X [rad/s]'),
            (gyro_y, 'Gyro Y [rad/s]'),
            (gyro_z, 'Gyro Z [rad/s]'),
            (accel_x, 'Accel X [m/s^2]'),
            (accel_y, 'Accel Y [m/s^2]'),
            (accel_z, 'Accel Z [m/s^2]'),
        ]

        for ax, (signal, label) in zip(axes.flat, signals):
            fft_db = compute_fft_db(signal)
            ax.plot(freq, fft_db, 'b-', linewidth=0.5)
            ax.set_xlabel('Frequency [Hz]')
            ax.set_ylabel('Magnitude [dB]')
            ax.set_title(label)
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0, min(fs/2, 250))  # Limit to Nyquist or 250Hz

            # Find top 3 peaks (excluding DC)
            fft_db_no_dc = fft_db.copy()
            fft_db_no_dc[:3] = -100  # Mask DC and very low freq
            for i in range(3):
                peak_idx = np.argmax(fft_db_no_dc)
                if fft_db_no_dc[peak_idx] > -60:  # Only show significant peaks
                    peak_freq = freq[peak_idx]
                    peak_db = fft_db[peak_idx]
                    color = ['r', 'orange', 'green'][i]
                    ax.axvline(peak_freq, color=color, linewidth=0.8, alpha=0.7)
                    ax.annotate(f'{peak_freq:.1f}Hz', xy=(peak_freq, peak_db),
                               fontsize=7, color=color)
                    # Mask this peak and nearby frequencies
                    mask_width = max(3, int(n * 0.01))
                    fft_db_no_dc[max(0, peak_idx-mask_width):peak_idx+mask_width] = -100

        plt.tight_layout()
        plt.show()


def progress_bar(elapsed, total, samples, frames):
    """Print progress bar"""
    pct = elapsed / total * 100
    bar_len = 30
    filled = int(bar_len * elapsed / total)
    bar = '=' * filled + '-' * (bar_len - filled)
    sample_rate = samples / elapsed if elapsed > 0 else 0
    frame_rate = frames / elapsed if elapsed > 0 else 0
    print(f"\r[{bar}] {pct:.0f}% | {samples} samples | {sample_rate:.0f} Hz | {frames} frames", end='', flush=True)


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
