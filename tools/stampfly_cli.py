#!/usr/bin/env python3
"""
StampFly CLI Helper - Send commands to StampFly via WiFi CLI

Usage:
    python tools/stampfly_cli.py [--ip IP] COMMAND [ARGS...]

Commands:
    jump [altitude]              - Jump to altitude and descend (no hovering)
    takeoff [altitude]           - Takeoff and hover at altitude
    land                         - Land gently from current altitude
    hover [altitude] [duration]  - Hover at altitude for duration
    flight status                - Show flight command status
    flight cancel                - Cancel current flight command

Options:
    --ip IP    StampFly IP address (default: 192.168.10.1 for AP mode)
               Use STAMPFLY_IP environment variable or --ip for STA mode

Examples:
    # AP mode (default) - Jump to 15cm and descend
    python tools/stampfly_cli.py jump 0.15

    # STA mode (specify IP)
    python tools/stampfly_cli.py --ip 192.168.1.100 jump 0.15

    # STA mode (environment variable)
    export STAMPFLY_IP=192.168.1.100
    python tools/stampfly_cli.py takeoff 0.3

    # Hover at 20cm for 2 seconds
    python tools/stampfly_cli.py hover 0.2 2.0
"""

import socket
import sys
import time
import os

# StampFly default settings
DEFAULT_IP = "192.168.10.1"  # AP mode default (Tello-compatible subnet)
STAMPFLY_PORT = 23  # Telnet port for WiFi CLI
TIMEOUT = 2.0

def send_command(cmd, ip, silent=False):
    """Send a command to StampFly and return response"""
    try:
        if not silent:
            print(f"🚁 Connecting to StampFly at {ip}:{STAMPFLY_PORT}...")

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(TIMEOUT)
        sock.connect((ip, STAMPFLY_PORT))

        if not silent:
            print("✅ Connected.")

        # Read welcome message
        welcome = sock.recv(1024).decode('utf-8', errors='ignore')

        # Send command
        if not silent:
            print(f"📤 Sending command: {cmd}")
        sock.send(f"{cmd}\n".encode())

        # Wait for command to process (WiFi latency + command execution)
        time.sleep(1.0)

        # Read response with larger buffer
        response = sock.recv(4096).decode('utf-8', errors='ignore')

        sock.close()

        if not silent:
            print(f"📥 Response: {response.strip()}")

        return True, response

    except socket.timeout as e:
        if not silent:
            print(f"❌ Timeout: StampFly not responding ({e})")
        return False, f"Timeout: {e}"
    except ConnectionRefusedError as e:
        if not silent:
            print(f"❌ Connection refused: {e}")
        return False, f"Connection refused: {e}"
    except OSError as e:
        if not silent:
            print(f"❌ Network error: {e}")
        return False, f"Network error: {e}"
    except Exception as e:
        if not silent:
            print(f"❌ Error: {type(e).__name__}: {e}")
        return False, f"{type(e).__name__}: {e}"

def main():
    # Parse command line arguments
    args = sys.argv[1:]

    # Get IP address (priority: --ip flag > STAMPFLY_IP env > default)
    ip = os.getenv('STAMPFLY_IP', DEFAULT_IP)

    if '--ip' in args:
        ip_idx = args.index('--ip')
        if ip_idx + 1 >= len(args):
            print("❌ Error: --ip requires an IP address")
            sys.exit(1)
        ip = args[ip_idx + 1]
        # Remove --ip and its value from args
        args = args[:ip_idx] + args[ip_idx+2:]

    if len(args) < 1:
        print("Usage:")
        print("  python tools/stampfly_cli.py [--ip IP] COMMAND [ARGS...]")
        print("")
        print("Commands:")
        print("  jump [altitude]              - Jump to altitude and descend (no hovering)")
        print("  takeoff [altitude]           - Takeoff and hover at altitude")
        print("  land                         - Land gently from current altitude")
        print("  hover [altitude] [duration]  - Hover at altitude for duration")
        print("  flight status                - Show flight command status")
        print("  flight cancel                - Cancel current flight command")
        print("")
        print("Options:")
        print("  --ip IP    StampFly IP address (default: 192.168.4.1 for AP mode)")
        print("")
        print("Examples:")
        print("  # AP mode (default) - Jump to 15cm and descend")
        print("  python tools/stampfly_cli.py jump 0.15")
        print("")
        print("  # STA mode (specify IP)")
        print("  python tools/stampfly_cli.py --ip 192.168.1.100 jump 0.15")
        print("")
        print("  # STA mode (environment variable)")
        print("  export STAMPFLY_IP=192.168.1.100")
        print("  python tools/stampfly_cli.py takeoff 0.3")
        print("")
        print("  # Hover at 20cm for 2 seconds")
        print("  python tools/stampfly_cli.py hover 0.2 2.0")
        sys.exit(1)

    # Build command from remaining arguments
    cmd = " ".join(args)

    success, response = send_command(cmd, ip)

    if success:
        sys.exit(0)
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
