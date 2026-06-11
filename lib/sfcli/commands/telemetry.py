"""
sf telemetry - Live 50Hz telemetry monitor (vehicle_new)

Receives the vehicle_new monitoring telemetry (104-byte binary packet,
magic 0xCAFE, UDP broadcast :5005, 50Hz) and shows a live terminal
dashboard. Optionally records to CSV. For graphs/offline analysis use the
Data Stream instead: `sf log wifi` -> `sf log viz` (400Hz, full sensors).

vehicle_new のモニタ用テレメトリ（104バイト固定バイナリ、magic 0xCAFE、
UDP ブロードキャスト :5005、50Hz）を受信し、ターミナルにライブ表示します。
--csv で記録も可能。グラフ・オフライン解析には Data Stream
（`sf log wifi` → `sf log viz`、400Hz・全センサ）を使ってください。
"""

import argparse
import math
import os
import socket
import struct
import sys
import time
from ..utils import console

COMMAND_NAME = "telemetry"
COMMAND_HELP = "Live 50Hz telemetry — terminal dashboard, or browser with --web"

# Wire format — MUST match firmware/vehicle_new/components/sf_telemetry/
# include/telemetry.hpp TelemetryPacket (static_assert 104 bytes).
# 電文形式 — ファームの TelemetryPacket（104B static_assert）と一致必須。
TELEM_PORT = 5005
TELEM_MAGIC = 0xCAFE
TELEM_FMT = "<HBBI23fB3x"   # magic, version, type, t_us, 23 floats, mode, pad
TELEM_SIZE = struct.calcsize(TELEM_FMT)

FLOAT_NAMES = [
    "roll", "pitch", "yaw",
    "gyro_x", "gyro_y", "gyro_z",
    "accel_x", "accel_y", "accel_z",
    "pos_x", "pos_y", "pos_z",
    "vel_x", "vel_y", "vel_z",
    "thrust", "tau_roll", "tau_pitch", "tau_yaw",
    "m1", "m2", "m3", "m4",
]

# FlightState enum order (firmware/vehicle_new/components/sf_state flight_state.hpp)
# FlightState の列挙順（ファーム側と一致必須）
STATE_NAMES = ["INIT", "IDLE_GROUND", "IDLE_HELD", "ARMED_GROUND",
               "TAKEOFF", "FLYING", "LANDING"]


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
    )
    parser.add_argument(
        "-p", "--port", type=int, default=TELEM_PORT,
        help=f"UDP listen port (default: {TELEM_PORT})",
    )
    parser.add_argument(
        "--csv", metavar="FILE", default=None,
        help="Also append every packet to a CSV file",
    )
    parser.add_argument(
        "--once", action="store_true",
        help="Print a single decoded packet and exit (for scripting/tests)",
    )
    parser.add_argument(
        "--timeout", type=float, default=10.0,
        help="Give up if no packet arrives for this many seconds (default: 10)",
    )
    parser.add_argument(
        "--web", action="store_true",
        help="Browser view instead of the terminal (UDP -> SSE proxy + charts)",
    )
    parser.add_argument(
        "--http-port", type=int, default=5006,
        help="HTTP port for --web (default: 5006)",
    )
    parser.add_argument(
        "--no-browser", action="store_true",
        help="Do not auto-open the browser (--web only)",
    )
    parser.set_defaults(func=run)


def _decode(data: bytes):
    """Decode one packet; returns dict or None if not a telemetry packet.
    1 パケットをデコード。テレメトリでなければ None。"""
    if len(data) != TELEM_SIZE:
        return None
    fields = struct.unpack(TELEM_FMT, data)
    magic, version, ptype, t_us = fields[0], fields[1], fields[2], fields[3]
    if magic != TELEM_MAGIC:
        return None
    out = {"version": version, "type": ptype, "t_us": t_us}
    out.update(dict(zip(FLOAT_NAMES, fields[4:4 + len(FLOAT_NAMES)])))
    out["mode"] = fields[4 + len(FLOAT_NAMES)]
    return out


def _state_name(mode: int) -> str:
    return STATE_NAMES[mode] if 0 <= mode < len(STATE_NAMES) else f"?{mode}"


def _bar(duty: float, width: int = 10) -> str:
    """duty 0..1 -> text bar / duty をテキストバーに"""
    n = max(0, min(width, int(round(duty * width))))
    return "#" * n + "." * (width - n)


def _dashboard(pkt: dict, rate_hz: float, n_packets: int) -> str:
    r2d = 180.0 / math.pi
    alt = -pkt["pos_z"]
    lines = [
        f"StampFly telemetry  :{rate_hz:5.1f} Hz   packets {n_packets}   "
        f"t={pkt['t_us'] / 1e6:9.2f}s",
        f"state : {_state_name(pkt['mode']):<14}",
        f"att   : roll {pkt['roll'] * r2d:+7.2f}  pitch {pkt['pitch'] * r2d:+7.2f}  "
        f"yaw {pkt['yaw'] * r2d:+7.2f}  [deg]",
        f"gyro  : p {pkt['gyro_x'] * r2d:+8.2f}  q {pkt['gyro_y'] * r2d:+8.2f}  "
        f"r {pkt['gyro_z'] * r2d:+8.2f}  [deg/s]",
        f"pos   : N {pkt['pos_x']:+7.2f}  E {pkt['pos_y']:+7.2f}  "
        f"alt {alt:+7.2f}  [m]",
        f"vel   : N {pkt['vel_x']:+6.2f}  E {pkt['vel_y']:+6.2f}  "
        f"D {pkt['vel_z']:+6.2f}  [m/s]",
        f"ctrl  : thrust {pkt['thrust']:6.3f} N   tau "
        f"[{pkt['tau_roll']*1e3:+6.2f} {pkt['tau_pitch']*1e3:+6.2f} "
        f"{pkt['tau_yaw']*1e3:+6.2f}] mNm",
        f"motor : M1 {_bar(pkt['m1'])} {pkt['m1']:4.2f}   "
        f"M2 {_bar(pkt['m2'])} {pkt['m2']:4.2f}",
        f"        M3 {_bar(pkt['m3'])} {pkt['m3']:4.2f}   "
        f"M4 {_bar(pkt['m4'])} {pkt['m4']:4.2f}",
        "",
        "Ctrl-C to quit. For graphs use the Data Stream: sf log wifi -> sf log viz",
    ]
    return "\n".join(lines)


def run(args: argparse.Namespace) -> int:
    if args.web:
        # Browser view (requirements §7 browser display) — UDP -> SSE proxy.
        # Same decoder, same --csv; only the front-end differs.
        # ブラウザ表示（requirements §7）— UDP → SSE プロキシ。デコーダも --csv も
        # 共通で、フロントエンドだけが異なる。
        from . import telemetry_web
        return telemetry_web.serve(http_port=args.http_port,
                                   telemetry_port=args.port,
                                   open_browser=not args.no_browser,
                                   csv_path=args.csv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # Allow `sf telemetry` and `sf monitor web` to listen simultaneously:
    # broadcast reception by multiple processes needs SO_REUSEPORT on
    # macOS/Linux (Windows: SO_REUSEADDR alone suffices; the attr is absent).
    # `sf telemetry` と `sf monitor web` の同時リッスンを許可: 複数プロセスでの
    # ブロードキャスト受信は macOS/Linux では SO_REUSEPORT が必要
    # （Windows は SO_REUSEADDR のみで足り、属性自体が存在しない）。
    if hasattr(socket, "SO_REUSEPORT"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    try:
        # The vehicle broadcasts to 255.255.255.255:5005 — bind the port on
        # all interfaces; no vehicle IP needed.
        # 機体は 255.255.255.255:5005 へブロードキャスト — 全 IF で bind すれば
        # 機体 IP の指定は不要。
        sock.bind(("", args.port))
    except OSError as exc:
        console.error(f"bind :{args.port} failed: {exc}")
        return 1
    sock.settimeout(args.timeout)

    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "a", buffering=1)
        if csv_file.tell() == 0:
            csv_file.write("t_us,mode," + ",".join(FLOAT_NAMES) + "\n")

    # The dashboard redraw uses ANSI escapes. Legacy Windows consoles need VT
    # processing enabled first — the empty system() call is the documented
    # stdlib-only trick (Windows Terminal / macOS / Linux need nothing).
    # ダッシュボード再描画は ANSI エスケープを使う。旧来の Windows コンソールは
    # VT 処理の有効化が必要 — 空 system() は stdlib のみでそれを行う既知の手法
    # （Windows Terminal / macOS / Linux では不要）。
    if os.name == "nt":
        os.system("")

    console.info(f"Listening for telemetry on UDP :{args.port} "
                 f"(timeout {args.timeout:.0f}s)...")

    n_packets = 0
    window = []          # arrival times for the measured-rate display
    last_draw = 0.0
    try:
        while True:
            try:
                data, _addr = sock.recvfrom(2048)
            except socket.timeout:
                if n_packets == 0:
                    console.error(
                        "No telemetry received. Checklist: same WiFi network as "
                        "the vehicle (AP mode: join StampFly-XXXX), vehicle "
                        "powered and out of INIT, no `sf log wifi` capture "
                        "running (exclusive-log mode suppresses telemetry).")
                    return 1
                console.warn("Telemetry stream stopped (timeout)")
                return 1

            pkt = _decode(data)
            if pkt is None:
                continue
            n_packets += 1

            now = time.monotonic()
            window.append(now)
            while window and now - window[0] > 2.0:
                window.pop(0)
            rate_hz = len(window) / 2.0

            if csv_file:
                csv_file.write(
                    f"{pkt['t_us']},{pkt['mode']},"
                    + ",".join(f"{pkt[k]:.6g}" for k in FLOAT_NAMES) + "\n")

            if args.once:
                for key in ["t_us", "mode"] + FLOAT_NAMES:
                    print(f"{key} = {pkt[key]}")
                return 0

            # Redraw at ~10Hz, not per packet (terminal I/O is the bottleneck).
            # 再描画は約10Hz（端末 I/O が律速のためパケット毎にしない）。
            if now - last_draw >= 0.1:
                last_draw = now
                sys.stdout.write("\x1b[H\x1b[2J" + _dashboard(pkt, rate_hz, n_packets) + "\n")
                sys.stdout.flush()
    except KeyboardInterrupt:
        print()
        console.info(f"Stopped. {n_packets} packets received"
                     + (f", CSV: {args.csv}" if args.csv else ""))
        return 0
    finally:
        if csv_file:
            csv_file.close()
        sock.close()
