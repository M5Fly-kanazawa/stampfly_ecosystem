"""
sf rc - RC (Remote Control) command
RC（遠隔操縦）コマンド

Send `rc a b c d` lines to the vehicle on UDP port 8889 (lib/sfcli/utils/vehicle_api.py). The
vehicle never answers `rc`, and ignores it unless it is flying under the API (after `sf takeoff`).
A stream that stops makes the vehicle hold its position.
機体の UDP 8889 へ `rc a b c d` を送る（lib/sfcli/utils/vehicle_api.py）。機体は `rc` に応答せず、
API で飛行中（`sf takeoff` の後）でなければ無視する。送信が止まると機体は位置を保持する。

Modes:
    sf rc <a> <b> <c> <d>   - One-shot: send a single rc line
    sf rc                   - Interactive: keyboard control mode

Parameters (Tello SDK compatible):
    a: left/right   (-100~100) → roll
    b: forward/back  (-100~100) → pitch
    c: up/down       (-100~100) → throttle
    d: yaw           (-100~100) → yaw

Exit status / 終了コード: 0 sent, 1 refused or the vehicle did not answer `command`,
3 no reply to `command` (timeout).
0 送信した、1 拒否または `command` へのエラー応答、3 `command` への応答なし（タイムアウト）。
"""

import argparse
import sys
import time

# Terminal raw mode modules (Unix only)
# ターミナル生入力モジュール（Unix専用）
try:
    import select
    import termios
    import tty
    _HAS_TERMIOS = True
except ImportError:
    _HAS_TERMIOS = False

from ..utils import console
from ..utils.vehicle_api import (
    DEFAULT_HOST,
    KIND_OK,
    KIND_TIMEOUT,
    LAND_TIMEOUT_S,
    RC_LIMIT,
    ApiResult,
    VehicleApi,
)

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_TIMEOUT = 3


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register rc command with CLI

    rcコマンドをCLIに登録する
    """
    parser = subparsers.add_parser(
        "rc",
        help="RC control (one-shot or interactive keyboard)",
        description=(
            "Send RC control inputs to StampFly.\n"
            "  sf rc <a> <b> <c> <d>  — One-shot mode (single command)\n"
            "  sf rc                  — Interactive keyboard control mode\n\n"
            "Parameters: a=roll, b=pitch, c=throttle, d=yaw (-100~100)"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "values",
        nargs="*",
        type=int,
        help="RC values: <a> <b> <c> <d> (-100~100). Omit for interactive mode.",
    )
    parser.add_argument(
        "--ip",
        default=DEFAULT_HOST,
        help=f"Vehicle IP address (default: {DEFAULT_HOST})",
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=20,
        help="Send rate in Hz for interactive mode (default: 20)",
    )
    parser.set_defaults(func=run_rc)


def run_rc(args: argparse.Namespace) -> int:
    """Execute RC command (one-shot or interactive)"""
    if args.values:
        if len(args.values) != 4:
            console.error("Expected 4 values: sf rc <a> <b> <c> <d>")
            return 1
        for v in args.values:
            if v < -RC_LIMIT or v > RC_LIMIT:
                console.error(f"Values must be -{RC_LIMIT}~{RC_LIMIT}")
                return 1
        return _run_oneshot(args)
    else:
        return _run_interactive(args)


# =============================================================================
# One-shot mode
# =============================================================================

def _run_oneshot(args: argparse.Namespace) -> int:
    """Send a single rc line and report what happened.

    rc を 1 行送り、結果を報告する
    """
    a, b, c, d = args.values
    console.info(f"RC one-shot: a={a} b={b} c={c} d={d}", prefix="RC")
    api = VehicleApi(args.ip)
    try:
        result = api.send_without_reply(f"rc {a} {b} {c} {d}")
    except OSError as error:
        console.error(f"RC: cannot send to {args.ip}:{api.port} ({error})")
        return EXIT_ERROR
    if result.kind == KIND_TIMEOUT:
        console.error("RC: no reply to command from the vehicle (timeout)")
        return EXIT_TIMEOUT
    if result.kind != KIND_OK:
        console.error(f"RC: {result.text}")
        return EXIT_ERROR
    console.success("rc sent (the vehicle does not answer rc; it ignores it unless flying under the API)")
    return EXIT_OK


# =============================================================================
# Interactive mode
# =============================================================================

# Key mappings for interactive control
# インタラクティブ制御のキーマッピング
KEY_HELP = """
  [RC Interactive Mode / インタラクティブモード]

  W/S     Forward / Backward (pitch)
  A/D     Left / Right (roll)
  Q/E     Yaw left / Yaw right
  Space   Up (throttle)
  Z       Down (throttle)
  +/-     Speed adjust (10-100)
  L       Land
  X       Emergency stop
  Ctrl+C  Quit (sends rc 0 0 0 0; the vehicle then holds its position)
"""


def _run_interactive(args: argparse.Namespace) -> int:
    """Run interactive keyboard RC control mode."""
    # termios is required for raw keyboard input (Unix only)
    # termiosはキーボード生入力に必要（Unix専用）
    if not _HAS_TERMIOS:
        console.error("Interactive RC mode is not supported on Windows")
        console.error("Use one-shot mode: sf rc <a> <b> <c> <d>")
        return 1

    # Check if stdin is a terminal
    # 標準入力がターミナルかチェック
    if not sys.stdin.isatty():
        console.error("Interactive mode requires a terminal")
        return 1

    console.info("Starting interactive RC mode", prefix="RC")
    console.print(KEY_HELP)

    try:
        return _interactive_loop(args.ip, args.rate)
    except KeyboardInterrupt:
        return 0


def _interactive_loop(host: str, rate: int) -> int:
    """Keyboard loop: read keys, send one rc line per period.

    キーボードループ: キーを読み、周期ごとに rc を 1 行送る
    """
    api = VehicleApi(host)
    try:
        stream = api.open_rc_stream()
    except OSError as error:
        console.error(f"RC: cannot reach {host}:{api.port} ({error})")
        return EXIT_ERROR
    if stream.handshake.kind != KIND_OK:
        stream.close()
        if stream.handshake.kind == KIND_TIMEOUT:
            console.error("RC: no reply to command from the vehicle (timeout)")
            return EXIT_TIMEOUT
        console.error(f"RC: {stream.handshake.text}")
        return EXIT_ERROR

    console.success("Command mode entered")
    console.print("  Press keys to control. Ctrl+C to quit.\n")

    # State for RC values
    # RC値の状態
    rc_state = {
        "a": 0,   # roll (left/right)
        "b": 0,   # pitch (forward/back)
        "c": 0,   # throttle (up/down)
        "d": 0,   # yaw
        "speed": 50,
        "running": True,
        "last_line_len": 0,
    }

    # Track which keys are pressed (for key release detection)
    # 押されているキーの追跡（キーリリース検出用）
    key_timers = {}

    # Save terminal settings
    # ターミナル設定を保存
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    interval = 1.0 / rate
    start_time = time.monotonic()
    land_requested = False
    emergency_requested = False

    try:
        # Set raw mode for character-by-character input
        # 1文字ずつ入力するためにrawモードに設定
        tty.setraw(fd)

        while rc_state["running"]:
            loop_start = time.monotonic()

            # Read all available keys (non-blocking)
            # 利用可能な全キーを読み取り（ノンブロッキング）
            while select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                _handle_key(ch, rc_state, key_timers, loop_start)

            # Key release detection: zero out axes if key not pressed recently
            # キーリリース検出: 最近押されていない軸をゼロにする
            _update_key_release(rc_state, key_timers, loop_start, timeout=0.08)

            if rc_state.pop("_land", False):
                land_requested = True
                break
            if rc_state.pop("_emergency", False):
                emergency_requested = True
                break

            # Send one rc line (fire-and-forget)
            # rc を 1 行送る（撃ちっぱなし）
            stream.send(rc_state["a"], rc_state["b"], rc_state["c"], rc_state["d"])

            _display_status(rc_state, start_time)

            # Rate limiting
            # レート制限
            elapsed = time.monotonic() - loop_start
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings
        # ターミナル設定を復元
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        # Send neutral rc to stop the stream's motion
        # ニュートラルの rc を送り、ストリームの動きを止める
        console.print("\n")
        console.info("Sending rc 0 0 0 0...", prefix="RC")
        stream.send(0, 0, 0, 0)
        stream.close()

    if emergency_requested:
        return _finish_with(api.emergency(), "EMERGENCY")
    if land_requested:
        console.info("land (waiting for the vehicle to reach the ground)", prefix="LAND")
        return _finish_with(api.send("land", LAND_TIMEOUT_S), "LAND")
    return EXIT_OK


def _finish_with(result: ApiResult, label: str) -> int:
    """Print the vehicle's reply to the closing land / emergency; return the exit status.

    締めの land / emergency への機体の応答を表示し、終了コードを返す
    """
    if result.kind == KIND_TIMEOUT:
        console.error(f"{label}: no reply from the vehicle (timeout)")
        return EXIT_TIMEOUT
    if not result.succeeded:
        console.error(f"{label}: {result.text}")
        return EXIT_ERROR
    console.success(f"{label}: {result.text}")
    return EXIT_OK


def _handle_key(ch: str, state: dict, timers: dict, now: float) -> None:
    """Handle a single keypress.

    キー入力を処理する
    """
    spd = state["speed"]

    if ch in ('\x03',):  # Ctrl+C
        state["running"] = False
    elif ch == 'w':
        state["b"] = spd
        timers["b+"] = now
    elif ch == 's':
        state["b"] = -spd
        timers["b-"] = now
    elif ch == 'a':
        state["a"] = -spd
        timers["a-"] = now
    elif ch == 'd':
        state["a"] = spd
        timers["a+"] = now
    elif ch == 'q':
        state["d"] = -spd
        timers["d-"] = now
    elif ch == 'e':
        state["d"] = spd
        timers["d+"] = now
    elif ch == ' ':
        state["c"] = spd
        timers["c+"] = now
    elif ch == 'z':
        state["c"] = -spd
        timers["c-"] = now
    elif ch in ('+', '='):
        state["speed"] = min(100, spd + 10)
    elif ch in ('-', '_'):
        state["speed"] = max(10, spd - 10)
    elif ch == 'l':
        state["a"] = 0
        state["b"] = 0
        state["c"] = 0
        state["d"] = 0
        state["_land"] = True
    elif ch == 'x':
        state["a"] = 0
        state["b"] = 0
        state["c"] = 0
        state["d"] = 0
        state["_emergency"] = True


def _update_key_release(state: dict, timers: dict, now: float, timeout: float) -> None:
    """Zero out axes whose keys haven't been pressed recently.

    最近キーが押されていない軸をゼロにする
    """
    # Check roll (a axis)
    a_plus = timers.get("a+", 0)
    a_minus = timers.get("a-", 0)
    if now - max(a_plus, a_minus) > timeout:
        state["a"] = 0

    # Check pitch (b axis)
    b_plus = timers.get("b+", 0)
    b_minus = timers.get("b-", 0)
    if now - max(b_plus, b_minus) > timeout:
        state["b"] = 0

    # Check throttle (c axis)
    c_plus = timers.get("c+", 0)
    c_minus = timers.get("c-", 0)
    if now - max(c_plus, c_minus) > timeout:
        state["c"] = 0

    # Check yaw (d axis)
    d_plus = timers.get("d+", 0)
    d_minus = timers.get("d-", 0)
    if now - max(d_plus, d_minus) > timeout:
        state["d"] = 0


def _display_status(state: dict, start_time: float) -> None:
    """Display real-time status line (overwrites with \\r).

    リアルタイムステータス行を表示（\\rで上書き）
    """
    a, b, c, d = state["a"], state["b"], state["c"], state["d"]
    spd = state["speed"]
    elapsed = time.monotonic() - start_time

    line = f"  [RC] R={a:+4d} P={b:+4d} T={c:+4d} Y={d:+4d} spd={spd:3d} | {elapsed:.1f}s"

    # Pad to clear previous line
    # 前の行を消すためにパディング
    pad = max(0, state["last_line_len"] - len(line))
    sys.stdout.write(f"\r{line}{' ' * pad}")
    sys.stdout.flush()
    state["last_line_len"] = len(line)
