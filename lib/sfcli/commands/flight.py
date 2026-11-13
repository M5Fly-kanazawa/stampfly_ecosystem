"""
sf takeoff / land / up / down / cw / ccw / forward / back / left / right / emergency / stop
Flight commands
飛行コマンド

Each verb sends one Tello-style text command to the vehicle on UDP port 8889 and waits for the
vehicle's reply (see lib/sfcli/utils/vehicle_api.py and docs/architecture/tello-api-reference.md).
各 verb は Tello 風のテキストコマンドを機体の UDP 8889 へ 1 つ送り、機体の応答を待つ
（lib/sfcli/utils/vehicle_api.py と docs/architecture/tello-api-reference.md 参照）。

Commands:
    takeoff             - Auto-takeoff to the fixed takeoff altitude (0.5 m)
    land                - Land
    up <cm>             - Move up (10-300 cm, stays below the 120 cm ceiling)
    down <cm>           - Move down (10-300 cm, stays above 20 cm)
    forward <cm>        - Move forward (10-300 cm)
    back <cm>           - Move backward (10-300 cm)
    left <cm>           - Move left (10-300 cm)
    right <cm>          - Move right (10-300 cm)
    cw <deg>            - Rotate clockwise (1-360 deg)
    ccw <deg>           - Rotate counter-clockwise (1-360 deg)
    stop                - Hold the current position (also interrupts a waiting command)
    emergency           - Cut the motors

Exit status / 終了コード:
    0 ok, 1 the vehicle replied "error ..." or the arguments were rejected,
    3 no reply (timeout), 130 interrupted with Ctrl-C.
    0 成功、1 機体が "error ..." を返した・引数を拒否した、3 応答なし（タイムアウト）、
    130 Ctrl-C で中断。

Ctrl-C while waiting for a reply / 応答待ちの間の Ctrl-C:
    takeoff, up, down, forward, back, left, right, cw, ccw: send `stop` and say so.
        `stop` holds the position (a move or rotation ends at once); it is refused while the
        takeoff is still climbing, which then goes on, and the message says how to land.
    land: send nothing; the landing is already what the vehicle is doing.
    `emergency` is never sent automatically: cutting the motors in the air is the operator's
    decision.
    takeoff・移動・回頭: `stop` を送り、そのことを表示する。`stop` は位置を保持する（移動・回頭は
    その場で終わる）。離陸の上昇中は拒否され、離陸は続く。表示で着陸のしかたを案内する。
    land: 何も送らない（機体は既に着陸している）。
    `emergency` は自動では送らない: 空中でモータを止めるかは操縦者の判断。
"""

import argparse
from typing import Callable, Optional

from ..utils import console
from ..utils.vehicle_api import (
    ALTITUDE_CEILING_CM,
    ALTITUDE_FLOOR_CM,
    DEFAULT_HOST,
    KIND_TIMEOUT,
    LAND_TIMEOUT_S,
    MOVE_MAX_CM,
    MOVE_MIN_CM,
    ROTATE_MAX_DEG,
    ROTATE_MIN_DEG,
    TAKEOFF_TIMEOUT_S,
    ApiResult,
    VehicleApi,
    move_timeout_s,
    rotate_timeout_s,
)

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_TIMEOUT = 3
EXIT_INTERRUPTED = 130

# Moves: verb -> help text / 移動: verb -> ヘルプ文
_MOVE_VERBS = {
    "up": "Move up",
    "down": "Move down",
    "forward": "Move forward",
    "back": "Move backward",
    "left": "Move left",
    "right": "Move right",
}
_ROTATE_VERBS = {
    "cw": "Rotate clockwise",
    "ccw": "Rotate counter-clockwise",
}


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register flight commands with CLI

    フライトコマンドをCLIに登録する
    """
    takeoff_parser = subparsers.add_parser(
        "takeoff",
        help="Auto-takeoff (rises to the fixed 0.5 m takeoff altitude)",
        description="Auto-takeoff. The vehicle climbs to its fixed takeoff altitude (0.5 m) and "
                    "replies when it is flying. 自動離陸。機体は固定の離陸高度（0.5 m）まで上昇し、"
                    "飛行状態になったら応答する。",
    )
    _add_common_args(takeoff_parser)
    takeoff_parser.set_defaults(func=run_takeoff)

    land_parser = subparsers.add_parser(
        "land",
        help="Land",
        description="Land. The vehicle replies when it is on the ground. "
                    "着陸。機体は地上に降りたら応答する。",
    )
    _add_common_args(land_parser)
    land_parser.set_defaults(func=run_land)

    for verb, help_text in _MOVE_VERBS.items():
        parser = subparsers.add_parser(
            verb,
            help=f"{help_text} (cm)",
            description=f"{help_text} by <distance> cm ({MOVE_MIN_CM}-{MOVE_MAX_CM}). "
                        "The vehicle replies when it has arrived. 到達したら応答する。",
        )
        parser.add_argument("distance", type=int,
                            help=f"Distance in cm ({MOVE_MIN_CM}-{MOVE_MAX_CM})")
        _add_common_args(parser)
        parser.set_defaults(func=_make_move_runner(verb))

    for verb, help_text in _ROTATE_VERBS.items():
        parser = subparsers.add_parser(
            verb,
            help=f"{help_text} (degrees)",
            description=f"{help_text} by <angle> degrees ({ROTATE_MIN_DEG}-{ROTATE_MAX_DEG}). "
                        "The vehicle replies when it has turned. 回り終えたら応答する。",
        )
        parser.add_argument("angle", type=int,
                            help=f"Angle in degrees ({ROTATE_MIN_DEG}-{ROTATE_MAX_DEG})")
        _add_common_args(parser)
        parser.set_defaults(func=_make_rotate_runner(verb))

    stop_parser = subparsers.add_parser(
        "stop",
        help="Hold the current position",
        description="Hold the current position; a command another terminal is waiting for ends "
                    "at once. Sent without waiting behind anything. "
                    "現在位置を保持する。他の端末が待っているコマンドはその場で終わる。"
                    "何の後ろにも待たずに送る。",
    )
    _add_common_args(stop_parser, with_timeout=False)
    stop_parser.set_defaults(func=run_stop)

    emergency_parser = subparsers.add_parser(
        "emergency",
        help="Cut the motors",
        description="Emergency stop: cut all motors at once, in the air as well. Sent without "
                    "waiting behind anything. 緊急停止: 空中でも全モータを即座に止める。"
                    "何の後ろにも待たずに送る。",
    )
    _add_common_args(emergency_parser, with_timeout=False)
    emergency_parser.set_defaults(func=run_emergency)


def _add_common_args(parser: argparse.ArgumentParser, with_timeout: bool = True) -> None:
    """Add common options to a flight command parser.

    フライトコマンドパーサーに共通オプションを追加する
    """
    parser.add_argument(
        "--ip",
        default=DEFAULT_HOST,
        help=f"Vehicle IP address (default: {DEFAULT_HOST})",
    )
    if with_timeout:
        parser.add_argument(
            "--timeout",
            type=float,
            default=None,
            help="Seconds to wait for the reply (default: derived from the vehicle's own timeouts)",
        )


# =============================================================================
# Sending and reporting / 送信と報告
# =============================================================================

def _report(result: ApiResult, label: str) -> int:
    """Print the vehicle's reply and return the exit status.

    機体の応答を表示し、終了コードを返す
    """
    if result.kind == KIND_TIMEOUT:
        console.error(f"{label}: no reply from the vehicle (timeout)")
        return EXIT_TIMEOUT
    if not result.succeeded:
        console.error(f"{label}: {result.text}")
        return EXIT_ERROR
    console.success(f"{label}: {result.text}")
    return EXIT_OK


def _send_waiting(
    args: argparse.Namespace,
    label: str,
    command: str,
    default_timeout_s: float,
    on_interrupt: Optional[Callable[[VehicleApi], None]],
) -> int:
    """Send `command`, wait for the reply, handle Ctrl-C by `on_interrupt`.

    `command` を送り、応答を待ち、Ctrl-C は `on_interrupt` で扱う
    """
    api = VehicleApi(args.ip)
    timeout_s = args.timeout if args.timeout is not None else default_timeout_s
    console.info(f"{command} (waiting up to {timeout_s:.0f} s for the reply)", prefix=label)
    try:
        result = api.send(command, timeout_s)
    except KeyboardInterrupt:
        console.print()
        if on_interrupt is not None:
            on_interrupt(api)
        return EXIT_INTERRUPTED
    except OSError as error:
        console.error(f"{label}: cannot send to {args.ip}:{api.port} ({error})")
        return EXIT_ERROR
    return _report(result, label)


def _stop_on_interrupt(api: VehicleApi) -> None:
    """Ctrl-C policy for takeoff / moves / rotations: send `stop`, report its reply.

    takeoff・移動・回頭での Ctrl-C の方針: `stop` を送り、その応答を表示する
    """
    console.warning("Interrupted: sending stop (hold position)", prefix="STOP")
    try:
        result = api.stop()
    except OSError as error:
        console.error(f"stop could not be sent ({error}). Use sf emergency to cut the motors.")
        return
    if result.kind == KIND_TIMEOUT:
        console.error("stop: no reply from the vehicle (timeout)")
    elif result.succeeded:
        console.success(f"stop: {result.text}")
    else:
        # Refused while the takeoff is still climbing: the vehicle goes on.
        # 離陸の上昇中は拒否される: 機体は続けている。
        console.warning(f"stop: {result.text}")
    console.print("  The vehicle keeps its state. sf land lands it; sf emergency cuts the motors.")


def _land_on_interrupt(api: VehicleApi) -> None:
    """Ctrl-C policy for land: send nothing.

    land での Ctrl-C の方針: 何も送らない
    """
    console.warning("Interrupted: the vehicle keeps landing (nothing was sent). "
                    "sf emergency cuts the motors.", prefix="LAND")


# =============================================================================
# Command Runners / コマンド実行
# =============================================================================

def run_takeoff(args: argparse.Namespace) -> int:
    """takeoff: no argument; the altitude is the vehicle's fixed takeoff altitude."""
    return _send_waiting(args, "TAKEOFF", "takeoff", TAKEOFF_TIMEOUT_S, _stop_on_interrupt)


def run_land(args: argparse.Namespace) -> int:
    """land"""
    return _send_waiting(args, "LAND", "land", LAND_TIMEOUT_S, _land_on_interrupt)


def _make_move_runner(verb: str):
    def run(args: argparse.Namespace) -> int:
        return run_move(args, verb)
    return run


def _make_rotate_runner(verb: str):
    def run(args: argparse.Namespace) -> int:
        return run_rotate(args, verb)
    return run


def check_move_range(distance_cm: int) -> Optional[str]:
    """Reason text if the distance is outside what the vehicle accepts, else None.

    距離が機体の受け付ける範囲外のときその理由文、そうでなければ None
    """
    if distance_cm < MOVE_MIN_CM or distance_cm > MOVE_MAX_CM:
        return (f"Distance must be {MOVE_MIN_CM}-{MOVE_MAX_CM} cm "
                f"(the vehicle refuses less and shortens more)")
    return None


def check_rotate_range(angle_deg: int) -> Optional[str]:
    """Reason text if the angle is outside what the vehicle accepts, else None.

    角度が機体の受け付ける範囲外のときその理由文、そうでなければ None
    """
    if angle_deg < ROTATE_MIN_DEG or angle_deg > ROTATE_MAX_DEG:
        return f"Angle must be {ROTATE_MIN_DEG}-{ROTATE_MAX_DEG} deg"
    return None


def check_vertical_move(verb: str, distance_cm: int, height_cm: int) -> Optional[str]:
    """Reason text if a vertical move from `height_cm` leaves the allowed altitude, else None.

    The vehicle clamps such a move to the ceiling / floor and still replies "ok"; the PC
    refuses it instead so that a shortened move is never reported as done.
    `height_cm` is the vehicle's estimated height (`height?`); the vehicle composes moves on its
    guidance target, which differs from the estimate by the tracking error (a few cm).
    機体はこうした移動を上限・下限へ縮めたうえで "ok" を返す。PC 側では縮んだ移動を「済んだ」と
    報告しないよう、あらかじめ拒否する。`height_cm` は機体の推定高度（`height?`）。機体は移動を
    誘導目標に対して合成するので、推定との差（追従誤差、数 cm）がある。
    """
    if verb == "up" and height_cm + distance_cm > ALTITUDE_CEILING_CM:
        return (f"up {distance_cm} from {height_cm} cm would pass the "
                f"{ALTITUDE_CEILING_CM} cm altitude ceiling")
    if verb == "down" and height_cm - distance_cm < ALTITUDE_FLOOR_CM:
        return (f"down {distance_cm} from {height_cm} cm would go below the "
                f"{ALTITUDE_FLOOR_CM} cm lower limit")
    return None


def _read_height_cm(api: VehicleApi) -> "tuple[Optional[int], Optional[str]]":
    """Ask `height?`; return (height_cm, None) or (None, reason).

    `height?` で問い合わせ、(高度 cm, None) か (None, 理由) を返す
    """
    result = api.send("height?")
    if result.kind == KIND_TIMEOUT:
        return None, "no reply to height? (timeout)"
    if not result.succeeded:
        return None, f"height?: {result.text}"
    try:
        return int(result.text), None
    except ValueError:
        return None, f"height? answered {result.text!r}, not a number of cm"


def run_move(args: argparse.Namespace, verb: str) -> int:
    """up / down / forward / back / left / right"""
    label = verb.upper()
    reason = check_move_range(args.distance)
    if reason is not None:
        console.error(reason)
        return EXIT_ERROR
    if verb in ("up", "down"):
        height_cm, reason = _read_height_cm(VehicleApi(args.ip))
        if reason is None:
            reason = check_vertical_move(verb, args.distance, height_cm)
        if reason is not None:
            console.error(f"{label}: {reason}")
            return EXIT_ERROR
    return _send_waiting(args, label, f"{verb} {args.distance}",
                         move_timeout_s(args.distance), _stop_on_interrupt)


def run_rotate(args: argparse.Namespace, verb: str) -> int:
    """cw / ccw"""
    reason = check_rotate_range(args.angle)
    if reason is not None:
        console.error(reason)
        return EXIT_ERROR
    return _send_waiting(args, verb.upper(), f"{verb} {args.angle}",
                         rotate_timeout_s(args.angle), _stop_on_interrupt)


def run_stop(args: argparse.Namespace) -> int:
    """stop: sent at once, with no handshake / 即座に送る（手順なし）"""
    console.info("stop", prefix="STOP")
    try:
        return _report(VehicleApi(args.ip).stop(), "STOP")
    except OSError as error:
        console.error(f"STOP: cannot send to {args.ip} ({error})")
        return EXIT_ERROR


def run_emergency(args: argparse.Namespace) -> int:
    """emergency: sent at once, with no handshake / 即座に送る（手順なし）"""
    console.warning("EMERGENCY STOP", prefix="EMERGENCY")
    try:
        return _report(VehicleApi(args.ip).emergency(), "EMERGENCY")
    except OSError as error:
        console.error(f"EMERGENCY: cannot send to {args.ip} ({error})")
        return EXIT_ERROR
