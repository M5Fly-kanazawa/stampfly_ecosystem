"""
sf motor test / sweep / stop
Bench motor commands (CW/CCW current-asymmetry diagnostics)

Send bench motor test/sweep commands to StampFly via WiFi CLI (TCP:23) and
print the vehicle's response verbatim — the vehicle formats the table.
WiFi CLI (TCP:23) 経由でベンチ用モータコマンドを送信し、機体の応答をそのまま表示する
（表の整形は機体側が行う）。

Subcommands:
    test <1-4> <duty%>  - Spin one motor at a fixed duty (~2s, disarmed only)
    sweep [--duty --sec] - CW/CCW current-asymmetry sweep across M1..M4
    stop                 - Stop an active motor test immediately

Both subcommands are DISARMED-only bench diagnostics (the vehicle firmware
refuses/ignores them while armed) — see firmware/vehicle/tasks/cli_task.cpp
cmd_motor / cmd_motor_sweep.
"""

import argparse
import asyncio

from ..utils import console
from ..utils.vehicle_connection import VehicleCLI, DEFAULT_HOST

COMMAND_NAME = "motor"
COMMAND_HELP = "Bench motor test / CW-CCW current sweep (disarmed only)"

# Mirror config.hpp's MOTOR_SWEEP_MIN/MAX so an out-of-range value is caught
# client-side before opening a connection; the vehicle clamps independently
# (this is a convenience check, not the source of truth — see config.hpp).
# config.hpp の MOTOR_SWEEP_MIN/MAX を写す（接続前にクライアント側で弾く利便性
# チェック。機体側も独立にクランプする — source of truth は config.hpp）。
SWEEP_DUTY_MIN, SWEEP_DUTY_MAX = 5, 80
SWEEP_SEC_MIN, SWEEP_SEC_MAX = 1.0, 10.0
SWEEP_DEFAULT_DUTY, SWEEP_DEFAULT_SEC = 40, 3.0

# Per-motor baseline + rest budget the firmware bench sweep spends outside the
# spin time itself (config.hpp MOTOR_SWEEP_BASELINE_US / MOTOR_SWEEP_REST_US),
# used only to size our WiFi read timeout with margin.
# ファーム側スイープが回転時間以外に使うベースライン＋休止時間の見積り
# （config.hpp 参照）。WiFi 応答待ちタイムアウトの見積りにのみ使う。
_SWEEP_BASELINE_S = 1.0
_SWEEP_REST_S = 1.0
_SWEEP_TIMEOUT_MARGIN_S = 10.0


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register the motor command / motor コマンドを登録"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(
        dest="motor_command", title="subcommands", metavar="<subcommand>"
    )

    test_parser = sub.add_parser(
        "test",
        help="Spin one motor at a fixed duty (~2s, disarmed only)",
        description="Bench wiring/direction check for one motor. "
                    "IDs: M1=FR M2=RR M3=RL M4=FL. KEEP PROPS OFF or hold the craft.",
    )
    test_parser.add_argument("motor_id", type=int, help="Motor ID 1-4")
    test_parser.add_argument("duty", type=int, help="Duty percent 0-100")
    _add_common_args(test_parser)
    test_parser.set_defaults(func=run_test)

    sweep_parser = sub.add_parser(
        "sweep",
        help="CW/CCW current-asymmetry sweep across M1..M4",
        description="Spin M1..M4 in turn at a fixed duty, sample battery current/"
                    "voltage per motor plus an all-off baseline, and print a "
                    "comparison table. Run with props OFF first (isolates motor/ESC "
                    "asymmetry), then with props ON (adds aero load) to see whether "
                    "the CW or CCW group draws more current. DISARMED ONLY.",
    )
    sweep_parser.add_argument(
        "--duty", type=int, default=SWEEP_DEFAULT_DUTY,
        help=f"Duty percent per motor ({SWEEP_DUTY_MIN}-{SWEEP_DUTY_MAX}, "
             f"default {SWEEP_DEFAULT_DUTY})",
    )
    sweep_parser.add_argument(
        "--sec", type=float, default=SWEEP_DEFAULT_SEC,
        help=f"Seconds per motor ({SWEEP_SEC_MIN:.0f}-{SWEEP_SEC_MAX:.0f}, "
             f"default {SWEEP_DEFAULT_SEC:.1f})",
    )
    _add_common_args(sweep_parser)
    sweep_parser.set_defaults(func=run_sweep)

    stop_parser = sub.add_parser(
        "stop",
        help="Stop an active motor test immediately",
    )
    _add_common_args(stop_parser)
    stop_parser.set_defaults(func=run_stop)

    parser.set_defaults(func=run_help)


def _add_common_args(parser: argparse.ArgumentParser) -> None:
    """Add the shared --ip option / 共通の --ip オプションを追加"""
    parser.add_argument(
        "--ip",
        default=DEFAULT_HOST,
        help=f"Vehicle IP address (default: {DEFAULT_HOST})",
    )


def run_help(args: argparse.Namespace) -> int:
    """Show help when no subcommand is given / サブコマンド無しでヘルプ表示"""
    console.print("Usage: sf motor <subcommand> [options]")
    console.print()
    console.print("Subcommands:")
    console.print("  test <1-4> <duty%>   Spin one motor (~2s, disarmed only)")
    console.print("  sweep [--duty --sec] CW/CCW current-asymmetry sweep")
    console.print("  stop                 Stop an active motor test")
    console.print()
    console.print("Examples:")
    console.print("  sf motor test 2 30            # spin M2 (RR) at 30%")
    console.print("  sf motor sweep                # default 40%, 3s/motor")
    console.print("  sf motor sweep --duty 50 --sec 4")
    return 0


def run_test(args: argparse.Namespace) -> int:
    """Execute `motor test <id> <duty>` / `motor test` を実行"""
    if not 1 <= args.motor_id <= 4:
        console.error("Motor ID must be 1-4 (M1=FR M2=RR M3=RL M4=FL)")
        return 1
    if not 0 <= args.duty <= 100:
        console.error("Duty must be 0-100 [%]")
        return 1
    console.info(f"M{args.motor_id} @ {args.duty}% for ~2s (props off or hold the craft)",
                 prefix="MOTOR")
    return _run_command(f"motor test {args.motor_id} {args.duty}", args.ip, timeout=5.0)


def run_sweep(args: argparse.Namespace) -> int:
    """Execute `motor sweep <duty> <sec>` / `motor sweep` を実行"""
    if not SWEEP_DUTY_MIN <= args.duty <= SWEEP_DUTY_MAX:
        console.error(f"Duty must be {SWEEP_DUTY_MIN}-{SWEEP_DUTY_MAX} [%]")
        return 1
    if not SWEEP_SEC_MIN <= args.sec <= SWEEP_SEC_MAX:
        console.error(f"Seconds per motor must be {SWEEP_SEC_MIN:.0f}-{SWEEP_SEC_MAX:.0f}")
        return 1

    total_s = _SWEEP_BASELINE_S + 4.0 * (args.sec + _SWEEP_REST_S)
    timeout = total_s + _SWEEP_TIMEOUT_MARGIN_S
    console.info(f"sweep: duty={args.duty}% sec/motor={args.sec:.1f} "
                 f"(~{total_s:.0f}s on the vehicle) — props off or hold the craft",
                 prefix="MOTOR")
    return _run_command(f"motor sweep {args.duty} {args.sec}", args.ip, timeout=timeout)


def run_stop(args: argparse.Namespace) -> int:
    """Execute `motor stop` / `motor stop` を実行"""
    console.info("stopping active motor test", prefix="MOTOR")
    return _run_command("motor stop", args.ip, timeout=5.0)


# =============================================================================
# WiFi CLI plumbing — connect, send one command, print the raw reply, disconnect.
# WiFi CLI 疎通 — 接続・コマンド送信・応答をそのまま表示・切断。
# =============================================================================

def _run_command(cli_cmd: str, ip: str, timeout: float) -> int:
    """Send `cli_cmd` over the WiFi CLI and print the response.
    WiFi CLI 経由で `cli_cmd` を送信し応答を表示する。"""
    try:
        return asyncio.run(_async_run_command(cli_cmd, ip, timeout))
    except KeyboardInterrupt:
        return 130


async def _async_run_command(cli_cmd: str, ip: str, timeout: float) -> int:
    """Connect -> send -> print -> disconnect (single command, no monitoring).
    接続 -> 送信 -> 表示 -> 切断（単発コマンド、モニタリングなし）。"""
    cli = VehicleCLI()
    try:
        try:
            await cli.connect(ip, timeout=5.0)
        except OSError:
            console.error("Vehicle not reachable. Connect to StampFly WiFi AP first.")
            return 1

        response = await cli.send_command(cli_cmd, timeout=timeout)
        if response:
            console.print(response)

        await cli.disconnect()
        if "refused" in response.lower() or "usage:" in response.lower():
            return 1
        return 0

    except ConnectionError as e:
        console.error(f"Connection lost: {e}")
        return 1
    except Exception as e:  # noqa: BLE001 - surface any transport error to the user
        console.error(f"Command failed: {e}")
        try:
            await cli.disconnect()
        except Exception:
            pass
        return 1
