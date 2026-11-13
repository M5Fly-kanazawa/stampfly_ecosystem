"""
sf battery / height / tof / baro / attitude / acceleration / speed
Telemetry query commands
状態の問い合わせコマンド

Each command sends one Tello-style query (`battery?` ...) to the vehicle on UDP port 8889 and
prints the vehicle's answer (lib/sfcli/utils/vehicle_api.py). The units are those of the
vehicle's replies (firmware/vehicle/tasks/api_task.cpp, cmdQuery).
各コマンドは Tello 風の問い合わせ（`battery?` など）を機体の UDP 8889 へ 1 つ送り、機体の答えを
表示する（lib/sfcli/utils/vehicle_api.py）。単位は機体の応答のもの（api_task.cpp の cmdQuery）。

Commands:
    battery       - Battery level (%)               reply: "87"
    height        - Estimated height (cm)           reply: "50"
    tof           - Downward ToF distance (cm)      reply: "10"
    baro          - Barometric altitude (m)         reply: "170.07"
    attitude      - Pitch, roll, yaw (deg)          reply: "pitch:0;roll:0;yaw:0;"
    acceleration  - IMU acceleration (0.001 g)      reply: "agx:-13.00;agy:-5.00;agz:-998.00;"
    speed         - Cruise speed setting (cm/s)     reply: "30"

Exit status / 終了コード: 0 answered, 1 the vehicle replied "error ...", 3 no reply (timeout).
0 応答あり、1 機体が "error ..." を返した、3 応答なし（タイムアウト）。
"""

import argparse
import re

from ..utils import console
from ..utils.vehicle_api import DEFAULT_HOST, KIND_TIMEOUT, VehicleApi

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_TIMEOUT = 3


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register query commands with CLI

    クエリコマンドをCLIに登録する
    """
    # (name, help, API query, formatter of the reply)
    # （名前、ヘルプ、API の問い合わせ、応答の整形関数）
    queries = [
        ("battery", "Query battery level (%%)", "battery?", _fmt_battery),
        ("height", "Query estimated height (cm)", "height?", _fmt_height),
        ("tof", "Query downward ToF distance (cm)", "tof?", _fmt_tof),
        ("baro", "Query barometric altitude (m)", "baro?", _fmt_baro),
        ("attitude", "Query attitude angles (deg)", "attitude?", _fmt_attitude),
        ("acceleration", "Query acceleration (0.001 g)", "acceleration?", _fmt_acceleration),
        ("speed", "Query cruise speed setting (cm/s)", "speed?", _fmt_speed),
    ]

    for name, help_text, query, fmt_func in queries:
        parser = subparsers.add_parser(
            name,
            help=help_text,
            description=f"{help_text}. UDP 8889 経由で機体に問い合わせる。",
        )
        parser.add_argument(
            "--ip",
            default=DEFAULT_HOST,
            help=f"Vehicle IP address (default: {DEFAULT_HOST})",
        )
        parser.set_defaults(
            func=lambda args, q=query, f=fmt_func: _run_query(args, q, f)
        )


# =============================================================================
# Format Functions / 整形関数
# =============================================================================

def _fmt_battery(raw: str) -> str:
    return f"Battery: {raw.strip()} %"


def _fmt_height(raw: str) -> str:
    return f"Height: {raw.strip()} cm"


def _fmt_tof(raw: str) -> str:
    return f"ToF: {raw.strip()} cm"


def _fmt_baro(raw: str) -> str:
    return f"Baro: {raw.strip()} m"


def _fields(raw: str) -> dict:
    """Split "a:1;b:2;" into {"a": "1", "b": "2"} / "a:1;b:2;" を辞書に分ける"""
    return dict(re.findall(r"(\w+):([^;]*)", raw))


def _fmt_attitude(raw: str) -> str:
    f = _fields(raw)
    if {"pitch", "roll", "yaw"} <= f.keys():
        return f"Attitude: pitch={f['pitch']} roll={f['roll']} yaw={f['yaw']} deg"
    return f"Attitude: {raw.strip()}"


def _fmt_acceleration(raw: str) -> str:
    f = _fields(raw)
    if {"agx", "agy", "agz"} <= f.keys():
        return f"Acceleration: x={f['agx']} y={f['agy']} z={f['agz']} (0.001 g)"
    return f"Acceleration: {raw.strip()}"


def _fmt_speed(raw: str) -> str:
    return f"Speed setting: {raw.strip()} cm/s"


# =============================================================================
# Query Execution / 問い合わせの実行
# =============================================================================

def _run_query(args: argparse.Namespace, query: str, fmt_func) -> int:
    """Send one query, print the answer or the vehicle's reply verbatim on error.

    問い合わせを 1 つ送り、答えを表示する。エラー時は機体の応答をそのまま表示する
    """
    api = VehicleApi(args.ip)
    try:
        result = api.send(query)
    except OSError as error:
        console.error(f"{query}: cannot send to {args.ip}:{api.port} ({error})")
        return EXIT_ERROR
    if result.kind == KIND_TIMEOUT:
        console.error(f"{query}: no reply from the vehicle (timeout)")
        return EXIT_TIMEOUT
    if not result.succeeded:
        console.error(f"{query}: {result.text}")
        return EXIT_ERROR
    console.success(fmt_func(result.text))
    return EXIT_OK
