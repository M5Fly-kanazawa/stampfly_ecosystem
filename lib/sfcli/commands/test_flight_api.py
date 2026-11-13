#!/usr/bin/env python3
"""
test_flight_api.py - tests for the UDP API of `sf takeoff/land/up/.../emergency/stop`, `sf rc` and
`sf battery/height/...` (lib/sfcli/utils/vehicle_api.py, commands/flight.py, rc.py, query.py).
A fake vehicle listens on 127.0.0.1 and speaks the Tello-style text API of
firmware/vehicle/tasks/api_task.cpp; no real vehicle and no other address is used.
`sf takeoff/land/up/.../emergency/stop`・`sf rc`・`sf battery/height/...` の UDP API の試験。
偽の機体が 127.0.0.1 で api_task.cpp の Tello 風テキスト API を話す。実機も他のアドレスも使わない。

Usage:
    python -m pytest lib/sfcli/commands/test_flight_api.py -q
"""

from __future__ import annotations

import math
import re
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sfcli.cli import main  # noqa: E402
from sfcli.commands import flight  # noqa: E402
from sfcli.utils import vehicle_api  # noqa: E402

HOST = "127.0.0.1"


class FakeVehicle:
    """Fake vehicle: answers each datagram at its sender, records what it received.

    偽の機体: 各データグラムを送信元へ返信し、受け取ったものを記録する。

    replies: command text -> reply text, or None for "never answers" (e.g. `rc`), or a float
             delay with the text as (delay_s, text).
    replies: コマンド文字列 -> 応答文字列。None は「応答しない」（`rc` など）。(遅れ秒, 文字列) も可。
    """

    def __init__(self, replies: dict):
        self.replies = {"command": "ok", "emergency": "ok", "stop": "ok", **replies}
        self.received: list[str] = []
        self._pending_takeoff_sender = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((HOST, 0))
        self._sock.settimeout(0.05)
        self.port = self._sock.getsockname()[1]
        self._running = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self) -> None:
        while self._running:
            try:
                data, sender = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                return
            text = data.decode().strip()
            self.received.append(text)
            reply = self.replies.get(text, "error unknown command")
            self._answer(text, sender, reply)

    def _answer(self, text: str, sender, reply) -> None:
        if reply is None:
            return
        if isinstance(reply, tuple):
            delay_s, reply_text = reply
            # A waiting command: an `emergency` / `stop` ends it with an error reply, as the
            # vehicle's waitUntil does (api_task.cpp).
            # 待機中のコマンド: `emergency` / `stop` はエラー応答でそれを終わらせる（機体の waitUntil と同じ）。
            self._pending_takeoff_sender = (sender, text)
            threading.Timer(delay_s, self._send_if_pending, (sender, reply_text)).start()
            return
        if text in ("emergency", "stop") and self._pending_takeoff_sender is not None:
            waiting_sender, _ = self._pending_takeoff_sender
            self._pending_takeoff_sender = None
            self._sock.sendto(f"error interrupted by {text}".encode(), waiting_sender)
        self._sock.sendto(reply.encode(), sender)

    def _send_if_pending(self, sender, reply_text: str) -> None:
        if self._pending_takeoff_sender is not None and self._pending_takeoff_sender[0] == sender:
            self._pending_takeoff_sender = None
            try:
                self._sock.sendto(reply_text.encode(), sender)
            except OSError:
                pass

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._sock.close()


@pytest.fixture
def vehicle(monkeypatch):
    created: list[FakeVehicle] = []

    def make(replies: dict) -> FakeVehicle:
        fake = FakeVehicle(replies)
        monkeypatch.setattr(vehicle_api, "API_PORT", fake.port)
        created.append(fake)
        return fake

    yield make
    for fake in created:
        fake.close()


def _output(capsys) -> str:
    """Everything printed so far (stdout and stderr) / ここまでに表示した全て（標準出力と標準エラー）"""
    captured = capsys.readouterr()
    return captured.out + captured.err


def run(*argv: str) -> int:
    return main([*argv, "--ip", HOST])


# --------------------------------------------------------------------------
# Every verb sends the expected text / 各 verb が期待どおりの文字列を送る
# --------------------------------------------------------------------------

@pytest.mark.parametrize(
    "argv, expected",
    [
        (["takeoff"], "takeoff"),
        (["land"], "land"),
        (["forward", "50"], "forward 50"),
        (["back", "50"], "back 50"),
        (["left", "30"], "left 30"),
        (["right", "30"], "right 30"),
        (["cw", "90"], "cw 90"),
        (["ccw", "45"], "ccw 45"),
    ],
)
def test_verb_sends_expected_text_after_command(vehicle, argv, expected):
    fake = vehicle({expected: "ok"})
    assert run(*argv) == 0
    assert fake.received == ["command", expected]


def test_up_and_down_check_height_first(vehicle):
    fake = vehicle({"height?": "50", "up 40": "ok", "down 20": "ok"})
    assert run("up", "40") == 0
    assert run("down", "20") == 0
    assert fake.received == ["command", "height?", "command", "up 40",
                             "command", "height?", "command", "down 20"]


def test_emergency_and_stop_are_sent_without_command_handshake(vehicle):
    fake = vehicle({})
    assert run("emergency") == 0
    assert run("stop") == 0
    assert fake.received == ["emergency", "stop"]


@pytest.mark.parametrize(
    "query, reply",
    [
        ("battery", "87"), ("height", "50"), ("tof", "48"), ("baro", "170.07"),
        ("attitude", "pitch:1;roll:2;yaw:3;"), ("speed", "30"),
        ("acceleration", "agx:-13.00;agy:-5.00;agz:-998.00;"),
    ],
)
def test_query_sends_question_mark_form(vehicle, query, reply, capsys):
    fake = vehicle({f"{query}?": reply})
    assert run(query) == 0
    assert fake.received == ["command", f"{query}?"]
    if query == "attitude":
        assert "pitch=1 roll=2 yaw=3" in _output(capsys)


def test_rc_one_shot_sends_rc_line_and_expects_no_reply(vehicle):
    fake = vehicle({"rc 0 50 0 -20": None})
    assert main(["rc", "0", "50", "0", "-20", "--ip", HOST]) == 0
    time.sleep(0.1)
    assert fake.received == ["command", "rc 0 50 0 -20"]


# --------------------------------------------------------------------------
# ok / error / timeout paths / ok・error・timeout の経路
# --------------------------------------------------------------------------

def test_error_reply_is_printed_verbatim_and_exits_1(vehicle, capsys):
    vehicle({"takeoff": "error calibrating - keep the craft still"})
    assert run("takeoff") == 1
    assert "error calibrating - keep the craft still" in _output(capsys)


def test_no_reply_is_timeout_exit_3_and_distinct_from_error(vehicle, capsys):
    vehicle({"takeoff": None})
    assert run("takeoff", "--timeout", "0.3") == 3
    out = _output(capsys)
    assert "no reply" in out and "timeout" in out


def test_no_reply_to_command_stops_before_sending_the_verb(vehicle):
    fake = vehicle({"command": None})
    assert run("land", "--timeout", "0.3") == 3
    assert fake.received == ["command"]   # the verb itself was not sent / 本体は送っていない


def test_query_error_and_timeout(vehicle, monkeypatch, capsys):
    vehicle({"baro?": "error unknown query", "tof?": None})
    assert run("baro") == 1
    assert "error unknown query" in _output(capsys)
    # An unanswered query waits the immediate window; narrow it to keep the test fast.
    # 応答のない問い合わせは即答用の窓を待つ。試験を速くするため狭める。
    monkeypatch.setattr(vehicle_api, "IMMEDIATE_TIMEOUT_S", 0.3)
    assert run("tof") == 3


# --------------------------------------------------------------------------
# Range rejection on the PC, nothing sent / PC 側の範囲拒否、何も送らない
# --------------------------------------------------------------------------

@pytest.mark.parametrize(
    "argv",
    [
        ["forward", "9"], ["forward", "301"], ["back", "0"], ["left", "-20"], ["right", "400"],
        ["up", "9"], ["down", "301"], ["cw", "0"], ["cw", "361"], ["ccw", "-5"],
    ],
)
def test_out_of_range_is_rejected_without_sending(vehicle, argv, capsys):
    fake = vehicle({})
    assert run(*argv) == 1
    time.sleep(0.1)
    assert fake.received == []
    assert _output(capsys) != ""


def test_up_above_ceiling_is_rejected_after_asking_height(vehicle, capsys):
    fake = vehicle({"height?": "100"})
    assert run("up", "50") == 1   # 100 + 50 > 120 cm ceiling / 120 cm の上限超え
    assert fake.received == ["command", "height?"]
    assert "ceiling" in _output(capsys)


def test_down_below_floor_is_rejected(vehicle, capsys):
    fake = vehicle({"height?": "50"})
    assert run("down", "40") == 1   # 50 - 40 < 20 cm
    assert fake.received == ["command", "height?"]


def test_up_with_unreadable_height_is_refused(vehicle):
    fake = vehicle({"height?": "error x"})
    assert run("up", "20") == 1
    assert "up 20" not in fake.received


def test_ceiling_in_python_matches_firmware_header():
    header = Path(__file__).resolve().parents[3] / (
        "firmware/vehicle/components/sf_core/include/flight_limits.hpp")
    text = header.read_text(encoding="utf-8")
    assert "kAltitudeCeilingM = 1.2f" in text
    assert vehicle_api.ALTITUDE_CEILING_CM == 120


def test_rotate_timeout_in_python_matches_firmware_headers():
    # Ties rotate_timeout_s to the firmware: yaw rate limit (flight_limits.hpp) and the wait margin
    # (rotate_guidance.hpp). / rotate_timeout_s をファームの定数に結びつける。
    root = Path(__file__).resolve().parents[3] / "firmware/vehicle/components"
    limits = (root / "sf_core/include/flight_limits.hpp").read_text(encoding="utf-8")
    guidance = (root / "sf_command/include/rotate_guidance.hpp").read_text(encoding="utf-8")
    rate = re.search(r"kGuidanceYawRateMaxRadPerSec\s*=\s*([0-9.]+)f", limits)
    margin = re.search(r"kRotateMarginMs\s*=\s*([0-9]+)", guidance)
    assert rate and margin
    assert vehicle_api.YAW_RATE_LIMIT_DEG_PER_S == pytest.approx(math.degrees(float(rate.group(1))))
    assert vehicle_api.ROTATE_EXTRA_S == pytest.approx(int(margin.group(1)) / 1000.0)
    firmware_ms = 360.0 * math.pi / 180.0 / float(rate.group(1)) * 1000.0 + int(margin.group(1))
    assert vehicle_api.rotate_timeout_s(360) == pytest.approx(
        firmware_ms / 1000.0 + vehicle_api.NETWORK_MARGIN_S)


def test_rc_value_out_of_range_is_rejected(vehicle):
    fake = vehicle({})
    assert main(["rc", "0", "101", "0", "0", "--ip", HOST]) == 1
    assert fake.received == []


def test_takeoff_takes_no_altitude_argument():
    with pytest.raises(SystemExit):
        main(["takeoff", "0.5"])


def test_hover_and_jump_do_not_exist():
    for verb in ("hover", "jump"):
        with pytest.raises(SystemExit):
            main([verb])


# --------------------------------------------------------------------------
# emergency is never blocked / emergency は待たされない
# --------------------------------------------------------------------------

def test_emergency_is_not_blocked_by_a_waiting_takeoff(vehicle):
    fake = vehicle({"takeoff": (5.0, "ok")})
    results: dict = {}

    def waiting_takeoff() -> None:
        results["takeoff"] = run("takeoff", "--timeout", "10")

    thread = threading.Thread(target=waiting_takeoff)
    thread.start()
    deadline = time.monotonic() + 2.0
    while "takeoff" not in fake.received and time.monotonic() < deadline:
        time.sleep(0.01)
    assert "takeoff" in fake.received

    started = time.monotonic()
    assert run("emergency") == 0
    assert time.monotonic() - started < 1.0   # not queued behind the 5 s wait / 5 秒の待ちの後ろに並ばない
    thread.join(timeout=3.0)
    assert results["takeoff"] == 1            # the vehicle ended it: error interrupted by emergency
    assert fake.received.count("command") == 1   # emergency sent no handshake / emergency は手順なし


def test_stop_is_not_blocked_by_a_waiting_move(vehicle):
    fake = vehicle({"forward 100": (5.0, "ok")})
    thread = threading.Thread(target=lambda: run("forward", "100", "--timeout", "10"))
    thread.start()
    deadline = time.monotonic() + 2.0
    while "forward 100" not in fake.received and time.monotonic() < deadline:
        time.sleep(0.01)
    started = time.monotonic()
    assert run("stop") == 0
    assert time.monotonic() - started < 1.0
    thread.join(timeout=3.0)


# --------------------------------------------------------------------------
# Ctrl-C policy / Ctrl-C の方針
# --------------------------------------------------------------------------

def _interrupt_when_sending(monkeypatch, command: str) -> None:
    """Make the wait for `command` raise KeyboardInterrupt, as Ctrl-C does.

    `command` の応答待ちで KeyboardInterrupt を起こす（Ctrl-C と同じ）。
    """
    original = vehicle_api.VehicleApi._exchange

    def exchange(self, sock, text, timeout_s):
        if text == command:
            raise KeyboardInterrupt
        return original(self, sock, text, timeout_s)

    monkeypatch.setattr(vehicle_api.VehicleApi, "_exchange", exchange)


@pytest.mark.parametrize("argv, command", [
    (["takeoff"], "takeoff"), (["forward", "100"], "forward 100"), (["cw", "90"], "cw 90"),
])
def test_ctrl_c_on_takeoff_move_rotate_sends_stop_never_emergency(
        vehicle, monkeypatch, argv, command):
    fake = vehicle({command: "ok"})
    _interrupt_when_sending(monkeypatch, command)
    assert run(*argv) == 130
    assert "stop" in fake.received
    assert "emergency" not in fake.received


def test_ctrl_c_on_land_sends_nothing_more(vehicle, monkeypatch):
    fake = vehicle({"land": "ok"})
    _interrupt_when_sending(monkeypatch, "land")
    assert run("land") == 130
    assert fake.received == ["command"]


def test_classify_reply():
    assert vehicle_api.classify_reply("ok") == vehicle_api.KIND_OK
    assert vehicle_api.classify_reply("ok kp=1.0") == vehicle_api.KIND_OK
    assert vehicle_api.classify_reply("error out of range") == vehicle_api.KIND_ERROR
    assert vehicle_api.classify_reply("87") == vehicle_api.KIND_VALUE


def test_timeouts_follow_the_firmware_timeouts():
    assert vehicle_api.TAKEOFF_TIMEOUT_S > 12.0     # kTakeoffTimeoutMs
    assert vehicle_api.LAND_TIMEOUT_S > 12.0 + 20.0  # kTakeoffTimeoutMs + kLandTimeoutMs
    # rotateTimeoutMs(angle) = angle / 1 rad/s + 6000 ms: 90 deg = 7571 ms, 360 deg = 12283 ms
    # rotateTimeoutMs(角度) = 角度 / 1 rad/s + 6000 ms: 90 度 = 7571 ms、360 度 = 12283 ms
    assert vehicle_api.rotate_timeout_s(90) > 7.571
    assert vehicle_api.rotate_timeout_s(360) > 12.283
    assert vehicle_api.rotate_timeout_s(360) > vehicle_api.rotate_timeout_s(90) + 4.7
    assert flight.move_timeout_s(300) > 300 / 10.0 + 6.0
