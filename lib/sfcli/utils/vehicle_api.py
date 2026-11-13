"""
UDP client for the StampFly Tello-style text API (vehicle UDP port 8889)
StampFly の Tello 風テキスト API（機体の UDP 8889）用クライアント

Every sf command that flies or queries the vehicle goes through this module. The command
grammar, the units and the replies are those of firmware/vehicle/tasks/api_task.cpp; the
reference is docs/architecture/tello-api-reference.md.
機体を飛ばす、または機体に問い合わせる sf コマンドは全てこのモジュールを通る。コマンドの
文法・単位・応答は firmware/vehicle/tasks/api_task.cpp のもので、参照文書は
docs/architecture/tello-api-reference.md。

How a command is sent / コマンドの送り方:
  - One UDP datagram per command, one fresh socket per call. The vehicle answers each datagram
    at its own sender, so a fresh socket receives exactly the reply to its command (a
    long-waiting `takeoff` and a later `emergency` never share a socket).
    1 コマンド = 1 データグラム、呼び出しごとに新しいソケット。機体は各データグラムの送信元へ
    返信するので、新しいソケットはそのコマンドへの返信だけを受け取る（長く待つ `takeoff` と
    後から出した `emergency` がソケットを共有しない）。
  - The vehicle ignores everything except `emergency` (and `stop` during a wait) until it has
    received `command`. `send()` therefore sends `command` first. `emergency` and `stop` are
    sent WITHOUT that handshake: while another command blocks the vehicle, `command` would be
    parked behind it, and the two must never wait.
    機体は `command` を受け取るまで `emergency`（と待機中の `stop`）以外を受け付けない。そこで
    `send()` は先に `command` を送る。`emergency` と `stop` はこの手順なしで送る: 別のコマンドが
    機体を待たせている間、`command` はその後ろに置かれてしまい、この 2 つは決して待たせない。
  - No automatic retry: a flight command is not idempotent (a repeated `up 50` climbs twice).
    自動再送はしない: 飛行コマンドは冪等でない（`up 50` を繰り返すと 2 回上昇する）。
"""

from __future__ import annotations

import socket
import time
from dataclasses import dataclass

# Vehicle address / 機体のアドレス
# The SoftAP is addressed at 192.168.10.1 (the DJI Tello subnet) so stock Tello programs connect.
# SoftAP は 192.168.10.1（DJI Tello のサブネット）で、純正 Tello 用プログラムがそのまま繋がる。
DEFAULT_HOST = "192.168.10.1"
API_PORT = 8889  # kApiPort in api_task.cpp

# ---------------------------------------------------------------------------
# Ranges the firmware accepts (api_task.cpp / api_arguments.hpp / flight_limits.hpp)
# 機体が受け付ける範囲
# ---------------------------------------------------------------------------
MOVE_MIN_CM = 10          # kMoveMinCm: below this the vehicle answers "error out of range"
MOVE_MAX_CM = 300         # kMoveMaxCm: above this the vehicle silently shortens the move
ROTATE_MIN_DEG = 1        # kRotateMinDegrees
ROTATE_MAX_DEG = 360      # kRotateMaxDegrees
ALTITUDE_CEILING_CM = 120  # kAltitudeCeilingM = 1.2 m (flight_limits.hpp)
ALTITUDE_FLOOR_CM = 20     # kAltMinM = 0.2 m (target altitude floor of the moves)
RC_LIMIT = 100            # `rc a b c d`: each -100..100

# ---------------------------------------------------------------------------
# Reply waiting times [s], derived from the firmware's own timeouts in api_task.cpp
# 応答待ち時間 [s]。api_task.cpp 自身のタイムアウトから導く
# ---------------------------------------------------------------------------
NETWORK_MARGIN_S = 3.0          # slack for the WiFi round trip / WiFi 往復の余裕
IMMEDIATE_TIMEOUT_S = 2.0       # commands the vehicle answers at once / 即答するコマンド
TAKEOFF_TIMEOUT_S = 12.0 + NETWORK_MARGIN_S         # kTakeoffTimeoutMs = 12000
# land: up to kTakeoffTimeoutMs to leave TAKEOFF, then kLandTimeoutMs = 20000
# land: TAKEOFF を抜けるまで最大 kTakeoffTimeoutMs、その後 kLandTimeoutMs = 20000
LAND_TIMEOUT_S = 12.0 + 20.0 + NETWORK_MARGIN_S
ROTATE_TIMEOUT_S = 8.0 + NETWORK_MARGIN_S           # kRotateTimeoutMs = 8000
# A move waits distance / speed + 6 s (cmdMove). The cruise speed is settable by another
# client (`speed`, minimum 10 cm/s = kSpeedMinCmPerSec), so the worst case is used.
# 移動は 距離 / 速度 + 6 s 待つ（cmdMove）。巡航速度は他のクライアントが変えられる
# （`speed`、最小 10 cm/s = kSpeedMinCmPerSec）ので、最悪の場合を使う。
MOVE_EXTRA_S = 6.0
SLOWEST_SPEED_CM_PER_S = 10.0

KIND_OK = "ok"
KIND_ERROR = "error"
KIND_TIMEOUT = "timeout"
KIND_VALUE = "value"  # a query's answer / 問い合わせの答え


@dataclass(frozen=True)
class ApiResult:
    """Outcome of one command / 1 コマンドの結果

    kind: "ok" (reply "ok" or "ok ..."), "error" (reply "error ..."), "value" (any other reply:
          a query answer) or "timeout" (no reply in time).
    text: the vehicle's reply verbatim ("" for a timeout).
    kind: "ok"（応答が "ok" か "ok ..."）、"error"（応答が "error ..."）、"value"（それ以外の応答 =
          問い合わせの答え）、"timeout"（時間内に応答なし）。
    text: 機体の応答そのまま（timeout では ""）。
    """

    kind: str
    text: str

    @property
    def succeeded(self) -> bool:
        return self.kind in (KIND_OK, KIND_VALUE)


def classify_reply(text: str) -> str:
    """Return the kind of a reply text / 応答文字列の種別を返す"""
    stripped = text.strip()
    if stripped == "ok" or stripped.startswith("ok "):
        return KIND_OK
    if stripped.startswith("error"):
        return KIND_ERROR
    return KIND_VALUE


def move_timeout_s(distance_cm: float) -> float:
    """Reply wait for a move of `distance_cm` / 距離 `distance_cm` の移動の応答待ち時間"""
    travel_s = distance_cm / SLOWEST_SPEED_CM_PER_S
    return travel_s + MOVE_EXTRA_S + NETWORK_MARGIN_S


class VehicleApi:
    """Sender of API commands to one vehicle / 1 機の機体への API コマンド送信者"""

    def __init__(self, host: str = DEFAULT_HOST, port: "int | None" = None):
        self.host = host
        # Resolved at call time so that a test can point API_PORT at a local fake vehicle.
        # 呼び出し時に決める（試験が API_PORT をローカルの偽の機体へ向けられるように）。
        self.port = API_PORT if port is None else port

    # -- low level / 低水準 ---------------------------------------------------

    def _open_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return sock

    def _exchange(self, sock: socket.socket, text: str, timeout_s: float) -> ApiResult:
        """Send `text` on `sock` and wait for one reply / `sock` で送り、応答を 1 つ待つ"""
        sock.sendto(text.encode("utf-8"), (self.host, self.port))
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return ApiResult(KIND_TIMEOUT, "")
            sock.settimeout(remaining)
            try:
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                return ApiResult(KIND_TIMEOUT, "")
            reply = data.decode("utf-8", errors="replace").strip()
            return ApiResult(classify_reply(reply), reply)

    # -- public / 公開 -------------------------------------------------------

    def send(self, text: str, timeout_s: "float | None" = None) -> ApiResult:
        """Enter SDK mode, send `text`, wait up to `timeout_s` for the reply.

        Returns the handshake's failure (kind error / timeout) if `command` was not answered
        "ok"; the command itself is then NOT sent.
        SDK モードに入り、`text` を送り、`timeout_s` まで応答を待つ。`command` が "ok" で
        応答されなければ、その失敗を返し、コマンド本体は送らない。
        """
        timeout_s = IMMEDIATE_TIMEOUT_S if timeout_s is None else timeout_s
        sock = self._open_socket()
        try:
            handshake = self._exchange(sock, "command", IMMEDIATE_TIMEOUT_S)
            if handshake.kind != KIND_OK:
                return handshake
            return self._exchange(sock, text, timeout_s)
        finally:
            sock.close()

    def send_urgent(self, text: str, timeout_s: "float | None" = None) -> ApiResult:
        """Send `text` at once, with no `command` handshake (for `emergency` and `stop`).

        `text` を `command` の手順なしで即座に送る（`emergency` と `stop` 用）。
        """
        timeout_s = IMMEDIATE_TIMEOUT_S if timeout_s is None else timeout_s
        sock = self._open_socket()
        try:
            return self._exchange(sock, text, timeout_s)
        finally:
            sock.close()

    def emergency(self) -> ApiResult:
        """Cut the motors / モータを止める"""
        return self.send_urgent("emergency")

    def stop(self) -> ApiResult:
        """Hold the current position (also interrupts a waiting command)

        現在位置を保持する（待機中のコマンドも打ち切る）
        """
        return self.send_urgent("stop")

    def send_without_reply(self, text: str) -> ApiResult:
        """Enter SDK mode and send `text`, which the vehicle never answers (`rc`).

        Returns the handshake's failure if the vehicle did not answer `command`.
        SDK モードに入り、機体が応答しない `text`（`rc`）を送る。`command` に応答がなければ
        その失敗を返す。
        """
        sock = self._open_socket()
        try:
            handshake = self._exchange(sock, "command", IMMEDIATE_TIMEOUT_S)
            if handshake.kind != KIND_OK:
                return handshake
            sock.sendto(text.encode("utf-8"), (self.host, self.port))
            return ApiResult(KIND_OK, "")
        finally:
            sock.close()

    def open_rc_stream(self) -> "RcStream":
        """Open a stream for repeated `rc` lines / 繰り返し `rc` を送るためのストリームを開く"""
        return RcStream(self)


class RcStream:
    """One socket that sends `rc` lines repeatedly after a single handshake

    1 回の手順の後、1 つのソケットで `rc` を繰り返し送る
    """

    def __init__(self, api: VehicleApi):
        self._api = api
        self._sock = api._open_socket()
        self.handshake = api._exchange(self._sock, "command", IMMEDIATE_TIMEOUT_S)

    def send(self, roll: int, pitch: int, throttle: int, yaw: int) -> None:
        line = f"rc {roll} {pitch} {throttle} {yaw}"
        self._sock.sendto(line.encode("utf-8"), (self._api.host, self._api.port))

    def close(self) -> None:
        self._sock.close()
