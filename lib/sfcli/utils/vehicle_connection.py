"""
Vehicle connection utilities for StampFly CLI

Provides async TCP CLI and WebSocket telemetry connections to the vehicle.
WiFi CLI (TCP:23) でコマンド送信、WebSocket (ws://host:80/ws) でテレメトリ受信。
"""

import asyncio
import re
import struct
import time
from typing import Optional, Callable, Any

from .packet_parser import parse_packet

# Default vehicle IP (StampFly WiFi AP). The SoftAP is addressed at 192.168.10.1
# (the DJI Tello subnet) so stock Tello/djitellopy programs connect unchanged.
# StampFly WiFi APのデフォルトIP。SoftAP は 192.168.10.1（DJI Tello のサブネット）で、
# 純正 Tello/djitellopy プログラムが無改変で繋がる。
DEFAULT_HOST = "192.168.10.1"
CLI_PORT = 23
WS_PORT = 80

# Telnet negotiation bytes
# Telnetネゴシエーションバイト
IAC = 0xFF
WILL = 0xFB
WONT = 0xFC
DO = 0xFD
DONT = 0xFE

# Prompt string from WiFi CLI
# WiFi CLIのプロンプト文字列
PROMPT = "stampfly> "


class VehicleCLI:
    """Async TCP client for StampFly WiFi CLI (port 23)

    StampFly WiFi CLI用の非同期TCPクライアント
    """

    def __init__(self):
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    async def connect(self, host: str = DEFAULT_HOST, timeout: float = 5.0) -> None:
        """Connect to WiFi CLI server and consume welcome banner.

        WiFi CLIサーバーに接続し、ウェルカムバナーを消費する
        """
        self._reader, self._writer = await asyncio.wait_for(
            asyncio.open_connection(host, CLI_PORT),
            timeout=timeout,
        )
        self._connected = True

        # Read and discard welcome banner + first prompt
        # ウェルカムバナーと最初のプロンプトを読み捨てる
        await self._read_until_prompt(timeout=timeout)

    async def send_command(self, cmd: str, timeout: float = 5.0) -> str:
        """Send a command and return the response text.

        コマンドを送信し、応答テキストを返す
        """
        if not self._connected:
            raise ConnectionError("Not connected to vehicle CLI")

        # Send command
        # コマンドを送信
        self._writer.write(f"{cmd}\n".encode())
        await self._writer.drain()

        # Read response until next prompt
        # 次のプロンプトまで応答を読み取る
        response = await self._read_until_prompt(timeout=timeout)
        return response

    async def poll_flight_status(self, timeout: float = 3.0) -> dict:
        """Send 'flight status' and parse response.

        Returns dict with keys: command, state, elapsed
        'flight status' を送信して応答をパースする
        """
        response = await self.send_command("flight status", timeout=timeout)

        result = {
            "command": "NONE",
            "state": "IDLE",
            "elapsed": 0.0,
        }

        for line in response.splitlines():
            line = line.strip()
            if line.startswith("Flight command:"):
                result["command"] = line.split(":", 1)[1].strip()
            elif line.startswith("State:"):
                result["state"] = line.split(":", 1)[1].strip()
            elif line.startswith("Elapsed:"):
                match = re.search(r"([\d.]+)", line)
                if match:
                    result["elapsed"] = float(match.group(1))

        return result

    async def disconnect(self) -> None:
        """Close connection.

        接続を閉じる
        """
        if self._writer:
            self._writer.close()
            try:
                await self._writer.wait_closed()
            except Exception:
                pass
        self._connected = False
        self._reader = None
        self._writer = None

    async def _read_until_prompt(self, timeout: float = 5.0) -> str:
        """Read data until the prompt string appears, handling telnet negotiation.

        プロンプト文字列が現れるまでデータを読み取る（Telnetネゴシエーション処理付き）
        """
        buf = bytearray()
        deadline = asyncio.get_event_loop().time() + timeout

        while True:
            remaining = deadline - asyncio.get_event_loop().time()
            if remaining <= 0:
                break

            try:
                chunk = await asyncio.wait_for(
                    self._reader.read(4096),
                    timeout=min(remaining, 1.0),
                )
            except asyncio.TimeoutError:
                # Check if we already have the prompt
                # 既にプロンプトがあるかチェック
                if PROMPT.encode() in buf:
                    break
                continue

            if not chunk:
                raise ConnectionError("Connection closed by vehicle")

            # Handle telnet negotiation bytes (IAC sequences)
            # Telnetネゴシエーションバイトを処理
            i = 0
            while i < len(chunk):
                if chunk[i] == IAC and i + 2 < len(chunk):
                    verb = chunk[i + 1]
                    option = chunk[i + 2]
                    # Refuse all options
                    # 全オプションを拒否
                    if verb == WILL:
                        self._writer.write(bytes([IAC, DONT, option]))
                    elif verb == DO:
                        self._writer.write(bytes([IAC, WONT, option]))
                    i += 3
                else:
                    buf.append(chunk[i])
                    i += 1

            await self._writer.drain()

            # Check for prompt
            # プロンプトをチェック
            if PROMPT.encode() in buf:
                break

        # Strip prompt from response
        # 応答からプロンプトを除去
        text = buf.decode("utf-8", errors="replace")
        text = text.replace(PROMPT, "").strip()
        return text


class VehicleTelemetry:
    """Async WebSocket client for StampFly telemetry (ws://host:80/ws)

    StampFly テレメトリ用の非同期WebSocketクライアント
    """

    def __init__(self):
        self._ws = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    async def connect(self, host: str = DEFAULT_HOST, timeout: float = 5.0) -> None:
        """Connect to WebSocket telemetry endpoint.

        WebSocketテレメトリエンドポイントに接続する
        """
        try:
            import websockets
        except ImportError:
            raise ImportError(
                "websockets library required. Install with: pip install websockets"
            )

        uri = f"ws://{host}:{WS_PORT}/ws"
        self._ws = await asyncio.wait_for(
            websockets.connect(uri, ping_interval=None),
            timeout=timeout,
        )
        self._connected = True

    async def receive(self, timeout: float = 1.0) -> Optional[dict]:
        """Receive and parse one telemetry packet.

        Returns the latest sample dict with keys like tof_bottom, pos_z, vel_z,
        ctrl_throttle, or None on timeout/parse failure.

        テレメトリパケットを1つ受信してパースする
        """
        if not self._ws:
            return None

        try:
            data = await asyncio.wait_for(self._ws.recv(), timeout=timeout)
        except asyncio.TimeoutError:
            return None

        if not isinstance(data, bytes):
            return None

        samples, mode = parse_packet(data)
        if samples:
            # Return the latest sample in the batch
            # バッチ内の最新サンプルを返す
            return samples[-1]
        return None

    async def disconnect(self) -> None:
        """Close WebSocket connection.

        WebSocket接続を閉じる
        """
        if self._ws:
            try:
                await self._ws.close()
            except Exception:
                pass
        self._connected = False
        self._ws = None


class VehicleConnection:
    """Unified connection to vehicle via WiFi CLI + WebSocket telemetry.

    WiFi CLIとWebSocketテレメトリを統合した機体接続クラス
    """

    def __init__(self):
        self.cli = VehicleCLI()
        self.telemetry = VehicleTelemetry()
        self._cancel_requested = False

    async def connect(self, host: str = DEFAULT_HOST, timeout: float = 5.0) -> None:
        """Connect both CLI and telemetry channels.

        CLIとテレメトリの両方を接続する
        """
        # Connect both in parallel
        # 両方を並行して接続
        await asyncio.gather(
            self.cli.connect(host, timeout),
            self.telemetry.connect(host, timeout),
        )

    async def send_flight_command(self, cmd: str, timeout: float = 5.0) -> str:
        """Send a flight command via CLI and return response.

        CLI経由でフライトコマンドを送信し応答を返す
        """
        return await self.cli.send_command(cmd, timeout)

    async def monitor_flight(
        self,
        callback: Callable[[dict, dict], None],
        poll_interval: float = 0.5,
        timeout: float = 120.0,
    ) -> None:
        """Monitor flight by combining telemetry + CLI status polling.

        Calls callback(telemetry_data, flight_status) periodically.
        Stops when flight state is IDLE or timeout reached.

        テレメトリ受信とCLIステータスポーリングを組み合わせて飛行を監視する
        """
        self._cancel_requested = False
        start_time = time.monotonic()
        last_poll = 0.0
        flight_status = {"command": "NONE", "state": "IDLE", "elapsed": 0.0}

        while not self._cancel_requested:
            elapsed = time.monotonic() - start_time
            if elapsed > timeout:
                break

            # Receive telemetry (non-blocking with short timeout)
            # テレメトリ受信（短いタイムアウトでノンブロッキング）
            telemetry = await self.telemetry.receive(timeout=0.1)

            # Poll CLI status at specified interval
            # 指定間隔でCLIステータスをポーリング
            now = time.monotonic()
            if now - last_poll >= poll_interval:
                try:
                    flight_status = await self.cli.poll_flight_status(timeout=2.0)
                except Exception:
                    pass
                last_poll = now

            # Call callback with latest data
            # 最新データでコールバックを呼び出す
            if telemetry:
                callback(telemetry, flight_status)

            # Stop if flight command finished
            # フライトコマンドが終了したら停止
            if flight_status["state"] == "IDLE" and elapsed > 1.0:
                # Give one extra poll after IDLE to catch final state
                # IDLE後にもう一回ポーリングして最終状態を取得
                callback(telemetry or {}, flight_status)
                break

    async def cancel_flight(self) -> str:
        """Send flight cancel command.

        フライトキャンセルコマンドを送信する
        """
        self._cancel_requested = True
        return await self.cli.send_command("flight cancel", timeout=3.0)

    async def disconnect(self) -> None:
        """Disconnect both channels.

        両チャネルを切断する
        """
        await asyncio.gather(
            self.cli.disconnect(),
            self.telemetry.disconnect(),
        )
