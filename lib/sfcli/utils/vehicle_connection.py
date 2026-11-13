"""
Vehicle TCP CLI connection (port 23) for StampFly CLI
機体の TCP CLI（ポート 23）接続

The vehicle's TCP 23 CLI is a bench / debug console (param, status, sensor, motor, wifi, reboot,
...). It has no flight commands: flying and querying go through the UDP API
(lib/sfcli/utils/vehicle_api.py). Used by `sf motor`.
機体の TCP 23 の CLI は台上・デバッグ用のコンソール（param・status・sensor・motor・wifi・
reboot など）。飛行コマンドは無い: 飛行と問い合わせは UDP API（lib/sfcli/utils/vehicle_api.py）を
通る。`sf motor` が使う。
"""

import asyncio
from typing import Optional

from .vehicle_api import DEFAULT_HOST

CLI_PORT = 23

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
