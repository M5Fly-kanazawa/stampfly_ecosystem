"""
record.py - record the replay page to an mp4 (headless Chrome + ffmpeg)
再生ページを mp4 に録画する（ヘッドレス Chrome + ffmpeg）

Serves page/, the vendored three.js, the STL parts and the replay data from a
temporary directory over a local HTTP server, drives Chrome through the DevTools
Protocol (websockets), asks the page for frame k (window.__frame(k)), takes a
JPEG screenshot and pipes it to ffmpeg. The end-to-end recording is a manual
check (it needs Chrome and ffmpeg); only Chrome discovery is unit-tested.
一時フォルダから page/・同梱の three.js・STL 部品・再生データを局所 HTTP
サーバで配り、DevTools Protocol（websockets）で Chrome を操作し、コマ k を
ページに描かせて（window.__frame(k)）JPEG のスクリーンショットを撮り、
ffmpeg に流し込む。録画全体の確認は手動（Chrome と ffmpeg が要る）で、
単体試験は Chrome の探索だけを対象にする。
"""

import asyncio
import base64
import functools
import http.server
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Callable, Optional, Tuple

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent.parent
PAGE_DIR = HERE / "page"
VENDOR_DIR = HERE / "vendor"
MESH_DIR = REPO_ROOT / "simulator" / "shared" / "assets" / "meshes" / "parts"
PART_NAMES = ["frame", "pcb", "m5stamps3", "battery", "battery_adapter",
              "motor_fl", "motor_fr", "motor_rl", "motor_rr"]

JPEG_QUALITY = 92
CHROME_START_TIMEOUT_S = 30.0
PAGE_READY_TIMEOUT_S = 60.0
CDP_TIMEOUT_S = 120.0

MAC_APPS = [
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    "/Applications/Chromium.app/Contents/MacOS/Chromium",
    "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge",
]
WINDOWS_RELATIVE = [
    ("PROGRAMFILES", r"Google\Chrome\Application\chrome.exe"),
    ("PROGRAMFILES(X86)", r"Google\Chrome\Application\chrome.exe"),
    ("LOCALAPPDATA", r"Google\Chrome\Application\chrome.exe"),
    ("PROGRAMFILES", r"Microsoft\Edge\Application\msedge.exe"),
    ("PROGRAMFILES(X86)", r"Microsoft\Edge\Application\msedge.exe"),
    ("LOCALAPPDATA", r"Microsoft\Edge\Application\msedge.exe"),
]
LINUX_NAMES = ["google-chrome", "google-chrome-stable", "chromium", "chromium-browser", "microsoft-edge"]

CHROME_MISSING = (
    "Google Chrome (or Chromium / Edge) was not found. Install Chrome from "
    "https://www.google.com/chrome/ , or set the environment variable CHROME_PATH "
    "to the browser executable."
)
FFMPEG_MISSING = (
    "ffmpeg was not found. Install it:\n"
    "  macOS:   brew install ffmpeg\n"
    "  Windows: winget install Gyan.FFmpeg   (then reopen the terminal)\n"
    "  Linux:   sudo apt install ffmpeg\n"
    "Or use --frames-dir DIR to write the JPEG frames only."
)


class RecordError(RuntimeError):
    """A recording problem with a message fit to show to the user.
    利用者にそのまま見せてよいメッセージ付きの録画エラー。"""


# ------------------------------------------------------------ Chrome search
def find_chrome(
    env: Optional[dict] = None,
    exists: Callable[[str], bool] = os.path.exists,
    which: Callable[[str], Optional[str]] = shutil.which,
    platform: Optional[str] = None,
) -> Optional[str]:
    """Path of a usable Chrome / Chromium / Edge, or None. CHROME_PATH wins.
    The environment, file test, PATH lookup and platform are parameters so the
    search can be tested with a fake file system.
    使える Chrome / Chromium / Edge のパス。無ければ None。CHROME_PATH が最優先。
    環境変数・ファイル有無・PATH 検索・OS は引数にして、偽のファイル構成で試験できる。"""
    env = os.environ if env is None else env
    platform = sys.platform if platform is None else platform
    override = env.get("CHROME_PATH")
    if override and exists(override):
        return override
    if platform == "darwin":
        for path in MAC_APPS:
            if exists(path):
                return path
    elif platform.startswith("win"):
        for var, rel in WINDOWS_RELATIVE:
            base = env.get(var)
            if base:
                path = base.rstrip("\\/") + "\\" + rel
                if exists(path):
                    return path
    for name in LINUX_NAMES:
        found = which(name)
        if found:
            return found
    return None


def find_ffmpeg(which: Callable[[str], Optional[str]] = shutil.which) -> Optional[str]:
    """Path of ffmpeg or None. ffmpeg のパス。無ければ None。"""
    return which("ffmpeg")


# --------------------------------------------------------------- HTTP server
class _QuietHandler(http.server.SimpleHTTPRequestHandler):
    """Static file handler without the access log. アクセスログを出さない静的ファイル配信。"""

    def log_message(self, *args):  # noqa: D401
        pass


class _Server(http.server.ThreadingHTTPServer):
    """Local server with a long accept queue: the page asks for nine STL files at
    once, and the default queue of 5 drops connections.
    長い接続待ち行列を持つ局所サーバ。ページは STL 9 個を同時に取りに来るので、
    既定の待ち行列 5 だと接続が落ちる。"""

    request_queue_size = 128


def stage_site(root: Path, data: dict) -> None:
    """Copy the page, vendored three.js, STL parts and the data into `root`.
    ページ・同梱 three.js・STL 部品・データを `root` へ集める。"""
    for item in PAGE_DIR.iterdir():
        if item.is_file():
            shutil.copy2(item, root / item.name)
    shutil.copytree(VENDOR_DIR, root / "vendor")
    (root / "meshes").mkdir()
    for name in PART_NAMES:
        shutil.copy2(MESH_DIR / f"{name}.stl", root / "meshes" / f"{name}.stl")
    (root / "data.json").write_text(json.dumps(data, separators=(",", ":")), encoding="utf-8")


def start_server(root: Path) -> Tuple[_Server, int]:
    """Serve `root` on a free local port in a background thread.
    `root` を空いている局所ポートで背景スレッドから配信する。"""
    handler = functools.partial(_QuietHandler, directory=str(root))
    server = _Server(("127.0.0.1", 0), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, server.server_address[1]


# --------------------------------------------------------------- Chrome / CDP
def chrome_args(chrome: str, profile: Path, size: Tuple[int, int], swiftshader: bool) -> list:
    """Command line for headless Chrome. ヘッドレス Chrome の起動コマンド。"""
    args = [
        chrome, "--headless=new", "--remote-debugging-port=0", f"--user-data-dir={profile}",
        "--hide-scrollbars", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist",
        f"--window-size={size[0]},{size[1]}", "--force-device-scale-factor=1",
        "--no-first-run", "--no-default-browser-check", "--disable-background-timer-throttling",
        "--disable-renderer-backgrounding", "--mute-audio",
    ]
    if swiftshader:
        args += ["--use-gl=angle", "--use-angle=swiftshader"]
    return args + ["about:blank"]


def read_debug_endpoint(profile: Path, proc: subprocess.Popen) -> str:
    """Browser WebSocket URL from the DevToolsActivePort file Chrome writes.
    Chrome が書く DevToolsActivePort ファイルからブラウザの WebSocket URL を得る。"""
    port_file = profile / "DevToolsActivePort"
    deadline = time.time() + CHROME_START_TIMEOUT_S
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RecordError("Chrome exited at start-up (is another Chrome using the same profile?)")
        if port_file.exists():
            lines = port_file.read_text().split("\n")
            if len(lines) >= 2 and lines[0].strip() and lines[1].strip():
                return f"ws://127.0.0.1:{lines[0].strip()}{lines[1].strip()}"
        time.sleep(0.1)
    raise RecordError("Chrome did not open its debugging port in time")


class Cdp:
    """Minimal Chrome DevTools Protocol client over one WebSocket.
    1 本の WebSocket 上の最小限の Chrome DevTools Protocol クライアント。"""

    def __init__(self, ws):
        self.ws, self.next_id, self.pending, self.console = ws, 0, {}, []
        self.reader = asyncio.ensure_future(self._read())

    async def _read(self):
        async for raw in self.ws:
            msg = json.loads(raw)
            if "id" in msg and msg["id"] in self.pending:
                self.pending.pop(msg["id"]).set_result(msg)
            elif msg.get("method") in ("Runtime.consoleAPICalled", "Runtime.exceptionThrown"):
                self.console.append(msg)

    async def call(self, method: str, params: Optional[dict] = None, session: Optional[str] = None) -> dict:
        self.next_id += 1
        msg = {"id": self.next_id, "method": method, "params": params or {}}
        if session:
            msg["sessionId"] = session
        fut = asyncio.get_running_loop().create_future()
        self.pending[self.next_id] = fut
        await self.ws.send(json.dumps(msg))
        reply = await asyncio.wait_for(fut, CDP_TIMEOUT_S)
        if "error" in reply:
            raise RecordError(f"Chrome {method} failed: {reply['error'].get('message')}")
        return reply.get("result", {})

    def console_errors(self) -> list:
        out = []
        for m in self.console:
            if m["method"] == "Runtime.exceptionThrown":
                out.append(m["params"]["exceptionDetails"].get("text", "exception"))
            elif m["params"].get("type") == "error":
                out.append(" ".join(str(a.get("value", a.get("description", ""))) for a in m["params"]["args"]))
        return out


async def _wait_ready(cdp: Cdp, session: str) -> int:
    """Wait for window.__ready, return window.__nframes. window.__ready を待ち、フレーム数を返す。"""
    deadline = time.time() + PAGE_READY_TIMEOUT_S
    while time.time() < deadline:
        res = await cdp.call("Runtime.evaluate", {
            "expression": "JSON.stringify([window.__ready===true, window.__nframes||0, window.__error||null])",
            "returnByValue": True}, session)
        ready, n, err = json.loads(res["result"]["value"])
        if err:
            raise RecordError(f"the replay page failed to start: {err}")
        if ready:
            return int(n)
        await asyncio.sleep(0.2)
    raise RecordError("the replay page did not become ready: " + "; ".join(cdp.console_errors()[:3]))


async def _record(url, chrome_proc_ws, size, n_expected, fps, sink, quality, progress):
    import websockets
    async with websockets.connect(chrome_proc_ws, max_size=None, ping_interval=None) as ws:
        cdp = Cdp(ws)
        target = await cdp.call("Target.createTarget", {"url": "about:blank"})
        att = await cdp.call("Target.attachToTarget", {"targetId": target["targetId"], "flatten": True})
        sid = att["sessionId"]
        await cdp.call("Page.enable", {}, sid)
        await cdp.call("Runtime.enable", {}, sid)
        await cdp.call("Emulation.setDeviceMetricsOverride", {
            "width": size[0], "height": size[1], "deviceScaleFactor": 1, "mobile": False}, sid)
        await cdp.call("Page.navigate", {"url": url}, sid)
        n = await _wait_ready(cdp, sid)
        if n != n_expected:
            raise RecordError(f"frame count mismatch: page {n}, data {n_expected}")
        started = time.time()
        for k in range(n):
            await cdp.call("Runtime.evaluate", {"expression": f"window.__frame({k})"}, sid)
            shot = await cdp.call("Page.captureScreenshot", {"format": "jpeg", "quality": quality}, sid)
            await sink(k, base64.b64decode(shot["data"]))
            if progress and (k % 30 == 0 or k == n - 1):
                done = k + 1
                eta = (time.time() - started) / done * (n - done)
                progress(done, n, eta)
        errors = cdp.console_errors()
        if errors:
            print("Page reported errors: " + "; ".join(errors[:3]), file=sys.stderr)
        cdp.reader.cancel()


def _kill(proc: subprocess.Popen) -> None:
    """Stop a child process and wait for it. 子プロセスを止めて終了を待つ。"""
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


def record_video(
    data: dict,
    out_path: Optional[Path],
    *,
    size: Tuple[int, int] = (1920, 1080),
    view: str = "oblique",
    tilt_gain: float = 3.0,
    vehicle_scale: float = 2.0,
    zoom: float = 0.6,
    frames_dir: Optional[Path] = None,
    swiftshader: bool = False,
    quality: int = JPEG_QUALITY,
    progress: Optional[Callable[[int, int, float], None]] = None,
) -> None:
    """Record `data` (replay_data.build_replay_data) to `out_path` (mp4), or to
    JPEG files in `frames_dir`. Chrome and the server are always stopped.
    `data`（replay_data.build_replay_data）を `out_path`（mp4）か `frames_dir` の
    JPEG 連番に録画する。Chrome とサーバは必ず止める。"""
    chrome = find_chrome()
    if not chrome:
        raise RecordError(CHROME_MISSING)
    ffmpeg = None
    if frames_dir is None:
        ffmpeg = find_ffmpeg()
        if not ffmpeg:
            raise RecordError(FFMPEG_MISSING)
    fps = data["meta"]["fps"]
    tmp = Path(tempfile.mkdtemp(prefix="sf_video_"))
    server = chrome_proc = ffmpeg_proc = None
    try:
        site, profile = tmp / "site", tmp / "profile"
        site.mkdir(); profile.mkdir()
        stage_site(site, data)
        server, port = start_server(site)
        query = (f"?record=1&view={view}&tilt={tilt_gain}&vscale={vehicle_scale}&zoom={zoom}")
        url = f"http://127.0.0.1:{port}/index.html{query}"
        chrome_proc = subprocess.Popen(chrome_args(chrome, profile, size, swiftshader),
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        ws_url = read_debug_endpoint(profile, chrome_proc)
        if frames_dir is not None:
            frames_dir.mkdir(parents=True, exist_ok=True)
        else:
            cmd = [ffmpeg, "-y", "-loglevel", "error", "-f", "image2pipe", "-framerate", f"{fps:g}",
                   "-c:v", "mjpeg", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18",
                   "-movflags", "+faststart", str(out_path)]
            ffmpeg_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)

        async def sink(k: int, jpeg: bytes) -> None:
            if frames_dir is not None:
                (frames_dir / f"frame_{k:05d}.jpg").write_bytes(jpeg)
            else:
                ffmpeg_proc.stdin.write(jpeg)
                await asyncio.get_running_loop().run_in_executor(None, ffmpeg_proc.stdin.flush)

        asyncio.run(_record(url, ws_url, size, len(data["t"]), fps, sink, quality, progress))
        if ffmpeg_proc is not None:
            ffmpeg_proc.stdin.close()
            if ffmpeg_proc.wait() != 0:
                raise RecordError("ffmpeg failed to encode the video")
    finally:
        if ffmpeg_proc is not None:
            _kill(ffmpeg_proc)
        if chrome_proc is not None:
            _kill(chrome_proc)
        if server is not None:
            server.shutdown()
            server.server_close()
        shutil.rmtree(tmp, ignore_errors=True)
