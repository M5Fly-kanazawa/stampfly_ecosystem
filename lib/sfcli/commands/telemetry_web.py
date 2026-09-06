"""
sf telemetry --web — browser telemetry view (UDP -> SSE proxy)

Receives the vehicle 50Hz monitoring telemetry (UDP broadcast :5005,
104-byte binary packet — decoder shared with `sf telemetry`) and serves a
single-page browser dashboard. Zero external dependencies, matching the SILS
GUI policy: a stdlib ThreadingHTTPServer serves the embedded page and pushes
live JSON over Server-Sent Events (`/events`) — SSE is the stdlib-friendly
equivalent of the WebSocket proxy named in requirements §7 (one-way push is
all a monitor needs).

vehicle の 50Hz モニタ用テレメトリ（UDP ブロードキャスト :5005、104B
バイナリ — デコーダは `sf telemetry` と共有）を受信し、ブラウザ用の
シングルページダッシュボードを提供する。SILS GUI と同じ「外部依存ゼロ」方針:
stdlib の ThreadingHTTPServer が埋め込みページを配信し、Server-Sent Events
（`/events`）でライブ JSON をプッシュする — SSE は requirements §7 の
WebSocket プロキシの stdlib 等価（モニタに必要なのは一方向プッシュのみ）。
"""

import json
import re
import socket
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from . import telemetry as telem
from ..utils import console, paths

# Latest decoded packet + arrival bookkeeping, shared between the UDP thread
# and the HTTP handler threads (GIL-atomic reference swap; no lock needed).
# 最新デコード済みパケット＋到着情報。UDP スレッドと HTTP ハンドラ間で共有
# （参照の差し替えは GIL でアトミック。ロック不要）。
_latest = {"pkt": None, "rx_monotonic": 0.0, "count": 0, "rate_hz": 0.0}


def _udp_listener(port: int, csv_path=None) -> None:
    """Background thread: UDP :port -> _latest (+ optional CSV).
    背景スレッド: UDP受信→_latest更新（＋任意で CSV 記録）"""
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
    sock.bind(("", port))
    csv_file = None
    if csv_path:
        csv_file = open(csv_path, "a", buffering=1)
        if csv_file.tell() == 0:
            csv_file.write("t_us,mode," + ",".join(telem.FLOAT_NAMES) + "\n")
    window = []
    while True:
        data, _addr = sock.recvfrom(2048)
        pkt = telem._decode(data)
        if pkt is None:
            continue
        now = time.monotonic()
        window.append(now)
        while window and now - window[0] > 2.0:
            window.pop(0)
        _latest["pkt"] = pkt
        _latest["rx_monotonic"] = now
        _latest["count"] += 1
        _latest["rate_hz"] = len(window) / 2.0
        if csv_file:
            csv_file.write(f"{pkt['t_us']},{pkt['mode']},"
                           + ",".join(f"{pkt[k]:.6g}" for k in telem.FLOAT_NAMES) + "\n")


# The dashboard page lives as a sibling asset (it grew a full SILS-GUI-ported 3D
# view); the STL body parts are the SAME files the SILS GUI and MuJoCo use.
# ダッシュボードページは隣接アセット（SILS GUI 移植の 3D ビューを含み大きい）。
# STL 本体パーツは SILS GUI・MuJoCo と「同一ファイル」。
_PAGE_PATH = Path(__file__).resolve().parent.parent / "assets" / "telemetry_web.html"
_MESH_DIR = None   # resolved lazily (repo root lookup) / 遅延解決（リポジトリルート探索）
_VENDOR_DIR = None # three.js, same lazy-resolve pattern as _mesh_dir() / three.js。_mesh_dir()と同じ遅延解決


def _mesh_dir() -> Path:
    global _MESH_DIR
    if _MESH_DIR is None:
        _MESH_DIR = paths.root() / "simulator" / "shared" / "assets" / "meshes" / "parts"
    return _MESH_DIR


def _vendor_dir() -> Path:
    # three.js is vendored (not CDN-loaded): this page's normal use is a PC
    # whose Wi-Fi is associated 1:1 with the vehicle's own SoftAP (or an
    # offline workshop LAN), which has no route to any CDN at all -- not an
    # occasional outage. See simulator/shared/assets/vendor/three/README.md.
    # three.js はCDNではなくローカル同梱: このページの通常利用はPCのWi-Fiが
    # 機体自身のSoftAP（またはオフライン講習LAN）に1対1接続された状態で、
    # CDNへの経路がそもそも無い -- 稀な障害ではない。詳細は
    # simulator/shared/assets/vendor/three/README.md 参照。
    global _VENDOR_DIR
    if _VENDOR_DIR is None:
        _VENDOR_DIR = paths.root() / "simulator" / "shared" / "assets" / "vendor" / "three"
    return _VENDOR_DIR



class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *fmt_args):          # silence per-request logging
        pass                                        # リクエスト毎ログを抑止

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            body = _PAGE_PATH.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/mesh/"):
            # StampFly STL part for the 3D view — whitelisted name, no traversal
            # (same rule as the SILS GUI server).
            # 3D 用 STL パーツ — 名前を制限しトラバーサル防止（SILS GUI と同じ規則）。
            name = self.path[len("/mesh/"):]
            if not re.fullmatch(r"[a-z0-9_]+\.stl", name):
                self.send_error(400, "bad mesh name")
                return
            mesh = _mesh_dir() / name
            if not mesh.exists():
                self.send_error(404)
                return
            body = mesh.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "model/stl")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/vendor/three/"):
            # Vendored three.js (see _vendor_dir()) -- whitelisted relative
            # path, no traversal (same rule as /mesh/ above).
            # 同梱三js（_vendor_dir()参照） -- 相対パスを許可リスト化しトラバーサル防止
            # （上の /mesh/ と同じ規則）。
            rel = self.path[len("/vendor/three/"):]
            if not re.fullmatch(r"[A-Za-z0-9_./-]+\.js", rel) or ".." in rel.split("/"):
                self.send_error(400, "bad vendor path")
                return
            asset = _vendor_dir() / rel
            if not asset.exists():
                self.send_error(404)
                return
            body = asset.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/javascript; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            # Push the latest packet at 10Hz until the browser disconnects.
            # ブラウザ切断まで最新パケットを 10Hz でプッシュ。
            try:
                while True:
                    payload = json.dumps({
                        "pkt": _latest["pkt"],
                        "count": _latest["count"],
                        "rate_hz": _latest["rate_hz"],
                        "age_s": (time.monotonic() - _latest["rx_monotonic"])
                                 if _latest["pkt"] else None,
                    })
                    self.wfile.write(f"data: {payload}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError,
                    ConnectionAbortedError):   # ConnectionAborted: Windows / Windows系
                return
        else:
            self.send_error(404)


def serve(http_port: int, telemetry_port: int, open_browser: bool,
          csv_path=None) -> int:
    """Run the UDP->SSE proxy + page server (blocks until Ctrl-C).
    UDP→SSE プロキシ＋ページサーバを起動（Ctrl-C まで実行）。"""
    threading.Thread(target=_udp_listener, args=(telemetry_port, csv_path),
                     daemon=True).start()
    httpd = ThreadingHTTPServer(("", http_port), _Handler)
    url = f"http://localhost:{http_port}/"
    console.info(f"Telemetry web view: {url}  (UDP :{telemetry_port} -> SSE)")
    console.info("Ctrl-C to stop")
    if open_browser:
        webbrowser.open(url)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print()
        console.info("Stopped")
    finally:
        httpd.server_close()
    return 0
