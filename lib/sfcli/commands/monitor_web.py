"""
sf monitor web — browser telemetry view (UDP -> SSE proxy)

Receives the vehicle_new 50Hz monitoring telemetry (UDP broadcast :5005,
104-byte binary packet — decoder shared with `sf telemetry`) and serves a
single-page browser dashboard. Zero external dependencies, matching the SIL
GUI policy: a stdlib ThreadingHTTPServer serves the embedded page and pushes
live JSON over Server-Sent Events (`/events`) — SSE is the stdlib-friendly
equivalent of the WebSocket proxy named in requirements §7 (one-way push is
all a monitor needs).

vehicle_new の 50Hz モニタ用テレメトリ（UDP ブロードキャスト :5005、104B
バイナリ — デコーダは `sf telemetry` と共有）を受信し、ブラウザ用の
シングルページダッシュボードを提供する。SIL GUI と同じ「外部依存ゼロ」方針:
stdlib の ThreadingHTTPServer が埋め込みページを配信し、Server-Sent Events
（`/events`）でライブ JSON をプッシュする — SSE は requirements §7 の
WebSocket プロキシの stdlib 等価（モニタに必要なのは一方向プッシュのみ）。
"""

import json
import socket
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from . import telemetry as telem
from ..utils import console

# Latest decoded packet + arrival bookkeeping, shared between the UDP thread
# and the HTTP handler threads (GIL-atomic reference swap; no lock needed).
# 最新デコード済みパケット＋到着情報。UDP スレッドと HTTP ハンドラ間で共有
# （参照の差し替えは GIL でアトミック。ロック不要）。
_latest = {"pkt": None, "rx_monotonic": 0.0, "count": 0, "rate_hz": 0.0}


def _udp_listener(port: int) -> None:
    """Background thread: UDP :port -> _latest. / 背景スレッド: UDP受信→_latest更新"""
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


_PAGE = """<!DOCTYPE html>
<html lang="ja"><head><meta charset="utf-8">
<title>StampFly telemetry</title>
<style>
 body{background:#10141a;color:#dde3ea;font-family:ui-monospace,Menlo,monospace;margin:18px}
 h1{font-size:16px;margin:0 0 10px} .muted{color:#7a8694}
 .row{display:flex;gap:14px;flex-wrap:wrap;margin-bottom:12px}
 .card{background:#1a212b;border:1px solid #2a3442;border-radius:8px;padding:10px 14px}
 .big{font-size:22px;font-weight:bold}
 .state{padding:2px 10px;border-radius:6px;background:#2a3442}
 .state.FLYING{background:#15803d}.state.ARMED_GROUND{background:#a16207}
 .state.LANDING{background:#c2410c}.state.INIT{background:#475569}
 canvas{background:#0c1015;border:1px solid #2a3442;border-radius:6px}
 .bar{height:14px;background:#2a3442;border-radius:4px;overflow:hidden;width:160px;display:inline-block;vertical-align:middle}
 .bar>div{height:100%;background:#3b82f6}
 table{border-collapse:collapse} td{padding:2px 10px 2px 0}
</style></head><body>
<h1>StampFly telemetry
 <span id="state" class="state">--</span>
 <span class="muted" id="meta">waiting for packets...</span></h1>
<div class="row">
 <div class="card"><div class="muted">attitude [deg]</div>
  <table><tr><td>roll</td><td class="big" id="roll">--</td></tr>
   <tr><td>pitch</td><td class="big" id="pitch">--</td></tr>
   <tr><td>yaw</td><td class="big" id="yaw">--</td></tr></table></div>
 <div class="card"><div class="muted">altitude / velocity</div>
  <table><tr><td>alt [m]</td><td class="big" id="alt">--</td></tr>
   <tr><td>vN vE vD [m/s]</td><td id="vel">--</td></tr>
   <tr><td>pos N E [m]</td><td id="pos">--</td></tr></table></div>
 <div class="card"><div class="muted">control</div>
  <table><tr><td>thrust [N]</td><td class="big" id="thrust">--</td></tr>
   <tr><td>&tau; r/p/y [mNm]</td><td id="tau">--</td></tr></table></div>
 <div class="card"><div class="muted">motors (duty)</div>
  <table>
   <tr><td>M1 FR</td><td><span class="bar"><div id="m1b"></div></span> <span id="m1">--</span></td></tr>
   <tr><td>M2 RR</td><td><span class="bar"><div id="m2b"></div></span> <span id="m2">--</span></td></tr>
   <tr><td>M3 RL</td><td><span class="bar"><div id="m3b"></div></span> <span id="m3">--</span></td></tr>
   <tr><td>M4 FL</td><td><span class="bar"><div id="m4b"></div></span> <span id="m4">--</span></td></tr>
  </table></div>
</div>
<div class="row">
 <div class="card"><div class="muted">attitude [deg] (30 s)</div><canvas id="c_att" width="560" height="160"></canvas></div>
 <div class="card"><div class="muted">altitude [m] (30 s)</div><canvas id="c_alt" width="560" height="160"></canvas></div>
</div>
<div class="row">
 <div class="card"><div class="muted">gyro [deg/s] (30 s)</div><canvas id="c_gyro" width="560" height="160"></canvas></div>
 <div class="card"><div class="muted">motor duty (30 s)</div><canvas id="c_duty" width="560" height="160"></canvas></div>
</div>
<script>
const R2D = 180/Math.PI, SPAN = 30;            // chart span [s]
const hist = [];                                // [{t, ...} ...]
const STATE = ["INIT","IDLE_GROUND","IDLE_HELD","ARMED_GROUND","TAKEOFF","FLYING","LANDING"];
function $(id){return document.getElementById(id);}
function fmt(x,d=2){return (x>=0?"+":"")+x.toFixed(d);}

// Minimal scrolling strip chart (no external libs — offline-friendly).
// 最小のスクロール式ストリップチャート（外部ライブラリなし — オフライン可）。
function drawChart(cv, series, colors, names){
  const ctx = cv.getContext("2d"), W = cv.width, H = cv.height;
  ctx.clearRect(0,0,W,H);
  if(hist.length < 2) return;
  const t1 = hist[hist.length-1].t, t0 = t1 - SPAN;
  let lo = Infinity, hi = -Infinity;
  for(const h of hist) for(const f of series){ lo = Math.min(lo,h[f]); hi = Math.max(hi,h[f]); }
  if(!isFinite(lo)) return;
  const pad = Math.max(0.1*(hi-lo), 0.5); lo -= pad; hi += pad;
  ctx.strokeStyle = "#2a3442"; ctx.beginPath();                  // zero line / ゼロ線
  const y0 = H - (0-lo)/(hi-lo)*H;
  if(y0 > 0 && y0 < H){ ctx.moveTo(0,y0); ctx.lineTo(W,y0); ctx.stroke(); }
  series.forEach((f,i)=>{
    ctx.strokeStyle = colors[i]; ctx.beginPath();
    let started = false;
    for(const h of hist){
      if(h.t < t0) continue;
      const x = (h.t-t0)/SPAN*W, y = H-(h[f]-lo)/(hi-lo)*H;
      if(!started){ ctx.moveTo(x,y); started = true; } else ctx.lineTo(x,y);
    }
    ctx.stroke();
    ctx.fillStyle = colors[i]; ctx.fillText(names[i]+" "+hi.toFixed(1)+"/"+lo.toFixed(1), 6+i*120, 12);
  });
}

const es = new EventSource("/events");
es.onmessage = (ev)=>{
  const d = JSON.parse(ev.data);
  if(!d.pkt) return;
  const p = d.pkt, t = p.t_us/1e6;
  $("state").textContent = STATE[p.mode] || ("?"+p.mode);
  $("state").className = "state " + (STATE[p.mode]||"");
  $("meta").textContent = d.rate_hz.toFixed(1)+" Hz  t="+t.toFixed(1)+"s  pkts "+d.count;
  $("roll").textContent  = fmt(p.roll*R2D);
  $("pitch").textContent = fmt(p.pitch*R2D);
  $("yaw").textContent   = fmt(p.yaw*R2D);
  $("alt").textContent   = fmt(-p.pos_z);
  $("vel").textContent   = fmt(p.vel_x)+" "+fmt(p.vel_y)+" "+fmt(p.vel_z);
  $("pos").textContent   = fmt(p.pos_x)+" "+fmt(p.pos_y);
  $("thrust").textContent= p.thrust.toFixed(3);
  $("tau").textContent   = fmt(p.tau_roll*1e3)+" "+fmt(p.tau_pitch*1e3)+" "+fmt(p.tau_yaw*1e3);
  for(const m of ["m1","m2","m3","m4"]){
    $(m).textContent = p[m].toFixed(2);
    $(m+"b").style.width = Math.min(100, p[m]*100)+"%";
  }
  hist.push({t:t, roll:p.roll*R2D, pitch:p.pitch*R2D, yaw:p.yaw*R2D,
             alt:-p.pos_z, gx:p.gyro_x*R2D, gy:p.gyro_y*R2D, gz:p.gyro_z*R2D,
             m1:p.m1, m2:p.m2, m3:p.m3, m4:p.m4});
  while(hist.length && hist[hist.length-1].t - hist[0].t > SPAN) hist.shift();
  drawChart($("c_att"),  ["roll","pitch","yaw"], ["#3b82f6","#22c55e","#eab308"], ["roll","pitch","yaw"]);
  drawChart($("c_alt"),  ["alt"],                ["#22d3ee"],                     ["alt"]);
  drawChart($("c_gyro"), ["gx","gy","gz"],       ["#3b82f6","#22c55e","#eab308"], ["p","q","r"]);
  drawChart($("c_duty"), ["m1","m2","m3","m4"],  ["#3b82f6","#22c55e","#eab308","#ef4444"], ["M1","M2","M3","M4"]);
};
es.onerror = ()=>{ $("meta").textContent = "connection lost — retrying..."; };
</script></body></html>
"""


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *fmt_args):          # silence per-request logging
        pass                                        # リクエスト毎ログを抑止

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            body = _PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
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


def serve(http_port: int, telemetry_port: int, open_browser: bool) -> int:
    """Run the UDP->SSE proxy + page server (blocks until Ctrl-C).
    UDP→SSE プロキシ＋ページサーバを起動（Ctrl-C まで実行）。"""
    threading.Thread(target=_udp_listener, args=(telemetry_port,),
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
