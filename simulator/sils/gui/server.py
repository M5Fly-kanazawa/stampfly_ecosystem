# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
# Part of StampFly Ecosystem (SILS GUI backend).
"""Local web server for the SILS GUI — scenario authoring, runs, graphs, 3D playback.

SILS GUI のローカル Web サーバ — シナリオ作成・実行・グラフ・3D 再生。

Zero external dependencies (Python stdlib only): a ThreadingHTTPServer serves the static
single-page app and a small JSON API. The API reuses the existing pipeline — it shells out
to ``sf sils scenario`` exactly as the command line does, then reads the run bundle
(trajectory.csv / results.json / events.jsonl) back for the browser. Parameter overrides go
through the new ``SILS_EMU_PARAMS_FILE`` hook so the panel feeds a run without a rebuild.

依存ゼロ（標準ライブラリのみ）。静的 SPA と小さな JSON API を配信。API は既存パイプライン
（`sf sils scenario`）をそのまま叩き、走行バンドルを読み戻してブラウザへ返す。パラメータ上書きは
`SILS_EMU_PARAMS_FILE` 経由（再ビルド不要）。

Launched by ``sf sils gui`` (lib/sfcli/commands/sils.py). Not a public service — binds
localhost only. `sf sils gui` から起動。localhost のみ。
"""

import json
import os
import subprocess
import sys
import tempfile
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

sys.path.insert(0, str(Path(__file__).resolve().parent))
import scn as scnmod              # noqa: E402  .scn parse / generate
import params_meta               # noqa: E402  params.cpp table → metadata

# --- Repo-relative paths (this file is simulator/sils/gui/server.py) ----------------------
GUI_DIR = Path(__file__).resolve().parent
STATIC = GUI_DIR / "static"
SILS_DIR = GUI_DIR.parent                       # simulator/sils
ROOT = SILS_DIR.parents[1]                       # repo root
SCN_DIR = SILS_DIR / "scenarios"
VIZ_DIR = SILS_DIR / "viz"
PARAMS_CPP = ROOT / "firmware/vehicle/components/sf_core/params.cpp"
# The 3D view renders the SAME STL meshes the MuJoCo model uses (the sim's authoritative
# StampFly geometry). 3D は MuJoCo モデルと同じ STL（sim 権威の StampFly 形状）を描く。
MESH_DIR = ROOT / "simulator/shared/assets/meshes/parts"

# Reusable scratch paths for ad-hoc (builder) runs and param overrides.
# ビルダー走行・パラメータ上書き用の使い回しスクラッチ。
SCRATCH_SCN = Path(tempfile.gettempdir()) / "sf_gui_run.scn"
SCRATCH_PARAMS = Path(tempfile.gettempdir()) / "sf_gui_params.txt"

# Single-flight lock so two browser tabs can't launch overlapping runs (the bundle dir and
# scratch files are shared). 2タブの同時実行を防ぐ単一実行ロック。
_RUN_LOCK = threading.Lock()


# =========================================================================================
# Scenario / params helpers
# =========================================================================================
def list_scenarios():
    """All *.scn with their first comment line as a one-line description."""
    out = []
    for p in sorted(SCN_DIR.glob("*.scn")):
        desc = ""
        for line in p.read_text().splitlines():
            s = line.strip()
            if s.startswith("#"):
                desc = s.lstrip("# ").strip()
                break
        out.append({"name": p.stem, "desc": desc})
    return out


def load_scenario(name: str):
    p = SCN_DIR / f"{name}.scn"
    if not p.exists():
        return None
    text = p.read_text()
    events = scnmod.parse_scn(text)
    expect = (SCN_DIR / f"{name}.expect")
    return {
        "name": name,
        "text": text,
        "events": events,
        "duration_us": scnmod.estimate_duration_us(events),
        "has_expect": expect.exists(),
    }


def read_trajectory(path: Path):
    """trajectory.csv → {columns:[...], data:{col:[...]}}. Empty dict if missing."""
    if not path.exists():
        return {}
    import csv
    with open(path) as f:
        rows = list(csv.reader(f))
    if not rows:
        return {}
    cols = rows[0]
    data = {c: [] for c in cols}
    for r in rows[1:]:
        for c, v in zip(cols, r):
            try:
                data[c].append(float(v))
            except ValueError:
                data[c].append(0.0)
    return {"columns": cols, "data": data}


def read_event_timeline(path: Path):
    """events.jsonl → list of meaningful markers (wind/fault/handle/bias notes), with
    seconds. Skips the high-rate espnow rc stream (not useful as markers).
    意味のあるマーカー（wind/fault/handle/bias）だけ抽出。高頻度 rc 列は除く。"""
    if not path.exists():
        return []
    out = []
    for line in path.read_text().splitlines():
        try:
            ev = json.loads(line)
        except ValueError:
            continue
        ch = ev.get("ch", "")
        if ch in ("espnow", "console"):
            continue
        out.append({"t": ev.get("t_us", 0) / 1e6, "ch": ch,
                    "note": ev.get("note", ev.get("text", ""))})
    return out


# =========================================================================================
# Run a scenario (shells out to `sf sils scenario`, same as the CLI)
# =========================================================================================
def run_scenario(req: dict):
    """req: {mode:'saved'|'custom', name?, events?, target, duration_us, noise, seed,
             battery, params:{name:value}}. Returns the run result dict."""
    target = req.get("target", "vehicle")
    duration_us = int(req.get("duration_us") or 25_000_000)
    noise = req.get("noise", "off") or "off"
    seed = int(req.get("seed", 12345))
    battery = bool(req.get("battery", True))
    params = req.get("params") or {}

    # 1. Resolve the .scn to run. A saved scenario runs from its real path so its .expect
    #    applies; a custom (builder) scenario is generated into a scratch file (no .expect →
    #    verdict is exit-code only). 保存済みは実パス（.expect 適用）、カスタムは scratch。
    mode = req.get("mode", "custom")
    if mode == "saved" and req.get("name"):
        scn_path = SCN_DIR / f"{req['name']}.scn"
        if not scn_path.exists():
            return {"error": f"scenario not found: {req['name']}"}
    else:
        text = scnmod.generate_scn(req.get("events", []),
                                   header="generated by SILS GUI (scratch run)")
        SCRATCH_SCN.write_text(text)
        scn_path = SCRATCH_SCN

    # 2. Parameter overrides → scratch file → SILS_EMU_PARAMS_FILE.
    env = dict(os.environ)
    if params:
        lines = ["# SILS GUI parameter overrides"]
        for k, v in params.items():
            lines.append(f"{k} {v}")
        SCRATCH_PARAMS.write_text("\n".join(lines) + "\n")
        env["SILS_EMU_PARAMS_FILE"] = str(SCRATCH_PARAMS)
    if not battery:
        env["SILS_EMU_BATTERY"] = "off"

    # 3. Run via the CLI (identical path to the command line). `sf` is on PATH because the
    #    GUI was launched from the sourced env. 環境を継承して sf を起動。
    cmd = ["sf", "sils", "scenario", str(scn_path), "--target", target,
           "--duration", str(duration_us), "--noise", noise, "--seed", str(seed)]
    with _RUN_LOCK:
        try:
            proc = subprocess.run(cmd, cwd=str(ROOT), env=env,
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  text=True, timeout=300)
        except FileNotFoundError:
            return {"error": "`sf` not found on PATH — launch the GUI from a shell where "
                             "`source setup_env.sh` has run."}
        except subprocess.TimeoutExpired:
            return {"error": "run timed out (>300 s)"}

        # 4. Read the bundle the run produced (stem = scn file stem).
        bundle = VIZ_DIR / f"out_scn_{scn_path.stem}"
        results = {}
        rj = bundle / "results.json"
        if rj.exists():
            try:
                results = json.loads(rj.read_text())
            except ValueError:
                results = {}
        traj = read_trajectory(bundle / "trajectory.csv")
        timeline = read_event_timeline(bundle / "events.jsonl")

    cli_tail = "\n".join(proc.stdout.splitlines()[-40:]) if proc.stdout else ""
    return {
        "ok": proc.returncode == 0,
        "exit_code": proc.returncode,
        "results": results,
        "trajectory": traj,
        "timeline": timeline,
        "cli_tail": cli_tail,
        "bundle": str(bundle.relative_to(ROOT)),
    }


# =========================================================================================
# HTTP handler
# =========================================================================================
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass   # quiet — no per-request console spam / リクエストログ抑制

    def _send_json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path: Path, ctype: str):
        if not path.exists():
            self.send_error(404, "not found")
            return
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        path, q = u.path, parse_qs(u.query)
        if path in ("/", "/index.html"):
            return self._send_file(STATIC / "index.html", "text/html; charset=utf-8")
        if path == "/app.js":
            return self._send_file(STATIC / "app.js", "application/javascript; charset=utf-8")
        if path == "/style.css":
            return self._send_file(STATIC / "style.css", "text/css; charset=utf-8")
        if path.startswith("/mesh/"):
            # Serve a StampFly STL part for the 3D view. Whitelist the name (no traversal).
            # 3D 用 STL パーツを配信。名前を制限しパストラバーサルを防ぐ。
            name = path[len("/mesh/"):]
            import re as _re
            if not _re.fullmatch(r"[a-z0-9_]+\.stl", name):
                return self.send_error(400, "bad mesh name")
            return self._send_file(MESH_DIR / name, "model/stl")
        if path == "/api/scenarios":
            return self._send_json(list_scenarios())
        if path == "/api/scenario":
            name = q.get("name", [""])[0]
            data = load_scenario(name)
            return self._send_json(data) if data else self._send_json(
                {"error": "not found"}, 404)
        if path == "/api/params":
            return self._send_json(params_meta.parse_params(PARAMS_CPP))
        self.send_error(404, "not found")

    def do_POST(self):
        u = urlparse(self.path)
        length = int(self.headers.get("Content-Length", 0))
        try:
            req = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            return self._send_json({"error": "bad JSON"}, 400)

        if u.path == "/api/run":
            return self._send_json(run_scenario(req))
        if u.path == "/api/save":
            text = scnmod.generate_scn(req.get("events", []), header=req.get("header", ""))
            # dry → just return the generated .scn text (the builder's "show .scn" preview);
            # do NOT touch the scenarios dir. dry はプレビュー（書き込まない）。
            if req.get("dry"):
                return self._send_json({"ok": True, "text": text})
            name = req.get("name", "").strip()
            if not name or "/" in name or name.startswith("."):
                return self._send_json({"error": "invalid scenario name"}, 400)
            (SCN_DIR / f"{name}.scn").write_text(text)
            return self._send_json({"ok": True, "path": f"scenarios/{name}.scn",
                                    "text": text})
        self.send_error(404, "not found")


def serve(port: int = 8765, open_browser: bool = True):
    httpd = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    url = f"http://127.0.0.1:{port}/"
    print(f"[sf sils gui] serving at {url}")
    print("[sf sils gui]  scenarios :", SCN_DIR)
    print("[sf sils gui]  Ctrl-C to stop")
    if open_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[sf sils gui] stopped")
        httpd.shutdown()


if __name__ == "__main__":
    serve(int(sys.argv[1]) if len(sys.argv) > 1 else 8765)
