# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
# Part of StampFly Ecosystem (SILS GUI backend).
"""Local web server for the SILS GUI — scenario authoring, runs, graphs, 3D playback.

SILS GUI のローカル Web サーバ — シナリオ作成・実行・グラフ・3D 再生。

Almost zero external dependencies: a ThreadingHTTPServer (stdlib only) serves the static
single-page app and a small JSON API. The API reuses the existing pipeline — it shells out
to ``sf sils scenario`` exactly as the command line does, then reads the run bundle
(the flight-log v1 ``sils_*.sflog.zip`` bundle, via ``lib/sflog`` — see
docs/plans/flight-log-format-plan.md section 3.3 — plus results.json / events.jsonl) back
for the browser. Parameter overrides go through the new ``SILS_EMU_PARAMS_FILE`` hook so the
panel feeds a run without a rebuild.

依存はほぼゼロ（ThreadingHTTPServer は標準ライブラリのみ）。静的 SPA と小さな JSON API を配信。
API は既存パイプライン（`sf sils scenario`）をそのまま叩き、走行バンドル（フライトログ v1 一式
`sils_*.sflog.zip`。`lib/sflog` 経由 — 計画書 3.3節参照 — に加え results.json / events.jsonl）を
読み戻してブラウザへ返す。パラメータ上書きは `SILS_EMU_PARAMS_FILE` 経由（再ビルド不要）。

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

# lib/sflog reads the flight-log v1 bundles the SILS emulator now writes per run
# (docs/plans/flight-log-format-plan.md section 3.3, replacing the old flat per-run
# CSV file). `sf sils gui` imports this module in-process with whatever interpreter
# ran `sf` -- normally the project's own venv, which already has sflog/pandas
# importable (pyproject.toml lists pandas/numpy/PyYAML) -- but this sys.path
# insertion is a defensive fallback for an interpreter that does not.
# lib/sflog は SILS エミュレータが実行ごとに書くフライトログ v1 一式を読む
# （計画書 3.3節、旧・実行ごとの平坦な CSV ファイルの後継）。`sf sils gui` は `sf` を
# 起動した Python で本モジュールを in-process import する -- 通常はプロジェクト
# 自身の venv で sflog/pandas は import 可能（pyproject.toml に pandas/numpy/PyYAML
# 記載）だが、そうでない実行環境向けの保険としてパスを追加する。
LIB_DIR = ROOT / "lib"
if str(LIB_DIR) not in sys.path:
    sys.path.insert(0, str(LIB_DIR))
import sflog                     # noqa: E402  flight-log v1 bundle reader
import numpy as np               # noqa: E402  quaternion/euler math (truth -> legacy columns)
import pandas as pd              # noqa: E402  merge_asof (estimate/motor alignment onto truth)

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
        for line in p.read_text(encoding="utf-8").splitlines():
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
    text = p.read_text(encoding="utf-8")
    events = scnmod.parse_scn(text)
    expect = (SCN_DIR / f"{name}.expect")
    return {
        "name": name,
        "text": text,
        "events": events,
        "duration_us": scnmod.estimate_duration_us(events),
        "has_expect": expect.exists(),
    }


# =========================================================================================
# Flight-log v1 bundle -> legacy trajectory-table adapter (plan section 3.3)
# フライトログ v1 一式 -> 旧トラジェクトリ表アダプタ（計画書 3.3節）
# =========================================================================================
# The SILS emulator writes one flight-log v1 bundle per run (a `sils_*.sflog.zip` in the
# run's bundle dir) instead of the old flat per-run CSV file. app.js (the browser
# frontend) is UNCHANGED by this migration -- it still consumes {"columns": [...],
# "data": {col: [...]}} with the old 20 legacy column names (t, px, py, pz, qw, qx, qy,
# qz, alt, roll, pitch, yawrate, yawcmd, alt_est, roll_est, pitch_est, m0..m3).
# read_flightlog() below is the read-side adapter that derives those columns from the
# bundle's `truth`/`attitude`/`posvel`/`motor` streams (protocol/spec/flight_log.yaml),
# so app.js needs no changes.
# SILS エミュレータは旧・実行ごとの平坦な CSV ファイルの代わりに、実行ごとに1つの
# フライトログ v1 一式（走行バンドル内の `sils_*.sflog.zip`）を書く。app.js
# （ブラウザ側フロントエンド）はこの移行で無改修 -- 旧20列名の {"columns": [...],
# "data": {col: [...]}} をそのまま消費し続ける。以下の read_flightlog() が、一式の
# `truth`/`attitude`/`posvel`/`motor` ストリーム（protocol/spec/flight_log.yaml）から
# その列を導出する読み込み側アダプタ。

RAD_TO_DEG = 180.0 / np.pi
_SQRT_HALF = 0.7071067811865476  # 1/sqrt(2)


def _find_latest_bundle_zip(bundle_dir: Path):
    """Newest `sils_*.sflog.zip` under a SILS run's bundle dir, or None if the
    run has not produced one yet (mirrors this module's old CSV-file-based
    reader's "file missing" tolerance).
    SILS 実行のバンドルディレクトリ直下にある最新の `sils_*.sflog.zip`
    （まだ無ければ None -- このモジュールの旧・CSV ファイル読み込み処理の
    「ファイル無し」許容に合わせる）。
    """
    candidates = sorted(bundle_dir.glob("sils_*.sflog.zip"), key=lambda p: p.stat().st_mtime)
    return candidates[-1] if candidates else None


def _euler_from_quat_ned(qw, qx, qy, qz):
    """roll/pitch/yaw [rad] from a body(FRD)->NED quaternion (w,x,y,z), vectorized
    over numpy arrays. Standard aerospace 3-2-1 Euler extraction; pitch is clipped
    to arcsin's domain to absorb float round-off near +-90 deg (gimbal lock).
    機体(FRD)→NED のクォータニオンから roll/pitch/yaw [rad]（numpy 配列対応）。
    標準的な航空 3-2-1 オイラー角抽出。pitch は arcsin の定義域にクランプする
    （±90度付近の浮動小数点丸め対策）。
    """
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def _ned_to_mujoco_pos(pos_x, pos_y, pos_z):
    """StampFly NED position -> MuJoCo/ENU world position (px, py, pz). The exact
    inverse of simulator/sils/frames/frames.hpp's `ned_to_enu()` ({n.y, n.x, -n.z}),
    reimplemented here because this Python reader does not link the C++ frames module.
    StampFly の NED 位置 -> MuJoCo/ENU 世界座標 (px, py, pz)。frames.hpp の
    `ned_to_enu()`（{n.y, n.x, -n.z}）そのままの逆変換 -- この Python 読み込み側は
    C++ の frames モジュールをリンクしないため、ここに再実装する。
    """
    return pos_y, pos_x, -pos_z


def _quat_mul(w1, x1, y1, z1, w2, x2, y2, z2):
    """Hamilton product (w1,x1,y1,z1) * (w2,x2,y2,z2), vectorized."""
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return w, x, y, z


def _qnb_to_mujoco_quat(qw, qx, qy, qz):
    """StampFly q_nb (body FRD -> world NED) -> MuJoCo framequat (body FLU -> world
    ENU), vectorized. The exact inverse of frames.hpp's `qnb_to_mujoco_quat()`:
        q_mj = q_enu_to_ned().conj() * q_nb * q_frd_to_flu().conj()   (normalized)
    where q_enu_to_ned = (w=0, x=y=1/sqrt(2), z=0) and q_frd_to_flu = (w=0, x=1,
    y=z=0). Reimplemented in plain numpy since this reader does not link the C++
    frames module.
    StampFly の q_nb（機体FRD→世界NED）-> MuJoCo framequat（機体FLU→世界ENU）、
    配列対応。frames.hpp の `qnb_to_mujoco_quat()` そのままの逆変換（式は英語側参照）
    -- この読み込み側は C++ frames モジュールをリンクしないため素の numpy で再実装。
    """
    ew, ex, ey, ez = 0.0, _SQRT_HALF, _SQRT_HALF, 0.0            # q_enu_to_ned
    ew_c, ex_c, ey_c, ez_c = ew, -ex, -ey, -ez                    # its conjugate
    fw, fx, fy, fz = 0.0, 1.0, 0.0, 0.0                           # q_frd_to_flu
    fw_c, fx_c, fy_c, fz_c = fw, -fx, -fy, -fz                    # its conjugate
    w1, x1, y1, z1 = _quat_mul(ew_c, ex_c, ey_c, ez_c, qw, qx, qy, qz)
    w2, x2, y2, z2 = _quat_mul(w1, x1, y1, z1, fw_c, fx_c, fy_c, fz_c)
    norm = np.sqrt(w2 * w2 + x2 * x2 + y2 * y2 + z2 * z2)
    return w2 / norm, x2 / norm, y2 / norm, z2 / norm


def _asof_nearest(base: pd.DataFrame, other, col: str):
    """merge_asof(nearest) `col` from `other` onto `base`'s timestamp_us; None if
    `other` is absent (the stream is missing from the bundle), else a numpy float
    array (NaN where no match exists, e.g. before that stream's first sample).
    `other` の `col` 列を `base` の timestamp_us へ merge_asof(nearest) で結合する。
    `other` が無い（バンドルにそのストリームが無い）場合は None、それ以外は numpy
    配列（対応が無い箇所は NaN。例: そのストリームの最初の標本より前）。
    """
    if other is None or col not in other.columns:
        return None
    other_sorted = other[["timestamp_us", col]].sort_values("timestamp_us", kind="stable")
    merged = pd.merge_asof(base[["timestamp_us"]], other_sorted, on="timestamp_us",
                            direction="nearest")
    return merged[col].to_numpy()


def _attitude_estimate_roll_pitch_deg(base: pd.DataFrame, attitude):
    """Estimated roll/pitch [deg] from the `attitude` stream's ESKF quaternion,
    matched onto `base`'s timeline by merge_asof(nearest); (None, None) if the
    stream is absent from the bundle.
    `attitude` ストリームの ESKF クォータニオンから推定 roll/pitch [deg]
    （`base` の時間軸へ merge_asof(nearest) で対応付け）。バンドルにストリームが
    無ければ (None, None)。
    """
    if attitude is None:
        return None, None
    cols = ["timestamp_us", "quat_w", "quat_x", "quat_y", "quat_z"]
    other_sorted = attitude[cols].sort_values("timestamp_us", kind="stable")
    merged = pd.merge_asof(base[["timestamp_us"]], other_sorted, on="timestamp_us",
                            direction="nearest")
    roll, pitch, _yaw = _euler_from_quat_ned(
        merged["quat_w"].to_numpy(), merged["quat_x"].to_numpy(),
        merged["quat_y"].to_numpy(), merged["quat_z"].to_numpy(),
    )
    return roll * RAD_TO_DEG, pitch * RAD_TO_DEG


def _to_json_list(arr, n: int):
    """numpy array -> plain list with NaN replaced by None (valid JSON `null`,
    unlike a bare NaN literal); an all-None list of length `n` when `arr` is None
    (the source stream is absent from the bundle).
    numpy 配列 -> NaN を None（正しい JSON の null。裸の NaN リテラルは不可）に
    置換した素のリスト。`arr` が None（元ストリームがバンドルに無い）なら長さ `n`
    の None 列。
    """
    if arr is None:
        return [None] * n
    return [None if v != v else float(v) for v in arr]  # v != v <=> NaN


def read_flightlog(bundle_dir: Path):
    """Load a SILS run's flight-log v1 bundle and return it in the legacy
    {"columns": [...], "data": {col: [...]}} shape app.js already consumes.
    Empty dict if the run has not produced a bundle yet (mirrors this module's
    old CSV-file-based reader's "file missing" behaviour) or the bundle has no
    `truth` stream (SILS-only; without it there is no timeline to build the
    table on).

    `t` is seconds (timestamp_us / 1e6); `px/py/pz`/`qw..qz` are in the MuJoCo/ENU
    frame the 3D view expects (derived from `truth`'s NED position/attitude via
    the inverse of frames.hpp's transforms, see the helpers above); `roll`/`pitch`/
    `roll_est`/`pitch_est` are DEGREES -- a display-only conversion, since app.js
    labels them "[deg]" (app.js:228) even though the bundle itself stores radians;
    `yawcmd` is always 0.0 (retired -- plan section 3.3, it was always 0 in
    emulator runs); `alt_est`/`roll_est`/`pitch_est`/`m0..m3` are merge_asof
    (nearest) matches from `posvel`/`attitude`/`motor` onto `truth`'s timeline,
    None (JSON null) wherever that source stream is absent from the bundle
    (e.g. `motor` on a workshop-target run) -- app.js tolerates null in these via
    `|| 0` (three.js prop spin) or a Plotly gap (the graphs).

    SILS 実行のフライトログ v1 一式を読み、app.js が既に消費している旧形状
    {"columns": [...], "data": {col: [...]}} で返す。実行がまだ一式を書いていない
    （このモジュールの旧・CSV ファイル読み込み処理の「ファイル無し」挙動と同じ）か、
    一式に `truth` ストリーム（SILS 専用。無ければ表の時間軸が組めない）が無ければ
    空の dict。

    `t` は秒（timestamp_us / 1e6）。`px/py/pz`/`qw..qz` は 3D 表示が期待する
    MuJoCo/ENU 座標系（`truth` の NED 位置・姿勢から frames.hpp の変換の逆で導出。
    上のヘルパー参照）。`roll`/`pitch`/`roll_est`/`pitch_est` は度 -- 一式自体は
    ラジアンで持つが、app.js が "[deg]" とラベルしている（app.js:228）ための
    表示専用変換。`yawcmd` は常に 0.0（廃止済み -- 計画書3.3節、エミュレータ実行では
    元々常に0だった）。`alt_est`/`roll_est`/`pitch_est`/`m0..m3` は `posvel`/
    `attitude`/`motor` を `truth` の時間軸へ merge_asof(nearest) で対応付けた値で、
    元ストリームが一式に無ければ None（JSON の null。例: workshop ターゲット実行の
    `motor`）-- app.js はこれらの null を `|| 0`（three.js のプロペラ回転）や
    Plotly のグラフの欠落として許容する。
    """
    zip_path = _find_latest_bundle_zip(bundle_dir)
    if zip_path is None:
        return {}
    log = sflog.load(zip_path)
    truth = log.streams.get("truth")
    if truth is None or len(truth) == 0:
        return {}
    truth = truth.sort_values("timestamp_us", kind="stable").reset_index(drop=True)
    n = len(truth)

    t_us = truth["timestamp_us"].to_numpy()
    qw, qx, qy, qz = (truth[c].to_numpy() for c in ("quat_w", "quat_x", "quat_y", "quat_z"))
    roll, pitch, _yaw = _euler_from_quat_ned(qw, qx, qy, qz)
    px, py, pz = _ned_to_mujoco_pos(*(truth[c].to_numpy() for c in ("pos_x", "pos_y", "pos_z")))
    mj_qw, mj_qx, mj_qy, mj_qz = _qnb_to_mujoco_quat(qw, qx, qy, qz)

    alt_est = _asof_nearest(truth, log.streams.get("posvel"), "pos_z")
    if alt_est is not None:
        alt_est = -alt_est
    roll_est, pitch_est = _attitude_estimate_roll_pitch_deg(truth, log.streams.get("attitude"))

    motor = log.streams.get("motor")
    motor_cols = ("duty_FR", "duty_RR", "duty_RL", "duty_FL")

    data = {
        "t": (t_us / 1e6).tolist(),
        "px": px.tolist(), "py": py.tolist(), "pz": pz.tolist(),
        "qw": mj_qw.tolist(), "qx": mj_qx.tolist(), "qy": mj_qy.tolist(), "qz": mj_qz.tolist(),
        "alt": (-truth["pos_z"].to_numpy()).tolist(),
        "roll": (roll * RAD_TO_DEG).tolist(), "pitch": (pitch * RAD_TO_DEG).tolist(),
        "yawrate": truth["rate_z"].to_numpy().tolist(),
        "yawcmd": [0.0] * n,
        "alt_est": _to_json_list(alt_est, n),
        "roll_est": _to_json_list(roll_est, n), "pitch_est": _to_json_list(pitch_est, n),
    }
    for i, col in enumerate(motor_cols):
        data[f"m{i}"] = _to_json_list(_asof_nearest(truth, motor, col), n)

    return {"columns": list(data.keys()), "data": data}


def read_event_timeline(path: Path):
    """events.jsonl → list of meaningful markers (wind/fault/handle/bias notes), with
    seconds. Skips the high-rate espnow rc stream (not useful as markers).
    意味のあるマーカー（wind/fault/handle/bias）だけ抽出。高頻度 rc 列は除く。"""
    if not path.exists():
        return []
    out = []
    for line in path.read_text(encoding="utf-8").splitlines():
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
        SCRATCH_SCN.write_text(text, encoding="utf-8")
        scn_path = SCRATCH_SCN

    # 2. Parameter overrides → scratch file → SILS_EMU_PARAMS_FILE.
    env = dict(os.environ)
    if params:
        lines = ["# SILS GUI parameter overrides"]
        for k, v in params.items():
            lines.append(f"{k} {v}")
        SCRATCH_PARAMS.write_text("\n".join(lines) + "\n", encoding="utf-8")
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
                                  text=True, encoding="utf-8", timeout=300)
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
                results = json.loads(rj.read_text(encoding="utf-8"))
            except ValueError:
                results = {}
        traj = read_flightlog(bundle)
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
            (SCN_DIR / f"{name}.scn").write_text(text, encoding="utf-8")
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
