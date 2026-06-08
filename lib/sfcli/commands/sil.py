"""
sf sil - Software-in-the-Loop bench (physics-based, MuJoCo, algorithm-independent)

物理ベースの SIL ベンチを操作する。ファームの本物ループをホストで走らせ、MuJoCo で
ループを閉じ、機械可読な合否(results.json)＋レビュー動画を成果物として出す。
アウトプット主導のマイルストーン(RESET_PLAN §8〜§10)を CLI から回す。

Subcommands:
  build      Build the host SIL (compat + firmware sources + MuJoCo)
  run        Run the closed loop and write the bundle (trajectory + results.json)
  video      Render the review video (MuJoCo 3D + state graphs)
  status     Show the machine verdict (results.json) for a milestone
  gate       Gate check: bundle complete AND verdict passes (output-driven)
  scenario   Run a *.scn input scenario (console/ESP-NOW) and assert outputs (E6)
  milestone  build → run → video → gate in one shot (the /sil-milestone skill)
"""

import argparse
import json
import math
import os
import shlex
import subprocess
import sys
from pathlib import Path

from ..utils import console, paths

COMMAND_NAME = "sil"
COMMAND_HELP = "Software-in-the-Loop bench (closed-loop hover, review video, gate)"

ESTIMATORS = {"eskf": 0, "complementary": 1}
ESTIMATOR_LABELS = {"eskf": "ESKF", "complementary": "Complementary"}
NOISE_LEVELS = ["off", "n0", "n1", "n2"]
# Default sensor-noise level per milestone (RESET_PLAN §13): noise milestones run N0.
# マイルストーン別の既定ノイズ（§13）: ノイズ系マイルストーンは N0 で走る。
MILESTONE_NOISE = {"P5": "n0", "P6": "n0", "P7": "n0"}


# --- path helpers / パスヘルパ -------------------------------------------------
def _sil_dir() -> Path:
    return paths.root() / "simulator" / "sil"


def _model() -> Path:
    return _sil_dir() / "models" / "stampfly.xml"


def _build_dir() -> Path:
    return _sil_dir() / "build"


def _bundle_dir(milestone: str) -> Path:
    return _sil_dir() / "viz" / f"out_{milestone.lower()}"


def _venv_python() -> Path:
    return _sil_dir() / "viz" / "venv" / "bin" / "python"


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register the sil command with the CLI."""
    parser = subparsers.add_parser(COMMAND_NAME, help=COMMAND_HELP, description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="sil_command", metavar="<subcommand>")

    p = sub.add_parser("build", help="Build the host SIL")
    p.add_argument("-j", "--jobs", type=int, default=8)
    p.add_argument("-t", "--target", default=None, help="cmake target (default: all)")
    p.set_defaults(func=run_build)

    p = sub.add_parser("run", help="Run the closed loop and write the bundle")
    p.add_argument("-m", "--milestone", default="P1")
    p.add_argument("-e", "--estimator", choices=list(ESTIMATORS), default="eskf")
    p.add_argument("--noise", choices=NOISE_LEVELS, default="off", help="sensor noise level (§13)")
    p.add_argument("--seed", type=int, default=12345, help="noise RNG seed (determinism)")
    p.set_defaults(func=run_run)

    p = sub.add_parser("video", help="Render the review video for a milestone bundle")
    p.add_argument("-m", "--milestone", default="P1")
    p.add_argument("--fps", type=int, default=50)
    p.set_defaults(func=run_video)

    p = sub.add_parser("status", help="Show the machine verdict (results.json)")
    p.add_argument("-m", "--milestone", default="P1")
    p.set_defaults(func=run_status)

    p = sub.add_parser("gate", help="Gate check: bundle complete and verdict passes")
    p.add_argument("-m", "--milestone", default="P1")
    p.set_defaults(func=run_gate)

    # E6: run a deterministic *.scn input scenario and assert the firmware output.
    # E6: 決定論的な *.scn 入力シナリオを走らせ、ファーム出力をアサートする。
    p = sub.add_parser("scenario", help="Run a *.scn input scenario and assert outputs (E6)")
    p.add_argument("scenario", help="path to the .scn scenario file")
    p.add_argument("--target", choices=["vehicle", "vehicle_new"], default="vehicle",
                   help="emulator binary (default: vehicle = old firmware)")
    p.add_argument("--expect", default=None,
                   help="assertions file (default: <scenario>.expect if it exists)")
    p.add_argument("--duration", type=int, default=25_000_000,
                   help="sim duration in microseconds (default 25 s)")
    p.add_argument("--noise", choices=NOISE_LEVELS, default="off",
                   help="sensor noise level on the emulator Plant (§13 P5; default off)")
    p.add_argument("--seed", type=int, default=12345,
                   help="noise RNG seed (determinism: same seed → byte-identical run)")
    p.add_argument("--video", action="store_true",
                   help="on PASS, render a review MP4 (MuJoCo 3D + state graphs) from the run")
    p.add_argument("--unpaired", action="store_true",
                   help="boot the vehicle UNPAIRED (skip the SIL pairing NVS seed) so it "
                        "auto-enters Pairing and binds via the injected RC — exercises the "
                        "real pairing handshake (pairing.scn)")
    p.set_defaults(func=run_scenario)

    p = sub.add_parser("compare", help="Side-by-side ESKF vs complementary video (P4/P6)")
    p.add_argument("-m", "--milestone", default="P4")
    p.add_argument("--ea", choices=list(ESTIMATORS), default="eskf", help="run A estimator")
    p.add_argument("--eb", choices=list(ESTIMATORS), default="complementary", help="run B estimator")
    p.add_argument("--noise", choices=NOISE_LEVELS, default=None, help="sensor noise (default per milestone)")
    p.add_argument("--seed", type=int, default=12345, help="noise RNG seed (same for both runs → fair)")
    p.add_argument("--fps", type=int, default=50)
    p.set_defaults(func=run_compare)

    # Web GUI: scenario authoring + runs + interactive graphs + live 3D playback.
    # Web GUI: シナリオ作成＋実行＋インタラクティブグラフ＋ライブ 3D 再生。
    p = sub.add_parser("gui", help="Launch the SIL Web GUI (scenarios, graphs, 3D playback)")
    p.add_argument("--port", type=int, default=8765, help="HTTP port (default 8765)")
    p.add_argument("--no-browser", action="store_true",
                   help="do not auto-open the browser")
    p.set_defaults(func=run_gui)

    p = sub.add_parser("milestone", help="build → run → video → gate in one shot")
    p.add_argument("-m", "--milestone", default="P1")
    p.add_argument("-e", "--estimator", choices=list(ESTIMATORS), default="eskf")
    # default None → resolved per-milestone (MILESTONE_NOISE) unless the user overrides.
    # 既定 None → ユーザー指定が無ければマイルストーン別(MILESTONE_NOISE)で解決。
    p.add_argument("--noise", choices=NOISE_LEVELS, default=None, help="sensor noise level (§13)")
    p.add_argument("--seed", type=int, default=12345, help="noise RNG seed (determinism)")
    p.set_defaults(func=run_milestone)

    parser.set_defaults(func=lambda a: (parser.print_help(), 0)[1])


# --- handlers / ハンドラ -------------------------------------------------------
def run_build(args: argparse.Namespace) -> int:
    # getattr defaults so the milestone flow (which lacks -j/-t) can reuse this.
    # milestone フローは -j/-t を持たないので getattr で既定値化して再利用できるようにする。
    jobs = getattr(args, "jobs", 8)
    target = getattr(args, "target", None)
    sd, bd = _sil_dir(), _build_dir()
    console.info("Configuring SIL (cmake)...")
    r = subprocess.run(["cmake", "-S", str(sd), "-B", str(bd)])
    if r.returncode != 0:
        console.error("cmake configure failed"); return r.returncode
    console.info("Building SIL (first time fetches MuJoCo — can take minutes)...")
    cmd = ["cmake", "--build", str(bd), "-j", str(jobs)]
    if target:
        cmd += ["--target", target]
    r = subprocess.run(cmd)
    if r.returncode == 0:
        console.success("SIL build OK")
    return r.returncode


def run_run(args: argparse.Namespace) -> int:
    exe = _build_dir() / "hover_smoke"
    if not exe.exists():
        console.error("hover_smoke not built — run 'sf sil build' first"); return 1
    bundle = _bundle_dir(args.milestone)
    bundle.mkdir(parents=True, exist_ok=True)
    et = ESTIMATORS[args.estimator]
    noise = getattr(args, "noise", "off") or "off"
    seed = getattr(args, "seed", 12345)
    console.info(f"Running closed loop (milestone={args.milestone}, "
                 f"estimator={args.estimator}, noise={noise})...")
    # argv: model, bundle, estimator_type, milestone, noise_level, seed.
    # 引数: モデル, バンドル, 推定器種別, マイルストーン, ノイズ準位, シード。
    r = subprocess.run([str(exe), str(_model()), str(bundle), str(et),
                        str(args.milestone), noise, str(seed)])
    if r.returncode == 0:
        console.success(f"Bundle written to {bundle}")
    else:
        console.error(f"Closed loop FAILED (exit {r.returncode}) — see output / results.json")
    return 0  # the verdict lives in results.json; gate decides pass/fail


def _traj_metric(traj_path: Path, name: str, t0=None, t1=None):
    """Compute a physical-truth metric from trajectory.csv for the numerical gates
    (G2 estimate tracking, G3 bounded attitude/position, G4 actuator health). The
    expect DSL is log-only (≈G1); this turns the bundle's truth+estimate columns into
    machine-judgeable numbers. Returns None on a missing file / unknown metric / empty
    window. Optional [t0, t1] seconds restrict the metric to a flight phase (e.g. the
    POS_HOLD window). trajectory.csv から物理真値メトリクスを算出（G2/G3/G4）。expect の
    ログ判定(≈G1)に対し、真値＋推定列を機械判定可能な数値にする。
    """
    import csv as _csv
    try:
        with open(traj_path) as f:
            rows = list(_csv.DictReader(f))
    except OSError:
        return None
    def fv(r, k): return float(r[k])
    if t0 is not None and t1 is not None:
        rows = [r for r in rows if t0 <= fv(r, "t") <= t1]
    if not rows:
        return None
    def col(k): return [fv(r, k) for r in rows]
    def rms(xs): return math.sqrt(sum(x * x for x in xs) / len(xs))

    if name == "horizontal_drift_max":   # G3: max planar distance from the window's start
        cx, cy = fv(rows[0], "px"), fv(rows[0], "py")
        return max(math.hypot(fv(r, "px") - cx, fv(r, "py") - cy) for r in rows)
    if name == "roll_rmse":              # G2: est roll vs truth roll
        return rms([fv(r, "roll_est") - fv(r, "roll") for r in rows])
    if name == "pitch_rmse":             # G2: est pitch vs truth pitch
        return rms([fv(r, "pitch_est") - fv(r, "pitch") for r in rows])
    if name == "att_rmse":               # G2: combined roll+pitch attitude error magnitude
        return rms([math.hypot(fv(r, "roll_est") - fv(r, "roll"),
                               fv(r, "pitch_est") - fv(r, "pitch")) for r in rows])
    if name == "alt_rmse":               # G2: est alt vs truth alt
        return rms([fv(r, "alt_est") - fv(r, "alt") for r in rows])
    if name == "tilt_max":               # G3: max true tilt magnitude (no tumble)
        return max(math.hypot(fv(r, "roll"), fv(r, "pitch")) for r in rows)
    if name == "alt_band":               # G3: peak-to-peak altitude over the window
        a = col("alt"); return max(a) - min(a)
    if name == "alt_mean":
        a = col("alt"); return sum(a) / len(a)
    if name == "alt_min":  return min(col("alt"))
    if name == "alt_max":  return max(col("alt"))
    if name == "duty_max":               # G4: peak motor duty (saturation guard)
        return max(max(fv(r, "m0"), fv(r, "m1"), fv(r, "m2"), fv(r, "m3")) for r in rows)
    return None  # unknown metric name / 未知のメトリクス名


_METRIC_OPS = {
    "<":  lambda a, b: a < b,   "<=": lambda a, b: a <= b,
    ">":  lambda a, b: a > b,   ">=": lambda a, b: a >= b,
}


def _eval_expect(expect_path: Path, out_text: str, err_text: str, exit_code: int,
                 traj_path: Path = None):
    """Evaluate an assertions file against the captured output. Returns (checks,
    all_pass). Assertions are anchored to OUTPUT TEXT/ORDER, never wall-clock, so
    a check is deterministic exactly because the scenario's output is byte-
    identical across runs. 出力テキスト/順序にアンカー（壁時計不使用）＝決定論的。

      log_contains <out|err|any> "<text>"   stream contains the text
      log_absent   <out|err|any> "<text>"   stream does NOT contain the text
      order "<a>" "<b>"                       a's first occurrence precedes b's
      exit <code>                             process exit code matches
      skip <reason...>                        record a capability-gated check as
                                              SKIPPED (passes, not evaluated) —
                                              e.g. stable hover gated on E3 INA3221
      metric <name> <op> <value> [in <t0> <t1>]
                                              numerical physical-truth gate from
                                              trajectory.csv (G2/G3/G4). op ∈ < <= > >= ;
                                              optional "in t0 t1" restricts to a phase.
                                              names: horizontal_drift_max, roll_rmse,
                                              pitch_rmse, att_rmse, alt_rmse, tilt_max,
                                              alt_band, alt_mean, alt_min, alt_max, duty_max
    """
    merged = out_text + err_text
    streams = {"out": out_text, "err": err_text, "any": merged}
    checks = []
    for raw in expect_path.read_text().splitlines():
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("skip"):
            reason = stripped[len("skip"):].strip()
            checks.append({"name": f"skipped: {reason}", "pass": True, "skipped": True,
                           "detail": "capability-gated; not evaluated"})
            continue
        # Quote-aware tokenize that also strips an UNQUOTED trailing '#' comment,
        # so a '#' inside a quoted assertion text survives. A malformed line is
        # recorded as a failing check rather than crashing the command.
        # 引用符を尊重したトークン化（引用符外の '#' のみコメント除去）。不正行はクラッシュ
        # させず失敗チェックとして記録する。
        try:
            toks = shlex.split(raw, comments=True)
        except ValueError as e:
            checks.append({"name": f"bad assertion: {stripped!r}", "pass": False, "detail": str(e)})
            continue
        if not toks:
            continue
        kind = toks[0]
        if kind in ("log_contains", "log_absent") and len(toks) >= 3:
            stream, text = toks[1], toks[2]
            found = text in streams.get(stream, merged)
            ok = found if kind == "log_contains" else not found
            checks.append({"name": f"{kind} {stream} {text!r}", "pass": bool(ok),
                           "detail": "found" if found else "absent"})
        elif kind == "order" and len(toks) >= 3:
            a, b = toks[1], toks[2]
            ia, ib = merged.find(a), merged.find(b)
            ok = ia >= 0 and ib >= 0 and ia < ib
            checks.append({"name": f"order {a!r} before {b!r}", "pass": bool(ok),
                           "detail": f"idx_a={ia} idx_b={ib}"})
        elif kind == "exit" and len(toks) >= 2:
            try:
                want = int(toks[1])
            except ValueError:
                checks.append({"name": f"bad assertion: {stripped!r}", "pass": False,
                               "detail": "non-numeric exit code"})
                continue
            checks.append({"name": f"exit == {want}", "pass": exit_code == want,
                           "detail": f"got {exit_code}"})
        elif kind == "metric" and len(toks) >= 4:
            # metric <name> <op> <value> [in <t0> <t1>]
            # Numerical physical-truth gate from trajectory.csv (G2/G3/G4). The
            # optional "in <t0> <t1>" window restricts it to a flight phase.
            # trajectory.csv からの数値ゲート。"in t0 t1" で飛行フェーズに限定。
            name, op, valstr = toks[1], toks[2], toks[3]
            t0 = t1 = None
            if len(toks) >= 7 and toks[4] == "in":
                try:
                    t0, t1 = float(toks[5]), float(toks[6])
                except ValueError:
                    checks.append({"name": f"bad assertion: {stripped!r}", "pass": False,
                                   "detail": "non-numeric window"}); continue
            if op not in _METRIC_OPS:
                checks.append({"name": f"bad assertion: {stripped!r}", "pass": False,
                               "detail": f"unknown op {op!r}"}); continue
            try:
                want = float(valstr)
            except ValueError:
                checks.append({"name": f"bad assertion: {stripped!r}", "pass": False,
                               "detail": "non-numeric threshold"}); continue
            m = _traj_metric(traj_path, name, t0, t1) if traj_path else None
            win = f" in [{t0},{t1}]" if t0 is not None else ""
            if m is None:
                checks.append({"name": f"metric {name} {op} {want}{win}", "pass": False,
                               "detail": "no trajectory / unknown metric / empty window"})
            else:
                ok = _METRIC_OPS[op](m, want)
                checks.append({"name": f"metric {name} {op} {want}{win}", "pass": bool(ok),
                               "detail": f"{name}={m:.4f}"})
        else:
            checks.append({"name": f"bad assertion: {stripped!r}", "pass": False, "detail": "syntax"})

    # An .expect that evaluates NO real assertion (empty / all-comment / all-skip)
    # must FAIL, not pass vacuously (all([]) is True in Python).
    # 実評価が1つも無い .expect は空虚 PASS にせず FAIL させる（all([])==True 対策）。
    evaluated = [c for c in checks if not c.get("skipped")]
    if not evaluated:
        checks.append({"name": "no assertions evaluated", "pass": False,
                       "detail": "expect file has no evaluable assertion"})
    all_pass = all(c["pass"] for c in checks)
    return checks, all_pass


def run_scenario(args: argparse.Namespace) -> int:
    target = getattr(args, "target", "vehicle")
    exe = _build_dir() / ("emu_vehicle" if target == "vehicle" else "emu_vehicle_new")
    if not exe.exists():
        console.error(f"{exe.name} not built — run 'sf sil build' first"); return 1
    scn = Path(args.scenario)
    if not scn.exists():
        console.error(f"scenario not found: {scn}"); return 1

    bundle = _sil_dir() / "viz" / f"out_scn_{scn.stem}"
    bundle.mkdir(parents=True, exist_ok=True)
    events = bundle / "events.jsonl"
    traj = bundle / "trajectory.csv"
    # SIL_EMU_TRAJ tells the emulator to record a render_video.py-compatible
    # trajectory.csv into the bundle (so --video can render the run). Harmless and
    # deterministic; the recorder is a no-op in any emulator that ignores it.
    # SIL_EMU_TRAJ は render_video.py 互換の trajectory.csv をバンドルへ記録させる
    # （--video で描画可能に）。決定論的で無害、未対応エミュレータでは no-op。
    env = dict(os.environ, SIL_EMU_EVENTS=str(events), SIL_EMU_TRAJ=str(traj))

    # SIL_EMU_NOISE/SIL_EMU_SEED turn on the seeded N0 sensor-noise model on the
    # emulator Plant (§13 P5). Default "off" → env passed but the emulator keeps the
    # clean path (byte-identical), so the no-noise scenario verdict is unaffected.
    # SIL_EMU_NOISE/SIL_EMU_SEED でエミュレータ Plant の N0 ノイズを ON（§13 P5）。
    # 既定 "off" では従来のクリーン経路（byte-identical）を保つ。
    noise = getattr(args, "noise", "off") or "off"
    seed = getattr(args, "seed", 12345)
    env["SIL_EMU_NOISE"] = noise
    env["SIL_EMU_SEED"] = str(seed)

    # --unpaired: skip the pairing NVS seed so the vehicle boots unpaired and runs the
    # real pairing handshake (auto-enter Pairing → bind via injected RC). Default keeps
    # the vehicle pre-paired so flight scenarios are not gated on pairing timing.
    # --unpaired: ペアリング NVS seed をスキップし機体を未ペア起動させ、実ハンドシェイク
    # （自動 Pairing 突入→注入 RC で bind）を走らせる。既定はペア済み起動で飛行シナリオを
    # ペアリングのタイミングに依存させない。
    if getattr(args, "unpaired", False):
        env["SIL_EMU_UNPAIRED"] = "1"

    console.info(f"Running scenario {scn.name} on {exe.name} ({args.duration} us, "
                 f"noise={noise}, seed={seed})...")
    # The emulator reads stdin (the firmware CLI); feed /dev/null so any non-key
    # read yields EAGAIN. Capture stdout and stderr SEPARATELY (ESP_LOGx → stderr).
    # ファーム CLI の stdin は /dev/null。stdout/stderr を分離捕捉（ESP_LOGx は stderr）。
    with open(os.devnull) as devnull:
        r = subprocess.run([str(exe), str(_model()), str(args.duration), str(scn)],
                           stdin=devnull, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           text=True, env=env)
    (bundle / "console.out").write_text(r.stdout)
    (bundle / "console.err").write_text(r.stderr)
    (bundle / "console.log").write_text(r.stdout + r.stderr)

    # Guardrail: every scenario injects at least one event, so events.jsonl must
    # exist and be non-empty. A target that is not wired for scripted input (e.g.
    # an emulator entry that ignores argv[3]/SIL_EMU_EVENTS) would inject NOTHING
    # and could otherwise masquerade as a passing run on exit==0 — fail it loudly.
    # ガードレール: シナリオは必ず1事象以上注入するので events.jsonl は非空のはず。台本入力
    # 未配線のターゲット（argv[3]/SIL_EMU_EVENTS を無視）は何も注入せず exit==0 で偽合格しうる。
    injected = events.exists() and events.stat().st_size > 0
    inject_check = {"name": "input injected (events.jsonl non-empty)", "pass": injected,
                    "detail": f"{events.stat().st_size if events.exists() else 0} bytes"
                              + ("" if injected else f" — is {exe.name} wired for scripted input?")}

    expect = Path(args.expect) if args.expect else scn.with_suffix(".expect")
    if expect.exists():
        checks, verdict = _eval_expect(expect, r.stdout, r.stderr, r.returncode, traj)
    else:
        console.info(f"(no .expect at {expect.name} — verdict = injection + exit code)")
        checks = [{"name": "exit == 0", "pass": r.returncode == 0, "detail": f"got {r.returncode}"}]
        verdict = (r.returncode == 0)
    checks.insert(0, inject_check)
    verdict = bool(verdict and injected)

    results = {
        "gate": "scenario", "milestone": f"scn_{scn.stem}", "kind": "scenario",
        "scenario": str(scn), "target": target, "exit_code": r.returncode,
        "noise": noise, "seed": seed,
        "pass": bool(verdict), "checks": checks,
        # Accurate flight label for the review-video title (no takeoff/landing claim).
        # レビュー動画タイトル用の正確な飛行ラベル（離着陸を主張しない）。
        "flight": "scripted-input scenario",
    }
    (bundle / "results.json").write_text(json.dumps(results, indent=2) + "\n")

    console.info(f"scenario {scn.name}: {'PASS' if verdict else 'FAIL'} "
                 f"(exit {r.returncode}, {len(checks)} checks, events={events.name})")
    for c in checks:
        if c.get("skipped"):
            print(f"  [SKIP] {c['name']}")
        else:
            print(f"  [{'PASS' if c['pass'] else 'FAIL'}] {c['name']}  ({c.get('detail','')})")
    if verdict:
        console.success(f"bundle: {bundle}")
    else:
        console.error(f"scenario FAILED — see {bundle}/console.log")

    # --video: render a review MP4 (MuJoCo 3D + state graphs) from the trajectory the
    # run just recorded. Only on PASS and only if a trajectory was actually written.
    # --video: 実行が記録した軌跡からレビュー動画（MuJoCo 3D＋状態グラフ）を描画。
    # PASS かつ軌跡が書かれた場合のみ。
    if verdict and getattr(args, "video", False):
        py = _venv()
        if py is None:
            console.error("viz venv missing — cannot render (see simulator/sil/viz)")
        elif not (traj.exists() and traj.stat().st_size > 0):
            console.error(f"no trajectory.csv in {bundle} — is {exe.name} wired for "
                          "SIL_EMU_TRAJ recording?")
        else:
            out = bundle / f"scn_{scn.stem}.mp4"
            console.info("Rendering review video (MuJoCo 3D + state graphs)...")
            rv = subprocess.run([str(py), str(_sil_dir() / "viz" / "render_video.py"),
                                 "--model", str(_model()), "--bundle", str(bundle),
                                 "--out", str(out), "--fps", "50"])
            if rv.returncode == 0:
                console.success(f"Video: {out}")
            else:
                console.error("render_video.py failed")
    return 0 if verdict else 2


def run_gui(args: argparse.Namespace) -> int:
    """Launch the SIL Web GUI (simulator/sil/gui/server.py). Serves the single-page app and
    a small JSON API that drives `sf sil scenario` runs and reads the bundles back for the
    browser (graphs + live 3D). Localhost only. SIL Web GUI を起動。localhost のみ。"""
    gui_dir = _sil_dir() / "gui"
    server = gui_dir / "server.py"
    if not server.exists():
        console.error(f"GUI server not found: {server}")
        return 1
    sys.path.insert(0, str(gui_dir))
    import importlib
    mod = importlib.import_module("server")
    try:
        mod.serve(port=args.port, open_browser=not args.no_browser)
    except OSError as e:
        console.error(f"could not bind port {args.port}: {e} "
                      f"(try `sf sil gui --port <other>`)")
        return 1
    return 0


def run_video(args: argparse.Namespace) -> int:
    py = _venv()
    if py is None:
        return 1
    bundle = _bundle_dir(args.milestone)
    if not (bundle / "trajectory.csv").exists():
        console.error(f"No trajectory in {bundle} — run 'sf sil run' first"); return 1
    fps = getattr(args, "fps", 50)
    out = bundle / f"{args.milestone.lower()}_flight.mp4"
    console.info("Rendering review video (MuJoCo 3D + state graphs)...")
    r = subprocess.run([str(py), str(_sil_dir() / "viz" / "render_video.py"),
                        "--model", str(_model()), "--bundle", str(bundle),
                        "--out", str(out), "--fps", str(fps)])
    if r.returncode == 0:
        console.success(f"Video: {out}")
    return r.returncode


def run_compare(args: argparse.Namespace) -> int:
    # P4: run both estimators through the SAME flight and render a side-by-side
    # review video (twin 3D + overlay graphs), then write the aggregate verdict.
    # The bundle is unchanged between runs — that is the algorithm-independence
    # proof (RESET_PLAN P2/§9) turned into one shareable artifact.
    # P4: 同じ飛行で両推定器を走らせ並置レビュー動画を描く → 集約判定を書く。
    # ベンチは実行間で無改変 ＝ アルゴリズム非依存の実証を1本の共有素材に。
    if args.ea == args.eb:
        # Comparing an estimator to itself would falsely "prove" independence.
        # 同じ推定器同士の比較は非依存を偽証する。
        console.error(f"--ea and --eb must differ (both '{args.ea}') — a comparison "
                      "needs two distinct estimators"); return 1
    exe = _build_dir() / "hover_smoke"
    if not exe.exists():
        console.error("hover_smoke not built — run 'sf sil build' first"); return 1
    py = _venv()
    if py is None:
        return 1
    bundle = _bundle_dir(args.milestone)          # out_p4 / out_p6
    bundle.mkdir(parents=True, exist_ok=True)
    # Same noise level AND seed for both runs → identical noise realization → a fair
    # estimator contrast. Noise default resolves per milestone (P6 → n0).
    # 両実行に同じノイズ準位＋同じシード＝同一ノイズ実現で公平に比較。既定はマイルストーン別。
    noise = getattr(args, "noise", None)
    if noise is None:
        noise = MILESTONE_NOISE.get(str(args.milestone).upper(), "off")
    seed = getattr(args, "seed", 12345)
    runs = {}
    for est in (args.ea, args.eb):
        sub = bundle / est
        sub.mkdir(parents=True, exist_ok=True)
        console.info(f"Running closed loop for comparison ({est}, noise={noise})...")
        rc = subprocess.run([str(exe), str(_model()), str(sub), str(ESTIMATORS[est]),
                             f"{args.milestone}-{est}", noise, str(seed)]).returncode
        # hover_smoke writes the bundle even on a G3 fail; only a hard early exit
        # (e.g. model load) leaves no files. Require both before render/aggregate so
        # a crash surfaces here, not as an opaque traceback downstream.
        # G3不合格でもバンドルは書かれる。ファイルが無いのは早期異常終了のみ。先に要求する。
        if not (sub / "trajectory.csv").exists() or not (sub / "results.json").exists():
            console.error(f"run '{est}' wrote no bundle (exit {rc}) — aborting comparison")
            return 1
        runs[est] = sub

    out = bundle / f"{args.milestone.lower()}_compare.mp4"
    console.info("Rendering side-by-side comparison (twin 3D + overlay graphs)...")
    fps = getattr(args, "fps", 50)
    r = subprocess.run([str(py), str(_sil_dir() / "viz" / "render_video.py"),
                        "--model", str(_model()),
                        "--bundle", str(runs[args.ea]), "--compare", str(runs[args.eb]),
                        "--label-a", ESTIMATOR_LABELS[args.ea],
                        "--label-b", ESTIMATOR_LABELS[args.eb],
                        "--out", str(out), "--fps", str(fps)])
    if r.returncode != 0:
        console.error("comparison render failed"); return r.returncode

    # Aggregate verdict: P4 passes iff BOTH runs pass G3 with the bench unchanged.
    # results.json stays the single source of truth (status/gate read only this).
    # 集約判定: 両実行がベンチ無改変で G3 合格のとき P4 合格。results.json が唯一の正。
    a = json.loads((runs[args.ea] / "results.json").read_text())
    b = json.loads((runs[args.eb] / "results.json").read_text())
    both = bool(a.get("pass") and b.get("pass"))
    agg = {
        "gate": "compare",
        "milestone": args.milestone,
        "kind": "comparison",
        "pass": both,
        "noise": noise,
        "flight": a.get("flight", "takeoff-hover-yaw-stop-landing"),
        "runs": [
            {"estimator": args.ea, "bundle": args.ea,
             "g3_pass": bool(a.get("pass")), "metrics": a.get("metrics", {})},
            {"estimator": args.eb, "bundle": args.eb,
             "g3_pass": bool(b.get("pass")), "metrics": b.get("metrics", {})},
        ],
        "checks": [
            {"name": f"{args.ea}_g3_pass", "pass": bool(a.get("pass"))},
            {"name": f"{args.eb}_g3_pass", "pass": bool(b.get("pass"))},
            {"name": "algorithm_independent", "pass": both},
        ],
    }
    (bundle / "results.json").write_text(json.dumps(agg, indent=2) + "\n")
    console.success(f"Comparison bundle written to {bundle}")
    # Gate the comparison bundle (bundle complete AND both runs passing).
    return run_gate(args)


def run_status(args: argparse.Namespace) -> int:
    res = _bundle_dir(args.milestone) / "results.json"
    if not res.exists():
        console.error(f"No results.json for {args.milestone} — run 'sf sil run' first"); return 1
    r = json.loads(res.read_text())
    verdict = "PASS" if r.get("pass") else "FAIL"
    console.info(f"{r.get('milestone','?')}/{r.get('gate','?')}: {verdict}")
    # A comparison bundle (P4) carries per-run metrics under "runs"; a single
    # bundle carries flat "metrics". Show whichever shape this results.json has.
    # 比較バンドル(P4)は "runs" に各実行の metrics を持つ。単一は "metrics"。
    if r.get("kind") == "comparison":
        for run in r.get("runs", []):
            m = run.get("metrics", {})
            tilt = m.get("max_tilt_deg", "?")
            alt = m.get("max_alt_m", "?")
            g3 = "PASS" if run.get("g3_pass") else "FAIL"
            name = run.get("estimator") or "?"
            print(f"  {name:14s} G3 {g3}  max_alt={alt} m  max_tilt={tilt} deg")
    else:
        for k, v in r.get("metrics", {}).items():
            print(f"  {k:20s} {v}")
    for c in r.get("checks", []):
        mark = "PASS" if c.get("pass") else "FAIL"
        print(f"  [{mark}] {c.get('name')}")
    return 0 if r.get("pass") else 2


def run_gate(args: argparse.Namespace) -> int:
    py = _venv()
    if py is None:
        return 1
    r = subprocess.run([str(py), str(_sil_dir() / "tools" / "sil_gate.py"),
                        str(_bundle_dir(args.milestone))])
    return r.returncode


def run_milestone(args: argparse.Namespace) -> int:
    # The milestone only needs hover_smoke (run) + render_video (video, via venv) —
    # build just that target so an unrelated SIL target can't break the milestone.
    # milestone に必要なのは hover_smoke（run）と render_video（video, venv経由）だけ。
    # 無関係な SIL ターゲットが milestone を壊さないよう、そのターゲットだけビルドする。
    if not getattr(args, "target", None):
        args.target = "hover_smoke"
    # Resolve the per-milestone sensor-noise default unless the user set --noise.
    # ユーザーが --noise 指定しなければマイルストーン別の既定ノイズを解決。
    if getattr(args, "noise", None) is None:
        args.noise = MILESTONE_NOISE.get(str(args.milestone).upper(), "off")
    # P4 is the side-by-side comparison milestone: build once, then run BOTH
    # estimators and render one compare video (run_compare gates at the end).
    # P4 は並置比較マイルストーン: 1回ビルドし両推定器を走らせ1本の比較動画を描く。
    if str(args.milestone).upper() == "P4":
        if run_build(args) != 0:
            console.error("Milestone P4 stopped at build"); return 1
        args.ea = getattr(args, "ea", "eskf")
        args.eb = getattr(args, "eb", "complementary")
        return run_compare(args)
    for step in (run_build, run_run, run_video, run_gate):
        rc = step(args)
        if rc != 0 and step is not run_run:  # run_run defers its verdict to the gate
            console.error(f"Milestone {args.milestone} stopped at {step.__name__} (rc={rc})")
            return rc
    console.success(f"Milestone {args.milestone} bundle complete and gated.")
    return 0


def _venv():
    """Return the SIL venv python, creating it (mujoco + deps) if missing."""
    py = _venv_python()
    if py.exists():
        return py
    console.info("Creating SIL viz venv (mujoco 3.9.0 + matplotlib + imageio)...")
    venv_dir = _sil_dir() / "viz" / "venv"
    if subprocess.run([sys.executable, "-m", "venv", str(venv_dir)]).returncode != 0:
        console.error("venv creation failed"); return None
    pip = venv_dir / "bin" / "pip"
    if subprocess.run([str(pip), "install", "-q", "mujoco==3.9.0", "numpy",
                       "matplotlib", "imageio", "imageio-ffmpeg"]).returncode != 0:
        console.error("pip install failed"); return None
    return py
