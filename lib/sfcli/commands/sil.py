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
  milestone  build → run → video → gate in one shot (the /sil-milestone skill)
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

from ..utils import console, paths

COMMAND_NAME = "sil"
COMMAND_HELP = "Software-in-the-Loop bench (closed-loop hover, review video, gate)"

ESTIMATORS = {"eskf": 0, "complementary": 1}
ESTIMATOR_LABELS = {"eskf": "ESKF", "complementary": "Complementary"}


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

    p = sub.add_parser("compare", help="Side-by-side ESKF vs complementary video (P4)")
    p.add_argument("-m", "--milestone", default="P4")
    p.add_argument("--ea", choices=list(ESTIMATORS), default="eskf", help="run A estimator")
    p.add_argument("--eb", choices=list(ESTIMATORS), default="complementary", help="run B estimator")
    p.add_argument("--fps", type=int, default=50)
    p.set_defaults(func=run_compare)

    p = sub.add_parser("milestone", help="build → run → video → gate in one shot")
    p.add_argument("-m", "--milestone", default="P1")
    p.add_argument("-e", "--estimator", choices=list(ESTIMATORS), default="eskf")
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
    console.info(f"Running closed loop (milestone={args.milestone}, estimator={args.estimator})...")
    # argv: model, bundle, estimator_type, milestone (labels the results.json bundle).
    # 引数: モデル, バンドル, 推定器種別, マイルストーン(results.json のラベル)。
    r = subprocess.run([str(exe), str(_model()), str(bundle), str(et), str(args.milestone)])
    if r.returncode == 0:
        console.success(f"Bundle written to {bundle}")
    else:
        console.error(f"Closed loop FAILED (exit {r.returncode}) — see output / results.json")
    return 0  # the verdict lives in results.json; gate decides pass/fail


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
    bundle = _bundle_dir(args.milestone)          # out_p4
    bundle.mkdir(parents=True, exist_ok=True)
    runs = {}
    for est in (args.ea, args.eb):
        sub = bundle / est
        sub.mkdir(parents=True, exist_ok=True)
        console.info(f"Running closed loop for comparison ({est})...")
        rc = subprocess.run([str(exe), str(_model()), str(sub), str(ESTIMATORS[est]),
                             f"{args.milestone}-{est}"]).returncode
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
        "gate": "P4-compare",
        "milestone": args.milestone,
        "kind": "comparison",
        "pass": both,
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
