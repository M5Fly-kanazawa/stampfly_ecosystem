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


def run_status(args: argparse.Namespace) -> int:
    res = _bundle_dir(args.milestone) / "results.json"
    if not res.exists():
        console.error(f"No results.json for {args.milestone} — run 'sf sil run' first"); return 1
    r = json.loads(res.read_text())
    verdict = "PASS" if r.get("pass") else "FAIL"
    console.info(f"{r.get('milestone','?')}/{r.get('gate','?')}: {verdict}")
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
