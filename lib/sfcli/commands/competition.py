"""
sf competition - Workshop competition tools

Tools for the StampFly workshop hover time competition (Day 5).
StampFly 勉強会 ホバリングタイム競技（Day 5）のためのツール。

Subcommands:
    hover-time - Hover time stopwatch (competition timer)
    score      - Display competition scores and rankings
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List

from ..utils import console, paths

COMMAND_NAME = "competition"
COMMAND_HELP = "Workshop competition tools"

# Score storage
SCORES_FILE = ".sf/competition_scores.json"


def _scores_path() -> Path:
    """Get scores file path"""
    return paths.root() / SCORES_FILE


def _load_scores() -> dict:
    """Load competition scores from file"""
    path = _scores_path()
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {"teams": {}}


def _save_scores(scores: dict) -> None:
    """Save competition scores to file"""
    path = _scores_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(scores, indent=2, ensure_ascii=False), encoding="utf-8")


def register(subparsers: argparse._SubParsersAction) -> None:
    """Register command with CLI"""
    parser = subparsers.add_parser(
        COMMAND_NAME,
        help=COMMAND_HELP,
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    comp_subparsers = parser.add_subparsers(
        dest="comp_command",
        title="subcommands",
        metavar="<subcommand>",
    )

    # --- hover-time ---
    hover_time_parser = comp_subparsers.add_parser(
        "hover-time",
        help="Hover time stopwatch",
        description="Stopwatch for hover time competition. "
        "Press Enter to start, Enter again to stop.",
    )
    hover_time_parser.add_argument(
        "--team",
        default=None,
        help="Team/participant name for score recording",
    )
    hover_time_parser.add_argument(
        "--max-time",
        type=float,
        default=60.0,
        help="Maximum hover time in seconds (default: 60.0)",
    )
    hover_time_parser.set_defaults(func=run_hover_time)

    # --- score ---
    score_parser = comp_subparsers.add_parser(
        "score",
        help="Display competition scores",
        description="Show hover time competition scores and rankings.",
    )
    score_parser.add_argument(
        "--clear",
        action="store_true",
        help="Clear all scores",
    )
    score_parser.set_defaults(func=run_score)

    # Default: show help
    parser.set_defaults(func=lambda args: (parser.print_help(), 0)[1])


def run_hover_time(args: argparse.Namespace) -> int:
    """Hover time competition stopwatch"""
    console.header("Hover Time Competition")
    console.print()

    if args.team:
        console.info(f"Participant: {args.team}")

    console.print(f"  Max time: {args.max_time:.0f} seconds")
    console.print()
    console.print("  Controls:")
    console.print("    Enter  - Start / Stop timer")
    console.print("    Ctrl+C - Cancel")
    console.print()
    console.info("Press Enter when the drone takes off...")

    try:
        input()
    except (EOFError, KeyboardInterrupt):
        return 0

    start_time = time.monotonic()
    console.success("TIMER STARTED!")
    console.print()
    console.info("Press Enter when the drone lands/crashes...")

    try:
        # Wait for Enter or timeout
        import select

        while True:
            elapsed = time.monotonic() - start_time
            if elapsed >= args.max_time:
                console.print(f"\r  Time: {elapsed:5.1f}s - MAX TIME REACHED!", end="")
                console.print()
                break

            console.print(f"\r  Time: {elapsed:5.1f}s", end="")

            # Check for input (non-blocking on Unix)
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                sys.stdin.readline()
                break

    except KeyboardInterrupt:
        console.print()
        console.info("Cancelled")
        return 0

    hover_time = min(time.monotonic() - start_time, args.max_time)

    console.print()
    console.header("Result")
    console.print()
    console.print(f"  Hover time: {hover_time:.2f} seconds")
    console.print()

    if hover_time >= args.max_time:
        console.success("PERFECT! Maximum time reached!")
    elif hover_time >= 30.0:
        console.success("Great hover!")
    elif hover_time >= 10.0:
        console.info("Good attempt!")
    else:
        console.info("Keep tuning those PID gains!")

    # Save score if team specified
    if args.team:
        scores = _load_scores()
        if args.team not in scores["teams"]:
            scores["teams"][args.team] = {"attempts": []}

        team_data = scores["teams"][args.team]
        if "attempts" not in team_data:
            team_data["attempts"] = []

        team_data["attempts"].append({
            "hover_time": round(hover_time, 2),
            "timestamp": datetime.now().isoformat(),
        })

        # Track best time
        best = max(a["hover_time"] for a in team_data["attempts"])
        team_data["best_time"] = best
        team_data["attempt_count"] = len(team_data["attempts"])

        _save_scores(scores)
        console.print()
        console.success(f"Score saved for: {args.team}")
        console.print(f"  Best time: {best:.2f}s ({len(team_data['attempts'])} attempts)")

    return 0


def run_score(args: argparse.Namespace) -> int:
    """Display competition scores"""
    if args.clear:
        path = _scores_path()
        if path.exists():
            path.unlink()
            console.success("Scores cleared")
        else:
            console.info("No scores to clear")
        return 0

    scores = _load_scores()

    if not scores["teams"]:
        console.info("No competition scores recorded yet")
        console.print("  Use 'sf competition hover-time --team <name>' to record")
        return 0

    console.header("Hover Time Competition Rankings")
    console.print()

    # Build ranking list
    rankings = []
    for team, data in scores["teams"].items():
        best = data.get("best_time", 0)
        attempts = data.get("attempt_count", 0)
        rankings.append((team, best, attempts))

    # Sort by best time (higher is better)
    rankings.sort(key=lambda x: x[1], reverse=True)

    console.print(f"  {'Rank':>4}  {'Name':<20}  {'Best Time':>10}  {'Attempts':>8}")
    console.print(f"  {'----':>4}  {'----':<20}  {'---------':>10}  {'--------':>8}")

    for rank, (team, best, attempts) in enumerate(rankings, 1):
        time_str = f"{best:.2f}s"
        console.print(f"  {rank:>4}  {team:<20}  {time_str:>10}  {attempts:>8}")

    console.print()
    return 0
