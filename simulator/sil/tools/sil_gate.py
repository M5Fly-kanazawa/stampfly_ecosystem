#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
# Part of StampFly Ecosystem (SIL milestone gate).
#
# Output-driven milestone enforcement (RESET_PLAN.md §8, §10): a SIL milestone is
# "approvable" ONLY if the whole artifact bundle exists AND the machine verdict in
# results.json passes. This is the hook that makes milestones structural — you
# cannot claim a milestone without the artifacts. Wire it into:
#   - the /sil-milestone skill (runs it after producing the bundle), and
#   - a git pre-tag / CI check on milestone tags (e.g. tag `sil-p1`),
# so a milestone tag or "gate approve" is refused unless the bundle is complete.
#
# アウトプット主導のマイルストーン強制（RESET_PLAN §8, §10）: SIL マイルストーンは
# 成果物バンドルが全て揃い、かつ results.json の機械判定が pass のときだけ「承認可」。
# これがマイルストーンを構造的にするフック — 成果物なしにマイルストーンを主張できない。
#
# Usage: sil_gate.py <bundle_dir>   (exit 0 = approved, 1 = rejected)

import argparse
import json
import os
import sys


def main():
    ap = argparse.ArgumentParser(description="SIL milestone gate check")
    ap.add_argument("bundle", help="directory holding the milestone bundle")
    args = ap.parse_args()

    problems = []
    traj = os.path.join(args.bundle, "trajectory.csv")
    res = os.path.join(args.bundle, "results.json")
    mp4s = ([f for f in os.listdir(args.bundle) if f.endswith(".mp4")]
            if os.path.isdir(args.bundle) else [])

    if not os.path.isfile(traj):
        problems.append("missing trajectory.csv (run did not record)")
    if not mp4s:
        problems.append("missing review video (*.mp4) — RESET_PLAN §9 requires it")

    verdict = None
    if not os.path.isfile(res):
        problems.append("missing results.json (machine verdict)")
    else:
        try:
            r = json.load(open(res))
            verdict = r.get("pass")
            if verdict is not True:
                problems.append(f"machine verdict not passing (pass={verdict})")
            # Surface any individual failing check.
            for c in r.get("checks", []):
                if c.get("pass") is not True:
                    problems.append(f"failing check: {c.get('name')}")
        except (ValueError, OSError) as e:
            problems.append(f"results.json unreadable: {e}")

    if problems:
        print("GATE REJECTED — milestone bundle incomplete or not passing:")
        for p in problems:
            print(f"  - {p}")
        print("Produce the full bundle (results.json + review video) and pass the "
              "machine verdict before approving the milestone.")
        return 1

    gate = r.get("gate", "?")
    ms = r.get("milestone", "?")
    print(f"GATE APPROVED — {ms}/{gate}: bundle complete, verdict pass=true, "
          f"video={mp4s[0]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
