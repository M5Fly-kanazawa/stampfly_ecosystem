#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
# Part of StampFly Ecosystem (SILS milestone gate).
#
# Output-driven milestone enforcement (RESET_PLAN.md §8, §10): a SILS milestone is
# "approvable" ONLY if the whole artifact bundle exists AND the machine verdict in
# results.json passes. This is the hook that makes milestones structural — you
# cannot claim a milestone without the artifacts. Wire it into:
#   - the /sils-milestone skill (runs it after producing the bundle), and
#   - a git pre-tag / CI check on milestone tags (e.g. tag `sils-p1`),
# so a milestone tag or "gate approve" is refused unless the bundle is complete.
#
# アウトプット主導のマイルストーン強制（RESET_PLAN §8, §10）: SILS マイルストーンは
# 成果物バンドルが全て揃い、かつ results.json の機械判定が pass のときだけ「承認可」。
# これがマイルストーンを構造的にするフック — 成果物なしにマイルストーンを主張できない。
#
# Usage: sils_gate.py <bundle_dir>   (exit 0 = approved, 1 = rejected)

import argparse
import json
import os
import sys


def main():
    ap = argparse.ArgumentParser(description="SILS milestone gate check")
    ap.add_argument("bundle", help="directory holding the milestone bundle")
    args = ap.parse_args()

    problems = []
    res = os.path.join(args.bundle, "results.json")
    mp4s = ([f for f in os.listdir(args.bundle) if f.endswith(".mp4")]
            if os.path.isdir(args.bundle) else [])

    if not mp4s:
        problems.append("missing review video (*.mp4) — RESET_PLAN §9 requires it")

    # Read the verdict first; its "kind" decides which trajectory layout to require.
    # A single bundle records trajectory.csv at the root; a comparison bundle (P4)
    # records one trajectory.csv per run subdir (runs[].bundle).
    # 判定を先に読む。"kind" でトラジェクトリの配置（単一=直下／比較=各実行サブ）を決める。
    r = None
    if not os.path.isfile(res):
        problems.append("missing results.json (machine verdict)")
    else:
        try:
            r = json.load(open(res))
            if not isinstance(r, dict):
                # Valid JSON but not an object (null/list/number from a partial or
                # error write) — reject cleanly instead of crashing on r.get(...).
                # 正しいJSONだがオブジェクトでない（部分/異常書き込み）→ クラッシュせず拒否。
                problems.append(f"results.json is not a JSON object (got {type(r).__name__})")
                r = None
            else:
                if r.get("pass") is not True:
                    problems.append(f"machine verdict not passing (pass={r.get('pass')})")
                # Surface any individual failing check.
                for c in r.get("checks", []):
                    if c.get("pass") is not True:
                        problems.append(f"failing check: {c.get('name')}")
        except (ValueError, OSError) as e:
            problems.append(f"results.json unreadable: {e}")

    if r is not None and r.get("kind") == "comparison":
        # Each compared run must have recorded its own trajectory.
        for run in r.get("runs", []):
            sub = os.path.join(args.bundle, run.get("bundle", run.get("estimator", "")))
            if not os.path.isfile(os.path.join(sub, "trajectory.csv")):
                problems.append(f"missing trajectory.csv for run '{run.get('estimator')}'")
    else:
        if not os.path.isfile(os.path.join(args.bundle, "trajectory.csv")):
            problems.append("missing trajectory.csv (run did not record)")

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
