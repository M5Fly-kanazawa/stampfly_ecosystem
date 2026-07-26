#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/ci/render_sil_summary.py

Render a `sf sil regression --json-out <path>` summary as a GitHub Actions
job summary (Markdown appended to $GITHUB_STEP_SUMMARY) — one row per
scenario (name / target / PASS-FAIL / elapsed seconds), used by
.github/workflows/sil-regression.yml.
`sf sil regression --json-out <path>` の出力を GitHub Actions のジョブサマリ
（$GITHUB_STEP_SUMMARY への Markdown 追記）として描画する。シナリオごとに
1行（名前/target/PASS-FAIL/所要秒）。.github/workflows/sil-regression.yml が使う。

A standalone script (not a `sf` subcommand) because it renders a
GitHub-Actions-specific artifact ($GITHUB_STEP_SUMMARY), matching the other
CI-only scripts in this directory (check_flasher_install.py, check_upgrade.py,
check_installer_gui.py) rather than the project's general `sf CLI` tools.
`sf` サブコマンドではなく単体スクリプトにしている理由: 描画先が
GitHub-Actions固有の成果物($GITHUB_STEP_SUMMARY)であり、一般開発者向けの
`sf CLI` ツールというより本ディレクトリの他CI専用スクリプトと同種のため。

Usage / 使い方:
    python3 tools/ci/render_sil_summary.py <results.json>

Reads $GITHUB_STEP_SUMMARY from the environment and appends to it. If that
variable is unset (e.g. run locally, outside Actions), prints the Markdown to
stdout instead so the script is still runnable for a quick local check.
環境変数 $GITHUB_STEP_SUMMARY に追記する。未設定（Actions 外のローカル実行等）
なら標準出力に Markdown を出す（ローカルでの簡易確認にも使えるようにする）。

Exit code: always 0 on a well-formed (or absent) results file — this script
renders a report, it does not itself gate CI (the `sf sil regression` step
that produced the JSON already set the job's pass/fail via its own exit
code; this step runs unconditionally via `if: always()` so a FAILing run
still shows which scenario(s) broke).
終了コード: 整形済み(または不在)の結果ファイルに対しては常に0 — 本スクリプトは
レポート描画のみ行いCI自体はゲートしない（ジョブの合否は結果を生成した
`sf sil regression` 側の終了コードが既に決めている。本ステップは
`if: always()` で常時実行し、FAIL時もどのシナリオが壊れたか表示する）。
"""

import json
import os
import sys
from pathlib import Path


def render(results_path: Path) -> str:
    if not results_path.exists():
        return (
            "## SIL scenario regression\n\n"
            "`sf sil regression` produced no results file — the build or the "
            "command itself likely failed before any scenario ran (see the "
            "previous step's log).\n"
        )

    data = json.loads(results_path.read_text(encoding="utf-8"))
    total, n_pass, n_fail = data["total"], data["pass"], data["fail"]
    # known_fail/xpass are new fields (xfail known-fail mechanism, 2026-07); absent on
    # an older results.json, so default to 0 for backward compatibility with a summary
    # produced by a pre-xfail `sf sil regression`.
    # known_fail/xpass は新フィールド（xfail 既知失敗機構、2026-07）。xfail 導入前の
    # 古い results.json には無いので、後方互換のため既定 0。
    n_known_fail = data.get("known_fail", 0)
    n_xpass = data.get("xpass", 0)
    verdict = "PASS" if n_fail == 0 else "FAIL"
    title = f"## SIL scenario regression: {verdict} ({n_pass}/{total} scenarios PASS"
    if n_known_fail or n_xpass:
        title += f", {n_known_fail} known-fail, {n_xpass} xpass"
    title += ")"
    lines = [
        title,
        "",
        "| Scenario | Target | Result | Time (s) | Known-fail reason |",
        "|----------|--------|--------|----------|--------------------|",
    ]
    for s in data["scenarios"]:
        # "status" is the new per-scenario field (PASS/FAIL/KNOWN-FAIL/XPASS); fall
        # back to the old boolean "pass" for a pre-xfail results.json.
        # "status" は新フィールド。xfail 導入前の results.json では旧来の
        # 真偽値 "pass" にフォールバックする。
        mark = s.get("status") or ("PASS" if s["pass"] else "FAIL")
        reason = s.get("xfail_reason") or ""
        lines.append(f"| {s['name']} | {s['target']} | {mark} | {s['elapsed_s']} | {reason} |")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: render_sil_summary.py <results.json>", file=sys.stderr)
        return 2
    markdown = render(Path(sys.argv[1]))

    summary_file = os.environ.get("GITHUB_STEP_SUMMARY")
    if summary_file:
        with open(summary_file, "a", encoding="utf-8") as f:
            f.write(markdown)
    else:
        print(markdown)
    return 0


if __name__ == "__main__":
    sys.exit(main())
