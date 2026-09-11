"""
test_schema_generated.py - protocol/tools/gen_flight_log.py --check must
pass, i.e. lib/sflog/schema.py and docs/reference/flight-log-format.md are
not stale relative to protocol/spec/flight_log.yaml (the SSOT).
test_schema_generated.py - protocol/tools/gen_flight_log.py --check が
通ること。つまり lib/sflog/schema.py と
docs/reference/flight-log-format.md が正本
protocol/spec/flight_log.yaml に対して古くなっていないこと。
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
GEN_SCRIPT = REPO_ROOT / "protocol" / "tools" / "gen_flight_log.py"


def test_generated_files_are_up_to_date():
    result = subprocess.run(
        [sys.executable, str(GEN_SCRIPT), "--check"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, (
        "lib/sflog/schema.py and/or docs/reference/flight-log-format.md are "
        "stale -- run `python3 protocol/tools/gen_flight_log.py --write`.\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )
