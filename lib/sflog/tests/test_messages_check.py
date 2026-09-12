"""
test_messages_check.py - protocol/tools/check_messages.py must pass, i.e.
protocol/spec/messages.yaml (the ESP-NOW wire-format SSOT) and
firmware/common/protocol/include/espnow_protocol.hpp (its hand-written C++
implementation) agree on message sizes, field layout, and flag-bit
constants. Mirrors the subprocess-and-check-returncode pattern of
test_schema_generated.py in this same directory.

test_messages_check.py - protocol/tools/check_messages.py が通ること。
つまり protocol/spec/messages.yaml（ESP-NOW 電文形式の SSOT）と、その
手書き C++ 実装 firmware/common/protocol/include/espnow_protocol.hpp が、
メッセージサイズ・フィールドレイアウト・フラグビット定数について一致して
いること。同じディレクトリの test_schema_generated.py と同じ、サブプロセス
実行+終了コード確認の流儀に倣う。
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
CHECK_SCRIPT = REPO_ROOT / "protocol" / "tools" / "check_messages.py"


def test_messages_yaml_matches_espnow_protocol_header():
    result = subprocess.run(
        [sys.executable, str(CHECK_SCRIPT)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, (
        "protocol/spec/messages.yaml and firmware/common/protocol/include/"
        "espnow_protocol.hpp disagree -- see the mismatch list below. Fix "
        "whichever side is wrong (the yaml is the SSOT, but the header may "
        "be the one that needs updating, or the yaml may need a missing "
        "item added) and re-run "
        "`python3 protocol/tools/check_messages.py` to confirm.\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )
