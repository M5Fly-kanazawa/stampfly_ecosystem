#!/usr/bin/env python3
"""
test_installer_stream.py - _stream_subprocess / _OutputRelay behaviour
インストーラの子プロセス出力中継（_stream_subprocess / _OutputRelay）の試験

Background: git clone --progress, ESP-IDF's install scripts (idf_tools.py
download counters) and pip rewrite one line in place with `\\r`. The
installer used to turn every `\\r` into `\\n`, which printed thousands of
progress lines and scrolled the whole install log out of the terminal
buffer (reported 2026-09-11). These tests pin the fixed behaviour: pass
`\\r` through on a terminal, throttle to one line per interval when stdout
is captured, never lose the final state of a line.
背景: git clone --progress、ESP-IDF の install スクリプト（idf_tools.py の
ダウンロードカウンタ）、pip は `\\r` で 1 行をその場で書き直す。以前の
インストーラは `\\r` を全て `\\n` にしていたため進捗行が数千行印字され、
インストールログ全体が端末バッファから流れ出ていた（2026-09-11 報告）。
本試験は修正後の挙動を固定する: 端末には `\\r` をそのまま通す、標準出力が
捕捉されているときは一定間隔で 1 行に間引く、行の最終状態は失わない。

Usage:
    pytest scripts/test_installer_stream.py
"""

import contextlib
import importlib.util
import io
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parent.parent


def _load_installer():
    spec = importlib.util.spec_from_file_location(
        "sf_installer_under_test", _REPO_ROOT / "scripts" / "installer.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


installer = _load_installer()


class _CapturedStdout(io.StringIO):
    """A captured (non-terminal) stdout, like the GUI installer's queue or a log file.
    捕捉された（端末ではない）標準出力。GUI インストーラのキューやログファイル相当。"""

    def isatty(self) -> bool:
        return False


class _TerminalStdout(io.StringIO):
    """A terminal-like stdout.
    端末相当の標準出力。"""

    def isatty(self) -> bool:
        return True


def _relay_through(stream, chunks, monkeypatch, interval=None):
    """Feed `chunks` to a fresh relay with sys.stdout replaced by `stream`.
    sys.stdout を `stream` に差し替えた状態で、新しい中継器に `chunks` を流す。"""
    if interval is not None:
        monkeypatch.setattr(installer, "PROGRESS_LOG_INTERVAL_SECONDS", interval)
    relay = installer._OutputRelay()
    with contextlib.redirect_stdout(stream):
        for chunk in chunks:
            relay.feed(chunk)
        relay.close()
    return stream.getvalue()


def test_captured_progress_is_throttled_and_final_state_kept(monkeypatch):
    out = _relay_through(
        _CapturedStdout(),
        ["Receiving objects:  10%\rReceiving objects:  50%\rReceiving objects: 100%, done.\nnext line\n"],
        monkeypatch,
        interval=3600.0,
    )
    # First progress update is printed immediately, the 50% one is throttled
    # away, the final state arrives with its newline, then the normal line.
    # 最初の進捗は即時印字、50% は間引かれ、最終状態は改行と共に届き、通常行が続く。
    assert out.splitlines() == [
        "Receiving objects:  10%",
        "Receiving objects: 100%, done.",
        "next line",
    ]


def test_captured_crlf_is_a_plain_line_ending(monkeypatch):
    out = _relay_through(_CapturedStdout(), ["a\r\nb\r\n"], monkeypatch)
    assert out.splitlines() == ["a", "b"]


def test_captured_cr_at_chunk_boundary_does_not_add_blank_line(monkeypatch):
    out = _relay_through(_CapturedStdout(), ["abc\r", "\ndef\n"], monkeypatch)
    assert out.splitlines() == ["abc", "def"]


def test_captured_unterminated_tail_is_flushed(monkeypatch):
    out = _relay_through(_CapturedStdout(), ["12%\r34%\r56%"], monkeypatch, interval=3600.0)
    assert out.splitlines() == ["12%", "56%"]


def test_terminal_passthrough_keeps_carriage_returns(monkeypatch):
    text = "  10%\r  50%\r 100%\ndone\n"
    out = _relay_through(_TerminalStdout(), [text], monkeypatch)
    assert out == text


def test_stream_subprocess_bounds_progress_lines(monkeypatch):
    """A child printing 500 `\\r` progress updates must not produce 500 lines
    when stdout is captured.
    捕捉時、`\\r` 進捗 500 回を出す子プロセスが 500 行にならないこと。"""
    monkeypatch.setattr(installer, "PROGRESS_LOG_INTERVAL_SECONDS", 3600.0)
    child = (
        "import sys\n"
        "for i in range(500):\n"
        "    sys.stdout.write('\\rDownloading %d%%' % (i * 100 // 499)); sys.stdout.flush()\n"
        "sys.stdout.write('\\nDone\\n')\n"
    )
    stream = _CapturedStdout()
    with contextlib.redirect_stdout(stream):
        rc = installer._stream_subprocess([sys.executable, "-c", child])
    assert rc == 0
    lines = stream.getvalue().splitlines()
    assert lines[-1] == "Done"
    assert lines[-2] == "Downloading 100%"
    assert len(lines) <= 4


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
