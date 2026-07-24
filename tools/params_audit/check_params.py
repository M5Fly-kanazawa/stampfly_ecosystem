#!/usr/bin/env python3
"""
sf params check — physical parameter consistency audit.
sf params check — 物理パラメータ整合検査。

StampFly's vehicle physical parameters (C_T, C_Q, kappa, inertia, ...) are
hand-copied across firmware, SIL, and multiple simulators because there is
no code-generation pipeline yet (Phase 1, see docs/architecture/
simulation-policy.md). This tool deterministically re-reads every copy
listed in params_manifest.MANIFEST and reports whether it still matches the
confirmed measured value.
StampFly の機体物理パラメータ（C_T, C_Q, kappa, 慣性 等）は、コード生成
パイプラインがまだ無い（Phase 1、docs/architecture/simulation-policy.md
参照）ため、ファーム・SIL・複数シミュレータへ手動でコピペされている。
本ツールは params_manifest.MANIFEST に列挙された全コピーを決定論的に
読み直し、確定実測値と一致しているかを報告する。

Standard library only (re, pathlib, json, argparse) — no numpy, no project
imports beyond params_manifest, so this always runs even in a bare Python.
標準ライブラリのみ使用（re, pathlib, json, argparse）— numpy 等の依存が無く
params_manifest 以外のプロジェクト内 import も無いため、素の Python でも
必ず動く。

Exit code: 1 if any MISMATCH or ERROR (--strict also fails on UNRESOLVED),
0 otherwise. See README.md in this directory for the manifest format and how
to extend it.
終了コード: MISMATCH または ERROR があれば 1（--strict では UNRESOLVED も
失敗扱い）、それ以外は 0。マニフェストの形式・拡張方法は本ディレクトリの
README.md を参照。
"""

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Union

# Support two import contexts: (a) `from params_audit import check_params`
# after the caller (e.g. lib/sfcli/commands/params.py) puts tools/ on
# sys.path — here this module is a submodule of the params_audit package,
# so the relative import is the correct one; (b) running this file directly
# as a script (`python check_params.py`) — Python auto-adds this file's own
# directory to sys.path in that mode, so the plain absolute import resolves.
# 2つの実行文脈をサポートする: (a) 呼び出し側（lib/sfcli/commands/params.py
# 等）が tools/ を sys.path に追加した上で `from params_audit import
# check_params` する場合 — このモジュールは params_audit パッケージの
# サブモジュールなので相対 import が正しい。(b) 本ファイルを直接スクリプト
# 実行する場合（`python check_params.py`）— このモードでは Python が本
# ファイル自身のディレクトリを自動的に sys.path へ追加するため、素の
# 絶対 import が解決できる。
try:
    from .params_manifest import MANIFEST, ParamCheck, Unresolved, Exempt
except ImportError:
    from params_manifest import MANIFEST, ParamCheck, Unresolved, Exempt

# Relative-error tolerance for numeric OK/MISMATCH comparison.
# 数値 OK/MISMATCH 判定の相対誤差許容値。
RELATIVE_TOLERANCE = 1e-6

# Unicode superscript digits/sign used by the Markdown doc's "1.00×10⁻⁸"
# notation (docs/architecture/stampfly-parameters.md). Translated to ASCII
# before int() parsing.
# Markdown 文書の "1.00×10⁻⁸" 表記で使われる上付き数字/符号。int() で解釈
# する前に ASCII へ変換する。
_SUPERSCRIPT_TRANS = str.maketrans("⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺", "0123456789-+")
_SUPERSCRIPT_SCI_RE = re.compile(r'^(-?[0-9.]+)×10([⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)$')


def find_repo_root() -> Path:
    """Walk up from this file looking for a repo marker (.git/CLAUDE.md).
    このファイルから上へ辿りリポジトリマーカー(.git/CLAUDE.md)を探す。

    Falls back to the fixed tools/params_audit/../.. relationship if no
    marker is found (e.g. the file was copied out of the repo).
    マーカーが見つからない場合は tools/params_audit/../.. の固定関係へ
    フォールバックする（リポジトリ外へコピーされた場合等）。
    """
    here = Path(__file__).resolve()
    for parent in [here] + list(here.parents):
        if (parent / ".git").exists() or (parent / "CLAUDE.md").exists():
            return parent
    return here.parents[2]


def parse_value(raw: str) -> float:
    """Parse a numeric token captured by a manifest regex into a float.
    マニフェスト正規表現が捕捉した数値トークンを float へ変換する。

    Handles two forms found in the repository:
      1. Plain Python float/scientific literal, e.g. "6.7e-9", "0.023".
      2. The Markdown doc's unicode-superscript scientific notation,
         e.g. "1.00×10⁻⁸" (docs/architecture/stampfly-parameters.md).
    リポジトリ内に存在する2種類の表記を扱う:
      1. 通常の Python float/指数リテラル（例: "6.7e-9", "0.023"）
      2. Markdown 文書の unicode 上付き指数表記（例: "1.00×10⁻⁸"）
    """
    raw = raw.strip()
    try:
        return float(raw)
    except ValueError:
        pass

    m = _SUPERSCRIPT_SCI_RE.match(raw)
    if not m:
        raise ValueError(f"cannot parse numeric value from {raw!r}")
    mantissa = float(m.group(1))
    exponent = int(m.group(2).translate(_SUPERSCRIPT_TRANS))
    return mantissa * (10.0 ** exponent)


@dataclass
class ResultRow:
    """One audited (parameter, location) result, ready for table/JSON output.
    1件の検査結果（パラメータ×場所）。表/JSON出力用に整形済み。"""
    parameter: str
    location: str          # "path/to/file.py:123 (note)"
    current: Optional[str]  # formatted current value, or None if unavailable
    expected: str           # formatted expected value / marker text
    status: str             # OK / MISMATCH / UNRESOLVED / EXEMPT / ERROR
    detail: str = ""        # error message, unresolved candidates, or exempt reason


def _format_number(value: float) -> str:
    """Compact scientific/decimal display for the table and JSON.
    表・JSON 用のコンパクトな数値表示。"""
    return f"{value:.6g}"


def _line_number(content: str, match_start: int) -> int:
    """1-based line number of a regex match start offset.
    正規表現マッチ開始位置の1始まり行番号。"""
    return content.count("\n", 0, match_start) + 1


def check_entry(repo_root: Path, param_name: str, entry: ParamCheck,
                 file_cache: Dict[str, Optional[str]]) -> ResultRow:
    """Evaluate one ParamCheck against the real file on disk.
    1件の ParamCheck を実ファイルに対して評価する。

    ERROR covers both "file missing" and "regex matched zero times" — the
    task's requirement is that a stale/broken pattern is never silently
    skipped, it always surfaces as a failure.
    ERROR は「ファイルが無い」と「正規表現が1件もマッチしない」の両方を含む
    — 失効したパターンを黙ってスキップせず必ず失敗として表面化させる。
    """
    location_base = f"{entry.file} (note: {entry.note})" if entry.note else entry.file
    if entry.file not in file_cache:
        path = repo_root / entry.file
        file_cache[entry.file] = path.read_text(encoding="utf-8") if path.exists() else None
    content = file_cache[entry.file]

    if content is None:
        return ResultRow(param_name, location_base, None, _expected_label(entry.expected),
                          "ERROR", detail="file not found")

    match = re.search(entry.regex, content)
    if match is None:
        return ResultRow(param_name, location_base, None, _expected_label(entry.expected),
                          "ERROR", detail="regex did not match — pattern is stale")

    line_no = _line_number(content, match.start())
    location = f"{entry.file}:{line_no}" + (f" ({entry.note})" if entry.note else "")
    try:
        current_value = parse_value(match.group(1))
    except ValueError as exc:
        return ResultRow(param_name, location, None, _expected_label(entry.expected),
                          "ERROR", detail=f"matched but unparseable: {exc}")

    return _judge(param_name, location, current_value, entry.expected)


def _expected_label(expected: Union[float, Unresolved, Exempt]) -> str:
    """Human-readable "expected value" cell for rows that never reach
    comparison (file missing / regex stale).
    比較まで到達しない行（ファイル無し／正規表現失効）向けの期待値欄表示。"""
    if isinstance(expected, Unresolved):
        return "UNRESOLVED"
    if isinstance(expected, Exempt):
        return "EXEMPT"
    return _format_number(expected)


def _judge(param_name: str, location: str, current_value: float,
           expected: Union[float, Unresolved, Exempt]) -> ResultRow:
    """Classify a successfully-parsed current value against its expected
    marker/value. Split out of check_entry to keep both functions short.
    正しく解析できた現在値を期待マーカー/値と照合し判定する。check_entry
    を短く保つため分離。"""
    current_str = _format_number(current_value)

    if isinstance(expected, Unresolved):
        return ResultRow(param_name, location, current_str, "UNRESOLVED",
                          "UNRESOLVED", detail=expected.candidates)

    if isinstance(expected, Exempt):
        return ResultRow(param_name, location, current_str, "EXEMPT",
                          "EXEMPT", detail=expected.reason)

    rel_err = abs(current_value - expected) / max(abs(expected), 1e-300)
    if rel_err <= RELATIVE_TOLERANCE:
        return ResultRow(param_name, location, current_str, _format_number(expected), "OK")
    return ResultRow(param_name, location, current_str, _format_number(expected), "MISMATCH",
                      detail=f"relative error {rel_err:.3%}")


def run_audit(repo_root: Path) -> List[ResultRow]:
    """Run every ParamCheck in MANIFEST and return the flat result list.
    MANIFEST 内の全 ParamCheck を実行しフラットな結果リストを返す。"""
    file_cache: Dict[str, Optional[str]] = {}
    rows: List[ResultRow] = []
    for param_name, entries in MANIFEST.items():
        for entry in entries:
            rows.append(check_entry(repo_root, param_name, entry, file_cache))
    return rows


def summarize(rows: List[ResultRow]) -> Dict[str, int]:
    """Count rows per status, plus a total. / ステータス別件数＋合計を集計。"""
    summary = {"OK": 0, "MISMATCH": 0, "UNRESOLVED": 0, "EXEMPT": 0, "ERROR": 0}
    for row in rows:
        summary[row.status] += 1
    summary["total"] = len(rows)
    return summary


def format_table(rows: List[ResultRow], summary: Dict[str, int]) -> str:
    """Render the plain-text report table + summary line.
    プレーンテキストのレポート表＋サマリ行を整形する。"""
    headers = ("パラメータ", "場所(file:line)", "現在値", "期待値", "判定")
    widths = [len(h) for h in headers]
    table_rows = []
    for row in rows:
        cells = (row.parameter, row.location, row.current or "-", row.expected, row.status)
        table_rows.append(cells)
        widths = [max(w, len(c)) for w, c in zip(widths, cells)]

    lines = [" | ".join(h.ljust(w) for h, w in zip(headers, widths))]
    lines.append("-+-".join("-" * w for w in widths))
    for cells in table_rows:
        lines.append(" | ".join(c.ljust(w) for c, w in zip(cells, widths)))

    lines.append("")
    lines.append(f"Summary: OK={summary['OK']} MISMATCH={summary['MISMATCH']} "
                 f"UNRESOLVED={summary['UNRESOLVED']} EXEMPT={summary['EXEMPT']} "
                 f"ERROR={summary['ERROR']} (total={summary['total']})")
    return "\n".join(lines)


def format_json(rows: List[ResultRow], summary: Dict[str, int]) -> str:
    """Render the machine-readable report. / 機械可読レポートを整形する。"""
    payload = {
        "results": [
            {
                "parameter": row.parameter,
                "location": row.location,
                "current": row.current,
                "expected": row.expected,
                "status": row.status,
                "detail": row.detail,
            }
            for row in rows
        ],
        "summary": summary,
    }
    return json.dumps(payload, ensure_ascii=False, indent=2)


def compute_exit_code(summary: Dict[str, int], strict: bool) -> int:
    """1 on any MISMATCH/ERROR; --strict also fails on UNRESOLVED.
    MISMATCH/ERROR が1件でもあれば1。--strict では UNRESOLVED も失敗扱い。"""
    if summary["MISMATCH"] or summary["ERROR"]:
        return 1
    if strict and summary["UNRESOLVED"]:
        return 1
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    """CLI entry point — also called directly by `sf params check`.
    CLI エントリポイント — `sf params check` からも直接呼ばれる。"""
    parser = argparse.ArgumentParser(
        description="Audit hand-copied StampFly physical parameters for consistency.",
    )
    parser.add_argument("--json", action="store_true",
                        help="emit machine-readable JSON instead of a text table")
    parser.add_argument("--strict", action="store_true",
                        help="also fail (exit 1) when any parameter is UNRESOLVED")
    args = parser.parse_args(argv)

    repo_root = find_repo_root()
    rows = run_audit(repo_root)
    summary = summarize(rows)

    if args.json:
        print(format_json(rows, summary))
    else:
        print(format_table(rows, summary))

    return compute_exit_code(summary, args.strict)


if __name__ == "__main__":
    sys.exit(main())
