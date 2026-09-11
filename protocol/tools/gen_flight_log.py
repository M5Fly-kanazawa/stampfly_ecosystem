#!/usr/bin/env python3
"""
gen_flight_log.py - Generate flight-log v1 code/docs from the SSOT YAML.
gen_flight_log.py - 正本 YAML からフライトログ v1 のコード/文書を生成する。

Reads `protocol/spec/flight_log.yaml` (the Single Source of Truth for the
StampFly flight-log bundle format) and writes two generated artifacts:

    lib/sflog/schema.py                    Python column constants
    docs/reference/flight-log-format.md    human-readable column tables

Neither generated file should ever be hand-edited -- re-run this script
after changing the YAML instead.

`protocol/spec/flight_log.yaml`（StampFly フライトログ一式形式の正本）を
読み込み、生成物を2つ書き出す:

    lib/sflog/schema.py                    Python の列定数
    docs/reference/flight-log-format.md    人間可読な列の一覧表

どちらの生成物も手で編集しないこと -- YAML を変更したら本スクリプトを
再実行する。

Usage / 使い方:
    python3 protocol/tools/gen_flight_log.py            # same as --write
    python3 protocol/tools/gen_flight_log.py --write    # regenerate on disk
    python3 protocol/tools/gen_flight_log.py --check    # CI: fail if stale
"""

from __future__ import annotations

import argparse
import pprint
import sys
from pathlib import Path
from typing import Any

import yaml

# =============================================================================
# Paths
# パス
# =============================================================================

REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC_PATH = REPO_ROOT / "protocol" / "spec" / "flight_log.yaml"
SCHEMA_PY_PATH = REPO_ROOT / "lib" / "sflog" / "schema.py"
DOC_MD_PATH = REPO_ROOT / "docs" / "reference" / "flight-log-format.md"

GENERATED_NOTICE = (
    "GENERATED FILE - do not edit; run protocol/tools/gen_flight_log.py\n"
    "生成ファイル - 手で編集しないこと。protocol/tools/gen_flight_log.py を実行して再生成する。"
)


# =============================================================================
# Spec loading / helpers
# 仕様読み込み・補助関数
# =============================================================================


def load_spec() -> dict[str, Any]:
    """Parse protocol/spec/flight_log.yaml.
    protocol/spec/flight_log.yaml を読み込みパースする。
    """
    with open(SPEC_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_streams_dict(spec: dict) -> dict[str, dict]:
    """Lean per-stream dict: file/source/nominal_rate_hz/required/columns.

    `columns` is a tuple of (name, type, unit) -- the compact form used for
    everyday reads (e.g. bundle.py deciding which CSV columns to expect).
    軽量な per-stream 辞書。`columns` は (name, type, unit) のタプルで、
    日常的な参照（bundle.py が CSV の列を判定する等）に使う簡潔な形。
    """
    streams = {}
    for s in spec["streams"]:
        columns = tuple((c["name"], c["type"], c["unit"]) for c in s["columns"])
        streams[s["name"]] = {
            "file": s["file"],
            "source": s["source"],
            "nominal_rate_hz": s["nominal_rate_hz"],
            "required": s["required"],
            "columns": columns,
        }
    return streams


def build_column_meta(spec: dict) -> dict[str, list[dict]]:
    """Full per-column metadata (unit/type/description in both languages).

    Used by schema_for() to build the schema.json embedded in a bundle --
    richer than the lean tuples in STREAMS above.
    列ごとの完全なメタデータ（単位・型・説明を日英で）。schema_for() が
    バンドル同梱の schema.json を作るのに使う -- 上の STREAMS の軽量タプル
    より詳細。
    """
    meta = {}
    for s in spec["streams"]:
        meta[s["name"]] = [
            {
                "name": c["name"],
                "type": c["type"],
                "unit": c["unit"],
                "description_ja": c["description_ja"],
                "description_en": c["description_en"],
            }
            for c in s["columns"]
        ]
    return meta


# =============================================================================
# lib/sflog/schema.py generation
# lib/sflog/schema.py の生成
# =============================================================================


def _pp(obj: Any) -> str:
    """Deterministic Python literal rendering (stable across runs, so
    --check does not flag spurious diffs from dict/set ordering).
    決定論的な Python リテラル整形（実行の度に変わらない -- dict/set の
    順序で --check が偽の差分を報告しないようにする）。
    """
    return pprint.pformat(obj, indent=4, width=88, sort_dicts=False)


def render_schema_py(spec: dict) -> str:
    """Render the full lib/sflog/schema.py source text.
    lib/sflog/schema.py のソース全文を生成する。
    """
    streams = build_streams_dict(spec)
    column_meta = build_column_meta(spec)
    required = [name for name, info in streams.items() if info["required"]]
    lockstep = [
        name for name, info in streams.items() if any(c[0] == "seq" for c in info["columns"])
    ]

    parts: list[str] = []
    parts.append(f'"""\n{GENERATED_NOTICE}\n\n')
    parts.append("StampFly flight-log v1 schema constants.\n")
    parts.append("StampFly フライトログ v1 形式のスキーマ定数。\n\n")
    parts.append(f"Source of truth / 正本: {SPEC_PATH.relative_to(REPO_ROOT).as_posix()}\n")
    parts.append('"""\n\n')

    parts.append(f"FORMAT = {spec['format']!r}\n")
    parts.append(f"VERSION = {spec['version']!r}\n\n")

    parts.append(
        "# Stream name -> {file, source, nominal_rate_hz, required, columns}.\n"
        "# columns is a tuple of (name, type, unit); the fuller per-column\n"
        "# metadata (description in both languages) lives in _COLUMN_META\n"
        "# below and is exposed through schema_for().\n"
        "# ストリーム名 -> {file, source, nominal_rate_hz, required, columns}。\n"
        "# columns は (name, type, unit) のタプル。説明文（日英）を含む\n"
        "# より詳細な列メタデータは下の _COLUMN_META にあり、schema_for()\n"
        "# 経由で取得する。\n"
    )
    parts.append(f"STREAMS = {_pp(streams)}\n\n")

    parts.append(
        "# Stream name -> ordered list of column names (including timestamp_us).\n"
        "# ストリーム名 -> 列名の順序付きリスト（timestamp_us を含む）。\n"
    )
    parts.append(
        "COLUMN_NAMES = {\n"
        "    name: [c[0] for c in info['columns']] for name, info in STREAMS.items()\n"
        "}\n\n"
    )

    parts.append(
        "# Streams that MUST be present in every v1 bundle (only imu.csv today --\n"
        "# every other stream is optional because not every firmware/scenario\n"
        "# sends every packet type).\n"
        "# 全ての v1 バンドルに必須のストリーム（現状 imu.csv のみ -- 他は全て\n"
        "# 任意。ファーム/シナリオによって送らないパケット種別があるため）。\n"
    )
    parts.append(f"REQUIRED_STREAMS = {_pp(required)}\n\n")

    parts.append(
        "# Required streams per capture source (meta.json `source`): the\n"
        "# firmware's Data Stream always has imu.csv, the SILS emulator adds\n"
        "# MuJoCo truth.csv, and a pure-physics simulator has ONLY truth.csv.\n"
        "# check.py falls back to REQUIRED_STREAMS for an unknown source.\n"
        "# 取得元（meta.json の `source`）ごとの必須ストリーム: 実機の Data\n"
        "# Stream は常に imu.csv を持ち、SILS エミュレータはそれに MuJoCo の\n"
        "# truth.csv を加え、純粋な物理シミュレータは truth.csv しか持たない。\n"
        "# 未知の取得元は check.py が REQUIRED_STREAMS で検査する。\n"
    )
    parts.append(f"REQUIRED_STREAMS_BY_SOURCE = {_pp(spec['required_streams'])}\n\n")

    parts.append(
        "# Streams that publish one row per CONTROL CYCLE, all sharing the\n"
        "# 'seq' column as their true per-observation key -- their timestamp_us\n"
        "# can legitimately repeat (a control cycle that did not get a new IMU\n"
        "# sample reuses its timestamp; see each stream's timestamp_us/seq\n"
        "# description above). Derived here as \"declares a 'seq' column\",\n"
        "# so this list never drifts out of sync with the YAML.\n"
        "# 制御周期ごとに1行発行するストリーム。真の観測識別子は 'seq' 列で\n"
        "# あり、timestamp_us は正当に重複し得る（新しい IMU 標本を得られ\n"
        "# なかった周期は時刻を再利用する。各ストリームの timestamp_us/seq\n"
        "# の説明を参照）。\"'seq' 列を持つ\" として導出するため、YAML との\n"
        "# 乖離が起こらない。\n"
    )
    parts.append(f"LOCKSTEP_STREAMS = {_pp(lockstep)}\n\n")

    parts.append(
        "# Full per-column metadata (name/type/unit/description in both\n"
        "# languages). Keyed by stream name; used by schema_for() to build the\n"
        "# schema.json that travels inside a bundle.\n"
        "# 列ごとの完全なメタデータ（名前・型・単位・日英の説明）。\n"
        "# ストリーム名で引く。schema_for() がバンドル同梱の schema.json を\n"
        "# 作るのに使う。\n"
    )
    parts.append(f"_COLUMN_META = {_pp(column_meta)}\n\n\n")

    parts.append(
        "def schema_for(stream_names):\n"
        '    """Build a schema.json-compatible dict for the given stream names.\n'
        "\n"
        "    Args:\n"
        "        stream_names: iterable of stream names to include -- normally\n"
        "            the streams actually present in one bundle, which may be\n"
        "            fewer than the full STREAMS set (e.g. a bundle with no\n"
        "            magnetometer data has no 'mag' stream). Names not found in\n"
        "            STREAMS are silently skipped.\n"
        "\n"
        "    Returns:\n"
        "        {'format': FORMAT, 'version': VERSION, 'streams': {name: {file,\n"
        "        source, nominal_rate_hz, required, columns}}} where columns is\n"
        "        the list of per-column metadata dicts from _COLUMN_META.\n"
        "\n"
        "    渡されたストリーム名から schema.json 相当の dict を作る。\n"
        "\n"
        "    引数 stream_names は通常、そのバンドルに実際に存在するストリーム\n"
        "    （STREAMS 全体より少ないことがある。例: 地磁気センサ無しのバンドル\n"
        "    には 'mag' が無い）。STREAMS に無い名前は黙って無視する。\n"
        '    """\n'
        "    streams = {}\n"
        "    for name in stream_names:\n"
        "        if name not in STREAMS:\n"
        "            continue\n"
        "        info = STREAMS[name]\n"
        "        streams[name] = {\n"
        "            'file': info['file'],\n"
        "            'source': info['source'],\n"
        "            'nominal_rate_hz': info['nominal_rate_hz'],\n"
        "            'required': info['required'],\n"
        "            'columns': _COLUMN_META[name],\n"
        "        }\n"
        "    return {'format': FORMAT, 'version': VERSION, 'streams': streams}\n"
    )

    return "".join(parts)


# =============================================================================
# docs/reference/flight-log-format.md generation
# docs/reference/flight-log-format.md の生成
# =============================================================================


def _md_cell(text: str) -> str:
    """Escape a value for safe use inside a Markdown table cell.

    A few unit/type strings in the YAML contain a literal `|` (e.g.
    "str|null", "N*m" is fine but "object|null" is not) which would
    otherwise be parsed as an extra table column delimiter.
    Markdown の表セルへ安全に埋め込めるようエスケープする。

    YAML 中の一部の型/単位文字列にはリテラルの `|` が含まれる
    （例: "str|null", "object|null"）。エスケープしないと表の列区切りと
    誤認識される。
    """
    return str(text).replace("|", "\\|")


def _rate_str_ja(nominal_rate_hz) -> str:
    if nominal_rate_hz is None:
        return "(可変/事象駆動)"
    return f"{nominal_rate_hz} Hz"


def _rate_str_en(nominal_rate_hz) -> str:
    if nominal_rate_hz is None:
        return "(variable / event-driven)"
    return f"{nominal_rate_hz} Hz"


def _stream_overview_table(spec: dict, lang: str) -> str:
    if lang == "ja":
        header = "| ストリーム | ファイル | 由来 | 公称レート | 必須 |\n|---|---|---|---|---|\n"
    else:
        header = "| Stream | File | Source | Nominal rate | Required |\n|---|---|---|---|---|\n"
    rows = []
    for s in spec["streams"]:
        rate = _rate_str_ja(s["nominal_rate_hz"]) if lang == "ja" else _rate_str_en(s["nominal_rate_hz"])
        if lang == "ja":
            required = "必須" if s["required"] else "-"
        else:
            required = "yes" if s["required"] else "-"
        rows.append(
            f"| `{s['name']}` | `{s['file']}` | {_md_cell(s['source'])} | {rate} | {required} |"
        )
    return header + "\n".join(rows) + "\n"


def _required_by_source_table(spec: dict, lang: str) -> str:
    """Table of the streams that can never be missing, per capture source
    (`required_streams` in the YAML; `schema.REQUIRED_STREAMS_BY_SOURCE`).
    取得元ごとに絶対に欠けないストリームの表（YAML の `required_streams`、
    `schema.REQUIRED_STREAMS_BY_SOURCE`）。
    """
    if lang == "ja":
        lines = [
            "### 取得元ごとの必須ストリーム",
            "",
            "`meta.json` の `source` に応じて `sf log check` が存在を要求するストリーム"
            "（上の表の「必須」列は実機の既定）。",
            "",
            "| 取得元 | 必須ストリーム |\n|---|---|",
        ]
    else:
        lines = [
            "### Required streams per source",
            "",
            "Streams `sf log check` requires depending on `meta.json`'s `source` "
            "(the \"Required\" column above is the vehicle default).",
            "",
            "| Source | Required streams |\n|---|---|",
        ]
    for source, names in spec["required_streams"].items():
        lines.append(f"| {source} | " + ", ".join(f"`{n}`" for n in names) + " |")
    return "\n".join(lines) + "\n"


def _stream_column_tables(spec: dict, lang: str) -> str:
    out = []
    for s in spec["streams"]:
        if lang == "ja":
            out.append(f"#### {s['name']}（`{s['file']}`）\n")
            out.append(
                f"由来: {_md_cell(s['source'])} ／ 公称レート: {_rate_str_ja(s['nominal_rate_hz'])} ／ "
                f"必須: {'必須' if s['required'] else '任意'}\n"
            )
            out.append("| 列名 | 型 | 単位 | 説明 |\n|---|---|---|---|")
            for c in s["columns"]:
                out.append(
                    f"| `{c['name']}` | {_md_cell(c['type'])} | {_md_cell(c['unit'])} | "
                    f"{_md_cell(c['description_ja'])} |"
                )
        else:
            out.append(f"#### {s['name']} (`{s['file']}`)\n")
            out.append(
                f"Source: {_md_cell(s['source'])} / Nominal rate: {_rate_str_en(s['nominal_rate_hz'])} / "
                f"Required: {'yes' if s['required'] else 'no'}\n"
            )
            out.append("| Column | Type | Unit | Description |\n|---|---|---|---|")
            for c in s["columns"]:
                out.append(
                    f"| `{c['name']}` | {_md_cell(c['type'])} | {_md_cell(c['unit'])} | "
                    f"{_md_cell(c['description_en'])} |"
                )
        out.append("")
    return "\n".join(out)


def _meta_fields_table(spec: dict, lang: str) -> str:
    if lang == "ja":
        header = "| キー | 型 | 説明 |\n|---|---|---|\n"
    else:
        header = "| Key | Type | Description |\n|---|---|---|\n"
    rows = []
    for key, info in spec["meta_fields"].items():
        desc = info["description_ja"] if lang == "ja" else info["description_en"]
        rows.append(f"| `{key}` | {_md_cell(info['type'])} | {_md_cell(desc)} |")
    return header + "\n".join(rows) + "\n"


def _units_table(spec: dict, lang: str) -> str:
    if lang == "ja":
        header = "| 物理量 | 単位 |\n|---|---|\n"
    else:
        header = "| Quantity | Unit |\n|---|---|\n"
    rows = [f"| {key} | {_md_cell(value)} |" for key, value in spec["units"].items()]
    return header + "\n".join(rows) + "\n"


def render_doc_md(spec: dict) -> str:
    """Render docs/reference/flight-log-format.md (Japanese section first,
    English section second, per the repository's bilingual doc convention
    -- see CLAUDE.md "Documentation").
    docs/reference/flight-log-format.md を生成する（リポジトリのバイリンガル
    文書規約どおり、日本語セクションを先に、英語セクションを後に置く --
    CLAUDE.md の「Documentation」参照）。
    """
    naming = spec["file_naming"]
    csv_rules = spec["csv_rules"]

    lines: list[str] = []
    lines.append(f"# StampFly Flight Log Format v{spec['version']}")
    lines.append("")
    lines.append(
        "> **Note:** [English version follows after the Japanese section.]"
        "(#english) / 日本語の後に英語版があります。"
    )
    lines.append("")
    lines.append(f"<!-- {GENERATED_NOTICE} -->")
    lines.append(
        f"<!-- Source of truth / 正本: "
        f"{SPEC_PATH.relative_to(REPO_ROOT).as_posix()} -->"
    )
    lines.append("")

    # ---- Japanese section ----
    lines.append("## 1. 概要")
    lines.append("")
    lines.append(
        "StampFly フライトログ一式（`.sflog.zip`、拡張子固定）は、1回の飛行/"
        "シミュレーション実行につき1個の zip ファイルに、取得条件を記した "
        "`meta.json`、列定義を記した `schema.json`、パケット種別ごとの CSV "
        "を平坦に格納したもの。決定文書: "
        "`docs/plans/flight-log-format-plan.md`。"
    )
    lines.append("")
    lines.append("### 容器")
    lines.append("")
    lines.append(
        f"種別: {spec['container']['type']}（{spec['container']['compression']} 圧縮）／"
        f"拡張子: `{spec['container']['extension']}`／"
        f"レイアウト: {spec['container']['layout']}（サブフォルダ無し）。"
        "展開済みフォルダも同じレイアウトで読み込み可能。未知のファイルは無視する。"
    )
    lines.append("")
    lines.append("### ファイル命名")
    lines.append("")
    lines.append("| 由来 | 命名パターン |\n|---|---|")
    lines.append(f"| 実機 (vehicle) | `{naming['vehicle']}` |")
    lines.append(f"| SILS | `{naming['sils']}` |")
    lines.append(f"| シミュレータ (sim) | `{naming['sim']}` |")
    lines.append("")
    lines.append("### CSV の規則")
    lines.append("")
    lines.append(
        f"文字コード {csv_rules['encoding']}、1行目が列名、コメント行なし。"
        f"1列目は常に `{csv_rules['first_column']}`（{csv_rules['first_column_type']}）。"
        f"数値は `{csv_rules['float_format']}` 形式。{csv_rules['empty_cell_policy_ja']}"
    )
    lines.append("")
    lines.append("### 単位（SI 系）")
    lines.append("")
    lines.append(_units_table(spec, "ja"))
    lines.append("## 2. ストリーム一覧")
    lines.append("")
    lines.append(_stream_overview_table(spec, "ja"))
    lines.append(_required_by_source_table(spec, "ja"))
    lines.append("## 3. 各ストリームの列")
    lines.append("")
    lines.append(_stream_column_tables(spec, "ja"))
    lines.append("## 4. meta.json")
    lines.append("")
    lines.append(_meta_fields_table(spec, "ja"))
    lines.append(
        "`schema.json` はこの文書のもとになった `protocol/spec/flight_log.yaml` "
        "から、バンドルに実際に含まれるストリームだけを抜き出して埋め込む "
        "（`lib/sflog/schema.py` の `schema_for()` が生成）。"
    )
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append('<a id="english"></a>')
    lines.append("")

    # ---- English section ----
    lines.append("## 1. Overview")
    lines.append("")
    lines.append(
        "A StampFly flight-log bundle (`.sflog.zip`, fixed extension) packs "
        "one zip file per flight/simulation run, containing `meta.json` "
        "(capture conditions), `schema.json` (column definitions), and one "
        "CSV per packet type, all flat (no subfolders). Decision document: "
        "`docs/plans/flight-log-format-plan.md`."
    )
    lines.append("")
    lines.append("### Container")
    lines.append("")
    lines.append(
        f"Type: {spec['container']['type']} ({spec['container']['compression']} "
        f"compression) / Extension: `{spec['container']['extension']}` / "
        f"Layout: {spec['container']['layout']} (no subfolders). Readers also "
        "accept an already-extracted directory with the same layout. Unknown "
        "files are ignored."
    )
    lines.append("")
    lines.append("### File naming")
    lines.append("")
    lines.append("| Source | Naming pattern |\n|---|---|")
    lines.append(f"| vehicle | `{naming['vehicle']}` |")
    lines.append(f"| SILS | `{naming['sils']}` |")
    lines.append(f"| sim | `{naming['sim']}` |")
    lines.append("")
    lines.append("### CSV rules")
    lines.append("")
    lines.append(
        f"Encoding {csv_rules['encoding']}, header row, no comment rows. "
        f"Column 1 is always `{csv_rules['first_column']}` "
        f"({csv_rules['first_column_type']}). Numbers use `{csv_rules['float_format']}`. "
        f"{csv_rules['empty_cell_policy_en']}"
    )
    lines.append("")
    lines.append("### Units (SI)")
    lines.append("")
    lines.append(_units_table(spec, "en"))
    lines.append("## 2. Stream list")
    lines.append("")
    lines.append(_stream_overview_table(spec, "en"))
    lines.append(_required_by_source_table(spec, "en"))
    lines.append("## 3. Columns per stream")
    lines.append("")
    lines.append(_stream_column_tables(spec, "en"))
    lines.append("## 4. meta.json")
    lines.append("")
    lines.append(_meta_fields_table(spec, "en"))
    lines.append(
        "`schema.json` is embedded per-bundle from the same "
        "`protocol/spec/flight_log.yaml` this document is generated from, "
        "restricted to the streams actually present (built by "
        "`schema_for()` in `lib/sflog/schema.py`)."
    )
    lines.append("")

    return "\n".join(lines)


# =============================================================================
# CLI
# =============================================================================


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--check",
        action="store_true",
        help="Regenerate in memory and exit non-zero if generated files are stale (CI).",
    )
    group.add_argument(
        "--write",
        action="store_true",
        help="Regenerate and write both generated files (default when no flag is given).",
    )
    args = parser.parse_args()

    spec = load_spec()
    schema_py_text = render_schema_py(spec)
    doc_md_text = render_doc_md(spec)

    if args.check:
        stale = []
        for path, text in ((SCHEMA_PY_PATH, schema_py_text), (DOC_MD_PATH, doc_md_text)):
            if not path.exists() or path.read_text(encoding="utf-8") != text:
                stale.append(path)
        if stale:
            print("gen_flight_log.py --check: STALE, regenerate with --write:")
            for path in stale:
                print(f"  {path.relative_to(REPO_ROOT)}")
            return 1
        print("gen_flight_log.py --check: OK (generated files are up to date)")
        return 0

    # --write (also the default when neither flag is given)
    # --write（フラグ無指定の既定でもある）
    SCHEMA_PY_PATH.parent.mkdir(parents=True, exist_ok=True)
    DOC_MD_PATH.parent.mkdir(parents=True, exist_ok=True)
    SCHEMA_PY_PATH.write_text(schema_py_text, encoding="utf-8")
    DOC_MD_PATH.write_text(doc_md_text, encoding="utf-8")
    print(f"wrote {SCHEMA_PY_PATH.relative_to(REPO_ROOT)}")
    print(f"wrote {DOC_MD_PATH.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
