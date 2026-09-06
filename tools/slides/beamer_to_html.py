#!/usr/bin/env python3
"""beamer_to_html.py — Convert the SCI/SICE tutorial Beamer deck to a
single-page HTML slide deck (docs/events/sci_tutorial_2026/web/index.html).

Beamer(LaTeX)のスライドを1枚のHTMLデッキに変換するツール。
LaTeXの完全な文法を実装するのではなく、このデッキで実際に使われている
構文（frame/block/itemize/tabular/sflisting/TikZ 入力/数式など）だけを
対象にした、目的特化の再帰下降パーサである。

Usage / 使い方:
    python3 beamer_to_html.py --sci-dir sci_tutorial_2026 \
        --tikz-dir _shared/tikz --images-dir _shared/images --build-tikz

Only the Python standard library is used (stdlib only, no pip install).
標準ライブラリのみを使用する（pip install 不要）。
"""
from __future__ import annotations

import argparse
import html
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

# ---------------------------------------------------------------------------
# Brand colors (Single Source of Truth: docs/events/_shared/beamer/
# stampfly_slides.sty). Kept in sync manually — this script only READS the
# .sty file's colors as documented in the task brief, it does not parse it.
# ブランドカラー（正本は stampfly_slides.sty）。このスクリプトは .sty を
# パースせず、ブリーフに明記された値を手動で転記して同期させる。
# ---------------------------------------------------------------------------
BRAND_COLORS = {
    "sfblue": "0,120,200",
    "sfdark": "0,60,100",
    "sflight": "220,240,255",
    "sfgray": "80,80,80",
    "sfred": "220,50,30",
    "sfgreen": "40,160,40",
    "black": "0,0,0",
    "white": "255,255,255",
}

# Counters incremented while walking the parsed tree, used for the final
# conversion report (frames converted, TODO markers per frame).
# 変換レポート用のカウンタ（グローバルな1状態として簡潔に保持する）。
STATS = {"frames": 0, "todo_total": 0, "todo_by_frame": {}}


# ===========================================================================
# Section 1 — low level text utilities
# 低レベルのテキストユーティリティ
# ===========================================================================

def strip_comments_outside_verbatim(text: str) -> str:
    """Remove LaTeX '%' comments, but never inside sflisting/lstlisting/
    tikzpicture bodies (which may contain a literal '%', e.g. printf("%.1f")).
    LaTeXの'%'コメントを除去する。ただし sflisting/lstlisting/tikzpicture の
    本文中（printf("%.1f") 等、リテラルの'%'を含みうる）では除去しない。
    """
    verbatim_envs = ("sflisting", "lstlisting", "tikzpicture")
    out = []
    i, n = 0, len(text)
    verbatim_end = None  # literal string that closes the current verbatim run
    while i < n:
        if verbatim_end is not None:
            if text.startswith(verbatim_end, i):
                out.append(verbatim_end)
                i += len(verbatim_end)
                verbatim_end = None
            else:
                out.append(text[i])
                i += 1
            continue
        for env in verbatim_envs:
            begin = "\\begin{%s}" % env
            if text.startswith(begin, i):
                out.append(begin)
                i += len(begin)
                verbatim_end = "\\end{%s}" % env
                break
        else:
            if text[i] == "%" and (i == 0 or text[i - 1] != "\\"):
                j = text.find("\n", i)
                i = n if j == -1 else j
                continue
            out.append(text[i])
            i += 1
    return "".join(out)


def find_balanced_close(text: str, open_pos: int) -> int:
    """Given the index of an opening '{', return the index of its matching
    '}' (brace-depth aware). 対応する閉じ'}'の位置を返す（深さを数える）。
    """
    assert text[open_pos] == "{"
    depth = 0
    i = open_pos
    n = len(text)
    while i < n:
        c = text[i]
        if c == "\\" and i + 1 < n:
            i += 2
            continue
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return n - 1  # malformed input: best-effort, stop at end


def read_brace_raw(text: str, pos: int):
    """Read a raw (unparsed) {...} group starting at pos. Returns
    (inner_text, pos_after_closing_brace). 未解析の {...} を読み取る。"""
    assert text[pos] == "{"
    close = find_balanced_close(text, pos)
    return text[pos + 1:close], close + 1


def try_read_bracket_raw(text: str, pos: int):
    """Optionally read a raw [...] group (no nesting expected in this
    corpus). Returns (inner_text_or_None, new_pos).
    任意の [...] を読む（本デッキでは入れ子は出現しない）。"""
    if pos < len(text) and text[pos] == "[":
        close = text.find("]", pos)
        if close != -1:
            return text[pos + 1:close], close + 1
    return None, pos


# ---------------------------------------------------------------------------
# Escaping / typography for prose text runs
# 地の文の脱エスケープ・タイポグラフィ変換
# ---------------------------------------------------------------------------

ESCAPED_CHAR_MAP = {
    "_": "_", "%": "%", "&": "&", "#": "#", "$": "$",
    "{": "{", "}": "}",
}


def convert_dashes(run: str) -> str:
    """Typographic dash conversion for PROSE only (never for \\code/\\api/
    \\texttt/\\url or listing bodies, where "--" is a literal CLI flag
    prefix such as "--target"). プレーンな地の文専用のダッシュ変換。"""
    return run.replace("---", "—").replace("--", "–")


def clean_verbatim(raw: str) -> str:
    """De-escape LaTeX specials for \\code/\\api/\\texttt/\\url content.
    No dash conversion (verbatim CLI flags must keep "--").
    \\code 等の中身: バックスラッシュ・エスケープのみ解除しダッシュ変換はしない。
    """
    out = []
    i, n = 0, len(raw)
    while i < n:
        c = raw[i]
        if c == "\\" and i + 1 < n and raw[i + 1] in ESCAPED_CHAR_MAP:
            out.append(ESCAPED_CHAR_MAP[raw[i + 1]])
            i += 2
        elif c == "\\" and i + 1 < n and raw[i + 1] == ",":
            out.append(" ")
            i += 2
        elif c == "\\" and raw[i + 1:i + 2] == "S" and not raw[i + 2:i + 3].isalpha():
            # \S (section sign) occasionally sits inside \code{...}, e.g.
            # \code{architecture.md \S2}. \S（節記号）が\code{}内に現れる例。
            out.append("§")
            i += 2
        else:
            out.append(c)
            i += 1
    return "".join(out)


def esc(s: str) -> str:
    """HTML-escape final display text (already de-escaped/dash-converted).
    表示用に確定したテキストをHTMLエスケープする。"""
    return html.escape(s)


# ===========================================================================
# Section 2 — verbatim pre-extraction (sflisting / lstlisting / tikzpicture)
# 検証本文の事前抽出（コードリスティングとインラインTikZ）
#
# These bodies must be pulled out BEFORE comment-stripping (they may contain
# a literal '%') and before the generic tokenizer runs (their syntax is not
# LaTeX prose). Each becomes a \SFRAW{n} placeholder command.
# コメント除去より前（リテラル'%'を含みうる）、かつ一般トークナイザより前に
# 取り出す必要がある（構文がLaTeXの地の文ではないため）。それぞれを
# \SFRAW{n} という置き換えコマンドに差し替える。
# ===========================================================================

VERBATIM_ENV_RE = re.compile(
    r"\\begin\{(sflisting|lstlisting|tikzpicture)\}"
)


@dataclass
class RawBlock:
    kind: str            # 'sflisting' | 'lstlisting' | 'tikzpicture'
    opt: str | None      # raw [...] options, if any
    title: str | None    # sflisting's mandatory {title} arg (raw)
    body: str            # verbatim body text


def extract_verbatim_blocks(text: str, store: list[RawBlock]) -> str:
    """Replace each sflisting/lstlisting/tikzpicture block with a
    \\SFRAW{index} placeholder, appending the extracted RawBlock to `store`.
    各検証本文を \\SFRAW{index} に置換し、抽出結果を store に積む。"""
    out = []
    pos = 0
    while True:
        m = VERBATIM_ENV_RE.search(text, pos)
        if not m:
            out.append(text[pos:])
            break
        out.append(text[pos:m.start()])
        kind = m.group(1)
        cursor = m.end()
        opt = None
        title = None
        if kind in ("sflisting", "lstlisting"):
            # Only these two ever carry a simple, non-nested [...] option
            # list. tikzpicture's own [...] (TikZ style options) commonly
            # nests a further "[...]" (e.g. Stealth[length=2.5mm]), so it
            # must stay untouched inside the verbatim body instead.
            # sflisting/lstlisting だけが単純な非入れ子 [...] を持つ。
            # tikzpicture の [...] はしばしば入れ子（例:
            # Stealth[length=2.5mm]）を含むため、本文にそのまま残す。
            opt, cursor = try_read_bracket_raw(text, cursor)
        if kind == "sflisting":
            title, cursor = read_brace_raw(text, cursor)
        end_tag = "\\end{%s}" % kind
        end_pos = text.find(end_tag, cursor)
        body = text[cursor:end_pos]
        store.append(RawBlock(kind, opt, title, body))
        out.append("\\SFRAW{%d}" % (len(store) - 1))
        pos = end_pos + len(end_tag)
    return "".join(out)


# ===========================================================================
# Section 2b — `% @web:` marker extraction (web-only widgets/videos)
# `% @web:` マーカー抽出（Web版限定のウィジェット・動画）
#
# LaTeX comments are invisible to lualatex, so `% @web: ...` lines are safe
# to leave in the .tex sources: the PDF deck is unaffected. This converter
# reads them from the RAW (not-yet-comment-stripped) per-frame text, before
# strip_comments_outside_verbatim() would erase them, and attaches the
# result to that Frame so make_content_slide() can splice in the extra HTML
# (see apply_web_markers in Section 6).
# LaTeXのコメントはlualatexから見えないため、`% @web: ...` 行はそのまま
# .texソースに残してもPDFデッキに影響しない。本コンバータはこれを
# strip_comments_outside_verbatim()でコメントが消される前の、フレーム単位の
# 生テキストから読み取り、そのFrameに結果を添付する（make_content_slide()が
# 追加HTMLを合成する、詳細はSection 6のapply_web_markers参照）。
# ===========================================================================

WEB_MARKER_LINE_RE = re.compile(r"^[ \t]*%[ \t]*@web:[ \t]*(.*)$", re.MULTILINE)
WEB_MARKER_KEY_RE = re.compile(r"(widget|axis|preset|alpha|height|video|caption|layout)=")


@dataclass
class WebWidgetSpec:
    widget: str            # 'pid_step' | 'mixer' | 'comp_filter'
    attrs: dict            # extra data-* attrs (axis, preset, alpha, ...)
    height: int | None


@dataclass
class WebVideoSpec:
    src: str               # raw marker path, e.g. "../fallback/S4_....mp4"
    caption: str


@dataclass
class WebMarkers:
    widgets: list           # list[WebWidgetSpec]
    videos: list             # list[WebVideoSpec]
    layout: str              # 'below' (default) | 'right' | 'replace'

    def is_empty(self) -> bool:
        return not self.widgets and not self.videos


def parse_web_marker_fields(content: str) -> dict:
    """Parse one '@web:' line's "k=v k=v ..." content into a dict. A value
    runs until the next recognized key= token (or end of line), so
    `caption=実習 5（P のみ）` can freely contain spaces/parentheses without
    a naive whitespace split cutting it short.
    '@web:' 1行分の "k=v k=v ..." を辞書へ変換する。値は次の既知キーの
    key= が現れるまで（無ければ行末まで）を1つの値として読む — 単純な
    空白区切りでは `caption=実習 5（P のみ）` のような値が途中で
    切れてしまうため。"""
    matches = list(WEB_MARKER_KEY_RE.finditer(content))
    fields = {}
    for i, m in enumerate(matches):
        end = matches[i + 1].start() if i + 1 < len(matches) else len(content)
        fields[m.group(1)] = content[m.end():end].strip()
    return fields


def extract_web_markers(frame_raw: str) -> WebMarkers:
    """Scan one frame's RAW text (still carrying '%' comments) for every
    `% @web: ...` line and fold them into one WebMarkers for that frame.
    Multiple `widget=`/`video=` lines accumulate; a `layout=` line (order-
    independent) sets the frame's layout, defaulting to 'below'.
    1フレームの生テキストから `% @web: ...` 行をすべて拾い、1つの
    WebMarkersにまとめる。`widget=`/`video=` は複数行あれば積み上がり、
    `layout=` はどこにあってもそのフレームのレイアウトを決める
    （既定は 'below'）。"""
    widgets: list = []
    videos: list = []
    layout = "below"
    for m in WEB_MARKER_LINE_RE.finditer(frame_raw):
        fields = parse_web_marker_fields(m.group(1))
        if "layout" in fields:
            layout = fields["layout"].strip() or layout
        if "widget" in fields:
            height = int(fields["height"]) if fields.get("height") else None
            attrs = {k: v for k, v in fields.items()
                     if k not in ("widget", "height", "layout")}
            widgets.append(WebWidgetSpec(widget=fields["widget"], attrs=attrs, height=height))
        elif "video" in fields:
            videos.append(WebVideoSpec(src=fields["video"], caption=fields.get("caption", "")))
    return WebMarkers(widgets=widgets, videos=videos, layout=layout)


def is_commented_out(text: str, pos: int) -> bool:
    """True if `pos` sits inside a LaTeX '%' comment on its own line (an
    unescaped '%' appears earlier on the same line). Used defensively when
    locating \\begin{frame}/\\end{frame} in text whose comments have NOT
    been stripped yet (see parse_chapter_frames), so a stray `% \\begin
    {frame}` inside a docstring is never mistaken for a real frame
    boundary. `pos` がその行内で（エスケープされていない）'%' より後に
    あるかどうかを判定する。コメント未除去のテキストでフレーム境界を
    探す際の防御用（docstring中の`% \\begin{frame}`等を実境界と誤認
    しないため）。"""
    line_start = text.rfind("\n", 0, pos) + 1
    i = line_start
    while i < pos:
        if text[i] == "\\" and i + 1 < pos:
            i += 2
            continue
        if text[i] == "%":
            return True
        i += 1
    return False


def find_uncommented(text: str, needle: str, start: int) -> int:
    """Like str.find, but skips any match that is_commented_out() flags.
    str.findと同じだが、is_commented_out()がコメント内と判定した
    マッチは読み飛ばす。"""
    pos = start
    while True:
        idx = text.find(needle, pos)
        if idx == -1 or not is_commented_out(text, idx):
            return idx
        pos = idx + len(needle)


# ===========================================================================
# Section 3 — Node tree and the recursive-descent parser
# ノード木と再帰下降パーサ
# ===========================================================================

# Inline (raw \begin{tikzpicture}) figures found while walking frame bodies.
# name -> full "\begin{tikzpicture}...\end{tikzpicture}" source, collected
# globally so the TikZ build stage (Section 7) can compile them all.
# フレーム本文中で見つかったインラインTikZ図（name -> ソース全文）。
# TikZビルド段でまとめてコンパイルできるようグローバルに集約する。
INLINE_TIKZ_SOURCES: dict[str, str] = {}
_inline_tikz_counters: dict[str, int] = {}


def next_inline_tikz_name(chapter: str) -> str:
    """Allocate a stable, deterministic name for the Nth inline tikzpicture
    found in `chapter`. 章内で見つかったN番目のインラインTikZ図に名前を割当。
    """
    _inline_tikz_counters[chapter] = _inline_tikz_counters.get(chapter, 0) + 1
    return "%s_inline_tikz_%d" % (chapter, _inline_tikz_counters[chapter])


# Command dispatch tables. Each maps a LaTeX command name to how its
# arguments are consumed and what Node it produces. Anything NOT listed
# here falls through to the "todo_cmd" branch — nothing is ever silently
# eaten without at least a chance to be flagged.
# コマンド分配表。ここに載っていないコマンドは todo_cmd 分岐へ落ち、
# 黙って消えることはない（必ずTODOとして拾える）。
VERBATIM_INLINE_CMDS = {"code", "api", "texttt", "url"}
RECURSIVE_INLINE_CMDS = {
    "textbf": "strong",
    "emph": "em",
    "warn": "warn",
    "textsuperscript": "sup",
}
ZERO_ARG_TEXT = {
    "S": "§",
    "checkmark": "✓",
    "footnotemark": '<sup class="footnotemark">*</sup>',
    "quad": "&nbsp;&nbsp;&nbsp;&nbsp;",
    "qquad": "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;",
}
SIZE_SCOPE = {
    "tiny": "fs-tiny",
    "scriptsize": "fs-scriptsize",
    "footnotesize": "fs-footnotesize",
    "small": "fs-small",
    "normalsize": "fs-normal",
    "large": "fs-large",
    "Large": "fs-large2",
}
DROP_ZERO_ARG = {
    "par", "raggedright", "arraybackslash", "bfseries", "ttfamily",
    "normalfont", "sffamily", "itshape", "hline", "noindent",
}
TRANSPARENT_WRAP = {"resizebox", "adjustbox", "mbox"}
FIGURE_CMDS = {"includegraphics", "input", "scisessionmap"}
TWO_BRACE_DROP = {"setlength", "renewcommand", "newcommand", "providecommand"}

ALIGN_LETTERS = {"l": "left", "c": "center", "r": "right"}
MULTICOL_RE = re.compile(r"^\\multicolumn\{(\d+)\}\{([^}]*)\}\{")
ROW_SEP_RE = re.compile(r"\\\\\s*(?:\[[^\]]*\])?")
CELL_SEP_RE = re.compile(r"&")


def split_top_level(text: str, sep_re: re.Pattern) -> list[str]:
    """Split `text` on `sep_re`, but only where brace-depth is 0. A
    backslash-escaped char is always consumed as one atomic unit first, so
    "\\&" (literal ampersand) never matches a bare "&" separator.
    brace深さ0の位置でのみ分割する。バックスラッシュ・エスケープは常に
    2文字1組として先に消費するため、"\\&"（リテラルの&）は区切りに
    誤ってマッチしない。"""
    parts, buf, depth, i, n = [], [], 0, 0, len(text)
    while i < n:
        c = text[i]
        if c in "{}":
            depth += 1 if c == "{" else -1
            buf.append(c)
            i += 1
            continue
        if depth == 0:
            m = sep_re.match(text, i)
            if m:
                parts.append("".join(buf))
                buf = []
                i = m.end()
                continue
        if c == "\\" and i + 1 < n:
            buf.append(text[i:i + 2])
            i += 2
            continue
        buf.append(c)
        i += 1
    parts.append("".join(buf))
    return parts


def split_tabular_rows(raw_body: str):
    """Split into row strings, and also detect whether the FIRST row is a
    header: that is true iff a \\hline sits between row 0 and row 1 (a
    plain leading/trailing \\hline is just the table's top/bottom border
    and proves nothing by itself — some tables here have no header row at
    all, only \\hline top and bottom, e.g. the S1..S5 schedule table).
    行に分割し、先頭行が見出しかどうかも判定する: 行0と行1の間に\\hline
    があるかどうかで判定する（先頭・末尾だけの\\hlineは単なる外枠であり、
    見出し無しの表もある — 例: S1〜S5の時間割表）。"""
    rows: list[str] = []
    has_header_sep = False
    for piece in split_top_level(raw_body, ROW_SEP_RE):
        had_hline_before = bool(re.match(r"\s*\\hline", piece))
        cleaned = re.sub(r"\\hline", "", piece).strip()
        if not cleaned:
            continue
        if len(rows) == 1 and had_hline_before:
            has_header_sep = True
        rows.append(cleaned)
    return rows, has_header_sep


def split_row_cells(row_text: str) -> list[str]:
    return split_top_level(row_text, CELL_SEP_RE)


def parse_colspec(spec: str) -> list[str]:
    """Turn a tabular column spec (e.g. "l>{\\raggedright}p{55mm}cc") into
    a list of CSS text-align values, one per column.
    列指定文字列を列ごとの text-align 値のリストに変換する。"""
    aligns = []
    i, n = 0, len(spec)
    while i < n:
        c = spec[i]
        if c in "lcr":
            aligns.append(ALIGN_LETTERS[c])
            i += 1
        elif c == "p" or c == ">":
            j = spec.find("{", i)
            if j == -1:
                break
            close = find_balanced_close(spec, j)
            if c == "p":
                aligns.append("left")
            i = close + 1
        else:
            i += 1
    return aligns


def append_result(nodes: list, result) -> None:
    """Append a parse result that may be None (drop), a single Node, or a
    list (splice) — used for transparent wrappers (\\resizebox/\\adjustbox/
    \\mbox) whose content must join the PARENT's node list directly so a
    wrapped figure/table is still classified as block-level there.
    None/単一Node/リスト（展開）を受け取る。透過ラッパー
    （\\resizebox等）の中身を親のノード列へ直接合流させ、図・表が
    ブロック要素として正しく分類されるようにするため。"""
    if result is None:
        return
    if isinstance(result, list):
        nodes.extend(result)
    else:
        nodes.append(result)


class Node:
    """A generic tree node. `kind` selects the renderer; other attributes
    are kind-specific (see the dispatch tables below).
    汎用ツリーノード。kind によって使うレンダラが決まる。"""

    def __init__(self, kind: str, **kw):
        self.kind = kind
        self.__dict__.update(kw)

    def __repr__(self):  # pragma: no cover - debugging aid
        d = {k: v for k, v in self.__dict__.items() if k != "kind"}
        return f"Node({self.kind}, {d})"


class Parser:
    """Recursive-descent parser over one already comment-stripped,
    placeholder-substituted chapter/frame text buffer.
    コメント除去・プレースホルダ置換済みのテキストに対する再帰下降パーサ。"""

    def __init__(self, text: str, raw_store: list[RawBlock], chapter: str):
        self.s = text
        self.n = len(text)
        self.raw_store = raw_store
        self.chapter = chapter
        self.inline_tikz_seq = 0

    def parse_until_env_end(self, env_name: str, pos: int):
        """Parse nodes until the literal \\end{env_name} marker."""
        return self._parse(pos, ("env", env_name))

    def parse_group(self, pos: int):
        """Parse nodes until the matching top-level '}' (the opening '{'
        must already have been consumed by the caller)."""
        return self._parse(pos, ("brace",))

    def parse_to_eof(self, pos: int):
        return self._parse(pos, ("eof",))

    # -- core loop ----------------------------------------------------
    def _parse(self, pos: int, stop):
        nodes: list[Node] = []
        s, n = self.s, self.n
        while pos < n:
            if stop[0] == "brace" and s[pos] == "}":
                return nodes, pos + 1
            if stop[0] == "env" and s.startswith("\\end{%s}" % stop[1], pos):
                return nodes, pos + len("\\end{%s}" % stop[1])
            if s.startswith("\\end{", pos):
                # Mismatched terminator (should not happen on well-formed
                # input) — stop defensively rather than loop forever.
                # 想定外の \end — 無限ループを避けて防御的に打ち切る。
                return nodes, pos
            c = s[pos]
            if s.startswith("\\[", pos):
                # Must be checked BEFORE the generic '\\' dispatch below,
                # or "\[" is mistaken for an escaped literal '['.
                # 汎用の'\\'分岐より先に判定しないと "\[" がエスケープ
                # されたリテラル'['と誤認識される。
                node, pos = self._parse_math(pos, display=True)
                nodes.append(node)
                continue
            if c == "\\":
                node, pos = self._parse_backslash(pos)
                append_result(nodes, node)
                continue
            if c == "$":
                node, pos = self._parse_math(pos, display=False)
                nodes.append(node)
                continue
            if c == "{":
                child_nodes, pos = self.parse_group(pos + 1)
                nodes.append(Node("group", children=child_nodes))
                continue
            # Plain text run up to the next special character ('\\','$','{','}').
            # 次の特殊文字までの地の文ラン。
            j = pos
            while j < n and s[j] not in "\\$}{":
                j += 1
            run = s[pos:j]
            nodes.extend(self._make_text_nodes(run))
            pos = j
        return nodes, pos

    def _make_text_nodes(self, run: str):
        """Split a plain-text run on blank lines into Text/ParaBreak nodes,
        applying dash conversion to each piece. 空行でParaBreakに分割する。"""
        pieces = re.split(r"\n[ \t]*\n+", run)
        out = []
        for i, piece in enumerate(pieces):
            if i > 0:
                out.append(Node("parabreak"))
            cleaned = convert_dashes(piece)
            if cleaned:
                out.append(Node("text", value=cleaned))
        return out

    def _parse_math(self, pos: int, display: bool):
        s = self.s
        if display:
            close = s.find("\\]", pos + 2)
            raw = s[pos + 2:close]
            return Node("math", display=True, raw=raw), close + 2
        close = pos + 1
        while close < self.n and not (s[close] == "$" and s[close - 1] != "\\"):
            close += 1
        raw = s[pos + 1:close]
        return Node("math", display=False, raw=raw), close + 1

    def _parse_backslash(self, pos: int):
        s, n = self.s, self.n
        nxt = s[pos + 1] if pos + 1 < n else ""
        if nxt == "\\":
            # "\\" line break, optionally followed by "[<dimension>]".
            pos += 2
            _, pos = try_read_bracket_raw(s, pos)
            return Node("br"), pos
        if nxt and not nxt.isalpha():
            # Escaped literal, e.g. "\_" "\%" "\," (thin space).
            if nxt == ",":
                return Node("text", value=" "), pos + 2
            if nxt in ESCAPED_CHAR_MAP:
                return Node("text", value=ESCAPED_CHAR_MAP[nxt]), pos + 2
            return Node("text", value=nxt), pos + 2
        j = pos + 1
        while j < n and (s[j].isalpha()):
            j += 1
        name = s[pos + 1:j]
        if name == "begin":
            return self._parse_begin(j)
        if name == "item":
            return self._parse_item(j)
        return self._parse_named_command(name, j)

    # -- environments ---------------------------------------------------
    def _parse_begin(self, pos: int):
        name, pos = read_brace_raw(self.s, pos)
        if name in ("block", "exampleblock", "alertblock"):
            return self._parse_titled_block(name, pos)
        if name in ("itemize", "enumerate"):
            body, pos = self.parse_until_env_end(name, pos)
            return Node("list", ordered=(name == "enumerate"), body=body), pos
        if name == "table":
            body, pos = self.parse_until_env_end(name, pos)
            return Node("table_wrap", body=body), pos
        if name == "tabular":
            return self._parse_tabular(pos)
        if name == "columns":
            _, pos = try_read_bracket_raw(self.s, pos)
            body, pos = self.parse_until_env_end(name, pos)
            return Node("columns", body=body), pos
        if name == "column":
            width, pos = read_brace_raw(self.s, pos)
            body, pos = self.parse_until_env_end(name, pos)
            return Node("column", width=width, body=body), pos
        if name == "center":
            body, pos = self.parse_until_env_end(name, pos)
            return Node("center", body=body), pos
        # Unknown environment: parse the body anyway (nothing lost) but
        # flag it so the report shows an unsupported construct was hit.
        body, pos = self.parse_until_env_end(name, pos)
        STATS["todo_total"] += 1
        return Node("todo_env", name=name, body=body), pos

    def _parse_titled_block(self, name: str, pos: int):
        title_nodes: list[Node] = []
        if pos < self.n and self.s[pos] == "{":
            title_nodes, pos = self.parse_group(pos + 1)
        body, pos = self.parse_until_env_end(name, pos)
        return Node("block", variant=name, title=title_nodes, body=body), pos

    def _parse_tabular(self, pos: int):
        colspec, pos = read_brace_raw(self.s, pos)
        end_tag = "\\end{tabular}"
        end_pos = self.s.find(end_tag, pos)
        raw_body = self.s[pos:end_pos]
        aligns = parse_colspec(colspec)
        row_texts, has_header_sep = split_tabular_rows(raw_body)
        rows = [self._parse_table_row(r, aligns) for r in row_texts]
        return (Node("tabular", aligns=aligns, rows=rows, has_header_sep=has_header_sep),
                end_pos + len(end_tag))

    def _parse_table_row(self, row_text: str, aligns: list[str]):
        return [self._parse_table_cell(c, i, aligns)
                for i, c in enumerate(split_row_cells(row_text))]

    def _parse_table_cell(self, raw_cell: str, col_index: int, aligns: list[str]):
        stripped = raw_cell.strip()
        m = MULTICOL_RE.match(stripped)
        if m:
            colspan = int(m.group(1))
            align = ALIGN_LETTERS.get(m.group(2).strip(), "left")
            open_pos = m.end() - 1
            close_pos = find_balanced_close(stripped, open_pos)
            inner = stripped[open_pos + 1:close_pos]
            content, _ = Parser(inner, self.raw_store, self.chapter).parse_to_eof(0)
            return Node("cell", colspan=colspan, align=align, content=content)
        align = aligns[col_index] if col_index < len(aligns) else "left"
        content, _ = Parser(raw_cell, self.raw_store, self.chapter).parse_to_eof(0)
        return Node("cell", colspan=1, align=align, content=content)

    def _parse_item(self, pos: int):
        marker = None
        if pos < self.n and self.s[pos] == "[":
            opt_raw, pos = try_read_bracket_raw(self.s, pos)
            marker, _ = Parser(opt_raw, self.raw_store, self.chapter).parse_to_eof(0)
        return Node("item", marker=marker), pos

    # -- commands ---------------------------------------------------------
    def _parse_named_command(self, name: str, pos: int):
        s = self.s
        if name == "SFRAW":
            idx_raw, pos = read_brace_raw(s, pos)
            return self._resolve_raw_block(int(idx_raw)), pos
        if name == "later":
            return Node("later_marker"), pos
        if name in VERBATIM_INLINE_CMDS:
            raw = ""
            if pos < self.n and s[pos] == "{":
                raw, pos = read_brace_raw(s, pos)
            return Node("verbatim_inline", cmd=name, text=clean_verbatim(raw)), pos
        if name in RECURSIVE_INLINE_CMDS:
            content: list[Node] = []
            if pos < self.n and s[pos] == "{":
                content, pos = self.parse_group(pos + 1)
            return Node("inline_fmt", tag=RECURSIVE_INLINE_CMDS[name], children=content), pos
        if name == "textcolor":
            return self._parse_textcolor(pos)
        if name == "footnotetext":
            content = []
            if pos < self.n and s[pos] == "{":
                content, pos = self.parse_group(pos + 1)
            return Node("footnotetext", children=content), pos
        if name in ZERO_ARG_TEXT:
            return Node("text_html", html_value=ZERO_ARG_TEXT[name]), pos
        if name in SIZE_SCOPE:
            return Node("scope", cls=SIZE_SCOPE[name]), pos
        if name == "centering":
            return Node("scope", cls="align-center"), pos
        if name in DROP_ZERO_ARG:
            return None, pos
        if name in ("vspace", "vspace*"):
            if pos < self.n and s[pos] == "{":
                _, pos = read_brace_raw(s, pos)
            return Node("parabreak"), pos
        if name in TWO_BRACE_DROP:
            return self._drop_upto_two_braces(pos)
        if name in TRANSPARENT_WRAP:
            return self._parse_transparent(name, pos)
        if name in FIGURE_CMDS:
            return self._parse_figure_cmd(name, pos)
        # Unknown command: never silently discard — surface it as a TODO
        # marker. Any following {...} is left for the main loop to parse
        # as an ordinary (transparent) group, so its content still shows.
        # 未知コマンドは黙って捨てずTODOとして表面化する。後続の{...}は
        # 通常のグループとして扱われ、中身は表示され続ける。
        STATS["todo_total"] += 1
        return Node("todo_cmd", name=name), pos

    def _parse_textcolor(self, pos: int):
        color, pos = read_brace_raw(self.s, pos)
        content: list[Node] = []
        if pos < self.n and self.s[pos] == "{":
            content, pos = self.parse_group(pos + 1)
        return Node("color", color=color.strip(), children=content), pos

    def _drop_upto_two_braces(self, pos: int):
        s = self.s
        if pos < self.n and s[pos] == "[":
            _, pos = try_read_bracket_raw(s, pos)
        for _ in range(2):
            if pos < self.n and s[pos] == "{":
                _, pos = read_brace_raw(s, pos)
        return None, pos

    def _parse_transparent(self, name: str, pos: int):
        s = self.s
        if name == "resizebox":
            for _ in range(2):
                if pos < self.n and s[pos] == "{":
                    _, pos = read_brace_raw(s, pos)
        elif name == "adjustbox":
            # adjustbox takes its key=value options as a BRACE group (not a
            # bracket group), e.g. \adjustbox{max width=..., max
            # height=...}{content}. adjustboxはオプションを角括弧ではなく
            # 波括弧で取る。
            if pos < self.n and s[pos] == "{":
                _, pos = read_brace_raw(s, pos)
        content: list[Node] = []
        if pos < self.n and s[pos] == "{":
            content, pos = self.parse_group(pos + 1)
        # Splice (not wrap): the content must join the caller's own node
        # list so a wrapped figure/table is classified as block-level
        # there, instead of being trapped inside an inline-only container.
        # ラップせず展開する: 中身は呼び出し元のノード列へ合流させる。
        # そうしないと図・表がインライン専用の入れ物に閉じ込められる。
        return content, pos

    def _parse_figure_cmd(self, name: str, pos: int):
        s = self.s
        if name == "includegraphics":
            _, pos = try_read_bracket_raw(s, pos)
            path, pos = read_brace_raw(s, pos)
            return Node("figure", ftype="img", ref=path.strip()), pos
        if name == "input":
            path, pos = read_brace_raw(s, pos)
            tikz_name = path.strip().rsplit("/", 1)[-1]
            return Node("figure", ftype="tikz", ref=tikz_name), pos
        digit, pos = read_brace_raw(s, pos)  # scisessionmap{N}
        return Node("figure", ftype="tikz", ref="scisessionmap_%s" % digit.strip()), pos

    def _resolve_raw_block(self, idx: int):
        rb = self.raw_store[idx]
        if rb.kind in ("sflisting", "lstlisting"):
            return self._make_listing_node(rb)
        name = next_inline_tikz_name(self.chapter)
        INLINE_TIKZ_SOURCES[name] = "\\begin{tikzpicture}%s\\end{tikzpicture}" % rb.body
        return Node("figure", ftype="inline_tikz", ref=name)

    def _make_listing_node(self, rb: RawBlock):
        lang = "cpp"
        if rb.opt:
            m = re.search(r"style\s*=\s*(sfcpp|sfbash|sfpython)", rb.opt)
            if m:
                lang = {"sfcpp": "cpp", "sfbash": "bash", "sfpython": "python"}[m.group(1)]
        title = clean_verbatim(rb.title) if rb.title else None
        return Node("listing", lang=lang, title=title, body=rb.body.strip("\n"))


# ===========================================================================
# Section 4 — rendering: Node tree -> HTML strings
# レンダリング: ノード木 -> HTML文字列
# ===========================================================================

BLOCK_NODE_KINDS = {
    "block", "list", "table_wrap", "tabular", "columns", "column", "center",
    "figure", "listing", "footnotetext", "todo_env", "scopebox",
}
BLOCK_TITLE_CSS = {"block": "block", "exampleblock": "example", "alertblock": "alert"}
STAGE_LABELS = ("見るだけ", "一緒に打つ", "帰宅後に再現")


def strip_tags(s: str) -> str:
    return re.sub("<[^>]+>", "", s)


def apply_scopes(nodes: list[Node], wrap_tag: str) -> list[Node]:
    """Resolve \\small/\\centering-style scope markers: everything after a
    'scope' node (to the end of THIS node list) gets wrapped in one
    <wrap_tag class="..."> element, mirroring LaTeX group scoping.
    \\small等のスコープ標識を解決する。'scope'ノード以降の全兄弟ノードを
    1つの <wrap_tag class="..."> にまとめ、LaTeXのグループスコープを模す。
    """
    resolved = []
    for nd in nodes:
        if nd.kind == "group":
            nd.children = apply_scopes(nd.children, "span")
        resolved.append(nd)
    return fold_scope_siblings(resolved, wrap_tag)


def fold_scope_siblings(nodes: list[Node], wrap_tag: str) -> list[Node]:
    for i, nd in enumerate(nodes):
        if nd.kind == "scope":
            rest = fold_scope_siblings(nodes[i + 1:], wrap_tag)
            box = Node("scopebox", tag=wrap_tag, cls=nd.cls, children=rest)
            return nodes[:i] + [box]
    return nodes


def render_inline(nodes: list[Node]) -> str:
    nodes = apply_scopes(nodes, "span")
    return "".join(render_inline_node(nd) for nd in nodes)


def render_inline_node(nd: Node) -> str:
    k = nd.kind
    if k == "text":
        return esc(nd.value)
    if k == "text_html":
        return nd.html_value
    if k == "later_marker":
        return ""
    if k == "br":
        return "<br>"
    if k == "parabreak":
        return " "
    if k == "group":
        return render_inline(nd.children)
    if k == "math":
        return render_math(nd)
    if k == "inline_fmt":
        return render_inline_fmt(nd)
    if k == "color":
        return render_color(nd)
    if k == "verbatim_inline":
        return render_verbatim_inline(nd)
    if k == "scopebox":
        return f'<{nd.tag} class="{nd.cls}">{render_inline(nd.children)}</{nd.tag}>'
    if k == "todo_cmd":
        return f'<span class="todo">TODO: \\{esc(nd.name)}</span>'
    if k == "todo_env":
        return f'<span class="todo">TODO: env {esc(nd.name)}</span>' + render_inline(nd.body)
    if k == "footnotetext":
        return f'<span class="footnote">* {render_inline(nd.children)}</span>'
    if k in BLOCK_NODE_KINDS:
        return render_block([nd])  # defensive: a block node leaked into inline context
    return ""  # 'item' outside a list context: nothing sensible to render


def render_math(nd: Node) -> str:
    raw = esc(nd.raw)
    if nd.display:
        return f'<div class="math-display">\\[{raw}\\]</div>'
    return f'<span class="math-inline">\\({raw}\\)</span>'


def render_inline_fmt(nd: Node) -> str:
    inner = render_inline(nd.children)
    if nd.tag == "warn":
        return f'<span class="warn">{inner}</span>'
    return f"<{nd.tag}>{inner}</{nd.tag}>"


def render_color(nd: Node) -> str:
    rgb = BRAND_COLORS.get(nd.color)
    inner = render_inline(nd.children)
    if not rgb:
        return inner
    return f'<span style="color:rgb({rgb})">{inner}</span>'


def render_verbatim_inline(nd: Node) -> str:
    text = esc(nd.text)
    if nd.cmd == "url":
        return f'<a class="tex-url" href="{text}" target="_blank" rel="noopener">{text}</a>'
    return f'<code class="tex-{nd.cmd}">{text}</code>'


def is_block_level(nd: Node) -> bool:
    return nd.kind in BLOCK_NODE_KINDS or (nd.kind == "math" and nd.display)


def render_block(nodes: list[Node]) -> str:
    nodes = apply_scopes(nodes, "div")
    parts: list[str] = []
    buffer: list[Node] = []
    for nd in nodes:
        if nd.kind == "parabreak":
            flush_paragraph(buffer, parts)
            buffer = []
        elif is_block_level(nd):
            flush_paragraph(buffer, parts)
            buffer = []
            parts.append(render_math(nd) if nd.kind == "math" else render_block_node(nd))
        else:
            buffer.append(nd)
    flush_paragraph(buffer, parts)
    return "\n".join(parts)


def flush_paragraph(buffer: list[Node], parts: list[str]) -> None:
    if not buffer:
        return
    pills = try_stage_pills(buffer)
    if pills:
        parts.append(pills)
        return
    rendered = render_inline(buffer)
    if rendered.strip():
        parts.append(f"<p>{rendered}</p>")


def split_nodes_on_bar(nodes: list[Node]) -> list:
    """Split a node list into groups at every "｜" found inside a text
    run. "｜"を含むテキストの箇所でノード列をグループに分割する。"""
    groups = [[]]
    for nd in nodes:
        if nd.kind == "text" and "｜" in nd.value:
            segments = nd.value.split("｜")
            groups[-1].append(Node("text", value=segments[0]))
            for seg in segments[1:]:
                groups.append([Node("text", value=seg)])
        else:
            groups[-1].append(nd)
    return groups


def drop_blank_text(nodes: list[Node]) -> list[Node]:
    """Drop whitespace-only text nodes (source indentation/newlines), so a
    "this list holds exactly one real node" check isn't fooled by them.
    空白のみのテキストノード（ソースのインデント等）を落とす。"""
    return [nd for nd in nodes if not (nd.kind == "text" and not nd.value.strip())]


def try_stage_pills(nodes: list[Node]):
    """Detect the "見るだけ｜一緒に打つ｜帰宅後に再現" 3-stage marker line
    and render it as three pills instead of a plain paragraph. Operates on
    NODES (not pre-rendered HTML) so a wrapping \\small scope around the
    whole line can be unwrapped without breaking tag balance.
    3段階マーカー行を検出しピル表示にする。事前レンダリング済みHTML文字列
    ではなくノード列を扱うため、行全体を覆う \\small スコープを外しても
    タグの対応が崩れない。
    """
    nodes = drop_blank_text(nodes)
    while len(nodes) == 1 and nodes[0].kind in ("group", "scopebox"):
        nodes = drop_blank_text(nodes[0].children)
    groups = split_nodes_on_bar(nodes)
    if len(groups) != 3:
        return None
    texts = [flatten_text(g).strip() for g in groups]
    if not all(t.startswith(lbl) for t, lbl in zip(texts, STAGE_LABELS)):
        return None
    pills = []
    for g in groups:
        html_ = render_inline(g).strip()
        cls = "pill active" if "<strong>" in html_ else "pill"
        # The pill's own background/text color already carries the
        # emphasis, so drop any inline \textcolor styling from inside it
        # (otherwise sfblue-on-sfblue text goes invisible on .pill.active).
        # ピル自体の背景・文字色で強調を表現するため、内側の\textcolorに
        # よるインラインstyleは外す（sfblue文字がsfblue背景で見えなくなる）。
        html_ = re.sub(r'\s*style="color:[^"]*"', "", html_)
        pills.append(f'<span class="{cls}">{html_}</span>')
    return '<div class="stage-pills">' + "".join(pills) + "</div>"


def render_block_node(nd: Node) -> str:
    k = nd.kind
    if k == "block":
        return render_titled_block(nd)
    if k == "list":
        return render_list(nd)
    if k == "table_wrap":
        return render_table_wrap(nd)
    if k == "tabular":
        return render_tabular(nd)
    if k == "columns":
        return render_columns(nd)
    if k == "column":
        return render_block(nd.body)
    if k == "center":
        return f'<div class="align-center">{render_block(nd.body)}</div>'
    if k == "figure":
        return render_figure(nd)
    if k == "listing":
        return render_listing(nd)
    if k == "footnotetext":
        return f'<p class="footnote">* {render_inline(nd.children)}</p>'
    if k == "todo_env":
        return (f'<div class="block todo"><span class="todo">TODO: env '
                f'{esc(nd.name)}</span>{render_block(nd.body)}</div>')
    if k == "scopebox":
        return f'<{nd.tag} class="{nd.cls}">{render_block(nd.children)}</{nd.tag}>'
    return ""


def render_titled_block(nd: Node) -> str:
    cls = BLOCK_TITLE_CSS[nd.variant]
    title_html = render_inline(nd.title) if nd.title else ""
    body_html = render_block(nd.body)
    header = f'<h3 class="block-title">{title_html}</h3>' if title_html.strip() else ""
    return f'<div class="block {cls}">{header}<div class="block-body">{body_html}</div></div>'


def render_list(nd: Node) -> str:
    tag = "ol" if nd.ordered else "ul"
    size_cls = next((c.cls for c in nd.body if c.kind == "scope" and c.cls.startswith("fs-")), "")
    groups = group_by_item(nd.body)
    lis = [render_list_item(marker, content) for marker, content in groups]
    cls_attr = f' class="{size_cls}"' if size_cls else ""
    return f"<{tag}{cls_attr}>" + "".join(lis) + f"</{tag}>"


def group_by_item(body: list[Node]):
    groups = []
    for c in body:
        if c.kind == "item":
            groups.append((c.marker, []))
        elif groups:
            groups[-1][1].append(c)
    return groups


def render_list_item(marker, content: list[Node]) -> str:
    content_html = render_inline(content)
    if marker is not None:
        marker_html = render_inline(marker)
        return f'<li class="li-marker"><span class="marker">{marker_html}</span>{content_html}</li>'
    return f"<li>{content_html}</li>"


def render_table_wrap(nd: Node) -> str:
    classes: list[str] = []
    tabular_node = None
    for c in nd.body:
        if c.kind == "scope":
            classes.append(c.cls)
        elif c.kind == "tabular":
            tabular_node = c
        elif c.kind == "group":
            found = next((gc for gc in c.children if gc.kind == "tabular"), None)
            tabular_node = found or tabular_node
    if tabular_node is None:
        return render_block(nd.body)
    classes = ["table-wrap"] + classes
    return f'<div class="{" ".join(classes)}">{render_tabular(tabular_node)}</div>'


def render_tabular(nd: Node) -> str:
    if not nd.rows:
        return ""
    if nd.has_header_sep:
        thead_part = f"<thead>{render_table_row(nd.rows[0], is_header=True)}</thead>"
        body_rows = nd.rows[1:]
    else:
        thead_part = ""
        body_rows = nd.rows
    body_html = "".join(render_table_row(r, is_header=False) for r in body_rows)
    return f'<div class="table-scroll"><table>{thead_part}<tbody>{body_html}</tbody></table></div>'


def render_table_row(cells: list[Node], is_header: bool) -> str:
    tag = "th" if is_header else "td"
    parts = ["<tr>"]
    for c in cells:
        content_html = render_inline(c.content)
        colspan_attr = f' colspan="{c.colspan}"' if c.colspan > 1 else ""
        parts.append(f'<{tag} style="text-align:{c.align}"{colspan_attr}>{content_html}</{tag}>')
    parts.append("</tr>")
    return "".join(parts)


def extract_width_fraction(width_raw: str):
    m = re.match(r"\s*([0-9]*\.?[0-9]+)\s*\\(?:textwidth|columnwidth|linewidth)", width_raw)
    return m.group(1) if m else None


def render_columns(nd: Node) -> str:
    cols = [c for c in nd.body if c.kind == "column"]
    parts = ['<div class="columns">']
    for c in cols:
        frac = extract_width_fraction(c.width)
        style = f' style="flex:{frac} 1 0"' if frac else ""
        parts.append(f'<div class="column"{style}>{render_block(c.body)}</div>')
    parts.append("</div>")
    return "".join(parts)


def render_figure(nd: Node) -> str:
    if nd.ftype == "img":
        basename = Path(nd.ref).name
        return f'<div class="figure"><img src="assets/img/{esc(basename)}" alt=""></div>'
    return f'<div class="figure"><img src="assets/tikz/{esc(nd.ref)}.svg" alt=""></div>'


def render_listing(nd: Node) -> str:
    code_html = esc(nd.body)
    title_html = f'<div class="code-title">{esc(nd.title)}</div>' if nd.title else ""
    cls = "code-block titled" if nd.title else "code-block"
    return f'<div class="{cls}">{title_html}<pre><code class="lang-{nd.lang}">{code_html}</code></pre></div>'


# ===========================================================================
# Section 5 — chapter/frame extraction and document-level assembly
# 章・フレーム抽出とドキュメント全体の組み立て
# ===========================================================================

@dataclass
class Frame:
    title_nodes: list
    is_later: bool
    body_nodes: list
    title_text: str
    todo_count: int
    web_markers: WebMarkers


def strip_later(nodes: list[Node]) -> bool:
    """Remove any \\later marker from `nodes` in place; return whether one
    was found. \\later標識を除去し、見つかったかどうかを返す。"""
    found = any(nd.kind == "later_marker" for nd in nodes)
    nodes[:] = [nd for nd in nodes if nd.kind != "later_marker"]
    return found


def flatten_text(nodes: list[Node]) -> str:
    """Extract plain text from a node list (markup stripped), for the
    data-title attribute and the outline/report. 見出し等の平文抽出。"""
    out = []
    for nd in nodes:
        k = nd.kind
        if k == "text":
            out.append(nd.value)
        elif k == "text_html":
            out.append(strip_tags(nd.html_value))
        elif k in ("group", "inline_fmt", "color"):
            out.append(flatten_text(nd.children))
        elif k == "verbatim_inline":
            out.append(nd.text)
        elif k == "math":
            out.append(nd.raw)
    return "".join(out)


def parse_chapter_frames(path: Path) -> list[Frame]:
    raw_text = path.read_text(encoding="utf-8")
    store: list[RawBlock] = []
    extracted = extract_verbatim_blocks(raw_text, store)
    chapter_stem = path.stem
    frames = []
    pos = 0
    begin_tag, end_tag = "\\begin{frame}", "\\end{frame}"
    while True:
        # Frame boundaries are located BEFORE comment-stripping (find_
        # uncommented, not str.find) so `% @web:` marker lines further
        # inside the frame body are still there to read (see
        # extract_web_markers) — each frame's own text is cleaned
        # individually right after, which is equivalent to the old
        # whole-file strip since '%' comments never span a newline.
        # コメント除去より前にフレーム境界を探す（str.findではなく
        # find_uncommented）— こうしないとフレーム本文内の`% @web:`行が
        # 消えてしまう（extract_web_markers参照）。各フレームのテキストは
        # 直後に個別クリーニングする — '%'コメントは改行を越えないため、
        # 旧来のファイル全体一括クリーニングと結果は同じになる。
        start = find_uncommented(extracted, begin_tag, pos)
        if start == -1:
            break
        body_start = start + len(begin_tag)
        end = find_uncommented(extracted, end_tag, body_start)
        frame_raw = extracted[body_start:end]
        web_markers = extract_web_markers(frame_raw)
        cleaned_frame = strip_comments_outside_verbatim(frame_raw)
        frames.append(parse_one_frame(cleaned_frame, store, chapter_stem, web_markers))
        pos = end + len(end_tag)
    return frames


def parse_one_frame(frame_text: str, store: list[RawBlock], chapter_stem: str,
                     web_markers: WebMarkers | None = None) -> Frame:
    pos = 0
    _, pos = try_read_bracket_raw(frame_text, pos)  # [fragile]/[plain]/... ignored
    parser = Parser(frame_text, store, chapter_stem)
    title_nodes: list[Node] = []
    if pos < len(frame_text) and frame_text[pos] == "{":
        title_nodes, pos = parser.parse_group(pos + 1)
    is_later = strip_later(title_nodes)
    before = STATS["todo_total"]
    body_nodes, _ = parser.parse_to_eof(pos)
    todo_count = STATS["todo_total"] - before
    STATS["frames"] += 1
    return Frame(title_nodes, is_later, body_nodes, flatten_text(title_nodes), todo_count,
                 web_markers or WebMarkers(widgets=[], videos=[], layout="below"))


def extract_command_raw(text: str, cmdname: str) -> str:
    """Find "\\cmdname{...}" and return its raw (unparsed) argument."""
    m = re.search(r"\\%s\{" % re.escape(cmdname), text)
    if not m:
        return ""
    inner, _ = read_brace_raw(text, m.end() - 1)
    return inner


def render_plain_prose(raw: str) -> str:
    """Render a short plain-LaTeX string (title/author/date/divider text)
    through the normal inline pipeline (de-escaping, dash conversion,
    \\\\ line breaks all handled uniformly)."""
    nodes, _ = Parser(raw, [], "titlepage").parse_to_eof(0)
    return render_inline(nodes)


MASTER_ITEM_RE = re.compile(
    r"\\input\{chapters/(\w+)\}"
    r"|\\scisession\{([^}]*)\}\{([^}]*)\}\{([^}]*)\}"
)


def parse_master_sequence(sci_tex_path: Path):
    """Walk sci_tutorial.tex in document order, returning the title-page
    fields plus an ordered list of ('input', chapter) / ('session', num,
    title, time) items — i.e. exactly the \\input/\\scisession interleaving
    the real deck follows. sci_tutorial.tex を文書順に読み、表紙情報と
    \\input/\\scisession の出現順リストを返す。"""
    text = sci_tex_path.read_text(encoding="utf-8")
    text = strip_comments_outside_verbatim(text)
    fields = {name: extract_command_raw(text, name) for name in
               ("title", "subtitle", "author", "date")}
    doc_start = text.find("\\begin{document}")
    body = text[doc_start:] if doc_start != -1 else text
    items = []
    for m in MASTER_ITEM_RE.finditer(body):
        if m.group(1):
            items.append(("input", m.group(1)))
        else:
            items.append(("session", m.group(2), m.group(3), m.group(4)))
    return fields, items


@dataclass
class Slide:
    kind: str
    title_html: str = ""
    title_text: str = ""
    body_html: str = ""
    subtitle_html: str = ""
    author_html: str = ""
    date_html: str = ""
    is_later: bool = False
    session_label: str = ""
    divider_num: str = ""
    divider_time: str = ""
    todo_count: int = 0
    has_web_extra: bool = False


def make_title_slide(fields: dict) -> Slide:
    title_html = render_plain_prose(fields["title"])
    return Slide(
        kind="title",
        title_html=title_html,
        title_text=strip_tags(title_html),
        subtitle_html=render_plain_prose(fields["subtitle"]),
        author_html=render_plain_prose(fields["author"]),
        date_html=render_plain_prose(fields["date"]),
        session_label="タイトル",
    )


def render_web_widget(w: WebWidgetSpec) -> str:
    """One widget mount point: an empty div deck.js fills via
    window.SfWidgets.<camelCase>(el, {...from data-* attrs...}) on slide
    show, and empties again (dispose()) on slide hide (see deck.js "web-only
    widgets" section). 1個のウィジェット差し込み先。空のdivをdeck.jsが
    表示時にwindow.SfWidgets経由でマウントし、非表示時にdispose()で
    空に戻す（deck.jsの「web-only widgets」節参照）。"""
    attrs_html = "".join(f' data-{esc(k)}="{esc(v)}"' for k, v in w.attrs.items())
    style = f' style="height:{w.height}px"' if w.height else ""
    return f'<div class="web-only widget" data-widget="{esc(w.widget)}"{attrs_html}{style}></div>'


def render_web_video(v: WebVideoSpec) -> str:
    basename = Path(v.src).name
    caption = f"<figcaption>{esc(v.caption)}</figcaption>" if v.caption else ""
    return (f'<figure class="web-only video"><video controls muted playsinline '
            f'preload="metadata" src="assets/video/{esc(basename)}"></video>{caption}</figure>')


def render_web_extra(markers: WebMarkers) -> str:
    parts = [render_web_widget(w) for w in markers.widgets]
    if len(markers.videos) > 1:
        # Multiple `% @web: video=...` lines on one frame -> side by side.
        # 同一フレームに複数の video= 行 -> 横並び。
        parts.append('<div class="web-videos">' +
                      "".join(render_web_video(v) for v in markers.videos) + "</div>")
    else:
        parts.extend(render_web_video(v) for v in markers.videos)
    return "".join(parts)


def apply_web_markers(body_html: str, markers: WebMarkers):
    """Splice a frame's `% @web:` widgets/videos into its rendered HTML per
    the marker's `layout` (see Section 2b): 'below' appends after the
    LaTeX content (default), 'right' puts the LaTeX content and the extra
    content in a two-column row (reusing the existing .columns/.column
    layout primitives from slide.css), 'replace' drops the LaTeX content
    entirely (title/footer stay). Returns (new_body_html, has_web_extra) —
    the latter drives the "Web 版のみ" header badge.
    フレームの`% @web:`ウィジェット/動画を、marker指定のlayoutに従って
    レンダリング済みHTMLへ合成する: 'below'はLaTeX本文の後ろに追加（既定）、
    'right'はLaTeX本文と追加分を2カラムに並べる（slide.cssの既存
    .columns/.columnをそのまま流用）、'replace'はLaTeX本文を丸ごと
    差し替える（タイトル・フッターはそのまま）。戻り値の
    has_web_extraは「Web 版のみ」バッジの表示に使う。"""
    if markers.is_empty():
        return body_html, False
    extra_html = render_web_extra(markers)
    if markers.layout == "replace":
        return extra_html, True
    if markers.layout == "right":
        return (f'<div class="columns web-columns">'
                f'<div class="column">{body_html}</div>'
                f'<div class="column">{extra_html}</div></div>'), True
    return body_html + extra_html, True


def make_content_slide(frame: Frame, session_label: str) -> Slide:
    body_html, has_web_extra = apply_web_markers(render_block(frame.body_nodes), frame.web_markers)
    return Slide(
        kind="content",
        title_html=render_inline(frame.title_nodes),
        title_text=frame.title_text,
        body_html=body_html,
        is_later=frame.is_later,
        session_label=session_label,
        todo_count=frame.todo_count,
        has_web_extra=has_web_extra,
    )


def make_divider_slide(num: str, title_raw: str, time_raw: str) -> Slide:
    title_html = render_plain_prose(title_raw)
    label = f"S{num}" if num.isdigit() else title_raw
    return Slide(
        kind="divider",
        title_html=title_html,
        title_text=strip_tags(title_html),
        divider_num=num,
        divider_time=strip_tags(render_plain_prose(time_raw)),
        session_label=label,
    )


def build_slides(sci_dir: Path) -> list[Slide]:
    slides_dir = sci_dir / "slides"
    fields, items = parse_master_sequence(slides_dir / "sci_tutorial.tex")
    slides = [make_title_slide(fields)]
    session_label = "オープニング"
    for item in items:
        if item[0] == "input":
            chapter_path = slides_dir / "chapters" / (item[1] + ".tex")
            for frame in parse_chapter_frames(chapter_path):
                slides.append(make_content_slide(frame, session_label))
        else:
            _, num, stitle, stime = item
            session_label = f"S{num}" if num.isdigit() else stitle
            slides.append(make_divider_slide(num, stitle, stime))
    return slides


# ===========================================================================
# Section 6 — HTML document assembly
# HTML文書の組み立て
# ===========================================================================

DECK_SHORT_TITLE = "StampFly Ecosystem"


def render_slide_section(slide: Slide, index: int, total: int) -> str:
    classes = ["slide", slide.kind]
    if slide.is_later:
        classes.append("later")
    attrs = (f'id="s-{index:03d}" data-title="{esc(slide.title_text)}" '
             f'data-session="{esc(slide.session_label)}"')
    if slide.kind == "title":
        inner = render_title_slide_body(slide)
    elif slide.kind == "divider":
        inner = render_divider_slide_body(slide)
    else:
        inner = render_content_slide_body(slide, index, total)
    return f'<section class="{" ".join(classes)}" {attrs}>{inner}</section>'


def render_content_slide_body(slide: Slide, index: int, total: int) -> str:
    later_badge = '<span class="later-badge">あとで読む</span>' if slide.is_later else ""
    web_badge = '<span class="web-badge">Web 版のみ</span>' if slide.has_web_extra else ""
    header = f'<header class="slide-header"><h2>{slide.title_html}</h2>{later_badge}{web_badge}</header>'
    body = f'<div class="slide-body">{slide.body_html}</div>'
    footer = (f'<footer class="slide-footer"><span class="footer-title">'
              f'{esc(DECK_SHORT_TITLE)}</span><span class="footer-page">'
              f'{index} / {total}</span></footer>')
    return header + body + footer


def render_title_slide_body(slide: Slide) -> str:
    return (f'<div class="title-content"><h1>{slide.title_html}</h1>'
            f'<p class="subtitle">{slide.subtitle_html}</p>'
            f'<p class="author">{slide.author_html}</p>'
            f'<p class="date">{slide.date_html}</p></div>')


def render_divider_slide_body(slide: Slide) -> str:
    label = f"Session {slide.divider_num}" if slide.divider_num.isdigit() else esc(slide.divider_num)
    return (f'<div class="divider-box"><div class="divider-num">{label}</div>'
            f'<div class="divider-title">{slide.title_html}</div>'
            f'<div class="divider-time">{slide.divider_time}</div></div>'
            f'<div class="divider-foot">SCI/SICE チュートリアル講座 2026</div>')


def render_outline(slides: list[Slide]) -> str:
    groups: list[tuple[str, list]] = []
    current_label = None
    for i, s in enumerate(slides, start=1):
        label = "タイトル" if s.kind == "title" else s.session_label
        if label != current_label:
            groups.append((label, []))
            current_label = label
        groups[-1][1].append((i, s))
    parts = ['<nav id="outline" class="outline" hidden><h3>目次</h3>']
    for label, entries in groups:
        parts.append(render_outline_group(label, entries))
    parts.append("</nav>")
    return "".join(parts)


def render_outline_group(label: str, entries: list) -> str:
    parts = [f'<div class="outline-group"><h4>{esc(label)}</h4><ul>']
    for i, s in entries:
        later_note = "（あとで読む）" if s.is_later else ""
        title_disp = s.title_text or ("表紙" if s.kind == "title" else "区切り")
        parts.append(f'<li><a href="?s={i}" data-slide="{i}">{i}. '
                      f'{esc(title_disp)}{esc(later_note)}</a></li>')
    parts.append("</ul></div>")
    return "".join(parts)


HELP_ROWS = [
    ("→ / Space / PageDown / n", "次のスライド"),
    ("← / PageUp / p", "前のスライド"),
    ("Home / End", "最初 / 最後のスライド"),
    ("f", "全画面表示"),
    ("t", "目次パネルの開閉"),
    ("?", "このヘルプの開閉"),
    ("クリック（右1/3・左1/3）", "次へ・前へ"),
    ("スワイプ", "次へ・前へ（左右）"),
]


def render_help_overlay() -> str:
    rows = "".join(f"<tr><td>{esc(k)}</td><td>{esc(v)}</td></tr>" for k, v in HELP_ROWS)
    return (f'<div id="help" class="overlay" hidden><div class="overlay-box">'
            f'<h3>キー操作</h3><table>{rows}</table>'
            f'<button id="help-close">閉じる</button></div></div>')


PAGE_TEMPLATE = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Noto+Sans+JP:wght@400;500;700;900&display=swap">
<link rel="stylesheet" href="assets/vendor/katex/katex.min.css">
<link rel="stylesheet" href="assets/css/deck.css">
<link rel="stylesheet" href="assets/css/slide.css">
<link rel="stylesheet" href="assets/widgets/widgets.css">
</head>
<body>
<div id="chrome">
  <div id="progress"><div id="progress-bar"></div></div>
  <div id="controls">
    <button id="btn-outline" title="目次 (t)">目次</button>
    <button id="btn-fullscreen" title="全画面 (f)">全画面</button>
    <button id="btn-help" title="ヘルプ (?)">?</button>
  </div>
</div>
<div id="viewport">
  <div id="stage">
{slides}
  </div>
</div>
{outline}
{help}
<script src="assets/vendor/katex/katex.min.js"></script>
<script src="assets/vendor/katex/auto-render.min.js"></script>
<script src="assets/widgets/pid_step.js"></script>
<script src="assets/widgets/mixer.js"></script>
<script src="assets/widgets/comp_filter.js"></script>
<script src="assets/js/deck.js"></script>
</body>
</html>
"""


def render_page(slides: list[Slide]) -> str:
    total = len(slides)
    sections = "\n".join(render_slide_section(s, i, total) for i, s in enumerate(slides, start=1))
    return PAGE_TEMPLATE.format(
        title="制御教育教材 StampFly Ecosystem の紹介 — SCI/SICEチュートリアル2026",
        slides=sections,
        outline=render_outline(slides),
        help=render_help_overlay(),
    )


# ===========================================================================
# Section 7 — TikZ -> SVG build pipeline
# TikZ -> SVG ビルドパイプライン
# ===========================================================================

# Brand colors + the tikz library set from stampfly_slides.sty, used only
# for the two kinds of SYNTHETIC standalone figures this script itself
# writes (scisessionmap_N, and any inline \begin{tikzpicture} found in a
# frame body) — the real per-figure .tex files under _shared/tikz/ are
# already self-contained standalone documents and are compiled unmodified.
# 合成TikZ図（scisessionmap_N・フレーム内インラインTikZ）専用のプリアンブル。
# _shared/tikz/ 配下の実ファイルは既にstandaloneなので無改変でコンパイルする。
SYNTHETIC_JP_PREAMBLE = r"""\documentclass[border=10pt]{standalone}
\usepackage{tikz}
\usetikzlibrary{arrows.meta, positioning, shapes.geometric, fit,
                 decorations.pathmorphing, decorations.pathreplacing,
                 automata, calc, patterns}
\usepackage{luatexja}
\usepackage[match]{luatexja-fontspec}
\usepackage{ifthen}
\newboolean{jfontset}\setboolean{jfontset}{false}
\IfFontExistsTF{Hiragino Kaku Gothic ProN}{%
  \setmainjfont{Hiragino Kaku Gothic ProN}%
  \setsansjfont{Hiragino Kaku Gothic ProN}%
  \setboolean{jfontset}{true}}{}
\ifthenelse{\boolean{jfontset}}{}{%
  \IfFontExistsTF{Yu Gothic}{%
    \setmainjfont{Yu Gothic}%
    \setsansjfont{Yu Gothic}%
    \setboolean{jfontset}{true}}{}}
\ifthenelse{\boolean{jfontset}}{}{%
  \IfFontExistsTF{Noto Sans CJK JP}{%
    \setmainjfont{Noto Sans CJK JP}%
    \setsansjfont{Noto Sans CJK JP}}{}}
\begin{document}
"""


def synthetic_color_defs() -> str:
    lines = [r"\definecolor{%s}{RGB}{%s}" % (name, rgb)
             for name, rgb in BRAND_COLORS.items() if name not in ("black", "white")]
    return "\n".join(lines) + "\n"


def extract_scisessionmap_macro(sci_intro_path: Path) -> str:
    """Pull the full \\newcommand{\\scisessionmap}[1]{...} definition out of
    sci_intro.tex verbatim, so the synthetic wrapper always matches
    whatever the (concurrently edited) source currently defines.
    sci_intro.tex から \\scisessionmap 定義をそのまま抜き出す。並行編集
    されうる本体と常に一致させるため、手で書き写さずソースから取得する。
    """
    text = sci_intro_path.read_text(encoding="utf-8")
    marker = r"\newcommand{\scisessionmap}"
    start = text.index(marker)
    brace1 = text.index("{", start + len(marker) + 3)  # skip past "[1]"
    close = find_balanced_close(text, brace1)
    return text[start:close + 1]


def write_scisessionmap_wrapper(scratch_tex_dir: Path, macro_src: str, n: int) -> Path:
    path = scratch_tex_dir / f"scisessionmap_{n}.tex"
    body = SYNTHETIC_JP_PREAMBLE + synthetic_color_defs() + macro_src + f"\n\\scisessionmap{{{n}}}\n\\end{{document}}\n"
    path.write_text(body, encoding="utf-8")
    return path


def write_inline_tikz_wrapper(scratch_tex_dir: Path, name: str, tikz_src: str) -> Path:
    path = scratch_tex_dir / f"{name}.tex"
    body = SYNTHETIC_JP_PREAMBLE + synthetic_color_defs() + tikz_src + "\n\\end{document}\n"
    path.write_text(body, encoding="utf-8")
    return path


def compile_tex_to_pdf(tex_path: Path, cwd: Path, out_dir: Path) -> Path:
    """Run lualatex with cwd=`cwd` (so a real _shared/tikz source resolves
    its own relative paths, of which it has none) and an absolute
    -output-directory, matching the exact command form from the task brief.
    タスクブリーフの実行形そのまま: cwd を図の置き場に、出力先は絶対パスで。
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = ["lualatex", "-interaction=nonstopmode",
           f"-output-directory={out_dir.resolve()}", tex_path.name]
    result = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    pdf_path = out_dir / (tex_path.stem + ".pdf")
    if not pdf_path.exists():
        tail = "\n".join(result.stdout.splitlines()[-40:])
        raise RuntimeError(f"lualatex failed for {tex_path.name}:\n{tail}")
    return pdf_path


def pdf_to_svg(pdf_path: Path, svg_path: Path) -> None:
    svg_path.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["pdftocairo", "-svg", str(pdf_path), str(svg_path)], check=True)


def build_all_tikz_svgs(tikz_names: set, tikz_dir: Path, sci_intro_path: Path,
                          scratch_dir: Path, out_svg_dir: Path) -> list[str]:
    """Compile every referenced TikZ figure to SVG. Returns a list of
    human-readable status lines for the conversion report.
    参照されている全TikZ図をSVGへコンパイルし、報告用の行を返す。"""
    scratch_tex = scratch_dir / "tikz"
    scratch_tex.mkdir(parents=True, exist_ok=True)
    macro_src = extract_scisessionmap_macro(sci_intro_path)
    report = []
    for name in sorted(tikz_names):
        try:
            tex_path, cwd = resolve_tikz_source(name, tikz_dir, scratch_tex, macro_src)
            pdf = compile_tex_to_pdf(tex_path, cwd, scratch_tex)
            pdf_to_svg(pdf, out_svg_dir / f"{name}.svg")
            report.append(f"OK   {name}")
        except Exception as e:  # noqa: BLE001 - report and keep going
            report.append(f"FAIL {name}: {e}")
    return report


def resolve_tikz_source(name: str, tikz_dir: Path, scratch_tex: Path, macro_src: str):
    """Return (tex_path, cwd_for_lualatex) for one figure name."""
    if name.startswith("scisessionmap_"):
        n = int(name.rsplit("_", 1)[1])
        return write_scisessionmap_wrapper(scratch_tex, macro_src, n), scratch_tex
    if name in INLINE_TIKZ_SOURCES:
        return write_inline_tikz_wrapper(scratch_tex, name, INLINE_TIKZ_SOURCES[name]), scratch_tex
    return tikz_dir / f"{name}.tex", tikz_dir


# ===========================================================================
# Section 8 — image asset collection
# 画像アセットの収集
# ===========================================================================

def collect_image_refs(slides: list[Slide]) -> set:
    refs = set()
    for s in slides:
        refs.update(re.findall(r'assets/img/([^"]+)', s.body_html))
    return refs


def copy_images(basenames: set, images_dir: Path, fallback_dir: Path, out_img_dir: Path) -> list[str]:
    out_img_dir.mkdir(parents=True, exist_ok=True)
    report = []
    for name in sorted(basenames):
        src = find_image_source(name, images_dir, fallback_dir)
        if src is None:
            report.append(f"MISSING {name}")
            continue
        shutil.copy2(src, out_img_dir / name)
        report.append(f"OK      {name}")
    return report


def find_image_source(name: str, images_dir: Path, fallback_dir: Path):
    for candidate_dir in (images_dir, fallback_dir):
        candidate = candidate_dir / name
        if candidate.exists():
            return candidate
    return None


# ---------------------------------------------------------------------------
# Video assets (from `% @web: video=...` markers, Section 2b)
# 動画アセット（`% @web: video=...` マーカー由来、Section 2b参照）
#
# The marker's path is written relative to slides/chapters/ (matching this
# deck's existing \includegraphics{../fallback/...} convention), so its
# source always resolves under <sci-dir>/fallback/ regardless of the exact
# relative prefix in the marker — only the basename is trusted, same as
# copy_images() above.
# マーカーのパスは slides/chapters/ からの相対（既存の
# \includegraphics{../fallback/...} 規約と同じ）で書くため、マーカー中の
# 相対プレフィックスに関わらず実体は常に <sci-dir>/fallback/ 配下にある
# — copy_images()と同じくbasenameのみを信頼する。
# ---------------------------------------------------------------------------

def collect_video_refs(slides: list[Slide]) -> set:
    refs = set()
    for s in slides:
        refs.update(re.findall(r'assets/video/([^"]+)', s.body_html))
    return refs


def copy_videos(basenames: set, fallback_dir: Path, out_video_dir: Path) -> list[str]:
    out_video_dir.mkdir(parents=True, exist_ok=True)
    report = []
    for name in sorted(basenames):
        src = fallback_dir / name
        if not src.exists():
            report.append(f"MISSING {name}")
            continue
        shutil.copy2(src, out_video_dir / name)
        report.append(f"OK      {name}")
    return report


# ===========================================================================
# Section 9 — CLI entry point and the conversion report
# CLIエントリポイントと変換レポート
# ===========================================================================

def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sci-dir", default="sci_tutorial_2026", type=Path,
                    help="Event root (contains slides/, fallback/, web/).")
    p.add_argument("--tikz-dir", default="_shared/tikz", type=Path,
                    help="Shared TikZ sources directory.")
    p.add_argument("--images-dir", default="_shared/images", type=Path,
                    help="Shared images directory.")
    p.add_argument("--out", default=None, type=Path,
                    help="Output index.html path (default: <sci-dir>/web/index.html).")
    p.add_argument("--build-tikz", action="store_true",
                    help="Compile TikZ figures to SVG (requires lualatex + pdftocairo).")
    p.add_argument("--scratch-dir", default=None, type=Path,
                    help="Scratch directory for TikZ build artifacts (default: a temp dir).")
    return p


def print_report(slides: list[Slide], tikz_report: list[str], img_report: list[str],
                  video_report: list[str] | None = None) -> None:
    total_todo = sum(s.todo_count for s in slides)
    web_slides = [s for s in slides if s.has_web_extra]
    print(f"[beamer_to_html] frames converted: {STATS['frames']} "
          f"(+{sum(1 for s in slides if s.kind == 'divider')} dividers, "
          f"+1 title) = {len(slides)} slides")
    print(f"[beamer_to_html] TODO markers total: {total_todo}")
    for i, s in enumerate(slides, start=1):
        if s.todo_count:
            print(f"  slide {i:3d} [{s.title_text}]: {s.todo_count} TODO")
    if web_slides:
        print(f"[beamer_to_html] web-only content (% @web:): {len(web_slides)} slides")
        for i, s in enumerate(slides, start=1):
            if s.has_web_extra:
                print(f"  slide {i:3d} [{s.title_text}]")
    if tikz_report:
        print(f"[beamer_to_html] TikZ SVG build: {len(tikz_report)} figures")
        for line in tikz_report:
            print(f"  {line}")
    if img_report:
        print(f"[beamer_to_html] image assets: {len(img_report)}")
        for line in img_report:
            print(f"  {line}")
    if video_report:
        print(f"[beamer_to_html] video assets: {len(video_report)}")
        for line in video_report:
            print(f"  {line}")


def main() -> int:
    args = build_argparser().parse_args()
    sci_dir: Path = args.sci_dir
    web_dir = sci_dir / "web"
    out_path = args.out or (web_dir / "index.html")

    slides = build_slides(sci_dir)
    tikz_report: list[str] = []
    if args.build_tikz:
        tikz_names = set(re.findall(r'assets/tikz/([^."]+)\.svg',
                                     "\n".join(s.body_html for s in slides)))
        scratch = args.scratch_dir or Path(tempfile.mkdtemp(prefix="sci_web_tikz_"))
        sci_intro = sci_dir / "slides" / "chapters" / "sci_intro.tex"
        tikz_report = build_all_tikz_svgs(tikz_names, args.tikz_dir, sci_intro,
                                           scratch, web_dir / "assets" / "tikz")

    img_refs = collect_image_refs(slides)
    img_report = copy_images(img_refs, args.images_dir, sci_dir / "fallback",
                              web_dir / "assets" / "img")

    video_refs = collect_video_refs(slides)
    video_report = copy_videos(video_refs, sci_dir / "fallback", web_dir / "assets" / "video")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(render_page(slides), encoding="utf-8")
    print(f"[beamer_to_html] wrote {out_path}")
    print_report(slides, tikz_report, img_report, video_report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
