#!/usr/bin/env python3
"""Convert a Beamer PDF deck into a browser-based HTML slide viewer.
Beamer の PDF デッキをブラウザで見られる HTML スライドビューアに変換する。

One <section class="slide"> is emitted per page (mirrors the reference UX
this tool was modeled on). Pages are rendered to SVG with `pdftocairo`
(vector, keeps text as glyph outlines); pages whose SVG comes out huge
(typically because a full-resolution photo is embedded) fall back to
`dvisvgm --pdf` and finally to a flattened PNG raster.
ページごとに1つの <section class="slide"> を出力する（本ツールが手本とした
リファレンスUXと同じ構造）。各ページは `pdftocairo` でSVG化する（ベクタ、
文字はグリフ輪郭として保持）。SVGが肥大化したページ（多くは高解像度の写真が
埋め込まれているため）は `dvisvgm --pdf`、それも失敗すればラスタPNGへ
順に代替する。

Usage / 使い方:
    python3 pdf_deck_html.py <deck.pdf> <out_dir> [--title T] [--inline] [--pages A-B]

Requires (stdlib only for this script; these are external CLI tools, not
Python packages): pdftocairo, pdftoppm, pdfinfo, pdftotext, dvisvgm.
外部コマンド（本スクリプト自体は標準ライブラリのみ使用）: 上記5点。
"""

import argparse
import base64
import html
import re
import shutil
import subprocess
import sys
from pathlib import Path

# --- Tunable constants (kept out of the function bodies) -------------------
# 調整可能な定数（関数本体にマジックナンバーを書かないためここへ集約する）
SVG_SIZE_WARN_BYTES = 400 * 1024   # 400 KB/page threshold from the spec / 仕様書の1ページ閾値
PNG_FALLBACK_DPI = 192              # 2x of the 96 DPI CSS reference / CSS基準96DPIの2倍
PDFTOCAIRO_TIMEOUT_SEC = 30
DVISVGM_TIMEOUT_SEC = 30
PDFTOTEXT_TIMEOUT_SEC = 15
PDFINFO_TIMEOUT_SEC = 15
DEFAULT_TITLE = "Slide Deck"

REPO_ROOT = Path(__file__).resolve().parents[2]


# =============================================================================
# PDF introspection / PDF の情報取得
# =============================================================================

def get_page_count(pdf_path):
    """Read the total page count via `pdfinfo`.
    `pdfinfo` からページ総数を読み取る。"""
    result = subprocess.run(
        ["pdfinfo", str(pdf_path)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        timeout=PDFINFO_TIMEOUT_SEC,
    )
    text = result.stdout.decode("utf-8", errors="replace")
    match = re.search(r"^Pages:\s*(\d+)", text, re.MULTILINE)
    if not match:
        raise RuntimeError("pdfinfo did not report a page count for {}".format(pdf_path))
    return int(match.group(1))


def get_deck_title_hint(pdf_path):
    """Read the PDF's Title metadata field, if present.
    PDFのTitleメタデータがあれば読み取る（--title 省略時の既定値に使う）。"""
    result = subprocess.run(
        ["pdfinfo", str(pdf_path)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        timeout=PDFINFO_TIMEOUT_SEC,
    )
    text = result.stdout.decode("utf-8", errors="replace")
    match = re.search(r"^Title:\s*(.+)$", text, re.MULTILINE)
    return match.group(1).strip() if match else None


def extract_page_title(pdf_path, page_num):
    """First non-empty text line on the page, used as the outline label.
    ページ内の最初の空でないテキスト行を、目次パネルの見出しに使う。"""
    cmd = ["pdftotext", "-f", str(page_num), "-l", str(page_num),
           "-layout", str(pdf_path), "-"]
    try:
        result = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=PDFTOTEXT_TIMEOUT_SEC,
        )
    except (subprocess.TimeoutExpired, OSError):
        return "Slide {}".format(page_num)
    text = result.stdout.decode("utf-8", errors="replace")
    for line in text.splitlines():
        stripped = line.strip()
        if stripped:
            return stripped
    return "Slide {}".format(page_num)


def parse_page_range(range_spec, total_pages):
    """Parse "--pages A-B" into an inclusive 1-based (start, end).
    "--pages A-B" を1始まりの (start, end) 区間へ変換する。"""
    if not range_spec:
        return 1, total_pages
    match = re.match(r"^(\d+)-(\d+)$", range_spec)
    if not match:
        raise ValueError("--pages must look like A-B, e.g. 1-16 (got: {})".format(range_spec))
    start, end = int(match.group(1)), int(match.group(2))
    if start < 1 or end > total_pages or start > end:
        raise ValueError(
            "--pages {} is out of range for a {}-page deck".format(range_spec, total_pages)
        )
    return start, end


# =============================================================================
# Per-page conversion (pdftocairo -> dvisvgm -> pdftoppm PNG fallback chain)
# 1ページ変換（pdftocairo → dvisvgm → PNG の代替チェーン）
# =============================================================================

def run_quiet(cmd, timeout):
    """Run a subprocess, returning True only on a clean exit.
    サブプロセスを実行し、正常終了(exit 0)のときだけ True を返す。"""
    try:
        result = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, OSError):
        return False


def convert_with_cairo(pdf_path, page_num, out_svg):
    """Render one page to SVG with pdftocairo (vector, glyph-outline text).
    pdftocairo で1ページをSVG化する（ベクタ、テキストはグリフ輪郭のまま）。"""
    cmd = ["pdftocairo", "-svg", "-f", str(page_num), "-l", str(page_num),
           str(pdf_path), str(out_svg)]
    ok = run_quiet(cmd, PDFTOCAIRO_TIMEOUT_SEC)
    return ok and out_svg.exists() and out_svg.stat().st_size > 0


def convert_with_dvisvgm(pdf_path, page_num, out_svg):
    """Fallback #1 for oversized pages: dvisvgm's PDF backend, WOFF2 fonts.
    肥大化ページの代替その1: dvisvgmのPDFバックエンド（WOFF2フォント埋込）。"""
    cmd = ["dvisvgm", "--pdf", "--page={}".format(page_num),
           "--font-format=woff2", "-o", str(out_svg), str(pdf_path)]
    ok = run_quiet(cmd, DVISVGM_TIMEOUT_SEC)
    return ok and out_svg.exists() and out_svg.stat().st_size > 0


def convert_with_pdftoppm_png(pdf_path, page_num, out_png):
    """Fallback #2: flatten the page to a raster PNG at PNG_FALLBACK_DPI.
    代替その2: PNG_FALLBACK_DPI でページをラスタPNGに落とす。"""
    prefix = out_png.with_suffix("")  # pdftoppm appends ".png" itself / 拡張子は自動付与
    cmd = ["pdftoppm", "-png", "-r", str(PNG_FALLBACK_DPI),
           "-f", str(page_num), "-l", str(page_num), "-singlefile",
           str(pdf_path), str(prefix)]
    ok = run_quiet(cmd, PDFTOCAIRO_TIMEOUT_SEC)
    return ok and out_png.exists() and out_png.stat().st_size > 0


def render_page(pdf_path, page_num, pages_dir):
    """Render one page, escalating svg(cairo) -> svg(dvisvgm) -> png(raster)
    only when the cairo SVG exceeds SVG_SIZE_WARN_BYTES.
    1ページを変換する。cairoのSVGが閾値超のときだけ dvisvgm→PNG の順で
    代替を試みる。"""
    stem = "p{:03d}".format(page_num)
    cairo_svg = pages_dir / (stem + ".svg")
    if not convert_with_cairo(pdf_path, page_num, cairo_svg):
        raise RuntimeError("pdftocairo failed on page {}".format(page_num))

    chosen, kind, source = cairo_svg, "svg", "cairo"
    if cairo_svg.stat().st_size > SVG_SIZE_WARN_BYTES:
        dvi_svg = pages_dir / (stem + "_dvi.svg")
        png_path = pages_dir / (stem + ".png")
        if convert_with_dvisvgm(pdf_path, page_num, dvi_svg):
            cairo_svg.unlink()
            chosen, kind, source = dvi_svg, "svg", "dvisvgm"
        elif convert_with_pdftoppm_png(pdf_path, page_num, png_path):
            cairo_svg.unlink()
            chosen, kind, source = png_path, "png", "png-fallback"
        # else: both fallbacks failed, keep the oversized cairo SVG as last resort
        # 両方失敗した場合は肥大化した cairo SVG を最後の手段として残す

    return {
        "num": page_num,
        "title": extract_page_title(pdf_path, page_num),
        "path": chosen,
        "kind": kind,
        "bytes": chosen.stat().st_size,
        "source": source,
    }


def data_uri_for_file(path, mime):
    """Base64 data URI for embedding a page asset directly in the HTML.
    ページ素材をHTMLへ直接埋め込むためのBase64データURIを作る。"""
    payload = base64.b64encode(path.read_bytes()).decode("ascii")
    return "data:{};base64,{}".format(mime, payload)


def build_pages(pdf_path, out_dir, start, end, inline):
    """Convert pages [start, end] and collect per-page records.
    [start, end] の各ページを変換し、ページ情報を収集する。
    In --inline mode the pages/ directory is a scratch step only: bytes are
    read into data URIs and the directory is removed afterward so out_dir
    ends up holding just index.html.
    --inline のとき pages/ は一時作業用に過ぎない。バイト列をデータURI化した
    後にディレクトリごと削除し、out_dir には index.html だけを残す。"""
    pages_dir = out_dir / "pages"
    pages_dir.mkdir(parents=True, exist_ok=True)
    pages = []
    for page_num in range(start, end + 1):
        record = render_page(pdf_path, page_num, pages_dir)
        if inline:
            mime = "image/svg+xml" if record["kind"] == "svg" else "image/png"
            record["data_uri"] = data_uri_for_file(record["path"], mime)
        else:
            record["file"] = "pages/" + record["path"].name
        pages.append(record)
    if inline:
        shutil.rmtree(pages_dir)
    return pages


# =============================================================================
# HTML / CSS / JS templates — plain string substitution, no braces to escape
# HTML/CSS/JS テンプレート（プレーンな文字列置換。波括弧のエスケープが不要）
# =============================================================================

CSS_TEMPLATE = """
:root {
  color-scheme: light dark;
  --chrome-bg: #f4f4f6;
  --chrome-fg: #1b1b1f;
  --chrome-border: #d4d4d9;
  --accent: #3661c9;
  --accent-fg: #ffffff;
  --stage-bg: #e7e7ec;
  --overlay-bg: rgba(20, 20, 24, 0.55);
  --panel-bg: #ffffff;
  --panel-fg: #1b1b1f;
}
@media (prefers-color-scheme: dark) {
  :root {
    --chrome-bg: #1c1c20;
    --chrome-fg: #e9e9ee;
    --chrome-border: #3a3a40;
    --accent: #7aa2ff;
    --accent-fg: #101014;
    --stage-bg: #0c0c0e;
    --overlay-bg: rgba(0, 0, 0, 0.65);
    --panel-bg: #202024;
    --panel-fg: #e9e9ee;
  }
}
* { box-sizing: border-box; }
html, body {
  margin: 0; padding: 0; height: 100%; overflow: hidden;
  background: var(--chrome-bg); color: var(--chrome-fg);
  font-family: -apple-system, "Hiragino Sans", "Yu Gothic", sans-serif;
}
#viewer {
  position: fixed; top: 0; left: 0; right: 0; bottom: 52px;
  display: flex; align-items: center; justify-content: center;
  background: var(--stage-bg);
}
#stage { position: relative; width: 100%; height: 100%; }
.slide {
  position: absolute; inset: 0; display: none;
  align-items: center; justify-content: center;
}
.slide.active { display: flex; }
.slide img {
  max-width: 96%; max-height: 96%; width: auto; height: auto;
  object-fit: contain;
  background: #ffffff; /* slides are always white / スライドは常に白背景 */
  box-shadow: 0 1px 10px rgba(0, 0, 0, 0.25);
}
.click-zone { position: absolute; top: 0; bottom: 0; width: 33.34%; cursor: pointer; z-index: 5; }
#click-left { left: 0; }
#click-right { right: 0; }
#bottombar {
  position: fixed; left: 0; right: 0; bottom: 0; height: 52px;
  display: flex; align-items: center; gap: 8px; padding: 0 10px;
  background: var(--chrome-bg); border-top: 1px solid var(--chrome-border);
  z-index: 20;
}
#bottombar button {
  background: transparent; color: var(--chrome-fg);
  border: 1px solid var(--chrome-border); border-radius: 6px;
  padding: 6px 10px; font-size: 13px; cursor: pointer;
}
#bottombar button:hover { border-color: var(--accent); color: var(--accent); }
#counter { font-variant-numeric: tabular-nums; font-size: 13px; min-width: 72px; text-align: center; }
#progress-track { flex: 1; height: 6px; background: var(--chrome-border); border-radius: 3px; overflow: hidden; }
#progress-fill { height: 100%; width: 0%; background: var(--accent); }
aside#outline {
  position: fixed; top: 0; right: 0; bottom: 52px; width: min(340px, 82vw);
  background: var(--panel-bg); color: var(--panel-fg);
  border-left: 1px solid var(--chrome-border);
  z-index: 30; display: flex; flex-direction: column;
  box-shadow: -4px 0 16px rgba(0, 0, 0, 0.2);
}
#outline-head { display: flex; align-items: center; justify-content: space-between; padding: 10px 12px; border-bottom: 1px solid var(--chrome-border); }
#outline-head h2 { font-size: 14px; margin: 0; }
#outline-list { list-style: none; margin: 0; padding: 6px; overflow-y: auto; }
.outline-item {
  display: flex; gap: 8px; width: 100%; text-align: left;
  background: transparent; border: none; color: inherit;
  padding: 6px 8px; border-radius: 6px; cursor: pointer; font-size: 13px;
}
.outline-item:hover { background: rgba(120, 120, 140, 0.15); }
.outline-item.current { background: var(--accent); color: var(--accent-fg); }
.outline-num { opacity: 0.7; min-width: 2.2em; }
#help-overlay {
  position: fixed; inset: 0; background: var(--overlay-bg);
  display: flex; align-items: center; justify-content: center; z-index: 40;
}
#help-card {
  background: var(--panel-bg); color: var(--panel-fg);
  border-radius: 10px; padding: 20px 24px; max-width: 420px; width: 88vw;
}
#help-card table { width: 100%; border-collapse: collapse; margin: 10px 0; }
#help-card th, #help-card td { text-align: left; padding: 4px 6px; font-size: 13px; }
#help-card th { color: var(--accent); font-weight: 600; white-space: nowrap; }
.hint { font-size: 12px; opacity: 0.75; }
[hidden] { display: none !important; }
"""

JS_TEMPLATE = """
(function () {
  "use strict";

  // Total slide count and swipe threshold are fixed at generation time.
  // 総スライド数とスワイプの閾値は生成時に固定される。
  var TOTAL = __TOTAL_PAGES__;
  var SWIPE_THRESHOLD_PX = 50;

  var current = 1;
  var stageEl = document.getElementById("stage");
  var slideEls = document.querySelectorAll(".slide");
  var counterEl = document.getElementById("counter");
  var progressEl = document.getElementById("progress-fill");
  var outlineEl = document.getElementById("outline");
  var helpEl = document.getElementById("help-overlay");
  var viewerEl = document.getElementById("viewer");

  function clamp(n) { return Math.max(1, Math.min(TOTAL, n)); }

  function readSlideFromUrl() {
    var params = new URLSearchParams(window.location.search);
    var n = parseInt(params.get("s"), 10);
    return isNaN(n) ? 1 : clamp(n);
  }

  // Only fetch a page's image the first time it becomes current or a
  // neighbor of the current slide (data-src -> src). Keeps a 122-page
  // deck from loading all pages up front.
  // ページ画像は「現在ページかその隣」になった最初のタイミングで初めて
  // 取得する（data-src → src）。122ページのデッキでも全ページを
  // 先読みしないための仕組み。
  function loadSlide(n) {
    if (n < 1 || n > TOTAL) return;
    var el = document.querySelector('.slide[data-page="' + n + '"]');
    var img = el && el.querySelector("img");
    if (img && !img.src && img.dataset.src) { img.src = img.dataset.src; }
  }

  function showSlide(n, pushHistory) {
    n = clamp(n);
    current = n;
    loadSlide(n - 1); loadSlide(n); loadSlide(n + 1);
    for (var i = 0; i < slideEls.length; i++) {
      slideEls[i].classList.toggle("active", Number(slideEls[i].dataset.page) === n);
    }
    updateChrome();
    var url = new URL(window.location.href);
    url.searchParams.set("s", String(n));
    if (pushHistory === false) { window.history.replaceState({ s: n }, "", url); }
    else { window.history.pushState({ s: n }, "", url); }
  }

  function next() { showSlide(current + 1, true); }
  function prev() { showSlide(current - 1, true); }

  function updateChrome() {
    counterEl.textContent = current + " / " + TOTAL;
    progressEl.style.width = (current / TOTAL * 100) + "%";
    var items = document.querySelectorAll(".outline-item");
    for (var i = 0; i < items.length; i++) {
      items[i].classList.toggle("current", Number(items[i].dataset.page) === current);
    }
  }

  function setOutline(open) { outlineEl.hidden = !open; }
  function setHelp(open) { helpEl.hidden = !open; }
  function closeAllPanels() { setOutline(false); setHelp(false); }

  function toggleFullscreen() {
    if (!document.fullscreenElement) { viewerEl.requestFullscreen().catch(function () {}); }
    else { document.exitFullscreen(); }
  }

  var NEXT_KEYS = ["ArrowRight", "PageDown", " ", "n", "N"];
  var PREV_KEYS = ["ArrowLeft", "PageUp", "p", "P"];

  document.addEventListener("keydown", function (ev) {
    if (ev.key === "Escape") { closeAllPanels(); return; }
    if (helpEl.hidden === false && ev.key !== "?") { return; }
    if (NEXT_KEYS.indexOf(ev.key) !== -1) { ev.preventDefault(); next(); }
    else if (PREV_KEYS.indexOf(ev.key) !== -1) { ev.preventDefault(); prev(); }
    else if (ev.key === "Home") { ev.preventDefault(); showSlide(1, true); }
    else if (ev.key === "End") { ev.preventDefault(); showSlide(TOTAL, true); }
    else if (ev.key === "f" || ev.key === "F") { toggleFullscreen(); }
    else if (ev.key === "t" || ev.key === "T") { setOutline(outlineEl.hidden); }
    else if (ev.key === "?") { setHelp(helpEl.hidden); }
  });

  document.getElementById("click-left").addEventListener("click", function () {
    if (outlineEl.hidden && helpEl.hidden) prev();
  });
  document.getElementById("click-right").addEventListener("click", function () {
    if (outlineEl.hidden && helpEl.hidden) next();
  });

  var touchStartX = null, touchStartY = null;
  stageEl.addEventListener("touchstart", function (ev) {
    var t = ev.changedTouches[0];
    touchStartX = t.clientX; touchStartY = t.clientY;
  }, { passive: true });
  stageEl.addEventListener("touchend", function (ev) {
    if (touchStartX === null) return;
    var t = ev.changedTouches[0];
    var dx = t.clientX - touchStartX, dy = t.clientY - touchStartY;
    if (Math.abs(dx) > SWIPE_THRESHOLD_PX && Math.abs(dx) > Math.abs(dy)) {
      if (dx < 0) next(); else prev();
    }
    touchStartX = null; touchStartY = null;
  }, { passive: true });

  document.getElementById("prevBtn").addEventListener("click", prev);
  document.getElementById("nextBtn").addEventListener("click", next);
  document.getElementById("fsBtn").addEventListener("click", toggleFullscreen);
  document.getElementById("outlineBtn").addEventListener("click", function () { setOutline(outlineEl.hidden); });
  document.getElementById("outlineCloseBtn").addEventListener("click", function () { setOutline(false); });
  document.getElementById("helpBtn").addEventListener("click", function () { setHelp(helpEl.hidden); });
  document.getElementById("helpCloseBtn").addEventListener("click", function () { setHelp(false); });
  document.getElementById("outline-list").addEventListener("click", function (ev) {
    var btn = ev.target.closest(".outline-item");
    if (!btn) return;
    showSlide(Number(btn.dataset.page), true);
    setOutline(false);
  });

  window.addEventListener("popstate", function () { showSlide(readSlideFromUrl(), false); });

  showSlide(readSlideFromUrl(), false);
})();
"""

HTML_TEMPLATE = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__DECK_TITLE__</title>
<style>__CSS__</style>
</head>
<body>
<div id="viewer">
  <div id="stage">
__SLIDES_HTML__
    <div id="click-left" class="click-zone" aria-label="前のスライド"></div>
    <div id="click-right" class="click-zone" aria-label="次のスライド"></div>
  </div>
</div>

<div id="bottombar">
  <button id="prevBtn" type="button" title="前へ（←）">◀ 前へ</button>
  <div id="progress-track"><div id="progress-fill"></div></div>
  <div id="counter">1 / __TOTAL_PAGES__</div>
  <button id="nextBtn" type="button" title="次へ（→）">次へ ▶</button>
  <button id="outlineBtn" type="button" title="目次（t）">目次</button>
  <button id="fsBtn" type="button" title="全画面（f）">全画面</button>
  <button id="helpBtn" type="button" title="ヘルプ（?）">?</button>
</div>

<aside id="outline" hidden>
  <div id="outline-head">
    <h2>目次 / Outline</h2>
    <button id="outlineCloseBtn" type="button" aria-label="閉じる">×</button>
  </div>
  <ol id="outline-list">
__OUTLINE_HTML__
  </ol>
</aside>

<div id="help-overlay" hidden>
  <div id="help-card">
    <h2>キーボード操作</h2>
    <table>
      <tr><th>→ / Space / PageDown / n</th><td>次のスライド</td></tr>
      <tr><th>← / PageUp / p</th><td>前のスライド</td></tr>
      <tr><th>Home / End</th><td>最初 / 最後のスライド</td></tr>
      <tr><th>f</th><td>全画面の切り替え</td></tr>
      <tr><th>t</th><td>目次パネルの表示切り替え</td></tr>
      <tr><th>?</th><td>このヘルプの表示切り替え</td></tr>
      <tr><th>Esc</th><td>パネル／ヘルプを閉じる</td></tr>
    </table>
    <p class="hint">スライド左右1/3のクリック、またはスワイプでも移動できます。</p>
    <button id="helpCloseBtn" type="button">閉じる</button>
  </div>
</div>

<script>__JS__</script>
</body>
</html>
"""


def build_slide_sections(pages, inline):
    """Render one <section class="slide"> per page.
    ページごとに <section class="slide"> を1つ生成する。
    Non-inline images carry data-src (not src) so loadSlide() in JS controls
    when each page is actually fetched.
    非inlineの画像は src ではなく data-src を持ち、実際の取得タイミングは
    JS側の loadSlide() が制御する。"""
    parts = []
    for p in pages:
        safe_title = html.escape(p["title"])
        alt = "Slide {}: {}".format(p["num"], safe_title)
        if inline:
            img = '<img src="{}" alt="{}">'.format(p["data_uri"], alt)
        else:
            img = '<img data-src="{}" alt="{}">'.format(p["file"], alt)
        parts.append('<section class="slide" data-page="{}">{}</section>'.format(p["num"], img))
    return "\n".join(parts)


def build_outline_list(pages):
    """Render the outline panel's page-number + title list.
    目次パネル用の「ページ番号＋タイトル」一覧を生成する。"""
    items = []
    for p in pages:
        safe_title = html.escape(p["title"])
        items.append(
            '<li><button type="button" class="outline-item" data-page="{n}">'
            '<span class="outline-num">{n}</span>'
            '<span class="outline-title">{title}</span></button></li>'.format(
                n=p["num"], title=safe_title
            )
        )
    return "\n".join(items)


def write_index_html(out_dir, deck_title, pages, inline):
    """Assemble HTML_TEMPLATE with the generated slide/outline markup.
    HTML_TEMPLATE にスライド／目次のマークアップを埋め込んで書き出す。"""
    doc = (
        HTML_TEMPLATE
        .replace("__DECK_TITLE__", html.escape(deck_title))
        .replace("__TOTAL_PAGES__", str(len(pages)))
        .replace("__SLIDES_HTML__", build_slide_sections(pages, inline))
        .replace("__OUTLINE_HTML__", build_outline_list(pages))
        .replace("__CSS__", CSS_TEMPLATE)
    )
    doc = doc.replace("__JS__", JS_TEMPLATE.replace("__TOTAL_PAGES__", str(len(pages))))
    out_path = out_dir / "index.html"
    out_path.write_text(doc, encoding="utf-8")
    return out_path


# =============================================================================
# CLI / レポート出力
# =============================================================================

def parse_args():
    """Define the command-line interface.
    コマンドライン引数を定義する。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pdf", help="Input Beamer PDF")
    parser.add_argument("out_dir", help="Output directory for the HTML viewer")
    parser.add_argument("--title", default=None, help="Deck title for <title> (default: PDF metadata)")
    parser.add_argument("--inline", action="store_true",
                         help="Embed pages as data URIs for a self-contained index.html")
    parser.add_argument("--pages", default=None, help="Page range, e.g. 1-16 (default: all pages)")
    return parser.parse_args()


def report_summary(pages, index_path):
    """Print a short conversion report: size range and any raster fallbacks.
    変換結果の要約（サイズ範囲・代替ページ）を表示する。"""
    sizes = [p["bytes"] for p in pages]
    fallback = [p for p in pages if p["source"] != "cairo"]
    print("Pages converted: {}".format(len(pages)))
    print("Per-page size: min={} KB, max={} KB, mean={:.0f} KB".format(
        min(sizes) // 1024, max(sizes) // 1024, (sum(sizes) / len(sizes)) / 1024
    ))
    if fallback:
        print("Fallback pages ({}):".format(len(fallback)))
        for p in fallback:
            print("  page {}: {} ({} KB)".format(p["num"], p["source"], p["bytes"] // 1024))
    else:
        print("No pages required a fallback (all cairo SVG <= {} KB).".format(
            SVG_SIZE_WARN_BYTES // 1024
        ))
    print("Wrote {} ({} KB)".format(index_path, index_path.stat().st_size // 1024))


def main():
    args = parse_args()
    pdf_path = Path(args.pdf).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    total_pages = get_page_count(pdf_path)
    start, end = parse_page_range(args.pages, total_pages)
    deck_title = args.title or get_deck_title_hint(pdf_path) or DEFAULT_TITLE

    print("Converting {} (pages {}-{} of {}) -> {}".format(
        pdf_path.name, start, end, total_pages, out_dir
    ))
    pages = build_pages(pdf_path, out_dir, start, end, args.inline)
    index_path = write_index_html(out_dir, deck_title, pages, args.inline)
    report_summary(pages, index_path)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError) as exc:
        print("error: {}".format(exc), file=sys.stderr)
        sys.exit(1)
