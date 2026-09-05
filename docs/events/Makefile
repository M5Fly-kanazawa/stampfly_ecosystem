# StampFly Events — Build Automation
# StampFly イベント資料 — ビルド自動化
#
# Layout / 構成:
#   docs/events/_shared/            common TikZ sources, generated images, Beamer
#                                    preamble/style
#                                    共通 TikZ ソース・生成画像・Beamer
#                                    プリアンブル/スタイル
#   docs/events/<event>/slides/     one Beamer deck per event (own chapters/)
#                                    イベント毎の Beamer デッキ（専用 chapters/）
#
# Usage:
#   make all           — Build everything (tikz + all decks)
#   make tikz          — TikZ diagrams → PDF → PNG (into _shared/images/)
#   make slides        — StampFly Workshop deck (all lessons)
#   make workshop       — Alias for `make slides`
#   make dxh            — DXH high-school-teacher workshop deck
#   make dxh-docswell   — DXH deck, Docswell upload variant (no QR codes)
#   make sci             — SCI/SICE tutorial course deck
#   make sci-docswell    — SCI deck, Docswell upload variant (no QR codes)
#   make chapter NAME=<x> — Single chapter, searched across all events'
#                           chapters/ directories: make chapter NAME=led_control
#   make clean           — Remove build artifacts

.PHONY: all tikz slides workshop dxh dxh-docswell sci sci-docswell chapter clean

# ---------------------------------------------------------------------------
# Shared TikZ diagrams
# 共有 TikZ 図
# ---------------------------------------------------------------------------

TIKZ_DIR   := _shared/tikz
IMAGES_DIR := _shared/images
TIKZ_DPI   := 300

TIKZ_SOURCES := $(wildcard $(TIKZ_DIR)/*.tex)
TIKZ_PDFS    := $(TIKZ_SOURCES:.tex=.pdf)
TIKZ_PNGS    := $(patsubst $(TIKZ_DIR)/%.tex,$(IMAGES_DIR)/%.png,$(TIKZ_SOURCES))

tikz: $(TIKZ_PNGS)

$(TIKZ_DIR)/%.pdf: $(TIKZ_DIR)/%.tex
	cd $(TIKZ_DIR) && lualatex -interaction=nonstopmode $(notdir $<)

$(IMAGES_DIR)/%.png: $(TIKZ_DIR)/%.pdf
	@mkdir -p $(IMAGES_DIR)
	pdftoppm -png -r $(TIKZ_DPI) -singlefile $< $(IMAGES_DIR)/$*

# ---------------------------------------------------------------------------
# Shared Beamer preamble / style
# 共有 Beamer プリアンブル・スタイル
#
# Each deck lives in its own <event>/slides/ directory and is compiled with
# that directory as cwd (`cd <event>/slides && lualatex ...`). The shared
# preamble.tex / stampfly_slides.sty are NOT copied alongside each deck —
# they are resolved via TEXINPUTS pointed at _shared/beamer/. From any
# <event>/slides/ directory, ../../_shared/beamer is always the right path
# (events sit exactly one level below docs/events/).
#
# 各デッキは自分専用の <event>/slides/ ディレクトリに置かれ、そのディレクトリを
# カレントディレクトリとしてコンパイルする（`cd <event>/slides && lualatex ...`）。
# 共有の preamble.tex / stampfly_slides.sty は各デッキ側にコピーせず、
# _shared/beamer/ を指す TEXINPUTS 経由で解決する。events は docs/events/ の
# ちょうど1階層下にあるため、どの <event>/slides/ からも ../../_shared/beamer
# で常に正しく届く。
# ---------------------------------------------------------------------------

SHARED_BEAMER_TEXINPUTS := TEXINPUTS=../../_shared/beamer//:

# ---------------------------------------------------------------------------
# StampFly Workshop deck (standard 4+1-day workshop, all lessons)
# StampFly 勉強会デッキ（標準4+1日構成・全レッスン）
# ---------------------------------------------------------------------------

WORKSHOP_DIR      := stampfly_workshop/slides
WORKSHOP_TEX      := stampfly_workshop.tex
WORKSHOP_PDF      := stampfly_workshop.pdf
WORKSHOP_CHAPTERS := $(wildcard $(WORKSHOP_DIR)/chapters/*.tex)

slides: $(WORKSHOP_DIR)/$(WORKSHOP_PDF)
workshop: slides

$(WORKSHOP_DIR)/$(WORKSHOP_PDF): $(WORKSHOP_DIR)/$(WORKSHOP_TEX) _shared/beamer/preamble.tex _shared/beamer/stampfly_slides.sty $(WORKSHOP_CHAPTERS)
	cd $(WORKSHOP_DIR) && $(SHARED_BEAMER_TEXINPUTS) lualatex -interaction=nonstopmode $(WORKSHOP_TEX) && $(SHARED_BEAMER_TEXINPUTS) lualatex -interaction=nonstopmode $(WORKSHOP_TEX)

# ---------------------------------------------------------------------------
# DXH workshop deck (high-school teacher hands-on session)
# DXH講座（高校教員向け体験講座）進行スライド
# ---------------------------------------------------------------------------

DXH_DIR      := dxh2026/slides
DXH_TEX      := dxh_workshop.tex
DXH_PDF      := dxh_workshop.pdf
DXH_CHAPTERS := $(wildcard $(DXH_DIR)/chapters/*.tex)

dxh: $(DXH_DIR)/$(DXH_PDF)

$(DXH_DIR)/$(DXH_PDF): $(DXH_DIR)/$(DXH_TEX) _shared/beamer/preamble.tex _shared/beamer/stampfly_slides.sty $(DXH_CHAPTERS)
	cd $(DXH_DIR) && $(SHARED_BEAMER_TEXINPUTS) lualatex -interaction=nonstopmode $(DXH_TEX) && $(SHARED_BEAMER_TEXINPUTS) lualatex -interaction=nonstopmode $(DXH_TEX)

# Docswell-upload variant: same deck without QR codes (title-page Docswell QR,
# instructor-survey QR). Production deck = dxh_workshop.pdf (QR入り・本番投影用),
# upload deck = dxh_workshop_docswell.pdf (QRなし・ドクセル公開用).
# ドクセル公開用（QRなし）。本番投影用は dxh_workshop.pdf（QR入り）。
DXH_DOCSWELL_PDF := dxh_workshop_docswell.pdf

dxh-docswell: $(DXH_DIR)/$(DXH_DOCSWELL_PDF)

$(DXH_DIR)/$(DXH_DOCSWELL_PDF): $(DXH_DIR)/$(DXH_TEX) _shared/beamer/preamble.tex _shared/beamer/stampfly_slides.sty $(DXH_CHAPTERS)
	cd $(DXH_DIR) && $(SHARED_BEAMER_TEXINPUTS) lualatex -interaction=nonstopmode -jobname=dxh_workshop_docswell "\def\DocswellBuild{1}\input{$(DXH_TEX)}" && $(SHARED_BEAMER_TEXINPUTS) lualatex -interaction=nonstopmode -jobname=dxh_workshop_docswell "\def\DocswellBuild{1}\input{$(DXH_TEX)}"

# ---------------------------------------------------------------------------
# SCI/SICE tutorial deck (2026-09-10, control-engineering researchers)
# SCI/SICE チュートリアル講座（2026-09-10、制御工学研究者向け）進行スライド
# ---------------------------------------------------------------------------

SCI_DIR          := sci_tutorial_2026/slides
SCI_TEX          := sci_tutorial.tex
SCI_PDF          := sci_tutorial.pdf
SCI_DOCSWELL_PDF := sci_tutorial_docswell.pdf
SCI_CHAPTERS     := $(wildcard $(SCI_DIR)/chapters/*.tex)

sci: $(SCI_DIR)/$(SCI_PDF)

$(SCI_DIR)/$(SCI_PDF): $(SCI_DIR)/$(SCI_TEX) _shared/beamer/preamble.tex _shared/beamer/stampfly_slides.sty $(SCI_CHAPTERS)
	cd $(SCI_DIR) && $(SHARED_BEAMER_TEXINPUTS) lualatex -interaction=nonstopmode $(SCI_TEX) && $(SHARED_BEAMER_TEXINPUTS) lualatex -interaction=nonstopmode $(SCI_TEX)

sci-docswell: $(SCI_DIR)/$(SCI_DOCSWELL_PDF)

$(SCI_DIR)/$(SCI_DOCSWELL_PDF): $(SCI_DIR)/$(SCI_TEX) _shared/beamer/preamble.tex _shared/beamer/stampfly_slides.sty $(SCI_CHAPTERS)
	cd $(SCI_DIR) && $(SHARED_BEAMER_TEXINPUTS) lualatex -interaction=nonstopmode -jobname=sci_tutorial_docswell "\def\DocswellBuild{1}\input{$(SCI_TEX)}" && $(SHARED_BEAMER_TEXINPUTS) lualatex -interaction=nonstopmode -jobname=sci_tutorial_docswell "\def\DocswellBuild{1}\input{$(SCI_TEX)}"

# ---------------------------------------------------------------------------
# Single chapter build (for fast iteration)
# Usage: make chapter NAME=led_control
# Searches every event's chapters/ directory for NAME.tex.
# 全イベントの chapters/ ディレクトリから NAME.tex を探して単体ビルドする。
# ---------------------------------------------------------------------------

CHAPTER_SEARCH_DIRS := $(WORKSHOP_DIR) $(DXH_DIR) $(SCI_DIR)

chapter:
ifndef NAME
	$(error Usage: make chapter NAME=<chapter_name> (e.g., make chapter NAME=led_control))
endif
	@dir=""; \
	for d in $(CHAPTER_SEARCH_DIRS); do \
		if [ -f "$$d/chapters/$(NAME).tex" ]; then dir="$$d"; break; fi; \
	done; \
	if [ -z "$$dir" ]; then \
		echo "chapter '$(NAME).tex' not found under: $(CHAPTER_SEARCH_DIRS)"; exit 1; \
	fi; \
	echo "Building single chapter: $(NAME) (found in $$dir/chapters/)"; \
	cd "$$dir" && $(SHARED_BEAMER_TEXINPUTS) lualatex -interaction=nonstopmode "\documentclass[aspectratio=169,14pt]{beamer}\usepackage{stampfly_slides}\input{preamble}\title{StampFly}\author{StampFly}\date{}\begin{document}\input{chapters/$(NAME)}\end{document}"

# ---------------------------------------------------------------------------
# All
# ---------------------------------------------------------------------------

all: tikz slides dxh dxh-docswell sci sci-docswell

# ---------------------------------------------------------------------------
# Clean
# ---------------------------------------------------------------------------

clean:
	rm -f $(TIKZ_DIR)/*.aux $(TIKZ_DIR)/*.log $(TIKZ_DIR)/*.pdf
	for d in $(WORKSHOP_DIR) $(DXH_DIR) $(SCI_DIR); do \
		rm -f $$d/*.aux $$d/*.log $$d/*.nav $$d/*.out $$d/*.snm $$d/*.toc $$d/*.vrb \
		      $$d/*.fls $$d/*.fdb_latexmk $$d/*.listing; \
	done
	rm -f $(WORKSHOP_DIR)/$(WORKSHOP_PDF)
	rm -f $(DXH_DIR)/$(DXH_PDF) $(DXH_DIR)/$(DXH_DOCSWELL_PDF)
	rm -f $(SCI_DIR)/$(SCI_PDF) $(SCI_DIR)/$(SCI_DOCSWELL_PDF)
	# Only remove PNGs that make tikz can regenerate — NOT the whole
	# images/ directory, which also holds photos/QR codes with no
	# source to rebuild from.
	# make tikz で再生成できるPNGのみ削除する — images/ ディレクトリ全体は
	# 削除しない（再生成元を持たない写真・QRコードも同じ場所にあるため）。
	rm -f $(TIKZ_PNGS)
