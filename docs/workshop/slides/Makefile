# StampFly Workshop Slides — Build Automation
# Usage:
#   make all       — Build everything (tikz + slides)
#   make tikz      — TikZ diagrams → PDF → PNG
#   make slides    — Unified Beamer PDF (all lessons)
#   make dxh       — DXH workshop deck (high-school teacher session)
#   make chapter   — Single chapter: make chapter NAME=led_control
#   make clean     — Remove build artifacts

.PHONY: all tikz slides dxh chapter clean

# ---------------------------------------------------------------------------
# TikZ diagrams
# ---------------------------------------------------------------------------

TIKZ_DIR   := tikz
IMAGES_DIR := images
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
# Unified Beamer PDF (all lessons in one document)
# ---------------------------------------------------------------------------

BEAMER_DIR      := beamer
SLIDES_TEX      := stampfly_workshop.tex
SLIDES_PDF      := stampfly_workshop.pdf
CHAPTER_SOURCES := $(wildcard $(BEAMER_DIR)/chapters/*.tex)

slides: $(BEAMER_DIR)/$(SLIDES_PDF)

$(BEAMER_DIR)/$(SLIDES_PDF): $(BEAMER_DIR)/$(SLIDES_TEX) $(BEAMER_DIR)/preamble.tex $(BEAMER_DIR)/stampfly_slides.sty $(CHAPTER_SOURCES)
	cd $(BEAMER_DIR) && lualatex -interaction=nonstopmode $(SLIDES_TEX) && lualatex -interaction=nonstopmode $(SLIDES_TEX)

# ---------------------------------------------------------------------------
# DXH workshop deck (high-school teacher hands-on session)
# DXH講座（高校教員向け体験講座）進行スライド
# ---------------------------------------------------------------------------

DXH_TEX := dxh_workshop.tex
DXH_PDF := dxh_workshop.pdf

dxh: $(BEAMER_DIR)/$(DXH_PDF)

$(BEAMER_DIR)/$(DXH_PDF): $(BEAMER_DIR)/$(DXH_TEX) $(BEAMER_DIR)/preamble.tex $(BEAMER_DIR)/stampfly_slides.sty $(CHAPTER_SOURCES)
	cd $(BEAMER_DIR) && lualatex -interaction=nonstopmode $(DXH_TEX) && lualatex -interaction=nonstopmode $(DXH_TEX)

# Docswell-upload variant: same deck without QR codes (title-page Docswell QR,
# instructor-survey QR). Production deck = dxh_workshop.pdf (QR入り・本番投影用),
# upload deck = dxh_workshop_docswell.pdf (QRなし・ドクセル公開用).
# ドクセル公開用（QRなし）。本番投影用は dxh_workshop.pdf（QR入り）。
DXH_DOCSWELL_PDF := dxh_workshop_docswell.pdf

dxh-docswell: $(BEAMER_DIR)/$(DXH_DOCSWELL_PDF)

$(BEAMER_DIR)/$(DXH_DOCSWELL_PDF): $(BEAMER_DIR)/$(DXH_TEX) $(BEAMER_DIR)/preamble.tex $(BEAMER_DIR)/stampfly_slides.sty $(CHAPTER_SOURCES)
	cd $(BEAMER_DIR) && lualatex -interaction=nonstopmode -jobname=dxh_workshop_docswell "\def\DocswellBuild{1}\input{$(DXH_TEX)}" && lualatex -interaction=nonstopmode -jobname=dxh_workshop_docswell "\def\DocswellBuild{1}\input{$(DXH_TEX)}"

# ---------------------------------------------------------------------------
# Single chapter build (for fast iteration)
# Usage: make chapter NAME=led_control
# ---------------------------------------------------------------------------

chapter:
ifndef NAME
	$(error Usage: make chapter NAME=<chapter_name> (e.g., make chapter NAME=led_control))
endif
	@echo "Building single chapter: $(NAME)"
	@cd $(BEAMER_DIR) && lualatex -interaction=nonstopmode "\documentclass[aspectratio=169,14pt]{beamer}\usepackage{stampfly_slides}\input{preamble}\title{StampFly Workshop}\author{StampFly Workshop}\date{}\begin{document}\input{chapters/$(NAME)}\end{document}"

# ---------------------------------------------------------------------------
# All
# ---------------------------------------------------------------------------

all: tikz slides

# ---------------------------------------------------------------------------
# Clean
# ---------------------------------------------------------------------------

clean:
	rm -f $(TIKZ_DIR)/*.aux $(TIKZ_DIR)/*.log $(TIKZ_DIR)/*.pdf
	rm -f $(BEAMER_DIR)/*.aux $(BEAMER_DIR)/*.log $(BEAMER_DIR)/*.nav \
	      $(BEAMER_DIR)/*.out $(BEAMER_DIR)/*.snm $(BEAMER_DIR)/*.toc \
	      $(BEAMER_DIR)/*.vrb
	rm -f $(BEAMER_DIR)/$(SLIDES_PDF)
	rm -f $(BEAMER_DIR)/$(DXH_PDF)
	rm -f $(IMAGES_DIR)/*.png
