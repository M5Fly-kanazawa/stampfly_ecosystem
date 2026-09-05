# イベント資料 / Event Materials

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このディレクトリについて

StampFly エコシステムを使った勉強会・講座・体験講座それぞれの資料を、イベント単位のディレクトリにまとめている。スライドの共通素材（TikZ図・画像・Beamerスタイル）は `_shared/` に集約し、各イベントは自分の章立て・進行スライドだけを持つ。

### ディレクトリ構成

| ディレクトリ | 内容 |
|------------|------|
| `_shared/` | 全イベント共通の TikZ・画像・Beamer プリアンブル/スタイル。詳細は [`_shared/README.md`](_shared/README.md) |
| `stampfly_workshop/` | 標準の StampFly 勉強会（4+1日構成、Lesson 0–13） |
| `dxh2026/` | DXH（高等学校DX加速化推進事業）高校教員向け体験講座（2026-07-18/19） |
| `sci_tutorial_2026/` | SCI/SICE チュートリアル講座 2026（2026-09-10、制御工学研究者向け） |

各イベントディレクトリは次の構成を基本とする:

```
<event>/
├── README.md 等          # ガイド・進行表・競技ルールなど、イベント固有の文書
├── survey/               # アンケート（存在する場合）
└── slides/
    ├── <deck>.tex         # 進行スライド本体
    ├── <deck>.pdf          # ビルド済みPDF（配布用に追跡）
    └── chapters/           # 章別 .tex ファイル
```

## 2. ビルド方法

ビルドは本ディレクトリ直下の `Makefile` が担当する。各デッキは自分の `slides/` ディレクトリをカレントディレクトリとしてコンパイルし、共有プリアンブル・スタイルは `TEXINPUTS` 経由で `_shared/beamer/` から解決する。

### 前提条件

```bash
# TeX Live (lualatex + luatexja)
lualatex --version
```

### ビルドコマンド

```bash
cd docs/events

make tikz             # _shared/tikz/*.tex → PDF → PNG（_shared/images/ に出力）
make slides           # StampFly Workshop デッキ（全レッスン統合PDF。make workshop でも同じ）
make dxh              # DXH講座デッキ（本番投影用・QR入り）
make dxh-docswell     # DXH講座デッキ（Docswell公開用・QRなし）
make sci              # SCI/SICEチュートリアル講座デッキ
make sci-docswell     # SCI/SICEチュートリアル講座デッキ（Docswell公開用・QRなし）
make chapter NAME=led_control   # 単一チャプターの高速イテレーション用ビルド
                                 # （全イベントの chapters/ から NAME.tex を探す）
make all              # 上記すべて
make clean            # 中間ファイル削除
```

### 個別ビルド（Makefile を介さない場合）

```bash
# TikZ 単体
cd _shared/tikz && lualatex motor_layout.tex

# 各デッキ（例: SCI チュートリアル）
cd sci_tutorial_2026/slides
TEXINPUTS=../../_shared/beamer//: lualatex -interaction=nonstopmode sci_tutorial.tex
TEXINPUTS=../../_shared/beamer//: lualatex -interaction=nonstopmode sci_tutorial.tex
```

## 3. イベント一覧

| イベント | ディレクトリ | スライド | 開催日 |
|---------|------------|---------|--------|
| StampFly 勉強会 | [`stampfly_workshop/`](stampfly_workshop/workshop_guide.md) | `stampfly_workshop/slides/stampfly_workshop.pdf`（157ページ） | 通年・随時開催 |
| DXH 高校教員向け体験講座 | [`dxh2026/`](dxh2026/README.md) | `dxh2026/slides/dxh_workshop.pdf`（41ページ） | 2026-07-18/19 |
| SCI/SICE チュートリアル講座 2026 | [`sci_tutorial_2026/`](sci_tutorial_2026/README.md) | `sci_tutorial_2026/slides/sci_tutorial.pdf`（122ページ） | 2026-09-10 |

## 4. 新しいイベントを追加する場合

1. `docs/events/<new_event>/slides/chapters/` を作成し、章別 `.tex` を置く
2. 進行スライド本体 `<new_event>/slides/<deck>.tex` で `\usepackage{stampfly_slides}` と `\input{preamble}` を使う（`TEXINPUTS` で自動解決される）
3. TikZ 図・画像は `_shared/tikz/` `_shared/images/` を再利用し、新規図のみ追加する
4. `Makefile` に新しいターゲット（例: `make new_event`）を追加する

---

<a id="english"></a>

## 1. Overview

### About This Directory

Materials for every StampFly-ecosystem workshop, course, and hands-on session live under one directory per event. Shared slide assets (TikZ diagrams, images, Beamer style) are consolidated in `_shared/`; each event holds only its own chapter files and main deck.

### Directory Layout

| Directory | Contents |
|-----------|----------|
| `_shared/` | TikZ, images, and Beamer preamble/style shared by every event. See [`_shared/README.md`](_shared/README.md) |
| `stampfly_workshop/` | The standard StampFly workshop (4+1-day format, Lessons 0-13) |
| `dxh2026/` | DXH (Digital Transformation High-school acceleration program) hands-on session for high-school teachers (2026-07-18/19) |
| `sci_tutorial_2026/` | SCI/SICE Tutorial Course 2026 (2026-09-10, for control-engineering researchers) |

Each event directory follows this basic layout:

```
<event>/
├── README.md, etc.      # Guides, schedules, competition rules — event-specific docs
├── survey/               # Survey materials (if any)
└── slides/
    ├── <deck>.tex         # Main deck source
    ├── <deck>.pdf          # Built PDF (tracked for distribution)
    └── chapters/           # Per-chapter .tex files
```

## 2. Build Instructions

The `Makefile` in this directory owns the build. Each deck compiles with its own `slides/` directory as the working directory; the shared preamble/style are resolved from `_shared/beamer/` via `TEXINPUTS`.

### Prerequisites

```bash
# TeX Live (lualatex + luatexja)
lualatex --version
```

### Build Commands

```bash
cd docs/events

make tikz             # _shared/tikz/*.tex -> PDF -> PNG (into _shared/images/)
make slides           # StampFly Workshop deck (all lessons; same as make workshop)
make dxh              # DXH workshop deck (production, with QR codes)
make dxh-docswell     # DXH deck, Docswell upload variant (no QR codes)
make sci              # SCI/SICE tutorial course deck
make sci-docswell     # Same, Docswell upload variant (no QR codes)
make chapter NAME=led_control   # Fast single-chapter build
                                 # (searches every event's chapters/ for NAME.tex)
make all              # All of the above
make clean            # Remove build artifacts
```

### Building Without the Makefile

```bash
# Single TikZ figure
cd _shared/tikz && lualatex motor_layout.tex

# Any deck (example: SCI tutorial)
cd sci_tutorial_2026/slides
TEXINPUTS=../../_shared/beamer//: lualatex -interaction=nonstopmode sci_tutorial.tex
TEXINPUTS=../../_shared/beamer//: lualatex -interaction=nonstopmode sci_tutorial.tex
```

## 3. Event Index

| Event | Directory | Slides | Date |
|-------|-----------|--------|------|
| StampFly Workshop | [`stampfly_workshop/`](stampfly_workshop/workshop_guide.md) | `stampfly_workshop/slides/stampfly_workshop.pdf` (157 pages) | Ongoing |
| DXH high-school teacher workshop | [`dxh2026/`](dxh2026/README.md) | `dxh2026/slides/dxh_workshop.pdf` (41 pages) | 2026-07-18/19 |
| SCI/SICE Tutorial Course 2026 | [`sci_tutorial_2026/`](sci_tutorial_2026/README.md) | `sci_tutorial_2026/slides/sci_tutorial.pdf` (122 pages) | 2026-09-10 |

## 4. Adding a New Event

1. Create `docs/events/<new_event>/slides/chapters/` and add per-chapter `.tex` files
2. In the main deck `<new_event>/slides/<deck>.tex`, use `\usepackage{stampfly_slides}` and `\input{preamble}` (resolved automatically via `TEXINPUTS`)
3. Reuse `_shared/tikz/` and `_shared/images/` for existing diagrams; add only new ones
4. Add a new target (e.g. `make new_event`) to the `Makefile`
