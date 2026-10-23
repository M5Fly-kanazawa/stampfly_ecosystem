# 共有スライド素材 / Shared Slide Assets

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このディレクトリについて

`docs/events/_shared/` は、`docs/events/` 配下の各イベント（勉強会・講座）のスライドが共通で使う素材を置く場所です。イベント固有の内容（章立て・進行スライド本体）は `docs/events/<event>/` 側にあります。全体の一覧・ビルド方法は [`docs/events/README.md`](../README.md) を参照してください。

### ディレクトリ構成

| ディレクトリ | 内容 |
|------------|------|
| `beamer/` | 共通 Beamer プリアンブル・スタイル（`preamble.tex`, `stampfly_slides.sty`）。各イベントの `.tex` からは `TEXINPUTS` 経由で参照する（後述） |
| `tikz/` | 全イベント共通の TikZ ダイアグラムソース（.tex、全30ファイル） |
| `images/` | `tikz/` から生成した PNG、写真・QRコード等の静止画像、送信機のイラスト（下記） |

### 送信機のイラスト

送信機（M5Stack Atom JoyStick）を上から見たイラスト。資料で送信機の写真を置き換えたいときに使う（置き換えは資料ごとに判断する）。

| ファイル | 用途 |
|---------|------|
| `images/controller_illust.svg` | 元データ。1400×882、背景透明。HTML に直接埋め込むときはこれを使う（Web フォント M PLUS Rounded 1c と Share Tech Mono を読み込むこと） |
| `images/controller_illust.pdf` | Beamer 等の LaTeX 用。ベクター、フォント埋め込み済み |
| `images/controller_illust.png` | Marp・Web・文書用。2800×1764、背景透明 |

- 形と部品の配置は、公式製品ページの寸法図（上面図）を 1 mm = 12.75 px で写したもの。表側の写真は透視投影で、背の高いスティックと画面が大きく外側に写るため、写真ではなく寸法図を基準にした
- 画面の表示は `MODE: 3`・`-Auto ALT-`・`-STABILIZE-`（Mode 3・ALT HOLD の状態）。スティックの根元の色のリング（左＝ピンク・水色、右＝緑・オレンジ）と右スティックの赤いリングは、操縦ガイド（`flight_experience/pilot_guide.html`）の色分けに合わせたもの
- SVG を直したら、PDF と PNG も作り直す。SVG を Web フォント付きの HTML（`@page{size:1400px 882px;margin:0}`）に埋め込み、Chrome のヘッドレスモードで `--print-to-pdf`（PDF）と `--screenshot --force-device-scale-factor=2 --default-background-color=00000000 --window-size=1400,882`（PNG）を実行する

### 各イベントからの参照方法

各イベントのデッキは `docs/events/<event>/slides/` を作業ディレクトリとしてコンパイルする。`<event>/slides/` はどの場合も `docs/events/` の直下から1階層下（イベント名） + 1階層下（`slides/`）にあるため、共有素材へは常に次の相対パスで届く:

- TikZ 図の `\input`: `../../_shared/tikz/<name>`
- 画像の `\includegraphics`: `../../_shared/images/<name>`
- `stampfly_slides.sty` の `\graphicspath`: `{../../_shared/images/}`
- プリアンブル・スタイルの解決: `TEXINPUTS=../../_shared/beamer//:`（`docs/events/Makefile` が各ビルドコマンドで自動設定する）

## 2. ビルド方法

ビルドは `docs/events/Makefile` が担当する（本ディレクトリにはビルド設定を置かない）。

```bash
cd docs/events
make tikz            # TikZ → PDF → PNG（_shared/images/ に出力）
make slides          # StampFly Workshop デッキ（= make workshop）
make dxh             # DXH講座デッキ
make dxh-docswell    # DXH講座デッキ（Docswell公開用・QRなし）
make sci             # SCI/SICEチュートリアル講座デッキ
make sci-docswell    # 同上（Docswell公開用・QRなし）
make chapter NAME=led_control   # 単一チャプター（全イベントの chapters/ から検索）
make clean           # 中間ファイル削除
```

詳細は [`docs/events/README.md`](../README.md) を参照。

## 3. TikZ 図の単体ビルド

```bash
cd _shared/tikz && lualatex motor_layout.tex
```

## 4. コードスニペット連携

ファームウェアソースからスライド内のコードを自動抽出できる。抽出先の探索対象チャプターディレクトリはイベント別に分かれたため、`tools/extract_snippets.py` は StampFly Workshop の `chapters/`（`docs/events/stampfly_workshop/slides/chapters/`）を対象にする:

```bash
python3 tools/extract_snippets.py check -v
python3 tools/extract_snippets.py expand
```

ソースファイルに `@@snippet: name` / `@@end-snippet: name` マーカーを設置し、`.tex` 側で `%%SNIPPET:lesson_03_led/solution.cpp:setup+loop%%` で参照する。

---

<a id="english"></a>

## 1. Overview

### About This Directory

`docs/events/_shared/` holds slide assets shared across every event (workshop, course) under `docs/events/`. Event-specific content (chapter files, the main deck) lives under `docs/events/<event>/`. See [`docs/events/README.md`](../README.md) for the full index and build instructions.

### Directory Layout

| Directory | Contents |
|-----------|----------|
| `beamer/` | Shared Beamer preamble/style (`preamble.tex`, `stampfly_slides.sty`), resolved from each event's `.tex` via `TEXINPUTS` (see below) |
| `tikz/` | TikZ diagram sources shared by every event (.tex, 30 files) |
| `images/` | PNGs generated from `tikz/`, photos, QR codes and other static images, and the controller illustration (below) |

### Controller Illustration

A top-view illustration of the controller (M5Stack Atom JoyStick), for use where a material should show a drawing instead of the controller photo (decide per material).

| File | Use |
|------|-----|
| `images/controller_illust.svg` | Source. 1400x882, transparent. Inline it in HTML (load the web fonts M PLUS Rounded 1c and Share Tech Mono) |
| `images/controller_illust.pdf` | For Beamer and other LaTeX. Vector, fonts embedded |
| `images/controller_illust.png` | For Marp, web and documents. 2800x1764, transparent |

- Outline and part placement follow the official top-view drawing from the product page at 1 mm = 12.75 px. The front photo is a perspective view in which the tall sticks and screen look larger and pushed outward, so the drawing, not the photo, is the reference
- The screen shows `MODE: 3`, `-Auto ALT-` and `-STABILIZE-` (Mode 3, ALT HOLD). The coloured rings at the stick bases (left pink/sky, right mint/orange) and the red ring on the right stick match the colour coding of the pilot guide (`flight_experience/pilot_guide.html`)
- After editing the SVG, regenerate the PDF and PNG: inline the SVG in an HTML page with the web fonts (`@page{size:1400px 882px;margin:0}`) and run headless Chrome with `--print-to-pdf` (PDF) and `--screenshot --force-device-scale-factor=2 --default-background-color=00000000 --window-size=1400,882` (PNG)

### How Each Event References These Assets

Every deck compiles with `docs/events/<event>/slides/` as its working directory. Since `<event>/slides/` is always exactly two levels below `docs/events/` (one for the event, one for `slides/`), the shared assets are always reachable via:

- TikZ `\input`: `../../_shared/tikz/<name>`
- `\includegraphics`: `../../_shared/images/<name>`
- `stampfly_slides.sty`'s `\graphicspath`: `{../../_shared/images/}`
- Preamble/style resolution: `TEXINPUTS=../../_shared/beamer//:` (set automatically by each `docs/events/Makefile` target)

## 2. Build Instructions

`docs/events/Makefile` owns the build (no build config lives in this directory).

```bash
cd docs/events
make tikz            # TikZ -> PDF -> PNG (into _shared/images/)
make slides          # StampFly Workshop deck (alias: make workshop)
make dxh             # DXH workshop deck
make dxh-docswell    # DXH deck, Docswell upload variant (no QR codes)
make sci             # SCI/SICE tutorial course deck
make sci-docswell    # Same, Docswell upload variant (no QR codes)
make chapter NAME=led_control   # Single chapter (searched across every event's chapters/)
make clean           # Remove build artifacts
```

See [`docs/events/README.md`](../README.md) for details.

## 3. Building a Single TikZ Figure

```bash
cd _shared/tikz && lualatex motor_layout.tex
```

## 4. Code Snippet Integration

Firmware source code can be auto-extracted into slides. Since chapters now live under per-event directories, `tools/extract_snippets.py` targets the StampFly Workshop chapters (`docs/events/stampfly_workshop/slides/chapters/`):

```bash
python3 tools/extract_snippets.py check -v   # Verify snippets
python3 tools/extract_snippets.py expand      # Expand to .build_chapters/
```
