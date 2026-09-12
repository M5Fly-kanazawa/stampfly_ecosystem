# docs/

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このディレクトリについて

`docs/` は人間が読むための文書と、公開サイト（GitHub Pages の `/docs/` 配下）のソースを置く場所である。
設計意図・使い方・教育資料など「なぜ・どうやって」を説明する文書を置き、コードのコメントや自動生成物は置かない。

### 記述規約

文書の書き方は [contributing/style-guide.md](contributing/style-guide.md) に従う。全文書の目録は
[DOCUMENT_INDEX.md](DOCUMENT_INDEX.md) が担うので、本ファイルでは網羅しない。

## 2. ディレクトリ構成

```
docs/
├── overview.md            # エコシステム全体の俯瞰図・推奨ワークフロー
├── next_step.md           # README（導入・初飛行）の次に読む詳細
├── index.md                # 公開サイト（mkdocs）のトップ
├── slides.md                # イベントスライド PDF の一覧
├── DOCUMENT_INDEX.md        # 全文書の目録（日英）
├── architecture/            # システム構成・設計判断・シミュレーション方針
├── reference/                # 仕様から生成される参照文書（手で書かない）
├── guides/                    # 利用者向けガイド
├── commands/                   # sf CLI コマンドリファレンス
├── setup/                       # OS 別セットアップ
├── contributing/                 # 開発規約
├── plans/                          # 計画文書（状態を明記、アーカイブは作らない）
├── events/                          # 勉強会・講座
├── university/                       # 大学講義
├── assets/, stylesheets/              # 画像・生成図・サイトのスタイル
├── telemetry/                          # UDP テレメトリ設計メモ
└── bonus/, experiments/                 # 番外資料・実験手順
```

## 3. 各ディレクトリの役割

| ディレクトリ | 役割 |
|---|---|
| `architecture/` | タスク分割・周期・優先度、vehicle / controller / protocol 間の責務境界、設計判断の背景。シミュレーション方針は `architecture/simulation-policy.md` を正とする |
| `reference/` | `protocol/tools/` が仕様から生成する参照文書（`flight-log-format.md` 等）。手書きしない |
| `guides/` | 安全・送信機・独自プログラム・ログ可視化・環境更新 等、利用者向けのガイド |
| `commands/` | sf CLI の各コマンドのリファレンス |
| `setup/` | OS 別のセットアップ手順 |
| `contributing/` | 文書スタイル・コミット規約・コマンド追加手順などの開発規約 |
| `plans/` | 機能ごとの計画・見直し文書。冒頭に状態（計画中／実装中／実装済み／見直し中）を書く |
| `events/` | イベント単位のディレクトリ + 共有素材 `_shared/`。Workshop は `events/stampfly_workshop/` にあるが、旧アーキテクチャで作られており廃棄・全面書き換え予定 |
| `university/` | シラバス・評価ルーブリックなど大学講義向けの資料 |

プロトコルの文章仕様（メッセージ一覧・オフセット表）は `protocol/README.md` と
`docs/reference/flight-log-format.md` にある。`docs/protocol/` は置かない。

## 4. 公開サイト

`landing/index.html` が GitHub Pages のルート、`docs/` は `.mkdocs/mkdocs.yml` で組版して `/docs/` に配信する
（`.github/workflows/deploy-pages.yml`）。`.mkdocs/mkdocs.yml` の目次は生きている文書だけを指す。

---

<a id="english"></a>

## 1. Overview

### About This Directory

`docs/` holds human-readable documentation and the source for the public site (served under
`/docs/` on GitHub Pages). It contains documents explaining "why" and "how" — design intent,
usage, educational material — not code comments or auto-generated output.

### Writing Guidelines

Follow [contributing/style-guide.md](contributing/style-guide.md). The full document index lives
in [DOCUMENT_INDEX.md](DOCUMENT_INDEX.md); this file does not attempt to be exhaustive.

## 2. Directory Structure

```
docs/
├── overview.md            # Ecosystem overview and recommended workflow
├── next_step.md           # What to read after the root README (setup, first flight)
├── index.md                # Public site (mkdocs) landing page
├── slides.md                # Index of event slide PDFs
├── DOCUMENT_INDEX.md        # Index of all documents (JA/EN)
├── architecture/            # System structure, design decisions, simulation policy
├── reference/                # Reference docs generated from spec (not hand-written)
├── guides/                    # User-facing guides
├── commands/                   # sf CLI command reference
├── setup/                       # Per-OS setup instructions
├── contributing/                 # Development conventions
├── plans/                          # Plan documents (state noted, no archive dir)
├── events/                          # Workshops and courses
├── university/                       # University course material
├── assets/, stylesheets/              # Images, generated figures, site styling
├── telemetry/                          # UDP telemetry design notes
└── bonus/, experiments/                 # Extra material and experiment procedures
```

## 3. Directory Roles

| Directory | Role |
|---|---|
| `architecture/` | Task partitioning, timing, priority; responsibility boundaries between vehicle / controller / protocol; background of design decisions. `architecture/simulation-policy.md` is the source of truth for simulation policy |
| `reference/` | Reference docs generated from spec by `protocol/tools/` (e.g. `flight-log-format.md`). Not hand-written |
| `guides/` | User-facing guides: safety, transmitter, writing your own program, flight-log visualization, environment upgrades, etc. |
| `commands/` | Reference for each sf CLI command |
| `setup/` | Per-OS setup instructions |
| `contributing/` | Development conventions: writing style, commit guidelines, how to add a command |
| `plans/` | Per-feature plan and review documents. State it up front (planned / in progress / done / under review) |
| `events/` | One directory per event plus shared material in `_shared/`. The Workshop lives in `events/stampfly_workshop/`, but it was built on the old architecture and is slated to be discarded and rewritten |
| `university/` | Syllabus and assessment rubrics for university courses |

The prose spec for the protocol (message list, offset tables) lives in `protocol/README.md` and
`docs/reference/flight-log-format.md`. There is no `docs/protocol/`.

## 4. Public Site

`landing/index.html` is the GitHub Pages root; `docs/` is built by `.mkdocs/mkdocs.yml` and served
under `/docs/` (`.github/workflows/deploy-pages.yml`). The `.mkdocs/mkdocs.yml` table of contents
only points at documents that are still live.
