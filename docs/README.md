# docs/ ディレクトリガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このディレクトリについて

`docs/` は人間が読むためのドキュメントを格納するディレクトリである。
コードのコメントや自動生成ドキュメントではなく、設計意図・使い方・教育資料など「なぜ・どうやって」を説明する文書を置く。

### 記述規約

ドキュメントの記述スタイルは [contributing/style-guide.md](./contributing/style-guide.md) に従うこと。

## 2. ディレクトリ構成

```
docs/
├── README.md              # 本ファイル（ディレクトリガイド）
├── contributing/          # 開発ガイドライン
│   ├── style-guide.md         # 記述スタイル規約
│   ├── commit-guidelines.md   # コミットメッセージ規約
├── overview.md            # プロジェクト全体の俯瞰図
├── getting-started.md     # 初学者向けクイックスタート
│
├── architecture/          # システム設計・技術仕様
│   ├── stampfly-parameters.md   # 物理パラメータリファレンス
│   ├── control-system.md        # 制御系設計
│   └── task-structure.md        # タスク構成・周期・優先度
│
├── protocol/              # 通信プロトコル仕様
│   └── (protocol/spec/ から生成または手書き)
│
├── guides/                # ユーザー向けガイド・リファレンス
│   ├── safety.md                # 安全ガイド
│   ├── troubleshooting.md       # トラブルシューティング
│   ├── glossary.md              # 用語集
│   ├── tools.md                 # 開発ツール使用ガイド
│   └── flight-log-viz.md        # フライトログ可視化
│
├── assets/                # メディア・プレゼン資料
│   └── (画像・PDF・PPTX)
│
├── workshop/              # ワークショップ資料
│   └── (実習ガイド・スライド・競技ルール)
│
├── university/            # 大学講義資料
│   └── (シラバス・評価ルーブリック)
│
└── plans/                 # 開発計画・設計メモ
    └── (機能別の実装計画)
```

## 3. 各ディレクトリの役割

### architecture/

**目的:** システムの設計判断を記録する

**置くべき内容:**
- 物理パラメータ・機体仕様
- 制御系の構成と設計根拠
- タスク分割・実行周期・優先度
- センサ融合アルゴリズムの選定理由
- ファームウェアのモジュール構成

**読者:** 開発者、メンテナ、設計を理解したい人

**例:**
- 「なぜPID制御を採用したか」
- 「なぜ制御周期は100Hzか」
- 「姿勢推定にMadgwickフィルタを選んだ理由」

### protocol/

**目的:** 通信プロトコルの仕様を人間向けに説明する

**置くべき内容:**
- パケット構造の解説
- 各フィールドの意味・単位・更新規則
- エラーハンドリングの規約
- バージョン互換性のルール

**読者:** プロトコル実装者、デバッグする人

**注意:** 機械可読な仕様は `protocol/spec/` に置く。ここは人間向けの解説。

### guides/

**目的:** ユーザー向けのガイド・リファレンスを提供する

**置くべき内容:**
- 安全ガイド・トラブルシューティング
- 用語集
- ツールの使い方ガイド
- チュートリアル（ログ可視化など）

**読者:** 全ユーザー（学生から開発者まで）

### assets/

**目的:** メディアファイル・プレゼン資料を格納する

**置くべき内容:**
- プロジェクト紹介用画像（機体写真、シミュレータスクリーンショット）
- プレゼンテーション資料（PDF、HTML、PPTX）

**読者:** ドキュメントから参照される

### workshop/

**目的:** ワークショップ（実習形式の教育）の資料を提供する

**置くべき内容:**
- ワークショップガイド・スケジュール
- Beamer スライド・TikZ 図
- 競技ルール

**読者:** 学生、初学者、教員

### university/

**目的:** 大学講義向けの資料を提供する

**置くべき内容:**
- シラバス
- 評価ルーブリック

**読者:** 教員、大学関係者

### plans/

**目的:** 開発計画・設計検討メモを保管する

**置くべき内容:**
- 機能別の実装計画
- 設計検討の経緯
- TODO・課題リスト

**読者:** 開発チーム

**注意:** 完了した計画は削除するか、architecture/ に成果を移す。

## 4. ファイル配置の判断基準

新しいドキュメントを作成する際の判断フロー：

```
Q: 誰が読むか？
├─ 学生・初学者 → workshop/ または university/
├─ 開発者・メンテナ → 次へ
│
Q: 何についてか？
├─ 設計判断・仕様 → architecture/
├─ 通信プロトコル → protocol/
├─ ツールの使い方・ハウツー → guides/
├─ 実装計画・TODO → plans/
└─ プロジェクト全体の紹介 → ルート（overview.md等）
```

## 5. 命名規約

| 種類 | 形式 | 例 |
|------|------|-----|
| 一般ドキュメント | `kebab-case.md` | `control-system.md` |
| 計画ドキュメント | `UPPER_SNAKE_CASE.md` | `HILS_FIRMWARE_PLAN.md` |
| ガイド・規約 | `UPPER_SNAKE_CASE.md` | `STYLE_GUIDE.md` |

---

<a id="english"></a>

## 1. Overview

### About This Directory

`docs/` is the directory for human-readable documentation.
It contains documents explaining "why" and "how" - design intent, usage guides, and educational materials - rather than code comments or auto-generated docs.

### Writing Guidelines

Follow the style guide in [STYLE_GUIDE.md](./STYLE_GUIDE.md).

## 2. Directory Structure

```
docs/
├── README.md              # This file (directory guide)
├── STYLE_GUIDE.md         # Writing style guide
├── overview.md            # Project overview
├── getting-started.md     # Quick start for beginners
│
├── architecture/          # System design & technical specs
│   ├── stampfly-parameters.md   # Physical parameters reference
│   ├── control-system.md        # Control system design
│   └── task-structure.md        # Task structure, timing, priority
│
├── protocol/              # Communication protocol specs
│   └── (generated from or handwritten based on protocol/spec/)
│
├── guides/                # User-facing guides & reference
│   ├── safety.md                # Safety guide
│   ├── troubleshooting.md       # Troubleshooting
│   ├── glossary.md              # Glossary
│   ├── tools.md                 # Development tools guide
│   └── flight-log-viz.md        # Flight log visualization
│
├── assets/                # Media & presentation files
│   └── (images, PDF, PPTX)
│
├── workshop/              # Workshop materials
│   └── (lab guides, slides, competition rules)
│
├── university/            # University course materials
│   └── (syllabus, assessment rubric)
│
└── plans/                 # Development plans & design notes
    └── (feature implementation plans)
```

## 3. Directory Roles

### architecture/

**Purpose:** Record system design decisions

**Content:**
- Physical parameters & vehicle specifications
- Control system structure and design rationale
- Task partitioning, execution cycles, priorities
- Sensor fusion algorithm selection rationale
- Firmware module structure

**Audience:** Developers, maintainers, those wanting to understand the design

**Examples:**
- "Why we chose PID control"
- "Why the control cycle is 100Hz"
- "Why we selected Madgwick filter for attitude estimation"

### protocol/

**Purpose:** Explain communication protocol specs for humans

**Content:**
- Packet structure explanations
- Field meanings, units, update rules
- Error handling conventions
- Version compatibility rules

**Audience:** Protocol implementers, debuggers

**Note:** Machine-readable specs go in `protocol/spec/`. This is for human explanations.

### guides/

**Purpose:** Provide user-facing guides and reference materials

**Content:**
- Safety guide & troubleshooting
- Glossary
- Tool usage guides
- Tutorials (log visualization, etc.)

**Audience:** All users (students to developers)

### assets/

**Purpose:** Store media files and presentation materials

**Content:**
- Project images (drone photos, simulator screenshots)
- Presentation files (PDF, HTML, PPTX)

**Audience:** Referenced from documents

### workshop/

**Purpose:** Provide workshop (hands-on education) materials

**Content:**
- Workshop guide & schedule
- Beamer slides & TikZ diagrams
- Competition rules

**Audience:** Students, beginners, instructors

### university/

**Purpose:** Provide university course materials

**Content:**
- Syllabus
- Assessment rubric

**Audience:** Instructors, university staff

### plans/

**Purpose:** Store development plans and design notes

**Content:**
- Feature implementation plans
- Design discussion history
- TODO lists and issues

**Audience:** Development team

**Note:** Completed plans should be deleted or their outcomes moved to architecture/.

## 4. File Placement Decision Guide

Decision flow for creating new documents:

```
Q: Who is the reader?
├─ Students/beginners → workshop/ or university/
├─ Developers/maintainers → continue
│
Q: What is it about?
├─ Design decisions/specs → architecture/
├─ Communication protocol → protocol/
├─ Tool usage / how-to → guides/
├─ Implementation plans/TODO → plans/
└─ Overall project intro → root (overview.md, etc.)
```

## 5. Naming Conventions

| Type | Format | Example |
|------|--------|---------|
| General docs | `kebab-case.md` | `control-system.md` |
| Plan docs | `UPPER_SNAKE_CASE.md` | `HILS_FIRMWARE_PLAN.md` |
| Guides/Standards | `UPPER_SNAKE_CASE.md` | `STYLE_GUIDE.md` |
