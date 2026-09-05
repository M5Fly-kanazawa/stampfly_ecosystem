# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Session Rules

- **セッション開始時またはコンテキスト圧縮後に以下を読むこと:**
  - `PROJECT_PLAN.md`
  - `.claude/settings.local.json`
  - 前回のコミットログ（Next stepsから作業を再開）
- **応答は日本語で行うこと**
- **コードを変更したら必ずコミットすること** - 変更をローカルに残さず、適切な単位でコミットする
  - **必ず `/commit` スキルを使用する** - `docs/contributing/commit-guidelines.md` に基づいたコミットメッセージを自動作成
  - **Next steps セクションを必ず含める** - 次回セッション開始時のタスクを明記
  - 作業終了時、ファイル変更後、重要な節目で自動実行
- **sf CLI を積極的に使用すること** - ビルド、書き込み、診断などは `idf.py` を直接呼ぶのではなく `sf` コマンドを優先する
- **制御系パラメータの変更提案は必ず数値的シミュレーションで裏付けてから行うこと** - PIDゲイン、フィルタ定数、制御リミット等の変更を提案する際、実際のフライトログデータを使ったシミュレーションで効果を定量的に確認してから提示する。「Tiを短くすれば改善する」のような定性的な推測だけで提案しない。シミュレーションの結果、逆効果であれば提案しない

## Build Environment

### ESP-IDF
開発環境の初期化（`setup_env.sh` が ESP-IDF の `export.sh` を内部で呼ぶ）:
```bash
source setup_env.sh
```

ファームウェアのビルド:
```bash
cd firmware/vehicle  # or firmware/controller
idf.py build
idf.py flash monitor
```

### sf CLI（推奨）
sf CLI は ESP-IDF 環境に統合された開発ツール。`idf.py` を直接使う代わりにこちらを優先する：
```bash
source setup_env.sh  # 開発環境をアクティブ化（ESP-IDF + sf CLI）

sf doctor              # 環境診断（問題があればまずこれを実行）
sf build vehicle       # vehicleファームウェアをビルド
sf build controller    # controllerファームウェアをビルド
sf flash vehicle       # vehicleに書き込み
sf monitor             # シリアルモニタを開く
sf flash vehicle -m    # 書き込み後にモニタを開く
```

**sf CLI の開発・改善:**
- コマンド実装: `lib/sfcli/commands/`
- ユーティリティ: `lib/sfcli/utils/`
- 新コマンド追加時は既存コマンドのパターンに従う
- 問題発見時は積極的に修正してフレームワークを改善する

**ツール統合方針:**
- **全てのツールは sf CLI 経由で使用する** - スタンドアロンの Python スクリプトを直接実行しない
- `tools/` 配下のスクリプトは sf CLI のバックエンド実装として扱う
- 新しいツールを作成する場合は、必ず対応する sf コマンドも追加する
- ラッパースクリプト（viz_*.py 等）は非推奨、sf コマンドのオプションで対応する

**sf CLI コマンド一覧:**
| コマンド | 説明 |
|---------|------|
| `sf doctor` | 環境診断 |
| `sf build [target]` | ファームウェアビルド |
| `sf flash [target]` | 書き込み（-m でモニタ付き、--gui でGUI書き込みアプリ起動）|
| `sf monitor` | シリアルモニタ |
| `sf telemetry` | 50Hzテレメトリのライブ表示。既定=ターミナル、`--web` でブラウザ表示（UDP:5005→SSE）|
| `sf log list` | ログファイル一覧 |
| `sf log capture` | USB経由バイナリログ取得 |
| `sf log wifi` | WiFi経由400Hzテレメトリ取得 |
| `sf log convert` | バイナリ→CSV変換 |
| `sf log info` | ログファイル情報表示 |
| `sf log analyze` | フライトログ解析 |
| `sf log viz` | ログ可視化 |
| `sf params check` | 物理パラメータ整合検査（C_T/C_Q/κ/慣性等の手動コピーの食い違い検出）|
| `sf cal list` | キャリブレーション一覧 |
| `sf cal gyro/accel/mag` | 各種キャリブレーション |
| `sf sim list/run` | シミュレータ操作 |
| `sf blocks` | ブロックプログラミングWeb UI（Blockly、--demo でデモモード）|

### Genesis Simulator
Genesis物理シミュレータはvenv仮想環境にインストールされている:
```bash
cd simulator/genesis
source venv/bin/activate
cd scripts
python <script_name>.py
```

## Writing Conventions

### Code Comments
コメントは英語と日本語を併記する：
```c
// Initialize the motor driver
// モータドライバを初期化
motor_init();
```

### Documentation

ドキュメントは以下のルールに従う：

#### 1. バイリンガル構成（Bilingual Structure）

- 日本語を先に、英語を後に記述
- 冒頭に英語版の存在を示す注記を入れる（`#english` へのリンク付き）
- 英語セクションは `---` で区切り、直前に `<a id="english"></a>` を設置

```markdown
# Document Title

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて
...

### 対象読者
...

## 2. 詳細

### 仕様
...

---

<a id="english"></a>

## 1. Overview

### About This Document
...

### Target Audience
...

## 2. Details

### Specifications
...
```

#### 2. 見出しフォーマット（Heading Format）

| レベル | 形式 | 例 |
|--------|------|-----|
| ドキュメントタイトル | `# Title` | `# StampFly Vehicle Firmware` |
| 章 | `## N. 章タイトル` | `## 1. 概要` / `## 1. Overview` |
| 節 | `### 節タイトル` | `### このプロジェクトについて` |
| 項 | `#### 項タイトル` | `#### パラメータ一覧` |

**注意:**
- 章には番号を付ける（`## 1.`, `## 2.`, ...）
- 節・項には番号を付けない（`## 1.1` ではなく `###`）
- 日本語と英語で同じ番号体系を使う

#### 3. 構造化情報（Structured Information）

リスト形式よりもテーブルを優先する：

```markdown
| 機能 | 説明 |
|------|------|
| IMU | BMI270（加速度・ジャイロ）400Hz |
| 気圧センサー | BMP280 50Hz |
```

#### 4. コードブロック（Code Blocks）

言語を明示する：

````markdown
```cpp
// Initialize motor
motor_init();
```

```bash
idf.py build flash monitor
```
````

#### 5. 図表（Diagrams）

ASCII アートを活用する：

```markdown
```
               Front
          FL (M4)   FR (M1)
             ╲   ▲   ╱
              ╲  │  ╱
               ╲ │ ╱
                ╲│╱
                 ╳         ← Center
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲
             ╱   │   ╲
          RL (M3)    RR (M2)
                Rear
```
```

#### 6. 参考資料（References）

外部リンクはテーブル形式で整理：

```markdown
| リポジトリ | 説明 |
|-----------|------|
| [StampFly技術仕様](https://github.com/...) | ハードウェア仕様書 |
```

## Slide Rules（スライドルール）

### 自動レビュー必須

**CRITICAL: スライド（Beamer / TikZ）の `.tex` ファイルを変更したら、必ず PDF リビルドとレビューを実行すること。ユーザーへの確認は不要 — 変更したら自動的に行う。**

- スライド `.tex` を編集 → PDF リビルド → サブエージェントで目視確認 → 問題があれば修正、の一連を1サイクルとして完結させる
- 「レビューしますか？」とユーザーに聞かない。変更したら必ずやる
- 作成だけして fix を次回に回さない

### 画像確認はサブエージェント限定

**CRITICAL: スライド PDF の画像をメインコンテキストで Read してはならない。必ずサブエージェント内で完結させること。**

このルールは以下の全てに適用される：
- フルレビュー（チェックリスト適用）
- 簡易確認（1〜2ページの目視チェック）
- ユーザーから「確認して」と依頼された場合

理由: 画像の Read はコンテキストを大きく消費し、rate limit に抵触する原因になる。サブエージェント内で画像確認を完結させ、メインコンテキストにはテキストの指摘事項だけを返す。

## Review Checklist

TikZ 図やスライド（Beamer / Marp）を変更したら、以下のチェックリストでレビューし、問題があれば同一セッション内で修正すること。上記「Slide Rules」の自動レビュー・サブエージェント限定ルールにも従う。

レビューは **2段構成** で行う:
1. **Stage 1: TikZ 図チェック** — フレームワーク非依存。図単体の品質を検証
2. **Stage 2: スライド/資料チェック** — 埋め込み先（Beamer / Marp）固有のレイアウト・構造を検証

### レビュー手順

1. **TikZ の `.tex` を1ファイルずつ読み、Stage 1 チェックリストを適用する。** T4（ルーティング）は図全体の全矢印を対象とし、直角ルーティング（フォーク形状）になっているか、ノード干渉がないか、矢頭前の幹が十分かを確認する。斜め直線があれば直角ルーティングに修正する。TikZ は線がノード背景の上に描画されるため、貫通が画像上で見えない場合がある — **直角ルーティングを使えば構造的に排除できるため、座標計算による事後検証より設計段階での排除を優先する**
2. **TikZ 図を PDF → PNG 化してサブエージェントで目視確認する**（テキストレビューだけでは枠線とテキストの干渉等を検出できないため）
3. **スライドの `.tex` / `.md` を読み、Stage 2 チェックリストを適用する**（Beamer なら BL + BS 項目、Marp なら ML 項目）
4. **スライド PDF を画像化してサブエージェントで目視確認する**（**Slide Image Rule 参照**）
5. 問題を発見したら修正し、修正ごとに fix コミットする

### 画像化による目視確認手順

テキストベースのレビューでは検出できない問題（はみ出し、重なり、切れ）を発見するため、**必ず PDF を画像化して目視確認する**。

#### サブエージェントによる目視確認（必須）

Agent ツールで以下のようなプロンプトのサブエージェントを起動する:

```
スライドの目視確認を行ってください。

1. TikZ をコンパイル・画像化して Read で確認:
   cd <tikz_dir>
   lualatex -interaction=nonstopmode <file>.tex
   magick -density 150 <file>.pdf -quality 85 /tmp/tikz_<file>.png

2. スライドをコンパイル・画像化して各ページを Read で確認:
   [Beamer の場合。共有プリアンブル/スタイルは docs/events/_shared/beamer/ にあり、
    docs/events/Makefile が TEXINPUTS を設定する。`cd docs/events && make sci` の
    ように Makefile 経由でビルドするか、下記のように TEXINPUTS を手動指定する]
   cd docs/events/sci_tutorial_2026/slides
   TEXINPUTS=../../_shared/beamer//: lualatex -interaction=nonstopmode sci_tutorial.tex
   magick -density 150 "sci_tutorial.pdf[<page>]" -quality 85 /tmp/beamer_p<N>.png

   [Marp の場合]
   cd docs/assets
   npx --yes @marp-team/marp-cli presentation.md --pdf --allow-local-files
   magick -density 150 "presentation.pdf[<page>]" -quality 85 /tmp/marp_p<N>.png

3. 以下のチェックリストで各ページを確認し、問題があれば報告:
   - 図・テキストがスライド端で切れていないか
   - ノード・ラベルが重なっていないか
   - ノードの枠線がテキストにかかっていないか
   - テキストがノードの枠からはみ出していないか
   - resizebox のスケーリングで横幅・縦幅が収まっているか（Beamer）
   - フッターにコンテンツが被っていないか

問題がなければ「問題なし」、問題があれば該当ページ番号・チェック項目ID・具体的な内容を報告してください。
コードの修正は行わず、レビュー結果の報告のみ行ってください。
```

**ポイント:**
- 解像度は 150dpi、品質 85 で十分（200dpi より軽量）
- 変更したページ付近のみ確認すればよい（全ページを毎回確認しない）
- サブエージェントの結果を受けて、メイン側で修正→再度サブエージェントで確認のサイクルを回す

---

### Stage 1: TikZ 図チェック（T 項目）

フレームワーク非依存。TikZ 図の standalone PDF / PNG 単体で検証する。Beamer・Marp どちらに埋め込む場合でも適用。

**基本原則:** `espnow_dataflow.tex` を模範とする。全矢印が直角ルーティング（フォーク形状）で配線され、斜め直線なし、分岐点にジャンクションドット、折れ曲がりと矢頭の間に十分な幹がある状態が基準。**チェックは図全体の全矢印に対して行う（修正箇所だけでなく既存の矢印も含む）。**

#### T: TikZ 図の品質

| ID | チェック内容 | 過去の発生例 |
|----|------------|-------------|
| T1 | **枠線とテキストの干渉:** ノードの枠線がテキストにかかっていないか。テキストのアセンダ（b, d の上部）やディセンダ（g, y, \_ の下部）が枠線を突き抜けていないか。ノード内テキストが枠からはみ出していないか（パディング不足）。特に `anchor=north west` 等で枠線上に配置したラベルに注意 | fw\_architecture 全層ラベルのディセンダが上辺枠線にかかる、HAL 層 "Buzzer" が右端ギリギリ |
| T2 | **ラベル重なり:** ラベル同士、ラベルと線・ノード・矢印が重なっていないか | gyro\_drift "True angle" vs curve, pid\_block labels vs sum node |
| T3 | **矢印の接続:** 矢印の始点/終点がブロック・加算点・分岐点に接続し、空白から発生/消滅していないか。同じ役割の矢印群の矢頭方向が統一されているか。入出力信号線にはラベル（`r(t)`, `y(t)` 等）があるか | espnow\_dataflow 上段/下段で矢頭方向が不統一 |
| T4 | **ルーティング:** 矢印の配線は水平・垂直のみ（直角ルーティング）。**以下の全てを満たすこと:** | espnow\_dataflow 斜め直線がapi1を貫通 |
|    | (a) **フォーク形状:** 1対多の分岐・配信はフォーク（水平バー + 垂直ドロップ）で描く。点と点を斜め直線で結ばない | |
|    | (b) **ノード干渉禁止:** 矢印が他のノードを貫通・辺に重なっていないか。直角ルーティングならノード間のギャップを通せば構造的に排除できる | |
|    | (c) **矢頭前の幹:** 折れ曲がりと矢頭（鏃）の間に最低 5mm の直線部分（幹）が必要。折れ曲がり直後に矢頭を置かない | |
|    | (d) **微小セグメント禁止:** 不自然に短い水平/垂直セグメントがないか。タップ点を接続先の真上/真横に合わせて解消する | sysid\_concept 微小左ジョグ |
|    | (e) **交差禁止:** 矢印が不自然に交差していないか | |
|    | (f) **フィードバック経路:** ノード下のテキストを貫通しない。終点は辺の中点（`.west`, `.south` 等）に接続し、角（`.south west` 等）に接続しない | build\_flash\_flow feedback |
| T5 | **分岐点のジャンクションドット:** 信号線が分岐・合流する箇所に `\fill circle` で黒丸を配置しているか | sysid\_concept output tap |
| T6 | **信号フローの正しさ:** ブロック図のフィードバック分岐点・加算点が制御工学の慣例に従っているか | feedback\_block tap point |
| T7 | **アノテーション位置:** 矢印・ラベルが指す先が実際のデータ点と一致しているか | step\_response overshoot arrow |
| T8 | **数式・物理量の正確性:** 伝達関数・単位・パラメータが技術的に正しいか。同一スライド内のブロック図と数式の整合性を照合すること | step\_response 2nd-order TF, pid\_block D項に $K_p$ 欠落 |
| T9 | **色定義:** `\definecolor` は `\begin{document}` の後に記述（standalone パッケージの互換性） | 全 TikZ ファイル |

**座標計算の注意:** `positioning` ライブラリの `below=Xmm` は**端-端間ギャップ**であり中心間距離ではない。`below=22mm` のノードの中心は `parent.south - 22mm - half_height` の位置になる。

#### T 項目の修正方針

1. **枠線・テキスト干渉（T1）:** ラベルを `yshift` で内側に移動 / `inner sep` を増やす / テキストを中央揃え (`anchor=center`) に変更 / ボックスの `minimum width` / `minimum height` を拡大
2. **ラベル重なり（T2）:** `above`/`below`/`left`/`right` の位置変更、または `xshift`/`yshift` で微調整
3. **矢印のルーティング（T4）:** フォーク形状（水平バー + 垂直ドロップ）で再配線する。模範: `espnow_dataflow.tex` — 幹線から水平バーを伸ばし、各端点から垂直にドロップして `.north` に接続。中間ノードがある場合はバーをノード群の上/下に配置し、垂直線をギャップに通す。フィードバック経路は終点を `.west`/`.south` に変更し、テキスト外側を迂回。分岐点には `\fill circle (2.5pt)` でジャンクションドットを追加

---

### Stage 2: スライド/資料チェック

TikZ 図を埋め込んだスライドや資料のレイアウト・構造を検証する。

#### BL: Beamer レイアウト

| ID | チェック内容 | 過去の発生例 |
|----|------------|-------------|
| BL1 | テキストがスライド下端・フッターにはみ出していないか（vbox overflow） | L0 P4/P10, L7 caption |
| BL2 | `block` / `alertblock` 内のテキストが長すぎて折り返しが不自然でないか | L2 goal text |
| BL3 | テーブルのカラム幅が適切か（`p{Nmm}` で不要な折り返しが発生していないか） | L0 P2 テーマ column |
| BL4 | コードリスティングがスライド1枚に収まるか（行数・フォントサイズ） | L7 code listing 20→14行 |
| BL5 | `\resizebox` や `columns` で図とテキストが重ならないか | L9 P4 top-bottom→side-by-side |
| BL6 | `\vspace` で要素間の余白が適切か（詰まりすぎ・空きすぎ） | L0 P10 exampleblock |
| BL7 | **ヘッダー拡張:** フレームタイトル直後に裸の `tabular` / `{\small ...}` を置いていないか。`\framesubtitle` を削除した場合、`\vspace{0.3em}` 等でヘッダー/ボディ境界を明示すること。ヘッダーバーがスライド高の 15% を超えていたら異常 | L10 API テーブルがヘッダーに吸収（framesubtitle 削除後） |
| BL8 | **コードブロック下端:** `sflisting` や `lstlisting` のコードブロック（および直後の `exampleblock` 等）がフッターバーに接触・重なっていないか。コードブロック下に最低 0.3em 以上の余白があること | L02 P40, L03 P48, L04 P54, L05 P62, L07 P85, L08 P95 |
| BL9 | **TikZ タイトル重複:** Beamer 埋め込み用の TikZ 図内にタイトルを持たないこと（Beamer の frame title が担当）。standalone で直接使う図にはタイトルがあってよい | feedback\_block, mixer\_matrix, imu\_axes, pid\_block |
| BL10 | **図のスケーリング:** `\resizebox{w}{h}` で幅と高さの両方指定は禁止（アスペクト比が歪む）→ `adjustbox{max width=..., max height=...}` を使う。図がスライドの半分以下しか占めない場合は TikZ 側の座標を調整してアスペクト比をスライド（4:3）に近づける | gyro\_drift, espnow\_dataflow フォント縦伸び |

#### ML: Marp レイアウト

| ID | チェック内容 | 過去の発生例 |
|----|------------|-------------|
| ML1 | テキストがスライド下端に溢れていないか（Marp は自動縮小しないため手動で分割が必要） | — |
| ML2 | 画像がスライド領域に収まっているか。`![center]` や `![bg right:N%]` のサイズ指定が適切か | — |
| ML3 | テーブルが横幅に収まっているか（カラム数が多い場合 `font-size` の CSS 調整が必要） | — |
| ML4 | コードブロック（` ``` `）がスライド1枚に収まるか（行数過多で下端切れ） | — |
| ML5 | `---` によるスライド区切りが正しい位置にあるか（意図しない分割・結合がないか） | — |

#### BS: Beamer スライド構造

| ID | チェック内容 |
|----|------------|
| BS1 | 各レッスンが標準構造に従っているか: タイトル → ゴール → 図/概念 → API → 実習コード → チェックポイント |
| BS2 | 「次のレッスン」ブロックが正しいレッスン番号・タイトルを参照しているか |
| BS3 | `\lesson{N}{日本語タイトル}{英語タイトル}` のフォーマットが正しいか |

#### BL 項目の修正方針

1. **レイアウト溢れ:** テキスト短縮 → フォントサイズ縮小（`\footnotesize`, `\scriptsize`）→ `columns` レイアウト変更の順で対応
2. **ヘッダー拡張防止:** `\framesubtitle` 削除時はタイトル直後に `\vspace{0.3em}` を挿入。裸のテーブルや `{\small ...}` をタイトル直後に置かない
3. **コードブロック下端の余白確保:** `\scriptsize` → 空行削除 → コード圧縮の順で対応
4. **図のスケーリング:** `\resizebox{w}{h}` 両方指定禁止 → `adjustbox{max width, max height}` を使う。TikZ 側の座標調整で対処し、Beamer 側のスケーリングだけに頼らない

## Project Overview

StampFly Ecosystem is an educational/research platform for drone control engineering. It covers the complete workflow: **design → implementation → experimentation → analysis → education**.

**Current Status:** Vehicle firmware supports ACRO/STABILIZE/ALT_HOLD/POS_HOLD, with POS_HOLD validated on real hardware (±6-7cm hold accuracy). Controller firmware is implemented and buildable.

**保留案件: プラットフォームレイヤー分離リファクタリング**
- ブランチ `refactor/platform-layer` で Phase 1〜3 まで進めていたが、mainの大幅改修と衝突するため一時保留（2026-04-07）
- mainが一区切りついた時点で、最新mainから再開予定
- 詳細は auto-memory の `project_platform_refactor_suspended.md` を参照

## Architecture

The project uses a **responsibility-based directory structure**:

```
stampfly-ecosystem/
├── docs/              # Human-readable documentation
├── firmware/
│   ├── vehicle/       # Vehicle firmware (primary, promoted from vehicle_new)
│   ├── vehicle_old/   # Legacy vehicle firmware (frozen, 87 real flights — see below)
│   ├── controller/    # Transmitter firmware
│   └── common/        # Shared embedded code (ESP-NOW protocol structs); used by
│                      # controller + vehicle_old + vehicle (protocol only — see below)
├── protocol/          # Communication spec - Single Source of Truth (SSOT)
│   ├── spec/          # Machine-readable protocol definition (YAML/proto)
│   ├── generated/     # Auto-generated code from spec
│   └── tools/         # Validation and code generation
├── control/           # Control systems design (models, PID, MPC, SILS)
├── analysis/          # Data analysis (notebooks, scripts, datasets)
├── tools/             # Utilities (flashing, calibration, log capture, CI)
├── simulator/         # SILS/HILS testing environments
├── examples/          # Minimal working examples for learning
└── third_party/       # External dependencies
```

### Key Design Principles

1. **Protocol as Foundation**: All communication implementations derive from `protocol/spec/`. This is the Single Source of Truth. The core ESP-NOW `ControlPacket`/`PairingPacket` structs are implemented once in `firmware/common/protocol/include/espnow_protocol.hpp` and shared by `firmware/vehicle`, `firmware/vehicle_old`, and `firmware/controller`.
2. **Responsibility Separation**: Each directory has a clear role. Don't mix concerns across boundaries.
3. **Educational Focus**: Code quality and documentation matter as much as functionality. This is built for students and researchers.

### Firmware Structure (ESP-IDF)

`firmware/vehicle/`（主力・旧 vehicle_new）は次世代アーキテクチャで書かれている: フラットな `sf_<name>` コンポーネント命名（`sf_algo_*`/`sf_svc_*` の層分けなし）、Pub-Sub トピック経由の疎結合、4階層アクセス制御。**実装・修正を行う場合は、作業開始前に必ず以下の6文書を読むこと:**

1. `firmware/vehicle/docs/requirements.md` — 要件定義書
2. `firmware/vehicle/docs/architecture.md` — アーキテクチャ設計書（v3: 4階層アクセス + 横断ルール R1〜R16 + BSP 層）
3. `firmware/vehicle/docs/detailed_design.md` — 詳細設計書
4. `firmware/vehicle/docs/coding_and_education.md` — **コーディング方針・教育計画（必読）**
5. `firmware/vehicle/docs/development_roadmap.md` — **開発ロードマップ・SILS→実機ワークフロー（必読）** — 3原則（Code/Param/Model Identity）、ACROレート制御を起点とする層別プラント同定戦略、Phase 0〜6 の合格基準
6. `firmware/vehicle/docs/hardware_init.md` — **BSP・ハードウェア初期化設計** — sf_board 責務、起動シーケンス、Critical/Optional/Recoverable 分類、HAL 接続規約、namespace 規約（sf::api / sf::internal）

**特に重要な原則:**
- コードは**ドローンファームを作ろうとする人が参考にできる模範的なソースコード**であること
- **可読性と簡潔さ**を兼ね備えること（1関数50行以内、ネスト2段まで、略語禁止）
- **バイリンガルコメント**（英語→日本語）を全関数・全ブロックに記載
- コンポーネント間は**Pub-Subトピック経由**で通信（直接呼び出し禁止）
- **マジックナンバー禁止** — 全数値にconfig定数名またはパラメータ名
- **@designタグ必須** — クラス・インターフェース・状態遷移に設計文書の参照と判定ステータス `[OK]`/`[NG]`/`[--]` を記載。リリース時は全て `[OK]` であること
- **設計矛盾は即時報告** — 実装中に設計文書との矛盾・不都合を発見したら実装を止めて報告・議論する
- **アーキテクチャ不変条件（INV）への照合を必須とする（場当たりパッチ再発防止, 2026-06-14）** — 制御則・状態機械・離着陸/飛行フェーズに関わる変更は、コミット前に `architecture.md` の「アーキテクチャ不変条件（INV）」節に照合すること。**新機能の追加・要件変更で、ある機能の前提が変わるときは、その前提を埋め込んでいる既存コンポーネントを必ず列挙し（リップル確認）、古い前提のまま並列経路・独自実装が残っていないか確認する。** 「最小変更で動かし SILS を通す」だけで満足しない（SILS が通っても INV 違反は退行）。機能追加時は常に**あるべき姿（INV準拠の統一構造）**で実装し、既存の局所形に引きずられて並列パッチを足さない。
- Exampleは**単独ビルド可能**、**コメントは本体より多くてもいい**

`firmware/vehicle_old/` は旧世代の実装（`sf_hal_*`/`sf_algo_*`/`sf_svc_*` の層分け命名、実飛行87回）で、**凍結されたレガシー**。新規開発は行わず、`firmware/common/` を controller と共有する。sf CLI・SILS回帰から `vehicle_old` として引き続きビルド・テスト可能（`sf build vehicle_old`、`sf sils scenario --target vehicle_old`）。

## Build System

ESP-IDF for embedded firmware (ESP32 target)。**sf CLI を優先して使用する:**
```bash
# 推奨: sf CLI を使用
source setup_env.sh
sf build vehicle
sf flash vehicle -m

# レガシーファームのビルド
sf build vehicle_old

# 代替: idf.py を直接使用（sf で問題がある場合のみ）
cd firmware/vehicle
idf.py build
idf.py flash monitor
```

## Implementation Priority

When developing this codebase, follow this order:
1. **protocol/spec/** - Define communication specification first
2. **firmware/common/protocol/** - Shared ESP-NOW protocol structs (encode/decode)
3. **firmware/vehicle/** - Basic task structure and sensor integration
4. **examples/** - Minimal working demonstrations
5. **tools/** - Development utilities
6. **control/** and **analysis/** - Design and analysis tooling

## Language Notes

- **Firmware**: C/C++ (ESP-IDF framework)
- **Analysis/Tools**: Python (Jupyter notebooks, scripts)
- **Protocol Spec**: YAML or Protocol Buffers

## 非公開文書の扱い

学会原稿・申請書（科研費等）・個人の研究計画は、この公開リポジトリには置かない（履歴にも残さない、2026-09-05 に履歴から除去済み）。それらは手元の別の非公開リポジトリで管理する。この公開リポジトリ側から非公開リポジトリへのリンクやパスを書かない。クラウド版セッションは作業ブランチを origin に push するため、非公開文書の作業をこのリポジトリで開いてはならない。`.gitignore` の `docs/_private/`・`papers/`・`grants/` は置き間違え防止用である。手元では `git config core.hooksPath .githooks` を一度実行すると、`.githooks/pre-commit` がこれらのパスや申請書関連の語を含むコミットを拒否する。

## Reference

All architectural decisions are documented in `PROJECT_PLAN.md`. Consult this document before making structural changes.
シミュレーション方針（3層構造・Model Fidelity 期の SILS 忠実度目標・改修バックログ）は `docs/architecture/simulation-policy.md` を正とする。
