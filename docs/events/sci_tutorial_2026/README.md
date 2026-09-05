# SCI/SICE チュートリアル講座 2026

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

システム制御情報学会・計測自動制御学会 チュートリアル講座 2026「制御教育教材 StampFly Ecosystem の紹介 ～コーディングから飛行試験、データ取得まで～」の資料索引と参加者向け事前準備をまとめる。

### 開催情報

| 項目 | 内容 |
|------|------|
| 日時 | 2026年9月10日（木） |
| 会場 | 大阪大学中之島センター + Zoom（後日オンデマンド視聴あり） |
| 講師 | 伊藤 恒平（金沢工業大学） |
| 対象 | 制御工学の研究者・教育者（対面15名程度・オンライン15名程度） |
| 持ち物 | ノートPC、StampFly実機とコントローラ（実習に参加する場合） |

### 対象読者

- チュートリアル参加者（事前準備は本文 §3 を参照）
- 当日資料を復習・再現したい方
- 講義スライドや持ち帰り資料の構成を確認したい関係者

## 2. タイムテーブル

| セッション | 時間 | 内容 |
|-----------|------|------|
| S1 | 10:05 -- 11:00 | StampFly Ecosystem の全体像と設計思想 |
| S2 | 11:00 -- 12:00 | 開発環境のセットアップとセンサデータの取得 |
| （昼休み） | 12:00 -- 13:00 | |
| S3 | 13:00 -- 14:00 | モータ制御とコントローラ入力の実装 |
| S4 | 14:00 -- 15:00 | フィードバック制御の基礎 --- PID による姿勢安定化 |
| （休憩） | 15:00 -- 15:30 | |
| S5 | 15:30 -- 16:30 | シミュレータ・解析ツールの活用と発展的テーマの紹介と質疑 |

各セッションは「地図（今どこにいるか）→ 要点3つ → デモ → 理論とコードの対応表 → 復習パス → チェックポイント」という共通の型で進む。デモは「見るだけ／一緒に打つ／帰宅後に再現」の3段階を示すので、環境構築が間に合わなくても最後まで内容を追える。

## 3. 資料索引

| 資料 | 場所 |
|------|------|
| スライド本編（PDF） | `docs/events/sci_tutorial_2026/slides/sci_tutorial.pdf`（`make sci` でビルド） |
| スライド（Docswell版・QRなし） | `docs/events/sci_tutorial_2026/slides/sci_tutorial_docswell.pdf`（`make sci-docswell` でビルド） |
| 告知チラシ | `docs/events/sci_tutorial_2026/チュートリアル講座2026広告最終案.pdf` |
| 復習手順ガイド | [`handson_guide.md`](handson_guide.md) |
| コマンド・API 早見表 | [`cheatsheet.md`](cheatsheet.md) |
| 実機検証チェックリスト（講師用） | [`verification_checklist.md`](verification_checklist.md) |
| 実習・デモの期待結果（SILS で生成したグラフ・動画・判定ログ） | [`fallback/`](fallback/README.md) |
| スライド付録（持ち帰り資料） | スライド本編の末尾「付録」章（チートシート・トラブルシューティング・復習パス） |

## 4. 参加者向けの事前準備

実習に参加する場合、以下を**事前に**済ませておくと当日スムーズに進められる。会場のWiFiやPC環境によっては時間がかかることがあるため、前日までの実施を推奨する。

### 開発環境のセットアップ

ターミナル操作に不慣れな場合は GUI インストーラ「StampFly Setup」を使う。

| 手順 | 内容 |
|------|------|
| 1 | [GUI インストーラガイド](../../guides/gui-installer.md) からダウンロードし、ウィザードに従って導入する |
| 2 | 導入後、`sf doctor` を実行して環境を診断する（問題があれば表示に従って解消する） |
| 3 | ターミナル操作に慣れている場合は CLI インストーラ（`install.sh`/`install.bat`）でもよい |
| 4 | VSCode 拡張機能 `alexnesnes.teleplot` を入れておく（Teleplot でセンサ波形をリアルタイム表示するために使う） |

### 機体・コントローラの準備

| 手順 | 内容 |
|------|------|
| 1 | [Webフラッシャ](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) から機体・コントローラのファームウェアを書き込む |
| 2 | コントローラのペアリングを行う（初回のみ手動: コントローラは LCD パネルボタンを押しながら電源投入，StampFly は本体ボタンを2秒長押し。双方がビープしたら完了。以降は電源投入だけで自動再接続する。手順は `docs/getting-started.md` §5 を参照） |
| 3 | プロペラを外した状態でコントローラから ARM（右スティック押し込み）し、モータが応答することを確認する |

### シミュレータの試走

実機を持ち込まない、または実機なしで復習したい場合は VPython シミュレータで代替できる。

```bash
sf sim run vpython
```

ブラウザに3D表示が出れば準備完了。詳細は `simulator/README.md` を参照。

### 事前準備チェック

| 確認項目 | 合格の目安 |
|---------|-----------|
| `sf doctor` | エラーなしで完了する |
| 機体の起動 | 緑点灯＋起動音まで到達する |
| Webフラッシャでの書き込み | 書き込み完了メッセージが出る |
| `sf sim run vpython` | ブラウザに3D表示が出る |

うまくいかない場合は [トラブルシューティング](../../guides/troubleshooting.md) を参照。それでも解決しない場合は、当日は「見るだけ」で参加し、資料と録画で後日復習してほしい。

---

<a id="english"></a>

## 1. Overview

### About This Document

This document is the material index and pre-workshop checklist for the SCI/SICE Tutorial 2026, "Hands-On Drone Education with the StampFly Ecosystem: From Coding to Flight Testing and Data Acquisition."

### Event Information

| Item | Detail |
|------|--------|
| Date | Thursday, September 10, 2026 |
| Venue | Osaka University Nakanoshima Center + Zoom (on-demand viewing available afterward) |
| Instructor | Kouhei Ito (Kanazawa Institute of Technology) |
| Audience | Control-engineering researchers and educators (about 15 on-site, about 15 online) |
| Bring | A laptop, and a StampFly with controller (for hands-on participation) |

### Target Audience

- Tutorial participants (see §3 for pre-workshop preparation)
- Anyone reviewing or reproducing the day's material afterward
- Organizers checking the structure of the slides and take-home material

## 2. Timetable

| Session | Time | Content |
|---------|------|---------|
| S1 | 10:05 -- 11:00 | Overview and design philosophy of the StampFly Ecosystem |
| S2 | 11:00 -- 12:00 | Environment setup and sensor data acquisition |
| (Lunch) | 12:00 -- 13:00 | |
| S3 | 13:00 -- 14:00 | Motor control and controller input implementation |
| S4 | 14:00 -- 15:00 | Feedback control basics --- PID attitude stabilization |
| (Break) | 15:00 -- 15:30 | |
| S5 | 15:30 -- 16:30 | Simulator and analysis tools, advanced topics, and Q&A |

Every session follows the same shape: map (where we are) -> three key points -> demo -> theory-to-code map -> review path -> checkpoint. Each demo is shown at three levels of engagement (watch only / follow along / reproduce at home), so falling behind on setup does not mean falling behind on content.

## 3. Material Index

| Material | Location |
|----------|----------|
| Main slide deck (PDF) | `docs/events/sci_tutorial_2026/slides/sci_tutorial.pdf` (build with `make sci`) |
| Slides (Docswell variant, no QR codes) | `docs/events/sci_tutorial_2026/slides/sci_tutorial_docswell.pdf` (build with `make sci-docswell`) |
| Announcement flyer | `docs/events/sci_tutorial_2026/チュートリアル講座2026広告最終案.pdf` |
| Review-session guide | [`handson_guide.md`](handson_guide.md) |
| Command / API cheat sheet | [`cheatsheet.md`](cheatsheet.md) |
| Hardware verification checklist (instructor) | [`verification_checklist.md`](verification_checklist.md) |
| Exercise / demo expected results (SILS-generated plots, videos, verdict logs) | [`fallback/`](fallback/README.md) |
| Slide appendix (take-home material) | The "Appendix" chapter at the end of the main deck (cheat sheets, troubleshooting, review paths) |

## 4. Pre-Workshop Preparation

If you plan to join the hands-on parts, complete the following **before** the day. Venue WiFi and laptop conditions vary, so finishing the day before is recommended.

### Development environment

If you are not comfortable with a terminal, use the "StampFly Setup" GUI installer.

| Step | Detail |
|------|--------|
| 1 | Download from the [GUI installer guide](../../guides/gui-installer.md) and follow the wizard |
| 2 | After install, run `sf doctor` to diagnose the environment (follow its output to resolve any issue) |
| 3 | If you are comfortable with a terminal, the CLI installer (`install.sh`/`install.bat`) works too |
| 4 | Install the VSCode extension `alexnesnes.teleplot` (used to graph sensor data live via Teleplot) |

### Vehicle and controller

| Step | Detail |
|------|--------|
| 1 | Flash the vehicle and controller firmware from the [Web Flasher](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) |
| 2 | Pair the controller (first time only, manual: power on the controller while holding its LCD panel button, then hold the StampFly's body button for 2 seconds; both beep when paired. After that, power-up alone reconnects automatically. See `docs/getting-started.md` §5) |
| 3 | With the propellers removed, arm from the controller (push the right stick) and confirm the motors respond |

### Simulator dry run

If you are not bringing hardware, or want to review without it, the VPython simulator is a substitute.

```bash
sf sim run vpython
```

You are ready once a 3D view opens in your browser. See `simulator/README.md` for details.

### Pre-workshop checklist

| Check | Pass criterion |
|-------|-----------------|
| `sf doctor` | Completes with no errors |
| Vehicle boot | Reaches steady green LED with the boot chime |
| Web Flasher | Shows a flash-complete message |
| `sf sim run vpython` | A 3D view opens in the browser |

If something does not work, see [Troubleshooting](../../guides/troubleshooting.md). Otherwise, plan to join the hands-on parts in "watch only" mode on the day, and catch up afterward using the recording and materials.
