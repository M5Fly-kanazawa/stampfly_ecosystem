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

各セッションは「地図（今どこにいるか）→ このセッションで伝えること → デモ → 理論とコードの対応表 → 復習パス → チェックポイント」という共通の進め方で進む。デモは「見るだけ／一緒に打つ／帰宅後に再現」の3段階を示すので、環境構築が間に合わなくても最後まで内容を追える。

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

コマンドライン（CLI）で導入する。GUI 版インストーラ「StampFly Setup」もあるが、動作の安定性を確認できていないため本チュートリアルでは使わない。手順はリポジトリ直下の README「インストール」節と同じ 3 段階で、コードブロックは 1 つずつ貼り付け、実行が終わるのを待ってから次へ進む。ESP-IDF のダウンロードを含むので、前日までの実施を勧める。

| 段階 | 内容 |
|------|------|
| ① 前提ツール | Git と Python 3.12 を入れる |
| ② 取得と導入 | リポジトリを取得してインストーラを実行する。途中で ESP-IDF の導入を尋ねられたら 1（Install ESP-IDF v5.5.2）を選ぶ |
| ③ 有効化と診断 | 開発環境を有効化し、`sf doctor` で確認する |

**Windows**（コマンドプロンプト。WSL は不要）

① Git と Python 3.12 を導入する。終わったら CMD を一度閉じて開き直す。

```cmd
winget install Git.Git --accept-source-agreements --accept-package-agreements
winget install Python.Python.3.12 --accept-source-agreements --accept-package-agreements
```

② リポジトリを取得してインストーラを実行する。

```cmd
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
install.bat
```

③ 開発環境を有効化して診断する。新しい CMD を開くたびに `setup_env.bat` を実行する。

```cmd
setup_env.bat
sf doctor
```

**macOS / Linux** も同じ 3 段階で、② は `./install.sh`、③ は `source setup_env.sh`。① の前提ツール（Homebrew／apt のコマンド）はリポジトリ直下の README を参照。

VSCode 拡張機能 `alexnesnes.teleplot` も入れておく（Teleplot でセンサ波形をリアルタイム表示するために使う）。

### 機体・コントローラの準備

| 手順 | 内容 |
|------|------|
| 1 | 開発環境で機体ファームをビルドして書き込む（当日の実習 1 と同じ手順）: `sf build vehicle` → `sf flash vehicle -m`。書き込み直後に起動音が鳴り、LED が白から緑の常灯になり、モニタに起動ログが流れれば成功 |
| 2 | コントローラのペアリングを行う（初回のみ手動: コントローラは LCD パネルボタンを押しながら電源投入，StampFly は本体ボタンを 3 秒以上押し続けてビープで離す。5 秒以上押し続けるとシステムリセットになる。続けてコントローラ画面の一覧から自分の機体（MAC下4桁。機体の USB CLI `mac` コマンドで確認できる）を選んでボタンで確定するとペアリング完了。以降は電源投入だけで自動再接続する。手順は [送信機の使い方](../../guides/controller.md) を参照）。自分で購入したコントローラは先に `sf flash controller` で書き直す |
| 3 | 機体を机の上に置き、モータ回転中は手を近づけない状態でコントローラから ARM（右スティック押し込み）し、モータが応答することを確認する。異常時は即 DISARM |

WiFi モードとチャンネルの設定は、当日の実習 1 (2/2) で講師の指定するチャンネルとあわせて行う。

### シミュレータの試走

開発環境が動いているかの確認として、VPython シミュレータを一度起動しておく。

```bash
sf sim run vpython
```

ブラウザに 3D 表示が出れば開発環境は動いている。実習 1〜9 は機体とコントローラが前提で、シミュレータは実機の代わりにはならない。実機を持たない場合は、当日は「見るだけ」で参加してほしい。

### 事前準備チェック

| 確認項目 | 合格の目安 |
|---------|-----------|
| `sf doctor` | エラーなしで完了する |
| 機体の起動 | 緑点灯＋起動音まで到達する |
| `sf flash vehicle -m` | 起動音・LED 緑・モニタに起動ログ |
| `sf sim run vpython` | ブラウザに3D表示が出る |

うまくいかない場合は [トラブルシューティング](../../guides/troubleshooting.md) を参照。それでも解決しない場合は、当日は「見るだけ」で参加し、資料と録画で後日復習してほしい。

---

<a id="english"></a>

## 1. Overview

### About This Document

This document is the material index and pre-tutorial checklist for the SCI/SICE Tutorial 2026, "Hands-On Drone Education with the StampFly Ecosystem: From Coding to Flight Testing and Data Acquisition."

### Event Information

| Item | Detail |
|------|--------|
| Date | Thursday, September 10, 2026 |
| Venue | Osaka University Nakanoshima Center + Zoom (on-demand viewing available afterward) |
| Instructor | Kouhei Ito (Kanazawa Institute of Technology) |
| Audience | Control-engineering researchers and educators (about 15 on-site, about 15 online) |
| Bring | A laptop, and a StampFly with controller (for hands-on participation) |

### Target Audience

- Tutorial participants (see §3 for pre-tutorial preparation)
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

Every session follows the same flow: map (where we are) -> what this session says -> demo -> theory-to-code map -> review path -> checkpoint. Each demo is shown at three levels of engagement (watch only / follow along / reproduce at home), so falling behind on setup does not mean falling behind on content.

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

## 4. Pre-Tutorial Preparation

If you plan to join the hands-on parts, complete the following **before** the day. Venue WiFi and laptop conditions vary, so finishing the day before is recommended.

### Development environment

Install from the command line (CLI). A GUI installer, "StampFly Setup", exists, but its stability has not been confirmed, so this tutorial does not use it. The procedure is the same three stages as the "Installation" section of the repository's top-level README: paste one code block at a time and wait for it to finish before the next. It downloads ESP-IDF, so do it the day before at the latest.

| Stage | Detail |
|-------|--------|
| 1. Prerequisites | Install Git and Python 3.12 |
| 2. Get and install | Clone the repository and run the installer. When it asks about ESP-IDF, choose 1 (Install ESP-IDF v5.5.2) |
| 3. Activate and check | Activate the environment and run `sf doctor` |

**Windows** (Command Prompt; no WSL needed)

Stage 1: install Git and Python 3.12, then close and reopen CMD.

```cmd
winget install Git.Git --accept-source-agreements --accept-package-agreements
winget install Python.Python.3.12 --accept-source-agreements --accept-package-agreements
```

Stage 2: clone the repository and run the installer.

```cmd
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
install.bat
```

Stage 3: activate and check. Run `setup_env.bat` in every new CMD window.

```cmd
setup_env.bat
sf doctor
```

**macOS / Linux** follow the same three stages with `./install.sh` in stage 2 and `source setup_env.sh` in stage 3; the stage-1 prerequisites (Homebrew / apt commands) are in the repository's top-level README.

Also install the VSCode extension `alexnesnes.teleplot` (used to graph sensor data live via Teleplot).

### Vehicle and controller

| Step | Detail |
|------|--------|
| 1 | Build and flash the vehicle firmware from the dev environment (the same steps as Exercise 1 on the day): `sf build vehicle` then `sf flash vehicle -m`. Success looks like the boot chime, the LED going white then steady green, and the boot log in the monitor |
| 2 | Pair the controller (first time only, manual: power on the controller while holding its LCD panel button, then hold the StampFly's body button for 3 s or more and release at the beep; holding 5 s or more triggers a system reset. Then pick your own vehicle — last 4 hex digits of its MAC, checked with the vehicle's USB CLI `mac` command — from the controller's on-screen list and confirm to complete pairing. After that, power-up alone reconnects automatically. See [Using the Transmitter](../../guides/controller.md)). A controller you bought yourself must first be reflashed with `sf flash controller` |
| 3 | Place the vehicle on a table, keep hands clear of the spinning motors, and arm from the controller (push the right stick) to confirm the motors respond. DISARM immediately if anything looks wrong |

The WiFi mode and channel are set on the day in Exercise 1 (2/2), together with the channel the instructor assigns you.

### Simulator dry run

As a check that the dev environment works, launch the VPython simulator once.

```bash
sf sim run vpython
```

A 3D view opening in your browser means the environment works. Exercises 1-9 require the vehicle and controller; the simulator is not a substitute for the hardware. If you do not have a vehicle, plan to join the hands-on parts in "watch only" mode.

### Pre-tutorial checklist

| Check | Pass criterion |
|-------|-----------------|
| `sf doctor` | Completes with no errors |
| Vehicle boot | Reaches steady green LED with the boot chime |
| `sf flash vehicle -m` | Boot chime, green LED, boot log in the monitor |
| `sf sim run vpython` | A 3D view opens in the browser |

If something does not work, see [Troubleshooting](../../guides/troubleshooting.md). Otherwise, plan to join the hands-on parts in "watch only" mode on the day, and catch up afterward using the recording and materials.
