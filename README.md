# StampFly Ecosystem

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 自分の手で、ドローンの飛行制御を作りたいあなたへ

**StampFly Ecosystem** は、ドローン制御を**学び、実装し、実験する**ための
教育・研究プラットフォームです。

「PID制御を教科書で学んだけど、実際に動くものを作りたい」
「姿勢推定アルゴリズムを自分で実装して試したい」
「研究用の飛行実験プラットフォームが欲しい」

そんなあなたのために、このエコシステムは存在します。

---

## 何ができるのか？

| できること | 内容 |
|-----------|------|
| **シミュレータで練習** | 実機がなくても、PC 上の 3D シミュレータで送信機を使った操縦を体験できる |
| **実機での飛行体験** | ファームウェアを書き込んだ StampFly を送信機で飛ばす。4 つの飛行モード（ACRO／STABILIZE／高度維持／位置保持）を搭載 |
| **センサー値の取得** | IMU・気圧・ToF・オプティカルフローの値をリアルタイムに読み出し、表示・記録できる |
| **コントローラからの指令の受信** | 送信機のスティック・ボタンの値を機体側で受け取り、自分のプログラムから使える |
| **独自の飛行プログラムの作成** | `sf app new` で自分のプロジェクトを作り、制御則や推定器を自分で書ける（**→ [独自プログラム開発入門](docs/guides/custom_program.md)**） |
| **飛行プログラムの SILS での検証** | 作成した飛行プログラムを SILS（Software In the Loop Simulation: ファームウェアそのものを PC 上で飛ばす試験）で、実機に書き込む前に確認できる |

---

## 📦 インストール

**コマンドライン（CLI）で導入してください。** GUI 版インストーラ「StampFly Setup」もありますが、
まだ動作の安定性を確認できていないため、以下の CLI 手順で進めてください。
GUI 版の説明は **[GUI インストーラガイド](docs/guides/gui-installer.md)** にあります。

手順はどの OS でも同じ 3 段階です。コードブロックは 1 つずつコピーして端末に貼り付け、
実行が終わるのを待ってから次のブロックに進んでください。

| 段階 | 内容 |
|------|------|
| ① 前提ツール | Git を入れる（Python は不要 — 専用の Python 3.12 と ESP-IDF v5.5.2 がインストーラによって自動導入される） |
| ② 取得と導入 | リポジトリを取得し、インストーラを実行する |
| ③ 有効化と診断 | 開発環境を有効化し、`sf doctor` で確認する |

専用の Python・ESP-IDF・ツール一式は `SF_HOME`（既定は Windows が `C:\StampFly`、
macOS/Linux が `~/.stampfly`）に自己完結導入され、容量は合計で約 4〜6 GB です。
ダウンロードを含むため、時間とディスクに余裕のあるときに実行してください。
既存の ESP-IDF・システム Python をそのまま使いたい開発者向けの**旧来モード**
（`install.bat --use-existing-idf` 等）もあります。詳細は
**[セットアップガイド](docs/setup/README.md)** を参照してください。

### Windows

コマンドプロンプト（CMD）で実行します。WSL は不要です。

**① Git を導入**（すでにあれば省略）。末尾の 2 つのオプションは、初回に出る利用規約への同意の質問を省くためのものです。
終わったら CMD を一度閉じて開き直します。

```cmd
winget install Git.Git --accept-source-agreements --accept-package-agreements
```

**② リポジトリを取得し、インストーラを実行**。途中の質問には画面の指示に従って答えます。

```cmd
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
install.bat
```

**③ 開発環境を有効化して診断**。新しい CMD を開くたびに `setup_env.bat` を実行します。

```cmd
setup_env.bat
sf doctor
```

**→ [Windows セットアップ（詳細・トラブル対応）](docs/setup/windows.md)**

### macOS

ターミナルで実行します。

**① Homebrew を導入**（すでにあれば省略）。Enter を押し、Mac のログインパスワードを入力します。
Xcode Command Line Tools が無い場合はここで一緒に導入されます。
終了時に「Next steps」として表示される、brew を PATH に追加するコマンドをそのままコピーして実行してください。

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

続けて、ビルドに必要なツールを入れます（Python は不要です）。

```bash
brew install cmake ninja dfu-util ccache
```

**② リポジトリを取得し、インストーラを実行**。途中の質問には画面の指示に従って答えます。

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

**③ 開発環境を有効化して診断**。新しいターミナルを開くたびに `source setup_env.sh` を実行します。

```bash
source setup_env.sh
sf doctor
```

**→ [macOS セットアップ（詳細・トラブル対応）](docs/setup/macos.md)**

### Ubuntu

ターミナルで実行します（Ubuntu 22.04 LTS 以降）。

**① Git・ビルドツールを導入**（Python は不要です）。パスワードを求められたら入力します。

```bash
sudo apt update
sudo apt install -y git curl tar cmake ninja-build wget flex bison gperf ccache \
    libffi-dev libssl-dev dfu-util libusb-1.0-0
```

**② リポジトリを取得し、インストーラを実行**。途中の質問には画面の指示に従って答えます。

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

**③ 開発環境を有効化して診断**。新しいターミナルを開くたびに `source setup_env.sh` を実行します。

```bash
source setup_env.sh
sf doctor
```

実機や送信機を USB で使うときは、シリアルポートの権限を一度だけ設定します（反映には再ログインが必要）。

```bash
sudo usermod -a -G dialout $USER
```

**→ [Linux セットアップ（詳細・トラブル対応）](docs/setup/linux.md)**

`sf doctor` が問題なしと表示すれば導入完了です。
後で最新版に更新するときは `sf upgrade` を実行します（**→ [アップグレードガイド](docs/guides/upgrading.md)**）。

---

## 🎮 まずはシミュレータで飛ばしてみよう！

**実機がなくても大丈夫。** 送信機（M5Stack AtomS3 + Atom JoyStick）と PC があれば、PC 上の 3D シミュレータでドローン操縦を体験できます。
シミュレータでは送信機を USB ゲームパッドとして使うので、先にファームウェアを書き込み、通信モードを USB HID に切り替えます。

**① 送信機にファームウェアを書き込む**（初回のみ）。送信機を USB ケーブルで PC につなぎ、開発環境を有効化した端末で実行します。

```bash
sf build controller
sf flash controller
```

**② 送信機を USB HID モードに切り替える**。送信機の画面での操作です。

| 手順 | 操作 |
|------|------|
| 1 | 画面（ボタン）を押してメニューを開く |
| 2 | 右スティックの上下で `Comm: ESP-NOW` の行を選ぶ |
| 3 | 右ボタン（モードボタン）を押すと `Comm: UDP` に変わる。もう一度押すと `Comm: USB HID` になり、送信機が自動で再起動する |
| 4 | 再起動後、画面に `= USB HID MODE =` と表示されれば切替完了。PC にゲームパッドとして認識される |

**③ シミュレータを起動する**。ブラウザが自動で開き、3D ビューが表示されます。スロットルをゆっくり上げると機体が浮き上がります。

```bash
sf sim run vpython
```

スティックの割り当て、地形の切替、うまく動かないときの対処は、シミュレータの使い方ページを参照してください。

**→ [シミュレータの使い方](docs/next_step.md#2-シミュレータの使い方)** | **[送信機の使い方](docs/guides/controller.md)**

---

## 🛸 実際に飛ばしてみよう

シミュレータで操縦に慣れたら、実機を飛ばします。機体（StampFly）へのファームウェア書き込み、送信機を実機操縦用の通信モードに戻す設定、機体と送信機のペアリングまでを行います。

**① 機体にファームウェアを書き込む**。機体を USB ケーブルで PC につなぎ、開発環境を有効化した端末で実行します。

```bash
sf build vehicle
sf flash vehicle
```

**② 送信機を ESP-NOW モードに戻す**。シミュレータ用に USB HID にした通信モードを、実機と通信する ESP-NOW に戻します。

| 手順 | 操作 |
|------|------|
| 1 | 画面（ボタン）を押してメニューを開く |
| 2 | 右スティックの上下で `Comm: USB HID` の行を選ぶ |
| 3 | 右ボタン（モードボタン）を 1 回押すと `Comm: ESP-NOW` に変わり、送信機が自動で再起動する |

**③ 機体と送信機をペアリングする**（初回のみ。以後は電源を入れるだけで自動接続します）。

| 手順 | 操作 |
|------|------|
| 1 | 送信機の電源を切り、画面（ボタン）を押したまま電源を入れる。画面に `Pairing mode...` と表示され、ビープ音が繰り返し鳴る |
| 2 | 機体の電源を入れ、機体のボタンを約 3 秒長押しする。LED が青の速い点滅になり、送信機を探し始める |
| 3 | 送信機のビープ音が止まって飛行画面に切り替わり、機体の青い点滅が止まればペアリング完了 |

ペアリング情報を持たない機体は、電源を入れるだけで自動的に探索を始めます。手順 2 の長押しは、以前の情報を消して確実に探索を始めさせるための操作で、別の送信機と組み替えるときにも使います。

飛行前の確認とスティック操作（アーム・離陸・着陸・飛行モードの切替）は、操縦方法のページを参照してください。

**→ [操縦方法](docs/next_step.md#5-飛行方法)** | **[送信機の使い方](docs/guides/controller.md)**

---

## 🎓 ワークショップで本格的に学ぶ

実機で飛ばせたら、目的に合った講習資料で制御の中身に進みましょう。これまでに実施した 3 つの講習のスライドと手順書を公開しています。

### StampFly 勉強会（4 日間 + 競技会 1 日）

大学生・大学院生向けの標準カリキュラムです。受講者は `setup()` と `loop_400Hz()` の 2 つの関数だけを書き、モータ制御 → コントローラ入力 → IMU → P 制御で初フライト → モデリングとシステム同定 → PID → 姿勢推定 → Python SDK と段階的に進み、最終日に精密着陸競技会を行います。各レッスンの実習コードは `sf lesson switch <レッスン名>` で切り替えます。

| 資料 | 内容 |
|------|------|
| [講師ガイド](docs/events/stampfly_workshop/workshop_guide.md) | Lesson 0〜13 の進め方、安全管理、トラブル対応 |
| [スケジュール](docs/events/stampfly_workshop/workshop_schedule.md) | 4+1 日の時間割と準備物 |
| [競技ルール](docs/events/stampfly_workshop/competition_rules.md) | 最終日の競技会の種目と採点 |
| [スライド（PDF）](docs/events/stampfly_workshop/slides/stampfly_workshop.pdf) | 全レッスン統合スライド |

### DXH 高校教員向け体験講座（2 時間）

プログラミング初心者も含む高校教員向けの体験講座です。操縦体験、開発環境のインストール、プログラムの書き換え、モータの制御の 4 テーマを 120 分で回ります。ビルドせずにブラウザから書き込む手順を使うので、短時間でも実機を動かせます。

| 資料 | 内容 |
|------|------|
| [開催概要](docs/events/dxh2026/README.md) | 対象・機材・文書一覧 |
| [参加者用配布資料](docs/events/dxh2026/handout.md) | Web 書き込みとモータ制御の実習手順 |
| [スライド（PDF）](docs/events/dxh2026/slides/dxh_workshop.pdf) | 進行スライド |

### SCI/SICE チュートリアル講座 2026（1 日）

制御工学の研究者・教育者向けに、エコシステムの全体像から環境構築、センサデータ取得、モータ制御とコントローラ入力、PID による姿勢安定化、シミュレータと解析ツールまでを 5 セッションで扱います。研究用の実験プラットフォームとして使いたい方の入口です。

| 資料 | 内容 |
|------|------|
| [事前準備と資料索引](docs/events/sci_tutorial_2026/README.md) | タイムテーブル、参加者の事前準備 |
| [復習ハンズオンガイド](docs/events/sci_tutorial_2026/handson_guide.md) | 各セッションの実習を自分で再現する手順 |
| [チートシート](docs/events/sci_tutorial_2026/cheatsheet.md) | 当日使うコマンドの一覧 |
| [スライド（PDF）](docs/events/sci_tutorial_2026/slides/sci_tutorial.pdf) | 全 5 セッションのスライド |

**→ [イベント資料の一覧](docs/events/README.md)**

---

## 工場出荷状態に戻す

ワークショップやカスタムファームウェアの書き込みにより、機体・送信機のファームウェアは上書きされます。
工場出荷状態に戻したい場合は、以下のコマンドを実行してください：

機体を工場出荷状態に戻す:

```bash
sf flash vehicle --legacy
```

送信機を工場出荷状態に戻す:

```bash
sf flash controller --legacy
```

> **注意:** 事前に `source setup_env.sh` で開発環境をセットアップしてください。デバイスを USB 接続した状態で実行してください。

---

## 技術仕様

### 機体（StampFly）

| 項目 | 仕様 |
|------|------|
| MCU | ESP32-S3（M5Stamp S3） |
| 質量 | 約 37 g（実測 36.8 g） |
| 寸法 | モータ間距離（対角）65 mm、アーム長（中心→モータ）32.5 mm |
| 慣性モーメント | Ixx 9.16e-6、Iyy 13.3e-6、Izz 20.4e-6 kg·m² |
| モータ | 4 基（X 配置）、PWM 150 kHz 駆動 |
| バッテリー | 1S LiPo（3.7 V）、低電圧警告 3.4 V |

### センサ

| センサ | 型番 | 更新周期 | 用途 |
|--------|------|---------|------|
| IMU（加速度・角速度） | BMI270 | 400 Hz | 姿勢推定の主センサ |
| 地磁気 | BMM150 | 25 Hz | ヨー推定（研究用途） |
| 気圧 | BMP280 | 50 Hz | 高度推定（ToF の測距範囲外） |
| ToF 距離（下面・前面） | VL53L3CX × 2 | 30 Hz | 高度計測、前方の障害物検知 |
| オプティカルフロー | PMW3901 | 100 Hz | 水平位置推定 |
| 電源モニタ | INA3221 | 10 Hz | 電池電圧・電流 |

### 機体ソフトウェア

| 項目 | 仕様 |
|------|------|
| フレームワーク | ESP-IDF v5.5.2 + FreeRTOS |
| 制御周期 | 400 Hz（IMU 同期） |
| 状態推定 | ESKF（Error-State Kalman Filter: 誤差状態カルマンフィルタ）で姿勢・速度・位置を推定 |
| 飛行モード | ACRO（角速度制御）/ STABILIZE（角度制御）/ ALT_HOLD（高度維持）/ POS_HOLD（位置保持） |
| 送信機との通信 | ESP-NOW（TDMA 同期、最大 10 台同時）または UDP（機体の WiFi アクセスポイント経由） |
| PC との通信 | WiFi テレメトリ 50 Hz、高速ログ 400 Hz、Tello SDK 互換 API |
| 設定 | PID ゲイン等のパラメータは実行時に変更し、NVS（不揮発メモリ）に保存 |

### 送信機

| 項目 | 仕様 |
|------|------|
| 本体 | M5Stack AtomS3（ESP32-S3、LCD 付き）+ Atom JoyStick |
| 通信モード | ESP-NOW / UDP / USB HID（PC のゲームパッドとして動作） |
| スティック | 2 軸 × 2 本、Mode 2 / Mode 3 切替、押し込みボタン付き |

### 開発ツール

| 項目 | 仕様 |
|------|------|
| sf CLI | Python 3.10〜3.12。ビルド・書き込み・ログ取得・キャリブレーション・シミュレータ・SILS を一つのコマンド体系で操作 |
| シミュレータ | VPython 版（ブラウザ 3D 表示）、Genesis 版（高精度物理エンジン）、SILS（ファームウェアそのものを PC 上で実行） |
| 対応 OS | Windows / macOS / Ubuntu |

物理パラメータの確定値と出典は [物理パラメータリファレンス](docs/architecture/stampfly-parameters.md)、機体ソフトウェアの要件と設計は [要件定義書](firmware/vehicle/docs/requirements.md) と [アーキテクチャ設計書](firmware/vehicle/docs/architecture.md) を参照してください。

### リポジトリ構成

```
stampfly_ecosystem/
├── docs/           # ドキュメント
├── firmware/       # 組込みファームウェア
│   ├── vehicle/     # 機体ファームウェア（主力）
│   ├── vehicle_old/ # レガシー機体ファームウェア（凍結）
│   ├── controller/  # 送信機ファームウェア
│   └── common/      # 共有コード（ESP-NOW プロトコル構造体）
├── protocol/       # 通信プロトコル仕様（構築中）
├── control/        # 制御設計資産（構築中）
├── analysis/       # 実験データ解析（構築中）
├── tools/          # 補助ツール（構築中）
├── simulator/      # 3Dフライトシミュレータ
├── ros/            # ROS連携（構築中）
├── examples/       # 学習用サンプル
└── third_party/    # 外部依存
```

---

## 🔗 リソースとドキュメント

| リソース | 説明 |
|---------|------|
| [📖 次のステップ](docs/next_step.md) | シミュレータの使い方、飛行前の確認、飛行方法、開発者向け機能 |
| [🎛️ 送信機の使い方](docs/guides/controller.md) | メニュー操作、通信モードの切替、ペアリング、ボタンの役割 |
| [🧪 独自プログラム開発入門](docs/guides/custom_program.md) | 自分の制御則・推定器を書き、SILS で確認して実機で飛ばすまで |
| [⌨️ sf コマンドリファレンス](docs/commands/README.md) | ビルド・書き込み・ログ取得など全コマンドの説明 |
| [🛠️ セットアップガイド](docs/setup/README.md) | OS 別の導入手順の詳細とトラブル対応 |
| [🌐 プロジェクト紹介](https://m5fly-kanazawa.github.io/stampfly_ecosystem/) | 実機3Dモデル付きランディングページ |
| [📚 ドキュメントサイト](https://m5fly-kanazawa.github.io/stampfly_ecosystem/docs/) | 全ドキュメントを検索・閲覧 |
| [🗂️ ドキュメント目録](docs/DOCUMENT_INDEX.md) | リポジトリ内の全ドキュメントの目録 |
| [🔌 Web書き込み](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) | ビルド不要、ブラウザから実機に書き込み |
| [🖥️ StampFly Flasher](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest) | デスクトップ書き込みアプリ（Windows/macOS・Python不要） |
| [📦 ビルド済みファームウェア](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases) | GitHub Releases（vehicle / controller） |
| [📐 物理パラメータリファレンス](docs/architecture/stampfly-parameters.md) | 機体の物理パラメータ（質量・慣性・モータ特性等）の確定値 |
| [stampfly_physical.yaml](control/models/stampfly_physical.yaml) | 物理パラメータの機械可読 SSOT（Single Source of Truth: 唯一の正となる定義） |

---

## ライセンス

MIT License

---

---

<a id="english"></a>

# StampFly Ecosystem

## For those who want to build their own drone control

**StampFly Ecosystem** is an educational and research platform for
**learning, implementing, and experimenting** with drone control.

"I learned PID control from textbooks, but I want to build something that actually flies."
"I want to implement my own attitude estimation algorithm and test it."
"I need a flight experiment platform for my research."

This ecosystem exists for you.

---

## What can you do?

| Capability | Description |
|-----------|-------------|
| **Practice in the simulator** | Fly a 3D simulator on your PC with the transmitter, no real drone needed |
| **Fly the real drone** | Flash the firmware and fly StampFly with the transmitter. Four flight modes included (ACRO / STABILIZE / Altitude Hold / Position Hold) |
| **Read sensor values** | Read IMU, barometer, ToF and optical-flow values in real time, display and record them |
| **Receive transmitter commands** | Receive stick and button values on the vehicle and use them from your own program |
| **Write your own flight program** | Create your own project with `sf app new` and write your own control law or estimator (**→ [Custom Program Guide](docs/guides/custom_program.md)**) |
| **Verify your flight program in SILS** | Check your flight program in SILS (Software In the Loop Simulation: the firmware itself flying on your PC) before flashing it to the real drone |

---

## 📦 Installation

**Install from the command line (CLI).** A GUI installer, "StampFly Setup", also exists,
but its stability is not yet confirmed, so please follow the CLI steps below.
The GUI installer is described in the **[GUI Installer Guide](docs/guides/gui-installer.md)**.

The procedure is the same three stages on every OS. Copy one code block at a time into
your terminal, wait for it to finish, then move on to the next block.

| Stage | What you do |
|-------|-------------|
| 1. Prerequisites | Install Git (Python is not required -- a private Python 3.12 and ESP-IDF v5.5.2 are installed automatically) |
| 2. Get and install | Clone the repository and run the installer |
| 3. Activate and check | Activate the development environment and run `sf doctor` |

The private Python, ESP-IDF, and toolchain are self-contained under `SF_HOME`
(default: `C:\StampFly` on Windows, `~/.stampfly` on macOS/Linux), about 4-6 GB in
total. It downloads a fair amount, so run it when you have some time and disk space
to spare. A **legacy mode** (`install.bat --use-existing-idf`, etc.) is also available
for developers who want to keep using an existing ESP-IDF and system Python -- see the
**[Setup Guide](docs/setup/README.md)** for details.

### Windows

Run in Command Prompt (CMD). WSL is not required.

**1. Install Git** (skip if already installed). The two trailing options skip the
license-agreement questions that appear on first use. Close and reopen CMD when done.

```cmd
winget install Git.Git --accept-source-agreements --accept-package-agreements
```

**2. Clone the repository and run the installer.** Answer its questions as prompted on screen.

```cmd
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
install.bat
```

**3. Activate the environment and check.** Run `setup_env.bat` in every new CMD window.

```cmd
setup_env.bat
sf doctor
```

**→ [Windows Setup (details and troubleshooting)](docs/setup/windows.md)**

### macOS

Run in Terminal.

**1. Install Homebrew** (skip if already installed). Press Enter and type your Mac login password.
If the Xcode Command Line Tools are missing, they are installed here as well.
When it finishes, copy and run the commands shown under "Next steps" to add brew to your PATH.

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Then install the build tools (Python is not required).

```bash
brew install cmake ninja dfu-util ccache
```

**2. Clone the repository and run the installer.** Answer its questions as prompted on screen.

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

**3. Activate the environment and check.** Run `source setup_env.sh` in every new terminal.

```bash
source setup_env.sh
sf doctor
```

**→ [macOS Setup (details and troubleshooting)](docs/setup/macos.md)**

### Ubuntu

Run in a terminal (Ubuntu 22.04 LTS or later).

**1. Install Git and the build tools** (Python is not required). Type your password when asked.

```bash
sudo apt update
sudo apt install -y git curl tar cmake ninja-build wget flex bison gperf ccache \
    libffi-dev libssl-dev dfu-util libusb-1.0-0
```

**2. Clone the repository and run the installer.** Answer its questions as prompted on screen.

```bash
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem.git
cd stampfly_ecosystem
./install.sh
```

**3. Activate the environment and check.** Run `source setup_env.sh` in every new terminal.

```bash
source setup_env.sh
sf doctor
```

To use the drone or transmitter over USB, grant serial-port permission once (log out and back in to apply).

```bash
sudo usermod -a -G dialout $USER
```

**→ [Linux Setup (details and troubleshooting)](docs/setup/linux.md)**

Installation is complete when `sf doctor` reports no problems.
To update later, run `sf upgrade` (**→ [Upgrading Guide](docs/guides/upgrading.md)**).

---

## 🎮 Try the Simulator First!

**No real drone needed.** With the transmitter (M5Stack AtomS3 + Atom JoyStick) and a PC, you can fly a 3D simulator on your PC.
The simulator uses the transmitter as a USB gamepad, so first flash its firmware and switch its communication mode to USB HID.

**1. Flash the transmitter firmware** (first time only). Connect the transmitter to the PC with a USB cable and run this in a terminal with the environment activated.

```bash
sf build controller
sf flash controller
```

**2. Switch the transmitter to USB HID mode.** These steps are done on the transmitter's screen.

| Step | Action |
|------|--------|
| 1 | Press the screen (button) to open the menu |
| 2 | Move the right stick up/down to select the `Comm: ESP-NOW` row |
| 3 | Press the right button (mode button): the row changes to `Comm: UDP`. Press again: it changes to `Comm: USB HID` and the transmitter restarts automatically |
| 4 | After the restart, the screen shows `= USB HID MODE =`. The PC now recognizes it as a gamepad |

**3. Launch the simulator.** A browser opens automatically with the 3D view. Raise the throttle slowly and the drone lifts off.

```bash
sf sim run vpython
```

For the stick assignment, world options, and troubleshooting, see the simulator guide.

**→ [Using the Simulator](docs/next_step.md#2-using-the-simulator)** | **[Controller Guide](docs/guides/controller.md)**

---

## 🛸 Fly the Real Drone

Once you are comfortable in the simulator, fly the real drone. This covers flashing the vehicle firmware, switching the transmitter back to the mode for real flight, and pairing the vehicle with the transmitter.

**1. Flash the vehicle firmware.** Connect the StampFly to the PC with a USB cable and run this in a terminal with the environment activated.

```bash
sf build vehicle
sf flash vehicle
```

**2. Switch the transmitter back to ESP-NOW mode.** This returns the communication mode from USB HID (simulator) to ESP-NOW, which talks to the real drone.

| Step | Action |
|------|--------|
| 1 | Press the screen (button) to open the menu |
| 2 | Move the right stick up/down to select the `Comm: USB HID` row |
| 3 | Press the right button (mode button) once: the row changes to `Comm: ESP-NOW` and the transmitter restarts automatically |

**3. Pair the vehicle with the transmitter** (first time only; afterwards they connect automatically at power-on).

| Step | Action |
|------|--------|
| 1 | Power off the transmitter, then power it on while holding the screen (button). The screen shows `Pairing mode...` and it beeps repeatedly |
| 2 | Power on the vehicle and hold its button for about 3 seconds. The LED blinks blue rapidly while it searches for a transmitter |
| 3 | Pairing is complete when the transmitter stops beeping and shows the flight screen, and the vehicle's blue blinking stops |

A vehicle with no pairing information starts searching as soon as it is powered on. The long press in step 2 clears any previous pairing so the search starts for certain, and is also how you re-pair with a different transmitter.

For the pre-flight checklist and stick operation (arm, take-off, landing, flight-mode switching), see the flying guide.

**→ [How to Fly](docs/next_step.md#5-how-to-fly)** | **[Controller Guide](docs/guides/controller.md)**

---

## 🎓 Learn Through the Workshops

Once you can fly the real drone, pick the course material that matches your goal. Slides and hands-on guides from three courses we have run are published here.

### StampFly Workshop (4 days + 1 competition day)

The standard curriculum for undergraduate and graduate students. Learners write only two functions, `setup()` and `loop_400Hz()`, and progress step by step: motor control → controller input → IMU → P control and first flight → modeling and system identification → PID → attitude estimation → Python SDK, ending with a precision-landing competition. Each lesson's exercise code is selected with `sf lesson switch <lesson>`.

| Material | Contents |
|----------|----------|
| [Instructor Guide](docs/events/stampfly_workshop/workshop_guide.md) | How to run Lessons 0–13, safety, troubleshooting |
| [Schedule](docs/events/stampfly_workshop/workshop_schedule.md) | 4+1 day timetable and equipment |
| [Competition Rules](docs/events/stampfly_workshop/competition_rules.md) | Events and scoring for the final day |
| [Slides (PDF)](docs/events/stampfly_workshop/slides/stampfly_workshop.pdf) | All lessons in one deck |

### DXH Workshop for High-School Teachers (2 hours)

A hands-on session for high-school teachers, including programming beginners. Four themes in 120 minutes: flying, installing the development environment, editing a program, and controlling the motors. It uses the browser-based flasher, so the real drone runs even in a short session.

| Material | Contents |
|----------|----------|
| [Overview](docs/events/dxh2026/README.md) | Audience, equipment, document list |
| [Participant Handout](docs/events/dxh2026/handout.md) | Web-flashing and motor-control exercise steps |
| [Slides (PDF)](docs/events/dxh2026/slides/dxh_workshop.pdf) | Session slides |

### SCI/SICE Tutorial 2026 (1 day)

For control-engineering researchers and educators. Five sessions cover the ecosystem overview, environment setup, sensor data acquisition, motor control and controller input, PID attitude stabilization, and the simulator and analysis tools. This is the entry point if you want to use StampFly as a research platform.

| Material | Contents |
|----------|----------|
| [Preparation and Index](docs/events/sci_tutorial_2026/README.md) | Timetable and participant preparation |
| [Hands-on Review Guide](docs/events/sci_tutorial_2026/handson_guide.md) | Reproduce each session's exercises on your own |
| [Cheat Sheet](docs/events/sci_tutorial_2026/cheatsheet.md) | Commands used on the day |
| [Slides (PDF)](docs/events/sci_tutorial_2026/slides/sci_tutorial.pdf) | All five sessions |

**→ [All Event Materials](docs/events/README.md)**

---

## Restore Factory Firmware

Workshop lessons and custom firmware will overwrite the factory firmware on your vehicle and controller.
To restore the factory state, run:

Restore the vehicle to factory firmware:

```bash
sf flash vehicle --legacy
```

Restore the controller to factory firmware:

```bash
sf flash controller --legacy
```

> **Note:** Run `source setup_env.sh` to set up the development environment first. Connect the device via USB before running.

---

## Technical Specifications

### Vehicle (StampFly)

| Item | Specification |
|------|---------------|
| MCU | ESP32-S3 (M5Stamp S3) |
| Mass | about 37 g (measured 36.8 g) |
| Size | motor-to-motor (diagonal) 65 mm, arm length (center to motor) 32.5 mm |
| Moments of inertia | Ixx 9.16e-6, Iyy 13.3e-6, Izz 20.4e-6 kg·m² |
| Motors | 4 (X configuration), 150 kHz PWM drive |
| Battery | 1S LiPo (3.7 V), low-voltage warning at 3.4 V |

### Sensors

| Sensor | Part | Rate | Purpose |
|--------|------|------|---------|
| IMU (accelerometer + gyro) | BMI270 | 400 Hz | Primary sensor for attitude estimation |
| Magnetometer | BMM150 | 25 Hz | Yaw estimation (research use) |
| Barometer | BMP280 | 50 Hz | Altitude beyond ToF range |
| ToF range (down, front) | VL53L3CX × 2 | 30 Hz | Altitude measurement, forward obstacle detection |
| Optical flow | PMW3901 | 100 Hz | Horizontal position estimation |
| Power monitor | INA3221 | 10 Hz | Battery voltage and current |

### Vehicle Software

| Item | Specification |
|------|---------------|
| Framework | ESP-IDF v5.5.2 + FreeRTOS |
| Control rate | 400 Hz (synchronized to the IMU) |
| State estimation | ESKF (Error-State Kalman Filter) for attitude, velocity and position |
| Flight modes | ACRO (rate control) / STABILIZE (angle control) / ALT_HOLD (altitude hold) / POS_HOLD (position hold) |
| Link to the controller | ESP-NOW (TDMA-synchronized, up to 10 vehicles) or UDP (via the vehicle's WiFi access point) |
| Link to a PC | WiFi telemetry at 50 Hz, high-rate log at 400 Hz, Tello-SDK-compatible API |
| Configuration | Parameters such as PID gains are changed at runtime and saved to NVS (non-volatile memory) |

### Controller

| Item | Specification |
|------|---------------|
| Hardware | M5Stack AtomS3 (ESP32-S3 with LCD) + Atom JoyStick |
| Communication modes | ESP-NOW / UDP / USB HID (acts as a PC gamepad) |
| Sticks | 2 axes × 2, Mode 2 / Mode 3 switchable, with push buttons |

### Development Tools

| Item | Specification |
|------|---------------|
| sf CLI | Python 3.10–3.12. Build, flash, log capture, calibration, simulator and SILS under one command set |
| Simulators | VPython (3D in the browser), Genesis (high-fidelity physics), SILS (the firmware itself running on the PC) |
| Supported OS | Windows / macOS / Ubuntu |

Confirmed physical parameters and their sources are in the [Physical Parameters Reference](docs/architecture/stampfly-parameters.md); vehicle software requirements and design are in the [Requirements](firmware/vehicle/docs/requirements.md) and [Architecture](firmware/vehicle/docs/architecture.md) documents.

### Repository Layout

```
stampfly_ecosystem/
├── docs/           # Documentation
├── firmware/       # Embedded firmware
│   ├── vehicle/     # Vehicle firmware (primary)
│   ├── vehicle_old/ # Legacy vehicle firmware (frozen)
│   ├── controller/  # Transmitter firmware
│   └── common/      # Shared code (ESP-NOW protocol structs)
├── protocol/       # Communication protocol spec (WIP)
├── control/        # Control design assets (WIP)
├── analysis/       # Experiment data analysis (WIP)
├── tools/          # Utility tools (WIP)
├── simulator/      # 3D flight simulator
├── ros/            # ROS integration (WIP)
├── examples/       # Learning examples
└── third_party/    # External dependencies
```

---

## 🔗 Resources and Documentation

| Resource | Description |
|----------|-------------|
| [📖 Next Steps](docs/next_step.md) | Using the simulator, pre-flight checks, how to fly, developer features |
| [🎛️ Controller Guide](docs/guides/controller.md) | Menu operation, communication modes, pairing, what each button does |
| [🧪 Custom Program Guide](docs/guides/custom_program.md) | Write your own controller or estimator, verify it in SILS, fly it on the real drone |
| [⌨️ sf Command Reference](docs/commands/README.md) | Every command: build, flash, log capture, and more |
| [🛠️ Setup Guide](docs/setup/README.md) | Detailed per-OS installation and troubleshooting |
| [🌐 Project Landing Page](https://m5fly-kanazawa.github.io/stampfly_ecosystem/) | Landing page with a 3D model of the real drone |
| [📚 Documentation Site](https://m5fly-kanazawa.github.io/stampfly_ecosystem/docs/) | Browse and search all documentation |
| [🗂️ Document Index](docs/DOCUMENT_INDEX.md) | Index of every document in the repository |
| [🔌 Web Flasher](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) | Flash the drone from your browser, no build needed |
| [🖥️ StampFly Flasher](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest) | Desktop flashing app (Windows/macOS, no Python required) |
| [📦 Pre-built Firmware](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases) | GitHub Releases (vehicle / controller) |
| [📐 Physical Parameters Reference](docs/architecture/stampfly-parameters.md) | Confirmed values of the vehicle's physical parameters (mass, inertia, motor characteristics, etc.) |
| [stampfly_physical.yaml](control/models/stampfly_physical.yaml) | Machine-readable SSOT (Single Source of Truth) for the physical parameters |

---

## License

MIT License
