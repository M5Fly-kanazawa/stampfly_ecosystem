# StampFly Flasher（GUI書き込みアプリ）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### これは何か

StampFly Flasher は、ビルド環境なしで StampFly 機体にファームウェアを書き込める
クロスプラットフォーム（Windows / macOS / Linux）の Tkinter 製 GUI アプリです。
GitHub Releases から最新の `full.bin`（フルイメージ）を自動ダウンロードし、
esptool をプロセス内で呼び出して書き込みます。

| 項目 | 内容 |
|------|------|
| 実装 | `tools/flasher_gui/stampfly_flasher.py`（Python + Tkinter） |
| 書き込み対象 | GitHub Releases の最新 `full.bin`（自動ダウンロード） |
| 書き込み方式 | esptool をインプロセス呼び出し |
| 対応OS | Windows / macOS |

### 対象ユーザー

| 想定ユーザー | 理由 |
|------------|------|
| ESP-IDF 等のビルド環境を持たない人 | 実行ファイルをダウンロードするだけで書き込みが完結する |
| macOS で Web Flasher（`/flash/`）が使えない人 | Chromium の Web Serial 実装のバグにより Chrome がクラッシュする既知の問題があり、その回避策になる |

## 2. 入手方法

### 実行ファイルをダウンロード（推奨）

[GitHub Releases](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases) から、
Windows・macOS 向けに CI がビルドした `StampFlyFlasher` 実行ファイルをダウンロードしてください。
Python のインストールは不要です。

### ソースから実行（開発者向け）

```bash
pip install esptool
python3 tools/flasher_gui/stampfly_flasher.py
```

## 3. 使い方

### 手順

1. 書き込み対象（target: `vehicle` / `controller` など）を選ぶ
2. シリアルポートは自動検出される（手動選択も可能）
3. 必要に応じて「erase」（フラッシュ全消去）を有効にする
4. 「Flash」ボタンを押して書き込みを実行

### 書き込み後の注意（ジャイロキャリブレーション）

書き込み完了後に機体が再起動すると、起動時ジャイロキャリブレーションが自動実行されます。
**このあいだは機体を静止させたまま動かさないでください。** キャリブレーション中に振動・移動があると、
以後の姿勢制御の基準がずれる原因になります。

### macOS で初回起動できない場合（Gatekeeper）

macOS は署名なしアプリの初回起動をブロックすることがあります（Gatekeeper）。その場合は
アプリを **右クリック（または Control+クリック）→「開く」** を選んで起動してください
（Finder のダブルクリックではブロックされたままになります）。

## 4. CI / 自動テスト

`--selftest` オプションで GUI を表示せずに自己診断のみ実行できます。CI でのビルド確認用です。

```bash
python3 tools/flasher_gui/stampfly_flasher.py --selftest
```

## 5. 関連

| コマンド / ページ | 説明 |
|------|------|
| `sf flash --gui` | sf CLI から本 GUI を起動するショートカット |
| [Web Flasher](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) | ブラウザから書き込む版（Chrome/Edge、macOSではクラッシュする既知の問題あり） |

---

<a id="english"></a>

# StampFly Flasher (GUI)

## 1. Overview

### What This Is

StampFly Flasher is a cross-platform (Windows / macOS / Linux) Tkinter GUI application that
flashes firmware to a StampFly vehicle without needing a build environment. It automatically
downloads the latest `full.bin` (full firmware image) from GitHub Releases and flashes it by
calling esptool in-process.

| Item | Details |
|------|---------|
| Implementation | `tools/flasher_gui/stampfly_flasher.py` (Python + Tkinter) |
| Flash target | Latest `full.bin` from GitHub Releases (auto-downloaded) |
| Flash method | In-process esptool call |
| Supported OS | Windows / macOS |

### Who It's For

| Intended user | Reason |
|---------------|--------|
| Users without a build environment (ESP-IDF, etc.) | Just download the executable and flash |
| macOS users who can't use the Web Flasher (`/flash/`) | Works around a known Chromium Web Serial bug that crashes Chrome on macOS |

## 2. How to Get It

### Download the Executable (Recommended)

Download the `StampFlyFlasher` executable built by CI for Windows and macOS from the
[GitHub Releases page](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases).
No Python installation required.

### Run From Source (For Developers)

```bash
pip install esptool
python3 tools/flasher_gui/stampfly_flasher.py
```

## 3. Usage

### Steps

1. Pick the flash target (e.g. `vehicle` / `controller`)
2. The serial port is auto-detected (manual selection is also available)
3. Optionally enable "erase" (full flash erase)
4. Click "Flash" to start flashing

### After Flashing (Gyro Calibration)

When the vehicle reboots after flashing, it automatically runs its startup gyro calibration.
**Keep the vehicle completely still during this time.** Any vibration or movement during
calibration will offset the attitude control reference going forward.

### First Launch on macOS (Gatekeeper)

macOS may block the first launch of an unsigned app (Gatekeeper). If this happens,
**right-click (or Control-click) the app and choose "Open"** instead of double-clicking it
in Finder, which would remain blocked.

## 4. CI / Self-Test

Use `--selftest` to run a self-check without showing the GUI, for CI build verification.

```bash
python3 tools/flasher_gui/stampfly_flasher.py --selftest
```

## 5. Related

| Command / Page | Description |
|-----------------|-------------|
| `sf flash --gui` | Shortcut to launch this GUI from the sf CLI |
| [Web Flasher](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) | Browser-based flashing (Chrome/Edge; known to crash on macOS) |
