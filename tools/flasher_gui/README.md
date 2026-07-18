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
| 対応OS | Windows / macOS / Linux |

### 対象ユーザー

| 想定ユーザー | 理由 |
|------------|------|
| ESP-IDF 等のビルド環境を持たない人 | 実行ファイルをダウンロードするだけで書き込みが完結する |
| macOS で Web Flasher（`/flash/`）が使えない人 | Chromium の Web Serial 実装のバグにより Chrome がクラッシュする既知の問題があり、その回避策になる |

## 2. 入手方法

### ネイティブアプリとしてインストール（推奨、`sf` 導入済みの場合）

`sf` CLI が使える環境（`source setup_env.sh` 済み）では、以下のコマンド1つで
最新リリースを GitHub からダウンロード（SHA256 検証つき）し、OS ネイティブの
デスクトップアプリとしてインストールできます。一度インストールすれば、以後は
ビルド環境なしでダブルクリックだけで起動できます。

```bash
sf flasher install
```

| OS | インストール先 | 起動導線 |
|----|--------------|---------|
| Windows | `%LOCALAPPDATA%\Programs\StampFly\`（管理者権限不要） | スタートメニュー + デスクトップ（任意）。「アプリと機能」からもアンインストール可 |
| macOS | `~/Applications/StampFlyFlasher.app`（quarantine属性は自動除去） | Launchpad |
| Linux | `~/.local/opt/stampfly/`（`v2026.07.2` 以降のリリースが必要） | アプリケーションメニュー（`.desktop` 登録） |

`sf flasher uninstall` / `sf flasher status` / `sf flasher update` にも対応しています。
詳細は [sf flasher コマンドリファレンス](../../docs/commands/sf-flasher.md) を参照してください。

### 実行ファイルを手動でダウンロード

`sf` CLI がまだ無い環境（配布のみを想定した端末等）では、
[GitHub Releases](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases) から、
Windows・macOS・Linux 向けに CI がビルドした `StampFlyFlasher` 実行ファイルを直接
ダウンロードして実行できます（Linux版は `v2026.07.2` 以降のリリースから提供）。
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
（Finder のダブルクリックではブロックされたままになります）。`sf flasher install`
経由でインストールした場合は quarantine 属性が自動的に除去されるため、この操作は不要です。

## 4. CI / 自動テスト

`--selftest` オプションで GUI を表示せずに自己診断のみ実行できます。CI でのビルド確認用です。

```bash
python3 tools/flasher_gui/stampfly_flasher.py --selftest
```

## 5. 関連

| コマンド / ページ | 説明 |
|------|------|
| `sf flash --gui` | sf CLI から本 GUI を起動するショートカット（インストール済みのネイティブアプリを優先） |
| [sf flasher](../../docs/commands/sf-flasher.md) | 本 GUI をネイティブアプリとしてインストール・更新・削除するコマンド |
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
| Supported OS | Windows / macOS / Linux |

### Who It's For

| Intended user | Reason |
|---------------|--------|
| Users without a build environment (ESP-IDF, etc.) | Just download the executable and flash |
| macOS users who can't use the Web Flasher (`/flash/`) | Works around a known Chromium Web Serial bug that crashes Chrome on macOS |

## 2. How to Get It

### Install as a Native App (Recommended, if `sf` Is Already Set Up)

If you have the `sf` CLI available (`source setup_env.sh` already run), a single
command downloads the latest release from GitHub (SHA256-verified) and installs it
as an OS-native desktop app. Once installed, it launches with a double-click — no
build environment needed.

```bash
sf flasher install
```

| OS | Install location | Launch entry point |
|----|--------------------|---------------------|
| Windows | `%LOCALAPPDATA%\Programs\StampFly\` (no admin rights required) | Start Menu + Desktop (optional). Can also be uninstalled from "Apps & features" |
| macOS | `~/Applications/StampFlyFlasher.app` (quarantine attribute removed automatically) | Launchpad |
| Linux | `~/.local/opt/stampfly/` (requires release `v2026.07.2` or later) | Applications menu (`.desktop` entry) |

`sf flasher uninstall` / `sf flasher status` / `sf flasher update` are also
available. See the [sf flasher command reference](../../docs/commands/sf-flasher.md)
for details.

### Download the Executable Manually

On a machine without the `sf` CLI (e.g. a device meant only for distribution), you
can download the `StampFlyFlasher` executable built by CI directly for Windows,
macOS, or Linux from the
[GitHub Releases page](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases)
(the Linux build is published starting with release `v2026.07.2`).
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
in Finder, which would remain blocked. This step is not needed when installed via
`sf flasher install`, since it removes the quarantine attribute automatically.

## 4. CI / Self-Test

Use `--selftest` to run a self-check without showing the GUI, for CI build verification.

```bash
python3 tools/flasher_gui/stampfly_flasher.py --selftest
```

## 5. Related

| Command / Page | Description |
|-----------------|-------------|
| `sf flash --gui` | Shortcut to launch this GUI from the sf CLI (prefers the installed native app) |
| [sf flasher](../../docs/commands/sf-flasher.md) | Install, update, or remove this GUI as a native app |
| [Web Flasher](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) | Browser-based flashing (Chrome/Edge; known to crash on macOS) |
