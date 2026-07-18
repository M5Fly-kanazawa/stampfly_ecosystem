# sf flasher

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

GUI 書き込みアプリ「StampFly Flasher」を、OS ネイティブのデスクトップアプリとして
インストール・更新・削除・状態確認します。GitHub Releases から最新版をダウンロードし
（SHA256 検証つき）、Windows / macOS / Linux それぞれの流儀でインストールします。

インストール後は `sf flash --gui` がインストール済みのネイティブアプリを自動的に
優先して起動します（未インストール時は従来どおり `tools/flasher_gui/stampfly_flasher.py`
スクリプトにフォールバック）。

## 2. 構文

```bash
sf flasher <subcommand> [options]
```

### サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `install` | 最新リリース（または `--from-file` で指定したファイル）をネイティブアプリとしてインストール |
| `uninstall` | インストール済みのネイティブアプリを削除 |
| `status` | インストール状況（バージョン・実行ファイルパス・導入日時）を表示 |
| `update` | 最新リリースを再取得してインストール（`install` のエイリアス） |

## 3. sf flasher install

```bash
sf flasher install [options]
```

### オプション

| オプション | 説明 |
|-----------|------|
| `--from-file PATH` | GitHub から取得せず、ローカルのビルド成果物からインストール（`.exe` / `.zip` / `.app` ディレクトリ / 素の実行ファイル。CI やローカルビルドの検証用） |
| `--no-desktop-shortcut` | デスクトップショートカットを作成しない（Windows のみ意味を持つ。macOS/Linux では無視される） |
| `-y, --yes` | 確認プロンプトを出さない（CI・`scripts/installer.py` 用） |

### OS ごとのインストール先

| OS | インストール先 | 起動導線 |
|----|--------------|---------|
| Windows | `%LOCALAPPDATA%\Programs\StampFly\StampFlyFlasher.exe`（管理者権限不要） | スタートメニュー + デスクトップ（任意）。「設定 > アプリ > インストールされているアプリ」（"StampFly Flasher"）にも登録され、そこからアンインストール可能 |
| macOS | `~/Applications/StampFlyFlasher.app`（quarantine 属性は自動除去、Gatekeeper 対策込み） | Launchpad |
| Linux | `~/.local/opt/stampfly/StampFlyFlasher` + `.desktop` ランチャー | アプリケーションメニュー |

Linux 版バイナリは `v2026.07.2` 以降のリリースから提供されます。それより古いリリースを
対象にすると、その旨を示すエラーメッセージが表示されます。

## 4. sf flasher uninstall

```bash
sf flasher uninstall [-y]
```

インストール時に記録したマニフェストを元に、実行ファイル・ショートカット・
レジストリ／`.desktop` エントリなどを削除します。Windows では
「設定 > アプリ > インストールされているアプリ」の "StampFly Flasher" から
アンインストールすることも可能です（内部的に同じ削除処理を実行します）。

## 5. sf flasher status

```bash
sf flasher status
```

インストール済みかどうか、インストール済みならバージョン・実行ファイルパス・
導入日時を表示します。

## 6. sf flasher update

```bash
sf flasher update [--no-desktop-shortcut] [-y]
```

`--from-file` を使わない `sf flasher install` のエイリアスで、常に最新の
GitHub Release を再取得してインストールし直します。

## 7. 使用例

```bash
# 最新版をインストール（対話確認あり）
sf flasher install

# CI・スクリプトから非対話でインストール
sf flasher install --yes

# ローカルビルドの成果物からインストール（検証用）
sf flasher install --from-file dist/StampFlyFlasher_v2026.07.2_macos-arm64.zip

# 状態確認
sf flasher status

# 削除
sf flasher uninstall --yes
```

## 8. 関連

| コマンド / ページ | 説明 |
|------|------|
| `sf flash --gui` | インストール済みのネイティブアプリを優先して起動（未インストール時はスクリプトにフォールバック） | 
| [tools/flasher_gui/README.md](../../tools/flasher_gui/README.md) | StampFly Flasher 本体（GUI アプリ）のドキュメント |
| [Web Flasher](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) | ブラウザから書き込む版（Chrome/Edge） |

---

<a id="english"></a>

## 1. Overview

Install, update, remove, or check the status of the "StampFly Flasher" GUI app as a
native desktop application. Downloads the latest release from GitHub Releases
(SHA256-verified) and installs it following each OS's own conventions
(Windows / macOS / Linux).

Once installed, `sf flash --gui` automatically prefers the installed native app
(falling back to the `tools/flasher_gui/stampfly_flasher.py` script as before when
nothing is installed).

## 2. Syntax

```bash
sf flasher <subcommand> [options]
```

### Subcommands

| Subcommand | Description |
|------------|-------------|
| `install` | Install the latest release (or a file given via `--from-file`) as a native app |
| `uninstall` | Remove the installed native app |
| `status` | Show installation status (version / executable path / install date) |
| `update` | Re-fetch the latest release and reinstall (alias for `install`) |

## 3. sf flasher install

```bash
sf flasher install [options]
```

### Options

| Option | Description |
|--------|-------------|
| `--from-file PATH` | Install from a local build artifact instead of fetching from GitHub (`.exe` / `.zip` / `.app` directory / bare executable — for CI or local-build verification) |
| `--no-desktop-shortcut` | Skip creating a desktop shortcut (Windows only; ignored on macOS/Linux) |
| `-y, --yes` | Skip the confirmation prompt (for CI / `scripts/installer.py`) |

### Install locations per OS

| OS | Install location | Launch entry point |
|----|-------------------|---------------------|
| Windows | `%LOCALAPPDATA%\Programs\StampFly\StampFlyFlasher.exe` (no admin rights required) | Start Menu + Desktop (optional). Also registered under Settings > Apps > Installed apps ("StampFly Flasher"), from which it can be uninstalled |
| macOS | `~/Applications/StampFlyFlasher.app` (quarantine attribute removed automatically, so Gatekeeper does not block the first launch) | Launchpad |
| Linux | `~/.local/opt/stampfly/StampFlyFlasher` + a `.desktop` launcher | Applications menu |

The Linux binary is published starting with release `v2026.07.2`. Targeting an older
release shows an error message explaining this.

## 4. sf flasher uninstall

```bash
sf flasher uninstall [-y]
```

Removes the executable, shortcuts, and registry/`.desktop` entries recorded in the
install manifest. On Windows, uninstalling via Settings > Apps > Installed apps
("StampFly Flasher") also works (it runs the same removal logic internally).

## 5. sf flasher status

```bash
sf flasher status
```

Shows whether the app is installed, and if so, its version, executable path, and
install date.

## 6. sf flasher update

```bash
sf flasher update [--no-desktop-shortcut] [-y]
```

An alias for `sf flasher install` without `--from-file`: always fetches and installs
the latest GitHub release.

## 7. Examples

```bash
# Install the latest release (interactive confirmation)
sf flasher install

# Non-interactive install from CI/scripts
sf flasher install --yes

# Install from a local build artifact (verification)
sf flasher install --from-file dist/StampFlyFlasher_v2026.07.2_macos-arm64.zip

# Check status
sf flasher status

# Remove
sf flasher uninstall --yes
```

## 8. Related

| Command / Page | Description |
|-----------------|-------------|
| `sf flash --gui` | Prefers the installed native app (falls back to the script when nothing is installed) |
| [tools/flasher_gui/README.md](../../tools/flasher_gui/README.md) | Documentation for the StampFly Flasher GUI app itself |
| [Web Flasher](https://m5fly-kanazawa.github.io/stampfly_ecosystem/flash/) | Browser-based flashing (Chrome/Edge) |
