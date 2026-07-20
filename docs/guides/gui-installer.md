# GUI インストーラガイド（StampFly Setup）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 3行でわかる結論

- ターミナルを使わずにエコシステムを導入したいなら、GUI インストーラ「**StampFly Setup**」を使ってください。ダウンロードして起動し、ウィザードの画面に従うだけです。
- 中身は CLI インストーラ（`install.sh` / `install.bat`）とまったく同じロジックが動いています。GUI はウィザード画面と進捗表示だけを担当するので、機能差はありません。
- 途中で詰まったら → [7. うまくいかないとき（CLI へのフォールバック）](#cli-fallback-ja)。

## 2. 対象読者

このガイドは、**ターミナル（CMD / シェル）を使った操作に慣れていない教員・学生**を対象にしています。GitやPythonのコマンドを打たなくても、ダウンロードしたアプリを起動してクリックするだけでエコシステム（`sf` CLI・ESP-IDF・書き込みアプリ）を導入できます。

すでにターミナル操作に慣れている場合は、[各OSのセットアップガイド](../setup/README.md)の CLI 手順（`install.sh` / `install.bat`）を直接使っても構いません。

## 3. ダウンロード

下の表のリンクをクリックすると、最新リリースの該当ファイルのダウンロードがそのまま始まります
（[Releases ページ](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest)の
Assets 一覧から選んでも同じものです）。

| OS / アーキテクチャ | ダウンロード（クリックで開始） |
|---------------------|--------------------------|
| Windows | [`StampFlySetup_windows-x64.exe`](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest/download/StampFlySetup_windows-x64.exe) |
| macOS（Apple Silicon: M1/M2/M3/M4） | [`StampFlySetup_macos-arm64.zip`](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest/download/StampFlySetup_macos-arm64.zip) |
| macOS（Intel） | [`StampFlySetup_macos-x64.zip`](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest/download/StampFlySetup_macos-x64.zip) |
| Linux（x64） | [`StampFlySetup_linux-x64`](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest/download/StampFlySetup_linux-x64) |

ファイル名にバージョンは含まれません（常に最新リリースのものが取得されます）。
Windows / macOS（Apple Silicon・Intel）/ Linux 版は v2026.07.2 以降のリリースから提供されます。

> **Tip（Macのチップ種別の確認方法）:** 画面左上の Apple メニュー →「この Mac について」で
> 「チップ」欄に表示される名称が `Apple M1/M2/M3/M4` なら Apple Silicon 版
> （`_macos-arm64.zip`）、`Intel` なら Intel 版（`_macos-x64.zip`）を選んでください。

導入されるものと目安のサイズ・時間（ESP-IDF を含む）は、起動直後の「ようこそ」画面
（[5. ウィザードの画面構成](#wizard-screens-ja)）に表示されます。おおむね数GB程度の
ダウンロードになるため、ネットワーク環境の良い場所での実行を推奨します。

インストール完了後は、OSごとに用意される「**StampFly Terminal**」ランチャー（Windows:
スタートメニュー、macOS: `~/Applications`、Linux: アプリ一覧）をダブルクリックすると、
`setup_env.sh`（Windows は `setup_env.bat`）を読み込み済みの端末がその場で開きます。
`cd` や `source` のコマンドを覚えなくても `sf` コマンドをすぐに使い始められます。

## 4. OS別の起動手順

### Windows

1. ダウンロードした `StampFlySetup_windows-x64.exe` をダブルクリックして起動します。
2. 初回起動時、Windows SmartScreen が「Windows によって PC が保護されました」という警告を
   表示することがあります（署名されていない実行ファイルであるため。書き込みアプリ
   「StampFly Flasher」と同じ注意点です）。「**詳細情報**」をクリックし、続けて表示される
   「**実行**」ボタンを押すと起動します。
3. ウィザードが開始されます（[5. ウィザードの画面構成](#wizard-screens-ja)を参照）。

### macOS

1. ダウンロードした zip ファイル（Apple Silicon なら `_macos-arm64.zip`、Intel なら
   `_macos-x64.zip`）をダブルクリックして展開すると、`StampFlySetup.app` が現れます。
2. 初回起動はダブルクリックだけでは開けません（ダウンロードしたファイルに自動で付く
   quarantine 属性（隔離属性。macOS が「インターネット経由で取得した未検証のアプリ」に
   自動で付けるマーク）を、Gatekeeper（macOS の実行許可チェック機構）がブロックするため）。
   `StampFlySetup.app` を**右クリック（または Control キーを押しながらクリック）→「開く」**
   を選び、表示される確認ダイアログでもう一度「開く」を押してください。
3. 2回目以降は通常のダブルクリックで起動できます。

> **補足:** これは Setup アプリ自体の初回起動時にのみ必要な操作です。Setup が導入する
> 「StampFly Flasher」（書き込みアプリ）を `sf flasher install` 経由で導入した場合は
> quarantine 属性が自動的に除去されるため、この手順は不要になります（詳細:
> [sf flasher](../commands/sf-flasher.md)）。

### Linux

1. ダウンロードした `StampFlySetup_linux-x64`（拡張子なしの実行ファイル）に実行権限を
   付与します。ファイルマネージャからなら、右クリック→「プロパティ」→「アクセス権」タブで
   「プログラムとして実行可能」にチェックします。ターミナルを使う場合は次の1行です:
   ```bash
   chmod +x StampFlySetup_linux-x64
   ```
2. ダブルクリック、またはターミナルで次のように実行します:
   ```bash
   ./StampFlySetup_linux-x64
   ```

<a id="wizard-screens-ja"></a>

## 5. ウィザードの画面構成

StampFly Setup は5画面のウィザード形式です。**UI は日本語 / English を切替可能**
（ようこそ画面右上のスイッチ。初期言語は OS のロケール設定から自動判定され、
判定材料が無い場合は日本語になります）。

| # | 画面 | 内容 |
|---|------|------|
| 1 | ようこそ | 何が導入されるか（sf CLI / ESP-IDF / 書き込みアプリ）、目安のダウンロードサイズ・所要時間を表示。右上に言語切替（日本語 / English） |
| 2 | 環境チェック | git・Python・ディスク空き容量・ネットワーク接続を検査。NG項目があれば OS別の対処コマンド（`winget` / `brew` / `apt` 等）をコピーボタン付きで提示し、直せたら「再チェック」できる |
| 3 | オプション | インストール先フォルダ、書き込みアプリの同梱（既定 ON）、ショートカット作成（既定 ON）、minimal インストール（シミュレータ依存を省略、既定 OFF）を選択。既存インストールを検出した場合は「そのまま使う（修復）」/「アンインストール」も選べる（詳細: [6. 既存インストールがある場合](#repair-uninstall-ja)） |
| 4 | 実行 | クローン → Step 1/4 〜 4/4 の進捗をステップインジケータで表示しながら、処理のログをその場に流す |
| 5 | 完了 | 成功時: 次にやること（`sf doctor` の実行など）を案内。失敗時: ログの保存ボタンと、CLI での復旧手順（[7. うまくいかないとき](#cli-fallback-ja)）を案内 |

<a id="repair-uninstall-ja"></a>

## 6. 既存インストールがある場合（修復・アンインストール）

StampFly Setup は既定でエコシステムを `~/stampfly_ecosystem`（変更可）へ取得します。この場所に
既にエコシステムが導入済みだと検出すると、「オプション」画面（[5.](#wizard-screens-ja)）で
次のどちらかを選べます。

| 選択肢 | 内容 | CLI 版での相当コマンド |
|--------|------|------------------------|
| そのまま使う（修復） | 既存のチェックアウトに対してインストール処理をやり直す。環境が壊れているかもしれないときの入れ直しに使う | `install.bat --force` / `./install.sh --force` |
| アンインストール | エコシステムを削除する | `install.bat --uninstall` / `./install.sh --uninstall` |

何が削除され、何が残るか（ESP-IDF本体やツールチェーンなど、他プロジェクトと共有され得る
ものは残す設計）の詳しい一覧は、[アップグレードガイド §5](upgrading.md#5)
の表を参照してください。同じ表が CLI 版の `--uninstall` / `--clean` 実行時にもコンソールへ表示されます。

<a id="cli-fallback-ja"></a>

## 7. うまくいかないとき（CLI へのフォールバック）

ウィザードが途中で止まる、ネットワークエラーが繰り返される等、GUI でうまく進まない場合は、
CLI 版のインストーラを直接実行できます。中身のロジックは同じなので、GUI で失敗した箇所も
CLI ならエラーメッセージが詳しく見えて対処しやすいことがあります。

```cmd
:: Windows（CMD）
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem
cd stampfly_ecosystem
install.bat
```

```bash
# macOS / Linux
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem
cd stampfly_ecosystem
./install.sh
```

OS別の詳しい手順・トラブルシューティングは [Windows](../setup/windows.md) /
[macOS](../setup/macos.md) / [Linux](../setup/linux.md) の各セットアップガイドを参照してください。

完了画面（失敗時）でログを保存した場合は、その内容を添えて問い合わせるとトラブルシューティングが
スムーズです。

## 8. よくある質問（FAQ）

### Python を別途インストールする必要がありますか？

StampFly Setup 自体は Python 実行環境を同梱しているため不要です。ただし ESP-IDF 自身が別途
Python を必要とする場面があり、その場合は警告として案内されます（エラーにはなりません）。

### git は必要ですか？

必要です。エコシステム本体を GitHub から取得するために使います。無い場合は「環境チェック」画面
（[5.](#wizard-screens-ja)）が OS別の導入コマンドを提示します。

### GUI 版と CLI 版（`install.sh` / `install.bat`）で機能に差はありますか？

ありません。StampFly Setup は CLI 版インストーラ（`scripts/installer.py`）をクローンした
リポジトリからそのまま呼び出すラッパーです。ウィザードとログ表示だけを GUI が担当し、実際の
導入処理は CLI 版とまったく同じロジックで行われます。そのため CLI 側の修正・改善は、GUI 側を
再リリースしなくてもそのまま反映されます。

### エコシステムはどこに導入されますか？

既定では `~/stampfly_ecosystem` です。ウィザードの「オプション」画面で変更できます。

### 導入後、最新版に更新するには？

StampFly Setup は初回導入専用です。導入後にエコシステムを最新化したくなったら、
[アップグレードガイド](upgrading.md)の `sf upgrade` を使ってください。

### アンインストールするには？

StampFly Setup をもう一度起動すると、既存インストールを検出して「アンインストール」を選べます。
詳細は [6. 既存インストールがある場合](#repair-uninstall-ja)を参照してください。

### ターミナルを開かずに `sf` を使うには？

インストール完了後に作成される「StampFly Terminal」ランチャー（Windows: スタートメニュー、
macOS: `~/Applications`、Linux: アプリ一覧）から起動してください。`setup_env.sh`
（Windows は `setup_env.bat`）を読み込み済みの端末がすぐに開くので、`cd` や `source` を
自分で打たなくても `sf` コマンドをそのまま使い始められます。

### StampFly Terminal を Terminal.app 以外（iTerm2 等）で開きたい

macOS の `.command` ファイルは「どのアプリで開くか」の関連付けで開くターミナルが決まります。
Finder で `~/Applications/StampFly Terminal.command` を選択 →「情報を見る」（⌘I）→
「このアプリケーションで開く」で iTerm 等を選んでください（「すべてを変更…」を押すと
全 `.command` ファイルに適用されます）。中身は対話シェルへ引き継ぐだけなので、どの
ターミナルで開いても普段のシェル設定がそのまま効きます。

## 9. 関連

| ページ | 内容 |
|--------|------|
| [Windows セットアップ](../setup/windows.md) / [macOS セットアップ](../setup/macos.md) / [Linux セットアップ](../setup/linux.md) | CLI（`install.sh` / `install.bat`）でのセットアップ手順 |
| [アップグレードガイド](upgrading.md) | 導入後の更新（`sf upgrade`）・インストール/アンインストールのライフサイクル |
| [sf flasher](../commands/sf-flasher.md) | StampFly Setup が同梱する書き込みアプリのコマンドリファレンス |
| `docs/plans/gui-installer-plan.md` | GUI インストーラの設計方針（開発者向け） |

---

<a id="english"></a>

## 1. Three-Line Summary

- If you'd rather install the ecosystem without touching a terminal, use the GUI installer
  "**StampFly Setup**." Download it, launch it, and follow the wizard.
- Internally it runs the exact same logic as the CLI installer (`install.sh` / `install.bat`).
  The GUI only adds the wizard screens and progress display, so there is no feature gap.
- If you get stuck, see [7. Falling Back to the CLI](#cli-fallback-en).

## 2. Target Audience

This guide targets **teachers and students who are not comfortable operating a terminal
(CMD / shell)**. You can install the ecosystem (`sf` CLI, ESP-IDF, the flashing app) by
downloading an app and clicking through it — no `git` or Python commands required.

If you're already comfortable with a terminal, feel free to use the CLI steps
(`install.sh` / `install.bat`) directly from the [per-OS setup guides](../setup/README.md)
instead.

## 3. Download

Clicking a link below starts the download of the latest release's file directly
(the same files are also listed under Assets on the
[Releases page](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest)).

| OS / Architecture | Download (starts on click) |
|--------------------|-------------------|
| Windows | [`StampFlySetup_windows-x64.exe`](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest/download/StampFlySetup_windows-x64.exe) |
| macOS (Apple Silicon: M1/M2/M3/M4) | [`StampFlySetup_macos-arm64.zip`](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest/download/StampFlySetup_macos-arm64.zip) |
| macOS (Intel) | [`StampFlySetup_macos-x64.zip`](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest/download/StampFlySetup_macos-x64.zip) |
| Linux (x64) | [`StampFlySetup_linux-x64`](https://github.com/M5Fly-kanazawa/stampfly_ecosystem/releases/latest/download/StampFlySetup_linux-x64) |

The filenames carry no version (the links always fetch the latest release).
Windows / macOS (Apple Silicon and Intel) / Linux builds are available starting
with release v2026.07.2.

> **Tip (checking your Mac's chip):** Apple menu (top-left) → "About This Mac." If "Chip" shows
> `Apple M1/M2/M3/M4`, use the Apple Silicon build (`_macos-arm64.zip`); if it shows `Intel`, use
> the Intel build (`_macos-x64.zip`).

What gets installed, and the approximate size/time (including ESP-IDF), is shown on the
"Welcome" screen right after launch (see
[5. Wizard Screens](#wizard-screens-en)). The download totals a few GB, so a good network
connection is recommended.

After the install finishes, double-clicking the OS-specific "**StampFly Terminal**" launcher
(Windows: Start Menu, macOS: `~/Applications`, Linux: your app launcher) opens a terminal with
`setup_env.sh` (`setup_env.bat` on Windows) already sourced. You can start typing `sf` commands
right away, without ever learning `cd` or `source`.

## 4. Launching on Each OS

### Windows

1. Double-click the downloaded `StampFlySetup_windows-x64.exe` to launch it.
2. On first launch, Windows SmartScreen may show "Windows protected your PC" (because the
   executable is unsigned — the same caveat applies to the "StampFly Flasher" flashing app).
   Click "**More info**," then click the "**Run anyway**" button that appears.
3. The wizard starts (see [5. Wizard Screens](#wizard-screens-en)).

### macOS

1. Double-click the downloaded zip (`_macos-arm64.zip` for Apple Silicon, `_macos-x64.zip` for
   Intel) to unzip it, revealing `StampFlySetup.app`.
2. A plain double-click won't open it the first time — the quarantine attribute (a flag macOS
   automatically attaches to apps downloaded from the internet that haven't been notarized) gets
   blocked by Gatekeeper (macOS's launch-permission check). **Right-click (or Control-click)
   `StampFlySetup.app` → "Open,"** then click "Open" again in the confirmation dialog.
3. From the second launch on, a normal double-click works.

> **Note:** This step is only needed the first time you launch the Setup app itself. The
> "StampFly Flasher" flashing app that Setup installs has its quarantine attribute removed
> automatically when installed via `sf flasher install`, so this step is not needed for it
> (details: [sf flasher](../commands/sf-flasher.md)).

### Linux

1. Mark the downloaded `StampFlySetup_linux-x64` (an extension-less executable) as
   executable. From a file manager: right-click → Properties → Permissions tab → check "Allow
   executing file as program." From a terminal:
   ```bash
   chmod +x StampFlySetup_linux-x64
   ```
2. Double-click it, or run it from a terminal:
   ```bash
   ./StampFlySetup_linux-x64
   ```

<a id="wizard-screens-en"></a>

## 5. Wizard Screens

StampFly Setup is a 5-screen wizard. **The UI is switchable between Japanese and
English** (switch at the top right of the Welcome screen; the initial language is
auto-detected from your OS locale, defaulting to Japanese when no signal is
available).

| # | Screen | Content |
|---|--------|---------|
| 1 | Welcome | What gets installed (sf CLI / ESP-IDF / flashing app), plus the approximate download size and time. Language switch (日本語 / English) at the top right |
| 2 | Environment Check | Probes git, Python, disk space, and network connectivity. Failing items show an OS-specific fix command (`winget` / `brew` / `apt`, etc.) with a copy button, and you can "Recheck" once fixed |
| 3 | Options | Choose the install location, whether to bundle the flashing app (default ON), whether to create shortcuts (default ON), and a minimal install (skips simulator dependencies, default OFF). If an existing install is detected, you can also choose "Use as-is (repair)" / "Uninstall" (details: [6. If an Existing Install Is Found](#repair-uninstall-en)) |
| 4 | Execute | Shows a step indicator progressing from clone through Step 1/4–4/4, streaming the live log alongside |
| 5 | Done | On success: what to do next (e.g. run `sf doctor`). On failure: a "Save log" button plus the CLI recovery steps ([7. Falling Back to the CLI](#cli-fallback-en)) |

<a id="repair-uninstall-en"></a>

## 6. If an Existing Install Is Found (Repair / Uninstall)

StampFly Setup clones the ecosystem to `~/stampfly_ecosystem` by default (changeable). If it
detects the ecosystem is already installed there, the "Options" screen
([5.](#wizard-screens-en)) offers a choice:

| Option | What it does | CLI equivalent |
|--------|---------------|-----------------|
| Use as-is (repair) | Re-runs the install process against the existing checkout — useful when the environment might be broken and you want a clean reinstall | `install.bat --force` / `./install.sh --force` |
| Uninstall | Removes the ecosystem | `install.bat --uninstall` / `./install.sh --uninstall` |

For the full list of what gets removed versus what's left alone (ESP-IDF itself, its toolchain,
and other items that may be shared with other projects are deliberately kept), see the table in
[Upgrading Guide §5](upgrading.md#5-install-uninstall-lifecycle). The same table is printed to
the console when running the CLI's `--uninstall` / `--clean`.

<a id="cli-fallback-en"></a>

## 7. Falling Back to the CLI

If the wizard stalls, or you keep hitting network errors, you can run the CLI installer
directly. It runs the same logic underneath, and the CLI's error messages are sometimes more
detailed than what the GUI shows, which can make a failure easier to diagnose.

```cmd
:: Windows (CMD)
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem
cd stampfly_ecosystem
install.bat
```

```bash
# macOS / Linux
git clone https://github.com/M5Fly-kanazawa/stampfly_ecosystem
cd stampfly_ecosystem
./install.sh
```

For detailed, per-OS steps and troubleshooting, see the [Windows](../setup/windows.md),
[macOS](../setup/macos.md), and [Linux](../setup/linux.md) setup guides.

If you saved a log from the "Done" screen on failure, attaching it makes troubleshooting faster.

## 8. FAQ

### Do I need to install Python separately?

No — StampFly Setup bundles its own Python runtime. However, ESP-IDF itself sometimes needs a
separate Python, in which case you'll see a warning (not an error) with guidance.

### Do I need git?

Yes — it's used to fetch the ecosystem from GitHub. If it's missing, the "Environment Check"
screen ([5.](#wizard-screens-en)) shows an OS-specific install command.

### Is there any feature difference between the GUI and CLI (`install.sh` / `install.bat`) installers?

None. StampFly Setup is a wrapper that calls the CLI installer (`scripts/installer.py`) straight
out of the cloned repository. The GUI only owns the wizard and log display; the actual install
logic is identical to the CLI's. That means fixes and improvements to the CLI side apply
automatically without a new GUI release.

### Where does the ecosystem get installed?

`~/stampfly_ecosystem` by default. Change it on the "Options" screen.

### How do I update after installing?

StampFly Setup is a one-time install tool. Once installed, use `sf upgrade` — see the
[Upgrading Guide](upgrading.md) — to bring the ecosystem up to date.

### How do I uninstall?

Launch StampFly Setup again; it will detect the existing install and offer "Uninstall." See
[6. If an Existing Install Is Found](#repair-uninstall-en) for
details.

### How do I use `sf` without opening a terminal manually?

Launch the "StampFly Terminal" entry created for your OS after install finishes (Windows: Start
Menu, macOS: `~/Applications`, Linux: your app launcher). It opens a terminal with
`setup_env.sh` (`setup_env.bat` on Windows) already sourced, so you can start typing `sf`
commands immediately — no need to type `cd` or `source` yourself.

### Can StampFly Terminal open in something other than Terminal.app (e.g. iTerm2)?

Yes. On macOS, which app opens a `.command` file is decided by the file
association: select `~/Applications/StampFly Terminal.command` in Finder,
Get Info (Cmd+I), then under "Open with" choose iTerm etc. ("Change All..."
applies it to every `.command` file). The script simply hands off to your
interactive shell, so your usual shell configuration applies in whichever
terminal opens it.

## 9. Related

| Page | Content |
|------|---------|
| [Windows setup](../setup/windows.md) / [macOS setup](../setup/macos.md) / [Linux setup](../setup/linux.md) | CLI (`install.sh` / `install.bat`) setup steps |
| [Upgrading Guide](upgrading.md) | Post-install updates (`sf upgrade`) and the install/uninstall lifecycle |
| [sf flasher](../commands/sf-flasher.md) | Command reference for the flashing app StampFly Setup bundles |
| `docs/plans/gui-installer-plan.md` | Design plan for the GUI installer (for developers) |
