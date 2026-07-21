# v2026.07.3 リリースノート

> **注記:** `v2026.07.3` は 2026-07-20 にコミット `cf1d084e` で公開済み（13アセット、
> `gh release view v2026.07.3` で確認済み）。それ以降にリポジトリへ積まれた変更点は
> [`release-v2026.07.4-notes.md`](release-v2026.07.4-notes.md) を参照。

`docs/contributing/versioning.md` §5「リリース手順」の手順3（前回リリースからの変更点整理）
に対応する、公開済みリリース `v2026.07.3` の変更点記録。対象範囲: `v2026.07.2..cf1d084e`。

## 0. 修正: Setup 修復モードが古いクローンで失敗する問題

v2026.07.2 の StampFly Setup で、既存の古いクローン（例: 講習前に導入した環境）に対して
「修復」を実行すると、その古い installer.py が現行の呼び出し
（`Installer.run(no_flasher=...)`）を解釈できず `TypeError` で失敗した
（2026-07-20、講習用 Windows 実機で発生）。2段構えで根治:

| 対策 | 内容 |
|------|------|
| pull 先行 | 修復開始時に対象クローンを `git pull --ff-only` で自動最新化（ローカル変更等で更新不能なら警告してそのまま続行。破壊的操作なし） |
| シグネチャ耐性 | `inspect.signature` で対象 installer.py の `Installer.run()` が受け付ける引数だけを渡す（未対応引数は警告ログを出して無視）。v2026.07.1 実物の installer.py で TypeError 解消を確認済み。selftest に退行チェック2件を追加 |

（コミット: `cf1d084e`）

## 1. StampFly Setup / インストーラに「StampFly Terminal」ランチャー機能を追加

これまでエコシステムを使い始めるには、ターミナルを開いて `cd` した上で `source setup_env.sh`
（Windows は `setup_env.bat` の実行）を打つ必要があり、ターミナル操作に不慣れな初心者には
この最初の一歩自体が障壁になっていた。本機能は、ダブルクリックするだけで
`setup_env` 読み込み済みの端末が開くランチャーを、CLI インストーラ（`scripts/installer.py`）の
Step 3/4（StampFly CLI）成功後に自動作成する。GUI インストーラ（StampFly Setup）は
`scripts/installer.py` を単一実体としてプロセス内実行するため、この変更は GUI 経由のインストール
にも自動的に反映される。

| 変更 | 内容 |
|------|------|
| `Installer._create_terminal_launcher()` 新設（`scripts/installer.py`） | `run()` の Step 3/4 成功後・Step 4/4（GUIフラッシャ）ヘッダの前に呼ぶ。独立した `Step N/4` は増やさない（GUI が `header("Step N/4: ...")` をパースしてステップインジケータを進める契約を維持するため）。GUIフラッシャの Step 4/4 と同じくベストエフォート — 失敗しても `warn()` に留め、インストーラ全体を失敗にしない |
| macOS: `~/Applications/StampFly Terminal.app`（正式バンドル） | Launchpad/Spotlight に専用アイコン付きで表示。実処理は Contents/Resources 内の `.command`（`cd "<root>" && source setup_env.sh && exec "${SHELL:-/bin/zsh}" -i`）が担い、どのターミナルで開くかは `.command` の関連付け（既定 Terminal.app、iTerm2 等に変更可）に従う。初期実装の素の `.command` は作成時/アンインストール時に自動移行（削除） |
| Linux: `~/.local/share/applications/stampfly-terminal.desktop` | `Exec=bash -c 'cd "<root>" && source setup_env.sh && exec bash -i'`。インストール先パスに `'`（シングルクォート）が含まれる場合は安全に埋め込めないため作成をスキップし警告する。`update-desktop-database` をベストエフォートで実行 |
| Windows: スタートメニュー `...\Start Menu\Programs\StampFly\StampFly Terminal.lnk` | PowerShell の `WScript.Shell` COM 経由で作成（`lib/sfcli/utils/flasher_install/_windows.py` の手法を踏襲、pywin32 非依存）。`TargetPath=%ComSpec%`、`Arguments=/k "<root>\setup_env.bat"`。GUIフラッシャと同じ `StampFly` スタートメニューフォルダを共有する |
| `Installer.uninstall()` に対応する削除を追加 | 3OSの既知パスをベストエフォートで削除。Windows は共有の `StampFly` スタートメニューフォルダを、GUIフラッシャのショートカットが残っていなければ削除する |
| 専用アイコン（3OS） | `tools/terminal_launcher/assets/gen_icon_3d.py`（フラッシャのレンダラを共用）で生成した「ダークスレート+緑のシェルプロンプト `>_`」アイコンを、macOS は .app バンドル、Windows はショートカットの IconLocation、Linux は .desktop の Icon= に設定（資産欠如時は OS 既定アイコンへ劣化） |
| GUI（`tools/installer_gui/stampfly_installer.py`） | 完了画面の文言（`STRINGS["done_install_body"]`、ja/en）に「StampFly Terminal」からも始められる旨を1文追記。新規 stdlib import `tempfile` を hidden-import 契約（Windows版 `.ps1` 生成に使用）に追加 |
| ドキュメント | `docs/guides/gui-installer.md`（§3・FAQ に日英で追記） |

（コミット: `a0e13327`, `31c264a6`, `bf1b0113`）

**検証:**

- `python3 -m py_compile scripts/installer.py tools/installer_gui/stampfly_installer.py`: OK
- `python3 tools/installer_gui/stampfly_installer.py --selftest`: 全PASS（Step ヘッダ数は4のまま）
- `python3 tools/ci/check_installer_gui.py`: 3/3 PASS（stdlib hidden-import 契約含む）
- macOS 実機: `HOME` を一時ディレクトリへ退避した上で `_create_terminal_launcher` /
  `_remove_terminal_launcher` を直接呼び出し、`.command` の生成・chmod 0o755・`cd` 先の一致・
  削除の冪等性を確認（実際の `~/Applications` は変更していない）
- Linux/.desktop・Windows/.lnk は開発機（macOS）では実動できないため、生成ロジックの単体検証
  （一時 `HOME`/`APPDATA` での生成物の内容確認、Windows は `.ps1` 生成内容の文字列検証）に
  留めた。**両OSでの実機検証は未実施**

## 2. その他（ドキュメント整備）

| 内容 | コミット |
|------|---------|
| `docs/contributing/versioning.md` §6 カリキュラム互換表に `v2026.07.2` の行を追加（初出のStampFly Setup・`rate.roll.td=0.002`既定値を記録） | `296c6cf8` |
| 同表に `v2026.07.3` の行を追加（`cf1d084e` までの公開内容＝Terminalランチャー・修復モード修正を記録） | `30fc04d0` |

## 3. Next steps

- Linux（GNOME/KDE 等のアプリ一覧からの起動）・Windows（日本語ユーザー名を含む実パスでの
  `.lnk` 起動）での StampFly Terminal 実機検証
- `v2026.07.3` 公開後に見つかった追加の不具合・機能は `release-v2026.07.4-notes.md` を参照
