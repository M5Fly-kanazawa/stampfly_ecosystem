# v2026.07.3 リリースノート（ドラフト）

`docs/contributing/versioning.md` §5「リリース手順」の手順3（前回リリースからの変更点整理）
に対応する最小限のドラフト。**`v2026.07.3` タグはまだ作成されていない**（本ドキュメント作成
時点で最新タグは `v2026.07.1..main` 相当の1コミットのみを含む `v2026.07.2`）。本ドラフトは
下記の1機能のみを記録する暫定版であり、実際にタグを打つ前には他の変更点も棚卸しした上で
追記・更新すること。

対象範囲: `v2026.07.2..main`。

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
| macOS: `~/Applications/StampFly Terminal.command` | `cd "<root>" && source setup_env.sh && exec "${SHELL:-/bin/zsh}" -i`（chmod 0o755）。ダブルクリックで Terminal.app が開き、export された環境がそのまま対話シェルへ引き継がれる |
| Linux: `~/.local/share/applications/stampfly-terminal.desktop` | `Exec=bash -c 'cd "<root>" && source setup_env.sh && exec bash -i'`。インストール先パスに `'`（シングルクォート）が含まれる場合は安全に埋め込めないため作成をスキップし警告する。`update-desktop-database` をベストエフォートで実行 |
| Windows: スタートメニュー `...\Start Menu\Programs\StampFly\StampFly Terminal.lnk` | PowerShell の `WScript.Shell` COM 経由で作成（`lib/sfcli/utils/flasher_install/_windows.py` の手法を踏襲、pywin32 非依存）。`TargetPath=%ComSpec%`、`Arguments=/k "<root>\setup_env.bat"`。GUIフラッシャと同じ `StampFly` スタートメニューフォルダを共有する |
| `Installer.uninstall()` に対応する削除を追加 | 3OSの既知パスをベストエフォートで削除。Windows は共有の `StampFly` スタートメニューフォルダを、GUIフラッシャのショートカットが残っていなければ削除する |
| GUI（`tools/installer_gui/stampfly_installer.py`） | 完了画面の文言（`STRINGS["done_install_body"]`、ja/en）に「StampFly Terminal」からも始められる旨を1文追記。新規 stdlib import `tempfile` を hidden-import 契約（Windows版 `.ps1` 生成に使用）に追加 |
| ドキュメント | `docs/guides/gui-installer.md`（§3・FAQ に日英で追記） |

**検証（本ドラフト作成時点、未コミットの作業ツリーに対して実施）:**

- `python3 -m py_compile scripts/installer.py tools/installer_gui/stampfly_installer.py`: OK
- `python3 tools/installer_gui/stampfly_installer.py --selftest`: 全PASS（Step ヘッダ数は4のまま）
- `python3 tools/ci/check_installer_gui.py`: 3/3 PASS（stdlib hidden-import 契約含む）
- macOS 実機（この開発機）: `HOME` を一時ディレクトリへ退避した上で `_create_terminal_launcher` /
  `_remove_terminal_launcher` を直接呼び出し、`.command` の生成・chmod 0o755・`cd` 先の一致・
  削除の冪等性を確認（実際の `~/Applications` は変更していない）
- Linux/.desktop・Windows/.lnk は本開発機（macOS）では実動できないため、生成ロジックの単体検証
  （一時 `HOME`/`APPDATA` での生成物の内容確認、Windows は `.ps1` 生成内容の文字列検証、
  Windows パス区切りは POSIX 環境の `pathlib.Path` 制約により `PureWindowsPath` での照合に限定）
  に留めた。**両OSでの実機検証は未実施**

## 2. Next steps

- 本ドラフトが記録する変更をコミットする（このセッションでは意図的に未コミット）
- タグ作成前に `v2026.07.2..main` の差分を棚卸しし、他の変更点があれば本ドキュメントへ追記する
- Linux（GNOME/KDE 等のアプリ一覧からの起動）・Windows（日本語ユーザー名を含む実パスでの
  `.lnk` 起動）の実機検証
- `docs/contributing/versioning.md` §6「カリキュラム互換表」に `v2026.07.3` の行を追加
  （タグ作成後に確定情報で追記する）
