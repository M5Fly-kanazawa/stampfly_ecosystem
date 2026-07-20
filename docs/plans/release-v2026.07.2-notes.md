# v2026.07.2 リリースノート（ドラフト）

`docs/contributing/versioning.md` §5「リリース手順」の手順3（前回リリースからの変更点整理）
に対応するドラフト。**`v2026.07.2` タグはまだ作成されていない**（本ドキュメント作成時点で
最新タグは `v2026.07.1`、作業ブランチは `feature/flasher-native-install`）。タグを打つ際は、
このドラフトの要点を GitHub の自動生成 release notes（`generate_release_notes: true`）に
追記する。

対象範囲: `v2026.07.1..main` の80コミット + `feature/flasher-native-install` ブランチの
4コミット（マージ後に `main` へ合流する想定）。

## 1. 目玉: GUIフラッシャのネイティブインストール（`sf flasher`）

これまで GUI フラッシャ（StampFly Flasher）は「実行ファイルをダウンロードして手動で置く」
運用しかできず、教員が `.exe` を手作業でコピーする必要があった（DXH講座準備で顕在化）。
本リリースでは、これを OS 標準の「アプリをインストールする」体験に揃えた。

| 変更 | コミット | 内容 |
|------|---------|------|
| `sf flasher` コマンド新設 | `e834558` | `install` / `uninstall` / `status` / `update`。GitHub Releases から最新版を取得し SHA256 検証の上、Windows は `%LOCALAPPDATA%\Programs\StampFly`＋スタートメニュー/デスクトップショートカット＋「アプリと機能」登録（管理者権限不要）、macOS は `~/Applications/StampFlyFlasher.app`（quarantine属性自動除去）、Linux は `~/.local/opt/stampfly`＋`.desktop`ランチャーとしてインストール |
| インストーラ Step 4/4 | `16a79df` | `install.bat` / `install.sh`（`scripts/installer.py`）が最後に任意の Step 4/4 として GUI フラッシャのインストールを提案（既定 Yes、`--no-flasher` でスキップ、失敗してもインストーラ全体は失敗にしないベストエフォート） |
| Linux版フラッシャビルド + 4OSスモークテスト | `fcaaf73` | リリースCIに `ubuntu-latest` を追加し `StampFlyFlasher_<tag>_linux-x64` を新規公開。4レッグ（windows-x64 / macos-arm64 / macos-x64 / linux-x64）全てで `sf flasher install --from-file` → 各OSの設置物存在確認 → `sf flasher uninstall` → 削除確認、のスモークテストを追加。Windowsは「設定 > アプリ」のアンインストール経路（`uninstall.cmd`）も別途検証 |
| controller CI修復 | `9304b7d` | `m5unified` を `^0.2.11` → `==0.2.11` に固定（0.2.18が公開されCI解決が壊れていたため）。挙動変化なし、CI安定化のみ |
| `sf flash --gui` の優先順位変更 | `e834558`（flash.py） | インストール済みのネイティブアプリを優先して起動し、未インストール時のみ従来のスクリプト起動にフォールバック |
| macOS の SSL 証明書検証失敗を修正 | `d77c595` | 凍結アプリの macOS Python は既定の証明書検証パスを持たず、GitHub への全通信が `CERTIFICATE_VERIFY_FAILED` になっていた（2026-07-20 実機で発見。**v2026.07.1 の macOS 版アプリはこの不具合の影響下にある**）。certifi の CA バンドルで検証するよう変更（アプリ・`sf flasher` 双方。certifi 無しの環境は従来動作へフォールバック） |
| アプリアイコンを実機3Dモデルから刷新 | `8bfedf55` ほか | 従来の平面プレースホルダを廃し、landing ページの実機9パーツ STL＋手続き生成プロペラを numpy 製 Zバッファレンダラで描画した「Navy Classic」アイコン（前方45°俯瞰・稲妻が M5 へ注ぐ構図）に全面差し替え。`icon.ico` / `icon.icns` / PNG 各サイズを `tools/flasher_gui/assets/gen_icon_3d.py`（自己完結・再現可能）で生成。Windows/macOS のアプリ本体・ショートカットに v2026.07.2 ビルドから反映される |
| アイコンの各OS標準サイズ最適化 | 本コミット | 各OSの実表示サイズ（macOS Launchpad 128pt=Retina実256px、GNOMEアプリ一覧96px/ドック64px、Windowsデスクトップ48px/タスクバー32〜24px/一覧16px）を全て実サイズで収録。**32px以下は3D機体が判読不能になるため、稲妻を大きくした簡略版アートワーク**を `.ico`/`.icns` の該当枠に埋め込み（小サイズアイコンの定石）。Linux は従来の256px 1枚から **hicolor テーマ9サイズ（16〜512px）設置**に変更し、GNOME が縮小せず正確なサイズを選べるようにした（`_linux.py`）。CIスモークテストに 256/96/16px の設置・削除検査を追加 |
| エコシステム GUI インストーラ（StampFly Setup）新設 | 本コミット | ターミナル不要でエコシステム一式（sf CLI + ESP-IDF + フラッシャ）を導入する5画面ウィザード。**CLI `installer.py` を単一実体としてプロセス内実行**（GUIは自前のインストールロジックを持たず、clone した最新の installer.py を driving — CLI 側の改善が GUI 再リリース無しで反映される）。非対話契約 `SF_INSTALLER_NONINTERACTIVE`+EOF安全化、**UI 日英切替**（ロケール自動判定）、既存インストールの修復/アンインストール対応、専用3Dアイコン（ティール+白ダウンロード矢印）、4OS 配布でリリースアセット 9 → 13。ガイド: `docs/guides/gui-installer.md` |

**ドキュメント（本コミット、P4）:** `docs/commands/sf-flasher.md` 新設、
`tools/flasher_gui/README.md` / `docs/setup/windows.md` / `docs/setup/macos.md` /
`docs/setup/linux.md` / `docs/getting-started.md` / `docs/commands/README.md` /
`.mkdocs/mkdocs.yml` を更新（詳細は本コミットの変更ファイル一覧を参照）。

## 2. `sf upgrade` 新設 + インストーラの依存欠落耐性強化

これまでエコシステムを最新化する手段は `git pull` を手動で行うしかなく、ローカル変更との
衝突・Python依存の再同期忘れ・ESP-IDFの `sdkconfig` 陳腐化に非エキスパートのユーザーが
個別に対処する必要があった。加えて、1つのサードパーティ依存（PyYAML等）が欠けているだけで
`sf` コマンド全体が起動不能になる問題があった（V3実測）。本リリースでこの両方に対処した。

| 変更 | 内容 |
|------|------|
| `sf upgrade` コマンド新設（`lib/sfcli/commands/upgrade.py`） | `git fetch` → 更新プレビュー→確認 → ローカル変更の自動stash保護→ff-onlyマージ→復元 → Python依存の再同期（`pip install -e .`） → `sdkconfig.defaults`/`partitions.csv` 変化時の既存 `sdkconfig` 退避（`*.pre-upgrade-backup`）→ ネイティブGUIフラッシャの更新提案（未導入なら、チェックアウトにつき一回だけインストールを提案。既定は入れない＝`n`、`--yes`/`--no-flasher` は機会を消費せずスキップし `.sf/flasher_install_offered` で二度と尋ねない）→ サマリ表示、を一括実行。`--yes` / `--discard-local` / `--no-flasher` / `--skip-deps` に対応。stash復元時の衝突やfast-forward不能時はexit code 2で終了し、rebase/resetは一切自動実行しない。**標準ライブラリのみで実装**（依存が全滅した壊れた環境でも実行できる復旧経路であるため）。取得した更新に `sf` 自身（`lib/sfcli`）への変更が含まれる場合は、プレビュー表示前に取得したばかりの最新版の `upgrade` ロジックへ自動的に処理を引き継ぐ**自己ブートストラップ**（`paths.root()` の `SF_ROOT_OVERRIDE` 内部機構と併用）を実装済みで、`upgrade` 自体へのバグ修正が1回の実行で反映される |
| `sf` の依存欠落耐性（`lib/sfcli/cli.py` / `lib/sfcli/commands/__init__.py`） | コマンドモジュールを個別に `try/except ImportError` で import する方式に変更。1コマンドの依存欠落が他の全コマンド・`sf`自体を道連れにしなくなった。読み込めなかったコマンドは `sf --help` 実行時に1行warningで通知（`sf upgrade` または `pip install -e <root>` を案内）。`cli.py` に `assert_all_commands_loadable()` を追加し、インストーラのプローブ（下記）から利用可能に |
| インストーラのプローブ修正（`scripts/installer.py`） | `_is_sfcli_installed()` を `import sfcli` から `import sfcli.cli; sfcli.cli.assert_all_commands_loadable()` に変更。旧プローブは `sfcli/__init__.py` が依存を読まないため「壊れているのに正常」と誤診していた（V3実測） |
| `sf doctor` の依存チェックリスト動的化 | `importlib.metadata.requires("stampfly-ecosystem")` から実行時依存を動的生成する方式に変更（パース失敗時は従来のハードコードリストへフォールバック） |
| ESP-IDFバージョン比較の修正 | `find_all()` の文字列ソートを数値タプル比較（`version_sort_key()`）に変更。`"v5.10.0"` が `"v5.5.2"` より古いと誤判定されていたバグを修正 |
| `--uninstall` / `--clean` の改善 | 先に `sfcli.cli flasher uninstall --yes` を実行してからsfcli本体を削除するよう順序を修正。削除しないもの（ESP-IDF本体・`IDF_TOOLS_PATH`・venv依存・udevルール・リポジトリ本体）の一覧表をコンソールに表示するようにした |
| `install.sh` / `install.bat` | `--help` / `--uninstall` / `--clean` 実行時は Linux/macOS の前提条件チェック（cmake/ninja等）をスキップし直接 `installer.py` へ委譲。ヘッダのUsageコメントに主要フラグを列挙 |
| 日本語Windows（cp932/cp1252）耐性 | `6abbbe2`/`378bc60a`: DXH貸出PC（日本語Windows）実機で `sf upgrade` がコミット題名の全角ダッシュでクラッシュ。①subprocess捕捉のデコードを UTF-8 明示+`errors="replace"` に統一（git は UTF-8 を出力する）②`sf` 起動時に stdout/stderr を `errors="replace"` 化し、コンソールに表示できない文字は `?` 表示に（全コマンド共通の恒久保護）。退行テストとして日本語+全角ダッシュのコミット題名フィクスチャを CI 4脚で常時実行 |

**ドキュメント（本コミット）:** `docs/guides/upgrading.md` 新設（Git初心者向けの `sf upgrade`
解説・衝突解決walkthrough・インストール/アンインストールのライフサイクル表・FAQ）、
`docs/commands/sf-upgrade.md` 新設（コマンドリファレンス）、`docs/commands/README.md` /
`docs/getting-started.md` / `README.md` / `.mkdocs/mkdocs.yml` を更新。

## 3. ファームウェアの変更点（`firmware/vehicle`, `v2026.07.1` 以降）

> **本ドラフト作成時の依頼メモには「ファームウェアはワークショップ再ビルド＋起動チャイム
> 以外は無変更」という前提があったが、`git log v2026.07.1..main -- firmware/vehicle` を
> 実際に確認したところ誤りだった。** 制御則のデフォルト値変更が複数含まれる
> （§3.1〜§3.2）。以下は実際のコミットとソース (`params.cpp`) の値を照合した内容。

### 3.1 既定挙動が変わるもの

| 変更 | コミット | 内容 |
|------|---------|------|
| ロールレートループ再チューニング | `1785143`（`544c6ac`を上書き） | `rate.roll.kp`: `9.759795e-4` → `1.0e-3`、`rate.roll.td`: `0.01` → `0.001`（パイロット手動調整、実質D項オフに近いPI）。7/17スタディ値(`td=0.02`,`kp`×1.3)はセッション間で汎化せず不採用、7/18に手動再調整した値が最終 |
| `rate.roll.td` リリース既定値 | 本コミット | `rate.roll.td`: `0.001` → **`0.002`**（2026-07-20 パイロット指示、v2026.07.2 の最終既定値。kp/ti 変更なし）。SIL 39シナリオの A/B で変更前後の pass/fail 集合一致（31 PASS / 8 FAIL は既知の既存失敗）を確認。**反映方法は書き込み方式で異なる: full bin（GUIフラッシャ）なら NVS ごと消えるので自動適用、`sf flash` 更新なら `param reset` → `param save` が必要**（NVS 保存値が新既定値より優先されるため。手順書: `docs/contributing/release-workflow.md` §2「既存機体への反映」） |
| ヨーミキサー kappa の実測値補正 | `0ae4dea` | ミキサー定数 `KAPPA`: `9.71e-3` → `6.12e-3`（2026-07-15計測値）。`rate.yaw.kp`: `1.901691e-3` → `1.198594e-3`（物理ゲイン一定を保つよう再スケール）。新規パラメータ `rate.yaw.max_torque`（既定 `1.83e-3` Nm）を追加。NT金沢での突発ヨー回転（2026-06-27）の対策 |

### 3.2 新規実装だが既定は無変更（オプトインのみ）

| 変更 | コミット | 内容 |
|------|---------|------|
| 高度加速度DOB（外乱オブザーバ） | `42c984c` 実装 → `ffbdae5`/`ebe45c5` で既定 `fc=1.5`/`ti_hover=1.5` に一時昇格 → `0a3d8a3` で同日中に既定を撤回 | 現在の既定値は `altitude.dob.fc=0.0`（無効）、`altitude.vel.ti_hover=2.5`（no-op）— **`v2026.07.1` と同じ既定挙動**。実飛行で−67%の高度ばらつき改善を確認済みだが、機体ごとのオプトイン（`param set altitude.dob.fc 1.5` 等）として提供する運用に確定 |
| 高度vel-loop位相スケジューリングTi | `16a7e9b` | 上記と同じ `altitude.vel.ti_hover` パラメータの土台実装。既定no-op |

### 3.3 その他の追加機能

| 変更 | コミット | 内容 |
|------|---------|------|
| `param save_one`/`has_saved` + ユーザーLEDオーバーライド | `91a0894` | 単一パラメータのNVS保存API、およびCLIから任意色でLEDを上書きするコマンド系（ワークショップL0移行の地ならし） |
| モータスイープ電流試験・バッテリー電流テレメトリ | `f4592c8` | `sf` 側にモータ電流スイープ試験コマンドを追加、バッテリー電流をテレメトリに追加 |

### 3.4 vpython シミュレータのホバートリム修正

| 変更 | 内容 |
|------|------|
| スティック中立で上昇し続ける問題の根治 | 2026-07-15 の同定でモータ実測値が制御側逆モデルにだけ反映され、プラント（`core/motors.py`）は旧 `Ct=1.00e-8` のままだったため、中立で重力の約1.18倍の推力（+1.74 m/s²）が出ていた（DXH Day 2 の貸出PCで発見）。プラント `Ct` を実測 `6.7e-9` に更新（プラント κ=6.12e-3 がファーム 0ae4dea と一致）し、逆モデル/アロケータ κ をプラントから導出する構造に変更 — 以後の同定更新でトリムずれが構造的に再発しない。平衡は数値検証で厳密成立（総推力/重力=1.000000、10秒維持でドリフト0） |

### 3.5 ワークショップファーム（`firmware/workshop`）

| 変更 | コミット | 内容 |
|------|---------|------|
| vehicleコンポーネントベースへ全面リビルド | `996ff5e` | レイヤ命名の旧実装から、現行 `firmware/vehicle` のコンポーネントを流用する構成へ刷新 |
| 起動チャイム | `3b2a913` | workshop ファーム専用の起動音を追加: 授業開始チャイム風（ウェストミンスターの鐘・第1フレーズ、E5→C5→D5→G4、約2.7秒）。vehicle の起動音（C5→E5→G5）は不変で、耳でどちらのファームか判別できる |

依頼メモが挙げていた「ワークショップ再ビルド＋起動チャイム」自体は正しく含まれるが、
それ「以外は無変更」という部分は誤りだった。§3.1のロール/ヨーは既定挙動そのものが変わる、
§3.3はvehicle本体への機能追加。

## 4. 変更なし・確認事項

- `firmware/controller`（本体ロジック）: 変更コミットなし。`9304b7d` は依存バージョン
  ピン留めのみでビルド成果物の挙動は変わらない見込み（要: `sf build controller` の
  バイナリサイズ・動作確認）
- `firmware/common`: 変更コミットなし

## 5. リリースアセット

| アセット | 状態 |
|---------|------|
| `stampfly_vehicle_<tag>_full.bin` / `_app.bin` | 既存（§2の変更を含む） |
| `stampfly_controller_<tag>_full.bin` / `_app.bin` | 既存（挙動変化なし） |
| `StampFlyFlasher_<tag>_windows-x64.exe` | 既存 |
| `StampFlyFlasher_<tag>_macos-arm64.zip` / `_macos-x64.zip` | 既存 |
| `StampFlyFlasher_<tag>_linux-x64` | **新規**（拡張子なし、`chmod +x` 必要） |
| `StampFlySetup_<tag>_windows-x64.exe` | **新規** |
| `StampFlySetup_<tag>_macos-arm64.zip` | **新規** |
| `StampFlySetup_<tag>_macos-x64.zip` | **新規** |
| `StampFlySetup_<tag>_linux-x64` | **新規**（拡張子なし、`chmod +x` 必要） |
| `SHA256SUMS.txt` | 既存（全アセット対象） |

リリース本文のテンプレートは `.github/workflows/release.yml` の
`Create GitHub Release` ステップ（`body:` ブロック）に既に更新済み
（Linux アセットと `sf flasher install` への言及を含む）。タグ作成時はこのテンプレートが
そのまま使われる。

## 6. リリース前チェックリスト（`versioning.md` §5 対応）

| # | 項目 | 状態 |
|---|------|------|
| 1 | SIL退行テストPASS | 各コミットのコミットメッセージに記載の結果を参照（例: `0ae4dea`は31/39 PASS、8件は既存インフラ起因で無関係）。本ドラフト作成時点で改めての一括実行は**未実施** |
| 2 | `sf build vehicle` / `sf build controller` ローカルビルド成功 | `9304b7d` で controller 1094.9KB、`91a0894` で vehicle 1118.8KB を確認済み（コミット時点） |
| 3 | 前回リリースからの変更点整理 | 本ドキュメント |
| 4-5 | タグ作成・push | 未実施（P4完了後の想定） |
| 6 | Release workflow成功確認 | `feature/flasher-native-install` での `workflow_dispatch` 実行（run 29662024927、2026-07-19）で**全6ジョブ緑を確認済み**: build(vehicle)/build(controller)/フラッシャ4レッグ（インストールスモーク・Windows `uninstall.cmd` 経路含む）。tag push 時の Release 発行ジョブのみ未実行（タグでのみ走る設計） |
| 7 | Release notes加筆 | 本ドキュメントがその元原稿 |

## 7. Next steps

- `feature/flasher-native-install` を `main` にマージ
- `git tag v2026.07.2` → push → Release workflow のグリーンを確認
- 本ドキュメント§1・§2・§3の要点を GitHub Release の自動生成ノートに追記
- `docs/contributing/versioning.md` §6「カリキュラム互換表」に `v2026.07.2` の行を追加
  （本コミットでは未実施 — タグ作成後に確定情報で追記する）
