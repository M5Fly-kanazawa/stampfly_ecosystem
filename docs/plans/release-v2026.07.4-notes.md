# v2026.07.4 リリースノート（ドラフト）

`docs/contributing/versioning.md` §5「リリース手順」の手順3（前回リリースからの変更点整理）
に対応するドラフト。**`v2026.07.4` タグはまだ作成されていない**（本ドキュメント作成時点で
`origin/main` の最新コミットは `d0d87f2c`）。

対象範囲: `v2026.07.3(cf1d084e)..origin/main` の11コミット。ファームウェア
（`firmware/vehicle`, `firmware/controller`, `firmware/common`）は変更なし
（`v2026.07.3` と同一挙動）。全て StampFly Setup / インストーラ / シミュレータ関連。

> **ユーザーへの要点:** `v2026.07.3` の `StampFlySetup` exe は、ESP-IDF v5.5 + Windows +
> Python 3.10（`sf doctor` が推奨する組み合わせそのもの）で **Step 3/4（sfcli インストール）
> が必ず失敗する** バグ（jinja2 制約衝突、節1の#5）を含む。該当条件に当てはまるユーザーは
> `v2026.07.4` の StampFlySetup への更新を強く推奨する。

## 1. Windows GUIインストーラの一連の修正（講習用実機での連続障害対応）

2026-07-20、講習用 Windows 実機で StampFly Setup を使ったところ、以下の障害が連鎖して発覚し、
その場で5コミットに分けて根治した。**Windows 上での StampFly Setup 利用者に影響が大きい変更。**

| # | 症状 | 原因 | 対策 | コミット |
|---|------|------|------|---------|
| 1 | Step 1/4（ESP-IDF）が `exit status 9009`（Windowsの「コマンドが見つからない」）で失敗 | ESP-IDFの `install.bat` が `python` をPATH経由で呼ぶが、GUI版インストーラは実行ファイル自身（`StampFlySetup.exe`）が `sys.executable` になり、CLI版と違って隣に本物の `python.exe` が存在しない | システムPython探索を新設（pyenv-win、`%LOCALAPPDATA%\Programs\Python`、`C:\Python*`、scoop、conda等）し、見つかった場合はPATHの先頭に追加。見つからない場合は winget での導入手順を案内 | `b9616a30` |
| 2 | StampFly Terminal 起動時に `'em' is not recognized`、`'yedexpansion' is not recognized` 等のエラーが連続 | `setup_env.bat`/`install.bat` がLF改行のみでコミットされており（`.gitattributes` 未整備）、Windows でも LF のまま checkout されていた。`cmd.exe` はLFのみの.batを誤読し各行先頭の1文字を欠落させる（`REM`→`EM`等） | `.gitattributes` を新設し `*.bat`/`*.cmd`/`*.ps1` は `eol=crlf` を強制。既存クローンにも再チェックアウトさせるため両ファイルにヘッダコメントを追加してblobを変更 | `143bc49b` |
| 3 | CRLF修正後も起動時に日本語コメントの文字化け断片が「コマンドとして認識されません」と表示 | 日本語Windows（cp932コンソール）では `cmd.exe` が.batのUTF-8バイト列を生バイトとして解釈し、一部が `&`/`|`/`<`/`>` 等の区切り文字に化けてREM行の一部がコマンドとして実行される | `setup_env.bat`/`install.bat` のコメントを全てASCII化（生成済み `uninstall.cmd` と同じ、この repo の「バイリンガルコメント原則」への意図的な例外）。`tools/ci/check_installer_gui.py` に4件目のチェックを追加し、コミット済み `.bat`/`.cmd`/`.ps1` が純ASCIIであることをCIの4OS全レグで検証 | `3bd845da` |
| 4 | 上記1の対策後も、`py` ランチャーや all-users版（`C:\Program Files\Python3x`）しか入っていない典型的な講習用PCでは依然 9009 の恐れ。サブプロセス出力がフリーズしたように見える。gitが無いと生の `FileNotFoundError` で落ちる | Python探索が `py -3` と all-users install を見ていなかった。`--windowed` ビルドではESP-IDFクローン/インストールの出力が不可視だった。git欠如の例外処理が無かった | `py -3` 優先探索・all-users パス追加。`_stream_subprocess()` で `git clone --progress` 等の出力をリアルタイムにGUIログへストリーミング（`CREATE_NO_WINDOW` でコンソールのちらつきを抑制）。git欠如時はバイリンガルな案内メッセージ（git-scm.comへの誘導）を表示 | `42cd0e85` |
| 5 | **Step 3/4（sfcli インストール）が ESP-IDF v5.5 + Windows + Python 3.10 の組み合わせで必ず exit 1 で失敗する構造的バグ** | `_run_in_idf_env()` が pip install に ESP-IDF の制約ファイルを注入するが、`espidf.constraints.v5.5.txt` は Windows + Python<3.11 で `jinja2<3.1` を固定しており、本リポジトリの `jinja2>=3.1.0` 指定と衝突し `ResolutionImpossible` になっていた | `requirements.txt`/`pyproject.toml` の jinja2 下限を `>=3.0` に緩和（jinja2は未実装の設計ドキュメントで言及されているのみで実コードは未使用）。`info`/`success`/`warn`/`error`/`header` に `flush=True` を追加しGUIログの順序を保証。pipの出力もストリーミング対象に追加 | `ea6e627f` |

**影響評価:** #5（jinja2衝突）は特に深刻で、ESP-IDF v5.5 と Windows + Python 3.10 の組み合わせ
（`v2026.07.3` 時点で `sf doctor` が推奨する構成そのもの）では StampFly Setup の Step 3/4 が
**発生条件下では必ず**失敗する状態だった。`v2026.07.3` リリース後に発覚したため、
`v2026.07.4` での修正が必須。

**検証（各コミットのコミットメッセージより）:**
- `python tools/installer_gui/stampfly_installer.py --selftest`: 各段階でPASS
- `python tools/ci/check_installer_gui.py`: 4/4 PASS（#3以降）
- 実際に障害が発生したPC上で `python scripts/installer.py --non-interactive` を再実行し
  exit 0・全4ステップ完了、sfcli / StampFly Terminal / StampFly Flasher のショートカット作成まで
  確認（`ea6e627f` 時点）

## 2. Python バージョン対応方針の確定（3.10〜3.12）

**ユーザー影響:** インストーラが要求・推奨するPythonバージョンの方針が明確化された。

| 変更 | 内容 |
|------|------|
| バージョン選好の明確化 | インストーラのPython探索ロジックを刷新し、3.10〜3.12（新しい順）を優先採用。3.13+は警告付きで許容していたが、実利用者からの3.13起因の不具合報告を受け、**本リリースで3.13+は不採用に変更**（3.12自動導入の提案へ誘導）。3.10未満は明示的に拒否（ESP-IDFのvenvに古いPythonを混入させない） |
| 自動インストール提案 | 適切なPythonが見つからない場合、同意ベースでの自動導入を提案: Windowsは`winget`、macOSは`brew`、Linuxは`apt`/`dnf`/`pacman`（対話環境ではy/n確認＋sudo、非対話環境では案内のみ）。新規 `--auto-install-python` フラグ。GUI側にも「Python 3.12 を自動インストール」ボタンを追加（ワーカースレッド実行＋再チェック、Linuxはコピー可能なコマンド表示） |
| 依存関係修正 | `from __future__ import annotations` を追加し、3.8/3.9でのバージョンチェック自体が `list[Path]` 注釈の `TypeError` で到達不能だった不具合を修正 |
| `pyproject.toml` | `requires-python` を `">=3.10,<3.13"` に確定 |
| ドキュメント | 「Python 3.8+」表記を「3.10+（3.12推奨）」に、setup/getting-started/workshopの各ガイドで統一。`CLAUDE.md`・`simulator/genesis/README.md` の Genesis パス誤記（`simulator/sandbox/genesis_sim` → `simulator/genesis`）も修正 |

（コミット: `560b211a`, `d0d87f2c`）

## 3. 仮想環境マネージャ対応（pyenv/uv/asdf/conda）

**ユーザー影響:** pyenv・uv・asdf・condaでPythonを管理している開発者・学生の環境でも
インストーラが正しく動作するようになった。加えて `sf doctor` に新しい診断項目が追加された。

| 変更 | 内容 |
|------|------|
| 候補探索の拡大 | Python探索候補に pyenv（Unix）、uv（全OS、`uv python find` 含む）、asdf（Unix）、より広いconda設置場所を追加 |
| venv経由シードの解決 | `_resolve_venv_seed()` が `pyvenv.cfg` の `home=` を辿って実体のベースインタプリタを解決（1ホップまで）。これにより ESP-IDF 用venvが別のvenvの中にネストされることを防ぐ |
| 候補の安定性ランキング | バージョン帯域内で「素の実体インストール > pyenv/uv/asdf > conda > venv解決済み」の順に安定性でランク付け。最終順序は「帯域 > 安定性 > 新しいバージョン」 |
| 環境変数のサニタイズ統一 | `_sanitize_activated_env()` が `VIRTUAL_ENV`/`PYTHONHOME`/`PYTHONPATH`/`CONDA_PREFIX`/`CONDA_DEFAULT_ENV`/`PYENV_VERSION` を全ての子プロセスから除去（従来は `PIP_*` 系の場当たり的な除去のみだった） |
| 壊死venv（dead venv）の検出・再作成 | Step 2/4で、専用venvのベースインタプリタが消えている場合（例: pyenvでアンインストール済み）はバイリンガルで通知し、venvを削除して自動再作成 |
| `sf doctor` 新チェック | ESP-IDF venv のベースインタプリタが現存するか診断する項目を追加（診断のみ・自動削除はしない） |

（コミット: `d0d87f2c`）

**検証（コミットメッセージより）:** GUI `--selftest` exit 0、`check_installer_gui.py` 4/4
PASS、venvシード解決・安定性ランキング・壊死venv検出・3.13拒否をカバーする単体テスト9件PASS、
既存の健全なvenvに対して `installer.py --non-interactive` を実行しても `pyvenv.cfg` の
mtimeが変化しない（誤検知なし）ことを確認、`sf doctor` が新チェックで `[OK] ESP-IDF venv
(Python 3.10)` を報告。

## 4. シミュレータ: Windows でのアセット参照障害の根治（git symlink 廃止）

**ユーザー影響:** Windows で `sf sim run` / `sf sim run genesis` を使う場合に影響。

2026-07-20、講習用 Windows PC で `sf sim run` が
`OSError: [Errno 22] Invalid argument: ...\simulator\vpython\assets\meshes\stampfly_v1.stl`
で失敗（該当フォルダはExplorerからも開けなかった）。`simulator/vpython/assets` 等3箇所が
git symlink（mode 120000、`../shared/assets` への相対リンク）になっており、git symlinkは
Windows環境（git設定・Developer Modeの有無）によってテキストファイルとして展開されたり、
トラバース不能なreparse pointになったりして不安定であることが根本原因。

| 段階 | 対応 | コミット |
|------|------|---------|
| 応急 | vpython の実行系（`vpython_backend.py`）が symlink を経由せず `simulator/shared/assets` を直接参照するよう修正。テキストファイル化パターンのフォールバックは維持 | `07222da9` |
| 応急 | genesisの2エントリポイント（`run_genesis_sim.py`, `run_genesis_headless.py`）にも同じ対応を横展開 | `9170ea56` |
| 根治 | `simulator/vpython/assets`・`simulator/genesis/assets`・`simulator/sandbox/assets` の3つのgit symlinkを`git rm`で完全撤去。全アクセスコードを`simulator/shared/assets`直接参照に統一し、応急対応で追加したフォールバックも削除。誤って`simulator/assets/meshes`（存在しないパス）を指していた`split_stl.py`も併せて修正 | `05d9a5f9` |

（コミット: `07222da9`, `9170ea56`, `05d9a5f9`）

## 5. 検証状況まとめ

| 項目 | 状態 |
|------|------|
| このPC（macOS）での `installer.py --non-interactive` / GUI `--selftest` / `check_installer_gui.py` | 全PASS（各節の「検証」参照） |
| 実際に障害が発生したWindows PC上での再実行（節1の#5まで） | exit 0・全4ステップ完了を確認済み |
| macOS/Linux の新規コード（仮想環境マネージャ対応、壊死venv検出、pyenvのみの環境等） | ロジック検証のみ。**実機未検証** |
| 新品 Windows 機（Python/git 完全に無い状態）での StampFlySetup exe 実行 | **未実施** |
| Linux（GNOME/KDE 等のアプリ一覧からの起動）・Windows（日本語ユーザー名を含む実パスでの `.lnk` 起動）での StampFly Terminal | **未実施**（`v2026.07.3` から持ち越し） |

## 6. リリース前チェックリスト（`versioning.md` §5 対応）

| # | 項目 | 状態 |
|---|------|------|
| 1 | SILS退行テストPASS | 未実施（ファームウェア変更なしのため影響は限定的だが、§5手順通り実行を推奨） |
| 2 | `sf build vehicle` / `sf build controller` ローカルビルド成功 | 未実施（ファームウェアソースに変更なし） |
| 3 | 前回リリースからの変更点整理 | 本ドキュメント |
| 4-5 | タグ作成・push | 未実施（メイン側で実施） |
| 6 | Release workflow成功確認 | 未実施（タグ push 後に確認） |
| 7 | Release notes加筆 | 本ドキュメントがその元原稿 |

## 7. Next steps

- `git tag v2026.07.4` → push → Release workflow のグリーン・13アセット揃いを確認
- **最優先**: 節1の#5（jinja2衝突）はリリース済み `v2026.07.3` の StampFlySetup exe が
  抱える実害バグのため、**新品 Windows 機（Python/git 無し）での実 exe テスト**を優先的に
  実施すること
- macOS/Linux の新規コード（仮想環境マネージャ対応、壊死venv検出、pyenvのみの環境等）を
  実機で検証する
- Linux（GNOME/KDE 等のアプリ一覧からの起動）・Windows（日本語ユーザー名を含む実パスでの
  `.lnk` 起動）での StampFly Terminal 実機検証
- `docs/contributing/versioning.md` §6「カリキュラム互換表」に `v2026.07.4` の行を追加
  （本ドラフトでは追加済み。タグ作成後、リリース日等の確定情報に齟齬がないか最終確認）
