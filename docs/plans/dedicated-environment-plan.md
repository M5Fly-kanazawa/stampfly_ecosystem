# 専用環境（固定 Python + 専用 ESP-IDF）への移行計画

作成: 2026-09-11。

発端: 講習で Python の版違いに起因する問題が頻発している（PATH 先頭の Python の版から
仮想環境名が決まる `export.bat` の仕様、pyenv-win の global と仮想環境の版の不一致、
3.13 の混入、公式 ESP-IDF インストーラが同梱する組み込み用 Python に tkinter が無い、等）。
`setup_env.bat` / `setup_env.sh` には探索の回避策が何段も積まれているが、参加者の PC の
状態は無限にあり、探索方式では終わらない。

方針の前提（2026-09-11 ユーザー確認）:

- 講習は事前インストールが前提で、会場でインストールさせる場合も待ち時間は他の話で
  埋める運用のため、**時間は制約ではなく、容量だけが制約**。
- 既にインストール済みの人は **`sf upgrade` 1 回で専用環境へ移行**できること。
- 新規インストールの既定は専用構成。開発者向けに既存 ESP-IDF を使う経路は残す。

## 0. 要旨

| 観点 | 内容 |
|------|------|
| 方針 | ecosystem 専用の **Python・ESP-IDF・ツール置き場・仮想環境を 1 つのフォルダに自己完結**させ、参加者の PC にある Python や ESP-IDF に一切依存しない。`setup_env` は設定ファイルに記録したパスを無条件に使う（探索をしない） |
| 専用 Python | python-build-standalone（以下 PBS。管理者権限なしで展開できる CPython 配布物）の `install_only` 版 3.12.14（リリース 20260901）を固定。tkinter・venv・pip・SSL を同梱。Windows 版は VC ランタイム DLL も同梱（本計画作成時に配布物の中身と macOS 版の実動作を確認済み） |
| 専用フォルダ | Windows `C:\StampFly`（ESP-IDF は非 ASCII・空白入りパスを嫌うため `%LOCALAPPDATA%` は使わない）、macOS/Linux `~/.stampfly`。環境変数 `SF_HOME` で上書き可 |
| 既存環境 | 触らない。`sf upgrade` が移行を提案し、承諾すれば専用環境を横に作って切り替える。旧環境の削除は手動（手順を文書化） |
| 容量 | 約 4〜6 GB（Python 0.05〜0.15 GB、ESP-IDF 0.65 GB、ツール 1.5 GB 前後、仮想環境 1.5〜3.4 GB）。公式 ESP-IDF がある PC ではその分が二重になる |
| 成果 | Python の版に起因する不具合が構造的に消える。tkinter が常にあるため matplotlib のウィンドウ表示は Tk で成立し、PyQt6 自動導入（209dd05f）は保険として残るだけになる。アンインストールはフォルダ 1 つの削除で完結 |

## 1. 現状の事実（調査 2026-09-11）

| 項目 | 現状 |
|------|------|
| `.sf/config.toml` | `installer.py` の `_save_config()` が `[esp_idf] path / version`、`[project] default_target` の 3 キーだけを書く。読み手は `setup_env.bat`（`findstr /b "path"`）、`setup_env.sh`（`grep '^path = '`）、`installer.py` の `clean()` / `uninstall()`（行走査）。TOML ライブラリはどこにも使われていない |
| `sf` 実行時の ESP-IDF 探索 | `lib/sfcli/utils/platform.py::esp_idf_path()` と `paths.py::esp_idf()` は設定ファイルを読まず、`IDF_PATH` 環境変数と既定パスだけを見る。`sf build/flash` は `espidf.idf_command()` で `sys.executable` から `idf.py` を起動し、環境は `setup_env` 済みであることを前提にする |
| ESP-IDF の取得 | `ESPIDFInstaller.install()` が `~/esp/esp-idf` に `git clone --branch v5.5.2 --depth 1` → `submodule update --init --depth 1 --recursive` → ESP-IDF の `install.bat esp32s3` / `install.sh esp32s3` を実行。**`IDF_TOOLS_PATH` は設定しない**（ESP-IDF 既定の `C:\Espressif` / `~/.espressif` を後から推測して仮想環境を探す） |
| Python の扱い | `_find_system_python_dir()` 等（約 700 行）が参加者の Python を探し、無ければ winget/brew/apt で自動導入を提案。目的は ESP-IDF の `install.bat/.sh` が名前で呼ぶ `python`/`python3` を解決させること。GUI インストーラにも同じ探索の複製があり、`scripts/test_gui_installer_parity.py` が乖離を監視 |
| 仮想環境 | `<IDF_TOOLS_PATH>/python_env/idf5.5_py3.X_env`。名前が Python の版に依存するため、PATH 先頭の Python が変わると `export.bat` が別名を探して失敗する |
| 安定契約 | `installer.py` は標準ライブラリのみ、`Installer.run()/uninstall()/clean()` の互換維持、`Step N/4:` ヘッダを GUI が解析（4 段固定）。CI: `tools/ci/check_installer_gui.py`、`scripts/test_gui_installer_parity.py`、`tools/ci/check_upgrade.py` |
| `sf upgrade` | 上流で `lib/sfcli` が変わっていれば取得したコードへ自己引き継ぎ（hop）→ プレビュー・マージ → 依存関係再同期 → sdkconfig 退避 → フラッシャ更新の提案。非対話（stdin EOF）では確認は既定値に落ちる |

## 2. 目標構成

### フォルダ配置（`SF_HOME`）

```
SF_HOME/                      Windows: C:\StampFly     macOS/Linux: ~/.stampfly
├── python/                   PBS を展開（Windows: python.exe、Unix: bin/python3）
├── esp-idf/                  専用の ESP-IDF v5.5.2（--depth 1 + submodules）
├── espressif/                IDF_TOOLS_PATH（tools/ dist/ python_env/idf5.5_py3.12_env）
├── downloads/                取得した配布物（検証後も残す。再実行時の再取得を省く）
└── manifest.json             導入済みの Python/ESP-IDF の版・配布物名・SHA-256・作成日時
```

### 設定ファイル `.sf/config.toml`（v2）

```toml
[esp_idf]
path = "C:\StampFly\esp-idf"
version = "v5.5.2"

[env]
kind = "dedicated"                      # dedicated | legacy
root = "C:\StampFly"
python = "C:\StampFly\python\python.exe"
python_dir = "C:\StampFly\python"       # PATH の先頭に置くフォルダ（Unix は .../python/bin）
tools_path = "C:\StampFly\espressif"

[project]
default_target = "vehicle"
```

読み手は従来どおり行解析のため、キー名は行頭一致で衝突しないものにする（`path`、`python`、
`python_dir`、`tools_path`、`kind`、`root`）。`.bat` では `findstr /r /b /c:"python *="` のように
`=` まで含めて一致させ、`python` が `python_dir` に誤一致しないようにする。
`[env]` が無い設定ファイル（v1）は「未移行」とみなす。

### 専用 Python の固定値（`scripts/installer.py` に定数として記述）

| 対象 | 配布物 | SHA-256（公式 SHA256SUMS より、2026-09-11 取得） | 大きさ |
|------|--------|------|------|
| Windows x64 | `cpython-3.12.14+20260901-x86_64-pc-windows-msvc-install_only.tar.gz` | `e90c1b6419da3bd812dd73bb3de40287a21abf153438147639ec5e20375ea93f` | 46 MB |
| Windows ARM64 | `cpython-3.12.14+20260901-aarch64-pc-windows-msvc-install_only.tar.gz` | `4e852236277eb8f7105cbe0f5adf45592f521af238bc0f700c351856e2c2e41a` | 43 MB |
| macOS Apple Silicon | `cpython-3.12.14+20260901-aarch64-apple-darwin-install_only.tar.gz` | `3ee3ee547cedfeb7c2b16b2b7156039f7b470bb8f857e226fd3d2eb11db83c76` | 25 MB |
| macOS Intel | `cpython-3.12.14+20260901-x86_64-apple-darwin-install_only.tar.gz` | `2e31b23f3f1319f707d0e620b48847a0046577541d357276821f9f1b5492e0ba` | 25 MB |
| Linux x86_64 | `cpython-3.12.14+20260901-x86_64-unknown-linux-gnu-install_only.tar.gz` | `936c246dfdbbfa7cb22dd01814a21f582a892689fae96b06071a5e433baffa22` | 111 MB |
| Linux aarch64 | `cpython-3.12.14+20260901-aarch64-unknown-linux-gnu-install_only.tar.gz` | `b61b856c3e1a4fc65b8f6e6b0495ef975dd0924f90c59f3ea61b38a079173b84` | 84 MB |

取得元: `https://github.com/astral-sh/python-build-standalone/releases/download/20260901/<配布物名>`
（URL 中の `+` は `%2B` に符号化する）。展開後は `python/` フォルダ 1 つ。
確認済み事項: Windows 版に `DLLs/_tkinter.pyd`、`tcl86t.dll`、`tk86t.dll`、`tcl/tcl8.6/init.tcl`、
`vcruntime140.dll` を同梱。macOS 版（framework ビルドではない）で `tkinter.Tk()`、`venv`、
`pip install matplotlib`、matplotlib の macosx バックエンドによる図の生成が動作。

### 環境の読み込み（`setup_env.bat` / `setup_env.sh`）

`[env] kind = "dedicated"` なら: `python_dir` を PATH 先頭に置く → `IDF_TOOLS_PATH` と `IDF_PATH`
を設定 → `IDF_PYTHON_ENV_PATH` を消す → ESP-IDF の `export.bat/.sh` を呼ぶ → `IDF_PYTHON_ENV_PATH`
の実在で成功を検証。**Python の探索・版照合は一切行わない。**
`kind` が `legacy` または `[env]` 無しなら従来の探索処理をそのまま使う（後方互換）。

## 3. 変更一覧

| 領域 | ファイル | 変更 |
|------|---------|------|
| 起動スクリプト | `install.bat`、`install.sh` | 参加者の Python を探さない。`SF_HOME/python` が無ければ `curl` + `tar`（Windows 10 1803 以降は両方標準装備）で PBS を取得・SHA-256 検証・展開し、その Python で `scripts/installer.py` を起動する。`--use-existing-idf` / `--idf-path` 指定時は従来どおり参加者の Python で起動 |
| インストーラ本体 | `scripts/installer.py` | 定数（PBS 版・配布物・SHA-256）、`sf_home_default()`、`provision_private_python(root)`（manifest 照合 → 取得 → 検証 → 一時フォルダに展開 → 改名 → `import tkinter, venv, pip, ssl` の起動確認 → manifest 更新。冪等）、`ESPIDFInstaller.install()` の取得先・`IDF_TOOLS_PATH`・PATH 先頭の指定、`_find_idf_python()` の `tools_path` 引数、`_save_config()` v2、`Installer.run(dedicated=True)` 既定化と `--use-existing-idf`（`--idf-path` は legacy を含意）、`uninstall(--purge)` で `SF_HOME` ごと削除。Step ヘッダは 4 段のまま（Step 1/4 の題名のみ「Python + ESP-IDF」に変更） |
| 環境スクリプト | `setup_env.bat`、`setup_env.sh` | 設定ファイル優先（上記）。従来処理は legacy 用に温存 |
| sf CLI | `lib/sfcli/utils/paths.py`、`platform.py` | `.sf/config.toml` の `path` を最初に見る簡易読み取りを追加（`read_config_value(section, key)`） |
| | `lib/sfcli/commands/doctor.py` | 「Environment」節: kind / root / 実行中 Python の `sys.base_prefix` が `root/python` か / `IDF_TOOLS_PATH` / ESP-IDF 版。legacy なら「`sf upgrade` で移行できる」と案内 |
| | `lib/sfcli/commands/upgrade.py` | 移行の提案 `_offer_dedicated_migration()`: `[env] kind` が `dedicated` でなければ実行（`--migrate` で確認なし、`--no-migrate` で省略、対話では既定 Y、非対話は省略）。承諾時は `scripts/installer.py --non-interactive --dedicated --no-flasher`（SILS ツールチェーンは既存の冪等処理に任せる）をサブプロセスで実行し、成功したら旧仮想環境での依存関係再同期を省略して「ターミナルを開き直す」旨を表示。辞退時は `kind = "legacy"` を設定に書き、以後は尋ねない |
| GUI インストーラ | `tools/installer_gui/stampfly_installer.py` | `Installer.run(dedicated=True)` を既定に。同梱 Python（PyInstaller）で `installer.py` を同一プロセス実行するため参加者の Python は不要になるが、参加者 Python の探索コードと定数は legacy 用に残し、`scripts/test_gui_installer_parity.py` を維持。新規 stdlib import（`hashlib`、`tarfile`、`urllib.request`、`platform` 等）を hidden-import 契約に追加 |
| テスト | `scripts/test_installer_dedicated.py`（新規） | 配布物選択（OS/CPU）、manifest 照合の冪等性（取得を偽装）、設定 v2 の書き出しと `.sh`/`.py` 読み取りの往復、`sf_home_default()` の分岐、小さな合成 tar.gz の展開、`sf upgrade` の移行提案の分岐（サブプロセス偽装） |
| 文書 | `docs/setup/{README,windows,macos,linux}.md`、`docs/guides/upgrading.md`、`docs/guides/troubleshooting.md`、`docs/commands/sf-doctor.md`、`docs/commands/sf-upgrade.md`、`docs/guides/gui-installer.md`、`README.md` | 前提条件（参加者の Python は不要。Windows 10 1803 以降 / macOS / Linux と git、`curl`、`tar`）、配置、容量、移行手順、旧環境の手動削除、`SF_HOME` |

## 4. 段階と検収

| Phase | 内容 | 検収 |
|-------|------|------|
| A | 本計画書 | コミット |
| B | `scripts/installer.py` の中核（定数・専用 Python 取得・専用配置・設定 v2・manifest・フラグ・`--purge`）と単体テスト | `pytest scripts/test_installer_dedicated.py scripts/test_gui_installer_parity.py`、`python tools/ci/check_installer_gui.py` |
| C | `install.sh` / `install.bat` の起動経路、`setup_env.sh` / `setup_env.bat` の設定優先化 | `.sh` は本 Mac で実行。`.bat` は机上レビュー（ASCII のみ・CRLF・`findstr` の一致規則）と `.sh` との対応表 |
| D | sf CLI（設定読み取り、`sf doctor` の Environment 節、`sf upgrade` の移行提案） | `python tools/ci/check_upgrade.py`（非対話では移行を提案しないこと）、単体テスト |
| E | GUI インストーラ、CI、文書 | `python tools/ci/check_installer_gui.py`、parity テスト |
| F | **本 Mac での実機相当試験**: (1) リポジトリを作業用に複製し `HOME` と `SF_HOME` を一時フォルダに向けて `./install.sh --non-interactive --minimal --no-flasher` を実行 → `source setup_env.sh` → `sf doctor`（dedicated、base_prefix が専用 Python）→ `sf build vehicle`（実ビルドでツールチェーンを検証）→ `sf log viz`（`Plot window backend: macosx`）。(2) v1 設定の複製に対し、ローカルの bare リポジトリを origin に見立てて `sf upgrade --migrate` → 専用環境が作られ設定が v2 になること | 結果を本計画書の進捗表に記録 |
| G | Windows 実機（別途）: `install.bat` 新規、既存環境からの `sf upgrade`、`sf log viz` の Tk ウィンドウ | 未実施のうちは本計画書に「未検証」と明記 |

## 5. 決定事項と理由

| 決定 | 理由 |
|------|------|
| Python は PBS を全 OS 共通で使う | 管理者権限不要・レジストリや PATH を汚さない・tar.gz 1 つの展開で完結・6 対象すべてに配布物がある。python.org の Windows インストーラも候補だったが、レジストリ登録と「アプリと機能」への出現が参加者を混乱させる |
| Windows の置き場は `C:\StampFly` | ESP-IDF は非 ASCII・空白入りパスを公式に非推奨。日本語ユーザー名では `%LOCALAPPDATA%` が非 ASCII になる。`C:\` 直下へのフォルダ作成は標準ユーザーでも可能（Espressif も `C:\Espressif` を使う）。作成できない場合は `%LOCALAPPDATA%\StampFly` に落とし、非 ASCII なら警告 |
| ツール置き場も専用にする | `C:\Espressif` を共用すると、公式インストーラの仮想環境名（`idf5.5_py3.11_env` 等）と衝突・再利用が起きうる。容量の二重化（約 1.5 GB）は許容と確認済み |
| 旧環境は削除しない | どれが ecosystem 由来かを確実には判別できない。手動削除の手順を文書化する |
| 辞退時は `kind = "legacy"` を記録 | 既存環境を意図して使う開発者に毎回尋ねない。`sf upgrade --migrate` でいつでも移行可能 |
| Step ヘッダは 4 段固定 | GUI の解析契約。Step 1 の中身が「Python + ESP-IDF」になるだけ |

## 6. 進捗

| 日付 | 内容 |
|------|------|
| 2026-09-11 | 計画作成。PBS の Windows 版の中身と macOS 版の動作を確認。SHA-256 を公式一覧から取得し手元の 2 配布物と一致確認 |
