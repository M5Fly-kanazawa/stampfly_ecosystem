# v2026.07.6 リリースノート（ドラフト）

`docs/contributing/versioning.md` §5「リリース手順」の手順3（前回リリースからの変更点整理）
に対応するドラフト。**`v2026.07.6` タグはまだ作成されていない**（本ドキュメント作成時点で
`main` の最新コミットは `742c6ea7`）。

対象範囲: `v2026.07.5(562c1c70)..main` の7コミット。ファームウェア
（`firmware/vehicle`, `firmware/controller`, `firmware/common`）は**挙動変更なし**（変更は
`firmware/workshop/lessons/lesson_manifest.yaml` のレッスンマニフェストのみ。後述）。
全てインストーラ・開発環境（`setup_env` / sf CLI）の互換性修正。

> **ユーザーへの要点:** 本リリースはインストーラ・開発環境（`setup_env` / sf CLI）の互換性
> 修正が中心。特に Windows で pyenv-win を使っている環境や、選択中の Python バージョンが
> ESP-IDF venv と食い違っている環境で GUI インストーラが `exit code 9009` で失敗する不具合、
> および macOS/Windows で開発環境起動時（`setup_env`）に対応外の Python が誤って選ばれる
> 不具合を修正した。飛行・書き込みのみを行う利用者への影響は無い。

## 1. GUI インストーラ: シミュレータ依存の導入検証・one-shot 化（`24419eb0`）

**ユーザー影響:** StampFly Setup（GUI インストーラ）の一括インストールで、シミュレータ用
Python パッケージ（numpy/scipy/vpython 等）が実は入っていないのに「Installation Complete!」
と表示されてしまう不具合を修正。導入失敗時は具体的な修復コマンドが表示されるようになった。

| 変更 | 内容 |
|------|------|
| `_fix_setuptools()` の実行順変更 | Step 3/4 の冒頭に移動し、`setuptools<81` ピン（`pkg_resources`／vpython用）を requirements.txt 一括インストールより前に適用。ESP-IDF自身のインストールが setuptools を82+へ上げてしまう窓を閉じた |
| 導入後の import 検証 | 一括インストール後、ESP-IDF venv 内で numpy/scipy/matplotlib/pandas/yaml/serial（常時）、vpython/pygame/cv2（フルインストール時）を import 検証。欠けていれば個別に pip retry し、それでも欠けていれば完了バナー内に修復コマンド付きで明示表示 |
| `sim.py` の対話的自己修復 | vpython が sf 自身の環境に無い場合、対話端末では「sf 環境へ今すぐ導入しますか？ [Y/n]」を確認し、成功したらそのままシミュレータ起動へ継続。案内文言も `<sys.executable> -m pip install vpython` に統一（従来はシステムの `pip3 install` を案内しており、pyenv 側に誤導入させていた） |

**検証（コミットメッセージより）:** Windows上で `installer.py --non-interactive` が exit 0、
setuptools が依存導入前にピン済み、「All key packages verified」を確認。`sf sim list` OK、
ESP-IDF venv内で `import vpython` 成功。`--minimal` では vpython/pygame/cv2 が検証対象から
正しく除外されることも確認。

## 2. GUI インストーラ: ESP-IDF セットアップ・venv 選定を対応内 Python に誘導（`f4bc30a5`）

**ユーザー影響:** macOS で GUI インストーラが `idf5.5_py3.9_env` を誤って選び、sfcli の
`requires-python >=3.10,<3.13` を満たせず `pip install -e` が失敗する不具合を修正。

| 変更 | 内容 |
|------|------|
| `_find_idf_python()` の選定ロジック修正 | 辞書順ソート（`"idf5.5_py3.9_env"` が `"idf5.5_py3.12_env"` より文字列比較で後に来てしまう）をやめ、数値バージョンでソートしつつ対応範囲（3.10-3.12）内の venv を優先するよう変更 |
| `_env_with_python3_steering()`（新設） | macOS/Linux で ESP-IDF の `install.sh` を実行する環境変数として、検出した対応内 system Python を PATH 先頭に追加。ディレクトリに `python3` の裸名が無い場合や `python3` 自体が対応外（例: Homebrew の python3 → 3.14）の場合は一時的な `python3` シムを合成 |
| `_run_install_script()` | Unix 経路にもこの誘導 env を渡すよう変更（Windows の `install.bat` 経路は既存の `_clean_env_for_cmd()` で同様の誘導済み） |

**根本原因:** GUI 起動は素の PATH（pyenv/Homebrew 無し）で走るため、ESP-IDF 自身の
`tools/detect_python.sh` が最初に試す裸の `python3` 名が `/usr/bin/python3`（3.9.6）を解決し、
`idf5.5_py3.9_env` を作成してしまう。さらに辞書順ソートがその venv を既存の
`idf5.5_py3.12_env` より優先して選んでしまう、という独立した2つのバグが重なっていた。

**検証（コミットメッセージより）:** macOS上（py3.9の迷子venvが残っている状態を含む）で
`_find_idf_python()` が `idf5.5_py3.12_env/bin/python` を返すことを確認。
`_env_with_python3_steering()` の PATH 先頭が `python3 -> 3.12.12` に解決することを確認。
`./install.sh --no-flasher` が正しい venv を選択して完了し、sfcli の import 検証・
`sf --version` もOK。

## 3. sf CLI: 環境チェックの追加とレッスン `has_solution` フラグ（`0a881447`）

**ユーザー影響:** ビルド/書き込み前の開発環境チェックが強化され、開発環境を読み込まずに
`sf build`/`sf flash`/`sf monitor` を実行した場合に「`No module named 'click'`」のような
分かりにくいエラーの代わりに「`source setup_env.sh` を実行してください」等の明確な案内が
出るようになった。また、正解が存在しないワークショップ用レッスンで誤った警告が出なくなった。

| 変更 | 内容 |
|------|------|
| `verify_idf_env()`（新設） | idf.py/esptool 実行前に、`IDF_PYTHON_ENV_PATH` が設定されておりその venv が実在するか、`idf.py` が PATH 上で解決するか（Unix）を検証。失敗時は日英併記のガイダンスを表示 |
| 死んだコード削除 | 一度も呼ばれていなかった `_prepare_idf_env_unix()`/`_windows()` を削除（`env -i` にハードコードされたシステム PATH を渡しており、インストーラが直前に修正したのと同じ「誤った Python」失敗クラスを再現する潜在バグだった） |
| `build.py`/`flash.py`/`monitor.py` | `prepare_idf_env()` 直後に `verify_idf_env()` を呼び出すよう統一 |
| `lesson_manifest.yaml` に `has_solution` フィールド追加（既定 `true`） | レッスン90（`dxh_workshop`）を `has_solution: false` に設定。当日参加者が duty 値を書き換えるイベント用テンプレートであり、設計上「唯一の正解」が存在しないため |
| `doctor.py` | `has_solution: false` のレッスンでは `solution.cpp` 存在チェックをスキップ（`student.cpp` は引き続き必須）。「Lesson 90: solution.cpp MISSING」という偽の警告を解消 |
| `lesson.py` | `sf lesson solution`／`sf lesson switch --solution` が該当レッスンで「設計上、解答は存在しない」旨を案内するよう変更 |

**検証（コミットメッセージより）:** macOS上で `sf doctor` が「All checks passed!」（レッスン90
の警告消失）。`sf lesson solution 90`／`switch 90 --solution` は明確な日英メッセージで exit 1
（レッスン5の `--solution` は従来通り動作）。開発環境を読み込んでいないシェルでの
`sf build vehicle` は新しい案内メッセージで exit 1、開発環境読み込み後は end-to-end で成功。

## 4. `setup_env`: macOS/Windows で対応 Python への誘導を追加（`4e25e875`）

**ユーザー影響:** StampFly Terminal アプリの `.command` ランチャーが非対話シェルで
`setup_env.sh` を読み込む際、`.zshrc`/`.zprofile` が読まれず素の PATH のまま Python 3.9.6が
選ばれて存在しない `idf5.5_py3.9_env` を探しに行き失敗するのに、`setup_env.sh` は
「[OK] ready」と表示し続けていた不具合を修正。

| 変更 | 内容 |
|------|------|
| `setup_env.sh` | `export.sh` を読み込む前に `python3` が動作確認済み範囲（3.10-3.12）内かを検証。範囲外/未検出ならPATH・pyenv versions・Homebrewケグ・python.orgフレームワークを探索し、見つかったインタプリタをPATH先頭に追加（ディレクトリに裸の `python3` が無い場合は一時シムを作成） |
| `setup_env.sh`（Homebrew） | `/opt/homebrew/bin` を存在すれば末尾に追加し、`.zprofile` を読まないシェルでも cmake/ninja が解決するように（ESP-IDF管理ツールを優先するため末尾に追加） |
| `setup_env.sh`（戻り値チェック） | `export.sh` の戻り値チェックを追加（従来は失敗直後でも「[OK] ready」と表示していた） |
| `setup_env.bat` | 検索リストから `Python313`（3.13+は非対応）を除外。全候補（pyenv-win版本ファイル・python.org・scoop・conda）を新設の `:sf_check_python` サブルーチンで実際に3.10-3.12範囲かを検証し、単なる存在チェックから範囲チェック+wingetガイダンスへ置換。`export.bat` が非ゼロを返したら明示的に失敗させる |

**検証（コミットメッセージより）:** macOS上、素のPATH（zsh/bash 両方）で `source setup_env.sh`
がpyenvの3.12.12へ誘導し `idf5.5_py3.12_env` を活性化、`sf build vehicle` が成功
（"Build successful"、1119KBバイナリ）。通常のpyenvシェルでは誘導メッセージ無しで従来通り。
`setup_env.bat` は静的レビューのみ（この時点ではWindows実機未検証）。

## 5. sf CLI: idf.py/esptool を venv python 経由で起動；ランチャーのシェル読み込み順修正（`e538253f`）

**ユーザー影響:** macOS で `.command` ランチャーが `setup_env.sh` を非対話シェルで読み込んだ
直後に対話シェルへ引き継ぐ際、ユーザーの `.zshrc`（pyenv init）が venv より後に PATH を
上書きしてしまい、`idf.py` のシバンが click の無いpyenv裸インタプリタを解決して
「No module named 'click'」で失敗する不具合を修正。

| 変更 | 内容 |
|------|------|
| `espidf.py` `idf_command()` | `idf.py` を全プラットフォームで `[sys.executable, <IDF_PATH>/tools/idf.py, ...]` として起動するよう変更（従来はWindowsのみ）。裸の `idf.py` 実行はその `#!/usr/bin/env python` シバンをPATHに対して解決するため、`setup_env.sh` 実行後にPATH先頭へ何かを追加されるとESP-IDF venvが覆い隠されていた |
| `flash.py`（legacy esptool経路） | 同様に `python -m esptool` → `sys.executable` 経由に変更 |
| `installer.py` `_create_terminal_launcher_macos()` | ワーカーの `.command` が対話シェルへ引き継ぐ際、一時 ZDOTDIR/`--rcfile` ラッパーでユーザー自身の設定（`.zshenv`/`.zprofile`/`.zshrc`、または `.bash_profile`/`.bashrc`）を**先に**、`setup_env.sh` を**最後に**読み込むよう変更してから自己削除（未知シェルは従来動作を維持） |

**検証（コミットメッセージより）:** macOS（素のPATH）で旧フローの再現失敗を確認後、
`idf_command` 修正適用後は `sf build vehicle` がランチャー経由でOK。再生成した `.command`
ランチャーもend-to-endで環境読み込み・ビルド成功、引き継ぎ後の対話シェルで
`which python` が `idf5.5_py3.12_env/bin/python` を解決。通常のシェルから直接読み込む
経路も変化なし。

## 6. `setup_env`: インストール済み ESP-IDF venv に Python を一致させ export 結果を検証（`045ffea4`）

**ユーザー影響:** Windows で pyenv-win のグローバルが3.12、インストーラが作成した venv が
`idf5.5_py3.10_env` という組み合わせのとき、`setup_env.bat` が3.12を「範囲内」として受理して
しまい、`export.bat` が `idf5.5_py3.12_env` を探しに行って失敗するのに、失敗が
`[OK]` 表示の直前に埋もれて後続の `sf build` が失敗する不具合を修正。

| 変更 | 内容 |
|------|------|
| `setup_env.bat` | `IDF_TOOLS_PATH\python_env` 配下のインストール済み `idf*_py3.Y_env` venv を走査し、`:sf_check_python` がそのいずれかのマイナーバージョンと一致するPythonのみ受理（venvが無ければ従来の3.10-3.12範囲チェックにフォールバック） |
| `setup_env.bat`（pyenv-win探索） | グローバルバージョンファイルだけでなく、pyenv-winのインストール済みバージョンディレクトリ群も走査し、グローバル設定がvenvと一致しない場合でも一致するインタプリタを発見できるように |
| `setup_env.bat`（export検証） | `export.bat` 実行後、`IDF_PYTHON_ENV_PATH` が設定されておりその `Scripts\python.exe` が実在することを検証してから `[OK]` を表示 |
| `setup_env.sh` | Unix側も同様にvenv一致を優先する探索順に再構成（`_sf_py_find_minor()` を新設し、優先マイナーバージョン順に探索）。`export.sh` 実行後、実際にvenvが存在することを検証してから `[OK]` を表示 |

**根本原因:** ESP-IDF v5.5 の export スクリプトは、内部のactivateが失敗しても成功終了する
（エラーは表示されるが何もexportされない）ため、終了ステータス0は環境が読み込まれた証拠に
ならない。venv実在チェックが両プラットフォームでこの隙間を塞いだ。

**検証（コミットメッセージより）:** cmd.exe + `setup_env.bat` がpyenv-win 3.10.11を選択、
`idf5.5_py3.10_env` を活性化（python依存関係OK）、`sf build vehicle` が成功
（`stampfly_vehicle.bin`、1119KB）。

## 7. GUI インストーラ: pyenv-win シムディレクトリを Python 探索から除外（`742c6ea7`、最新）

**ユーザー影響:** Windows で pyenv-win を使っている環境において、GUI インストーラが
`exit code 9009` で失敗し「python がインストールされていません」という誤ったメッセージが
出る不具合を修正。本リリースの直接のきっかけとなった不具合。

**根本原因（詳細）:**

1. `shutil.which("python")` は `PATHEXT`（`.EXE;.BAT;...`）経由で解決するため、pyenv-win の
   `shims\python.bat` シム（バッチファイル）を返してしまう。この候補は
   `STABILITY_CANONICAL` と評価され優先順位が高い
2. このシムは実体のバージョンを正しく報告する（例: `3.12.10`）ため、探索ロジックはこの
   シムディレクトリを、対応するpyenvの実体ディレクトリ（`versions\3.12.10`）より優先して
   採用してしまう
3. `_clean_env_for_cmd()` はこのシムディレクトリをPATH誘導先として選ぶが、シムディレクトリ
   には `.bat` シムしか無く `python.exe` が存在しない
4. ESP-IDF の `install.bat` は `python.exe --version`（`.exe` を明示したリテラル名）を実行
   するため解決に失敗し、errorlevel 9009 が「python がインストールされていません」という
   誤ったメッセージとして伝播する

**修正内容:**

| # | 修正 | 内容 |
|---|------|------|
| 1 | `_find_system_python_dir()` | Windows において、実体の `python.exe` を含まない候補ディレクトリ（pyenv-winの`python.bat`シムを指すもの）を不採用にする |
| 2 | `_clean_env_for_cmd()` | 発見したPythonディレクトリをPATHの先頭へ**無条件に** prepend するよう変更（従来はPATH中の**どこかに**既に存在すれば追加をスキップしていたが、WindowsAppsのスタブより**後**に位置していると結局解決に失敗していた） |

修正後は、シム候補が除外されても pyenv インストールは `_windows_pyenv_win_python_dir()`
（実体の `versions\<ver>` ディレクトリを返す）経由で引き続き認識される。

**検証（コミットメッセージより）:** 実際に不具合が発生していたマシン（pyenv-win環境）上で、
Python探索が `.pyenv\pyenv-win\versions\3.12.10` を返すようになったことを確認。誘導後の環境
下で `python.exe --version` が rc=0・`Python 3.12.10` を返すことを確認。
`tools/ci/check_installer_gui.py` 4/4 PASS。

**エンドツーエンド検証（タグ作成前に追加実施、2026-07-23）:** 同じ pyenv-win マシン上で、
修正済みコード経路（`_run_install_script()` → `_clean_env_for_cmd()` → `install.bat`）による
ESP-IDF ツールインストールを実行し、9009 で失敗していた操作が最後まで完走
（`idf5.5_py3.12_env` venv 作成、"All done!"）することを確認。その環境で
`setup_env.bat` → `sf build vehicle` / `sf build controller` の両ビルド成功も確認。

## 8. ファームウェア挙動への影響

**ファームウェアの挙動変更なし。** `firmware/vehicle`/`firmware/controller`/`firmware/common`
に対するソースコード変更は一切無い。本リリースの範囲内で唯一 `firmware/` 配下に触れている
変更は `firmware/workshop/lessons/lesson_manifest.yaml` へのメタデータフィールド
（`has_solution`）追加のみ（節3）で、これはワークショップ教材のレッスン定義（sf CLI が
参照する解答チェック要否のフラグ）であり、ビルドされるファームウェアのソース・挙動には
一切影響しない。`v2026.07.5` と同一挙動。

## 9. 検証状況まとめ

| 項目 | 状態 |
|------|------|
| Windows: GUIインストーラ `--selftest` / `check_installer_gui.py` | PASS（`742c6ea7`時点で4/4） |
| Windows: `setup_env.bat` + `sf build vehicle`（pyenv-win環境） | 成功確認済み（`045ffea4`）。実機での不具合報告からの再現・修正・実機再検証まで完了（`742c6ea7`） |
| macOS: `setup_env.sh` + `sf build vehicle`（素のPATH、zsh/bash） | 成功確認済み（`4e25e875`, `e538253f`） |
| macOS: `.command` ランチャー経由 end-to-end | 成功確認済み（`e538253f`） |
| macOS: GUIインストーラ ESP-IDF venv選定 | 成功確認済み（`f4bc30a5`） |
| GUIインストーラ: シミュレータ依存の導入検証 | Windows上で確認済み（`24419eb0`）。macOS実機での再検証は各コミットの Next steps で継続案内されている |
| `sf doctor` / `sf lesson solution`・`switch` | macOS上で確認済み（`0a881447`） |
| Windows: `verify_idf_env()` のvenv不在検証分岐 | **未実施**（`0a881447` のNext stepsより。実ワークショップ準備時に検証予定） |

## 10. リリース前チェックリスト（`versioning.md` §5 対応）

| # | 項目 | 状態 |
|---|------|------|
| 1 | SILS退行テストPASS | 確認済み。SILS関連パス（`lib/sfcli/**` 等）に触れた本範囲最後のコミット `e538253f` の push で CI `SILS scenario regression`（Actions run `29903750603`）が success。以降のコミットはパスフィルタ対象外（`scripts/`・`setup_env.*`・docs のみ） |
| 2 | `sf build vehicle` / `sf build controller` ローカルビルド成功 | 確認済み（2026-07-23、Windows pyenv-win マシン上で両ターゲットとも Build successful。vehicle 1119.0 KB） |
| 3 | 前回リリースからの変更点整理 | 本ドキュメント |
| 4-5 | タグ作成・push | **未実施** |
| 6 | Release workflow成功確認 | **未実施**（タグ push 後に確認） |
| 7 | Release notes加筆 | 本ドキュメントがその元原稿 |

## 11. Next steps

- `git tag v2026.07.6` → push → Release workflow のグリーン・13アセット揃いを確認
  （チェックリスト #1・#2 は実施済み）
- `045ffea4` の Next steps: macOS 側の「No module named 'click'」報告は `e538253f` で既に
  修正済みのはずなので、Mac チェックアウトで `git pull` → 新規 StampFly Terminal で
  `sf build vehicle` を再試行し確認
- `0a881447` の Next steps: Windows で `verify_idf_env()` のvenv不在検証分岐が未検証のため、
  次回ワークショップ準備時に実機確認
- `742c6ea7` の Next steps: pyenv-win 実機での ESP-IDF ツール導入エンドツーエンド確認は
  実施済み（節7の追加検証参照。なお StampFly Setup GUI は実行時に clone したリポジトリの
  `scripts/installer.py` をプロセス内 import する設計のため、既存 exe でも main 反映後は
  修正が有効）。選択中バージョンが対応外（3.13+）だが対応内バージョンが未選択で
  インストール済み、という pyenv-win 構成のカバーは引き続き検討
- `docs/contributing/versioning.md` §6「カリキュラム互換表」への `v2026.07.6` 行追加は
  実施済み（本リリースのコミットに含まれる）
