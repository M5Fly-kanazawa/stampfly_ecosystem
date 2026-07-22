# v2026.07.5 リリースノート（ドラフト）

`docs/contributing/versioning.md` §5「リリース手順」の手順3（前回リリースからの変更点整理）
に対応するドラフト。**`v2026.07.5` タグはまだ作成されていない**（本ドキュメント作成時点で
`origin/main` の最新コミットは `0ec83035`）。

対象範囲: `v2026.07.4(ea0749c8)..origin/main` の6コミット。ファームウェア
（`firmware/vehicle`, `firmware/controller`, `firmware/common`）は**挙動変更なし**（後述）。
全て SIL シミュレータ・CI・インストーラ関連。

> **ユーザーへの要点:** 本リリースは SIL（Software-In-the-Loop）シミュレータの開発基盤強化が
> 中心で、飛行・書き込みのみを行う利用者への影響は無い。SIL 開発をする人・GUI インストーラで
> SIL ツールチェーンを導入したい人が対象。

## 1. SIL シミュレータの Windows ネイティブ対応（MinGW-w64）

**ユーザー影響:** これまで SIL シミュレータの開発には WSL（Windows Subsystem for Linux）が
実質必須だったが、Windows ネイティブで完結するようになった。

| 変更 | 内容 |
|------|------|
| Windows 互換シム新設 | `esp_idf_host` に `fcntl.h`/`unistd.h` の互換シムを追加（`pipe()` は名前付きパイプ + `PIPE_NOWAIT` で実際にノンブロッキング動作、`fcntl`/`F_SETFL`/`O_NONBLOCK`、`fsync`）。TCP CLI リダイレクト用に `cstdio` + `win_stdio_shim.cpp`（stdout/stderr のシャドーイング） |
| フィールド順序修正 | `esp_idf_host/led_strip.h` を実際の ESP-IDF と同じフィールド順に修正し、ファームウェアの指定初期化子が C++ のフィールド順規則下でもコンパイル可能に |
| ビルド警告抑制 | `cmake/mujoco.cmake` にベンダー同梱の mujoco ターゲット限定で `-Wno-format` を追加（GCC の ms_printf 規約と `%zu` の不一致、到達不能パス） |
| 終了処理修正 | `emu/emu_main.cpp` を `std::_Exit(0)` で終了するよう修正し、グローバル静的破棄中のセグフォルトを解消（`emu_main_generic.cpp` と同じパターン） |
| `sf sil build` | Windows で `C:\msys64\mingw64\bin` の MinGW を自動検出し、Ninja+GCC で `build-mingw/` にビルド（`build/`（MSVC）は変更なし）。`scenario`/`run`/`compare` は DLL 検索パスを自動注入 |
| `sf doctor` | 「SIL host toolchain」の新診断項目を追加（gcc の有無・posix スレッドモデル確認、SIL はオプションのため WARN レベル） |

（コミット: `ae8535f2`）

**検証（コミットメッセージより）:** SIL 全13ターゲットが MinGW-w64（GCC 16、posix threads）で
ビルド成功、`sf sil scenario` による32シナリオ全regressionスイート（TEST_MATRIX.md の L1〜L4）
がWindows上でPASS。ファームウェアソースは無変更（GCCがファームウェアの指定初期化子・
`__attribute__` をそのまま受理するため、MSVC移植ではなくMinGWを選択）。

## 2. GUI インストーラ: SIL 開発ツールチェーン導入オプション追加

**ユーザー影響:** StampFly Setup で、SIL 開発に必要なツールチェーン（MSYS2/MinGW-w64）を
オプションとして導入できるようになった（既定は無効）。

| 変更 | 内容 |
|------|------|
| 新フラグ | `installer.py` に `--with-sil-toolchain`（既定オフ）を追加。Step 4/4 完了後に追加のオプション処理として実行され、4ステップという GUI 進捗契約は変更なし |
| Windows | `C:\msys64\mingw64` に g++/ninja が既にあれば冪等にスキップ。無ければ winget での MSYS2 導入 → `pacman -Syu`（1回リトライ）→ mingw-w64 ツールチェーン/cmake/ninja をベストエフォートで導入。全て `_stream_subprocess()` でストリーミング表示。失敗してもインストール自体は失敗させず `simulator/sil/README.md` への案内に留める |
| macOS/Linux | 案内のみ（xcode-select / apt・dnf・pacman でのgcc+cmake+ninja導入コマンドを表示） |
| 対話モード | 常に y/N で確認（既定 No。約2GBのニッチなオプションのため、GUI Flasher の既定Yesとは異なる） |
| GUI | オプション画面に「SIL 開発ツールチェーンを導入」チェックボックスを追加。日英の文言、selftest の期待値も更新 |

（コミット: `920bad6c`）

**検証（コミットメッセージより）:** `--selftest` exit 0、`check_installer_gui.py` 4/4 PASS、
このPC上で `--non-interactive --with-sil-toolchain` を実行し既存のMSYS2ツールチェーンを検出して
スキップ（winget経路には入らない）、全ステップ冪等・exit 0 を確認。

## 3. SIL 回帰テストの CI 自動化（`sf sil regression` + GitHub Actions）

**ユーザー影響:** これまでリリース前の SIL 退行テストはメンテナ個人のローカル環境に依存して
いたが、main への push・PR ごとに自動実行されるようになった（`versioning.md` §5 手順1の
ギャップを解消）。

| 変更 | 内容 |
|------|------|
| `sf sil regression`（新サブコマンド） | `simulator/sil/scenarios/` 配下で `*.expect` を伴う全 `*.scn`（現在32件）を自動検出し、各シナリオのヘッダコメントから実行コマンドを取得（ドリフトするマニフェスト無し。`console_cli`/`hover_espnow` は `vehicle_old` を対象とする小さな例外テーブルで対応）。PASS/FAILを集計し、任意で `--json-out` サマリを出力、1件でも失敗すれば非ゼロ終了 |
| `sf sil build` | Linux/macOS でも `ninja` がPATH上にあればNinjaジェネレータを使用（従来はWindows/MinGW限定） |
| `.github/workflows/sil-regression.yml`（新設） | ubuntu-latest、build-essential + cmake + ninja-build、Python 3.12、`pip install -e .`。ESP-IDF は不要（`sil.py` がESP-IDFに依存しないことを確認済み）。`simulator/sil/build` を `mujoco.cmake` キーでキャッシュ（actions/cacheはmtimeを保持するため実質的なインクリメンタルビルドキャッシュ）。`sf sil build` → `sf sil regression` を実行し、`tools/ci/render_sil_summary.py` でMarkdownジョブサマリを生成、失敗シナリオのconsole/trajectory/resultsバンドルをartifactとしてアップロード |
| トリガー | main への push + pull_request（`firmware/**`, `simulator/sil/**`, `control/**`, `lib/sfcli/**`, `protocol/**` にパスフィルタ）+ `workflow_dispatch` |
| `versioning.md` §5 手順1（日英） | SIL退行テストがCIで強制されるようになった旨に更新（ローカル実行は事前確認・デバッグ用の補助という位置付けに変更） |

（コミット: `d6b13dde`）

## 4. hover_smoke ハーネスの離陸フェーズ修正

**ユーザー影響:** SIL開発者向け。`hover_smoke` シナリオが実際の離陸フェーズ機械（Grounded →
TakeoffClimb → Airborne）を経由せず、地上ARM安全ゲートによって推力が常時ゼロにクランプされて
いた不具合を修正（`max_alt` が期待値0.5mに対し0.013mしか出ない）。

| 変更 | 内容 |
|------|------|
| 離陸ハンドシェイク追加 | `hover_smoke.cpp` が ALT_HOLD 突入時に `ControllerCmd::Takeoff` を発行し、`controller_status` が `takeoff_reached` を報告した時点で `ControllerCmd::TakeoffComplete` を発行するよう修正。これによりPID鉛直制御の離陸フェーズ機械が実際に進行するようになった |
| タイムスケジュール再調整 | 実際の 0.3 m/s 自動上昇（0.5m到達まで約3.8秒。旧スケジュールは1.6秒を仮定していた）に合わせて `T_CLIMB`/`T_HOVER`/`T_YAW`/`T_STOP`/`T_LAND` とシミュレーション時間を再設定（各フェーズの相対長は維持） |
| リンク修正 | `win_stdio_shim.cpp` を `sf_cores` にもリンクし、`cores`/`rtos`/`hover`/`rate_tune` 系ターゲットでも stdout/stderr シャドーイングが機能するように |

原因は `hover_smoke` が `StateManager` を経由せず `system_mode`/`controller_command` トピックへ
直接注入していたため `onTakeoff()` が発火せず、フェーズ機械が Grounded のまま固定されていた点。
Windows/MinGW固有の問題ではなく、`.scn` シナリオ群は実際のARM/pilot_request経路を使うため
無影響。**ファームウェアソースは無変更**（テストハーネス側の修正）。

（コミット: `22c76d7f`）

**検証（コミットメッセージより）:** Windows/MinGWビルドで `hover_smoke` 7/7チェックPASS
（ESKF・相補フィルタ両推定器、max_alt 0.518m vs 期待値0.500m、yaw_spin 1.02 vs 1.002、
hold_band 0.004m、N0センサーノイズ下でもPASS）。`.scn` フルスイート再実行も全PASS。

## 5. Linux/Ubuntu 互換性修正（CI 初回グリーン化）

**背景:** 節3のCIワークフロー新設後、初回のubuntu-latest実行（SILベンチ初のLinuxビルド）で
2件のビルド衝突が発覚し、それぞれ別コミットで修正した。**修正が完了するまでの3回のCI実行は
failureだった（下表参照）。**

| # | 症状 | 原因 | 対策 | コミット |
|---|------|------|------|---------|
| 1 | `struct timeval` の再定義エラー | SILソケットシムが独自に `struct timeval` を無条件定義しており、glibcが `<sys/select.h>`（`<cstdlib>` 等から間接的にインクルードされる）経由で本物の `struct timeval` を定義するのと衝突（シム側ガード `_STRUCT_TIMEVAL` がglibc側ガード `__timeval_defined` と噛み合っていなかった） | `lwip/sockets.h` で `struct timeval` の自前定義をWindows（MinGW）限定に変更。Linux/macOSでは `<sys/time.h>` をインクルードしシステム型を使用（ファームウェアが触るのは `tv_sec`/`tv_usec` のみで両者は同一） | `6103bdc0` |
| 2 | `fcntl` の競合する関数宣言エラー | SILソケットシムの固定引数の静的 `fcntl()` スタブと、glibcの可変引数externの `fcntl()` が、両方をインクルードする唯一のファイル（`data_stream.cpp`）で衝突。`fcntl()`/`F_GETFL`/`F_SETFL`/`O_NONBLOCK` は `data_stream.hpp` 経由の `lwip/sockets.h` から既に提供されており、`<fcntl.h>` の直接includeは冗長だった（`telemetry.cpp`/`api_task.cpp` と同じノンブロッキングソケットパターン） | `data_stream.cpp` から冗長な `#include <fcntl.h>` を削除。`lwip/sockets.h` に「このヘッダと本物の `<fcntl.h>` は同一翻訳単位に共存させてはならない」旨のバイリンガルコメントを追加 | `0ec83035` |

`6103bdc0` の修正後もCIは別要因（上記#2）でfailureのままだったが、`0ec83035` の適用により
Actions run `29886097108` が初めてsuccessになった（下記「検証状況」参照）。

**検証（コミットメッセージより）:**
- `6103bdc0`: MinGWインクリメンタルリビルド 15/15ターゲットOK、Windows上で `sf sil regression`
  32/32 PASS。`esp_idf_host`/`compat` 配下の自前定義シンボルを全数監査し、他に衝突の恐れが
  無いことを確認
- `0ec83035`: `sf build vehicle`（実ESP-IDFターゲット）ビルド成功（削除したincludeは実機側でも
  冗長 — lwIPの`sockets.h`が`fcntl()`を提供するため）。MinGWインクリメンタルリビルド
  15/15ターゲットOK、Windows上で `sf sil regression` 32/32 PASS

## 6. ファームウェア挙動への影響

**ファームウェアの挙動変更なし。** `firmware/vehicle`/`firmware/controller`/`firmware/common`
に対する唯一の変更は `data_stream.cpp` の冗長な `#include <fcntl.h>` 削除のみ（節5 #2）。
削除後も `sf build vehicle` の実機ビルドを確認済みで、実際に使用していたシンボルは全て
`lwip/sockets.h` 経由で提供されていたため機能的な差分はない。`v2026.07.4` と同一挙動。

## 7. 検証状況まとめ

| 項目 | 状態 |
|------|------|
| Windows: `sf sil scenario`（32シナリオ） | 全コミット時点でPASS（最終的に32/32 PASS、`ae8535f2`〜`0ec83035` 各コミットメッセージより） |
| GitHub Actions `sil-regression.yml`（ubuntu-latest） | 初回〜3回目（`d6b13dde`, `22c76d7f`, `6103bdc0` push時点）はfailure。4回目（`0ec83035` push、run `29886097108`）でsuccess（グリーン） |
| `sf build vehicle`（実機ビルド） | 成功（`0ec83035` コミットメッセージより、data_stream.cpp修正後に確認） |
| GUI インストーラ `--selftest` / `check_installer_gui.py` | PASS（節2） |
| macOS/Linuxでの `sf sil build`（Ninja経路） | ロジック変更のみ。ubuntu CIでのグリーン化により間接的に検証済みだが、macOS実機での確認は**未実施** |
| Windows以外（macOS/Linux）でのGUIインストーラ「SIL開発ツールチェーン導入」オプション | 案内表示のみのロジックで、実機での導入確認は**未実施** |

## 8. リリース前チェックリスト（`versioning.md` §5 対応）

| # | 項目 | 状態 |
|---|------|------|
| 1 | SIL退行テストPASS | 実施済み。Windows `sf sil regression` 32/32 PASS、CI（ubuntu-latest）最終run `29886097108` success |
| 2 | `sf build vehicle` / `sf build controller` ローカルビルド成功 | `sf build vehicle` は `0ec83035` 時点で確認済み。`sf build controller` は未実施（ファームウェアソース自体は無変更のため影響は限定的） |
| 3 | 前回リリースからの変更点整理 | 本ドキュメント |
| 4-5 | タグ作成・push | 未実施（メイン側で実施） |
| 6 | Release workflow成功確認 | 未実施（タグ push 後に確認） |
| 7 | Release notes加筆 | 本ドキュメントがその元原稿。scratchpad に短縮版（`rel_notes_075.md`）を用意済み |

## 9. Next steps

- `git tag v2026.07.5` → push → Release workflow のグリーン・13アセット揃いを確認
- `sf build controller` のローカルビルド確認（`vehicle` は確認済みだが `controller` は未実施）
- macOS実機での `sf sil build`（Ninja経路）とGUIインストーラ「SIL開発ツールチェーン導入」
  オプションの動作確認
- `docs/contributing/versioning.md` §6「カリキュラム互換表」に `v2026.07.5` の行を追加
  （本ドラフトでは追加済み。タグ作成後、リリース日等の確定情報に齟齬がないか最終確認）
- v2026.07.4 から持ち越しの未検証項目（新品Windows機での実exeテスト、macOS/Linuxでの
  仮想環境マネージャ対応の実機検証、Linux/Windowsでの StampFly Terminal 起動確認）は
  引き続き未実施
