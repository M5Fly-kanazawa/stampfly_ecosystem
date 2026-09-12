# `sf app` を L1（Topic API）の入口にし、SILS で検証できるようにする計画

作成: 2026-09-07（同日、L0 骨格への一本化案を取り下げて改訂）。
状態: **実装済み**（Phase 0〜3、2026-09-08）／Phase 4（書き込み系 API）は未着手／「独自コードの入口」全体の設計は見直し中（[`user-programming-entry-review.md`](user-programming-entry-review.md)、2026-09-12）。

発端: README「何ができるのか？」に「独自の飛行プログラムの作成」「飛行プログラムの SILS
（Software In the Loop Simulation: ファームウェアそのものを PC 上で動かす試験）での検証」を
掲げたが、`sf app` で作った自作プロジェクトは現状 SILS で動かせないと判明した。

方針の前提（2026-09-07 ユーザー確認）:

- workshop 骨格（`firmware/workshop`、`setup()` / `loop_400Hz()`）は **L0**（最下層の飛行制御を
  自分で書けるようになるための初心者の層）であり、`sf app` の対象ではない。
- `sf app` が担うのは **L1**: vehicle 本体の Pub-Sub（Topic API、`sf::api::`）を使って自分の
  プログラム（コントローラ・推定器・ガイダンス、トピックを読むタスク）を書き、それを
  **vehicle 本体に組み込んで**実機と SILS の両方で動かすこと。
- HAL（L2）や BSP（L3）を自分で書きたい人は現時点では考慮しない。

## 0. 要旨

| 観点 | 内容 |
|------|------|
| 現状 | `sf app` プロジェクトは例題 09 / 10 の複製で、vehicle 本体とは別の独立した ESP-IDF プロジェクト。例題は vehicle のタスクを起動せず、10 番は合成信号でコントローラだけを回す**ベンチ**。実機で飛ばすには vehicle 本体の `control_task.cpp` を手で 1 行書き換える必要があり、`sf app` も SILS もそれを自動化していない |
| 何が無いか | (1) vehicle 本体に「ユーザーのコントローラ／推定器／タスク」を差し込む正式な口、(2) 実機ビルドと SILS ビルドの両方に外部ディレクトリ（`firmware/apps/<name>`）を取り込む CMake の口、(3) それを呼ぶ `sf app` / `sf sils` の配線 |
| 方針 | vehicle 本体に **アプリフック**（`sf::app::start()`、`sf::app::controller()`、`sf::app::estimator()`）を 1 組定義し、`firmware/apps/<name>` をその実装として **vehicle の main コンポーネントに直接コンパイルする**。実機ビルドも SILS の `emu_vehicle` も同じ変数 `SF_APP_DIR` でそのディレクトリを取り込む。L0（workshop）とは並列に共存する層であり、置き換えない |
| 成果 | `sf app new my_ctrl` → `sf app build my_ctrl`（実機）／ `sf app sils my_ctrl`（SILS）が**同一ソース**を vehicle 本体に組み込んで動かす。Code Identity（実機と SILS で同じコードが動く）を自作プログラムに拡張する |
| 見積り | Phase 0〜3 で 5〜7 日。Phase 4（書き込み系 API の拡張）は別途設計 |
| 進捗（2026-09-08） | Phase 0〜3 完了。コミット: 2ebd9bf1（Phase 0〜1: アプリフック・CMake）、440d37b4（Phase 2 前半・ファーム側: stock フック・テンプレート 11/12）、c6ad829c（Phase 2 後半・sf CLI 側: `sf app new/list/build/flash/sils`）。本表以下の Phase 見出しに完了状況を付記 |

## 1. 何ができていないか（事実）

調査は 2026-09-07 に実施し、行番号はその時点のもの。★は計画立案者が直接コードで再確認した項目。

### vehicle ファームウェア層

| 項目 | 現状 | 根拠 |
|------|------|------|
| コントローラの選択 ★ | `static sf::PidController controller;` の 1 行で固定。ファクトリ・レジストリ・弱シンボル・マクロは無い | `firmware/vehicle/tasks/control_task.cpp:63` |
| 推定器の選択 ★ | `createEstimator()` がパラメータ `estimator.type` で ESKF と相補フィルタの**既存 2 実装**から選ぶだけ。ユーザー実装を差し込む口は無い | `firmware/vehicle/tasks/imu_task.cpp:115-129` |
| タスク起動 | `start_all()` は 16 タスクの固定列挙。`app_main()` は 5 フェーズ（NVS → BSP → topics → params → `start_all()`）。ユーザー追加タスクを起動するフックは無い | `firmware/vehicle/tasks/tasks.cpp:42-104`、`firmware/vehicle/main/main.cpp:38-89` |
| Topic API の範囲 ★ | `sf::api::` は「最新値の読み取り」のみ（`imu_latest()`、`estimate_latest()`、`command_latest()`、`control_latest()`、`motor_latest()`、`power_latest()`、`current_mode()`、`is_armed()`）。アクチュエータ操作・状態要求は「M5 に先送り」と明記 | `firmware/vehicle/components/sf_api/include/sf_api.hpp:24-34, 55-162` |
| `IController` | 12 メソッド（`compute` / `reset` / `onModeChange` / `onLanding` / `onTakeoff` / `onTakeoffComplete` / `isTakeoffComplete` / `setGuidanceTarget` / `isGuidanceActive` / `startExcitation` / `fetchSysidResult` / `reloadParams`）。実装は `PidController` のみ | `firmware/vehicle/components/sf_controller/include/controller.hpp:56-201` |
| `IEstimator` | 実装は `EskfEstimator`、`ComplementaryEstimator` | `firmware/vehicle/components/sf_estimator/include/estimator.hpp:54-248` |
| 外部ディレクトリの取り込み ★ | `EXTRA_COMPONENT_DIRS` は `components` と `../common` の 2 つ固定。`main/CMakeLists.txt` の `REQUIRES` は 27 コンポーネントの静的列挙 | `firmware/vehicle/CMakeLists.txt:13-16`、`firmware/vehicle/main/CMakeLists.txt:21-55` |
| 新規 Topic の追加 | `data_types.hpp` → `topics.hpp` → `topics.cpp` → 文書更新、の vehicle 本体編集が前提。外部から追加する経路は無い | `firmware/vehicle/docs/topic_reference.md` §7（335-346 行） |

### 例題・`sf app` 層

| 項目 | 現状 | 根拠 |
|------|------|------|
| `sf app` の実体 | `firmware/vehicle/examples/<N>` を `firmware/apps/<name>` に複製し、`EXTRA_COMPONENT_DIRS` を `../../vehicle/components` に向け直すだけ。`build` / `flash` は `sf build apps/<name>` / `sf flash apps/<name>` への委譲。SILS には触れない | `lib/sfcli/commands/app.py:238-268, 397-425` |
| 09_topic_api_hello | vehicle のタスクを起動せず、`internal_sensor_feed.cpp` が自前で BMI270 を読み相補フィルタで `estimate_state` を publish する独立プロジェクト（README で「L2 の下ごしらえ」と明記） | `firmware/vehicle/examples/09_topic_api_hello/main/main.cpp:75-131` |
| 10_custom_controller | `LearnerController` は `IController` を実装しているが、`main.cpp` は合成サイン波に対して単体で回すベンチ。README §8 が「実機で飛ばすレシピ」として、新コンポーネント化 → `main/CMakeLists.txt` の `REQUIRES` 追加 → `control_task.cpp` に include 追加 → 63 行目を `static sf::LearnerController controller;` に書き換え → `sf build vehicle` と SILS 退行確認、を**手作業**で示す | `firmware/vehicle/examples/10_custom_controller/README.md`（§2、§8） |
| 設計原則との関係 | 例題集は「単独でビルド・実行可能（vehicle 全体のビルド不要）」を設計原則としており、計画中の L1 例題（`11_pid_single_axis` 〜 `20_pubsub_basics`）も同じ単独ベンチ路線の見積りになっている。vehicle 組み込みを前提とした例題計画は無い | `firmware/vehicle/docs/coding_and_education.md:218-227, 254-284` |
| 廃止された先行例 | `firmware/my_drone`（2026-03 最終更新）は vehicle のタスクを共有するユーザーファームを試みた形跡があるが、参照するコンポーネント名が現行と異なり、現状はビルドできない可能性が高い（推測） | `firmware/my_drone/main/CMakeLists.txt` |

### SILS・sf CLI 層

| 項目 | 現状 | 根拠 |
|------|------|------|
| SILS ターゲット | `SILS_TARGETS = ("vehicle", "vehicle_old", "workshop")` 固定。`sf sils scenario --target` は `choices` で `apps/<name>` を引数解析の段階で拒否 | `lib/sfcli/commands/sils.py:122-123, 374` |
| `emu_vehicle` のソース ★ | `firmware/vehicle` の `main` / `tasks` / `components` を `GLOB_RECURSE` で収集し `docs` / `examples` / `test` / `build` を除外。外部ディレクトリを足す変数は無い | `simulator/sils/CMakeLists.txt:289-293` |
| `sf sils build` の configure | `-D` は `CMAKE_BUILD_TYPE`・コンパイラ・ジェネレータのみ | `sils.py:634-647` |
| 近い前例 | `emu_workshop` は vehicle のソースから `main.cpp` / `control_task.cpp` / `tasks.cpp` を除き、`firmware/workshop/main/*.cpp` を足す。実機側 `firmware/workshop/main/CMakeLists.txt` も同じ除外をする。「vehicle の main コンポーネントに外部ソースを混ぜる」実働例 | `simulator/sils/CMakeLists.txt:437-511`、`firmware/workshop/main/CMakeLists.txt:30-92` |
| テスト・CI | apps 向けシナリオ・CI ジョブは無い | `simulator/sils/scenarios/`、`.github/workflows/sils-regression.yml` |

## 2. 設計判断

### 拡張点: vehicle 本体の「アプリフック」

vehicle 本体に、ユーザープログラムが実装を提供できる関数を 1 組だけ定義する（仮称、命名は
Phase 0 で確定）。

| フック | 既定の実装（app 無し） | ユーザー実装の例 |
|--------|----------------------|------------------|
| `sf::IController& sf::app::controller()` | `PidController` の静的インスタンスを返す（現行 63 行目と同じ挙動） | 自作 `IController` を返す（10 番の `LearnerController` 相当） |
| `sf::IEstimator& sf::app::estimator()` | 現行 `createEstimator()` と同じ（`estimator.type` で ESKF／相補） | 自作 `IEstimator` を返す |
| `void sf::app::start()` | 何もしない | トピックを読んで記録・判定・通知するタスクを起動する（`sf::api::*_latest()` を使う） |

- `control_task.cpp:63` と `imu_task.cpp` の `createEstimator()` は、このフックを呼ぶ形に置き換える。
  現行の手作業レシピ（README §8）を正式な口にするだけで、制御則そのものは変えない。
- `main.cpp` の起動フェーズ末尾で `sf::app::start()` を呼ぶ。
- 既定実装は `firmware/vehicle/main/app_default.cpp` に置く。

### 取り込み方式: main コンポーネントへの直接コンパイル

| 方式 | 判断 |
|------|------|
| A. 弱シンボル（既定を `__attribute__((weak))`、app が強シンボル） | **不採用**。ESP-IDF はコンポーネントを静的ライブラリにするため、弱定義で参照が満たされると app 側のアーカイブ要素が引き込まれない事故が起きうる（`WHOLE_ARCHIVE` 指定で回避できるが構成が増える） |
| B. 同名コンポーネントの差し替え（`EXTRA_COMPONENT_DIRS` の優先順で `sf_app` を上書き） | **不採用**。優先順の規則に依存し、SILS 側（ESP-IDF を使わない素の CMake）には同じ仕組みが無い |
| C. **`SF_APP_DIR` の `*.cpp` を main コンポーネントの `SRCS` に足し、`app_default.cpp` を外す** | **採用**。workshop 骨格が `user_code.cpp` で行っている方式と同型で、実機（ESP-IDF）と SILS（素の CMake）の両方に同じ 5 行で書ける。同一アーカイブ内なので参照解決の事故が無い |

`firmware/apps/<name>/` の中身は `app.cpp`（フックの実装）と任意の追加 `*.cpp` / `*.hpp`、
`app.yaml`（複製元・説明・対応: 実機 / SILS）、`README.md`。ESP-IDF の依存（`REQUIRES`）は
main コンポーネントのものを共有するので、app 側に CMakeLists は不要。

### L0 との関係

L0（workshop）は `ControlTask` を丸ごと `WorkshopControlTask` に置き換える層、L1（本計画）は
vehicle 本体のタスク構成をそのまま使い `IController` / `IEstimator` / 追加タスクを差し込む層。
アーキテクチャ設計書 §2.5 が「各層は並列に共存する」と定めるとおり、両者は別の入口として
維持する。将来 `WorkshopControlTask` を L1 フックの上に載せ替える案はあるが、本計画の範囲外。

### 例題 09 / 10 の扱い

- 09 / 10 は「vehicle 全体をビルドせずに API を学ぶ」単独ベンチとして残す（設計原則どおり）。
- `sf app new` の既定の複製元は、新設する **L1 組み込み型テンプレート**に変える。09 / 10 からの
  複製は引き続き可能だが、`app.yaml` に `sils: false` を記録し、`sf app list` に「ベンチ（SILS 不可）」
  と表示する。
- 10 番の README §8 は、新方式では「`sf app new --from 11_app_controller` でそのまま使える」に
  書き換える。

### 書き込み系 API（後続）

`sf::api::` は現状読み取り専用で、アクチュエータ操作・状態要求は「M5 に先送り」と明記されて
いる。ガイダンス目標の設定（`IController::setGuidanceTarget` を外から呼ぶ）、モード要求、
パラメータの読み書きを L1 から行えるようにする範囲は、Phase 4 として別途設計する（状態機械の
不変条件との照合が必要なため、本計画では範囲を決めない）。

## 3. 目標の使い方（受け入れ基準）

```bash
sf app new my_ctrl
```

```bash
sf app build my_ctrl
```

```bash
sf app sils my_ctrl
```

| 基準 | 内容 |
|------|------|
| 同一ソース | `sf app build` と `sf app sils` が `firmware/apps/my_ctrl/*.cpp` を vehicle 本体の main コンポーネントに組み込む |
| 既定挙動の維持 | app 無しの vehicle（既定実装）は現行と同じバイナリ挙動。SILS 回帰（vehicle / vehicle_old / workshop）に退行が無い |
| 合格基準 | 既定テンプレート（PID と同等の `IController`）で既存シナリオ `alt_flight`・`acro_flight` が PASS する |
| 分離 | app ごとに実機・SILS のビルド成果物が分かれ、app の切替でキャッシュ汚染が起きない |
| 3 OS | Windows（CMD）/ macOS / Ubuntu で同じコマンドが通る（パスに空白を含む場合を含む） |

## 4. 実装計画

### Phase 0: 仕様確定（0.5〜1 日）（完了）

| 作業 | 内容 |
|------|------|
| フックの名称と署名 | `firmware/vehicle/main/app_hooks.hpp` に 3 関数を宣言。`@design` タグで `architecture.md` §2.5（L1）と `detailed_design.md` の該当節を参照 |
| 設計文書の更新 | `architecture.md` §2.5 の L1 行に「入口は `sf app`、フックは `app_hooks.hpp`」を追記。`coding_and_education.md` §3 の「例題は単独ビルド可能」原則に「L1 組み込み型テンプレート（`sf app` 用）は vehicle 本体と一緒にビルドする」例外を明記。不変条件（INV）節との照合を記録 |
| テンプレートの仕様 | `11_app_controller`（`PidController` に委譲しつつ 1 軸だけ自分の式に置き換えられる `IController`）と `12_app_task_hello`（`estimate_latest()` を読んで一定周期で記録するタスク）の 2 つ。既定の複製元は 11 |

### Phase 1: vehicle 本体のフックと CMake（1.5〜2 日）（完了）

| 作業 | 対象 | 内容 |
|------|------|------|
| フック定義 | `firmware/vehicle/main/app_hooks.hpp`、`app_default.cpp` | 3 関数の宣言と既定実装。既定の `controller()` は現行 `PidController` の静的インスタンス、`estimator()` は現行 `createEstimator()` の移設 |
| 呼び出し側 | `tasks/control_task.cpp`、`tasks/imu_task.cpp`、`main/main.cpp` | 63 行目と `createEstimator()` をフック呼び出しに置き換え、起動フェーズ末尾で `sf::app::start()` |
| 実機ビルド | `firmware/vehicle/main/CMakeLists.txt` | `SF_APP_DIR` が与えられたら `${SF_APP_DIR}/*.cpp` を `SRCS` に、ディレクトリを `INCLUDE_DIRS` に加え、`app_default.cpp` を外す |
| SILS ビルド | `simulator/sils/CMakeLists.txt` | `emu_vehicle` に同じ変数で同じ差し替え（`app_default.cpp` を `EXCLUDE REGEX`、`${SF_APP_DIR}/*.cpp` を追加）。`emu_workshop` は影響を受けないことを確認 |
| 退行確認 | SILS 回帰全件 | app 無しで A/B 比較し退行ゼロ。`sf params check` も通す |

### Phase 2: テンプレートと sf CLI（1.5〜2 日）（完了）

| 作業 | 対象 | 内容 |
|------|------|------|
| テンプレート | `firmware/vehicle/examples/11_app_controller/`、`12_app_task_hello/` | `app.cpp` と README。README には「何を書き換えるか」「SILS で確認 → 実機」の順を書く |
| `sf app new` | `app.py` | 既定 `--from` を 11 に。複製後に `app.yaml` を書く（09 / 10 からは `sils: false`） |
| `sf app list` | `app.py` | 各 app に「実機: 可 / SILS: 可・不可（ベンチ）」を表示 |
| `sf app build` / `flash` | `app.py`、`build.py`、`flash.py` | 組み込み型 app は `firmware/vehicle` を `idf.py -B build_apps_<name> -DSF_APP_DIR=<abs path>` でビルド・書き込み。ベンチ型は従来どおり独立プロジェクト |
| `sf app sils` | `app.py` → `sils.py` | `sf app sils <name> [--scenario <scn>] [--expect <file>] [--noise ...]`。内部は `run_scenario(target="vehicle", app_dir=..., build_dir=simulator/sils/build/apps/<name>)` の薄い包み |
| `sf sils --target apps/<name>` | `sils.py` | `choices` を「3 ターゲット + `apps/<存在するディレクトリ>`」を返す関数に変更し、`build` / `scenario` の両方で受ける。実装は `sf app sils` と共有 |
| 案内文 | 両ファイル | ベンチ型 app に `sf app sils` を打ったら「この app は単独ベンチの複製で vehicle 本体には組み込めません。`sf app new --from 11_app_controller` を使ってください」 |

### Phase 3: テスト・CI・文書（1 日）（完了）

| 作業 | 対象 | 内容 |
|------|------|------|
| CI | `.github/workflows/sils-regression.yml` | `sf app new ci_app && sf app sils ci_app --scenario alt_flight` と、`--from 10_custom_controller` の従来経路ビルドを追加 |
| CLI テスト | `lib/sfcli/tests/` | `new → list → sils` の流れと、ベンチ型への案内文 |
| 文書 | `docs/guides/custom_program.md`（独自プログラム開発入門）、`docs/commands/sf-app.md`、`firmware/apps/README.md`、`docs/next_step.md` §8、`examples/10_custom_controller/README.md` §8 | 新しい流れ（new → SILS → 実機）、テンプレートの区分、手作業レシピの置き換え。README「何ができるのか？」の SILS 行の表現を最終確認 |

### Phase 4（別途設計、未着手）: 書き込み系 API

| 作業 | 内容 |
|------|------|
| 範囲の決定 | ガイダンス目標の設定、モード要求、パラメータ読み書きのうち L1 に開放するもの。`architecture.md` の不変条件（状態機械・離着陸）と照合 |
| 実装 | `sf::api::` に追加し、`12_app_task_hello` を「読む」から「読んで指示する」に拡張 |

## 5. リスクと未確認事項

| 項目 | 内容 | 対処 |
|------|------|------|
| ESP-IDF の `-D` と `-B` | `idf.py -B <dir> -D SF_APP_DIR=...` で sdkconfig の扱いが従来ビルドと変わらないか | Phase 1 で app 無し・有りのビルドを比較 |
| 400 Hz の予算 | ユーザーの `IController::compute()` が 400 Hz の周期を超えると制御が崩れる | テンプレート README に計測方法（`sf log wifi` の周期統計）と目安を書く。SILS では実時間より速く回るため実機で確認する旨を明記 |
| 推定器差し替えの影響 | 自作推定器が発散した場合の安全装置（現行の ESKF 発散検知は ESKF 専用か） | Phase 0 で `imu_task.cpp` の発散検知の対象を確認し、`IEstimator` 共通の監視に寄せるか判断 |
| Windows のパス | CMake 変数に空白・バックスラッシュを含む絶対パスを渡す | 実装は `Path.resolve()` で絶対パス化して `-D SF_APP_DIR=...` に渡す（`app.py`/`sils.py`）。**未検証（2026-09-08 時点）**: `windows-e2e.yml` への `sf app` ケース追加はまだ行っていない。Windows 実機・CI での確認が残作業 |
| 例題の設計原則との整合 | 「単独ビルド可能」原則との例外を文書で明示しないと、後続の例題が再びベンチ路線に戻る | Phase 0 の設計文書更新を先に行う |
| `firmware/my_drone` の遺物 | 現行構成でビルドできない可能性が高く、読者を混乱させる | 2026-09-12 に main から削除済み（タグ `archive/2026-09-12` で参照可） |

## 6. 関連文書

| 文書 | 関係 |
|------|------|
| `firmware/vehicle/docs/architecture.md` §2.5 | 4 階層アクセス（L0〜L3）の定義。本計画は L1 の入口を定める |
| `firmware/vehicle/docs/coding_and_education.md` | 例題の設計原則と Level / Tier の分類 |
| `firmware/vehicle/docs/topic_reference.md` | Topic 一覧と追加手順 |
| `firmware/vehicle/examples/10_custom_controller/README.md` §8 | 現行の手作業による実機統合手順。本計画で正式な口に置き換える |
| `docs/architecture/simulation-policy.md` | SILS の位置づけと忠実度目標 |
| `firmware/vehicle/docs/development_roadmap.md` | Code / Param / Model Identity の 3 原則 |
| `docs/guides/custom_program.md` | 独自プログラム開発入門（利用者向け。本計画の Phase 3 で新方式に更新） |
