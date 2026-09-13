# StampFly Ecosystem PROJECT_PLAN

最終更新: 2026-09-12（`docs/plans/project-plan-conformance.md` の棚卸しに基づく全面改訂）。

本文書は **リポジトリ構造の原典** である。各ディレクトリの責務と設計判断を書く。
構造（ディレクトリ・責務・命名）を変えるときは、本文書を同じコミットで更新する（§15）。
本文書はファイルの網羅目録ではない——目録は `docs/DOCUMENT_INDEX.md` が担う。

## 1. 本プロジェクトの目的と位置づけ

StampFly Ecosystem は、StampFly 機体を中心に、ドローン制御を **設計・実装・実験・解析・教育** の
すべての段階で一貫して扱うための **教育・研究用エコシステム**である。

本リポジトリは単なるコード置き場ではなく、以下を同時に満たすことを目的とする。

- 制御工学の設計プロセスを「実機ベース」で循環させる
- 学生・研究者が迷わず参加できる構造を提供する
- 長期的に拡張・派生しても破綻しない責務分割を維持する

そのため、本リポジトリは **責務（role）ベースのディレクトリ構造**を採用する。

### 目指す姿（2026-09-13 整理）

本リポジトリは **マルチコプタのプログラムを学ぶための総合環境** である。次の 6 点を満たすことを目指し、構築の途上にある。

| # | 目指す姿 |
|---|---------|
| P1 | **どの階層からでもプログラムが組める**（L0 Workshop API / L1 Topic API / L2 HAL / L3 BSP・ファーム全体。外側に Python・ブロック） |
| P2 | StampFly の **仕様・パラメータの資料がほぼ全て揃う** |
| P3 | パラメータの **求め方（実験方法）と分析方法が全て分かる** |
| P4 | **各階層用の講習資料が一通り揃う** |
| P5 | StampFly で講習やワークショップを行う **講師のために、準備と講習を簡単にする道具** を提供する |
| P6 | **制御教育と組み込み教育** に特に力を入れる（2026-09 時点でほぼ未着手） |

利用者向けの道具は二本柱で、併存させる。

| 道具 | 位置づけ | 対応する階層 |
|------|---------|-------------|
| `sf app` | **プログラミングツール**。自分のプログラムを作り、SILS と実機で動かす | 全階層が目標（2026-09 時点は L1 のみ） |
| `sf lesson` | **教育補助ツール**。講師が用意したレッスンを切り替え、ビルド・書き込み・解答表示を行う | 全階層が目標（2026-09 時点は L0 の Workshop のみ） |

整備状況の全体像は §16 の地図で追う。

---

## 2. トップレベル構成の意図

```
stampfly-ecosystem/
├── README.md          # 入口（要約・初飛行まで）
├── LICENSE
├── CLAUDE.md          # AI 支援ツール向けの作業規約（人間向けの説明は docs/ に置く）
├── docs/              # 人間が読む文書と公開サイト（§3）
├── firmware/          # 組込みで動く実体（§4）
├── protocol/          # 通信・ログ形式の仕様と整合検査（§5）
├── control/           # 制御設計資産（§6）
├── analysis/          # 実験結果の評価（§7）
├── tools/             # sf CLI のバックエンドと補助ツール（§8）
├── lib/               # PC 側の Python 実装。sf CLI 本体はここ（§9）
├── simulator/         # 仮想実験環境（§10）
├── scripts/           # インストーラの実装（§12）
├── ros/               # ROS2 連携（構築中。docs/plans/ros2-integration.md）
├── landing/           # 公開サイトのランディングページ（§3）
├── .mkdocs/           # docs/ を公開サイトにするための設定（§3）
├── .github/           # CI（§14）
├── .githooks/         # 非公開文書の混入を防ぐ pre-commit・節目の pre-push
├── install.sh / install.bat / setup_env.sh / setup_env.bat   # 導入と環境の入口（§12）
└── pyproject.toml / requirements.txt / requirements-docs.txt # Python パッケージ定義
```

### README.md
- リポジトリ全体の要約と入口。初学者・外部者が最初に読む
- 「何のためのエコシステムか」「どこから触るか」を示し、詳細は `docs/next_step.md` へ送る

### LICENSE
- 本リポジトリの利用条件。教育・研究用途での再利用を前提とする

### CLAUDE.md
- Claude Code 等の AI 支援ツールに対する作業規約。人間向けの説明は書かない（`docs/` に置く）
- 本文書と矛盾させない。構造に関わる規約は本文書が正

---

## 3. docs/ : 人間が読むための入口

```
docs/
├── overview.md        # エコシステム全体の俯瞰図・各ディレクトリの役割・推奨ワークフロー
├── next_step.md       # README（導入・初飛行）の次に読む詳細
├── index.md           # 公開サイト（mkdocs）のトップ
├── slides.md          # イベントスライド PDF の一覧
├── README.md          # docs/ の案内（本節の要約に留める）
├── DOCUMENT_INDEX.md  # 全文書の目録（日英）
├── architecture/      # システム構成・設計判断・シミュレーション方針・Tello 互換 API 参照
├── reference/         # 仕様から生成される参照文書（flight-log-format.md）
├── guides/            # 利用者向けガイド（安全・送信機・独自プログラム・ログ可視化・環境更新 等）
├── commands/          # sf CLI コマンドリファレンス
├── setup/             # OS 別セットアップ
├── contributing/      # 開発規約（文書スタイル・コミット規約・コマンド追加手順）
├── plans/             # 計画文書。冒頭に状態を明記し、アーカイブは作らない（§15）
├── events/            # 勉強会・講座（イベント単位のディレクトリ + 共有素材 _shared/）
├── assets/, stylesheets/  # 画像・生成図・サイトのスタイル
└── bonus/, experiments/   # 番外資料・実験手順（LaTeX）
```

### 役割の要点
- `architecture/`: タスク分割・周期・優先度、vehicle / controller / protocol 間の責務境界、設計判断の背景。
  シミュレーション方針は `architecture/simulation-policy.md` を正とする
- `reference/`: 手で書かない。`protocol/tools/` が仕様から生成する（§5）
- `plans/`: 機能ごとの計画・見直し文書。状態（計画中／実装中／実装済み／見直し中）を冒頭に書く
- `events/`: `stampfly_workshop/`（Workshop。§4 の `firmware/workshop/` と対）、`dxh2026/`、`sci_tutorial_2026/`
  （いずれも L0 `ws::` API 上の実習。L0 の API・レッスン更新後に再検証が要る）
- プロトコルの文章仕様は `protocol/README.md`（メッセージ一覧・オフセット表）と
  `docs/reference/flight-log-format.md` にある。`docs/protocol/` は置かない

### 公開サイト
- `landing/index.html` が GitHub Pages のルート、`docs/` は `.mkdocs/mkdocs.yml` で組版して `/docs/` に配信する
  （`.github/workflows/deploy-pages.yml`）。`.mkdocs/mkdocs.yml` の目次は生きている文書だけを指す

---

## 4. firmware/ : 組込みで動く実体

```
firmware/
├── vehicle/       # 主力ファームウェア（旧 vehicle_new を昇格）
├── controller/    # 送信機ファームウェア
├── common/        # vehicle と controller が共有する ESP-NOW / UDP プロトコル実装
├── apps/          # sf app new が生成する利用者のプロジェクト（全階層対応が目標。現状は L1）
├── workshop/      # Workshop 骨格（ws::、L0）。vehicle のコンポーネント基盤上で動く。API・レッスンを現行設計へ更新する対象
└── legacy/        # 出荷時バイナリ（sf flash --legacy による工場出荷状態への復旧）
```

### firmware/vehicle/
StampFly 機体上で動作する主力ファームウェア。制御工学的には **plant（制御対象）** に相当する。
`vehicle_new` として開発され、POS_HOLD（位置制御）の実機検証まで到達した時点で昇格した。

```
vehicle/
├── components/        # ESP-IDF コンポーネント（下記）
├── tasks/             # タスク定義（imu_task, control_task, state_task, api_task 等）
├── main/              # アプリエントリポイントと config.hpp
├── docs/              # 設計文書
├── examples/          # 学習用例題 01〜12（11・12 は sf app new の雛形）
├── test/              # Unity 形式のユニットテスト
├── sdkconfig.defaults
├── NOTICE.md
└── README.md
（CMakeLists.txt・partitions.csv は ESP-IDF の定型。dependencies.lock・sdkconfig・managed_components/ は生成物）
```

作業開始前に必ず読むべき設計文書（`firmware/vehicle/docs/`）:
1. `requirements.md` — 要件定義書
2. `architecture.md` — アーキテクチャ設計書（4 階層アクセス + 横断ルール R1〜R16 + BSP 層 + 不変条件 INV）
3. `detailed_design.md` — 詳細設計書
4. `coding_and_education.md` — コーディング方針・教育計画
5. `development_roadmap.md` — 開発ロードマップ・SILS→実機ワークフロー
6. `hardware_init.md` — BSP・ハードウェア初期化設計

同ディレクトリの他の文書（実装ログ・調査メモ・運用手引・トピック一覧など約 20 件）は記録であり、必読ではない。

#### vehicle/components/
ESP-IDF component 単位での機能分割。命名はフラットな `sf_<name>`
（旧 `sf_hal_*`/`sf_algo_*`/`sf_svc_*` の層分けは廃止し、コンポーネント間は Pub-Sub トピック経由で疎結合）。

- HAL: sf_hal_bmi270, sf_hal_bmm150, sf_hal_bmp280, sf_hal_vl53l3cx, sf_hal_pmw3901,
  sf_hal_motor, sf_hal_led, sf_hal_buzzer, sf_hal_button, sf_hal_power
- コア基盤: sf_core（データ型・パラメータテーブル）, sf_board（BSP・起動シーケンス）,
  sf_math（ベクトル・行列・クォータニオン、ヘッダオンリー）
- 推定: sf_estimator（IEstimator 抽象）, sf_estimator_eskf, sf_estimator_complementary
- 制御: sf_controller（IController 抽象）, sf_controller_pid, sf_actuator（ミキサ）
- 状態・離着陸: sf_state, sf_takeoff_landing, sf_failsafe, sf_calibration
- 通信: sf_comm（ESP-NOW 受信）, sf_command（正規化・調停）,
  sf_api（**L1 Topic API**: 学習者コード向けの読み取り関数群 `sf::api::*`）,
  sf_telemetry（400Hz 統一テレメトリと、Tello 互換の状態送信 10Hz UDP:8890）
- 拡張点: sf_app_hooks（L1 の差し替え口 `sf::app::controller()` / `estimator()` / `start()`。
  `firmware/apps/<name>` を `SF_APP_DIR` で main コンポーネントに組み込む）
- その他: sf_logger, sf_notify, sf_autotune

Tello 互換の外部 API（UDP:8889 のテキストコマンド）はコンポーネントではなく `tasks/api_task.cpp` が担う。

#### vehicle/main/
- `config.hpp`: コンパイル時に決まる固定定数（ハードウェア固有値・周期など）
- 調整可能なパラメータ（PID ゲイン・推定器設定など）の正は `components/sf_core/params.cpp`
  のパラメータテーブルで、NVS に永続化され `param set` で実行中に変更できる

#### 通信プロトコル
- ESP-NOW `ControlPacket`(14B)/`PairingPacket`(11B) は `firmware/common/protocol/` に共有実装
  （`vehicle`・`controller` で共通）。正は `protocol/spec/messages.yaml`（§5）
- vehicle は `firmware/common/` のプロトコル以外には依存しない自己完結設計

#### 推定・制御
- `IEstimator`/`IController` 抽象インターフェース経由（ESKF・相補フィルタ・PID はその一実装）。
  詳細は `firmware/vehicle/docs/architecture.md`

### firmware/vehicle_old/（削除済み）
旧世代の機体ファームウェア（`sf_hal_*`/`sf_algo_*`/`sf_svc_*` の層分け命名、実飛行 87 回、2026-07-05 に凍結）は
**2026-09-13 に削除した**（タグ `archive/2026-09-13`）。同時に、SILS の `emu_vehicle_old` と `vehicle_old` 専用の
シナリオ（`console_cli`・`hover_espnow`・`hover_alt`・`hover_long`）、`sf build/sils` の `vehicle_old` ターゲット、
`sf params check` の参照、`vehicle_old` 専用だった Python SDK `lib/stampfly/`（TCP 23＋WebSocket 80 前提）も削除した。

### firmware/controller/
操縦用コントローラ（送信機）側のファームウェア。人間の意思を信号に変換する HMI。

```
controller/
├── components/   # 入力デバイス（スティック・スイッチ）、デッドゾーン・正規化・フェイルセーフ
├── main/         # 制御コマンド生成ループ
├── sdkconfig.defaults
├── LICENSE
└── README.md     # 対象プラットフォーム、入力→コマンドの流れ
```

### firmware/common/
vehicle / controller が共有する **組込み向けプロトコル実装**。

```
common/
└── protocol/
    ├── include/espnow_protocol.hpp   # ESP-NOW ControlPacket/PairingPacket（主系統、両ファーム共有）
    └── include/udp_protocol.hpp      # WiFi 代替 UDP モードのパケット定義（controller の sf_udp_client が使う）
```

`espnow_protocol.hpp` は `protocol/spec/messages.yaml` の C++ 実装であり手書きである。両者の整合は
`protocol/tools/check_messages.py` が検査する（§5）。共有の数値演算・汎用ヘルパは置かない
（vehicle は自前の `sf_math` を持つ。以前の `math/`・`utils/` は空のまま 2026-09-12 に削除）。

### firmware/apps/
`sf app new <name>` が雛形を複製して作る、利用者自身のプロジェクトの置き場。`sf app sils / build / flash` で
SILS と実機の両方に同じソースを組み込む（現行の仕組みは `docs/plans/sf-app-sils-plan.md`、雛形は
`firmware/vehicle/examples/`）。`sf app` は **全階層に対応するプログラミングツール** を目指すが、2026-09 時点で
組み込めるのは L1（`IController`/`IEstimator` の差し替えと追加タスク）だけであり、L0・L2・L3 の雛形と
組み込み方は未整備。入口の設計は見直し中（`docs/plans/user-programming-entry-review.md`）。

### firmware/workshop/
講習会向けの Workshop 骨格。`ws::` 名前空間の簡易 API と `setup()`/`loop_400Hz()` の 2 関数で書く
（4 階層アクセスの L0）。`sf lesson` と CI が参照する稼働中の実体。ビルドは vehicle のコンポーネント
（`EXTRA_COMPONENT_DIRS ../vehicle/components`）を使い、**2026-07-18（`39d02c1b`）に vehicle 基盤へ移行済み**。
ただし L0 の API 層・ミキサ等に vehicle と重複する実装が残り（横断ルール R12 との乖離、`architecture.md` §2.5 末尾、
`workshop_migration.md`）、レッスン内容も移行前の設計を引き継いでいる。方針は **廃棄ではなくアップグレード**——
「2 関数で書ける」というやりたいことは変えず、重複を vehicle 側に寄せて `ws::` を薄いラッパーに戻し、レッスンを
現行設計（Pub-Sub・不変条件）に合わせて更新し、L1 以上の階層にも広げる。完了までは L0 の機能追加を行わない。

### firmware/legacy/
本エコシステム以前の出荷時（PlatformIO 版）Vehicle/Controller のバイナリ。`sf flash --legacy` で
工場出荷状態へ戻すために使う。

---

## 5. protocol/ : 共通言語（Single Source of Truth）

```
protocol/
├── README.md              # メッセージ一覧・オフセット表（人間向けの文章仕様）
├── spec/
│   ├── messages.yaml      # ESP-NOW ControlPacket/PairingPacket の正
│   ├── flight_log.yaml    # 標準フライトログ一式（.sflog.zip）の正: ストリーム名・列名・単位・レート
│   ├── espnow_tdma.yaml   # 実装からの逆文書化（生成・検査の対象外）
│   └── websocket.yaml     # 同上
└── tools/
    ├── gen_flight_log.py  # flight_log.yaml → lib/sflog/schema.py と docs/reference/flight-log-format.md を生成（--check で鮮度検査）
    └── check_messages.py  # messages.yaml ⇔ firmware/common/protocol/include/espnow_protocol.hpp の整合検査
```

- 仕様の正は `spec/` に置く。**生成物は消費側に置く**（Python は `lib/sflog/schema.py`、参照文書は
  `docs/reference/`）。`protocol/generated/` は置かない
- 「正」を名乗る仕様には整合検査を付ける。`flight_log.yaml` は `gen_flight_log.py --check`、
  `messages.yaml` は `check_messages.py`。いずれも CI（§14）で実行する
- `espnow_tdma.yaml`・`websocket.yaml` は firmware 実装から書き起こした文書であり、生成・検査の対象ではない

---

## 6. control/ : 制御設計資産

```
control/
├── README.md
├── models/    # stampfly_physical.yaml = 機体物理パラメータの正。同定結果もここ
└── design/    # 設計根拠。loop_shaping_tool/（ブラウザ完結のループ整形ツール）
```

- `models/stampfly_physical.yaml` から `sf params generate` が `tools/sysid/_generated_params.py` を生成し、
  CI が `--check` で鮮度を検査する
- SILS は `simulator/sils/`（§10）、実機ログとの照合は `analysis/`（§7）と `sf sils` の合否判定で行う。
  `control/` に検証環境は置かない（以前の `simulation/`・`validation/` は空のまま 2026-09-12 に削除）

---

## 7. analysis/ : 実験結果の評価

```
analysis/
├── README.md
├── notebooks/   # 探索的解析（大学講義用の 15 本は 2026-09-13 に削除。§11）
├── scripts/     # 再現性重視の解析処理・指標算出（案件別のサブディレクトリを持つ）
├── datasets/    # 小規模なサンプルログ（flightlog/, sysid/, motor_sweep_*/）
├── notes/       # 調査ノート
└── reports/     # 生成された図・結果。原則 git 管理しない（例外: rate_sysid_reference/）。以前の out/ は統合
```

---

## 8. tools/ : sf CLI のバックエンドと補助ツール

利用者に提供するツールは **sf コマンドとして公開する**（§9 の `lib/sfcli`）。`tools/` は主にその
バックエンド実装を置く場所であり、単独実行を前提としたスクリプトは増やさない。

```
tools/
├── README.md
├── calibration/       # sf cal
├── log_analyzer/      # sf log（wifi / convert / analyze / viz / list / info）。UDP 取得 udp_capture.py を含む
├── params_audit/      # sf params check / generate
├── sysid/             # sf sysid（同定・自動調整）
├── flasher_gui/       # sf flasher（GUI 書き込み）。書き込み処理の本体は lib/sfcli/commands/flash.py
├── installer_gui/     # StampFly Setup（GUI インストーラ）
├── stampfly_py/       # 配布用 Python SDK サンプル（Tello 互換）
├── ci/                # CI 補助スクリプト
├── slides/            # docs/events のスライド HTML 化（sf 非経由、docs/events/Makefile から）
├── udev/              # Linux udev ルール（sf 非経由）
├── terminal_launcher/ # インストーラの端末起動（sf 非経由）
└── extract_snippets.py  # 教材コードの抜き出し（sf 非経由）
```

- sf を経由しない補助は上記 4 つに限り、増やすときは本節に追記する
- 書き込み・ログ取得の実装は sf CLI 側にある（以前の `flashing/` は空のまま 2026-09-12 に削除、`log_capture/` は不在）

---

## 9. lib/ : PC 側の Python 実装

```
lib/
├── sfcli/         # sf CLI 本体（commands/, utils/, assets/vendor/blockly）
├── sflog/         # フライトログ一式のスキーマと読み書き（schema.py は生成物、§5）
└── stampfly/      # Tello 風 Python SDK（`tools/stampfly_py/` との二系統を整理中。`docs/plans/repository-cleanup-candidates.md` C1）
```

（`stampfly_edu/`（大学講義用ヘルパ）は 2026-09-13 に削除。§11）

- `pyproject.toml` の `package-dir = {"" = "lib"}` により `pip install -e .` で導入される
- sf CLI は開発・書き込み・診断・ログ・シミュレーション・自作プロジェクト・講習の一貫した入口。
  コマンド実装は `lib/sfcli/commands/`、新コマンドの追加手順は `docs/contributing/adding-sf-commands.md`
- 利用者向けの二本柱（§1）: `sf app`（プログラミングツール）と `sf lesson`（教育補助ツール）。
  `sf lesson` は現在 `firmware/workshop/lessons/`（`lesson_manifest.yaml`）の L0 レッスンに結び付いているが、
  レッスンの置き場と階層に依存しない道具にし、全階層のレッスンを扱えるようにする。`sf competition` は講習会の競技運営を担う

---

## 10. simulator/ : 仮想実験環境

```
simulator/
├── genesis/    # Genesis 物理エンジン版（高精度物理・物理量ベース制御）
├── sandbox/    # STL 分割・WebGL ビューア等の実験
├── shared/     # 機体モデル・設定・シナリオ（assets/configs/scenarios）を各シミュレータが共有
├── sils/       # Software-in-the-Loop 本体（決定論的・ESP-IDF ホストビルド）
├── tests/      # シミュレータ横断のテスト
├── tools/      # シミュレータ間比較などの補助
└── vpython/    # VPython 版（軽量・ブラウザ 3D 表示）
```

- シミュレーション全体の方針（3 層構造・忠実度目標）は `docs/architecture/simulation-policy.md` を正とする。
  SILS 立ち上げ期（2026-06 の再構築 E0〜E8・P1〜P10）の経緯は同文書に要約し、原文 `simulator/sils/RESET_PLAN.md`
  と `docs/plans/simulator-migration.md` は 2026-09-13 に削除（タグ `archive/2026-09-13`）
- protocol を介した I/O により、実機との一貫性を保つ

---

## 11. 大学講義用の教材（計画）

大学講義（半期 15 回、Python/Jupyter）向けの教材一式（`docs/university/`、`examples/education/`、`lib/stampfly_edu/`、
`analysis/notebooks/education/`）は **2026-09-13 に一度削除**した（タグ `archive/2026-09-13`）。作る予定だけを
`docs/plans/university-course-plan.md` に残す。次に作るときは §1 の P4・P6 に合わせ、どの階層に位置づけるかを
決めてから着手する。トップレベルの `examples/` はこれに伴い無くなった。ファームウェア側の例題は
`firmware/vehicle/examples/`（01〜12、§4）にある。

---

## 12. scripts/ とインストーラ

- `install.sh` / `install.bat` は `scripts/installer.py` を呼ぶ。専用の Python と ESP-IDF を自己完結で導入する
  （`docs/plans/dedicated-environment-plan.md`）。`sf upgrade` が更新を担う
- `setup_env.sh` / `setup_env.bat` は開発環境を有効化する（ESP-IDF の `export` と sf CLI）
- `scripts/` にはインストーラの実装とそのテストを置く。それ以外の補助スクリプトは §8 の規則に従う

---

## 13. 外部資産（ベンダリング）の規則

- 外部ライブラリを同梱するときは、**使う側の隣の `vendor/`** に置く。集約ディレクトリは持たない
  （以前の `third_party/` は空のまま 2026-09-12 に削除）
- 同梱する資産にはライセンス全文のファイルとバージョンの記録を必ず添える
- 現在の同梱資産: `lib/sfcli/assets/vendor/blockly/`（LICENSE, VERSION.txt）、
  `simulator/shared/assets/vendor/three/`（README に出典と MIT 表記）、
  `tools/log_analyzer/vendor/plotly.min.js`（`plotly.min.js.LICENSE.txt`）
- Python の依存は `pyproject.toml` / `requirements*.txt` で管理し、同梱しない

---

## 14. .github/workflows/

| ワークフロー | 役割 |
|-------------|------|
| `sils-regression.yml` | SILS シナリオの退行試験、`sf params check/generate --check`、フライトログ形式と `messages.yaml` の整合検査、`lib/sflog` のテスト |
| `macos-linux-e2e.yml` / `windows-e2e.yml` | インストーラの端末間 E2E |
| `release.yml` | ファームウェア・フラッシャ・インストーラのリリースビルド |
| `deploy-pages.yml` | ランディングと docs サイトの配信 |

静的解析（lint・型検査）は導入していない。整合性の担保は上記の生成物鮮度検査と退行試験で行う。

---

## 15. 原典の運用規則（拡張の受け入れ）

本文書を長く正しく保つための規則。「原典に無い＝禁止」ではなく、「原典に書いてから置く」が原則である。

1. **同じコミットで更新する。** ディレクトリ・責務・命名を変えるときは、本文書の該当節を同じコミットで直す
2. **本文書は意図と責務を書く。** ファイルの網羅目録は `docs/DOCUMENT_INDEX.md` が担う。両者の役割を混ぜない
3. **サブ README は本文書の要約に留める。** `docs/README.md`・`tools/README.md` 等は本文書と異なる構造を独自に主張しない
4. **計画文書は状態を明記し、アーカイブは作らない。** `docs/plans/` の文書は冒頭に状態（計画中／実装中／実装済み／見直し中）を書く。
   完了・廃止した文書、使われなくなったコードは main から削除し、削除直前のコミットに注釈付きタグ
   `archive/YYYY-MM-DD` を付ける（取り出しは `git show archive/YYYY-MM-DD:<path>`）。中身の無い `.gitkeep` のみの削除にタグは要らない
5. **空の置き場は作らない。** ディレクトリは中身と一緒に作る。将来の予定は本文書か `docs/plans/` に文章で書く
6. **新しい用途・技術は節を足してから置く。** 既存の節に収まらないものは、本文書に節（または既存節への行）を追加してから配置する
7. **大きな見直しは計画文書で行う。** 入口の再設計・Workshop の書き換えのような構造に及ぶ検討は `docs/plans/` に
   「見直し中」として置き、本文書からリンクする。結論が出たら本文書に反映し、計画文書は削除する（規則 4）
8. **CLAUDE.md は本文書に従う。** AI 向け規約が構造に触れるときは本文書を参照し、独自の構造を定めない

現在「見直し中」の事項: 利用者に独自コードを書いてもらう入口（`docs/plans/user-programming-entry-review.md`）、
Workshop（L0）の API・レッスンの現行設計への更新（§4）、`sf app`・`sf lesson` の全階層対応（§1・§9）、
整理対象の候補（`docs/plans/repository-cleanup-candidates.md`）。

---

## 16. 整備状況の地図（階層 × 道具・雛形・資料・講習）

§1 の「目指す姿」を、階層ごとに何が揃っていて何が無いかで追う表。`coding_and_education.md` §4 の
一本道の章立て表（Ch.1〜10）の後継であり、整備が進むごとにここを更新する。2026-09-13 時点の実態。
凡例: ○ あり／△ 部分的／× なし／— 対象外。

| 階層 | プログラミング（`sf app`） | 雛形 | 仕様・実験・分析の資料 | 講習資料（制御） | 講習資料（組み込み） | 講師支援（`sf lesson`） |
|------|------------------------|------|--------------------|----------------|-------------------|---------------------|
| 外側: Python / ブロック | ×（`sf app` の対象外。道具は Tello 互換 SDK・`sf blocks`） | △ `tools/stampfly_py` の例 | △ `docs/architecture/tello-api-reference.md`（現行との整合を調査中） | ×（大学講義用は 2026-09-13 に削除、計画のみ `university-course-plan.md`） | × | △ Blockly ガイド |
| L0 Workshop API（`ws::`） | ×（`sf lesson` 側のみ） | ○ `firmware/workshop/`（vehicle 基盤上の `ws::` ラッパー） | △ `docs/events/sci_tutorial_2026/cheatsheet.md`（`ws::` 早見表） | △ Workshop レッスン・SCI 2026（`ws::` 上のレート／姿勢の実習） | △ 同（モータ・センサの読み書き） | ○ `sf lesson`（13 レッスン＋`sci2026` コース）。L0 のレッスンのみ |
| L1 Topic API（`sf::api`、`IController`/`IEstimator`） | ○ `sf app new/sils/build/flash`（Topic の書き込みは未実装） | ○ `11_app_controller`・`12_app_task_hello`（PidController 委譲前提） | ○ `topic_reference.md`、設計 6 文書、`params.cpp`、`sf sysid`・`sf autotune` | △ `docs/guides/custom_program.md`（ACRO PID を 0 から、1 本） | × | × |
| L2 HAL 直叩き | × | ○ 例題 01〜08（単独ビルド、SILS 不可） | ○ HAL ドライバ README・データシート要約 | — | △ 例題 README（改造課題なし） | × |
| L3 BSP・ファーム全体 | × | × | ○ `hardware_init.md`・`architecture.md`・`coding_and_education.md` | — | × | × |

欠けの要点（優先順）:
1. 講習資料が L0（旧基盤）と外側の Python に偏り、**制御教育・組み込み教育の段階的な教材が L1〜L3 に無い**（P4・P6）
2. **`sf app` は L1 のみ、L3 には入口が無い**。L1 の Topic API は読み取りのみ（P1）
3. **`sf lesson` が L0 のレッスンにしか対応していない**。L0 の API・レッスンの更新と、レッスンの置き場・階層に依存しない道具への再設計が先（P5）
4. **実験→パラメータ→分析を一通り追える手順書が無い**。道具（`sf sysid`・`sf cal`・`sf log`）は揃っている（P3）
5. 資料が 3 か所（`firmware/vehicle/docs`・`docs/architecture`・`control/models`）に散在し、入口からの導線が無い（P2）

---

## 17. まとめ

StampFly Ecosystem は完成品ではなく、**制御工学教育と研究を育て続けるための基盤**である。

この PROJECT_PLAN.md は、その思想と設計判断を将来へ残すための文書である。
