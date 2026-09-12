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
├── examples/          # 学習用サンプル（§11）
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
├── university/        # 大学講義（シラバス・評価ルーブリック）
├── assets/, stylesheets/  # 画像・生成図・サイトのスタイル
├── telemetry/         # UDP テレメトリ設計メモ
└── bonus/, experiments/   # 番外資料・実験手順（LaTeX）
```

### 役割の要点
- `architecture/`: タスク分割・周期・優先度、vehicle / controller / protocol 間の責務境界、設計判断の背景。
  シミュレーション方針は `architecture/simulation-policy.md` を正とする
- `reference/`: 手で書かない。`protocol/tools/` が仕様から生成する（§5）
- `plans/`: 機能ごとの計画・見直し文書。状態（計画中／実装中／実装済み／見直し中）を冒頭に書く
- `events/`: `stampfly_workshop/`（Workshop。§4 の `firmware/workshop/` と対、廃棄・全面書き換え予定）、
  `dxh2026/`、`sci_tutorial_2026/`
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
├── vehicle_old/   # レガシーファームウェア（凍結、実飛行87回）
├── controller/    # 送信機ファームウェア
├── common/        # 3 ファームが共有する ESP-NOW プロトコル実装
├── apps/          # sf app new が生成する利用者のプロジェクト（L1 の入口）
├── workshop/      # Workshop 骨格（ws::、L0）。旧アーキテクチャ、廃棄・全面書き換え予定
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
  （`vehicle`・`vehicle_old`・`controller` の 3 ファームで共通）。正は `protocol/spec/messages.yaml`（§5）
- vehicle は `firmware/common/` のプロトコル以外には依存しない自己完結設計

#### 推定・制御
- `IEstimator`/`IController` 抽象インターフェース経由（ESKF・相補フィルタ・PID はその一実装）。
  詳細は `firmware/vehicle/docs/architecture.md`

### firmware/vehicle_old/
旧世代の機体ファームウェア（実飛行 87 回、**凍結・新規開発なし**）。
`sf_hal_*`/`sf_algo_*`/`sf_svc_*` の層分け命名。sf CLI・SILS 退行試験から `--target vehicle_old` として
引き続きビルド・テスト可能。`firmware/common/` を controller と共有する。

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
vehicle / vehicle_old / controller が共有する **組込み向けプロトコル実装**。

```
common/
└── protocol/
    ├── include/espnow_protocol.hpp   # ESP-NOW ControlPacket/PairingPacket（主系統、3 ファーム共有）
    └── include/udp_protocol.hpp      # WiFi 代替 UDP モード（vehicle_old の sf_svc_udp と controller の sf_udp_client のみ）
```

`espnow_protocol.hpp` は `protocol/spec/messages.yaml` の C++ 実装であり手書きである。両者の整合は
`protocol/tools/check_messages.py` が検査する（§5）。共有の数値演算・汎用ヘルパは置かない
（vehicle は自前の `sf_math` を持つ。以前の `math/`・`utils/` は空のまま 2026-09-12 に削除）。

### firmware/apps/
`sf app new <name>` が `firmware/vehicle/examples/` の雛形を複製して作る、利用者自身のプロジェクトの置き場。
`sf app sils / build / flash` で SILS と実機の両方に同じソースを組み込む（設計は `docs/plans/sf-app-sils-plan.md`）。
利用者に独自コードを書いてもらう入口の設計そのものは見直し中（`docs/plans/user-programming-entry-review.md`）。

### firmware/workshop/
講習会向けの Workshop 骨格。`ws::` 名前空間の簡易 API と `setup()`/`loop_400Hz()` の 2 関数で書く
（4 階層アクセスの L0）。`sf lesson` と CI が参照する稼働中の実体だが、**旧アーキテクチャで作られており、
内容的には廃棄して全面的に書き直す予定**。vehicle との HAL 二重管理（横断ルール R12 との乖離）は
`firmware/vehicle/docs/workshop_migration.md` が記録している。新規開発は行わない。

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
├── notebooks/   # 探索的解析。education/ に講義用ノートブック
├── scripts/     # 再現性重視の解析処理・指標算出（案件別のサブディレクトリを持つ）
├── datasets/    # 小規模なサンプルログ（education/, flightlog/, sysid/, motor_sweep_*/）
├── notes/       # 調査ノート
├── reports/     # 生成された図・結果。原則 git 管理しない（例外: rate_sysid_reference/）
└── out/         # 同上（案件別の出力）
```

---

## 8. tools/ : sf CLI のバックエンドと補助ツール

利用者に提供するツールは **sf コマンドとして公開する**（§9 の `lib/sfcli`）。`tools/` は主にその
バックエンド実装を置く場所であり、単独実行を前提としたスクリプトは増やさない。

```
tools/
├── README.md
├── calibration/       # sf cal
├── log_analyzer/      # sf log（capture / wifi / convert / analyze / viz）。UDP 取得 udp_capture.py を含む
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
├── stampfly/      # Tello 風 Python SDK
└── stampfly_edu/  # 教育用ヘルパ（実機が無ければシミュレータへ切り替える connect_or_simulate 等）
```

- `pyproject.toml` の `package-dir = {"" = "lib"}` により `pip install -e .` で導入される
- sf CLI は開発・書き込み・診断・ログ・シミュレーション・自作プロジェクトの一貫した入口。
  コマンド実装は `lib/sfcli/commands/`、新コマンドの追加手順は `docs/contributing/adding-sf-commands.md`

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
  `simulator/sils/RESET_PLAN.md` は立ち上げ期の記録
- protocol を介した I/O により、実機との一貫性を保つ

---

## 11. examples/ : 学習用サンプル

```
examples/
├── README.md
└── education/   # 大学講義向けの Python サンプル（lib/stampfly_edu を使用）
```

- ファームウェア側の例題は `firmware/vehicle/examples/`（01〜12）にあり、本ディレクトリとは別系統
- 二つの例題群と `sf app`・Workshop の関係（利用者に独自コードを書いてもらう入口）は
  `docs/plans/user-programming-entry-review.md` で見直し中。決まるまで構成は変えない
- 以前の `protocol_roundtrip/`・`pid_tuning/` は空のまま 2026-09-12 に削除

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
Workshop の全面書き換え（§4）。

---

## 16. まとめ

StampFly Ecosystem は完成品ではなく、**制御工学教育と研究を育て続けるための基盤**である。

この PROJECT_PLAN.md は、その思想と設計判断を将来へ残すための文書である。
