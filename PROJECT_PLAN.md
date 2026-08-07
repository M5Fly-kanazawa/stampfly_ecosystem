# StampFly Ecosystem PROJECT_PLAN

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
├── README.md
├── LICENSE
├── docs/
├── firmware/
├── protocol/
├── control/
├── analysis/
├── tools/
├── simulator/
├── examples/
├── third_party/
└── .github/
```

### README.md
- リポジトリ全体の要約と入口
- 初学者・外部者が最初に読むファイル
- 「何のためのエコシステムか」「どこから触るか」を示す

### LICENSE
- 本リポジトリの利用条件を明示
- 教育・研究用途での再利用を前提とする

---

## 3. docs/ : 人間が読むための入口

```
docs/
├── overview.md
├── getting-started.md
├── architecture/
├── protocol/
├── workshop/
└── university/
```

### docs/overview.md
- エコシステム全体の俯瞰図
- 各ディレクトリの役割
- 推奨ワークフロー（設計→実装→実験→解析）

### docs/getting-started.md
- 初学者向けの最短導線
- examples・firmware への案内
- 環境構築の最小手順

### docs/architecture/
- システム構成図
- タスク分割・周期・優先度
- vehicle / controller / protocol 間の責務境界
- 設計判断の背景を残す場所

### docs/protocol/
- プロトコルの文章仕様
- 各フィールドの意味・単位・更新規則
- 設計意図を人間向けに説明

### docs/workshop/
- ワークショップ資料（スライド・実習ガイド・競技ルール）

### docs/university/
- 大学講義資料（シラバス・評価ルーブリック）

---

## 4. firmware/ : 組込みで動く実体

```
firmware/
├── vehicle/       # 主力ファームウェア（旧 vehicle_new を昇格）
├── vehicle_old/    # レガシーファームウェア（凍結、実飛行87回）
├── controller/
└── common/
```

### firmware/vehicle/
StampFly 機体上で動作する主力ファームウェア。
制御工学的には **plant（制御対象）** に相当する。

`vehicle_new` として開発され、POS_HOLD（位置制御）の実機検証まで到達した時点で
`firmware/vehicle/` に昇格した（旧実装は `firmware/vehicle_old/` へ）。

```
vehicle/
├── components/
├── tasks/
├── main/
├── docs/
├── examples/
├── sdkconfig.defaults
└── README.md
```

作業開始前に必ず読むべき設計文書（`firmware/vehicle/docs/`）:
1. `requirements.md` — 要件定義書
2. `architecture.md` — アーキテクチャ設計書（4階層アクセス + 横断ルール R1〜R16 + BSP 層）
3. `detailed_design.md` — 詳細設計書
4. `coding_and_education.md` — コーディング方針・教育計画
5. `development_roadmap.md` — 開発ロードマップ・SILS→実機ワークフロー
6. `hardware_init.md` — BSP・ハードウェア初期化設計

#### vehicle/components/
ESP-IDF component 単位での機能分割。命名規則: フラットな `sf_<name>`
（旧 `sf_hal_*`/`sf_algo_*`/`sf_svc_*` の層分けは廃止し、コンポーネント間は
Pub-Sub トピック経由で疎結合）。

- HAL: sf_hal_bmi270, sf_hal_bmm150, sf_hal_bmp280, sf_hal_vl53l3cx,
  sf_hal_pmw3901, sf_hal_motor, sf_hal_led, sf_hal_buzzer, sf_hal_button,
  sf_hal_power
- コア基盤: sf_core（データ型・パラメータテーブル）, sf_board（BSP・起動シーケンス）,
  sf_math（ベクトル・行列・クォータニオン、ヘッダオンリー）
- 推定: sf_estimator（IEstimator抽象）, sf_estimator_eskf, sf_estimator_complementary
- 制御: sf_controller（IController抽象）, sf_controller_pid, sf_actuator（ミキサ）
- 状態・離着陸: sf_state, sf_takeoff_landing, sf_failsafe, sf_calibration
- 通信: sf_comm（ESP-NOW受信）, sf_command（正規化・調停）, sf_api（Tello風API）,
  sf_telemetry（400Hz統一テレメトリ）
- その他: sf_logger, sf_notify, sf_autotune

- vehicle/tasks/ — タスク定義（imu_task, control_task, state_task 等）
- vehicle/main/ — config.hpp（全パラメータの一元管理）、アプリエントリポイント

#### 通信プロトコル
- ESP-NOW `ControlPacket`(14B)/`PairingPacket`(11B) は `firmware/common/protocol/`
  に共有実装（`firmware/vehicle`・`firmware/vehicle_old`・`firmware/controller` の
  3ファームで共通、protocol/spec/messages.yaml が SSOT）
- vehicle 自体は `firmware/common/` の他部分（math/utils）には依存しない自己完結設計

#### 推定・制御
- `IEstimator`/`IController` 抽象インターフェース経由（ESKF・相補フィルタ・PIDは
  その一実装）。詳細は `firmware/vehicle/docs/architecture.md` を参照

---

### firmware/vehicle_old/
旧世代の機体ファームウェア（実飛行87回、**凍結・新規開発なし**）。
`sf_hal_*`/`sf_algo_*`/`sf_svc_*` の層分け命名（旧 `firmware/vehicle/` 時代の構成）。
sf CLI・SILS回帰から `--target vehicle_old` として引き続きビルド・テスト可能。
`firmware/common/` を controller と共有する。

---

### firmware/controller/
操縦用コントローラ（送信機）側のファームウェア。
人間の意思を信号に変換する HMI。

```
controller/
├── components/
├── main/
└── README.md
```

- components/
  - 入力デバイス（スティック、スイッチ）
  - デッドゾーン、正規化、フェイルセーフ

- main/
  - 制御コマンド生成ループ

- README.md
  - 対象プラットフォーム
  - 入力→コマンドの流れ

---

### firmware/common/
vehicle / vehicle_old / controller で共有される **組込み向け共通実装**。

```
common/
├── protocol/
├── math/    # (未実装プレースホルダ)
└── utils/   # (未実装プレースホルダ)
```

- protocol/
  - `espnow_protocol.hpp`: ESP-NOW `ControlPacket`/`PairingPacket`（主系統、
    3ファーム全てが共有）
  - `udp_protocol.hpp`: WiFi 代替 UDP モードのパケット定義（`vehicle_old` の
    `sf_svc_udp` + `controller` の `sf_udp_client` のみが使用）
  - protocol/spec に基づく組込み側実装、エンコード・デコード、チェックサム等

- math/, utils/
  - 当初想定されていた共有数値演算・汎用ヘルパの置き場だが、これまで
    実装されたことがない（`.gitkeep` のみ）。`vehicle` は自前の `sf_math`
    コンポーネントを持ち、このディレクトリには依存しない

※ 仕様の単一の真実は protocol/ に置く。共有される**コード実装**は現状
`protocol/` の ESP-NOW 構造体定義のみ（ESP-NOW 主系統は元々 `vehicle_old` の
ESP-NOW 側もここを経由していなかったが、リファクタリングで統合した）。

---

## 5. protocol/ : 共通言語（Single Source of Truth）

```
protocol/
├── spec/
├── generated/
└── tools/
```

- spec/
  - 機械可読なプロトコル仕様（YAML, proto 等）
  - エコシステム全体の中心

- generated/
  - 仕様から生成されたコード
  - 教育用途ではコミット可

- tools/
  - 仕様検証、コード生成
  - CI での整合性チェック

---

## 6. control/ : 制御設計資産

```
control/
├── models/
├── design/
├── simulation/
└── validation/
```

- models/
  - 数学モデル、同定結果

- design/
  - PID・ループ整形・MPC 等
  - 設計根拠を残す

- simulation/
  - SILS 等の検証環境

- validation/
  - 実機ログとの照合
  - 設計の妥当性評価

---

## 7. analysis/ : 実験結果の評価

```
analysis/
├── notebooks/
├── scripts/
├── datasets/
└── reports/
```

- notebooks/
  - 授業・検討用の探索的解析

- scripts/
  - 再現性重視の解析処理
  - 指標算出の自動化

- datasets/
  - 小規模なサンプルログ

- reports/
  - 生成された図・結果
  - 原則 git 管理しない

---

## 8. tools/ : 横断的補助ツール

```
tools/
├── flashing/
├── calibration/
├── log_capture/
├── log_analyzer/
└── ci/
```

- flashing/
  - 書き込み・DFU・ボード検出

- calibration/
  - センサ校正ツール

- log_capture/
  - 実験ログ取得（PC 側）

- log_analyzer/
  - ログ解析

- ci/
  - CI 用補助スクリプト

---

## 9. simulator/ : 仮想実験環境

> **【2026-07-22 実態更新】** 以下のディレクトリツリーは実態（`ls simulator/`）に合わせて更新した。旧記載の `assets/`・`environments/` は現存しない。

```
simulator/
├── genesis/    # Genesis物理エンジン版シミュレータ（高精度物理演算・物理量ベース制御）
├── sandbox/    # STL分割・WebGLビューア等の実験用フォルダ
├── shared/     # 機体モデル・設定・シナリオ（assets/configs/scenarios）— 各シミュレータが共有
├── sils/        # Software-in-the-Loop 本体（決定論的・ESP-IDFホストビルド、設計の正は sils/RESET_PLAN.md）
├── tests/      # シミュレータ横断のテスト（例: 制御アロケーション互換性検証）
├── tools/      # シミュレータ間比較などの補助ツール（compare_simulators）
└── vpython/    # VPython版シミュレータ（軽量・ブラウザ3D表示・SILS/HIL向け）
```

- SILS 本体の設計・構築経緯は `simulator/sils/RESET_PLAN.md` を正とする。
- シミュレーション全体の方針（3層構造: 設計用線形モデル / 実ログ駆動再生 / SILS、Model Fidelity 期の忠実度目標）は `docs/architecture/simulation-policy.md` を正とする。
- protocol を介した I/O により、実機との一貫性を保つ。

---

## 10. examples/ : 最短で動く入口

```
examples/
├── protocol_roundtrip/
└── pid_tuning/
```

- protocol_roundtrip/
  - 仕様→エンコード/デコードの最小例

- pid_tuning/
  - 設計→パラメータ→実機反映

---

## 11. third_party/

- 外部ライブラリ・サブモジュール
- ライセンス明記必須

---

## 12. .github/workflows/

- CI 定義
- プロトコル整合性・静的チェック

---

## 13. まとめ

StampFly Ecosystem は完成品ではなく、
**制御工学教育と研究を育て続けるための基盤**である。

この PROJECT_PLAN.md は、その思想と設計判断を将来へ残すための文書である。
