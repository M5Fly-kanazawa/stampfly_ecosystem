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
└── education/
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

### docs/education/
- 授業・実習用の資料
- 課題、評価観点、チェックリスト
- 学年・レベル別の利用指針

---

## 4. firmware/ : 組込みで動く実体

```
firmware/
├── vehicle/
├── controller/
└── common/
```

### firmware/vehicle/
StampFly 機体上で動作するファームウェア。
制御工学的には **plant（制御対象）** に相当する。

```
vehicle/
├── components/
├── main/
├── sdkconfig.defaults
└── README.md
```

#### vehicle/README.md
- 機体ファームの責務と全体構成
- ビルド方法・ターゲット MCU
- 制御系の階層構造の説明

#### vehicle/main/
- アプリケーションのエントリポイント
- タスク生成・初期化・依存関係の定義

#### vehicle/sdkconfig.defaults
- ESP-IDF 用の推奨設定
- 再現性のあるビルド環境を確保する

#### vehicle/components/
ESP-IDF component 単位での機能分割。

- sensors/
  - IMU・気圧・ToF・光学フロー等のセンサ取得
  - 非同期取得、バス管理、キャリブレーション

- estimation/
  - 姿勢推定・状態推定（AHRS, EKF 等）
  - センサ融合、バイアス推定

- control/
  - 角速度・姿勢・位置制御ループ
  - control/ ディレクトリで設計された結果の反映点

- actuators/
  - ミキサ、モータ出力、飽和処理
  - フェイルセーフ・停止処理

- comms/
  - テレメトリ送受信
  - protocol に基づくパケット処理

- system/
  - 状態管理、パラメータ管理、診断
  - CLI、ログ、共通基盤

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
vehicle / controller で共有される **組込み向け共通実装**。

```
common/
├── protocol/
├── math/
└── utils/
```

- protocol/
  - protocol/spec に基づく組込み側実装
  - エンコード・デコード、CRC 等

- math/
  - 組込み向け数値演算ユーティリティ
  - 行列・ベクトル・フィルタ補助

- utils/
  - ログ、リングバッファ、汎用ヘルパ

※ 仕様の単一の真実は protocol/ に置く。

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
  - SIL 等の検証環境

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
└── ci/
```

- flashing/
  - 書き込み・DFU・ボード検出

- calibration/
  - センサ校正ツール

- log_capture/
  - 実験ログ取得（PC 側）

- ci/
  - CI 用補助スクリプト

---

## 9. simulator/ : 仮想実験環境

```
simulator/
├── assets/
└── environments/
```

- assets/
  - 機体モデル、メッシュ、設定

- environments/
  - SIL/HIL/3D 環境定義

protocol を介した I/O により、実機との一貫性を保つ。

---

## 10. examples/ : 最短で動く入口

```
examples/
├── protocol_roundtrip/
├── log_to_plot/
└── pid_tuning/
```

- protocol_roundtrip/
  - 仕様→エンコード/デコードの最小例

- log_to_plot/
  - ログ取得→解析→プロット

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
