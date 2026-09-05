# StampFly 勉強会 競技会ルールブック

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### 競技会について

StampFly 勉強会の最終日（Day 5）に開催するホバリング競技会のルールブックです。4日間のワークショップで学んだ角速度フィードバック制御を実践で発揮します。

### 種目

| 種目 | 内容 |
|------|------|
| ホバリングタイム | 自分が実装した角速度フィードバック制御でどれだけ長くホバリングできるかを競う |

## 2. ホバリングタイム競技

### ルール

| 項目 | 内容 |
|------|------|
| 競技内容 | 離陸してからできるだけ長くホバリングを維持する |
| 制限時間 | 60 秒（上限） |
| 試行回数 | 3 回（ベストタイム採用） |
| 操縦方式 | コントローラ操縦（スロットルは手動、姿勢安定化は自分のプログラム） |
| 制御方式 | Lesson 5/8 で実装した角速度フィードバック制御（P制御 or PID制御） |

### ホバリングの定義

以下の条件を**全て**満たしている間がホバリングとみなされます:

| 条件 | 基準 |
|------|------|
| 離陸 | 機体が地面から浮いている |
| 制御維持 | 壁・天井・障害物に接触していない |
| 飛行高度 | おおよそ 10cm〜1.5m の範囲内 |

### ホバリング終了の判定

以下のいずれかでホバリング終了:

- 機体が地面に着地または墜落
- 壁・天井に接触
- フライトエリア外に逸脱
- 参加者が DISARM した場合
- 制限時間（60秒）に到達

### 採点基準

| 順位 | ホバリングタイム |
|------|----------------|
| 1 位 | 最長ホバリングタイム |
| 2 位 | 2番目に長い |
| 3 位以降 | 順次 |
| 同タイムの場合 | 安定性（目視判断）で決定 |

### 計測方法

講師がストップウォッチで計測します:

```bash
sf competition hover-time --team "TeamName"
```

## 3. 参加コードの規定

### 使用できるコード

| 項目 | 規定 |
|------|------|
| ベースコード | Lesson 5（P制御）または Lesson 8（PID制御）で実装したコード |
| ゲイン調整 | 自由（Kp, Ki, Kd の値は各自最適化してよい） |
| ヨー制御 | 追加してよい |
| テレメトリ | `ws::telemetry_send()` の使用可 |
| LED 表示 | `ws::led_color()` の使用可 |

### 禁止事項

| 禁止 | 理由 |
|------|------|
| カスケード制御（姿勢→角速度） | Lesson 5/8 の範囲外 |
| 高度制御の自動化 | スロットルは手動操作 |
| 他の参加者のコードのコピー | 自分で実装したコードで競う |
| 機体の物理的改造 | 公平性の確保 |

### 推奨する最適化

- PID ゲインの調整（P → D → I の順でチューニング）
- アンチワインドアップの実装（I項の制限）
- D項のローパスフィルタ追加（ノイズ対策）
- テレメトリを活用したデータ駆動チューニング

## 4. 機体規定

| 項目 | 規定 |
|------|------|
| 機体 | StampFly 標準仕様のみ |
| プロペラ | 標準プロペラのみ |
| バッテリー | 標準バッテリーのみ |
| ファームウェア | Workshop Skeleton ベースのみ |
| 外部センサ | 追加不可 |

## 5. 安全規定

| ルール | 詳細 |
|--------|------|
| 保護メガネ | フライトエリア内の全員が着用 |
| フライトエリア | ネット/フェンスで囲った 3m x 3m 以上の空間 |
| バッテリー | 3.3V 以下で使用禁止 |
| 異常動作 | 即座に DISARM |
| 審判の指示 | 必ず従う |

> **Note / 注:** StampFly はプロペラガード一体型で、極めて小型のためプロペラの危険性は低いです。

## 6. タイムスケジュール

| 時間 | 内容 |
|------|------|
| 9:00-9:15 | ルール説明・機体確認 |
| 9:15-9:45 | 自由練習・最終チューニング |
| 9:45-10:45 | 競技本番（全員 3 回ずつ、ベストタイム採用） |
| 10:45-11:00 | 結果発表・表彰・講評 |
| 11:00-11:30 | アンケート記入・振り返り |

## 7. 進行のヒント（講師向け）

- 練習時間（30分）でゲイン最終調整の機会を与える
- テレメトリのリアルタイム表示をプロジェクターに映すと盛り上がる
- 60秒ホバリング達成者が出た場合は「完璧！」と称える
- 全員の記録を順位表にまとめてリアルタイムで更新する
- アンケートは紙 or Google Forms で準備（勉強会全体の振り返り）

---

<a id="english"></a>

## 1. Overview

Competition rulebook for the StampFly Workshop final day (Day 5). Participants compete using the angular rate feedback control programs they built during the workshop.

### Event

| Event | Description |
|-------|-------------|
| Hover Time | Compete for the longest hover using your own rate feedback controller |

## 2. Hover Time Competition

### Rules

| Item | Detail |
|------|--------|
| Objective | Take off and maintain hover as long as possible |
| Time limit | 60 seconds (maximum) |
| Attempts | 3 (best time counts) |
| Control | Controller-operated (manual throttle, attitude stabilized by your program) |
| Code | Angular rate feedback from Lessons 5/8 (P or PID control) |

### Hover Definition

Hovering is maintained while ALL of these conditions are met:

- Drone is airborne (off the ground)
- No contact with walls, ceiling, or obstacles
- Approximately within 10cm - 1.5m altitude range

### Hover Ends When

- Drone lands or crashes
- Contact with walls/ceiling
- Exits flight area
- Participant disarms
- Time limit (60s) reached

### Scoring

Rankings are determined by longest hover time. Ties broken by stability (visual judgment).

### Measurement

```bash
sf competition hover-time --team "TeamName"
```

## 3. Code Rules

### Allowed

- Code from Lesson 5 (P control) or Lesson 8 (PID control)
- Gain tuning (any Kp, Ki, Kd values)
- Yaw control addition
- Telemetry and LED usage

### Not Allowed

- Cascade control (attitude → rate)
- Automated altitude control
- Copying other participants' code
- Physical drone modifications

## 4. Safety

- Safety glasses required in flight area
- Flight area enclosed with nets/fences (3m x 3m minimum)
- Battery cutoff at 3.3V
- Immediately disarm if unstable
- Follow referee instructions

> **Note:** StampFly has built-in propeller guards and is small enough that propellers pose minimal risk.
