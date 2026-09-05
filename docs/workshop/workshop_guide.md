# StampFly 勉強会 講師ガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このガイドについて

本ガイドは StampFly 勉強会（4+1日間）の講師向け運営マニュアルです。Workshop Skeleton ファームウェアと sf CLI を使用した教育カリキュラムの進め方を説明します。

### カリキュラム設計思想

| 原則 | 説明 |
|------|------|
| シンプルさ | 学生は `setup()` / `loop_400Hz()` の 2 関数だけ実装 |
| 段階的学習 | Lesson 0（Hello World）→ Lesson 11（競技会）まで段階的に難易度が上がる |
| ツール活用 | sf CLI / WiFi テレメトリ / Python SDK を各レッスンに統合 |
| 実践重視 | 理論 30% : 実習 70% の配分 |

## 2. 事前準備

### 環境構築チェックリスト

各学生の PC で以下を確認:

```bash
# 1. 開発環境のセットアップ
source setup_env.sh
idf.py --version    # v5.4 以上

# 2. sf CLI 動作確認
sf doctor

# 3. Workshop ビルド確認
sf lesson switch 0
sf lesson build

# 4. Python 環境（Day 4 用）
python3 -c "import stampfly_sdk"
```

### 機材チェックリスト

- [ ] StampFly 全台の動作確認（モータ回転、LED 点灯）
- [ ] コントローラ全台の充電・ペアリング確認
- [ ] 予備バッテリーの充電
- [ ] 安全ネット / フェンスの設置
- [ ] WiFi ルーター（開発用）の設置
- [ ] プロジェクター / スクリーンの準備

## 3. レッスン運営ガイド

### 各レッスンの進め方

```
1. 講義（10-15分）: 理論・概念の説明
2. デモ（5分）: solution.cpp を使ったデモフライト
3. 実習（30-40分）: student.cpp の TODO を埋める
4. テスト（10-15分）: ビルド → フラッシュ → 動作確認
5. まとめ（5分）: 質疑応答
```

### Lesson 0: 環境セットアップ

**目的:** ビルド → フラッシュ → シリアルモニタの流れを体験

**手順:**
1. `sf lesson switch 0` で Lesson 0 に切り替え
2. `sf lesson build` でビルド
3. `sf lesson flash` でフラッシュ（モニタ自動起動）
4. シリアルモニタで `"Hello StampFly!"` の出力を確認

**よくある問題:**
- ポート未検出 → USB ケーブルがデータ対応か確認、`sf doctor` 実行
- ビルドエラー → ESP-IDF のバージョン確認、`sf lesson build -c` でクリーンビルド

### Lesson 1: モータ制御

**目的:** PWM 制御の基礎、モータ番号と配置の理解

**安全注意:**
- duty は 0.2 以下で開始（低推力で安全に確認）
- モータの回転方向を確認
- 機体を手で押さえた状態でテスト

**確認ポイント:**
- 各モータが個別に回転するか
- duty 値と回転速度の関係を理解しているか

### Lesson 2: コントローラ入力

**目的:** ESP-NOW 通信、スティック値のマッピング

**手順:**
1. コントローラの電源を入れペアリング
2. `ws::rc_throttle()` でスロットル値をシリアル出力
3. スティック操作とモータ出力の対応を実装

### Lesson 3: LED 表示

**目的:** 状態表示の実装

**ポイント:** ARM/DISARM 状態を色で表現する

### Lesson 4: IMU センサ

**目的:** ジャイロ/加速度センサの読み取りとテレメトリ送信

**デモ:** `sf log wifi` で WiFi テレメトリを取得し、リアルタイムでセンサ値を確認

### Lesson 5: P 制御 + 初フライト

**安全注意:**
- 初フライトは広い場所で実施
- 保護メガネ必須
- Kp は小さい値（0.5 程度）から開始
- 講師が横で監視

**手順:**
1. 角速度偏差を計算
2. Kp を掛けてモータミキサーに入力
3. Kp = 0.5 からテスト、徐々に上げる

### Lesson 6: システムモデリング

**前提知識:** ラプラス変換、伝達関数の基礎

**ポイント:**
- モータ+プロペラ+機体の伝達関数 G_p(s) = K / (s·(τm·s + 1)) を導出
- 軸ごとのプラントゲイン K（Roll/Pitch/Yaw）の計算
- 設計式 Kp = 1 / (4·ζ²·K·τm) による Kp 設計

**ツール:** Loop Shaping Tool でボード線図を確認

### Lesson 7: システム同定

**デモ:**
```bash
# WiFi テレメトリ取得
sf log wifi

# ログ解析
sf log analyze <logfile>
```

**ポイント:**
- 2 種類の Kp モード（L5 一律 vs L6 モデルベース）で飛行
- オーバーシュート比から減衰比 ζ を推定
- 理論値と実測値を比較してモデルの妥当性を検証

### Lesson 8: PID 制御

**ポイント:**
- L6 のモデルベース Kp を起点に I 項・D 項を追加
- I 項のアンチワインドアップが重要
- D 項のノイズ対策（ローパスフィルタ）
- ゲイン調整の体系的アプローチ: P → D → I の順

**チューニングのコツ:**
1. モデルベース Kp（Roll:0.25, Pitch:0.36）で開始
2. D ゲインで振動を抑える
3. 最後に I ゲインを少しずつ追加

### Lesson 9: 姿勢推定

**ポイント:**
- 相補フィルタの直感的理解（ジャイロは短期信頼、加速度は長期信頼）
- ESKF との性能比較

### Lesson 10: Python SDK

**環境:** Jupyter Notebook を使用

**手順:**
1. `connect_or_simulate()` でシミュレータに接続
2. RC 制御コマンドでホバリング
3. テレメトリ取得で外部 PID を実装

### Lesson 11: ホバリングタイム競技

**内容:** 競技ルール説明 + PIDゲイン最適化 + 自由練習

**目的:** Lesson 5/8 で実装した角速度フィードバック制御のゲインを最適化し、最長ホバリングタイムを競う

**ポイント:**
- カスケード制御は禁止（角速度フィードバックのみ）
- スロットルは手動操作（高度制御の自動化は禁止）
- P → D → I の順でチューニング
- テレメトリで効果を確認しながら調整

### 学習者コードを SILS で試す（Lesson 5 / 8 / 11 補足）

Lesson 5・8・11 で書く `setup()`/`loop_400Hz()`（`main/user_code.cpp`）は、実機フラッシュ前に SILS（`simulator/sils/`、`--target workshop`）上でも走らせられる。実機に書き込まれるのと同じソースがそのまま動く（Code Identity）ので、ARM・状態遷移（ARMED_GROUND→TAKEOFF→FLYING）・モータ応答の配線ミスは実機を壊さずに気付ける。

```bash
sf sils build --target workshop                  # 初回のみ
sf lesson switch 8 --solution                    # 動作確認したいレッスンに切替
touch firmware/workshop/main/user_code.cpp       # 切替は更新日時を保持するため touch する
sf sils build --target workshop                  # touch 後にもう一度ビルドして反映
sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop
```

**手順の注意:** `sf lesson switch` は `student.cpp`/`solution.cpp` をコピーする際に更新日時を保持したままにするため、切替直後にそのまま `sf sils build --target workshop` を実行しても新しいコードとして認識されない。上記のとおり `touch firmware/workshop/main/user_code.cpp` で更新日時を進めてから**もう一度**再ビルドする、という2段階を毎回踏むこと（省略できる例外ではなく通常の手順）。また、`workshop_acro.scn` のスティック指令（duty）は Lesson 5/8 のレート制御解答が実測ホバー duty（~0.676）付近で中立に収束する前提でチューニングされている。旧 `acro_flight.scn`（vehicle 用の推力比率規約で調整済み）の値を workshop の生 duty として流用すると約35%不足し離陸しないため、離陸+ホバー相当のデモには本シナリオを使うこと。Lesson 5/8 では ARM→離陸検知→ホバー相当巡航→空中 DISARM の一連を確認できる（合格基準: 真値高度が0.1mを超える、roll+pitch合成傾きが15°未満）。Lesson 11（独自ファームウェア開発）は制御則が学習者ごとに異なるため、同じ duty で離陸する保証はない — ARM・モード遷移・モータ応答の配線確認が主な用途になる。詳細は `simulator/sils/README.md`「エミュレータターゲット」節を参照。

## 4. 安全管理

### 基本ルール

| ルール | 詳細 |
|--------|------|
| 保護メガネ | フライト時は全員着用 |
| フライトエリア | ネット/フェンスで囲う |
| バッテリー | 3.3V 以下で使用禁止 |
| 地上テスト | 低 duty（0.2以下）で機体を手で押さえてテスト |
| 緊急停止 | コントローラの DISARM ボタンを常に意識 |

### 事故対応

1. 即座にモータ停止（DISARM）
2. バッテリーを外す
3. 機体の破損確認
4. 人的被害の確認
5. 原因調査

## 5. トラブルシューティング

### よくある問題

| 症状 | 原因 | 対処 |
|------|------|------|
| ビルドエラー | 開発環境未初期化 | `source setup_env.sh` |
| ポート未検出 | USB ケーブル | データ対応ケーブルに交換 |
| フラッシュ失敗 | Boot モード | BOOT ボタン押しながらリセット |
| モータ不安定 | PID ゲイン過大 | P ゲインを 50% に下げる |
| ドリフト | IMU バイアス | `sf cal gyro` でキャリブレーション |
| WiFi テレメトリ切断 | 距離/干渉 | WiFi チャンネル変更、距離を近づける |

---

<a id="english"></a>

## 1. Overview

### About This Guide

This is the instructor's manual for the StampFly Workshop (4+1 days). It covers how to run the educational curriculum using the Workshop Skeleton firmware and sf CLI.

### Curriculum Design Principles

| Principle | Description |
|-----------|-------------|
| Simplicity | Students only implement `setup()` / `loop_400Hz()` |
| Progressive | Difficulty increases from Lesson 0 to Lesson 11 |
| Tool integration | sf CLI, WiFi telemetry, Python SDK in each lesson |
| Practice-focused | 30% theory : 70% hands-on |

## 2. Preparation

### Environment Checklist

```bash
source setup_env.sh
sf doctor
sf lesson switch 0 && sf lesson build
```

## 3. Lesson Guides

Each lesson follows: Lecture (10-15 min) → Demo (5 min) → Hands-on (30-40 min) → Test (10-15 min) → Wrap-up (5 min).

See the Japanese section above for detailed lesson-by-lesson instructions.

**Trying learner code in SILS (Lessons 5/8/11):** the exact `main/user_code.cpp` a student writes can run on the host SILS bench before it ever touches hardware:

```bash
sf sils build --target workshop                  # once
sf lesson switch 8 --solution                     # switch to the lesson you want to check
touch firmware/workshop/main/user_code.cpp        # switching preserves the mtime, so touch it
sf sils build --target workshop                   # rebuild again so the new code is picked up
sf sils scenario simulator/sils/scenarios/workshop_acro.scn --target workshop
```

`sf lesson switch` copies `student.cpp`/`solution.cpp` while preserving their old mtime, so a build run right after the switch will not detect the new code as changed. Touching the file and then rebuilding a **second** time is a required two-step sequence every time — not an occasional workaround. (See `simulator/sils/README.md`'s "Emulator targets" section; use `workshop_acro.scn`, not `acro_flight.scn` — the latter's stick values are tuned for vehicle's thrust-fraction convention and undershoot workshop's raw-duty convention by about 35%, so the craft never lifts off.) With `workshop_acro.scn`, Lessons 5/8's rate-control solutions do lift off and hold a hover-equivalent attitude (pass criteria: true altitude clears 0.1 m, combined roll+pitch tilt stays under 15°) before an in-air DISARM. Lesson 11 (open-ended custom firmware) has no such guarantee, since the control law varies by student — there, the scenario is mainly useful for catching ARM/state-machine/motor-wiring mistakes safely.

## 4. Safety

- Safety glasses required during flight
- Flight area must be enclosed
- Battery cutoff at 3.3V
- Ground tests at low duty (hold drone by hand)
- Always be ready to DISARM

## 5. Troubleshooting

See the Japanese troubleshooting table above for common issues and solutions.
