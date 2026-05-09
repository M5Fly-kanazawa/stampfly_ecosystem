# StampFly 勉強会 スケジュール (4+1日)

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

| 項目 | 内容 |
|------|------|
| 日程 | 4日間講義/実習 + 1日競技会 |
| 対象 | 大学生・大学院生（制御工学/ロボティクス） |
| 前提 | C/C++ 基礎、Python 基礎 |
| 機材 | StampFly 1台/人、コントローラ 1台/人、ノートPC |

## 2. Day 1: エコシステム紹介 + ハードウェア入門 (6h)

### 午前 (9:00-12:00)

| 時間 | 内容 | ツール/API | 備考 |
|------|------|-----------|------|
| 9:00-10:00 | **エコシステム全体紹介** | スライド | リポジトリ構成、設計思想、ツール群デモ |
| 10:00-11:00 | **Lesson 0: 環境セットアップ** | `sf doctor`, `sf lesson build`, `sf lesson flash` | ビルド＆フラッシュの動作確認 |
| 11:00-12:00 | **Lesson 1: モータ制御** | `ws::motor_set_duty()`, `sf monitor` | PWM、duty cycle、モータ番号 |

### 午後 (13:00-16:00)

| 時間 | 内容 | ツール/API | 備考 |
|------|------|-----------|------|
| 13:00-14:00 | **Lesson 2: コントローラ入力** | `ws::rc_*()` | ESP-NOW、スティック値、開ループ制御 |
| 14:00-15:00 | **Lesson 3: LED 表示** | `ws::led_color()` | WS2812、ARM 状態表示 |
| 15:00-16:00 | **Lesson 4: IMU センサ取得** | `ws::gyro_*()`, Teleplot, `sf monitor` | 座標系、Teleplot 可視化 |

### エコシステム紹介の内容 (Day 1 冒頭 1時間)

- リポジトリ構成図（firmware / control / analysis / tools / simulator / protocol の6層）
- 設計思想: 関心の分離、HAL / SVC / Algo 3層構成
- ツール群デモ: `sf doctor` → `sf sim list` → `sf cal list` → `sf log wifi`
- 旧 M5StampFly_skeleton vs 新 ecosystem の違い
- ワークショップでの使い方: スケルトン（C++ 制御学習）+ sf CLI + Python SDK
- 将来の発展: 15回大学カリキュラム、ROS2 連携、Genesis シミュレータ

## 3. Day 2: フィードバック制御 (6h)

### 午前 (9:00-12:00)

| 時間 | 内容 | ツール/API | 備考 |
|------|------|-----------|------|
| 9:00-10:30 | **Lesson 5: 角速度 P 制御 + 初フライト** | `ws::motor_mixer()` | P 制御実装、初フライト |
| 10:30-12:00 | **Lesson 6: システムモデリング** | 伝達関数、設計式 | モデルの導出、Kp 設計 |

### 午後 (13:00-16:00)

| 時間 | 内容 | ツール/API | 備考 |
|------|------|-----------|------|
| 13:00-14:30 | **Lesson 7: システム同定** | `sf log wifi`, `sf log analyze` | 2モードで飛行、モデル検証 |
| 14:30-16:00 | 自由チューニング + フライトテスト | Kp 調整 | 各自フライトテスト |

## 4. Day 3: 制御理論 + 応用 (6h)

### 午前 (9:00-12:00)

| 時間 | 内容 | ツール/API | 備考 |
|------|------|-----------|------|
| 9:00-10:30 | **Lesson 8: PID 制御** | I 項/D 項追加 | モデルベース Kp + アンチワインドアップ |
| 10:30-12:00 | **Lesson 9: 姿勢推定** | 相補フィルタ + Teleplot | CF 実装 + Madgwick/EKF/ESKF 紹介 |

### 午後 (13:00-16:00)

| 時間 | 内容 | ツール/API | 備考 |
|------|------|-----------|------|
| 13:00-14:30 | **Lesson 10: API 総覧とセンサ活用** | StampFlyState, `sf app new`, Teleplot | 全 API + 独自ファーム作成 |
| 14:30-16:00 | 自由練習 + エコシステム深掘り | `sf sysid`, `sf cal`, ESKF Sim | デモ + 自由練習 |

## 5. Day 4: Python SDK + 競技会準備 (6h)

### 午前 (9:00-12:00)

| 時間 | 内容 | ツール/API | 備考 |
|------|------|-----------|------|
| 9:00-12:00 | **Lesson 11: Python SDK プログラム飛行** | `StampFly`, `connect_or_simulate()`, Jupyter | 将来像解説、Tello 互換 |

### 午後 (13:00-16:00)

| 時間 | 内容 | ツール/API | 備考 |
|------|------|-----------|------|
| 13:00-13:30 | **Lesson 12: 精密着陸競技会ルール説明** | スライド | 3m 先のヘリポートに着陸 |
| 13:30-16:00 | 競技会準備・自由練習・予選 | 全ツール | |

## 6. Day 5: 精密着陸競技会 + アンケート (半日)

### 種目

| 種目 | 内容 | 評価基準 | ツール |
|------|------|---------|--------|
| 精密着陸 | パイロット定位置から3m先のヘリポートに着陸 | 着陸タイム（ベスト採用） | Teleplot, `sf monitor` |

### タイムスケジュール

| 時間 | 内容 |
|------|------|
| 9:00-9:15 | ルール説明・機体確認 |
| 9:15-9:45 | 自由練習・最終チューニング |
| 9:45-10:45 | 競技本番（全員 3 回ずつ、ベストタイム採用） |
| 10:45-11:00 | 結果発表・表彰・講評 |
| 11:00-11:30 | アンケート記入・振り返り |

## 7. 準備物

### 講師側

| 項目 | 数量 | 備考 |
|------|------|------|
| StampFly 予備機 | 2-3台 | 故障対応用 |
| 予備プロペラ | 20セット | |
| 予備バッテリー | 人数分 | |
| 充電器 | 3-4台 | |
| 安全ネット/フェンス | 適量 | フライトエリア囲い |
| ヘリポートマット | 2枚 | 40cm × 40cm |
| プロジェクター | 1台 | |
| WiFi ルーター | 1台 | 開発環境用（StampFly WiFi とは別） |

### 学生側

| 項目 | 備考 |
|------|------|
| ノートPC | ESP-IDF + Python インストール済み |
| USB-C ケーブル | 書き込み用 |
| 保護メガネ | フライト時必須 |

---

<a id="english"></a>

## 1. Overview

| Item | Detail |
|------|--------|
| Schedule | 4 days lecture/lab + 1 day competition |
| Target | University students (control engineering / robotics) |
| Prerequisites | C/C++ basics, Python basics |
| Equipment | 1 StampFly + 1 controller + laptop per student |

## 2. Day 1: Ecosystem Introduction + Hardware Basics (6h)

| Time | Content | Tools |
|------|---------|-------|
| 9:00-10:00 | Ecosystem overview | Slides |
| 10:00-11:00 | Lesson 0: Environment setup | `sf doctor`, `sf build`, `sf flash` |
| 11:00-12:00 | Lesson 1: Motor control | `ws::motor_set_duty()` |
| 13:00-14:00 | Lesson 2: Controller input | `ws::rc_*()` |
| 14:00-15:00 | Lesson 3: LED display | `ws::led_color()` |
| 15:00-16:00 | Lesson 4: IMU sensors | `ws::gyro_*()`, Teleplot |

## 3. Day 2: Feedback Control (6h)

| Time | Content | Tools |
|------|---------|-------|
| 9:00-10:30 | Lesson 5: P control + first flight | `ws::motor_mixer()` |
| 10:30-12:00 | Lesson 6: System Modeling | Transfer function, Kp design |
| 13:00-14:30 | Lesson 7: System Identification | `sf log wifi`, model verification |
| 14:30-16:00 | Free tuning + flight test | Kp tuning |

## 4. Day 3: Control Theory + Applications (6h)

| Time | Content | Tools |
|------|---------|-------|
| 9:00-10:30 | Lesson 8: PID control | Model-based Kp + anti-windup |
| 10:30-12:00 | Lesson 9: Attitude estimation | Complementary filter + Teleplot, Madgwick/EKF/ESKF intro |
| 13:00-14:30 | Lesson 10: API overview + sensor exploration | StampFlyState, `sf app new`, Teleplot |
| 14:30-16:00 | Free practice + ecosystem deep dive | `sf sysid`, `sf cal`, ESKF Sim |

## 5. Day 4: Python SDK + Competition Prep (6h)

| Time | Content | Tools |
|------|---------|-------|
| 9:00-12:00 | Lesson 11: Python SDK programmatic flight | `StampFly`, Tello compat, Jupyter |
| 13:00-13:30 | Lesson 12: Precision landing competition rules | Slides |
| 13:30-16:00 | Competition prep / qualifying | All tools |

## 6. Day 5: Precision Landing Competition + Survey (half day)

| Time | Content |
|------|---------|
| 9:00-9:15 | Rules explanation, drone check |
| 9:15-9:45 | Free practice, final tuning |
| 9:45-10:45 | Competition (3 attempts each, best time counts) |
| 10:45-11:00 | Results, awards, wrap-up |
| 11:00-11:30 | Survey and feedback |
