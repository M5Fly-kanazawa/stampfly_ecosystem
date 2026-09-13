# 制御工学用語集

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

言い方の規則（言い換え・禁止語・訳語）は [`../contributing/terminology.md`](../contributing/terminology.md) を参照。本用語集は語の**意味**の定義のみを扱う。
/ For word-choice rules (phrasing, banned terms, translation conventions), see [`../contributing/terminology.md`](../contributing/terminology.md). This glossary covers term **meaning** only.

## 1. 制御一般 / General Control

| 日本語 | English | 説明 |
|--------|---------|------|
| フィードバック制御 | Feedback control | 出力を測定して入力を調整する制御方式 |
| 開ループ制御 | Open-loop control | フィードバックなしの制御 |
| 閉ループ制御 | Closed-loop control | フィードバックありの制御 |
| 目標値 | Setpoint / Reference | 制御系の望ましい出力 |
| 偏差 | Error | 目標値と実際の出力の差 |
| 定常偏差 | Steady-state error | 十分時間が経った後の残留偏差 |
| 外乱 | Disturbance | 制御系に加わる望ましくない入力 |
| プラント | Plant | 制御対象 |

## 2. PID 制御 / PID Control

| 日本語 | English | 説明 |
|--------|---------|------|
| 比例制御 (P) | Proportional control | 偏差に比例した出力 |
| 積分制御 (I) | Integral control | 偏差の積分に比例した出力 |
| 微分制御 (D) | Derivative control | 偏差の微分に比例した出力 |
| 比例ゲイン | Proportional gain (Kp) | P 項の強さ |
| 積分時間 | Integral time (Ti) | I 項の時定数 |
| 微分時間 | Derivative time (Td) | D 項の時定数 |
| 不完全微分 | Incomplete derivative | 高周波を制限した微分 |
| アンチワインドアップ | Anti-windup | 積分飽和の防止機構 |

## 3. 応答特性 / Response Characteristics

| 日本語 | English | 説明 |
|--------|---------|------|
| ステップ応答 | Step response | ステップ入力に対する出力変化 |
| 立ち上がり時間 | Rise time | 10% から 90% に達する時間 |
| 整定時間 | Settling time | 目標値の ±2% に収まるまでの時間 |
| オーバーシュート | Overshoot | 目標値を超えるピーク量 |
| 安定性 | Stability | 出力が有界に保たれる性質 |
| ゲイン余裕 | Gain margin | 不安定になるまでのゲイン余裕 |
| 位相余裕 | Phase margin | 不安定になるまでの位相余裕 |

## 4. ドローン / Drone

| 日本語 | English | 説明 |
|--------|---------|------|
| 姿勢 | Attitude | 機体の傾き（ロール、ピッチ、ヨー）|
| ロール | Roll | 機体の左右方向の傾き |
| ピッチ | Pitch | 機体の前後方向の傾き |
| ヨー | Yaw | 機体の水平面内の回転 |
| 角速度 | Angular rate | 回転の速さ (rad/s) |
| 推力 | Thrust | モーターが発生する力 |
| 推重比 | Thrust-to-weight ratio | 最大推力と重力の比 |
| カスケード制御 | Cascade control | 多段ループの制御構造 |

## 5. センサ / Sensors

| 日本語 | English | 説明 |
|--------|---------|------|
| ジャイロスコープ | Gyroscope | 角速度を計測 |
| 加速度計 | Accelerometer | 加速度を計測 |
| 気圧センサ | Barometer | 気圧（高度）を計測 |
| ToF センサ | Time-of-Flight sensor | レーザー測距 |
| オプティカルフロー | Optical flow | 画像から速度を推定 |
| Allan 分散 | Allan variance | センサノイズの時間スケール分析 |
| ARW | Angle Random Walk | ジャイロのホワイトノイズ指標 |

## 6. 推定 / Estimation

| 日本語 | English | 説明 |
|--------|---------|------|
| カルマンフィルタ | Kalman filter | 最適状態推定フィルタ |
| ESKF | Error-State Kalman Filter | 誤差状態カルマンフィルタ |
| 相補フィルタ | Complementary filter | 2 つのセンサの簡易融合 |
| センサフュージョン | Sensor fusion | 複数センサの統合 |
| バイアス | Bias | センサの系統的誤差 |
| ドリフト | Drift | バイアスによる積分誤差の蓄積 |

---

<a id="english"></a>

## English Glossary

See the Japanese section above. Each table includes English terms alongside Japanese equivalents.
