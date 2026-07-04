# sf_algo_fusion

センサーフュージョンコンポーネント

## 役割

ESKFを用いたセンサーフュージョン（姿勢・位置・速度推定）のオーケストレーション。
ESKF への入力ゲーティングと品質閾値チェックを担当。

## 入出力

- 入力:
  - IMU (accel, gyro) — 必須、400Hz
  - ToF — オプション、30Hz
  - Baro — オプション、50Hz
  - OpticalFlow — オプション、100Hz
  - Mag — オプション、100Hz
- 出力: 姿勢(roll, pitch, yaw), 位置, 速度, ジャイロバイアス, 加速度バイアス

## センサ制御

- センサ ON/OFF は `ESKF::Config::sensor_enabled[]` で一元管理
- SensorFusion は `eskf_.isSensorEnabled()` でゲート判定
- 品質閾値（SQUAL、距離範囲）は `SensorThresholds` 構造体で管理

## 依存

- sf_algo_eskf (ESKFアルゴリズム)
- sf_algo_math (数学ライブラリ)
- FreeRTOS: **なし** (algo_*レイヤー)
