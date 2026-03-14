# sf_algo_fusion

センサーフュージョンコンポーネント

## 役割

ESKFを用いたセンサーフュージョン（姿勢・位置推定）

## 入出力

- 入力: IMU(accel, gyro), OpticalFlow, Baro, ToF, Mag
- 出力: 姿勢(roll, pitch, yaw), 位置, 速度, バイアス

## 依存

- stampfly_eskf (ESKFアルゴリズム)
- stampfly_math (数学ライブラリ)
- FreeRTOS: **なし** (algo_*レイヤー)
