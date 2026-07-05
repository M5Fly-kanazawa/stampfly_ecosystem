# 04_read_imu — IMUのデータを読む

## 目的

SPI通信でBMI270 IMU（慣性計測装置）からデータを取得する方法を学びます。

## 必要な知識

- SPI通信の基本（MOSI, MISO, SCK, CS）
- 加速度とジャイロスコープの意味

## ハードウェア

| 部品 | 説明 |
|------|------|
| StampFly | BMI270 6軸IMU (SPI2) |

| 信号 | GPIO |
|------|------|
| MOSI | 14 |
| MISO | 43 |
| SCK | 44 |
| CS | 46 |

## 実行手順

```bash
cd firmware/vehicle/examples/04_read_imu
idf.py set-target esp32s3
idf.py build flash monitor
```

## 動作

加速度（g）と角速度（rad/s）の6軸データを10Hzでシリアルコンソールに表示します。

## 学べること

- BMI270Wrapper HALドライバの使い方
- SPI通信の初期化
- IMUデータの読み取りと表示
- 加速度と角速度の物理的意味
