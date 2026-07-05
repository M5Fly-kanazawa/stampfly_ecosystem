# 06_read_baro — 気圧センサで高度を表示する

## 目的

BMP280気圧センサからデータを取得し、気圧から高度を計算する方法を学びます。

## 必要な知識

- I2C通信の基本
- 気圧と高度の関係（国際標準大気）

## ハードウェア

| 部品 | 説明 |
|------|------|
| StampFly | BMP280 気圧センサ (I2C, addr=0x76) |

| 信号 | GPIO |
|------|------|
| SDA | 3 |
| SCL | 4 |

## 実行手順

```bash
cd firmware/vehicle/examples/06_read_baro
idf.py set-target esp32s3
idf.py build flash monitor
```

## 動作

気圧（hPa）、温度（C）、推定高度（m）を5Hzでシリアルコンソールに表示します。

## 学べること

- BMP280 HALドライバの使い方
- 気圧→高度の変換公式
- センサのオーバーサンプリング設定
- 温度補償の役割
