# 02_buzzer_melody — ブザーでメロディを鳴らす

## 目的

ESP-IDFのLEDC PWMペリフェラルを使って、ブザーからトーンを生成する方法を学びます。

## 必要な知識

- PWM（パルス幅変調）の基本概念
- 周波数と音階の関係

## ハードウェア

| 部品 | 説明 |
|------|------|
| StampFly | ブザー (GPIO 40) |

## 実行手順

```bash
cd firmware/vehicle_new/examples/02_buzzer_melody
idf.py set-target esp32s3
idf.py build flash monitor
```

## 動作

Cメジャースケール（ドレミファソラシド）を繰り返し再生します。

## 学べること

- LEDC PWMの初期化（タイマー、チャネル）
- PWM周波数の変更によるトーン生成
- デューティサイクルと音量の関係
