# 03_button_event — ボタンでLEDを制御する

## 目的

GPIO入力の読み取りとソフトウェアデバウンスの仕組みを学びます。

## 必要な知識

- GPIO入力（プルアップ、アクティブLOW）
- チャタリング（バウンス）とは何か

## ハードウェア

| 部品 | 説明 |
|------|------|
| M5Stamp S3 | ボタン (GPIO 0, アクティブLOW) |
| M5Stamp S3 | MCU LED (GPIO 21, WS2812) |

## 実行手順

```bash
cd firmware/vehicle/examples/03_button_event
idf.py set-target esp32s3
idf.py build flash monitor
```

## 動作

- ボタンを押すとLEDが緑に点灯
- ボタンを離すとLEDが消灯
- シリアルモニタに押下回数を表示

## 学べること

- `gpio_config` によるGPIO入力設定
- ソフトウェアデバウンスのアルゴリズム
- ポーリングによるイベント検出
