# 01_blink_led — MCU内蔵LEDを光らせる

## 目的

ESP-IDFの基本的なプロジェクト構造と、WS2812 RGB LEDの制御方法を学びます。

## 必要な知識

- ESP-IDFプロジェクトのビルド・書き込み方法
- C/C++の基本文法

## ハードウェア

| 部品 | 説明 |
|------|------|
| M5Stamp S3 | MCU内蔵WS2812 LED (GPIO 21) |

## 実行手順

```bash
cd firmware/vehicle/examples/01_blink_led
idf.py set-target esp32s3
idf.py build flash monitor
```

## 動作

LEDが赤→緑→青の順に1秒間隔で切り替わります。

## 学べること

- `led_strip` APIによるWS2812制御
- FreeRTOS `vTaskDelay` による時間待ち
- ESP-IDFのログマクロ (`ESP_LOGI`)
