# 07_motor_spin — モーターを回す

## 目的

LEDC PWMでブラシレスモーターを低速で駆動する方法と、安全機構の実装を学びます。

## 必要な知識

- PWMによるモーター制御の基本
- StampFlyのモーター配置（X-quadコンフィグ）

## ハードウェア

| 部品 | 説明 |
|------|------|
| StampFly | モーター M1 (GPIO 42, Front-Right) |
| StampFly | ボタン (GPIO 0, 緊急停止) |

| モーター | GPIO | 位置 | 回転方向 |
|---------|------|------|---------|
| M1 | 42 | Front-Right | CCW |
| M2 | 41 | Rear-Right | CW |
| M3 | 10 | Rear-Left | CCW |
| M4 | 5 | Front-Left | CW |

## 安全上の注意

- **必ずプロペラを外してから実行してください**
- デューティサイクルは20%に制限されています
- ボタン（GPIO 0）で緊急停止できます

## 実行手順

```bash
cd firmware/vehicle_new/examples/07_motor_spin
idf.py set-target esp32s3
idf.py build flash monitor
```

## 動作

1. 3秒のカウントダウン後、M1がゆっくり回転を開始
2. 20%デューティまで段階的に上昇
3. ボタンを押すと即座に停止

## 学べること

- LEDC PWMによるモーター制御（150kHz, 8-bit）
- 安全機構（デューティ制限、緊急停止、段階的起動）
- GPIO入力とモーター出力の組み合わせ
