# Lesson 9: Attitude Estimation

## Goal / 目標
Implement a complementary filter for roll/pitch estimation and compare it with the onboard ESKF.

相補フィルタを実装してロール/ピッチの推定を行い、機体搭載のESKFと比較する。

## API / 使用するAPI
| Function | Description | Unit |
|----------|-------------|------|
| `ws::gyro_x/y/z()` | Angular velocity | rad/s |
| `ws::accel_x/y/z()` | Linear acceleration | m/s^2 |
| `ws::estimated_roll()` | ESKF roll estimate | rad |
| `ws::estimated_pitch()` | ESKF pitch estimate | rad |
| `ws::print(fmt, ...)` | Serial print (Teleplot compatible) | - |

## Background / 背景

### Why Sensor Fusion? / なぜセンサフュージョン？

| Sensor | Strength | Weakness |
|--------|----------|----------|
| Gyroscope | Low noise, fast response | Drifts over time (integration error) |
| Accelerometer | No drift (gravity reference) | Noisy, affected by vibration/motion |

Neither sensor alone gives a good attitude estimate. By combining both, we get the best of each.

ジャイロは短期的に正確だがドリフトする。加速度センサはドリフトしないがノイズが多い。
両方を組み合わせることで、それぞれの長所を活かす。

### Complementary Filter / 相補フィルタ

```
angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
```

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `alpha` | 0.98 | Trust gyro 98% (short-term accuracy) |
| `1 - alpha` | 0.02 | Trust accelerometer 2% (long-term correction) |

### Block Diagram / ブロック図
```
  Gyroscope ──> [Integrate] ──> ┐
                                 ├──[alpha blend]──> Estimated Angle
  Accelerometer ──> [atan2] ──> ┘

  Detail:
  ┌─────────────────────────────────────────────────┐
  │                                                 │
  │  gyro ──> [× dt] ──> [+] ──> [× alpha] ──┐     │
  │                       ^                    │     │
  │                       │                    ├─[+]─┼──> angle
  │                       └── angle(prev)      │     │
  │                                            │     │
  │  accel ──> [atan2] ──> [× (1-alpha)] ─────┘     │
  │                                                 │
  └─────────────────────────────────────────────────┘
```

### Accelerometer Angles / 加速度からの角度計算
```cpp
accel_roll  = atan2f(-ay, -az);                       // Roll from gravity
accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az));   // Pitch from gravity
```

At rest the accelerometer reads $[a_x, a_y, a_z] = [g\sin\theta,\ -g\sin\phi\cos\theta,\ -g\cos\phi\cos\theta]$ in the body frame (FRD), so a level craft gives $a_z \approx -9.81$ m/s². Hence the minus signs: $\phi = \mathrm{atan2}(-a_y, -a_z)$, $\theta = \mathrm{atan2}(a_x, \sqrt{a_y^2 + a_z^2})$ (same as `solution.cpp`).

静止時の加速度の測定値は機体座標（FRD）で $[a_x, a_y, a_z] = [g\sin\theta,\ -g\sin\phi\cos\theta,\ -g\cos\phi\cos\theta]$ となり、水平なら $a_z \approx -9.81$ m/s²。そのため負号が付き、$\phi = \mathrm{atan2}(-a_y, -a_z)$、$\theta = \mathrm{atan2}(a_x, \sqrt{a_y^2 + a_z^2})$ となる（`solution.cpp` と同じ）。

These are valid only when the drone is **not accelerating** (static or constant velocity).

加速度から角度を計算できるのは、機体が加速していない（静止または等速直線運動）場合のみ。

### Complementary Filter vs ESKF / 相補フィルタとESKFの比較
| Feature | Complementary Filter | ESKF |
|---------|---------------------|------|
| Complexity | Simple (2 lines) | Complex (matrix math) |
| Tuning | 1 parameter (alpha) | Process/measurement noise |
| Accuracy | Good for static/slow | Better under vibration |
| Magnetometer | Not used | Uses mag for yaw |
| CPU cost | Minimal | Higher |

## Teleplot Setup / Teleplotセットアップ

### What is Teleplot? / Teleplotとは？

Teleplot is a VSCode extension that visualizes serial output as real-time graphs.
Simply print data in the format `>variable_name:value` and Teleplot graphs it automatically.

TeleplotはVSCode拡張機能で、シリアル出力をリアルタイムグラフとして可視化します。
`>変数名:値` の形式でprintするだけで自動的にグラフ化されます。

### Setup / セットアップ手順

1. Install VSCode extension: `alexnesnes.teleplot`
2. Connect via `sf monitor`
3. Open Teleplot panel in VSCode
4. Data in `>name:value` format will be graphed automatically

### Output Format / 出力フォーマット
```cpp
// Teleplot format: >variable_name:value
ws::print(">cf_roll:%.2f", cf_roll * 57.3f);
ws::print(">cf_pitch:%.2f", cf_pitch * 57.3f);
ws::print(">eskf_roll:%.2f", eskf_roll * 57.3f);
ws::print(">eskf_pitch:%.2f", eskf_pitch * 57.3f);
```

**Decimation:** Output at 100Hz (every 4 ticks) to avoid serial bandwidth overload.

## Steps / 手順
1. `sf lesson switch 9`
2. Compute accelerometer-based roll and pitch angles using `atan2f`
3. Implement the complementary filter with `alpha = 0.98`
4. Add Teleplot output for CF and ESKF angles
5. Tilt the drone by hand and compare the two estimates in Teleplot
6. Try different alpha values (0.9, 0.99) and observe the effect
7. View telemetry: `sf monitor` + Teleplot

## Challenge / チャレンジ
- Try alpha = 0.5 (equal trust) and observe the noise
- Shake the drone rapidly and see which estimate is more stable
- Why does the complementary filter not estimate yaw? (Hint: gravity is vertical)

## Key Concepts / キーコンセプト
- Sensor fusion combines multiple noisy sensors for better estimates
- Complementary filter is the simplest sensor fusion algorithm
- Alpha controls the trade-off between noise rejection and drift correction
- ESKF (Error-State Kalman Filter) is the production-grade approach
- `57.3f` converts radians to degrees (180/pi)
- Teleplot enables real-time visualization without extra tools
