# オプティカルフローセンサー検証実験計画

## 目的

オプティカルフローセンサーの軸マッピング・スケール・符号を正確に特定し、ESKFの位置推定精度を向上させる。

---

## 座標系定義

### ボディ座標系 (NED準拠)
- **X軸**: 機体前方 (+X = 前進)
- **Y軸**: 機体右方 (+Y = 右移動)
- **Z軸**: 機体下方 (+Z = 下降)

### フローセンサー出力
- **flow_dx**: センサーX軸方向の角度変位 [rad]
- **flow_dy**: センサーY軸方向の角度変位 [rad]

---

## 実験1: 単軸移動テスト（基本検証）

### 1-1. X軸テスト（前後移動）

```
開始位置: 原点 (0, 0)
動作: 前方へ20cm移動 → 停止(2秒) → 後方へ20cm移動（元の位置へ）
期待: X: 0 → +20cm → 0cm, Y: 0cm固定
所要時間: 約10秒
```

**手順:**
1. 機体を水平に保持、Yaw=0°（前方向き）
2. 高度30-40cmを維持
3. ゆっくり前方へ20cm移動（約2秒）
4. 2秒間停止
5. ゆっくり後方へ20cm移動（約2秒）
6. 2秒間停止してログ終了

### 1-2. Y軸テスト（左右移動）

```
開始位置: 原点 (0, 0)
動作: 右へ20cm移動 → 停止(2秒) → 左へ20cm移動（元の位置へ）
期待: X: 0cm固定, Y: 0 → +20cm → 0cm
所要時間: 約10秒
```

**手順:**
1. 機体を水平に保持、Yaw=0°（前方向き）
2. 高度30-40cmを維持
3. ゆっくり右方へ20cm移動（約2秒）
4. 2秒間停止
5. ゆっくり左方へ20cm移動（約2秒）
6. 2秒間停止してログ終了

### 確認項目

- [ ] flow_dx と flow_dy のどちらが反応するか
- [ ] 符号（+/-）が移動方向と一致するか
- [ ] 20cm移動時の積分値（スケール確認）

---

## 実験2: 正方形軌道テスト（統合検証）

### 2-1. 時計回り正方形

```
開始位置: 左前角 (0, 0)
動作: 右(+Y) → 後(-X) → 左(-Y) → 前(+X) → 元の位置
一辺: 20cm
Yaw: 0°固定（機体向き一定）
所要時間: 約20秒
```

**軌道図:**
```
    +X (前方)
      ↑
      |
  +---+---+
  |   |   |
  | 4 | 1 |  → +Y (右方)
  |   |   |
  +---●---+
  |   |   |
  | 3 | 2 |
  |   |   |
  +---+---+

● = 開始位置 (0, 0)
1 = 右移動 (0,0) → (0, +20cm)
2 = 後移動 (0, +20cm) → (-20cm, +20cm)
3 = 左移動 (-20cm, +20cm) → (-20cm, 0)
4 = 前移動 (-20cm, 0) → (0, 0)
```

### 2-2. 反時計回り正方形

```
開始位置: 左前角 (0, 0)
動作: 後(-X) → 右(+Y) → 前(+X) → 左(-Y) → 元の位置
一辺: 20cm
Yaw: 0°固定
```

---

## 実験3: 床面条件テスト

異なる床面でのフローセンサー性能を比較する。

| 条件 | 説明 | 期待される影響 |
|------|------|----------------|
| A | 白い無地の床 | 低コントラスト、精度低下の可能性 |
| B | 木目調フローリング | 適度なテクスチャ、標準条件 |
| C | タイル/模様あり | 高コントラスト、良好な精度 |
| D | カーペット | テクスチャあり、ただし暗い可能性 |

各条件で実験1-1と1-2を実施し、スケール値の変動を確認する。

---

## 実験条件

### 固定パラメータ

| パラメータ | 値 | 備考 |
|------------|-----|------|
| 高度 | 30-40cm | ToFで確認 |
| 移動速度 | 約10cm/s | ゆっくり、手動移動 |
| 姿勢 | Roll/Pitch/Yaw ≈ 0° | 水平維持 |
| 照明 | 一定 | 室内照明 |
| 移動距離 | 20cm | 測定しやすい距離 |

### ログ取得項目

| センサー | サンプリングレート | 記録項目 |
|----------|-------------------|----------|
| IMU | 400Hz | gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z |
| Flow | 100Hz | flow_dx, flow_dy |
| ToF | 50Hz | height |
| ESKF | 100Hz | position, velocity, attitude, biases |

---

## データ分析手順

### Step 1: 生データ確認

```python
import matplotlib.pyplot as plt

# 各センサーの時系列プロット
fig, axes = plt.subplots(3, 1, figsize=(12, 8))

axes[0].plot(time, flow_dx, label='flow_dx')
axes[0].plot(time, flow_dy, label='flow_dy')
axes[0].set_ylabel('Flow [rad]')
axes[0].legend()

axes[1].plot(time, gyro_x, label='gyro_x')
axes[1].plot(time, gyro_y, label='gyro_y')
axes[1].plot(time, gyro_z, label='gyro_z')
axes[1].set_ylabel('Gyro [rad/s]')
axes[1].legend()

axes[2].plot(time, height)
axes[2].set_ylabel('Height [m]')
axes[2].set_xlabel('Time [s]')

plt.tight_layout()
plt.savefig('raw_data_check.png')
```

### Step 2: 軸マッピング確認

**判定基準:**

```
X軸テスト（前後移動）時:
  → flow_dx が主に反応 → flow_dx = X軸成分
  → flow_dy が主に反応 → flow_dy = X軸成分（軸入れ替え必要）

Y軸テスト（左右移動）時:
  → flow_dx が主に反応 → flow_dx = Y軸成分
  → flow_dy が主に反応 → flow_dy = Y軸成分

符号確認:
  前進(+X)時に flow > 0 → 符号正しい
  前進(+X)時に flow < 0 → 符号反転必要
  右移動(+Y)時に flow > 0 → 符号正しい
  右移動(+Y)時に flow < 0 → 符号反転必要
```

**結果記録テンプレート:**

```
X軸テスト結果:
  前進時: flow_dx = [+/-]_____, flow_dy = [+/-]_____
  後退時: flow_dx = [+/-]_____, flow_dy = [+/-]_____

Y軸テスト結果:
  右移動時: flow_dx = [+/-]_____, flow_dy = [+/-]_____
  左移動時: flow_dx = [+/-]_____, flow_dy = [+/-]_____

軸マッピング結論:
  Body X (前方) ← flow_[dx/dy] × [+1/-1]
  Body Y (右方) ← flow_[dx/dy] × [+1/-1]
```

### Step 3: スケール算出

**計算式:**

```
scale = 実移動距離 / (Σ|flow| × 平均height)
```

**計算例:**

```
例: 20cm移動、Σflow=0.8rad、height=0.35m の場合
    scale = 0.20 / (0.8 × 0.35) = 0.71

例: 20cm移動、Σflow=1.2rad、height=0.40m の場合
    scale = 0.20 / (1.2 × 0.40) = 0.42
```

**分析コード:**

```python
import numpy as np

# 移動区間を特定（手動または自動）
start_idx = ...
end_idx = ...

# フロー積分
flow_sum = np.sum(np.abs(flow_data[start_idx:end_idx]))

# 平均高度
avg_height = np.mean(height_data[start_idx:end_idx])

# スケール算出
actual_distance = 0.20  # 20cm
scale = actual_distance / (flow_sum * avg_height)

print(f"Flow sum: {flow_sum:.4f} rad")
print(f"Avg height: {avg_height:.4f} m")
print(f"Calculated scale: {scale:.4f}")
```

### Step 4: ジャイロ補償係数の確認

**静止状態でのYaw回転テスト:**

1. 機体を同じ位置で保持
2. Yaw軸周りにゆっくり回転（±30°程度）
3. flow出力とgyro出力を記録

**回帰分析:**

```python
from sklearn.linear_model import LinearRegression

# 回帰モデル
# flow_dx = k_xx * gyro_x + k_xy * gyro_y + offset_x
# flow_dy = k_yx * gyro_x + k_yy * gyro_y + offset_y

X = np.column_stack([gyro_x, gyro_y])

model_x = LinearRegression().fit(X, flow_dx)
model_y = LinearRegression().fit(X, flow_dy)

print(f"flow_dx = {model_x.coef_[0]:.2f}*gyro_x + {model_x.coef_[1]:.2f}*gyro_y + {model_x.intercept_:.4f}")
print(f"flow_dy = {model_y.coef_[0]:.2f}*gyro_x + {model_y.coef_[1]:.2f}*gyro_y + {model_y.intercept_:.4f}")
```

---

## 推奨実験順序

1. **実験1-1, 1-2** を各3回実施（再現性確認）
2. 軸マッピングとスケールを暫定決定
3. **実験2-1** でパラメータ検証
4. 必要に応じて **実験3** で床面依存性確認

---

## キャリブレーション結果 (2025-12-02)

### 実験データ

| テスト | ログファイル | 移動方向 | 実測距離 | フロー合計 | 高度 |
|--------|-------------|----------|----------|-----------|------|
| X前進 | flow_test_x_forward_01.bin | +X (前方) | 21.9cm | flow_dx: +322 | 29.7cm |
| Y右移動 | flow_test_y_right_01.bin | +Y (右方) | 18.7cm | flow_dy: +241 | 33.3cm |
| 正方形 | flow_test_square_01.bin | 時計回り20cm四方 | X:21.5cm, Y:22.6cm | - | 33.3cm |

### 1. 軸マッピング表 (確定)

```
Body X (前方) ← flow_dx × (+1)  ✓
Body Y (右方) ← flow_dy × (+1)  ✓
```

**結論:** 軸マッピングと符号は正しい（変更不要）

### 2. スケール値 (確定)

```
flow_scale = 0.23  (旧: 0.16)
```

**計算:**
- X軸テスト: scale = 0.219 / (322 × 0.297) = 0.229
- Y軸テスト: scale = 0.187 / (241 × 0.333) = 0.233
- 平均: 0.23

### 3. ジャイロ補償係数 (既存値を継続使用)

```
k_xx = 1.35 × flow_scale  (gyro_x → flow_dx)
k_xy = 9.30 × flow_scale  (gyro_y → flow_dx)
k_yx = -2.65 × flow_scale (gyro_x → flow_dy)
k_yy = 0.0 × flow_scale   (gyro_y → flow_dy)
```

### 4. ESKFパラメータ更新 (完了)

以下のファイルを更新済み:
- `components/stampfly_eskf/eskf.cpp` - flow_scale = 0.23f
- `tools/eskf_debug/eskf_pc.cpp` - flow_scale = 0.23f

### 5. 検証結果

正方形軌道テスト (20cm四方、時計回り):
- X範囲: 21.5cm (期待: 20cm) ✓
- Y範囲: 22.6cm (期待: 20cm) ✓
- 終了位置誤差: X=0.8cm, Y=3.0cm ✓

![正方形軌道テスト結果](../flow_test_square_01_result.png)

---

## 期待される成果物 (テンプレート)

### 1. 軸マッピング表

```
Body X (前方) ← flow_?? × (±1)
Body Y (右方) ← flow_?? × (±1)
```

### 2. スケール値

```
flow_scale_x = ??? [m/rad/m] または無次元
flow_scale_y = ??? [m/rad/m] または無次元
```

### 3. ジャイロ補償係数

```
k_xx = ???  (gyro_x → flow_dx)
k_xy = ???  (gyro_y → flow_dx)
k_yx = ???  (gyro_x → flow_dy)
k_yy = ???  (gyro_y → flow_dy)
```

### 4. ESKFパラメータ更新

検証結果を以下のファイルに反映:
- `components/stampfly_eskf/eskf.cpp` - updateFlowWithGyro()
- `tools/eskf_debug/eskf_pc.cpp` - 同上（PC版）

---

## トラブルシューティング

### フロー出力がゼロまたは非常に小さい

- 床面のテクスチャが不足している可能性
- 高度が高すぎる（>1m）
- センサーの汚れ・故障

### スケールが不安定

- 移動速度が速すぎる
- 高度変動が大きい
- 照明条件の変化

### 軸が入れ替わっている

- センサーの取り付け向きを確認
- ソフトウェアで軸入れ替えを実装

### ジャイロ補償が不十分

- 補償係数の再キャリブレーション
- 高次の項（非線形性）の考慮

---

## 関連ファイル

- `tools/eskf_debug/replay.cpp` - PCリプレイツール
- `tools/eskf_debug/eskf_pc.cpp` - PC版ESKF実装
- `tools/eskf_debug/visualize_eskf.py` - 可視化スクリプト
- `components/stampfly_eskf/eskf.cpp` - デバイス版ESKF実装

---

## 更新履歴

| 日付 | 内容 |
|------|------|
| 2025-12-02 | 初版作成 |
| 2025-12-02 | キャリブレーション実施、flow_scale=0.23に更新 |
