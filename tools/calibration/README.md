# calibration

センサキャリブレーション確認・支援ツール。

## ファイル構成

```
calibration/
└── plot_mag_xy.py    # 地磁気キャリブレーション確認
```

## 必要なライブラリ

```bash
pip install numpy matplotlib
```

---

## plot_mag_xy.py - 地磁気キャリブレーション確認

地磁気センサのXYプロットでキャリブレーションを確認します。

### 使い方

```bash
python plot_mag_xy.py sensor.bin
```

### 判定基準

- **正常**: 点が原点 (0, 0) を中心とした円状に分布
- **要調整**: 円の中心がオフセットしている、または楕円形

### 出力例

```
Magnetometer XY Plot
  Center offset: (2.3, -1.5) μT
  Radius: 45.2 μT
  Circularity: 0.95
```

---

## 将来の追加予定

- `calibrate_mag.py` - 地磁気オフセット・スケール自動算出
- `calibrate_accel.py` - 加速度計6点キャリブレーション
- `calibrate_gyro.py` - ジャイロバイアス自動算出

---

# calibration

Sensor calibration verification and support tools.

## Files

```
calibration/
└── plot_mag_xy.py    # Magnetometer calibration check
```

## Usage

```bash
python plot_mag_xy.py sensor.bin
```

## Validation

- **Good**: Points form a circle centered at origin (0, 0)
- **Needs adjustment**: Circle center is offset or elliptical
