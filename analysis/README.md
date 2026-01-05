# analysis

実験結果の事後評価・研究・教育向け解析。

> **注意**: 開発中のパラメータチューニングは `tools/log_analyzer/` を使用。
> ここは実験後の結果評価・論文/レポート用。

## ディレクトリ構成

- `notebooks/` - 授業・検討用の探索的解析（Jupyter notebooks）
- `scripts/` - 再現性重視の解析処理、指標算出の自動化
- `datasets/` - 小規模なサンプルログ
- `reports/` - 生成された図・結果（原則 git 管理しない）

## 将来の解析項目（計画）

### 飛行性能評価
- 軌跡精度評価（Ground Truth比較）
- 位置・速度誤差の統計分析
- 姿勢推定精度（Roll/Pitch/Yaw）

### センサ特性解析
- Allan分散によるIMUノイズ特性
- 地磁気キャリブレーション品質評価
- 気圧計ドリフト解析
- ToF/OpticalFlow精度評価

### 制御系評価
- PIDステップ応答解析
- 安定性マージン評価
- 外乱応答特性

### ESKF性能評価
- 状態推定精度 vs Ground Truth
- フィルタ収束特性
- パラメータ感度分析

### 比較研究
- パラメータ変更前後の比較
- アルゴリズム比較（ESKF vs EKF等）

## tools/ との違い

| 項目 | tools/ | analysis/ |
|------|--------|-----------|
| 目的 | 開発支援 | 結果評価 |
| 使用タイミング | 開発中 | 実験後 |
| 出力先 | コード修正 | 図表・レポート |
| 例 | optimize_eskf.py | flight_accuracy.ipynb |

---

# analysis

Post-experiment evaluation for research and education.

> **Note**: Use `tools/log_analyzer/` for parameter tuning during development.
> This directory is for post-experiment evaluation and papers/reports.

## Directory Structure

- `notebooks/` - Exploratory analysis for classes and discussions (Jupyter notebooks)
- `scripts/` - Reproducible analysis processing, automated metric calculation
- `datasets/` - Small sample logs
- `reports/` - Generated figures and results (typically not tracked by git)

## Planned Analysis Items

### Flight Performance
- Trajectory accuracy (vs Ground Truth)
- Position/velocity error statistics
- Attitude estimation accuracy (Roll/Pitch/Yaw)

### Sensor Characterization
- Allan variance for IMU noise
- Magnetometer calibration quality
- Barometer drift analysis
- ToF/OpticalFlow accuracy

### Control System
- PID step response analysis
- Stability margin evaluation
- Disturbance response

### ESKF Performance
- State estimation accuracy vs Ground Truth
- Filter convergence characteristics
- Parameter sensitivity analysis

### Comparative Studies
- Before/after parameter tuning
- Algorithm comparison (ESKF vs EKF, etc.)
