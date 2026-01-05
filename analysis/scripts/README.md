# scripts

再現性重視の解析スクリプト。

## 計画中のスクリプト

### 精度評価
- `evaluate_trajectory.py` - 軌跡精度評価（Ground Truth比較）
- `evaluate_attitude.py` - 姿勢推定精度評価
- `compute_rmse.py` - RMSE/MAE等の指標算出

### センサ解析
- `allan_variance.py` - Allan分散計算
- `noise_characterization.py` - センサノイズ特性解析

### バッチ処理
- `batch_analysis.py` - 複数ログの一括解析
- `generate_report.py` - レポート自動生成

### ユーティリティ
- `load_flight_log.py` - 飛行ログ読み込み共通関数
- `plot_utils.py` - プロット共通関数

## 使用方法

```bash
python scripts/evaluate_trajectory.py --input flight.bin --ground-truth mocap.csv
python scripts/batch_analysis.py --input-dir logs/ --output-dir reports/
```

## notebooks/ との違い

| 項目 | notebooks/ | scripts/ |
|------|------------|----------|
| 用途 | 探索的解析 | 再現性重視 |
| 形式 | Jupyter | Python |
| 出力 | インタラクティブ | 自動化 |

---

# scripts

Reproducible analysis scripts.

## Planned Scripts

### Accuracy Evaluation
- `evaluate_trajectory.py` - Trajectory accuracy (vs Ground Truth)
- `evaluate_attitude.py` - Attitude estimation accuracy
- `compute_rmse.py` - RMSE/MAE metrics computation

### Sensor Analysis
- `allan_variance.py` - Allan variance computation
- `noise_characterization.py` - Sensor noise characterization

### Batch Processing
- `batch_analysis.py` - Batch analysis of multiple logs
- `generate_report.py` - Automated report generation

### Utilities
- `load_flight_log.py` - Common flight log loader
- `plot_utils.py` - Common plotting functions
