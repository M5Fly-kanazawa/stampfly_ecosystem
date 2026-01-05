# notebooks

探索的解析用 Jupyter Notebooks。

## 計画中のノートブック

### 飛行性能
- `flight_trajectory.ipynb` - 飛行軌跡の可視化と精度評価
- `position_error_analysis.ipynb` - 位置誤差の統計分析
- `attitude_accuracy.ipynb` - 姿勢推定精度評価

### センサ解析
- `imu_allan_variance.ipynb` - IMU Allan分散解析
- `magnetometer_calibration.ipynb` - 地磁気キャリブレーション評価
- `barometer_drift.ipynb` - 気圧計長期ドリフト解析
- `opticalflow_accuracy.ipynb` - OpticalFlow精度評価

### ESKF評価
- `eskf_convergence.ipynb` - フィルタ収束特性
- `eskf_parameter_sensitivity.ipynb` - パラメータ感度分析

### 制御系
- `pid_step_response.ipynb` - PIDステップ応答解析
- `stability_analysis.ipynb` - 安定性解析

## 使用方法

```bash
cd analysis/notebooks
jupyter notebook
```

## 依存関係

```bash
pip install jupyter numpy pandas matplotlib scipy
```

---

# notebooks

Exploratory analysis Jupyter Notebooks.

## Planned Notebooks

### Flight Performance
- `flight_trajectory.ipynb` - Flight trajectory visualization and accuracy
- `position_error_analysis.ipynb` - Position error statistics
- `attitude_accuracy.ipynb` - Attitude estimation accuracy

### Sensor Analysis
- `imu_allan_variance.ipynb` - IMU Allan variance analysis
- `magnetometer_calibration.ipynb` - Magnetometer calibration evaluation
- `barometer_drift.ipynb` - Barometer long-term drift
- `opticalflow_accuracy.ipynb` - OpticalFlow accuracy evaluation

### ESKF Evaluation
- `eskf_convergence.ipynb` - Filter convergence characteristics
- `eskf_parameter_sensitivity.ipynb` - Parameter sensitivity analysis

### Control System
- `pid_step_response.ipynb` - PID step response analysis
- `stability_analysis.ipynb` - Stability analysis
