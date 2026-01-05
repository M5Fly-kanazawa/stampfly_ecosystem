# datasets

解析用サンプルログ。

## 計画中のデータセット

### 基準データ
- `static_calibration/` - 静止状態キャリブレーションデータ
- `ground_truth/` - モーションキャプチャ等のGround Truth

### 飛行データ
- `hover/` - ホバリングデータ
- `trajectory/` - 軌跡追従データ
- `disturbance/` - 外乱応答データ

### センサデータ
- `imu_noise/` - IMUノイズ特性評価用
- `mag_calibration/` - 地磁気キャリブレーション用

## ファイル形式

- `.bin` - バイナリログ（`tools/log_capture/log_capture.py`で取得）
- `.csv` - 変換済みCSV
- `.json` - メタデータ

## 注意

- 大容量ファイルはgit管理外（.gitignore）
- 必要に応じてGit LFSまたは外部ストレージを使用

---

# datasets

Sample logs for analysis.

## Planned Datasets

### Reference Data
- `static_calibration/` - Static calibration data
- `ground_truth/` - Motion capture Ground Truth

### Flight Data
- `hover/` - Hover data
- `trajectory/` - Trajectory tracking data
- `disturbance/` - Disturbance response data

### Sensor Data
- `imu_noise/` - IMU noise characterization
- `mag_calibration/` - Magnetometer calibration

## File Formats

- `.bin` - Binary log (captured with `tools/log_capture/log_capture.py`)
- `.csv` - Converted CSV
- `.json` - Metadata

## Note

- Large files excluded from git (.gitignore)
- Use Git LFS or external storage as needed
