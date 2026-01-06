# Sensor simulation modules
# センサシミュレーションモジュール
"""
Sensor models for StampFly simulator.
StampFlyシミュレータ用センサモデル

Matches firmware sensor components (sf_hal_*):
ファームウェアのセンサコンポーネント（sf_hal_*）と対応：

- BMI270: 6-axis IMU (accelerometer + gyroscope) / 6軸IMU
- BMP280: Barometric pressure sensor / 気圧センサ
- BMM150: 3-axis magnetometer / 3軸地磁気センサ
- PMW3901: Optical flow sensor / オプティカルフローセンサ
- VL53L3CX: Time-of-Flight distance sensor / ToF距離センサ
- INA3221: Power monitor (voltage/current) / 電源モニタ
"""

# Legacy function (backward compatibility)
# レガシー関数（後方互換性）
from .imu import imu

# New sensor classes
# 新しいセンサクラス
from .imu import IMU
from .barometer import Barometer
from .magnetometer import Magnetometer
from .opticalflow import OpticalFlow
from .tof import ToF
from .power_monitor import PowerMonitor

# Noise models
# ノイズモデル
from .noise_models import (
    AllanVarianceParams,
    BMI270_GyroParams,
    BMI270_AccelParams,
    IMUNoiseGenerator,
    compute_allan_variance,
    fit_allan_parameters,
)

__all__ = [
    # Sensor classes
    'IMU',
    'Barometer',
    'Magnetometer',
    'OpticalFlow',
    'ToF',
    'PowerMonitor',
    # Noise models
    'AllanVarianceParams',
    'BMI270_GyroParams',
    'BMI270_AccelParams',
    'IMUNoiseGenerator',
    'compute_allan_variance',
    'fit_allan_parameters',
    # Legacy
    'imu',
]
