# StampFly Simulator

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

StampFly用のPythonベースフライトシミュレータ。物理エンジン、センサモデル、制御システム、HILテスト機能を備えた教育・研究プラットフォーム。

## 1. 概要

### このシミュレータについて

StampFly Simulatorは、マイクロドローン「StampFly」の動力学をPythonでシミュレートするツールです。実機ファームウェア（`firmware/vehicle/`）と互換性のある制御アルゴリズムを実装しており、以下の用途に使用できます：

- **制御アルゴリズムの開発・検証**: 安全な環境でPIDゲインの調整やフライトモードの実装
- **センサフュージョンの研究**: リアルなノイズモデルを持つセンサシミュレーション
- **教育**: ドローン動力学と制御理論の可視化学習
- **HILテスト**: 実機ファームウェアとシミュレータの連携テスト

### 機体構成

```
               Front
          FL (M4)   FR (M1)
             ╲   ▲   ╱
              ╲  │  ╱       CW: M2, M4
               ╲ │ ╱       CCW: M1, M3
                ╲│╱
                 ╳         ← Center of Mass
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲
             ╱   │   ╲
          RL (M3)    RR (M2)
                Rear
```

### 機体パラメータ

| パラメータ | 値 | 単位 |
|-----------|-----|------|
| 質量 | 35 | g |
| 慣性モーメント Ixx | 9.16e-6 | kg·m² |
| 慣性モーメント Iyy | 13.3e-6 | kg·m² |
| 慣性モーメント Izz | 20.4e-6 | kg·m² |
| バッテリー電圧 | 3.7 | V |

## 2. セットアップ

### 必要環境

- Python 3.10以上
- pip（パッケージマネージャ）

### インストール

```bash
# リポジトリのルートディレクトリで実行
cd stampfly_ecosystem/simulator

# 依存パッケージのインストール
pip install -r requirements.txt
```

### 依存パッケージ

| パッケージ | 用途 | 必須 |
|-----------|------|------|
| numpy | 数値計算 | ○ |
| matplotlib | グラフ描画 | ○ |
| vpython | 3D可視化 | ○ |
| numpy-stl | STLモデル読み込み | ○ |
| Pillow | 画像処理 | ○ |
| opencv-python | 画像処理（オプティカルフロー） | ○ |
| hid | ジョイスティック入力 | △ |
| pyserial | HIL通信 | △ |

**注意**: `hid`パッケージはシステムレベルのHIDライブラリが必要です。

```bash
# macOS
brew install hidapi

# Ubuntu/Debian
sudo apt-get install libhidapi-hidraw0
```

## 3. ディレクトリ構成

```
simulator/
├── README.md              # 本ドキュメント
├── MIGRATION_PLAN.md      # 移植計画（開発者向け）
├── requirements.txt       # 依存パッケージ
├── __init__.py
│
├── core/                  # 物理エンジン
│   ├── physics.py         # 剛体力学（RigidBody）
│   ├── dynamics.py        # マルチコプタ動力学
│   ├── aerodynamics.py    # 空気力学
│   ├── motors.py          # モーター・プロペラモデル
│   └── battery.py         # バッテリーモデル
│
├── sensors/               # センサモデル
│   ├── imu.py             # BMI270 6軸IMU
│   ├── magnetometer.py    # BMM150 地磁気センサ
│   ├── barometer.py       # BMP280 気圧センサ
│   ├── tof.py             # VL53L3CX ToFセンサ
│   ├── opticalflow.py     # PMW3901 オプティカルフロー
│   ├── power_monitor.py   # INA3221 電源モニタ
│   └── noise_models.py    # Allan分散ベースノイズ
│
├── control/               # 制御システム
│   ├── pid.py             # 不完全微分フィルタ付きPID
│   ├── rate_controller.py # レート（角速度）制御
│   ├── attitude_controller.py  # 姿勢・高度制御
│   └── motor_mixer.py     # X-quadモーターミキサー
│
├── interfaces/            # 外部インターフェース
│   ├── joystick.py        # HIDジョイスティック
│   ├── messages.py        # プロトコルメッセージ
│   ├── protocol_bridge.py # シミュレータ状態↔プロトコル変換
│   ├── sil_interface.py   # SILインターフェース
│   └── hil_interface.py   # HILインターフェース
│
├── visualization/         # 可視化
│   └── vpython_backend.py # VPython 3Dレンダラ
│
├── assets/                # リソース
│   ├── meshes/            # 3Dモデル（STL）
│   ├── textures/          # テクスチャ画像
│   └── loaders/           # ファイルローダ
│
├── scripts/               # 実行スクリプト
│   ├── run_sim.py         # メインシミュレータ
│   ├── sandbox.py         # 開発用テスト
│   ├── test_sensors.py    # センサモデルテスト
│   ├── test_protocol.py   # プロトコルテスト
│   ├── test_control.py    # 制御システムテスト
│   └── visualize_sensors.py  # センサ特性可視化
│
└── output/                # 出力ファイル（.gitignore済み）
```

## 4. 基本的な使い方

### シミュレータの起動

```bash
cd stampfly_ecosystem/simulator/scripts

# ジョイスティック操作でフライト
python run_sim.py
```

### ジョイスティック操作

ATOM Joystickを使用：

| 入力 | 機能 |
|------|------|
| 左スティック Y軸 | スロットル |
| 右スティック X軸 | ロール |
| 右スティック Y軸 | ピッチ |
| 左スティック X軸 | ヨー |

### コード例：基本的な物理シミュレーション

```python
import numpy as np
from simulator.core import multicopter

# 機体の作成
mass = 0.035  # 35g
inertia = [
    [9.16e-6, 0.0, 0.0],
    [0.0, 13.3e-6, 0.0],
    [0.0, 0.0, 20.4e-6]
]
drone = multicopter(mass=mass, inersia=inertia)

# 初期状態の設定
drone.set_pqr([[0.0], [0.0], [0.0]])  # 角速度 [rad/s]
drone.set_uvw([[0.0], [0.0], [0.0]])  # 速度 [m/s]

# ホバリング電圧の計算
weight = mass * 9.81
hover_voltage = drone.motor_prop[0].equilibrium_voltage(weight / 4)

# シミュレーションループ
dt = 0.001  # 1ms
for _ in range(1000):
    voltage = [hover_voltage] * 4
    drone.step(voltage, dt)

    # 状態の取得
    position = drone.body.position
    euler = drone.body.euler
    print(f"Position: {position.T}, Euler: {euler.T}")
```

### コード例：センサシミュレーション

```python
import numpy as np
from simulator.sensors import IMU, Barometer, Magnetometer

# センサの作成
imu = IMU(sample_rate=400)  # 400Hz
baro = Barometer(sample_rate=50)  # 50Hz
mag = Magnetometer(sample_rate=100)  # 100Hz

# 真値からセンサ出力を生成
true_accel = np.array([0.0, 0.0, -9.81])  # m/s²
true_gyro = np.array([0.0, 0.0, 0.0])     # rad/s
true_altitude = 1.5  # m

# IMU出力（ノイズ付き）
gyro_meas, accel_meas = imu.update(true_gyro, true_accel, dt=0.0025)

# 気圧計出力
pressure, temperature = baro.update(true_altitude, dt=0.02)

# 地磁気センサ出力
mag_field = mag.update(attitude=np.array([0, 0, 0]), dt=0.01)
```

### コード例：制御システム

```python
import numpy as np
from simulator.control import RateController, MotorMixer

# 制御器の作成（ファームウェア互換ゲイン）
rate_ctrl = RateController()
mixer = MotorMixer()

# スティック入力からモーター出力を計算
gyro = np.array([0.1, -0.05, 0.02])  # 現在の角速度
throttle = 0.5

roll_out, pitch_out, yaw_out = rate_ctrl.update_from_stick(
    stick_roll=0.3,
    stick_pitch=-0.2,
    stick_yaw=0.1,
    gyro=gyro,
    dt=0.0025
)

motors = mixer.mix(throttle, roll_out, pitch_out, yaw_out)
print(f"Motor outputs: {motors}")  # [0.0, 1.0] の範囲
```

## 5. 制御モード

### Rate（Acro）モード

角速度を直接制御するモード。スティック入力が目標角速度にマッピングされます。

```python
from simulator.control import RateController

ctrl = RateController()

# スティック入力 [-1, 1] → 目標レート
# max_rate: Roll=220°/s, Pitch=220°/s, Yaw=200°/s
roll_out, pitch_out, yaw_out = ctrl.update_from_stick(
    stick_roll=0.5,    # 110°/s 目標
    stick_pitch=0.0,
    stick_yaw=0.0,
    gyro=current_gyro,
    dt=0.0025
)
```

### Angle（Stabilize）モード

姿勢角を制御するモード。セルフレベリング機能付き。

```python
from simulator.control import AttitudeController

ctrl = AttitudeController()

# スティック入力 [-1, 1] → 目標角度
# max_angle: Roll=30°, Pitch=30°
roll_out, pitch_out, yaw_out = ctrl.update(
    stick_roll=0.5,    # 15° 目標
    stick_pitch=0.0,
    stick_yaw=0.0,
    attitude=current_attitude,  # [roll, pitch, yaw]
    gyro=current_gyro,
    dt=0.0025
)
```

### Altitude Holdモード

高度を維持するモード。

```python
from simulator.control import AltitudeController

ctrl = AltitudeController(hover_throttle=0.5)

# 目標高度の設定
ctrl.set_altitude(1.5)  # 1.5m

# 更新
throttle = ctrl.update(
    current_altitude=1.2,
    vertical_velocity=0.1,
    stick_throttle=0.5,
    dt=0.02
)
```

## 6. SIL/HILテスト

### SIL（Software-in-the-Loop）

Pythonで実装した制御器をシミュレータ内で実行：

```python
from simulator.interfaces import SILInterface, SensorData

sil = SILInterface()

# 制御器の更新
sensor_data = SensorData(
    gyro=np.array([0.0, 0.0, 0.0]),
    accel=np.array([0.0, 0.0, -9.81]),
    mag=np.array([20.0, 0.0, 45.0]),
    pressure=101325.0,
    altitude=0.0,
)

commands = sil.update(sensor_data, dt=0.0025)
motor_outputs = commands.motors
```

### HIL（Hardware-in-the-Loop）

実機ファームウェアとシリアル通信でセンサデータを注入：

```python
from simulator.interfaces import HILInterface, HILSimulationRunner

# HILインターフェースの作成
hil = HILInterface()
hil.connect(port="/dev/tty.usbserial-XXX", baudrate=921600)

# シミュレータからセンサデータを注入
hil.inject_imu(
    timestamp_us=1000,
    gyro=np.array([0.0, 0.0, 0.0]),
    accel=np.array([0.0, 0.0, -9.81])
)

# ファームウェアからモーター出力を取得
motor_output = hil.get_motor_output(timeout=0.01)
if motor_output:
    print(f"Motors: {motor_output.motors}")
```

## 7. プロトコル

シミュレータは `protocol/spec/messages.yaml` で定義されたメッセージ形式をサポート：

| メッセージ | サイズ | 方向 | 用途 |
|-----------|--------|------|------|
| ControlPacket | 14 bytes | Controller→Vehicle | スティック入力 |
| TelemetryPacket | 22 bytes | Vehicle→Controller | ESP-NOWテレメトリ |
| TelemetryWSPacket | 108 bytes | Vehicle→GCS | WebSocketテレメトリ |

### バイナリログ

```python
from simulator.interfaces import BinaryLogger, BinaryLogReader

# ログの書き込み
logger = BinaryLogger("flight_log.bin")
logger.write_control(control_packet)
logger.write_telemetry(telemetry_packet)
logger.close()

# ログの読み込み
reader = BinaryLogReader("flight_log.bin")
for packet_type, packet in reader:
    print(f"{packet_type}: {packet}")
```

## 8. テストの実行

```bash
cd stampfly_ecosystem/simulator/scripts

# センサモデルのテスト
python test_sensors.py

# プロトコルのテスト
python test_protocol.py

# 制御システムのテスト
python test_control.py

# センサ特性の可視化
python visualize_sensors.py
```

## 9. トラブルシューティング

### VPythonが起動しない

```bash
# Jupyter環境では以下を試す
jupyter nbextension enable --py vpython

# または、ブラウザで直接開く
python -c "from vpython import *; sphere()"
```

### ジョイスティックが認識されない

1. HIDAPIライブラリがインストールされているか確認
2. ジョイスティックがUSB接続されているか確認
3. 他のアプリケーションがデバイスを占有していないか確認

```python
import hid
for device in hid.enumerate():
    print(f"VID: {device['vendor_id']:04x}, PID: {device['product_id']:04x}")
```

### HIL通信エラー

1. シリアルポートが正しいか確認
2. ボーレート（921600）が一致しているか確認
3. ファームウェアがHILモードで起動しているか確認

## 10. 参考資料

| リソース | 説明 |
|----------|------|
| [MIGRATION_PLAN.md](MIGRATION_PLAN.md) | 移植計画（開発者向け） |
| [stampfly_sim](https://github.com/kouhei1970/stampfly_sim) | オリジナルシミュレータ |
| `firmware/vehicle/` | 実機ファームウェア |
| `protocol/spec/` | プロトコル定義 |

---

<a id="english"></a>

## 1. Overview

### About This Simulator

StampFly Simulator is a Python-based tool for simulating the dynamics of the "StampFly" micro drone. It implements control algorithms compatible with the actual firmware (`firmware/vehicle/`) and can be used for:

- **Control Algorithm Development**: Safely tune PID gains and implement flight modes
- **Sensor Fusion Research**: Sensor simulation with realistic noise models
- **Education**: Visual learning of drone dynamics and control theory
- **HIL Testing**: Integration testing between firmware and simulator

### Vehicle Configuration

```
               Front
          FL (M4)   FR (M1)
             ╲   ▲   ╱
              ╲  │  ╱       CW: M2, M4
               ╲ │ ╱       CCW: M1, M3
                ╲│╱
                 ╳         ← Center of Mass
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲
             ╱   │   ╲
          RL (M3)    RR (M2)
                Rear
```

### Vehicle Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Mass | 35 | g |
| Inertia Ixx | 9.16e-6 | kg·m² |
| Inertia Iyy | 13.3e-6 | kg·m² |
| Inertia Izz | 20.4e-6 | kg·m² |
| Battery Voltage | 3.7 | V |

## 2. Setup

### Requirements

- Python 3.10 or later
- pip (package manager)

### Installation

```bash
# Run from the repository root
cd stampfly_ecosystem/simulator

# Install dependencies
pip install -r requirements.txt
```

### Dependencies

| Package | Purpose | Required |
|---------|---------|----------|
| numpy | Numerical computation | ○ |
| matplotlib | Plotting | ○ |
| vpython | 3D visualization | ○ |
| numpy-stl | STL model loading | ○ |
| Pillow | Image processing | ○ |
| opencv-python | Image processing (optical flow) | ○ |
| hid | Joystick input | △ |
| pyserial | HIL communication | △ |

**Note**: The `hid` package requires system-level HID libraries.

```bash
# macOS
brew install hidapi

# Ubuntu/Debian
sudo apt-get install libhidapi-hidraw0
```

## 3. Directory Structure

```
simulator/
├── README.md              # This document
├── MIGRATION_PLAN.md      # Migration plan (for developers)
├── requirements.txt       # Dependencies
├── __init__.py
│
├── core/                  # Physics engine
│   ├── physics.py         # Rigid body dynamics
│   ├── dynamics.py        # Multicopter dynamics
│   ├── aerodynamics.py    # Aerodynamics
│   ├── motors.py          # Motor/propeller model
│   └── battery.py         # Battery model
│
├── sensors/               # Sensor models
│   ├── imu.py             # BMI270 6-axis IMU
│   ├── magnetometer.py    # BMM150 magnetometer
│   ├── barometer.py       # BMP280 barometer
│   ├── tof.py             # VL53L3CX ToF sensor
│   ├── opticalflow.py     # PMW3901 optical flow
│   ├── power_monitor.py   # INA3221 power monitor
│   └── noise_models.py    # Allan variance-based noise
│
├── control/               # Control system
│   ├── pid.py             # PID with incomplete derivative filter
│   ├── rate_controller.py # Rate (angular velocity) control
│   ├── attitude_controller.py  # Attitude/altitude control
│   └── motor_mixer.py     # X-quad motor mixer
│
├── interfaces/            # External interfaces
│   ├── joystick.py        # HID joystick
│   ├── messages.py        # Protocol messages
│   ├── protocol_bridge.py # Simulator state ↔ protocol conversion
│   ├── sil_interface.py   # SIL interface
│   └── hil_interface.py   # HIL interface
│
├── visualization/         # Visualization
│   └── vpython_backend.py # VPython 3D renderer
│
├── assets/                # Resources
│   ├── meshes/            # 3D models (STL)
│   ├── textures/          # Texture images
│   └── loaders/           # File loaders
│
├── scripts/               # Execution scripts
│   ├── run_sim.py         # Main simulator
│   ├── sandbox.py         # Development testing
│   ├── test_sensors.py    # Sensor model tests
│   ├── test_protocol.py   # Protocol tests
│   ├── test_control.py    # Control system tests
│   └── visualize_sensors.py  # Sensor visualization
│
└── output/                # Output files (.gitignore'd)
```

## 4. Basic Usage

### Running the Simulator

```bash
cd stampfly_ecosystem/simulator/scripts

# Flight with joystick control
python run_sim.py
```

### Joystick Controls

Using ATOM Joystick:

| Input | Function |
|-------|----------|
| Left Stick Y | Throttle |
| Right Stick X | Roll |
| Right Stick Y | Pitch |
| Left Stick X | Yaw |

### Code Example: Basic Physics Simulation

```python
import numpy as np
from simulator.core import multicopter

# Create vehicle
mass = 0.035  # 35g
inertia = [
    [9.16e-6, 0.0, 0.0],
    [0.0, 13.3e-6, 0.0],
    [0.0, 0.0, 20.4e-6]
]
drone = multicopter(mass=mass, inersia=inertia)

# Set initial state
drone.set_pqr([[0.0], [0.0], [0.0]])  # Angular velocity [rad/s]
drone.set_uvw([[0.0], [0.0], [0.0]])  # Velocity [m/s]

# Calculate hover voltage
weight = mass * 9.81
hover_voltage = drone.motor_prop[0].equilibrium_voltage(weight / 4)

# Simulation loop
dt = 0.001  # 1ms
for _ in range(1000):
    voltage = [hover_voltage] * 4
    drone.step(voltage, dt)

    # Get state
    position = drone.body.position
    euler = drone.body.euler
    print(f"Position: {position.T}, Euler: {euler.T}")
```

### Code Example: Sensor Simulation

```python
import numpy as np
from simulator.sensors import IMU, Barometer, Magnetometer

# Create sensors
imu = IMU(sample_rate=400)  # 400Hz
baro = Barometer(sample_rate=50)  # 50Hz
mag = Magnetometer(sample_rate=100)  # 100Hz

# Generate sensor output from true values
true_accel = np.array([0.0, 0.0, -9.81])  # m/s²
true_gyro = np.array([0.0, 0.0, 0.0])     # rad/s
true_altitude = 1.5  # m

# IMU output (with noise)
gyro_meas, accel_meas = imu.update(true_gyro, true_accel, dt=0.0025)

# Barometer output
pressure, temperature = baro.update(true_altitude, dt=0.02)

# Magnetometer output
mag_field = mag.update(attitude=np.array([0, 0, 0]), dt=0.01)
```

### Code Example: Control System

```python
import numpy as np
from simulator.control import RateController, MotorMixer

# Create controllers (firmware-compatible gains)
rate_ctrl = RateController()
mixer = MotorMixer()

# Calculate motor output from stick input
gyro = np.array([0.1, -0.05, 0.02])  # Current angular velocity
throttle = 0.5

roll_out, pitch_out, yaw_out = rate_ctrl.update_from_stick(
    stick_roll=0.3,
    stick_pitch=-0.2,
    stick_yaw=0.1,
    gyro=gyro,
    dt=0.0025
)

motors = mixer.mix(throttle, roll_out, pitch_out, yaw_out)
print(f"Motor outputs: {motors}")  # Range [0.0, 1.0]
```

## 5. Control Modes

### Rate (Acro) Mode

Direct angular velocity control. Stick input maps to target angular rates.

```python
from simulator.control import RateController

ctrl = RateController()

# Stick input [-1, 1] → Target rate
# max_rate: Roll=220°/s, Pitch=220°/s, Yaw=200°/s
roll_out, pitch_out, yaw_out = ctrl.update_from_stick(
    stick_roll=0.5,    # 110°/s target
    stick_pitch=0.0,
    stick_yaw=0.0,
    gyro=current_gyro,
    dt=0.0025
)
```

### Angle (Stabilize) Mode

Attitude angle control with self-leveling.

```python
from simulator.control import AttitudeController

ctrl = AttitudeController()

# Stick input [-1, 1] → Target angle
# max_angle: Roll=30°, Pitch=30°
roll_out, pitch_out, yaw_out = ctrl.update(
    stick_roll=0.5,    # 15° target
    stick_pitch=0.0,
    stick_yaw=0.0,
    attitude=current_attitude,  # [roll, pitch, yaw]
    gyro=current_gyro,
    dt=0.0025
)
```

### Altitude Hold Mode

Maintains altitude.

```python
from simulator.control import AltitudeController

ctrl = AltitudeController(hover_throttle=0.5)

# Set target altitude
ctrl.set_altitude(1.5)  # 1.5m

# Update
throttle = ctrl.update(
    current_altitude=1.2,
    vertical_velocity=0.1,
    stick_throttle=0.5,
    dt=0.02
)
```

## 6. SIL/HIL Testing

### SIL (Software-in-the-Loop)

Run Python-implemented controllers within the simulator:

```python
from simulator.interfaces import SILInterface, SensorData

sil = SILInterface()

# Update controller
sensor_data = SensorData(
    gyro=np.array([0.0, 0.0, 0.0]),
    accel=np.array([0.0, 0.0, -9.81]),
    mag=np.array([20.0, 0.0, 45.0]),
    pressure=101325.0,
    altitude=0.0,
)

commands = sil.update(sensor_data, dt=0.0025)
motor_outputs = commands.motors
```

### HIL (Hardware-in-the-Loop)

Inject sensor data to real firmware via serial communication:

```python
from simulator.interfaces import HILInterface, HILSimulationRunner

# Create HIL interface
hil = HILInterface()
hil.connect(port="/dev/tty.usbserial-XXX", baudrate=921600)

# Inject sensor data from simulator
hil.inject_imu(
    timestamp_us=1000,
    gyro=np.array([0.0, 0.0, 0.0]),
    accel=np.array([0.0, 0.0, -9.81])
)

# Get motor output from firmware
motor_output = hil.get_motor_output(timeout=0.01)
if motor_output:
    print(f"Motors: {motor_output.motors}")
```

## 7. Protocol

The simulator supports message formats defined in `protocol/spec/messages.yaml`:

| Message | Size | Direction | Purpose |
|---------|------|-----------|---------|
| ControlPacket | 14 bytes | Controller→Vehicle | Stick input |
| TelemetryPacket | 22 bytes | Vehicle→Controller | ESP-NOW telemetry |
| TelemetryWSPacket | 108 bytes | Vehicle→GCS | WebSocket telemetry |

### Binary Logging

```python
from simulator.interfaces import BinaryLogger, BinaryLogReader

# Write log
logger = BinaryLogger("flight_log.bin")
logger.write_control(control_packet)
logger.write_telemetry(telemetry_packet)
logger.close()

# Read log
reader = BinaryLogReader("flight_log.bin")
for packet_type, packet in reader:
    print(f"{packet_type}: {packet}")
```

## 8. Running Tests

```bash
cd stampfly_ecosystem/simulator/scripts

# Sensor model tests
python test_sensors.py

# Protocol tests
python test_protocol.py

# Control system tests
python test_control.py

# Sensor visualization
python visualize_sensors.py
```

## 9. Troubleshooting

### VPython Won't Start

```bash
# In Jupyter environment, try:
jupyter nbextension enable --py vpython

# Or open directly in browser:
python -c "from vpython import *; sphere()"
```

### Joystick Not Recognized

1. Check if HIDAPI library is installed
2. Check if joystick is connected via USB
3. Check if another application is using the device

```python
import hid
for device in hid.enumerate():
    print(f"VID: {device['vendor_id']:04x}, PID: {device['product_id']:04x}")
```

### HIL Communication Errors

1. Verify the serial port is correct
2. Verify baud rate (921600) matches
3. Verify firmware is running in HIL mode

## 10. References

| Resource | Description |
|----------|-------------|
| [MIGRATION_PLAN.md](MIGRATION_PLAN.md) | Migration plan (for developers) |
| [stampfly_sim](https://github.com/kouhei1970/stampfly_sim) | Original simulator |
| `firmware/vehicle/` | Actual firmware |
| `protocol/spec/` | Protocol definitions |
