# Lesson 10: API Overview & App Development

## Goal / 目標
Understand the full ws:: API, learn about the L1 sf_api layer it sits on, and create a custom firmware project.

ws:: API の全体像を理解し、その土台となる L1 sf_api 層について学び、独自ファームウェアプロジェクトを作成する。

## API / 使用するAPI

### ws:: API (Workshop wrapper)
| Function | Description | Unit |
|----------|-------------|------|
| `ws::gyro_x/y/z()` | Angular velocity | rad/s |
| `ws::accel_x/y/z()` | Linear acceleration | m/s^2 |
| `ws::estimated_roll/pitch/yaw()` | ESKF attitude estimate | rad |
| `ws::estimated_altitude()` | ESKF altitude estimate | m |
| `ws::baro_altitude()/baro_pressure()` | Barometric altitude/pressure | m / Pa |
| `ws::mag_x/y/z()` | Magnetic field | uT |
| `ws::tof_bottom()` | Ground distance (downward ToF) | m |
| `ws::flow_vx/vy()` | Optical flow raw counts (see note below) | count |
| `ws::battery_voltage()` | Battery voltage | V |
| `ws::print(fmt, ...)` | Serial print (Teleplot compatible) | - |

Note: `ws::flow_vx/vy()` return the raw optical-flow counts, not a
converted velocity (unlike the old vehicle_old API). For a physical
velocity estimate, use the ESKF-fused values instead.

注: `ws::flow_vx/vy()` は（旧vehicle_old APIと異なり）変換済みの速度ではなく
光学フローの生カウント値を返す。物理的な速度が必要な場合はESKF推定値を使うこと。

### L1: sf_api layer (what ws:: is built on) / L1: sf_api層（ws::の土台）

`ws::` functions are a thin, Arduino-like wrapper around the vehicle
firmware's own public API layer, `sf::api`, defined in
`firmware/vehicle/components/sf_api`. That layer exposes the same
sensor/estimate/command data (via Pub-Sub topics) to any component in
the vehicle firmware, not just workshop lessons. Reading `sf_api`'s
headers is a good next step once you outgrow the simplified `ws::`
surface — for example when writing a custom project with `sf app new`
(see below) that needs finer control than `ws::` offers.

`ws::` 関数は、vehicle ファームウェア自身の公開APIレイヤーである
`sf::api`（`firmware/vehicle/components/sf_api` に定義）を薄くラップした、
Arduinoライクなインターフェース。そのレイヤーはPub-Subトピック経由で
センサ・推定値・コマンドのデータを、ワークショップのレッスンに限らず
vehicleファームウェアの任意のコンポーネントに公開している。簡易な `ws::`
では足りなくなったら（例: 下記の `sf app new` で `ws::` より細かい制御が
必要なカスタムプロジェクトを書くとき）、`sf_api` のヘッダを読むのが
次のステップになる。

## Hardware Sensors / ハードウェアセンサ

| Sensor | Model | Sample Rate | Measurement |
|--------|-------|-------------|-------------|
| IMU | BMI270 | 400 Hz | Acceleration + Gyroscope |
| Barometer | BMP280 | 50 Hz | Pressure → Altitude |
| Magnetometer | BMM150 | 10-30 Hz | Magnetic vector |
| ToF (bottom) | VL53L3CX | 30 Hz | Ground distance (0-2 m) |
| Optical Flow | PMW3901 | 100 Hz | Ground-relative motion (raw counts via ws::) |

Note: the vehicle sensing pipeline does not read the front ToF sensor,
so `ws::tof_front()` always returns -1.0 and is not used in this lesson.

注: vehicleのセンシングでは前方ToFセンサを読まないため、`ws::tof_front()`は
常に-1.0を返し、本レッスンでは使用しない。

## Custom Firmware Project / カスタムファームウェアプロジェクト

### Create with sf app new / sf app new で作成

```bash
# Create new project / 新しいプロジェクトを作成
sf app new my_sensor_app

# Build and flash / ビルドして書き込み
sf build my_sensor_app
sf flash my_sensor_app -m
```

This creates `firmware/my_sensor_app/` with access to all vehicle components via `EXTRA_COMPONENT_DIRS`.

### Project Structure / プロジェクト構成
```
firmware/my_sensor_app/
├── CMakeLists.txt       # References vehicle/components and common
├── sdkconfig.defaults   # Hardware config (from vehicle)
├── partitions.csv       # Flash partitions (from vehicle)
└── main/
    ├── CMakeLists.txt   # Component registration
    └── main.cpp         # Your custom entry point
```

## Steps / 手順

### Workshop version (sf lesson switch 10) / ワークショップ版
1. `sf lesson switch 10`
2. Uncomment the `ws::` sensor access code in `user_code.cpp`
3. Build and flash: `sf lesson build` → `sf lesson flash`
4. Open Teleplot in VSCode to visualize all sensor data
5. Compare barometric altitude vs ToF vs ESKF altitude
6. Calculate magnetic heading from magnetometer data

### Custom project version / カスタムプロジェクト版
1. `sf app new my_sensor_app`
2. Edit `firmware/my_sensor_app/main/main.cpp`
3. `sf build my_sensor_app` → `sf flash my_sensor_app -m`

## Teleplot Setup / Teleplotセットアップ

1. Install VSCode extension: `alexnesnes.teleplot`
2. Connect via `sf monitor`
3. Open Teleplot panel in VSCode
4. Data in `>name:value` format will be graphed automatically

### Teleplot Channels / チャンネル例
| Channel | Description |
|---------|-------------|
| `>baro_alt` | Barometric altitude [m] |
| `>tof_bottom` | ToF ground distance [m] |
| `>eskf_alt` | ESKF estimated altitude [m] |
| `>mag_x/y/z` | Magnetic field [uT] |
| `>heading` | Magnetic heading [deg] |
| `>flow_vx/vy` | Optical flow raw counts |
| `>voltage` | Battery voltage [V] |

## Key Concepts / キーコンセプト
- ws:: API is a simplified workshop wrapper around the vehicle firmware's L1 sf_api layer
- sf_api (`firmware/vehicle/components/sf_api`) exposes the same sensor/estimation data to any vehicle component
- `sf app new` creates a custom firmware project that reuses vehicle components
- Teleplot enables real-time visualization of any sensor data
- Multiple altitude sources (baro, ToF, ESKF) can be compared for understanding sensor fusion
