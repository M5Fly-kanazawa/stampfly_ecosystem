# HIL対応ファームウェア実装計画

## 1. 概要

### 目標
シミュレータ（Python）と実機ファームウェア（ESP32）をシリアル接続し、センサデータ注入とモーター出力読み取りでHILテストを実現する。

### ユーザー選択
- **モード切り替え**: ランタイム（CLIコマンド `hil start/stop`）
- **通信インターフェース**: USB CDC（既存CLIと共用）
- **タイミング**: タイムスタンプ同期（シミュレータ時刻をファームウェアに合わせる）

## 2. プロトコル（simulator/interfaces/hil_interface.py と一致）

### メッセージ構造
```
| Type (1B) | Timestamp_us (4B) | Payload | Checksum (1B) |
```
チェックサム: SUM（全バイトの和を256で割った余り）

### シミュレータ → ファームウェア（センサ注入）

| Type | 名前 | サイズ | Payload |
|------|------|--------|---------|
| 0x10 | IMU_DATA | 28B | gyro[3] + accel[3] (float×6) |
| 0x11 | MAG_DATA | 16B | mag[3] (float×3) |
| 0x12 | BARO_DATA | 14B | pressure + temp (float×2) |
| 0x13 | TOF_DATA | 12B | distance_mm (u16) + valid (u8) |
| 0x14 | FLOW_DATA | 14B | delta_x/y (i16×2) + quality (u8) |
| 0x40 | HIL_ENABLE | 2B | - |
| 0x41 | HIL_DISABLE | 2B | - |

### ファームウェア → シミュレータ（出力）

| Type | 名前 | サイズ | Payload |
|------|------|--------|---------|
| 0x20 | MOTOR_OUTPUT | 22B | motor[4] (float×4) |
| 0x21 | STATE_UPDATE | 10B | flight_state + sensor_status + armed |
| 0x31 | SYNC_RESPONSE | 6B | firmware_timestamp_us (u32) |

## 3. アーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│ Simulator (Python)                                          │
│   HILInterface.inject_imu() ─────────────────┐              │
│   HILInterface.inject_mag() ─────────────────┤ USB Serial   │
│   HILInterface.inject_baro() ────────────────┤ 921600bps    │
│                                              ↓              │
└──────────────────────────────────────────────┼──────────────┘
                                               │
┌──────────────────────────────────────────────┼──────────────┐
│ Firmware (ESP32)                             ↓              │
│                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │ HILTask     │←──→│ HILReceiver │←──→│ USB CDC     │     │
│  │ (新規)      │    │ (sf_svc_hil)│    │ (既存)      │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│        ↓                                                    │
│  ┌─────────────┐                                           │
│  │ State       │ ← HILモード時: センサタスク停止           │
│  │ Manager     │   HILTaskがstate.update*()を直接呼出     │
│  └─────────────┘                                           │
│        ↓                                                    │
│  ┌─────────────┐    ┌─────────────┐                        │
│  │ ControlTask │───→│ Motor       │→ PWM (実機) or         │
│  │ (400Hz)     │    │ Driver      │→ HIL送信 (HILモード)   │
│  └─────────────┘    └─────────────┘                        │
└─────────────────────────────────────────────────────────────┘
```

## 4. 実装ステップ

### Phase 1: HILコンポーネント作成

**1. `sf_svc_hil` コンポーネント新規作成**

```
firmware/vehicle/components/sf_svc_hil/
├── CMakeLists.txt
├── include/
│   ├── hil_types.hpp      # メッセージ構造体（simulator側と一致）
│   └── hil_receiver.hpp   # HIL受信クラス
└── src/
    └── hil_receiver.cpp   # パーサー・バッファ管理
```

**hil_types.hpp の内容:**
```cpp
#pragma pack(push, 1)
struct HILIMUData {
    uint8_t type;           // 0x10
    uint32_t timestamp_us;
    float gyro[3];          // rad/s
    float accel[3];         // m/s²
    uint8_t checksum;
};  // 28 bytes

struct HILMotorOutput {
    uint8_t type;           // 0x20
    uint32_t timestamp_us;
    float motors[4];        // 0-1 normalized
    uint8_t checksum;
};  // 22 bytes
#pragma pack(pop)
```

**2. HILReceiver クラス:**
- USB CDCからのバイナリデータを受信
- リングバッファでパケットを蓄積
- 型別にパース → キューに格納

### Phase 2: HILタスク作成

**3. `hil_task.cpp` 新規作成**

```cpp
void HILTask(void* pvParameters) {
    while (true) {
        if (!g_hil_mode) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // IMUデータ受信 → State更新
        HILIMUData imu_data;
        if (g_hil_receiver.getIMU(&imu_data)) {
            Vec3 accel(imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
            Vec3 gyro(imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);
            state.updateIMU(accel, gyro);
            xSemaphoreGive(g_control_semaphore);  // ControlTaskを起動
        }

        // 他センサも同様...

        vTaskDelay(pdMS_TO_TICKS(1));  // 1ms ポーリング
    }
}
```

### Phase 3: モーター出力送信

**4. control_task.cpp にフック追加**

```cpp
// 既存のsetMixerOutput()の後に追加
g_motor.setMixerOutput(throttle, roll_out, pitch_out, yaw_out);

if (g_hil_mode) {
    HILMotorOutput pkt;
    pkt.type = 0x20;
    pkt.timestamp_us = esp_timer_get_time();
    g_motor.getOutputs(pkt.motors);  // [0]=M1, [1]=M2, [2]=M3, [3]=M4
    pkt.checksum = hil_checksum(&pkt, sizeof(pkt) - 1);
    g_hil_receiver.send(&pkt, sizeof(pkt));
}
```

### Phase 4: CLIコマンド追加

**5. CLI に `hil` コマンド追加**

```cpp
// cli.cpp に追加
void cmd_hil(int argc, char** argv, void* ctx) {
    if (argc < 2) {
        g_cli.print("Usage: hil [start|stop|status]\r\n");
        return;
    }

    if (strcmp(argv[1], "start") == 0) {
        g_hil_mode = true;
        // センサタスクを停止（または無視モードに）
        g_cli.print("HIL mode enabled\r\n");
    } else if (strcmp(argv[1], "stop") == 0) {
        g_hil_mode = false;
        g_cli.print("HIL mode disabled\r\n");
    } else if (strcmp(argv[1], "status") == 0) {
        g_cli.print("HIL mode: %s\r\n", g_hil_mode ? "ON" : "OFF");
    }
}

// registerDefaultCommands() に追加
registerCommand("hil", cmd_hil, "HIL mode control");
```

### Phase 5: グローバル変数・タスク管理

**6. globals.hpp/cpp に追加**

```cpp
// globals.hpp
extern bool g_hil_mode;
extern HILReceiver g_hil_receiver;

// globals.cpp
bool g_hil_mode = false;
HILReceiver g_hil_receiver;
```

**7. main.cpp でHILタスク起動**

```cpp
// タスク生成部分に追加
xTaskCreatePinnedToCore(
    HILTask,
    "HILTask",
    4096,
    nullptr,
    3,  // 中優先度
    nullptr,
    0   // Core 0
);
```

**8. センサタスクのHILモード対応**

各センサタスク（imu_task, mag_task, baro_task等）の先頭に追加:
```cpp
if (g_hil_mode) {
    vTaskDelay(pdMS_TO_TICKS(100));  // HILモード時はスキップ
    continue;
}
```

## 5. ファイル一覧

### 新規作成
| ファイル | 内容 |
|----------|------|
| `components/sf_svc_hil/CMakeLists.txt` | コンポーネント定義 |
| `components/sf_svc_hil/include/hil_types.hpp` | メッセージ構造体 |
| `components/sf_svc_hil/include/hil_receiver.hpp` | 受信クラス定義 |
| `components/sf_svc_hil/src/hil_receiver.cpp` | 受信・解析実装 |
| `main/tasks/hil_task.cpp` | HILタスク |

### 修正
| ファイル | 変更内容 |
|----------|----------|
| `main/globals.hpp` | `g_hil_mode`, `g_hil_receiver` 追加 |
| `main/globals.cpp` | 同上の定義 |
| `main/main.cpp` | HILタスク起動追加 |
| `main/tasks/control_task.cpp` | モーター出力送信フック |
| `main/tasks/imu_task.cpp` | HILモード時スキップ |
| `main/tasks/mag_task.cpp` | 同上 |
| `main/tasks/baro_task.cpp` | 同上 |
| `main/tasks/tof_task.cpp` | 同上 |
| `main/tasks/optflow_task.cpp` | 同上 |
| `components/sf_svc_cli/src/cli.cpp` | `hil` コマンド追加 |

## 6. テスト手順

1. **ビルド・フラッシュ**
   ```bash
   cd firmware/vehicle
   idf.py build flash monitor
   ```

2. **HILモード有効化**
   ```
   > hil start
   HIL mode enabled
   ```

3. **シミュレータ接続**
   ```python
   from simulator.interfaces import HILInterface

   hil = HILInterface()
   hil.connect('/dev/tty.usbmodem*')
   hil.enable_hil()

   # センサ注入
   hil.inject_imu(timestamp, gyro, accel)

   # モーター出力取得
   motor = hil.get_motor_output(timeout=0.01)
   print(motor.motors)
   ```

4. **ループバックテスト**
   - センサ注入 → State更新 → 制御計算 → モーター出力
   - 期待値と比較

## 7. 注意事項

- **タイミング**: USB遅延（1-5ms）があるため、リアルタイム性は制限される
- **初期化**: HILモード時もセンサ安定化チェックはスキップが必要
- **安全**: g_hil_mode中はモーターPWM出力を無効化（オプション）
- **同期**: シミュレータ側でファームウェアのタイムスタンプに追従
