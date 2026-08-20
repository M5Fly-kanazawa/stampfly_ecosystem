# HILS対応ファームウェア実装計画

> **実装状況（2026-08-20 注記）**: 本計画は未実施。ファームウェア側（`sf_svc_hils` 等）は実装されておらず、Python 側のインターフェース（`simulator/vpython/interfaces/hils_interface.py`）のみが存在する。呼び出し元も無いため、現状シミュレータと実機を接続する HILS 機能は動作しない。

## 1. 概要

### 目標
シミュレータ（Python）と実機ファームウェア（ESP32）をシリアル接続し、センサデータ注入とモーター出力読み取りでHILSテストを実現する。

### ユーザー選択
- **モード切り替え**: ランタイム（CLIコマンド `hils start/stop`）
- **通信インターフェース**: USB CDC（既存CLIと共用）
- **タイミング**: タイムスタンプ同期（シミュレータ時刻をファームウェアに合わせる）

## 2. プロトコル（simulator/interfaces/hils_interface.py と一致）

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
| 0x40 | HILS_ENABLE | 2B | - |
| 0x41 | HILS_DISABLE | 2B | - |

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
│   HILSInterface.inject_imu() ─────────────────┐             │
│   HILSInterface.inject_mag() ─────────────────┤ USB Serial  │
│   HILSInterface.inject_baro() ────────────────┤ 921600bps   │
│                                              ↓              │
└──────────────────────────────────────────────┼──────────────┘
                                               │
┌──────────────────────────────────────────────┼──────────────┐
│ Firmware (ESP32)                             ↓              │
│                                                             │
│  ┌─────────────┐    ┌──────────────┐    ┌─────────────┐    │
│  │ HILSTask    │←──→│ HILSReceiver │←──→│ USB CDC     │    │
│  │ (新規)      │    │ (sf_svc_hils)│    │ (既存)      │    │
│  └─────────────┘    └──────────────┘    └─────────────┘    │
│        ↓                                                    │
│  ┌─────────────┐                                           │
│  │ State       │ ← HILSモード時: センサタスク停止          │
│  │ Manager     │   HILSTaskがstate.update*()を直接呼出    │
│  └─────────────┘                                           │
│        ↓                                                    │
│  ┌─────────────┐    ┌─────────────┐                        │
│  │ ControlTask │───→│ Motor       │→ PWM (実機) or         │
│  │ (400Hz)     │    │ Driver      │→ HILS送信 (HILSモード) │
│  └─────────────┘    └─────────────┘                        │
└─────────────────────────────────────────────────────────────┘
```

## 4. 実装ステップ

### Phase 1: HILSコンポーネント作成

**1. `sf_svc_hils` コンポーネント新規作成**

```
firmware/vehicle/components/sf_svc_hils/
├── CMakeLists.txt
├── include/
│   ├── hils_types.hpp      # メッセージ構造体（simulator側と一致）
│   └── hils_receiver.hpp   # HILS受信クラス
└── src/
    └── hils_receiver.cpp   # パーサー・バッファ管理
```

**hils_types.hpp の内容:**
```cpp
#pragma pack(push, 1)
struct HILSIMUData {
    uint8_t type;           // 0x10
    uint32_t timestamp_us;
    float gyro[3];          // rad/s
    float accel[3];         // m/s²
    uint8_t checksum;
};  // 28 bytes

struct HILSMotorOutput {
    uint8_t type;           // 0x20
    uint32_t timestamp_us;
    float motors[4];        // 0-1 normalized
    uint8_t checksum;
};  // 22 bytes
#pragma pack(pop)
```

**2. HILSReceiver クラス:**
- USB CDCからのバイナリデータを受信
- リングバッファでパケットを蓄積
- 型別にパース → キューに格納

### Phase 2: HILSタスク作成

**3. `hils_task.cpp` 新規作成**

```cpp
void HILSTask(void* pvParameters) {
    while (true) {
        if (!g_hils_mode) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // IMUデータ受信 → State更新
        HILSIMUData imu_data;
        if (g_hils_receiver.getIMU(&imu_data)) {
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

if (g_hils_mode) {
    HILSMotorOutput pkt;
    pkt.type = 0x20;
    pkt.timestamp_us = esp_timer_get_time();
    g_motor.getOutputs(pkt.motors);  // [0]=M1, [1]=M2, [2]=M3, [3]=M4
    pkt.checksum = hils_checksum(&pkt, sizeof(pkt) - 1);
    g_hils_receiver.send(&pkt, sizeof(pkt));
}
```

### Phase 4: CLIコマンド追加

**5. CLI に `hils` コマンド追加**

```cpp
// cli.cpp に追加
void cmd_hils(int argc, char** argv, void* ctx) {
    if (argc < 2) {
        g_cli.print("Usage: hils [start|stop|status]\r\n");
        return;
    }

    if (strcmp(argv[1], "start") == 0) {
        g_hils_mode = true;
        // センサタスクを停止（または無視モードに）
        g_cli.print("HILS mode enabled\r\n");
    } else if (strcmp(argv[1], "stop") == 0) {
        g_hils_mode = false;
        g_cli.print("HILS mode disabled\r\n");
    } else if (strcmp(argv[1], "status") == 0) {
        g_cli.print("HILS mode: %s\r\n", g_hils_mode ? "ON" : "OFF");
    }
}

// registerDefaultCommands() に追加
registerCommand("hils", cmd_hils, "HILS mode control");
```

### Phase 5: グローバル変数・タスク管理

**6. globals.hpp/cpp に追加**

```cpp
// globals.hpp
extern bool g_hils_mode;
extern HILSReceiver g_hils_receiver;

// globals.cpp
bool g_hils_mode = false;
HILSReceiver g_hils_receiver;
```

**7. main.cpp でHILSタスク起動**

```cpp
// タスク生成部分に追加
xTaskCreatePinnedToCore(
    HILSTask,
    "HILSTask",
    4096,
    nullptr,
    3,  // 中優先度
    nullptr,
    0   // Core 0
);
```

**8. センサタスクのHILSモード対応**

各センサタスク（imu_task, mag_task, baro_task等）の先頭に追加:
```cpp
if (g_hils_mode) {
    vTaskDelay(pdMS_TO_TICKS(100));  // HILSモード時はスキップ
    continue;
}
```

## 5. ファイル一覧

### 新規作成
| ファイル | 内容 |
|----------|------|
| `components/sf_svc_hils/CMakeLists.txt` | コンポーネント定義 |
| `components/sf_svc_hils/include/hils_types.hpp` | メッセージ構造体 |
| `components/sf_svc_hils/include/hils_receiver.hpp` | 受信クラス定義 |
| `components/sf_svc_hils/src/hils_receiver.cpp` | 受信・解析実装 |
| `main/tasks/hils_task.cpp` | HILSタスク |

### 修正
| ファイル | 変更内容 |
|----------|----------|
| `main/globals.hpp` | `g_hils_mode`, `g_hils_receiver` 追加 |
| `main/globals.cpp` | 同上の定義 |
| `main/main.cpp` | HILSタスク起動追加 |
| `main/tasks/control_task.cpp` | モーター出力送信フック |
| `main/tasks/imu_task.cpp` | HILSモード時スキップ |
| `main/tasks/mag_task.cpp` | 同上 |
| `main/tasks/baro_task.cpp` | 同上 |
| `main/tasks/tof_task.cpp` | 同上 |
| `main/tasks/optflow_task.cpp` | 同上 |
| `components/sf_svc_cli/src/cli.cpp` | `hils` コマンド追加 |

## 6. テスト手順

1. **ビルド・フラッシュ**
   ```bash
   cd firmware/vehicle
   idf.py build flash monitor
   ```

2. **HILSモード有効化**
   ```
   > hils start
   HILS mode enabled
   ```

3. **シミュレータ接続**
   ```python
   from simulator.interfaces import HILSInterface

   hils = HILSInterface()
   hils.connect('/dev/tty.usbmodem*')
   hils.enable_hils()

   # センサ注入
   hils.inject_imu(timestamp, gyro, accel)

   # モーター出力取得
   motor = hils.get_motor_output(timeout=0.01)
   print(motor.motors)
   ```

4. **ループバックテスト**
   - センサ注入 → State更新 → 制御計算 → モーター出力
   - 期待値と比較

## 7. 注意事項

- **タイミング**: USB遅延（1-5ms）があるため、リアルタイム性は制限される
- **初期化**: HILSモード時もセンサ安定化チェックはスキップが必要
- **安全**: g_hils_mode中はモーターPWM出力を無効化（オプション）
- **同期**: シミュレータ側でファームウェアのタイムスタンプに追従
