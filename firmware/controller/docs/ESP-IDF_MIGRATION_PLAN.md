# ESP-IDF v5.4.1 移行計画書

## 1. 概要

### 現在の構成
- **ビルドシステム**: PlatformIO
- **フレームワーク**: Arduino Framework
- **ボード**: M5Stack AtomS3 (ESP32-S3)
- **主要機能**: TDMA同期ドローンコントローラー、ESP-NOW通信、I2Cセンサー、LCD表示、PWMブザー

### 移行先
- **ビルドシステム**: ESP-IDF v5.4.1 ネイティブ (CMake)
- **フレームワーク**: ESP-IDF (Arduino依存なし)
- **M5Stack対応**: M5Unified + M5GFX (ESP Component Registry)

### 移行の複雑度
**低〜中** - M5Unified/M5GFX の活用により大幅に簡略化

---

## 2. M5Unified / M5GFX の活用 (推奨)

### 2.1 概要

M5Stack社が提供する公式ライブラリ:
- **M5Unified**: M5Stackシリーズの統合ライブラリ (ESP-IDF対応)
- **M5GFX**: グラフィックスライブラリ (M5Unifiedの依存)

### 2.2 ESP Component Registry

```yaml
# main/idf_component.yml
dependencies:
  m5stack/m5unified: "^0.2.11"
```

または:
```bash
idf.py add-dependency "m5stack/m5unified^0.2.11"
```

### 2.3 M5Unifiedで提供される機能

| 機能 | 対応状況 | 備考 |
|---|---|---|
| LCD表示 (M5GFX) | ✅ | fillScreen, printf, setRotation等 |
| ボタン検出 | ✅ | wasPressed, pressedFor等 |
| I2C初期化 | ✅ | Wire相当の機能 |
| IMU (MPU6886) | ✅ | 内蔵サポート |
| RGB LED | ✅ | 内蔵サポート |
| 電源管理 | ✅ | バッテリー電圧読み取り等 |

### 2.4 M5AtomS3 サポート状況

**完全サポート対象**: M5ATOMS3 / S3Lite / S3U

### 2.5 既知の注意点

- ESP-IDF v5.2以降でI2Cドライバの警告が出る場合がある
- M5GFXのバージョン不整合に注意 (M5Unifiedと合わせる)
- 最新バージョン (v0.2.11) で多くの問題が修正済み

---

## 3. ファイル別 移行方針

### 3.1 src/main.cpp

| 現在のAPI | 移行後 | 方針 |
|---|---|---|
| `#include <M5AtomS3.h>` | `#include <M5Unified.h>` | M5Unified使用 |
| `M5.begin()` | `M5.begin()` | **そのまま** |
| `M5.update()` | `M5.update()` | **そのまま** |
| `M5.Btn.*` | `M5.BtnA.*` | **ほぼそのまま** |
| `M5.Lcd.*` | `M5.Display.*` | API名変更あり |
| `Wire1.begin()` | M5Unified内部で初期化 | 不要 |
| `delay()` | `vTaskDelay()` | 要変更 |
| `millis()` | `esp_timer_get_time()/1000` | 要変更 |

### 3.2 src/buzzer.cpp

| 現在のAPI | ESP-IDF API | 難易度 |
|---|---|---|
| `ledcSetup()` | `ledc_timer_config()` | 低 |
| `ledcAttachPin()` | `ledc_channel_config()` | 低 |
| `ledcWriteTone()` | `ledc_set_freq()` | 低 |

### 3.3 lib/ATOMS3Joy/ (ジョイスティック)

M5UnifiedはI2Cバスを初期化するため、I2C通信部分のみ置き換え:

| 現在のAPI | ESP-IDF API | 難易度 |
|---|---|---|
| `Wire1.*` | `i2c_master_*` | 中 |

### 3.4 lib/MPU6886/

**オプション1**: M5Unified内蔵IMUを使用 (推奨)
```cpp
M5.Imu.getAccel(&ax, &ay, &az);
M5.Imu.getGyro(&gx, &gy, &gz);
```

**オプション2**: 既存コードをESP-IDF I2C APIで書き換え

### 3.5 lib/FastLED/

ESP-IDF互換のため、そのまま利用可能。
または、M5Unified内蔵のLED制御を使用。

---

## 4. 推奨ディレクトリ構造

```
firmware_controller/
├── CMakeLists.txt              # メインプロジェクト設定
├── sdkconfig                   # ESP-IDF設定
├── sdkconfig.defaults          # デフォルト設定
├── partitions.csv              # パーティションテーブル
│
├── components/                 # カスタムコンポーネント
│   ├── atoms3joy/              # ジョイスティック (I2C書き換え)
│   │   ├── CMakeLists.txt
│   │   ├── idf_component.yml
│   │   ├── include/
│   │   │   └── atoms3joy.h
│   │   └── atoms3joy.cpp
│   │
│   └── buzzer/                 # ブザー (LEDC)
│       ├── CMakeLists.txt
│       ├── idf_component.yml
│       ├── include/
│       │   └── buzzer.h
│       └── buzzer.cpp
│
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml       # M5Unified依存関係
│   ├── Kconfig.projbuild
│   └── main.cpp
│
└── docs/
    └── ESP-IDF_MIGRATION_PLAN.md
```

---

## 5. CMakeLists.txt / idf_component.yml

### プロジェクトルート CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(firmware_controller)
```

### main/idf_component.yml

```yaml
dependencies:
  m5stack/m5unified: "^0.2.11"
  idf:
    version: ">=5.0.0"
```

### main/CMakeLists.txt

```cmake
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_wifi
        esp_now
        esp_timer
        driver
        freertos
        nvs_flash
        spiffs
        atoms3joy
        buzzer
)
```

---

## 6. 実装フェーズ (改訂版)

### フェーズ1: ビルドシステム構築 (1日)
- [ ] ESP-IDFプロジェクト構造作成
- [ ] CMakeLists.txt 作成
- [ ] idf_component.yml でM5Unified依存追加
- [ ] sdkconfig.defaults 作成
- [ ] 空のmain.cpp でビルド確認

### フェーズ2: M5Unified統合 (1日)
- [ ] M5Unified初期化 (`M5.begin()`)
- [ ] LCD表示テスト (`M5.Display.*`)
- [ ] ボタン動作テスト (`M5.BtnA.*`)
- [ ] IMU動作テスト (`M5.Imu.*`)

### フェーズ3: 基本API置き換え (1日)
- [ ] `Arduino.h` 削除
- [ ] delay → vTaskDelay
- [ ] millis → esp_timer
- [ ] LEDC ブザー実装

### フェーズ4: I2Cドライバ実装 (2日)
- [ ] atoms3joy をESP-IDF I2C APIで再実装
- [ ] ジョイスティック動作確認
- [ ] (必要なら) MPU6886 I2C書き換え

### フェーズ5: メインロジック移行 (2日)
- [ ] main.cpp のAPI置き換え
- [ ] TDMA/ESP-NOW ロジック確認
- [ ] SPIFFS → VFS API

### フェーズ6: 統合テスト (2日)
- [ ] TDMA タイミング検証
- [ ] ESP-NOW 通信確認
- [ ] 全機能結合テスト

---

## 7. 工数見積もり (改訂版)

| タスク | 工数 |
|---|---|
| ビルドシステム構築 | 1日 |
| M5Unified統合 | 1日 |
| 基本API置き換え | 1日 |
| I2Cドライバ実装 | 2日 |
| メインロジック移行 | 2日 |
| 統合テスト | 2日 |
| **合計** | **9日 (約2週間)** |

**※ M5Unified活用により、当初見積もり17日 → 9日に短縮**

---

## 8. リスクと対策

| リスク | 影響度 | 対策 |
|---|---|---|
| M5Unified の ESP-IDF v5.4 互換性 | 中 | 最新版 (v0.2.11) を使用、問題あれば v5.3 へダウングレード検討 |
| I2C ドライバ警告 | 低 | M5Unified最新版で対応済みの可能性大 |
| M5GFX バージョン不整合 | 低 | M5Unified依存で自動解決 |
| TDMAタイミング精度 | 中 | esp_timerの精度確認、優先度調整 |

---

## 9. 互換性のあるAPI (変更不要)

以下のAPIはESP-IDFでそのまま利用可能:

- FreeRTOS: `xTaskCreatePinnedToCore`, `xSemaphore*`, `vTaskDelay`
- esp_timer: `esp_timer_create`, `esp_timer_start_periodic`, `esp_timer_get_time`
- ESP-NOW: `esp_now_init`, `esp_now_send`, `esp_now_register_recv_cb`
- WiFi: `esp_wifi_*`
- ESP_LOG: `ESP_LOGI`, `ESP_LOGD`, `ESP_LOGW`, `ESP_LOGE`

---

## 10. M5Unified API対応表

| 現在 (M5AtomS3.h) | M5Unified | 備考 |
|---|---|---|
| `M5.Lcd.fillScreen()` | `M5.Display.fillScreen()` | |
| `M5.Lcd.setCursor()` | `M5.Display.setCursor()` | |
| `M5.Lcd.printf()` | `M5.Display.printf()` | |
| `M5.Lcd.setRotation()` | `M5.Display.setRotation()` | |
| `M5.Lcd.setTextFont()` | `M5.Display.setTextFont()` | |
| `M5.Btn.wasPressed()` | `M5.BtnA.wasPressed()` | ボタン名変更 |
| `M5.Btn.pressedFor()` | `M5.BtnA.pressedFor()` | |
| `M5.Btn.isPressed()` | `M5.BtnA.isPressed()` | |
| (なし) | `M5.Imu.getAccel()` | IMU内蔵サポート |
| (なし) | `M5.Imu.getGyro()` | IMU内蔵サポート |

---

## 11. 参考資料

- [M5Unified GitHub](https://github.com/m5stack/M5Unified)
- [M5Unified ESP Component Registry](https://components.espressif.com/components/m5stack/m5unified)
- [ESP-IDF プログラミングガイド](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/)
- [ESP-IDF I2C マスタードライバ](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/api-reference/peripherals/i2c.html)
- [ESP-IDF LEDC](https://docs.espressif.com/projects/esp-idf/en/v5.4.1/esp32s3/api-reference/peripherals/ledc.html)

---

## 12. 次のステップ

1. 本計画書のレビュー・承認
2. フェーズ1から順次実装開始
3. 各フェーズ完了時に動作確認・コミット
