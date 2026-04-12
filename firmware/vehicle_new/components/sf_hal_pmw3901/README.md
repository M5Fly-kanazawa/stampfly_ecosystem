# PMW3901 Optical Flow Sensor Driver for StampFly

ESP32-S3ベースのStampFlyドローン向けPMW3901オプティカルフローセンサードライバ

## 概要

このプロジェクトは、PixArt PMW3901MB-TXQT光学式モーションセンサー用のESP-IDF対応ドライバです。StampFly（ESP32-S3ベースドローン）での使用を想定して開発されています。

### 特徴

- ✅ **公式マニュアル完全準拠**: PMW3901公式マニュアル通りの初期化・データ読み出し実装
- ✅ **C/C++両対応**: CとC++の両方から使用可能
  - C API: 従来のC言語インターフェース
  - C++ API: RAII、例外処理、型安全な構造体対応
- ✅ **整理されたプロジェクト構造**: src/, include/, docs/, examples/
- ✅ ESP-IDF 5.4対応
- ✅ SPI通信（2MHz, MODE3）
- ✅ StampFly専用ピン配置（デフォルト設定済み）
- ✅ 2つの速度計算方式を実装
  - **StampFly式**: 高度センサーと組み合わせた直接速度計算
  - **PX4式**: Kalmanフィルタ統合向け角速度計算
- ✅ バーストリードモード対応（高効率データ取得）
- ✅ データ品質フィルタリング（SQUAL/Shutter検証）
- ✅ Teleplotリアルタイムグラフ対応
- ✅ フレームキャプチャモード対応（35×35ピクセル）
- ✅ 詳細な実装リファレンスドキュメント付き

## ハードウェア仕様

### StampFly ピン配置

| 機能 | GPIO |
|------|------|
| MISO | 43 |
| MOSI | 14 |
| SCLK | 44 |
| CS   | 12 |

**注意**: GPIO 46はBMI270 IMU（6軸センサー）のCSピンです。PMW3901のCSピンはGPIO 12を使用します。

詳細は[STAMPFLY_HARDWARE_SPEC.md](docs/STAMPFLY_HARDWARE_SPEC.md)を参照してください。

## プロジェクト構造

```
stampfly_opticalflow/
├── src/                        # ソースコード
│   ├── pmw3901.c              # C API実装
│   └── pmw3901_wrapper.cpp    # C++ラッパー実装
├── include/                    # ヘッダーファイル
│   ├── pmw3901.h              # C API
│   ├── pmw3901_wrapper.hpp    # C++ラッパー
│   └── pmw3901_exception.hpp  # C++例外クラス
├── docs/                       # ドキュメント
│   ├── PMW3901_IMPLEMENTATION_REFERENCE.md
│   ├── STAMPFLY_HARDWARE_SPEC.md
│   └── how_to_use_pwm3901.md
├── examples/                   # サンプルコード
│   ├── basic/                 # Cサンプル
│   └── basic_cpp/             # C++サンプル
├── CMakeLists.txt             # ESP-IDFコンポーネント定義
├── README.md
└── CLAUDE.md                  # Claude Code向けドキュメント
```

## インストール

### 1. ESP-IDF環境のセットアップ

```bash
. $HOME/esp/esp-idf/export.sh
```

### 2. プロジェクトへの組み込み

このドライバをESP-IDFプロジェクトに組み込むには、以下のいずれかの方法を使用します。

#### 方法1: GitHubから直接クローン（推奨）

プロジェクトの`components/pmw3901/`ディレクトリにこのリポジトリをクローン：

```bash
cd your_project/components
git clone https://github.com/YOUR_USERNAME/stampfly_opticalflow.git pmw3901
```

#### 方法2: Git submoduleとして追加（推奨）

プロジェクトのGit管理下に追加する場合：

```bash
cd your_project
git submodule add https://github.com/YOUR_USERNAME/stampfly_opticalflow.git components/pmw3901
git submodule update --init --recursive
```

他の環境でクローンする際は：

```bash
git clone --recurse-submodules https://github.com/YOUR_PROJECT.git
# または既存のクローンで
git submodule update --init --recursive
```

#### 方法3: ダウンロードして配置

GitHubからZIPファイルをダウンロードして展開：

```bash
cd your_project/components
# GitHubからダウンロードしたZIPを展開
unzip stampfly_opticalflow-main.zip
mv stampfly_opticalflow-main pmw3901
```

#### main/CMakeLists.txtに依存関係を追加

**重要**: ディレクトリ名は`pmw3901`ですが、コンポーネント名（REQUIRES）は`stampfly_opticalflow`を使用します。

**C言語プロジェクトの場合:**

```cmake
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES stampfly_opticalflow)
```

**C++プロジェクトの場合:**

```cmake
idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES stampfly_opticalflow)
```

C++プロジェクトでは、`sdkconfig.defaults`に以下を追加してC++例外とRTTIを有効化：

```
CONFIG_COMPILER_CXX_EXCEPTIONS=y
CONFIG_COMPILER_CXX_RTTI=y
```

## 使用方法

### 基本的な使い方（C言語）

```c
#include "pmw3901.h"  // include/pmw3901.h

void app_main(void)
{
    // デバイスハンドルと設定
    pmw3901_t dev;
    pmw3901_config_t config;

    // StampFly用のデフォルト設定を取得
    pmw3901_get_default_config(&config);

    // センサー初期化
    ESP_ERROR_CHECK(pmw3901_init(&dev, &config));

    // モーションデータ読み取り
    int16_t delta_x, delta_y;
    while (1) {
        ESP_ERROR_CHECK(pmw3901_read_motion(&dev, &delta_x, &delta_y));

        // 速度計算（例：高度1m, 100Hzサンプリング）
        float vx, vy;
        pmw3901_calculate_velocity_direct(delta_x, delta_y,
                                          1.0f, 0.01f, &vx, &vy);

        ESP_LOGI("MAIN", "Velocity: X=%.3f m/s, Y=%.3f m/s", vx, vy);

        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz
    }

    // クリーンアップ
    pmw3901_deinit(&dev);
}
```

### 基本的な使い方（C++）

```cpp
#include "pmw3901_wrapper.hpp"  // include/pmw3901_wrapper.hpp

extern "C" void app_main(void)
{
    try {
        // センサー初期化（RAII - 自動初期化）
        stampfly::PMW3901 sensor;

        // 高度1m, 100Hzサンプリング
        float altitude = 1.0f;
        float interval = 0.01f;

        while (true) {
            // バーストリードで効率的にデータ取得
            auto burst = sensor.readMotionBurst();

            // 速度計算
            auto velocity = sensor.calculateVelocityDirect(
                burst.delta_x, burst.delta_y, altitude, interval);

            ESP_LOGI("MAIN", "Velocity: X=%.3f m/s, Y=%.3f m/s",
                     velocity.x, velocity.y);

            vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz
        }

        // デストラクタで自動クリーンアップ

    } catch (const stampfly::PMW3901Exception& e) {
        ESP_LOGE("MAIN", "Error: %s", e.what());
    }
}
```

### 速度計算方式

#### 方式1: 直接速度計算

シンプルな位置制御に最適：

```c
float vx, vy;
float altitude = get_altitude();  // 高度センサーから取得
float interval = 0.01f;  // 100Hz
pmw3901_calculate_velocity_direct(delta_x, delta_y, altitude, interval, &vx, &vy);
```

#### 方式2: 角速度計算

航法システム統合に最適：

```c
float flow_x, flow_y;
pmw3901_calculate_flow_rate(delta_x, delta_y, interval, &flow_x, &flow_y);

// 速度に変換
float vx, vy;
pmw3901_flow_rate_to_velocity(flow_x, flow_y, altitude, &vx, &vy);
```

詳細は[PMW3901_IMPLEMENTATION_REFERENCE.md](docs/PMW3901_IMPLEMENTATION_REFERENCE.md)を参照してください。

## サンプルアプリケーション

### C言語サンプル

[examples/basic/](examples/basic/)ディレクトリにサンプルコードがあります。

**機能:**
- PMW3901センサーの初期化（公式マニュアル準拠）
- バーストリードモードによる高速データ取得
- StampFly式速度計算
- データ品質フィルタリング（SQUAL/Shutter）
- **Teleplotリアルタイムグラフ出力**（delta_x, delta_y, velocity_x, velocity_y, squal, shutter, raw_sum）

**ビルドと実行:**
```bash
cd examples/basic
. $HOME/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### C++サンプル

[examples/basic_cpp/](examples/basic_cpp/)ディレクトリにC++サンプルコードがあります。

**機能:**
- C++ラッパーを使用したモダンな実装
- RAII（自動リソース管理）
- 例外ハンドリング
- 型安全な構造体（MotionBurst, Velocity）
- Teleplotリアルタイムグラフ出力

**ビルドと実行:**
```bash
cd examples/basic_cpp
. $HOME/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Teleplotでリアルタイムグラフ表示

1. VSCode拡張機能 "Teleplot" をインストール、または https://teleplot.fr/ にアクセス
2. シリアルポートに接続すると、自動的にグラフが表示されます
3. 7つの変数（delta_x, delta_y, velocity_x, velocity_y, squal, shutter, raw_sum）がリアルタイムで可視化されます

## API リファレンス

### 初期化関数

- `pmw3901_get_default_config()` - StampFly用デフォルト設定取得
- `pmw3901_init()` - センサー初期化
- `pmw3901_deinit()` - センサー終了処理

### データ読み取り関数

- `pmw3901_read_motion()` - モーションデータ読み取り（X, Y）
- `pmw3901_read_motion_burst()` - バースト読み取り（全データ一括）
- `pmw3901_is_motion_detected()` - モーション検出確認

### 速度計算関数

- `pmw3901_calculate_velocity_direct()` - 直接速度計算（StampFly式）
- `pmw3901_calculate_flow_rate()` - 角速度計算（PX4式）
- `pmw3901_flow_rate_to_velocity()` - 角速度→速度変換

### レジスタアクセス関数

- `pmw3901_read_register()` - レジスタ読み取り
- `pmw3901_write_register()` - レジスタ書き込み
- `pmw3901_get_product_id()` - 製品ID取得
- `pmw3901_get_revision_id()` - リビジョンID取得

### フレームキャプチャ関数

- `pmw3901_enable_frame_capture()` - フレームキャプチャ有効化
- `pmw3901_read_frame()` - フレーム読み取り（35×35ピクセル）

## ドキュメント

- [STAMPFLY_HARDWARE_SPEC.md](docs/STAMPFLY_HARDWARE_SPEC.md) - StampFlyハードウェア仕様とピン配置
- [PMW3901_IMPLEMENTATION_REFERENCE.md](docs/PMW3901_IMPLEMENTATION_REFERENCE.md) - 実装詳細と速度計算方式の比較
- [how_to_use_pwm3901.md](docs/how_to_use_pwm3901.md) - PMW3901公式マニュアル完全準拠ドキュメント

## トラブルシューティング

### ビルドエラー

**エラー**: `implicit declaration of function 'ets_delay_us'`

**解決策**: `pmw3901.c`に`#include "rom/ets_sys.h"`を追加してください。

### センサーが検出されない

1. SPI接続を確認
2. ピン配置が正しいか確認（StampFlyのピン配置を使用）
3. Product ID（0x49）とInverse Product ID（0xB6）を確認

### データが不安定

- SPI遅延時間を50μs → 100μs → 200μsに増やしてテスト
- サンプリング周波数を下げる（100Hz → 50Hz）

## 技術仕様

| 項目 | 値 |
|------|-----|
| センサー | PMW3901MB-TXQT |
| 通信方式 | SPI (MODE3) |
| クロック速度 | 2MHz |
| SPI遅延 | 50μs（読み取り/書き込み後） |
| サンプリング推奨 | 100Hz |
| 製品ID | 0x49 |
| Inverse Product ID | 0xB6 |

## ライセンス

MIT License

## 参考文献

- [PixArt PMW3901MB Datasheet](https://wiki.bitcraze.io/_media/projects:crazyflie2:expansionboards:pot0189-pmw3901mb-txqt-ds-r1.00-200317_20170331160807_public.pdf)
- [Bitcraze PMW3901 Arduino Driver](https://github.com/bitcraze/Bitcraze_PMW3901)
- [PX4 Autopilot PMW3901 Driver](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/optical_flow/pmw3901)
- [StampFly Official Repository](https://github.com/M5Fly-kanazawa/StampFly)

## 貢献

バグ報告や機能リクエストはIssueでお願いします。

## 変更履歴

### v1.0.0 (2025-11-14)

#### 実装完了
- ✅ **公式マニュアル完全準拠実装**
  - PMW3901公式マニュアル (how_to_use_pwm3901.md) の全章を完全実装
  - 初期化シーケンス8ステップ（電源投入→SPIリセット→パワーアップリセット→パフォーマンス最適化）
  - 条件分岐・検証・C1/C2キャリブレーション処理を含む完全な実装
- ✅ **データ読み出し**
  - 標準モード：Motion凍結→Delta読み出し→16bit合成
  - バーストモード：0x16コマンド→12バイト連続取得
  - データ品質フィルタリング（SQUAL < 0x19 AND Shutter_Upper == 0x1F）
- ✅ **ESP-IDF 5.4対応**
  - SPI通信（2MHz, MODE3）
  - StampFly専用ピン配置（GPIO 12: CS, 43: MISO, 14: MOSI, 44: SCLK）
- ✅ **速度計算方式**
  - StampFly式：高度センサーと組み合わせた直接速度計算
  - PX4式：Kalmanフィルタ統合向け角速度計算
- ✅ **Teleplot対応**
  - リアルタイムグラフ出力（delta_x, delta_y, velocity_x, velocity_y, squal, shutter, raw_sum）
  - 20Hzストリーミング出力
- ✅ **実機動作確認**
  - Product ID: 0x49, Inverse ID: 0xB6, Revision ID: 0x00
  - SQUAL: 55-158（良好）
  - モーション検出: 正常動作
  - バーストリード: 13バイト正常取得

#### ドキュメント
- 詳細な実装リファレンス (docs/PMW3901_IMPLEMENTATION_REFERENCE.md)
- ハードウェア仕様書 (docs/STAMPFLY_HARDWARE_SPEC.md)
- 公式マニュアル完全準拠ドキュメント (docs/how_to_use_pwm3901.md)

#### 技術仕様
- センサー：PMW3901MB-TXQT
- MCU：ESP32-S3
- フレームワーク：ESP-IDF v5.4.1
- 通信：SPI MODE3, 2MHz
- サンプリング：100Hz推奨（example: 20Hz）
