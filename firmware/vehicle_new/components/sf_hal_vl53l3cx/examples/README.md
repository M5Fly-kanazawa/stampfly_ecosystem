# StampFly ToF サンプルプログラム

StampFly ToFドライバのサンプルプログラム集です。

## サンプルの構成

### 初心者向けサンプル（⭐まずはこちら）

最小限のコードでToFセンサーを使い始められます：

| サンプル | 説明 | 測定方式 | 推奨度 |
|---------|------|---------|-------|
| [basic_polling](basic_polling/) | シンプルなポーリング測定 | ポーリング | ⭐⭐ |
| [basic_interrupt](basic_interrupt/) | シンプルな割り込み測定 | 割り込み | ⭐⭐⭐ |

**どちらを使うべきか？**
- **basic_polling**: よりシンプルで理解しやすい（入門者向け）
- **basic_interrupt**: より効率的で低消費電力（推奨）

### 開発用詳細サンプル

段階的な学習や高度な機能の実装に使用します：

| サンプル | 説明 | ステータス |
|---------|------|-----------|
| [development/stage1_i2c_scan](development/stage1_i2c_scan/) | I2Cバススキャン | ✅ |
| [development/stage2_register_test](development/stage2_register_test/) | レジスタ読み書きテスト | ✅ |
| [development/stage3_device_init](development/stage3_device_init/) | デバイス初期化 | ✅ |
| [development/stage4_polling_measurement](development/stage4_polling_measurement/) | ポーリング測定（詳細版） | ✅ |
| [development/stage5_interrupt_measurement](development/stage5_interrupt_measurement/) | 割り込み測定（詳細版） | ✅ |
| [development/stage6_dual_sensor](development/stage6_dual_sensor/) | 2センサー同時使用 | ✅ |
| [development/stage7_teleplot_streaming](development/stage7_teleplot_streaming/) | Teleplotリアルタイム可視化 | ✅ |
| [development/stage8_filtered_streaming](development/stage8_filtered_streaming/) | カルマンフィルタ付きストリーミング | ✅ |

詳細は [development/README.md](development/README.md) を参照してください。

## クイックスタート

### 1. 初めての測定（ポーリング方式）

```bash
cd examples/basic_polling
idf.py set-target esp32s3
idf.py build flash monitor
```

**出力例:**
```
[1] Distance:  245 mm, Status: 0, Signal: 15.32 Mcps
[2] Distance:  247 mm, Status: 0, Signal: 15.28 Mcps
...
```

### 2. 割り込み方式で測定（より効率的）

```bash
cd examples/basic_interrupt
idf.py set-target esp32s3
idf.py build flash monitor
```

**違い:**
- ポーリング: CPUが常にデータ準備を確認（シンプル）
- 割り込み: データ準備時にGPIO割り込みで通知（効率的）

## 電源要件

**⚠️ 重要：センサーごとに電源要件が異なります**

| センサー | GPIO | 電源要件 | デフォルト |
|---------|------|---------|----------|
| 底面ToF | GPIO7 | USB給電のみで動作 | ✅ 有効 |
| 前方ToF | GPIO9 | バッテリー必要 | ❌ 無効 |

**推奨テスト手順:**
1. まず底面ToF（USB給電のみで動作）でテスト
2. 前方ToFをテストする場合はバッテリーを接続

## 自分のプロジェクトで使う

サンプルコードをベースに、自分のプロジェクトに組み込めます。

### 基本的な手順

1. **コンポーネントをコピー**
   ```bash
   cp -r /path/to/stampfly_tof your_project/components/
   ```

2. **CMakeLists.txtで要求**
   ```cmake
   idf_component_register(
       SRCS "main.c"
       INCLUDE_DIRS "."
       REQUIRES stampfly_tof  # <- 追加
   )
   ```

3. **コードを実装**

   [basic_polling/main/main.c](basic_polling/main/main.c) または [basic_interrupt/main/main.c](basic_interrupt/main/main.c) を参考にしてください。

### 最小限のコード例（ポーリング）

```c
#include "vl53lx_platform.h"
#include "vl53lx_api.h"
#include "stampfly_tof_config.h"

void app_main(void) {
    // 1. I2C初期化
    i2c_master_bus_handle_t bus;
    // ... (詳細は basic_polling 参照)

    // 2. センサー電源ON
    gpio_set_level(STAMPFLY_TOF_BOTTOM_XSHUT, 1);

    // 3. センサー初期化
    VL53LX_Dev_t dev;
    dev.I2cDevAddr = 0x29;
    VL53LX_platform_init(&dev, bus);
    VL53LX_WaitDeviceBooted(&dev);
    VL53LX_DataInit(&dev);

    // 4. 測定
    VL53LX_StartMeasurement(&dev);
    while (1) {
        uint8_t ready = 0;
        VL53LX_GetMeasurementDataReady(&dev, &ready);
        if (ready) {
            VL53LX_MultiRangingData_t data;
            VL53LX_GetMultiRangingData(&dev, &data);

            uint16_t distance = data.RangeData[0].RangeMilliMeter;
            printf("Distance: %d mm\n", distance);

            VL53LX_ClearInterruptAndStartMeasurement(&dev);
        }
    }
}
```

完全なコードは [basic_polling](basic_polling/) または [basic_interrupt](basic_interrupt/) を参照してください。

## よくある質問

### Q: どのサンプルから始めればいいですか？

**A:** [basic_polling](basic_polling/) から始めてください。より効率的な実装が必要になったら [basic_interrupt](basic_interrupt/) に進んでください。

### Q: 前方ToFセンサーを使いたい

**A:** バッテリーを接続してから、コード内の `ENABLE_FRONT_SENSOR` を 1 に設定してください（[stage6](development/stage6_dual_sensor/) 以降のサンプルを参照）。

### Q: リアルタイムでグラフ表示したい

**A:** [stage7_teleplot_streaming](development/stage7_teleplot_streaming/) を使用してください。VSCodeのTeleplot拡張機能でリアルタイム可視化できます。

### Q: ノイズの多い環境で使いたい

**A:** [stage8_filtered_streaming](development/stage8_filtered_streaming/) の1Dカルマンフィルタを使用してください。外れ値を自動的に除去します。

### Q: 両方のセンサーを同時に使いたい

**A:** [stage6_dual_sensor](development/stage6_dual_sensor/) 以降のサンプルを参照してください。バッテリー接続が必要です。

## トラブルシューティング

### センサーが検出されない

1. USB給電が正常か確認
2. I2C配線を確認（SDA: GPIO3, SCL: GPIO4）
3. XSHUTピンが HIGH になっているか確認
4. [stage1_i2c_scan](development/stage1_i2c_scan/) でI2Cバスをスキャン

### 測定が失敗する（前方ToF）

- **原因**: 前方ToFはバッテリー電源が必要です
- **対処**: バッテリーを接続してください

### 測定値が不安定

1. カルマンフィルタを使用（[stage8](development/stage8_filtered_streaming/)）
2. タイミングバジェットを増やす（精度向上）
3. 測定対象の反射率を確認

## 次のステップ

1. ✅ [basic_polling](basic_polling/) でポーリング測定を試す
2. ✅ [basic_interrupt](basic_interrupt/) で割り込み測定を試す
3. 📚 [development/](development/) で高度な機能を学ぶ
4. 📖 [API仕様書](../docs/API.md) で関数リファレンスを確認
5. 🚀 自分のプロジェクトに組み込む

詳細なドキュメントは [メインREADME](../README.md) を参照してください。
