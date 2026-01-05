# Stage 3: VL53L3CX デバイス初期化テスト

## 概要

このステージでは、VL53LX API を使用したデバイス初期化シーケンスをテストします。

## 初期化シーケンス

```c
// 1. デバイスブート待機
VL53LX_WaitDeviceBooted(&vl53lx_dev);

// 2. デバイスデータ初期化
VL53LX_DataInit(&vl53lx_dev);

// 3. デバイス情報取得
VL53LX_GetDeviceInfo(&vl53lx_dev, &device_info);
```

## 期待される出力

```
Stage 3: Device Initialization
VL53LX API Test
==================================
XSHUT pins initialized
Front ToF (GPIO9): ENABLED
Bottom ToF (GPIO7): DISABLED
I2C master initialized successfully
SDA: GPIO3, SCL: GPIO4
==================================
VL53LX API Initialization Sequence
==================================
Step 1: Waiting for device boot...
✓ Device booted successfully
Step 2: Initializing device data...
✓ Device data initialized successfully
Step 3: Reading device information...
==================================
Device Information:
==================================
Product Type    : 0xAA
Product Revision: 1.0
==================================
✓ VL53L3CX device confirmed (Product Type: 0xAA)
==================================
✓ Device initialization complete!
==================================
```

## デバイス情報

VL53LX_DeviceInfo_t 構造体:
- **ProductType**: 0xAA (VL53L3CX固有の値、データシートではModule Type)
- **ProductRevisionMajor**: メジャーバージョン番号
- **ProductRevisionMinor**: マイナーバージョン番号

## ハードウェア接続

- **I2C SDA**: GPIO3
- **I2C SCL**: GPIO4
- **Front ToF XSHUT**: GPIO9 (HIGH: 有効)
- **Bottom ToF XSHUT**: GPIO7 (LOW: 無効)

## ビルド方法

```bash
cd examples/stage3_device_init
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 実装の詳細

### プラットフォーム層の拡張

このステージでは、以下のプラットフォーム機能を追加実装しました:

1. **VL53LX_WaitValueMaskEx()**: レジスタポーリング関数
   - タイムアウト付きのマスク値ポーリング
   - VL53LX_WaitDeviceBooted()内部で使用

2. **ESP-IDF互換性の修正**:
   - `vl53lx_platform_user_data.h`: STM32 HAL依存を削除
   - `I2C_HandleTypeDef` → `i2c_master_dev_handle_t`

### VL53LX BareDriver統合

- **ドライババージョン**: 1.2.14
- **統合ファイル数**: 58ファイル (20ソース + 38ヘッダー)
- **コア機能**:
  - デバイスブート検出
  - データ構造初期化
  - デバイス情報読み出し

## トラブルシューティング

### Product Type が 0xCC の場合

```
✗ VL53L1 device detected (Product Type: 0xCC)
  This is not a VL53L3CX sensor!
```

→ VL53L1センサーが接続されています。VL53L3CX (0xAA) に交換してください。

### デバイスブート失敗

```
Device boot failed (status: -1)
```

確認事項:
1. XSHUT ピンがHIGHになっているか
2. I2C配線が正しいか (SDA/SCL)
3. センサーに電源が供給されているか

## 次のステップ

Stage 4 では、実際の距離測定機能を実装します:
- 測定開始 (VL53LX_StartMeasurement)
- データ待機 (ポーリング)
- 測定結果取得 (VL53LX_GetRangingMeasurementData)
