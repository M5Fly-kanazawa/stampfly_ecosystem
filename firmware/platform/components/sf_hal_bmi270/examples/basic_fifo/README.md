<!--
SPDX-License-Identifier: MIT

Copyright (c) 2025 Kouhei Ito

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-->

# Basic FIFO Example - FIFOサンプル

BMI270のFIFOバッファとウォーターマーク割り込みを使用した、高性能データ取得のサンプルです。

## 特徴

- **高速**: 1600Hz ODRで連続取得、50Hz出力
- **効率的**: バッチ処理により最小のCPU使用率
- **低レイテンシ**: 20ms間隔で32フレームを一括読み取り
- **データロス最小**: FIFOバッファでデータを保持
- **実用的**: 高速モーショントラッキングに最適

## 他の方式との比較

| 項目 | ポーリング | 割り込み | FIFO（本サンプル） |
|------|----------|---------|------------------|
| CPU使用率 | 高い | 低い | **最小** |
| データレート | 〜200Hz | 〜200Hz | **1600Hz+** |
| レイテンシ | ポーリング間隔 | 即座 | **バッチ処理** |
| データロス | 高い | 中程度 | **最小** |
| 実装難易度 | 簡単 | 中程度 | やや複雑 |

## 動作概要

### FIFOバッファの仕組み

```
[BMI270 FIFO] (2048バイト)
    ↓ センサーデータが蓄積 (1600Hz)
    ↓ 32フレーム = 416バイトに到達
    ↓ ウォーターマーク割り込み発生
[INT1] GPIO11
    ↓ ESP32-S3割り込み
[タスク起床]
    ↓ FIFO一括読み取り (416バイト)
    ↓ 32フレーム解析
    ↓ 平均値計算
    ↓ 出力 (50Hz)
[タスクスリープ] (20ms待機)
```

### 詳細フロー

1. SPI通信初期化
2. BMI270センサー初期化
3. センサー設定（**1600Hz**, ±4g, ±1000°/s）
4. FIFO設定（ACC+GYR、ヘッダーモード、ストリームモード）
5. ウォーターマーク設定（**416バイト = 32フレーム**）
6. INT1割り込み設定（GPIO11、立ち上がりエッジ）
7. 割り込みマッピング（FIFO watermark → INT1）
8. **ループ開始**:
   - FIFOに32フレーム蓄積（20ms）
   - ウォーターマーク割り込み発生
   - タスク起床 → FIFO一括読み取り
   - 32フレーム解析 → 平均値計算 → 出力
   - タスクスリープ（次の割り込みまで）

## ハードウェア接続

このサンプルはM5StampFly向けに設定されています：

| BMI270 | ESP32-S3 GPIO | 用途 |
|--------|---------------|------|
| MOSI   | GPIO14        | SPI データ出力 |
| MISO   | GPIO43        | SPI データ入力 |
| SCK    | GPIO44        | SPI クロック |
| CS     | GPIO46        | SPI チップセレクト |
| **INT1** | **GPIO11** | **FIFO割り込み** |

## ビルド＆実行

```bash
# 環境セットアップ
source ~/esp/esp-idf/export.sh

# ターゲット設定（初回のみ）
cd examples/basic_fifo
idf.py set-target esp32s3

# ビルド＆書き込み
idf.py build flash monitor
```

## 期待される出力

```
I (XXX) BMI270_BASIC_FIFO: ========================================
I (XXX) BMI270_BASIC_FIFO:  BMI270 Step 4: FIFO Watermark Interrupt
I (XXX) BMI270_BASIC_FIFO: ========================================
I (XXX) BMI270_BASIC_FIFO: Step 1: Initializing SPI bus...
I (XXX) BMI270_BASIC_FIFO: SPI initialized successfully
I (XXX) BMI270_BASIC_FIFO: Step 2: Initializing BMI270...
I (XXX) BMI270_BASIC_FIFO: BMI270 initialized successfully
I (XXX) BMI270_BASIC_FIFO: Step 3: Configuring accelerometer (1600Hz, ±4g)...
I (XXX) BMI270_BASIC_FIFO: Step 4: Configuring gyroscope (1600Hz, ±1000°/s)...
I (XXX) BMI270_BASIC_FIFO: Step 5: Configuring FIFO...
I (XXX) BMI270_BASIC_FIFO: FIFO configured: ACC+GYR enabled, Header mode, Stream mode
I (XXX) BMI270_BASIC_FIFO: Step 7: Configuring GPIO INT1 (GPIO11)...
I (XXX) BMI270_BASIC_FIFO: GPIO INT1 configured successfully
I (XXX) BMI270_BASIC_FIFO: Step 8: Flushing FIFO before enabling interrupt...
I (XXX) BMI270_BASIC_FIFO: Step 9: Creating semaphore for interrupt notification...
I (XXX) BMI270_BASIC_FIFO: Step 10: Mapping FIFO watermark interrupt to INT1...
I (XXX) BMI270_BASIC_FIFO: Step 11: Creating FIFO read task...
I (XXX) BMI270_BASIC_FIFO: ========================================
I (XXX) BMI270_BASIC_FIFO:  Interrupt-driven FIFO read active
I (XXX) BMI270_BASIC_FIFO:  Watermark: 416 bytes (32 frames)
I (XXX) BMI270_BASIC_FIFO: ========================================
I (XXX) BMI270_BASIC_FIFO: Press 't' or 'o' to toggle Teleplot output ON/OFF
I (XXX) BMI270_BASIC_FIFO: Teleplot output: ENABLED
>gyr_x:-0.001
>gyr_y:0.002
>gyr_z:-0.001
>acc_x:0.012
>acc_y:-0.024
>acc_z:0.995
```

## FIFO設定詳細

### ウォーターマーク設定

```c
#define FIFO_WATERMARK_BYTES 416  // 32フレーム × 13バイト/フレーム

// ウォーターマークレジスタに書き込み
uint16_t watermark = FIFO_WATERMARK_BYTES;
uint8_t wtm_lsb = watermark & 0xFF;        // 0xA0
uint8_t wtm_msb = (watermark >> 8) & 0x07; // 0x01
bmi270_write_register(&g_dev, BMI270_REG_FIFO_WTM_0, wtm_lsb);
bmi270_write_register(&g_dev, BMI270_REG_FIFO_WTM_1, wtm_msb);
```

**なぜ416バイト（32フレーム）？**
- FIFO総容量: 2048バイト
- 1フレーム: 13バイト（ヘッダー1 + GYR6 + ACC6）
- 最大格納: 157フレーム
- **ウォーターマーク: 32フレーム** = 1600Hz ÷ 32 = **50Hz出力**
- 割り込み間隔: 20ms（1秒に50回）

### FIFOフレームフォーマット

ヘッダーモードの場合、各フレームは以下の構造：

```
[Header: 1バイト] [GYR_X_L, GYR_X_H: 2バイト] [GYR_Y_L, GYR_Y_H: 2バイト]
[GYR_Z_L, GYR_Z_H: 2バイト] [ACC_X_L, ACC_X_H: 2バイト] [ACC_Y_L, ACC_Y_H: 2バイト]
[ACC_Z_L, ACC_Z_H: 2バイト]

合計: 13バイト
```

**ヘッダー値**:
- `0x8C`: 正常なACC+GYRフレーム
- `0x40`: スキップフレーム（データロス）
- `0x48`: コンフィグ変更フレーム

## パラメータ調整

### 出力周波数を変更

100Hz出力にする場合（ウォーターマーク=16フレーム）：

```c
#define FIFO_WATERMARK_BYTES 208  // 16フレーム × 13バイト = 208バイト
// 1600Hz ÷ 16フレーム = 100Hz出力、10ms間隔
```

25Hz出力にする場合（ウォーターマーク=64フレーム）：

```c
#define FIFO_WATERMARK_BYTES 832  // 64フレーム × 13バイト = 832バイト
// 1600Hz ÷ 64フレーム = 25Hz出力、40ms間隔
```

### センサーODRを変更

さらに高速化（3200Hz）：

```c
bmi270_set_accel_config(&g_dev, BMI270_ACC_ODR_3200HZ, BMI270_FILTER_PERFORMANCE);
bmi270_set_gyro_config(&g_dev, BMI270_GYR_ODR_3200HZ, BMI270_FILTER_PERFORMANCE);

#define FIFO_WATERMARK_BYTES 416  // 32フレーム
// 3200Hz ÷ 32フレーム = 100Hz出力、10ms間隔
```

### Teleplot出力をオフ/オン切り替え

シリアルモニタで`t`または`o`キーを押すとTeleplot出力を切り替えできます：

```c
// コード内での制御
g_teleplot_enabled = false;  // 無効化
g_teleplot_enabled = true;   // 有効化
```

## データロス検出と対策

### Skip Frame検出

FIFOバッファがオーバーフローすると`0x40`（スキップフレーム）が記録されます：

```c
if (frame[0] == FIFO_HEADER_SKIP) {
    ESP_LOGW(TAG, "Data loss detected!");
    skip_detected = true;
    g_skip_frames++;
}
```

### 自動復旧

スキップフレーム検出時、FIFOを自動フラッシュして復旧：

```c
if (skip_detected) {
    bmi270_write_register(&g_dev, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
    ESP_LOGW(TAG, "FIFO flushed");
}
```

### データロスを防ぐ方法

1. **ウォーターマークを小さく設定**（読み取り頻度を上げる）
2. **タスク優先度を上げる**
   ```c
   xTaskCreate(fifo_read_task, "fifo_read", 4096, NULL, 10, NULL);  // 優先度10
   ```
3. **割り込み処理を高速化**（SPI通信を最小限に）

## CPU使用率の最適化

FIFOバッチ処理の利点：

- **1フレームずつ読み取る場合**: 1600回/秒のSPI通信
- **32フレームまとめて読み取る場合**: 50回/秒のSPI通信
- **CPU負荷削減**: 約32倍の効率化

## トラブルシューティング

### "FIFO flushed" が頻繁に出る

データロスが発生しています：

1. **ウォーターマークを下げる**
   ```c
   #define FIFO_WATERMARK_BYTES 208  // 32 → 16フレーム
   ```

2. **タスク優先度を上げる**
   ```c
   xTaskCreate(fifo_read_task, "fifo_read", 4096, NULL, 10, NULL);
   ```

3. **printf出力を減らす**
   ```c
   #define OUTPUT_DECIMATION 2  // 1回おきに出力
   ```

### 割り込みが発生しない

1. **INT1ピン設定を確認**
   ```c
   uint8_t int1_ctrl;
   bmi270_read_register(&g_dev, BMI270_REG_INT1_IO_CTRL, &int1_ctrl);
   ESP_LOGI(TAG, "INT1_IO_CTRL: 0x%02X", int1_ctrl);  // 0x0Aのはず
   ```

2. **割り込みマッピングを確認**
   ```c
   uint8_t int_map;
   bmi270_read_register(&g_dev, BMI270_REG_INT_MAP_DATA, &int_map);
   ESP_LOGI(TAG, "INT_MAP_DATA: 0x%02X", int_map);  // 0x02のはず
   ```

### "Unknown header: 0xXX"

FIFOフレーム同期エラー：

- FIFOをフラッシュすると自動的に復旧
- 頻繁に発生する場合、SPI配線を確認

## 次のステップ

さらに高度な応用：

- [`development/stage5_fifo/`](../development/stage5_fifo/) - FIFO機能の段階的学習
- カスタムフィルタリング（移動平均、ローパスフィルタ）の実装
- センサーフュージョン（ジャイロ+加速度）の実装

## API仕様

詳細なAPI仕様は[docs/API.md](../../docs/API.md)を参照してください。
