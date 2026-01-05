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

# Stage 2: BMI270 Initialization Sequence

## 目的

BMI270の完全な初期化シーケンスを実行し、以下を確認します：

- ソフトリセット実行
- 8KBコンフィグファイルアップロード
- INTERNAL_STATUSポーリング
- 初期化完了確認（message = 0x01）
- 加速度センサー・ジャイロセンサーの有効化

## ハードウェア

- **MCU**: ESP32-S3 (M5StampS3)
- **センサー**: BMI270 6軸IMU
- **通信**: SPI (10 MHz, Mode 0)

### ピン接続

| 信号 | ESP32-S3 GPIO | BMI270ピン |
|------|---------------|------------|
| MOSI | GPIO14 | SDx (SDI) |
| MISO | GPIO43 | SDO |
| SCK  | GPIO44 | SCx |
| CS   | GPIO46 | CSB |

## ビルド＆実行

### 1. 環境セットアップ

```bash
source ~/esp/esp-idf/export.sh
```

### 2. ターゲット設定（初回のみ）

```bash
cd examples/stage2_init
idf.py set-target esp32s3
```

### 3. ビルド

```bash
idf.py build
```

### 4. フラッシュ＆モニター

```bash
idf.py flash monitor
```

終了: `Ctrl+]`

## 期待される出力

```
I (xxx) BMI270_TEST: ========================================
I (xxx) BMI270_TEST:  BMI270 Stage 2: Initialization Test
I (xxx) BMI270_TEST: ========================================

I (xxx) BMI270_TEST: Step 1: Initializing SPI...
I (xxx) BMI270_SPI: SPI bus initialized on host 1
I (xxx) BMI270_SPI: BMI270 SPI device added successfully
I (xxx) BMI270_TEST: ✓ SPI initialized successfully

I (xxx) BMI270_TEST: Step 2: Activating SPI mode...
I (xxx) BMI270_TEST: SPI mode activated

I (xxx) BMI270_TEST: Step 3: Initializing BMI270...
I (xxx) BMI270_INIT: ========================================
I (xxx) BMI270_INIT:  BMI270 Initialization Sequence
I (xxx) BMI270_INIT: ========================================

I (xxx) BMI270_INIT: Step 1: Soft reset
I (xxx) BMI270_INIT: Performing soft reset...
I (xxx) BMI270_INIT: Soft reset complete, CHIP_ID verified: 0x24

I (xxx) BMI270_INIT: Step 2: Upload configuration file
I (xxx) BMI270_INIT: Starting config file upload (8192 bytes)...
I (xxx) BMI270_INIT: Disabling advanced power save...
I (xxx) BMI270_INIT: Preparing for config file upload (INIT_CTRL = 0x00)...
I (xxx) BMI270_INIT: Uploading config file in 32-byte bursts...
I (xxx) BMI270_INIT: Progress: 1024 / 8192 bytes (12.5%)
I (xxx) BMI270_INIT: Progress: 2048 / 8192 bytes (25.0%)
I (xxx) BMI270_INIT: Progress: 3072 / 8192 bytes (37.5%)
I (xxx) BMI270_INIT: Progress: 4096 / 8192 bytes (50.0%)
I (xxx) BMI270_INIT: Progress: 5120 / 8192 bytes (62.5%)
I (xxx) BMI270_INIT: Progress: 6144 / 8192 bytes (75.0%)
I (xxx) BMI270_INIT: Progress: 7168 / 8192 bytes (87.5%)
I (xxx) BMI270_INIT: Config file upload complete (8192 bytes)
I (xxx) BMI270_INIT: Signaling config upload complete (INIT_CTRL = 0x01)...

I (xxx) BMI270_INIT: Step 3: Wait for initialization complete
I (xxx) BMI270_INIT: Waiting for initialization to complete (max 20 ms)...
I (xxx) BMI270_INIT: INTERNAL_STATUS = 0x01, message = 0x01
I (xxx) BMI270_INIT: ✓ Initialization complete (message = 0x01)

I (xxx) BMI270_INIT: Step 4: Enable sensors
I (xxx) BMI270_INIT: Accelerometer and gyroscope enabled (PWR_CTRL = 0x06)
I (xxx) BMI270_SPI: BMI270 initialization complete - switched to normal mode timing

I (xxx) BMI270_INIT: ========================================
I (xxx) BMI270_INIT:  ✓ BMI270 Initialization Complete
I (xxx) BMI270_INIT: ========================================

I (xxx) BMI270_TEST: Step 4: Verifying initialization...
I (xxx) BMI270_TEST: Verifying initialization status...
I (xxx) BMI270_TEST: INTERNAL_STATUS = 0x01, message = 0x01
I (xxx) BMI270_TEST: ✓ Initialization status: OK (message = 0x01)

I (xxx) BMI270_TEST: Step 5: Verifying sensor power...
I (xxx) BMI270_TEST: Verifying sensor power status...
I (xxx) BMI270_TEST: PWR_CTRL = 0x06
I (xxx) BMI270_TEST:   Accelerometer: ENABLED
I (xxx) BMI270_TEST:   Gyroscope:     ENABLED
I (xxx) BMI270_TEST: ✓ Both sensors enabled

I (xxx) BMI270_TEST: ========================================
I (xxx) BMI270_TEST:  ✓ ALL TESTS PASSED
I (xxx) BMI270_TEST: ========================================
I (xxx) BMI270_TEST: Stage 2 completed successfully!
I (xxx) BMI270_TEST: BMI270 is initialized and ready for data reading.
I (xxx) BMI270_TEST:
I (xxx) BMI270_TEST: Next step: Stage 3 - Polling data read
```

## 成功基準

- [x] ソフトリセット成功（CHIP_ID = 0x24）
- [x] 8KBコンフィグファイル完全アップロード
- [x] INTERNAL_STATUS message = 0x01（初期化OK）
- [x] PWR_CTRL = 0x06（ACC + GYR有効）
- [x] 初期化完了時間 < 20ms

## 重要な技術ポイント

### 1. 初期化シーケンス

BMI270の初期化は以下の順序で実行する必要があります：

```
1. ソフトリセット (CMD = 0xB6)
   ↓ 2ms待機
2. PWR_CONF = 0x00 (advanced power save無効化)
   ↓ 450µs待機
3. INIT_CTRL = 0x00 (アップロード準備)
   ↓
4. 8KBコンフィグファイルアップロード (32バイトバースト)
   ↓
5. INIT_CTRL = 0x01 (アップロード完了)
   ↓
6. INTERNAL_STATUSポーリング (message = 0x01を待つ)
   ↓ 最大20ms
7. PWR_CTRL = 0x06 (ACC + GYR有効化)
```

### 2. タイミング要件

- **ソフトリセット後**: 2ms待機必須
- **PWR_CONF設定後**: 450µs待機必須
- **INTERNAL_STATUSポーリング**: 1ms間隔、最大20msタイムアウト
- **初期化完了後**: 通常モードタイミング（2µs）に切り替え

### 3. 動的タイミング制御

`bmi270_dev_t`の`init_complete`フラグにより、レジスタアクセス遅延が自動切り替え：

- **低電力モード（初期化前）**: 1000µs遅延
- **通常モード（初期化後）**: 2µs遅延

この切り替えにより、初期化後の高速データ読み取りが可能になります。

### 4. コンフィグファイルアップロード

8192バイトのコンフィグファイルは32バイト単位でバースト書き込み：

```c
#define BMI270_CONFIG_FILE_SIZE  8192
#define BMI270_CONFIG_BURST_SIZE 32

for (size_t offset = 0; offset < 8192; offset += 32) {
    bmi270_write_burst(dev, BMI270_REG_INIT_DATA,
                      &config_file[offset], 32);
}
```

## トラブルシューティング

### INTERNAL_STATUS がタイムアウトする

**原因**: コンフィグファイルアップロード失敗の可能性

**対策**:
1. コンフィグファイルサイズを確認（正確に8192バイト）
2. SPI通信エラーログを確認
3. バースト書き込みが正常に完了しているか確認

### 初期化後も init_complete が false のまま

**原因**: `bmi270_set_init_complete()`が呼ばれていない

**対策**:
- `bmi270_init()`関数が正常に完了しているか確認
- ログで "BMI270 initialization complete - switched to normal mode timing" を確認

### センサーが有効にならない (PWR_CTRL != 0x06)

**原因**: 初期化が完了していない可能性

**対策**:
1. INTERNAL_STATUSが0x01になっているか確認
2. 初期化シーケンスが全て完了しているか確認
3. エラーログを詳細確認

## 次のステップ

Stage 2が成功したら、次は **Stage 3: ポーリング読み取り** に進みます。

Stage 3では以下を実装します：
- 加速度データ読み取り（ACC_X, ACC_Y, ACC_Z）
- 角速度データ読み取り（GYR_X, GYR_Y, GYR_Z）
- データフォーマット変換（LSB → 物理値）
- サンプリングレート設定

## 参考ドキュメント

- [docs/bmi270_doc_ja.md](../../docs/bmi270_doc_ja.md) - BMI270実装ガイド
- [include/bmi270_init.h](../../include/bmi270_init.h) - 初期化API仕様
- [src/bmi270_init.c](../../src/bmi270_init.c) - 初期化実装
