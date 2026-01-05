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

# Step 1: FIFO Basic Read - FIFO基本読み取り

## 概要

BMI270のFIFO機能の最小構成を実装し、1フレーム手動読み取りで基本動作を確認します。

## 目的

1. **FIFO基本設定**: ACC+GYR、ヘッダーモード、ストリームモード
2. **FIFO_LENGTH読み取り**: ポーリングでFIFOデータ量を確認
3. **FIFO_DATA手動読み取り**: 1フレーム（13バイト）を読み取り
4. **ヘッダー検証**: 期待値0x8Cの確認
5. **データ精度検証**: Stage 4との比較

## FIFO設定詳細

### レジスタ設定

```c
// FIFO_CONFIG_0 (0x48): ストリームモード
// stop_on_full = 0 (満杯時に古いデータを上書き)
uint8_t fifo_config_0 = 0x00;

// FIFO_CONFIG_1 (0x49): ACC+GYR有効、ヘッダーモード
// bit 7: fifo_gyr_en = 1 (ジャイロ有効)
// bit 6: fifo_acc_en = 1 (加速度有効)
// bit 4: fifo_header_en = 1 (ヘッダーモード)
uint8_t fifo_config_1 = 0xD0;
```

### FIFOフレーム構成

**ACC+GYRフレーム（13バイト）:**
```
[0]     ヘッダー: 0x8C (ACC+GYRフレームを示す)
[1-2]   GYR_X (LSB, MSB) ← ジャイロが先！
[3-4]   GYR_Y (LSB, MSB)
[5-6]   GYR_Z (LSB, MSB)
[7-8]   ACC_X (LSB, MSB) ← 加速度が後！
[9-10]  ACC_Y (LSB, MSB)
[11-12] ACC_Z (LSB, MSB)
```

**重要**: FIFOでは**ジャイロデータが先、加速度データが後**の順序で格納されます。

### 動作フロー

1. センサー初期化（ACC/GYR 100Hz設定）
2. FIFO設定（ACC+GYR、ヘッダーモード）
3. ループ:
   - `FIFO_LENGTH`レジスタ（0x24-0x25）読み取り
   - 13バイト以上あれば、`FIFO_DATA`（0x26）から13バイト読み取り
   - ヘッダー確認（0x8Cか？）
   - **異常ヘッダー検出時（0x8C以外）**: テスト終了
   - GYR/ACCデータ解析・表示
   - 100ms待機（次回ポーリング）

**注**: 1フレームのみ読み取るため、FIFOが満杯になりデータロスが発生します。異常ヘッダー（0x40=スキップフレーム、0x48=設定変更）が検出された時点でテストは正常に終了します。

## ビルド＆実行

### 1. 環境セットアップ

```bash
source ~/esp/esp-idf/export.sh
```

### 2. ビルド

```bash
cd examples/stage5_fifo/step1_basic_read
idf.py build
```

### 3. 書き込みとモニタ

```bash
idf.py flash monitor
```

## 期待される出力

### 起動時ログ

```
I (XXX) BMI270_STEP1: ========================================
I (XXX) BMI270_STEP1:  BMI270 Step 1: FIFO Basic Manual Read
I (XXX) BMI270_STEP1: ========================================
I (XXX) BMI270_STEP1: Step 7: Configuring FIFO...
I (XXX) BMI270_STEP1: FIFO configured: ACC+GYR enabled, Header mode, Stream mode
I (XXX) BMI270_STEP1: FIFO_CONFIG_1 readback: 0xD0 (expected 0xD0)
```

### データ読み取りログ

```
I (XXX) BMI270_STEP1: ----------------------------------------
I (XXX) BMI270_STEP1: Frame #1, FIFO length: 26 bytes
I (XXX) BMI270_STEP1: Frame header: 0x8C
I (XXX) BMI270_STEP1: GYR RAW: X=    12, Y=   -45, Z=    78
I (XXX) BMI270_STEP1: ACC RAW: X=  -123, Y=   456, Z=  8192
I (XXX) BMI270_STEP1: GYR: X=   0.37°/s, Y=  -1.37°/s, Z=   2.38°/s
I (XXX) BMI270_STEP1: ACC: X=-0.015g, Y= 0.056g, Z= 1.000g
I (XXX) BMI270_STEP1: FIFO length after read: 13 bytes (consumed: 13 bytes)
```

## 検証ポイント

### ✓ FIFO設定が正しいか

```
FIFO_CONFIG_1 readback: 0xD0 (expected 0xD0)
```
→ 0xD0が読み取れればOK

### ✓ ヘッダーが正しいか

```
Frame header: 0x8C
```
→ 0x8C（ACC+GYRフレーム）が読み取れればOK

### ✓ データ精度が正しいか

- `acc_z` ≈ 1.0g（静止時の重力加速度）
- `gyr` ≈ 0°/s（静止時の角速度）

Stage 4の出力と比較して、同等の精度であることを確認

### ✓ FIFOが正しく消費されているか

```
FIFO length after read: 13 bytes (consumed: 13 bytes)
```
→ 読み取り前後でFIFO長さが13バイト減っていればOK

### ✓ テスト正常終了

```
W (XXX) BMI270_STEP1: Unexpected header! Expected 0x8C, got 0x40
W (XXX) BMI270_STEP1: This indicates FIFO overflow or config change frame
I (XXX) BMI270_STEP1: ========================================
I (XXX) BMI270_STEP1:  Step 1 Complete: FIFO basic operation verified
I (XXX) BMI270_STEP1:  Successfully read 11 valid frames
I (XXX) BMI270_STEP1: ========================================
```
→ 異常ヘッダー検出後、正常フレーム数を報告して終了

## 動作特性（意図的な設計）

- **1フレームのみ読み取り**: 複数フレームが蓄積していても1フレーム（13バイト）のみ処理
- **データロスで終了**: FIFOが満杯になりデータロスが発生すると自動終了
- **ウォーターマークなし**: FIFOが満杯（2048バイト）になるまで蓄積
- **割り込みなし**: ポーリング（100ms間隔）でFIFO長さをチェック

これらの制限はStep 1の目的（基本動作確認）のための意図的な設計です。Step 2以降で改善します。

## 次のステップ

**Step 2 (step2_multi_frames)**: 複数フレームの連続読み取りとパーシング
