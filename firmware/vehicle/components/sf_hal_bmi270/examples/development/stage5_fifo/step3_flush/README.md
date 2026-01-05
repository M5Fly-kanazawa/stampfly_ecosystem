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

# Step 3: FIFO Flush and Stream Mode - FIFOフラッシュとストリームモード

## 概要

FIFO内の全データを一括読み取りし、**Skip frame（0x40）検出時にFIFOをフラッシュ**して自動復旧することで、データロスからの回復機能を実装します。

## 目的

1. **FIFOフラッシュ機能**: CMD 0xB0でFIFO内の全データをクリア
2. **自動復旧**: Skip frame検出時にFIFOをフラッシュして正常動作に復帰
3. **ストリームモード確認**: FIFO満杯時の動作（古いデータを上書き）
4. **フラッシュ統計**: フラッシュ実行回数を記録
5. **平均値出力**: 複数フレームの平均値を計算してprintf回数を削減（処理速度向上）

## Step 2との違い

| 項目 | Step 2 | Step 3 |
|------|--------|--------|
| Skip frame処理 | ログ出力のみ | ログ出力 + FIFOフラッシュ |
| 復旧機能 | なし | あり（自動フラッシュ） |
| 統計情報 | Total/Valid/Skip/Config | Total/Valid/Skip/Config/Flush |
| データロス対応 | 継続読み取り | フラッシュして再開 |

## FIFO設定詳細

### 設定（Step 2と同じ）

```c
// FIFO_CONFIG_0 (0x48): ストリームモード
uint8_t fifo_config_0 = 0x00;

// FIFO_CONFIG_1 (0x49): ACC+GYR有効、ヘッダーモード
uint8_t fifo_config_1 = 0xD0;
```

### FIFOフラッシュコマンド

```c
// CMD register (0x7E) に 0xB0 を書き込み
#define BMI270_REG_CMD              0x7E
#define BMI270_CMD_FIFO_FLUSH       0xB0

esp_err_t fifo_flush(void)
{
    return bmi270_write_register(&g_dev, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
}
```

### 動作フロー

1. センサー初期化（ACC/GYR 100Hz設定）
2. FIFO設定（ACC+GYR、ヘッダーモード、ストリームモード）
3. ループ:
   - `FIFO_LENGTH`レジスタ（0x24-0x25）読み取り
   - FIFO_LENGTH分を**一括バースト読み取り**
   - バッファを13バイトずつパース:
     - 各フレームのデータを蓄積（平均値計算用）
     - 0x8C（ACC+GYR）: データ蓄積
     - 0x48（設定変更）: デバッグログ出力、継続
     - **0x40（スキップ）**: 警告ログ出力、skip_detectedフラグセット
   - 平均値計算とTeleplot出力
   - **Skip frame検出時**: FIFOフラッシュ（CMD 0xB0）
   - 統計情報表示（Flush回数含む）
   - 100ms待機（次回ポーリング）

## タイムスタンプと平均値出力

### 実装詳細（Step 2と同じ）

複数フレームを読み取り、**平均値を計算して出力**することでprintf処理のオーバーヘッドを削減します。

**タイムスタンプ:**
- 初回読み取り時: `g_start_time_us`を現在時刻に設定（基準点）
- 各バッチ読み取り時: 読み取り時刻を記録
- 相対時刻(秒) = (読み取り時刻(us) - g_start_time_us) / 1,000,000

**平均値計算:**
- 全有効フレームのデータを蓄積
- 平均値 = 合計値 / 有効フレーム数
- 1回の読み取りで6行のprintf出力のみ（従来は6×フレーム数行）

## FIFOフラッシュ機能

### 使用タイミング

1. **Skip frame（0x40）検出時**: データロスが発生している可能性があるため、FIFOをクリアして再開
2. **異常ヘッダー多発時**: フレーム同期が崩れている可能性があるため、FIFOをクリアして再同期

### フラッシュの効果

- **FIFO内の全データをクリア**: 古いデータや異常なデータを削除
- **フレーム同期の回復**: ヘッダー位置のずれを修正
- **正常動作への復帰**: 次回読み取りから正常なフレームを取得可能

### 注意点

- フラッシュ実行中のデータは失われる（数フレーム程度）
- フラッシュ後は10ms待機してから次の読み取りを行う

## ストリームモード

### 動作

- **FIFO満杯時**: 古いデータを上書きして新しいデータを格納（FIFO_CONFIG_0 = 0x00）
- **Stop-on-fullモード**: FIFO満杯時に新しいデータを破棄（FIFO_CONFIG_0 = 0x01）

### Stream modeのメリット

- 常に最新のデータを保持
- データロス時も自動的に復旧

### Stream modeのデメリット

- FIFO満杯時に古いデータが失われる
- Skip frame（0x40）が発生する可能性がある

## ビルド＆実行

### 1. 環境セットアップ

```bash
source ~/esp/esp-idf/export.sh
```

### 2. ターゲット設定（初回のみ）

```bash
cd examples/stage5_fifo/step3_flush
idf.py set-target esp32s3
```

### 3. ビルド

```bash
idf.py build
```

### 4. 書き込みとモニタ

```bash
idf.py flash monitor
```

## 期待される出力

### 起動時ログ

```
I (XXX) BMI270_STEP3: ========================================
I (XXX) BMI270_STEP3:  BMI270 Step 3: FIFO Flush and Stream Mode
I (XXX) BMI270_STEP3: ========================================
I (XXX) BMI270_STEP3: FIFO_CONFIG_1 readback: 0xD0 (expected 0xD0)
```

### データ読み取りログ（正常時）

```
I (XXX) BMI270_STEP3: ----------------------------------------
I (XXX) BMI270_STEP3: Loop #1, FIFO length: 143 bytes
I (XXX) BMI270_STEP3: Parsing 11 frames (143 bytes)
I (XXX) BMI270_STEP3: Initialized timestamp baseline (t=0.000000s)
D (XXX) BMI270_STEP3: Config change frame (0x48)
I (XXX) BMI270_STEP3: Valid frames: 10/11
>gyr_x:0.123456:-0.05
>gyr_y:0.123456:0.12
>gyr_z:0.123456:-0.03
>acc_x:0.123456:0.012
>acc_y:0.123456:-0.024
>acc_z:0.123456:0.995
I (XXX) BMI270_STEP3: FIFO length after read: 0 bytes
I (XXX) BMI270_STEP3: Statistics: Total=11 Valid=10 Skip=0 Config=1 Flush=0
```
→ 正常動作時はFlush=0

### Skip frame検出とフラッシュ

```
I (XXX) BMI270_STEP3: Loop #50, FIFO length: 2004 bytes
I (XXX) BMI270_STEP3: Parsing 154 frames (2004 bytes)
W (XXX) BMI270_STEP3: Skip frame detected at position 1/154
I (XXX) BMI270_STEP3: Valid frames: 153/154
W (XXX) BMI270_STEP3: Skip frame detected - flushing FIFO to recover
W (XXX) BMI270_STEP3: FIFO flushed (count: 1)
I (XXX) BMI270_STEP3: FIFO length after read: 0 bytes
I (XXX) BMI270_STEP3: Statistics: Total=1545 Valid=1543 Skip=1 Config=1 Flush=1
```
→ Skip frame検出後、自動的にFIFOフラッシュして復旧

## 検証ポイント

### ✓ FIFOフラッシュが機能しているか

```
W (XXX) BMI270_STEP3: FIFO flushed (count: 1)
```
→ Skip frame検出後、フラッシュが実行されればOK

### ✓ フラッシュ後に正常動作に復帰しているか

```
I (XXX) BMI270_STEP3: Loop #51, FIFO length: 130 bytes
I (XXX) BMI270_STEP3: Valid frames: 10/10
```
→ フラッシュ後、次のループで正常なフレームが取得できればOK

### ✓ フラッシュ回数が適切か

```
Statistics: ... Flush=1
```
→ Flush回数が少なければ、正常に動作している証拠

## 動作特性

- **全データ読み取り**: FIFO内の全データを毎回消費
- **データロス復旧**: Skip frame検出時にFIFOフラッシュして自動復旧
- **統計情報**: フレームタイプごとにカウント + フラッシュ回数
- **平均値出力**: printf処理オーバーヘッド削減
- **ウォーターマークなし**: FIFOが満杯になるまで蓄積（Step 4で対応）
- **割り込みなし**: ポーリング（100ms間隔）でFIFO長さをチェック（Step 5で対応）

## 次のステップ

**Step 4 (step4_watermark_poll)**: ウォーターマーク設定（割り込みなし）
