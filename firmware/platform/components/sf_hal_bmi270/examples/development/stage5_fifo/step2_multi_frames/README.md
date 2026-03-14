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

# Step 2: FIFO Multiple Frames Read (Averaged Output) - FIFO複数フレーム読み取り（平均値出力）

## 概要

FIFO内の全データを一括読み取りし、複数フレームを連続パースして**平均値を出力**することで、データロスを防ぎながらprintf処理のオーバーヘッドを削減します。

## 目的

1. **データロス防止**: FIFO_LENGTH分を全て読み取り、データロスを防ぐ
2. **複数フレーム処理**: バッファ内の複数フレームを連続してパース
3. **特殊ヘッダー処理**: 0x40（スキップ）、0x48（設定変更）の適切な処理
4. **統計情報**: 有効フレーム、スキップフレームの統計を収集
5. **平均値出力**: 複数フレームの平均値を計算してprintf回数を削減（処理速度向上）

## Step 1との違い

| 項目 | Step 1 | Step 2 |
|------|--------|--------|
| 読み取り方法 | 1フレーム（13バイト）のみ | FIFO_LENGTH分全て（最大2048バイト） |
| データロス | 発生する（意図的） | 発生しない |
| フレーム処理 | 1フレーム/ループ | 最大157フレーム/ループ |
| 終了条件 | 異常ヘッダー検出時 | なし（無限ループ） |
| 統計情報 | なし | あり（有効/スキップ/設定変更フレーム数） |

## FIFO設定詳細

### 設定（Step 1と同じ）

```c
// FIFO_CONFIG_0 (0x48): ストリームモード
uint8_t fifo_config_0 = 0x00;

// FIFO_CONFIG_1 (0x49): ACC+GYR有効、ヘッダーモード
uint8_t fifo_config_1 = 0xD0;
```

### 動作フロー

1. センサー初期化（ACC/GYR 100Hz設定）
2. FIFO設定（ACC+GYR、ヘッダーモード）
3. ループ:
   - `FIFO_LENGTH`レジスタ（0x24-0x25）読み取り
   - FIFO_LENGTH分を**一括バースト読み取り**
   - バッファを13バイトずつパース:
     - 各フレームにタイムスタンプを付与（10ms間隔で逆算）
     - 0x8C（ACC+GYR）: データ解析・Teleplot出力（タイムスタンプ付き）
     - 0x48（設定変更）: デバッグログ出力、継続
     - 0x40（スキップ）: 警告ログ出力、継続
   - FIFO長さ確認（全データ消費を確認）
   - 統計情報表示
   - 100ms待機（次回ポーリング）

## タイムスタンプと平均値出力

### 実装詳細

複数フレームを読み取り、**平均値を計算して出力**することでprintf処理のオーバーヘッドを削減します。

**タイムスタンプ:**
- 初回読み取り時: `g_start_time_us`を現在時刻に設定（基準点）
- 各バッチ読み取り時: 読み取り時刻を記録
- 相対時刻(秒) = (読み取り時刻(us) - g_start_time_us) / 1,000,000

**平均値計算:**
- 全有効フレームのデータを蓄積
- 平均値 = 合計値 / 有効フレーム数
- 1回の読み取りで6行のprintf出力のみ（従来は6×フレーム数行）

**例（10フレーム読み取り時）:**
```
10フレーム読み取り → 平均値計算 → 6行出力
・従来: 10フレーム × 6チャンネル = 60行のprintf
・改善: 1バッチ × 6チャンネル = 6行のprintf（約10倍高速化）
```

**Teleplot出力形式:**
```
>channel_name:timestamp_sec:averaged_value
>gyr_x:0.123456:-0.05  ← 10フレームの平均値
>gyr_y:0.123456:0.12
>gyr_z:0.123456:-0.03
```

## データロス防止のメカニズム

### 100Hz ODRの場合

- **データ生成速度**: 100フレーム/秒 × 13バイト = 1300バイト/秒
- **ポーリング周期**: 100ms（10回/秒）
- **1回あたりの蓄積**: 約130バイト（10フレーム）
- **読み取り**: FIFO_LENGTH分全て（130バイト以上でも対応）
- **結果**: ✅ データロスなし

### 最大対応ODR

理論上、ポーリング周期100msでFIFOサイズ2048バイトなら:
- 2048バイト / 13バイト/フレーム = 157フレーム
- 157フレーム / 0.1秒 = 1570Hz

**1600Hz ODRまで対応可能**（理論値）

## ビルド＆実行

### 1. 環境セットアップ

```bash
source ~/esp/esp-idf/export.sh
```

### 2. ターゲット設定（初回のみ）

```bash
cd examples/stage5_fifo/step2_multi_frames
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
I (XXX) BMI270_STEP2: ========================================
I (XXX) BMI270_STEP2:  BMI270 Step 2: FIFO Multiple Frames Read
I (XXX) BMI270_STEP2: ========================================
I (XXX) BMI270_STEP2: FIFO_CONFIG_1 readback: 0xD0 (expected 0xD0)
```

### データ読み取りログ（平均値出力）

```
I (XXX) BMI270_STEP2: ----------------------------------------
I (XXX) BMI270_STEP2: Loop #1, FIFO length: 143 bytes
I (XXX) BMI270_STEP2: Parsing 11 frames (143 bytes)
I (XXX) BMI270_STEP2: Initialized timestamp baseline (t=0.000000s)
D (XXX) BMI270_STEP2: Config change frame (0x48)
I (XXX) BMI270_STEP2: Valid frames: 10/11
>gyr_x:0.123456:-0.05
>gyr_y:0.123456:0.12
>gyr_z:0.123456:-0.03
>acc_x:0.123456:0.012
>acc_y:0.123456:-0.024
>acc_z:0.123456:0.995
I (XXX) BMI270_STEP2: FIFO length after read: 0 bytes (consumed: 143 bytes)
I (XXX) BMI270_STEP2: Statistics: Total=11 Valid=10 Skip=0 Config=1
```
→ 10フレームの平均値を6行で出力（従来は60行）

### データロスがない場合

```
I (XXX) BMI270_STEP2: Loop #10, FIFO length: 130 bytes
I (XXX) BMI270_STEP2: Parsing 10 frames (130 bytes)
I (XXX) BMI270_STEP2: Valid frames: 10/10
I (XXX) BMI270_STEP2: FIFO length after read: 0 bytes (consumed: 130 bytes)
I (XXX) BMI270_STEP2: Statistics: Total=111 Valid=100 Skip=0 Config=1
```
→ Skip=0 であればデータロスなし！

### データロスが発生した場合（異常時）

```
I (XXX) BMI270_STEP2: Loop #50, FIFO length: 2004 bytes
I (XXX) BMI270_STEP2: Parsing 154 frames (2004 bytes)
W (XXX) BMI270_STEP2: Skip frame (0x40) - data loss detected!
I (XXX) BMI270_STEP2: Valid frames: 153/154
I (XXX) BMI270_STEP2: Statistics: Total=1545 Valid=1543 Skip=1 Config=1
```
→ Skip > 0 の場合、読み取り速度が追いついていない

## 検証ポイント

### ✓ 全フレームを読み取れているか

```
FIFO length after read: 0 bytes (consumed: 130 bytes)
```
→ 読み取り後のFIFO長が0であればOK

### ✓ データロスがないか

```
Statistics: Total=111 Valid=100 Skip=0 Config=1
```
→ Skip=0 であれば正常（データロスなし）

### ✓ フレーム数が正しいか

100Hz ODR、100ms周期なら:
- 期待フレーム数: 約10フレーム/ループ
- 実測が10±1フレームであればOK

## 動作特性

- **全データ読み取り**: FIFO内の全データを毎回消費
- **データロスなし**: 100Hz ODRで安定動作
- **統計情報**: フレームタイプごとにカウント
- **ウォーターマークなし**: FIFOが満杯になるまで蓄積（Step 4で対応）
- **割り込みなし**: ポーリング（100ms間隔）でFIFO長さをチェック（Step 5で対応）

## 次のステップ

**Step 3 (step3_flush)**: FIFOフラッシュとストリームモード確認
